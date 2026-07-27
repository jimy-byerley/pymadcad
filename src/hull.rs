/*!
    convex hull of point clouds in 2d and 3d

    This is a port of the joggled-input branch of qhull (its `QJ` option), which is the
    configuration pymadcad has always asked for through scipy. The hull is built with the
    Quickhull algorithm of Barber, Dobkin & Huhdanpaa (1996) — beneath-beyond incremental
    insertion driven by per-facet outside sets — with facet merging disabled.

    Instead of maintaining thick facets and merging the non-convex ones (the branch qhull
    uses by default), every input coordinate is perturbed by a small random amount so that
    no point sits exactly on a facet plane, and the whole build is restarted with a bigger
    perturbation whenever a precision failure is detected. This is what makes the output
    always simplicial and clearly convex.

    The returned simplices index the *input* points, so the caller keeps its own point
    buffer and any per-point attributes. Points strictly inside the hull are simply never
    referenced. Because the build runs on perturbed coordinates, points that were exactly
    coplanar with a hull facet come out as their own thin triangles rather than being
    merged into one facet.
*/

use crate::math::*;
use crate::rand::{Rand, RESEED};
use rustc_hash::{FxHashMap, FxHashSet};

/// default joggle is this many times the roundoff of a distance computation (`qh_JOGGLEdefault`)
const JOGGLE_DEFAULT: Float = 30_000.0;
/// factor the joggle is multiplied by on each escalation (`qh_JOGGLEincrease`)
const JOGGLE_INCREASE: Float = 10.0;
/// the joggle is never escalated above this fraction of the cloud width (`qh_JOGGLEmaxincrease`)
const JOGGLE_MAXINCREASE: Float = 1e-2;
/// number of attempts at the initial joggle before starting to escalate it (`qh_JOGGLEretry`)
const JOGGLE_RETRY: u32 = 2;
/// total number of attempts before giving up (`qh_JOGGLEmaxretry`)
const JOGGLE_MAXRETRY: u32 = 50;
/// seed of the joggle, fixed so that hulls are reproducible across runs
const JOGGLE_SEED: u64 = 0x2545_F491_4F6C_DD1D;

/// failure of a hull computation
#[derive(Debug, Clone, PartialEq)]
pub enum HullError {
    /// not enough points to define a hull of that dimension
    Incomplete,
    /// no joggle magnitude could produce a valid hull, the last one tried is given
    JoggleExhausted(Float),
}

impl core::fmt::Display for HullError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Incomplete => write!(f, "not enough points to define a convex hull"),
            Self::JoggleExhausted(joggle) => write!(f,
                "convex hull failed with every joggle magnitude up to {joggle:e}, the input is likely too degenerate to be representable"),
        }
    }
}

/**
    a precision failure met during a build, asking for the input to be joggled again

    Each message matches a `qh_joggle_restart` call site in qhull, which is the exhaustive
    list of situations where the merge-free build cannot continue.
*/
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Precision(&'static str);

// ---------------------------------------------------------------------------
// joggle
// ---------------------------------------------------------------------------

/// extent of a point cloud, as `qh_maxmin` measures it
struct Scale {
    /// biggest absolute coordinate over all axes
    maxabs: Float,
    /// sum over axes of the biggest absolute coordinate of that axis
    maxsumabs: Float,
    /// biggest extent of the cloud along an axis
    maxwidth: Float,
}

fn scale<const N: usize>(points: &[Vector<Float, N>]) -> Scale {
    let mut maxabs: Float = 0.;
    let mut maxsumabs: Float = 0.;
    let mut maxwidth: Float = 0.;
    for k in 0..N {
        let (min, max) = points.iter().map(|p| p[k]).fold(
            (Float::INFINITY, Float::NEG_INFINITY),
            |(min, max), c| (min.min(c), max.max(c)),
        );
        if !min.is_finite() || !max.is_finite() {
            continue;
        }
        let abscoord = max.max(-min);
        maxabs = maxabs.max(abscoord);
        maxsumabs += abscoord;
        maxwidth = maxwidth.max(max - min);
    }
    Scale { maxabs, maxsumabs, maxwidth }
}

/// maximum roundoff error of a distance computation over such a cloud (`qh_distround`)
fn distround<const N: usize>(scale: &Scale) -> Float {
    let dim = N as Float;
    let maxdistsum = (dim.sqrt() * scale.maxabs).min(scale.maxsumabs);
    Float::EPSILON * (dim * maxdistsum * 1.01 + scale.maxabs)
}

/// index of the item maximising `key`, along with that maximum
fn argmax<T>(items: &[T], key: impl Fn(&T) -> Float) -> (usize, Float) {
    items.iter()
        .map(key)
        .enumerate()
        .max_by(|(_, a), (_, b)| a.total_cmp(b))
        .unwrap_or((0, Float::NEG_INFINITY))
}

/// indices of the two extreme points along the axis the cloud is widest along
///
/// This is how qhull seeds its build: extreme points along a coordinate axis are the ones
/// most certainly on the hull.
fn widest_extremes<const N: usize>(points: &[Vector<Float, N>]) -> (usize, usize) {
    (0..N)
        .map(|k| {
            let (low, min) = argmax(points, |p| -p[k]);
            let (high, max) = argmax(points, |p| p[k]);
            (max + min, low, high)
        })
        .max_by(|(a, ..), (b, ..)| a.total_cmp(b))
        .map_or((0, 0), |(_, low, high)| (low, high))
}

/**
    run `build` on joggled copies of `points`, escalating the joggle until it succeeds

    `build` receives the joggled points and the roundoff tolerance distances must be
    compared against. It must report [`Precision`] rather than produce a doubtful hull,
    since a restart is always cheaper than a wrong result.
*/
fn with_joggle<const N: usize, S>(
    points: &[Vector<Float, N>],
    build: impl Fn(&[Vector<Float, N>], Float) -> Result<Vec<S>, Precision>,
) -> Result<Vec<S>, HullError> {
    let extent = scale(points);
    // the magnitude comes from the original cloud, like qh_detjoggle. the epsilon floor is
    // what makes a cloud of identical points work at all
    let mut joggle = distround::<N>(&extent).max(Float::EPSILON) * JOGGLE_DEFAULT;
    let ceiling = extent.maxwidth * JOGGLE_MAXINCREASE;
    let unusable = (extent.maxwidth / 4.).max(0.1);

    let mut joggled = vec![Vector::fill(0.); points.len()];
    for attempt in 0..JOGGLE_MAXRETRY {
        if attempt > JOGGLE_RETRY && joggle < ceiling {
            joggle = (joggle * JOGGLE_INCREASE).min(ceiling);
        }
        if attempt > 0 && joggle > unusable {
            return Err(HullError::JoggleExhausted(joggle));
        }
        // a new seed per attempt, so retrying at the same magnitude still explores
        let mut rand = Rand::new(JOGGLE_SEED ^ (attempt as u64).wrapping_mul(RESEED));
        for (joggled, point) in joggled.iter_mut().zip(points) {
            *joggled = *point + Vector::init(|| rand.uniform(joggle));
        }
        // the tolerance comes from the joggled cloud, like qh_detroundoff runs after
        // qh_joggleinput. it matters when the original cloud has no extent of its own
        let round = distround::<N>(&scale(&joggled));
        if let Ok(simplices) = build(&joggled, round) {
            return Ok(simplices);
        }
    }
    Err(HullError::JoggleExhausted(joggle))
}

// ---------------------------------------------------------------------------
// 3d
// ---------------------------------------------------------------------------

/**
    triangular facet of a 3d hull under construction

    Adjacency is stored as half edges: edge `i` of the facet is `(pts[i], pts[i+1])`, the
    facet across it is `adj[i]`, and that same edge is edge `adj_edge[i]` of the neighbour,
    traversed the other way round.
*/
struct Facet {
    pts: [Index; 3],
    adj: [u32; 3],
    adj_edge: [u8; 3],
    normal: Vec3,
    offset: Float,
    /// points known to be above this facet, qhull's outside set
    outside: Vec<Index>,
    furthest: Index,
    furthest_dist: Float,
    /// false once the facet has been replaced by a cone
    alive: bool,
}

impl Facet {
    /// new facet through the 3 given points, its normal on the side they turn
    /// counterclockwise around
    fn new(pts: [Index; 3], points: &[Vec3], round: Float) -> Result<Self, Precision> {
        let (a, b, c) = (
            points[pts[0] as usize],
            points[pts[1] as usize],
            points[pts[2] as usize],
        );
        let cross = (b - a).cross(c - a);
        // the triangle is degenerate when its smallest height falls below the tolerance,
        // that height being 2*area/longest_edge = |cross|/longest_edge
        let longest = [b - a, c - b, a - c].iter()
            .map(|edge| edge.square_length())
            .fold(0., Float::max)
            .sqrt();
        if cross.square_length() <= (round * longest) * (round * longest) {
            return Err(Precision("degenerate facet"));
        }
        let normal = cross.normalize();
        Ok(Self {
            pts,
            adj: [0; 3],
            adj_edge: [0; 3],
            normal,
            offset: normal.dot(a),
            outside: Vec::new(),
            furthest: 0,
            furthest_dist: 0.,
            alive: true,
        })
    }

    /// signed distance of a point to the facet plane, positive outside
    fn above(&self, point: Vec3) -> Float {
        self.normal.dot(point) - self.offset
    }

    /// the 3 half edges of the facet, in the order their neighbours are stored in
    fn edges(&self) -> impl Iterator<Item = (Index, Index)> + '_ {
        (0..3).map(|i| (self.pts[i], self.pts[(i + 1) % 3]))
    }

    fn add_outside(&mut self, point: Index, distance: Float) {
        if distance > self.furthest_dist {
            self.furthest_dist = distance;
            self.furthest = point;
        }
        self.outside.push(point);
    }
}

/**
    convex hull of a 3d point cloud, as triangles indexing `points`

    The triangles are oriented outward and form a closed manifold envelope. Points inside
    the hull are not referenced by any triangle.
*/
pub fn convexhull_3d(points: &[Vec3]) -> Result<Vec<UVec3>, HullError> {
    if points.len() < 4 {
        return Err(HullError::Incomplete);
    }
    with_joggle(points, build_3d)
}

fn build_3d(points: &[Vec3], round: Float) -> Result<Vec<UVec3>, Precision> {
    let (mut facets, interior) = initial_simplex(points, round)?;

    // partition every remaining point into the outside set of a facet it is above
    let corners: FxHashSet<Index> = facets.iter().flat_map(|f| f.pts).collect();
    for (i, point) in points.iter().enumerate() {
        if !corners.contains(&(i as Index)) {
            assign_outside(&mut facets, 0..4, i as Index, *point, round);
        }
    }

    // scratch buffers reused across iterations
    let mut stamp = vec![0u32; facets.len()];
    let mut epoch = 0u32;
    let mut visible = Vec::new();
    let mut horizon = Vec::new();
    let mut stack = Vec::new();
    let mut orphans = Vec::new();

    // each iteration promotes one point to a hull vertex, so this terminates
    let mut current = 0;
    while current < facets.len() {
        if !facets[current].alive || facets[current].outside.is_empty() {
            current += 1;
            continue;
        }
        let apex = facets[current].furthest;
        let apex_point = points[apex as usize];

        // visible set: every facet the apex is above, reachable from this one
        epoch += 1;
        stamp.resize(facets.len(), 0);
        visible.clear();
        stack.clear();
        stamp[current] = epoch;
        visible.push(current as u32);
        stack.push(current as u32);
        while let Some(facet) = stack.pop() {
            for neighbour in facets[facet as usize].adj {
                if stamp[neighbour as usize] != epoch
                && facets[neighbour as usize].above(apex_point) > round {
                    stamp[neighbour as usize] = epoch;
                    visible.push(neighbour);
                    stack.push(neighbour);
                }
            }
        }

        // horizon: the edges where the visible set meets the rest of the hull, recorded
        // from the facet that stays
        horizon.clear();
        for &facet in &visible {
            let facet = &facets[facet as usize];
            horizon.extend(
                facet.adj.iter().zip(&facet.adj_edge)
                    .map(|(&neighbour, &edge)| (neighbour, edge))
                    .filter(|&(neighbour, _)| stamp[neighbour as usize] != epoch),
            );
        }
        order_horizon(&mut horizon, &facets)?;

        // replace the visible facets by a cone from the apex to the horizon
        for &facet in &visible {
            facets[facet as usize].alive = false;
            orphans.append(&mut facets[facet as usize].outside);
        }
        let cone = facets.len() as u32;
        let ring = horizon.len() as u32;
        for (i, &(facet, edge)) in horizon.iter().enumerate() {
            let i = i as u32;
            // the shared edge, in the direction the removed facet traversed it
            let u = facets[facet as usize].pts[edge as usize];
            let v = facets[facet as usize].pts[(edge as usize + 1) % 3];
            let mut new = Facet::new([v, u, apex], points, round)?;
            new.adj = [facet, cone + (i + ring - 1) % ring, cone + (i + 1) % ring];
            new.adj_edge = [edge, 2, 1];
            facets[facet as usize].adj[edge as usize] = cone + i;
            facets[facet as usize].adj_edge[edge as usize] = 0;
            facets.push(new);
        }
        // a cone facet facing inward means the apex was not really beyond the horizon
        for facet in &facets[cone as usize..] {
            if facet.above(interior) > -round {
                return Err(Precision("flipped facet"));
            }
        }

        // hand the orphaned points over to the new facets, dropping those now inside
        for &point in &orphans {
            if point == apex {
                continue;
            }
            assign_outside(
                &mut facets,
                cone as usize..cone as usize + ring as usize,
                point,
                points[point as usize],
                round,
            );
        }
        orphans.clear();

        current += 1;
    }

    check_3d(&facets, points, round)?;
    Ok(facets.iter()
        .filter(|f| f.alive)
        .map(|f| Vector::from(f.pts))
        .collect())
}

/// add `point` to the outside set of the facet among `range` it is furthest above, if any
fn assign_outside(
    facets: &mut [Facet],
    range: core::ops::Range<usize>,
    point: Index,
    coords: Vec3,
    round: Float,
) {
    let best = range
        .filter(|&i| facets[i].alive)
        .map(|i| (i, facets[i].above(coords)))
        .filter(|&(_, distance)| distance > round)
        .max_by(|(_, a), (_, b)| a.total_cmp(b));
    if let Some((i, distance)) = best {
        facets[i].add_outside(point, distance);
    }
}

/**
    initial tetrahedron of the build, from extreme points of the cloud, along with a point
    strictly inside it

    The interior point stays inside for the whole build since the hull only ever grows, so
    it serves as the reference for detecting facets that come out facing inward.

    Fails when the cloud has no 3d extent, which the caller answers by joggling harder —
    joggling is precisely what turns a flat cloud into a thin but valid hull.
*/
fn initial_simplex(points: &[Vec3], round: Float) -> Result<(Vec<Facet>, Vec3), Precision> {
    // the two extremes of the widest axis
    let (low, high) = widest_extremes(points);
    let (a, b) = (points[low], points[high]);
    if (b - a).square_length() <= round * round {
        return Err(Precision("initial simplex is a point"));
    }

    // the point furthest from that line
    let direction = (b - a).normalize();
    let (third, best) = argmax(points, |p| (*p - a).cross(direction).square_length().sqrt());
    if best <= round {
        return Err(Precision("initial simplex is a segment"));
    }

    // the point furthest from the plane of the first three
    let c = points[third];
    let normal = (b - a).cross(c - a).normalize();
    let (fourth, best) = argmax(points, |p| (*p - a).dot(normal).abs());
    if best <= round {
        return Err(Precision("initial simplex is flat"));
    }

    // order the first three so the fourth ends up below their plane, which puts all four
    // facets built below facing outward
    let (i, j) = if (points[fourth] - a).dot(normal) > 0. {
        (high, low)
    } else {
        (low, high)
    };
    let (k, l) = (third, fourth);

    let mut facets = vec![
        Facet::new([i as Index, j as Index, k as Index], points, round)?,
        Facet::new([i as Index, k as Index, l as Index], points, round)?,
        Facet::new([i as Index, l as Index, j as Index], points, round)?,
        Facet::new([j as Index, l as Index, k as Index], points, round)?,
    ];
    link_facets(&mut facets)?;
    let interior = (points[i] + points[j] + points[k] + points[l]) / 4.;
    Ok((facets, interior))
}

/// fill in the adjacency of a set of facets by matching opposite half edges
fn link_facets(facets: &mut [Facet]) -> Result<(), Precision> {
    let mut edges = FxHashMap::default();
    for (i, facet) in facets.iter().enumerate() {
        for (edge, key) in facet.edges().enumerate() {
            if edges.insert(key, (i as u32, edge as u8)).is_some() {
                return Err(Precision("two facets with the same edge"));
            }
        }
    }
    for facet in facets.iter_mut() {
        for edge in 0..3 {
            let key = (facet.pts[(edge + 1) % 3], facet.pts[edge]);
            let (neighbour, other) = *edges.get(&key)
                .ok_or(Precision("facet edge without a neighbour"))?;
            facet.adj[edge] = neighbour;
            facet.adj_edge[edge] = other;
        }
    }
    Ok(())
}

/**
    sort horizon edges into a single closed loop, in place

    Ordering them lets the cone replacing the visible facets be linked in one pass. A
    horizon that is not one simple loop is a precision failure: those are qhull's empty and
    coplanar horizons, and its dupridge cases.
*/
fn order_horizon(horizon: &mut Vec<(u32, u8)>, facets: &[Facet]) -> Result<(), Precision> {
    if horizon.is_empty() {
        return Err(Precision("empty horizon"));
    }
    // index the edges by their first vertex, which may only appear once around a loop
    let mut starts = FxHashMap::default();
    for &(facet, edge) in horizon.iter() {
        let start = facets[facet as usize].pts[edge as usize];
        if starts.insert(start, (facet, edge)).is_some() {
            return Err(Precision("horizon crossing itself"));
        }
    }
    let start = horizon[0];
    let mut ordered = Vec::with_capacity(horizon.len());
    let mut current = start;
    loop {
        ordered.push(current);
        let (facet, edge) = current;
        let end = facets[facet as usize].pts[(edge as usize + 1) % 3];
        let next = *starts.get(&end).ok_or(Precision("open horizon"))?;
        if next == start {
            break;
        }
        if ordered.len() >= horizon.len() {
            return Err(Precision("horizon in several loops"));
        }
        current = next;
    }
    if ordered.len() != horizon.len() {
        return Err(Precision("horizon in several loops"));
    }
    *horizon = ordered;
    Ok(())
}

/**
    check the finished hull is a closed convex envelope

    This is what replaces the guarantees facet merging would otherwise give. The convexity
    test is on vertices rather than centrums, which is what qhull itself does for
    simplicial facets.
*/
fn check_3d(facets: &[Facet], points: &[Vec3], round: Float) -> Result<(), Precision> {
    let alive = || facets.iter().enumerate().filter(|(_, f)| f.alive);

    for (i, facet) in alive() {
        for (&adj, &other) in facet.adj.iter().zip(&facet.adj_edge) {
            let neighbour = &facets[adj as usize];
            let other = other as usize;
            if !neighbour.alive {
                return Err(Precision("neighbour of a live facet is dead"));
            }
            if neighbour.adj[other] != i as u32 {
                return Err(Precision("asymmetric adjacency"));
            }
            // the vertex the neighbour does not share with us must not stick out
            let opposite = neighbour.pts[(other + 2) % 3];
            if facet.above(points[opposite as usize]) > round {
                return Err(Precision("concave ridge"));
            }
        }
    }

    // closed manifold: every half edge used once, and its opposite present
    let mut edges = FxHashSet::default();
    let mut vertices = FxHashSet::default();
    for (_, facet) in alive() {
        vertices.extend(facet.pts);
        for edge in facet.edges() {
            if !edges.insert(edge) {
                return Err(Precision("edge shared by more than two facets"));
            }
        }
    }
    if !edges.iter().all(|&(a, b)| edges.contains(&(b, a))) {
        return Err(Precision("edge without an opposite"));
    }
    let faces = alive().count();
    if edges.len() != faces * 3 {
        return Err(Precision("inconsistent edge count"));
    }
    // euler characteristic of a sphere
    if vertices.len() + faces != edges.len() / 2 + 2 {
        return Err(Precision("hull is not closed"));
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// 2d
// ---------------------------------------------------------------------------

/// segment of a 2d hull under construction, part of a doubly linked loop
struct Segment {
    pts: [Index; 2],
    previous: u32,
    next: u32,
    normal: Vec2,
    offset: Float,
    outside: Vec<Index>,
    furthest: Index,
    furthest_dist: Float,
    alive: bool,
}

impl Segment {
    fn new(pts: [Index; 2], points: &[Vec2], round: Float) -> Result<Self, Precision> {
        let (a, b) = (points[pts[0] as usize], points[pts[1] as usize]);
        let direction = b - a;
        if direction.square_length() <= round * round {
            return Err(Precision("degenerate segment"));
        }
        // outward normal of a counterclockwise loop
        let normal = Vec2::from([direction[1], -direction[0]]).normalize();
        Ok(Self {
            pts,
            previous: 0,
            next: 0,
            normal,
            offset: normal.dot(a),
            outside: Vec::new(),
            furthest: 0,
            furthest_dist: 0.,
            alive: true,
        })
    }

    fn above(&self, point: Vec2) -> Float {
        self.normal.dot(point) - self.offset
    }

    fn add_outside(&mut self, point: Index, distance: Float) {
        if distance > self.furthest_dist {
            self.furthest_dist = distance;
            self.furthest = point;
        }
        self.outside.push(point);
    }
}

/**
    convex hull of a 2d point cloud, as the edges of its outline indexing `points`

    The edges are given in loop order, oriented counterclockwise.
*/
pub fn convexhull_2d(points: &[Vec2]) -> Result<Vec<UVec2>, HullError> {
    if points.len() < 3 {
        return Err(HullError::Incomplete);
    }
    with_joggle(points, build_2d)
}

fn build_2d(points: &[Vec2], round: Float) -> Result<Vec<UVec2>, Precision> {
    // the two extremes of the widest axis, as a degenerate two sided loop
    let (low, high) = widest_extremes(points);
    if (points[high] - points[low]).square_length() <= round * round {
        return Err(Precision("initial outline is a point"));
    }
    let (low, high) = (low as Index, high as Index);
    let mut segments = vec![
        Segment::new([low, high], points, round)?,
        Segment::new([high, low], points, round)?,
    ];
    segments[0].previous = 1;
    segments[0].next = 1;
    segments[1].previous = 0;
    segments[1].next = 0;

    for i in 0..points.len() as Index {
        if i == low || i == high {
            continue;
        }
        assign_outside_2d(&mut segments, 0..2, i, points[i as usize], round);
    }

    let mut orphans = Vec::new();
    let mut current = 0;
    while current < segments.len() {
        if !segments[current].alive || segments[current].outside.is_empty() {
            current += 1;
            continue;
        }
        let apex = segments[current].furthest;
        let [a, b] = segments[current].pts;
        let (previous, next) = (segments[current].previous, segments[current].next);

        // split the segment in two around the apex
        segments[current].alive = false;
        let mut taken = core::mem::take(&mut segments[current].outside);
        orphans.append(&mut taken);
        let split = segments.len() as u32;
        let mut first = Segment::new([a, apex], points, round)?;
        let mut second = Segment::new([apex, b], points, round)?;
        first.previous = previous;
        first.next = split + 1;
        second.previous = split;
        second.next = next;
        segments.push(first);
        segments.push(second);
        segments[previous as usize].next = split;
        segments[next as usize].previous = split + 1;

        for &point in &orphans {
            if point == apex {
                continue;
            }
            assign_outside_2d(
                &mut segments,
                split as usize..split as usize + 2,
                point,
                points[point as usize],
                round,
            );
        }
        orphans.clear();

        current += 1;
    }

    // walk the loop, which also checks it is closed and convex
    let start = segments.iter().position(|s| s.alive)
        .ok_or(Precision("empty outline"))? as u32;
    let alive = segments.iter().filter(|s| s.alive).count();
    let mut outline = Vec::with_capacity(alive);
    let mut current = start;
    loop {
        let segment = &segments[current as usize];
        if !segment.alive {
            return Err(Precision("dead segment in the outline"));
        }
        outline.push(Vector::from(segment.pts));
        let next = &segments[segment.next as usize];
        if segment.pts[1] != next.pts[0] {
            return Err(Precision("broken outline"));
        }
        // convexity, on the far vertex of the next segment
        if segment.above(points[next.pts[1] as usize]) > round {
            return Err(Precision("concave outline"));
        }
        current = segment.next;
        if current == start {
            break;
        }
        if outline.len() >= alive {
            return Err(Precision("outline in several loops"));
        }
    }
    if outline.len() != alive {
        return Err(Precision("outline in several loops"));
    }
    Ok(outline)
}

fn assign_outside_2d(
    segments: &mut [Segment],
    range: core::ops::Range<usize>,
    point: Index,
    coords: Vec2,
    round: Float,
) {
    let best = range
        .filter(|&i| segments[i].alive)
        .map(|i| (i, segments[i].above(coords)))
        .filter(|&(_, distance)| distance > round)
        .max_by(|(_, a), (_, b)| a.total_cmp(b));
    if let Some((i, distance)) = best {
        segments[i].add_outside(point, distance);
    }
}

// ---------------------------------------------------------------------------
// tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn vec3(x: Float, y: Float, z: Float) -> Vec3 { Vec3::from([x, y, z]) }
    fn vec2(x: Float, y: Float) -> Vec2 { Vec2::from([x, y]) }

    /// check a hull is a closed orientable surface of genus 0
    ///
    /// This is all that can be checked when the input cloud is degenerate, since the
    /// original coordinates then give no usable facet normals.
    fn check_manifold(faces: &[UVec3]) {
        assert!(!faces.is_empty(), "empty hull");
        let mut edges = FxHashSet::default();
        let mut vertices = FxHashSet::default();
        for face in faces {
            assert!(face[0] != face[1] && face[1] != face[2] && face[0] != face[2],
                "facet {face:?} repeats a vertex");
            for i in 0..3 {
                vertices.insert(face[i]);
                assert!(edges.insert((face[i], face[(i + 1) % 3])),
                    "half edge ({}, {}) used twice", face[i], face[(i + 1) % 3]);
            }
        }
        for &(a, b) in &edges {
            assert!(edges.contains(&(b, a)), "half edge ({a}, {b}) has no opposite");
        }
        assert_eq!(vertices.len() + faces.len(), edges.len() / 2 + 2,
            "euler characteristic is not that of a sphere");
    }

    /// check a 3d hull is a closed, outward oriented, convex envelope of `points`
    ///
    /// `tolerance` must exceed the joggle magnitude, since the hull was built on perturbed
    /// coordinates but is checked against the originals. Only usable on clouds with a real
    /// 3d extent and no duplicate points, otherwise facets are degenerate in the original
    /// coordinates and have no meaningful normal.
    fn check_hull(points: &[Vec3], faces: &[UVec3], tolerance: Float) {
        check_manifold(faces);

        // convex, with every input point inside, and normals pointing outward
        let inside = faces.iter()
            .flat_map(|f| (0..3).map(move |i| f[i]))
            .fold(vec3(0., 0., 0.), |sum, i| sum + points[i as usize])
            / (faces.len() * 3) as Float;
        for face in faces {
            let (a, b, c) = (
                points[face[0] as usize],
                points[face[1] as usize],
                points[face[2] as usize],
            );
            let normal = (b - a).cross(c - a).normalize();
            assert!(normal.dot(inside - a) < 0., "facet {face:?} faces inward");
            for point in points {
                assert!(normal.dot(*point - a) <= tolerance,
                    "point {point:?} is {} outside facet {face:?}", normal.dot(*point - a));
            }
        }
    }

    /// check a 2d outline is a closed convex counterclockwise loop containing `points`
    fn check_outline(points: &[Vec2], edges: &[UVec2], tolerance: Float) {
        assert!(edges.len() >= 3, "outline of {} edges", edges.len());
        for (i, edge) in edges.iter().enumerate() {
            let next = edges[(i + 1) % edges.len()];
            assert_eq!(edge[1], next[0], "outline is not a loop at edge {i}");
        }
        for edge in edges {
            let (a, b) = (points[edge[0] as usize], points[edge[1] as usize]);
            let normal = Vec2::from([(b - a)[1], -(b - a)[0]]).normalize();
            for point in points {
                assert!(normal.dot(*point - a) <= tolerance,
                    "point {point:?} is {} outside edge {edge:?}", normal.dot(*point - a));
            }
        }
    }

    /// points spread over a sphere by the golden angle, so all of them are extreme
    fn sphere(count: usize) -> Vec<Vec3> {
        let golden = std::f64::consts::PI * (3. - 5_f64.sqrt());
        (0..count).map(|i| {
            let z = 1. - 2. * (i as Float + 0.5) / count as Float;
            let r = (1. - z * z).sqrt();
            let a = golden * i as Float;
            vec3(r * a.cos(), r * a.sin(), z)
        }).collect()
    }

    fn cube() -> Vec<Vec3> {
        let mut points = Vec::new();
        for i in 0..8 {
            points.push(vec3(
                if i & 1 != 0 { 1. } else { -1. },
                if i & 2 != 0 { 1. } else { -1. },
                if i & 4 != 0 { 1. } else { -1. },
            ));
        }
        points
    }

    #[test]
    fn test_hull_tetrahedron() {
        let points = [
            vec3(0., 0., 0.), vec3(1., 0., 0.),
            vec3(0., 1., 0.), vec3(0., 0., 1.),
        ];
        let faces = convexhull_3d(&points).unwrap();
        assert_eq!(faces.len(), 4);
        check_hull(&points, &faces, 1e-9);
    }

    #[test]
    fn test_hull_cube() {
        let points = cube();
        let faces = convexhull_3d(&points).unwrap();
        // joggling splits each square face, a closed hull over 8 vertices has 12 triangles
        assert_eq!(faces.len(), 12);
        check_hull(&points, &faces, 1e-9);
    }

    #[test]
    fn test_hull_ignores_interior_points() {
        let mut points = cube();
        let corners = points.len();
        // a grid of points strictly inside
        for i in 0..5 {
            for j in 0..5 {
                points.push(vec3(i as Float * 0.1, j as Float * 0.1, 0.2));
            }
        }
        let faces = convexhull_3d(&points).unwrap();
        assert_eq!(faces.len(), 12);
        check_hull(&points, &faces, 1e-9);
        for face in &faces {
            for i in 0..3 {
                assert!((face[i] as usize) < corners, "an interior point became a vertex");
            }
        }
    }

    #[test]
    fn test_hull_sphere_all_extreme() {
        let points = sphere(200);
        let faces = convexhull_3d(&points).unwrap();
        // every point is extreme, so a simplicial sphere over 200 vertices
        assert_eq!(faces.len(), 2 * 200 - 4);
        check_hull(&points, &faces, 1e-9);
    }

    #[test]
    fn test_hull_small_and_far_from_origin() {
        // the joggle magnitude scales with the absolute coordinates, not with the extent of
        // the cloud, so a small object far from the origin gets perturbed by a sizeable
        // fraction of its own size and loses some of its vertices. qhull behaves the same
        // way, this test pins the behaviour rather than endorsing it
        let points: Vec<Vec3> = sphere(60).iter()
            .map(|p| *p * 1e-4 + vec3(1e6, -5e5, 3e5))
            .collect();
        let faces = convexhull_3d(&points).unwrap();
        check_manifold(&faces);
        assert!(faces.len() < 2 * 60 - 4);

        // the same cloud centred on the origin keeps every vertex
        let points: Vec<Vec3> = sphere(60).iter().map(|p| *p * 1e-4).collect();
        let faces = convexhull_3d(&points).unwrap();
        assert_eq!(faces.len(), 2 * 60 - 4);
        check_hull(&points, &faces, 1e-12);
    }

    #[test]
    fn test_hull_coplanar_gives_a_thin_envelope() {
        // a flat grid: joggling is what makes this buildable at all
        let mut points = Vec::new();
        for i in 0..4 {
            for j in 0..4 {
                points.push(vec3(i as Float, j as Float, 0.));
            }
        }
        let faces = convexhull_3d(&points).unwrap();
        check_manifold(&faces);
        // the 4 corners are extreme by a whole unit, far beyond the joggle, so they must be
        // kept. points along the straight sides are only extreme by nothing at all, and may
        // fall inside either sheet depending on how they were perturbed
        let vertices: FxHashSet<Index> = faces.iter()
            .flat_map(|f| (0..3).map(move |i| f[i]))
            .collect();
        for i in 0..points.len() as Index {
            let p = points[i as usize];
            if (p[0] == 0. || p[0] == 3.) && (p[1] == 0. || p[1] == 3.) {
                assert!(vertices.contains(&i), "corner {p:?} is not a hull vertex");
            }
        }
    }

    #[test]
    fn test_hull_collinear_points() {
        let mut points: Vec<Vec3> = (0..10).map(|i| vec3(i as Float, 0., 0.)).collect();
        // one point off the line still leaves the cloud flat
        points.push(vec3(3., 1., 0.));
        let faces = convexhull_3d(&points).unwrap();
        check_manifold(&faces);
    }

    #[test]
    fn test_hull_identical_points() {
        // qhull's own stress case for joggled input. joggling separates the copies, so the
        // result is the hull of a tiny random cloud rather than a single point
        let points = vec![vec3(2., -3., 5.); 100];
        let faces = convexhull_3d(&points).unwrap();
        check_manifold(&faces);
    }

    #[test]
    fn test_hull_duplicated_points() {
        let mut points = cube();
        points.extend(cube());
        points.extend(cube());
        let faces = convexhull_3d(&points).unwrap();
        check_manifold(&faces);
        // joggling separates the copies, so several copies of a corner may be kept, but
        // every corner of the cube must be represented
        let corners: FxHashSet<[u64; 3]> = faces.iter()
            .flat_map(|f| (0..3).map(move |i| f[i]))
            .map(|i| {
                let p = points[i as usize];
                [p[0].to_bits(), p[1].to_bits(), p[2].to_bits()]
            })
            .collect();
        assert_eq!(corners.len(), 8);
    }

    #[test]
    fn test_hull_too_few_points() {
        let points = [vec3(0., 0., 0.), vec3(1., 0., 0.), vec3(0., 1., 0.)];
        assert_eq!(convexhull_3d(&points), Err(HullError::Incomplete));
    }

    #[test]
    fn test_hull_is_deterministic() {
        let points = sphere(80);
        let once = convexhull_3d(&points).unwrap();
        let twice = convexhull_3d(&points).unwrap();
        assert_eq!(once, twice);
    }

    #[test]
    fn test_outline_square() {
        let points = [vec2(0., 0.), vec2(1., 0.), vec2(1., 1.), vec2(0., 1.)];
        let edges = convexhull_2d(&points).unwrap();
        assert_eq!(edges.len(), 4);
        check_outline(&points, &edges, 1e-9);
    }

    #[test]
    fn test_outline_ignores_interior_points() {
        let mut points = vec![vec2(0., 0.), vec2(3., 0.), vec2(3., 3.), vec2(0., 3.)];
        for i in 1..3 {
            for j in 1..3 {
                points.push(vec2(i as Float, j as Float));
            }
        }
        let edges = convexhull_2d(&points).unwrap();
        assert_eq!(edges.len(), 4);
        check_outline(&points, &edges, 1e-9);
        for edge in &edges {
            assert!(edge[0] < 4 && edge[1] < 4, "an interior point became a vertex");
        }
    }

    #[test]
    fn test_outline_circle_all_extreme() {
        let count = 64;
        let points: Vec<Vec2> = (0..count).map(|i| {
            let a = std::f64::consts::TAU * i as Float / count as Float;
            vec2(a.cos(), a.sin())
        }).collect();
        let edges = convexhull_2d(&points).unwrap();
        assert_eq!(edges.len(), count);
        check_outline(&points, &edges, 1e-9);
    }

    #[test]
    fn test_outline_collinear_points() {
        let points: Vec<Vec2> = (0..8).map(|i| vec2(i as Float, 0.)).collect();
        let edges = convexhull_2d(&points).unwrap();
        // joggling turns the segment into a very thin polygon
        check_outline(&points, &edges, 1e-9);
    }

    #[test]
    fn test_outline_identical_points() {
        // as in 3d, joggling separates the copies into a tiny cloud with a real outline
        let points = vec![vec2(-7., 4.); 50];
        let edges = convexhull_2d(&points).unwrap();
        assert!(edges.len() >= 3);
        for (i, edge) in edges.iter().enumerate() {
            assert_eq!(edge[1], edges[(i + 1) % edges.len()][0]);
        }
    }

    #[test]
    fn test_outline_too_few_points() {
        let points = [vec2(0., 0.), vec2(1., 0.)];
        assert_eq!(convexhull_2d(&points), Err(HullError::Incomplete));
    }

    #[test]
    fn test_outline_is_deterministic() {
        let points: Vec<Vec2> = (0..40).map(|i| {
            let a = std::f64::consts::TAU * i as Float / 40.;
            vec2(2. * a.cos(), a.sin())
        }).collect();
        assert_eq!(convexhull_2d(&points).unwrap(), convexhull_2d(&points).unwrap());
    }
}
