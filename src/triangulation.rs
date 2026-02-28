/*!
    Triangulation algorithms for polygon outlines

    Port of madcad.triangulation
*/

use crate::math::*;
use crate::hashing::{Asso, connpe};

use std::collections::HashSet;
use std::f64::consts::PI;
use rustc_hash::{FxHashMap, FxHashSet};
use multiversion::multiversion;


/// Triangulate a wire outline into a surface mesh
///
/// Returns a mesh with the triangles formed in the outline.
/// Points are treated as a sequential loop (indices 0, 1, 2, ..., n-1).
///
/// Complexity: O(n*k) where k is the number of non-convex points
#[multiversion(targets = "simd")]
pub fn triangulation_loop_d2(
    points: &[Vec2],
    prec: Float,
) -> Result<Vec<UVec3>, Vec<Index>> {
    let l = points.len();

    // Reducing contour, indexing points directly
    let mut hole: Vec<usize> = (0..l).collect();

    // Remove last point if it's the same as first (closed loop)
    if hole.len() >= 2 {
        let last_idx = hole.len() - 1;
        let first = points[hole[0]];
        let last = points[hole[last_idx]];
        // inverted condition to handle possible nan
        if ! ((last - first).square_length() > prec) {
            hole.pop();
        }
    }

    if hole.len() < 3 {
        return Ok(Vec::new());
    }

    // Set of remaining non-convex points, indexing points
    let mut nonconvex: HashSet<usize> = HashSet::with_capacity(l);
    for &i in &hole {
        let prev = if i == 0 { l - 1 } else { i - 1 };
        let next = (i + 1) % l;
        let pd = perpdot(points[i] - points[prev], points[next] - points[i]);
        if pd <= prec {
            nonconvex.insert(i);
        }
    }
    
    // Priority criterion for 2D triangles, depending on shape quality
    let priority = |u: Vec2, v: Vec2| -> Float {
        let uv = u.length() * v.length();
        if uv == 0.0 {
            0.0
        } else {
            (u.dot(v) + uv) / (uv * uv)
        }
    };

    // Score function for a hole position
    let score = |hole: &[usize], nonconvex: &HashSet<usize>, i: usize| -> Float {
        let hl = hole.len();
        let o = points[hole[i]];
        let u = points[hole[(i + 1) % hl]] - o;
        let v = points[hole[(i + hl - 1) % hl]] - o;
        let triangle = [hole[(i + hl - 1) % hl], hole[i], hole[(i + 1) % hl]];

        let pd = perpdot(u, v);

        // Check for badly oriented triangle
        if pd < -prec {
            return Float::NEG_INFINITY;
        }

        // Check for intersection with the rest
        if pd > prec {
            // Check that there is no point of the outline inside the triangle
            for &j in nonconvex.iter() {
                if !triangle.contains(&j) {
                    let mut inside = true;
                    for k in 0..3 {
                        let a = points[triangle[k]];
                        let b = points[triangle[(k + 2) % 3]]; // k-1 mod 3
                        let mut s = perpdot(a - b, points[j] - a);
                        if k == 1 {
                            s -= 2.0 * prec;
                        }
                        if s <= -prec {
                            inside = false;
                            break;
                        }
                    }
                    if inside {
                        return Float::NEG_INFINITY;
                    }
                }
            }
        } else if u.dot(v) >= prec {
            // Flat triangles that change direction are forbidden
            return Float::NEG_INFINITY;
        }

        priority(u, v)
    };

    // Initialize scores
    let mut scores: Vec<Float> = (0..hole.len())
        .map(|i| score(&hole, &nonconvex, i))
        .collect();

    let mut simplices = Vec::new();

    while hole.len() > 2 {
        let hl = hole.len();
        let (i, &best_score) = scores.iter()
            .enumerate()
            .fold((0, &Float::NEG_INFINITY), |(best_i, best_s), (i, s)| {
                if s >= best_s { (i, s) } else { (best_i, best_s) }
            });

        if best_score == Float::NEG_INFINITY {
            return Err(hole.iter().map(|&h| h as Index).collect());
        }

        // Add triangle (hole indices are the point indices directly)
        let a = hole[(i + hl - 1) % hl] as Index;
        let b = hole[i] as Index;
        let c = hole[(i + 1) % hl] as Index;
        simplices.push(Vector::from([a, b, c]));

        // Remove the point from the hole
        let removed = hole.remove(i);
        nonconvex.remove(&removed);
        scores.remove(i);

        if hole.len() <= 2 {
            break;
        }

        // Update neighboring scores
        let hl = hole.len();
        let prev = (i + hl - 1) % hl;
        let curr = i % hl;

        scores[prev] = score(&hole, &nonconvex, prev);
        scores[curr] = score(&hole, &nonconvex, curr);
    }

    Ok(simplices)
}


/// Point-to-edge segment distance
pub fn distance_pe(pt: Vec3, edge: [Vec3; 2]) -> Float {
    let dir = edge[1] - edge[0];
    let l = dir.square_length();
    if l == 0.0 {
        return 0.0;
    }
    let x = (pt - edge[0]).dot(dir) / l;
    if x < 0.0 {
        (pt - edge[0]).length()
    } else if x > 1.0 {
        (pt - edge[1]).length()
    } else {
        noproject(pt - edge[0], dir).length()
    }
}


/// Compute the area-weighted convex normal from points and edges
pub fn convex_normal(points: &[Vec3], edges: &[UVec2]) -> Vec3 {
    let mut area = Vec3::zero();
    let c = points[edges[0][0] as usize];
    for e in edges {
        let p0 = points[e[0] as usize];
        let p1 = points[e[1] as usize];
        area += (p1 - p0).cross(c - p0);
    }
    area.normalize()
}


/// BFS flood-fill from a start point through edge connectivity
fn propagate(
    start: Index,
    edges: &[UVec2],
    conn: &Asso<Index, usize>,
    reached_points: &mut FxHashMap<Index, (Float, UVec2)>,
    reached_edges: &mut FxHashMap<usize, (Float, UVec2)>,
    remain_points: &mut FxHashSet<Index>,
    remain_edges: &mut FxHashSet<usize>,
) {
    let mut front = vec![start];
    while let Some(s) = front.pop() {
        if reached_points.contains_key(&s) {
            continue;
        }
        reached_points.insert(s, (Float::INFINITY, UVec2::from([s, s])));
        remain_points.remove(&s);
        for &e in conn.get(&s) {
            if reached_edges.contains_key(&e) {
                continue;
            }
            reached_edges.insert(e, (Float::INFINITY, UVec2::from([s, s])));
            remain_edges.remove(&e);
            front.push(edges[e][0]);
            front.push(edges[e][1]);
        }
    }
}

/// Find bridge edges to connect all disconnected components of a web.
///
/// Returns pairs of point indices forming the bridge edges
/// (each bridge added in both directions).
///
/// Complexity: O(n^2 + n*k)
pub fn line_bridges(points: &[Vec3], edges: &[UVec2]) -> Vec<UVec2> {
    if edges.is_empty() {
        return Vec::new();
    }

    // connpe expects &[[Index; 2]], convert at boundary
    let raw_edges: Vec<[Index; 2]> = edges.iter().map(|e| *e.as_array()).collect();
    let conn = connpe(&raw_edges);
    let mut bridges: Vec<UVec2> = Vec::new();

    let mut reached_points: FxHashMap<Index, (Float, UVec2)> = FxHashMap::default();
    let mut reached_edges: FxHashMap<usize, (Float, UVec2)> = FxHashMap::default();
    let mut remain_points: FxHashSet<Index> =
        edges.iter().flat_map(|e| [e[0], e[1]]).collect();
    let mut remain_edges: FxHashSet<usize> = (0..edges.len()).collect();

    // initial propagation from the first edge's start point
    propagate(
        edges[0][0],
        edges,
        &conn,
        &mut reached_points,
        &mut reached_edges,
        &mut remain_points,
        &mut remain_edges,
    );

    while !remain_edges.is_empty() {
        // update closest for reached_points
        let pt_keys: Vec<Index> = reached_points.keys().copied().collect();
        for s in pt_keys {
            let (_, best) = reached_points[&s];
            if reached_points.contains_key(&best[0]) {
                // find_closest_point: nearest remaining edge to reached point s
                let mut best_score = Float::INFINITY;
                let mut best_bridge = UVec2::from([s, s]);
                for &ma in &remain_edges {
                    let ep = [
                        points[edges[ma][0] as usize],
                        points[edges[ma][1] as usize],
                    ];
                    let d = distance_pe(points[s as usize], ep);
                    if d < best_score {
                        let e = edges[ma];
                        best_score = d;
                        if (points[s as usize] - points[e[0] as usize]).square_length()
                            < (points[s as usize] - points[e[1] as usize]).square_length()
                        {
                            best_bridge = UVec2::from([e[0], s]);
                        } else {
                            best_bridge = UVec2::from([e[1], s]);
                        }
                    }
                }
                reached_points.insert(s, (best_score, best_bridge));
            }
        }

        // update closest for reached_edges
        let edge_keys: Vec<usize> = reached_edges.keys().copied().collect();
        for e_idx in edge_keys {
            let (_, best) = reached_edges[&e_idx];
            if reached_points.contains_key(&best[0]) {
                // find_closest_edge: nearest remaining point to reached edge e_idx
                let mut best_score = Float::INFINITY;
                let mut best_bridge = UVec2::from([0, 0]);
                let ep = [
                    points[edges[e_idx][0] as usize],
                    points[edges[e_idx][1] as usize],
                ];
                for &ma in &remain_points {
                    let d = distance_pe(points[ma as usize], ep);
                    if d < best_score {
                        let e = edges[e_idx];
                        best_score = d;
                        if (points[ma as usize] - points[e[0] as usize]).square_length()
                            < (points[ma as usize] - points[e[1] as usize]).square_length()
                        {
                            best_bridge = UVec2::from([ma, e[0]]);
                        } else {
                            best_bridge = UVec2::from([ma, e[1]]);
                        }
                    }
                }
                reached_edges.insert(e_idx, (best_score, best_bridge));
            }
        }

        // find minimum across both reached_points and reached_edges
        let min_point = reached_points
            .values()
            .min_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
        let min_edge = reached_edges
            .values()
            .min_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));

        let closest = match (min_point, min_edge) {
            (Some(p), Some(e)) => {
                if p.0 <= e.0 {
                    p.1
                } else {
                    e.1
                }
            }
            (Some(p), None) => p.1,
            (None, Some(e)) => e.1,
            (None, None) => break,
        };

        bridges.push(closest);
        bridges.push(UVec2::from([closest[1], closest[0]]));

        propagate(
            closest[0],
            edges,
            &conn,
            &mut reached_points,
            &mut reached_edges,
            &mut remain_points,
            &mut remain_edges,
        );
    }

    bridges
}


/// Extract closed oriented loops from a web of edges.
///
/// Each returned loop is a sequence of point indices forming a closed polygon.
/// Loops are oriented so that the most inward-turning edges are followed first.
pub fn flat_loops(
    points: &[Vec3],
    edges: &[UVec2],
    normal: Vec3,
) -> Result<Vec<Vec<Index>>, String> {
    if edges.is_empty() {
        return Ok(Vec::new());
    }

    let (_, _, z) = dirbase(normal, None);

    // connectivity: start point of each edge -> edge index
    let mut conn: Asso<Index, usize> = Asso::new();
    for (i, e) in edges.iter().enumerate() {
        conn.add(e[0], i);
    }

    let mut used = vec![false; edges.len()];

    // classify edges as doubled (has reverse edge) or not
    let mut doubled = Vec::new();
    let mut nondoubled = Vec::new();
    for (i, e) in edges.iter().enumerate() {
        let mut double = false;
        for &j in conn.get(&e[1]) {
            if edges[j][1] == e[0] {
                double = true;
                break;
            }
        }
        if double {
            doubled.push(i);
        } else {
            nondoubled.push(i);
        }
    }

    // candidates: nondoubled first, then doubled
    let candidates: Vec<usize> = nondoubled.into_iter().chain(doubled).collect();
    let mut choice_idx = 0;

    let mut loops = Vec::new();

    loop {
        // find next unused edge
        let end = loop {
            if choice_idx >= candidates.len() {
                return Ok(loops);
            }
            let i = candidates[choice_idx];
            choice_idx += 1;
            if !used[i] {
                break i;
            }
        };

        // start a loop with the two endpoints of the starting edge
        let mut lp: Vec<Index> = vec![edges[end][0], edges[end][1]];

        loop {
            let last = *lp.last().unwrap();
            let prev_pt = lp[lp.len() - 2];
            let prev_dir = (points[last as usize] - points[prev_pt as usize]).normalize();

            // find the most inward-turning next edge
            let mut best: Option<usize> = None;
            let mut best_score = Float::NEG_INFINITY;

            for &edge_idx in conn.get(&last) {
                if used[edge_idx] {
                    continue;
                }
                let e = edges[edge_idx];
                let dir = (points[e[1] as usize] - points[e[0] as usize]).normalize();

                let mut angle = if is_finite_vec(dir) && is_finite_vec(prev_dir) {
                    prev_dir.cross(dir).dot(z).atan2(prev_dir.dot(dir))
                } else {
                    -PI
                };

                // angles very close to pi wrap around to avoid ambiguity
                if PI - angle <= PI * NUMPREC {
                    angle -= 2.0 * PI;
                }

                if angle > best_score {
                    best_score = angle;
                    best = Some(edge_idx);
                }
            }

            let best = match best {
                Some(b) => b,
                None => return Err(format!("no continuation found for loop at point {}", last)),
            };

            used[best] = true;

            if best == end {
                break; // loop closed
            }

            lp.push(edges[best][1]);

            // detect premature loop closure (straight-line return to start)
            if lp[0] == *lp.last().unwrap() {
                let prev_dir =
                    (points[lp[lp.len() - 2] as usize] - points[lp[lp.len() - 1] as usize])
                        .normalize();
                let start_dir =
                    (points[lp[1] as usize] - points[lp[0] as usize]).normalize();
                let cos_val = prev_dir.dot(start_dir).clamp(-1.0, 1.0);
                if cos_val.acos() <= PI * NUMPREC {
                    used[end] = true;
                    break;
                }
            }
        }

        loops.push(lp);
    }
}


/// Signed area of a 2D polygon (shoelace formula).
/// Positive for CCW winding, negative for CW.
pub fn loop_area_2d(points: &[Vec2]) -> Float {
    let n = points.len();
    if n < 3 {
        return 0.0;
    }
    let mut area = 0.0;
    for i in 0..n {
        let j = (i + 1) % n;
        area += points[i][0] * points[j][1];
        area -= points[j][0] * points[i][1];
    }
    area * 0.5
}

/// Ray-casting point-in-polygon test for a 2D contour.
/// The contour is given as a sequence of points forming a closed polygon
/// (last point connects back to first).
pub fn point_in_loop_2d(point: Vec2, contour: &[Vec2]) -> bool {
    let n = contour.len();
    let mut inside = false;
    let mut j = n - 1;
    for i in 0..n {
        let pi = contour[i];
        let pj = contour[j];
        if ((pi[1] > point[1]) != (pj[1] > point[1]))
            && (point[0] < (pj[0] - pi[0]) * (point[1] - pi[1]) / (pj[1] - pi[1]) + pi[0])
        {
            inside = !inside;
        }
        j = i;
    }
    inside
}


/// A group of contours: one outer boundary and zero or more holes.
/// Each contour is a list of 2D points with a parallel list of original point indices.
struct LoopGroup {
    /// 2D points of the outer contour
    outer: Vec<Vec2>,
    /// Original point indices for the outer contour
    outer_indices: Vec<Index>,
    /// 2D points of each hole
    holes: Vec<Vec<Vec2>>,
    /// Original point indices for each hole
    holes_indices: Vec<Vec<Index>>,
}

/// Group oriented 2D loops into (outer + holes) sets.
///
/// Each input loop is a pair of (2D projected points, original point indices).
/// Loops are sorted by |area| descending. Each loop is either assigned as a hole
/// to an existing group whose outer contour contains it, or creates a new group.
fn group_loops(loops: Vec<(Vec<Vec2>, Vec<Index>)>) -> Vec<LoopGroup> {
    if loops.is_empty() {
        return Vec::new();
    }

    // Compute areas and sort by |area| descending
    let mut with_area: Vec<(Float, Vec<Vec2>, Vec<Index>)> = loops
        .into_iter()
        .map(|(pts, idx)| {
            let area = loop_area_2d(&pts);
            (area, pts, idx)
        })
        .collect();
    with_area.sort_by(|a, b| b.0.abs().partial_cmp(&a.0.abs()).unwrap_or(std::cmp::Ordering::Equal));

    let mut groups: Vec<LoopGroup> = Vec::new();

    for (_area, pts, idx) in with_area {
        if pts.is_empty() {
            continue;
        }
        // Test first point of this loop against existing groups' outers
        let test_point = pts[0];
        let target = groups.iter().position(|g| point_in_loop_2d(test_point, &g.outer));
        match target {
            Some(i) => {
                groups[i].holes.push(pts);
                groups[i].holes_indices.push(idx);
            }
            None => {
                groups.push(LoopGroup {
                    outer: pts,
                    outer_indices: idx,
                    holes: Vec::new(),
                    holes_indices: Vec::new(),
                });
            }
        }
    }

    groups
}

/// Triangulate a web of edges (possibly with multiple disconnected loops/holes).
///
/// Takes raw points and edges, extracts oriented loops, groups them by
/// containment, and triangulates each group using constrained Delaunay.
///
/// Returns triangle indices into the original `points` array.
pub fn triangulation_closest(
    points: &[Vec3],
    edges: &[UVec2],
    normal: Vec3,
    _prec: Float,
) -> Result<Vec<UVec3>, String> {
    use i_triangle::float::triangulatable::Triangulatable;

    let (x, y, _) = dirbase(normal, None);

    // find bridge edges to connect disconnected components
    let bridges = line_bridges(points, edges);

    // combine original edges with bridges
    let mut all_edges: Vec<UVec2> = edges.to_vec();
    all_edges.extend_from_slice(&bridges);

    // extract flat oriented loops
    let loops = flat_loops(points, &all_edges, normal)?;

    // project each loop to 2D and collect original indices
    // strip closing duplicate (flat_loops returns loops where last == first)
    let projected: Vec<(Vec<Vec2>, Vec<Index>)> = loops
        .into_iter()
        .map(|mut lp| {
            if lp.len() >= 2 && lp.first() == lp.last() {
                lp.pop();
            }
            lp
        })
        .filter(|lp| lp.len() >= 3)
        .map(|loop_indices| {
            let proj: Vec<Vec2> = loop_indices
                .iter()
                .map(|&idx| {
                    let p = points[idx as usize];
                    Vec2::from([p.dot(x), p.dot(y)])
                })
                .collect();
            (proj, loop_indices)
        })
        .collect();

    let mut result = Vec::new();

    for (proj, loop_indices) in projected {
        // collect unique input points with their original indices
        let mut input_points: Vec<([f64; 2], Index)> = Vec::new();
        let mut seen: FxHashSet<Index> = FxHashSet::default();
        for (p, &idx) in proj.iter().zip(loop_indices.iter()) {
            if seen.insert(idx) {
                input_points.push((p.into_array(), idx));
            }
        }

        // build single contour for iTriangle
        let contour: Vec<[f64; 2]> = proj.iter().map(|p| p.into_array()).collect();

        // triangulate with delaunay
        let triangulation = vec![contour]
            .triangulate()
            .into_delaunay()
            .to_triangulation::<u32>();

        // map output points to original indices by nearest-neighbor
        let output_map: Vec<Option<Index>> = triangulation.points
            .iter()
            .map(|op| {
                let mut best_idx = None;
                let mut best_dist = Float::INFINITY;
                for &(ip, idx) in &input_points {
                    let dx = op[0] - ip[0];
                    let dy = op[1] - ip[1];
                    let d = dx * dx + dy * dy;
                    if d < best_dist {
                        best_dist = d;
                        best_idx = Some(idx);
                    }
                }
                best_idx
            })
            .collect();

        for tri in triangulation.indices.chunks_exact(3) {
            let a = output_map[tri[0] as usize];
            let b = output_map[tri[1] as usize];
            let c = output_map[tri[2] as usize];
            if let (Some(a), Some(b), Some(c)) = (a, b, c) {
                result.push(UVec3::from([a, b, c]));
            }
        }
    }

    Ok(result)
}


#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_distance_pe_on_segment() {
        let edge = [
            Vec3::from([0.0, 0.0, 0.0]),
            Vec3::from([2.0, 0.0, 0.0]),
        ];
        // point directly above middle of segment
        let d = distance_pe(Vec3::from([1.0, 1.0, 0.0]), edge);
        assert!((d - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_distance_pe_before_segment() {
        let edge = [
            Vec3::from([0.0, 0.0, 0.0]),
            Vec3::from([2.0, 0.0, 0.0]),
        ];
        // point behind start of segment
        let d = distance_pe(Vec3::from([-1.0, 0.0, 0.0]), edge);
        assert!((d - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_distance_pe_after_segment() {
        let edge = [
            Vec3::from([0.0, 0.0, 0.0]),
            Vec3::from([2.0, 0.0, 0.0]),
        ];
        // point beyond end of segment
        let d = distance_pe(Vec3::from([3.0, 0.0, 0.0]), edge);
        assert!((d - 1.0).abs() < 1e-10);
    }

    fn e(a: Index, b: Index) -> UVec2 { UVec2::from([a, b]) }

    #[test]
    fn test_convex_normal_triangle() {
        let points = vec![
            Vec3::from([0.0, 0.0, 0.0]),
            Vec3::from([1.0, 0.0, 0.0]),
            Vec3::from([0.0, 1.0, 0.0]),
        ];
        let edges = vec![e(0, 1), e(1, 2), e(2, 0)];
        let n = convex_normal(&points, &edges);
        // normal should point in +z or -z direction
        assert!(n[2].abs() > 0.99);
    }

    #[test]
    fn test_line_bridges_single_component() {
        // triangle: already connected, no bridges needed
        let points = vec![
            Vec3::from([0.0, 0.0, 0.0]),
            Vec3::from([1.0, 0.0, 0.0]),
            Vec3::from([0.0, 1.0, 0.0]),
        ];
        let edges = vec![e(0, 1), e(1, 2), e(2, 0)];
        let bridges = line_bridges(&points, &edges);
        assert!(bridges.is_empty());
    }

    #[test]
    fn test_line_bridges_two_components() {
        // two separate triangles
        let points = vec![
            Vec3::from([0.0, 0.0, 0.0]),
            Vec3::from([1.0, 0.0, 0.0]),
            Vec3::from([0.0, 1.0, 0.0]),
            Vec3::from([10.0, 0.0, 0.0]),
            Vec3::from([11.0, 0.0, 0.0]),
            Vec3::from([10.0, 1.0, 0.0]),
        ];
        let edges = vec![e(0, 1), e(1, 2), e(2, 0), e(3, 4), e(4, 5), e(5, 3)];
        let bridges = line_bridges(&points, &edges);
        // should have 2 bridge edges (one in each direction)
        assert_eq!(bridges.len(), 2);
        // bridges should connect the two components
        let b = bridges[0];
        let comp1: FxHashSet<Index> = [0, 1, 2].into_iter().collect();
        let comp2: FxHashSet<Index> = [3, 4, 5].into_iter().collect();
        assert!(
            (comp1.contains(&b[0]) && comp2.contains(&b[1]))
                || (comp2.contains(&b[0]) && comp1.contains(&b[1]))
        );
    }

    #[test]
    fn test_flat_loops_triangle() {
        let points = vec![
            Vec3::from([0.0, 0.0, 0.0]),
            Vec3::from([1.0, 0.0, 0.0]),
            Vec3::from([0.0, 1.0, 0.0]),
        ];
        let edges = vec![e(0, 1), e(1, 2), e(2, 0)];
        let normal = Vec3::from([0.0, 0.0, 1.0]);
        let loops = flat_loops(&points, &edges, normal).unwrap();
        assert_eq!(loops.len(), 1);
        // loop includes closing duplicate: [0, 1, 2, 0]
        assert_eq!(loops[0].len(), 4);
        assert_eq!(loops[0].first(), loops[0].last());
    }

    #[test]
    fn test_flat_loops_square() {
        let points = vec![
            Vec3::from([0.0, 0.0, 0.0]),
            Vec3::from([1.0, 0.0, 0.0]),
            Vec3::from([1.0, 1.0, 0.0]),
            Vec3::from([0.0, 1.0, 0.0]),
        ];
        let edges = vec![e(0, 1), e(1, 2), e(2, 3), e(3, 0)];
        let normal = Vec3::from([0.0, 0.0, 1.0]);
        let loops = flat_loops(&points, &edges, normal).unwrap();
        assert_eq!(loops.len(), 1);
        // loop includes closing duplicate: [0, 1, 2, 3, 0]
        assert_eq!(loops[0].len(), 5);
        assert_eq!(loops[0].first(), loops[0].last());
    }

    #[test]
    fn test_triangulation_closest_triangle() {
        let points = vec![
            Vec3::from([0.0, 0.0, 0.0]),
            Vec3::from([1.0, 0.0, 0.0]),
            Vec3::from([0.0, 1.0, 0.0]),
        ];
        let edges = vec![e(0, 1), e(1, 2), e(2, 0)];
        let normal = Vec3::from([0.0, 0.0, 1.0]);
        let result = triangulation_closest(&points, &edges, normal, 1e-6).unwrap();
        assert_eq!(result.len(), 1);
        // all three point indices should be present in the triangle
        let tri = result[0];
        let mut indices: Vec<Index> = vec![tri[0], tri[1], tri[2]];
        indices.sort();
        assert_eq!(indices, vec![0, 1, 2]);
    }

    #[test]
    fn test_triangulation_closest_square() {
        let points = vec![
            Vec3::from([0.0, 0.0, 0.0]),
            Vec3::from([1.0, 0.0, 0.0]),
            Vec3::from([1.0, 1.0, 0.0]),
            Vec3::from([0.0, 1.0, 0.0]),
        ];
        let edges = vec![e(0, 1), e(1, 2), e(2, 3), e(3, 0)];
        let normal = Vec3::from([0.0, 0.0, 1.0]);
        let result = triangulation_closest(&points, &edges, normal, 1e-6).unwrap();
        // square should produce 2 triangles
        assert_eq!(result.len(), 2);
    }
}
