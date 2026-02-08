/*!
    Boolean operations on meshes

    Port of madcad.boolean - union, intersection, difference
*/

use crate::math::*;
use crate::mesh::*;
use crate::hashing::*;
use crate::triangulation::{triangulation_closest, distance_pe};
use rustc_hash::{FxHashMap, FxHashSet};
use std::borrow::Cow;

/// Result vertex from triangle-triangle intersection
pub struct IntersectionVertex {
    /// face index (0 or 1)
    pub face: usize,
    /// edge index on that face (0, 1, or 2)
    pub edge: usize,
    /// 3D intersection point
    pub point: Vec3,
}

/// Sign function for floats, returns -1, 0, or 1 as i32
fn dsign(v: Float) -> i32 {
    if v > 0.0 { 1 }
    else if v < 0.0 { -1 }
    else { 0 }
}

/// Check if all components of a Vec3 are finite
fn is_finite_vec3(v: Vec3) -> bool {
    v.as_array().iter().all(|x| x.is_finite())
}

/// Intersect two triangles and output intersection vertices
///
/// fa = first face (3 vertices in clockwise orientation)
/// fb = second face (3 vertices)
/// prec = precision threshold
///
/// Returns None if no intersection, or two intersection vertices:
///   ((fi, ej, xj), (fk, em, xm)) where
///     fi, fk = face id (0 or 1)
///     ej, em = edge id on face (0-2)
///     xj, xm = intersection point
pub fn intersect_triangles(
    fa: &[Vec3; 3],
    fb: &[Vec3; 3],
    prec: Float,
) -> Option<(IntersectionVertex, IntersectionVertex)> {
    // normals of both faces
    let a1a2 = fa[1] - fa[0];
    let a1a3 = fa[2] - fa[0];
    let n_a = a1a2.cross(a1a3).normalize();

    let b1b2 = fb[1] - fb[0];
    let b1b3 = fb[2] - fb[0];
    let n_b = b1b2.cross(b1b3).normalize();

    // intersection line direction between the two planes
    let d1 = n_a.cross(n_b);
    let ld1 = d1.square_length().sqrt();
    if ld1 <= prec {
        // coplanar or parallel faces
        return None;
    }
    let d = d1 * (1.0 / ld1);

    // projection directions onto d from fA and fB
    let t_a = n_a.cross(d);
    let t_b = n_b.cross(d);

    // project fA vertices onto line d
    let p_a1 = fa[0] - t_a * ((fa[0] - fb[0]).dot(n_b) / t_a.dot(n_b));
    let x_a = [0.0, a1a2.dot(d), a1a3.dot(d)];
    let pf_a = [
        p_a1,
        p_a1 + d * x_a[1],
        p_a1 + d * x_a[2],
    ];

    // project fB vertices onto line d
    let x_b = [
        (fb[0] - fa[0]).dot(d),
        (fb[1] - fa[0]).dot(d),
        (fb[2] - fa[0]).dot(d),
    ];
    let pf_b = [
        p_a1 + d * x_b[0],
        p_a1 + d * x_b[1],
        p_a1 + d * x_b[2],
    ];

    // project vertices onto transversal directions tA and tB
    let mut y_a = [0.0; 3];
    let mut y_b = [0.0; 3];
    for i in 0..3 {
        y_a[i] = (fa[i] - pf_a[i]).dot(t_a);
        y_b[i] = (fb[i] - pf_b[i]).dot(t_b);
    }

    // identify signs of yA and yB
    let mut s_ya = [0i32; 3];
    let mut s_yb = [0i32; 3];
    for i in 0..3 {
        if y_a[i].abs() <= prec {
            s_ya[i] = 0;
            y_a[i] = 0.0;
        } else {
            s_ya[i] = dsign(y_a[i]);
        }
        if y_b[i].abs() <= prec {
            s_yb[i] = 0;
            y_b[i] = 0.0;
        } else {
            s_yb[i] = dsign(y_b[i]);
        }
    }

    // check if triangles have no intersection with line D
    if (s_ya[0] + s_ya[1] + s_ya[2]).abs() == 3
    || (s_yb[0] + s_yb[1] + s_yb[2]).abs() == 3 {
        return None;
    }

    // find edges of intersection on A and B
    // edge i connects vertex i and vertex (i+1)%3
    let mut e_ia = [0usize; 3];
    let mut e_ib = [0usize; 3];
    let mut ne_ia = 0usize;
    let mut ne_ib = 0usize;

    // prioritize edges really getting through the face (not just touching)
    let j_start_a = (0..3)
        .find(|&j| s_ya[j] * s_ya[(j + 1) % 3] < 0)
        .unwrap_or(2);
    // collect intersecting edges starting from the through-edge
    for k in j_start_a..j_start_a + 3 {
        let i = k % 3;
        let next = (i + 1) % 3;
        if s_ya[i] * s_ya[next] <= 0 && (s_ya[i].abs() + s_ya[next].abs()) > 0 {
            e_ia[ne_ia] = i;
            ne_ia += 1;
        }
    }

    // same for B
    let j_start_b = (0..3)
        .find(|&j| s_yb[j] * s_yb[(j + 1) % 3] < 0)
        .unwrap_or(2);
    for k in j_start_b..j_start_b + 3 {
        let i = k % 3;
        let next = (i + 1) % 3;
        if s_yb[i] * s_yb[next] <= 0 && (s_yb[i].abs() + s_yb[next].abs()) > 0 {
            e_ib[ne_ib] = i;
            ne_ib += 1;
        }
    }

    if ne_ia == 1 { e_ia[1] = e_ia[0]; }
    if ne_ib == 1 { e_ib[1] = e_ib[0]; }

    // intersection coordinates onto line D
    let mut x_ia = [0.0; 2];
    let mut x_ib = [0.0; 2];
    for i in 0..2 {
        let ea = e_ia[i];
        let ea_next = (ea + 1) % 3;
        x_ia[i] = (y_a[ea_next] * x_a[ea] - y_a[ea] * x_a[ea_next])
                 / (y_a[ea_next] - y_a[ea]);

        let eb = e_ib[i];
        let eb_next = (eb + 1) % 3;
        x_ib[i] = (y_b[eb_next] * x_b[eb] - y_b[eb] * x_b[eb_next])
                 / (y_b[eb_next] - y_b[eb]);
    }

    // intervals of intersections: pi = index of max, mi = index of min
    let (pi_a, mi_a) = if x_ia[0] > x_ia[1] { (0, 1) } else { (1, 0) };
    let (pi_b, mi_b) = if x_ib[0] > x_ib[1] { (0, 1) } else { (1, 0) };

    // helper to create intersection point on line D
    let make_point = |x: Float| -> Vec3 { p_a1 + d * x };

    // one intersection at the border of the intervals
    if (x_ia[pi_a] - x_ib[mi_b]).abs() <= prec {
        return Some((
            IntersectionVertex { face: 0, edge: e_ia[pi_a], point: make_point(x_ia[pi_a]) },
            IntersectionVertex { face: 1, edge: e_ib[mi_b], point: make_point(x_ib[mi_b]) },
        ));
    }
    if (x_ib[pi_b] - x_ia[mi_a]).abs() <= prec {
        return Some((
            IntersectionVertex { face: 0, edge: e_ia[mi_a], point: make_point(x_ia[mi_a]) },
            IntersectionVertex { face: 1, edge: e_ib[pi_b], point: make_point(x_ib[pi_b]) },
        ));
    }

    // no intersection - intervals don't cross
    if x_ib[pi_b] - prec < x_ia[mi_a] || x_ia[pi_a] - prec < x_ib[mi_b] {
        return None;
    }

    // one interval is included in the other
    if x_ib[mi_b] - prec <= x_ia[mi_a] && x_ia[pi_a] - prec <= x_ib[pi_b] {
        // edges of A cross face B
        return Some((
            IntersectionVertex { face: 0, edge: e_ia[mi_a], point: make_point(x_ia[mi_a]) },
            IntersectionVertex { face: 0, edge: e_ia[pi_a], point: make_point(x_ia[pi_a]) },
        ));
    }
    if x_ia[mi_a] - prec <= x_ib[mi_b] && x_ib[pi_b] - prec <= x_ia[pi_a] {
        // B interval contained in A - give priority to face 0 when equivalent
        let mr = if (x_ia[mi_a] - x_ib[mi_b]).abs() <= prec {
            IntersectionVertex { face: 0, edge: e_ia[mi_a], point: make_point(x_ia[mi_a]) }
        } else {
            IntersectionVertex { face: 1, edge: e_ib[mi_b], point: make_point(x_ib[mi_b]) }
        };
        let pr = if (x_ib[pi_b] - x_ia[pi_a]).abs() <= prec {
            IntersectionVertex { face: 0, edge: e_ia[pi_a], point: make_point(x_ia[pi_a]) }
        } else {
            IntersectionVertex { face: 1, edge: e_ib[pi_b], point: make_point(x_ib[pi_b]) }
        };
        return Some((mr, pr));
    }

    // intervals cross each other
    if x_ib[mi_b] > x_ia[mi_a] - prec && x_ia[pi_a] - prec < x_ib[pi_b] {
        return Some((
            IntersectionVertex { face: 0, edge: e_ia[pi_a], point: make_point(x_ia[pi_a]) },
            IntersectionVertex { face: 1, edge: e_ib[mi_b], point: make_point(x_ib[mi_b]) },
        ));
    }
    if x_ia[mi_a] > x_ib[mi_b] - prec && x_ib[pi_b] - prec < x_ia[pi_a] {
        return Some((
            IntersectionVertex { face: 0, edge: e_ia[mi_a], point: make_point(x_ia[mi_a]) },
            IntersectionVertex { face: 1, edge: e_ib[pi_b], point: make_point(x_ib[pi_b]) },
        ));
    }

    unreachable!("unexpected case in intersect_triangles");
}


// ---- Line simplification ----

/// Find collinear points to merge in a set of edges.
///
/// Returns a dictionary mapping point indices to their merge targets.
/// Port of madcad.mesh.line_simplification
fn line_simplification(
    points: &[Vec3],
    edges: &[UVec2],
    prec: Float,
) -> FxHashMap<Index, Index> {
    let edges_raw: Vec<[Index; 2]> = edges.iter().map(|e| *e.as_array()).collect();
    let chains = suites(&edges_raw, false, false, false);

    let mut merges: FxHashMap<Index, Index> = FxHashMap::default();

    for line in &chains {
        if line.len() < 3 { continue; }

        let mut s = line[0];
        for i in 2..line.len() {
            let a = s;
            let b = line[i - 1];
            let c = line[i];
            let height = noproject(
                points[c as usize] - points[b as usize],
                points[c as usize] - points[a as usize],
            ).square_length().sqrt();

            if height > prec {
                s = b;
            } else {
                merges.insert(b, a);
            }
        }

        // Handle closed loop
        if line.first() == line.last() && line.len() >= 3 {
            let a = s;
            let b = line[0];
            let c = line[1];
            let height = noproject(
                points[c as usize] - points[b as usize],
                points[c as usize] - points[a as usize],
            ).square_length().sqrt();
            if height <= prec {
                merges.insert(b, a);
            }
        }
    }

    // Resolve merge chains
    let keys: Vec<Index> = merges.keys().copied().collect();
    for k in keys {
        let mut v = merges[&k];
        while let Some(&next) = merges.get(&v) {
            if next == v { break; }
            v = next;
        }
        merges.insert(k, v);
    }

    merges
}

/// Apply index merges to edges, removing degenerate edges where both endpoints merge to same
fn apply_merges(
    edges: &mut Vec<UVec2>,
    tracks: &mut Vec<Index>,
    merges: &FxHashMap<Index, Index>,
) {
    for e in edges.iter_mut() {
        let a = e[0];
        let b = e[1];
        *e = UVec2::from([
            merges.get(&a).copied().unwrap_or(a),
            merges.get(&b).copied().unwrap_or(b),
        ]);
    }

    let mut i = 0;
    while i < edges.len() {
        if edges[i][0] == edges[i][1] {
            edges.swap_remove(i);
            tracks.swap_remove(i);
        } else {
            i += 1;
        }
    }
}


// ---- cut_surface ----

/// Cut m1 faces at their intersections with m2.
///
/// Returns `(cutted, frontier)` where:
/// - `cutted` is m1 with intersected faces retriangulated
/// - `frontier` is a Web whose edges are intersection edges, tracks are the causing m2 face indices
///
/// Port of madcad.boolean.cut_surface
pub fn cut_surface(
    m1: &Surface,
    m2: &Surface,
    prec: Float,
) -> (Surface<'static>, Web<'static>) {
    // Point set wrapping m1's points (allows adding new intersection points)
    let mut points = PointSet::wrap(prec, m1.points.to_vec());

    // Spatial proximity map for m2 faces
    let m1_bounds = m1.bounds();
    let m2_bounds = m2.bounds();
    let m1_size = m1_bounds.max - m1_bounds.min;
    let m2_size = m2_bounds.max - m2_bounds.min;
    let cellsize = meshcellsize(m1_size, m1.points.len())
        .max(meshcellsize(m2_size, m2.points.len()));

    let mut prox2: PositionMap<usize> = PositionMap::new(cellsize);
    for (f2, _face) in m2.simplices.iter().enumerate() {
        let pts = m2.simplexpoints(f2);
        prox2.add_triangle(&pts, f2);
    }

    // Edge-to-face connectivity for m1
    let faces_raw: Vec<[Index; 3]> = m1.simplices.iter().map(|f| *f.as_array()).collect();
    let conn = connef(&faces_raw);

    // Result accumulators
    let mut result_faces: Vec<UVec3> = Vec::new();
    let mut result_tracks: Vec<Index> = Vec::new();
    let mut frontier_edges: Vec<UVec2> = Vec::new();
    let mut frontier_tracks: Vec<Index> = Vec::new();

    // Flat region grouping
    let mut grp: Vec<i32> = vec![-1; m1.simplices.len()];
    let mut currentgrp: i32 = -1;

    for i in 0..m1.simplices.len() {
        // Skip already grouped faces
        if grp[i] != -1 { continue; }

        // Check if face i is near m2
        let face_pts = m1.simplexpoints(i);
        let face_keys = prox2.keysfor_triangle(&face_pts);
        if !prox2.contains(&face_keys) { continue; }

        let normal = m1.facenormal(i);
        if !is_finite_vec3(normal) { continue; }
        let track = m1.tracks[i];

        // Flood-fill flat coplanar region
        currentgrp += 1;
        let mut surf: Vec<usize> = Vec::new();
        let mut front = vec![i];

        while let Some(fi) = front.pop() {
            if grp[fi] != -1 { continue; }
            if m1.tracks[fi] != track { continue; }
            let fi_normal = m1.facenormal(fi);
            if (fi_normal - normal).square_length() > NUMPREC { continue; }

            surf.push(fi);
            grp[fi] = currentgrp;

            let f = m1.simplices[fi];
            for &(a, b) in &[(f[1], f[0]), (f[2], f[1]), (f[0], f[2])] {
                if let Some(&next) = conn.get(&(a, b)) {
                    front.push(next);
                }
            }
        }

        // Get oriented outline: boundary edges of the flat region
        let mut outline: FxHashSet<(Index, Index)> = FxHashSet::default();
        for &f1 in &surf {
            let f = m1.simplices[f1];
            for &(a, b) in &[(f[1], f[0]), (f[2], f[1]), (f[0], f[2])] {
                if outline.contains(&(a, b)) {
                    outline.remove(&(a, b));
                } else {
                    outline.insert((b, a));
                }
            }
        }
        let original: FxHashSet<(Index, Index)> = outline.clone();

        // Find triangle-triangle intersections
        let mut segts: FxHashMap<(Index, Index), usize> = FxHashMap::default();

        for &f1 in &surf {
            let f = m1.simplices[f1];
            let f1_pts = m1.simplexpoints(f1);
            let f1_keys = prox2.keysfor_triangle(&f1_pts);
            let nearby: FxHashSet<usize> = prox2.get(&f1_keys).copied().collect();

            for f2 in nearby {
                let f2_pts = m2.simplexpoints(f2);
                let (iv_a, iv_b) = match intersect_triangles(&f1_pts, &f2_pts, 8.0 * prec) {
                    Some(r) => r,
                    None => continue,
                };

                let ia = iv_a.point;
                let ib = iv_b.point;
                if (ia - ib).square_length() <= prec * prec { continue; }

                let ia_idx = points.add(ia);
                let ib_idx = points.add(ib);
                let seg = (ia_idx, ib_idx);

                if segts.contains_key(&seg) { continue; }
                segts.insert(seg, f2);

                // Cut outline edges where intersection points land on boundary
                let ivs = [&iv_a, &iv_b];
                let seg_indices = [ia_idx, ib_idx];
                let seg_positions = [ia, ib];

                for idx in 0..2 {
                    let iv = ivs[idx];
                    if iv.face != 0 { continue; } // must be on m1's face

                    // edge on m1's face: vertex edge → vertex (edge+1)%3
                    let o = (f[iv.edge], f[(iv.edge + 1) % 3]);
                    if !original.contains(&o) { continue; }

                    let p = seg_positions[idx];
                    let pt_idx = seg_indices[idx];

                    // Find closest outline edge to point p
                    let closest = outline.iter()
                        .min_by(|&&e1, &&e2| {
                            let d1 = distance_pe(p, &[
                                points.points()[e1.0 as usize],
                                points.points()[e1.1 as usize],
                            ]);
                            let d2 = distance_pe(p, &[
                                points.points()[e2.0 as usize],
                                points.points()[e2.1 as usize],
                            ]);
                            d1.partial_cmp(&d2).unwrap_or(std::cmp::Ordering::Equal)
                        })
                        .copied();

                    if let Some(e) = closest {
                        if pt_idx != e.0 && pt_idx != e.1 {
                            outline.remove(&e);
                            outline.insert((e.0, pt_idx));
                            outline.insert((pt_idx, e.1));
                        }
                    }
                }
            }
        }

        // Build intersection edge/track vectors
        let mut segt_edges: Vec<UVec2> = Vec::new();
        let mut segt_tracks: Vec<Index> = Vec::new();
        for (&(a, b), &f2) in &segts {
            segt_edges.push(UVec2::from([a, b]));
            segt_tracks.push(f2 as Index);
        }

        // Line simplification
        let merges = line_simplification(points.points(), &segt_edges, prec);
        apply_merges(&mut segt_edges, &mut segt_tracks, &merges);

        // Add to frontier
        frontier_edges.extend_from_slice(&segt_edges);
        frontier_tracks.extend_from_slice(&segt_tracks);

        // Build edges for triangulation: intersection (both dirs) + outline
        let mut tri_edges: Vec<UVec2> = Vec::new();
        for e in &segt_edges {
            tri_edges.push(*e);
            tri_edges.push(UVec2::from([e[1], e[0]]));
        }
        for &(a, b) in &outline {
            tri_edges.push(UVec2::from([a, b]));
        }

        // Triangulate and add results
        if let Ok(tris) = triangulation_closest(points.points(), &tri_edges, normal, prec) {
            for tri in tris {
                result_faces.push(tri);
                result_tracks.push(track);
            }
        }
    }

    // Append non-intersected faces
    for (fi, (&f, &t)) in m1.simplices.iter().zip(m1.tracks.iter()).enumerate() {
        if grp[fi] == -1 {
            result_faces.push(f);
            result_tracks.push(t);
        }
    }

    let final_points = points.unwrap();

    (
        Surface {
            points: Cow::Owned(final_points.clone()),
            simplices: Cow::Owned(result_faces),
            tracks: Cow::Owned(result_tracks),
        },
        Web {
            points: Cow::Owned(final_points),
            simplices: Cow::Owned(frontier_edges),
            tracks: Cow::Owned(frontier_tracks),
        },
    )
}


// ---- pierce_surface ----

/// Pierce m1 with m2: cut and select faces on one side of the piercing surface.
///
/// - `side`: false keeps the outside, true keeps the inside
/// - `strict`: if true, error when a face is on both sides of the piercing surface
///
/// Port of madcad.boolean.pierce_surface
pub fn pierce_surface(
    m1: &Surface,
    m2: &Surface,
    side: bool,
    prec: Float,
    strict: bool,
) -> Result<Surface<'static>, String> {
    let (cut, frontier) = cut_surface(m1, m2, prec);

    let faces_raw: Vec<[Index; 3]> = cut.simplices.iter().map(|f| *f.as_array()).collect();
    let mut conn1 = connef(&faces_raw);

    let stops: FxHashSet<(Index, Index)> = frontier.simplices.iter()
        .map(|e| edgekey(e[0], e[1]))
        .collect();

    let mut used: Vec<i32> = vec![0; cut.simplices.len()];
    let mut front: Vec<(Index, Index)> = Vec::new();

    // Seed propagation from frontier faces
    for (e, &f2_track) in frontier.simplices.iter().zip(frontier.tracks.iter()) {
        let f2 = f2_track as usize;
        for &edge in &[(e[0], e[1]), (e[1], e[0])] {
            if let Some(&fi) = conn1.get(&edge) {
                let f = cut.simplices[fi];
                for i in 0..3usize {
                    let fi_pt = f[i];
                    if fi_pt != edge.0 && fi_pt != edge.1 {
                        // fi_pt is the vertex opposite to the frontier edge
                        let prev = f[(i + 2) % 3]; // f[i-1] in Python
                        let next_v = f[(i + 1) % 3]; // f[i-2] in Python
                        let m2_normal = m2.facenormal(f2);
                        let proj = (cut.points[fi_pt as usize] - cut.points[prev as usize])
                            .dot(m2_normal)
                            * if side { -1.0 } else { 1.0 };

                        if proj > prec {
                            used[fi] = 1;
                        } else if proj < -prec {
                            used[fi] = -1;
                        } else {
                            break;
                        }

                        conn1.insert((prev, fi_pt), fi);
                        conn1.insert((fi_pt, next_v), fi);
                        front.push((fi_pt, prev));
                        front.push((next_v, fi_pt));
                        break;
                    }
                }
            }
        }
    }

    if front.is_empty() {
        if side {
            return Ok(Surface {
                points: Cow::Owned(cut.points.into_owned()),
                simplices: Cow::Owned(Vec::new()),
                tracks: Cow::Owned(Vec::new()),
            });
        }
        return Ok(cut.into_owned());
    }

    // Filter front: remove edges on the frontier stops
    front.retain(|e| !stops.contains(&edgekey(e.0, e.1)));

    // Propagation
    while !front.is_empty() {
        let mut newfront: Vec<(Index, Index)> = Vec::new();

        for &edge in &front {
            if let Some(&fi) = conn1.get(&edge) {
                let reverse = (edge.1, edge.0);
                let source = match conn1.get(&reverse) {
                    Some(&s) => s,
                    None => continue,
                };
                if used[source] == 0 { continue; }

                if used[fi] == 0
                    || (used[source] * used[fi] < 0 && used[fi].abs() > used[source].abs())
                {
                    if strict && used[fi] != 0 {
                        return Err(
                            "the pierced surface is both side of the piercing surface".into(),
                        );
                    }
                    used[fi] = used[source] + if used[source] > 0 { 1 } else { -1 };

                    let f = cut.simplices[fi];
                    for i in 0..3usize {
                        let prev = f[(i + 2) % 3]; // f[i-1]
                        let curr = f[i];
                        if !stops.contains(&edgekey(prev, curr)) {
                            conn1.insert((prev, curr), fi);
                            newfront.push((curr, prev));
                        }
                    }
                }
            }
        }

        front = newfront;
    }

    // Filter faces: keep those with used > 0
    let mut out_faces = Vec::new();
    let mut out_tracks = Vec::new();
    for (i, (&f, &t)) in cut.simplices.iter().zip(cut.tracks.iter()).enumerate() {
        if used[i] > 0 {
            out_faces.push(f);
            out_tracks.push(t);
        }
    }

    Ok(Surface {
        points: Cow::Owned(cut.points.into_owned()),
        simplices: Cow::Owned(out_faces),
        tracks: Cow::Owned(out_tracks),
    })
}


#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_intersect_triangles_crossing() {
        // two triangles crossing each other
        let fa = [
            Vec3::from([-1.0, -1.0, 0.0]),
            Vec3::from([ 1.0, -1.0, 0.0]),
            Vec3::from([ 0.0,  1.0, 0.0]),
        ];
        let fb = [
            Vec3::from([0.0, 0.0, -1.0]),
            Vec3::from([0.0, 0.0,  1.0]),
            Vec3::from([0.0, 2.0,  0.0]),
        ];
        let result = intersect_triangles(&fa, &fb, 1e-6);
        assert!(result.is_some());
    }

    #[test]
    fn test_intersect_triangles_parallel() {
        // two parallel triangles - no intersection
        let fa = [
            Vec3::from([0.0, 0.0, 0.0]),
            Vec3::from([1.0, 0.0, 0.0]),
            Vec3::from([0.0, 1.0, 0.0]),
        ];
        let fb = [
            Vec3::from([0.0, 0.0, 1.0]),
            Vec3::from([1.0, 0.0, 1.0]),
            Vec3::from([0.0, 1.0, 1.0]),
        ];
        let result = intersect_triangles(&fa, &fb, 1e-6);
        assert!(result.is_none());
    }

    #[test]
    fn test_intersect_triangles_no_overlap() {
        // planes intersect but triangles don't overlap
        let fa = [
            Vec3::from([0.0, 0.0, 0.0]),
            Vec3::from([1.0, 0.0, 0.0]),
            Vec3::from([0.0, 1.0, 0.0]),
        ];
        let fb = [
            Vec3::from([5.0, 0.0, -1.0]),
            Vec3::from([5.0, 0.0,  1.0]),
            Vec3::from([5.0, 1.0,  0.0]),
        ];
        let result = intersect_triangles(&fa, &fb, 1e-6);
        assert!(result.is_none());
    }

    // ---- cut_surface / pierce_surface tests ----

    /// Build a simple quad (two triangles) in the XY plane
    fn make_quad(z: Float, size: Float) -> Surface<'static> {
        let points = vec![
            Vec3::from([-size, -size, z]),
            Vec3::from([ size, -size, z]),
            Vec3::from([ size,  size, z]),
            Vec3::from([-size,  size, z]),
        ];
        Surface {
            points: Cow::Owned(points),
            simplices: Cow::Owned(vec![
                UVec3::from([0, 1, 2]),
                UVec3::from([0, 2, 3]),
            ]),
            tracks: Cow::Owned(vec![0, 0]),
        }
    }

    /// Build a vertical quad (two triangles) in the XZ plane
    fn make_vertical_quad(y: Float, size: Float) -> Surface<'static> {
        let points = vec![
            Vec3::from([-size, y, -size]),
            Vec3::from([ size, y, -size]),
            Vec3::from([ size, y,  size]),
            Vec3::from([-size, y,  size]),
        ];
        Surface {
            points: Cow::Owned(points),
            simplices: Cow::Owned(vec![
                UVec3::from([0, 1, 2]),
                UVec3::from([0, 2, 3]),
            ]),
            tracks: Cow::Owned(vec![0, 0]),
        }
    }

    #[test]
    fn test_cut_mesh_crossing_quads() {
        let m1 = make_quad(0.0, 2.0);
        let m2 = make_vertical_quad(0.0, 2.0);
        let (cut, frontier) = cut_surface(&m1, &m2, 1e-6);

        // The cut mesh should have more faces than original (faces were split)
        assert!(cut.simplices.len() >= m1.simplices.len());
        // Frontier should have intersection edges
        assert!(!frontier.simplices.is_empty());
    }

    #[test]
    fn test_cut_mesh_no_intersection() {
        let m1 = make_quad(0.0, 1.0);
        let m2 = make_quad(5.0, 1.0); // far away, parallel
        let (cut, frontier) = cut_surface(&m1, &m2, 1e-6);

        // No intersection: cut mesh should have same faces
        assert_eq!(cut.simplices.len(), m1.simplices.len());
        assert!(frontier.simplices.is_empty());
    }

    #[test]
    fn test_pierce_mesh_crossing_quads() {
        let m1 = make_quad(0.0, 2.0);
        let m2 = make_vertical_quad(0.0, 2.0);
        let result = pierce_surface(&m1, &m2, false, 1e-6, false).unwrap();

        // Should keep only faces on one side
        assert!(!result.simplices.is_empty());
        assert!(result.simplices.len() < m1.simplices.len() + 10); // bounded output
    }

    #[test]
    fn test_pierce_mesh_no_intersection() {
        let m1 = make_quad(0.0, 1.0);
        let m2 = make_quad(5.0, 1.0);
        let result = pierce_surface(&m1, &m2, false, 1e-6, false).unwrap();

        // No intersection: should return all faces
        assert_eq!(result.simplices.len(), m1.simplices.len());
    }

    #[test]
    fn test_line_simplification_collinear() {
        let points = vec![
            Vec3::from([0.0, 0.0, 0.0]),
            Vec3::from([1.0, 0.0, 0.0]),
            Vec3::from([2.0, 0.0, 0.0]),
        ];
        let edges = vec![UVec2::from([0, 1]), UVec2::from([1, 2])];
        let merges = line_simplification(&points, &edges, 1e-6);
        // Middle point should be merged
        assert!(merges.contains_key(&1));
    }

    #[test]
    fn test_line_simplification_non_collinear() {
        let points = vec![
            Vec3::from([0.0, 0.0, 0.0]),
            Vec3::from([1.0, 1.0, 0.0]),
            Vec3::from([2.0, 0.0, 0.0]),
        ];
        let edges = vec![UVec2::from([0, 1]), UVec2::from([1, 2])];
        let merges = line_simplification(&points, &edges, 1e-6);
        // No point should be merged
        assert!(merges.is_empty());
    }
}
