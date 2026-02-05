/*!
    Boolean operations on meshes

    Port of madcad.boolean - union, intersection, difference
*/

#[allow(unused_imports)]
use crate::math::*;
#[allow(unused_imports)]
use crate::mesh::*;

/// Compute the intersection/piercing of two surfaces
pub fn pierce<'a>(_a: &Surface<'a>, _b: &Surface<'a>) -> Surface<'a> {
    todo!()
}

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

    // unexpected case - should not be reachable
    debug_assert!(false, "unexpected case in intersect_triangles");
    None
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
}
