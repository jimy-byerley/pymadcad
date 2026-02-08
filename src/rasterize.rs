/*!
    Spatial rasterization for hash-based proximity queries

    Port of the rasterization functions from madcad/hashing.py keysfor
*/

use crate::math::*;
use crate::hashing::edgekey;

use heapless::Vec as SVec;
// use multiversion::multiversion;

pub type IVec3 = Vector<i64, 3>;

/// Hashing key for a 3d position
fn key3(p: Vec3, cell: Float) -> IVec3 {
    (p / cell).map(|x| x.floor() as i64)
}

/// Snap a float down to grid boundary
fn snap_down(v: Float, cell: Float) -> Float {
    v - v.rem_euclid(cell)
}

/// Snap a Vec3 down to grid boundary (element-wise)
fn snap_down3(v: Vec3, cell: Float) -> Vec3 {
    v.map(|x| snap_down(x, cell))
}

/// Element-wise min of two Vec3
fn vmin(a: Vec3, b: Vec3) -> Vec3 {
    a.zip(b).map(|(a, b)| a.min(b))
}

/// Element-wise max of two Vec3
fn vmax(a: Vec3, b: Vec3) -> Vec3 {
    a.zip(b).map(|(a, b)| a.max(b))
}

/// Element-wise absolute value
fn vabs(v: Vec3) -> Vec3 {
    v.map(Float::abs)
}

/// Number of cells spanning [min, max], at least 1
fn ncells(min: Float, max: Float, cell: Float) -> usize {
    ((max - min) / cell).ceil().max(1.0) as usize
}

/// Permutation of coordinates to align the dominant axis with Z.
/// Returns (order, reorder) permutation indices.
fn coordinate_order(n: Vec3) -> ([usize; 3], [usize; 3]) {
    if n[1] >= n[0] && n[1] >= n[2] {
        ([2, 0, 1], [1, 2, 0])
    } else if n[0] >= n[1] && n[0] >= n[2] {
        ([1, 2, 0], [2, 0, 1])
    } else {
        ([0, 1, 2], [0, 1, 2])
    }
}

/// Apply index permutation to a vector's components
fn permute<T: Copy, const N: usize>(v: Vector<T, N>, order: &[usize; N]) -> Vector<T, N> {
    Vector::from(order.map(|i| v[i]))
}

/// Return a list of hashing keys for a segment between two 3D points.
///
/// The algorithm permutes coordinates so Z is the dominant direction,
/// then sweeps Z → Y → X computing the parametric line intersection
/// with each grid band.
// #[multiversion(targets = "simd")]
pub fn rasterize_segment(space: &[Vec3; 2], cell: Float) -> Result<Vec<IVec3>, &'static str> {
    if !(cell > 0.0) {
        return Err("cell must be strictly positive");
    }
    if !space.iter().all(|p| p.as_array().iter().all(|x| x.is_finite())) {
        return Err("cannot rasterize non finite space");
    }

    let mut result = Vec::new();
    let prec = NUMPREC * space.iter().map(|p| vabs(*p).max()).fold(0.0_f64, Float::max);

    // direction absolute values, degenerate if shorter than precision
    let n = vabs(space[1] - space[0]);
    if n.max() < prec {
        return Ok(result);
    }

    // permute coordinates so Z is the dominant direction
    let (order, reorder) = coordinate_order(n);
    let s = [permute(space[0], &order), permute(space[1], &order)];

    let v = s[1] - s[0];
    let cell2 = cell / 2.0;
    let o = s[0];
    // parametric line: at height z, x = fx(z), y = fy(z)
    let dx = v[0] / v[2];
    let dy = v[1] / v[2];
    let fx = |z: Float| o[0] + dx * (z - o[2]);
    let fy = |z: Float| o[1] + dy * (z - o[2]);

    // z sweep
    let (mut zmin, mut zmax) = if v[2] >= 0.0 { (s[0][2], s[1][2]) } else { (s[1][2], s[0][2]) };
    zmin -= prec;
    zmax += prec;
    zmin = snap_down(zmin, cell);

    for i in 0..ncells(zmin, zmax, cell) {
        let z = zmin + cell * i as Float + cell2;

        // y range at this z band
        let [mut ymin, mut ymax] = edgekey([fy(z - cell2), fy(z + cell2)]);
        ymin -= prec;
        ymax += prec;
        ymin = snap_down(ymin, cell);

        for j in 0..ncells(ymin, ymax, cell) {
            let y = ymin + j as Float * cell + cell2;

            // x range at this z band
            let [mut xmin, mut xmax] = edgekey([fx(z - cell2), fx(z + cell2)]);
            xmin -= prec;
            xmax += prec;
            xmin = snap_down(xmin, cell);

            for k in 0..ncells(xmin, xmax, cell) {
                let x = xmin + k as Float * cell + cell2;
                let pk = key3(Vec3::from([x, y, z]), cell);
                result.push(permute(pk, &reorder));
            }
        }
    }

    Ok(result)
}



/// Return a list of hashing keys for a triangle (three 3D points).
///
/// The algorithm permutes coordinates so Z aligns with the face normal,
/// then sweeps X → Y computing edge intersections, and Z from the plane equation.
// #[multiversion(targets = "simd")]
pub fn rasterize_triangle(space: &[Vec3; 3], cell: Float) -> Result<Vec<IVec3>, &'static str> {
    if !(cell > 0.0) {
        return Err("cell must be strictly positive");
    }
    if !space.iter().all(|p| p.as_array().iter().all(|x| x.is_finite())) {
        return Err("cannot rasterize non finite space");
    }

    let mut result = Vec::new();
    let prec = NUMPREC * space.iter().map(|p| vabs(*p).max()).fold(0.0_f64, Float::max);

    // permute coordinates so Z aligns with the dominant normal component
    let normal = (space[1] - space[0]).cross(space[2] - space[0]).abs();
    if normal.max() < prec {
        return Ok(result);
    }

    let (order, reorder) = coordinate_order(normal);
    let s: [Vec3; 3] = std::array::from_fn(|i| permute(space[i], &order));

    // edge vectors: v[i] goes from s[i] to s[i-1]  (i.e. s[i] - s[(i+1)%3])
    let v: [Vec3; 3] = std::array::from_fn(|i| s[i] - s[(i + 1) % 3]);
    let n = v[0].cross(v[1]);
    let o = s[0];
    let cell2 = cell / 2.0;
    // plane equation:  z = fz(x, y) = o.z + dx*(x-o.x) + dy*(y-o.y)
    let dx = -n[0] / n[2];
    let dy = -n[1] / n[2];
    let fz = |x: Float, y: Float| o[2] + dx * (x - o[0]) + dy * (y - o[1]);

    // bounding box, snapped to grid
    let pmin_raw = s.iter().copied().fold(s[0], |a, b| vmin(a, b));
    let pmax_raw = s.iter().copied().fold(s[0], |a, b| vmax(a, b));
    let pmin = snap_down3(pmin_raw, cell);
    let pmax = pmax_raw.map(|c| c + cell - c.rem_euclid(cell));

    // x sweep
    let xmin = snap_down(pmin_raw[0] - prec, cell);
    let xmax = pmax_raw[0] + prec;

    for xi in 0..ncells(xmin, xmax, cell) {
        let x = xmin + cell * xi as Float + cell2;

        // y range: find where triangle edges cross the x-band [x-cell2, x+cell2]
        let mut candy = SVec::<Float, 6>::new();
        for i in 0..3 {
            let next = (i + 1) % 3;
            if (s[next][0] - x + cell2) * (s[i][0] - x - cell2) <= 0.0
            || (s[next][0] - x - cell2) * (s[i][0] - x + cell2) <= 0.0
            {
                let slope = v[i][1] / if v[i][0] != 0.0 { v[i][0] } else { Float::INFINITY };
                let _ = candy.push(s[i][1] + slope * (x - cell2 - s[i][0]));
                let _ = candy.push(s[i][1] + slope * (x + cell2 - s[i][0]));
            }
        }
        if candy.is_empty() { continue; }

        let mut ymin = candy.iter().copied().fold(Float::INFINITY, Float::min).max(pmin[1]) - prec;
        let ymax = candy.iter().copied().fold(Float::NEG_INFINITY, Float::max).min(pmax[1]) + prec;
        ymin = snap_down(ymin, cell);
        if ymax <= ymin { continue; }

        for yj in 0..ncells(ymin, ymax, cell) {
            let y = ymin + cell * yj as Float + cell2;

            // z range from the 4 corners of the xy cell projected onto the plane
            let z_corners = [
                fz(x - cell2, y - cell2),
                fz(x + cell2, y - cell2),
                fz(x - cell2, y + cell2),
                fz(x + cell2, y + cell2),
            ];
            let mut zmin = z_corners.iter().copied().fold(Float::INFINITY, Float::min).max(pmin[2]) - prec;
            let zmax = z_corners.iter().copied().fold(Float::NEG_INFINITY, Float::max).min(pmax[2]) + prec;
            zmin = snap_down(zmin, cell);
            if zmax <= zmin { continue; }

            for zk in 0..ncells(zmin, zmax, cell) {
                let z = zmin + cell * zk as Float + cell2;
                let p = Vec3::from([x, y, z]);

                // remove corner cells that extend beyond the bounding area
                if (0..3).all(|i| pmin[i] < p[i] && p[i] < pmax[i]) {
                    result.push(permute(key3(p, cell), &reorder));
                }
            }
        }
    }

    Ok(result)
}


#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_snap_down() {
        assert_eq!(snap_down(5.3, 1.0), 5.0);
        assert_eq!(snap_down(-0.3, 1.0), -1.0);
        assert_eq!(snap_down(0.0, 1.0), 0.0);
    }

    #[test]
    fn test_key3() {
        let k = key3(Vec3::from([0.5, 1.5, -0.5]), 1.0);
        assert_eq!(*k.as_array(), [0, 1, -1]);
    }

    #[test]
    fn test_rasterize_segment_basic() {
        let space = [
            Vec3::from([0.0, 0.0, 0.0]),
            Vec3::from([0.0, 0.0, 2.0]),
        ];
        let result = rasterize_segment(&space, 1.0).unwrap();
        assert!(!result.is_empty());
        assert!(result.contains(&IVec3::from([0, 0, 0])));
        assert!(result.contains(&IVec3::from([0, 0, 1])));
    }

    #[test]
    fn test_rasterize_segment_degenerate() {
        let space = [
            Vec3::from([1.0, 1.0, 1.0]),
            Vec3::from([1.0, 1.0, 1.0]),
        ];
        let result = rasterize_segment(&space, 1.0).unwrap();
        assert!(result.is_empty());
    }

    #[test]
    fn test_rasterize_segment_invalid_cell() {
        let space = [Vec3::from([0.0, 0.0, 0.0]), Vec3::from([1.0, 0.0, 0.0])];
        assert!(rasterize_segment(&space, 0.0).is_err());
        assert!(rasterize_segment(&space, -1.0).is_err());
    }

    #[test]
    fn test_rasterize_triangle_basic() {
        let space = [
            Vec3::from([0.0, 0.0, 0.0]),
            Vec3::from([2.0, 0.0, 0.0]),
            Vec3::from([0.0, 2.0, 0.0]),
        ];
        let result = rasterize_triangle(&space, 1.0).unwrap();
        assert!(!result.is_empty());
    }

    #[test]
    fn test_rasterize_triangle_degenerate() {
        let space = [
            Vec3::from([0.0, 0.0, 0.0]),
            Vec3::from([1.0, 0.0, 0.0]),
            Vec3::from([2.0, 0.0, 0.0]),
        ];
        let result = rasterize_triangle(&space, 1.0).unwrap();
        assert!(result.is_empty());
    }

    #[test]
    fn test_rasterize_triangle_invalid_cell() {
        let space = [
            Vec3::from([0.0, 0.0, 0.0]),
            Vec3::from([1.0, 0.0, 0.0]),
            Vec3::from([0.0, 1.0, 0.0]),
        ];
        assert!(rasterize_triangle(&space, 0.0).is_err());
    }
}
