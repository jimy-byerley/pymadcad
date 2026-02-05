/*!
    Spatial rasterization for hash-based proximity queries

    Port of rasterize_segment and rasterize_triangle from madcad/core.pyx
*/

use crate::math::*;

/// Python-like modulo for floats (always non-negative result)
fn pmod(l: Float, r: Float) -> Float {
    let m = l % r;
    if m < 0.0 { m + r } else { m }
}

/// Hashing key for a float coordinate
fn key(f: Float, cell: Float) -> i64 {
    (f / cell).floor() as i64
}

/// Infinity norm of a Vec3 (max absolute component)
fn norminf(v: Vec3) -> Float {
    v[0].abs().max(v[1].abs()).max(v[2].abs())
}

pub type HashKey = (i64, i64, i64);

/// Permutation of coordinates to align the dominant axis to Z
fn coordinate_order(n: Vec3) -> ([usize; 3], [usize; 3]) {
    if n[1] >= n[0] && n[1] >= n[2] {
        ([2, 0, 1], [1, 2, 0])
    } else if n[0] >= n[1] && n[0] >= n[2] {
        ([1, 2, 0], [2, 0, 1])
    } else {
        ([0, 1, 2], [0, 1, 2])
    }
}

/// Apply coordinate permutation to a Vec3
fn permute(v: Vec3, order: &[usize; 3]) -> Vec3 {
    Vec3::from([v[order[0]], v[order[1]], v[order[2]]])
}

/// Return a list of hashing keys for an edge (segment between two 3D points)
pub fn rasterize_segment(space: &[Vec3; 2], cell: Float) -> Result<Vec<HashKey>, &'static str> {
    if !(cell > 0.0) {
        return Err("cell must be strictly positive");
    }
    if !space[0].as_array().iter().all(|x| x.is_finite())
    || !space[1].as_array().iter().all(|x| x.is_finite()) {
        return Err("cannot rasterize non finite space");
    }

    let mut result = Vec::new();
    let prec = NUMPREC * norminf(space[0]).max(norminf(space[1]));

    // direction absolute values
    let n = Vec3::from([
        (space[1][0] - space[0][0]).abs(),
        (space[1][1] - space[0][1]).abs(),
        (space[1][2] - space[0][2]).abs(),
    ]);
    if n[0].max(n[1]).max(n[2]) < prec {
        return Ok(result);
    }

    // permutation of coordinates to get the direction closest to Z
    let (order, reorder) = coordinate_order(n);
    let s = [permute(space[0], &order), permute(space[1], &order)];

    // prepare variables
    let v = s[1] - s[0];
    let cell2 = cell / 2.0;
    let dy = v[1] / v[2];
    let dx = v[0] / v[2];
    let o = s[0];

    // z selection
    let (mut zmin, mut zmax) = (s[0][2], s[1][2]);
    if v[2] < 0.0 { std::mem::swap(&mut zmin, &mut zmax); }
    zmin -= prec;
    zmax += prec;
    zmin -= pmod(zmin, cell);
    let nz = ((zmax - zmin) / cell).ceil().max(1.0) as usize;

    for i in 0..nz {
        let z = zmin + cell * i as Float + cell2;

        // y selection
        let (mut ymin, mut ymax) = (
            o[1] + dx * (z - cell2 - o[2]),
            o[1] + dx * (z + cell2 - o[2]),
        );
        if dy < 0.0 { std::mem::swap(&mut ymin, &mut ymax); }
        ymin -= prec;
        ymax += prec;
        ymin -= pmod(ymin, cell);
        let ny = ((ymax - ymin) / cell).ceil().max(1.0) as usize;

        for j in 0..ny {
            let y = ymin + j as Float * cell + cell2;

            // x selection
            let (mut xmin, mut xmax) = (
                o[0] + dx * (z - cell2 - o[2]),
                o[0] + dx * (z + cell2 - o[2]),
            );
            if dx < 0.0 { std::mem::swap(&mut xmin, &mut xmax); }
            xmin -= prec;
            xmax += prec;
            xmin -= pmod(xmin, cell);
            let nx = ((xmax - xmin) / cell).ceil().max(1.0) as usize;

            for k in 0..nx {
                let x = xmin + k as Float * cell + cell2;

                let pk = [key(x, cell), key(y, cell), key(z, cell)];
                result.push((pk[reorder[0]], pk[reorder[1]], pk[reorder[2]]));
            }
        }
    }

    Ok(result)
}

/// Return a list of hashing keys for a triangle (three 3D points)
pub fn rasterize_triangle(space: &[Vec3; 3], cell: Float) -> Result<Vec<HashKey>, &'static str> {
    if !(cell > 0.0) {
        return Err("cell must be strictly positive");
    }
    if !space[0].as_array().iter().all(|x| x.is_finite())
    || !space[1].as_array().iter().all(|x| x.is_finite())
    || !space[2].as_array().iter().all(|x| x.is_finite()) {
        return Err("cannot rasterize non finite space");
    }

    let mut result = Vec::new();
    let prec = NUMPREC * norminf(space[0]).max(norminf(space[1])).max(norminf(space[2]));

    // permutation of coordinates to get the normal closest to Z
    let normal = {
        let e1 = space[1] - space[0];
        let e2 = space[2] - space[0];
        let c = e1.cross(e2);
        Vec3::from([c[0].abs(), c[1].abs(), c[2].abs()])
    };
    if normal[0].max(normal[1]).max(normal[2]) < prec {
        return Ok(result);
    }

    let (order, reorder) = coordinate_order(normal);
    let s = [
        permute(space[0], &order),
        permute(space[1], &order),
        permute(space[2], &order),
    ];

    // edge vectors: v[i] = s[i] - s[(i+1)%3]
    let v = [s[0] - s[1], s[1] - s[2], s[2] - s[0]];
    let n = v[0].cross(v[1]);
    let dx = -n[0] / n[2];
    let dy = -n[1] / n[2];
    let o = s[0];
    let cell2 = cell / 2.0;

    // bounding box
    let mut pmin = Vec3::from([
        s[0][0].min(s[1][0]).min(s[2][0]),
        s[0][1].min(s[1][1]).min(s[2][1]),
        s[0][2].min(s[1][2]).min(s[2][2]),
    ]);
    let mut pmax = Vec3::from([
        s[0][0].max(s[1][0]).max(s[2][0]),
        s[0][1].max(s[1][1]).max(s[2][1]),
        s[0][2].max(s[1][2]).max(s[2][2]),
    ]);
    let xmin_orig = pmin[0];
    let xmax_orig = pmax[0];
    // snap bounding box to grid
    for i in 0..3 {
        pmin[i] -= pmod(pmin[i], cell);
        pmax[i] += cell - pmod(pmax[i], cell);
    }

    // x selection
    let mut xmin = xmin_orig - prec;
    let xmax = xmax_orig + prec;
    xmin -= pmod(xmin, cell);
    let nx = ((xmax - xmin) / cell).ceil().max(1.0) as usize;

    for xi in 0..nx {
        let x = xmin + cell * xi as Float + cell2;

        // y selection - find y range from triangle edge intersections
        let mut candy = [0.0_f64; 6];
        let mut candylen = 0usize;

        for ei in 0..3usize {
            let next = (ei + 1) % 3;
            // check if edge crosses the x-band [x-cell2, x+cell2]
            if (s[next][0] - x + cell2) * (s[ei][0] - x - cell2) <= 0.0
            || (s[next][0] - x - cell2) * (s[ei][0] - x + cell2) <= 0.0
            {
                let d_ratio = v[ei][1] / if v[ei][0] != 0.0 { v[ei][0] } else { Float::INFINITY };
                candy[candylen]     = s[ei][1] + d_ratio * (x - cell2 - s[ei][0]);
                candy[candylen + 1] = s[ei][1] + d_ratio * (x + cell2 - s[ei][0]);
                candylen += 2;
            }
        }

        if candylen == 0 { continue; }

        let candy_min = candy[..candylen].iter().copied().fold(Float::INFINITY, Float::min);
        let candy_max = candy[..candylen].iter().copied().fold(Float::NEG_INFINITY, Float::max);
        let mut ymin = candy_min.max(pmin[1]) - prec;
        let ymax = candy_max.min(pmax[1]) + prec;
        ymin -= pmod(ymin, cell);
        if ymax <= ymin { continue; }
        let ny = ((ymax - ymin) / cell).ceil().max(1.0) as usize;

        for yj in 0..ny {
            let y = ymin + cell * yj as Float + cell2;

            // z selection from plane equation
            let candz = [
                o[2] + dx * (x - cell2 - o[0]) + dy * (y - cell2 - o[1]),
                o[2] + dx * (x + cell2 - o[0]) + dy * (y - cell2 - o[1]),
                o[2] + dx * (x - cell2 - o[0]) + dy * (y + cell2 - o[1]),
                o[2] + dx * (x + cell2 - o[0]) + dy * (y + cell2 - o[1]),
            ];
            let candz_min = candz.iter().copied().fold(Float::INFINITY, Float::min);
            let candz_max = candz.iter().copied().fold(Float::NEG_INFINITY, Float::max);
            let mut zmin = candz_min.max(pmin[2]) - prec;
            let zmax = candz_max.min(pmax[2]) + prec;
            zmin -= pmod(zmin, cell);
            if zmax <= zmin { continue; }
            let nz = ((zmax - zmin) / cell).ceil().max(1.0) as usize;

            for zk in 0..nz {
                let z = zmin + cell * zk as Float + cell2;

                // remove box corners that go out of the bounding area
                if pmin[0] < x && pmin[1] < y && pmin[2] < z
                && x < pmax[0] && y < pmax[1] && z < pmax[2]
                {
                    let pk = [key(x, cell), key(y, cell), key(z, cell)];
                    result.push((pk[reorder[0]], pk[reorder[1]], pk[reorder[2]]));
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
    fn test_pmod() {
        assert_eq!(pmod(5.0, 3.0), 2.0);
        assert_eq!(pmod(-1.0, 3.0), 2.0);
        assert_eq!(pmod(0.0, 3.0), 0.0);
    }

    #[test]
    fn test_key() {
        assert_eq!(key(0.5, 1.0), 0);
        assert_eq!(key(1.5, 1.0), 1);
        assert_eq!(key(-0.5, 1.0), -1);
    }

    #[test]
    fn test_rasterize_segment_basic() {
        let space = [
            Vec3::from([0.0, 0.0, 0.0]),
            Vec3::from([0.0, 0.0, 2.0]),
        ];
        let result = rasterize_segment(&space, 1.0).unwrap();
        assert!(!result.is_empty());
        // segment along Z from 0 to 2 with cell=1 should hit cells at z=0 and z=1
        assert!(result.contains(&(0, 0, 0)));
        assert!(result.contains(&(0, 0, 1)));
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
        // collinear points → zero-area triangle
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
