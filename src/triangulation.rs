/*!
    Triangulation algorithms for polygon outlines

    Port of madcad.triangulation focusing on triangulation_outline
*/

use crate::math::*;
use crate::mesh::{Surface, Wire};
use std::borrow::Cow;
use std::collections::HashSet;

/// Error type for triangulation failures
#[derive(Debug)]
pub struct TriangulationError {
    pub message: String,
    pub remaining_indices: Vec<Index>,
}

impl std::fmt::Display for TriangulationError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.message)
    }
}

impl std::error::Error for TriangulationError {}

/// Project an outline onto a 2D plane, returning 2D coordinates
///
/// Uses the given normal to determine the projection plane,
/// or computes one from the points if not provided.
fn planeproject(wire: &Wire<'_>, normal: Option<Vec3>) -> Result<Vec<Vec2>, &'static str> {
    let (x, y, _z) = guessbase(wire, normal)?;

    let proj: Vec<Vec2> = (0..wire.len())
        .map(|i| {
            let p = wire.point(i);
            Vec2::from([p.dot(x), p.dot(y)])
        })
        .collect();

    Ok(proj)
}

/// Build a base in which the points will be in plane XY
fn guessbase(wire: &Wire<'_>, normal: Option<Vec3>) -> Result<(Vec3, Vec3, Vec3), &'static str> {
    if let Some(n) = normal {
        return Ok(dirbase(n, None));
    }

    let thres = 10.0 * NUMPREC;

    if wire.len() < 2 {
        return Err("unable to extract 2 directions");
    }

    let o = wire.point(0);
    let ol = o[0].abs().max(o[1].abs()).max(o[2].abs());

    // Find first direction
    let mut x = Vec3::zero();
    let mut xl = 0.0;
    let mut i = 1;
    while xl <= thres && i < wire.len() {
        let p = wire.point(i);
        x = p - o;
        let pl = p[0].abs().max(p[1].abs()).max(p[2].abs());
        xl = x.square_length() / pl.max(ol);
        i += 1;
    }
    if xl <= thres {
        return Err("unable to extract 2 directions");
    }
    x = x.normalize();

    // Find second direction
    let mut y;
    let mut zl;
    while i < wire.len() {
        let p = wire.point(i);
        y = p - o;
        zl = x.cross(y).length();
        if zl > thres {
            y = noproject(y, x).normalize();
            return Ok((x, y, x.cross(y)));
        }
        i += 1;
    }

    Err("unable to extract 2 directions")
}

/// Priority criterion for 2D triangles, depending on shape quality
#[inline]
fn priority(u: Vec2, v: Vec2) -> Float {
    let uv = u.length() * v.length();
    if uv == 0.0 {
        0.0
    } else {
        (u.dot(v) + uv) / (uv * uv)
    }
}

/// Find index of maximum value in slice
fn imax(scores: &[Float]) -> usize {
    let mut best = 0;
    let mut best_score = Float::NEG_INFINITY;
    for (i, &s) in scores.iter().enumerate() {
        if s >= best_score {
            best_score = s;
            best = i;
        }
    }
    best
}

/// Triangulate a wire outline into a surface mesh
///
/// Returns a mesh with the triangles formed in the outline.
/// The returned mesh uses the same buffer of points as the input.
///
/// Complexity: O(n*k) where k is the number of non-convex points
pub fn triangulation_outline<'a>(
    wire: &Wire<'a>,
    normal: Option<Vec3>,
    prec: Option<Float>,
) -> Result<Surface<'a>, TriangulationError> {
    // Get a normal in the right direction for loop winding
    let normal = normal.unwrap_or_else(|| wire.normal());
    let base_prec = prec.unwrap_or_else(|| wire.precision(3));

    // The precision we use is a surface precision: (edge_length)^2 / NUMPREC
    let prec = base_prec * base_prec / NUMPREC;

    // Project all points in the plane
    let proj = match planeproject(wire, Some(normal)) {
        Ok(p) => p,
        Err(_) => {
            return Ok(Surface {
                points: wire.points.clone(),
                simplices: Cow::Owned(Vec::new()),
                tracks: Cow::Owned(Vec::new()),
            })
        }
    };

    // Reducing contour, indexing proj and wire indices
    let mut hole: Vec<usize> = (0..wire.len()).collect();

    // Remove last point if it's the same as first (closed loop)
    if hole.len() >= 2 {
        let last_idx = hole.len() - 1;
        let first = proj[hole[0]];
        let last = proj[hole[last_idx]];
        if (last - first).square_length() <= prec {
            hole.pop();
        }
    }

    if hole.len() < 3 {
        return Ok(Surface {
            points: wire.points.clone(),
            simplices: Cow::Owned(Vec::new()),
            tracks: Cow::Owned(Vec::new()),
        });
    }

    let l = wire.len();

    // Set of remaining non-convex points, indexing proj
    let mut nonconvex: HashSet<usize> = HashSet::new();
    for &i in &hole {
        let prev = if i == 0 { l - 1 } else { i - 1 };
        let next = (i + 1) % l;
        let pd = perpdot(proj[i] - proj[prev], proj[next] - proj[i]);
        if pd <= prec {
            nonconvex.insert(i);
        }
    }

    // Score function for a hole position
    let score = |hole: &[usize], nonconvex: &HashSet<usize>, i: usize| -> Float {
        let hl = hole.len();
        let o = proj[hole[i]];
        let u = proj[hole[(i + 1) % hl]] - o;
        let v = proj[hole[(i + hl - 1) % hl]] - o;
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
                        let a = proj[triangle[k]];
                        let b = proj[triangle[(k + 2) % 3]]; // k-1 mod 3
                        let mut s = perpdot(a - b, proj[j] - a);
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
        let i = imax(&scores);

        if scores[i] == Float::NEG_INFINITY {
            return Err(TriangulationError {
                message: "no more feasible triangles (algorithm failure or bad input outline)"
                    .to_string(),
                remaining_indices: hole.iter().map(|&h| wire.index(h)).collect(),
            });
        }

        // Add triangle
        let a = wire.index(hole[(i + hl - 1) % hl]);
        let b = wire.index(hole[i]);
        let c = wire.index(hole[(i + 1) % hl]);
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

    Ok(Surface {
        points: wire.points.clone(),
        tracks: Cow::Owned((0 .. simplices.len() as _).collect()),
        simplices: Cow::Owned(simplices),
    })
}



#[cfg(test)]
mod tests {
    use super::*;

    fn make_square_wire() -> Wire<'static> {
        // Simple square: (0,0) -> (1,0) -> (1,1) -> (0,1) -> (0,0)
        let points = vec![
            Vec3::from([0.0, 0.0, 0.0]),
            Vec3::from([1.0, 0.0, 0.0]),
            Vec3::from([1.0, 1.0, 0.0]),
            Vec3::from([0.0, 1.0, 0.0]),
        ];
        let indices: Vec<Vector<Index, 1>> = vec![
            Vector::from([0]),
            Vector::from([1]),
            Vector::from([2]),
            Vector::from([3]),
            Vector::from([0]), // closed
        ];
        Wire {
            points: Cow::Owned(points),
            simplices: Cow::Owned(indices),
            tracks: Cow::Owned(Vec::new()),
        }
    }

    #[test]
    fn test_triangulate_square() {
        let wire = make_square_wire();
        let result = triangulation_outline(&wire, None, None);

        assert!(result.is_ok());
        let mesh = result.unwrap();
        assert_eq!(mesh.simplices.len(), 2); // Square splits into 2 triangles
    }

    #[test]
    fn test_perpdot() {
        let a = Vec2::from([1.0, 0.0]);
        let b = Vec2::from([0.0, 1.0]);
        assert!((perpdot(a, b) - 1.0).abs() < NUMPREC);
    }
}
