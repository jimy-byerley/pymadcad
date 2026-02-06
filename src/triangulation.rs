/*!
    Triangulation algorithms for polygon outlines

    Port of madcad.triangulation focusing on triangulation_outline
*/

use crate::math::*;
use std::collections::HashSet;


/// Triangulate a wire outline into a surface mesh
///
/// Returns a mesh with the triangles formed in the outline.
/// Points are treated as a sequential loop (indices 0, 1, 2, ..., n-1).
///
/// Complexity: O(n*k) where k is the number of non-convex points
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
    let mut nonconvex: HashSet<usize> = HashSet::new();
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
