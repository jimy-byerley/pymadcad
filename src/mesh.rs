/*!
    Mesh data structures mirroring madcad.mesh in Python

    These are the Rust equivalents of:
    - Mesh (2D topology: triangulated surfaces with faces + points)
    - Web (1D topology: edge networks)
    - Wire (0D topology: point sequences, indices only)

    Like in Python, meshes reference buffers rather than owning them exclusively.
    Cow (Copy-on-Write) allows efficient borrowing with lazy cloning when mutation is needed.
*/

use crate::math::*;
use crate::aabox::*;
use crate::hashing::{edgekey, connef};
use std::borrow::Cow;
use rustc_hash::{FxHashMap, FxHashSet};


/// Generic mesh structure for D-dimensional space and S-simplex dimension
///
/// Uses Cow (Copy-on-Write) for buffers, matching Python's shared buffer design.
/// Meshes can borrow existing data or own it, with automatic cloning on mutation.
#[derive(Clone, Debug, Default)]
pub struct Mesh<'a, const D: usize, const S: usize> {
    /// buffer of points, not all necessarily used in the mesh
    pub points: Cow<'a, [Vector<Float, D>]>,
    /// simplices as tuples of indices in the buffer of points
    pub simplices: Cow<'a, [Vector<Index, S>]>,
    /// group associated to each simplex
    pub tracks: Cow<'a, [Index]>,
}

/// Surface mesh: 3D points with triangle faces
pub type Surface<'a> = Mesh<'a, 3, 3>;
/// Web mesh: 3D points with edges
pub type Web<'a> = Mesh<'a, 3, 2>;
/// Wire mesh: 3D points with point indices (curves/paths)
pub type Wire<'a> = Mesh<'a, 3, 1>;


/// GPU-ready display buffers for a surface mesh
pub struct DisplayBuffers {
    pub points: Vec<Vector<f32, 3>>,   // f32 positions
    pub normals: Vec<Vector<f32, 3>>,  // f32 normals
    pub faces: Vec<UVec3>,             // triangle indices
    pub edges: Vec<UVec2>,             // outline edge indices
    pub idents: Vec<Index>,            // group id per point
}

impl<'a, const D: usize, const S: usize> Mesh<'a, D, S> {
    /// Convert to an owned mesh with 'static lifetime (consumes and clones borrowed data)
    pub fn into_owned(self) -> Mesh<'static, D, S> {
        Mesh {
            points: Cow::Owned(self.points.into_owned()),
            simplices: Cow::Owned(self.simplices.into_owned()),
            tracks: Cow::Owned(self.tracks.into_owned()),
        }
    }

    /// Calculate the barycenter (centroid) of all points referenced by simplices
    pub fn barycenter(&self) -> Vector<Float, D> {
        let (sum, count) = self
            .simplices
            .iter()
            .flat_map(|s| (0..S).map(|j| self.points[s[j] as usize]))
            .fold((Vector::zero(), 0usize), |(acc, n), p| (acc + p, n + 1));

        if count == 0 {
            Vector::zero()
        } else {
            sum / (count as Float)
        }
    }

    /// Maximum numeric value of the mesh (max absolute coordinate)
    /// Use this to get a hint on mesh size or to evaluate numeric precision
    pub fn maxnum(&self) -> Float {
        self.points
            .iter()
            .flat_map(|p| (0..D).map(|i| p[i].abs()))
            .fold(0.0, Float::max)
    }

    /// Numeric coordinate precision allowed by floating point precision
    /// `propag` is the number of error propagation steps (default 3)
    pub fn precision(&self, propag: u32) -> Float {
        self.maxnum() * NUMPREC * (2.0_f64.powi(propag as i32))
    }
    
    pub fn bounds(&self) -> AABox<D> {
        AABox::from_iter(self.simplices
                .iter()
                .flat_map(|s|  s.iter())
                .cloned()
                .map(|i|  self.points[i as usize])
                )
    }

    /// Get the D-dimensional points of simplex `si`
    pub fn simplexpoints(&self, si: usize) -> [Vector<Float, D>; S] {
        self.simplices[si].as_array().map(|i|  self.points[i as usize])
    }
}

// Specific implementations for Surface
impl Surface<'_> {
    /// Get the unit normal of face `fi`
    pub fn facenormal(&self, fi: usize) -> Vec3 {
        let [a, b, c] = self.simplexpoints(fi);
        (b - a).cross(c - a).normalize()
    }

    /// Set of oriented edges delimiting the surface boundary
    pub fn outlines_oriented(&self) -> FxHashSet<[Index; 2]> {
        let mut edges: FxHashSet<[Index; 2]> = FxHashSet::default();
        for face in self.simplices.iter() {
            for &[a, b] in &[[face[0], face[1]], [face[1], face[2]], [face[2], face[0]]] {
                if edges.contains(&[a, b]) {
                    edges.remove(&[a, b]);
                } else {
                    edges.insert([b, a]);
                }
            }
        }
        edges
    }

    /// Vertex normals weighted by face angle at interior points,
    /// and by outline edge ownership at border points
    pub fn vertexnormals(&self) -> Vec<Vec3> {
        let npoints = self.points.len();
        let outline = self.outlines_oriented();
        let border: FxHashSet<Index> = outline.iter()
            .flat_map(|&[a, b]| [a, b])
            .collect();

        let mut normals = vec![Vec3::zero(); npoints];
        for (fi, face) in self.simplices.iter().enumerate() {
            let normal = self.facenormal(fi);
            if !is_finite_vec(normal) { continue; }

            for k in 0..3usize {
                let pi = face[k];
                let o = self.points[pi as usize];

                if !border.contains(&pi) {
                    // interior point: weight by angle at this vertex
                    let prev = self.points[face[(k + 1) % 3] as usize] - o;
                    let next = self.points[face[(k + 2) % 3] as usize] - o;
                    normals[pi as usize] = normals[pi as usize] + normal * anglebt(prev, next);
                } else if outline.contains(&[pi, face[(k + 2) % 3]]) {
                    // border point on outline edge: unweighted
                    normals[pi as usize] = normals[pi as usize] + normal;
                    let other = face[(k + 2) % 3] as usize;
                    normals[other] = normals[other] + normal;
                }
            }
        }

        for n in normals.iter_mut() {
            *n = n.normalize();
        }
        normals
    }

    /// Split the mesh around the given edges, returning a new surface.
    /// Points shared by two or more designated edges are duplicated,
    /// and face indices are reassigned so faces on each side of the split
    /// own separate copies of the point.
    pub fn split(&self, edges: &[[Index; 2]]) -> Surface<'static> {
        let faces: Vec<[Index; 3]> = self.simplices.iter()
            .map(|v| *v.as_array())
            .collect();
        let conn: FxHashMap<[Index; 2], usize> = connef(&faces);
        let split_edges: FxHashSet<[Index; 2]> = edges.iter()
            .map(|&e| edgekey(e[0], e[1]))
            .collect();

        let mut newfaces = faces.clone();
        let mut points: Vec<Vec3> = self.points.to_vec();

        for &edge in edges {
            let ekey = edgekey(edge[0], edge[1]);
            for &pivot in &[edge[0], edge[1]] {
                if !conn.contains_key(&ekey) {
                    continue;
                }
                // check that the pivot still appears in the face at this edge
                let fi = conn[&ekey];
                if !newfaces[fi].contains(&pivot) {
                    continue;
                }

                let dupli = points.len() as Index;
                points.push(points[pivot as usize]);

                // walk through faces around pivot, reassigning to the duplicate
                // front is an oriented edge matching conn keys (from connef)
                let mut front: [Index; 2] = ekey;
                loop {
                    if !conn.contains_key(&front) {
                        break;
                    }
                    let fi = conn[&front];
                    let f = simplex_phase(faces[fi], pivot);
                    let fm = simplex_phase(newfaces[fi], pivot);

                    assert_eq!(f[0], pivot);
                    if fm[0] != pivot {
                        break;
                    }

                    newfaces[fi] = [dupli, fm[1], fm[2]];

                    // advance to the next oriented edge around pivot
                    if pivot == front[0] {
                        front = [pivot, f[2]];
                    } else {
                        front = [f[1], pivot];
                    }

                    if split_edges.contains(&edgekey(front[0], front[1])) {
                        break;
                    }
                }
            }
        }

        Surface {
            points: Cow::Owned(points),
            simplices: Cow::Owned(newfaces.into_iter().map(Vector::from).collect()),
            tracks: Cow::Owned(self.tracks.to_vec()),
        }
    }

    /// Edges at frontiers between different groups (where adjacent faces have different tracks)
    pub fn frontiers(&self) -> Vec<[Index; 2]> {
        let faces: Vec<[Index; 3]> = self.simplices.iter()
            .map(|v| *v.as_array())
            .collect();
        let mut belong: FxHashMap<[Index; 2], Index> = FxHashMap::default();
        let mut frontier = Vec::new();
        for (i, face) in faces.iter().enumerate() {
            let track = self.tracks[i];
            for &[a, b] in &[[face[0], face[1]], [face[1], face[2]], [face[2], face[0]]] {
                let e = edgekey(a, b);
                if let Some(other_track) = belong.remove(&e) {
                    if other_track != track {
                        frontier.push(e);
                    }
                } else {
                    belong.insert(e, track);
                }
            }
        }
        frontier
    }

    /// Prepare display buffers: split at group frontiers and sharp edges,
    /// compute vertex normals, and convert to GPU-ready formats.
    pub fn display_buffers(&self, sharp_angle: Float) -> DisplayBuffers {
        let thresh = sharp_angle.cos();

        // 1. split at group frontiers
        let frontier_edges = self.frontiers();
        let m = self.split(&frontier_edges);

        // 2. collect outline edges (boundary)
        let outline: Vec<UVec2> = m.outlines_oriented().into_iter().map(UVec2::from).collect();

        // 3. find sharp edges to split
        let faces: Vec<[Index; 3]> = m.simplices.iter()
            .map(|v| *v.as_array())
            .collect();
        let conn = connef(&faces);
        let mut tosplit = Vec::new();
        for (&edge, &f1) in conn.iter() {
            if edge[1] > edge[0] { continue; }
            if let Some(&f2) = conn.get(&[edge[1], edge[0]]) {
                if m.tracks[f1] != m.tracks[f2]
                    || m.facenormal(f1).dot(m.facenormal(f2)) <= thresh
                {
                    tosplit.push(edge);
                }
            }
        }

        // 4. split at sharp edges
        let m = m.split(&tosplit);

        // 5. build idents: group id per point
        let mut idents = vec![0 as Index; m.points.len()];
        for (face, &track) in m.simplices.iter().zip(m.tracks.iter()) {
            for j in 0..3 {
                idents[face[j] as usize] = track;
            }
        }

        // 6. vertex normals
        let normals = m.vertexnormals();

        // 7. convert to f32
        let points_f32: Vec<Vector<f32, 3>> = m.points.iter()
            .map(|p| Vector::from([p[0] as f32, p[1] as f32, p[2] as f32]))
            .collect();
        let normals_f32: Vec<Vector<f32, 3>> = normals.iter()
            .map(|n| Vector::from([n[0] as f32, n[1] as f32, n[2] as f32]))
            .collect();
        let faces_out: Vec<UVec3> = m.simplices.into_owned();

        DisplayBuffers {
            points: points_f32,
            normals: normals_f32,
            faces: faces_out,
            edges: outline,
            idents,
        }
    }
}

// Specific implementations for Wire (used by triangulation_outline)
impl Wire<'_> {
    /// Get the ith point of the wire
    #[inline]
    pub fn point(&self, i: usize) -> Vec3 {
        self.points[self.simplices[i][0] as usize]
    }

    /// Get a point by its index in the points buffer
    #[inline]
    pub fn point_at(&self, idx: Index) -> Vec3 {
        self.points[idx as usize]
    }

    /// Get the index at position i
    #[inline]
    pub fn index(&self, i: usize) -> Index {
        self.simplices[i][0]
    }

    /// Number of indices in the wire
    #[inline]
    pub fn len(&self) -> usize {
        self.simplices.len()
    }

    /// Check if wire is empty
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.simplices.is_empty()
    }

    /// Check if the wire is closed (first == last index)
    pub fn is_closed(&self) -> bool {
        if self.simplices.len() < 2 {
            return false;
        }
        self.simplices[0][0] == self.simplices[self.simplices.len() - 1][0]
    }

    /// Calculate the normal of the wire (as if it were a flat surface outline)
    pub fn normal(&self) -> Vec3 {
        let c = self.barycenter();
        let mut area = Vec3::zero();
        for i in 1..self.len() {
            let prev = self.point(i - 1) - c;
            let curr = self.point(i) - c;
            area += prev.cross(curr);
        }
        area.normalize()
    }
}


/// Roll the simplex so that the given item is first
pub fn simplex_phase<T: Copy + PartialEq, const N: usize>(simplex: [T; N], first: T) -> [T; N] {
    for i in 0..N {
        if simplex[i] == first {
            return std::array::from_fn(|j| simplex[(j + i) % N]);
        }
    }
    panic!("requested first item not in simplex");
}

/// Return all possible rolls of the simplex
pub fn simplex_roll<T: Copy, const N: usize>(simplex: [T; N]) -> [[T; N]; N] {
    std::array::from_fn(|i| std::array::from_fn(|j| simplex[(i + j) % N]))
}

/// revert the simplex orientation (direction for edge, normal for triangle)
pub fn simplex_revert<T: Copy, const N: usize>(mut simplex: [T; N]) -> [T; N] {
    (simplex[1], simplex[0]) = (simplex[0], simplex[1]);
    simplex
}


#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_simplex_phase_face() {
        assert_eq!(simplex_phase([0, 1, 2], 0), [0, 1, 2]);
        assert_eq!(simplex_phase([0, 1, 2], 1), [1, 2, 0]);
        assert_eq!(simplex_phase([0, 1, 2], 2), [2, 0, 1]);
    }

    #[test]
    fn test_simplex_phase_edge() {
        assert_eq!(simplex_phase([0, 1], 0), [0, 1]);
        assert_eq!(simplex_phase([0, 1], 1), [1, 0]);
    }

    #[test]
    fn test_simplex_roll_face() {
        assert_eq!(
            simplex_roll([0, 1, 2]),
            [[0, 1, 2], [1, 2, 0], [2, 0, 1]],
        );
    }

    #[test]
    fn test_simplex_roll_edge() {
        assert_eq!(
            simplex_roll([0, 1]),
            [[0, 1], [1, 0]],
        );
    }
}
