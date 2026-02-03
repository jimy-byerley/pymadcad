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
use std::borrow::Cow;

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
