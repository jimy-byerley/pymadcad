/*!
    math definitions and functions mirroring madcad.mathutils in python

    only the functions needed for the rust implementations are here so this is an incomplete subset of madcad.mathutils
*/

pub use vecmat::prelude::*;
pub use vecmat::{Matrix, Vector};

pub type Index = u32;
pub type Float = f64;

/// numerical precision of floats used (float64 = 14 decimals)
pub const NUMPREC: Float = 1e-13;

pub type Vec2 = Vector<Float, 2>;
pub type Vec3 = Vector<Float, 3>;
pub type IVec2 = Vector<Index, 2>;
pub type IVec3 = Vector<Index, 3>;

/// Component of `vec` along `dir`, equivalent to `dot(vec,dir) / dot(dir,dir) * dir`
/// The result is not sensitive to the length of `dir`
pub fn project<const N: usize>(v: Vector<Float, N>, dir: Vector<Float, N>) -> Vector<Float, N> {
    let dd = dir.square_length();
    if dd == 0.0 {
        if v.square_length() != 0.0 {
            Vector::fill(Float::NAN)
        } else {
            Vector::zero()
        }
    } else {
        dir * (v.dot(dir) / dd)
    }
}

/// Components of `vec` not along `dir`, equivalent to `vec - project(vec,dir)`
/// The result is not sensitive to the length of `dir`
pub fn noproject<const N: usize>(v: Vector<Float, N>, dir: Vector<Float, N>) -> Vector<Float, N> {
    v - project(v, dir)
}

/// Return the vector in the given direction as if `vec` was its projection on it
/// equivalent to `dot(vec,vec) / dot(vec,dir) * dir`
/// The result is not sensitive to the length of `dir`
pub fn unproject<const N: usize>(v: Vector<Float, N>, dir: Vector<Float, N>) -> Vector<Float, N> {
    let vd = v.dot(dir);
    if vd == 0.0 {
        if v.square_length() != 0.0 {
            Vector::fill(Float::NAN)
        } else {
            Vector::zero()
        }
    } else {
        dir * (v.square_length() / vd)
    }
}

/// Perpendicular vector to the given 2D vector
pub fn perp(v: Vec2) -> Vec2 {
    Vec2::from([-v[1], v[0]])
}

/// Dot product of a with perpendicular vector to b, equivalent to `dot(a, perp(b))`
pub fn perpdot(a: Vec2, b: Vec2) -> Float {
    -a[1] * b[0] + a[0] * b[1]
}

/// Return a base using the given direction as z axis
pub fn dirbase(dir: Vec3, align: Option<Vec3>) -> (Vec3, Vec3, Vec3) {
    let mut align = align.unwrap_or(Vec3::from([1.0, 0.0, 0.0]));

    let mut x = noproject(align, dir);
    if x.square_length() <= NUMPREC * NUMPREC {
        align = Vec3::from([align[2], -align[0], align[1]]);
        x = noproject(align, dir);
    }
    if x.square_length() <= NUMPREC * NUMPREC {
        align = Vec3::from([align[1], -align[2], align[0]]);
        x = noproject(align, dir);
    }

    x = x.normalize();
    let y = dir.cross(x);
    (x, y, dir)
}
