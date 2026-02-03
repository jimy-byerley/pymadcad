use crate::math::*;

pub struct AABox<const D: usize> {
    pub min: Vector<Float, D>,
    pub max: Vector<Float, D>,
}
impl<const D: usize> AABox<D> {
    pub fn empty() -> Self {
        Self {
            min: Vector::fill(Float::INFINITY),
            max: Vector::fill(-Float::INFINITY),
        }
    }
    pub fn from_iter(iter: impl IntoIterator<Item=Vector<Float, D>>) -> Self {
        iter.into_iter().fold(Self::empty(), |b, p| b.append(p)) 
    }
    pub fn append(&self, other: Vector<Float, D>) -> Self {
        Self {
            min: self.min.zip(other).map(|(a, b)|  a.min(b)),
            max: self.max.zip(other).map(|(a, b)|  a.max(b)),
        }
    }
}
