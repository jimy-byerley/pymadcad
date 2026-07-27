/*!
    minimal reproducible pseudo random generator

    This exists instead of the `rand` crate because the algorithms that would be convenient
    there are explicitly not reproducible: both `rand::rngs::SmallRng` and `StdRng` opt out
    of value stability — "any future library version may replace the algorithm and results
    may be platform-dependent" — and `SmallRng` is not even the same algorithm on 32 and 64
    bit targets. Any algorithm we compute from random numbers would then change its result
    under a `cargo update` or between wheels.

    Keeping the few lines of arithmetic here pins the stream for good. The uses in this
    crate (breaking ties between degenerate geometry) need no statistical quality to speak
    of; qhull for instance joggles its input with a plain Lehmer generator.

    If a dependency is ever wanted anyway, `rand_pcg` and `rand_xoshiro` are the value
    stable ones. Whatever backs this, [`tests::test_stream_is_pinned`] guards the guarantee.
*/

use crate::math::Float;

/// odd constant to spread a seed into a distant part of the stream
pub const RESEED: u64 = 0x9E37_79B9_7F4A_7C15;

/// xorshift64* generator, reproducible across versions and platforms
pub struct Rand(u64);

impl Rand {
    pub fn new(seed: u64) -> Self {
        // any nonzero state works, xorshift is stuck on zero
        Self(seed | 1)
    }

    /// next raw value, uniform over the whole `u64` range
    pub fn next(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.0 = x;
        x.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }

    /// next value, uniform in `[0, 1)`
    pub fn unit(&mut self) -> Float {
        // 53 bits is the f64 mantissa width, so every value drawn is exactly representable
        (self.next() >> 11) as Float / (1u64 << 53) as Float
    }

    /// next value, uniform in `[-magnitude, magnitude)`
    pub fn uniform(&mut self, magnitude: Float) -> Float {
        (self.unit() * 2. - 1.) * magnitude
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /**
        reference vector, the way `rand_pcg` pins its own generators

        Every algorithm seeded from this generator is only reproducible as long as this
        stream does not move. An edit that changes these numbers silently changes the result
        of every part in every pymadcad script, so this test failing is a decision to make,
        not a number to update.
    */
    #[test]
    fn test_stream_is_pinned() {
        let mut rand = Rand::new(0x2545_F491_4F6C_DD1D);
        let values: Vec<Float> = (0..6).map(|_| rand.uniform(1.)).collect();
        assert_eq!(values, [
            0.3544223361175032,
            0.4910572010685965,
            -0.9209022310452766,
            0.17989972859330727,
            0.9840016181753954,
            -0.43657245670907185,
        ]);
    }

    #[test]
    fn test_ranges() {
        let mut rand = Rand::new(1);
        assert!((0..10_000)
            .map(|_| rand.unit())
            .all(|v| (0. ..1.).contains(&v)));
        assert!((0..10_000)
            .map(|_| rand.uniform(0.25))
            .all(|v| (-0.25..0.25).contains(&v)));
    }

    /// a stuck generator would silently make every joggle identical
    #[test]
    fn test_does_not_degenerate() {
        for seed in [0, 1, u64::MAX, 0x2545_F491_4F6C_DD1D] {
            let mut rand = Rand::new(seed);
            let values: Vec<u64> = (0..1_000).map(|_| rand.next()).collect();
            let distinct: std::collections::HashSet<u64> = values.iter().copied().collect();
            assert_eq!(distinct.len(), values.len(), "seed {seed} repeats values");
        }
    }
}
