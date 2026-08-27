//! Helpers for calculating fractional clock divider values.
//!
//! Several peripherals (LCD_CAM, I2S, ...) divide their source clock by `N + B/A`, where `B/A`
//! is a proper fraction whose numerator and denominator are limited to a few bits.

// Which parts of this module are used depends on the set of drivers supported by the target chip.
#![allow(dead_code)]

/// The largest denominator [`FractionalDivider::new`] can be asked for, so that comparing the
/// error of two candidates stays within a `u64`. Hardware fields are far narrower than this.
const MAX_DENOMINATOR: u32 = 1 << 16;

#[derive(Clone, Copy)]
pub(crate) struct Fraction {
    pub numerator: u32,
    pub denominator: u32,
}

/// A candidate fraction, paired with how far it sits from the target.
///
/// The distance is `|target - fraction|` scaled by `target.denominator * fraction.denominator`,
/// which is why comparing two candidates has to undo the differing denominators.
#[derive(Clone, Copy)]
struct Candidate {
    fraction: Fraction,
    error: u32,
}

impl Candidate {
    /// Returns whichever of `self` and `other` is closer to the target, preferring `self` on a
    /// tie.
    fn closer_of(self, other: Candidate) -> Candidate {
        // This is cheaper than it looks, both architectures implement
        // widening multiplication cheaply.
        if self.error as u64 * other.fraction.denominator as u64
            <= other.error as u64 * self.fraction.denominator as u64
        {
            self
        } else {
            other
        }
    }
}

/// Returns the fraction closest to `target` for each denominator from 1 to `max_denominator`,
/// stopping early if one of them represents `target` exactly.
///
/// A denominator only admits one closest numerator, so the closest fraction overall is whichever
/// of these is closest.
///
/// `target` must be between 0 and 1.
fn candidates(target: Fraction, max_denominator: u32) -> impl Iterator<Item = Candidate> {
    let Fraction {
        numerator: n,
        denominator: d,
    } = target;

    // `acc` tracks `denominator * n % d` and `whole` tracks `denominator * n / d`, so that no
    // division is needed per candidate. Stepping `acc` by `n` without overflowing means
    // subtracting the complement whenever it would wrap past `d` instead of adding to it.
    let complement = d - n;
    let mut acc = 0_u32;
    let mut whole = 0_u32;

    let mut denominator = 0;
    let mut exact = false;

    core::iter::from_fn(move || {
        if exact || denominator == max_denominator {
            return None;
        }
        denominator += 1;

        if acc >= complement {
            acc -= complement;
            whole += 1;
        } else {
            acc += n;
        }

        // A zero remainder means this denominator represents the target exactly, so no larger
        // one can do better.
        exact = acc == 0;

        // Round to nearest: `acc` is how far the target is above `whole`, `d - acc` how far it
        // is below the next integer. Either way that distance is the candidate's error.
        let below = d - acc;
        let (numerator, error) = if acc <= below {
            (whole, acc)
        } else {
            (whole + 1, below)
        };

        Some(Candidate {
            fraction: Fraction {
                numerator,
                denominator,
            },
            error,
        })
    })
}

/// A clock divider of the form `N + B/A`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct FractionalDivider {
    /// The integral part of the divider, `N`.
    pub integer: u32,

    /// The numerator of the fractional part, `B`. Zero if the divider is integral.
    pub numerator: u32,

    /// The denominator of the fractional part, `A`. Zero if the divider is integral.
    pub denominator: u32,
}

impl FractionalDivider {
    /// Calculates the divider that takes `source` closest to `target`.
    ///
    /// `max_denominator` is the largest value the hardware's `A` field can hold. Callers are
    /// responsible for checking [`Self::integer`] against the range of the hardware's `N`
    /// field: the divider is rounded up to the next integer when the fractional part cannot
    /// be represented, so the result may be one larger than `source / target`.
    ///
    /// `target` must not be 0, and `max_denominator` must be between 1 and
    /// [`MAX_DENOMINATOR`].
    pub(crate) fn new(source: u32, target: u32, max_denominator: u32) -> Self {
        debug_assert!(target != 0);
        debug_assert!((1..=MAX_DENOMINATOR).contains(&max_denominator));

        let integer = source / target;

        let remainder = Fraction {
            numerator: source % target,
            denominator: target,
        };
        if remainder.numerator == 0 {
            return Self::integral(integer);
        }

        // Dropping the remainder entirely is always an option, and is the yardstick the
        // candidates are measured against.
        let dropped = Candidate {
            fraction: Fraction {
                numerator: 0,
                denominator: 1,
            },
            error: remainder.numerator,
        };
        let closest = candidates(remainder, max_denominator)
            .fold(dropped, Candidate::closer_of)
            .fraction;

        if closest.numerator == closest.denominator {
            // The remainder rounds up to a whole step.
            Self::integral(integer + 1)
        } else if closest.numerator == 0 {
            // The remainder is too small to represent, so it rounds away.
            Self::integral(integer)
        } else {
            Self {
                integer,
                numerator: closest.numerator,
                denominator: closest.denominator,
            }
        }
    }

    const fn integral(integer: u32) -> Self {
        Self {
            integer,
            numerator: 0,
            denominator: 0,
        }
    }

    /// Returns the frequency this divider produces from `source`.
    pub(crate) fn output_frequency(&self, source: u32) -> u32 {
        if self.numerator == 0 || self.denominator == 0 {
            return source / self.integer;
        }

        // OUTPUT = SOURCE / (N + B/A) = (SOURCE * A) / (N * A + B)
        //
        // u64 is required to fit the numbers from this arithmetic.
        let source = source as u64;
        let n = self.integer as u64;
        let b = self.numerator as u64;
        let a = self.denominator as u64;

        ((source * a) / (n * a + b)) as u32
    }
}
