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

/// Returns the fraction closest to `target` for each denominator from 1 to `max_denominator`,
/// stopping early if one of them represents `target` exactly.
///
/// A denominator only admits one closest numerator, so the closest fraction overall is whichever
/// of these is closest.
///
/// `target` must be between 0 and 1.
fn candidates(target: Fraction, max_denominator: u32) -> impl Iterator<Item = Fraction> {
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
        // is below the next integer.
        let numerator = if acc <= d - acc { whole } else { whole + 1 };

        Some(Fraction {
            numerator,
            denominator,
        })
    })
}

/// Returns whichever of `a` and `b` is closer to `target`, preferring `a` on a tie.
fn closer_to(target: Fraction, a: Fraction, b: Fraction) -> Fraction {
    // |t_num/t_den - num/den| = |t_num * den - num * t_den| / (t_den * den). The common `t_den`
    // factor cancels, so the errors compare as `err_a * b_den` against `err_b * a_den`.
    let error = |f: Fraction| {
        (target.numerator as u64 * f.denominator as u64)
            .abs_diff(f.numerator as u64 * target.denominator as u64)
    };

    if error(a) * b.denominator as u64 <= error(b) * a.denominator as u64 {
        a
    } else {
        b
    }
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

        let zero = Fraction {
            numerator: 0,
            denominator: 1,
        };
        let closest = candidates(remainder, max_denominator).fold(zero, |best, candidate| {
            closer_to(remainder, best, candidate)
        });

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
