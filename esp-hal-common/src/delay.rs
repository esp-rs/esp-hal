//! # Delay driver
//!
//! ## Overview
//! The Delay driver provides blocking delay functionalities using the
//! `SYSTIMER` peripheral for RISC-V devices and the built-in Xtensa timer for
//! Xtensa devices. This module implements the blocking [DelayMs] and [DelayUs]
//! traits from [embedded-hal].
//!
//! The delays are implemented in a "best-effort" way, meaning that the CPU will
//! block for at least the amount of time specified, but accuracy can be
//! affected by many factors, including interrupt usage.
//!
//! ## Example
//! ```no_run
//! let mut clocks = ClockControl::boot_defaults(system.clock_control).freeze();
//! let mut delay = Delay::new(&clocks);
//!
//! delay.delay_ms(1000 as u32);
//! ```
//!
//! [DelayMs]: embedded_hal::blocking::delay::DelayMs
//! [DelayUs]: embedded_hal::blocking::delay::DelayUs
//! [embedded-hal]: https://docs.rs/embedded-hal/latest/embedded_hal/

use fugit::HertzU64;

/// Delay driver
///
/// Uses the `SYSTIMER` peripheral internally for RISC-V devices, and the
/// built-in Xtensa timer for Xtensa devices.
#[derive(Clone, Copy)]
pub struct Delay {
    freq: HertzU64,
}

impl<T> embedded_hal::blocking::delay::DelayMs<T> for Delay
where
    T: Into<u32>,
{
    fn delay_ms(&mut self, ms: T) {
        for _ in 0..ms.into() {
            self.delay(1000u32);
        }
    }
}

impl<T> embedded_hal::blocking::delay::DelayUs<T> for Delay
where
    T: Into<u32>,
{
    fn delay_us(&mut self, us: T) {
        self.delay(us.into());
    }
}

#[cfg(feature = "eh1")]
impl embedded_hal_1::delay::DelayUs for Delay {
    fn delay_us(&mut self, us: u32) {
        self.delay(us);
    }
}

#[cfg(riscv)]
mod delay {
    use super::*;
    use crate::{clock::Clocks, systimer::SystemTimer};

    impl Delay {
        /// Create a new `Delay` instance
        pub fn new(clocks: &Clocks) -> Self {
            // The counters and comparators are driven using `XTAL_CLK`.
            // The average clock frequency is fXTAL_CLK/2.5, which is 16 MHz.
            // The timer counting is incremented by 1/16 μs on each `CNT_CLK` cycle.
            Self {
                freq: HertzU64::MHz(clocks.xtal_clock.to_MHz() as u64 * 10 / 25),
            }
        }

        /// Delay for the specified number of microseconds
        pub fn delay(&self, us: u32) {
            let t0 = SystemTimer::now();
            let clocks = us as u64 * (self.freq / HertzU64::MHz(1));

            while SystemTimer::now().wrapping_sub(t0) & SystemTimer::BIT_MASK <= clocks {}
        }
    }
}

#[cfg(xtensa)]
mod delay {
    use super::*;
    use crate::clock::Clocks;

    impl Delay {
        /// Create a new `Delay` instance
        pub fn new(clocks: &Clocks) -> Self {
            Self {
                freq: clocks.cpu_clock.into(),
            }
        }

        /// Delay for the specified number of microseconds
        pub fn delay(&self, us: u32) {
            let clocks = us as u64 * (self.freq / HertzU64::MHz(1));
            xtensa_lx::timer::delay(clocks as u32);
        }
    }
}
