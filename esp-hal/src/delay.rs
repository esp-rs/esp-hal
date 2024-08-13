//! # Delay
//!
//! ## Overview
//! The Delay driver provides blocking delay functionalities using the
//! `SYSTIMER` peripheral for RISC-V devices and the built-in Xtensa timer for
//! Xtensa devices.
//!
//! ## Configuration
//! The delays are implemented in a "best-effort" way, meaning that the CPU will
//! block for at least the amount of time specified, but accuracy can be
//! affected by many factors, including interrupt usage.
//!
//! ## Usage
//! This module implements the blocking [DelayMs] and [DelayUs] traits from
//! [embedded-hal], both 0.2.x and 1.x.x.
//!
//! ## Examples
//! ### Delay for 1 second
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! # use esp_hal::delay::Delay;
//! # use embedded_hal::delay::DelayNs;
//! let mut delay = Delay::new(&clocks);
//!
//! delay.delay_ms(1000 as u32);
//! # }
//! ```
//! 
//! [DelayMs]: embedded_hal_02::blocking::delay::DelayMs
//! [DelayUs]: embedded_hal_02::blocking::delay::DelayUs
//! [embedded-hal]: https://docs.rs/embedded-hal/1.0.0/embedded_hal/delay/index.html

use fugit::HertzU64;
pub use fugit::MicrosDurationU64;

/// Delay driver
///
/// Uses the `SYSTIMER` peripheral internally for RISC-V devices, and the
/// built-in Xtensa timer for Xtensa devices.
#[derive(Clone, Copy)]
pub struct Delay {
    freq: HertzU64,
}

impl Delay {
    /// Delay for the specified number of milliseconds
    pub fn delay_millis(&self, ms: u32) {
        for _ in 0..ms {
            self.delay_micros(1000);
        }
    }
}

#[cfg(feature = "embedded-hal-02")]
impl<T> embedded_hal_02::blocking::delay::DelayMs<T> for Delay
where
    T: Into<u32>,
{
    fn delay_ms(&mut self, ms: T) {
        for _ in 0..ms.into() {
            self.delay_micros(1000);
        }
    }
}

#[cfg(feature = "embedded-hal-02")]
impl<T> embedded_hal_02::blocking::delay::DelayUs<T> for Delay
where
    T: Into<u32>,
{
    fn delay_us(&mut self, us: T) {
        self.delay_micros(us.into());
    }
}

#[cfg(feature = "embedded-hal")]
impl embedded_hal::delay::DelayNs for Delay {
    fn delay_ns(&mut self, ns: u32) {
        self.delay_nanos(ns);
    }
}

#[cfg(riscv)]
mod implementation {
    use super::*;
    use crate::{clock::Clocks, timer::systimer::SystemTimer};

    impl Delay {
        /// Create a new `Delay` instance
        pub fn new(clocks: &Clocks<'_>) -> Self {
            // The counters and comparators are driven using `XTAL_CLK`.
            // The average clock frequency is fXTAL_CLK/2.5, which is 16 MHz.
            // The timer counting is incremented by 1/16 μs on each `CNT_CLK` cycle.
            Self {
                #[cfg(not(esp32h2))]
                freq: HertzU64::MHz(clocks.xtal_clock.to_MHz() as u64 * 10 / 25),
                #[cfg(esp32h2)]
                // esp32h2 TRM, section 11.2 Clock Source Selection
                freq: HertzU64::MHz(clocks.xtal_clock.to_MHz() as u64 * 10 / 20),
            }
        }

        /// Delay for the specified time
        pub fn delay(&self, time: MicrosDurationU64) {
            let t0 = SystemTimer::now();
            let rate: HertzU64 = MicrosDurationU64::from_ticks(1).into_rate();
            let clocks = time.ticks() * (self.freq / rate);

            while SystemTimer::now().wrapping_sub(t0) & SystemTimer::BIT_MASK <= clocks {}
        }

        /// Delay for the specified number of microseconds
        pub fn delay_micros(&self, us: u32) {
            let t0 = SystemTimer::now();
            let clocks = us as u64 * (self.freq / HertzU64::MHz(1));

            while SystemTimer::now().wrapping_sub(t0) & SystemTimer::BIT_MASK <= clocks {}
        }

        /// Delay for the specified number of nanoseconds
        pub fn delay_nanos(&self, ns: u32) {
            let t0 = SystemTimer::now();
            let clocks = ns as u64 * (self.freq / HertzU64::MHz(1)) / 1000;

            while SystemTimer::now().wrapping_sub(t0) & SystemTimer::BIT_MASK <= clocks {}
        }
    }
}

#[cfg(xtensa)]
mod implementation {
    use super::*;
    use crate::clock::Clocks;

    impl Delay {
        /// Create a new `Delay` instance
        pub fn new(clocks: &Clocks<'_>) -> Self {
            Self {
                freq: clocks.cpu_clock.into(),
            }
        }

        /// Delay for the specified time
        pub fn delay(&self, time: MicrosDurationU64) {
            let rate: HertzU64 = MicrosDurationU64::from_ticks(1).into_rate();
            let clocks = time.ticks() * (self.freq / rate);
            xtensa_lx::timer::delay(clocks as u32);
        }

        /// Delay for the specified number of microseconds
        pub fn delay_micros(&self, us: u32) {
            let clocks = us as u64 * (self.freq / HertzU64::MHz(1));
            xtensa_lx::timer::delay(clocks as u32);
        }

        /// Delay for the specified number of nanoseconds
        pub fn delay_nanos(&self, ns: u32) {
            let clocks = ns as u64 * (self.freq / HertzU64::MHz(1)) / 1000;
            xtensa_lx::timer::delay(clocks as u32);
        }
    }
}
