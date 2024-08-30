//! # Delay
//!
//! ## Overview
//!
//! The Delay driver provides blocking delay functionalities using the
//! [current_time] function.
//!
//! ## Configuration
//!
//! The delays are implemented in a "best-effort" way, meaning that the CPU will
//! block for at least the amount of time specified, but accuracy can be
//! affected by many factors, including interrupt usage.
//!
//! ## Usage
//!
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
//! [current_time]: crate::time::current_time

pub use fugit::MicrosDurationU64;

use crate::clock::Clocks;

/// Delay driver
///
/// Uses the `SYSTIMER` peripheral internally for RISC-V devices, and the
/// built-in Xtensa timer for Xtensa devices.
#[derive(Clone, Copy)]
#[non_exhaustive]
pub struct Delay;

impl<T> embedded_hal_02::blocking::delay::DelayMs<T> for Delay
where
    T: Into<u32>,
{
    fn delay_ms(&mut self, ms: T) {
        self.delay_millis(ms.into());
    }
}

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

impl Delay {
    /// Creates a new `Delay` instance.
    // Do not remove the argument, it makes sure that the clocks are initialized.
    pub fn new(_clocks: &Clocks<'_>) -> Self {
        Self {}
    }

    /// Delay for the specified time
    pub fn delay(&self, delay: MicrosDurationU64) {
        let start = crate::time::current_time();

        while elapsed_since(start) < delay {}
    }

    /// Delay for the specified number of milliseconds
    pub fn delay_millis(&self, ms: u32) {
        for _ in 0..ms {
            self.delay_micros(1000);
        }
    }

    /// Delay for the specified number of microseconds
    pub fn delay_micros(&self, us: u32) {
        let delay = MicrosDurationU64::micros(us as u64);
        self.delay(delay);
    }

    /// Delay for the specified number of nanoseconds
    pub fn delay_nanos(&self, ns: u32) {
        let delay = MicrosDurationU64::nanos(ns as u64);
        self.delay(delay);
    }
}

fn elapsed_since(start: fugit::Instant<u64, 1, 1_000_000>) -> MicrosDurationU64 {
    let now = crate::time::current_time();

    if start.ticks() <= now.ticks() {
        now - start
    } else {
        // current_time specifies at least 7 happy years, let's ignore this issue for
        // now.
        panic!("Time has wrapped around, which we currently don't handle");
    }
}
