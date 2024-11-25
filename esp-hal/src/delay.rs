//! # Delay
//!
//! ## Overview
//!
//! The Delay driver provides blocking delay functionalities using the
//! [now] function.
//!
//! ## Configuration
//!
//! The delays are implemented in a "best-effort" way, meaning that the CPU will
//! block for at least the amount of time specified, but accuracy can be
//! affected by many factors, including interrupt usage.
//!
//! ## Usage
//!
//! This module implements the blocking [DelayNs] trait [embedded-hal].
//!
//! ## Examples
//! ### Delay for 1 second
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! # use esp_hal::delay::Delay;
//! # use embedded_hal::delay::DelayNs;
//! let mut delay = Delay::new();
//!
//! delay.delay_ms(1000 as u32);
//! # }
//! ```
//! 
//! [DelayNs]: https://docs.rs/embedded-hal/1.0.0/embedded_hal/delay/trait.DelayNs.html
//! [embedded-hal]: https://docs.rs/embedded-hal/1.0.0/embedded_hal/delay/index.html
//! [now]: crate::time::now

pub use fugit::MicrosDurationU64;

/// Delay driver
///
/// Uses the `SYSTIMER` peripheral internally for RISC-V devices, and the
/// built-in Xtensa timer for Xtensa devices.
#[derive(Clone, Copy, Default)]
#[non_exhaustive]
pub struct Delay;

impl embedded_hal::delay::DelayNs for Delay {
    fn delay_ns(&mut self, ns: u32) {
        self.delay_nanos(ns);
    }
}

impl Delay {
    /// Creates a new `Delay` instance.
    pub const fn new() -> Self {
        Self {}
    }

    /// Delay for the specified time
    pub fn delay(&self, delay: MicrosDurationU64) {
        let start = crate::time::now();

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
    let now = crate::time::now();

    if start.ticks() <= now.ticks() {
        now - start
    } else {
        // now specifies at least 7 happy years, let's ignore this issue for
        // now.
        panic!("Time has wrapped around, which we currently don't handle");
    }
}
