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
//! This module implements the blocking [DelayNs] trait from [embedded-hal].
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
//! # Ok(())
//! # }
//! ```
//! 
//! [DelayNs]: https://docs.rs/embedded-hal/1.0.0/embedded_hal/delay/trait.DelayNs.html
//! [embedded-hal]: https://docs.rs/embedded-hal/1.0.0/embedded_hal/delay/index.html
//! [now]: crate::time::now

use crate::time::{Duration, Instant};

/// Delay driver, using [`Instant`].
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
    pub fn delay(&self, delay: Duration) {
        let start = Instant::now();

        while start.elapsed() < delay {}
    }

    /// Delay for the specified number of milliseconds
    pub fn delay_millis(&self, ms: u32) {
        self.delay(Duration::from_millis(ms as u64));
    }

    /// Delay for the specified number of microseconds
    pub fn delay_micros(&self, us: u32) {
        self.delay(Duration::from_micros(us as u64));
    }

    /// Delay for the specified number of nanoseconds
    pub fn delay_nanos(&self, ns: u32) {
        self.delay(Duration::from_micros(ns.div_ceil(1000) as u64));
    }
}
