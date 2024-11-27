//! # Delay driver
//!
//! ## Overview
//!
//! The delay driver provides blocking delay functionality. The driver
//! implements the relevant traits from `embedded-hal`.
//!
//! ## Examples
//!
//! ```rust
//! esp_lp_hal::delay::Delay.delay_millis(500);
//! ```

/// Delay driver
#[derive(Debug, Clone, Copy)]
pub struct Delay;

impl Delay {
    /// Delay for at least the number of specific milliseconds.
    pub fn delay_millis(&self, mut ms: u32) {
        const MICROS_PER_MILLI: u32 = 1_000;
        const MAX_MILLIS: u32 = u32::MAX / MICROS_PER_MILLI;

        // Avoid potential overflow if milli -> micro conversion is too large
        while ms > MAX_MILLIS {
            ms -= MAX_MILLIS;
            self.delay_micros(MAX_MILLIS * MICROS_PER_MILLI);
        }

        self.delay_micros(ms * MICROS_PER_MILLI);
    }

    /// Delay for at least the number of specific microseconds.
    pub fn delay_micros(&self, mut us: u32) {
        const NANOS_PER_MICRO: u32 = 1_000;
        const MAX_MICROS: u32 = u32::MAX / NANOS_PER_MICRO;

        // Avoid potential overflow if micro -> nano conversion is too large
        while us > MAX_MICROS {
            us -= MAX_MICROS;
            self.delay_nanos(MAX_MICROS * NANOS_PER_MICRO);
        }

        self.delay_nanos(us * NANOS_PER_MICRO);
    }

    /// Delay for at least the number of specific nanoseconds.
    pub fn delay_nanos(&self, ns: u32) {
        let ticks_seconds = unsafe { crate::CPU_CLOCK };
        let clock = (ns as u64 * (ticks_seconds as u64)) / 1_000_000_000u64;
        let t0 = cycles();

        while cycles().wrapping_sub(t0) <= clock {}
    }
}

#[cfg(feature = "esp32c6")]
#[inline(always)]
fn cycles() -> u64 {
    riscv::register::mcycle::read64()
}

#[cfg(any(feature = "esp32s2", feature = "esp32s3"))]
#[inline(always)]
fn cycles() -> u64 {
    let mut cycles: u32;
    unsafe {
        core::arch::asm!(
            "rdcycle {cycles}",
            cycles = out(reg) cycles,
        )
    }

    cycles as u64
}

#[cfg(feature = "embedded-hal")]
impl embedded_hal::delay::DelayNs for Delay {
    #[inline(always)]
    fn delay_ns(&mut self, ns: u32) {
        self.delay_nanos(ns);
    }
}
