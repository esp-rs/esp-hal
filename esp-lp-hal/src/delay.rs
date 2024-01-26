//! Simple blocking delay functionality.

#[derive(Clone, Copy)]
pub struct Delay;

impl Delay {
    /// Delay for at least the number of specific microseconds.
    pub fn delay_micros(&mut self, mut us: u32) {
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
    pub fn delay_nanos(&mut self, ns: u32) {
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

#[cfg(feature = "embedded-hal-02")]
impl embedded_hal_02::blocking::delay::DelayUs<u64> for Delay {
    #[inline(always)]
    fn delay_us(&mut self, us: u64) {
        self.delay_micros(us as u32);
    }
}

#[cfg(feature = "embedded-hal-02")]
impl embedded_hal_02::blocking::delay::DelayUs<u32> for Delay {
    #[inline(always)]
    fn delay_us(&mut self, us: u32) {
        self.delay_micros(us);
    }
}

#[cfg(feature = "embedded-hal-02")]
impl embedded_hal_02::blocking::delay::DelayMs<u32> for Delay {
    #[inline(always)]
    fn delay_ms(&mut self, ms: u32) {
        self.delay_micros(ms * 1000);
    }
}

#[cfg(feature = "embedded-hal-1")]
impl embedded_hal_1::delay::DelayNs for Delay {
    fn delay_ns(&mut self, ns: u32) {
        self.delay_nanos(ns);
    }
}
