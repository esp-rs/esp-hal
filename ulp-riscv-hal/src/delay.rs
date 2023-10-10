//! Simple blocking delay functionality
//!
//! This uses cycle count under the hood.

use core::arch::asm;

use embedded_hal::blocking::delay::{DelayMs, DelayUs};

// see components\ulp\ulp_riscv\ulp_core\include\ulp_riscv_utils.h in esp-idf
#[cfg(feature = "esp32s2")]
const CYCLES_PER_US_M10: u32 = 85;
#[cfg(feature = "esp32s3")]
const CYCLES_PER_US_M10: u32 = 175;

pub struct Delay {}

impl Delay {
    pub fn new() -> Self {
        Self {}
    }
}

impl Default for Delay {
    fn default() -> Self {
        Self::new()
    }
}

impl DelayUs<u64> for Delay {
    #[inline(always)]
    fn delay_us(&mut self, us: u64) {
        DelayUs::<u32>::delay_us(self, us as u32);
    }
}

impl DelayUs<u32> for Delay {
    #[inline(always)]
    fn delay_us(&mut self, us: u32) {
        let t0 = cycles();
        let clock = us * CYCLES_PER_US_M10 / 10;
        while cycles().wrapping_sub(t0) <= clock {}
    }
}

impl DelayMs<u32> for Delay {
    #[inline(always)]
    fn delay_ms(&mut self, ms: u32) {
        DelayUs::<u32>::delay_us(self, ms * 1000);
    }
}

#[inline(always)]
fn cycles() -> u32 {
    let mut cycles;
    unsafe {
        asm!(
            "rdcycle {cycles}",
            cycles = out(reg) cycles,
        )
    }
    cycles
}
