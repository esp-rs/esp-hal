//! # SOC (System-on-Chip) module (ESP32-H4)
//!
//! ## Overview
//!
//! The `SOC` module provides access, functions and structures that are useful
//! for interacting with various system-related peripherals on `ESP32-H4` chip.

crate::unstable_module! {
    pub mod clocks;
}

pub(crate) mod cpu_control;
pub(crate) mod regi2c;

pub(crate) use esp32h4 as pac;

#[cfg(feature = "rt")]
pub(crate) fn riscv_preinit() {}

pub(crate) fn pre_init() {
    // Core 1 is gated and held in reset after a chip reset, but a software reset of Core 0 leaves
    // whatever start_core1() set up. Undo it so that Core 1 cannot run during startup.
    unsafe { cpu_control::internal_park_core(crate::system::Cpu::AppCpu, true) };
    cpu_control::disable_core1();
    cpu_control::enable_core1_interrupt_matrix();

    // By default, the APM access-path filters only allow masters that are in TEE mode. Every
    // master except the HP CPU boots in REE mode, so the GDMA can neither fetch descriptors nor
    // access memory until the filters are disabled. Reads return zeroes and writes are dropped,
    // without a bus error. Mirrors ESP-IDF's `bootloader_init_mem`.
    crate::peripherals::HP_APM::regs()
        .func_ctrl()
        .write(|w| unsafe { w.bits(0) });

    // The RNG's clocks and its sampling logic are off after reset, and the driver has no way to
    // request them - `Peripheral::KEEP_ENABLED` only prevents the clock from being disabled later.
    // Run the same sequence as ESP-IDF's `rng_ll_enable` so that reads return random data.
    crate::peripherals::LP_PERI::regs().clk_en().modify(|_, w| {
        w.rng_apb_ck_en().set_bit();
        w.rng_ck_en().set_bit()
    });
    crate::peripherals::RNG::regs().cfg().modify(|_, w| unsafe {
        w.sample_enable().set_bit();
        w.timer_en().set_bit();
        w.rtc_timer_en().bits(3)
    });
}

pub(crate) fn enable_branch_predictor() {
    // Enable branch predictor
    // Note that the branch predictor will start cache requests and needs to be disabled when
    // the cache is disabled.
    // MHCR: CSR 0x7c1
    const MHCR_RS: u32 = 1 << 4; // R/W, address return stack set bit
    const MHCR_BFE: u32 = 1 << 5; // R/W, allow predictive jump set bit
    const MHCR_BTB: u32 = 1 << 12; // R/W, branch target prediction enable bit
    unsafe {
        core::arch::asm!("csrrs x0, 0x7c1, {0}", in(reg) MHCR_RS | MHCR_BFE | MHCR_BTB);
    }
}

/// Writes back a specific range of data in the cache.
#[doc(hidden)]
#[crate::ram]
#[expect(dead_code, reason = "No enabled driver needs cache maintenance yet.")]
pub unsafe fn cache_writeback_addr(addr: u32, size: u32) {
    unsafe extern "C" {
        fn Cache_WriteBack_Addr(addr: u32, size: u32);
    }

    unsafe {
        Cache_WriteBack_Addr(addr, size);
    }
}

/// Invalidate a specific range of addresses in the cache.
#[doc(hidden)]
#[crate::ram]
#[expect(dead_code, reason = "No enabled driver needs cache maintenance yet.")]
pub unsafe fn cache_invalidate_addr(addr: u32, size: u32) {
    const CACHE_MAP_L1_DCACHE: u32 = 1 << 4;

    unsafe extern "C" {
        fn Cache_Invalidate_Addr(map: u32, addr: u32, size: u32);
    }
    unsafe {
        Cache_Invalidate_Addr(CACHE_MAP_L1_DCACHE, addr, size);
    }
}
