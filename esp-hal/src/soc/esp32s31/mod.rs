//! # SOC (System-on-Chip) module (ESP32-S31)
//!
//! ## Overview
//!
//! The `SOC` module provides access, functions and structures that are useful
//! for interacting with various system-related peripherals on `ESP32-S31` chip.

crate::unstable_module! {
    pub mod clocks;
}
pub(crate) mod cpu_control;
pub(crate) mod regi2c;

pub(crate) use esp32s31 as pac;

#[cfg(i2s_driver_supported)]
#[cfg_attr(not(feature = "unstable"), allow(unused))]
pub(crate) fn i2s_sclk_frequency() -> u32 {
    clocks::pll_f160m_frequency()
}

pub(crate) fn enable_branch_predictor() {
    // Enable branch predictor.
    // Note that the branch predictor will start cache requests and needs to be
    // disabled when the cache is disabled.
    // MHCR: CSR 0x7c1 — sourced from SOC_BRANCH_PREDICTOR_SUPPORTED in
    // esp-idf/components/soc/esp32s31/include/soc/soc_caps.h
    const MHCR_RS: u32 = 1 << 4; // R/W, address return stack set bit
    const MHCR_BFE: u32 = 1 << 5; // R/W, allow predictive jump set bit
    const MHCR_BTB: u32 = 1 << 12; // R/W, branch target prediction enable bit
    unsafe {
        core::arch::asm!("csrrs x0, 0x7c1, {0}", in(reg) MHCR_RS | MHCR_BFE | MHCR_BTB);
    }
}

#[cfg(feature = "rt")]
pub(crate) fn riscv_preinit() {}

pub(crate) fn pre_init() {
    if crate::system::Cpu::current() == crate::system::Cpu::ProCpu {
        // The ROM may enter on either core. Only the designated primary core
        // should park and gate its sibling during HAL startup.
        unsafe { cpu_control::internal_park_core(crate::system::Cpu::AppCpu, true) };
        cpu_control::disable_core1();
    }

    // Unlike the other supported chips, the S31 ROM leaves both SYSTIMER clock
    // gates disabled. `Peripheral::KEEP_ENABLED` only prevents the clock from
    // being disabled later, so perform the same enable/reset sequence as
    // ESP-IDF's S31 systimer LL before `Instant` starts using the counter.
    let systimer = crate::peripherals::HP_SYS_CLKRST::regs().systimer_ctrl0();
    systimer.modify(|_, w| w.systimer_apb_clk_en().set_bit());
    systimer.modify(|_, w| w.systimer_rst_en().set_bit());
    systimer.modify(|_, w| w.systimer_rst_en().clear_bit());
    systimer.modify(|_, w| w.systimer_clk_en().set_bit());
}
