//! # Control CPU Cores (ESP32)
//!
//! ## Overview
//!
//! This module provides essential functionality for controlling
//! and managing the APP (second) CPU core on the `ESP32` chip. It is used to
//! start and stop program execution on the APP core.

use crate::{
    peripherals::{DPORT, LPWR, SPI0},
    system::Cpu,
};

pub(crate) unsafe fn internal_park_core(core: Cpu, park: bool) {
    let c1_value = if park { 0x21 } else { 0 };
    let c0_value = if park { 0x02 } else { 0 };
    match core {
        Cpu::ProCpu => {
            LPWR::regs()
                .sw_cpu_stall()
                .modify(|_, w| unsafe { w.sw_stall_procpu_c1().bits(c1_value) });
            LPWR::regs()
                .options0()
                .modify(|_, w| unsafe { w.sw_stall_procpu_c0().bits(c0_value) });
        }
        Cpu::AppCpu => {
            LPWR::regs()
                .sw_cpu_stall()
                .modify(|_, w| unsafe { w.sw_stall_appcpu_c1().bits(c1_value) });
            LPWR::regs()
                .options0()
                .modify(|_, w| unsafe { w.sw_stall_appcpu_c0().bits(c0_value) });
        }
    }
}

/// Returns whether the specified core is currently running (not stalled).
#[instability::unstable]
pub fn is_running(core: Cpu) -> bool {
    if core == Cpu::AppCpu {
        let dport = DPORT::regs();
        // DPORT_APPCPU_CLKGATE_EN in APPCPU_CTRL_B bit 0 -> needs to be 1 to even be enabled
        // DPORT_APPCPU_RUNSTALL in APPCPU_CTRL_C bit 0 -> needs to be 0 to not stall
        if dport
            .appcpu_ctrl_b()
            .read()
            .appcpu_clkgate_en()
            .bit_is_clear()
            || dport.appcpu_ctrl_c().read().appcpu_runstall().bit_is_set()
        {
            // If the core is not enabled or is stallled, we can take this shortcut
            return false;
        }
    }

    // sw_stall_appcpu_c1[5:0],  sw_stall_appcpu_c0[1:0]} == 0x86 will stall APP CPU
    // sw_stall_procpu_c1[5:0],  reg_sw_stall_procpu_c0[1:0]} == 0x86 will stall PRO CPU
    let is_stalled = match core {
        Cpu::ProCpu => {
            let c1 = LPWR::regs()
                .sw_cpu_stall()
                .read()
                .sw_stall_procpu_c1()
                .bits();
            let c0 = LPWR::regs().options0().read().sw_stall_procpu_c0().bits();
            (c1 << 2) | c0
        }
        Cpu::AppCpu => {
            let c1 = LPWR::regs()
                .sw_cpu_stall()
                .read()
                .sw_stall_appcpu_c1()
                .bits();
            let c0 = LPWR::regs().options0().read().sw_stall_appcpu_c0().bits();
            (c1 << 2) | c0
        }
    };

    is_stalled != 0x86
}

fn flush_cache(core: Cpu) {
    let dport_control = DPORT::regs();

    match core {
        Cpu::ProCpu => {
            dport_control
                .pro_cache_ctrl()
                .modify(|_, w| w.pro_cache_flush_ena().clear_bit());
            dport_control
                .pro_cache_ctrl()
                .modify(|_, w| w.pro_cache_flush_ena().set_bit());
            while dport_control
                .pro_cache_ctrl()
                .read()
                .pro_cache_flush_done()
                .bit_is_clear()
            {}

            dport_control
                .pro_cache_ctrl()
                .modify(|_, w| w.pro_cache_flush_ena().clear_bit());
        }
        Cpu::AppCpu => {
            dport_control
                .app_cache_ctrl()
                .modify(|_, w| w.app_cache_flush_ena().clear_bit());
            dport_control
                .app_cache_ctrl()
                .modify(|_, w| w.app_cache_flush_ena().set_bit());
            while dport_control
                .app_cache_ctrl()
                .read()
                .app_cache_flush_done()
                .bit_is_clear()
            {}
            dport_control
                .app_cache_ctrl()
                .modify(|_, w| w.app_cache_flush_ena().clear_bit());
        }
    };
}

fn enable_cache(core: Cpu) {
    let spi0 = SPI0::regs();
    let dport_control = DPORT::regs();

    match core {
        Cpu::ProCpu => {
            spi0.cache_fctrl().modify(|_, w| w.cache_req_en().set_bit());
            dport_control
                .pro_cache_ctrl()
                .modify(|_, w| w.pro_cache_enable().set_bit());
        }
        Cpu::AppCpu => {
            spi0.cache_fctrl().modify(|_, w| w.cache_req_en().set_bit());
            dport_control
                .app_cache_ctrl()
                .modify(|_, w| w.app_cache_enable().set_bit());
        }
    };
}

pub(crate) fn start_core1(entry_point: *const u32) {
    let dport_control = DPORT::regs();

    flush_cache(Cpu::AppCpu);
    enable_cache(Cpu::AppCpu);

    dport_control
        .appcpu_ctrl_d()
        .write(|w| unsafe { w.appcpu_boot_addr().bits(entry_point as u32) });

    dport_control
        .appcpu_ctrl_b()
        .modify(|_, w| w.appcpu_clkgate_en().set_bit());
    dport_control
        .appcpu_ctrl_c()
        .modify(|_, w| w.appcpu_runstall().clear_bit());
    dport_control
        .appcpu_ctrl_a()
        .modify(|_, w| w.appcpu_resetting().set_bit());
    dport_control
        .appcpu_ctrl_a()
        .modify(|_, w| w.appcpu_resetting().clear_bit());
}
