//! # Control CPU Cores (ESP32-S3)
//!
//! ## Overview
//!
//! This module provides essential functionality for controlling
//! and managing the APP (second) CPU core on the `ESP32-S3` chip. It is used to
//! start and stop program execution on the APP core.

use core::sync::atomic::Ordering;

use crate::{
    peripherals::{LPWR, SYSTEM},
    system::{Cpu, multi_core::*},
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

/// Returns `true` if the specified core is currently running (not stalled).
#[instability::unstable]
pub fn is_running(core: Cpu) -> bool {
    if core == Cpu::AppCpu {
        // CORE_1_RUNSTALL in bit 0 -> needs to be 0 to not stall
        // CORE_1_CLKGATE_EN in bit 1 -> needs to be 1 to even be enabled
        let system = SYSTEM::regs();
        let r = system.core_1_control_0().read();
        if r.control_core_1_clkgate_en().bit_is_clear() || r.control_core_1_runstall().bit_is_set()
        {
            // If the core is not enabled we can take this shortcut
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

pub(crate) fn start_core1(entry_point: *const u32) {
    let system_control = SYSTEM::regs();

    crate::rom::ets_set_appcpu_boot_addr(entry_point as u32);

    system_control
        .core_1_control_0()
        .modify(|_, w| w.control_core_1_clkgate_en().set_bit());
    system_control
        .core_1_control_0()
        .modify(|_, w| w.control_core_1_runstall().clear_bit());
    system_control
        .core_1_control_0()
        .modify(|_, w| w.control_core_1_reseting().set_bit());
    system_control
        .core_1_control_0()
        .modify(|_, w| w.control_core_1_reseting().clear_bit());
}

pub(crate) fn start_core1_init<F>() -> !
where
    F: FnOnce(),
{
    // disables interrupts
    unsafe {
        xtensa_lx::interrupt::set_mask(0);
    }

    // reset cycle compare registers
    xtensa_lx::timer::set_ccompare0(0);
    xtensa_lx::timer::set_ccompare1(0);
    xtensa_lx::timer::set_ccompare2(0);

    unsafe extern "C" {
        static mut _init_start: u32;
    }

    // set vector table and stack pointer
    unsafe {
        xtensa_lx::set_vecbase(&raw const _init_start);
        xtensa_lx::set_stack_pointer(APP_CORE_STACK_TOP.load(Ordering::Acquire));

        #[cfg(all(feature = "rt", stack_guard_monitoring))]
        {
            let stack_guard = APP_CORE_STACK_GUARD.load(Ordering::Acquire);
            stack_guard.write_volatile(esp_config::esp_config_int!(
                u32,
                "ESP_HAL_CONFIG_STACK_GUARD_VALUE"
            ));
            // setting 0 effectively disables the functionality
            crate::debugger::set_stack_watchpoint(stack_guard as usize);
        }
    }

    // Trampoline to run from the new stack.
    // start_core1_run should _NEVER_ be inlined
    // as we rely on the function call to use
    // the new stack.
    unsafe { CpuControl::start_core1_run::<F>() }
}
