#[cfg(feature = "unstable")]
use crate::system::multi_core;
use crate::{
    peripherals::{HP_SYS, HP_SYS_CLKRST, LP_AON_CLKRST, PMU},
    system::Cpu,
};

pub(crate) unsafe fn internal_park_core(core: Cpu, park: bool) {
    // 0x86 = stalled, 0xFF = running (matches IDF cpu_utility_ll.h)
    let code: u8 = if park { 0x86 } else { 0xFF };
    PMU::regs().cpu_sw_stall().modify(|_, w| unsafe {
        match core {
            Cpu::ProCpu => w.hpcore0_sw_stall_code().bits(code),
            Cpu::AppCpu => w.hpcore1_sw_stall_code().bits(code),
        }
    });

    // On unstall, wait for the corestalled status bit to drop, matching IDF's
    // cpu_utility_ll_unstall_cpu. We do *not* wait on stall: if the target core
    // was in WFI when the stall took effect, the status bit may never assert,
    // making the check unreliable.
    if !park {
        let st = HP_SYS::regs().cpu_corestalled_st();
        match core {
            Cpu::ProCpu => while st.read().reg_core0_corestalled_st().bit_is_set() {},
            Cpu::AppCpu => while st.read().reg_core1_corestalled_st().bit_is_set() {},
        }
    }
}

#[instability::unstable]
pub fn is_running(core: Cpu) -> bool {
    let stall_reg = PMU::regs().cpu_sw_stall().read();
    let code = match core {
        Cpu::ProCpu => stall_reg.hpcore0_sw_stall_code().bits(),
        Cpu::AppCpu => stall_reg.hpcore1_sw_stall_code().bits(),
    };
    code != 0x86
}

/// Prepare Core 1 for an imminent system reset.
///
/// Mirrors IDF's `esp_restart_noos()` pre-reset sequence for ESP32-P4:
/// 1. Briefly reset Core 1's CPU (LP AON domain, so it takes effect even with global reset still
///    cleared from a previous `start_core1` call).
/// 2. Stall Core 1 via PMU (LP AON domain — this value **persists** through the system software
///    reset, keeping Core 1 stalled during the ROM bootloader phase and preventing it from
///    interfering with USB Serial/JTAG).
/// 3. Clear the AppCpu boot address so that, even if Core 1 is somehow allowed to run during the
///    next ROM bootloader phase, it will spin in the ROM polling loop instead of jumping to a stale
///    `start_core1_init` from this image.
///
/// Must be called before any `software_reset()` when Core 1 is running.
pub(crate) fn pre_system_reset() {
    // Assert a brief CPU reset pulse for Core 1 (LP AON → persists, self-clears).
    LP_AON_CLKRST::regs()
        .lp_aonclkrst_hpcpu_reset_ctrl0()
        .modify(|_, w| w.lp_aonclkrst_hpcore1_sw_reset().set_bit());

    // Stall Core 1 via PMU (LP AON domain). The stall code 0x86 persists through
    // the subsequent software reset and keeps Core 1 stalled during ROM boot.
    unsafe { internal_park_core(Cpu::AppCpu, true) };

    // Clear the AppCpu boot address so the ROM won't jump back into a stale `start_core1_init`
    // after the reset. Mirrors IDF's `esp_restart_noos_inner`.
    crate::rom::ets_set_appcpu_boot_addr(0);
}

/// Disable Core 1's CPU clock and hold it in global reset.
///
/// Called from `pre_init` on every boot to undo any state left by a previous
/// run that survived a software reset (HP_SYS_CLKRST registers are not
/// cleared by software-triggered resets, only by a power-on or EN-pin reset).
/// Mirrors IDF's single-core-mode disable sequence for ESP32-P4.
pub(crate) fn disable_core1() {
    HP_SYS_CLKRST::regs()
        .soc_clk_ctrl0()
        .modify(|_, w| w.core1_cpu_clk_en().clear_bit());
    HP_SYS_CLKRST::regs()
        .hp_rst_en0()
        .modify(|_, w| w.rst_en_core1_global().set_bit());
}

#[cfg(feature = "unstable")]
pub(crate) fn start_core1(entry_point: *const u32) {
    // Enable Core 1's CPU clock.
    HP_SYS_CLKRST::regs()
        .soc_clk_ctrl0()
        .modify(|_, w| w.core1_cpu_clk_en().set_bit());

    // Release Core 1 from global reset.
    HP_SYS_CLKRST::regs()
        .hp_rst_en0()
        .modify(|_, w| w.rst_en_core1_global().clear_bit());

    // Hand the entry point to the ROM, which Core 1 is polling.
    crate::rom::ets_set_appcpu_boot_addr(entry_point as u32);
}

/// Core 1 entry point set as the boot address.
///
/// The ROM jumps here directly, bypassing `_start`, so `gp` and the FPU
/// are not yet initialised. The naked prologue handles that before calling
/// regular Rust.
#[unsafe(naked)]
#[cfg(feature = "unstable")]
pub(crate) extern "C" fn start_core1_init<F>() -> !
where
    F: FnOnce(),
{
    core::arch::naked_asm!(
        // Set up the global pointer so GP-relative symbol accesses work.
        ".option push",
        ".option norelax",
        "la gp, __global_pointer$",
        ".option pop",
        // Initialize the FPU (riscv32imafc has F).
        "li t0, 0x6000",
        "csrrs x0, mstatus, t0",
        "fscsr x0",
        // Switch to Core 1's stack (stored by Core 0 before releasing this core).
        "la t0, {stack_top}",
        "lw sp, 0(t0)",
        // Tail-call into regular Rust.
        "j {init}",
        stack_top = sym multi_core::APP_CORE_STACK_TOP,
        init      = sym start_core1_init_impl::<F>,
    )
}

#[cfg(feature = "unstable")]
fn start_core1_init_impl<F>() -> !
where
    F: FnOnce(),
{
    crate::soc::enable_branch_predictor();

    // The ROM has already handed off to us; clear the AppCpu boot address so a
    // subsequent software reset doesn't re-enter this stale entry point.
    // Matches IDF's `call_start_cpu1`.
    crate::rom::ets_set_appcpu_boot_addr(0);

    unsafe {
        #[cfg(all(feature = "rt", stack_guard_monitoring))]
        {
            let guard =
                multi_core::APP_CORE_STACK_GUARD.load(core::sync::atomic::Ordering::Acquire);
            guard.write_volatile(esp_config::esp_config_int!(
                u32,
                "ESP_HAL_CONFIG_STACK_GUARD_VALUE"
            ));
            crate::debugger::set_stack_watchpoint(guard as usize);
        }
        crate::interrupt::init_vectoring();
    }

    unsafe { multi_core::CpuControl::start_core1_run::<F>() }
}
