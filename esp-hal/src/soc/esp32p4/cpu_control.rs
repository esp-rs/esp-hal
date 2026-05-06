use crate::{
    peripherals::{HP_SYS_CLKRST, PMU},
    system::{Cpu, multi_core},
};

pub(crate) unsafe fn internal_park_core(core: Cpu, park: bool) {
    // 0x86 = stalled, 0xFF = running (matches IDF cpu_utility_ll.h)
    let code: u8 = if park { 0x86 } else { 0xFF };
    match core {
        Cpu::ProCpu => PMU::regs()
            .cpu_sw_stall()
            .modify(|_, w| unsafe { w.hpcore0_sw_stall_code().bits(code) }),
        Cpu::AppCpu => PMU::regs()
            .cpu_sw_stall()
            .modify(|_, w| unsafe { w.hpcore1_sw_stall_code().bits(code) }),
    };
}

#[instability::unstable]
pub fn is_running(core: Cpu) -> bool {
    let code = match core {
        Cpu::ProCpu => PMU::regs()
            .cpu_sw_stall()
            .read()
            .hpcore0_sw_stall_code()
            .bits(),
        Cpu::AppCpu => PMU::regs()
            .cpu_sw_stall()
            .read()
            .hpcore1_sw_stall_code()
            .bits(),
    };
    code != 0x86
}

pub(crate) fn start_core1(entry_point: *const u32) {
    // Enable Core 1's CPU clock if not already on.
    if !HP_SYS_CLKRST::regs()
        .soc_clk_ctrl0()
        .read()
        .core1_cpu_clk_en()
        .bit()
    {
        HP_SYS_CLKRST::regs()
            .soc_clk_ctrl0()
            .modify(|_, w| w.core1_cpu_clk_en().set_bit());
    }

    // Release Core 1 from global reset if it is currently held in reset.
    if HP_SYS_CLKRST::regs()
        .hp_rst_en0()
        .read()
        .rst_en_core1_global()
        .bit()
    {
        HP_SYS_CLKRST::regs()
            .hp_rst_en0()
            .modify(|_, w| w.rst_en_core1_global().clear_bit());
    }

    // Hand the entry point to the ROM, which Core 1 is polling.
    crate::rom::ets_set_appcpu_boot_addr(entry_point as u32);
}

/// Core 1 entry point set as the boot address.
///
/// The ROM jumps here directly, bypassing `_start`, so `gp` and the FPU
/// are not yet initialised. The naked prologue handles that before calling
/// regular Rust.
#[unsafe(naked)]
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

fn start_core1_init_impl<F>() -> !
where
    F: FnOnce(),
{
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
            crate::interrupt::init_vectoring();
        }
    }

    unsafe { multi_core::CpuControl::start_core1_run::<F>() }
}
