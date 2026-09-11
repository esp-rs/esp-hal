#[cfg(feature = "unstable")]
use crate::system::multi_core;
use crate::{
    peripherals::{HP_SYS, LP_AON, PCR},
    system::Cpu,
};

pub(crate) unsafe fn internal_park_core(core: Cpu, park: bool) {
    // 0x86 = stalled, 0xFF = running (matches IDF cpu_utility_ll.h).
    let code: u8 = if park { 0x86 } else { 0xFF };
    LP_AON::regs().cpucore_cfg().modify(|_, w| unsafe {
        match core {
            Cpu::ProCpu => w.cpu_core0_sw_stall().bits(code),
            Cpu::AppCpu => w.cpu_core1_sw_stall().bits(code),
        }
    });

    // IDF waits for the run-stalled status to clear after unstalling.
    if !park {
        let status = HP_SYS::regs().core_debug_runstall_conf();
        match core {
            Cpu::ProCpu => while status.read().core0_runstalled().bit_is_set() {},
            Cpu::AppCpu => while status.read().core1_runstalled().bit_is_set() {},
        }
    }
}

#[instability::unstable]
pub fn is_running(core: Cpu) -> bool {
    let stall = LP_AON::regs().cpucore_cfg().read();
    let code = match core {
        Cpu::ProCpu => stall.cpu_core0_sw_stall().bits(),
        Cpu::AppCpu => stall.cpu_core1_sw_stall().bits(),
    };
    if code == 0x86 {
        return false;
    }

    match core {
        Cpu::ProCpu => true,
        Cpu::AppCpu => {
            let core1 = PCR::regs().core1_conf().read();
            core1.core1_clk_en().bit_is_set() && core1.core1_rst_en().bit_is_clear()
        }
    }
}

pub(crate) fn pre_system_reset() {
    // Match IDF's esp_restart_noos(): reset and stall only the other core.
    // The caller must remain alive to request the subsequent system reset.
    let other_core = match Cpu::current() {
        Cpu::ProCpu => Cpu::AppCpu,
        Cpu::AppCpu => Cpu::ProCpu,
    };
    LP_AON::regs()
        .cpucore_cfg()
        .modify(|_, w| match other_core {
            Cpu::ProCpu => w.cpu_core0_sw_reset().set_bit(),
            Cpu::AppCpu => w.cpu_core1_sw_reset().set_bit(),
        });
    unsafe { internal_park_core(other_core, true) };

    // Prevent ROM from jumping to an entry point from the previous image.
    crate::rom::ets_set_appcpu_boot_addr(0);
}

pub(crate) fn disable_core1() {
    // The reset defaults of the register, which is how ESP-IDF leaves Core 1 in single-core mode:
    // the core is clock gated and held in reset.
    PCR::regs()
        .core1_conf()
        .modify(|_, w| w.core1_clk_en().clear_bit().core1_rst_en().set_bit());
}

/// Releases Core 1's interrupt matrix from its power-on state, in which it is clock gated and held
/// in reset.
///
/// ESP-IDF only does this while starting Core 1, because it allocates interrupts for a core after
/// the core is running. esp-hal maps interrupts to Core 1 before that - `init` already binds the
/// default GPIO handler on both cores - and writes to the matrix are dropped while it is in reset,
/// so keep it running from the start.
pub(crate) fn enable_core1_interrupt_matrix() {
    PCR::regs().intmtx_conf().modify(|_, w| {
        w.intmtx_core1_clk_en()
            .set_bit()
            .intmtx_core1_rst_en()
            .clear_bit()
    });
}

#[cfg(feature = "unstable")]
pub(crate) fn start_core1(entry_point: *const u32) {
    // cpu_utility_ll_enable_clock_and_reset_app_cpu()
    PCR::regs()
        .core1_conf()
        .modify(|_, w| w.core1_clk_en().set_bit().core1_rst_en().clear_bit());

    // Core 1's ROM waits for this address before handing control to the app.
    crate::rom::ets_set_appcpu_boot_addr(entry_point as u32);
}

/// Core 1 entry point set as the boot address.
///
/// ROM jumps here directly, bypassing `_start`, so initialize `gp` and the FPU
/// before entering regular Rust.
#[unsafe(naked)]
#[cfg(feature = "unstable")]
pub(crate) extern "C" fn start_core1_init<F>() -> !
where
    F: FnOnce(),
{
    core::arch::naked_asm!(
        ".option push",
        ".option norelax",
        "la gp, __global_pointer$",
        "li ra, 0", // ensure probe-rs stops unwinding
        ".option pop",
        // Follow IDF's FPU initialization, enable it in Initial state, touch
        // fcsr (which makes the state Dirty), then clear the dirty bit to
        // leave mstatus.FS in Clean state (0b10).
        "li t0, 0x2000",
        "csrs mstatus, t0",
        "li t0, 1",
        "csrw fcsr, t0",
        "li t0, 0x2000",
        "csrc mstatus, t0",
        "la t0, {stack_top}",
        "lw sp, 0(t0)",
        "j {init}",
        stack_top = sym multi_core::APP_CORE_STACK_TOP,
        init = sym start_core1_init_impl::<F>,
    )
}

#[cfg(feature = "unstable")]
fn start_core1_init_impl<F>() -> !
where
    F: FnOnce(),
{
    crate::soc::enable_branch_predictor();
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
