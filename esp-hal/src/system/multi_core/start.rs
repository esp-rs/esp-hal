//! Starts the APP core.
//!
//! A build can hold esp-hal versions without the `rt` feature. Those versions call
//! [`start_app_core`] and [`app_core_running`] by symbol name. The names and the signatures must
//! stay stable.

use core::sync::atomic::{AtomicBool, AtomicPtr, Ordering};

use crate::{soc::cpu_control::internal_park_core, system::Cpu};

pub(super) static APP_CORE_TAKES_IPC: AtomicBool = AtomicBool::new(false);
static APP_CORE_STACK_TOP: AtomicPtr<u32> = AtomicPtr::new(core::ptr::null_mut());
static APP_CORE_STACK_GUARD: AtomicPtr<u32> = AtomicPtr::new(core::ptr::null_mut());
static APP_CORE_ENTRY: AtomicPtr<()> = AtomicPtr::new(core::ptr::null_mut());

/// Starts the APP core with the stack that ends at `stack_top`. The APP core calls `entry`, and
/// stops when `entry` returns. The start ends all parks of the APP core.
///
/// Returns `false`, and does not start the APP core, if the APP core runs.
///
/// # Safety
///
/// The stack must stay valid while the APP core runs. `stack_guard` must be null or point into
/// the stack.
#[unsafe(export_name = "esp_hal_start_app_core")]
pub(super) unsafe fn start_app_core(
    stack_top: *mut u32,
    stack_guard: *mut u32,
    entry: unsafe fn(),
) -> bool {
    if app_core_running() {
        return false;
    }

    #[cfg(soc_cpu_pma_needs_core1_setup)]
    pma::save_core0_pma();
    APP_CORE_STACK_TOP.store(stack_top, Ordering::Release);
    APP_CORE_STACK_GUARD.store(stack_guard, Ordering::Release);
    APP_CORE_ENTRY.store(entry as *mut (), Ordering::Release);

    app_core_starting();
    crate::soc::cpu_control::start_core1(start_core1_init as *const u32);

    unsafe { internal_park_core(Cpu::AppCpu, false) };

    // The APP core can hold a lock during startup. The flash driver must not park the APP
    // core before startup ends.
    while !app_core_takes_ipc() {
        core::hint::spin_loop();
    }

    true
}

/// Marks the APP core as not running code of this program, and resets its park state, before the
/// APP core starts.
fn app_core_starting() {
    APP_CORE_TAKES_IPC.store(false, Ordering::SeqCst);
    // Posts to the APP core return `Offline` after the store, so no post adds a request
    // to the APP core while this call fails the requests that the APP core did not run.
    unsafe { crate::interrupt::ipc::fail_stranded(Cpu::AppCpu) };
    // The APP core can stop while a `SchedulerSuspendGuard` is alive on the APP core.
    crate::interrupt::ipc::reset_context_switch_deferral(Cpu::AppCpu);
    super::park::reset_app_core();
}

/// Lets the APP core take IPC interrupts. The APP core calls this function after it
/// enables its interrupts.
fn app_core_started() {
    debug_assert_eq!(Cpu::current(), Cpu::AppCpu);
    #[cfg(riscv)]
    crate::interrupt::ipc::install_app();
    APP_CORE_TAKES_IPC.store(true, Ordering::Release);
}

/// Returns `true` if [`super::CpuControl`] started the APP core in this boot, and the core has not
/// stopped since. That core takes IPC interrupts unless it masks them.
pub(crate) fn app_core_takes_ipc() -> bool {
    // A post orders this load after its count in `APP_POSTS`.
    APP_CORE_TAKES_IPC.load(Ordering::SeqCst)
}

/// Returns whether the APP core runs code of this program.
#[unsafe(export_name = "esp_hal_app_core_running")]
pub(super) fn app_core_running() -> bool {
    app_core_takes_ipc() && crate::soc::cpu_control::is_running(Cpu::AppCpu)
}

/// Runs the entry of the APP core.
#[inline(never)]
unsafe fn start_core1_run() -> ! {
    let entry = APP_CORE_ENTRY.load(Ordering::Acquire);
    debug_assert!(!entry.is_null());

    unsafe {
        let entry = core::mem::transmute::<*mut (), unsafe fn()>(entry);
        entry();
    }
    super::park::stop_current_core()
}

#[cfg(soc_cpu_pma_needs_core1_setup)]
mod pma {
    use core::sync::atomic::{AtomicU32, Ordering};

    static CORE0_PMA: [AtomicU32; 32] = [const { AtomicU32::new(0) }; 32];

    macro_rules! read_csr {
        ($csr:literal) => {{
            let value: u32;
            unsafe { core::arch::asm!(concat!("csrr {0}, ", $csr), out(reg) value) };
            value
        }};
    }

    macro_rules! write_csr {
        ($csr:literal, $value:expr) => {
            unsafe { core::arch::asm!(concat!("csrw ", $csr, ", {0}"), in(reg) $value) }
        };
    }

    macro_rules! pma_entries {
        ($($i:literal: $cfg:literal, $addr:literal;)*) => {
            pub(super) fn save_core0_pma() {
                $(
                    CORE0_PMA[$i].store(read_csr!($cfg), Ordering::Relaxed);
                    CORE0_PMA[$i + 16].store(read_csr!($addr), Ordering::Relaxed);
                )*
            }

            pub(super) fn set_core1_pma() {
                $(
                    write_csr!($addr, CORE0_PMA[$i + 16].load(Ordering::Relaxed));
                    write_csr!($cfg, CORE0_PMA[$i].load(Ordering::Relaxed));
                )*
            }
        };
    }

    pma_entries! {
        0: "0xbc0", "0xbd0";
        1: "0xbc1", "0xbd1";
        2: "0xbc2", "0xbd2";
        3: "0xbc3", "0xbd3";
        4: "0xbc4", "0xbd4";
        5: "0xbc5", "0xbd5";
        6: "0xbc6", "0xbd6";
        7: "0xbc7", "0xbd7";
        8: "0xbc8", "0xbd8";
        9: "0xbc9", "0xbd9";
        10: "0xbca", "0xbda";
        11: "0xbcb", "0xbdb";
        12: "0xbcc", "0xbdc";
        13: "0xbcd", "0xbdd";
        14: "0xbce", "0xbde";
        15: "0xbcf", "0xbdf";
    }
}

/// Core 1 entry point, set as the boot address.
#[cfg(xtensa)]
fn start_core1_init() -> ! {
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

        #[cfg(stack_guard_monitoring)]
        {
            let stack_guard = APP_CORE_STACK_GUARD.load(Ordering::Acquire);
            if !stack_guard.is_null() {
                stack_guard.write_volatile(esp_config::esp_config_int!(
                    u32,
                    "ESP_HAL_CONFIG_STACK_GUARD_VALUE"
                ));
            }
            crate::debugger::set_stack_watchpoint(stack_guard as usize);
        }
    }

    // The ROM has already handed off to us; clear the AppCpu boot address so a
    // subsequent reset doesn't see a stale entry point. Matches IDF's
    // `call_start_cpu1`.
    cfg_select! {
        esp32 => {
            crate::peripherals::DPORT::regs()
                .appcpu_ctrl_d()
                .write(|w| unsafe { w.appcpu_boot_addr().bits(0) });
        }
        _ => crate::rom::ets_set_appcpu_boot_addr(0),
    }

    // Do not call setup_interrupts as that would disable peripheral interrupts, too.
    unsafe { crate::interrupt::init_vectoring() };

    enable_app_core_interrupts();

    // Trampoline to run from the new stack.
    // start_core1_run should _NEVER_ be inlined
    // as we rely on the function call to use
    // the new stack.
    unsafe { start_core1_run() }
}

/// Core 1 entry point, set as the boot address.
///
/// ROM jumps here directly, bypassing `_start`, so `gp`, the FPU and the stack pointer need to be
/// set up before entering regular Rust.
#[cfg(riscv)]
#[unsafe(naked)]
extern "C" fn start_core1_init() -> ! {
    core::arch::naked_asm!(
        ".option push",
        ".option norelax",
        "la gp, __global_pointer$",
        "li ra, 0", // ensure probe-rs stops unwinding
        ".option pop",
        // Follow IDF's FPU initialization (rv_utils_enable_fpu, rv_utils_clear_fpu): enable the
        // FPU in Initial state, touch fcsr (which makes the state Dirty), then clear the dirty
        // bit to leave mstatus.FS in Clean state (0b10), which is what the interrupt handler
        // expects.
        "li t0, 0x2000",
        "csrs mstatus, t0",
        "li t0, 1",
        "csrw fcsr, t0",
        "li t0, 0x2000",
        "csrc mstatus, t0",
        // Switch to Core 1's stack (stored by Core 0 before releasing this core).
        "la t0, {stack_top}",
        "lw sp, 0(t0)",
        "j {init}",
        stack_top = sym APP_CORE_STACK_TOP,
        init = sym start_core1_init_impl,
    )
}

#[cfg(riscv)]
fn start_core1_init_impl() -> ! {
    #[cfg(soc_cpu_pma_needs_core1_setup)]
    pma::set_core1_pma();

    crate::soc::branch_predictor::enable();

    // The ROM has already handed off to us; clear the AppCpu boot address so a subsequent software
    // reset doesn't re-enter this stale entry point. Matches IDF's `call_start_cpu1`.
    crate::rom::ets_set_appcpu_boot_addr(0);

    unsafe {
        #[cfg(stack_guard_monitoring)]
        {
            let stack_guard = APP_CORE_STACK_GUARD.load(Ordering::Acquire);
            if !stack_guard.is_null() {
                stack_guard.write_volatile(esp_config::esp_config_int!(
                    u32,
                    "ESP_HAL_CONFIG_STACK_GUARD_VALUE"
                ));
            }
            crate::debugger::set_stack_watchpoint(stack_guard as usize);
        }
        crate::interrupt::init_vectoring();
    }

    enable_app_core_interrupts();

    unsafe { start_core1_run() }
}

fn enable_app_core_interrupts() {
    unsafe {
        crate::interrupt::RunLevel::change(crate::interrupt::RunLevel::ThreadMode);
        #[cfg(riscv)]
        core::arch::asm!("csrsi mstatus, 8", options(nomem, nostack));
    }
    app_core_started();
}
