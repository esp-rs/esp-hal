//! Software CPU retention across a light sleep.
//!
//! esp-idf saves the CPU-domain device registers with plain loops, not with the PAU regdma engine
//! (`esp32c6/sleep_cpu.c:237-341`).

use core::{mem::offset_of, ptr, slice};

use portable_atomic::{AtomicPtr, Ordering};

use super::{
    chips::chip,
    device_regs,
    frames::chip::{
        CriticalSleepFrame,
        NonCriticalSleepFrame,
        PMUFUNC_GOING_TO_SLEEP,
        PMUFUNC_JUST_WOKE,
    },
};
use crate::system::{self, Cpu};

// The wake stub is an `extern "C" fn()` with no argument, so the critical frame address of each
// core lives here. Every core of a chip returns through the one wake stub register, so the restore
// assembly picks its entry by hart id.
static CRITICAL_FRAME_PTR: [AtomicPtr<CriticalSleepFrame>; Cpu::COUNT] =
    [const { AtomicPtr::new(ptr::null_mut()) }; Cpu::COUNT];

/// Emits `critical_regs_save` and `critical_regs_restore` for one frame shape.
///
/// The two shapes differ by the registers that only one of them holds. `save_extra` runs after the
/// save wrote `mepc`, and `restore_extra` before the restore reads it back, which is where
/// `<chip>/sleep_cpu_asm.S` puts them.
macro_rules! critical_regs_asm {
    // A frame shape that holds the FPU registers. Every such chip moves the same registers, so
    // this block lives here once, and it runs after the extras of the caller. esp-idf moves the
    // FPU only when the FreeRTOS port marks the context dirty; esp-hal tracks no such state, so
    // this code moves it every time.
    //
    // Both halves set `mstatus.FS` before they touch an FPU register, as `sleep_fpu_asm.S` does,
    // because an access traps while the unit is off. The bit needs no undo: `mstatus` is in the
    // frame, and the restore writes it back after the FPU registers. A core that had the unit off
    // saves and restores register values that no later code can read.
    (
        fpu,
        save_extra: [$($save_extra:literal),* $(,)?],
        restore_extra: [$($restore_extra:literal),* $(,)?]
        $(, $operand:ident = const $value:expr)* $(,)?
    ) => {
        critical_regs_asm!(
            save_extra: [
                $($save_extra,)*
                "li t2, {fpu_enable}",
                "csrs mstatus, t2",
                "fsw ft0, {fpu_ft0}(t0)",
                "fsw ft1, {fpu_ft1}(t0)",
                "fsw ft2, {fpu_ft2}(t0)",
                "fsw ft3, {fpu_ft3}(t0)",
                "fsw ft4, {fpu_ft4}(t0)",
                "fsw ft5, {fpu_ft5}(t0)",
                "fsw ft6, {fpu_ft6}(t0)",
                "fsw ft7, {fpu_ft7}(t0)",
                "fsw fs0, {fpu_fs0}(t0)",
                "fsw fs1, {fpu_fs1}(t0)",
                "fsw fa0, {fpu_fa0}(t0)",
                "fsw fa1, {fpu_fa1}(t0)",
                "fsw fa2, {fpu_fa2}(t0)",
                "fsw fa3, {fpu_fa3}(t0)",
                "fsw fa4, {fpu_fa4}(t0)",
                "fsw fa5, {fpu_fa5}(t0)",
                "fsw fa6, {fpu_fa6}(t0)",
                "fsw fa7, {fpu_fa7}(t0)",
                "fsw fs2, {fpu_fs2}(t0)",
                "fsw fs3, {fpu_fs3}(t0)",
                "fsw fs4, {fpu_fs4}(t0)",
                "fsw fs5, {fpu_fs5}(t0)",
                "fsw fs6, {fpu_fs6}(t0)",
                "fsw fs7, {fpu_fs7}(t0)",
                "fsw fs8, {fpu_fs8}(t0)",
                "fsw fs9, {fpu_fs9}(t0)",
                "fsw fs10, {fpu_fs10}(t0)",
                "fsw fs11, {fpu_fs11}(t0)",
                "fsw ft8, {fpu_ft8}(t0)",
                "fsw ft9, {fpu_ft9}(t0)",
                "fsw ft10, {fpu_ft10}(t0)",
                "fsw ft11, {fpu_ft11}(t0)",
                "csrr t2, fcsr",
                "sw t2, {fpu_fcsr}(t0)",
            ],
            restore_extra: [
                $($restore_extra,)*
                "li t2, {fpu_enable}",
                "csrs mstatus, t2",
                "flw ft0, {fpu_ft0}(t0)",
                "flw ft1, {fpu_ft1}(t0)",
                "flw ft2, {fpu_ft2}(t0)",
                "flw ft3, {fpu_ft3}(t0)",
                "flw ft4, {fpu_ft4}(t0)",
                "flw ft5, {fpu_ft5}(t0)",
                "flw ft6, {fpu_ft6}(t0)",
                "flw ft7, {fpu_ft7}(t0)",
                "flw fs0, {fpu_fs0}(t0)",
                "flw fs1, {fpu_fs1}(t0)",
                "flw fa0, {fpu_fa0}(t0)",
                "flw fa1, {fpu_fa1}(t0)",
                "flw fa2, {fpu_fa2}(t0)",
                "flw fa3, {fpu_fa3}(t0)",
                "flw fa4, {fpu_fa4}(t0)",
                "flw fa5, {fpu_fa5}(t0)",
                "flw fa6, {fpu_fa6}(t0)",
                "flw fa7, {fpu_fa7}(t0)",
                "flw fs2, {fpu_fs2}(t0)",
                "flw fs3, {fpu_fs3}(t0)",
                "flw fs4, {fpu_fs4}(t0)",
                "flw fs5, {fpu_fs5}(t0)",
                "flw fs6, {fpu_fs6}(t0)",
                "flw fs7, {fpu_fs7}(t0)",
                "flw fs8, {fpu_fs8}(t0)",
                "flw fs9, {fpu_fs9}(t0)",
                "flw fs10, {fpu_fs10}(t0)",
                "flw fs11, {fpu_fs11}(t0)",
                "flw ft8, {fpu_ft8}(t0)",
                "flw ft9, {fpu_ft9}(t0)",
                "flw ft10, {fpu_ft10}(t0)",
                "flw ft11, {fpu_ft11}(t0)",
                "lw t2, {fpu_fcsr}(t0)",
                "csrw fcsr, t2",
            ],
            $($operand = const $value,)*
            fpu_enable = const 1 << 13,
            fpu_ft0 = const offset_of!(CriticalSleepFrame, fpu_ft0),
            fpu_ft1 = const offset_of!(CriticalSleepFrame, fpu_ft1),
            fpu_ft2 = const offset_of!(CriticalSleepFrame, fpu_ft2),
            fpu_ft3 = const offset_of!(CriticalSleepFrame, fpu_ft3),
            fpu_ft4 = const offset_of!(CriticalSleepFrame, fpu_ft4),
            fpu_ft5 = const offset_of!(CriticalSleepFrame, fpu_ft5),
            fpu_ft6 = const offset_of!(CriticalSleepFrame, fpu_ft6),
            fpu_ft7 = const offset_of!(CriticalSleepFrame, fpu_ft7),
            fpu_fs0 = const offset_of!(CriticalSleepFrame, fpu_fs0),
            fpu_fs1 = const offset_of!(CriticalSleepFrame, fpu_fs1),
            fpu_fa0 = const offset_of!(CriticalSleepFrame, fpu_fa0),
            fpu_fa1 = const offset_of!(CriticalSleepFrame, fpu_fa1),
            fpu_fa2 = const offset_of!(CriticalSleepFrame, fpu_fa2),
            fpu_fa3 = const offset_of!(CriticalSleepFrame, fpu_fa3),
            fpu_fa4 = const offset_of!(CriticalSleepFrame, fpu_fa4),
            fpu_fa5 = const offset_of!(CriticalSleepFrame, fpu_fa5),
            fpu_fa6 = const offset_of!(CriticalSleepFrame, fpu_fa6),
            fpu_fa7 = const offset_of!(CriticalSleepFrame, fpu_fa7),
            fpu_fs2 = const offset_of!(CriticalSleepFrame, fpu_fs2),
            fpu_fs3 = const offset_of!(CriticalSleepFrame, fpu_fs3),
            fpu_fs4 = const offset_of!(CriticalSleepFrame, fpu_fs4),
            fpu_fs5 = const offset_of!(CriticalSleepFrame, fpu_fs5),
            fpu_fs6 = const offset_of!(CriticalSleepFrame, fpu_fs6),
            fpu_fs7 = const offset_of!(CriticalSleepFrame, fpu_fs7),
            fpu_fs8 = const offset_of!(CriticalSleepFrame, fpu_fs8),
            fpu_fs9 = const offset_of!(CriticalSleepFrame, fpu_fs9),
            fpu_fs10 = const offset_of!(CriticalSleepFrame, fpu_fs10),
            fpu_fs11 = const offset_of!(CriticalSleepFrame, fpu_fs11),
            fpu_ft8 = const offset_of!(CriticalSleepFrame, fpu_ft8),
            fpu_ft9 = const offset_of!(CriticalSleepFrame, fpu_ft9),
            fpu_ft10 = const offset_of!(CriticalSleepFrame, fpu_ft10),
            fpu_ft11 = const offset_of!(CriticalSleepFrame, fpu_ft11),
            fpu_fcsr = const offset_of!(CriticalSleepFrame, fpu_fcsr),
        );
    };

    (
        save_extra: [$($save_extra:literal),* $(,)?],
        restore_extra: [$($restore_extra:literal),* $(,)?]
        $(, $operand:ident = const $value:expr)* $(,)?
    ) => {
        core::arch::global_asm!(
            ".section .rwtext,\"ax\",@progbits",
            ".global critical_regs_save",
            ".type critical_regs_save, @function",
            ".align 4",
            "critical_regs_save:",
            "csrw mscratch, t0",
            "mv t0, a0",
            "sw ra, {ra}(t0)",
            "sw sp, {sp}(t0)",
            "sw gp, {gp}(t0)",
            "sw tp, {tp}(t0)",
            "sw t1, {t1}(t0)",
            "sw t2, {t2}(t0)",
            "sw s0, {s0}(t0)",
            "sw s1, {s1}(t0)",
            "mv a0, t0",
            "sw a0, {a0}(t0)",
            "sw a1, {a1}(t0)",
            "sw a2, {a2}(t0)",
            "sw a3, {a3}(t0)",
            "sw a4, {a4}(t0)",
            "sw a5, {a5}(t0)",
            "sw a6, {a6}(t0)",
            "sw a7, {a7}(t0)",
            "sw s2, {s2}(t0)",
            "sw s3, {s3}(t0)",
            "sw s4, {s4}(t0)",
            "sw s5, {s5}(t0)",
            "sw s6, {s6}(t0)",
            "sw s7, {s7}(t0)",
            "sw s8, {s8}(t0)",
            "sw s9, {s9}(t0)",
            "sw s10, {s10}(t0)",
            "sw s11, {s11}(t0)",
            "sw t3, {t3}(t0)",
            "sw t4, {t4}(t0)",
            "sw t5, {t5}(t0)",
            "sw t6, {t6}(t0)",
            "csrr t1, mstatus",
            "sw t1, {mstatus}(t0)",
            "csrr t2, mtvec",
            "sw t2, {mtvec}(t0)",
            "csrr t3, mcause",
            "sw t3, {mcause}(t0)",
            "csrr t1, mtval",
            "sw t1, {mtval}(t0)",
            "csrr t2, mie",
            "sw t2, {mie}(t0)",
            "csrr t3, mip",
            "sw t3, {mip}(t0)",
            "csrr t1, mepc",
            "sw t1, {mepc}(t0)",
            $($save_extra,)*
            "li t1, ~0x3",
            "lw t2, {pmufunc}(t0)",
            "and t2, t1, t2",
            "ori t2, t2, {going_to_sleep}",
            "sw t2, {pmufunc}(t0)",
            "mv t3, t0",
            "csrr t0, mscratch",
            "lw t1, {t1}(t3)",
            "lw t2, {t2}(t3)",
            "lw t3, {t3}(t3)",
            "ret",
            ".size critical_regs_save, . - critical_regs_save",

            ".global critical_regs_restore",
            ".type critical_regs_restore, @function",
            ".align 4",
            "critical_regs_restore:",
            "la t1, {critical_frame_ptr}",
            // The mask keeps a single-core chip on entry 0 without a second copy of this block.
            // No general register is live here, so `t1` and `t2` are free.
            "csrr t2, mhartid",
            "andi t2, t2, {core_mask}",
            "slli t2, t2, 2",
            "add t1, t1, t2",
            "lw t0, 0(t1)",
            "beqz t0, 2f",
            "lw t1, {pmufunc}(t0)",
            "ori t1, t1, {just_woke}",
            "sw t1, {pmufunc}(t0)",
            $($restore_extra,)*
            "lw t2, {mepc}(t0)",
            "csrw mepc, t2",
            "lw t3, {mip}(t0)",
            "csrw mip, t3",
            "lw t1, {mie}(t0)",
            "csrw mie, t1",
            "lw t2, {mstatus}(t0)",
            "csrw mstatus, t2",
            "lw t3, {mtvec}(t0)",
            "csrw mtvec, t3",
            "lw t1, {mcause}(t0)",
            "csrw mcause, t1",
            "lw t2, {mtval}(t0)",
            "csrw mtval, t2",
            "lw t6, {t6}(t0)",
            "lw t5, {t5}(t0)",
            "lw t4, {t4}(t0)",
            "lw t3, {t3}(t0)",
            "lw s11, {s11}(t0)",
            "lw s10, {s10}(t0)",
            "lw s9, {s9}(t0)",
            "lw s8, {s8}(t0)",
            "lw s7, {s7}(t0)",
            "lw s6, {s6}(t0)",
            "lw s5, {s5}(t0)",
            "lw s4, {s4}(t0)",
            "lw s3, {s3}(t0)",
            "lw s2, {s2}(t0)",
            "lw a7, {a7}(t0)",
            "lw a6, {a6}(t0)",
            "lw a5, {a5}(t0)",
            "lw a4, {a4}(t0)",
            "lw a3, {a3}(t0)",
            "lw a2, {a2}(t0)",
            "lw a1, {a1}(t0)",
            "lw a0, {a0}(t0)",
            "lw s1, {s1}(t0)",
            "lw s0, {s0}(t0)",
            "lw t2, {t2}(t0)",
            "lw t1, {t1}(t0)",
            "lw tp, {tp}(t0)",
            "lw gp, {gp}(t0)",
            "lw sp, {sp}(t0)",
            "lw ra, {ra}(t0)",
            "lw t0, {t0}(t0)",
            "2:",
            "ret",
            ".size critical_regs_restore, . - critical_regs_restore",
            ra = const offset_of!(CriticalSleepFrame, ra),
            sp = const offset_of!(CriticalSleepFrame, sp),
            gp = const offset_of!(CriticalSleepFrame, gp),
            tp = const offset_of!(CriticalSleepFrame, tp),
            t0 = const offset_of!(CriticalSleepFrame, t0),
            t1 = const offset_of!(CriticalSleepFrame, t1),
            t2 = const offset_of!(CriticalSleepFrame, t2),
            s0 = const offset_of!(CriticalSleepFrame, s0),
            s1 = const offset_of!(CriticalSleepFrame, s1),
            a0 = const offset_of!(CriticalSleepFrame, a0),
            a1 = const offset_of!(CriticalSleepFrame, a1),
            a2 = const offset_of!(CriticalSleepFrame, a2),
            a3 = const offset_of!(CriticalSleepFrame, a3),
            a4 = const offset_of!(CriticalSleepFrame, a4),
            a5 = const offset_of!(CriticalSleepFrame, a5),
            a6 = const offset_of!(CriticalSleepFrame, a6),
            a7 = const offset_of!(CriticalSleepFrame, a7),
            s2 = const offset_of!(CriticalSleepFrame, s2),
            s3 = const offset_of!(CriticalSleepFrame, s3),
            s4 = const offset_of!(CriticalSleepFrame, s4),
            s5 = const offset_of!(CriticalSleepFrame, s5),
            s6 = const offset_of!(CriticalSleepFrame, s6),
            s7 = const offset_of!(CriticalSleepFrame, s7),
            s8 = const offset_of!(CriticalSleepFrame, s8),
            s9 = const offset_of!(CriticalSleepFrame, s9),
            s10 = const offset_of!(CriticalSleepFrame, s10),
            s11 = const offset_of!(CriticalSleepFrame, s11),
            t3 = const offset_of!(CriticalSleepFrame, t3),
            t4 = const offset_of!(CriticalSleepFrame, t4),
            t5 = const offset_of!(CriticalSleepFrame, t5),
            t6 = const offset_of!(CriticalSleepFrame, t6),
            mstatus = const offset_of!(CriticalSleepFrame, mstatus),
            mtvec = const offset_of!(CriticalSleepFrame, mtvec),
            mcause = const offset_of!(CriticalSleepFrame, mcause),
            mtval = const offset_of!(CriticalSleepFrame, mtval),
            mie = const offset_of!(CriticalSleepFrame, mie),
            mip = const offset_of!(CriticalSleepFrame, mip),
            mepc = const offset_of!(CriticalSleepFrame, mepc),
            pmufunc = const offset_of!(CriticalSleepFrame, pmufunc),
            going_to_sleep = const PMUFUNC_GOING_TO_SLEEP,
            just_woke = const PMUFUNC_JUST_WOKE,
            critical_frame_ptr = sym CRITICAL_FRAME_PTR,
            core_mask = const Cpu::COUNT - 1,
            $($operand = const $value,)*
        );
    };
}

#[cfg(cpu_retention_frame = "c6_h2")]
critical_regs_asm!(save_extra: [], restore_extra: []);

// A CLIC part also holds the interrupt threshold. `MINTTHRESH_CSR` comes from
// `components/riscv/include/riscv/csr_clic.h`.
#[cfg(cpu_retention_frame = "clic")]
critical_regs_asm!(
    save_extra: ["csrr t2, {mintthresh_csr}", "sw t2, {mintthresh}(t0)"],
    restore_extra: ["lw t2, {mintthresh}(t0)", "csrw {mintthresh_csr}, t2"],
    mintthresh = const offset_of!(CriticalSleepFrame, mintthresh),
    mintthresh_csr = const 0x347,
);

// The S31 critical frame also holds the interrupt threshold and the FPU.
#[cfg(cpu_retention_frame = "s31")]
critical_regs_asm!(
    fpu,
    save_extra: ["csrr t2, {mintthresh_csr}", "sw t2, {mintthresh}(t0)"],
    restore_extra: ["lw t2, {mintthresh}(t0)", "csrw {mintthresh_csr}, t2"],
    mintthresh = const offset_of!(CriticalSleepFrame, mintthresh),
    mintthresh_csr = const 0x347,
);

// The P4 critical frame also holds the trap-vector table base, the interrupt threshold and the
// FPU. `MTVT_CSR` comes from `components/riscv/include/riscv/csr_clic.h`.
#[cfg(cpu_retention_frame = "p4")]
critical_regs_asm!(
    fpu,
    save_extra: [
        "csrr t2, {mtvt_csr}",
        "sw t2, {mtvt}(t0)",
        "csrr t2, {mintthresh_csr}",
        "sw t2, {mintthresh}(t0)",
    ],
    restore_extra: [
        "lw t2, {mtvt}(t0)",
        "csrw {mtvt_csr}, t2",
        "lw t2, {mintthresh}(t0)",
        "csrw {mintthresh_csr}, t2",
    ],
    mtvt = const offset_of!(CriticalSleepFrame, mtvt),
    mtvt_csr = const 0x307,
    mintthresh = const offset_of!(CriticalSleepFrame, mintthresh),
    mintthresh_csr = const 0x347,
);

unsafe extern "C" {
    fn critical_regs_save(frame: *mut CriticalSleepFrame) -> *mut CriticalSleepFrame;
    fn critical_regs_restore();
}

/// Per-core retention buffer layout for one save or restore pass.
pub(crate) struct CoreRetentionContext {
    pub(crate) buffer: *mut u8,
    pub(crate) core: usize,
}

impl CoreRetentionContext {
    #[crate::ram]
    pub(crate) fn new(buffer: *mut u8, core: usize) -> Self {
        // SAFETY: `install_cpu_retention_memory` checked the buffer against the size of every
        // block.
        let buffer = unsafe { buffer.add(core * chip::BLOCK_SIZE) };
        Self { buffer, core }
    }

    #[crate::ram]
    pub(crate) fn device_frame(&mut self) -> &mut [u32] {
        // SAFETY: the buffer is installed and sized for this chip.
        unsafe {
            slice::from_raw_parts_mut(
                self.buffer.add(chip::DEVICE_REGIONS_OFFSET) as *mut u32,
                chip::DEVICE_REGION_WORDS,
            )
        }
    }

    #[crate::ram]
    pub(crate) fn non_critical(&self) -> *mut NonCriticalSleepFrame {
        // SAFETY: the buffer is installed and sized for this chip.
        unsafe { self.buffer.add(chip::NON_CRITICAL_FRAME_OFFSET) as *mut NonCriticalSleepFrame }
    }

    #[crate::ram]
    pub(crate) fn critical(&self) -> *mut CriticalSleepFrame {
        // SAFETY: the buffer is installed and sized for this chip.
        unsafe { self.buffer.add(chip::CRITICAL_FRAME_OFFSET) as *mut CriticalSleepFrame }
    }
}

/// Saves the device registers and the non-critical frame.
#[crate::ram]
pub(crate) fn save_pre_critical(ctx: &mut CoreRetentionContext) {
    device_regs::save(&chip::regions(), ctx.device_frame());
    // SAFETY: the buffer is installed and sized for this chip.
    unsafe { ctx.non_critical().as_mut().unwrap().save() };
}

/// Saves the critical frame and returns its pointer.
#[crate::ram]
pub(crate) fn save_critical_frame(ctx: &CoreRetentionContext) -> *mut CriticalSleepFrame {
    let critical = ctx.critical();
    CRITICAL_FRAME_PTR[ctx.core].store(critical, Ordering::Release);
    // This call returns twice. It returns here after it saved the frame, and again from the wake
    // stub, with every register of the frame back in place.
    unsafe { critical_regs_save(critical) }
}

/// Saves and restores the CPU domain around a light sleep request.
///
/// The caller must run with interrupts disabled on this core. [`LowPower::sleep_light`] holds that
/// state for the whole sleep path.
///
/// `enter_sleep` requests the sleep. `wait` blocks until the hardware reports the result.
///
/// esp-idf does not write back or invalidate the cache on this chip. It saves and restores the
/// cache configuration registers in the device-region frame instead
/// (`esp32c6/sleep_cpu.c:317-341`).
#[crate::ram]
pub(crate) fn sleep_retained(buffer: *mut u8, enter_sleep: fn(), wait: fn() -> bool) -> bool {
    // The rendezvous exists on a multi-core chip only, and this body is what it delegates to.
    #[cfg(all(multi_core, feature = "rt", feature = "unstable"))]
    if super::rendezvous::helper_enlisted() {
        return super::rendezvous::sleep_retained(buffer, enter_sleep, wait);
    }

    let core = system::raw_core();
    let mut ctx = CoreRetentionContext::new(buffer, core);
    save_pre_critical(&mut ctx);
    let frame = save_critical_frame(&ctx);

    // The wake stub writes this word while the compiler believes that nothing did.
    // SAFETY: `save_critical_frame` returns the frame pointer that it was given.
    let pmufunc = unsafe { ptr::read_volatile(&raw const (*frame).pmufunc) };
    let rejected = if pmufunc & 3 == PMUFUNC_GOING_TO_SLEEP {
        arm_wake_stub();
        enter_sleep();
        wait()
    } else {
        false
    };

    // A rejected request never powered the domain down, so the registers still hold what the save
    // read. Writing them back would repeat side effects such as a PLIC interrupt claim.
    // `esp_sleep_cpu_retention` restores on a wake only, for the same reason
    // (`esp32c6/sleep_cpu.c:331-341`).
    if !rejected {
        // SAFETY: the frame was filled on this path before the sleep.
        unsafe { ctx.non_critical().as_mut().unwrap().restore() };
        device_regs::restore(&chip::regions(), &*ctx.device_frame());
    }

    rejected
}

/// Clears the wake stub address that [`arm_wake_stub`] wrote.
#[crate::ram]
pub(crate) fn disarm_wake_stub() {
    // SAFETY: the register is the retention word of this chip, and it holds no other state.
    unsafe { chip::wake_stub_reg().write_volatile(0) };
}

#[crate::ram]
pub(crate) fn arm_wake_stub() {
    let stub = critical_regs_restore as *const () as usize as u32;
    // SAFETY: the register is the retention word of this chip, and it holds no other state.
    unsafe { chip::wake_stub_reg().write_volatile(stub) };
}
