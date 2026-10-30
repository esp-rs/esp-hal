//! Interrupt stacks.
//!
//! Interrupt handlers run on a stack of their own on each CPU. The entry code of an interrupt
//! saves the interrupted context on the interrupted stack first, then moves to the interrupt
//! stack. A nested interrupt stays on the interrupt stack. Exceptions run on the current stack.
//!
//! The linker script reserves the stacks, `ESP_HAL_CONFIG_ISR_STACK_SIZE` bytes each.

use crate::system::Cpu;

/// The interrupt nesting depth of each CPU.
///
/// The entry code moves to the interrupt stack only when the depth was 0. The entry and exit
/// code change the depth while interrupts that use the interrupt stack are masked.
static mut NESTING: [u32; Cpu::COUNT] = [0; Cpu::COUNT];

/// Concatenates the assembly on dual-core chips. Expands to an empty string on single-core chips.
#[cfg(multi_core)]
macro_rules! multi_core {
    ($($asm:tt)*) => {
        concat!($($asm)*)
    };
}

#[cfg(not(multi_core))]
macro_rules! multi_core {
    ($($asm:tt)*) => {
        ""
    };
}

#[cfg(riscv)]
mod riscv {
    use super::NESTING;
    use crate::interrupt::{CpuInterrupt, rt::dispatch_interrupts};

    /// Moves to the interrupt stack, and dispatches the interrupt.
    ///
    /// The vectored interrupt entry of esp-riscv-rt saves the caller-saved registers on the
    /// interrupted stack, then jumps here with the CPU interrupt number in `a0`. Interrupts are
    /// disabled. The function returns to the entry code on the interrupted stack, with interrupts
    /// disabled.
    ///
    /// Handlers that [`enable_direct`][crate::interrupt::enable_direct] binds do not come here,
    /// and run on the interrupted stack.
    #[unsafe(naked)]
    #[unsafe(no_mangle)]
    #[unsafe(link_section = ".rwtext")]
    unsafe extern "C" fn handle_interrupts(_cpu_intr: CpuInterrupt) {
        core::arch::naked_asm!(
            concat!(
                "
                mv   t0, sp

                # t1 = &NESTING[hart]
                la   t1, {nesting}
                ",
                multi_core!(
                    "
                csrr t2, mhartid
                slli t2, t2, 2
                add  t1, t1, t2
                "
                ),
                "
                lw   t3, 0(t1)
                addi t4, t3, 1
                sw   t4, 0(t1)
                bnez t3, 1f

                la   sp, _isr_stack_top_cpu0
                ",
                multi_core!(
                    "
                beqz t2, 1f
                la   sp, _isr_stack_top_cpu1
                "
                ),
                "
            1:
                addi sp, sp, -16
                sw   ra, 0(sp)
                sw   t0, 4(sp)
                sw   t1, 8(sp)

                call {dispatch}

                lw   ra, 0(sp)
                lw   t0, 4(sp)
                lw   t1, 8(sp)

                # The dispatcher returns with interrupts disabled.
                lw   t3, 0(t1)
                addi t3, t3, -1
                sw   t3, 0(t1)

                mv   sp, t0
                ret
                "
            ),
            nesting = sym NESTING,
            dispatch = sym dispatch_interrupts,
        )
    }
}

#[cfg(xtensa)]
mod xtensa {
    use core::mem::offset_of;

    use super::NESTING;
    use crate::trapframe::TrapFrame;

    /// The size of the frame that the entry code allocates on the interrupted stack.
    ///
    /// The value and the layout come from xtensa-lx-rt, because `save_context` and
    /// `restore_context` fill and read the frame.
    const FRAME_SIZE: usize = 256;

    /// `save_context` uses a word of the frame at this offset as a scratch slot.
    const FRAME_TMP: usize = 216;

    const _: () = core::assert!(size_of::<TrapFrame>() <= FRAME_TMP);

    const PS_UM: u32 = 1 << 5;
    const PS_WOE: u32 = 1 << 18;

    /// The highest level that `PS.EXCM` masks. Levels up to this one use the interrupt stack.
    const EXCM_LEVEL: u32 = 3;

    /// Saves the interrupted context of `$level` on the interrupted stack, moves to the
    /// interrupt stack, calls the level handler, then restores the context and returns from the
    /// interrupt.
    ///
    /// The vector enters with `call0`, and keeps the interrupted `a0` in `EXCSAVE<level>`.
    /// `PS.EXCM` is set, so interrupts up to `EXCM_LEVEL` are masked until the code writes `PS`.
    ///
    /// Register use after `save_context`, kept by the windowed `call4`:
    /// - `a2`: the frame
    /// - `a3`: `&NESTING[cpu]`
    macro_rules! entry {
        (
            level: $level:literal,
            name: $name:literal,
            save_ps: $save_ps:literal,
            restore_ps: $restore_ps:literal,
            ret: $ret:literal,
            prefix: $prefix:literal,
            handler: $handler:literal,
            operands: { $($operands:tt)* } $(,)?
        ) => {
            core::arch::global_asm!(
                concat!(
                    "
                    .pushsection .rwtext,\"ax\",@progbits
                    .literal .Lesp_hal_isr_nesting_", $level, ", {nesting}
                    .literal .Lesp_hal_isr_top0_", $level, ", _isr_stack_top_cpu0
                    ",
                    multi_core!("
                    .literal .Lesp_hal_isr_top1_", $level, ", _isr_stack_top_cpu1
                    "),
                    $prefix,
                    "
                    .global ", $name, "
                    .p2align 2
                    .type ", $name, ",@function
                ", $name, ":
                    mov     a0, a1
                    addmi   sp, sp, -{frame_size}

                    s32i    a0, sp, {a1}
                    s32e    a0, sp, -12               // for debug backtrace
                    ",
                    $save_ps,
                    "
                    rsr     a0, EPC", $level, "
                    s32i    a0, sp, {pc}
                    s32e    a0, sp, -16               // for debug backtrace
                    rsr     a0, EXCSAVE", $level, "
                    s32i    a0, sp, {a0}

                    call0   save_context

                    // Increment the nesting depth. Move to the interrupt stack if it was 0.
                    mov     a2, sp
                    l32r    a3, .Lesp_hal_isr_nesting_", $level, "
                    ",
                    multi_core!("
                    rsr.prid a4
                    extui   a4, a4, 13, 1
                    addx4   a3, a4, a3
                    "),
                    "
                    l32i    a5, a3, 0
                    addi    a6, a5, 1
                    s32i    a6, a3, 0
                    bnez    a5, 2f

                    l32r    sp, .Lesp_hal_isr_top0_", $level, "
                    ",
                    multi_core!("
                    beqz    a4, 1f
                    l32r    sp, .Lesp_hal_isr_top1_", $level, "
                1:
                    "),
                    "
                    // Let a backtrace continue from the interrupt stack to the interrupted code.
                    l32i    a5, a2, {pc}
                    s32e    a5, sp, -16
                    l32i    a5, a2, {a1}
                    s32e    a5, sp, -12
                2:
                    movi    a0, (", $level, " | {ps_woe} | {ps_um})
                    wsr     a0, PS
                    rsync

                    mov     a6, a2
                    call4   ", $handler, "

                    // Decrement the nesting depth, then go back to the frame.
                    rsil    a0, {excm_level}
                    l32i    a5, a3, 0
                    addi    a5, a5, -1
                    s32i    a5, a3, 0
                    mov     sp, a2

                    call0   restore_context
                    ",
                    $restore_ps,
                    "
                    l32i    a0, sp, {pc}
                    wsr     a0, EPC", $level, "
                    l32i    a0, sp, {a0}
                    l32i    sp, sp, {a1}
                    rsync
                    ", $ret, "
                    .size ", $name, ", . - ", $name, "
                    .popsection
                    "
                ),
                nesting = sym NESTING,
                frame_size = const FRAME_SIZE,
                a0 = const offset_of!(TrapFrame, A0),
                a1 = const offset_of!(TrapFrame, A1),
                pc = const offset_of!(TrapFrame, PC),
                ps = const offset_of!(TrapFrame, PS),
                ps_woe = const PS_WOE,
                ps_um = const PS_UM,
                excm_level = const EXCM_LEVEL,
                $($operands)*
            );
        };
    }

    // Level 1 shares its vector with exceptions. Exceptions keep the xtensa-lx-rt path and the
    // current stack.
    entry! {
        level: 1,
        name: "__esp_hal_level_1_interrupt_entry",
        save_ps: "
            rsr     a0, PS
            s32i    a0, sp, {ps}
            rsr     a0, EXCCAUSE
            s32i    a0, sp, {exccause}
            rsr     a0, EXCVADDR
            s32i    a0, sp, {excvaddr}
        ",
        restore_ps: "
            l32i    a0, sp, {ps}
            wsr     a0, PS
        ",
        ret: "rfe",
        prefix: "
            .literal .Lesp_hal_isr_exception, __default_naked_exception

            .global __naked_user_exception
            .global __naked_kernel_exception
            .p2align 2
            .type __naked_user_exception,@function
            .type __naked_kernel_exception,@function
        __naked_user_exception:
        __naked_kernel_exception:
            rsr     a0, EXCCAUSE
            beqi    a0, 4, __esp_hal_level_1_interrupt_entry    // EXCCAUSE_LEVEL1_INTERRUPT
            l32r    a0, .Lesp_hal_isr_exception
            jx      a0
            .size __naked_user_exception, . - __naked_user_exception
            .size __naked_kernel_exception, . - __naked_kernel_exception
        ",
        handler: "__level_1_interrupt",
        operands: {
            exccause = const offset_of!(TrapFrame, EXCCAUSE),
            excvaddr = const offset_of!(TrapFrame, EXCVADDR),
        },
    }

    entry! {
        level: 2,
        name: "__naked_level_2_interrupt",
        save_ps: "
            rsr     a0, EPS2
            s32i    a0, sp, {ps}
        ",
        restore_ps: "
            l32i    a0, sp, {ps}
            wsr     a0, EPS2
        ",
        ret: "rfi 2",
        prefix: "",
        handler: "__level_2_interrupt",
        operands: {},
    }

    entry! {
        level: 3,
        name: "__naked_level_3_interrupt",
        save_ps: "
            rsr     a0, EPS3
            s32i    a0, sp, {ps}
        ",
        restore_ps: "
            l32i    a0, sp, {ps}
            wsr     a0, EPS3
        ",
        ret: "rfi 3",
        prefix: "",
        handler: "__level_3_interrupt",
        operands: {},
    }
}
