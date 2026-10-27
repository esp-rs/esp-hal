//! Interrupt stacks.
//!
//! Interrupt handlers run on a stack of their own on each CPU. The entry code of an interrupt
//! saves the interrupted context on the interrupted stack first, then moves to the interrupt
//! stack. A nested interrupt stays on the interrupt stack. Exceptions run on the current stack.
//!
//! The exit code of the outermost handler of [`Priority::min`][super::Priority::min] runs the
//! context switch, if one is pending. See [`super::__rtos_implementation`].
//!
//! The linker script reserves the stacks, `ESP_HAL_CONFIG_ISR_STACK_SIZE` bytes each.

use crate::system::Cpu;

cfg_select! {
    riscv => {
        pub(super) use riscv::TASK_FRAME_SIZE;
        #[cfg(any(context_switch_source = "clint", context_switch_source = "ipc"))]
        pub(super) use riscv::machine_software_interrupt_entry;
    }
    xtensa => {
        pub(super) use xtensa::FRAME_SIZE as TASK_FRAME_SIZE;
    }
}

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
    use core::mem::offset_of;

    use esp_riscv_rt::TrapFrame;

    use crate::interrupt::{
        __rtos_implementation::TaskFrame,
        CpuInterrupt,
        rt::dispatch_interrupts,
    };

    pub(crate) const TASK_FRAME_SIZE: usize = size_of::<TaskFrame>();

    macro_rules! frame_offset {
        ($field:ident) => {
            core::mem::offset_of!(crate::interrupt::__rtos_implementation::TaskFrame, $field)
        };
    }

    /// The part of a [`TaskFrame`] that the exit code saves below the frame of esp-riscv-rt.
    const SWITCH_SIZE: usize = offset_of!(TaskFrame, ra);

    const _: () = {
        core::assert!(TASK_FRAME_SIZE - SWITCH_SIZE == size_of::<TrapFrame>());
        core::assert!(offset_of!(TaskFrame, a0) - SWITCH_SIZE == offset_of!(TrapFrame, a0));
        core::assert!(SWITCH_SIZE.is_multiple_of(16) && TASK_FRAME_SIZE.is_multiple_of(16));
    };

    /// Defines an interrupt entry that moves to the interrupt stack and calls `$handler`.
    ///
    /// The interrupt entry of esp-riscv-rt saves the caller-saved registers on the interrupted
    /// stack, then jumps to the function. Interrupts are disabled. `$handler` takes `a0`, and
    /// returns with interrupts disabled. It returns whether the interrupt has
    /// [`Priority::min`][crate::interrupt::Priority::min], which lets the exit code switch tasks.
    /// The function returns to the entry code on the stack of the context to resume.
    macro_rules! isr_stack_entry {
        ($(#[$attr:meta])* $name:ident($arg:ident: $arg_ty:ty) => $handler:path) => {
            $(#[$attr])*
            #[unsafe(naked)]
            #[unsafe(link_section = ".rwtext")]
            unsafe extern "C" fn $name($arg: $arg_ty) {
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
                        slli t3, t2, 2
                        add  t1, t1, t3
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

                        call {handler}

                        lw   t0, 4(sp)
                        lw   t1, 8(sp)

                        # The handler returns with interrupts disabled.
                        lw   t3, 0(t1)
                        addi t3, t3, -1
                        sw   t3, 0(t1)

                        # Switch tasks only at the outermost exit of a minimum-priority
                        # interrupt. The interrupted context then runs at the base run level.
                        bnez t3, 2f
                        beqz a0, 2f
                        la   t2, {pending}
                        ",
                        multi_core!(
                            "
                        csrr t3, mhartid
                        add  t2, t2, t3
                        "
                        ),
                        "
                        lbu  t2, 0(t2)
                        beqz t2, 2f

                        # Complete the frame of the interrupted context before the switch.
                        addi a0, t0, -{switch_size}
                        sw   s0, {s0}(a0)
                        sw   s1, {s1}(a0)
                        sw   s2, {s2}(a0)
                        sw   s3, {s3}(a0)
                        sw   s4, {s4}(a0)
                        sw   s5, {s5}(a0)
                        sw   s6, {s6}(a0)
                        sw   s7, {s7}(a0)
                        sw   s8, {s8}(a0)
                        sw   s9, {s9}(a0)
                        sw   s10, {s10}(a0)
                        sw   s11, {s11}(a0)
                        sw   tp, {tp}(a0)
                        csrr t2, mepc
                        sw   t2, {pc}(a0)

                        call {switch}

                        # From here on, only the returned frame and this stack are in use.
                        lw   s0, {s0}(a0)
                        lw   s1, {s1}(a0)
                        lw   s2, {s2}(a0)
                        lw   s3, {s3}(a0)
                        lw   s4, {s4}(a0)
                        lw   s5, {s5}(a0)
                        lw   s6, {s6}(a0)
                        lw   s7, {s7}(a0)
                        lw   s8, {s8}(a0)
                        lw   s9, {s9}(a0)
                        lw   s10, {s10}(a0)
                        lw   s11, {s11}(a0)
                        lw   tp, {tp}(a0)
                        lw   t2, {pc}(a0)
                        csrw mepc, t2
                        addi t0, a0, {switch_size}

                    2:
                        lw   ra, 0(sp)
                        mv   sp, t0
                        ret
                        "
                    ),
                    nesting = sym crate::interrupt::isr_stack::NESTING,
                    pending = sym crate::interrupt::__rtos_implementation::SWITCH_PENDING,
                    handler = sym $handler,
                    switch = sym crate::interrupt::__rtos_implementation::switch_context,
                    switch_size = const crate::interrupt::isr_stack::riscv::SWITCH_SIZE,
                    s0 = const frame_offset!(s0),
                    s1 = const frame_offset!(s1),
                    s2 = const frame_offset!(s2),
                    s3 = const frame_offset!(s3),
                    s4 = const frame_offset!(s4),
                    s5 = const frame_offset!(s5),
                    s6 = const frame_offset!(s6),
                    s7 = const frame_offset!(s7),
                    s8 = const frame_offset!(s8),
                    s9 = const frame_offset!(s9),
                    s10 = const frame_offset!(s10),
                    s11 = const frame_offset!(s11),
                    tp = const frame_offset!(tp),
                    pc = const frame_offset!(pc),
                )
            }
        };
    }

    isr_stack_entry! {
        /// Moves to the interrupt stack, and dispatches the interrupt.
        ///
        /// The vectored interrupt entry of esp-riscv-rt jumps here with the CPU interrupt number
        /// in `a0`.
        ///
        /// Handlers that [`enable_direct`][crate::interrupt::enable_direct] binds do not come
        /// here, and run on the interrupted stack.
        #[unsafe(no_mangle)]
        handle_interrupts(_cpu_intr: CpuInterrupt) => dispatch_interrupts
    }

    #[cfg(any(context_switch_source = "clint", context_switch_source = "ipc"))]
    mod machine_software {
        use esp_riscv_rt::TrapFrame;

        cfg_select! {
            context_switch_source = "clint" => {
                use crate::interrupt::__rtos_implementation::machine_software_interrupt as handler;
            }
            _ => {
                use crate::interrupt::ipc::machine_software_interrupt as handler;
            }
        }

        /// The vector entry of the machine software interrupt.
        ///
        /// The interrupt has no peripheral interrupt to dispatch, so the HAL binds this entry to
        /// its CPU interrupt directly. It saves the caller-saved registers in the same way and in
        /// the same place as the vectored interrupt entry of esp-riscv-rt, so that a context
        /// switch can resume a task that either entry saved.
        #[cfg_attr(
            not(interrupt_controller = "clic"),
            unsafe(link_section = ".trap.rust")
        )]
        #[cfg_attr(interrupt_controller = "clic", unsafe(link_section = ".rwtext"))]
        #[unsafe(naked)]
        pub(crate) unsafe extern "C" fn machine_software_interrupt_entry() {
            core::arch::naked_asm!(
                "
                addi sp, sp, -{frame_size}
                sw   a0, {a0}(sp)
                la   a0, {entry}
                j    _continue_interrupt_trap
                ",
                frame_size = const size_of::<TrapFrame>(),
                a0 = const core::mem::offset_of!(TrapFrame, a0),
                entry = sym handle_machine_software_interrupt,
            )
        }

        #[crate::ram]
        extern "C" fn machine_software_interrupt() -> bool {
            handler();
            true
        }

        isr_stack_entry! {
            /// Moves to the interrupt stack, and handles the machine software interrupt.
            handle_machine_software_interrupt(_unused: usize) => machine_software_interrupt
        }
    }

    #[cfg(any(context_switch_source = "clint", context_switch_source = "ipc"))]
    pub(crate) use machine_software::machine_software_interrupt_entry;
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
    pub(crate) const FRAME_SIZE: usize = 256;

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
    /// - `a2`: the frame, and in `$exit`, the frame to resume
    /// - `a3`: `&NESTING[cpu]`
    ///
    /// `$exit` runs after the nesting depth is decremented into `a5`, with interrupts up to
    /// `EXCM_LEVEL` masked, on the interrupt stack.
    macro_rules! entry {
        (
            level: $level:literal,
            name: $name:literal,
            save_ps: $save_ps:literal,
            restore_ps: $restore_ps:literal,
            ret: $ret:literal,
            prefix: $prefix:literal,
            handler: $handler:literal,
            exit: [$($exit:tt)*],
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
                    ",
                    $($exit)*
                    "
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
            .literal .Lesp_hal_isr_pending, {pending}

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
        // Level 1 is the minimum priority. It interrupts only code that runs at INTLEVEL 0, so the
        // nesting depth is 0 here, and the CPU can switch tasks. The frame is complete.
        exit: [
            "
            l32r    a4, .Lesp_hal_isr_pending
            ",
            multi_core!("
            rsr.prid a5
            extui   a5, a5, 13, 1
            add     a4, a4, a5
            "),
            "
            l8ui    a4, a4, 0
            beqz    a4, 3f

            mov     a6, a2
            call4   {switch}
            mov     a2, a6
        3:
            ",
        ],
        operands: {
            exccause = const offset_of!(TrapFrame, EXCCAUSE),
            excvaddr = const offset_of!(TrapFrame, EXCVADDR),
            pending = sym crate::interrupt::__rtos_implementation::SWITCH_PENDING,
            switch = sym crate::interrupt::__rtos_implementation::switch_context,
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
        exit: [],
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
        exit: [],
        operands: {},
    }
}
