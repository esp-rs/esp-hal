//! Private implementation details of the RTOS that owns context switching.
//!
//! A context switch runs at [`Priority::min`][super::Priority::min]. It is the last action of a
//! CPU before the CPU returns to the interrupted context. An RTOS registers its context-switch
//! handler here, then asks a CPU to run the handler.
//!
//! These functions are not part of the application API.
#![doc = ""]
#![doc = cfg_select! {
    context_switch_source = "software0" => "A CPU switches its own tasks through the `Software0` CPU-internal interrupt. The HAL defines the handler of that interrupt.",
    context_switch_source = "clint" => "A CPU switches its own tasks through the machine software interrupt of the CLINT.",
    context_switch_source = "from_cpu" => "A CPU switches its own tasks through the `FROM_CPU_INTR0` interrupt. The HAL reserves that interrupt.",
    _ => "",
}]
#![cfg_attr(
    multi_core,
    doc = "\nA CPU switches the tasks of the other CPU through [`Ipc`][super::ipc::Ipc]."
)]

use crate::system::Cpu;
#[cfg(xtensa)]
use crate::trapframe::TrapFrame;

/// The signature of a context-switch handler.
#[doc = ""]
#[doc = cfg_select! {
    riscv => "The handler runs as a direct-bound interrupt handler, and returns from the interrupt itself.",
    xtensa => "The handler runs at the exit of the level 1 interrupt, after all level 1 handlers, on the stack that [`set_context_switch_stack`] sets, with levels 1 to 3 masked. `saved` points to the context of the interrupted code, on the interrupted stack. `next` points to 256 bytes of free space at the top of the switch stack. The handler returns the context to restore: `saved`, or `next` after it copies the context of another task there.",
}]
pub type ContextSwitchHandler = cfg_select! {
    riscv => {
        unsafe extern "C" fn()
    }
    xtensa => {
        extern "C" fn(saved: *mut TrapFrame, next: *mut TrapFrame) -> *mut TrapFrame
    }
};

/// Sets the stack that the context-switch handler of `core` runs on.
///
/// Call this function before [`set_context_switch_handler`]. The top 256 bytes of the stack hold
/// the next context. Interrupts above level 3 can occur during a switch, and save their context
/// on this stack.
///
/// # Safety
///
/// `stack` must stay valid, and only the context switch of `core` may use it. Its end must be
/// aligned to 16 bytes.
#[cfg(xtensa)]
pub unsafe fn set_context_switch_stack(core: Cpu, stack: *mut [u8]) {
    let top = unsafe { stack.cast::<u8>().add(stack.len()) };
    assert!(
        stack.len() > exit::NEXT_CONTEXT_SIZE && top.addr().is_multiple_of(16),
        "The context switch stack is too small or not aligned"
    );
    exit::set_stack(core, top);
}

/// Registers the context-switch handler of `core`.
///
/// A CPU accepts one context-switch handler. [`request_context_switch`] therefore takes no
/// handler of its own, and never waits.
///
/// # Panics
///
/// Panics if `core` already has a context-switch handler.
#[cfg_attr(
    not(context_switch_source = "ipc"),
    doc = "\nIn debug builds, panics if `core` is not the current CPU. A CPU-local interrupt
    carries the context switch, and only that CPU can enable the interrupt.\n"
)]
/// # Safety
#[doc = ""]
#[doc = cfg_select! {
    riscv => "The interrupt jumps to `handler` with the interrupted `t0` in `mscratch`. `handler` must restore `t0` from that CSR, save and restore every other register it modifies, and return with `mret`.",
    xtensa => "[`set_context_switch_stack`] must have set the stack of `core`. `handler` must return a context that the CPU can run.",
}]
#[cfg_attr(
    multi_core,
    doc = "",
    doc = "The other CPU can take the interrupted task as soon as the RTOS makes it available, for example when the scheduler releases its lock. Before that point, the RTOS must store the complete context of the task. After that point, the CPU must not use the stack of the task.",
    doc = cfg_select! {
        xtensa => "`handler` runs on its own stack, so only the context needs care.",
        riscv => "`handler` starts on the stack of the task, and must move to another stack before that point. The esp-rtos handler shows one way to do this.",
    }
)]
pub unsafe fn set_context_switch_handler(core: Cpu, handler: ContextSwitchHandler) {
    #[cfg(xtensa)]
    debug_assert!(
        exit::has_stack(core),
        "Set the context switch stack before the handler"
    );

    cfg_select! {
        context_switch_source = "ipc" => {}
        _ => {
            debug_assert_eq!(core, Cpu::current(), "A CPU installs its own handler");
            unsafe { local::install(handler) };
        }
    }

    // A CPU-local interrupt cannot reach the other CPU. Dual-core chips that keep
    // that interrupt for a local switch still register the same handler with IPC.
    #[cfg(multi_core)]
    unsafe {
        super::ipc::set_context_switch_handler(core, handler);
    }
}

/// Asks `core` to run its context-switch handler.
///
/// The handler runs at [`Priority::min`][super::Priority::min]. The call does not wait.
///
/// # Panics
///
/// In debug builds, panics if `core` has no context-switch handler.
#[cfg_attr(
    multi_core,
    doc = "\nIn debug builds, panics if `core` is the APP CPU and that CPU is stopped or not
    started."
)]
#[inline]
pub fn request_context_switch(core: Cpu) {
    cfg_select! {
        context_switch_source = "ipc" => {
            super::ipc::request_context_switch(core);
        }
        multi_core => {
            if core == Cpu::current() {
                local::trigger();
            } else {
                super::ipc::request_context_switch(core);
            }
        }
        _ => {
            debug_assert_eq!(core, Cpu::current(), "A CPU switches its own tasks");
            local::trigger();
        }
    }
}

// esp-hal copies without the `rt` feature call the two functions below by name. The names and
// signatures must stay stable.

/// Defers the context switches of the current CPU, so that the current thread stays on the CPU.
///
/// Interrupt handlers and IPC requests still run. The deferral lasts until each call has a
/// matching call to [`resume_context_switch`].
#[cfg(multi_core)]
#[crate::ram]
#[unsafe(export_name = "esp_hal_defer_context_switch")]
pub fn defer_context_switch() {
    super::ipc::defer_context_switch();
}

/// Ends one deferral of the current CPU. The end of the last deferral requests the deferred
/// context switch.
///
/// # Safety
///
/// The call must match an earlier call to [`defer_context_switch`] on the current CPU.
#[cfg(multi_core)]
#[crate::ram]
#[unsafe(export_name = "esp_hal_resume_context_switch")]
pub unsafe fn resume_context_switch() {
    unsafe { super::ipc::resume_context_switch() };
}

/// Masks the `Software0` context switch of the current CPU, and returns whether the context switch
/// was enabled.
#[cfg(all(context_switch_source = "software0", multi_core))]
#[inline(always)]
pub(crate) fn mask_local_context_switch() -> bool {
    local::mask()
}

/// Unmasks the `Software0` context switch of the current CPU. A pending context switch then runs.
#[cfg(all(context_switch_source = "software0", multi_core))]
#[inline(always)]
pub(crate) fn unmask_local_context_switch() {
    local::unmask();
}

/// The `Software0` context-switch path.
#[cfg(context_switch_source = "software0")]
mod local {
    use portable_atomic::{AtomicPtr, Ordering};

    use super::*;
    use crate::ram;

    /// `Software0` is CPU interrupt 7, which runs at the minimum priority.
    const SW_INTERRUPT: u32 = 1 << 7;

    static HANDLER: AtomicPtr<()> = AtomicPtr::new(core::ptr::null_mut());

    pub(super) unsafe fn install(handler: ContextSwitchHandler) {
        HANDLER.store(handler as *mut (), Ordering::Release);
        unsafe { xtensa_lx::interrupt::enable_mask(SW_INTERRUPT) };
    }

    #[inline]
    pub(super) fn trigger() {
        unsafe { xtensa_lx::interrupt::set(SW_INTERRUPT) };
    }

    /// The request stays pending while the interrupt is masked.
    #[cfg(multi_core)]
    #[inline(always)]
    pub(super) fn mask() -> bool {
        xtensa_lx::interrupt::disable_mask(SW_INTERRUPT) & SW_INTERRUPT != 0
    }

    #[cfg(multi_core)]
    #[inline(always)]
    pub(super) fn unmask() {
        unsafe { xtensa_lx::interrupt::enable_mask(SW_INTERRUPT) };
    }

    /// The vectored dispatcher clears the request before it calls this handler. The switch runs
    /// at the exit of the level 1 interrupt.
    #[ram]
    #[unsafe(export_name = "Software0")]
    extern "C" fn context_switch() {
        let handler = HANDLER.load(Ordering::Acquire);
        debug_assert!(!handler.is_null(), "The CPU has no context switch handler");

        super::exit::switch_at_exit(handler);
    }
}

/// The RISC-V context-switch path.
///
/// A CPU takes the context switch in a handler that this module binds directly to the
/// interrupt. The handler borrows `t0` only, and keeps the interrupted value in `mscratch`.
/// That CSR is free, because the hardware clears `mstatus.MIE` on trap entry, and nothing here
/// enables interrupts again.
#[cfg(any(context_switch_source = "from_cpu", context_switch_source = "clint"))]
mod local {
    use portable_atomic::{AtomicPtr, Ordering};

    use super::*;
    use crate::interrupt;

    cfg_select! {
        context_switch_source = "clint" => {
            use crate::{interrupt::CpuInterrupt, peripherals::CLINT};
        }
        _ => {
            use crate::{
                interrupt::software::SoftwareInterrupt,
                peripherals::{FROM_CPU_INTR0, Interrupt},
            };
        }
    }

    /// The address of the register that clears the request.
    static REQUEST: AtomicPtr<u32> = AtomicPtr::new(core::ptr::null_mut());
    static HANDLER: AtomicPtr<()> = AtomicPtr::new(core::ptr::null_mut());

    pub(super) unsafe fn install(handler: ContextSwitchHandler) {
        HANDLER.store(handler as *mut (), Ordering::Release);

        cfg_select! {
            context_switch_source = "clint" => {
                REQUEST.store(CLINT::regs().msip().as_ptr(), Ordering::Release);

                // Both controllers serve the machine software interrupt as interrupt 3. The
                // CLIC applies the priority of that line to it, and needs the line enabled.
                // The PLIC does not serve the interrupt at all: `mie` enables it, and the run
                // level masks it.
                interrupt::bind_cpu_interrupt(CpuInterrupt::Interrupt3, stub);
                #[cfg(interrupt_controller = "clic")]
                interrupt::enable_cpu_interrupt(
                    CpuInterrupt::Interrupt3,
                    interrupt::Priority::min(),
                );
            }
            _ => {
                let regs = cfg_select! {
                    soc_has_intpri => crate::peripherals::INTPRI::regs(),
                    _ => crate::peripherals::SYSTEM::regs(),
                };
                REQUEST.store(regs.cpu_intr_from_cpu(0).as_ptr(), Ordering::Release);

                interrupt::enable_direct_inner(
                    Interrupt::FROM_CPU_INTR0,
                    interrupt::Priority::min(),
                    interrupt::IPC_INTERRUPT,
                    stub,
                );
            }
        }
    }

    #[inline]
    pub(super) fn trigger() {
        cfg_select! {
            context_switch_source = "clint" => {
                CLINT::regs().msip().write(|w| w.msip().set_bit());
            }
            _ => {
                SoftwareInterrupt::new(unsafe { FROM_CPU_INTR0::steal() }).raise();
            }
        }
    }

    /// Clears the request, then jumps to the context-switch handler.
    #[cfg_attr(
        not(interrupt_controller = "clic"),
        unsafe(link_section = ".trap.rust")
    )]
    #[cfg_attr(interrupt_controller = "clic", unsafe(link_section = ".rwtext"))]
    #[unsafe(naked)]
    unsafe extern "C" fn stub() {
        core::arch::naked_asm!(
            "
            csrw mscratch, t0

            # Clear the request before the switch, so that a request raised for the next
            # switch survives.
            la   t0, {request}
            lw   t0, 0(t0)
            sw   zero, 0(t0)

            # The handler restores t0 from mscratch and returns with mret.
            la   t0, {handler}
            lw   t0, 0(t0)
            jr   t0
            ",
            request = sym REQUEST,
            handler = sym HANDLER,
        )
    }
}

/// The context switch at the exit of the level 1 interrupt.
///
/// The request interrupts run in the level 1 dispatcher, on the stack of the interrupted task.
/// They only record the handler. xtensa-lx-rt calls `__level_1_interrupt_exit` after all level 1
/// handlers, and that function runs the handler on the switch stack of the CPU. Once the RTOS
/// releases its lock, the other CPU can take the interrupted task and run it on its stack, so this
/// CPU must not use that stack any more.
#[cfg(xtensa)]
pub(crate) mod exit {
    use portable_atomic::{AtomicPtr, Ordering};

    use super::*;

    /// The space for the next context at the top of a switch stack. The restore code of
    /// xtensa-lx-rt reads a context at the offsets of its 256-byte frame.
    pub(super) const NEXT_CONTEXT_SIZE: usize = 256;

    const _: () = core::assert!(core::mem::size_of::<TrapFrame>() <= NEXT_CONTEXT_SIZE);

    /// The context switch state of one CPU. The assembly below reads the fields at fixed offsets.
    #[repr(C)]
    struct ExitSwitch {
        /// The handler to run at the exit of the current level 1 interrupt, or null.
        handler: AtomicPtr<()>,
        /// The top of the switch stack.
        stack_top: AtomicPtr<u8>,
    }

    static EXIT_SWITCH: [ExitSwitch; Cpu::COUNT] = [const {
        ExitSwitch {
            handler: AtomicPtr::new(core::ptr::null_mut()),
            stack_top: AtomicPtr::new(core::ptr::null_mut()),
        }
    }; Cpu::COUNT];

    pub(super) fn set_stack(core: Cpu, top: *mut u8) {
        EXIT_SWITCH[core as usize]
            .stack_top
            .store(top, Ordering::Relaxed);
    }

    pub(super) fn has_stack(core: Cpu) -> bool {
        !EXIT_SWITCH[core as usize]
            .stack_top
            .load(Ordering::Relaxed)
            .is_null()
    }

    /// Runs `handler` at the exit of the current level 1 interrupt of the current CPU.
    #[inline(always)]
    pub(crate) fn switch_at_exit(handler: *mut ()) {
        EXIT_SWITCH[Cpu::current() as usize]
            .handler
            .store(handler, Ordering::Relaxed);
    }

    cfg_select! {
        multi_core => {
            /// Points `a2` to the `ExitSwitch` of the current CPU.
            macro_rules! index_by_cpu {
                () => {
                    "
                    rsr.prid a3
                    extui   a3, a3, 13, 1
                    addx8   a2, a3, a2
                    "
                };
            }
        }
        _ => {
            macro_rules! index_by_cpu {
                () => {
                    ""
                };
            }
        }
    }

    // See the `exception` module of xtensa-lx-rt for the contract of this function.
    core::arch::global_asm!(
        concat!(
            "
        .section .rwtext,\"ax\",@progbits
        .literal sym_exit_switch, {exit_switch}

        .global __level_1_interrupt_exit
        .p2align 2
        .type __level_1_interrupt_exit,@function
    __level_1_interrupt_exit:
        l32r    a2, sym_exit_switch
            ",
            index_by_cpu!(),
            "
        l32i    a8, a2, {handler}
        bnez    a8, 1f
        ret

    1:
        movi    a4, 0
        s32i    a4, a2, {handler}
        // Interrupts of levels 2 and 3 stay masked until the restore of the next context.
        rsil    a4, 3

        // The next context goes to the top of the switch stack, and the handler runs below it.
        l32i    a4, a2, {stack_top}
        addmi   a4, a4, -{next_context_size}
        mov     a6, a1
        mov     a7, a4
        mov     a1, a4
        callx4  a8

        mov     a1, a6
        ret
        .size __level_1_interrupt_exit, . - __level_1_interrupt_exit
            "
        ),
        exit_switch = sym EXIT_SWITCH,
        handler = const core::mem::offset_of!(ExitSwitch, handler),
        stack_top = const core::mem::offset_of!(ExitSwitch, stack_top),
        next_context_size = const NEXT_CONTEXT_SIZE,
    );
}
