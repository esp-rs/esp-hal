//! Private implementation details of the RTOS that owns context switching.
//!
//! A context switch runs at [`Priority::min`][super::Priority::min]. It is the last action of a
//! CPU before the CPU returns to the interrupted context. An RTOS registers its context-switch
//! handler here, then asks a CPU to run the handler.
//!
//! These functions are not part of the application API.
#![cfg_attr(
    context_switch_source = "software0",
    doc = "\nA CPU switches its own tasks through the `Software0` CPU-internal interrupt. The HAL
    defines the handler of that interrupt."
)]
#![cfg_attr(
    context_switch_source = "clint",
    doc = "\nA CPU switches its own tasks through the machine software interrupt of the CLINT."
)]
#![cfg_attr(
    context_switch_source = "from_cpu",
    doc = "\nA CPU switches its own tasks through the `FROM_CPU_INTR0` interrupt. The HAL reserves
    that interrupt."
)]
#![cfg_attr(
    multi_core,
    doc = "\nA CPU switches the tasks of the other CPU through [`Ipc`][super::ipc::Ipc]."
)]

use crate::system::Cpu;
#[cfg(xtensa)]
use crate::trapframe::TrapFrame;

/// The signature of a context-switch handler.
///
/// On RISC-V, the handler runs as a direct-bound interrupt handler, and returns from the
/// interrupt itself. On Xtensa, the vectored dispatcher calls the handler with the trap frame,
/// and the interrupt returns to the context in that trap frame.
pub type ContextSwitchHandler = cfg_select! {
    riscv => {
        unsafe extern "C" fn()
    }
    xtensa => {
        extern "C" fn(&mut TrapFrame)
    }
};

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
///
/// On RISC-V, the interrupt jumps to `handler` with the interrupted `t0` in `mscratch`.
/// `handler` must restore `t0` from that CSR, save and restore every other register it
/// modifies, and return with `mret`.
///
/// On Xtensa, the interrupt returns to the context in the trap frame. `handler` must leave a
/// context that the CPU can run.
pub unsafe fn set_context_switch_handler(core: Cpu, handler: ContextSwitchHandler) {
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
    doc = "\nIn debug builds, panics if `core` is the APP CPU and that CPU is not running."
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

    /// The vectored dispatcher clears the request before it calls this handler.
    #[ram]
    #[unsafe(export_name = "Software0")]
    extern "C" fn context_switch(context: &mut TrapFrame) {
        let handler = HANDLER.load(Ordering::Acquire);
        debug_assert!(!handler.is_null(), "The CPU has no context switch handler");

        let handler = unsafe { core::mem::transmute::<*mut (), ContextSwitchHandler>(handler) };
        handler(context);
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
