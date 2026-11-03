//! Private implementation details of the RTOS that owns context switching.
//!
//! A CPU switches tasks when it leaves the outermost interrupt handler of
//! [`Priority::min`][super::Priority::min], if a context switch is pending on it. The CPU runs on
//! its interrupt stack, with interrupts masked, and calls the context-switch handler of the RTOS.
//! The handler chooses the context that the CPU resumes. An RTOS registers its context-switch
//! handler here, then asks a CPU to switch tasks.
//!
//! These functions are not part of the application API.
#![cfg_attr(
    context_switch_source = "software0",
    doc = "\nA CPU asks itself to switch tasks through the `Software0` CPU-internal interrupt. The HAL
    defines the handler of that interrupt."
)]
#![cfg_attr(
    context_switch_source = "clint",
    doc = "\nA CPU asks itself to switch tasks through the machine software interrupt of the CLINT."
)]
#![cfg_attr(
    context_switch_source = "from_cpu",
    doc = "\nA CPU asks itself to switch tasks through the `FROM_CPU_INTR0` interrupt. The HAL
    reserves that interrupt."
)]
#![cfg_attr(
    multi_core,
    doc = "\nA CPU asks the other CPU to switch tasks through [`Ipc`][super::ipc::Ipc]."
)]

use portable_atomic::{AtomicBool, AtomicPtr, Ordering};

use crate::{ram, system::Cpu};

/// The saved context of an interrupted task, on the stack of that task.
///
/// The interrupted stack pointer is the address of the frame plus [`TASK_FRAME_SIZE`].
#[cfg(riscv)]
#[derive(Debug, Default, Clone)]
#[repr(C)]
#[allow(missing_docs)]
pub struct TaskFrame {
    pub s0: usize,
    pub s1: usize,
    pub s2: usize,
    pub s3: usize,
    pub s4: usize,
    pub s5: usize,
    pub s6: usize,
    pub s7: usize,
    pub s8: usize,
    pub s9: usize,
    pub s10: usize,
    pub s11: usize,
    pub tp: usize,
    /// The address that the task resumes at.
    pub pc: usize,
    _reserved: [usize; 2],

    // The part that the interrupt entry of esp-riscv-rt saves.
    pub ra: usize,
    pub t0: usize,
    pub t1: usize,
    pub t2: usize,
    pub t3: usize,
    pub t4: usize,
    pub t5: usize,
    pub t6: usize,
    pub a0: usize,
    pub a1: usize,
    pub a2: usize,
    pub a3: usize,
    pub a4: usize,
    pub a5: usize,
    pub a6: usize,
    pub a7: usize,
}

/// The saved context of an interrupted task, on the stack of that task.
///
/// `A1` holds the interrupted stack pointer.
#[cfg(xtensa)]
pub use crate::trapframe::TrapFrame as TaskFrame;

/// The number of bytes that the interrupt entry reserves on the interrupted stack for a
/// [`TaskFrame`].
///
/// The frame starts at the interrupted stack pointer minus this value.
pub const TASK_FRAME_SIZE: usize = super::isr_stack::TASK_FRAME_SIZE;

/// The signature of a context-switch handler.
///
/// `saved` points to the complete frame of the interrupted context. The handler returns the frame
/// of the context to resume, which can be `saved`.
pub type ContextSwitchHandler = extern "C" fn(saved: *mut TaskFrame) -> *mut TaskFrame;

static HANDLERS: [AtomicPtr<()>; Cpu::COUNT] =
    [const { AtomicPtr::new(core::ptr::null_mut()) }; Cpu::COUNT];

/// Set while a context switch is pending on the CPU. The interrupt exit code reads it.
pub(super) static SWITCH_PENDING: [AtomicBool; Cpu::COUNT] =
    [const { AtomicBool::new(false) }; Cpu::COUNT];

/// Registers the context-switch handler of `core`.
///
/// A CPU accepts one context-switch handler.
///
/// # Panics
///
/// Panics if `core` already has a context-switch handler.
#[cfg_attr(
    not(context_switch_source = "ipc"),
    doc = "\nIn debug builds, panics if `core` is not the current CPU. A CPU-local interrupt
    carries the request, and only that CPU can enable the interrupt.\n"
)]
/// # Safety
///
/// The handler runs on the interrupt stack, with interrupts masked, and must not enable them. It
/// must return a frame that no other CPU resumes. After the handler returns, the CPU reads only
/// that frame and its interrupt stack.
pub unsafe fn set_context_switch_handler(core: Cpu, handler: ContextSwitchHandler) {
    unwrap!(
        HANDLERS[core as usize].compare_exchange(
            core::ptr::null_mut(),
            handler as *mut (),
            Ordering::Release,
            Ordering::Relaxed,
        ),
        "A CPU accepts one context switch handler"
    );

    #[cfg(not(context_switch_source = "ipc"))]
    {
        debug_assert_eq!(core, Cpu::current(), "A CPU installs its own handler");
        local::install();
    }
}

/// Asks `core` to switch tasks.
///
/// The CPU switches when it leaves its next interrupt handler of
/// [`Priority::min`][super::Priority::min]. The call does not wait.
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
    debug_assert!(
        !HANDLERS[core as usize].load(Ordering::Relaxed).is_null(),
        "The CPU has no context switch handler"
    );

    // The exit code takes the request after the interrupt that this call raises.
    SWITCH_PENDING[core as usize].store(true, Ordering::Release);

    cfg_select! {
        context_switch_source = "ipc" => {
            super::ipc::raise(core);
        }
        multi_core => {
            if core == Cpu::current() {
                local::trigger();
            } else {
                super::ipc::raise(core);
            }
        }
        _ => {
            debug_assert_eq!(core, Cpu::current(), "A CPU switches its own tasks");
            local::trigger();
        }
    }
}

/// Runs the context-switch handler of the current CPU, if a switch is pending.
///
/// The interrupt exit code calls this function on the interrupt stack, with interrupts masked,
/// when it leaves the outermost handler of [`Priority::min`][super::Priority::min]. `saved` is the
/// complete frame of the interrupted context. Returns the frame to resume.
#[ram]
pub(super) extern "C" fn switch_context(saved: *mut TaskFrame) -> *mut TaskFrame {
    let core = Cpu::current() as usize;
    if !SWITCH_PENDING[core].swap(false, Ordering::Acquire) {
        return saved;
    }

    let handler = HANDLERS[core].load(Ordering::Acquire);
    if handler.is_null() {
        return saved;
    }

    let handler = unsafe { core::mem::transmute::<*mut (), ContextSwitchHandler>(handler) };
    handler(saved)
}

/// The `Software0` request path.
#[cfg(context_switch_source = "software0")]
mod local {
    use crate::{ram, trapframe::TrapFrame};

    /// `Software0` is CPU interrupt 7, which runs at the minimum priority.
    const SW_INTERRUPT: u32 = 1 << 7;

    pub(super) fn install() {
        unsafe { xtensa_lx::interrupt::enable_mask(SW_INTERRUPT) };
    }

    #[inline]
    pub(super) fn trigger() {
        unsafe { xtensa_lx::interrupt::set(SW_INTERRUPT) };
    }

    /// The vectored dispatcher clears the request before it calls this handler. The switch runs
    /// when the interrupt exits.
    #[ram]
    #[unsafe(export_name = "Software0")]
    extern "C" fn context_switch_request(_context: &mut TrapFrame) {}
}

/// The machine software interrupt request path of the CLINT.
#[cfg(context_switch_source = "clint")]
mod local {
    use crate::{
        interrupt::{self, CpuInterrupt},
        peripherals::CLINT,
    };

    pub(super) fn install() {
        // Both controllers serve the machine software interrupt as interrupt 3. The CLIC applies
        // the priority of that line to it, and needs the line enabled. The PLIC does not serve
        // the interrupt at all: `mie` enables it, and the run level masks it.
        interrupt::bind_cpu_interrupt(
            CpuInterrupt::Interrupt3,
            interrupt::isr_stack::machine_software_interrupt_entry,
        );
        #[cfg(interrupt_controller = "clic")]
        interrupt::enable_cpu_interrupt(CpuInterrupt::Interrupt3, interrupt::Priority::min());
    }

    #[inline]
    pub(super) fn trigger() {
        CLINT::regs().msip().write(|w| w.msip().set_bit());
    }

    /// Clears the request. The switch runs when the interrupt exits.
    #[crate::ram]
    pub(crate) extern "C" fn machine_software_interrupt() {
        CLINT::regs().msip().write(|w| w.msip().clear_bit());
    }
}

#[cfg(context_switch_source = "clint")]
pub(super) use local::machine_software_interrupt;

/// The `FROM_CPU_INTR0` request path.
#[cfg(context_switch_source = "from_cpu")]
mod local {
    use crate::{
        interrupt::{self, InterruptHandler, Priority, software::SoftwareInterrupt},
        peripherals::{FROM_CPU_INTR0, Interrupt},
    };

    pub(super) fn install() {
        interrupt::bind_handler(
            Interrupt::FROM_CPU_INTR0,
            InterruptHandler::new(context_switch_request, Priority::min()),
        );
    }

    #[inline]
    pub(super) fn trigger() {
        SoftwareInterrupt::new(unsafe { FROM_CPU_INTR0::steal() }).raise();
    }

    /// Clears the request. The switch runs when the interrupt exits.
    #[crate::ram]
    extern "C" fn context_switch_request() {
        SoftwareInterrupt::new(unsafe { FROM_CPU_INTR0::steal() }).reset();
    }
}
