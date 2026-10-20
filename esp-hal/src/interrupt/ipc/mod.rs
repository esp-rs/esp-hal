#![cfg_attr(docsrs, procmacros::doc_replace)]
//! # Inter-Processor Call (IPC)
//!
//! ## Overview
//!
//! [`Ipc`] posts a function to a CPU, then raises that CPU's IPC interrupt. The interrupt
//! handler runs the posted function at [`Priority::min`][super::Priority::min].
//!
//! The same interrupt also carries a context switch. A CPU asks the other CPU to switch tasks
//! through this interrupt. The context-switch handler of the RTOS that owns scheduling runs
//! after the posted function, and chooses the context that the interrupt returns to. A posted
//! function and a context switch do not wait for each other. A posted function alone does not
//! make a CPU switch tasks.
//!
//! ## Configuration
#![cfg_attr(
    xtensa,
    doc = " The HAL reserves `FROM_CPU_INTR0` and `FROM_CPU_INTR1` for this path."
)]
#![cfg_attr(
    riscv,
    doc = " Each CPU takes the call in its machine software interrupt. The path needs no peripheral interrupt."
)]
//! [`crate::init`] installs the handler on the PRO CPU. The start of the APP CPU installs the
//! handler on that CPU.
//!
//! ## Usage
//!
//! [`Ipc::new`] creates a handle from the `IPC` peripheral singleton, and
//! [`Ipc::call_function`] posts a function to a CPU.
//!
//! The posted function runs in an interrupt handler. The handler saves the registers that the
//! calling convention lets the callee clobber, except the floating-point registers. The posted
//! function must not use those registers, and must be callable with the cache disabled.
//!
//! ## Examples
//!
//! ### Run a Function on the PRO CPU
//!
//! ```rust, no_run
//! # {before_snippet}
//! use esp_hal::{interrupt::ipc::Ipc, system::Cpu};
//!
//! fn on_cpu() {
//!     // Runs on the selected CPU at minimum interrupt priority.
//! }
//!
//! let ipc = Ipc::new(peripherals.IPC);
//! ipc.call_function(Cpu::ProCpu, on_cpu);
//! # {after_snippet}
//! ```

use portable_atomic::{AtomicPtr, Ordering};

use crate::{
    interrupt::__rtos_implementation::ContextSwitchHandler,
    peripherals::IPC,
    system::Cpu,
};

#[cfg_attr(riscv, path = "riscv.rs")]
#[cfg_attr(xtensa, path = "xtensa.rs")]
mod implem;

pub(crate) use implem::install;
use implem::raise;

#[cfg(riscv)]
pub(crate) use self::implem::install_app;

/// The IPC state of one CPU.
///
/// Every CPU has its own handler, which reads the fields at constant addresses.
#[repr(C)]
struct CoreState {
    /// The function that [`Ipc::call_function`] posts, or null if the CPU has none.
    callback: AtomicPtr<()>,

    /// The context-switch handler that the CPU runs, or null if the CPU has none pending.
    ///
    /// [`request_context_switch`] copies `context_switch_handler` here. A posted function alone
    /// therefore does not make the CPU switch tasks.
    pending_context_switch: AtomicPtr<()>,

    /// The context-switch handler of the CPU, or null if no RTOS registered one.
    ///
    /// This field is set once, and holds the only handler that the CPU accepts.
    context_switch_handler: AtomicPtr<()>,

    /// The address of the register that clears the request of this CPU.
    #[cfg(riscv)]
    request: AtomicPtr<u32>,
}

impl CoreState {
    const fn new() -> Self {
        Self {
            callback: AtomicPtr::new(core::ptr::null_mut()),
            pending_context_switch: AtomicPtr::new(core::ptr::null_mut()),
            context_switch_handler: AtomicPtr::new(core::ptr::null_mut()),
            #[cfg(riscv)]
            request: AtomicPtr::new(core::ptr::null_mut()),
        }
    }
}

static STATE: [CoreState; Cpu::COUNT] = [const { CoreState::new() }; Cpu::COUNT];

/// Handle that posts a function to a CPU through the IPC interrupt.
///
/// Copies of this handle share one state table.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct Ipc;

impl Ipc {
    /// Creates a new [`Ipc`] handle.
    ///
    /// The constructor does not install an interrupt handler. [`crate::init`] installs the IPC
    /// path.
    pub const fn new(_ipc: IPC<'_>) -> Self {
        Self
    }

    /// Runs `handler` on `core`.
    ///
    /// `handler` runs in an interrupt handler at [`Priority::min`][super::Priority::min]. This
    /// call does not wait for `handler` to return, and a context switch does not delay it.
    ///
    /// The interrupt handler saves the registers that the calling convention lets the callee
    /// clobber, except the floating-point registers. `handler` must not use those registers, and
    /// must be callable with the cache disabled.
    ///
    /// A CPU holds one posted function at a time. If `core` already holds one, this call behaves
    /// as follows:
    ///
    /// - If `core` holds `handler`, that function has not run yet, and it runs after this call. The
    ///   two calls coalesce into one run, and this call returns. A caller that needs a run of its
    ///   own must wait for the pending run.
    /// - If `core` holds a different function, this call waits until `core` takes that function.
    ///   `core` takes it in a minimum-priority interrupt. That interrupt cannot run in an
    ///   interrupt-free context, and cannot run on the APP CPU while that CPU is in reset. The wait
    ///   never ends in those cases.
    ///
    /// # Panics
    ///
    /// In debug builds, panics if `core` is the APP CPU and that CPU is not running.
    pub fn call_function(self, core: Cpu, handler: fn()) {
        let handler = handler as *mut ();
        assert_target_runs(core);

        while let Err(pending) = STATE[core as usize].callback.compare_exchange(
            core::ptr::null_mut(),
            handler,
            Ordering::AcqRel,
            Ordering::Acquire,
        ) {
            // The IPC handler takes the callback before it runs it, so a pending copy of `handler`
            // has not run yet. It runs after this call, which covers this call, too.
            if pending == handler {
                break;
            }

            core::hint::spin_loop();
        }

        raise(core);
    }
}

/// Registers the context-switch handler of `core`.
///
/// A CPU accepts one context-switch handler. [`request_context_switch`] therefore takes no
/// handler of its own, and never waits.
///
/// # Panics
///
/// Panics if `core` already has a context-switch handler.
///
/// # Safety
///
/// See [`__rtos_implementation::set_context_switch_handler`].
///
/// [`__rtos_implementation::set_context_switch_handler`]:
///     super::__rtos_implementation::set_context_switch_handler
pub(crate) unsafe fn set_context_switch_handler(core: Cpu, handler: ContextSwitchHandler) {
    unwrap!(
        STATE[core as usize]
            .context_switch_handler
            .compare_exchange(
                core::ptr::null_mut(),
                handler as *mut (),
                Ordering::Release,
                Ordering::Relaxed,
            ),
        "A CPU accepts one context switch handler"
    );
}

/// Asks `core` to run its context-switch handler.
///
/// The handler runs at [`Priority::min`][super::Priority::min], after the function that
/// [`Ipc::call_function`] posts. The call does not wait, not even for a pending posted function.
///
/// # Panics
///
/// In debug builds, panics if `core` has no context-switch handler, or if `core` is the APP CPU
/// and that CPU is not running.
#[inline]
pub(crate) fn request_context_switch(core: Cpu) {
    assert_target_runs(core);

    let state = &STATE[core as usize];

    // A CPU accepts one context switch handler, so this needs no read-modify-write: the store
    // is idempotent, and the IPC handler takes the handler before it runs it, which makes the
    // handler run at least once after this store.
    let handler = state.context_switch_handler.load(Ordering::Relaxed);
    debug_assert!(!handler.is_null(), "The CPU has no context switch handler");
    state
        .pending_context_switch
        .store(handler, Ordering::Release);

    raise(core);
}

fn assert_target_runs(core: Cpu) {
    debug_assert!(
        core != Cpu::AppCpu || crate::soc::cpu_control::is_running(Cpu::AppCpu),
        "IPC target APP core is not running"
    );
}
