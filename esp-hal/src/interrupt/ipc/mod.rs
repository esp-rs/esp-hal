#![cfg_attr(docsrs, procmacros::doc_replace)]
//! # Inter-Processor Call (IPC)
//!
//! ## Overview
//!
//! [`Ipc::post`] queues a run of an [`IpcRequest`] on a CPU, then raises that CPU's IPC
//! interrupt. The interrupt handler calls the function of each queued request at
//! [`Priority::min`][super::Priority::min], in post order.
//!
//! A post never waits. It returns a [`Ticket`], and [`Ticket::wait`] waits for the run with a
//! deadline. A busy CPU does not block the poster.
//!
//! A request has at most one pending run. A post of a pending request with the same argument
//! joins that run. Two concurrent runs need two requests.
//!
//! The same interrupt also carries a context switch. A CPU asks the other CPU to switch tasks
//! through this interrupt. The context-switch handler of the RTOS that owns scheduling runs
//! after the posted requests, and chooses the context that the interrupt returns to. A posted
//! request and a context switch do not wait for each other. A posted request alone does not
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
//! [`Ipc::new`] creates a handle from the `IPC` peripheral singleton, and [`Ipc::post`] posts an
//! [`IpcRequest`] to a CPU.
//!
//! The function of a request runs in an interrupt handler. The handler saves the registers that
//! the calling convention lets the callee clobber, except the floating-point registers.
//! [`IpcRequest`] lists the rules for the function.
//!
//! ## Examples
//!
//! ### Run a Function on the PRO CPU
//!
//! ```rust, no_run
//! # {before_snippet}
//! use esp_hal::{
//!     interrupt::ipc::{Ipc, IpcRequest},
//!     system::Cpu,
//! };
//!
//! extern "C" fn on_cpu(arg: usize) {
//!     // Runs on the selected CPU at minimum interrupt priority.
//! }
//!
//! static ON_CPU: IpcRequest = IpcRequest::new(on_cpu);
//!
//! let ipc = Ipc::new(peripherals.IPC);
//! let ticket = ipc.post(Cpu::ProCpu, &ON_CPU, 42)?;
//! ticket.wait(Instant::now() + Duration::from_millis(10))?;
//! # {after_snippet}
//! ```

#[cfg(feature = "rt")]
use portable_atomic::{AtomicPtr, Ordering};

pub use self::queue::IpcRequest;
#[cfg(feature = "rt")]
use crate::interrupt::__rtos_implementation::ContextSwitchHandler;
use crate::{peripherals::IPC, system::Cpu, time::Instant};

#[cfg(feature = "rt")]
#[cfg_attr(riscv, path = "riscv.rs")]
#[cfg_attr(xtensa, path = "xtensa.rs")]
mod implem;
#[allow(dead_code)]
mod queue;

#[cfg(feature = "rt")]
pub(crate) use implem::install;
#[cfg(feature = "rt")]
use implem::raise;

#[cfg(all(feature = "rt", riscv))]
pub(crate) use self::implem::install_app;

/// The IPC state of one CPU.
///
/// Every CPU has its own handler, which reads the fields at constant addresses.
#[cfg(feature = "rt")]
#[repr(C)]
struct CoreState {
    /// The context-switch handler that the CPU runs, or null if the CPU has none pending.
    ///
    /// [`request_context_switch`] copies `context_switch_handler` here. A posted request alone
    /// therefore does not make the CPU switch tasks.
    pending_context_switch: AtomicPtr<()>,

    /// The context-switch handler of the CPU, or null if no RTOS registered one.
    ///
    /// This field is set once, and holds the only handler that the CPU accepts.
    context_switch_handler: AtomicPtr<()>,

    /// The address of the register that clears the request of this CPU.
    #[cfg(riscv)]
    request: AtomicPtr<u32>,

    /// The requests that [`Ipc::post`] queued for this CPU.
    lists: queue::Lists,
}

#[cfg(feature = "rt")]
impl CoreState {
    const fn new() -> Self {
        Self {
            pending_context_switch: AtomicPtr::new(core::ptr::null_mut()),
            context_switch_handler: AtomicPtr::new(core::ptr::null_mut()),
            #[cfg(riscv)]
            request: AtomicPtr::new(core::ptr::null_mut()),
            lists: queue::Lists::new(),
        }
    }
}

#[cfg(feature = "rt")]
static STATE: [CoreState; Cpu::COUNT] = [const { CoreState::new() }; Cpu::COUNT];

/// Completes each request that `core` did not run with [`IpcError::Offline`], and empties the
/// lists of `core`.
///
/// # Safety
///
/// `core` does not run, and posts to `core` return [`IpcError::Offline`].
#[cfg(feature = "rt")]
pub(crate) unsafe fn fail_stranded(core: Cpu) {
    unsafe { queue::fail_stranded(&STATE[core as usize].lists, core as u32) };
}

/// Queues a run of `request` on the CPU `core`. Returns the encoding of [`esp_hal_ipc_post`].
#[cfg(feature = "rt")]
fn post_in_runtime(core: u8, request: &IpcRequest, arg: usize) -> u32 {
    use esp_sync::raw::{RawLock, SingleCoreInterruptLock};

    let core = match core {
        0 => Cpu::ProCpu,
        1 => Cpu::AppCpu,
        _ => return queue::POST_OFFLINE,
    };

    // A context switch must not stop the post between the push and the raise.
    let token = unsafe { SingleCoreInterruptLock.enter() };
    let result = if core == Cpu::AppCpu && !crate::system::multi_core::app_core_takes_ipc() {
        queue::POST_OFFLINE
    } else {
        unsafe {
            queue::post(
                &STATE[core as usize].lists,
                core as u32,
                request,
                arg,
                || raise(core),
            )
        }
    };
    unsafe { SingleCoreInterruptLock.exit(token) };

    result
}

/// Queues a run of `request` on the CPU `core`.
///
/// Returns `seq << 5` for a queued or joined run, 1 if `core` does not take IPC interrupts, and 2
/// if `request` is busy.
// esp-hal copies without the `rt` feature call this function by name. The name, the argument
// types, the encoding of the return value and the layout of `IpcRequest` must stay stable.
#[cfg(feature = "rt")]
#[unsafe(export_name = "esp_hal_ipc_post")]
extern "C" fn esp_hal_ipc_post(core: u8, request: *const IpcRequest, arg: usize) -> u32 {
    post_in_runtime(core, unsafe { &*request }, arg)
}

// A build can hold esp-hal copies without the `rt` feature. Those copies post through the symbol
// of the copy with `rt`.
#[cfg(not(feature = "rt"))]
unsafe extern "C" {
    fn esp_hal_ipc_post(core: u8, request: *const IpcRequest, arg: usize) -> u32;
}

/// Handle that posts requests to a CPU through the IPC interrupt.
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

    /// Queues a run of `request` on `core` with the argument `arg`, and returns a [`Ticket`] that
    /// can be used to wait for the run.
    ///
    /// This function never waits. If `request` is pending on `core` with the same `arg`, the post
    /// joins that run. If `request` runs on `core`, the post queues a new run, which starts after
    /// the current run returns.
    ///
    /// # Errors
    ///
    /// - [`IpcError::Offline`] when `core` does not take IPC interrupts.
    /// - [`IpcError::Busy`] when `request` is pending with a different argument, when another CPU
    ///   posts it at the same moment, or when it runs on the other CPU.
    pub fn post(
        self,
        core: Cpu,
        request: &'static IpcRequest,
        arg: usize,
    ) -> Result<Ticket, IpcError> {
        let result = cfg_select! {
            feature = "rt" => post_in_runtime(core as u8, request, arg),
            _ => unsafe { esp_hal_ipc_post(core as u8, request, arg) },
        };
        match result & queue::POST_RESULT_MASK {
            queue::POST_OK => Ok(Ticket {
                request,
                core,
                seq: result,
            }),
            queue::POST_OFFLINE => Err(IpcError::Offline),
            _ => Err(IpcError::Busy),
        }
    }
}

/// A run that [`Ipc::post`] queued or joined.
///
/// If a later run of the request ended, the ticket gives the outcome of that later run.
#[derive(Clone, Copy, Debug)]
pub struct Ticket {
    request: &'static IpcRequest,
    core: Cpu,
    /// The sequence number of the run, in the `seq` bits.
    seq: u32,
}

impl Ticket {
    /// Waits until the run of this ticket ends, or until `deadline`.
    ///
    /// # Errors
    ///
    /// - [`IpcError::Offline`] when the CPU stopped before the run.
    /// - [`IpcError::Timeout`] when the deadline passed before the run ended. The run stays queued,
    ///   and can happen later.
    /// - [`IpcError::WouldDeadlock`] when the run is on the current CPU, and the caller masks the
    ///   IPC interrupt: it runs in an interrupt handler, or with interrupts disabled.
    pub fn wait(self, deadline: Instant) -> Result<(), IpcError> {
        // A masked context cannot move to the other CPU, so the check has no race.
        if self.core == Cpu::current() && ipc_interrupt_masked() {
            return Err(IpcError::WouldDeadlock);
        }

        loop {
            if let Some(outcome) = self.poll() {
                return outcome;
            }
            if Instant::now() >= deadline {
                return Err(IpcError::Timeout);
            }
            core::hint::spin_loop();
        }
    }

    /// Returns the outcome of the run of this ticket, or `None` if the run did not end.
    ///
    /// This function never waits.
    pub fn poll(self) -> Option<Result<(), IpcError>> {
        queue::poll(self.request, self.seq).map(|outcome| match outcome {
            queue::DONE => Ok(()),
            _ => Err(IpcError::Offline),
        })
    }
}

/// Returns whether the IPC interrupt cannot preempt the current code.
fn ipc_interrupt_masked() -> bool {
    if super::current_raw_runlevel() != 0 {
        return true;
    }
    cfg_select! {
        riscv => !riscv::register::mstatus::read().mie(),
        // `PS.INTLEVEL` is the run level.
        xtensa => false,
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for Ticket {
    fn format(&self, f: defmt::Formatter<'_>) {
        defmt::write!(
            f,
            "Ticket {{ core: {}, seq: {} }}",
            self.core,
            self.seq >> queue::SEQ_SHIFT
        )
    }
}

/// An error of an IPC post or wait.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum IpcError {
    /// The CPU does not take IPC interrupts.
    ///
    /// The APP core does not take IPC interrupts before [`CpuControl`] starts it. A run that was
    /// queued on the APP core when [`CpuControl`] starts the APP core again also ends with this
    /// error. A stop through [`AppCoreGuard`] or the return of the APP closure does not give this
    /// error: a post to the stopped APP core queues a run, and its wait returns
    /// [`IpcError::Timeout`].
    ///
    /// [`CpuControl`]: crate::system::CpuControl
    /// [`AppCoreGuard`]: crate::system::AppCoreGuard
    Offline,
    /// The request is pending with a different argument, another CPU posts it at this moment, or
    /// it runs on the other CPU.
    Busy,
    /// The deadline passed before the run ended.
    Timeout,
    /// The caller waits for a run on its own CPU, and masks the IPC interrupt.
    WouldDeadlock,
}

impl core::fmt::Display for IpcError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            IpcError::Offline => write!(f, "The CPU does not take IPC interrupts"),
            IpcError::Busy => write!(f, "The request is busy"),
            IpcError::Timeout => write!(f, "The deadline passed before the run ended"),
            IpcError::WouldDeadlock => write!(f, "The wait cannot end"),
        }
    }
}

impl core::error::Error for IpcError {}

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
#[cfg(feature = "rt")]
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
/// The handler runs at [`Priority::min`][super::Priority::min], after the requests that
/// [`Ipc::post`] queued. The call does not wait, not even for a pending request.
///
/// # Panics
///
/// In debug builds, panics if `core` has no context-switch handler, or if `core` is the APP CPU
/// and that CPU is not running.
#[cfg(feature = "rt")]
#[inline]
pub(crate) fn request_context_switch(core: Cpu) {
    debug_assert!(
        core != Cpu::AppCpu || crate::soc::cpu_control::is_running(Cpu::AppCpu),
        "IPC target APP core is not running"
    );

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
