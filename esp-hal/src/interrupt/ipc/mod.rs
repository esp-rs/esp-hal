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
//! A request has at most one pending run. A post of a pending request joins that run.
//! [`IpcRequest`] lists the rules for the function.
//!
//! ## Configuration
#![doc = cfg_select! {
    xtensa => "The HAL reserves `FROM_CPU_INTR0` and `FROM_CPU_INTR1` for this path.",
    riscv => "Each CPU takes the call in its machine software interrupt. The path needs no peripheral interrupt.",
}]
//! [`crate::init`] installs the handler on the PRO CPU. The start of the APP CPU installs the
//! handler on that CPU.
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
//! extern "C" fn on_cpu() {
//!     // Runs on the selected CPU at minimum interrupt priority.
//! }
//!
//! static ON_CPU: IpcRequest = IpcRequest::new(on_cpu);
//!
//! let ipc = Ipc::new(peripherals.IPC);
//! let ticket = ipc.post(Cpu::ProCpu, &ON_CPU)?;
//! ticket.wait(Instant::now() + Duration::from_millis(10))?;
//! # {after_snippet}
//! ```

use core::ptr::NonNull;

pub use self::request::IpcRequest;
use crate::{peripherals::IPC, system::Cpu, time::Instant};

mod request;

cfg_select! {
    feature = "rt" => {
        #[cfg_attr(riscv, path = "riscv.rs")]
        #[cfg_attr(xtensa, path = "xtensa.rs")]
        mod implem;
        mod queue;
        mod runtime;

        pub(crate) use implem::install;
        #[cfg(riscv)]
        pub(crate) use implem::install_app;
        use runtime::esp_hal_ipc_post;
        pub(crate) use runtime::{
            defer_context_switch,
            fail_stranded,
            request_context_switch,
            reset_context_switch_deferral,
            resume_context_switch,
            set_context_switch_handler,
        };
    }
    _ => {
        // A build can hold esp-hal copies without the `rt` feature. Those copies post through the
        // symbol of the copy with `rt`.
        unsafe extern "C" {
            fn esp_hal_ipc_post(core: u8, request: NonNull<IpcRequest>) -> u32;
        }
    }
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

    /// Queues a run of `request` on `core`, and returns a [`Ticket`] that can be used to wait for
    /// the run.
    ///
    /// This function never waits. If `request` is pending on `core`, the post joins that run. If
    /// `request` runs on `core`, the post queues a new run, which starts after the current run
    /// returns.
    ///
    /// # Errors
    ///
    /// - [`IpcError::Offline`] when `core` does not take IPC interrupts.
    /// - [`IpcError::Busy`] when `request` is pending on a CPU other than `core`, or runs on it.
    pub fn post(self, core: Cpu, request: &'static IpcRequest) -> Result<Ticket, IpcError> {
        let result = unsafe { esp_hal_ipc_post(core as u8, NonNull::from(request)) };
        match result & request::POST_RESULT_MASK {
            request::POST_OK => Ok(Ticket {
                request,
                core,
                seq: result,
            }),
            request::POST_OFFLINE => Err(IpcError::Offline),
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
        request::poll(self.request, self.seq).map(|outcome| match outcome {
            request::DONE => Ok(()),
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
            self.seq >> request::SEQ_SHIFT
        )
    }
}

/// An error of an IPC post or wait.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum IpcError {
    /// The CPU does not take IPC interrupts, or [`CpuControl`] started it again before the run.
    ///
    /// [`CpuControl`]: crate::system::CpuControl
    Offline,
    /// The request is pending on a CPU other than the target CPU, or runs on it.
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
