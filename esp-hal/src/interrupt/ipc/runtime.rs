//! The IPC state and the post of the esp-hal copy with the `rt` feature.

use core::ptr::NonNull;

use portable_atomic::{AtomicPtr, Ordering};

use super::{IpcRequest, implem::raise, queue};
use crate::{interrupt::__rtos_implementation::ContextSwitchHandler, system::Cpu};

/// The IPC state of one CPU.
///
/// Every CPU has its own handler, which reads the fields at constant addresses.
#[repr(C)]
pub(super) struct CoreState {
    /// The context-switch handler that the CPU runs, or null if the CPU has none pending.
    ///
    /// [`request_context_switch`] copies `context_switch_handler` here. A posted request alone
    /// therefore does not make the CPU switch tasks.
    pub(super) pending_context_switch: AtomicPtr<()>,

    /// The context-switch handler of the CPU, or null if no RTOS registered one.
    ///
    /// This field is set once, and holds the only handler that the CPU accepts.
    pub(super) context_switch_handler: AtomicPtr<()>,

    /// The address of the register that clears the request of this CPU.
    #[cfg(riscv)]
    pub(super) request: AtomicPtr<u32>,

    /// The requests that [`Ipc::post`][super::Ipc::post] queued for this CPU.
    pub(super) lists: queue::Lists,
}

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

pub(super) static STATE: [CoreState; Cpu::COUNT] = [const { CoreState::new() }; Cpu::COUNT];

/// Completes each request that `core` did not run with
/// [`IpcError::Offline`][super::IpcError::Offline], and empties the lists of `core`.
///
/// # Safety
///
/// `core` does not run, and posts to `core` return [`IpcError::Offline`][super::IpcError::Offline].
pub(crate) unsafe fn fail_stranded(core: Cpu) {
    unsafe { queue::fail_stranded(&STATE[core as usize].lists, core as u32) };
}

/// Queues a run of `request` on the CPU `core`.
///
/// Returns `seq << 5` for a queued or joined run, 1 if `core` does not take IPC interrupts, and 2
/// if `request` is busy.
///
/// # Safety
///
/// `request` stays valid as long as the runtime can reach it.
// esp-hal copies without the `rt` feature call this function by name. The name, the argument
// types, the encoding of the return value and the layout of `IpcRequest` must stay stable.
#[unsafe(export_name = "esp_hal_ipc_post")]
pub(super) unsafe extern "C" fn esp_hal_ipc_post(
    core: u8,
    request: NonNull<IpcRequest>,
    arg: usize,
) -> u32 {
    use esp_sync::raw::{RawLock, SingleCoreInterruptLock};

    let request = unsafe { request.as_ref() };
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
///     crate::interrupt::__rtos_implementation::set_context_switch_handler
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
/// The handler runs at [`Priority::min`][crate::interrupt::Priority::min], after the requests that
/// [`Ipc::post`][super::Ipc::post] queued. The call does not wait, not even for a pending request.
///
/// # Panics
///
/// In debug builds, panics if `core` has no context-switch handler, or if `core` is the APP CPU
/// and that CPU is not running.
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
