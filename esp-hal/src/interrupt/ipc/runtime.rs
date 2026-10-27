//! The IPC state and the post of the esp-hal copy with the `rt` feature.

use core::ptr::NonNull;

use esp_sync::raw::{RawLock, SingleCoreInterruptLock};
use portable_atomic::{AtomicU32, Ordering};

use super::{IpcRequest, queue};
use crate::system::Cpu;

/// The IPC state of one CPU.
#[repr(C)]
pub(super) struct CoreState {
    /// Non-zero while the CPU defers its context switches. The CPU then keeps a pending context
    /// switch until [`resume_context_switch`]. Only the CPU itself writes this field while the CPU
    /// runs.
    deferral: AtomicU32,

    /// The requests that [`Ipc::post`][super::Ipc::post] queued for this CPU.
    pub(super) lists: queue::Lists,
}

impl CoreState {
    const fn new() -> Self {
        Self {
            deferral: AtomicU32::new(0),
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
/// `core` does not run its IPC handler again before a restart, and
/// [`app_core_takes_ipc`][crate::system::multi_core::app_core_takes_ipc] returns `false` for it.
pub(crate) unsafe fn fail_stranded(core: Cpu) {
    // A post that read `true` from `app_core_takes_ipc` can push until it leaves `APP_POSTS`.
    if core == Cpu::AppCpu {
        // This load and the load in `esp_hal_ipc_post` must be `SeqCst`. Otherwise each side can
        // miss the store of the other side.
        while APP_POSTS.load(Ordering::SeqCst) != 0 {
            core::hint::spin_loop();
        }
    }
    unsafe { queue::fail_stranded(&STATE[core as usize].lists, core as u32) };
}

/// The number of posts to the APP CPU between their check of
/// [`app_core_takes_ipc`][crate::system::multi_core::app_core_takes_ipc] and their push.
static APP_POSTS: AtomicU32 = AtomicU32::new(0);

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
pub(super) unsafe extern "C" fn esp_hal_ipc_post(core: u8, request: NonNull<IpcRequest>) -> u32 {
    let request = unsafe { request.as_ref() };
    let core = match core {
        0 => Cpu::ProCpu,
        1 => Cpu::AppCpu,
        _ => return queue::POST_OFFLINE,
    };

    // A context switch must not stop the post between the push and the raise.
    let token = unsafe { SingleCoreInterruptLock.enter() };
    // A stop of the APP CPU clears `app_core_takes_ipc`, then waits until `APP_POSTS` is 0.
    let counted = core == Cpu::AppCpu;
    if counted {
        APP_POSTS.fetch_add(1, Ordering::SeqCst);
    }
    let result = if counted && !crate::system::multi_core::app_core_takes_ipc() {
        queue::POST_OFFLINE
    } else {
        unsafe {
            queue::post(&STATE[core as usize].lists, core as u32, request, || {
                super::implem::raise(core)
            })
        }
    };
    if counted {
        APP_POSTS.fetch_sub(1, Ordering::Release);
    }
    unsafe { SingleCoreInterruptLock.exit(token) };

    result
}

/// Raises the IPC interrupt of `core`.
///
/// # Panics
///
/// In debug builds, panics if `core` is the APP CPU and that CPU is stopped or not started. A
/// parked CPU takes the interrupt after the park ends.
#[inline]
pub(crate) fn raise(core: Cpu) {
    debug_assert!(
        core != Cpu::AppCpu || crate::system::multi_core::app_core_takes_ipc(),
        "IPC target APP core is stopped or not started"
    );
    super::implem::raise(core);
}

/// Defers the context switches of the current CPU, so that the current thread stays on the CPU.
///
/// Interrupt handlers and IPC requests still run. The deferral lasts until each call has a
/// matching call to [`resume_context_switch`].
#[inline(always)]
pub(crate) fn defer_context_switch() {
    let token = unsafe { SingleCoreInterruptLock.enter() };
    let deferral = &STATE[Cpu::current() as usize].deferral;
    let state = deferral.load(Ordering::Relaxed);
    debug_assert_ne!(state, u32::MAX);
    deferral.store(state + 1, Ordering::Relaxed);
    unsafe { SingleCoreInterruptLock.exit(token) };
}

/// Ends one deferral of the current CPU. The end of the last deferral requests the deferred
/// context switch.
///
/// # Safety
///
/// The call must match an earlier call to [`defer_context_switch`] on the current CPU.
#[inline(always)]
pub(crate) unsafe fn resume_context_switch() {
    // The active deferral keeps the thread on the current CPU.
    let core = Cpu::current();
    let deferral = &STATE[core as usize].deferral;

    let token = unsafe { SingleCoreInterruptLock.enter() };
    let state = deferral.load(Ordering::Relaxed);
    debug_assert_ne!(state, 0);
    deferral.store(state - 1, Ordering::Relaxed);
    if state == 1 && crate::interrupt::__rtos_implementation::context_switch_pending(core) {
        // Any interrupt of the minimum priority switches tasks when it exits.
        raise(core);
    }
    unsafe { SingleCoreInterruptLock.exit(token) };
}

/// Returns `true` while the current CPU defers its context switches.
#[inline(always)]
pub(crate) fn context_switch_deferred() -> bool {
    STATE[Cpu::current() as usize]
        .deferral
        .load(Ordering::Relaxed)
        != 0
}

/// Clears the deferral of `core` before the CPU starts.
pub(crate) fn reset_context_switch_deferral(core: Cpu) {
    STATE[core as usize].deferral.store(0, Ordering::Relaxed);
}
