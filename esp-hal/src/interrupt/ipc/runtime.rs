//! The IPC state and the post of the esp-hal copy with the `rt` feature.

use core::ptr::NonNull;

use esp_sync::raw::{RawLock, SingleCoreInterruptLock};
use portable_atomic::{AtomicPtr, AtomicU32, Ordering};

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

    /// Non-zero while the CPU defers its context switches. The IPC handler then leaves
    /// `pending_context_switch` for [`resume_context_switch`]. Only the CPU itself writes this
    /// field while the CPU runs.
    pub(super) deferral: AtomicU32,

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
            deferral: AtomicU32::new(0),
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
/// `core` does not run its IPC handler again before a restart, and
/// [`app_core_takes_ipc`][crate::system::multi_core::app_core_takes_ipc] returns `false` for it.
pub(crate) unsafe fn fail_stranded(core: Cpu) {
    // A post that read `true` from `app_core_takes_ipc` can push until it leaves `APP_POSTS`.
    if core == Cpu::AppCpu {
        while APP_POSTS.load(Ordering::Acquire) != 0 {
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
                raise(core)
            })
        }
    };
    if counted {
        APP_POSTS.fetch_sub(1, Ordering::Release);
    }
    unsafe { SingleCoreInterruptLock.exit(token) };

    result
}

cfg_select! {
    context_switch_source = "software0" => {
        /// The bits of the deferral that count the active deferrals.
        const DEFERRAL_COUNT: u32 = u32::MAX >> 1;
        /// Set when the `Software0` context switch was enabled before the first active deferral.
        const SOFTWARE0_ENABLED: u32 = !DEFERRAL_COUNT;
    }
    _ => {
        /// The bits of the deferral that count the active deferrals.
        const DEFERRAL_COUNT: u32 = u32::MAX;
    }
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
    debug_assert_ne!(state & DEFERRAL_COUNT, DEFERRAL_COUNT);
    #[cfg(context_switch_source = "software0")]
    let state =
        if state == 0 && crate::interrupt::__rtos_implementation::mask_local_context_switch() {
            SOFTWARE0_ENABLED
        } else {
            state
        };
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
    let state = &STATE[core as usize];

    let token = unsafe { SingleCoreInterruptLock.enter() };
    let deferral = state.deferral.load(Ordering::Relaxed);
    debug_assert_ne!(deferral & DEFERRAL_COUNT, 0);
    let remaining = deferral - 1;
    let last = remaining & DEFERRAL_COUNT == 0;
    state
        .deferral
        .store(if last { 0 } else { remaining }, Ordering::Relaxed);
    if last {
        #[cfg(context_switch_source = "software0")]
        if deferral & SOFTWARE0_ENABLED != 0 {
            crate::interrupt::__rtos_implementation::unmask_local_context_switch();
        }
        if !state
            .pending_context_switch
            .load(Ordering::Acquire)
            .is_null()
        {
            raise(core);
        }
    }
    unsafe { SingleCoreInterruptLock.exit(token) };
}

/// Clears the deferral of `core` before the CPU starts.
pub(crate) fn reset_context_switch_deferral(core: Cpu) {
    STATE[core as usize].deferral.store(0, Ordering::Relaxed);
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
/// and that CPU is stopped or not started. A parked CPU runs the handler after the park ends.
#[inline]
pub(crate) fn request_context_switch(core: Cpu) {
    debug_assert!(
        core != Cpu::AppCpu || crate::system::multi_core::app_core_takes_ipc(),
        "IPC target APP core is stopped or not started"
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
