//! The save of the APP CPU across a light sleep that powers the CPU domain down.
//!
//! Software retention means that each core saves itself, so a stalled core comes back with
//! garbage. The PRO CPU requests every light sleep. It parks the APP CPU, and lets the parked APP
//! CPU save itself and wait for the end of the sleep. A light sleep that the APP CPU asks for runs
//! on the PRO CPU too.

use core::{cell::UnsafeCell, mem::MaybeUninit, ptr, ptr::NonNull};

use portable_atomic::{AtomicPtr, AtomicU8, Ordering};

use super::{
    CoreRetentionContext,
    device_regs,
    frames::chip::PMUFUNC_GOING_TO_SLEEP,
    save_critical_frame,
    save_pre_critical,
};
use crate::{
    interrupt::ipc::{Ipc, IpcRequest},
    peripherals::{IPC, LPWR},
    ram,
    rtc_cntl::sleep::{LowPower, RtcSleepConfig, SleepKind},
    system::{Cpu, multi_core::park},
};

/// The step of the APP CPU in a sleep.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Eq)]
enum Phase {
    /// The APP CPU takes no part in a sleep.
    Idle,

    /// The APP CPU saves itself.
    Saving,

    /// The APP CPU saved itself, and waits for the sleep to end.
    Saved,

    /// The sleep is over, or it never started. The APP CPU restores itself if it lost power.
    Resume,
}

static PHASE: AtomicU8 = AtomicU8::new(Phase::Idle as u8);

/// The retention buffer of the sleep that the APP CPU saves itself for.
static BUFFER: AtomicPtr<u8> = AtomicPtr::new(ptr::null_mut());

/// Lets the parked APP CPU save itself, and returns whether it saved itself.
///
/// Runs on the PRO CPU, with interrupts disabled, while a park holds the APP CPU. A `true` return
/// needs [`resume_app_cpu`].
#[crate::ram]
pub(crate) fn save_app_cpu(buffer: NonNull<u8>) -> bool {
    debug_assert_eq!(Cpu::current(), Cpu::ProCpu);

    BUFFER.store(buffer.as_ptr(), Ordering::Relaxed);
    PHASE.store(Phase::Saving as u8, Ordering::Relaxed);
    if !park::start_run(Cpu::AppCpu, save_and_wait) {
        // A core that never started, or that the program stopped, holds nothing to save.
        PHASE.store(Phase::Idle as u8, Ordering::Relaxed);
        return false;
    }

    while phase() != Phase::Saved {
        core::hint::spin_loop();
    }
    true
}

/// Returns whether the APP CPU saved itself and waits for the sleep to end.
#[inline(always)]
pub(crate) fn app_cpu_saved() -> bool {
    phase() == Phase::Saved
}

/// Lets the APP CPU restore itself, and waits until the park holds it again.
///
/// Runs on the PRO CPU after [`save_app_cpu`] returned `true`. After a power-down wake the APP CPU
/// passes the wake stub, so the stub must stay armed until this function returns.
#[crate::ram]
pub(crate) fn resume_app_cpu() {
    PHASE.store(Phase::Resume as u8, Ordering::Release);
    park::end_run(Cpu::AppCpu);
    PHASE.store(Phase::Idle as u8, Ordering::Relaxed);
}

/// Saves the APP CPU, waits for the sleep to end, then restores the APP CPU if it lost power.
///
/// Runs in the park of the APP CPU, with interrupts disabled. The function must not return before
/// the sleep ends, because the wake stub returns into its stack frame.
///
/// The branch predictor starts cache requests of its own, so it must be off while the CPU domain
/// has no power. esp-idf turns it off for this routine only, and not for the single-core path.
///
/// The APP CPU waits here while the PRO CPU suspends the caches. After a wake, the APP CPU
/// restores its state before the caches resume.
#[ram(unstable(checked))]
fn save_and_wait() {
    let _predictor = crate::soc::branch_predictor::DisabledBranchPredictor::new();

    let buffer = BUFFER.load(Ordering::Relaxed);
    let mut ctx = CoreRetentionContext::new(buffer, Cpu::AppCpu as usize);
    save_pre_critical(&mut ctx);
    let frame = save_critical_frame(&ctx);

    // The wake stub writes this word while the compiler believes that nothing did.
    // SAFETY: `save_critical_frame` returns the frame pointer that it was given.
    let pmufunc = unsafe { device_regs::load_word(&raw const (*frame).pmufunc) };
    if pmufunc & 3 == PMUFUNC_GOING_TO_SLEEP {
        // The CPU domain can lose power from here on. This core then comes back through the wake
        // stub, after the PRO CPU released it from reset. This core does not arm the wake stub:
        // the PRO CPU arms it for both cores.
        PHASE.store(Phase::Saved as u8, Ordering::Release);
    } else {
        // The wake stub restored the critical frame already.
        // SAFETY: the frame holds what this core saved before the sleep.
        unsafe { (*ctx.non_critical()).restore() };
        device_regs::restore(&super::regions(), ctx.device_frame());
    }

    while phase() != Phase::Resume {
        core::hint::spin_loop();
    }
}

/// The result of a sleep that the APP CPU asks for.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Eq)]
enum Outcome {
    /// The PRO CPU has not finished the sleep.
    Pending,

    /// The sleep is over.
    Ended,

    /// A wakeup source refused the sleep.
    Refused,
}

static OUTCOME: AtomicU8 = AtomicU8::new(Outcome::Ended as u8);

/// The sleep that the APP CPU asks for.
#[derive(Clone, Copy)]
struct Request {
    config: RtcSleepConfig,
    may_sleep: Option<fn() -> bool>,
}

struct RequestSlot(UnsafeCell<MaybeUninit<Request>>);

// SAFETY: the APP CPU writes the request before it moves the outcome to `Pending`, and the PRO CPU
// reads it only while the outcome is `Pending`.
unsafe impl Sync for RequestSlot {}

static REQUEST: RequestSlot = RequestSlot(UnsafeCell::new(MaybeUninit::uninit()));

/// Lets the PRO CPU run a light sleep for the APP CPU, and returns whether a wakeup source or
/// `may_sleep` refused it.
///
/// Runs on the APP CPU, with interrupts disabled. The PRO CPU parks this core for the sleep.
#[crate::ram]
pub(crate) fn sleep_from_app_cpu(config: RtcSleepConfig, may_sleep: Option<fn() -> bool>) -> bool {
    // SAFETY: the outcome is not `Pending`, so the PRO CPU does not read the request.
    unsafe { (*REQUEST.0.get()).write(Request { config, may_sleep }) };
    OUTCOME.store(Outcome::Pending as u8, Ordering::Release);

    let result = Ipc::new(unsafe { IPC::steal() }).post(Cpu::ProCpu, &SLEEP);
    // Only the APP CPU posts `SLEEP`, and the PRO CPU always takes IPC interrupts.
    debug_assert!(result.is_ok(), "Posting the light sleep failed");
    park::wait_until(|| OUTCOME.load(Ordering::Acquire) != Outcome::Pending as u8);

    OUTCOME.load(Ordering::Relaxed) == Outcome::Refused as u8
}

/// Answers the request of the APP CPU with the light sleep that the PRO CPU has finished.
///
/// Runs on the PRO CPU, after a light sleep that no wakeup source refused.
#[inline(always)]
pub(crate) fn answer_app_cpu() {
    let _ = OUTCOME.compare_exchange(
        Outcome::Pending as u8,
        Outcome::Ended as u8,
        Ordering::AcqRel,
        Ordering::Relaxed,
    );
}

static SLEEP: IpcRequest = IpcRequest::new(sleep_entry);

/// Runs the sleep that the APP CPU asks for.
#[crate::ram]
extern "C" fn sleep_entry() {
    // A sleep of the PRO CPU can answer the request before this function runs.
    if OUTCOME.load(Ordering::Acquire) != Outcome::Pending as u8 {
        return;
    }

    // SAFETY: the APP CPU wrote the request before it moved the outcome to `Pending`.
    let request = unsafe { (*REQUEST.0.get()).assume_init() };
    let refused = LowPower::new(unsafe { LPWR::steal() }).sleep(
        request.config,
        SleepKind::Light,
        true,
        request.may_sleep,
    );
    if refused {
        OUTCOME.store(Outcome::Refused as u8, Ordering::Release);
    }
}

#[inline(always)]
fn phase() -> Phase {
    // SAFETY: every write to `PHASE` writes a `Phase`.
    unsafe { core::mem::transmute::<u8, Phase>(PHASE.load(Ordering::Acquire)) }
}
