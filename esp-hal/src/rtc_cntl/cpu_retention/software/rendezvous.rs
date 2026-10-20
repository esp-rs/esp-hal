//! The rendezvous that lets both cores retain themselves across a light sleep.
//!
//! Software retention means that each core saves itself, so a stalled core comes back with
//! garbage. The PRO CPU requests every light sleep, and the APP CPU saves itself in the same window
//! and then waits for the sleep to end. A light sleep that the APP CPU asks for starts on the PRO
//! CPU too: the APP CPU kicks the PRO CPU, which requests the sleep from its IPC handler, and the
//! APP CPU takes its part of the handshake from its own call.
//!
//! The handshake is one shared [`Phase`]. Every transition has one writer, except the ways out of
//! [`Phase::Wanted`] and [`Phase::Requested`], which both cores take with a compare-and-swap.

use core::{cell::UnsafeCell, mem::MaybeUninit, ptr};

use esp_sync::raw::{RawLock, SingleCoreInterruptLock};
use portable_atomic::{AtomicU8, Ordering};

use super::{
    CoreRetentionContext,
    device_regs,
    frames::chip::PMUFUNC_GOING_TO_SLEEP,
    save_critical_frame,
    save_pre_critical,
};
use crate::{
    interrupt::ipc::Ipc,
    peripherals::{IPC, LPWR},
    rtc_cntl::sleep::{LowPower, RtcSleepConfig, SleepKind},
    soc::cpu_control,
    system::Cpu,
    time::implem as raw_time,
};

/// How long a core waits for the other core to answer.
///
/// esp-idf waits without a bound, because it delivers the helper routine in a high-priority
/// interrupt that a critical section cannot mask. esp-hal has no such path: the IPC interrupt runs
/// at the lowest priority, so a core that holds a critical section of its own cannot answer it. A
/// request without an answer refuses the sleep.
const ANSWER_TIMEOUT_US: u64 = 1_000;

/// The step that the rendezvous has reached.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Eq)]
enum Phase {
    /// No sleep uses the rendezvous.
    Idle,

    /// The APP CPU asks for a sleep, and waits for the PRO CPU to take the request.
    Wanted,

    /// The PRO CPU took the request of the APP CPU, and prepares the sleep.
    Taken,

    /// The PRO CPU asks the APP CPU to save itself.
    Requested,

    /// The APP CPU refused the sleep.
    Refused,

    /// The APP CPU took the request, and saves itself.
    Joined,

    /// The APP CPU saved itself, and waits for the sleep to end.
    Saved,

    /// The sleep is over, or it never started. The APP CPU restores itself if it lost power.
    Resume,

    /// The APP CPU is back, and waits for the PRO CPU to finish the sleep.
    Done,
}

static PHASE: AtomicU8 = AtomicU8::new(Phase::Idle as u8);

/// The configuration of the sleep that the APP CPU asks for.
struct SleepConfig(UnsafeCell<MaybeUninit<RtcSleepConfig>>);

// SAFETY: the APP CPU writes the configuration before it moves the phase to `Wanted`, and the PRO
// CPU reads it only after it moved the phase from `Wanted` to `Taken`.
unsafe impl Sync for SleepConfig {}

static SLEEP_CONFIG: SleepConfig = SleepConfig(UnsafeCell::new(MaybeUninit::uninit()));

/// Takes a sleep that the APP CPU asks for, so that the light sleep that the PRO CPU starts
/// answers it.
///
/// Runs on the PRO CPU, before a light sleep.
#[inline(always)]
pub(crate) fn take_request() {
    swap_phase(Phase::Wanted, Phase::Taken);
}

/// Lets the APP CPU return from a light sleep, which the PRO CPU has finished.
///
/// This answers the request that [`take_request`] took, and releases the APP CPU from the
/// rendezvous. Runs on the PRO CPU, after a light sleep.
#[inline(always)]
pub(crate) fn release_app_cpu() {
    // No compare-and-swap needed: the APP CPU never writes the phase while it is `Taken` or `Done`.
    if matches!(phase(), Phase::Taken | Phase::Done) {
        PHASE.store(Phase::Idle as u8, Ordering::Release);
    }
}

/// Brings the APP CPU into the rendezvous, and returns whether the PRO CPU sleeps.
///
/// Runs on the PRO CPU, with interrupts disabled. A `false` return means that the APP CPU refused
/// the sleep, or that it did not answer in time. Nothing needs an undo then. A `true` return needs
/// [`finish`], whether the sleep happens or not.
#[crate::ram]
pub(crate) fn engage() -> bool {
    if !cpu_control::is_running(Cpu::AppCpu) {
        // A core that never started, or that the program stalled, holds nothing to save. The
        // caller then sleeps as a single-core chip does.
        return true;
    }

    // The APP CPU can move the phase between `Idle` and `Wanted` meanwhile, so the swap retries.
    let app_cpu_waits = loop {
        let current = phase();
        debug_assert!(matches!(
            current,
            Phase::Idle | Phase::Wanted | Phase::Taken
        ));
        if swap_phase(current, Phase::Requested) {
            break current != Phase::Idle;
        }
    };

    // An APP CPU that asked for a sleep waits for it with interrupts disabled, and reads the phase
    // itself. It could not take the doorbell anyway.
    if !app_cpu_waits {
        // Only this function posts to the APP CPU, so the post merges with a pending doorbell
        // instead of waiting for it.
        Ipc::new(unsafe { IPC::steal() }).call_function(Cpu::AppCpu, helper_entry);
    }

    let deadline = raw_time::raw_counter() + raw_time::us_to_ticks(ANSWER_TIMEOUT_US);
    loop {
        match phase() {
            Phase::Saved => return true,
            Phase::Refused => {
                PHASE.store(Phase::Idle as u8, Ordering::Release);
                return false;
            }
            // Once the APP CPU took the request, it saves itself without a further answer, so
            // the timeout covers the request only.
            Phase::Requested if raw_time::raw_counter() >= deadline => {
                if swap_phase(Phase::Requested, Phase::Idle) {
                    return false;
                }
            }
            _ => core::hint::spin_loop(),
        }
    }
}

/// Returns whether the APP CPU saved itself and waits for the sleep to end.
#[inline(always)]
pub(crate) fn helper_saved() -> bool {
    phase() == Phase::Saved
}

/// Lets the APP CPU restore itself, and waits until it is back.
///
/// Runs on the PRO CPU after [`engage`] returned `true`, and does nothing if the APP CPU is not in
/// the rendezvous. After a power-down wake the APP CPU passes the wake stub, so the stub must stay
/// armed until this function returns. The APP CPU returns only after [`release_app_cpu`], once the
/// PRO CPU has restored the time base and recorded the wakeup cause.
#[crate::ram]
pub(crate) fn finish() {
    if !helper_saved() {
        return;
    }

    PHASE.store(Phase::Resume as u8, Ordering::Release);
    while phase() != Phase::Done {
        core::hint::spin_loop();
    }
}

/// Asks the PRO CPU for a light sleep, and takes the part of the APP CPU in it.
///
/// Runs on the APP CPU, with interrupts disabled. Returns when the sleep is over, when the PRO CPU
/// refused it, or when the PRO CPU did not take the request in time.
#[crate::ram]
pub(crate) fn sleep_from_app_cpu(config: RtcSleepConfig) {
    let mut deadline = None;
    loop {
        match phase() {
            Phase::Idle => match deadline {
                // The PRO CPU answered the request without the rendezvous, or refused it.
                Some(_) => return,
                None => {
                    // SAFETY: the phase is not `Wanted`, so the PRO CPU does not read the
                    // configuration.
                    unsafe { (*SLEEP_CONFIG.0.get()).write(config) };
                    if swap_phase(Phase::Idle, Phase::Wanted) {
                        // Only this function posts to the PRO CPU, so the post never waits.
                        Ipc::new(unsafe { IPC::steal() }).call_function(Cpu::ProCpu, kick_entry);
                        deadline = Some(
                            raw_time::raw_counter() + raw_time::us_to_ticks(ANSWER_TIMEOUT_US),
                        );
                    }
                }
            },
            // Every sleep that the PRO CPU requests needs this core, the one it requests for this
            // call and one it requests on its own.
            Phase::Requested => {
                if swap_phase(Phase::Requested, Phase::Joined) {
                    run_helper();
                    return;
                }
            }
            Phase::Wanted => {
                if deadline.is_some_and(|deadline| raw_time::raw_counter() >= deadline)
                    && swap_phase(Phase::Wanted, Phase::Idle)
                {
                    return;
                }
            }
            _ => core::hint::spin_loop(),
        }
    }
}

/// Takes the request of the APP CPU, and requests the sleep for it.
#[crate::ram]
fn kick_entry() {
    // The answer of `can_sleep` holds for the whole sleep, so nothing may give this core work
    // between the two.
    let irq_token = unsafe { SingleCoreInterruptLock.enter() };

    // A kick of a request that the APP CPU withdrew, or that an earlier sleep answered, finds the
    // phase moved on.
    if swap_phase(Phase::Wanted, Phase::Taken) {
        if can_sleep() {
            // SAFETY: the APP CPU wrote the configuration before it set `Wanted`.
            let config = unsafe { (*SLEEP_CONFIG.0.get()).assume_init() };
            LowPower::new(unsafe { LPWR::steal() }).sleep(config, SleepKind::Light, true);
        } else {
            PHASE.store(Phase::Idle as u8, Ordering::Release);
        }
    }

    unsafe { SingleCoreInterruptLock.exit(irq_token) };
}

/// Takes the doorbell of the PRO CPU.
#[crate::ram]
fn helper_entry() {
    // The answer of `can_sleep` holds for the whole sleep, so nothing may give this core work
    // between the two.
    let irq_token = unsafe { SingleCoreInterruptLock.enter() };

    // A doorbell of a request that the PRO CPU withdrew finds the phase moved on.
    if phase() == Phase::Requested {
        let join = can_sleep();
        let next = if join { Phase::Joined } else { Phase::Refused };
        if swap_phase(Phase::Requested, next) && join {
            run_helper();
        }
    }

    unsafe { SingleCoreInterruptLock.exit(irq_token) };
}

/// Saves the APP CPU, waits for the sleep to end, then restores the APP CPU if it lost power.
///
/// The caller holds interrupts disabled, so that no handler runs between the two frames, and it
/// moved the phase to [`Phase::Joined`].
///
/// The branch predictor starts cache requests of its own, so it must be off while the CPU domain
/// has no power. esp-idf turns it off for this routine only, and not for the single-core path.
#[crate::ram]
fn run_helper() {
    crate::soc::disable_branch_predictor();

    // SAFETY: the PRO CPU starts the rendezvous for a sleep that retains the CPU only, so the
    // memory of the frames is installed.
    let buffer = unsafe { crate::rtc_cntl::installed_buffer_ptr().unwrap_unchecked() };
    let mut ctx = CoreRetentionContext::new(buffer.as_ptr(), Cpu::AppCpu as usize);
    save_pre_critical(&mut ctx);
    let frame = save_critical_frame(&ctx);

    // The wake stub writes this word while the compiler believes that nothing did.
    // SAFETY: `save_critical_frame` returns the frame pointer that it was given.
    let pmufunc = unsafe { ptr::read_volatile(&raw const (*frame).pmufunc) };
    if pmufunc & 3 == PMUFUNC_GOING_TO_SLEEP {
        // The CPU domain can lose power from here on. This core then comes back through the wake
        // stub, after the PRO CPU released it from reset. This core does not arm the wake stub:
        // the PRO CPU arms it for both cores.
        PHASE.store(Phase::Saved as u8, Ordering::Release);
    } else {
        // The wake stub restored the critical frame already.
        // SAFETY: the frame holds what this core saved before the sleep.
        unsafe { ctx.non_critical().as_ref().unwrap().restore() };
        device_regs::restore(&super::regions(), ctx.device_frame());
    }

    while phase() != Phase::Resume {
        core::hint::spin_loop();
    }
    PHASE.store(Phase::Done as u8, Ordering::Release);

    // The PRO CPU can start the next sleep right after it releases this core, so this core waits
    // for the phase to leave `Done`, and not for a particular phase.
    while phase() == Phase::Done {
        core::hint::spin_loop();
    }

    crate::soc::enable_branch_predictor();
}

/// Returns whether the core that runs this function agrees to a light sleep.
#[inline(always)]
fn can_sleep() -> bool {
    let handler = crate::rtc_cntl::sleep::CAN_SLEEP_HANDLER.load(Ordering::Acquire);
    // SAFETY: `set_can_sleep_handler` takes a `fn() -> bool`, and this is that pointer.
    handler.is_null() || unsafe { core::mem::transmute::<*mut (), fn() -> bool>(handler)() }
}

#[inline(always)]
fn phase() -> Phase {
    // SAFETY: every write to `PHASE` writes a `Phase`.
    unsafe { core::mem::transmute::<u8, Phase>(PHASE.load(Ordering::Acquire)) }
}

#[inline(always)]
fn swap_phase(current: Phase, new: Phase) -> bool {
    PHASE
        .compare_exchange(
            current as u8,
            new as u8,
            Ordering::AcqRel,
            Ordering::Acquire,
        )
        .is_ok()
}
