//! The rendezvous that lets both cores retain themselves across a light sleep.
//!
//! Software retention means that each core saves itself, so a stalled core comes back with
//! garbage. One core requests the sleep, and the other core saves itself in the same window and
//! then waits for the sleep to end. This module mirrors the state machine of
//! `esp32s31/sleep_cpu.c`.

use core::ptr;

use esp_sync::raw::{RawLock, SingleCoreInterruptLock};
use portable_atomic::{AtomicBool, AtomicU8, Ordering};

use super::{
    chips,
    device_regs,
    frames::chip::PMUFUNC_GOING_TO_SLEEP,
    software::{CoreRetentionContext, arm_wake_stub, save_critical_frame, save_pre_critical},
};
use crate::{
    interrupt::ipc::Ipc,
    peripherals::IPC,
    soc::cpu_control,
    system::{self, Cpu},
    time::implem as raw_time,
};

/// How long [`engage`] waits for the helper to start its backup.
///
/// esp-idf waits without a bound, because it delivers the helper routine in a high-priority
/// interrupt that a critical section cannot mask. esp-hal has no such path: the IPC interrupt runs
/// at the lowest priority, so a core that holds a critical section of its own cannot answer it.
const ENLIST_TIMEOUT_US: u64 = 1_000;

/// No core holds the initiator slot.
const INITIATOR_NONE: u8 = u8::MAX;

/// The step that one core has reached, from `smp_retention_state_t`.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Eq)]
enum State {
    Idle,
    BackupStart,
    BackupDone,
    RestoreStart,
    RestoreDone,

    /// The sleep did not happen, which releases the core that waits for it.
    SkipRetention,
}

static STATES: [AtomicU8; Cpu::COUNT] = [const { AtomicU8::new(State::Idle as u8) }; Cpu::COUNT];

/// The core that requests the sleep, or [`INITIATOR_NONE`].
static INITIATOR: AtomicU8 = AtomicU8::new(INITIATOR_NONE);

/// Whether the core of each entry must run the helper routine.
///
/// The initiator sets the entry of the other core before it raises the IPC interrupt, and the
/// helper takes it. A doorbell that arrives after the sleep therefore finds nothing to do.
static REQUEST: [AtomicBool; Cpu::COUNT] = [const { AtomicBool::new(false) }; Cpu::COUNT];

/// Whether the initiator has a helper that saved itself and waits for the sleep.
static HELPER_ENLISTED: AtomicBool = AtomicBool::new(false);

/// Brings the other core into the rendezvous, and returns whether the caller sleeps.
///
/// A return of `false` means that the other core requested the sleep, and that this core saved
/// itself, lost its power, and came back. The sleep is over, so the caller must return without a
/// sleep of its own.
///
/// The caller must run with interrupts disabled, which is what stops a doorbell from arriving
/// while this core acts as the helper.
#[crate::ram]
pub(crate) fn engage() -> bool {
    if !cpu_control::is_running(Cpu::AppCpu) {
        // A core that never started, or that the program stalled, holds nothing to save. The
        // caller then sleeps as a single-core chip does.
        return true;
    }

    let core = system::raw_core();
    let other = other_core();

    if INITIATOR
        .compare_exchange(
            INITIATOR_NONE,
            core as u8,
            Ordering::AcqRel,
            Ordering::Acquire,
        )
        .is_err()
    {
        // The other core asked first. This core cannot answer the doorbell, because it holds
        // interrupts off for the sleep path, so it runs the helper routine here instead. The
        // request flag is cleared afterwards, when the initiator has certainly set it, so that
        // the doorbell that arrives later finds nothing to do.
        run_helper();
        REQUEST[core].store(false, Ordering::Release);
        return false;
    }

    // The state must reach `BackupStart` before the doorbell, because that is what the helper
    // waits for.
    wait_for(other, State::Idle);
    set_state(core, State::BackupStart);
    REQUEST[other].store(true, Ordering::Release);

    // Only a function that the program posts can make this call wait, because the rendezvous
    // posts one function, and two posts of the same function coalesce.
    Ipc::new(unsafe { IPC::steal() }).call_function(other_cpu(), helper_entry);

    let enlisted = {
        let deadline = raw_time::raw_counter() + raw_time::us_to_ticks(ENLIST_TIMEOUT_US);
        loop {
            match state_of(other) {
                State::BackupStart => break Enlisted::Joined,
                State::SkipRetention => break Enlisted::Refused,
                _ if raw_time::raw_counter() >= deadline => break Enlisted::NoAnswer,
                _ => core::hint::spin_loop(),
            }
        }
    };

    match enlisted {
        Enlisted::Joined => {
            HELPER_ENLISTED.store(true, Ordering::Release);
            true
        }
        Enlisted::Refused => {
            // The other core has work of its own, so the system must not sleep. The helper waits
            // for this store before it leaves its routine.
            REQUEST[other].store(false, Ordering::Release);
            set_state(core, State::Idle);
            release_initiator(core);
            false
        }
        Enlisted::NoAnswer => {
            // `SkipRetention` stays until [`finish`], so that a helper which took the request flag
            // before this store leaves its routine, instead of waiting for a sleep that keeps the
            // CPU domain powered.
            REQUEST[other].store(false, Ordering::Release);
            set_state(core, State::SkipRetention);
            true
        }
    }
}

/// Returns whether a helper saved itself and waits for this core to request the sleep.
#[inline(always)]
pub(crate) fn helper_enlisted() -> bool {
    HELPER_ENLISTED.load(Ordering::Acquire)
}

/// Returns whether the sleep can power the CPU domain down.
///
/// A sleep that powers the domain down needs every running core to save itself.
#[inline(always)]
pub(crate) fn retention_allowed() -> bool {
    !cpu_control::is_running(Cpu::AppCpu) || helper_enlisted()
}

/// Saves and restores the CPU domain of the initiator, and requests the sleep.
///
/// The initiator arms the wake stub for both cores, because the chip has one wake stub register,
/// and the stub reads the frame of the core that runs it.
#[crate::ram]
pub(crate) fn sleep_retained(buffer: *mut u8, enter_sleep: fn(), wait: fn() -> bool) -> bool {
    let core = system::raw_core();
    let other = other_core();

    let mut ctx = CoreRetentionContext::new(buffer, core);
    save_pre_critical(&mut ctx);
    let frame = save_critical_frame(&ctx);

    // The wake stub writes this word while the compiler believes that nothing did.
    // SAFETY: `save_critical_frame` returns the frame pointer that it was given.
    let pmufunc = unsafe { ptr::read_volatile(&raw const (*frame).pmufunc) };
    let woke = pmufunc & 3 != PMUFUNC_GOING_TO_SLEEP;

    if woke {
        cpu_control::restart_core1_after_wake();
        set_state(core, State::RestoreStart);
        restore(&mut ctx);
        set_state(core, State::RestoreDone);
        return false;
    }

    arm_wake_stub();
    set_state(core, State::BackupDone);
    wait_for(other, State::BackupDone);

    enter_sleep();
    let rejected = wait();

    if rejected {
        // This is the only store that releases the helper from its wait. A rejected request
        // leaves both frames alone, for the reason that `software::sleep_retained` gives: the
        // registers still hold what the save read.
        set_state(core, State::SkipRetention);
    }

    rejected
}

/// Waits for the helper to finish, and returns the rendezvous to [`State::Idle`].
#[crate::ram]
pub(crate) fn finish() {
    let core = system::raw_core();
    if state_of(core) == State::RestoreDone {
        wait_for(other_core(), State::RestoreDone);
    }

    HELPER_ENLISTED.store(false, Ordering::Release);
    set_state(core, State::Idle);
    release_initiator(core);
}

/// Gives the initiator slot back, if this core holds it.
#[inline(always)]
fn release_initiator(core: usize) {
    let _ = INITIATOR.compare_exchange(
        core as u8,
        INITIATOR_NONE,
        Ordering::AcqRel,
        Ordering::Acquire,
    );
}

/// Takes the request of this core, and runs the helper routine.
#[inline(always)]
fn helper_entry() {
    if REQUEST[system::raw_core()].swap(false, Ordering::AcqRel) {
        run_helper();
    }
}

/// Saves this core, waits for the sleep to end, then restores this core.
///
/// The branch predictor starts cache requests of its own, so it must be off while the CPU domain
/// has no power. esp-idf turns it off for this routine only, and not for the single-core path.
#[crate::ram]
fn run_helper() {
    crate::soc::disable_branch_predictor();

    // Interrupts stay off for the whole routine, so that no handler runs between the two frames.
    // esp-idf disables them later, before its critical frame only.
    let irq_token = unsafe { SingleCoreInterruptLock.enter() };

    let core = system::raw_core();
    let other = other_core();

    // The answer holds for the whole sleep, because the interrupts of this core are off from here
    // until the sleep is over, and nothing else can give this core work.
    let can_sleep = {
        let handler = crate::rtc_cntl::sleep::CAN_SLEEP_HANDLER.load(Ordering::Acquire);
        match handler.is_null() {
            true => true,
            // SAFETY: `set_can_sleep_handler` takes a `fn() -> bool`, and this is that pointer.
            false => unsafe { core::mem::transmute::<*mut (), fn() -> bool>(handler)() },
        }
    };

    if !can_sleep {
        set_state(core, State::SkipRetention);
    } else {
        let skip_retention = loop {
            match state_of(core) {
                State::SkipRetention => break true,
                State::BackupStart => break false,
                _ => core::hint::spin_loop(),
            }
        };

        if !skip_retention {
            set_state(core, State::BackupStart);

            // SAFETY: the rendezvous runs for a sleep that retains the CPU, so the memory of the
            // frames is installed.
            let buffer = unsafe { crate::rtc_cntl::installed_buffer_ptr().unwrap_unchecked() };
            let mut ctx = CoreRetentionContext::new(buffer.as_ptr(), core);
            save_pre_critical(&mut ctx);
            let frame = save_critical_frame(&ctx);

            // SAFETY: `save_critical_frame` returns the frame pointer that it was given.
            let pmufunc = unsafe { ptr::read_volatile(&raw const (*frame).pmufunc) };
            if pmufunc & 3 == PMUFUNC_GOING_TO_SLEEP {
                set_state(core, State::BackupDone);
                // Either the CPU domain loses power here, or the initiator reports that the
                // hardware rejected the request. This core does not arm the wake
                // stub: the initiator arms it for both cores.
                wait_for(other, State::SkipRetention);
            } else {
                cpu_control::restart_core1_after_wake();
                set_state(core, State::RestoreStart);
                restore(&mut ctx);
                set_state(core, State::RestoreDone);
            }
        }
    }

    wait_for(other, State::Idle);
    set_state(core, State::Idle);

    unsafe { SingleCoreInterruptLock.exit(irq_token) };
    crate::soc::enable_branch_predictor();
}

/// Restores the frames that memory kept. The wake stub restored the critical frame already.
#[inline(always)]
fn restore(ctx: &mut CoreRetentionContext) {
    // SAFETY: the frame holds what this core saved before the sleep.
    unsafe { ctx.non_critical().as_ref().unwrap().restore() };
    device_regs::restore(&chips::regions(), ctx.device_frame());
}

/// Returns the index of the core that this core shares the rendezvous with.
#[inline(always)]
fn other_core() -> usize {
    other_cpu() as usize
}

#[inline(always)]
fn other_cpu() -> Cpu {
    match system::raw_core() {
        0 => Cpu::AppCpu,
        _ => Cpu::ProCpu,
    }
}

#[inline(always)]
fn state_of(core: usize) -> State {
    // SAFETY: `set_state` is the only writer, and it writes a `State`.
    unsafe { core::mem::transmute::<u8, State>(STATES[core].load(Ordering::Acquire)) }
}

#[inline(always)]
fn set_state(core: usize, state: State) {
    STATES[core].store(state as u8, Ordering::Release);
}

#[inline(always)]
fn wait_for(core: usize, state: State) {
    while state_of(core) != state {
        core::hint::spin_loop();
    }
}

/// What the helper answered to the doorbell.
enum Enlisted {
    /// The helper started its backup, so the sleep can power the CPU domain down.
    Joined,

    /// The helper has work of its own, so the sleep must not happen.
    Refused,

    /// The helper did not answer inside [`ENLIST_TIMEOUT_US`].
    NoAnswer,
}
