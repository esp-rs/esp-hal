//! Parks a core at a point where the core holds no lock.
//!
//! A hardware stall stops a core at any instruction, also in a critical section or a cache
//! operation. The other core then waits forever for the lock of the stopped core.
//!
//! A park request therefore goes to the target core through the IPC interrupt. The target core
//! masks its interrupts and waits in a RAM loop. Then the requesting core stalls the target core.
//! The IPC interrupt has the lowest priority, so the IPC handler holds no lock that masks
//! interrupts.
//!
//! The claim covers the changes of the park state. A core never waits for the other core while it
//! holds the claim. The requesting core waits without the claim, and does not mask its interrupts.
//! Interrupt handlers can thus run, and can also park the same core. The requesting core also
//! runs the IPC requests that the target core posts, so the target core can wait for them.
//!
//! The requesting core defers its context switches from the choice of the target core until the
//! target core parks. The requesting thread thus cannot move to the target core. There, the IPC
//! handler would park the core under the requesting thread, and the thread would never release
//! the park.
//!
//! A core answers the park request of the other core before it takes the claim. A core thus asks
//! for a park only if the other core does not ask the core to park. Two cores never park each
//! other. The requesting core must not hold a lock that the target core can wait for with masked
//! interrupts. The target core then never answers.
//!
//! Each park adds one to the count of the parked core. Each [`unpark`] removes one. The core runs
//! again when the count is zero.
//!
//! A build can hold esp-hal versions without the `rt` feature. Those versions call the exported
//! functions of this module by symbol name. The names and signatures must stay stable.

use esp_sync::{
    RestoreState,
    raw::{RawLock, SingleCoreInterruptLock},
};
use portable_atomic::{AtomicU8, AtomicU32, AtomicUsize, Ordering};

use super::{
    APP_CORE_STOPPING,
    APP_CORE_TAKES_IPC,
    NO_CORE,
    SchedulerSuspendGuard,
    cpu_from_u8,
    is_running,
    park,
    takes_ipc,
};
use crate::{
    interrupt::ipc::{self, Ipc, IpcRequest},
    peripherals::IPC,
    ram,
    soc::cpu_control::internal_park_core,
    system::Cpu,
};

#[derive(Clone, Copy)]
#[repr(u8)]
enum Phase {
    /// The core is not in [`serve`].
    Idle,
    /// The other core asks the core to park.
    Requested,
    /// The core masked its interrupts, and waits in [`serve`].
    Parked,
    /// The other core lets the core leave [`serve`].
    Released,
}

/// The park phase of each core.
///
/// Only the other core writes `Requested` and `Released`. Only the core itself writes `Parked`
/// and `Idle`.
static PHASE: [AtomicU8; Cpu::COUNT] = [const { AtomicU8::new(Phase::Idle as u8) }; Cpu::COUNT];

/// The number of parks that hold each core.
static COUNT: [AtomicUsize; Cpu::COUNT] = [const { AtomicUsize::new(0) }; Cpu::COUNT];

const NO_CLAIM: u8 = u8::MAX;

/// The core that parks, unparks or stops the other core, or [`NO_CLAIM`].
///
/// Two cores that park each other at the same time stall each other. The claim prevents that case.
static CLAIM: AtomicU8 = AtomicU8::new(NO_CLAIM);

/// The number of starts of the APP core.
///
/// An interrupt handler can stop and start the APP core while a park of the APP core waits. That
/// park then belongs to the earlier run.
static GENERATION: AtomicU32 = AtomicU32::new(0);

/// Counts a park of `core`, or of the other core if `core` is `None`, and waits until that core is
/// in [`serve`]. [`park_stall`] then stalls the core.
///
/// Returns the core and its [`GENERATION`], or `None` where [`park`] returns `false`. If the call
/// returns a core, the current core defers its context switches until [`park_stall`] returns.
///
/// # Panics
///
/// Panics if `core` is the current core.
#[ram]
pub(crate) fn park_request(core: Option<Cpu>) -> Option<(Cpu, u32)> {
    let suspend = SchedulerSuspendGuard::new();
    let current = Cpu::current();
    let core = core.unwrap_or(match current {
        Cpu::ProCpu => Cpu::AppCpu,
        Cpu::AppCpu => Cpu::ProCpu,
    });
    assert_ne!(core, current, "A core cannot park itself");

    let phase = &PHASE[core as usize];
    let count = &COUNT[core as usize];
    'park: loop {
        let claim = Claim::take();
        if !takes_ipc(core) {
            drop(claim);
            // A stopping core runs until it stalls itself.
            while is_running(core) {
                core::hint::spin_loop();
            }
            return None;
        }
        if count.load(Ordering::Relaxed) == 0 {
            // After an unpark, `core` must leave `serve` before the next request.
            if phase.load(Ordering::Acquire) != Phase::Idle as u8 {
                continue;
            }
            phase.store(Phase::Requested as u8, Ordering::Release);
            let result = Ipc::new(unsafe { IPC::steal() }).post(core, &SERVE[core as usize]);
            // Only the other core posts `SERVE[core]`. The claim checked that `core` takes IPC
            // interrupts, and a stop of `core` needs the claim.
            debug_assert!(result.is_ok(), "Posting the park request failed");
        }
        count.fetch_add(1, Ordering::Relaxed);
        let generation = GENERATION.load(Ordering::Relaxed);
        drop(claim);

        while phase.load(Ordering::Acquire) != Phase::Parked as u8 {
            if GENERATION.load(Ordering::Acquire) != generation {
                continue 'park;
            }
            core::hint::spin_loop();
        }
        // A context switch before the stall leaves `core` in `serve` until this thread runs again.
        core::mem::forget(suspend);
        return Some((core, generation));
    }
}

/// Stalls `core` after [`park_request`] returned it with `generation`, and ends the deferral of
/// the context switches.
///
/// Returns `false`, and does not stall `core`, if `core` stopped or started again since
/// [`park_request`] returned.
///
/// The caller masks the interrupts of the current core before the call. A caller that keeps the
/// mask until [`unpark`] runs no interrupt handler and no context switch while `core` is parked.
#[ram]
pub(crate) fn park_stall(core: Cpu, generation: u32) -> bool {
    let claim = Claim::take();
    let stall = takes_ipc(core) && GENERATION.load(Ordering::Relaxed) == generation;
    if stall {
        // `core` waits in a RAM loop with masked interrupts. The stall stops `core` in that loop.
        unsafe { internal_park_core(core, true) };
    }
    drop(claim);
    unsafe { ipc::resume_context_switch() };
    stall
}

/// Releases a park that [`park`] took. `core` runs again if no other park holds `core`.
///
/// Does nothing if no park holds `core`. A start of the APP core ends all parks of the APP core.
#[ram]
pub(crate) fn unpark(core: Cpu) {
    let _claim = Claim::take();
    let count = &COUNT[core as usize];
    let previous = count.load(Ordering::Relaxed);
    if previous == 0 {
        return;
    }
    count.store(previous - 1, Ordering::Relaxed);
    if previous == 1 {
        PHASE[core as usize].store(Phase::Released as u8, Ordering::Release);
        unsafe { internal_park_core(core, false) };
    }
}

/// Parks the APP core until [`super::CpuControl`] starts it again.
///
/// The parks that hold the APP core keep working. The APP core stays parked after the last park
/// ends.
#[ram]
pub(crate) fn stop_app_core() {
    let _suspend = SchedulerSuspendGuard::new();
    if Cpu::current() == Cpu::AppCpu {
        stop_current_core();
    }

    // A park that never ends keeps the core stalled after the other parks end.
    if park(Cpu::AppCpu) {
        let claim = Claim::take();
        APP_CORE_TAKES_IPC.store(false, Ordering::SeqCst);
        drop(claim);
        // Posts to the APP core return `Offline` after the store.
        unsafe { ipc::fail_stranded(Cpu::AppCpu) };
    }
}

/// Stalls the current core, which must be the APP core, until [`super::CpuControl`] starts it
/// again.
#[ram]
pub(crate) fn stop_current_core() -> ! {
    // The interrupts stay masked after the claim ends. The APP core runs no code of this program
    // again.
    let _mask = unsafe { SingleCoreInterruptLock.enter() };
    debug_assert_eq!(Cpu::current(), Cpu::AppCpu);

    // The claim orders the stop against a park from the PRO core. That park ends before the stop,
    // or waits until the stall below takes effect.
    let claim = Claim::take();
    APP_CORE_STOPPING.store(true, Ordering::SeqCst);
    APP_CORE_TAKES_IPC.store(false, Ordering::SeqCst);
    drop(claim);
    // Posts to the APP core return `Offline` after the store.
    unsafe { ipc::fail_stranded(Cpu::AppCpu) };

    loop {
        unsafe { internal_park_core(Cpu::AppCpu, true) };
    }
}

/// Resets the park state of the APP core before the APP core starts.
pub(crate) fn reset_app_core() {
    let _claim = Claim::take();
    GENERATION.fetch_add(1, Ordering::Release);
    COUNT[Cpu::AppCpu as usize].store(0, Ordering::Relaxed);
    PHASE[Cpu::AppCpu as usize].store(Phase::Idle as u8, Ordering::Relaxed);
}

/// The park request of each target core.
static SERVE: [IpcRequest; Cpu::COUNT] = [const { IpcRequest::new(serve_ipc) }; Cpu::COUNT];

#[ram(unstable(checked))]
extern "C" fn serve_ipc() {
    serve(Cpu::current());
}

/// Parks the current core if the other core asks for a park. Returns after the other core
/// releases the current core.
///
/// The IPC function of an answered request finds a different phase, and does nothing.
///
/// The current core runs this loop until the stall takes effect. The other core can suspend the
/// cache before that.
#[ram(unstable(checked))]
fn serve(current: Cpu) {
    let phase = &PHASE[current as usize];
    if phase.load(Ordering::Acquire) != Phase::Requested as u8 {
        return;
    }

    // The other core can turn off the cache after it reads `Parked`. The mask and the branch
    // predictor change must thus come before `Parked`, and end after `Idle`.
    let token = unsafe { SingleCoreInterruptLock.enter() };
    {
        #[cfg(soc_cpu_has_branch_predictor)]
        let _predictor = crate::soc::branch_predictor::DisabledBranchPredictor::new();

        phase.store(Phase::Parked as u8, Ordering::Release);
        while phase.load(Ordering::Acquire) != Phase::Released as u8 {
            core::hint::spin_loop();
        }
        phase.store(Phase::Idle as u8, Ordering::Release);
    }
    unsafe { SingleCoreInterruptLock.exit(token) };
}

/// The [`CLAIM`] of the current core.
///
/// The current core keeps its interrupts masked while the core holds the claim. An interrupt
/// handler or a context switch thus cannot stop a change of the park state.
struct Claim(RestoreState);

impl Claim {
    #[inline(always)]
    fn take() -> Self {
        loop {
            let token = unsafe { SingleCoreInterruptLock.enter() };
            // The thread can move to the other core while the interrupts are on.
            let current = Cpu::current();
            match CLAIM.compare_exchange(
                NO_CLAIM,
                current as u8,
                Ordering::Acquire,
                Ordering::Relaxed,
            ) {
                // The other core sets `Requested` only while it holds the claim.
                Ok(_)
                    if PHASE[current as usize].load(Ordering::Acquire)
                        != Phase::Requested as u8 =>
                {
                    return Self(token);
                }
                // The other core needs the claim to stall the current core.
                Ok(_) => CLAIM.store(NO_CLAIM, Ordering::Release),
                Err(owner) => {
                    debug_assert_ne!(owner, current as u8, "The claim cannot nest")
                }
            }
            serve(current);
            unsafe { SingleCoreInterruptLock.exit(token) };
            core::hint::spin_loop();
        }
    }
}

impl Drop for Claim {
    #[inline(always)]
    fn drop(&mut self) {
        CLAIM.store(NO_CLAIM, Ordering::Release);
        unsafe { SingleCoreInterruptLock.exit(self.0) };
    }
}

// esp-hal copies without the `rt` feature call these functions by name. The names, the argument
// types and their meaning must stay stable.

#[ram]
#[unsafe(export_name = "esp_hal_park_request")]
extern "C" fn export_park_request(core: u8, generation: &mut u32) -> u8 {
    match park_request((core != NO_CORE).then(|| cpu_from_u8(core))) {
        Some((core, parked_generation)) => {
            *generation = parked_generation;
            core as u8
        }
        None => NO_CORE,
    }
}

#[ram]
#[unsafe(export_name = "esp_hal_park_stall")]
extern "C" fn export_park_stall(core: u8, generation: u32) -> bool {
    park_stall(cpu_from_u8(core), generation)
}

#[ram]
#[unsafe(export_name = "esp_hal_unpark_core")]
extern "C" fn export_unpark_core(core: u8) {
    unpark(cpu_from_u8(core))
}

#[ram]
#[unsafe(export_name = "esp_hal_stop_app_core")]
extern "C" fn export_stop_app_core() {
    stop_app_core()
}
