//! Parks a core at a point where it holds no lock.
//!
//! A hardware stall freezes a core wherever it stands, which can be inside a critical section or
//! a cache operation. The other core then waits for ever for what the frozen core holds. This
//! module therefore asks the core to park itself first: the core takes a request in its IPC
//! interrupt, masks its interrupts, and waits in a RAM loop. Only then does the requesting core
//! stall it. The IPC interrupt runs at the lowest priority, so a core that takes it holds no lock
//! that a critical section guards.
//!
//! A core that waits for a park answers the parks of the other core, so a park request never
//! waits for a core that waits for it. A core that requests a park must not hold a lock that the
//! other core can wait for with its interrupts masked, because that core would never answer.
//!
//! Every park adds one to the count of the parked core, and every [`unpark`] removes one. The core
//! runs again when the count reaches zero.

use esp_sync::{
    RestoreState,
    raw::{RawLock, SingleCoreInterruptLock},
};
use portable_atomic::{AtomicU8, AtomicUsize, Ordering};
use portable_atomic_enum::atomic_enum;

use super::APP_CORE_TAKES_IPC;
use crate::{interrupt::ipc, ram, soc::cpu_control::internal_park_core, system::Cpu};

#[atomic_enum]
#[derive(PartialEq, Debug, Clone, Copy)]
enum Phase {
    /// The core runs, or a park stalled it in [`serve`].
    Idle,
    /// The other core asks this core to park.
    Requested,
    /// The core masked its interrupts and waits in [`serve`].
    Parked,
    /// The other core lets this core leave [`serve`].
    Released,
}

/// The park phase of each core. Only the other core writes `Requested` and `Released`, and only
/// the core itself writes `Parked` and `Idle`.
static PHASE: [AtomicPhase; Cpu::COUNT] = [const { AtomicPhase::new(Phase::Idle) }; Cpu::COUNT];

/// The number of parks that hold each core.
static COUNT: [AtomicUsize; Cpu::COUNT] = [const { AtomicUsize::new(0) }; Cpu::COUNT];

const NO_CLAIM: u8 = u8::MAX;

/// The core that parks, unparks or stops the other core, or [`NO_CLAIM`].
///
/// The claim makes sure that two cores never park each other at the same time, which would stall
/// both.
static CLAIM: AtomicU8 = AtomicU8::new(NO_CLAIM);

/// Parks `core`, and returns whether the call took a park that [`unpark`] must release.
///
/// Returns `false` if `core` runs no code of this program, which leaves nothing to park.
///
/// # Panics
///
/// Panics if `core` is the current core.
#[ram]
pub(crate) fn park(core: Cpu) -> bool {
    assert_ne!(core, Cpu::current(), "A core cannot park itself");

    let _claim = Claim::take();
    let count = &COUNT[core as usize];
    if count.load(Ordering::Relaxed) == 0 {
        if !takes_ipc(core) {
            return false;
        }
        request(core);
    }
    count.fetch_add(1, Ordering::Relaxed);
    true
}

/// Releases a park that [`park`] took. `core` runs again if no other park holds it.
#[ram]
pub(crate) fn unpark(core: Cpu) {
    let _claim = Claim::take();
    let previous = COUNT[core as usize].fetch_sub(1, Ordering::Relaxed);
    debug_assert_ne!(previous, 0, "The core is not parked");
    if previous == 1 {
        let phase = &PHASE[core as usize];
        phase.store(Phase::Released, Ordering::Release);
        unsafe { internal_park_core(core, false) };
        while phase.load(Ordering::Acquire) != Phase::Idle {
            core::hint::spin_loop();
        }
    }
}

/// Parks the APP core for good.
///
/// The parks that hold the core keep working, and the core stays parked after the last of them
/// ends. [`super::CpuControl`] can start the core again.
#[ram]
pub(crate) fn stop_app_core() {
    if Cpu::current() == Cpu::AppCpu {
        stop_current_core();
    }

    let _claim = Claim::take();
    let count = &COUNT[Cpu::AppCpu as usize];
    if count.load(Ordering::Relaxed) == 0 {
        if !takes_ipc(Cpu::AppCpu) {
            return;
        }
        request(Cpu::AppCpu);
    }
    // A park that never ends keeps the core stalled after the other parks end.
    count.fetch_add(1, Ordering::Relaxed);
    APP_CORE_TAKES_IPC.store(false, Ordering::Release);
}

/// Stalls the current core, which must be the APP core, for good.
#[ram]
pub(crate) fn stop_current_core() -> ! {
    debug_assert_eq!(Cpu::current(), Cpu::AppCpu);

    // The interrupts stay masked after the claim ends, because this core never runs code of this
    // program again.
    let _mask = unsafe { SingleCoreInterruptLock.enter() };

    // The claim orders this against a park from the other core. That park either finished
    // before this, or sees that this core runs no code of this program.
    let claim = Claim::take();
    APP_CORE_TAKES_IPC.store(false, Ordering::Release);
    drop(claim);

    loop {
        unsafe { internal_park_core(Cpu::AppCpu, true) };
    }
}

/// Resets the park state of the APP core, which is not running, before it starts.
pub(crate) fn reset_app_core() {
    COUNT[Cpu::AppCpu as usize].store(0, Ordering::Relaxed);
    PHASE[Cpu::AppCpu as usize].store(Phase::Idle, Ordering::Relaxed);
}

/// Returns whether `core` runs code of this program, and therefore takes park requests.
#[inline(always)]
fn takes_ipc(core: Cpu) -> bool {
    core == Cpu::ProCpu || APP_CORE_TAKES_IPC.load(Ordering::Acquire)
}

/// Asks `core` to park, waits until it does, then stalls it.
///
/// The caller holds the claim.
#[inline(always)]
fn request(core: Cpu) {
    let phase = &PHASE[core as usize];
    debug_assert!(phase.load(Ordering::Relaxed) == Phase::Idle);
    phase.store(Phase::Requested, Ordering::Release);

    // `core` can hold a different IPC function. It can also answer this request without the IPC
    // interrupt, while it waits for the claim, and then never take that function. The post
    // therefore must not wait.
    let mut posted = false;
    while phase.load(Ordering::Acquire) != Phase::Parked {
        if !posted {
            posted = ipc::try_call_function(core, serve_ipc);
        }
        core::hint::spin_loop();
    }

    // `core` waits in a RAM loop with its interrupts masked, so the stall freezes it there.
    unsafe { internal_park_core(core, true) };
}

#[ram]
fn serve_ipc() {
    serve(Cpu::current());
}

/// Parks the current core if the other core asks for it, and returns when that core releases it.
///
/// A request that the core already answered finds the phase moved on, so the IPC function of an
/// answered request does nothing.
#[ram]
fn serve(current: Cpu) {
    let phase = &PHASE[current as usize];
    if phase.load(Ordering::Acquire) != Phase::Requested {
        return;
    }

    // The mask and the branch predictor change come before `Parked`, because the other core can
    // turn the cache off right after it reads `Parked`. Both end after `Idle`.
    let token = unsafe { SingleCoreInterruptLock.enter() };
    {
        #[cfg(soc_cpu_has_branch_predictor)]
        let _predictor = crate::soc::branch_predictor::DisabledBranchPredictor::new();

        phase.store(Phase::Parked, Ordering::Release);
        while phase.load(Ordering::Acquire) != Phase::Released {
            core::hint::spin_loop();
        }
        phase.store(Phase::Idle, Ordering::Release);
    }
    unsafe { SingleCoreInterruptLock.exit(token) };
}

/// The [`CLAIM`] of the current core.
///
/// The current core keeps its interrupts masked while it holds the claim, so that an interrupt
/// handler on this core cannot park while the core parks.
struct Claim(RestoreState);

impl Claim {
    #[inline(always)]
    fn take() -> Self {
        let current = Cpu::current() as u8;
        loop {
            let token = unsafe { SingleCoreInterruptLock.enter() };
            match CLAIM.compare_exchange(NO_CLAIM, current, Ordering::Acquire, Ordering::Relaxed) {
                Ok(_) => return Self(token),
                Err(owner) => {
                    debug_assert_ne!(owner, current, "A park cannot nest in a park");
                    // The other core holds the claim, and can wait for this core to park.
                    serve(Cpu::current());
                    unsafe { SingleCoreInterruptLock.exit(token) };
                    core::hint::spin_loop();
                }
            }
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
