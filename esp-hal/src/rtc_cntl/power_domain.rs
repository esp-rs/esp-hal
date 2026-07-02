//! Power-domain locks for light sleep (ESP32-C6).
//!
//! An active, un-retained peripheral in a power-downable domain holds a
//! [`PowerDomainLock`]: unlike a [`WakeLock`](crate::rtc_cntl::WakeLock) it
//! doesn't prevent light sleep, only powering its domain down (which degrades to
//! clock-gating), so it can't lose state. Retaining the peripheral drops the
//! lock and lets regDMA save/restore its state around the power-down instead.

use core::sync::atomic::{AtomicU32, Ordering};

/// A power domain that can be independently powered down during light sleep.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub(crate) enum Domain {
    /// The CPU power domain (`pd_cpu`).
    Cpu = 0,
    /// The digital `TOP` power domain (`pd_top`).
    Top = 1,
}

const DOMAIN_COUNT: usize = 2;

/// Per-domain count of active, unretained peripherals holding it powered.
#[allow(clippy::declare_interior_mutable_const)]
const ZERO: AtomicU32 = AtomicU32::new(0);
static LOCKS: [AtomicU32; DOMAIN_COUNT] = [ZERO; DOMAIN_COUNT];

/// A guard that keeps `domain` powered across light sleep while held (degrading
/// to clock-gating), without preventing sleep like a `WakeLock` would.
pub(crate) struct PowerDomainLock {
    domain: Domain,
}

impl PowerDomainLock {
    /// Keep `domain` powered until the guard is dropped.
    pub(crate) fn new(domain: Domain) -> Self {
        LOCKS[domain as usize].fetch_add(1, Ordering::AcqRel);
        Self { domain }
    }
}

impl Drop for PowerDomainLock {
    fn drop(&mut self) {
        LOCKS[self.domain as usize].fetch_sub(1, Ordering::AcqRel);
    }
}

/// Whether `domain` may be powered down. On the C6 powering `TOP` down also
/// tears down the CPU domain, so `Top` requires both to be free.
pub(crate) fn can_power_down(domain: Domain) -> bool {
    let blocked = match domain {
        Domain::Cpu => LOCKS[Domain::Cpu as usize].load(Ordering::Acquire) != 0,
        Domain::Top => {
            LOCKS[Domain::Top as usize].load(Ordering::Acquire) != 0
                || LOCKS[Domain::Cpu as usize].load(Ordering::Acquire) != 0
        }
    };
    !blocked
}
