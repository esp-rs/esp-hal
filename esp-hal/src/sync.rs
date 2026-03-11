//! Under construction: This is public only for tests, please avoid using it
//! directly.

use core::sync::atomic::{Ordering, compiler_fence};

#[cfg(riscv)]
use esp_sync::raw::SingleCoreInterruptLock;
use esp_sync::{GenericRawMutex, RestoreState, raw::RawLock};

use crate::interrupt::{Priority, RunLevel};

/// A lock that disables interrupts below a certain priority.
pub struct PriorityLock(pub RunLevel);

impl PriorityLock {
    fn current_priority() -> RunLevel {
        RunLevel::current()
    }

    /// Prevents interrupts above `level` from firing and returns the
    /// current run level.
    unsafe fn change_current_level(level: RunLevel) -> RunLevel {
        unsafe { RunLevel::change(level) }
    }
}

impl RawLock for PriorityLock {
    unsafe fn enter(&self) -> RestoreState {
        #[cfg(riscv)]
        if self.0 == Priority::max() {
            return unsafe { SingleCoreInterruptLock.enter() };
        }

        let prev_level = unsafe { Self::change_current_level(self.0) };
        assert!(prev_level <= self.0);

        // Ensure no subsequent memory accesses are reordered to before interrupts are
        // disabled.
        compiler_fence(Ordering::SeqCst);

        unsafe { RestoreState::new(u32::from(prev_level)) }
    }

    unsafe fn exit(&self, token: RestoreState) {
        #[cfg(riscv)]
        if self.0 == Priority::max() {
            return unsafe { SingleCoreInterruptLock.exit(token) };
        }
        assert!(Self::current_priority() <= self.0);
        // Ensure no preceeding memory accesses are reordered to after interrupts are
        // enabled.
        compiler_fence(Ordering::SeqCst);

        let level = unwrap!(RunLevel::try_from_u32(token.inner()));
        unsafe { Self::change_current_level(level) };
    }
}

/// A mutual exclusion primitive that only disables a limited range of
/// interrupts.
///
/// Trying to acquire or release the lock at a higher priority level will panic.
pub struct RawPriorityLimitedMutex {
    inner: GenericRawMutex<PriorityLock>,
}

impl RawPriorityLimitedMutex {
    /// Create a new lock that is accessible at or below the given `priority`.
    pub const fn new(priority: Priority) -> Self {
        Self {
            inner: GenericRawMutex::new(PriorityLock(RunLevel::Interrupt(priority))),
        }
    }

    /// Runs the callback with this lock locked.
    pub fn lock<R>(&self, f: impl FnOnce() -> R) -> R {
        self.inner.lock(f)
    }
}

unsafe impl embassy_sync::blocking_mutex::raw::RawMutex for RawPriorityLimitedMutex {
    #[allow(clippy::declare_interior_mutable_const)]
    const INIT: Self = Self::new(Priority::max());

    fn lock<R>(&self, f: impl FnOnce() -> R) -> R {
        self.inner.lock(f)
    }
}

#[cfg(impl_critical_section)]
#[cfg(feature = "rt")]
mod critical_section {
    struct CriticalSection;

    critical_section::set_impl!(CriticalSection);

    static CRITICAL_SECTION: esp_sync::RawMutex = esp_sync::RawMutex::new();

    unsafe impl critical_section::Impl for CriticalSection {
        unsafe fn acquire() -> critical_section::RawRestoreState {
            unsafe { CRITICAL_SECTION.acquire().inner() }
        }

        unsafe fn release(token: critical_section::RawRestoreState) {
            unsafe {
                CRITICAL_SECTION.release(esp_sync::RestoreState::new(token));
            }
        }
    }
}
