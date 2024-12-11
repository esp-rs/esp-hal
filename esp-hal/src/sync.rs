//! Under construction: This is public only for tests, please avoid using it
//! directly.

#[cfg(single_core)]
use core::cell::Cell;
use core::cell::UnsafeCell;

use crate::interrupt::Priority;

mod single_core {
    use core::sync::atomic::{compiler_fence, Ordering};

    use crate::interrupt::Priority;

    /// Trait for single-core locks.
    pub trait RawLock {
        unsafe fn enter(&self) -> critical_section::RawRestoreState;
        unsafe fn exit(&self, token: critical_section::RawRestoreState);
    }

    /// A lock that disables interrupts below a certain priority.
    pub struct PriorityLock(pub Priority);

    impl PriorityLock {
        fn current_priority() -> Priority {
            crate::interrupt::current_runlevel()
        }

        /// Prevents interrupts above `level` from firing and returns the
        /// current run level.
        unsafe fn change_current_level(level: Priority) -> Priority {
            crate::interrupt::change_current_runlevel(level)
        }
    }

    impl RawLock for PriorityLock {
        unsafe fn enter(&self) -> critical_section::RawRestoreState {
            let prev_interrupt_priority = unsafe { Self::change_current_level(self.0) };
            assert!(prev_interrupt_priority <= self.0);

            // Ensure no subsequent memory accesses are reordered to before interrupts are
            // disabled.
            compiler_fence(Ordering::SeqCst);

            prev_interrupt_priority as _
        }

        unsafe fn exit(&self, token: critical_section::RawRestoreState) {
            assert!(Self::current_priority() <= self.0);
            // Ensure no preceeding memory accesses are reordered to after interrupts are
            // enabled.
            compiler_fence(Ordering::SeqCst);

            #[cfg(xtensa)]
            let token = token as u8;

            let priority = unwrap!(Priority::try_from(token));
            unsafe { Self::change_current_level(priority) };
        }
    }

    /// A lock that disables interrupts.
    pub struct InterruptLock;

    impl RawLock for InterruptLock {
        unsafe fn enter(&self) -> critical_section::RawRestoreState {
            cfg_if::cfg_if! {
                if #[cfg(riscv)] {
                    let mut mstatus = 0u32;
                    core::arch::asm!("csrrci {0}, mstatus, 8", inout(reg) mstatus);
                    let token = ((mstatus & 0b1000) != 0) as critical_section::RawRestoreState;
                } else if #[cfg(xtensa)] {
                    let token: critical_section::RawRestoreState;
                    core::arch::asm!("rsil {0}, 5", out(reg) token);
                } else {
                    compile_error!("Unsupported architecture")
                }
            };

            // Ensure no subsequent memory accesses are reordered to before interrupts are
            // disabled.
            compiler_fence(Ordering::SeqCst);

            token
        }

        unsafe fn exit(&self, token: critical_section::RawRestoreState) {
            // Ensure no preceeding memory accesses are reordered to after interrupts are
            // enabled.
            compiler_fence(Ordering::SeqCst);

            cfg_if::cfg_if! {
                if #[cfg(riscv)] {
                    if token != 0 {
                        esp_riscv_rt::riscv::interrupt::enable();
                    }
                } else if #[cfg(xtensa)] {
                    // Reserved bits in the PS register, these must be written as 0.
                    const RESERVED_MASK: u32 = 0b1111_1111_1111_1000_1111_0000_0000_0000;
                    debug_assert!(token & RESERVED_MASK == 0);
                    core::arch::asm!(
                        "wsr.ps {0}",
                        "rsync", in(reg) token)
                } else {
                    compile_error!("Unsupported architecture")
                }
            }
        }
    }
}

#[cfg(multi_core)]
mod multicore {
    use portable_atomic::{AtomicUsize, Ordering};

    // Safety: Ensure that when adding new chips `raw_core` doesn't return this
    // value.
    // FIXME: ensure in HIL tests this is the case!
    const UNUSED_THREAD_ID_VALUE: usize = 0x100;

    pub fn thread_id() -> usize {
        crate::raw_core()
    }

    pub(super) struct AtomicLock {
        owner: AtomicUsize,
    }

    impl AtomicLock {
        pub const fn new() -> Self {
            Self {
                owner: AtomicUsize::new(UNUSED_THREAD_ID_VALUE),
            }
        }

        pub fn is_owned_by_current_thread(&self) -> bool {
            self.is_owned_by(thread_id())
        }

        pub fn is_owned_by(&self, thread: usize) -> bool {
            self.owner.load(Ordering::Relaxed) == thread
        }

        pub fn try_lock(&self, new_owner: usize) -> Result<(), usize> {
            self.owner
                .compare_exchange(
                    UNUSED_THREAD_ID_VALUE,
                    new_owner,
                    Ordering::Acquire,
                    Ordering::Relaxed,
                )
                .map(|_| ())
        }

        /// # Safety:
        ///
        /// This function must only be called if the lock was acquired by the
        /// current thread.
        pub unsafe fn unlock(&self) {
            debug_assert!(self.is_owned_by_current_thread());
            self.owner.store(UNUSED_THREAD_ID_VALUE, Ordering::Release);
        }
    }
}

cfg_if::cfg_if! {
    if #[cfg(riscv)] {
        // The restore state is a u8 that is casted from a bool, so it has a value of
        // 0x00 or 0x01 before we add the reentry flag to it.
        pub const REENTRY_FLAG: u8 = 1 << 7;
    } else if #[cfg(xtensa)] {
        // PS has 15 useful bits. Bits 12..16 and 19..32 are unused, so we can use bit
        // #31 as our reentry flag.
        // We can assume the reserved bit is 0 otherwise rsil - wsr pairings would be
        // undefined behavior: Quoting the ISA summary, table 64:
        // Writing a non-zero value to these fields results in undefined processor behavior.
        pub const REENTRY_FLAG: u32 = 1 << 31;
    }
}

/// A generic lock that wraps [`single_core::RawLock`] and
/// [`multicore::AtomicLock`] and tracks whether the caller has locked
/// recursively.
struct GenericRawMutex<L: single_core::RawLock> {
    lock: L,
    #[cfg(multi_core)]
    inner: multicore::AtomicLock,
    #[cfg(single_core)]
    is_locked: Cell<bool>,
}

unsafe impl<L: single_core::RawLock> Sync for GenericRawMutex<L> {}

impl<L: single_core::RawLock> GenericRawMutex<L> {
    /// Create a new lock.
    pub const fn new(lock: L) -> Self {
        Self {
            lock,
            #[cfg(multi_core)]
            inner: multicore::AtomicLock::new(),
            #[cfg(single_core)]
            is_locked: Cell::new(false),
        }
    }

    /// Acquires the lock.
    ///
    /// # Safety
    ///
    /// - Each release call must be paired with an acquire call.
    /// - The returned token must be passed to the corresponding `release` call.
    /// - The caller must ensure to release the locks in the reverse order they
    ///   were acquired.
    unsafe fn acquire(&self) -> critical_section::RawRestoreState {
        cfg_if::cfg_if! {
            if #[cfg(single_core)] {
                let mut tkn = unsafe { self.lock.enter() };
                let was_locked = self.is_locked.replace(true);
                if was_locked {
                    tkn |= REENTRY_FLAG;
                }
                tkn
            } else if #[cfg(multi_core)] {
                // We acquire the lock inside an interrupt-free context to prevent a subtle
                // race condition:
                // In case an interrupt handler tries to lock the same resource, it could win if
                // the current thread is holding the lock but isn't yet in interrupt-free context.
                // If we maintain non-reentrant semantics, this situation would panic.
                // If we allow reentrancy, the interrupt handler would technically be a different
                // context with the same `current_thread_id`, so it would be allowed to lock the
                // resource in a theoretically incorrect way.
                let try_lock = |current_thread_id| {
                    let mut tkn = unsafe { self.lock.enter() };

                    match self.inner.try_lock(current_thread_id) {
                        Ok(()) => Some(tkn),
                        Err(owner) if owner == current_thread_id => {
                            tkn |= REENTRY_FLAG;
                            Some(tkn)
                        }
                        Err(_) => {
                            unsafe { self.lock.exit(tkn) };
                            None
                        }
                    }
                };

                let current_thread_id = multicore::thread_id();
                loop {
                    if let Some(token) = try_lock(current_thread_id) {
                        return token;
                    }
                }
            }
        }
    }

    /// Releases the lock.
    ///
    /// # Safety
    ///
    /// - This function must only be called if the lock was acquired by the
    ///   current thread.
    /// - The caller must ensure to release the locks in the reverse order they
    ///   were acquired.
    /// - Each release call must be paired with an acquire call.
    unsafe fn release(&self, token: critical_section::RawRestoreState) {
        if token & REENTRY_FLAG == 0 {
            #[cfg(multi_core)]
            self.inner.unlock();

            #[cfg(single_core)]
            self.is_locked.set(false);

            self.lock.exit(token)
        }
    }

    /// Runs the callback with this lock locked.
    ///
    /// Note that this function is not reentrant, calling it reentrantly will
    /// panic.
    pub fn lock<R>(&self, f: impl FnOnce() -> R) -> R {
        let _token = LockGuard::new(self);
        f()
    }
}

/// A mutual exclusion primitive.
///
/// This lock disables interrupts on the current core while locked.
#[cfg_attr(
    multi_core,
    doc = r#"It needs a bit of memory, but it does not take a global critical
    section, making it preferrable for use in multi-core systems."#
)]
pub struct RawMutex {
    inner: GenericRawMutex<single_core::InterruptLock>,
}

impl Default for RawMutex {
    fn default() -> Self {
        Self::new()
    }
}

impl RawMutex {
    /// Create a new lock.
    pub const fn new() -> Self {
        Self {
            inner: GenericRawMutex::new(single_core::InterruptLock),
        }
    }

    /// Acquires the lock.
    ///
    /// # Safety
    ///
    /// - Each release call must be paired with an acquire call.
    /// - The returned token must be passed to the corresponding `release` call.
    /// - The caller must ensure to release the locks in the reverse order they
    ///   were acquired.
    pub unsafe fn acquire(&self) -> critical_section::RawRestoreState {
        self.inner.acquire()
    }

    /// Releases the lock.
    ///
    /// # Safety
    ///
    /// - This function must only be called if the lock was acquired by the
    ///   current thread.
    /// - The caller must ensure to release the locks in the reverse order they
    ///   were acquired.
    /// - Each release call must be paired with an acquire call.
    pub unsafe fn release(&self, token: critical_section::RawRestoreState) {
        self.inner.release(token);
    }

    /// Runs the callback with this lock locked.
    ///
    /// Note that this function is not reentrant, calling it reentrantly will
    /// panic.
    pub fn lock<R>(&self, f: impl FnOnce() -> R) -> R {
        self.inner.lock(f)
    }
}

unsafe impl embassy_sync::blocking_mutex::raw::RawMutex for RawMutex {
    #[allow(clippy::declare_interior_mutable_const)]
    const INIT: Self = Self::new();

    fn lock<R>(&self, f: impl FnOnce() -> R) -> R {
        // embassy_sync semantics allow reentrancy.
        let _token = LockGuard::new_reentrant(&self.inner);
        f()
    }
}

/// A mutual exclusion primitive that only disables a limited range of
/// interrupts.
///
/// Trying to acquire or release the lock at a higher priority level will panic.
pub struct RawPriorityLimitedMutex {
    inner: GenericRawMutex<single_core::PriorityLock>,
}

impl RawPriorityLimitedMutex {
    /// Create a new lock that is accessible at or below the given `priority`.
    pub const fn new(priority: Priority) -> Self {
        Self {
            inner: GenericRawMutex::new(single_core::PriorityLock(priority)),
        }
    }

    /// Runs the callback with this lock locked.
    ///
    /// Note that this function is not reentrant, calling it reentrantly will
    /// panic.
    pub fn lock<R>(&self, f: impl FnOnce() -> R) -> R {
        self.inner.lock(f)
    }
}

unsafe impl embassy_sync::blocking_mutex::raw::RawMutex for RawPriorityLimitedMutex {
    #[allow(clippy::declare_interior_mutable_const)]
    const INIT: Self = Self::new(Priority::max());

    fn lock<R>(&self, f: impl FnOnce() -> R) -> R {
        // embassy_sync semantics allow reentrancy.
        let _token = LockGuard::new_reentrant(&self.inner);
        f()
    }
}

// Prefer this over a critical-section as this allows you to have multiple
// locks active at the same time rather than using the global mutex that is
// critical-section.
pub(crate) fn lock<T>(lock: &RawMutex, f: impl FnOnce() -> T) -> T {
    lock.lock(f)
}

/// Data protected by a [RawMutex].
///
/// This is largely equivalent to a `Mutex<RefCell<T>>`, but accessing the inner
/// data doesn't hold a critical section on multi-core systems.
pub struct Locked<T> {
    lock_state: RawMutex,
    data: UnsafeCell<T>,
}

impl<T> Locked<T> {
    /// Create a new instance
    pub const fn new(data: T) -> Self {
        Self {
            lock_state: RawMutex::new(),
            data: UnsafeCell::new(data),
        }
    }

    /// Provide exclusive access to the protected data to the given closure.
    ///
    /// Calling this reentrantly will panic.
    pub fn with<R>(&self, f: impl FnOnce(&mut T) -> R) -> R {
        lock(&self.lock_state, || f(unsafe { &mut *self.data.get() }))
    }
}

unsafe impl<T> Sync for Locked<T> {}

struct CriticalSection;

critical_section::set_impl!(CriticalSection);

static CRITICAL_SECTION: RawMutex = RawMutex::new();

unsafe impl critical_section::Impl for CriticalSection {
    unsafe fn acquire() -> critical_section::RawRestoreState {
        CRITICAL_SECTION.acquire()
    }

    unsafe fn release(token: critical_section::RawRestoreState) {
        CRITICAL_SECTION.release(token);
    }
}

struct LockGuard<'a, L: single_core::RawLock> {
    lock: &'a GenericRawMutex<L>,
    token: critical_section::RawRestoreState,
}

impl<'a, L: single_core::RawLock> LockGuard<'a, L> {
    fn new(lock: &'a GenericRawMutex<L>) -> Self {
        let this = Self::new_reentrant(lock);
        assert!(this.token & REENTRY_FLAG == 0, "lock is not reentrant");
        this
    }

    fn new_reentrant(lock: &'a GenericRawMutex<L>) -> Self {
        let token = unsafe {
            // SAFETY: the same lock will be released when dropping the guard.
            // This ensures that the lock is released on the same thread, in the reverse
            // order it was acquired.
            lock.acquire()
        };

        Self { lock, token }
    }
}

impl<L: single_core::RawLock> Drop for LockGuard<'_, L> {
    fn drop(&mut self) {
        unsafe { self.lock.release(self.token) };
    }
}
