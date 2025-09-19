//! Syncronization primitives for ESP32 devices
//!
//! ## Feature Flags
#![doc = document_features::document_features!(feature_label = r#"<span class="stab portability"><code>{feature}</code></span>"#)]
#![doc(html_logo_url = "https://avatars.githubusercontent.com/u/46717278")]
#![cfg_attr(xtensa, feature(asm_experimental_arch))]
#![deny(missing_docs, rust_2018_idioms, rustdoc::all)]
// Don't trip up on broken/private links when running semver-checks
#![cfg_attr(
    semver_checks,
    allow(rustdoc::private_intra_doc_links, rustdoc::broken_intra_doc_links)
)]
#![no_std]

// MUST be the first module
mod fmt;

use core::{cell::UnsafeCell, marker::PhantomData};

pub mod raw;

use raw::{RawLock, SingleCoreInterruptLock};

/// Opaque token that can be used to release a lock.
// The interpretation of this value depends on the lock type that created it,
// but bit #31 is reserved for the reentry flag.
//
// Xtensa: PS has 15 useful bits. Bits 12..16 and 19..32 are unused, so we can
// use bit #31 as our reentry flag.
// We can assume the reserved bit is 0 otherwise rsil - wsr pairings would be
// undefined behavior: Quoting the ISA summary, table 64:
// Writing a non-zero value to these fields results in undefined processor
// behavior.
//
// Risc-V: we either get the restore state from bit 3 of mstatus, or
// we create the restore state from the current Priority, which is at most 31.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RestoreState(u32, PhantomData<*const ()>);

impl RestoreState {
    const REENTRY_FLAG: u32 = 1 << 31;

    /// Creates a new RestoreState from a raw inner state.
    ///
    /// # Safety
    ///
    /// The `inner` value must be appropriate for the [RawMutex] implementation that creates it.
    pub const unsafe fn new(inner: u32) -> Self {
        Self(inner, PhantomData)
    }

    /// Returns an invalid RestoreState.
    ///
    /// Note that due to the safety contract of [`RawLock::enter`]/[`RawLock::exit`], you must not
    /// pass a `RestoreState` obtained from this method to [`RawLock::exit`].
    pub const fn invalid() -> Self {
        Self(0, PhantomData)
    }

    fn mark_reentry(&mut self) {
        self.0 |= Self::REENTRY_FLAG;
    }

    fn is_reentry(self) -> bool {
        self.0 & Self::REENTRY_FLAG != 0
    }

    /// Returns the raw value used to create this RestoreState.
    pub fn inner(self) -> u32 {
        self.0
    }
}

#[cfg(single_core)]
mod single_core {
    use core::cell::Cell;

    #[repr(transparent)]
    pub(super) struct LockedState {
        locked: Cell<bool>,
    }

    impl LockedState {
        pub const fn new() -> Self {
            Self {
                locked: Cell::new(false),
            }
        }

        pub fn lock(&self, lock: &impl crate::RawLock) -> crate::RestoreState {
            let mut tkn = unsafe { lock.enter() };
            let was_locked = self.locked.replace(true);
            if was_locked {
                tkn.mark_reentry();
            }
            tkn
        }

        /// # Safety:
        ///
        /// This function must only be called if the lock was acquired by the
        /// current thread.
        pub unsafe fn unlock(&self) {
            self.locked.set(false)
        }
    }
}

#[cfg(multi_core)]
mod multi_core {
    use core::sync::atomic::{AtomicUsize, Ordering};

    // Safety: Ensure that when adding new chips `raw_core` doesn't return this
    // value.
    const UNUSED_THREAD_ID_VALUE: usize = 0x100;

    fn thread_id() -> usize {
        // This method must never return UNUSED_THREAD_ID_VALUE
        cfg_if::cfg_if! {
            if #[cfg(all(multi_core, riscv))] {
                riscv::register::mhartid::read()
            } else if #[cfg(all(multi_core, xtensa))] {
                (xtensa_lx::get_processor_id() & 0x2000) as usize
            } else {
                0
            }
        }
    }

    #[repr(transparent)]
    pub(super) struct LockedState {
        owner: AtomicUsize,
    }

    impl LockedState {
        pub const fn new() -> Self {
            Self {
                owner: AtomicUsize::new(UNUSED_THREAD_ID_VALUE),
            }
        }

        fn is_owned_by(&self, thread: usize) -> bool {
            self.owner.load(Ordering::Relaxed) == thread
        }

        pub fn lock(&self, lock: &impl crate::RawLock) -> crate::RestoreState {
            // We acquire the lock inside an interrupt-free context to prevent a subtle
            // race condition:
            // In case an interrupt handler tries to lock the same resource, it could win if
            // the current thread is holding the lock but isn't yet in interrupt-free context.
            // If we maintain non-reentrant semantics, this situation would panic.
            // If we allow reentrancy, the interrupt handler would technically be a different
            // context with the same `current_thread_id`, so it would be allowed to lock the
            // resource in a theoretically incorrect way.
            let try_lock = || {
                let mut tkn = unsafe { lock.enter() };

                let current_thread_id = thread_id();

                let try_lock_result = self
                    .owner
                    .compare_exchange(
                        UNUSED_THREAD_ID_VALUE,
                        current_thread_id,
                        Ordering::Acquire,
                        Ordering::Relaxed,
                    )
                    .map(|_| ());

                match try_lock_result {
                    Ok(()) => Some(tkn),
                    Err(owner) if owner == current_thread_id => {
                        tkn.mark_reentry();
                        Some(tkn)
                    }
                    Err(_) => {
                        unsafe { lock.exit(tkn) };
                        None
                    }
                }
            };

            loop {
                if let Some(token) = try_lock() {
                    return token;
                }
            }
        }

        /// # Safety:
        ///
        /// This function must only be called if the lock was acquired by the
        /// current thread.
        pub unsafe fn unlock(&self) {
            debug_assert!(
                self.is_owned_by(thread_id()),
                "tried to unlock a mutex locked on a different thread"
            );
            self.owner.store(UNUSED_THREAD_ID_VALUE, Ordering::Release);
        }
    }
}

#[cfg(multi_core)]
use multi_core::LockedState;
#[cfg(single_core)]
use single_core::LockedState;

/// A generic lock that wraps a [`RawLock`] implementation and tracks
/// whether the caller has locked recursively.
pub struct GenericRawMutex<L: RawLock> {
    lock: L,
    inner: LockedState,
}

// Safety: LockedState ensures thread-safety
unsafe impl<L: RawLock> Sync for GenericRawMutex<L> {}

impl<L: RawLock> GenericRawMutex<L> {
    /// Create a new lock.
    pub const fn new(lock: L) -> Self {
        Self {
            lock,
            inner: LockedState::new(),
        }
    }

    /// Acquires the lock.
    ///
    /// # Safety
    ///
    /// - Each release call must be paired with an acquire call.
    /// - The returned token must be passed to the corresponding `release` call.
    /// - The caller must ensure to release the locks in the reverse order they were acquired.
    unsafe fn acquire(&self) -> RestoreState {
        self.inner.lock(&self.lock)
    }

    /// Releases the lock.
    ///
    /// # Safety
    ///
    /// - This function must only be called if the lock was acquired by the current thread.
    /// - The caller must ensure to release the locks in the reverse order they were acquired.
    /// - Each release call must be paired with an acquire call.
    unsafe fn release(&self, token: RestoreState) {
        unsafe {
            if !token.is_reentry() {
                self.inner.unlock();

                self.lock.exit(token)
            }
        }
    }

    /// Runs the callback with this lock locked.
    ///
    /// Note that this function is not reentrant, calling it reentrantly will
    /// panic.
    pub fn lock_non_reentrant<R>(&self, f: impl FnOnce() -> R) -> R {
        let _token = LockGuard::new_non_reentrant(self);
        f()
    }

    /// Runs the callback with this lock locked.
    pub fn lock<R>(&self, f: impl FnOnce() -> R) -> R {
        let _token = LockGuard::new_reentrant(self);
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
    inner: GenericRawMutex<SingleCoreInterruptLock>,
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
            inner: GenericRawMutex::new(SingleCoreInterruptLock),
        }
    }

    /// Acquires the lock.
    ///
    /// # Safety
    ///
    /// - Each release call must be paired with an acquire call.
    /// - The returned token must be passed to the corresponding `release` call.
    /// - The caller must ensure to release the locks in the reverse order they were acquired.
    pub unsafe fn acquire(&self) -> RestoreState {
        unsafe { self.inner.acquire() }
    }

    /// Releases the lock.
    ///
    /// # Safety
    ///
    /// - This function must only be called if the lock was acquired by the current thread.
    /// - The caller must ensure to release the locks in the reverse order they were acquired.
    /// - Each release call must be paired with an acquire call.
    pub unsafe fn release(&self, token: RestoreState) {
        unsafe {
            self.inner.release(token);
        }
    }

    /// Runs the callback with this lock locked.
    ///
    /// Note that this function is not reentrant, calling it reentrantly will
    /// panic.
    pub fn lock_non_reentrant<R>(&self, f: impl FnOnce() -> R) -> R {
        self.inner.lock_non_reentrant(f)
    }

    /// Runs the callback with this lock locked.
    pub fn lock<R>(&self, f: impl FnOnce() -> R) -> R {
        self.inner.lock(f)
    }
}

unsafe impl embassy_sync_06::blocking_mutex::raw::RawMutex for RawMutex {
    #[allow(clippy::declare_interior_mutable_const)]
    const INIT: Self = Self::new();

    fn lock<R>(&self, f: impl FnOnce() -> R) -> R {
        self.inner.lock(f)
    }
}

unsafe impl embassy_sync_07::blocking_mutex::raw::RawMutex for RawMutex {
    #[allow(clippy::declare_interior_mutable_const)]
    const INIT: Self = Self::new();

    fn lock<R>(&self, f: impl FnOnce() -> R) -> R {
        self.inner.lock(f)
    }
}

/// A non-reentrant (panicking) mutex.
///
/// This is largely equivalent to a `critical_section::Mutex<RefCell<T>>`, but accessing the inner
/// data doesn't hold a critical section on multi-core systems.
pub struct NonReentrantMutex<T> {
    lock_state: RawMutex,
    data: UnsafeCell<T>,
}

impl<T> NonReentrantMutex<T> {
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
        self.lock_state
            .lock_non_reentrant(|| f(unsafe { &mut *self.data.get() }))
    }
}

unsafe impl<T: Send> Send for NonReentrantMutex<T> {}
unsafe impl<T: Send> Sync for NonReentrantMutex<T> {}

struct LockGuard<'a, L: RawLock> {
    lock: &'a GenericRawMutex<L>,
    token: RestoreState,
}

impl<'a, L: RawLock> LockGuard<'a, L> {
    fn new_non_reentrant(lock: &'a GenericRawMutex<L>) -> Self {
        let this = Self::new_reentrant(lock);
        assert!(!this.token.is_reentry(), "lock is not reentrant");
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

impl<L: RawLock> Drop for LockGuard<'_, L> {
    fn drop(&mut self) {
        unsafe { self.lock.release(self.token) };
    }
}
