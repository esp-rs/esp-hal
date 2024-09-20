use core::cell::UnsafeCell;

mod single_core {
    pub unsafe fn disable_interrupts() -> critical_section::RawRestoreState {
        cfg_if::cfg_if! {
            if #[cfg(riscv)] {
                let mut mstatus = 0u32;
                core::arch::asm!("csrrci {0}, mstatus, 8", inout(reg) mstatus);
                ((mstatus & 0b1000) != 0) as critical_section::RawRestoreState
            } else if #[cfg(xtensa)] {
                let token: critical_section::RawRestoreState;
                core::arch::asm!("rsil {0}, 5", out(reg) token);
                token
            } else {
                compile_error!("Unsupported architecture")
            }
        }
    }

    pub unsafe fn reenable_interrupts(token: critical_section::RawRestoreState) {
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

#[cfg(multi_core)]
mod multicore {
    use portable_atomic::{AtomicUsize, Ordering};

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

    // Safety: Ensure that when adding new chips `get_raw_core` doesn't return this
    // value.
    // FIXME: ensure in HIL tests this is the case!
    const UNUSED_THREAD_ID_VALUE: usize = 0x100;

    pub fn thread_id() -> usize {
        crate::get_raw_core()
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

pub(crate) struct Lock {
    #[cfg(multi_core)]
    inner: multicore::AtomicLock,
}

impl Lock {
    pub const fn new() -> Self {
        Self {
            #[cfg(multi_core)]
            inner: multicore::AtomicLock::new(),
        }
    }

    fn acquire(&self) -> critical_section::RawRestoreState {
        cfg_if::cfg_if! {
            if #[cfg(single_core)] {
                unsafe { single_core::disable_interrupts() }
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
                    let mut tkn = unsafe { single_core::disable_interrupts() };

                    match self.inner.try_lock(current_thread_id) {
                        Ok(()) => Some(tkn),
                        Err(owner) if owner == current_thread_id => {
                            tkn |= multicore::REENTRY_FLAG;
                            Some(tkn)
                        }
                        Err(_) => {
                            unsafe { single_core::reenable_interrupts(tkn) };
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

    /// # Safety
    /// This function must only be called if the lock was acquired by the
    /// current thread.
    unsafe fn release(&self, token: critical_section::RawRestoreState) {
        #[cfg(multi_core)]
        {
            if token & multicore::REENTRY_FLAG != 0 {
                return;
            }

            self.inner.unlock();
        }

        single_core::reenable_interrupts(token);
    }
}

// This is preferred over critical-section as this allows you to have multiple
// locks active at the same time rather than using the global mutex that is
// critical-section.
#[allow(unused_variables)]
pub(crate) fn lock<T>(lock: &Lock, f: impl FnOnce() -> T) -> T {
    // In regards to disabling interrupts, we only need to disable
    // the interrupts that may be calling this function.

    struct LockGuard<'a> {
        lock: &'a Lock,
        token: critical_section::RawRestoreState,
    }

    impl<'a> LockGuard<'a> {
        fn new(lock: &'a Lock) -> Self {
            let token = lock.acquire();

            #[cfg(multi_core)]
            assert!(
                token & multicore::REENTRY_FLAG == 0,
                "lock is not reentrant"
            );

            Self { lock, token }
        }
    }

    impl<'a> Drop for LockGuard<'a> {
        fn drop(&mut self) {
            unsafe { self.lock.release(self.token) };
        }
    }

    let _token = LockGuard::new(lock);
    f()
}

/// Data protected by [LockState]
#[allow(unused)]
pub(crate) struct Locked<T> {
    lock_state: Lock,
    data: UnsafeCell<T>,
}

#[allow(unused)]
impl<T> Locked<T> {
    /// Create a new instance
    pub(crate) const fn new(data: T) -> Self {
        Self {
            lock_state: Lock::new(),
            data: UnsafeCell::new(data),
        }
    }

    /// Provide exclusive access to the protected data to the given closure
    pub(crate) fn with<R>(&self, f: impl FnOnce(&mut T) -> R) -> R {
        lock(&self.lock_state, || f(unsafe { &mut *self.data.get() }))
    }
}

unsafe impl<T> Sync for Locked<T> {}

struct CriticalSection;

critical_section::set_impl!(CriticalSection);

static CRITICAL_SECTION: Lock = Lock::new();

unsafe impl critical_section::Impl for CriticalSection {
    unsafe fn acquire() -> critical_section::RawRestoreState {
        CRITICAL_SECTION.acquire()
    }

    unsafe fn release(token: critical_section::RawRestoreState) {
        CRITICAL_SECTION.release(token);
    }
}
