struct CriticalSection;

critical_section::set_impl!(CriticalSection);

#[cfg(xtensa)]
mod xtensa {
    // PS has 15 useful bits. Bits 12..16 and 19..32 are unused, so we can use bit
    // #31 as our reentry flag.
    #[cfg(multi_core)]
    const REENTRY_FLAG: u32 = 1 << 31;

    unsafe impl critical_section::Impl for super::CriticalSection {
        unsafe fn acquire() -> critical_section::RawRestoreState {
            let mut tkn: critical_section::RawRestoreState;
            core::arch::asm!("rsil {0}, 5", out(reg) tkn);
            #[cfg(multi_core)]
            {
                use super::multicore::{LockKind, MULTICORE_LOCK};

                match MULTICORE_LOCK.lock() {
                    LockKind::Lock => {
                        // We can assume the reserved bit is 0 otherwise
                        // rsil - wsr pairings would be undefined behavior
                    }
                    LockKind::Reentry => tkn |= REENTRY_FLAG,
                }
            }
            tkn
        }

        unsafe fn release(token: critical_section::RawRestoreState) {
            #[cfg(multi_core)]
            {
                use super::multicore::MULTICORE_LOCK;

                debug_assert!(MULTICORE_LOCK.is_owned_by_current_thread());

                if token & REENTRY_FLAG != 0 {
                    return;
                }

                MULTICORE_LOCK.unlock();
            }

            const RESERVED_MASK: u32 = 0b1111_1111_1111_1000_1111_0000_0000_0000;
            debug_assert!(token & RESERVED_MASK == 0);

            core::arch::asm!(
                    "wsr.ps {0}",
                    "rsync", in(reg) token)
        }
    }
}

#[cfg(riscv)]
mod riscv {
    // The restore state is a u8 that is casted from a bool, so it has a value of
    // 0x00 or 0x01 before we add the reentry flag to it.
    #[cfg(multi_core)]
    const REENTRY_FLAG: u8 = 1 << 7;

    unsafe impl critical_section::Impl for super::CriticalSection {
        unsafe fn acquire() -> critical_section::RawRestoreState {
            let mut mstatus = 0u32;
            core::arch::asm!("csrrci {0}, mstatus, 8", inout(reg) mstatus);

            #[cfg_attr(single_core, allow(unused_mut))]
            let mut tkn = ((mstatus & 0b1000) != 0) as critical_section::RawRestoreState;

            #[cfg(multi_core)]
            {
                use super::multicore::{LockKind, MULTICORE_LOCK};

                match MULTICORE_LOCK.lock() {
                    LockKind::Lock => {}
                    LockKind::Reentry => tkn |= REENTRY_FLAG,
                }
            }

            tkn
        }

        unsafe fn release(token: critical_section::RawRestoreState) {
            #[cfg(multi_core)]
            {
                use super::multicore::MULTICORE_LOCK;

                debug_assert!(MULTICORE_LOCK.is_owned_by_current_thread());

                if token & REENTRY_FLAG != 0 {
                    return;
                }

                MULTICORE_LOCK.unlock();
            }

            if token != 0 {
                esp_riscv_rt::riscv::interrupt::enable();
            }
        }
    }
}

#[cfg(multi_core)]
mod multicore {
    use portable_atomic::{AtomicUsize, Ordering};

    // We're using a value that we know get_raw_core() will never return. This
    // avoids an unnecessary increment of the core ID.
    //
    // Safety: Ensure that when adding new chips get_raw_core doesn't return this
    // value. TODO when we have HIL tests ensure this is the case!
    const UNUSED_THREAD_ID_VALUE: usize = 0x100;

    pub fn thread_id() -> usize {
        crate::get_raw_core()
    }

    pub(super) static MULTICORE_LOCK: ReentrantMutex = ReentrantMutex::new();

    pub(super) enum LockKind {
        Lock = 0,
        Reentry,
    }

    pub(super) struct ReentrantMutex {
        owner: AtomicUsize,
    }

    impl ReentrantMutex {
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

        pub fn lock(&self) -> LockKind {
            let current_thread_id = thread_id();

            if self.try_lock(current_thread_id) {
                return LockKind::Lock;
            }

            let current_owner = self.owner.load(Ordering::Relaxed);
            if current_owner == current_thread_id {
                return LockKind::Reentry;
            }

            while !self.try_lock(current_thread_id) {}

            LockKind::Lock
        }

        pub fn try_lock(&self, new_owner: usize) -> bool {
            self.owner
                .compare_exchange(
                    UNUSED_THREAD_ID_VALUE,
                    new_owner,
                    Ordering::Acquire,
                    Ordering::Relaxed,
                )
                .is_ok()
        }

        pub fn unlock(&self) {
            self.owner.store(UNUSED_THREAD_ID_VALUE, Ordering::Release);
        }
    }
}

// The state of a re-entrant lock
pub(crate) struct LockState {
    #[cfg(multi_core)]
    inner: multicore::ReentrantMutex,
}

impl LockState {
    pub const fn new() -> Self {
        Self {
            #[cfg(multi_core)]
            inner: multicore::ReentrantMutex::new(),
        }
    }
}

fn interrupt_free<T>(f: impl FnOnce() -> T) -> T {
    cfg_if::cfg_if! {
        if #[cfg(riscv)] {
            esp_riscv_rt::riscv::interrupt::free(f)
        } else if #[cfg(xtensa)] {
            xtensa_lx::interrupt::free(|_| f())
        } else {
            panic!()
        }
    }
}

// This is preferred over critical-section as this allows you to have multiple
// locks active at the same time rather than using the global mutex that is
// critical-section.
#[allow(unused_variables)]
pub(crate) fn lock<T>(state: &LockState, f: impl FnOnce() -> T) -> T {
    // In regards to disabling interrupts, we only need to disable
    // the interrupts that may be calling this function.

    cfg_if::cfg_if! {
        if #[cfg(multi_core)] {
            let mut f = f;
            let current_thread_id = multicore::thread_id();
            loop {
                let func = || {
                    if state.inner.try_lock(current_thread_id) {
                        let result = f();

                        state.inner.unlock();

                        Ok(result)
                    } else {
                        assert!(
                            !state.inner.is_owned_by(current_thread_id),
                            "lock is not re-entrant!"
                        );

                        Err(f)
                    }
                };

                match interrupt_free(func) {
                    Ok(result) => return result,
                    Err(the_function) => f = the_function,
                }

                // Consider using core::hint::spin_loop(); Might need SW_INT.
            }
        } else {
            // Disabling interrupts is enough on single core chips to ensure mutual
            // exclusion.
            interrupt_free(f)
        }
    }
}
