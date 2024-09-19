struct CriticalSection;

critical_section::set_impl!(CriticalSection);

unsafe impl critical_section::Impl for CriticalSection {
    unsafe fn acquire() -> critical_section::RawRestoreState {
        #[cfg_attr(single_core, allow(unused_mut))]
        let mut tkn = single_core::disable_interrupts();

        #[cfg(multi_core)]
        {
            use multicore::{LockKind, MULTICORE_LOCK, REENTRY_FLAG};

            // FIXME: don't spin with interrupts disabled
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
            use multicore::{MULTICORE_LOCK, REENTRY_FLAG};

            debug_assert!(MULTICORE_LOCK.is_owned_by_current_thread());

            if token & REENTRY_FLAG != 0 {
                return;
            }

            MULTICORE_LOCK.unlock();
        }

        single_core::reenable_interrupts(token);
    }
}

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
                panic!()
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
                const RESERVED_MASK: u32 = 0b1111_1111_1111_1000_1111_0000_0000_0000;
                debug_assert!(token & RESERVED_MASK == 0);
                core::arch::asm!(
                            "wsr.ps {0}",
                            "rsync", in(reg) token)
            } else {
                panic!()
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
            pub const REENTRY_FLAG: u32 = 1 << 31;
        }
    }

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
                // We acquire the lock inside an interrupt-free context to prevent a subtle
                // race condition:
                // In case an interrupt handler tries to lock the same resource, it could win if
                // the current thread is holding the lock but isn't yet in interrupt-free context.
                // If we maintain non-reentrant semantics, this situation would panic.
                // If we allow reentrancy, the interrupt handler would technically be a different
                // context with the same `current_thread_id`, so it would be allowed to lock the
                // resource in a theoretically incorrect way.
                let func = || {
                    // Only use `try_lock` here so that we don't spin in interrupt-free context.
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
