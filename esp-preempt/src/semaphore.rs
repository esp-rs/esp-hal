use alloc::boxed::Box;
use core::ptr::NonNull;

use esp_hal::time::{Duration, Instant};
use esp_radio_preempt_driver::{
    register_semaphore_implementation,
    semaphore::{SemaphoreImplementation, SemaphoreKind, SemaphorePtr},
    yield_task,
};
use esp_sync::NonReentrantMutex;

use crate::task::{TaskPtr, current_task};

enum SemaphoreInner {
    Counting {
        current: u32,
        max: u32,
    },
    RecursiveMutex {
        owner: Option<TaskPtr>,
        lock_counter: u32,
    },
}

impl SemaphoreInner {
    fn try_take(&mut self) -> bool {
        match self {
            SemaphoreInner::Counting { current, .. } => {
                if *current > 0 {
                    *current -= 1;
                    true
                } else {
                    false
                }
            }
            SemaphoreInner::RecursiveMutex {
                owner,
                lock_counter,
            } => {
                let current = current_task();
                if owner.is_none() || owner.unwrap() == current {
                    *lock_counter += 1;
                    true
                } else {
                    false
                }
            }
        }
    }

    fn try_give(&mut self) -> bool {
        match self {
            SemaphoreInner::Counting { current, max } => {
                if *current < *max {
                    *current += 1;
                    true
                } else {
                    false
                }
            }
            SemaphoreInner::RecursiveMutex {
                owner,
                lock_counter,
            } => {
                let current = current_task();

                if *owner == Some(current) && *lock_counter > 0 {
                    *lock_counter -= 1;
                    if *lock_counter == 0 {
                        *owner = None;
                    }
                    true
                } else {
                    false
                }
            }
        }
    }

    fn current_count(&mut self) -> u32 {
        match self {
            SemaphoreInner::Counting { current, .. } => *current,
            SemaphoreInner::RecursiveMutex { .. } => {
                panic!("RecursiveMutex does not support current_count")
            }
        }
    }
}

pub struct Semaphore {
    inner: NonReentrantMutex<SemaphoreInner>,
}

impl Semaphore {
    pub fn new(kind: SemaphoreKind) -> Self {
        let inner = match kind {
            SemaphoreKind::Counting { initial, max } => SemaphoreInner::Counting {
                current: initial,
                max,
            },
            SemaphoreKind::RecursiveMutex => SemaphoreInner::RecursiveMutex {
                owner: None,
                lock_counter: 0,
            },
        };
        Semaphore {
            inner: NonReentrantMutex::new(inner),
        }
    }

    unsafe fn from_ptr<'a>(ptr: SemaphorePtr) -> &'a Self {
        unsafe { ptr.cast::<Self>().as_ref() }
    }

    pub fn try_take(&self) -> bool {
        self.inner.with(|sem| sem.try_take())
    }

    fn yield_loop_with_timeout(timeout_us: Option<u32>, cb: impl Fn() -> bool) -> bool {
        let start = if timeout_us.is_some() {
            Instant::now()
        } else {
            Instant::EPOCH
        };

        let timeout = timeout_us
            .map(|us| Duration::from_micros(us as u64))
            .unwrap_or(Duration::MAX);

        loop {
            if cb() {
                return true;
            }

            if timeout_us.is_some() && start.elapsed() > timeout {
                return false;
            }

            yield_task();
        }
    }

    pub fn take(&self, timeout_us: Option<u32>) -> bool {
        Self::yield_loop_with_timeout(timeout_us, || self.try_take())
    }

    pub fn current_count(&self) -> u32 {
        self.inner.with(|sem| sem.current_count())
    }

    pub fn give(&self) -> bool {
        self.inner.with(|sem| sem.try_give())
    }
}

impl SemaphoreImplementation for Semaphore {
    fn create(kind: SemaphoreKind) -> SemaphorePtr {
        let sem = Box::new(Semaphore::new(kind));
        NonNull::from(Box::leak(sem)).cast()
    }

    unsafe fn delete(semaphore: SemaphorePtr) {
        let sem = unsafe { Box::from_raw(semaphore.cast::<Semaphore>().as_ptr()) };
        core::mem::drop(sem);
    }

    unsafe fn take(semaphore: SemaphorePtr, timeout_us: Option<u32>) -> bool {
        let semaphore = unsafe { Semaphore::from_ptr(semaphore) };

        semaphore.take(timeout_us)
    }

    unsafe fn give(semaphore: SemaphorePtr) -> bool {
        let semaphore = unsafe { Semaphore::from_ptr(semaphore) };

        semaphore.give()
    }

    unsafe fn current_count(semaphore: SemaphorePtr) -> u32 {
        let semaphore = unsafe { Semaphore::from_ptr(semaphore) };

        semaphore.current_count()
    }

    unsafe fn try_take(semaphore: SemaphorePtr) -> bool {
        let semaphore = unsafe { Semaphore::from_ptr(semaphore) };

        semaphore.try_take()
    }

    unsafe fn try_give_from_isr(semaphore: SemaphorePtr, _hptw: Option<&mut bool>) -> bool {
        unsafe { <Self as SemaphoreImplementation>::give(semaphore) }
    }

    unsafe fn try_take_from_isr(semaphore: SemaphorePtr, _hptw: Option<&mut bool>) -> bool {
        unsafe { <Self as SemaphoreImplementation>::try_take(semaphore) }
    }
}

register_semaphore_implementation!(Semaphore);
