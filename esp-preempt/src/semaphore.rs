use alloc::boxed::Box;
use core::ptr::NonNull;

use esp_hal::time::{Duration, Instant};
use esp_radio_preempt_driver::{
    register_semaphore_implementation,
    semaphore::{SemaphoreImplementation, SemaphoreKind, SemaphorePtr},
};
use esp_sync::NonReentrantMutex;

use crate::{
    SCHEDULER,
    task::{TaskPtr, WaitQueue, current_task},
};

enum SemaphoreInner {
    Counting {
        current: u32,
        max: u32,
        waiting: WaitQueue,
    },
    RecursiveMutex {
        owner: Option<TaskPtr>,
        lock_counter: u32,
        waiting: WaitQueue,
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
                ..
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
            SemaphoreInner::Counting { current, max, .. } => {
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
                ..
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

    fn push_waiting_task(&mut self, task: TaskPtr) {
        match self {
            SemaphoreInner::Counting { waiting, .. } => waiting.push(task),
            SemaphoreInner::RecursiveMutex { waiting, .. } => waiting.push(task),
        }
    }

    fn pop_waiting(&mut self) -> Option<TaskPtr> {
        match self {
            SemaphoreInner::Counting { waiting, .. } => waiting.pop(),
            SemaphoreInner::RecursiveMutex { waiting, .. } => waiting.pop(),
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
                waiting: WaitQueue::new(),
            },
            SemaphoreKind::RecursiveMutex => SemaphoreInner::RecursiveMutex {
                owner: None,
                lock_counter: 0,
                waiting: WaitQueue::new(),
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

    pub fn take(&self, timeout_us: Option<u32>) -> bool {
        let deadline = timeout_us.map(|us| Instant::now() + Duration::from_micros(us as u64));
        loop {
            let taken = self.inner.with(|sem| {
                if sem.try_take() {
                    true
                } else {
                    // The task will go to sleep when the above critical section is released.
                    SCHEDULER.with(|scheduler| {
                        sem.push_waiting_task(unwrap!(scheduler.current_task));

                        let wake_at = if let Some(deadline) = deadline {
                            deadline
                        } else {
                            Instant::EPOCH + Duration::MAX
                        };
                        scheduler.sleep_until(wake_at);
                    });
                    false
                }
            });

            if taken {
                return true;
            }

            // We are here because we weren't able to take the semaphore previously. We've either
            // timed out, or the semaphore is ready for taking. However, any higher priority task
            // can wake up and preempt us still. Let's just check for the timeout, and
            // try the whole process again.

            if let Some(deadline) = deadline
                && deadline < Instant::now()
            {
                // We have a deadline and we've timed out.
                return false;
            }
            // We can block more, so let's attempt to take the semaphore again.
        }
    }

    pub fn current_count(&self) -> u32 {
        self.inner.with(|sem| sem.current_count())
    }

    pub fn give(&self) -> bool {
        self.inner.with(|sem| {
            if sem.try_give() {
                if let Some(waken_task) = sem.pop_waiting() {
                    SCHEDULER.with(|scheduler| scheduler.resume_task(waken_task));
                }
                true
            } else {
                false
            }
        })
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
