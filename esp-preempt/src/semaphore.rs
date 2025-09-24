use esp_hal::{
    system::Cpu,
    time::{Duration, Instant},
};
use esp_sync::NonReentrantMutex;

use crate::{
    SCHEDULER,
    task::{TaskExt, TaskPtr},
    wait_queue::WaitQueue,
};

enum SemaphoreInner {
    Counting {
        current: u32,
        max: u32,
        waiting: WaitQueue,
    },
    Mutex {
        recursive: bool,
        owner: Option<TaskPtr>,
        original_priority: usize,
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
            SemaphoreInner::Mutex {
                recursive,
                owner,
                lock_counter,
                original_priority,
                ..
            } => {
                SCHEDULER.with(|scheduler| {
                    let current_cpu = Cpu::current() as usize;
                    let current = unwrap!(scheduler.per_cpu[current_cpu].current_task);
                    if let Some(owner) = owner {
                        if *owner == current && *recursive {
                            *lock_counter += 1;
                            true
                        } else {
                            // We can't lock the mutex. Make sure the mutex holder has a high enough
                            // priority to avoid priority inversion.
                            let current_priority = current.priority(&mut scheduler.run_queue);
                            if owner.priority(&mut scheduler.run_queue) < current_priority {
                                owner.set_priority(&mut scheduler.run_queue, current_priority);
                                scheduler.resume_task(*owner);
                            }
                            false
                        }
                    } else {
                        *owner = Some(current);
                        *lock_counter += 1;
                        *original_priority = current.priority(&mut scheduler.run_queue);
                        true
                    }
                })
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
            SemaphoreInner::Mutex {
                owner,
                lock_counter,
                original_priority,
                ..
            } => SCHEDULER.with(|scheduler| {
                let current_cpu = Cpu::current() as usize;
                let current = unwrap!(scheduler.per_cpu[current_cpu].current_task);

                if *owner == Some(current) && *lock_counter > 0 {
                    *lock_counter -= 1;
                    if *lock_counter == 0
                        && let Some(owner) = owner.take()
                    {
                        owner.set_priority(&mut scheduler.run_queue, *original_priority);
                    }
                    true
                } else {
                    false
                }
            }),
        }
    }

    fn current_count(&mut self) -> u32 {
        match self {
            SemaphoreInner::Counting { current, .. } => *current,
            SemaphoreInner::Mutex { .. } => {
                panic!("RecursiveMutex does not support current_count")
            }
        }
    }

    fn wait_with_deadline(&mut self, deadline: Option<Instant>) {
        trace!("Semaphore wait_with_deadline - {:?}", deadline);
        match self {
            SemaphoreInner::Counting { waiting, .. } => waiting.wait_with_deadline(deadline),
            SemaphoreInner::Mutex { waiting, .. } => waiting.wait_with_deadline(deadline),
        }
    }

    fn notify(&mut self) {
        trace!("Semaphore notify");
        match self {
            SemaphoreInner::Counting { waiting, .. } => waiting.notify(),
            SemaphoreInner::Mutex { waiting, .. } => waiting.notify(),
        }
    }
}

pub struct Semaphore {
    inner: NonReentrantMutex<SemaphoreInner>,
}

impl Semaphore {
    pub fn new_counting(initial: u32, max: u32) -> Self {
        Semaphore {
            inner: NonReentrantMutex::new(SemaphoreInner::Counting {
                current: initial,
                max,
                waiting: WaitQueue::new(),
            }),
        }
    }
    pub fn new_mutex(recursive: bool) -> Self {
        Semaphore {
            inner: NonReentrantMutex::new(SemaphoreInner::Mutex {
                recursive,
                owner: None,
                lock_counter: 0,
                original_priority: 0,
                waiting: WaitQueue::new(),
            }),
        }
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
                    sem.wait_with_deadline(deadline);
                    false
                }
            });

            if taken {
                debug!("Semaphore - take - success");
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
                trace!("Semaphore - take - timed out");
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
                sem.notify();
                true
            } else {
                false
            }
        })
    }
}
