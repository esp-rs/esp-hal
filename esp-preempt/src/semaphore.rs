use alloc::boxed::Box;
use core::ptr::NonNull;

use esp_hal::time::{Duration, Instant};
use esp_radio_preempt_driver::{
    register_semaphore_implementation,
    semaphore::{SemaphoreImplementation, SemaphorePtr},
    yield_task,
};
use esp_sync::NonReentrantMutex;

struct SemaphoreInner {
    current: u32,
    max: u32,
}

impl SemaphoreInner {
    fn try_take(&mut self) -> bool {
        if self.current > 0 {
            self.current -= 1;
            true
        } else {
            false
        }
    }

    fn try_give(&mut self) -> bool {
        if self.current < self.max {
            self.current += 1;
            true
        } else {
            false
        }
    }

    fn current_count(&mut self) -> u32 {
        self.current
    }
}

pub struct Semaphore {
    inner: NonReentrantMutex<SemaphoreInner>,
}

impl Semaphore {
    pub fn new(max: u32, initial: u32) -> Self {
        Semaphore {
            inner: NonReentrantMutex::new(SemaphoreInner {
                current: initial,
                max,
            }),
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
    fn create(max: u32, initial: u32) -> SemaphorePtr {
        let sem = Box::new(Semaphore::new(max, initial));
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
}

register_semaphore_implementation!(Semaphore);
