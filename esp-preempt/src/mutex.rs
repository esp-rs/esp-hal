use alloc::boxed::Box;
use core::ptr::NonNull;

use esp_hal::time::{Duration, Instant};
use esp_radio_preempt_driver::{
    current_task,
    mutex::{MutexImplementation, MutexPtr},
    register_mutex_implementation,
    yield_task,
};
use esp_sync::NonReentrantMutex;

struct MutexInner {
    recursive: bool,
    owner: usize,
    lock_counter: u32,
}

impl MutexInner {
    fn try_lock(&mut self) -> bool {
        let current = current_task() as usize;

        if self.owner == 0 || (self.owner == current && self.recursive) {
            self.owner = current;
            self.lock_counter += 1;
            true
        } else {
            false
        }
    }

    fn unlock(&mut self) -> bool {
        let current = current_task() as usize;

        if self.owner == current && self.lock_counter > 0 {
            self.lock_counter -= 1;
            if self.lock_counter == 0 {
                self.owner = 0;
            }
            true
        } else {
            false
        }
    }
}

pub struct Mutex {
    inner: NonReentrantMutex<MutexInner>,
}

impl Mutex {
    pub fn new(recursive: bool) -> Self {
        Mutex {
            inner: NonReentrantMutex::new(MutexInner {
                recursive,
                owner: 0,
                lock_counter: 0,
            }),
        }
    }

    unsafe fn from_ptr<'a>(ptr: MutexPtr) -> &'a Self {
        unsafe { ptr.cast::<Self>().as_ref() }
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

    pub fn lock(&self, timeout_us: Option<u32>) -> bool {
        Self::yield_loop_with_timeout(timeout_us, || self.inner.with(|mutex| mutex.try_lock()))
    }

    pub fn unlock(&self) -> bool {
        self.inner.with(|mutex| mutex.unlock())
    }
}

impl MutexImplementation for Mutex {
    fn create(recursive: bool) -> MutexPtr {
        let mutex = Box::new(Mutex::new(recursive));
        NonNull::from(Box::leak(mutex)).cast()
    }

    unsafe fn delete(mutex: MutexPtr) {
        let mutex = unsafe { Box::from_raw(mutex.cast::<Mutex>().as_ptr()) };
        core::mem::drop(mutex);
    }

    unsafe fn lock(mutex: MutexPtr, timeout_us: Option<u32>) -> bool {
        let mutex = unsafe { Mutex::from_ptr(mutex) };

        mutex.lock(timeout_us)
    }

    unsafe fn unlock(mutex: MutexPtr) -> bool {
        let mutex = unsafe { Mutex::from_ptr(mutex) };

        mutex.unlock()
    }
}

register_mutex_implementation!(Mutex);
