//! esp-radio support

use core::{ffi::c_void, ptr::NonNull};

use allocator_api2::boxed::Box;
use esp_hal::{
    system::Cpu,
    time::{Duration, Instant},
};
use esp_radio_rtos_driver::{
    register_semaphore_implementation,
    semaphore::{SemaphoreImplementation, SemaphoreKind, SemaphorePtr},
};

use crate::{
    SCHEDULER,
    run_queue::MaxPriority,
    scheduler::Scheduler,
    semaphore::Semaphore,
    task::{self, Task},
};

mod queue;
mod timer_queue;

impl esp_radio_rtos_driver::Scheduler for Scheduler {
    fn initialized(&self) -> bool {
        self.with(|scheduler| {
            if scheduler.time_driver.is_none() {
                warn!("Trying to initialize esp-radio before starting esp-rtos");
                return false;
            }

            let current_cpu = Cpu::current() as usize;
            if !scheduler.per_cpu[current_cpu].initialized {
                warn!(
                    "Trying to initialize esp-radio on {:?} but esp-rtos is not running on this core",
                    current_cpu
                );
                return false;
            }

            true
        })
    }

    fn yield_task(&self) {
        task::yield_task();
    }

    fn yield_task_from_isr(&self) {
        task::yield_task();
    }

    fn max_task_priority(&self) -> u32 {
        MaxPriority::MAX_PRIORITY as u32
    }

    fn task_create(
        &self,
        name: &str,
        task: extern "C" fn(*mut c_void),
        param: *mut c_void,
        priority: u32,
        pin_to_core: Option<u32>,
        task_stack_size: usize,
    ) -> *mut c_void {
        self.create_task(
            name,
            task,
            param,
            task_stack_size,
            priority.min(self.max_task_priority()),
            pin_to_core.and_then(|core| match core {
                0 => Some(Cpu::ProCpu),
                #[cfg(multi_core)]
                1 => Some(Cpu::AppCpu),
                _ => {
                    warn!("Invalid core number: {}", core);
                    None
                }
            }),
        )
        .as_ptr()
        .cast()
    }

    fn current_task(&self) -> *mut c_void {
        self.current_task().as_ptr().cast()
    }

    fn schedule_task_deletion(&self, task_handle: *mut c_void) {
        task::schedule_task_deletion(task_handle as *mut Task)
    }

    fn current_task_thread_semaphore(&self) -> SemaphorePtr {
        task::with_current_task(|task| {
            NonNull::from(
                task.thread_semaphore
                    .get_or_insert_with(|| Semaphore::new_counting(0, 1)),
            )
            .cast()
        })
    }

    fn usleep(&self, us: u32) {
        SCHEDULER.sleep_until(Instant::now() + Duration::from_micros(us as u64));
    }

    fn now(&self) -> u64 {
        // We're using a SingleShotTimer as the time driver, which lets us use the system timer's
        // timestamps.
        crate::now()
    }
}

impl Semaphore {
    unsafe fn from_ptr<'a>(ptr: SemaphorePtr) -> &'a Self {
        unsafe { ptr.cast::<Self>().as_ref() }
    }
}

impl SemaphoreImplementation for Semaphore {
    fn create(kind: SemaphoreKind) -> SemaphorePtr {
        let sem = Box::new(match kind {
            SemaphoreKind::Counting { max, initial } => Semaphore::new_counting(initial, max),
            SemaphoreKind::Mutex => Semaphore::new_mutex(false),
            SemaphoreKind::RecursiveMutex => Semaphore::new_mutex(true),
        });
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
