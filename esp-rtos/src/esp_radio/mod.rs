//! esp-radio support

use core::{ffi::c_void, ptr::NonNull};

use allocator_api2::boxed::Box;
use esp_hal::{
    system::Cpu,
    time::{Duration, Instant},
};
use esp_radio_rtos_driver::{
    ThreadPtr,
    queue::CompatQueue,
    register_queue_implementation,
    register_semaphore_implementation,
    register_timer_implementation,
    register_wait_queue_implementation,
    semaphore::{CompatSemaphore, SemaphoreHandle, SemaphoreKind, SemaphorePtr},
    timer::CompatTimer,
    wait_queue::{WaitQueueImplementation, WaitQueuePtr},
};

use crate::{
    SCHEDULER,
    run_queue::{MaxPriority, Priority},
    scheduler::Scheduler,
    task::{self, Task, TaskExt},
    wait_queue::WaitQueue,
};

impl esp_radio_rtos_driver::SchedulerImplementation for Scheduler {
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
    ) -> ThreadPtr {
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
        .cast()
    }

    fn current_task(&self) -> ThreadPtr {
        self.current_task().cast()
    }

    fn schedule_task_deletion(&self, task_handle: ThreadPtr) {
        task::schedule_task_deletion(task_handle.cast::<Task>().as_ptr())
    }

    fn current_task_thread_semaphore(&self) -> SemaphorePtr {
        task::with_current_task(|task| {
            *task.thread_local.thread_semaphore.get_or_insert_with(|| {
                SemaphoreHandle::new(SemaphoreKind::Counting { initial: 0, max: 1 }).leak()
            })
        })
    }

    fn usleep(&self, us: u32) {
        self.usleep_until(crate::now() + us as u64);
    }

    fn usleep_until(&self, target: u64) {
        SCHEDULER.sleep_until(Instant::EPOCH + Duration::from_micros(target));
    }

    fn now(&self) -> u64 {
        // We're using a SingleShotTimer as the time driver, which lets us use the system timer's
        // timestamps.
        crate::now()
    }

    unsafe fn task_priority(&self, task: ThreadPtr) -> u32 {
        SCHEDULER
            .with(|scheduler| task.cast::<Task>().priority(&mut scheduler.run_queue).get() as u32)
    }

    unsafe fn set_task_priority(&self, task: ThreadPtr, priority: u32) {
        SCHEDULER.with(|scheduler| {
            let task = task.cast::<Task>();

            let wake = {
                let task = unsafe { task.as_ref() };
                // run_queued is also set if the task is in a wait queue :(
                // current_queue, however, is Some only if the task is in a wait queue.
                task.current_queue.is_none() && task.run_queued
            };
            task.set_priority(&mut scheduler.run_queue, Priority::new(priority as usize));
            if wake {
                // The task was removed from its run queue, put it back at the right priority.
                scheduler.resume_task(task);
            }
        })
    }
}

impl WaitQueue {
    unsafe fn from_ptr<'a>(ptr: WaitQueuePtr) -> &'a mut Self {
        // Both wait_with_deadline and notify use a scheduler lock, so it should be okay to
        // return a mutable reference here.
        unsafe { ptr.cast::<Self>().as_mut() }
    }
}

impl WaitQueueImplementation for WaitQueue {
    fn create() -> WaitQueuePtr {
        let wait_queue = Box::new(WaitQueue::new());
        NonNull::from(Box::leak(wait_queue)).cast()
    }

    unsafe fn delete(queue: WaitQueuePtr) {
        let queue = unsafe { Box::from_raw(queue.cast::<WaitQueue>().as_ptr()) };
        core::mem::drop(queue);
    }

    unsafe fn wait_until(queue: WaitQueuePtr, deadline_instant: Option<u64>) {
        let queue = unsafe { WaitQueue::from_ptr(queue) };
        let deadline = Instant::EPOCH
            + deadline_instant
                .map(Duration::from_micros)
                .unwrap_or(Duration::MAX);
        queue.wait_with_deadline(deadline);
    }

    unsafe fn notify(queue: WaitQueuePtr) {
        let queue = unsafe { WaitQueue::from_ptr(queue) };
        queue.notify();
    }
}

register_timer_implementation!(CompatTimer);
register_queue_implementation!(CompatQueue);
register_semaphore_implementation!(CompatSemaphore);
register_wait_queue_implementation!(WaitQueue);
