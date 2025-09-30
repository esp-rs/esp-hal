//! # esp-radio task scheduler interface.
//!
//! `esp-radio` requires a task scheduler to operate. This crate allows the task scheduler to be
//! tailored to specific software platforms (such as ArielOS). Trying to use multiple scheduler
//! crates in a firmware project will not build.
//!
//! If you want to use esp-radio without any OS, you can use the [`esp-rtos`]
//! crate as the task scheduler.
//!
//! ## Implementing a scheduler driver
//!
//! This crate abstracts the capabilities of FreeRTOS. The scheduler must implement the following
//! capabilities:
//!
//! - A preemptive task scheduler: [`Scheduler`]
//! - Semaphores: [`semaphore::SemaphoreImplementation`]
//! - Queues: [`queue::QueueImplementation`]
//! - Timers (functions that are executed at a specific time): [`timer::TimerImplementation`]
//!
//! [`esp-rtos`]: https://crates.io/crates/esp-rtos

#![no_std]

pub mod queue;
pub mod semaphore;
pub mod timer;

use core::ffi::c_void;

// Timer callbacks need to be heap-allocated.
extern crate alloc;

use crate::semaphore::SemaphorePtr;

unsafe extern "Rust" {
    fn esp_rtos_initialized() -> bool;
    fn esp_rtos_yield_task();
    fn esp_rtos_yield_task_from_isr();
    fn esp_rtos_current_task() -> *mut c_void;
    fn esp_rtos_max_task_priority() -> u32;
    fn esp_rtos_task_create(
        name: &str,
        task: extern "C" fn(*mut c_void),
        param: *mut c_void,
        priority: u32,
        pin_to_core: Option<u32>,
        task_stack_size: usize,
    ) -> *mut c_void;
    fn esp_rtos_schedule_task_deletion(task_handle: *mut c_void);
    fn esp_rtos_current_task_thread_semaphore() -> SemaphorePtr;

    fn esp_rtos_usleep(us: u32);
    fn esp_rtos_now() -> u64;
}

/// Set the Scheduler implementation.
///
/// See the [module documentation][crate] for an example.
#[macro_export]
macro_rules! scheduler_impl {
    ($vis:vis static $driver:ident: $t: ty = $val:expr) => {
        $vis static $driver: $t = $val;

        #[unsafe(no_mangle)]
        #[inline]
        fn esp_rtos_initialized() -> bool {
            <$t as $crate::Scheduler>::initialized(&$driver)
        }

        #[unsafe(no_mangle)]
        #[inline]
        fn esp_rtos_yield_task() {
            <$t as $crate::Scheduler>::yield_task(&$driver)
        }

        #[unsafe(no_mangle)]
        #[inline]
        fn esp_rtos_yield_task_from_isr() {
            <$t as $crate::Scheduler>::yield_task_from_isr(&$driver)
        }

        #[unsafe(no_mangle)]
        #[inline]
        fn esp_rtos_current_task() -> *mut c_void {
            <$t as $crate::Scheduler>::current_task(&$driver)
        }

        #[unsafe(no_mangle)]
        #[inline]
        fn esp_rtos_max_task_priority() -> u32 {
            <$t as $crate::Scheduler>::max_task_priority(&$driver)
        }

        #[unsafe(no_mangle)]
        #[inline]
        fn esp_rtos_task_create(
            name: &str,
            task: extern "C" fn(*mut c_void),
            param: *mut c_void,
            priority: u32,
            core_id: Option<u32>,
            task_stack_size: usize,
        ) -> *mut c_void {
            <$t as $crate::Scheduler>::task_create(
                &$driver,
                name,
                task,
                param,
                priority,
                core_id,
                task_stack_size,
            )
        }

        #[unsafe(no_mangle)]
        #[inline]
        fn esp_rtos_schedule_task_deletion(task_handle: *mut c_void) {
            <$t as $crate::Scheduler>::schedule_task_deletion(&$driver, task_handle)
        }

        #[unsafe(no_mangle)]
        #[inline]
        fn esp_rtos_current_task_thread_semaphore() -> $crate::semaphore::SemaphorePtr {
            <$t as $crate::Scheduler>::current_task_thread_semaphore(&$driver)
        }

        #[unsafe(no_mangle)]
        #[inline]
        fn esp_rtos_usleep(us: u32) {
            <$t as $crate::Scheduler>::usleep(&$driver, us)
        }

        #[unsafe(no_mangle)]
        #[inline]
        fn esp_rtos_now() -> u64 {
            <$t as $crate::Scheduler>::now(&$driver)
        }
    };
}

/// The scheduler interface.
///
/// This trait needs to be implemented by a driver crate to integrate esp-radio with a software
/// platform.
///
/// The following snippet demonstrates the boilerplate necessary to implement a scheduler using the
/// `Scheduler` trait:
///
/// ```rust,no_run
/// struct MyScheduler {}
///
/// impl esp_radio_rtos_driver::Scheduler for MyScheduler {
///
///     fn initialized(&self) -> bool {
///         unimplemented!()
///     }
///
///     fn yield_task(&self) {
///         unimplemented!()
///     }
///
///     fn yield_task_from_isr(&self) {
///         unimplemented!()
///     }
///
///     fn max_task_priority(&self) -> u32 {
///         unimplemented!()
///     }
///
///     fn task_create(
///        &self,
///        name: &str,
///        task: extern "C" fn(*mut c_void),
///        param: *mut c_void,
///        priority: u32,
///        pin_to_core: Option<u32>,
///        task_stack_size: usize,
///     ) -> *mut c_void {
///         unimplemented!()
///     }
///
///     fn current_task(&self) -> *mut c_void {
///         unimplemented!()
///     }
///
///     fn schedule_task_deletion(&self, task_handle: *mut c_void) {
///         unimplemented!()
///     }
///
///     fn current_task_thread_semaphore(&self) -> SemaphorePtr {
///         unimplemented!()
///     }
///
///     fn usleep(&self, us: u32) {
///         unimplemented!()
///     }
///
///     fn now(&self) -> u64 {
///         unimplemented!()
///     }
/// }
///
/// esp_radio_rtos_driver::scheduler_impl!(static SCHEDULER: MyScheduler = MyScheduler {});
/// ```
pub trait Scheduler: Send + Sync + 'static {
    /// This function is called by `esp_radio::init` to verify that the scheduler is properly set
    /// up.
    fn initialized(&self) -> bool;

    /// This function is called by `esp_radio` to yield control to another task.
    fn yield_task(&self);

    /// This function is called by `esp_radio` to yield control to another task.
    fn yield_task_from_isr(&self);

    /// This function is called by `esp_radio::init` to retrieve a pointer to the current task.
    fn current_task(&self) -> *mut c_void;

    /// This function returns the maximum task priority level.
    /// Higher number is considered to be higher priority.
    fn max_task_priority(&self) -> u32;

    /// This function is used to create threads.
    /// It should allocate the stack.
    fn task_create(
        &self,
        name: &str,
        task: extern "C" fn(*mut c_void),
        param: *mut c_void,
        priority: u32,
        core_id: Option<u32>,
        task_stack_size: usize,
    ) -> *mut c_void;

    /// This function is called to let the scheduler know this thread is not
    /// needed anymore and should be deleted. After this function is called,
    /// the thread should not be scheduled anymore. The thread stack can be
    /// free'ed.
    ///
    /// Passing `null` as the task handle should delete the currently running task.
    fn schedule_task_deletion(&self, task_handle: *mut c_void);

    /// This function should return an opaque per-thread pointer to an
    /// usize-sized memory location, which will be used to store a pointer
    /// to a semaphore for this thread.
    fn current_task_thread_semaphore(&self) -> SemaphorePtr;

    /// This function is called by a task to sleep for the specified number of microseconds.
    fn usleep(&self, us: u32);

    /// Returns the current timestamp in microseconds.
    ///
    /// The underlying timer is expected not to overflow during the lifetime of the program.
    ///
    /// The clock that generates this timestamp must be the same one used to trigger timer events.
    fn now(&self) -> u64;
}

// API used (mostly) by esp-radio

/// Returns whether the task scheduler has been initialized.
#[inline]
pub fn initialized() -> bool {
    unsafe { esp_rtos_initialized() }
}

/// Yields control to another task.
#[inline]
pub fn yield_task() {
    unsafe { esp_rtos_yield_task() }
}

/// Yields control to another task for an interrupt.
#[inline]
pub fn yield_task_from_isr() {
    unsafe { esp_rtos_yield_task_from_isr() }
}

/// Returns a pointer to the current task.
#[inline]
pub fn current_task() -> *mut c_void {
    unsafe { esp_rtos_current_task() }
}

/// Returns the maximum priority a task can have.
///
/// This function assumes that a bigger number means higher priority.
#[inline]
pub fn max_task_priority() -> u32 {
    unsafe { esp_rtos_max_task_priority() }
}

/// Creates a new task with the given initial parameter and stack size.
///
/// ## Safety
///
/// The `param` parameter must be valid for the lifetime of the task. The data
/// pointed to by `param` needs to be `Send` and the task takes ownership over it.
#[inline]
pub unsafe fn task_create(
    name: &str,
    task: extern "C" fn(*mut c_void),
    param: *mut c_void,
    priority: u32,
    pin_to_core: Option<u32>,
    task_stack_size: usize,
) -> *mut c_void {
    unsafe { esp_rtos_task_create(name, task, param, priority, pin_to_core, task_stack_size) }
}

/// Schedules the given task for deletion.
///
/// Passing `null` will schedule the current task to be deleted.
///
/// ## Safety
///
/// The `task_handle` must be a pointer to a task, obtained either by calling [`task_create`] or
/// [`current_task`].
#[inline]
pub unsafe fn schedule_task_deletion(task_handle: *mut c_void) {
    unsafe { esp_rtos_schedule_task_deletion(task_handle) }
}

/// Returns a pointer to the current thread's semaphore.
#[inline]
pub fn current_task_thread_semaphore() -> SemaphorePtr {
    unsafe { esp_rtos_current_task_thread_semaphore() }
}

/// Puts the current task to sleep for the specified number of microseconds.
#[inline]
pub fn usleep(us: u32) {
    unsafe { esp_rtos_usleep(us) }
}

/// Returns the current timestamp, in microseconds.
#[inline]
pub fn now() -> u64 {
    unsafe { esp_rtos_now() }
}
