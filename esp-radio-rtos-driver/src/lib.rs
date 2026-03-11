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
//! This crate abstracts the capabilities of FreeRTOS. The implementor crate has two different
//! possible ways to implement the required capabilities. The `SchedulerImplementation` trait must
//! be implemented in both cases.
//!
//! The implementation types must be registered using the respective `register_x_implementation`
//! macros.
//!
//! ### Without `ipc-implementations`
//!
//! The implementor must implement the [`semaphore::SemaphoreImplementation`],
//! [`queue::QueueImplementation`], and [`timer::TimerImplementation`] traits.
//!
//! ```rust
//! use esp_radio_rtos_driver::{
//!     SchedulerImplementation,
//!     queue::QueueImplementation,
//!     register_queue_implementation,
//!     register_scheduler_implementation,
//!     register_semaphore_implementation,
//!     register_timer_implementation,
//!     semaphore::SemaphoreImplementation,
//!     timer::TimerImplementation,
//! };
//!
//! struct MyScheduler {
//!     // ...
//! }
//! impl SchedulerImplementation for MyScheduler {
//!     // ...
//! }
//!
//! struct MySemaphore {
//!     // ...
//! }
//! impl SemaphoreImplementation for MySemaphore {
//!     // ...
//! }
//!
//! struct MyTimer {
//!     // ...
//! }
//! impl TimerImplementation for MyTimer {
//!     // ...
//! }
//!
//! struct MyQueue {
//!     // ...
//! }
//! impl QueueImplementation for MyQueue {
//!     // ...
//! }
//!
//! register_scheduler_implementation!(static SCHEDULER: MyScheduler = MyScheduler {});
//! register_semaphore_implementation!(MySemaphore);
//! register_timer_implementation!(MyTimer);
//! register_queue_implementation!(MyQueue);
//! ```
//!
//! ### Using `ipc-implementations`
//!
//! The implementor must implement the [`wait_queue::WaitQueueImplementation`] trait, and can use
//! the various `Compat` types as the default implementations for the IPC types.
//!
//! You still have the option to provide custom implementations for the IPC types.
//!
//! ```rust, ignore
//! use esp_radio_rtos_driver::{
//!     SchedulerImplementation,
//!     queue::CompatQueue,
//!     register_queue_implementation,
//!     register_scheduler_implementation,
//!     register_semaphore_implementation,
//!     register_timer_implementation,
//!     register_wait_queue_implementation,
//!     semaphore::CompatSemaphore,
//!     timer::CompatTimer,
//!     wait_queue::WaitQueueImplementation,
//! };Expand commentComment on lines R78 to R89ResolvedCode has comments. Press enter to view.
//!
//! struct MyScheduler {
//!     // ...
//! }
//! impl SchedulerImplementation for MyScheduler {
//!     // ...
//! }
//!
//! struct MyWaitQueue {
//!     // ...
//! }
//! impl WaitQueueImplementation for MyWaitQueue {
//!     // ...
//! }
//!
//! register_scheduler_implementation!(static SCHEDULER: MyScheduler = MyScheduler {});
//! register_wait_queue_implementation!(MyWaitQueue);
//!
//! register_semaphore_implementation!(CompatSemaphore);
//! register_timer_implementation!(CompatTimer);
//! register_queue_implementation!(CompatQueue);
//! ```
//!
//! [`esp-rtos`]: https://crates.io/crates/esp-rtos

#![no_std]

// MUST be the first module
mod fmt;

pub mod queue;
pub mod semaphore;
pub mod timer;
#[cfg(feature = "ipc-implementations")]
pub mod wait_queue;

use core::{ffi::c_void, ptr::NonNull};

// Timer callbacks need to be heap-allocated.
extern crate alloc;

use crate::semaphore::SemaphorePtr;

pub type ThreadPtr = NonNull<()>;

unsafe extern "Rust" {
    fn esp_rtos_initialized() -> bool;
    fn esp_rtos_yield_task();
    fn esp_rtos_yield_task_from_isr();
    fn esp_rtos_current_task() -> ThreadPtr;
    fn esp_rtos_max_task_priority() -> u32;
    fn esp_rtos_task_create(
        name: &str,
        task: extern "C" fn(*mut c_void),
        param: *mut c_void,
        priority: u32,
        pin_to_core: Option<u32>,
        task_stack_size: usize,
    ) -> ThreadPtr;
    fn esp_rtos_schedule_task_deletion(task_handle: Option<ThreadPtr>);
    fn esp_rtos_current_task_thread_semaphore() -> SemaphorePtr;
    #[cfg(feature = "ipc-implementations")]
    fn esp_rtos_task_priority(task: ThreadPtr) -> u32;
    #[cfg(feature = "ipc-implementations")]
    fn esp_rtos_set_task_priority(task: ThreadPtr, priority: u32);

    fn esp_rtos_usleep(us: u32);
    fn esp_rtos_usleep_until(us: u64);
    fn esp_rtos_now() -> u64;
}

/// Set the Scheduler implementation.
///
/// See the [module documentation][crate] for an example.
#[macro_export]
macro_rules! register_scheduler_implementation {
    ($vis:vis static $driver:ident: $t: ty = $val:expr) => {
        $vis static $driver: $t = $val;

        #[unsafe(no_mangle)]
        #[inline]
        fn esp_rtos_initialized() -> bool {
            <$t as $crate::SchedulerImplementation>::initialized(&$driver)
        }

        #[unsafe(no_mangle)]
        #[inline]
        fn esp_rtos_yield_task() {
            <$t as $crate::SchedulerImplementation>::yield_task(&$driver)
        }

        #[unsafe(no_mangle)]
        #[inline]
        fn esp_rtos_yield_task_from_isr() {
            <$t as $crate::SchedulerImplementation>::yield_task_from_isr(&$driver)
        }

        #[unsafe(no_mangle)]
        #[inline]
        fn esp_rtos_current_task() -> $crate::ThreadPtr {
            <$t as $crate::SchedulerImplementation>::current_task(&$driver)
        }

        #[unsafe(no_mangle)]
        #[inline]
        fn esp_rtos_max_task_priority() -> u32 {
            <$t as $crate::SchedulerImplementation>::max_task_priority(&$driver)
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
        ) -> $crate::ThreadPtr {
            <$t as $crate::SchedulerImplementation>::task_create(
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
        fn esp_rtos_schedule_task_deletion(task_handle: Option<$crate::ThreadPtr>) {
            <$t as $crate::SchedulerImplementation>::schedule_task_deletion(&$driver, task_handle)
        }

        #[unsafe(no_mangle)]
        #[inline]
        fn esp_rtos_current_task_thread_semaphore() -> $crate::semaphore::SemaphorePtr {
            <$t as $crate::SchedulerImplementation>::current_task_thread_semaphore(&$driver)
        }

        #[unsafe(no_mangle)]
        #[inline]
        fn esp_rtos_task_priority(task: $crate::ThreadPtr) -> u32 {
            unsafe { <$t as $crate::SchedulerImplementation>::task_priority(&$driver, task) }
        }

        #[unsafe(no_mangle)]
        #[inline]
        fn esp_rtos_set_task_priority(task: $crate::ThreadPtr, priority: u32) {
            unsafe { <$t as $crate::SchedulerImplementation>::set_task_priority(&$driver, task, priority) }
        }

        #[unsafe(no_mangle)]
        #[inline]
        fn esp_rtos_usleep(us: u32) {
            <$t as $crate::SchedulerImplementation>::usleep(&$driver, us)
        }

        #[unsafe(no_mangle)]
        #[inline]
        fn esp_rtos_usleep_until(target: u64) {
            <$t as $crate::SchedulerImplementation>::usleep_until(&$driver, target)
        }

        #[unsafe(no_mangle)]
        #[inline]
        fn esp_rtos_now() -> u64 {
            <$t as $crate::SchedulerImplementation>::now(&$driver)
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
/// ```rust, no_run
/// use esp_radio_rtos_driver::{ThreadPtr, SchedulerImplementation, register_scheduler_implementation};
/// struct MyScheduler {}
///
/// impl SchedulerImplementation for MyScheduler {
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
///     ) -> ThreadPtr {
///         unimplemented!()
///     }
///
///     fn current_task(&self) -> ThreadPtr {
///         unimplemented!()
///     }
///
///     fn schedule_task_deletion(&self, task_handle: Option<ThreadPtr>) {
///         unimplemented!()
///     }
///
///     fn current_task_thread_semaphore(&self) -> SemaphorePtr {
///         unimplemented!()
///     }
///
///     fn task_priority(&self, task: ThreadPtr) -> u32 {
///         unimplemented!()
///     }
///
///     fn set_task_priority(&self, task: ThreadPtr, priority: u32) {
///         unimplemented!()
///     }
///
///     fn usleep(&self, us: u32) {
///         unimplemented!()
///     }
///
///     fn usleep_until(&self, target: u64) {
///         unimplemented!()
///     }
///
///     fn now(&self) -> u64 {
///         unimplemented!()
///     }
/// }
///
/// register_scheduler_implementation!(static SCHEDULER: MyScheduler = MyScheduler {});
/// ```
pub trait SchedulerImplementation: Send + Sync + 'static {
    /// This function is called by `esp_radio::init` to verify that the scheduler is properly set
    /// up.
    fn initialized(&self) -> bool;

    /// This function is called by `esp_radio` to yield control to another task.
    fn yield_task(&self);

    /// This function is called by `esp_radio` to yield control to another task.
    fn yield_task_from_isr(&self);

    /// This function is called by `esp_radio::init` to retrieve a pointer to the current task.
    fn current_task(&self) -> ThreadPtr;

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
    ) -> ThreadPtr;

    /// This function is called to let the scheduler know this thread is not
    /// needed anymore and should be deleted. After this function is called,
    /// the thread should not be scheduled anymore. The thread stack can be
    /// free'ed.
    ///
    /// Passing `None` as the task handle should delete the currently running task.
    fn schedule_task_deletion(&self, task_handle: Option<ThreadPtr>);

    /// This function should return an opaque per-thread pointer to an
    /// usize-sized memory location, which will be used to store a pointer
    /// to a semaphore for this thread.
    fn current_task_thread_semaphore(&self) -> SemaphorePtr;

    /// This function returns the priority of the given task.
    ///
    /// # Safety
    ///
    /// The task pointer must be valid and point to a task that was created using
    /// [`Self::task_create`].
    unsafe fn task_priority(&self, task: ThreadPtr) -> u32;

    /// This function sets the priority of the given task.
    ///
    /// # Safety
    ///
    /// The task pointer must be valid and point to a task that was created using
    /// [`Self::task_create`].
    unsafe fn set_task_priority(&self, task: ThreadPtr, priority: u32);

    /// This function is called by a task to sleep for the specified number of microseconds.
    fn usleep(&self, us: u32);

    /// This function is called by a task to sleep until the specified timestamp.
    ///
    /// The timestamp is measured in microseconds, from the time the timer was started.
    fn usleep_until(&self, target: u64);

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
pub fn current_task() -> ThreadPtr {
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
) -> ThreadPtr {
    let priority = priority.min(max_task_priority());
    unsafe { esp_rtos_task_create(name, task, param, priority, pin_to_core, task_stack_size) }
}

/// Schedules the given task for deletion.
///
/// Passing `None` will schedule the current task to be deleted.
///
/// ## Safety
///
/// The `task_handle` must be a pointer to a task, obtained either by calling [`task_create`] or
/// [`current_task`].
#[inline]
pub unsafe fn schedule_task_deletion(task_handle: Option<ThreadPtr>) {
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

/// Puts the current task to sleep until the specified timestamp.
///
/// The timestamp is measured in microseconds, from the time the timer was started.
#[inline]
pub fn usleep_until(target: u64) {
    unsafe { esp_rtos_usleep_until(target) }
}

/// Returns the current timestamp, in microseconds.
#[inline]
pub fn now() -> u64 {
    unsafe { esp_rtos_now() }
}

#[inline]
#[cfg(feature = "ipc-implementations")]
unsafe fn task_priority(task: ThreadPtr) -> u32 {
    unsafe { esp_rtos_task_priority(task) }
}

#[inline]
#[cfg(feature = "ipc-implementations")]
unsafe fn set_task_priority(task: ThreadPtr, priority: u32) {
    unsafe { esp_rtos_set_task_priority(task, priority) }
}
