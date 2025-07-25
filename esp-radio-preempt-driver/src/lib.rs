//! # esp-radio task scheduler interface.
//!
//! `esp-radio` requires a task scheduler to operate. This crate allows the task scheduler to be
//! tailored to specific software platforms (such as ArielOS). Trying to use multiple scheduler
//! crates in a firmware project will not build.
//!
//! If you want to use esp-radio without any OS, you can use the [`esp-radio-preempt-baremetal`]
//! crate as the task scheduler.
//!
//! ## Implementing a scheduler driver
//!
//! In order to hook up a scheduler, implement the `Scheduler` trait for a struct, and register it
//! using the `scheduler_impl!()` macro. Only one scheduler can be registered in a firmware.
//!
//! Example:
//!
//! ```rust,ignore
//! struct MyScheduler {}
//!
//! impl esp_radio_preempt_driver::Scheduler for MyScheduler {
//!     // impl goes here
//! }
//!
//! esp_radio_preempt_driver::scheduler_impl!(static SCHEDULER: MyScheduler = MyScheduler {});
//! ```
//!
//! [`esp-radio-preempt-baremetal`]: https://crates.io/crates/esp-radio-preempt-baremetal

#![no_std]

use core::ffi::c_void;

unsafe extern "Rust" {
    fn esp_wifi_preempt_initialized() -> bool;
    fn esp_wifi_preempt_usleep(us: u32);
    fn esp_wifi_preempt_enable();
    fn esp_wifi_preempt_disable();
    fn esp_wifi_preempt_yield_task();
    fn esp_wifi_preempt_current_task() -> *mut c_void;
    fn esp_wifi_preempt_task_create(
        task: extern "C" fn(*mut c_void),
        param: *mut c_void,
        task_stack_size: usize,
    ) -> *mut c_void;
    fn esp_wifi_preempt_schedule_task_deletion(task_handle: *mut c_void);
    fn esp_wifi_preempt_current_task_thread_semaphore() -> *mut c_void;
}

/// Set the Scheduler implementation.
///
/// See the [module documentation][crate] for an example.
#[macro_export]
macro_rules! scheduler_impl {
    (static $name:ident: $t: ty = $val:expr) => {
        static $name: $t = $val;

        #[unsafe(no_mangle)]
        #[inline]
        fn esp_wifi_preempt_initialized() -> bool {
            <$t as $crate::Scheduler>::initialized(&$name)
        }

        #[unsafe(no_mangle)]
        #[inline]
        fn esp_wifi_preempt_usleep(us: u32) {
            <$t as $crate::Scheduler>::usleep(&$name, us)
        }

        #[unsafe(no_mangle)]
        #[inline]
        fn esp_wifi_preempt_enable() {
            <$t as $crate::Scheduler>::enable(&$name)
        }
        #[unsafe(no_mangle)]
        #[inline]
        fn esp_wifi_preempt_disable() {
            <$t as $crate::Scheduler>::disable(&$name)
        }
        #[unsafe(no_mangle)]
        #[inline]
        fn esp_wifi_preempt_yield_task() {
            <$t as $crate::Scheduler>::yield_task(&$name)
        }
        #[unsafe(no_mangle)]
        #[inline]
        fn esp_wifi_preempt_current_task() -> *mut c_void {
            <$t as $crate::Scheduler>::current_task(&$name)
        }
        #[unsafe(no_mangle)]
        #[inline]
        fn esp_wifi_preempt_task_create(
            task: extern "C" fn(*mut c_void),
            param: *mut c_void,
            task_stack_size: usize,
        ) -> *mut c_void {
            <$t as $crate::Scheduler>::task_create(&$name, task, param, task_stack_size)
        }
        #[unsafe(no_mangle)]
        #[inline]
        fn esp_wifi_preempt_schedule_task_deletion(task_handle: *mut c_void) {
            <$t as $crate::Scheduler>::schedule_task_deletion(&$name, task_handle)
        }
        #[unsafe(no_mangle)]
        #[inline]
        fn esp_wifi_preempt_current_task_thread_semaphore() -> *mut c_void {
            <$t as $crate::Scheduler>::current_task_thread_semaphore(&$name)
        }
    };
}

/// The scheduler interface.
///
/// This trait needs to be implemented by a driver crate to integrate esp-radio with a software
/// platform.
pub trait Scheduler: Send + Sync + 'static {
    /// This function is called by `esp_wifi::init` to verify that the scheduler is properly set up.
    fn initialized(&self) -> bool;

    /// This function is called by `esp_wifi::init` to put the current task to sleep for the
    /// specified number of microseconds.
    fn usleep(&self, us: u32);

    /// This function is called by `esp-radio` to start the task scheduler.
    fn enable(&self);

    /// This function is called by `esp-radio` to stop the task scheduler.
    fn disable(&self);

    /// This function is called by `esp_wifi::init` to yield control to another task.
    fn yield_task(&self);

    /// This function is called by `esp_wifi::init` to retrieve a pointer to the current task.
    fn current_task(&self) -> *mut c_void;

    /// This function is used to create threads.
    /// It should allocate the stack.
    fn task_create(
        &self,
        task: extern "C" fn(*mut c_void),
        param: *mut c_void,
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
    fn current_task_thread_semaphore(&self) -> *mut c_void;
}

// API used (mostly) by esp-radio

/// Returns whether the task scheduler has been initialized.
#[inline]
pub fn initialized() -> bool {
    unsafe { esp_wifi_preempt_initialized() }
}

/// Puts the current task to sleep for the specified number of microseconds.
#[inline]
pub fn usleep(us: u32) {
    unsafe { esp_wifi_preempt_usleep(us) }
}

/// Starts the task scheduler.
#[inline]
pub fn enable() {
    unsafe { esp_wifi_preempt_enable() }
}

/// Stops the task scheduler.
#[inline]
pub fn disable() {
    unsafe { esp_wifi_preempt_disable() }
}

/// Yields control to another task.
#[inline]
pub fn yield_task() {
    unsafe { esp_wifi_preempt_yield_task() }
}

/// Returns a pointer to the current task.
#[inline]
pub fn current_task() -> *mut c_void {
    unsafe { esp_wifi_preempt_current_task() }
}

/// Creates a new task with the given initial parameter and stack size.
///
/// ## Safety
///
/// The `param` parameter must be valid for the lifetime of the task. The data
/// pointed to by `param` needs to be `Send` and the task takes ownership over it.
#[inline]
pub unsafe fn task_create(
    task: extern "C" fn(*mut c_void),
    param: *mut c_void,
    task_stack_size: usize,
) -> *mut c_void {
    unsafe { esp_wifi_preempt_task_create(task, param, task_stack_size) }
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
    unsafe { esp_wifi_preempt_schedule_task_deletion(task_handle) }
}

/// Returns a pointer to the current thread's semaphore.
#[inline]
pub fn current_task_thread_semaphore() -> *mut c_void {
    unsafe { esp_wifi_preempt_current_task_thread_semaphore() }
}
