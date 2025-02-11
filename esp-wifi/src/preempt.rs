//! This module allows hooking `esp-wifi` into an external scheduler, instead
//! of using the integrated one as provided by the `preempt` module.
//!
//! In order to hook up an external scheduler, enable the `preempt-extern`
//! feature, implement the `Scheduler` trait for a struct, and create the
//! necessary `extern` functions using the `scheduler_impl!()` macro.
//!
//! Example:
//!
//! ```ignore
//! use esp_wifi::preempt::Scheduler;
//!
//! struct MyScheduler {}
//!
//! impl Scheduler for MyScheduler {
//!     // impl goes here
//! }
//!
//! esp_wifi::scheduler_impl!(static SCHEDULER: MyScheduler = MyScheduler {});
//! ```
use core::ffi::c_void;

pub trait Scheduler: Send + Sync + 'static {
    /// This function is called by `esp-wifi` when starting up the WiFi stack.
    fn enable(&self);

    /// This function is called by `esp-wifi` when shutting down the WiFi stack.
    fn disable(&self);

    /// This function is called by threads and should switch to the next thread.
    fn yield_task(&self);

    /// This function is called by threads and should return an opaque handle
    /// for the calling thread. The same handle will be passed to
    /// `esp_wifi_preempt_schedule_task_deletion`.
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
    fn schedule_task_deletion(&self, task_handle: *mut c_void);

    /// This function should return an opaque per-thread pointer to an
    /// usize-sized memory location, which will be used to store a pointer
    /// to a semaphore for this thread.
    fn current_task_thread_semaphore(&self) -> *mut c_void;
}

extern "Rust" {
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

pub(crate) fn enable() {
    unsafe { esp_wifi_preempt_enable() }
}

pub(crate) fn disable() {
    unsafe { esp_wifi_preempt_disable() }
}

pub(crate) fn yield_task() {
    unsafe { esp_wifi_preempt_yield_task() }
}

pub(crate) fn current_task() -> *mut c_void {
    unsafe { esp_wifi_preempt_current_task() }
}

pub(crate) fn task_create(
    task: extern "C" fn(*mut c_void),
    param: *mut c_void,
    task_stack_size: usize,
) -> *mut c_void {
    unsafe { esp_wifi_preempt_task_create(task, param, task_stack_size) }
}

pub(crate) fn schedule_task_deletion(task_handle: *mut c_void) {
    unsafe { esp_wifi_preempt_schedule_task_deletion(task_handle) }
}

pub(crate) fn current_task_thread_semaphore() -> *mut c_void {
    unsafe { esp_wifi_preempt_current_task_thread_semaphore() }
}

/// Set the Scheduler implementation.
///
/// See the module documentation for an example.
#[macro_export]
macro_rules! scheduler_impl {
    (static $name:ident: $t: ty = $val:expr) => {
        static $name: $t = $val;

        #[no_mangle]
        fn esp_wifi_preempt_enable() {
            <$t as $crate::preempt::Scheduler>::enable(&$name)
        }
        #[no_mangle]
        fn esp_wifi_preempt_disable() {
            <$t as $crate::preempt::Scheduler>::disable(&$name)
        }
        #[no_mangle]
        fn esp_wifi_preempt_yield_task() {
            <$t as $crate::preempt::Scheduler>::yield_task(&$name)
        }
        #[no_mangle]
        fn esp_wifi_preempt_current_task() -> *mut c_void {
            <$t as $crate::preempt::Scheduler>::current_task(&$name)
        }
        #[no_mangle]
        fn esp_wifi_preempt_task_create(
            task: extern "C" fn(*mut c_void),
            param: *mut c_void,
            task_stack_size: usize,
        ) -> *mut c_void {
            <$t as $crate::preempt::Scheduler>::task_create(&$name, task, param, task_stack_size)
        }
        #[no_mangle]
        fn esp_wifi_preempt_schedule_task_deletion(task_handle: *mut c_void) {
            <$t as $crate::preempt::Scheduler>::schedule_task_deletion(&$name, task_handle)
        }
        #[no_mangle]
        fn esp_wifi_preempt_current_task_thread_semaphore() -> *mut c_void {
            <$t as $crate::preempt::Scheduler>::current_task_thread_semaphore(&$name)
        }
    };
}
