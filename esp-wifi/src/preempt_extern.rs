/// This module allows hooking `esp-wifi` into an external scheduler, instead
/// of using the integrated one as provided by the `preempt` module.
///
/// In order to hook up an external scheduler, enable the `preempt-extern`
/// feature and provide the `esp_wifi_preempt_*` functions as documented
/// somewhere.
///
/// Use non-mangled extern "Rust" functions.
///
/// Example:
///
/// ```Rust
/// #[no_mangle]
/// pub extern "Rust" fn esp_wifi_preempt_setup(_timer: TimeBase) {
///     // your code here
/// }
/// ```
use crate::binary::c_types;

extern "Rust" {
    /// This function is called by `esp-wifi` when starting up the WiFi stack.
    /// The passed `timer` can be used for e.g., implementing a periodic timer.
    fn esp_wifi_preempt_setup(timer: crate::TimeBase);

    /// This function is called by `esp-wifi` when shutting down the WiFi stack.
    fn esp_wifi_preempt_disable();

    /// This function is called by threads and should switch to the next thread.
    fn esp_wifi_preempt_yield_task();

    /// This function is called by threads and should return an opaque handle
    /// for the calling thread. The same handle will be passed to
    /// `esp_wifi_preempt_schedule_task_deletion`.
    fn esp_wifi_preempt_current_task() -> *mut c_types::c_void;

    /// This function is used to create threads.
    /// It should allocate the stack.
    fn esp_wifi_preempt_task_create(
        task: extern "C" fn(*mut c_types::c_void),
        param: *mut c_types::c_void,
        task_stack_size: usize,
    ) -> *mut c_types::c_void;

    /// This function is called to let the scheduler now this thread is not
    /// needed anymore and should be deleted. After this function is called,
    /// the thread should not be scheduled anymore. The thread stack can be
    /// free'ed.
    fn esp_wifi_preempt_schedule_task_deletion(task_handle: *mut c_types::c_void);

    /// This function should return an opaque per-thread pointer to an
    /// usize-sized memory location, which will be used to store a pointer
    /// to a semaphore for this thread.
    fn esp_wifi_preempt_current_task_thread_semaphore() -> *mut c_types::c_void;
}

pub(crate) fn setup(timer: crate::TimeBase) {
    unsafe { esp_wifi_preempt_setup(timer) }
}

pub(crate) fn disable() {
    unsafe { esp_wifi_preempt_disable() }
}

pub(crate) fn yield_task() {
    unsafe { esp_wifi_preempt_yield_task() }
}

pub(crate) fn current_task() -> *mut c_types::c_void {
    unsafe { esp_wifi_preempt_current_task() }
}

pub(crate) fn task_create(
    task: extern "C" fn(*mut c_types::c_void),
    param: *mut c_types::c_void,
    task_stack_size: usize,
) -> *mut c_types::c_void {
    unsafe { esp_wifi_preempt_task_create(task, param, task_stack_size) }
}

pub(crate) fn schedule_task_deletion(task_handle: *mut c_types::c_void) {
    unsafe { esp_wifi_preempt_schedule_task_deletion(task_handle) }
}

pub(crate) fn current_task_thread_semaphore() -> *mut c_types::c_void {
    unsafe { esp_wifi_preempt_current_task_thread_semaphore() }
}
