use crate::{
    compat::timer_compat::get_next_due_timer,
    preempt::arch_specific::task_create,
    timer::yield_task,
};

/// Initializes the `main` and `timer` tasks for the Wi-Fi driver.
pub(crate) fn init_tasks() {
    // allocate the main task
    crate::preempt::allocate_main_task();

    // schedule the timer task
    task_create(timer_task, core::ptr::null_mut(), 8192);
}

/// Entry point for the timer task responsible for handling scheduled timer
/// events.
pub(crate) extern "C" fn timer_task(_param: *mut esp_wifi_sys::c_types::c_void) {
    loop {
        // run the due timer callback NOT in an interrupt free context
        if let Some((func, param)) = get_next_due_timer() {
            if let Some(func) = func {
                trace!(
                    "trigger timer callback {:x} {:x}",
                    func as usize,
                    param as usize
                );
                unsafe {
                    func(param as *mut _);
                }
                trace!("timer callback done");
            }
        } else {
            yield_task();
        }
    }
}
