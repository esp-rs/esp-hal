use crate::{
    compat::timer_compat::TIMERS,
    preempt::arch_specific::task_create,
    timer::{get_systimer_count, yield_task},
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
        let current_timestamp = get_systimer_count();
        let to_run = critical_section::with(|cs| unsafe {
            TIMERS.borrow_ref_mut(cs).find_next_due(current_timestamp)
        });

        // run the due timer callback NOT in an interrupt free context
        if let Some(to_run) = to_run {
            unsafe {
                (*to_run).active = (*to_run).periodic;

                if (*to_run).periodic {
                    (*to_run).started = current_timestamp;
                }

                trace!("trigger timer....");
                (*to_run).callback.call();
                trace!("timer callback called");
            }
        } else {
            yield_task();
        }
    }
}
