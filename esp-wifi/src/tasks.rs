use crate::{
    compat::{queue::SimpleQueue, timer_compat::TIMERS},
    memory_fence::memory_fence,
    preempt::arch_specific::task_create,
    timer::{get_systimer_count, yield_task},
};

/// Initializes the `main` and `timer` tasks for the Wi-Fi driver.
pub fn init_tasks() {
    // allocate the main task
    crate::preempt::allocate_main_task();

    // schedule the timer task
    task_create(timer_task, core::ptr::null_mut(), 8192);
}

/// Entry point for the timer task responsible for handling scheduled timer
/// events.
pub extern "C" fn timer_task(_param: *mut esp_wifi_sys::c_types::c_void) {
    loop {
        let mut to_run = SimpleQueue::<_, 20>::new();

        let current_timestamp = get_systimer_count();
        critical_section::with(|_| unsafe {
            memory_fence();
            for timer in TIMERS.iter_mut() {
                if timer.active
                    && crate::timer::time_diff(timer.started, current_timestamp) >= timer.timeout
                {
                    trace!("timer is due.... {:x}", timer.id());

                    if to_run.enqueue(timer.callback).is_err() {
                        break;
                    }

                    timer.active = timer.periodic;
                };
            }
            memory_fence();
        });

        // run the due timer callbacks NOT in an interrupt free context
        while let Some(callback) = to_run.dequeue() {
            trace!("trigger timer....");
            callback.call();
            trace!("timer callback called");
        }

        yield_task();
    }
}
