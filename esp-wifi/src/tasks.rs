use crate::{
    compat::{self, queue::SimpleQueue, timer_compat::TIMERS},
    memory_fence::memory_fence,
    preempt::preempt::task_create,
    timer::{get_systimer_count, yield_task},
};

pub fn init_tasks() {
    task_create(compat::task_runner::run_c_task);
    task_create(timer_task);

    // if coex then we know we have ble + wifi
    #[cfg(coex)]
    task_create(compat::task_runner::run_c_task);
}

pub extern "C" fn timer_task() {
    loop {
        let mut to_run = SimpleQueue::<_, 20>::new();

        let current_timestamp = get_systimer_count();
        critical_section::with(|_| unsafe {
            memory_fence();
            for timer in TIMERS.iter_mut() {
                if timer.active
                    && crate::timer::time_diff(timer.started, current_timestamp) >= timer.timeout
                {
                    debug!("timer is due.... {:x}", timer.id());

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
