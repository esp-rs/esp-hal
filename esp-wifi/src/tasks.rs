use core::mem;

use crate::{
    binary::c_types,
    compat::{self, queue::SimpleQueue, timer_compat::TIMERS},
    memory_fence::memory_fence,
    preempt::preempt::task_create,
    timer::{get_systimer_count, yield_task},
};

pub fn init_tasks() {
    task_create(worker_task1);
    task_create(worker_task2);

    // if coex then we know we have ble + wifi
    #[cfg(coex)]
    task_create(worker_task3);
}

pub extern "C" fn worker_task1() {
    loop {
        compat::work_queue::do_work();
        yield_task();
    }
}

pub extern "C" fn worker_task3() {
    loop {
        compat::work_queue::do_work();
        yield_task();
    }
}

pub extern "C" fn worker_task2() {
    loop {
        let mut to_run = SimpleQueue::<_, 20>::new();

        let current_timestamp = get_systimer_count();
        critical_section::with(|_| unsafe {
            memory_fence();
            for i in 0..TIMERS.len() {
                if let Some(ref mut timer) = TIMERS[i] {
                    if timer.active && current_timestamp >= timer.expire {
                        debug!("timer is due.... {:?}", timer.ptimer);
                        let fnctn: fn(*mut c_types::c_void) = mem::transmute(timer.timer_ptr);

                        _ = to_run.enqueue((fnctn, timer.arg_ptr));

                        if timer.period != 0 {
                            timer.expire = current_timestamp + timer.period;
                            timer.active = timer.active;
                        } else {
                            timer.expire = 0;
                            timer.active = false;
                        };
                    }
                };
            }
            memory_fence();
        });

        // run the due timer callbacks NOT in an interrupt free context
        while let Some((fnc, arg)) = to_run.dequeue() {
            trace!("trigger timer....");
            fnc(arg);
            trace!("timer callback called");
        }

        yield_task();
    }
}
