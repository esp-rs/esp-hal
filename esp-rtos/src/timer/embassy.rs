use core::task::Waker;

use embassy_time_queue_utils::Queue;

use crate::SCHEDULER;

pub(crate) struct TimerQueue {
    queue: Queue,
    pub next_wakeup: u64,
}

impl TimerQueue {
    pub const fn new() -> Self {
        Self {
            queue: Queue::new(),
            next_wakeup: u64::MAX,
        }
    }

    pub fn handle_alarm(&mut self, now: u64) {
        if now < self.next_wakeup {
            trace!(
                "Not processing embassy timer queue. Now: {}, expected next wakeup: {}",
                now, self.next_wakeup
            );
            return;
        }
        trace!("Processing embassy timer queue at {}", now);

        self.next_wakeup = self.queue.next_expiration(now);
    }

    fn schedule_wake(&mut self, at: u64, waker: &Waker) -> bool {
        if self.queue.schedule_wake(at, waker) {
            self.next_wakeup = self.next_wakeup.min(at);
            true
        } else {
            false
        }
    }
}

pub(crate) struct EmbassyTimeDriver;
impl embassy_time_driver::Driver for EmbassyTimeDriver {
    #[inline]
    fn now(&self) -> u64 {
        crate::now()
    }

    #[inline]
    fn schedule_wake(&self, at: u64, waker: &Waker) {
        // Note that we don't put the thread to sleep here, as other embassy tasks may be
        // ready. The thread will go to sleep when it can.
        SCHEDULER.with_shared(|global_state| {
            let mut embassy_timer_queue = global_state.embassy_timer_queue();
            if embassy_timer_queue.schedule_wake(at, waker) {
                // Next wakeup time became shorter, re-arm the timer.
                let mut scheduler = global_state.scheduler();
                let time_driver = unwrap!(scheduler.time_driver.as_mut());

                time_driver
                    .timer_queue
                    .signal_next_embassy_wakeup(embassy_timer_queue.next_wakeup);
                time_driver.arm_next_wakeup(crate::now());
            }
        });
    }
}
