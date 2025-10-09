use core::task::Waker;

use embassy_time_queue_utils::Queue;
use esp_sync::NonReentrantMutex;

use crate::SCHEDULER;

pub(super) struct TimerQueueInner {
    queue: Queue,
    pub next_wakeup: u64,
}

impl TimerQueueInner {
    const fn new() -> Self {
        Self {
            queue: Queue::new(),
            next_wakeup: u64::MAX,
        }
    }

    pub(crate) fn handle_alarm(&mut self, now: u64) {
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

pub(crate) struct TimerQueue {
    inner: NonReentrantMutex<TimerQueueInner>,
}

impl TimerQueue {
    pub(crate) const fn new() -> Self {
        Self {
            inner: NonReentrantMutex::new(TimerQueueInner::new()),
        }
    }

    pub(crate) fn handle_alarm(&self, now: u64) {
        self.inner.with(|inner| {
            inner.handle_alarm(now);
        });
    }

    pub(crate) fn next_wakeup(&self) -> u64 {
        self.inner.with(|inner| inner.next_wakeup)
    }
}

impl embassy_time_driver::Driver for TimerQueue {
    #[inline]
    fn now(&self) -> u64 {
        crate::now()
    }

    #[inline]
    fn schedule_wake(&self, at: u64, waker: &Waker) {
        // Note that we don't put the thread to sleep here, as other embassy tasks may be
        // ready. The thread will go to sleep when it can.
        if self.inner.with(|inner| inner.schedule_wake(at, waker)) {
            // Next wakeup time became shorter, re-arm the timer.
            // FIXME: avoid two separate critical sections.
            SCHEDULER.with(|scheduler| {
                unwrap!(scheduler.time_driver.as_mut()).arm_next_wakeup(crate::now());
            });
        }
    }
}
