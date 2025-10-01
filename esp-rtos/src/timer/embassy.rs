use core::task::Waker;

use esp_sync::NonReentrantMutex;

pub(super) struct TimerQueueInner {
    queue: embassy_time_queue_utils::Queue,
    pub next_wakeup: u64,
}

impl TimerQueueInner {
    const fn new() -> Self {
        Self {
            queue: embassy_time_queue_utils::Queue::new(),
            next_wakeup: u64::MAX,
        }
    }

    pub(crate) fn handle_alarm(&mut self, now: u64) {
        if now >= self.next_wakeup {
            self.next_wakeup = self.queue.next_expiration(now);
        }
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
        if self.inner.with(|inner| inner.schedule_wake(at, waker)) {
            // Next wakeup time became shorter, yield to re-arm the timer.
            crate::task::yield_task();
        }
    }
}
