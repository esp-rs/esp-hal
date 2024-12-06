#[cfg(not(feature = "single-queue"))]
use core::cell::Cell;
use core::cell::UnsafeCell;

use embassy_sync::blocking_mutex::Mutex;
use esp_hal::{interrupt::Priority, sync::RawPriorityLimitedMutex};

use crate::time_driver::{set_up_alarm, AlarmHandle};

pub(crate) struct TimerQueue {
    inner: Mutex<RawPriorityLimitedMutex, adapter::RawQueue>,
    priority: Priority,
    #[cfg(not(feature = "single-queue"))]
    context: Cell<*mut ()>,
    alarm: UnsafeCell<Option<AlarmHandle>>,
}

unsafe impl Sync for TimerQueue {}

impl TimerQueue {
    pub(crate) const fn new(prio: Priority) -> Self {
        Self {
            inner: Mutex::const_new(RawPriorityLimitedMutex::new(prio), adapter::new_queue()),
            priority: prio,
            #[cfg(not(feature = "single-queue"))]
            context: Cell::new(core::ptr::null_mut()),
            alarm: UnsafeCell::new(None),
        }
    }

    #[cfg(not(feature = "single-queue"))]
    pub(crate) fn set_context(&self, context: *mut ()) {
        self.context.set(context);
    }

    #[cfg(not(feature = "single-queue"))]
    fn context(&self) -> *mut () {
        self.context.get()
    }

    #[cfg(feature = "single-queue")]
    fn context(&self) -> *mut () {
        core::ptr::null_mut()
    }

    pub fn alarm(&self) -> AlarmHandle {
        unsafe {
            let alarm = &mut *self.alarm.get();
            *alarm.get_or_insert_with(|| set_up_alarm(self.priority, self.context()))
        }
    }

    pub fn dispatch(&self) {
        let now = esp_hal::time::now().ticks();
        let next_expiration = self.inner.lock(|q| adapter::dequeue(q, now));
        self.arm_alarm(next_expiration);
    }

    fn arm_alarm(&self, mut next_expiration: u64) {
        let alarm = self.alarm();

        while !alarm.update(next_expiration) {
            // next_expiration is in the past, dequeue and find a new expiration
            next_expiration = self.inner.lock(|q| adapter::dequeue(q, next_expiration));
        }
    }
}

impl embassy_time_queue_driver::TimerQueue for crate::time_driver::TimerQueueDriver {
    fn schedule_wake(&'static self, at: u64, waker: &core::task::Waker) {
        #[cfg(feature = "integrated-timers")]
        let waker = embassy_executor::raw::task_from_waker(waker);

        #[cfg(not(feature = "single-queue"))]
        unsafe {
            let executor = &*(waker.executor().unwrap_unchecked()
                as *const embassy_executor::raw::Executor)
                .cast::<crate::executor::InnerExecutor>();
            executor.timer_queue.schedule_wake(waker, at);
        }

        #[cfg(feature = "single-queue")]
        self.inner.schedule_wake(waker, at);
    }
}

#[cfg(feature = "integrated-timers")]
mod adapter {
    use embassy_executor::raw;

    pub(super) type RawQueue = raw::timer_queue::TimerQueue;

    pub(super) const fn new_queue() -> RawQueue {
        raw::timer_queue::TimerQueue::new()
    }

    pub(super) fn dequeue(q: &RawQueue, now: u64) -> u64 {
        unsafe { q.next_expiration(now, embassy_executor::raw::wake_task) }
    }

    impl super::TimerQueue {
        pub fn schedule_wake(&self, task: raw::TaskRef, at: u64) {
            if unsafe { self.inner.lock(|q| q.schedule_wake(task, at)) } {
                self.arm_alarm(at);
            }
        }
    }
}

#[cfg(not(feature = "integrated-timers"))]
mod adapter {
    use core::{cell::RefCell, task::Waker};

    pub(super) type RawQueue = RefCell<
        embassy_time_queue_driver::queue_generic::Queue<
            { esp_config::esp_config_int!(usize, "ESP_HAL_EMBASSY_GENERIC_QUEUE_SIZE") },
        >,
    >;

    pub(super) const fn new_queue() -> RawQueue {
        RefCell::new(embassy_time_queue_driver::queue_generic::Queue::new())
    }

    pub(super) fn dequeue(q: &RawQueue, now: u64) -> u64 {
        q.borrow_mut().next_expiration(now)
    }

    impl super::TimerQueue {
        pub fn schedule_wake(&self, waker: &Waker, at: u64) {
            if self.inner.lock(|q| q.borrow_mut().schedule_wake(at, waker)) {
                self.arm_alarm(at);
            }
        }
    }
}
