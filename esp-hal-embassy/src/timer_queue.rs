#[cfg(not(single_queue))]
use core::cell::Cell;
use core::cell::UnsafeCell;

use embassy_sync::blocking_mutex::Mutex;
use esp_hal::{interrupt::Priority, sync::RawPriorityLimitedMutex};

use crate::time_driver::{set_up_alarm, AlarmHandle};

pub(crate) struct TimerQueue {
    inner: Mutex<RawPriorityLimitedMutex, adapter::RawQueue>,
    priority: Priority,
    #[cfg(not(single_queue))]
    context: Cell<*mut ()>,
    alarm: UnsafeCell<Option<AlarmHandle>>,
}

unsafe impl Sync for TimerQueue {}

impl TimerQueue {
    pub(crate) const fn new(prio: Priority) -> Self {
        Self {
            inner: Mutex::const_new(RawPriorityLimitedMutex::new(prio), adapter::RawQueue::new()),
            priority: prio,
            #[cfg(not(single_queue))]
            context: Cell::new(core::ptr::null_mut()),
            alarm: UnsafeCell::new(None),
        }
    }

    #[cfg(not(single_queue))]
    pub(crate) fn set_context(&self, context: *mut ()) {
        self.context.set(context);
    }

    #[cfg(not(single_queue))]
    fn context(&self) -> *mut () {
        self.context.get()
    }

    #[cfg(single_queue)]
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
        #[cfg(integrated_timers)]
        let waker = embassy_executor::raw::task_from_waker(waker);

        #[cfg(not(single_queue))]
        unsafe {
            // FIXME: this is UB, use Exposed Provenance API (or something better) when
            // available. Expose provenance in `InnerExecutor::init`, and use it here.
            let executor = &*(waker.executor().unwrap_unchecked()
                as *const embassy_executor::raw::Executor)
                .cast::<crate::executor::InnerExecutor>();
            executor.timer_queue.schedule_wake(at, waker);
        }

        #[cfg(single_queue)]
        self.inner.schedule_wake(at, waker);
    }
}

#[cfg(integrated_timers)]
mod adapter {
    use core::cell::RefCell;

    use embassy_executor::raw;

    type Q = embassy_time_queue_driver::queue_integrated::TimerQueue;

    /// A simple wrapper around a `Queue` to provide interior mutability.
    pub struct RefCellQueue {
        inner: RefCell<Q>,
    }

    impl RefCellQueue {
        /// Creates a new timer queue.
        pub const fn new() -> Self {
            Self {
                inner: RefCell::new(Q::new()),
            }
        }

        /// Schedules a task to run at a specific time, and returns whether any
        /// changes were made.
        pub fn schedule_wake(&self, at: u64, waker: raw::TaskRef) -> bool {
            self.inner.borrow_mut().schedule_wake(at, waker)
        }

        /// Dequeues expired timers and returns the next alarm time.
        pub fn next_expiration(&self, now: u64) -> u64 {
            self.inner.borrow_mut().next_expiration(now)
        }
    }

    pub(super) type RawQueue = RefCellQueue;

    pub(super) fn dequeue(q: &RawQueue, now: u64) -> u64 {
        q.next_expiration(now)
    }

    impl super::TimerQueue {
        pub fn schedule_wake(&self, at: u64, task: raw::TaskRef) {
            if self.inner.lock(|q| q.schedule_wake(at, task)) {
                self.dispatch();
            }
        }
    }
}

#[cfg(generic_timers)]
mod adapter {
    use core::{cell::RefCell, task::Waker};

    type Q = embassy_time_queue_driver::queue_generic::ConstGenericQueue<
        { esp_config::esp_config_int!(usize, "ESP_HAL_EMBASSY_GENERIC_QUEUE_SIZE") },
    >;

    /// A simple wrapper around a `Queue` to provide interior mutability.
    pub struct RefCellQueue {
        inner: RefCell<Q>,
    }

    impl RefCellQueue {
        /// Creates a new timer queue.
        pub const fn new() -> Self {
            Self {
                inner: RefCell::new(Q::new()),
            }
        }

        /// Schedules a task to run at a specific time, and returns whether any
        /// changes were made.
        pub fn schedule_wake(&self, at: u64, waker: &core::task::Waker) -> bool {
            self.inner.borrow_mut().schedule_wake(at, waker)
        }

        /// Dequeues expired timers and returns the next alarm time.
        pub fn next_expiration(&self, now: u64) -> u64 {
            self.inner.borrow_mut().next_expiration(now)
        }
    }

    pub(super) type RawQueue = RefCellQueue;

    pub(super) fn dequeue(q: &RawQueue, now: u64) -> u64 {
        q.next_expiration(now)
    }

    impl super::TimerQueue {
        pub fn schedule_wake(&self, at: u64, waker: &Waker) {
            if self.inner.lock(|q| q.schedule_wake(at, waker)) {
                self.dispatch();
            }
        }
    }
}
