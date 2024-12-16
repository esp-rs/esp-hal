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

    pub fn schedule_wake(&self, at: u64, waker: &core::task::Waker) {
        if self.inner.lock(|q| q.schedule_wake(at, waker)) {
            self.dispatch();
        }
    }
}

#[cfg(integrated_timers)]
mod adapter {
    use core::{
        cell::{Cell, RefCell},
        cmp::min,
        ptr,
        task::Waker,
    };

    use embassy_executor::raw::TaskRef;
    use portable_atomic::{AtomicPtr, Ordering};

    /// Copy of the embassy integrated timer queue, that clears the owner upon
    /// dequeueing.
    pub struct Q {
        head: Cell<Option<TaskRef>>,
    }

    impl Q {
        /// Creates a new timer queue.
        pub const fn new() -> Self {
            Self {
                head: Cell::new(None),
            }
        }

        /// Schedules a task to run at a specific time.
        ///
        /// If this function returns `true`, the called should find the next
        /// expiration time and set a new alarm for that time.
        pub fn schedule_wake(&mut self, at: u64, waker: &Waker) -> bool {
            let task = embassy_executor::raw::task_from_waker(waker);
            let item = task.timer_queue_item();
            if item.next.get().is_none() {
                // If not in the queue, add it and update.
                let prev = self.head.replace(Some(task));
                item.next.set(if prev.is_none() {
                    Some(unsafe { TaskRef::dangling() })
                } else {
                    prev
                });
                item.expires_at.set(at);
                true
            } else if at <= item.expires_at.get() {
                // If expiration is sooner than previously set, update.
                item.expires_at.set(at);
                true
            } else {
                // Task does not need to be updated.
                false
            }
        }

        /// Dequeues expired timers and returns the next alarm time.
        ///
        /// The provided callback will be called for each expired task. Tasks
        /// that never expire will be removed, but the callback will not
        /// be called.
        pub fn next_expiration(&mut self, now: u64) -> u64 {
            let mut next_expiration = u64::MAX;

            self.retain(|p| {
                let item = p.timer_queue_item();
                let expires = item.expires_at.get();

                if expires <= now {
                    // Timer expired, process task.
                    embassy_executor::raw::wake_task(p);
                    false
                } else {
                    // Timer didn't yet expire, or never expires.
                    next_expiration = min(next_expiration, expires);
                    expires != u64::MAX
                }
            });

            next_expiration
        }

        fn retain(&self, mut f: impl FnMut(TaskRef) -> bool) {
            let mut prev = &self.head;
            while let Some(p) = prev.get() {
                if unsafe { p == TaskRef::dangling() } {
                    // prev was the last item, stop
                    break;
                }
                let item = p.timer_queue_item();
                if f(p) {
                    // Skip to next
                    prev = &item.next;
                } else {
                    // Remove it
                    prev.set(item.next.get());
                    // Clear owner
                    unsafe {
                        // SAFETY: our payload is an AtomicPtr.
                        item.payload
                            .as_ref::<AtomicPtr<()>>()
                            .store(ptr::null_mut(), Ordering::Relaxed);
                    }
                    item.next.set(None);
                }
            }
        }
    }

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
}
