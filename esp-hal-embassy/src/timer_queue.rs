//! Timer waiter queue.
//!
//! This module implements the timer queue, which is managed by the time driver.
//! The timer queue contains wakers and their expiration times, and is used to
//! wake tasks at the correct time.

#[cfg(not(single_queue))]
use core::cell::Cell;
use core::cell::RefCell;

use embassy_sync::blocking_mutex::Mutex;
use esp_hal::{interrupt::Priority, sync::RawPriorityLimitedMutex};
use queue_impl::RawQueue;

use crate::time_driver::{AlarmHandle, set_up_alarm};

struct TimerQueueInner {
    queue: RawQueue,
    alarm: Option<AlarmHandle>,
}

pub(crate) struct TimerQueue {
    inner: Mutex<RawPriorityLimitedMutex, RefCell<TimerQueueInner>>,
    priority: Priority,
    #[cfg(not(single_queue))]
    context: Cell<*mut ()>,
}

unsafe impl Sync for TimerQueue {}

impl TimerQueue {
    pub(crate) const fn new(prio: Priority) -> Self {
        Self {
            inner: Mutex::const_new(
                RawPriorityLimitedMutex::new(prio),
                RefCell::new(TimerQueueInner {
                    queue: RawQueue::new(),
                    alarm: None,
                }),
            ),
            priority: prio,
            #[cfg(not(single_queue))]
            context: Cell::new(core::ptr::null_mut()),
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

    pub fn dispatch(&self) {
        let now = esp_hal::time::Instant::now()
            .duration_since_epoch()
            .as_micros();
        self.arm_alarm(now);
    }

    fn arm_alarm(&self, mut next_expiration: u64) {
        loop {
            let set = self.inner.lock(|inner| {
                let mut q = inner.borrow_mut();
                next_expiration = q.queue.next_expiration(next_expiration);

                let alarm = q
                    .alarm
                    .get_or_insert_with(|| set_up_alarm(self.priority, self.context()));
                alarm.update(next_expiration)
            });
            if set {
                break;
            }
        }
    }

    pub fn schedule_wake(&self, at: u64, waker: &core::task::Waker) {
        if self
            .inner
            .lock(|inner| inner.borrow_mut().queue.schedule_wake(at, waker))
        {
            self.dispatch();
        }
    }
}

#[cfg(integrated_timers)]
pub(crate) mod queue_impl {
    use core::{
        cell::Cell,
        cmp::min,
        ptr::{self, NonNull},
        task::Waker,
    };

    use embassy_executor::raw::Executor as RawExecutor;
    use embassy_executor_timer_queue::TimerQueueItem;
    use portable_atomic::{AtomicPtr, Ordering};

    /// An item in the timer queue.
    #[derive(Default)]
    pub(crate) struct QueueItem {
        /// The next item in the queue.
        ///
        /// If this field contains `Some`, the item is in the queue. The last item in the queue has
        /// a value of `Some(dangling_pointer)`
        pub next: Cell<Option<NonNull<QueueItem>>>,

        /// The time at which this item expires.
        pub expires_at: u64,

        /// The registered waker. If Some, the item is enqueued in the timer queue.
        pub waker: Option<Waker>,

        pub owner: AtomicPtr<RawExecutor>,
    }

    unsafe impl Sync for QueueItem {}

    /// Copy of the embassy integrated timer queue, that clears the owner upon
    /// dequeueing.
    pub(super) struct RawQueue {
        head: Cell<Option<NonNull<QueueItem>>>,
    }

    unsafe impl Send for RawQueue {}
    unsafe impl Sync for RawQueue {}

    impl RawQueue {
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
            let item = unsafe {
                // Safety: the `&mut self`, along with the Safety note of the Queue, are sufficient
                // to ensure that this function creates the only mutable reference
                // to the queue item.
                TimerQueueItem::from_embassy_waker(waker)
            };
            let item = unsafe { item.as_mut::<QueueItem>() };
            match item.waker.as_ref() {
                Some(_) if at <= item.expires_at => {
                    // If expiration is sooner than previously set, update.
                    item.expires_at = at;
                    // The waker is always stored in its own queue item, so we don't need to update
                    // it.

                    // Trigger a queue update in case this item can be immediately dequeued.
                    true
                }
                Some(_) => {
                    // Queue item does not need to be updated, the task will be scheduled to be
                    // woken before the new expiration.
                    false
                }
                None => {
                    // If not in the queue, add it and update.
                    let mut item_ptr = NonNull::from(item);
                    let prev = self.head.replace(Some(item_ptr));

                    let item = unsafe { item_ptr.as_mut() };

                    item.expires_at = at;
                    item.waker = Some(waker.clone());
                    item.next.set(prev);
                    // The default implementation doesn't care about the
                    // opaque payload, leave it unchanged.

                    true
                }
            }
        }

        /// Dequeues expired timers and returns the next alarm time.
        ///
        /// The provided callback will be called for each expired task. Tasks
        /// that never expire will be removed, but the callback will not
        /// be called.
        pub fn next_expiration(&mut self, now: u64) -> u64 {
            let mut next_expiration = u64::MAX;

            self.retain(|item| {
                if item.expires_at <= now {
                    // Timer expired, process task.
                    if let Some(waker) = item.waker.take() {
                        waker.wake();
                    }
                    false
                } else {
                    // Timer didn't yet expire, or never expires.
                    next_expiration = min(next_expiration, item.expires_at);
                    item.expires_at != u64::MAX
                }
            });

            next_expiration
        }

        fn retain(&self, mut f: impl FnMut(&mut QueueItem) -> bool) {
            let mut prev = &self.head;
            while let Some(mut p) = prev.get() {
                let item = unsafe { p.as_mut() };

                if f(item) {
                    // Skip to next
                    prev = &item.next;
                } else {
                    // Remove it
                    prev.set(item.next.get());
                    // Clear owner
                    item.owner.store(ptr::null_mut(), Ordering::Relaxed);
                    item.next.set(None);
                }
            }
        }
    }
}

#[cfg(generic_timers)]
mod queue_impl {
    pub(super) type RawQueue = embassy_time_queue_utils::queue_generic::ConstGenericQueue<
        { esp_config::esp_config_int!(usize, "ESP_HAL_EMBASSY_CONFIG_GENERIC_QUEUE_SIZE") },
    >;
}
