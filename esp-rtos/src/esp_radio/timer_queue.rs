use alloc::boxed::Box;
use core::{
    cell::{RefCell, UnsafeCell},
    ffi::c_void,
    ptr::NonNull,
};

use esp_hal::time::{Duration, Instant};
use esp_radio_rtos_driver::{
    register_timer_implementation,
    timer::{TimerImplementation, TimerPtr},
};
use esp_sync::NonReentrantMutex;

use crate::{
    SCHEDULER,
    task::{TaskExt, TaskPtr},
};

static TIMER_QUEUE: TimerQueue = TimerQueue::new();

struct TimerQueueInner {
    // A linked list of active timers
    head: Option<NonNull<Timer>>,
    next_wakeup: u64,
    task: Option<TaskPtr>,
    processing: bool,
    scheduled_for_drop: alloc::vec::Vec<Box<Timer>>,
}

unsafe impl Send for TimerQueueInner {}

impl TimerQueueInner {
    const fn new() -> Self {
        Self {
            head: None,
            next_wakeup: 0,
            task: None,
            processing: false,
            scheduled_for_drop: alloc::vec::Vec::new(),
        }
    }

    fn enqueue(&mut self, timer: &Timer) {
        let head = self.head;
        let props = timer.properties(self);
        let due = props.next_due;

        if !props.enqueued {
            debug!("Enqueueing timer {:x}", timer as *const _ as usize);
            props.enqueued = true;

            props.next = head;
            self.head = Some(NonNull::from(timer));
        } else {
            trace!("Already enqueued timer {:x}", timer as *const _ as usize);
        }

        if let Some(task) = self.task {
            if due < self.next_wakeup {
                self.next_wakeup = due;

                // Do not resume the task unless it is sleeping. But if it is sleeping, we need to
                // unconditionally resume it, so that it can re-schedule its next wakeup time
                // according to the new due time.
                if !self.processing {
                    task.resume();
                }
            }
        } else {
            // create the timer task
            let task_ptr =
                SCHEDULER.create_task("timer", timer_task, core::ptr::null_mut(), 8192, 2, None);
            self.task = Some(task_ptr);
            self.next_wakeup = due;
        }
    }

    fn dequeue(&mut self, timer: &Timer) -> bool {
        let mut current = self.head;
        let mut prev: Option<NonNull<Timer>> = None;

        // Scan through the queue until we find the timer
        while let Some(current_timer) = current {
            if core::ptr::eq(current_timer.as_ptr(), timer) {
                // If we find the timer, remove it from the queue by bypassing it in the linked
                // list. The previous element, if any, will point at the next element.

                let timer_props = timer.properties(self);
                let next = timer_props.next.take();
                timer_props.enqueued = false;

                if let Some(mut p) = prev {
                    unsafe { p.as_mut().properties(self).next = next };
                } else {
                    self.head = next;
                }
                return true;
            }

            prev = current;
            current = unsafe { current_timer.as_ref().properties(self).next };
        }

        false
    }
}

struct TimerQueue {
    inner: NonReentrantMutex<TimerQueueInner>,
}

unsafe impl Send for TimerQueue {}

impl TimerQueue {
    const fn new() -> Self {
        Self {
            inner: NonReentrantMutex::new(TimerQueueInner::new()),
        }
    }

    /// Trigger due timers.
    ///
    /// The timer queue needs to be re-processed when a new timer is armed, because the new timer
    /// may need to be triggered before the next scheduled wakeup.
    fn process(&self) {
        debug!("Processing timers");
        let mut timers = self.inner.with(|q| {
            q.processing = true;
            q.next_wakeup = u64::MAX;
            q.head.take()
        });

        while let Some(current) = timers {
            trace!("Checking timer: {:x}", current.addr());
            let current_timer = unsafe { current.as_ref() };

            let run_callback = self.inner.with(|q| {
                let props = current_timer.properties(q);

                // Remove current timer from the list.
                timers = props.next.take();
                props.enqueued = false;

                if !props.is_active || props.drop {
                    trace!(
                        "Timer {:x} is inactive or dropped",
                        current_timer as *const _ as usize
                    );
                    return false;
                }

                if props.next_due > crate::now() {
                    // Not our time yet.
                    trace!(
                        "Timer {:x} is not due yet",
                        current_timer as *const _ as usize
                    );
                    return false;
                }

                // Re-arm periodic timer
                if props.periodic {
                    props.next_due += props.period;
                }
                props.is_active = props.periodic;
                true
            });

            if run_callback {
                debug!("Triggering timer: {:x}", current_timer as *const _ as usize);
                (current_timer.callback.borrow_mut())();
            }

            self.inner.with(|q| {
                let props = current_timer.properties(q);

                if props.is_active && !props.drop {
                    q.enqueue(current_timer);
                } else {
                    trace!(
                        "Timer {:x} inactive or about to get dropped",
                        current_timer as *const _ as usize
                    );
                }
            });
        }

        self.inner.with(|q| {
            for timer in q.scheduled_for_drop.drain(..) {
                debug!(
                    "Dropping timer {:x} (delayed)",
                    (&*timer) as *const _ as usize
                );
                core::mem::drop(timer);
            }

            q.processing = false;
            let next_wakeup = q.next_wakeup;
            debug!("next_wakeup: {}", next_wakeup);
            SCHEDULER.sleep_until(Instant::EPOCH + Duration::from_micros(next_wakeup));
        });
    }
}

struct TimerProperties {
    is_active: bool,
    next_due: u64,
    period: u64,
    periodic: bool,
    drop: bool,

    enqueued: bool,
    next: Option<NonNull<Timer>>,
}

struct TimerQueueCell<T>(UnsafeCell<T>);

impl<T> TimerQueueCell<T> {
    const fn new(inner: T) -> Self {
        Self(UnsafeCell::new(inner))
    }

    fn get_mut<'a>(&'a self, _q: &'a mut TimerQueueInner) -> &'a mut T {
        unsafe { &mut *self.0.get() }
    }
}

pub struct Timer {
    callback: RefCell<Box<dyn FnMut() + Send>>,
    // Timer properties, not available in `callback` due to how the timer is constructed.
    timer_properties: TimerQueueCell<TimerProperties>,
}

impl Timer {
    pub fn new(callback: Box<dyn FnMut() + Send>) -> Self {
        Timer {
            callback: RefCell::new(callback),
            timer_properties: TimerQueueCell::new(TimerProperties {
                is_active: false,
                next_due: 0,
                period: 0,
                periodic: false,
                drop: false,

                enqueued: false,
                next: None,
            }),
        }
    }

    unsafe fn from_ptr<'a>(ptr: TimerPtr) -> &'a Self {
        unsafe { ptr.cast::<Self>().as_mut() }
    }

    fn arm(&self, q: &mut TimerQueueInner, timeout: u64, periodic: bool) {
        let next_due = crate::now() + timeout;

        let props = self.properties(q);
        props.is_active = true;
        props.next_due = next_due;
        props.period = timeout;
        props.periodic = periodic;

        q.enqueue(self);
    }

    fn is_active(&self, q: &mut TimerQueueInner) -> bool {
        self.properties(q).is_active
    }

    fn disarm(&self, q: &mut TimerQueueInner) {
        self.properties(q).is_active = false;

        // We don't dequeue the timer - processing the queue will just skip it. If we re-arm,
        // the timer may already be in the queue.
    }

    fn properties<'a>(&'a self, q: &'a mut TimerQueueInner) -> &'a mut TimerProperties {
        self.timer_properties.get_mut(q)
    }
}

impl TimerImplementation for Timer {
    fn create(func: unsafe extern "C" fn(*mut c_void), data: *mut c_void) -> TimerPtr {
        // TODO: get rid of the inner box (or its heap allocation) somehow
        struct CCallback {
            func: unsafe extern "C" fn(*mut c_void),
            data: *mut c_void,
        }
        unsafe impl Send for CCallback {}

        impl CCallback {
            unsafe fn call(&mut self) {
                unsafe { (self.func)(self.data) }
            }
        }

        let mut callback = CCallback { func, data };

        let timer = Box::new(Timer::new(Box::new(move || unsafe { callback.call() })));
        let ptr = NonNull::from(Box::leak(timer)).cast();
        debug!("Created timer: {:x}", ptr.addr());
        ptr
    }

    unsafe fn delete(timer: TimerPtr) {
        debug!("Deleting timer: {:x}", timer.addr());
        TIMER_QUEUE.inner.with(|q| {
            let timer = unsafe { Box::from_raw(timer.cast::<Timer>().as_ptr()) };

            // There are multiple cases:
            // - the queue is currently processing - mem-forget the timer and mark it to be dropped
            //   later in during processing.
            //
            // - the queue is currently not processing
            //      - we can dequeue the timer -> drop it
            //      - we can't dequeue the timer (i.e. it's not in the queue) -> drop it
            if q.processing {
                timer.properties(q).drop = true;

                if timer.properties(q).enqueued {
                    // it's queued so process will make sure to drop it
                    debug!("schedule timer for dropping after processing the queue");
                    q.scheduled_for_drop.push(timer);
                } else {
                    // drop it since process won't do it for us
                    debug!("dropping not enqueued timer while processing the queue");
                    core::mem::drop(timer);
                }
            } else {
                if q.dequeue(&timer) {
                    debug!("dropping dequeued timer");
                    core::mem::drop(timer);
                } else {
                    debug!("dropping timer not in queue, queue is not processing");
                    core::mem::drop(timer);
                }
            }
        })
    }

    unsafe fn arm(timer: TimerPtr, timeout: u64, periodic: bool) {
        debug!(
            "Arming {:?} for {} us, periodic = {:?}",
            timer, timeout, periodic
        );
        let timer = unsafe { Timer::from_ptr(timer) };
        TIMER_QUEUE.inner.with(|q| timer.arm(q, timeout, periodic))
    }

    unsafe fn is_active(timer: TimerPtr) -> bool {
        debug!("Checking if timer {:?} is active", timer);
        let timer = unsafe { Timer::from_ptr(timer) };
        TIMER_QUEUE.inner.with(|q| timer.is_active(q))
    }

    unsafe fn disarm(timer: TimerPtr) {
        debug!("Disarming {:?}", timer);
        let timer = unsafe { Timer::from_ptr(timer) };
        TIMER_QUEUE.inner.with(|q| timer.disarm(q))
    }
}

register_timer_implementation!(Timer);

/// Entry point for the timer task responsible for handling scheduled timer
/// events.
///
/// The timer task is created when the first timer is armed.
pub(crate) extern "C" fn timer_task(_: *mut c_void) {
    loop {
        TIMER_QUEUE.process();
    }
}
