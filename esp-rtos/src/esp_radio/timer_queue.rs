use alloc::{boxed::Box, vec::Vec};
use core::{
    cell::{RefCell, UnsafeCell},
    ffi::c_void,
    ptr::NonNull,
    sync::atomic::Ordering,
};

use esp_radio_rtos_driver::{
    register_timer_implementation,
    semaphore::{SemaphoreHandle, SemaphoreKind, SemaphorePtr},
    timer::{TimerImplementation, TimerPtr},
};
use portable_atomic::AtomicPtr;

struct TimerQueueInner {
    // A linked list of active timers
    head: Option<NonNull<CompatTimer>>,
    next_wakeup: u64,
    semaphore: Option<SemaphorePtr>,
    processing: bool,
    scheduled_for_drop: Vec<TimerPtr>,
}

unsafe impl Send for TimerQueueInner {}

impl TimerQueueInner {
    const fn new() -> Self {
        Self {
            head: None,
            next_wakeup: 0,
            semaphore: None,
            processing: false,
            scheduled_for_drop: Vec::new(),
        }
    }

    /// Returns the Semaphore that should be given.
    fn enqueue(&mut self, timer: &CompatTimer) -> Option<SemaphorePtr> {
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

        if let Some(semaphore) = self.semaphore {
            if due < self.next_wakeup {
                self.next_wakeup = due;
                return Some(semaphore);
            }
        } else {
            // create the timer task
            let semaphore =
                SemaphoreHandle::new(SemaphoreKind::Counting { max: 1, initial: 0 }).leak();
            unsafe {
                esp_radio_rtos_driver::task_create(
                    "timer",
                    timer_task,
                    semaphore.as_ptr().cast(),
                    2,
                    None,
                    8192,
                )
            };
            self.semaphore = Some(semaphore);
            self.next_wakeup = due;
        }

        None
    }

    fn dequeue(&mut self, timer: &CompatTimer) -> bool {
        let mut current = self.head;
        let mut prev: Option<NonNull<CompatTimer>> = None;

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

struct CompatTimerQueue {
    inner: UnsafeCell<TimerQueueInner>,
    mutex: SemaphoreHandle,
}

unsafe impl Send for CompatTimerQueue {}

impl CompatTimerQueue {
    fn new() -> Self {
        // TODO figure out how to make this work with a heap-allocated mutex
        Self {
            inner: UnsafeCell::new(TimerQueueInner::new()),
            mutex: SemaphoreHandle::new(SemaphoreKind::Mutex),
        }
    }

    /// Ensures that the timer queue is initialized, then provides a reference to it.
    fn ensure_initialized<'a>() -> &'a CompatTimerQueue {
        static TIMER_QUEUE: AtomicPtr<CompatTimerQueue> = AtomicPtr::new(core::ptr::null_mut());

        if let Some(queue) = NonNull::new(TIMER_QUEUE.load(Ordering::Relaxed)) {
            unsafe { queue.as_ref() }
        } else {
            let boxed = Box::new(CompatTimerQueue::new());
            let queue_ptr = NonNull::from(boxed.as_ref());

            let mut forget = false;
            let queue_ptr = loop {
                match TIMER_QUEUE.compare_exchange(
                    core::ptr::null_mut(),
                    queue_ptr.as_ptr(),
                    Ordering::SeqCst,
                    Ordering::SeqCst,
                ) {
                    Ok(_) => {
                        // We're using our queue, forget it so we don't drop it.
                        forget = true;
                        break queue_ptr;
                    }
                    Err(queue) => {
                        // In case the queue is somehow still null, we will re-attempt storing our
                        // own pointer.
                        if let Some(queue_ptr) = NonNull::new(queue) {
                            break queue_ptr;
                        }
                    }
                }
            };

            if forget {
                core::mem::forget(boxed);
            }

            unsafe { queue_ptr.as_ref() }
        }
    }

    /// Calls a closure with a mutable reference to the global timer queue.
    ///
    /// If the queue is not initialized, it will be initialized first.
    fn with_global<F, R>(f: F) -> R
    where
        F: FnOnce(&mut TimerQueueInner) -> R,
    {
        let queue = Self::ensure_initialized();
        queue.with(f)
    }

    /// Calls a closure with a mutable reference to the timer queue, in case you already have a
    /// reference to it.
    fn with<F, R>(&self, f: F) -> R
    where
        F: FnOnce(&mut TimerQueueInner) -> R,
    {
        struct DropGuard<'a> {
            mutex: &'a SemaphoreHandle,
        }

        impl Drop for DropGuard<'_> {
            fn drop(&mut self) {
                self.mutex.give();
            }
        }

        self.mutex.take(None);
        let _guard = DropGuard { mutex: &self.mutex };

        unsafe { f(&mut *self.inner.get()) }
    }

    /// Trigger due timers.
    ///
    /// The timer queue needs to be re-processed when a new timer is armed, because the new timer
    /// may need to be triggered before the next scheduled wakeup.
    fn process(&self, semaphore: &SemaphoreHandle) {
        debug!("Processing timers");
        let mut timers = self.with(|q| {
            q.processing = true;
            q.next_wakeup = u64::MAX;
            q.head.take()
        });

        while let Some(current) = timers {
            trace!("Checking timer: {:x}", current.addr());
            let current_timer = unsafe { current.as_ref() };

            let run_callback = self.with(|q| {
                let props = current_timer.properties(q);

                // Remove current timer from the list.
                timers = props.next.take();
                props.enqueued = false;

                if !props.is_active {
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

            self.with(|q| {
                let props = current_timer.properties(q);

                if props.is_active {
                    q.enqueue(current_timer);
                } else {
                    trace!(
                        "Timer {:x} inactive or about to get dropped",
                        current_timer as *const _ as usize
                    );
                }
            });
        }

        let next_wakeup = self.with(|q| {
            while let Some(timer) = q.scheduled_for_drop.pop() {
                debug!(
                    "Dropping timer {:x} (delayed)",
                    timer.as_ptr() as *const _ as usize
                );
                let timer = unsafe { Box::from_raw(timer.cast::<CompatTimer>().as_ptr()) };
                q.dequeue(&timer);
                core::mem::drop(timer);
            }

            q.processing = false;
            let next_wakeup = q.next_wakeup;
            debug!("next_wakeup: {}", next_wakeup);
            next_wakeup
        });

        semaphore.take_with_deadline(Some(next_wakeup));
    }
}

struct TimerProperties {
    is_active: bool,
    next_due: u64,
    period: u64,
    periodic: bool,

    enqueued: bool,
    next: Option<NonNull<CompatTimer>>,
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

pub struct CompatTimer {
    callback: RefCell<Box<dyn FnMut() + Send>>,
    // Timer properties, not available in `callback` due to how the timer is constructed.
    timer_properties: TimerQueueCell<TimerProperties>,
}

impl CompatTimer {
    pub fn new(callback: Box<dyn FnMut() + Send>) -> Self {
        CompatTimer {
            callback: RefCell::new(callback),
            timer_properties: TimerQueueCell::new(TimerProperties {
                is_active: false,
                next_due: 0,
                period: 0,
                periodic: false,

                enqueued: false,
                next: None,
            }),
        }
    }

    unsafe fn from_ptr<'a>(ptr: TimerPtr) -> &'a Self {
        unsafe { ptr.cast::<Self>().as_mut() }
    }

    fn arm(&self, q: &mut TimerQueueInner, timeout: u64, periodic: bool) -> Option<SemaphorePtr> {
        let next_due = crate::now() + timeout;

        let props = self.properties(q);
        props.is_active = true;
        props.next_due = next_due;
        props.period = timeout;
        props.periodic = periodic;

        q.enqueue(self)
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

impl TimerImplementation for CompatTimer {
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

        let timer = Box::new(CompatTimer::new(Box::new(move || unsafe {
            callback.call()
        })));
        let ptr = NonNull::from(Box::leak(timer)).cast();
        debug!("Created timer: {:x}", ptr.addr());
        ptr
    }

    unsafe fn delete(timer: TimerPtr) {
        debug!("Deleting timer: {:x}", timer.addr());
        let mut semaphore_to_give = None;
        CompatTimerQueue::with_global(|q| {
            // we don't drop the timer right now, since it might be
            // processed currently
            debug!("schedule timer for dropping after processing the queue");
            q.scheduled_for_drop.push(timer);

            // make sure the queue will get processed soon
            // and cleanup will happen
            if !q.processing && q.next_wakeup == u64::MAX {
                q.next_wakeup = 0;

                semaphore_to_give = q.semaphore;
            }

            let timer = unsafe { CompatTimer::from_ptr(timer) };
            timer.properties(q).is_active = false;
        });

        if let Some(semaphore_ptr) = semaphore_to_give {
            let semaphore = unsafe { SemaphoreHandle::ref_from_ptr(&semaphore_ptr) };
            semaphore.give();
        }
    }

    unsafe fn arm(timer: TimerPtr, timeout: u64, periodic: bool) {
        debug!(
            "Arming {:?} for {} us, periodic = {:?}",
            timer, timeout, periodic
        );
        let timer = unsafe { CompatTimer::from_ptr(timer) };
        if let Some(semaphore_ptr) =
            CompatTimerQueue::with_global(|q| timer.arm(q, timeout, periodic))
        {
            let semaphore = unsafe { SemaphoreHandle::ref_from_ptr(&semaphore_ptr) };
            semaphore.give();
        }
    }

    unsafe fn is_active(timer: TimerPtr) -> bool {
        debug!("Checking if timer {:?} is active", timer);
        let timer = unsafe { CompatTimer::from_ptr(timer) };
        CompatTimerQueue::with_global(|q| timer.is_active(q))
    }

    unsafe fn disarm(timer: TimerPtr) {
        debug!("Disarming {:?}", timer);
        let timer = unsafe { CompatTimer::from_ptr(timer) };
        CompatTimerQueue::with_global(|q| timer.disarm(q))
    }
}

register_timer_implementation!(CompatTimer);

/// Entry point for the timer task responsible for handling scheduled timer
/// events.
///
/// The timer task is created when the first timer is armed.
pub(crate) extern "C" fn timer_task(semaphore_ptr: *mut c_void) {
    let semaphore_ptr = NonNull::new(semaphore_ptr).unwrap().cast();
    let semaphore = unsafe { SemaphoreHandle::ref_from_ptr(&semaphore_ptr) };

    let queue = CompatTimerQueue::ensure_initialized();

    loop {
        queue.process(semaphore);
    }
}
