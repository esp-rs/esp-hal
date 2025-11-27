//! Timers (callbacks scheduled to run in the future)
//!
//! ## Implementation
//!
//! Implement the `TimerImplementation` trait for an object, and use the
//! `register_timer_implementation` to register that implementation for esp-radio.
//!
//! See the [`TimerImplementation`] documentation for more information.
//!
//! As an alternative, you may use the `CompatTimer` structure as your timer implementation.
//!
//! ## Usage
//!
//! Users should use [`TimerHandle`] to interact with timers created by the driver implementation.
//!
//! > Note that the only expected user of this crate is esp-radio.

use core::{ffi::c_void, ptr::NonNull};

/// Pointer to an opaque timer created by the driver implementation.
pub type TimerPtr = NonNull<()>;

unsafe extern "Rust" {
    fn esp_rtos_timer_create(
        function: unsafe extern "C" fn(*mut c_void),
        data: *mut c_void,
    ) -> TimerPtr;
    fn esp_rtos_timer_delete(timer: TimerPtr);

    fn esp_rtos_timer_arm(timer: TimerPtr, timeout: u64, periodic: bool);
    fn esp_rtos_timer_is_active(timer: TimerPtr) -> bool;
    fn esp_rtos_timer_disarm(timer: TimerPtr);
}

/// A timer implementation.
///
/// The following snippet demonstrates the boilerplate necessary to implement a timer using the
/// `TimerImplementation` trait:
///
/// ```rust,no_run
/// use esp_radio_rtos_driver::{
///     register_timer_implementation,
///     timer::{TimerImplementation, TimerPtr},
/// };
///
/// struct MyTimer {
///     // Timer implementation details
/// }
///
/// impl TimerImplementation for MyTimer {
///     fn create(function: unsafe extern "C" fn(*mut c_void), data: *mut c_void) -> TimerPtr {
///         unimplemented!()
///     }
///
///     unsafe fn delete(mutex: MutexPtr) {
///         unimplemented!()
///     }
///
///     unsafe fn arm(timer: TimerPtr, timeout: u64, periodic: bool) {
///         unimplemented!()
///     }
///
///     unsafe fn is_active(timer: TimerPtr) -> bool {
///         unimplemented!()
///     }
///
///     unsafe fn disarm(timer: TimerPtr) -> bool {
///         unimplemented!()
///     }
/// }
///
/// register_timer_implementation!(MyTimer);
/// ```
pub trait TimerImplementation {
    /// Creates a new timer instance from the given callback.
    fn create(function: unsafe extern "C" fn(*mut c_void), data: *mut c_void) -> TimerPtr;

    /// Deletes a timer instance.
    ///
    /// # Safety
    ///
    /// `timer` must be a pointer returned from [`Self::create`].
    unsafe fn delete(timer: TimerPtr);

    /// Configures the timer to be triggered after the given timeout.
    ///
    /// The timeout is specified in microsecond. If the timer is set to be periodic,
    /// the timer will be triggered with a constant frequency.
    ///
    /// # Safety
    ///
    /// `timer` must be a pointer returned from [`Self::create`].
    unsafe fn arm(timer: TimerPtr, timeout: u64, periodic: bool);

    /// Checks if the timer is currently active.
    ///
    /// # Safety
    ///
    /// `timer` must be a pointer returned from [`Self::create`].
    unsafe fn is_active(timer: TimerPtr) -> bool;

    /// Stops the timer.
    ///
    /// # Safety
    ///
    /// `timer` must be a pointer returned from [`Self::create`].
    unsafe fn disarm(timer: TimerPtr);
}

#[macro_export]
macro_rules! register_timer_implementation {
    ($t: ty) => {
        #[unsafe(no_mangle)]
        #[inline]
        fn esp_rtos_timer_create(
            function: unsafe extern "C" fn(*mut ::core::ffi::c_void),
            data: *mut ::core::ffi::c_void,
        ) -> $crate::timer::TimerPtr {
            <$t as $crate::timer::TimerImplementation>::create(function, data)
        }

        #[unsafe(no_mangle)]
        #[inline]
        fn esp_rtos_timer_delete(timer: $crate::timer::TimerPtr) {
            unsafe { <$t as $crate::timer::TimerImplementation>::delete(timer) }
        }

        #[unsafe(no_mangle)]
        #[inline]
        fn esp_rtos_timer_arm(timer: $crate::timer::TimerPtr, timeout: u64, periodic: bool) {
            unsafe { <$t as $crate::timer::TimerImplementation>::arm(timer, timeout, periodic) }
        }

        #[unsafe(no_mangle)]
        #[inline]
        fn esp_rtos_timer_is_active(timer: $crate::timer::TimerPtr) -> bool {
            unsafe { <$t as $crate::timer::TimerImplementation>::is_active(timer) }
        }

        #[unsafe(no_mangle)]
        #[inline]
        fn esp_rtos_timer_disarm(timer: $crate::timer::TimerPtr) {
            unsafe { <$t as $crate::timer::TimerImplementation>::disarm(timer) }
        }
    };
}

/// A timer handle.
///
/// This handle is used to interact with timers created by the driver implementation.
#[repr(transparent)]
pub struct TimerHandle(TimerPtr);
impl TimerHandle {
    /// Creates a new timer instance from the given callback.
    ///
    /// # Safety
    ///
    /// - The callback and its data must be valid for the lifetime of the timer.
    /// - The callback and its data need to be able to be sent across threads.
    #[inline]
    pub unsafe fn new(function: unsafe extern "C" fn(*mut c_void), data: *mut c_void) -> Self {
        Self(unsafe { esp_rtos_timer_create(function, data) })
    }

    /// Converts this object into a pointer without dropping it.
    #[inline]
    pub fn leak(self) -> TimerPtr {
        let ptr = self.0;
        core::mem::forget(self);
        ptr
    }

    /// Recovers the object from a leaked pointer.
    ///
    /// # Safety
    ///
    /// - The caller must only use pointers created using [`Self::leak`].
    /// - The caller must ensure the pointer is not shared.
    #[inline]
    pub unsafe fn from_ptr(ptr: TimerPtr) -> Self {
        Self(ptr)
    }

    /// Creates a reference to this object from a leaked pointer.
    ///
    /// This function is used in the esp-radio code to interact with the timer.
    ///
    /// # Safety
    ///
    /// - The caller must only use pointers created using [`Self::leak`].
    #[inline]
    pub unsafe fn ref_from_ptr(ptr: &TimerPtr) -> &Self {
        unsafe { core::mem::transmute(ptr) }
    }

    /// Configures the timer to be triggered after the given timeout.
    ///
    /// The timeout is specified in microsecond. If the timer is set to be periodic,
    /// the timer will be triggered with a constant frequency.
    #[inline]
    pub fn arm(&self, timeout: u64, periodic: bool) {
        unsafe { esp_rtos_timer_arm(self.0, timeout, periodic) }
    }

    /// Checks if the timer is currently active.
    #[inline]
    pub fn is_active(&self) -> bool {
        unsafe { esp_rtos_timer_is_active(self.0) }
    }

    /// Stops the timer.
    #[inline]
    pub fn disarm(&self) {
        unsafe { esp_rtos_timer_disarm(self.0) }
    }
}

impl Drop for TimerHandle {
    #[inline]
    fn drop(&mut self) {
        unsafe { esp_rtos_timer_delete(self.0) };
    }
}

#[cfg(feature = "ipc-implementations")]
mod implementation {
    use alloc::{boxed::Box, vec::Vec};
    use core::{
        cell::{RefCell, UnsafeCell},
        ptr::NonNull,
        sync::atomic::Ordering,
    };

    use portable_atomic::AtomicPtr;

    use super::*;
    use crate::semaphore::{SemaphoreHandle, SemaphoreKind, SemaphorePtr};

    // The following code implements a timer queue based solely on portable_atomic and blocks
    // defined in this crate.

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
                props.enqueued = true;

                props.next = head;
                self.head = Some(NonNull::from(timer));
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
                    crate::task_create(
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

            #[cold]
            #[inline(never)]
            fn initialize<'a>() -> &'a CompatTimerQueue {
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
                            // In case the queue is somehow still null, we will re-attempt storing
                            // our own pointer.
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

            if let Some(queue) = NonNull::new(TIMER_QUEUE.load(Ordering::Acquire)) {
                unsafe { queue.as_ref() }
            } else {
                initialize()
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
        /// The timer queue needs to be re-processed when a new timer is armed, because the new
        /// timer may need to be triggered before the next scheduled wakeup.
        fn process(&self, semaphore: &SemaphoreHandle) {
            let mut timers = self.with(|q| {
                q.processing = true;
                q.next_wakeup = u64::MAX;
                q.head.take()
            });

            while let Some(current) = timers {
                let current_timer = unsafe { current.as_ref() };

                let run_callback = self.with(|q| {
                    let props = current_timer.properties(q);

                    // Remove current timer from the list.
                    timers = props.next.take();
                    props.enqueued = false;

                    if !props.is_active {
                        return false;
                    }

                    if props.next_due > crate::now() {
                        // Not our time yet.
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
                    (current_timer.callback.borrow_mut())();
                }

                self.with(|q| {
                    let props = current_timer.properties(q);

                    if props.is_active {
                        q.enqueue(current_timer);
                    }
                });
            }

            let next_wakeup = self.with(|q| {
                while let Some(timer) = q.scheduled_for_drop.pop() {
                    let timer = unsafe { Box::from_raw(timer.cast::<CompatTimer>().as_ptr()) };
                    q.dequeue(&timer);
                    core::mem::drop(timer);
                }

                q.processing = false;
                q.next_wakeup
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

    /// A timer implementation that uses a background thread.
    ///
    /// This implementation uses thread and Semaphore APIs that the RTOS implementation provides.
    ///
    /// To use this implementation, add `register_timer_implementation!(CompatTimer)` to your
    /// implementation.
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

        fn arm(
            &self,
            q: &mut TimerQueueInner,
            timeout: u64,
            periodic: bool,
        ) -> Option<SemaphorePtr> {
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
            NonNull::from(Box::leak(timer)).cast()
        }

        unsafe fn delete(timer: TimerPtr) {
            let mut semaphore_to_give = None;
            CompatTimerQueue::with_global(|q| {
                // we don't drop the timer right now, since it might be
                // processed currently
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
            let timer = unsafe { CompatTimer::from_ptr(timer) };
            if let Some(semaphore_ptr) =
                CompatTimerQueue::with_global(|q| timer.arm(q, timeout, periodic))
            {
                let semaphore = unsafe { SemaphoreHandle::ref_from_ptr(&semaphore_ptr) };
                semaphore.give();
            }
        }

        unsafe fn is_active(timer: TimerPtr) -> bool {
            let timer = unsafe { CompatTimer::from_ptr(timer) };
            CompatTimerQueue::with_global(|q| timer.is_active(q))
        }

        unsafe fn disarm(timer: TimerPtr) {
            let timer = unsafe { CompatTimer::from_ptr(timer) };
            CompatTimerQueue::with_global(|q| timer.disarm(q))
        }
    }

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
}

#[cfg(feature = "ipc-implementations")]
pub use implementation::CompatTimer;
