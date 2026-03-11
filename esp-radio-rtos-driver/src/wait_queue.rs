//! Thread wait queues
//!
//! Wait queues are synchronization primitives that allow threads to be put to sleep until a
//! condition is met or a timeout occurs.
//!
//! ## Implementation
//!
//! Implement the `WaitQueueImplementation` trait for an object, and use the
//! `register_wait_queue_implementation` to register that implementation for esp-radio.
//!
//! See the [`WaitQueueImplementation`] documentation for more information.
//!
//! Note that this trait is only available and required if the `ipc-implementations` feature is
//! enabled.

use core::ptr::NonNull;

/// Pointer to an opaque wait queue created by the driver implementation.
pub type WaitQueuePtr = NonNull<()>;

unsafe extern "Rust" {
    fn esp_rtos_wait_queue_create() -> WaitQueuePtr;
    fn esp_rtos_wait_queue_delete(queue: WaitQueuePtr);

    fn esp_rtos_wait_queue_wait_until(queue: WaitQueuePtr, deadline_instant: Option<u64>);
    fn esp_rtos_wait_queue_notify(queue: WaitQueuePtr);
    fn esp_rtos_wait_queue_notify_from_isr(
        queue: WaitQueuePtr,
        higher_prio_task_waken: Option<&mut bool>,
    );
}

/// A thread wait queue.
///
/// The following snippet demonstrates the boilerplate necessary to implement a wait queue using the
/// `WaitQueueImplementation` trait:
///
/// ```rust,no_run
/// use esp_radio_rtos_driver::{
///     register_wait_queue_implementation,
///     wait_queue::{WaitQueueImplementation, WaitQueuePtr},
/// };
///
/// struct MyWaitQueue {
///     // Wait queue implementation details
/// }
///
/// impl WaitQueueImplementation for MyWaitQueue {
///     fn create() -> WaitQueuePtr {
///         unimplemented!()
///     }
///
///     unsafe fn delete(queue: WaitQueuePtr) {
///         unimplemented!()
///     }
///
///     unsafe fn wait_until(queue: WaitQueuePtr, deadline_instant: Option<u64>) {
///         unimplemented!()
///     }
///
///     unsafe fn notify(queue: WaitQueuePtr) {
///         unimplemented!()
///     }
///
///     unsafe fn notify_from_isr(queue: WaitQueuePtr, higher_prio_task_waken: Option<&mut bool>) {
///         unimplemented!()
///     }
/// }
///
/// register_wait_queue_implementation!(MyWaitQueue);
/// ```
pub trait WaitQueueImplementation {
    /// Creates a new wait queue instance.
    fn create() -> WaitQueuePtr;

    /// Deletes a wait queue instance.
    ///
    /// # Safety
    ///
    /// `queue` must be a pointer returned from [`Self::create`].
    unsafe fn delete(queue: WaitQueuePtr);

    /// Enqueues the current thread to wait for until the specified deadline, or for a notification.
    unsafe fn wait_until(queue: WaitQueuePtr, deadline_instant: Option<u64>);

    /// Notifies the wait queue that the highest priority thread can be scheduled.
    unsafe fn notify(queue: WaitQueuePtr);

    /// Notifies the wait queue from an ISR that the highest priority thread can be scheduled.
    ///
    /// The `higher_prio_task_waken` parameter is an optional mutable reference to a boolean flag.
    /// If the flag is `Some`, the implementation may set it to `true` to request a context switch.
    unsafe fn notify_from_isr(queue: WaitQueuePtr, higher_prio_task_waken: Option<&mut bool>);
}

#[macro_export]
macro_rules! register_wait_queue_implementation {
    ($t: ty) => {
        #[unsafe(no_mangle)]
        #[inline]
        fn esp_rtos_wait_queue_create() -> $crate::wait_queue::WaitQueuePtr {
            <$t as $crate::wait_queue::WaitQueueImplementation>::create()
        }

        #[unsafe(no_mangle)]
        #[inline]
        fn esp_rtos_wait_queue_delete(queue: $crate::wait_queue::WaitQueuePtr) {
            unsafe { <$t as $crate::wait_queue::WaitQueueImplementation>::delete(queue) }
        }

        #[unsafe(no_mangle)]
        #[inline]
        fn esp_rtos_wait_queue_wait_until(
            queue: $crate::wait_queue::WaitQueuePtr,
            deadline_instant: Option<u64>,
        ) {
            unsafe {
                <$t as $crate::wait_queue::WaitQueueImplementation>::wait_until(
                    queue,
                    deadline_instant,
                )
            }
        }

        #[unsafe(no_mangle)]
        #[inline]
        fn esp_rtos_wait_queue_notify(queue: $crate::wait_queue::WaitQueuePtr) {
            unsafe { <$t as $crate::wait_queue::WaitQueueImplementation>::notify(queue) }
        }

        #[unsafe(no_mangle)]
        #[inline]
        fn esp_rtos_wait_queue_notify_from_isr(
            queue: $crate::wait_queue::WaitQueuePtr,
            higher_prio_task_waken: Option<&mut bool>,
        ) {
            unsafe {
                <$t as $crate::wait_queue::WaitQueueImplementation>::notify_from_isr(
                    queue,
                    higher_prio_task_waken,
                )
            }
        }
    };
}

/// Wait queue handle.
///
/// This handle is used to interact with wait queues created by the driver implementation.
#[repr(transparent)]
pub struct WaitQueueHandle(WaitQueuePtr);
impl WaitQueueHandle {
    /// Creates a new wait queue instance.
    #[inline]
    pub fn new() -> Self {
        let ptr = unsafe { esp_rtos_wait_queue_create() };
        Self(ptr)
    }

    /// Enqueues the current thread to wait for until the specified deadline, or for a notification.
    #[inline]
    pub fn wait_until(&self, deadline_instant: Option<u64>) {
        unsafe { esp_rtos_wait_queue_wait_until(self.0, deadline_instant) }
    }

    /// Notifies the wait queue that the highest priority thread can be scheduled.
    #[inline]
    pub fn notify(&self) {
        unsafe { esp_rtos_wait_queue_notify(self.0) }
    }

    /// Notifies the wait queue from an ISR that the highest priority thread can be scheduled.
    #[inline]
    pub fn notify_from_isr(&self, higher_prio_task_waken: Option<&mut bool>) {
        unsafe { esp_rtos_wait_queue_notify_from_isr(self.0, higher_prio_task_waken) }
    }
}

impl Drop for WaitQueueHandle {
    #[inline]
    fn drop(&mut self) {
        unsafe { esp_rtos_wait_queue_delete(self.0) };
    }
}
