//! # Queues
//!
//! Queues are a synchronization primitive used to communicate between tasks.
//! They allow tasks to send and receive data in a first-in-first-out (FIFO) manner.
//!
//! ## Implementation
//!
//! Implement the `QueueImplementation` trait for an object, and use the
//! `register_queue_implementation` to register that implementation for esp-radio.
//!
//! See the [`QueueImplementation`] documentation for more information.
//!
//! ## Usage
//!
//! Users should use [`QueueHandle`] to interact with queues created by the driver implementation.
//!
//! > Note that the only expected user of this crate is esp-radio.

use core::ptr::NonNull;

/// Pointer to an opaque queue created by the driver implementation.
pub type QueuePtr = NonNull<()>;

unsafe extern "Rust" {
    fn esp_rtos_queue_create(capacity: usize, item_size: usize) -> QueuePtr;
    fn esp_rtos_queue_delete(queue: QueuePtr);

    fn esp_rtos_queue_send_to_front(
        queue: QueuePtr,
        item: *const u8,
        timeout_us: Option<u32>,
    ) -> bool;
    fn esp_rtos_queue_send_to_back(
        queue: QueuePtr,
        item: *const u8,
        timeout_us: Option<u32>,
    ) -> bool;
    fn esp_rtos_queue_try_send_to_back_from_isr(
        queue: QueuePtr,
        item: *const u8,
        higher_prio_task_waken: Option<&mut bool>,
    ) -> bool;
    fn esp_rtos_queue_receive(queue: QueuePtr, item: *mut u8, timeout_us: Option<u32>) -> bool;
    fn esp_rtos_queue_try_receive_from_isr(
        queue: QueuePtr,
        item: *mut u8,
        higher_prio_task_waken: Option<&mut bool>,
    ) -> bool;
    fn esp_rtos_queue_remove(queue: QueuePtr, item: *const u8);
    fn esp_rtos_queue_messages_waiting(queue: QueuePtr) -> usize;
}

/// A queue primitive.
///
/// The following snippet demonstrates the boilerplate necessary to implement a queue using the
/// `QueueImplementation` trait:
///
/// ```rust,no_run
/// use esp_radio_rtos_driver::{
///     queue::{QueueImplementation, QueuePtr},
///     register_queue_implementation,
/// };
///
/// struct MyQueue {
///     // Queue implementation details
/// }
///
/// impl QueueImplementation for MyQueue {
///     fn create(capacity: usize, item_size: usize) -> QueuePtr {
///         unimplemented!()
///     }
///
///     unsafe fn delete(queue: QueuePtr) {
///         unimplemented!()
///     }
///
///     unsafe fn send_to_front(queue: QueuePtr, item: *const u8, timeout_us: Option<u32>) -> bool {
///         unimplemented!()
///     }
///
///     unsafe fn send_to_back(queue: QueuePtr, item: *const u8, timeout_us: Option<u32>) -> bool {
///         unimplemented!()
///     }
///
///     unsafe fn try_send_to_back_from_isr(
///         queue: QueuePtr,
///         item: *const u8,
///         higher_prio_task_waken: Option<&mut bool>,
///     ) -> bool {
///         unimplemented!()
///     }
///
///     unsafe fn receive(queue: QueuePtr, item: *mut u8, timeout_us: Option<u32>) -> bool {
///         unimplemented!()
///     }
///
///     unsafe fn try_receive_from_isr(
///         queue: QueuePtr,
///         item: *mut u8,
///         higher_prio_task_waken: Option<&mut bool>,
///     ) -> bool {
///         unimplemented!()
///     }
///
///     unsafe fn remove(queue: QueuePtr, item: *const u8) {
///         unimplemented!()
///     }
///
///     fn messages_waiting(queue: QueuePtr) -> usize {
///         unimplemented!()
///     }
/// }
///
/// register_queue_implementation!(MyQueue);
/// ```
pub trait QueueImplementation {
    /// Creates a new, empty queue instance.
    ///
    /// The queue must have a capacity for `capacity` number of `item_size` byte items.
    fn create(capacity: usize, item_size: usize) -> QueuePtr;

    /// Deletes a queue instance.
    ///
    /// # Safety
    ///
    /// `queue` must be a pointer returned from [`Self::create`].
    unsafe fn delete(queue: QueuePtr);

    /// Enqueues a high-priority item.
    ///
    /// If the queue is full, this function will block for the given timeout. If timeout is None,
    /// the function will block indefinitely.
    ///
    /// This function returns `true` if the item was successfully enqueued, `false` otherwise.
    ///
    /// # Safety
    ///
    /// The caller must ensure that `item` can be dereferenced and points to an allocation of
    /// a size equal to the queue's item size.
    unsafe fn send_to_front(queue: QueuePtr, item: *const u8, timeout_us: Option<u32>) -> bool;

    /// Enqueues an item.
    ///
    /// If the queue is full, this function will block for the given timeout. If timeout is None,
    /// the function will block indefinitely.
    ///
    /// This function returns `true` if the item was successfully enqueued, `false` otherwise.
    ///
    /// # Safety
    ///
    /// The caller must ensure that `item` can be dereferenced and points to an allocation of
    /// a size equal to the queue's item size.
    unsafe fn send_to_back(queue: QueuePtr, item: *const u8, timeout_us: Option<u32>) -> bool;

    /// Attempts to enqueues an item.
    ///
    /// If the queue is full, this function will immediately return `false`.
    ///
    /// The `higher_prio_task_waken` parameter is an optional mutable reference to a boolean flag.
    /// If the flag is `Some`, the implementation may set it to `true` to request a context switch.
    ///
    /// # Safety
    ///
    /// The caller must ensure that `item` can be dereferenced and points to an allocation of
    /// a size equal to the queue's item size.
    unsafe fn try_send_to_back_from_isr(
        queue: QueuePtr,
        item: *const u8,
        higher_prio_task_waken: Option<&mut bool>,
    ) -> bool;

    /// Dequeues an item from the queue.
    ///
    /// If the queue is empty, this function will block for the given timeout. If timeout is None,
    /// the function will block indefinitely.
    ///
    /// This function returns `true` if the item was successfully dequeued, `false` otherwise.
    ///
    /// # Safety
    ///
    /// The caller must ensure that `item` can be dereferenced and points to an allocation of
    /// a size equal to the queue's item size.
    unsafe fn receive(queue: QueuePtr, item: *mut u8, timeout_us: Option<u32>) -> bool;

    /// Attempts to dequeue an item from the queue.
    ///
    /// If the queue is empty, this function will return `false` immediately.
    ///
    /// The `higher_prio_task_waken` parameter is an optional mutable reference to a boolean flag.
    /// If the flag is `Some`, the implementation may set it to `true` to request a context switch.
    ///
    /// This function returns `true` if the item was successfully dequeued, `false` otherwise.
    ///
    /// # Safety
    ///
    /// The caller must ensure that `item` can be dereferenced and points to an allocation of
    /// a size equal to the queue's item size.
    unsafe fn try_receive_from_isr(
        queue: QueuePtr,
        item: *mut u8,
        higher_prio_task_waken: Option<&mut bool>,
    ) -> bool;

    /// Removes an item from the queue.
    ///
    /// # Safety
    ///
    /// The caller must ensure that `item` can be dereferenced and points to an allocation of
    /// a size equal to the queue's item size.
    unsafe fn remove(queue: QueuePtr, item: *const u8);

    /// Returns the number of messages in the queue.
    fn messages_waiting(queue: QueuePtr) -> usize;
}

#[macro_export]
macro_rules! register_queue_implementation {
    ($t: ty) => {
        #[unsafe(no_mangle)]
        #[inline]
        fn esp_rtos_queue_create(capacity: usize, item_size: usize) -> $crate::queue::QueuePtr {
            <$t as $crate::queue::QueueImplementation>::create(capacity, item_size)
        }

        #[unsafe(no_mangle)]
        #[inline]
        fn esp_rtos_queue_delete(queue: $crate::queue::QueuePtr) {
            unsafe { <$t as $crate::queue::QueueImplementation>::delete(queue) }
        }

        #[unsafe(no_mangle)]
        #[inline]
        fn esp_rtos_queue_send_to_front(
            queue: QueuePtr,
            item: *const u8,
            timeout_us: Option<u32>,
        ) -> bool {
            unsafe {
                <$t as $crate::queue::QueueImplementation>::send_to_front(queue, item, timeout_us)
            }
        }

        #[unsafe(no_mangle)]
        #[inline]
        fn esp_rtos_queue_send_to_back(
            queue: QueuePtr,
            item: *const u8,
            timeout_us: Option<u32>,
        ) -> bool {
            unsafe {
                <$t as $crate::queue::QueueImplementation>::send_to_back(queue, item, timeout_us)
            }
        }

        #[unsafe(no_mangle)]
        #[inline]
        fn esp_rtos_queue_try_send_to_back_from_isr(
            queue: QueuePtr,
            item: *const u8,
            higher_prio_task_waken: Option<&mut bool>,
        ) -> bool {
            unsafe {
                <$t as $crate::queue::QueueImplementation>::try_send_to_back_from_isr(
                    queue,
                    item,
                    higher_prio_task_waken,
                )
            }
        }

        #[unsafe(no_mangle)]
        #[inline]
        fn esp_rtos_queue_receive(queue: QueuePtr, item: *mut u8, timeout_us: Option<u32>) -> bool {
            unsafe { <$t as $crate::queue::QueueImplementation>::receive(queue, item, timeout_us) }
        }

        #[unsafe(no_mangle)]
        #[inline]
        fn esp_rtos_queue_try_receive_from_isr(
            queue: QueuePtr,
            item: *mut u8,
            higher_prio_task_waken: Option<&mut bool>,
        ) -> bool {
            unsafe {
                <$t as $crate::queue::QueueImplementation>::try_receive_from_isr(
                    queue,
                    item,
                    higher_prio_task_waken,
                )
            }
        }

        #[unsafe(no_mangle)]
        #[inline]
        fn esp_rtos_queue_remove(queue: QueuePtr, item: *mut u8) {
            unsafe { <$t as $crate::queue::QueueImplementation>::remove(queue, item) }
        }

        #[unsafe(no_mangle)]
        #[inline]
        fn esp_rtos_queue_messages_waiting(queue: QueuePtr) -> usize {
            unsafe { <$t as $crate::queue::QueueImplementation>::messages_waiting(queue) }
        }
    };
}

/// Queue handle.
///
/// This handle is used to interact with queues created by the driver implementation.
#[repr(transparent)]
pub struct QueueHandle(QueuePtr);
impl QueueHandle {
    /// Creates a new queue instance.
    #[inline]
    pub fn new(capacity: usize, item_size: usize) -> Self {
        let ptr = unsafe { esp_rtos_queue_create(capacity, item_size) };
        Self(ptr)
    }

    /// Converts this object into a pointer without dropping it.
    #[inline]
    pub fn leak(self) -> QueuePtr {
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
    pub unsafe fn from_ptr(ptr: QueuePtr) -> Self {
        Self(ptr)
    }

    /// Creates a reference to this object from a leaked pointer.
    ///
    /// This function is used in the esp-radio code to interact with the queue.
    ///
    /// # Safety
    ///
    /// - The caller must only use pointers created using [`Self::leak`].
    #[inline]
    pub unsafe fn ref_from_ptr(ptr: &QueuePtr) -> &Self {
        unsafe { core::mem::transmute(ptr) }
    }

    /// Enqueues a high-priority item.
    ///
    /// If the queue is full, this function will block for the given timeout. If timeout is None,
    /// the function will block indefinitely.
    ///
    /// This function returns `true` if the item was successfully enqueued, `false` otherwise.
    ///
    /// # Safety
    ///
    /// The caller must ensure that `item` can be dereferenced and points to an allocation of
    /// a size equal to the queue's item size.
    #[inline]
    pub unsafe fn send_to_front(&self, item: *const u8, timeout_us: Option<u32>) -> bool {
        unsafe { esp_rtos_queue_send_to_front(self.0, item, timeout_us) }
    }

    /// Enqueues an item.
    ///
    /// If the queue is full, this function will block for the given timeout. If timeout is None,
    /// the function will block indefinitely.
    ///
    /// This function returns `true` if the item was successfully enqueued, `false` otherwise.
    ///
    /// # Safety
    ///
    /// The caller must ensure that `item` can be dereferenced and points to an allocation of
    /// a size equal to the queue's item size.
    #[inline]
    pub unsafe fn send_to_back(&self, item: *const u8, timeout_us: Option<u32>) -> bool {
        unsafe { esp_rtos_queue_send_to_back(self.0, item, timeout_us) }
    }

    /// Attempts to enqueues an item.
    ///
    /// If the queue is full, this function will immediately return `false`.
    ///
    /// If a higher priority task is woken up by this operation, the `higher_prio_task_waken` flag
    /// is set to `true`.
    ///
    /// # Safety
    ///
    /// The caller must ensure that `item` can be dereferenced and points to an allocation of
    /// a size equal to the queue's item size.
    #[inline]
    pub unsafe fn try_send_to_back_from_isr(
        &self,
        item: *const u8,
        higher_priority_task_waken: Option<&mut bool>,
    ) -> bool {
        unsafe {
            esp_rtos_queue_try_send_to_back_from_isr(self.0, item, higher_priority_task_waken)
        }
    }

    /// Dequeues an item from the queue.
    ///
    /// If the queue is empty, this function will block for the given timeout. If timeout is None,
    /// the function will block indefinitely.
    ///
    /// This function returns `true` if the item was successfully dequeued, `false` otherwise.
    ///
    /// # Safety
    ///
    /// The caller must ensure that `item` can be dereferenced and points to an allocation of
    /// a size equal to the queue's item size.
    #[inline]
    pub unsafe fn receive(&self, item: *mut u8, timeout_us: Option<u32>) -> bool {
        unsafe { esp_rtos_queue_receive(self.0, item, timeout_us) }
    }

    /// Attempts to dequeue an item from the queue.
    ///
    /// If the queue is empty, this function will return `false` immediately.
    ///
    /// This function returns `true` if the item was successfully dequeued, `false` otherwise.
    ///
    /// If a higher priority task is woken up by this operation, the `higher_prio_task_waken` flag
    /// is set to `true`.
    ///
    /// # Safety
    ///
    /// The caller must ensure that `item` can be dereferenced and points to an allocation of
    /// a size equal to the queue's item size.
    #[inline]
    pub unsafe fn try_receive_from_isr(
        &self,
        item: *mut u8,
        higher_priority_task_waken: Option<&mut bool>,
    ) -> bool {
        unsafe { esp_rtos_queue_try_receive_from_isr(self.0, item, higher_priority_task_waken) }
    }

    /// Removes an item from the queue.
    ///
    /// # Safety
    ///
    /// The caller must ensure that `item` can be dereferenced and points to an allocation of
    /// a size equal to the queue's item size.
    #[inline]
    pub unsafe fn remove(&self, item: *const u8) {
        unsafe { esp_rtos_queue_remove(self.0, item) }
    }

    /// Returns the number of messages in the queue.
    #[inline]
    pub fn messages_waiting(&self) -> usize {
        unsafe { esp_rtos_queue_messages_waiting(self.0) }
    }
}

impl Drop for QueueHandle {
    #[inline]
    fn drop(&mut self) {
        unsafe { esp_rtos_queue_delete(self.0) };
    }
}
