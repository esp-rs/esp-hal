//! Queues

use core::ptr::NonNull;

/// Pointer to an opaque queue created by the driver implementation.
pub type QueuePtr = NonNull<()>;

unsafe extern "Rust" {
    fn esp_preempt_queue_create(capacity: usize, item_size: usize) -> QueuePtr;
    fn esp_preempt_queue_delete(queue: QueuePtr);

    fn esp_preempt_queue_send_to_front(
        queue: QueuePtr,
        item: *const u8,
        timeout_us: Option<u32>,
    ) -> bool;
    fn esp_preempt_queue_send_to_back(
        queue: QueuePtr,
        item: *const u8,
        timeout_us: Option<u32>,
    ) -> bool;
    fn esp_preempt_queue_try_send_to_back(queue: QueuePtr, item: *const u8) -> bool;
    fn esp_preempt_queue_receive(queue: QueuePtr, item: *mut u8, timeout_us: Option<u32>) -> bool;
    fn esp_preempt_queue_try_receive(queue: QueuePtr, item: *mut u8) -> bool;
    fn esp_preempt_queue_remove(queue: QueuePtr, item: *const u8);
    fn esp_preempt_queue_messages_waiting(queue: QueuePtr) -> usize;
}

pub trait QueueImplementation {
    /// Creates a new queue instance.
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
    /// # Safety
    ///
    /// The caller must ensure that `item` can be dereferenced and points to an allocation of
    /// a size equal to the queue's item size.
    unsafe fn try_send_to_back(queue: QueuePtr, item: *const u8) -> bool;

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
    /// This function returns `true` if the item was successfully dequeued, `false` otherwise.
    ///
    /// # Safety
    ///
    /// The caller must ensure that `item` can be dereferenced and points to an allocation of
    /// a size equal to the queue's item size.
    unsafe fn try_receive(queue: QueuePtr, item: *mut u8) -> bool;

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
        fn esp_preempt_queue_create(capacity: usize, item_size: usize) -> $crate::queue::QueuePtr {
            <$t as $crate::queue::QueueImplementation>::create(capacity, item_size)
        }

        #[unsafe(no_mangle)]
        #[inline]
        fn esp_preempt_queue_delete(queue: $crate::queue::QueuePtr) {
            unsafe { <$t as $crate::queue::QueueImplementation>::delete(queue) }
        }

        #[unsafe(no_mangle)]
        #[inline]
        fn esp_preempt_queue_send_to_front(
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
        fn esp_preempt_queue_send_to_back(
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
        fn esp_preempt_queue_try_send_to_back(queue: QueuePtr, item: *const u8) -> bool {
            unsafe { <$t as $crate::queue::QueueImplementation>::try_send_to_back(queue, item) }
        }

        #[unsafe(no_mangle)]
        #[inline]
        fn esp_preempt_queue_receive(
            queue: QueuePtr,
            item: *mut u8,
            timeout_us: Option<u32>,
        ) -> bool {
            unsafe { <$t as $crate::queue::QueueImplementation>::receive(queue, item, timeout_us) }
        }

        #[unsafe(no_mangle)]
        #[inline]
        fn esp_preempt_queue_try_receive(queue: QueuePtr, item: *mut u8) -> bool {
            unsafe { <$t as $crate::queue::QueueImplementation>::try_receive(queue, item) }
        }

        #[unsafe(no_mangle)]
        #[inline]
        fn esp_preempt_queue_remove(queue: QueuePtr, item: *mut u8) {
            unsafe { <$t as $crate::queue::QueueImplementation>::remove(queue, item) }
        }

        #[unsafe(no_mangle)]
        #[inline]
        fn esp_preempt_queue_messages_waiting(queue: QueuePtr) -> usize {
            unsafe { <$t as $crate::queue::QueueImplementation>::messages_waiting(queue) }
        }
    };
}

#[repr(transparent)]
pub struct QueueHandle(QueuePtr);
impl QueueHandle {
    /// Creates a new queue instance.
    pub fn new(capacity: usize, item_size: usize) -> Self {
        let ptr = unsafe { esp_preempt_queue_create(capacity, item_size) };
        Self(ptr)
    }

    /// Converts this object into a pointer without dropping it.
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
    pub unsafe fn send_to_front(&self, item: *const u8, timeout_us: Option<u32>) -> bool {
        unsafe { esp_preempt_queue_send_to_front(self.0, item, timeout_us) }
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
    pub unsafe fn send_to_back(&self, item: *const u8, timeout_us: Option<u32>) -> bool {
        unsafe { esp_preempt_queue_send_to_back(self.0, item, timeout_us) }
    }

    /// Attempts to enqueues an item.
    ///
    /// If the queue is full, this function will immediately return `false`.
    ///
    /// # Safety
    ///
    /// The caller must ensure that `item` can be dereferenced and points to an allocation of
    /// a size equal to the queue's item size.
    pub unsafe fn try_send_to_back(&self, item: *const u8) -> bool {
        unsafe { esp_preempt_queue_try_send_to_back(self.0, item) }
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
    pub unsafe fn receive(&self, item: *mut u8, timeout_us: Option<u32>) -> bool {
        unsafe { esp_preempt_queue_receive(self.0, item, timeout_us) }
    }

    /// Attempts to dequeue an item from the queue.
    ///
    /// If the queue is empty, this function will return `false` immediately.
    ///
    /// This function returns `true` if the item was successfully dequeued, `false` otherwise.
    ///
    /// # Safety
    ///
    /// The caller must ensure that `item` can be dereferenced and points to an allocation of
    /// a size equal to the queue's item size.
    pub unsafe fn try_receive(&self, item: *mut u8) -> bool {
        unsafe { esp_preempt_queue_try_receive(self.0, item) }
    }

    /// Removes an item from the queue.
    ///
    /// # Safety
    ///
    /// The caller must ensure that `item` can be dereferenced and points to an allocation of
    /// a size equal to the queue's item size.
    pub unsafe fn remove(&self, item: *const u8) {
        unsafe { esp_preempt_queue_remove(self.0, item) }
    }

    /// Returns the number of messages in the queue.
    pub fn messages_waiting(&self) -> usize {
        unsafe { esp_preempt_queue_messages_waiting(self.0) }
    }
}

impl Drop for QueueHandle {
    fn drop(&mut self) {
        unsafe { esp_preempt_queue_delete(self.0) };
    }
}
