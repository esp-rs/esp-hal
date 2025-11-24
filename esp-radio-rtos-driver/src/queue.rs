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
//! You may also choose to use the [`CompatQueue`] implementation provided by this crate.
//!
//! ## Usage
//!
//! Users should use [`QueueHandle`] to interact with queues created by the driver implementation.
//!
//! > Note that the only expected user of this crate is esp-radio.

use core::{cell::UnsafeCell, ptr::NonNull};

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
    fn esp_rtos_queue_send_to_front_with_deadline(
        queue: QueuePtr,
        item: *const u8,
        deadline_instant: Option<u64>,
    ) -> bool;

    fn esp_rtos_queue_send_to_back(
        queue: QueuePtr,
        item: *const u8,
        timeout_us: Option<u32>,
    ) -> bool;
    fn esp_rtos_queue_send_to_back_with_deadline(
        queue: QueuePtr,
        item: *const u8,
        deadline_instant: Option<u64>,
    ) -> bool;

    fn esp_rtos_queue_try_send_to_back_from_isr(
        queue: QueuePtr,
        item: *const u8,
        higher_prio_task_waken: Option<&mut bool>,
    ) -> bool;
    fn esp_rtos_queue_receive(queue: QueuePtr, item: *mut u8, timeout_us: Option<u32>) -> bool;
    fn esp_rtos_queue_receive_with_deadline(
        queue: QueuePtr,
        item: *mut u8,
        deadline_instant: Option<u64>,
    ) -> bool;
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

    /// Enqueues a high-priority item.
    ///
    /// If the queue is full, this function will block until the deadline is reached. If the
    /// deadline is None, the function will block indefinitely.
    ///
    /// This function returns `true` if the item was successfully enqueued, `false` otherwise.
    ///
    /// # Safety
    ///
    /// The caller must ensure that `item` can be dereferenced and points to an allocation of
    /// a size equal to the queue's item size.
    unsafe fn send_to_front_with_deadline(
        queue: QueuePtr,
        item: *const u8,
        deadline_instant: Option<u64>,
    ) -> bool;

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

    /// Enqueues an item.
    ///
    /// If the queue is full, this function will block until the given deadline. If deadline is
    /// None, the function will block indefinitely.
    ///
    /// This function returns `true` if the item was successfully enqueued, `false` otherwise.
    ///
    /// # Safety
    ///
    /// The caller must ensure that `item` can be dereferenced and points to an allocation of
    /// a size equal to the queue's item size.
    unsafe fn send_to_back_with_deadline(
        queue: QueuePtr,
        item: *const u8,
        deadline_instant: Option<u64>,
    ) -> bool;

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

    /// Dequeues an item from the queue.
    ///
    /// If the queue is empty, this function will block until the given deadline is reached. If the
    /// deadline is None, the function will block indefinitely.
    ///
    /// This function returns `true` if the item was successfully dequeued, `false` otherwise.
    ///
    /// # Safety
    ///
    /// The caller must ensure that `item` can be dereferenced and points to an allocation of
    /// a size equal to the queue's item size.
    unsafe fn receive_with_deadline(
        queue: QueuePtr,
        item: *mut u8,
        deadline_instant: Option<u64>,
    ) -> bool;

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
        fn esp_rtos_queue_send_to_front_with_deadline(
            queue: QueuePtr,
            item: *const u8,
            deadline_instant: Option<u64>,
        ) -> bool {
            unsafe {
                <$t as $crate::queue::QueueImplementation>::send_to_front_with_deadline(
                    queue,
                    item,
                    deadline_instant,
                )
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
        fn esp_rtos_queue_send_to_back_with_deadline(
            queue: QueuePtr,
            item: *const u8,
            deadline_instant: Option<u64>,
        ) -> bool {
            unsafe {
                <$t as $crate::queue::QueueImplementation>::send_to_back_with_deadline(
                    queue,
                    item,
                    deadline_instant,
                )
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
        fn esp_rtos_queue_receive_with_deadline(
            queue: QueuePtr,
            item: *mut u8,
            deadline_instant: Option<u64>,
        ) -> bool {
            unsafe {
                <$t as $crate::queue::QueueImplementation>::receive_with_deadline(
                    queue,
                    item,
                    deadline_instant,
                )
            }
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

    /// Enqueues a high-priority item.
    ///
    /// If the queue is full, this function will block until the deadline is reached. If the
    /// deadline is None, the function will block indefinitely.
    ///
    /// This function returns `true` if the item was successfully enqueued, `false` otherwise.
    ///
    /// # Safety
    ///
    /// The caller must ensure that `item` can be dereferenced and points to an allocation of
    /// a size equal to the queue's item size.
    #[inline]
    pub unsafe fn send_to_front_with_deadline(
        &self,
        item: *const u8,
        deadline_instant: Option<u64>,
    ) -> bool {
        unsafe { esp_rtos_queue_send_to_front_with_deadline(self.0, item, deadline_instant) }
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

    /// Enqueues an item.
    ///
    /// If the queue is full, this function will block until the given deadline. If deadline is
    /// None, the function will block indefinitely.
    ///
    /// This function returns `true` if the item was successfully enqueued, `false` otherwise.
    ///
    /// # Safety
    ///
    /// The caller must ensure that `item` can be dereferenced and points to an allocation of
    /// a size equal to the queue's item size.
    #[inline]
    pub unsafe fn send_to_back_with_deadline(
        &self,
        item: *const u8,
        deadline_instant: Option<u64>,
    ) -> bool {
        unsafe { esp_rtos_queue_send_to_back_with_deadline(self.0, item, deadline_instant) }
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

    /// Dequeues an item from the queue.
    ///
    /// If the queue is empty, this function will block until the given deadline is reached. If
    /// deadline is None, the function will block indefinitely.
    ///
    /// This function returns `true` if the item was successfully dequeued, `false` otherwise.
    ///
    /// # Safety
    ///
    /// The caller must ensure that `item` can be dereferenced and points to an allocation of
    /// a size equal to the queue's item size.
    #[inline]
    pub unsafe fn receive_with_deadline(
        &self,
        item: *mut u8,
        deadline_instant: Option<u64>,
    ) -> bool {
        unsafe { esp_rtos_queue_receive_with_deadline(self.0, item, deadline_instant) }
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

use alloc::{boxed::Box, vec};

use crate::{
    now,
    semaphore::{SemaphoreHandle, SemaphoreKind},
};

struct QueueInner {
    storage: Box<[u8]>,
    item_size: usize,
    capacity: usize,
    count: usize,
    current_read: usize,
    current_write: usize,
}

impl QueueInner {
    fn get(&self, index: usize) -> &[u8] {
        let item_start = self.item_size * index;
        &self.storage[item_start..][..self.item_size]
    }

    fn get_mut(&mut self, index: usize) -> &mut [u8] {
        let item_start = self.item_size * index;
        &mut self.storage[item_start..][..self.item_size]
    }

    fn len(&self) -> usize {
        self.count
    }

    fn send_to_back(&mut self, item: *const u8) {
        let item = unsafe { core::slice::from_raw_parts(item, self.item_size) };

        let dst = self.get_mut(self.current_write);
        dst.copy_from_slice(item);

        self.current_write = (self.current_write + 1) % self.capacity;
        self.count += 1;
    }

    fn send_to_front(&mut self, item: *const u8) {
        let item = unsafe { core::slice::from_raw_parts(item, self.item_size) };

        self.current_read = (self.current_read + self.capacity - 1) % self.capacity;

        let dst = self.get_mut(self.current_read);
        dst.copy_from_slice(item);

        self.count += 1;
    }

    fn read_from_front(&mut self, dst: *mut u8) {
        let dst = unsafe { core::slice::from_raw_parts_mut(dst, self.item_size) };

        let src = self.get(self.current_read);
        dst.copy_from_slice(src);

        self.current_read = (self.current_read + 1) % self.capacity;
        self.count -= 1;
    }

    fn remove(&mut self, item: *const u8) -> bool {
        let count = self.len();

        if count == 0 {
            return false;
        }

        let mut tmp_item = vec![0; self.item_size];

        let mut found = false;
        let item_slice = unsafe { core::slice::from_raw_parts(item, self.item_size) };
        for _ in 0..count {
            self.read_from_front(tmp_item.as_mut_ptr().cast());

            if found || &tmp_item[..] != item_slice {
                self.send_to_back(tmp_item.as_mut_ptr().cast());
            } else {
                found = true;
            }

            // Note that even if we find our item, we'll need to keep cycling through everything to
            // keep insertion order.
        }

        found
    }
}

/// A suitable queue implementation that only requires semaphores from the OS.
///
/// Register in your OS implementation by adding the following code:
///
/// ```rust
/// use esp_radio_rtos_driver::{queue::CompatQueue, register_queue_implementation};
///
/// register_queue_implementation!(CompatQueue);
/// ```
pub struct CompatQueue {
    /// Allows interior mutability for the queue's inner state, when the mutex is held.
    inner: UnsafeCell<QueueInner>,

    semaphore_empty: SemaphoreHandle,
    semaphore_full: SemaphoreHandle,
    mutex: SemaphoreHandle,
}

impl CompatQueue {
    fn new(capacity: usize, item_size: usize) -> Self {
        let storage = vec![0; capacity * item_size].into_boxed_slice();
        let semaphore_empty = SemaphoreHandle::new(SemaphoreKind::Counting {
            max: capacity as u32,
            initial: capacity as u32,
        });
        let semaphore_full = SemaphoreHandle::new(SemaphoreKind::Counting {
            max: capacity as u32,
            initial: 0,
        });
        let mutex = SemaphoreHandle::new(SemaphoreKind::Mutex);
        Self {
            inner: UnsafeCell::new(QueueInner {
                storage,
                item_size,
                capacity,
                count: 0,
                current_read: 0,
                current_write: 0,
            }),
            semaphore_empty,
            semaphore_full,
            mutex,
        }
    }

    unsafe fn from_ptr<'a>(ptr: QueuePtr) -> &'a Self {
        unsafe { ptr.cast::<Self>().as_ref() }
    }
}

impl QueueImplementation for CompatQueue {
    fn create(capacity: usize, item_size: usize) -> QueuePtr {
        let q = Box::new(CompatQueue::new(capacity, item_size));
        NonNull::from(Box::leak(q)).cast()
    }

    unsafe fn delete(queue: QueuePtr) {
        let q = unsafe { Box::from_raw(queue.cast::<CompatQueue>().as_ptr()) };
        core::mem::drop(q);
    }

    unsafe fn send_to_front(queue: QueuePtr, item: *const u8, timeout_us: Option<u32>) -> bool {
        let deadline_instant = timeout_us.map(|timeout| now() + timeout as u64);
        unsafe { Self::send_to_front_with_deadline(queue, item, deadline_instant) }
    }

    unsafe fn send_to_front_with_deadline(
        queue: QueuePtr,
        item: *const u8,
        deadline_instant: Option<u64>,
    ) -> bool {
        let queue = unsafe { CompatQueue::from_ptr(queue) };

        if queue.semaphore_empty.take_with_deadline(deadline_instant) {
            // The inner mutex shouldn't be held for a long time, but we still shouldn't block
            // indefinitely.
            if queue.mutex.take_with_deadline(deadline_instant) {
                let inner = unsafe { &mut *queue.inner.get() };
                inner.send_to_front(item);

                queue.mutex.give();
                queue.semaphore_full.give();
                true
            } else {
                queue.semaphore_empty.give();
                false
            }
        } else {
            false
        }
    }

    unsafe fn send_to_back(queue: QueuePtr, item: *const u8, timeout_us: Option<u32>) -> bool {
        let deadline_instant = timeout_us.map(|timeout| now() + timeout as u64);
        unsafe { Self::send_to_back_with_deadline(queue, item, deadline_instant) }
    }

    unsafe fn send_to_back_with_deadline(
        queue: QueuePtr,
        item: *const u8,
        deadline_instant: Option<u64>,
    ) -> bool {
        let queue = unsafe { CompatQueue::from_ptr(queue) };

        if queue.semaphore_empty.take_with_deadline(deadline_instant) {
            // The inner mutex shouldn't be held for a long time, but we still shouldn't block
            // indefinitely.
            if queue.mutex.take_with_deadline(deadline_instant) {
                let inner = unsafe { &mut *queue.inner.get() };
                inner.send_to_back(item);

                queue.mutex.give();
                queue.semaphore_full.give();
                true
            } else {
                queue.semaphore_empty.give();
                false
            }
        } else {
            false
        }
    }

    unsafe fn try_send_to_back_from_isr(
        queue: QueuePtr,
        item: *const u8,
        mut higher_prio_task_waken: Option<&mut bool>,
    ) -> bool {
        let queue = unsafe { CompatQueue::from_ptr(queue) };

        if queue
            .semaphore_empty
            .try_take_from_isr(higher_prio_task_waken.as_deref_mut())
        {
            if queue
                .mutex
                .try_take_from_isr(higher_prio_task_waken.as_deref_mut())
            {
                let inner = unsafe { &mut *queue.inner.get() };
                inner.send_to_back(item);

                queue
                    .mutex
                    .try_give_from_isr(higher_prio_task_waken.as_deref_mut());
                queue
                    .semaphore_full
                    .try_give_from_isr(higher_prio_task_waken);
                true
            } else {
                queue
                    .semaphore_empty
                    .try_give_from_isr(higher_prio_task_waken);
                false
            }
        } else {
            false
        }
    }

    unsafe fn receive(queue: QueuePtr, item: *mut u8, timeout_us: Option<u32>) -> bool {
        let deadline_instant = timeout_us.map(|timeout| now() + timeout as u64);
        unsafe { Self::receive_with_deadline(queue, item, deadline_instant) }
    }

    unsafe fn receive_with_deadline(
        queue: QueuePtr,
        item: *mut u8,
        deadline_instant: Option<u64>,
    ) -> bool {
        let queue = unsafe { CompatQueue::from_ptr(queue) };

        if queue.semaphore_full.take_with_deadline(deadline_instant) {
            if queue.mutex.take_with_deadline(deadline_instant) {
                let inner = unsafe { &mut *queue.inner.get() };
                inner.read_from_front(item);

                queue.mutex.give();
                queue.semaphore_empty.give();
                true
            } else {
                queue.semaphore_full.give();
                false
            }
        } else {
            false
        }
    }

    unsafe fn try_receive_from_isr(
        queue: QueuePtr,
        item: *mut u8,
        mut higher_prio_task_waken: Option<&mut bool>,
    ) -> bool {
        let queue = unsafe { CompatQueue::from_ptr(queue) };

        if queue
            .semaphore_full
            .try_take_from_isr(higher_prio_task_waken.as_deref_mut())
        {
            if queue
                .mutex
                .try_take_from_isr(higher_prio_task_waken.as_deref_mut())
            {
                let inner = unsafe { &mut *queue.inner.get() };
                inner.read_from_front(item);

                queue
                    .mutex
                    .try_give_from_isr(higher_prio_task_waken.as_deref_mut());
                queue
                    .semaphore_empty
                    .try_give_from_isr(higher_prio_task_waken);
                true
            } else {
                queue
                    .semaphore_full
                    .try_give_from_isr(higher_prio_task_waken);
                false
            }
        } else {
            false
        }
    }

    unsafe fn remove(queue: QueuePtr, item: *const u8) {
        let queue = unsafe { CompatQueue::from_ptr(queue) };

        if queue.semaphore_full.take(Some(0)) {
            queue.mutex.take(None);

            let inner = unsafe { &mut *queue.inner.get() };
            let item_removed = inner.remove(item);

            queue.mutex.give();

            if item_removed {
                queue.semaphore_empty.give();
            } else {
                queue.semaphore_full.give();
            }
        }
    }

    fn messages_waiting(queue: QueuePtr) -> usize {
        let queue = unsafe { CompatQueue::from_ptr(queue) };

        queue.semaphore_full.current_count() as usize
    }
}
