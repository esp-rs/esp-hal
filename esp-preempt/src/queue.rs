use alloc::{boxed::Box, vec};
use core::ptr::NonNull;

use esp_hal::time::{Duration, Instant};
use esp_radio_preempt_driver::{
    queue::{QueueImplementation, QueuePtr},
    register_queue_implementation,
    yield_task,
};
use esp_sync::NonReentrantMutex;

struct QueueInner {
    storage: Box<[u8]>,
    item_size: usize,
    capacity: usize,
    current_read: usize,
    current_write: usize,
}

impl QueueInner {
    fn new(capacity: usize, item_size: usize) -> Self {
        Self {
            item_size,
            capacity,
            current_read: 0,
            current_write: 0,
            storage: vec![0; capacity * item_size].into_boxed_slice(),
        }
    }

    fn get(&self, index: usize) -> &[u8] {
        let item_start = self.item_size * index;
        &self.storage[item_start..][..self.item_size]
    }

    fn get_mut(&mut self, index: usize) -> &mut [u8] {
        let item_start = self.item_size * index;
        &mut self.storage[item_start..][..self.item_size]
    }

    unsafe fn try_enqueue(&mut self, item: *const u8) -> bool {
        if self.len() == self.capacity {
            return false;
        }

        let item = unsafe { core::slice::from_raw_parts(item, self.item_size) };

        let dst = self.get_mut(self.current_write);
        dst.copy_from_slice(item);

        self.current_write = (self.current_write + 1) % self.capacity;

        true
    }

    unsafe fn try_dequeue(&mut self, dst: *mut u8) -> bool {
        if self.len() == 0 {
            return false;
        }

        let dst = unsafe { core::slice::from_raw_parts_mut(dst, self.item_size) };

        let src = self.get(self.current_read);
        dst.copy_from_slice(src);

        self.current_read = (self.current_read + 1) % self.capacity;

        true
    }

    unsafe fn remove(&mut self, item: *const u8) {
        // do what the ESP-IDF implementations does...
        // just remove all elements and add them back except the one we need to remove -
        // good enough for now
        let count = self.len();

        if count == 0 {
            return;
        }

        let mut tmp_item = vec![0; self.item_size];

        let item_slice = unsafe { core::slice::from_raw_parts(item, self.item_size) };
        for _ in 0..count {
            if !unsafe { self.try_dequeue(tmp_item.as_mut_ptr().cast()) } {
                break;
            }
            if &tmp_item[..] != item_slice {
                _ = unsafe { self.try_enqueue(tmp_item.as_mut_ptr().cast()) };
            }
            // Note that even if we find our item, we'll need to keep cycling through everything to
            // keep insertion order.
        }
    }

    fn len(&self) -> usize {
        if self.current_write >= self.current_read {
            self.current_write - self.current_read
        } else {
            self.capacity - self.current_read + self.current_write
        }
    }
}

pub struct Queue {
    inner: NonReentrantMutex<QueueInner>,
}

impl Queue {
    pub fn new(capacity: usize, item_size: usize) -> Self {
        Queue {
            inner: NonReentrantMutex::new(QueueInner::new(capacity, item_size)),
        }
    }

    unsafe fn from_ptr<'a>(ptr: QueuePtr) -> &'a Self {
        unsafe { ptr.cast::<Self>().as_ref() }
    }

    fn yield_loop_with_timeout(timeout_us: Option<u32>, mut cb: impl FnMut() -> bool) -> bool {
        let start = if timeout_us.is_some() {
            Instant::now()
        } else {
            Instant::EPOCH
        };

        let timeout = timeout_us
            .map(|us| Duration::from_micros(us as u64))
            .unwrap_or(Duration::MAX);

        loop {
            if cb() {
                return true;
            }

            if timeout_us.is_some() && start.elapsed() > timeout {
                return false;
            }

            yield_task();
        }
    }

    unsafe fn send_to_back(&self, item: *const u8, timeout_us: Option<u32>) -> bool {
        Self::yield_loop_with_timeout(timeout_us, || unsafe { self.try_send_to_back(item) })
    }

    unsafe fn try_send_to_back(&self, item: *const u8) -> bool {
        self.inner.with(|queue| unsafe { queue.try_enqueue(item) })
    }

    unsafe fn receive(&self, item: *mut u8, timeout_us: Option<u32>) -> bool {
        Self::yield_loop_with_timeout(timeout_us, || unsafe { self.try_receive(item) })
    }

    unsafe fn try_receive(&self, item: *mut u8) -> bool {
        self.inner.with(|queue| unsafe { queue.try_dequeue(item) })
    }

    unsafe fn remove(&self, item: *const u8) {
        self.inner.with(|queue| unsafe { queue.remove(item) })
    }

    fn messages_waiting(&self) -> usize {
        self.inner.with(|queue| queue.len())
    }
}

impl QueueImplementation for Queue {
    fn create(capacity: usize, item_size: usize) -> QueuePtr {
        let q = Box::new(Queue::new(capacity, item_size));
        NonNull::from(Box::leak(q)).cast()
    }

    unsafe fn delete(queue: QueuePtr) {
        let q = unsafe { Box::from_raw(queue.cast::<Queue>().as_ptr()) };
        core::mem::drop(q);
    }

    unsafe fn send_to_front(_queue: QueuePtr, _item: *const u8, _timeout_us: Option<u32>) -> bool {
        unimplemented!()
    }

    unsafe fn send_to_back(queue: QueuePtr, item: *const u8, timeout_us: Option<u32>) -> bool {
        let queue = unsafe { Queue::from_ptr(queue) };

        unsafe { queue.send_to_back(item, timeout_us) }
    }

    unsafe fn try_send_to_back_from_isr(
        queue: QueuePtr,
        item: *const u8,
        _higher_prio_task_waken: Option<&mut bool>,
    ) -> bool {
        let queue = unsafe { Queue::from_ptr(queue) };

        unsafe { queue.try_send_to_back(item) }
    }

    unsafe fn receive(queue: QueuePtr, item: *mut u8, timeout_us: Option<u32>) -> bool {
        let queue = unsafe { Queue::from_ptr(queue) };

        unsafe { queue.receive(item, timeout_us) }
    }

    unsafe fn try_receive_from_isr(
        queue: QueuePtr,
        item: *mut u8,
        _higher_prio_task_waken: Option<&mut bool>,
    ) -> bool {
        let queue = unsafe { Queue::from_ptr(queue) };

        unsafe { queue.try_receive(item) }
    }

    unsafe fn remove(queue: QueuePtr, item: *const u8) {
        let queue = unsafe { Queue::from_ptr(queue) };

        unsafe { queue.remove(item) }
    }

    fn messages_waiting(queue: QueuePtr) -> usize {
        let queue = unsafe { Queue::from_ptr(queue) };

        queue.messages_waiting()
    }
}

register_queue_implementation!(Queue);
