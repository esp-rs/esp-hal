#![allow(unused)]

use core::{
    cell::RefCell,
    fmt::Write,
    mem::size_of_val,
    ptr::{self, addr_of, addr_of_mut},
};

use esp_wifi_sys::include::malloc;

use super::malloc::free;
use crate::{
    binary::c_types::{c_int, c_void},
    hal::sync::Locked,
    memory_fence::memory_fence,
    preempt::{current_task, yield_task},
};

pub(crate) const OSI_FUNCS_TIME_BLOCKING: u32 = u32::MAX;

#[derive(Clone, Copy, Debug)]
struct Mutex {
    locking_pid: usize,
    count: u32,
    recursive: bool,
}

pub(crate) struct ConcurrentQueue {
    raw_queue: Locked<RawQueue>,
}

impl ConcurrentQueue {
    pub(crate) fn new(count: usize, item_size: usize) -> Self {
        Self {
            raw_queue: Locked::new(RawQueue::new(count, item_size)),
        }
    }

    fn release_storage(&mut self) {
        self.raw_queue.with(|q| unsafe { q.release_storage() })
    }

    pub(crate) fn enqueue(&mut self, item: *mut c_void) -> i32 {
        self.raw_queue.with(|q| unsafe { q.enqueue(item) })
    }

    pub(crate) fn try_dequeue(&mut self, item: *mut c_void) -> bool {
        self.raw_queue.with(|q| unsafe { q.try_dequeue(item) })
    }

    pub(crate) fn remove(&mut self, item: *mut c_void) {
        self.raw_queue.with(|q| unsafe { q.remove(item) })
    }

    pub(crate) fn count(&self) -> usize {
        self.raw_queue.with(|q| unsafe { q.count() })
    }
}

impl Drop for ConcurrentQueue {
    fn drop(&mut self) {
        self.release_storage();
    }
}

/// A naive and pretty much unsafe queue to back the queues used in drivers and
/// supplicant code.
///
/// The [ConcurrentQueue] wrapper should be used.
pub struct RawQueue {
    capacity: usize,
    item_size: usize,
    current_read: usize,
    current_write: usize,
    storage: *mut u8,
}

impl RawQueue {
    /// This allocates underlying storage. See [release_storage]
    pub fn new(capacity: usize, item_size: usize) -> Self {
        let storage = unsafe { malloc((capacity * item_size) as u32) as *mut u8 };
        assert!(!storage.is_null());

        Self {
            capacity,
            item_size,
            current_read: 0,
            current_write: 0,
            storage,
        }
    }

    /// Call `release_storage` to deallocate the underlying storage
    unsafe fn release_storage(&mut self) {
        unsafe {
            free(self.storage);
        }
        self.storage = core::ptr::null_mut();
    }

    unsafe fn enqueue(&mut self, item: *mut c_void) -> i32 {
        if self.count() < self.capacity {
            unsafe {
                let p = self.storage.byte_add(self.item_size * self.current_write);
                p.copy_from(item as *mut u8, self.item_size);
                self.current_write = (self.current_write + 1) % self.capacity;
            }

            1
        } else {
            0
        }
    }

    unsafe fn try_dequeue(&mut self, item: *mut c_void) -> bool {
        if self.count() > 0 {
            unsafe {
                let p = self.storage.byte_add(self.item_size * self.current_read) as *const c_void;
                item.copy_from(p, self.item_size);
                self.current_read = (self.current_read + 1) % self.capacity;
            }
            true
        } else {
            false
        }
    }

    unsafe fn remove(&mut self, item: *mut c_void) {
        // do what the ESP-IDF implementations does ...
        // just remove all elements and add them back except the one we need to remove -
        // good enough for now
        let item_slice = core::slice::from_raw_parts_mut(item as *mut u8, self.item_size);
        let count = self.count();

        if count == 0 {
            return;
        }

        let tmp_item = crate::compat::malloc::malloc(self.item_size);

        if tmp_item.is_null() {
            panic!("Out of memory");
        }

        for _ in 0..count {
            if self.try_dequeue(tmp_item as *mut c_void) {
                let tmp_slice = core::slice::from_raw_parts_mut(tmp_item, self.item_size);
                if tmp_slice != item_slice {
                    self.enqueue(tmp_item as *mut c_void);
                }
            }
        }

        crate::compat::malloc::free(tmp_item);
    }

    unsafe fn count(&self) -> usize {
        if self.current_write >= self.current_read {
            self.current_write - self.current_read
        } else {
            self.capacity - self.current_read + self.current_write
        }
    }
}

pub unsafe fn str_from_c<'a>(s: *const u8) -> &'a str {
    let c_str = core::ffi::CStr::from_ptr(s.cast());
    core::str::from_utf8_unchecked(c_str.to_bytes())
}

#[no_mangle]
unsafe extern "C" fn strnlen(chars: *const u8, maxlen: usize) -> usize {
    let mut len = 0;
    loop {
        if chars.offset(len).read_volatile() == 0 {
            break;
        }
        len += 1;
    }

    len as usize
}

pub(crate) fn sem_create(max: u32, init: u32) -> *mut c_void {
    unsafe {
        let ptr = malloc(4) as *mut u32;
        ptr.write_volatile(init);

        trace!("sem created res = {:?}", ptr);
        ptr.cast()
    }
}

pub(crate) fn sem_delete(semphr: *mut c_void) {
    trace!(">>> sem delete");

    unsafe {
        free(semphr.cast());
    }
}

pub(crate) fn sem_take(semphr: *mut c_void, tick: u32) -> i32 {
    trace!(">>>> semphr_take {:?} block_time_tick {}", semphr, tick);

    let forever = tick == OSI_FUNCS_TIME_BLOCKING;
    let timeout = tick as u64;
    let start = crate::time::systimer_count();

    let sem = semphr as *mut u32;

    'outer: loop {
        let res = critical_section::with(|_| unsafe {
            memory_fence();
            let cnt = *sem;
            if cnt > 0 {
                *sem = cnt - 1;
                1
            } else {
                0
            }
        });

        if res == 1 {
            trace!(">>>> return from semphr_take");
            return 1;
        }

        if !forever && crate::time::elapsed_time_since(start) > timeout {
            break 'outer;
        }

        yield_task();
    }

    trace!(">>>> return from semphr_take with timeout");
    0
}

pub(crate) fn sem_give(semphr: *mut c_void) -> i32 {
    trace!("semphr_give {:?}", semphr);
    let sem = semphr as *mut u32;

    critical_section::with(|_| unsafe {
        let cnt = *sem;
        *sem = cnt + 1;
        1
    })
}

pub(crate) fn thread_sem_get() -> *mut c_void {
    trace!("wifi_thread_semphr_get");
    unsafe { &mut ((*current_task()).thread_semaphore) as *mut _ as *mut c_void }
}

pub(crate) fn create_recursive_mutex() -> *mut c_void {
    let mutex = Mutex {
        locking_pid: 0xffff_ffff,
        count: 0,
        recursive: true,
    };

    let ptr = unsafe { malloc(size_of_val(&mutex) as u32) as *mut Mutex };
    unsafe {
        ptr.write(mutex);
    }
    memory_fence();

    trace!("recursive_mutex_create called {:?}", ptr);
    ptr as *mut c_void
}

pub(crate) fn mutex_delete(mutex: *mut c_void) {
    let ptr = mutex as *mut Mutex;
    unsafe {
        free(mutex.cast());
    }
}

/// Lock a mutex. Block until successful.
pub(crate) fn lock_mutex(mutex: *mut c_void) -> i32 {
    trace!("mutex_lock ptr = {:?}", mutex);

    let ptr = mutex as *mut Mutex;
    let current_task = current_task() as usize;

    loop {
        let mutex_locked = critical_section::with(|_| unsafe {
            if (*ptr).count == 0 {
                (*ptr).locking_pid = current_task;
                (*ptr).count += 1;
                true
            } else if (*ptr).locking_pid == current_task {
                (*ptr).count += 1;
                true
            } else {
                false
            }
        });
        memory_fence();

        if mutex_locked {
            return 1;
        }

        yield_task();
    }
}

pub(crate) fn unlock_mutex(mutex: *mut c_void) -> i32 {
    trace!("mutex_unlock {:?}", mutex);

    let ptr = mutex as *mut Mutex;
    critical_section::with(|_| unsafe {
        memory_fence();
        if (*ptr).count > 0 {
            (*ptr).count -= 1;
            1
        } else {
            0
        }
    })
}

pub(crate) fn create_queue(queue_len: c_int, item_size: c_int) -> *mut ConcurrentQueue {
    trace!("wifi_create_queue len={} size={}", queue_len, item_size,);

    let queue = ConcurrentQueue::new(queue_len as usize, item_size as usize);
    let ptr = unsafe { malloc(size_of_val(&queue) as u32) as *mut ConcurrentQueue };
    unsafe {
        ptr.write(queue);
    }

    trace!("created queue @{:?}", ptr);

    ptr
}

pub(crate) fn delete_queue(queue: *mut ConcurrentQueue) {
    trace!("delete_queue {:?}", queue);

    unsafe {
        ptr::drop_in_place(queue);
        crate::compat::malloc::free(queue.cast());
    }
}

pub(crate) fn send_queued(
    queue: *mut ConcurrentQueue,
    item: *mut c_void,
    block_time_tick: u32,
) -> i32 {
    trace!(
        "queue_send queue {:?} item {:x} block_time_tick {}",
        queue,
        item as usize,
        block_time_tick
    );

    let queue: *mut ConcurrentQueue = queue.cast();
    unsafe { (*queue).enqueue(item) }
}

pub(crate) fn receive_queued(
    queue: *mut ConcurrentQueue,
    item: *mut c_void,
    block_time_tick: u32,
) -> i32 {
    trace!(
        "queue_recv {:?} item {:?} block_time_tick {}",
        queue,
        item,
        block_time_tick
    );

    let forever = block_time_tick == OSI_FUNCS_TIME_BLOCKING;
    let timeout = block_time_tick as u64;
    let start = crate::time::systimer_count();

    loop {
        if unsafe { (*queue).try_dequeue(item) } {
            trace!("received");
            return 1;
        }

        if !forever && crate::time::elapsed_time_since(start) > timeout {
            trace!("queue_recv returns with timeout");
            return -1;
        }

        yield_task();
    }
}

pub(crate) fn number_of_messages_in_queue(queue: *const ConcurrentQueue) -> u32 {
    trace!("queue_msg_waiting {:?}", queue);

    let queue: *const ConcurrentQueue = queue.cast();
    unsafe { (*queue).count() as u32 }
}

/// Implementation of sleep() from newlib in esp-idf.
/// components/newlib/time.c
#[no_mangle]
pub(crate) unsafe extern "C" fn sleep(
    seconds: crate::binary::c_types::c_uint,
) -> crate::binary::c_types::c_uint {
    trace!("sleep");

    usleep(seconds * 1_000);
    0
}

/// Implementation of usleep() from newlib in esp-idf.
/// components/newlib/time.c
#[no_mangle]
unsafe extern "C" fn usleep(us: u32) -> crate::binary::c_types::c_int {
    trace!("usleep");
    extern "C" {
        fn esp_rom_delay_us(us: u32);
    }
    esp_rom_delay_us(us);
    0
}

#[no_mangle]
unsafe extern "C" fn putchar(c: i32) -> crate::binary::c_types::c_int {
    trace!("putchar {}", c as u8 as char);
    c
}
