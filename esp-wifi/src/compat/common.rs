#![allow(unused)]

use core::{
    fmt::Write,
    mem::size_of_val,
    ptr::{addr_of, addr_of_mut},
};

use esp_wifi_sys::include::malloc;

use super::malloc::free;
use crate::{
    binary::{
        c_types::{c_int, c_void},
        include::OSI_FUNCS_TIME_BLOCKING,
    },
    memory_fence::memory_fence,
    preempt::current_task,
    timer::yield_task,
};

#[derive(Clone, Copy, Debug)]
struct Mutex {
    locking_pid: usize,
    count: u32,
    recursive: bool,
}

static mut MUTEXES: [Mutex; 10] = [Mutex {
    locking_pid: 0xffff_ffff,
    count: 0,
    recursive: false,
}; 10];
static mut MUTEX_IDX_CURRENT: usize = 0;

/// A naive and pretty much unsafe queue to back the queues used in drivers and
/// supplicant code
struct RawQueue {
    count: usize,
    item_size: usize,
    current_read: usize,
    current_write: usize,
    storage: *mut u8,
}

impl RawQueue {
    fn new(count: usize, item_size: usize) -> Self {
        let storage = unsafe { malloc((count * item_size) as u32) as *mut u8 };
        Self {
            count,
            item_size,
            current_read: 0,
            current_write: 0,
            storage,
        }
    }

    fn free_storage(&mut self) {
        unsafe {
            free(self.storage);
        }
        self.storage = core::ptr::null_mut();
    }

    fn enqueue(&mut self, item: *mut c_void) -> i32 {
        if (self.current_write - self.current_read) % self.count < self.count {
            unsafe {
                let p = self.storage.byte_add(self.item_size * self.current_write);
                p.copy_from(item as *mut u8, self.item_size);
                self.current_write = (self.current_write + 1) % self.count;
            }

            1
        } else {
            0
        }
    }

    fn try_dequeue(&mut self, item: *mut c_void) -> bool {
        if (self.current_write - self.current_read) % self.count > 0 {
            unsafe {
                let p = self.storage.byte_add(self.item_size * self.current_read) as *const c_void;
                item.copy_from(p, self.item_size);
                self.current_read = (self.current_read + 1) % self.count;
            }
            true
        } else {
            false
        }
    }

    fn count(&self) -> usize {
        (self.current_write - self.current_read) % self.count
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

pub fn sem_create(max: u32, init: u32) -> *mut c_void {
    critical_section::with(|_| unsafe {
        let ptr = malloc(4) as *mut u32;
        ptr.write_volatile(init);

        trace!("sem created res = {:?}", ptr);
        ptr.cast()
    })
}

pub fn sem_delete(semphr: *mut c_void) {
    trace!(">>> sem delete");

    // TODO remove this once fixed in esp_supplicant AND we updated to the fixed -
    // JIRA: WIFI-6676
    unsafe {
        if semphr as usize > addr_of!(MUTEXES) as usize
            && semphr as usize
                <= unsafe { addr_of!(MUTEXES).byte_add(size_of_val(&*addr_of!(MUTEXES))) } as usize
        {
            warn!("trying to remove a mutex via sem_delete");
            return;
        }
    }

    critical_section::with(|_| unsafe {
        free(semphr.cast());
    })
}

pub fn sem_take(semphr: *mut c_void, tick: u32) -> i32 {
    trace!(">>>> semphr_take {:?} block_time_tick {}", semphr, tick);

    let forever = tick == OSI_FUNCS_TIME_BLOCKING;
    let timeout = tick as u64;
    let start = crate::timer::get_systimer_count();

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

        if !forever && crate::timer::elapsed_time_since(start) > timeout {
            break 'outer;
        }

        yield_task();
    }

    trace!(">>>> return from semphr_take with timeout");
    0
}

pub fn sem_give(semphr: *mut c_void) -> i32 {
    trace!("semphr_give {:?}", semphr);
    let sem = semphr as *mut u32;

    critical_section::with(|_| unsafe {
        let cnt = *sem;
        *sem = cnt + 1;
        1
    })
}

pub fn thread_sem_get() -> *mut c_void {
    trace!("wifi_thread_semphr_get");
    unsafe { &mut ((*current_task()).thread_semaphore) as *mut _ as *mut c_void }
}

pub fn create_recursive_mutex() -> *mut c_void {
    critical_section::with(|_| unsafe {
        let ptr = &mut MUTEXES[MUTEX_IDX_CURRENT] as *mut Mutex;
        (*ptr).recursive = true;
        MUTEX_IDX_CURRENT += 1;
        memory_fence();
        trace!("recursive_mutex_create called {:?}", ptr);
        ptr as *mut c_void
    })
}

/// Lock a mutex. Block until successful.
pub fn lock_mutex(mutex: *mut c_void) -> i32 {
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

pub fn unlock_mutex(mutex: *mut c_void) -> i32 {
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

pub fn create_queue(queue_len: c_int, item_size: c_int) -> *mut c_void {
    trace!("wifi_create_queue len={} size={}", queue_len, item_size,);

    let queue = RawQueue::new(queue_len as usize, item_size as usize);
    let ptr = unsafe { malloc(size_of_val(&queue) as u32) as *mut RawQueue };
    unsafe {
        ptr.write(queue);
    }

    trace!("created queue @{:?}", ptr);

    ptr.cast()
}

pub fn delete_queue(queue: *mut c_void) {
    trace!("delete_queue {:?}", queue);

    let queue: *mut RawQueue = queue.cast();
    unsafe {
        (*queue).free_storage();
        free(queue.cast());
    }
}

pub fn send_queued(queue: *mut c_void, item: *mut c_void, block_time_tick: u32) -> i32 {
    trace!(
        "queue_send queue {:?} item {:x} block_time_tick {}",
        queue,
        item as usize,
        block_time_tick
    );

    let queue: *mut RawQueue = queue.cast();
    critical_section::with(|_| unsafe { (*queue).enqueue(item) })
}

pub fn receive_queued(queue: *mut c_void, item: *mut c_void, block_time_tick: u32) -> i32 {
    trace!(
        "queue_recv {:?} item {:?} block_time_tick {}",
        queue,
        item,
        block_time_tick
    );

    let forever = block_time_tick == OSI_FUNCS_TIME_BLOCKING;
    let timeout = block_time_tick as u64;
    let start = crate::timer::get_systimer_count();

    let queue: *mut RawQueue = queue.cast();

    loop {
        if critical_section::with(|_| unsafe { (*queue).try_dequeue(item) }) {
            trace!("received");
            return 1;
        }

        if !forever && crate::timer::elapsed_time_since(start) > timeout {
            trace!("queue_recv returns with timeout");
            return -1;
        }

        yield_task();
    }
}

pub fn number_of_messages_in_queue(queue: *const c_void) -> u32 {
    trace!("queue_msg_waiting {:?}", queue);

    let queue: *const RawQueue = queue.cast();
    critical_section::with(|_| unsafe { (*queue).count() as u32 })
}

/// Implementation of sleep() from newlib in esp-idf.
/// components/newlib/time.c
#[no_mangle]
unsafe extern "C" fn sleep(
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
