#![allow(unused)]

use core::{
    fmt::Write,
    ptr::{addr_of, addr_of_mut},
};

use super::queue::SimpleQueue;
use crate::{
    binary::{
        c_types::{c_int, c_void},
        include::OSI_FUNCS_TIME_BLOCKING,
    },
    memory_fence::memory_fence,
    preempt::current_task,
    timer::yield_task,
};

static mut CURR_SEM: [Option<u32>; 20] = [
    None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None,
    None, None, None, None,
];

static mut PER_THREAD_SEM: [Option<*mut c_void>; 4] = [None; 4];

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

static mut FAKE_WIFI_QUEUE: *const SimpleQueue<[u8; 8], 200> = unsafe { addr_of!(REAL_WIFI_QUEUE) };
static mut REAL_WIFI_QUEUE: SimpleQueue<[u8; 8], 200> = SimpleQueue::new(); // first there is a ptr to the real queue - driver checks it's not null

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
        let mut res = 0xffff;
        memory_fence();
        for (i, sem) in CURR_SEM.iter().enumerate() {
            memory_fence();
            if sem.is_none() {
                res = i;
                break;
            }
        }

        trace!("sem created res = {} (+1)", res);

        if res != 0xffff {
            memory_fence();
            CURR_SEM[res] = Some(init);
            (res + 1) as *mut c_void
        } else {
            core::ptr::null_mut()
        }
    })
}

pub fn sem_delete(semphr: *mut c_void) {
    trace!(">>> sem delete");
    critical_section::with(|_| unsafe {
        CURR_SEM[semphr as usize - 1] = None;
        memory_fence();
    })
}

pub fn sem_take(semphr: *mut c_void, tick: u32) -> i32 {
    trace!(">>>> semphr_take {:?} block_time_tick {}", semphr, tick);

    let forever = tick == OSI_FUNCS_TIME_BLOCKING;
    let timeout = tick as u64;
    let start = crate::timer::get_systimer_count();

    let sem_idx = semphr as usize - 1;

    'outer: loop {
        let res = critical_section::with(|_| unsafe {
            memory_fence();
            if let Some(cnt) = CURR_SEM[sem_idx] {
                if cnt > 0 {
                    CURR_SEM[sem_idx] = Some(cnt - 1);
                    1
                } else {
                    0
                }
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
    let sem_idx = semphr as usize - 1;

    critical_section::with(|_| unsafe {
        if let Some(cnt) = CURR_SEM[sem_idx] {
            CURR_SEM[sem_idx] = Some(cnt + 1);
            memory_fence();
            1
        } else {
            0
        }
    })
}

pub fn thread_sem_get() -> *mut c_void {
    trace!("wifi_thread_semphr_get");
    critical_section::with(|_| unsafe {
        let tid = current_task();
        if let Some(sem) = PER_THREAD_SEM[tid] {
            trace!("wifi_thread_semphr_get - return for {} {:?}", tid, sem);
            sem
        } else {
            let sem = sem_create(1, 0);
            trace!("wifi_thread_semphr_get - create for {} {:?}", tid, sem);
            PER_THREAD_SEM[tid] = Some(sem);
            sem
        }
    })
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
    let current_task = current_task();

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

pub fn create_wifi_queue(queue_len: c_int, item_size: c_int) -> *mut c_void {
    trace!(
        "wifi_create_queue len={} size={} ptr={:?} real-queue {:?}  - not checked",
        queue_len,
        item_size,
        unsafe { addr_of_mut!(FAKE_WIFI_QUEUE) },
        unsafe { addr_of_mut!(REAL_WIFI_QUEUE) },
    );

    if item_size > 8 {
        // TODO: don't assume
        panic!("don't expecting the wifi queue to hold items larger than 8");
    }

    unsafe { addr_of_mut!(FAKE_WIFI_QUEUE).cast() }
}

pub fn send_queued(queue: *mut c_void, item: *mut c_void, block_time_tick: u32) -> i32 {
    trace!(
        "queue_send queue {:?} item {:x} block_time_tick {}",
        queue,
        item as usize,
        block_time_tick
    );

    // handle the WIFI_QUEUE
    unsafe {
        if queue != addr_of_mut!(REAL_WIFI_QUEUE).cast() {
            warn!("Posting message to an unknown queue");
            return 0;
        }
    }

    let message = unsafe {
        // SAFETY: we checked that our queue is used and it stores with 8 byte items
        core::slice::from_raw_parts(item.cast::<u8>(), 8)
    };
    let mut data: [u8; 8] = unwrap!(message.try_into());
    trace!("queue posting {:?}", data);

    critical_section::with(|_| {
        if unsafe { REAL_WIFI_QUEUE.enqueue(data).is_ok() } {
            memory_fence();
            1
        } else {
            warn!("queue_send failed");
            0
        }
    })
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

    // handle the WIFI_QUEUE
    if queue != unsafe { addr_of_mut!(REAL_WIFI_QUEUE).cast() } {
        warn!("Posting message to an unknown queue");
        return 0;
    }

    loop {
        let message = critical_section::with(|_| unsafe { REAL_WIFI_QUEUE.dequeue() });

        if let Some(message) = message {
            let out_message = unsafe {
                // SAFETY: we checked that our queue is used and it stores with 8 byte items
                core::slice::from_raw_parts_mut(item.cast::<u8>(), 8)
            };
            out_message.copy_from_slice(&message);
            trace!("received {:?}", message);

            return 1;
        }

        if !forever && crate::timer::elapsed_time_since(start) > timeout {
            trace!("queue_recv returns with timeout");
            return -1;
        }

        yield_task();
    }
}
pub fn number_of_messages_in_queue(queue: *mut c_void) -> u32 {
    trace!("queue_msg_waiting {:?}", queue);
    if queue != unsafe { addr_of_mut!(REAL_WIFI_QUEUE).cast() } {
        warn!("queue_msg_waiting: Unknown queue.");
        return 0;
    }
    critical_section::with(|_| unsafe { REAL_WIFI_QUEUE.len() as u32 })
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
    info!("putchar {}", c as u8 as char);
    c
}
