#![allow(unused)]

use core::{ffi::VaListImpl, fmt::Write, ptr::addr_of_mut};

use super::queue::SimpleQueue;
use crate::{
    binary::{c_types::c_void, include::OSI_FUNCS_TIME_BLOCKING},
    memory_fence::memory_fence,
    preempt::preempt::current_task,
    timer::yield_task,
};

static mut CURR_SEM: [Option<u32>; 20] = [
    None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None,
    None, None, None, None,
];

static mut PER_THREAD_SEM: [Option<*mut crate::binary::c_types::c_void>; 4] = [None; 4];

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

static mut FAKE_WIFI_QUEUE: &Option<SimpleQueue<[u8; 8], 200>> = unsafe { &REAL_WIFI_QUEUE };
static mut REAL_WIFI_QUEUE: Option<SimpleQueue<[u8; 8], 200>> = None; // first there is a ptr to the real queue - driver checks it's not null

pub struct StrBuf {
    buffer: [u8; 512],
    len: usize,
}

impl StrBuf {
    pub fn new() -> StrBuf {
        StrBuf {
            buffer: [0u8; 512],
            len: 0,
        }
    }

    pub unsafe fn from(c_str: *const u8) -> StrBuf {
        let mut res = StrBuf {
            buffer: [0u8; 512],
            len: 0,
        };

        let mut idx: usize = 0;
        while *(c_str.offset(idx as isize)) != 0 {
            res.buffer[idx] = *(c_str.offset(idx as isize));
            idx += 1;
        }

        res.len = idx;
        res
    }

    pub unsafe fn append_from(&mut self, c_str: *const u8) {
        let mut src_idx: usize = 0;
        let mut idx: usize = self.len;
        while *(c_str.offset(src_idx as isize)) != 0 {
            self.buffer[idx] = *(c_str.offset(src_idx as isize));
            idx += 1;
            src_idx += 1;
        }

        self.len = idx;
    }

    pub fn append(&mut self, s: &str) {
        let mut idx: usize = self.len;
        s.chars().for_each(|c| {
            self.buffer[idx] = c as u8;
            idx += 1;
        });
        self.len = idx;
    }

    pub fn append_char(&mut self, c: char) {
        let mut idx: usize = self.len;
        self.buffer[idx] = c as u8;
        idx += 1;
        self.len = idx;
    }

    pub unsafe fn as_str_ref(&self) -> &str {
        core::str::from_utf8_unchecked(&self.buffer[..self.len])
    }
}

impl Write for StrBuf {
    fn write_str(&mut self, s: &str) -> Result<(), core::fmt::Error> {
        self.append(s);
        Ok(())
    }
}

pub unsafe extern "C" fn syslog(_priority: u32, format: *const u8, mut args: VaListImpl) {
    #[cfg(all(feature = "wifi-logs", target_arch = "riscv32"))]
    {
        let mut buf = [0u8; 512];
        vsnprintf(&mut buf as *mut u8, 511, format, args);
        let res_str = StrBuf::from(&buf as *const u8);
        info!("{}", res_str.as_str_ref());
    }
    #[cfg(all(feature = "wifi-logs", not(target_arch = "riscv32")))]
    {
        let res_str = StrBuf::from(format);
        info!("{}", res_str.as_str_ref());
    }
}

pub(crate) unsafe fn vsnprintf(
    dst: *mut u8,
    _n: u32,
    format: *const u8,
    mut args: VaListImpl,
) -> i32 {
    let fmt_str_ptr = format;

    let mut res_str = StrBuf::new();

    let strbuf = StrBuf::from(fmt_str_ptr);
    let s = strbuf.as_str_ref();

    let mut format_char = ' ';
    let mut is_long = false;
    let mut found = false;
    for c in s.chars().into_iter() {
        if !found {
            if c == '%' {
                found = true;
            }

            if !found {
                res_str.append_char(c);
            }
        } else {
            if c.is_numeric() || c == '-' || c == 'l' {
                if c == 'l' {
                    is_long = true;
                }
                // ignore
            } else {
                // a format char
                format_char = c;
            }
        }

        if found && format_char != ' ' {
            // have to format an arg
            match format_char {
                'd' => {
                    if is_long {
                        let v = args.arg::<i32>();
                        write!(res_str, "{}", v).ok();
                    } else {
                        let v = args.arg::<i32>();
                        write!(res_str, "{}", v).ok();
                    }
                }

                'u' => {
                    let v = args.arg::<u32>();
                    write!(res_str, "{}", v).ok();
                }

                'p' => {
                    let v = args.arg::<u32>();
                    write!(res_str, "0x{:x}", v).ok();
                }

                'X' => {
                    let v = args.arg::<u32>();
                    write!(res_str, "{:02x}", (v & 0xff000000) >> 24).ok();
                }

                'x' => {
                    let v = args.arg::<u32>();
                    write!(res_str, "{:02x}", v).ok();
                }

                's' => {
                    let v = args.arg::<u32>() as *const u8;
                    let vbuf = StrBuf::from(v);
                    write!(res_str, "{}", vbuf.as_str_ref()).ok();
                }

                'c' => {
                    let v = args.arg::<u8>();
                    if v != 0 {
                        write!(res_str, "{}", v as char).ok();
                    }
                }

                _ => {
                    write!(res_str, "<UNKNOWN{}>", format_char).ok();
                }
            }

            format_char = ' ';
            found = false;
            is_long = false;
        }
    }
    let mut idx = 0;
    res_str.as_str_ref().chars().for_each(|c| {
        *(dst.offset(idx)) = c as u8;
        idx += 1;
    });
    *(dst.offset(idx)) = 0;

    idx as i32
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

pub fn sem_create(max: u32, init: u32) -> *mut crate::binary::c_types::c_void {
    critical_section::with(|_| unsafe {
        let mut res = 0xffff;
        memory_fence();
        for (i, sem) in CURR_SEM.iter().enumerate() {
            memory_fence();
            if let None = *sem {
                res = i;
                break;
            }
        }

        trace!("sem created res = {} (+1)", res);

        if res != 0xffff {
            memory_fence();
            CURR_SEM[res] = Some(init);
            (res + 1) as *mut crate::binary::c_types::c_void
        } else {
            core::ptr::null_mut()
        }
    })
}

pub fn sem_delete(semphr: *mut crate::binary::c_types::c_void) {
    trace!(">>> sem delete");
    critical_section::with(|_| unsafe {
        CURR_SEM[semphr as usize - 1] = None;
        memory_fence();
    })
}

pub fn sem_take(semphr: *mut crate::binary::c_types::c_void, tick: u32) -> i32 {
    trace!(">>>> semphr_take {:?} block_time_tick {}", semphr, tick);

    let forever = if tick == OSI_FUNCS_TIME_BLOCKING {
        true
    } else {
        false
    };
    let tick = if tick == 0 { 1 } else { tick };
    let end_time = crate::timer::get_systimer_count() + tick as u64;

    'outer: loop {
        loop {
            let res = critical_section::with(|_| unsafe {
                memory_fence();
                if let Some(cnt) = CURR_SEM[semphr as usize - 1] {
                    if cnt > 0 {
                        CURR_SEM[semphr as usize - 1] = Some(cnt - 1);
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

            if !forever {
                if crate::timer::get_systimer_count() > end_time {
                    break 'outer;
                }
            }

            yield_task();
        }
    }

    trace!(">>>> return from semphr_take with timeout");
    0
}

pub fn sem_give(semphr: *mut crate::binary::c_types::c_void) -> i32 {
    trace!("semphr_give {:?}", semphr);

    let res = critical_section::with(|_| unsafe {
        if let Some(cnt) = CURR_SEM[semphr as usize - 1] {
            CURR_SEM[semphr as usize - 1] = Some(cnt + 1);
            memory_fence();
            1
        } else {
            0
        }
    });

    res
}

pub fn thread_sem_get() -> *mut crate::binary::c_types::c_void {
    trace!("wifi_thread_semphr_get");
    critical_section::with(|_| unsafe {
        let tid = current_task();
        if PER_THREAD_SEM[tid].is_none() {
            let sem = sem_create(1, 0);
            trace!("wifi_thread_semphr_get - create for {} {:?}", tid, sem);
            PER_THREAD_SEM[tid] = Some(sem);
            sem
        } else {
            let sem = unwrap!(PER_THREAD_SEM[tid]);
            trace!("wifi_thread_semphr_get - return for {} {:?}", tid, sem);
            sem
        }
    })
}

pub fn create_recursive_mutex() -> *mut crate::binary::c_types::c_void {
    critical_section::with(|_| unsafe {
        let ptr = &mut MUTEXES[MUTEX_IDX_CURRENT] as *mut _ as *mut Mutex;
        (*ptr).recursive = true;
        MUTEX_IDX_CURRENT += 1;
        memory_fence();
        trace!("recursive_mutex_create called {:?}", ptr);
        ptr as *mut crate::binary::c_types::c_void
    })
}

pub fn lock_mutex(mutex: *mut crate::binary::c_types::c_void) -> i32 {
    trace!("mutex_lock ptr = {:?}", mutex);

    unsafe {
        let success = loop {
            let ptr = mutex as *mut Mutex;
            let current_task = current_task();

            let (should_break, success) = critical_section::with(|_| {
                if (*ptr).count == 0 {
                    (*ptr).locking_pid = current_task;
                    (*ptr).count += 1;
                    (true, true)
                } else if (*ptr).count != 0 && (*ptr).locking_pid == current_task {
                    (*ptr).count += 1;
                    (true, true)
                } else if (*ptr).count != 0 && (*ptr).locking_pid != current_task {
                    (true, false)
                } else {
                    (false, false)
                }
            });
            memory_fence();

            if should_break {
                break success;
            }

            yield_task();
        };

        if success {
            1
        } else {
            0
        }
    }
}

pub fn unlock_mutex(mutex: *mut crate::binary::c_types::c_void) -> i32 {
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

pub fn create_wifi_queue(
    queue_len: crate::binary::c_types::c_int,
    item_size: crate::binary::c_types::c_int,
) -> *mut crate::binary::c_types::c_void {
    unsafe {
        trace!(
            "wifi_create_queue len={} size={} ptr={:?} real-queue {:?}  - not checked",
            queue_len,
            item_size,
            addr_of_mut!(FAKE_WIFI_QUEUE),
            addr_of_mut!(REAL_WIFI_QUEUE),
        );
    }

    if item_size > 8 {
        panic!("don't expecting the wifi queue to hold items larger than 8");
    }

    unsafe {
        if REAL_WIFI_QUEUE.is_none() {
            REAL_WIFI_QUEUE = Some(SimpleQueue::new());
        } else {
            panic!("don't expecting more than one wifi queue");
        }

        &mut FAKE_WIFI_QUEUE as *mut _ as *mut crate::binary::c_types::c_void
    }
}

pub fn send_queued(
    queue: *mut crate::binary::c_types::c_void,
    item: *mut crate::binary::c_types::c_void,
    block_time_tick: u32,
) -> i32 {
    trace!(
        "queue_send queue {:?} item {:?} block_time_tick {}",
        queue,
        item,
        block_time_tick
    );

    // handle the WIFI_QUEUE
    unsafe {
        if queue == &mut REAL_WIFI_QUEUE as *mut _ as *mut crate::binary::c_types::c_void {
            critical_section::with(|_| {
                // assume the size is 8 - shouldn't rely on that
                let message = item as *const u8;
                let mut data = [0u8; 8];
                for i in 0..8 as usize {
                    data[i] = *(message.offset(i as isize));
                }
                trace!("queue posting {:?}", data);

                unwrap!(REAL_WIFI_QUEUE.as_mut()).enqueue(data);
                memory_fence();
            });
        }
    }

    1
}

pub fn receive_queued(
    queue: *mut crate::binary::c_types::c_void,
    item: *mut crate::binary::c_types::c_void,
    block_time_tick: u32,
) -> i32 {
    trace!(
        "queue_recv {:?} item {:?} block_time_tick {}",
        queue,
        item,
        block_time_tick
    );

    let end_time = crate::timer::get_systimer_count() + block_time_tick as u64;

    // handle the WIFI_QUEUE
    unsafe {
        if queue == &mut REAL_WIFI_QUEUE as *mut _ as *mut crate::binary::c_types::c_void {
            loop {
                let res = critical_section::with(|_| {
                    memory_fence();
                    let message = unwrap!(REAL_WIFI_QUEUE.as_mut()).dequeue();
                    if message.is_some() {
                        let message = unwrap!(message);
                        let item = item as *mut u8;
                        for i in 0..8 {
                            item.offset(i).write_volatile(message[i as usize]);
                        }
                        trace!("received {:?}", message);
                        1
                    } else {
                        0
                    }
                });

                if res == 1 {
                    trace!("queue_recv returns");
                    return res;
                }

                if block_time_tick != OSI_FUNCS_TIME_BLOCKING
                    && crate::timer::get_systimer_count() > end_time
                {
                    trace!("queue_recv returns with timeout");
                    return -1;
                }

                yield_task();
            }
        } else {
            panic!("Unknown queue to handle in queue_recv");
            -1
        }
    }
}
