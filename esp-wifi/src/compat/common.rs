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

static mut FAKE_WIFI_QUEUE: &SimpleQueue<[u8; 8], 200> = unsafe { &REAL_WIFI_QUEUE };
static mut REAL_WIFI_QUEUE: SimpleQueue<[u8; 8], 200> = SimpleQueue::new(); // first there is a ptr to the real queue - driver checks it's not null

pub struct StrWriter {
    dst: *mut u8,
    capacity: usize,
    len: usize,
}

impl StrWriter {
    pub fn new(dst: *mut u8, capacity: usize) -> Self {
        Self {
            dst,
            capacity,
            len: 0,
        }
    }

    pub fn len(&self) -> usize {
        self.len
    }

    fn space(&self) -> usize {
        self.capacity - self.len
    }

    fn write(&mut self, byte: u8) {
        unsafe {
            self.dst.write(byte);
            self.dst = self.dst.add(1);
        }
    }

    pub fn append_char(&mut self, c: char) {
        let mut buf = [0u8; 4];
        let char = c.encode_utf8(&mut buf);
        self.append(char);
    }

    pub fn append(&mut self, s: &str) {
        // Write as many bytes as possible. We're writing a c string which means we don't have
        // to deal with utf8 character boundaries, so this should be fine.
        let len = s.len().min(self.space());
        for byte in &s.as_bytes()[..len] {
            self.write(*byte);
        }

        // vsnprintf's semantics: it counts unwritten bytes, too
        self.len += s.len();
    }

    pub fn append_byte(&mut self, b: u8) {
        if self.space() >= 1 {
            self.write(b);
        }

        // vsnprintf's semantics: it counts unwritten bytes, too
        self.len += 1;
    }
}

impl Write for StrWriter {
    fn write_str(&mut self, s: &str) -> Result<(), core::fmt::Error> {
        self.append(s);
        Ok(())
    }
}

pub unsafe fn str_from_c<'a>(s: *const u8) -> &'a str {
    let c_str = core::ffi::CStr::from_ptr(s.cast());
    core::str::from_utf8_unchecked(c_str.to_bytes())
}

pub unsafe extern "C" fn syslog(_priority: u32, format: *const u8, mut args: VaListImpl) {
    #[cfg(feature = "wifi-logs")]
    cfg_if::cfg_if! {
        if #[cfg(any(target_arch = "riscv32", all(target_arch = "xtensa", xtensa_has_vaarg)))]
        {
            let mut buf = [0u8; 512];
            vsnprintf(&mut buf as *mut u8, 512, format, args);
            let res_str = str_from_c(&buf as *const u8);
            info!("{}", res_str);
        }
        else
        {
            let res_str = str_from_c(format);
            info!("{}", res_str);
        }
    }
}

/// Returns the number of character that would have been written if the buffer was big enough.
pub(crate) unsafe fn vsnprintf(
    dst: *mut u8,
    capacity: u32,
    format: *const u8,
    mut args: VaListImpl,
) -> i32 {
    let mut res_str = StrWriter::new(dst, capacity as usize - 1);

    let s = str_from_c(format);

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
                    // FIXME: This is sus - both branches have the same impl
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
                    let vbuf = str_from_c(v);
                    res_str.append(vbuf);
                }

                'c' => {
                    let v = args.arg::<u8>();
                    if v != 0 {
                        res_str.append_byte(v);
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

    let chars_written = res_str.len();
    let terminating_at = chars_written.min(capacity as usize - 1);
    dst.add(terminating_at).write(0);

    chars_written as i32
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

pub fn sem_give(semphr: *mut crate::binary::c_types::c_void) -> i32 {
    trace!("semphr_give {:?}", semphr);
    let sem_idx = semphr as usize - 1;

    let res = critical_section::with(|_| unsafe {
        if let Some(cnt) = CURR_SEM[sem_idx] {
            CURR_SEM[sem_idx] = Some(cnt + 1);
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

/// Lock a mutex. Block until successful.
pub fn lock_mutex(mutex: *mut crate::binary::c_types::c_void) -> i32 {
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

    unsafe { &mut FAKE_WIFI_QUEUE as *mut _ as *mut crate::binary::c_types::c_void }
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
            // assume the size is 8 - shouldn't rely on that
            let message = item as *const u8;
            let mut data = [0u8; 8];
            for i in 0..8 as usize {
                data[i] = *(message.offset(i as isize));
            }
            trace!("queue posting {:?}", data);

            critical_section::with(|_| {
                REAL_WIFI_QUEUE.enqueue(data);
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

    let forever = block_time_tick == OSI_FUNCS_TIME_BLOCKING;
    let timeout = block_time_tick as u64;
    let start = crate::timer::get_systimer_count();

    // handle the WIFI_QUEUE
    unsafe {
        if queue != &mut REAL_WIFI_QUEUE as *mut _ as *mut crate::binary::c_types::c_void {
            panic!("Unknown queue to handle in queue_recv");
        }

        loop {
            let message = critical_section::with(|_| {
                memory_fence();
                REAL_WIFI_QUEUE.dequeue()
            });

            if let Some(message) = message {
                let item = item as *mut u8;
                for i in 0..8 {
                    item.offset(i).write_volatile(message[i as usize]);
                }
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
}
