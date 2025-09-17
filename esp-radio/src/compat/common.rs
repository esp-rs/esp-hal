#![allow(unused)]

use alloc::{boxed::Box, vec::Vec};
use core::{
    cell::RefCell,
    fmt::Write,
    mem::size_of_val,
    ptr::{self, addr_of, addr_of_mut},
};

use esp_hal::time::{Duration, Instant};
use esp_sync::NonReentrantMutex;
use esp_wifi_sys::{c_types::c_char, include::malloc};

use super::{OSI_FUNCS_TIME_BLOCKING, malloc::free};
use crate::{
    ESP_RADIO_LOCK,
    binary::c_types::{c_int, c_uint, c_void},
    memory_fence::memory_fence,
    preempt::{current_task, yield_task},
};

#[derive(Clone, Copy, Debug)]
struct Mutex {
    locking_pid: usize,
    count: u32,
    recursive: bool,
}

pub unsafe fn str_from_c<'a>(s: *const c_char) -> &'a str {
    unsafe {
        let c_str = core::ffi::CStr::from_ptr(s.cast());
        core::str::from_utf8_unchecked(c_str.to_bytes())
    }
}

pub(crate) fn thread_sem_get() -> *mut c_void {
    trace!("wifi_thread_semphr_get");
    crate::preempt::current_task_thread_semaphore()
        .as_ptr()
        .cast::<c_void>()
}

/// Implementation of sleep() from newlib in esp-idf.
/// components/newlib/time.c
#[unsafe(no_mangle)]
pub(crate) unsafe extern "C" fn __esp_radio_sleep(seconds: c_uint) -> c_uint {
    trace!("sleep");

    unsafe { __esp_radio_usleep(seconds * 1_000_000) };
    0
}

/// Implementation of usleep() from newlib in esp-idf.
/// components/newlib/time.c
#[unsafe(no_mangle)]
pub(crate) unsafe extern "C" fn __esp_radio_usleep(us: u32) -> c_int {
    #[cfg(any(feature = "wifi", feature = "ble"))]
    crate::preempt::usleep(us);

    #[cfg(not(any(feature = "wifi", feature = "ble")))]
    crate::hal::delay::Delay::new().delay_micros(us);

    0
}
