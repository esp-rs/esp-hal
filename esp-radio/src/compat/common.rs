use esp_hal::system::Cpu;

use crate::sys::c_types::{c_char, c_int, c_uint, c_void};

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

/// Returns the core for a task that a driver creates.
///
/// A core number that is not valid means that the driver names no core. A task with no core runs
/// on the first core.
pub(crate) fn task_core(core_id: u32) -> u32 {
    if core_id < Cpu::COUNT as u32 {
        core_id
    } else {
        Cpu::ProCpu as u32
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
    {
        let start = esp_hal::time::Instant::now();
        let duration = esp_hal::time::Duration::from_micros(us as u64);
        while start.elapsed() < duration {}
    }

    0
}
