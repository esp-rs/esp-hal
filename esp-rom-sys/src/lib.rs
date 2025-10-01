#![doc = include_str!("../README.md")]
//! ## Feature Flags
#![doc = document_features::document_features!()]
#![doc(html_logo_url = "https://avatars.githubusercontent.com/u/46717278")]
#![allow(rustdoc::bare_urls)]
#![no_std]

use core::ffi::c_char;

#[doc(hidden)]
/// Helper macro for checking doctest code snippets
#[macro_export]
macro_rules! before_snippet {
    () => {
        r#"
# #![no_std]
# use procmacros::handler;
# use esp_hal::{interrupt::{self, InterruptConfigurable}, time::{Duration, Instant, Rate}};
# macro_rules! println {
#     ($($tt:tt)*) => { };
# }
# macro_rules! print {
#     ($($tt:tt)*) => { };
# }
# #[panic_handler]
# fn panic(_ : &core::panic::PanicInfo) -> ! {
#     loop {}
# }
# fn main() {
#   let _ = example();
# }
# struct ExampleError {}
# impl <T> From<T> for ExampleError where T: core::fmt::Debug {
#   fn from(_value: T) -> Self {
#       Self{}
#   }
# }
# async fn example() -> Result<(), ExampleError> {
#   let mut peripherals = esp_hal::init(esp_hal::Config::default());
"#
    };
}

pub mod rom;
mod syscall;

pub use syscall::init_syscall_table;

/// This is needed by `libesp_rom.a` (if used)
/// Other crates (i.e. esp-radio) also rely on this being defined somewhere
#[unsafe(no_mangle)]
unsafe extern "C" fn __assert_func(
    file: *const core::ffi::c_char,
    line: i32,
    func: *const core::ffi::c_char,
    expr: *const core::ffi::c_char,
) -> ! {
    unsafe {
        panic!(
            "__assert_func in {}:{} ({}): {}",
            core::ffi::CStr::from_ptr(file).to_str().unwrap(),
            line,
            core::ffi::CStr::from_ptr(func).to_str().unwrap(),
            core::ffi::CStr::from_ptr(expr).to_str().unwrap(),
        );
    }
}

// We cannot just use the ROM function since (on some targets, currently ESP32-S2) it calls
// `__getreent`
//
// From docs: The __getreent() function returns a per-task pointer to struct
// _reent in newlib libc. This structure is allocated on the TCB of each task.
// i.e. it assumes a FreeRTOS task calling it.
#[unsafe(no_mangle)]
unsafe extern "C" fn __strcasecmp(
    s1: *const core::ffi::c_char,
    s2: *const core::ffi::c_char,
) -> i32 {
    let mut i = 0;
    loop {
        unsafe {
            let s1_i = s1.add(i);
            let s2_i = s2.add(i);

            let val = (*s1_i).to_ascii_lowercase() as i32 - (*s2_i).to_ascii_lowercase() as i32;
            if val != 0 || *s1_i == 0 {
                return val;
            }
        }

        i += 1;
    }
}

#[unsafe(no_mangle)]
unsafe extern "C" fn __strnlen(chars: *const c_char, maxlen: isize) -> usize {
    let mut len = 0;
    loop {
        unsafe {
            if chars.offset(len).read_volatile() == 0 {
                break;
            }
            len += 1;

            if len >= maxlen {
                break;
            }
        }
    }

    len as usize
}

// We cannot just use the ROM function since it calls `__getreent`
//
// From docs: The __getreent() function returns a per-task pointer to struct
// _reent in newlib libc. This structure is allocated on the TCB of each task.
// i.e. it assumes a FreeRTOS task calling it.
#[unsafe(no_mangle)]
unsafe extern "C" fn __atoi(str: *const i8) -> i32 {
    let mut sign: i32 = 1;
    let mut res: i32 = 0;
    let mut idx = 0;

    // skip leading spaces
    while unsafe { str.add(idx).read() } as u8 == b' ' {
        idx += 1;
    }

    // check sign
    let c = unsafe { str.add(idx).read() } as u8;
    if c == b'-' || c == b'+' {
        if c == b'-' {
            sign = -1;
        }
        idx += 1;
    }

    // parse number digit by digit
    loop {
        let c = unsafe { str.add(idx).read() } as u8;

        if !c.is_ascii_digit() {
            break;
        }

        // if the result would exceed the bounds - return max-value
        if res > i32::MAX / 10 || (res == i32::MAX / 10 && c - b'0' > 7) {
            return if sign == 1 { i32::MAX } else { i32::MIN };
        }

        res = 10 * res + (c - b'0') as i32;
        idx += 1;
    }
    res * sign
}

#[derive(Debug, Copy, Clone)]
#[repr(C)]
struct Tm {
    tm_sec: u32,   // seconds after the minute - [0, 60] including leap second
    tm_min: u32,   // minutes after the hour - [0, 59]
    tm_hour: u32,  // hours since midnight - [0, 23]
    tm_mday: u32,  // day of the month - [1, 31]
    tm_mon: u32,   // months since January - [0, 11]
    tm_year: u32,  // years since 1900
    tm_wday: u32,  // days since Sunday - [0, 6]
    tm_yday: u32,  // days since January 1 - [0, 365]
    tm_isdst: u32, // daylight savings time flag
}

#[unsafe(no_mangle)]
unsafe extern "C" fn __mktime(time: *const Tm) -> i64 {
    let time = unsafe { *time };

    // Simplified implementation, ignoring time zones, leap seconds, and other
    // complexities
    let mut days_in_month = [31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31];

    let is_leap_year = |year: u32| {
        year.is_multiple_of(4) && (!year.is_multiple_of(100) || year.is_multiple_of(400))
    };

    let mut days = 0;
    let year = time.tm_year + 1900;
    for y in 1970..year {
        days += if is_leap_year(y) { 366 } else { 365 };
    }

    if is_leap_year(year) {
        days_in_month[1] = 29;
    }

    for m in 0..time.tm_mon {
        days += days_in_month[m as usize];
    }
    days += time.tm_mday - 1;

    let seconds = days * 24 * 60 * 60 + time.tm_hour * 60 * 60 + time.tm_min * 60 + time.tm_sec;

    seconds as i64
}
