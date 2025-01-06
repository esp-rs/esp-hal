use crate::compat::malloc::malloc;

// these are not called but needed for linking
#[no_mangle]
unsafe extern "C" fn fwrite(ptr: *const (), size: usize, count: usize, stream: *const ()) -> usize {
    todo!("fwrite {:?} {} {} {:?}", ptr, size, count, stream)
}

#[no_mangle]
unsafe extern "C" fn fopen(filename: *const u8, mode: *const u8) -> *const () {
    todo!("fopen {:?} {:?}", filename, mode)
}

#[no_mangle]
unsafe extern "C" fn fgets(str: *const u8, count: u32, file: *const ()) -> *const u8 {
    todo!("fgets {:?} {} {:?}", str, count, file)
}

#[no_mangle]
unsafe extern "C" fn fclose(stream: *const ()) -> i32 {
    todo!("fclose {:?}", stream);
}

// We cannot just use the ROM function since it needs to allocate memory
#[no_mangle]
unsafe extern "C" fn strdup(str: *const core::ffi::c_char) -> *const core::ffi::c_char {
    trace!("strdup {:?}", str);

    unsafe {
        let s = core::ffi::CStr::from_ptr(str);
        let len = s.count_bytes() + 1;
        let p = malloc(len);
        if !p.is_null() {
            core::ptr::copy_nonoverlapping(str, p.cast(), len);
        }
        p.cast()
    }
}

// We cannot just use the ROM function since it calls `__getreent`
//
// From docs: The __getreent() function returns a per-task pointer to struct
// _reent in newlib libc. This structure is allocated on the TCB of each task.
// i.e. it assumes a FreeRTOS task calling it.
#[no_mangle]
unsafe extern "C" fn atoi(str: *const i8) -> i32 {
    trace!("atoi {:?}", str);

    let mut sign: i32 = 1;
    let mut res: i32 = 0;
    let mut idx = 0;

    // skip leading spaces
    while str.add(idx).read() as u8 == b' ' {
        idx += 1;
    }

    // check sign
    let c = str.add(idx).read() as u8;
    if c == b'-' || c == b'+' {
        if c == b'-' {
            sign = -1;
        }
        idx += 1;
    }

    // parse number digit by digit
    loop {
        let c = str.add(idx).read() as u8;

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

#[no_mangle]
unsafe extern "C" fn mktime(time: *const Tm) -> i64 {
    trace!("mktime {:?}", time);
    let time = *time;

    // Simplified implementation, ignoring time zones, leap seconds, and other
    // complexities
    let mut days_in_month = [31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31];

    let is_leap_year = |year: u32| year % 4 == 0 && (year % 100 != 0 || year % 400 == 0);

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
