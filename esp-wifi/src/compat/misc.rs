use crate::compat::malloc::*;

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

#[no_mangle]
unsafe extern "C" fn strcat(destination: *mut u8, source: *const u8) -> *const u8 {
    trace!("strcat {:?} {:?}", destination, source);

    let dst: *mut u8 = strchr(destination.cast(), 0) as *mut u8;
    let len = strchr(source.cast(), 0) as usize - source as usize;
    core::ptr::copy(source, dst, len);
    destination
}

#[no_mangle]
unsafe extern "C" fn strcmp(str1: *const i8, str2: *const i8) -> i32 {
    trace!("strcmp {:?} {:?}", str1, str2);

    let s1 = core::ffi::CStr::from_ptr(str1).to_str().unwrap();
    let s2 = core::ffi::CStr::from_ptr(str2).to_str().unwrap();

    let x = s1.cmp(s2);

    match x {
        core::cmp::Ordering::Less => -1,
        core::cmp::Ordering::Equal => 0,
        core::cmp::Ordering::Greater => 1,
    }
}

#[cfg(feature = "have-strchr")]
extern "C" {
    fn strchr(str: *const i8, c: i32) -> *const i8;
}

#[cfg(not(feature = "have-strchr"))]
#[no_mangle]
unsafe extern "C" fn strchr(str: *const i8, c: i32) -> *const i8 {
    trace!("strchr {:?} {}", str, c);

    unsafe {
        let mut p = str;
        loop {
            if *p == c as i8 {
                return p;
            }

            if *p == 0 {
                return core::ptr::null();
            }

            p = p.add(1);
        }
    }
}

#[no_mangle]
unsafe extern "C" fn strlcpy(dst: *mut u8, src: *const u8, size: usize) -> usize {
    trace!("strlcpy {:?} {:?} {}", dst, src, size);

    let mut dst = dst;
    let mut src = src;
    let mut cnt = 0;
    loop {
        dst.write_volatile(0);

        let c = src.read_volatile();

        if c == 0 || cnt >= size {
            break;
        }

        dst.write_volatile(c);
        dst = dst.add(1);
        src = src.add(1);
        cnt += 1;
    }

    cnt
}

#[no_mangle]
unsafe extern "C" fn strstr(str1: *const i8, str2: *const i8) -> *const i8 {
    trace!("strstr {:?} {:?}", str1, str2);

    let s1 = core::ffi::CStr::from_ptr(str1).to_str().unwrap();
    let s2 = core::ffi::CStr::from_ptr(str2).to_str().unwrap();

    let idx = s1.find(s2);

    match idx {
        Some(offset) => str1.add(offset),
        None => core::ptr::null(),
    }
}

#[no_mangle]
unsafe extern "C" fn strcasecmp(str1: *const u8, str2: *const u8) -> i32 {
    trace!("strcasecmp {:?} {:?}", str1, str2);

    let mut str1 = str1;
    let mut str2 = str2;

    let mut c1 = *str1 as char;
    let mut c2 = *str2 as char;

    while c1 != '\0' && c2 != '\0' {
        c1 = c1.to_ascii_lowercase();
        c2 = c2.to_ascii_lowercase();

        if c1 != c2 {
            return c1 as i32 - c2 as i32;
        }

        str1 = str1.add(1);
        str2 = str2.add(1);

        c1 = *str1 as char;
        c2 = *str2 as char;
    }

    c1 as i32 - c2 as i32
}

#[no_mangle]
unsafe extern "C" fn strdup(str: *const i8) -> *const u8 {
    trace!("strdup {:?}", str);

    unsafe {
        let s = core::ffi::CStr::from_ptr(str);
        let s = s.to_str().unwrap();

        let p = malloc(s.len() + 1);
        core::ptr::copy_nonoverlapping(str, p as *mut i8, s.len() + 1);
        p as *const u8
    }
}

#[no_mangle]
unsafe extern "C" fn atoi(str: *const i8) -> i32 {
    trace!("atoi {:?}", str);

    let mut sign: i32 = 1;
    let mut res: i32 = 0;
    let mut idx = 0;

    while str.add(idx).read_volatile() as u8 == b' ' {
        idx += 1;
    }

    let c = str.add(idx).read_volatile() as u8;
    if c == b'-' || c == b'+' {
        if c == b'-' {
            sign = -1;
        }
        idx += 1;
    }

    loop {
        let c = str.add(idx).read_volatile() as u8;

        if c < b'0' || c > b'9' {
            break;
        }

        if res > i32::MAX / 10 || (res == i32::MAX / 10 && c - b'0' > 7) {
            return if sign == 1 { i32::MAX } else { i32::MIN };
        }

        res = 10 * res + (c - b'0') as i32;
        idx += 1;
    }
    return res * sign;
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
