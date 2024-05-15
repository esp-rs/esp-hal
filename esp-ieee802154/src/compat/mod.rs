use core::{ffi::VaListImpl, fmt::Write};

use log::info;

use self::str_buf::StrBuf;

mod str_buf;

#[no_mangle]
pub unsafe extern "C" fn phy_printf(format: *const u8, args: ...) {
    syslog(format, args);
}

#[no_mangle]
pub unsafe extern "C" fn rtc_printf(format: *const u8, args: ...) {
    syslog(format, args);
}

#[no_mangle]
pub unsafe extern "C" fn coexist_printf(format: *const u8, args: ...) {
    syslog(format, args);
}

pub unsafe extern "C" fn syslog(format: *const u8, args: VaListImpl) {
    let mut buf = [0u8; 512];
    vsnprintf(&mut buf as *mut u8, 511, format, args);
    let res_str = StrBuf::from(&buf as *const u8);
    info!("{}", res_str.as_str_ref());
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
    for c in s.chars() {
        if !found {
            if c == '%' {
                found = true;
            }

            if !found {
                res_str.append_char(c);
            }
        } else if c.is_numeric() || c == '-' || c == 'l' {
            if c == 'l' {
                is_long = true;
            }
            // ignore
        } else {
            // a format char
            format_char = c;
        }

        if found && format_char != ' ' {
            // have to format an arg
            match format_char {
                'd' => {
                    if is_long {
                        let v = args.arg::<i64>();
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
