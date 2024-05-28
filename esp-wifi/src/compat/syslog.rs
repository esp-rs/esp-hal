use core::{ffi::VaListImpl, fmt::Write};

use super::common::str_from_c;

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
        // Write as many bytes as possible. We're writing a c string which means we
        // don't have to deal with utf8 character boundaries, so this should be
        // fine.
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

#[allow(unused)]
pub unsafe extern "C" fn syslog(_priority: u32, _format: *const u8, _args: VaListImpl) {
    #[cfg(feature = "wifi-logs")]
    cfg_if::cfg_if! {
        if #[cfg(any(target_arch = "riscv32", all(target_arch = "xtensa", xtensa_has_vaarg)))]
        {
            let mut buf = [0u8; 512];
            vsnprintf(&mut buf as *mut u8, 512, _format, _args);
            let res_str = str_from_c(&buf as *const u8);
            info!("{}", res_str);
        }
        else
        {
            let res_str = str_from_c(_format);
            info!("{}", res_str);
        }
    }
}

/// Returns the number of character that would have been written if the buffer
/// was big enough.
pub(crate) unsafe fn vsnprintf(
    dst: *mut u8,
    capacity: u32,
    format: *const u8,
    mut args: VaListImpl,
) -> i32 {
    let mut res_str = StrWriter::new(dst, capacity as usize - 1);

    let s = str_from_c(format);

    let mut format_char = ' ';
    let mut is_long = 0;
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
                is_long += 1;
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
                    if is_long < 2 {
                        let v = args.arg::<i32>();
                        write!(res_str, "{}", v).ok();
                    } else {
                        let v = args.arg::<i64>();
                        write!(res_str, "{}", v).ok();
                    }
                }

                'u' => {
                    if is_long < 2 {
                        let v = args.arg::<u32>();
                        write!(res_str, "{}", v).ok();
                    } else {
                        let v = args.arg::<u64>();
                        write!(res_str, "{}", v).ok();
                    }
                }

                'p' => {
                    let v = args.arg::<u32>();
                    write!(res_str, "0x{:x}", v).ok();
                }

                'X' => {
                    let v = args.arg::<u32>();
                    write!(res_str, "{:02X}", v).ok();
                }

                'x' => {
                    let v = args.arg::<u32>();
                    write!(res_str, "{:02x}", v).ok();
                }

                's' => {
                    let v = args.arg::<*const u8>();
                    let vbuf = str_from_c(v);
                    res_str.append(vbuf);
                }

                'c' => {
                    let v = args.arg::<u8>();
                    if v != 0 {
                        res_str.append_byte(v);
                    }
                }

                '%' => {
                    res_str.append_char('%');
                }

                _ => {
                    write!(res_str, "<UNKNOWN{}>", format_char).ok();
                }
            }

            format_char = ' ';
            found = false;
            is_long = 0;
        }
    }

    let chars_written = res_str.len();
    let terminating_at = chars_written.min(capacity as usize - 1);
    dst.add(terminating_at).write(0);

    chars_written as i32
}
