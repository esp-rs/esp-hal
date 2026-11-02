//! `printf` and the related functions that the ESP-SR libraries import, and their output, which
//! goes to the `log` crate line by line.
//!
//! Stable Rust cannot define C variadic functions, so each variadic function is an assembly shim.
//! On RISC-V, a `va_list` is a pointer to the variadic arguments, which the shim makes contiguous:
//! it stores the argument registers after the fixed arguments right below the arguments the caller
//! passed on the stack, as GCC does. The shim then calls the `va_list` variant of the function.

use core::{
    cell::RefCell,
    ffi::{CStr, c_char, c_int, c_void},
    fmt::{self, Write as _},
    ptr,
    slice,
};

use critical_section::Mutex;

/// Defines `$name`, a C variadic function with `$fixed` fixed arguments, which calls the function
/// `$target` with the fixed arguments followed by the `va_list`.
macro_rules! variadic_shim {
    ($name:literal, $fixed:literal, $target:literal) => {
        #[cfg(target_arch = "riscv32")]
        core::arch::global_asm!(
            concat!(".section .text.", $name, ",\"ax\",@progbits"),
            concat!(".globl ", $name),
            concat!(".type ", $name, ",@function"),
            ".p2align 1",
            concat!($name, ":"),
            // 16 bytes for `ra`, then 32 for the eight argument registers, ending where the
            // caller's stack arguments start.
            "addi sp, sp, -48",
            "sw ra, 12(sp)",
            ".irp reg, 1, 2, 3, 4, 5, 6, 7",
            concat!(".if \\reg >= ", $fixed),
            "sw a\\reg, 16 + 4 * \\reg(sp)",
            ".endif",
            ".endr",
            concat!("addi a", $fixed, ", sp, 16 + 4 * ", $fixed),
            concat!("call ", $target),
            "lw ra, 12(sp)",
            "addi sp, sp, 48",
            "ret",
            concat!(".size ", $name, ", . - ", $name),
        );
    };
}

variadic_shim!("esp_sr_printf", 1, "esp_sr_vprintf");
variadic_shim!("esp_sr_fprintf", 2, "esp_sr_vfprintf");
variadic_shim!("esp_sr_sprintf", 2, "esp_sr_vsprintf");
variadic_shim!("esp_sr_snprintf", 3, "esp_sr_vsnprintf");
variadic_shim!("esp_sr_esp_log", 3, "esp_sr_esp_log_va");

/// The variadic arguments, as a `va_list` points to them.
struct Arguments(*const u8);

impl Arguments {
    /// # Safety
    ///
    /// The next argument must have 4 bytes, such as an `int` or a pointer.
    unsafe fn u32(&mut self) -> u32 {
        unsafe {
            let value = self.0.cast::<u32>().read_unaligned();
            self.0 = self.0.add(4);
            value
        }
    }

    /// # Safety
    ///
    /// The next argument must have 8 bytes, such as a `long long` or a `double`. These are aligned
    /// to 8 bytes.
    unsafe fn u64(&mut self) -> u64 {
        unsafe {
            self.0 = self.0.add(self.0.align_offset(8));
            let value = self.0.cast::<u64>().read_unaligned();
            self.0 = self.0.add(8);
            value
        }
    }
}

/// Where formatted text goes.
trait Output {
    fn put(&mut self, bytes: &[u8]);
}

/// A buffer from the caller. Without a size, the caller promises that it is large enough.
struct Buffer {
    start: *mut u8,
    size: Option<usize>,
    length: usize,
}

impl Output for Buffer {
    fn put(&mut self, bytes: &[u8]) {
        // One byte stays free for the NUL.
        let room = self.size.map_or(usize::MAX, |size| size.saturating_sub(1));
        let count = bytes.len().min(room.saturating_sub(self.length));
        // SAFETY: `count` keeps the text within the buffer.
        unsafe { ptr::copy_nonoverlapping(bytes.as_ptr(), self.start.add(self.length), count) };
        self.length += count;
    }
}

impl Buffer {
    fn terminate(&mut self) {
        if self.size != Some(0) {
            // SAFETY: `put` left room for the NUL.
            unsafe { self.start.add(self.length).write(0) };
        }
    }
}

/// Counts the bytes that `format` produced, which is what `printf` and friends return.
struct Counted<'a, O: Output> {
    output: &'a mut O,
    count: usize,
}

impl<O: Output> Output for Counted<'_, O> {
    fn put(&mut self, bytes: &[u8]) {
        self.count += bytes.len();
        self.output.put(bytes);
    }
}

/// Formats `format` with `arguments` as C's `printf` does, for the conversions `d i u o x X c s p
/// f F e E g G n %`, the flags `- + space # 0`, width and precision, also from `*`, and the length
/// modifiers `hh h l ll z j t`.
///
/// # Safety
///
/// `format` must be NUL-terminated, and `arguments` must match it.
unsafe fn format(
    output: &mut impl Output,
    format: *const c_char,
    mut arguments: Arguments,
) -> c_int {
    let mut output = Counted { output, count: 0 };
    let format = unsafe { CStr::from_ptr(format) }.to_bytes();
    let mut i = 0;
    while i < format.len() {
        let literal = format[i..]
            .iter()
            .position(|&b| b == b'%')
            .unwrap_or(format.len() - i);
        output.put(&format[i..i + literal]);
        i += literal + 1;
        if i > format.len() {
            break;
        }

        let mut spec = Spec::default();
        while let Some(&flag) = format.get(i) {
            match flag {
                b'-' => spec.left = true,
                b'+' => spec.plus = true,
                b' ' => spec.space = true,
                b'#' => spec.alternate = true,
                b'0' => spec.zero = true,
                _ => break,
            }
            i += 1;
        }
        if format.get(i) == Some(&b'*') {
            let width = unsafe { arguments.u32() } as i32;
            spec.left |= width < 0;
            spec.width = width.unsigned_abs() as usize;
            i += 1;
        } else {
            while let Some(digit) = format.get(i).filter(|b| b.is_ascii_digit()) {
                spec.width = spec.width * 10 + usize::from(digit - b'0');
                i += 1;
            }
        }
        if format.get(i) == Some(&b'.') {
            i += 1;
            if format.get(i) == Some(&b'*') {
                let precision = unsafe { arguments.u32() } as i32;
                spec.precision = usize::try_from(precision).ok();
                i += 1;
            } else {
                let mut precision = 0;
                while let Some(digit) = format.get(i).filter(|b| b.is_ascii_digit()) {
                    precision = precision * 10 + usize::from(digit - b'0');
                    i += 1;
                }
                spec.precision = Some(precision);
            }
        }
        let mut wide = false;
        let mut narrow = 0;
        while let Some(&modifier) = format.get(i) {
            match modifier {
                b'l' if format.get(i + 1) == Some(&b'l') => {
                    wide = true;
                    i += 1;
                }
                b'j' | b'L' => wide = true,
                b'l' | b'z' | b't' => {}
                b'h' => narrow += 1,
                _ => break,
            }
            i += 1;
        }
        let Some(&conversion) = format.get(i) else {
            break;
        };
        i += 1;

        // SAFETY: The caller promises that the arguments match the format.
        unsafe {
            match conversion {
                b'd' | b'i' => {
                    let value = if wide {
                        arguments.u64() as i64
                    } else {
                        let value = arguments.u32() as i32;
                        match narrow {
                            0 => i64::from(value),
                            1 => i64::from(value as i16),
                            _ => i64::from(value as i8),
                        }
                    };
                    spec.integer(&mut output, value < 0, value.unsigned_abs(), 10, false);
                }
                b'u' | b'o' | b'x' | b'X' => {
                    let value = if wide {
                        arguments.u64()
                    } else {
                        let value = arguments.u32();
                        u64::from(match narrow {
                            0 => value,
                            1 => u32::from(value as u16),
                            _ => u32::from(value as u8),
                        })
                    };
                    let base = match conversion {
                        b'o' => 8,
                        b'u' => 10,
                        _ => 16,
                    };
                    spec.plus = false;
                    spec.space = false;
                    spec.integer(&mut output, false, value, base, conversion == b'X');
                }
                b'p' => {
                    spec.alternate = true;
                    spec.integer(&mut output, false, u64::from(arguments.u32()), 16, false);
                }
                b'c' => {
                    spec.precision = None;
                    spec.pad(&mut output, &[arguments.u32() as u8], b"");
                }
                b's' => {
                    let text = arguments.u32() as usize as *const c_char;
                    let text = if text.is_null() {
                        b"(null)".as_slice()
                    } else if let Some(precision) = spec.precision {
                        // The string may end without a NUL after `precision` bytes.
                        let bytes = text.cast::<u8>();
                        let length = (0..precision).take_while(|&k| *bytes.add(k) != 0).count();
                        slice::from_raw_parts(bytes, length)
                    } else {
                        CStr::from_ptr(text).to_bytes()
                    };
                    spec.precision = None;
                    spec.pad(&mut output, text, b"");
                }
                b'f' | b'F' | b'e' | b'E' | b'g' | b'G' | b'a' | b'A' => {
                    spec.float(&mut output, f64::from_bits(arguments.u64()), conversion);
                }
                b'n' => {
                    let count = arguments.u32() as usize as *mut c_int;
                    if !count.is_null() {
                        count.write(output.count as c_int);
                    }
                }
                b'%' => output.put(b"%"),
                other => output.put(&[b'%', other]),
            }
        }
    }
    output.count as c_int
}

#[derive(Default)]
struct Spec {
    left: bool,
    plus: bool,
    space: bool,
    alternate: bool,
    zero: bool,
    width: usize,
    precision: Option<usize>,
}

impl Spec {
    /// Writes `prefix` and `body` padded to the width.
    fn pad(&self, output: &mut impl Output, body: &[u8], prefix: &[u8]) {
        let padding = self.width.saturating_sub(prefix.len() + body.len());
        let zeros = self.zero && !self.left;
        if !self.left && !zeros {
            repeat(output, b' ', padding);
        }
        output.put(prefix);
        if zeros {
            repeat(output, b'0', padding);
        }
        output.put(body);
        if self.left {
            repeat(output, b' ', padding);
        }
    }

    fn sign(&self, negative: bool) -> &'static [u8] {
        match (negative, self.plus, self.space) {
            (true, _, _) => b"-",
            (false, true, _) => b"+",
            (false, false, true) => b" ",
            _ => b"",
        }
    }

    fn integer(
        &mut self,
        output: &mut impl Output,
        negative: bool,
        value: u64,
        base: u64,
        upper: bool,
    ) {
        let digits = if upper {
            b"0123456789ABCDEF"
        } else {
            b"0123456789abcdef"
        };
        let mut text = [0u8; 64];
        let mut start = text.len();
        let mut rest = value;
        while rest != 0 {
            start -= 1;
            text[start] = digits[(rest % base) as usize];
            rest /= base;
        }
        // The precision is the minimum number of digits. A precision of 0 prints no digits for 0.
        let minimum = self.precision.unwrap_or(1);
        while text.len() - start < minimum {
            start -= 1;
            text[start] = b'0';
        }
        let mut prefix = [0u8; 3];
        let sign = self.sign(negative);
        prefix[..sign.len()].copy_from_slice(sign);
        let mut prefix_length = sign.len();
        if self.alternate && value != 0 {
            let marker: &[u8] = match base {
                16 if upper => b"0X",
                16 => b"0x",
                _ => b"",
            };
            prefix[prefix_length..prefix_length + marker.len()].copy_from_slice(marker);
            prefix_length += marker.len();
            if base == 8 && text[start] != b'0' {
                start -= 1;
                text[start] = b'0';
            }
        }
        // With a precision, `0` does not pad.
        self.zero &= self.precision.is_none();
        self.pad(output, &text[start..], &prefix[..prefix_length]);
    }

    fn float(&mut self, output: &mut impl Output, value: f64, conversion: u8) {
        let mut text = Text::default();
        let upper = conversion.is_ascii_uppercase();
        let negative = value.is_sign_negative() && !value.is_nan();
        let magnitude = value.abs();
        if !magnitude.is_finite() {
            let name = match (magnitude.is_nan(), upper) {
                (true, false) => "nan",
                (true, true) => "NAN",
                (false, false) => "inf",
                (false, true) => "INF",
            };
            self.zero = false;
            self.pad(output, name.as_bytes(), self.sign(negative));
            return;
        }
        let precision = self.precision.unwrap_or(6);
        match conversion.to_ascii_lowercase() {
            b'f' => {
                let _ = write!(text, "{magnitude:.precision$}");
                if self.alternate && precision == 0 {
                    text.push(b'.');
                }
            }
            b'e' | b'a' => exponential(&mut text, magnitude, precision, self.alternate),
            _ => {
                // %g: the shorter of %e and %f, with `precision` significant digits.
                let significant = precision.max(1);
                let exponent = decimal_exponent(magnitude, significant);
                if exponent < -4 || exponent >= significant as i32 {
                    exponential(&mut text, magnitude, significant - 1, self.alternate);
                } else {
                    let decimals = (significant as i32 - 1 - exponent) as usize;
                    let _ = write!(text, "{magnitude:.decimals$}");
                }
                if !self.alternate {
                    text.trim_fraction_zeros();
                }
            }
        }
        if upper {
            text.bytes_mut().make_ascii_uppercase();
        }
        self.pad(output, text.bytes(), self.sign(negative));
    }
}

/// The decimal exponent of `value` once it is rounded to `significant` digits.
fn decimal_exponent(value: f64, significant: usize) -> i32 {
    if value == 0.0 {
        return 0;
    }
    let mut text = Text::default();
    let _ = write!(text, "{:.*e}", significant - 1, value);
    let bytes = text.bytes();
    let e = bytes.iter().position(|&b| b == b'e').unwrap();
    core::str::from_utf8(&bytes[e + 1..])
        .unwrap()
        .parse()
        .unwrap()
}

/// Writes `value` as C's `%e` does: a mantissa with `precision` decimals and an exponent with a
/// sign and at least two digits.
fn exponential(text: &mut Text, value: f64, precision: usize, alternate: bool) {
    let mut rust = Text::default();
    let _ = write!(rust, "{value:.precision$e}");
    let bytes = rust.bytes();
    let e = bytes.iter().position(|&b| b == b'e').unwrap();
    text.extend(&bytes[..e]);
    if alternate && precision == 0 {
        text.push(b'.');
    }
    let exponent: i32 = core::str::from_utf8(&bytes[e + 1..])
        .unwrap()
        .parse()
        .unwrap();
    let _ = write!(
        text,
        "e{}{:02}",
        if exponent < 0 { '-' } else { '+' },
        exponent.unsigned_abs()
    );
}

fn repeat(output: &mut impl Output, byte: u8, count: usize) {
    for _ in 0..count {
        output.put(&[byte]);
    }
}

/// Text of a formatted number, cut off after `CAPACITY` bytes. A `%f` of a huge `double` has
/// more digits than that, which nothing in ESP-SR prints.
struct Text {
    bytes: [u8; Text::CAPACITY],
    length: usize,
}

impl Default for Text {
    fn default() -> Self {
        Self {
            bytes: [0; Self::CAPACITY],
            length: 0,
        }
    }
}

impl Text {
    const CAPACITY: usize = 128;

    fn bytes(&self) -> &[u8] {
        &self.bytes[..self.length]
    }

    fn bytes_mut(&mut self) -> &mut [u8] {
        &mut self.bytes[..self.length]
    }

    fn push(&mut self, byte: u8) {
        self.extend(&[byte]);
    }

    fn extend(&mut self, bytes: &[u8]) {
        let count = bytes.len().min(Self::CAPACITY - self.length);
        self.bytes[self.length..self.length + count].copy_from_slice(&bytes[..count]);
        self.length += count;
    }

    /// Removes the trailing zeros of a fraction, and the point if nothing is left after it.
    fn trim_fraction_zeros(&mut self) {
        let e = self
            .bytes()
            .iter()
            .position(|&b| b == b'e')
            .unwrap_or(self.length);
        let Some(point) = self.bytes()[..e].iter().position(|&b| b == b'.') else {
            return;
        };
        let mut end = e;
        while end > point + 1 && self.bytes[end - 1] == b'0' {
            end -= 1;
        }
        if end == point + 1 {
            end = point;
        }
        self.bytes.copy_within(e..self.length, end);
        self.length -= e - end;
    }
}

impl fmt::Write for Text {
    fn write_str(&mut self, text: &str) -> fmt::Result {
        self.extend(text.as_bytes());
        Ok(())
    }
}

// Output to the log. ESP-SR writes lines in pieces, so the pieces are collected until a newline.

/// A line of output, cut off after `CAPACITY` bytes.
struct Line {
    bytes: [u8; Line::CAPACITY],
    length: usize,
}

impl Line {
    const CAPACITY: usize = 256;
}

static LINE: Mutex<RefCell<Line>> = Mutex::new(RefCell::new(Line {
    bytes: [0; Line::CAPACITY],
    length: 0,
}));

/// Writes to the log at `level`.
struct Log(log::Level);

impl Output for Log {
    fn put(&mut self, bytes: &[u8]) {
        for &byte in bytes {
            let complete = critical_section::with(|cs| {
                let mut line = LINE.borrow_ref_mut(cs);
                if byte != b'\n' {
                    let length = line.length;
                    if length < Line::CAPACITY {
                        line.bytes[length] = byte;
                        line.length += 1;
                    }
                    return None;
                }
                let complete = (line.bytes, line.length);
                line.length = 0;
                Some(complete)
            });
            if let Some((bytes, length)) = complete {
                log::log!(target: "esp_sr", self.0, "{}", String(&bytes[..length]));
            }
        }
    }
}

/// Text that is probably UTF-8, for the log.
struct String<'a>(&'a [u8]);

impl fmt::Display for String<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        for chunk in self.0.utf8_chunks() {
            f.write_str(chunk.valid())?;
            if !chunk.invalid().is_empty() {
                f.write_char(char::REPLACEMENT_CHARACTER)?;
            }
        }
        Ok(())
    }
}

/// The level of output to a stream: standard error is for warnings.
fn stream_level(stream: *mut c_void) -> log::Level {
    if ptr::eq(stream.cast_const(), crate::libc::esp_sr_stderr.0) {
        log::Level::Warn
    } else {
        log::Level::Info
    }
}

#[unsafe(no_mangle)]
unsafe extern "C" fn esp_sr_vprintf(format_: *const c_char, arguments: *const u8) -> c_int {
    unsafe { format(&mut Log(log::Level::Info), format_, Arguments(arguments)) }
}

#[unsafe(no_mangle)]
unsafe extern "C" fn esp_sr_vfprintf(
    stream: *mut c_void,
    format_: *const c_char,
    arguments: *const u8,
) -> c_int {
    unsafe {
        format(
            &mut Log(stream_level(stream)),
            format_,
            Arguments(arguments),
        )
    }
}

#[unsafe(no_mangle)]
unsafe extern "C" fn esp_sr_vsprintf(
    buffer: *mut c_char,
    format_: *const c_char,
    arguments: *const u8,
) -> c_int {
    let mut buffer = Buffer {
        start: buffer.cast(),
        size: None,
        length: 0,
    };
    let count = unsafe { format(&mut buffer, format_, Arguments(arguments)) };
    buffer.terminate();
    count
}

#[unsafe(no_mangle)]
unsafe extern "C" fn esp_sr_vsnprintf(
    buffer: *mut c_char,
    size: usize,
    format_: *const c_char,
    arguments: *const u8,
) -> c_int {
    let mut buffer = Buffer {
        start: buffer.cast(),
        size: Some(size),
        length: 0,
    };
    let count = unsafe { format(&mut buffer, format_, Arguments(arguments)) };
    buffer.terminate();
    count
}

/// ESP-IDF's `esp_log`: `config` holds the level in its lowest 3 bits.
#[unsafe(no_mangle)]
unsafe extern "C" fn esp_sr_esp_log_va(
    config: u32,
    tag: *const c_char,
    format_: *const c_char,
    arguments: *const u8,
) {
    let level = match config & 0b111 {
        0 => return,
        1 => log::Level::Error,
        2 => log::Level::Warn,
        3 => log::Level::Info,
        4 => log::Level::Debug,
        _ => log::Level::Trace,
    };
    let mut log = Log(level);
    // SAFETY: ESP-IDF passes a NUL-terminated tag.
    log.put(unsafe { CStr::from_ptr(tag) }.to_bytes());
    log.put(b": ");
    unsafe { format(&mut log, format_, Arguments(arguments)) };
    log.put(b"\n");
}

#[unsafe(no_mangle)]
unsafe extern "C" fn esp_sr_puts(text: *const c_char) -> c_int {
    let mut log = Log(log::Level::Info);
    // SAFETY: C callers pass a NUL-terminated string.
    log.put(unsafe { CStr::from_ptr(text) }.to_bytes());
    log.put(b"\n");
    1
}

#[unsafe(no_mangle)]
extern "C" fn esp_sr_putchar(c: c_int) -> c_int {
    Log(log::Level::Info).put(&[c as u8]);
    c
}

#[unsafe(no_mangle)]
unsafe extern "C" fn esp_sr_perror(text: *const c_char) {
    let mut log = Log(log::Level::Error);
    if !text.is_null() {
        // SAFETY: C callers pass a NUL-terminated string or null.
        log.put(unsafe { CStr::from_ptr(text) }.to_bytes());
    }
    log.put(b"\n");
}

#[unsafe(no_mangle)]
unsafe extern "C" fn esp_sr_fwrite(
    data: *const c_void,
    size: usize,
    count: usize,
    stream: *mut c_void,
) -> usize {
    let length = size.saturating_mul(count);
    // SAFETY: C callers pass `size * count` bytes.
    Log(stream_level(stream)).put(unsafe { slice::from_raw_parts(data.cast(), length) });
    count
}

#[cfg(test)]
mod tests {
    //! The formatter, tested on the host through the `va_list` functions with hand-made argument
    //! arrays, which are laid out as on RISC-V.

    use super::*;

    extern crate std;
    use std::{string::String as StdString, vec::Vec};

    enum Arg {
        Int(i32),
        Long(i64),
        Double(f64),
    }

    fn sprintf(format_: &CStr, args: &[Arg]) -> StdString {
        let mut memory: Vec<u8> = Vec::new();
        for arg in args {
            match arg {
                Arg::Int(v) => memory.extend(v.to_le_bytes()),
                Arg::Long(_) | Arg::Double(_) => {
                    while !memory.len().is_multiple_of(8) {
                        memory.push(0);
                    }
                    let bits = match arg {
                        Arg::Long(v) => *v as u64,
                        Arg::Double(v) => v.to_bits(),
                        _ => unreachable!(),
                    };
                    memory.extend(bits.to_le_bytes());
                }
            }
        }
        // Align the start of the arguments to 8 bytes, as the shims do.
        let mut aligned = std::vec![0u64; memory.len() / 8 + 1];
        let bytes = aligned.as_mut_ptr().cast::<u8>();
        unsafe { ptr::copy_nonoverlapping(memory.as_ptr(), bytes, memory.len()) };
        let mut buffer = [0u8; 256];
        let count = unsafe {
            esp_sr_vsnprintf(
                buffer.as_mut_ptr().cast(),
                buffer.len(),
                format_.as_ptr(),
                bytes,
            )
        };
        let text = CStr::from_bytes_until_nul(&buffer)
            .unwrap()
            .to_str()
            .unwrap();
        assert_eq!(count as usize, text.len());
        text.into()
    }

    #[test]
    fn integers() {
        use Arg::*;
        assert_eq!(
            sprintf(
                c"%d|%5d|%-5d|%05d|%+d|% d",
                &[Int(42), Int(42), Int(42), Int(-42), Int(7), Int(7)]
            ),
            "42|   42|42   |-0042|+7| 7"
        );
        assert_eq!(
            sprintf(
                c"%u %x %X %#x %o %#o",
                &[Int(-1), Int(255), Int(255), Int(255), Int(8), Int(8)]
            ),
            "4294967295 ff FF 0xff 10 010"
        );
        assert_eq!(
            sprintf(
                c"%.3d|%.0d|%lld|%hhd",
                &[Int(5), Int(0), Long(-1234567890123), Int(300)]
            ),
            "005||-1234567890123|44"
        );
        assert_eq!(
            sprintf(
                c"%*d|%-*d|%c|%%",
                &[Int(4), Int(1), Int(3), Int(2), Int(b'A' as i32)]
            ),
            "   1|2  |A|%"
        );
    }

    #[test]
    fn floats() {
        use Arg::*;
        assert_eq!(
            sprintf(
                c"%f|%.2f|%8.3f|%-8.1f|%.0f",
                &[
                    Double(1.23456),
                    Double(2.005),
                    Double(-1.5),
                    Double(1.25),
                    Double(2.5)
                ]
            ),
            "1.234560|2.00|  -1.500|1.2     |2"
        );
        assert_eq!(
            sprintf(
                c"%e|%.2E|%e",
                &[Double(12345.678), Double(0.000123), Double(0.0)]
            ),
            "1.234568e+04|1.23E-04|0.000000e+00"
        );
        assert_eq!(
            sprintf(
                c"%g|%g|%g|%g|%.3g|%g",
                &[
                    Double(0.0001),
                    Double(0.00001),
                    Double(123456.0),
                    Double(1234567.0),
                    Double(1.23456),
                    Double(100.0)
                ]
            ),
            "0.0001|1e-05|123456|1.23457e+06|1.23|100"
        );
        assert_eq!(
            sprintf(c"%d %f %d", &[Int(1), Double(0.5), Int(2)]),
            "1 0.500000 2"
        );
    }
}
