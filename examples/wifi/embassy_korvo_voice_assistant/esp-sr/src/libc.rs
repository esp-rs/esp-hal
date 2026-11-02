//! The C library, ESP-IDF and FreeRTOS functions that the ESP-SR libraries import, under the names
//! `tools/renames.txt` gives them.
//!
//! The libraries only run on one core at a time, see [`crate::MultiNet`], so the FreeRTOS mutexes
//! do nothing and `strtok` keeps its state in a static.

use core::{
    cell::Cell,
    ffi::{CStr, c_char, c_int, c_long, c_void},
    ptr,
    sync::atomic::{AtomicPtr, Ordering},
};

use critical_section::Mutex;

use crate::memory::{allocate, allocate_zeroed, allocation_size, deallocate, duplicate};

// Memory, all in PSRAM, see `memory`.

#[unsafe(no_mangle)]
extern "C" fn esp_sr_malloc(size: usize) -> *mut c_void {
    allocate(size, 4).cast()
}

#[unsafe(no_mangle)]
extern "C" fn esp_sr_calloc(count: usize, size: usize) -> *mut c_void {
    allocate_zeroed(count, size).cast()
}

#[unsafe(no_mangle)]
unsafe extern "C" fn esp_sr_realloc(ptr: *mut c_void, size: usize) -> *mut c_void {
    let new = allocate(size, 4);
    if !new.is_null() && !ptr.is_null() {
        // SAFETY: C callers pass pointers from these functions that they have not freed.
        unsafe {
            let old = ptr.cast::<u8>();
            ptr::copy_nonoverlapping(old, new, allocation_size(old).min(size));
            deallocate(old);
        }
    }
    new.cast()
}

#[unsafe(no_mangle)]
unsafe extern "C" fn esp_sr_free(ptr: *mut c_void) {
    // SAFETY: C callers pass pointers from these functions that they have not freed.
    unsafe { deallocate(ptr.cast()) }
}

#[unsafe(no_mangle)]
extern "C" fn esp_sr_heap_caps_malloc(size: usize, _caps: u32) -> *mut c_void {
    allocate(size, 4).cast()
}

#[unsafe(no_mangle)]
extern "C" fn esp_sr_heap_caps_calloc(count: usize, size: usize, _caps: u32) -> *mut c_void {
    allocate_zeroed(count, size).cast()
}

#[unsafe(no_mangle)]
extern "C" fn esp_sr_heap_caps_aligned_alloc(align: usize, size: usize, _caps: u32) -> *mut c_void {
    allocate(size, align).cast()
}

#[unsafe(no_mangle)]
unsafe extern "C" fn esp_sr_heap_caps_free(ptr: *mut c_void) {
    unsafe { deallocate(ptr.cast()) }
}

#[unsafe(no_mangle)]
extern "C" fn esp_sr_heap_caps_get_largest_free_block(_caps: u32) -> usize {
    crate::memory::free_bytes()
}

// Strings and numbers.

#[unsafe(no_mangle)]
unsafe extern "C" fn esp_sr_strdup(text: *const c_char) -> *mut c_char {
    // SAFETY: C callers pass a NUL-terminated string.
    duplicate(unsafe { CStr::from_ptr(text) }.to_bytes())
}

#[unsafe(no_mangle)]
unsafe extern "C" fn esp_sr_strrchr(text: *const c_char, c: c_int) -> *mut c_char {
    // SAFETY: C callers pass a NUL-terminated string.
    let bytes = unsafe { CStr::from_ptr(text) }.to_bytes_with_nul();
    match bytes.iter().rposition(|&byte| byte == c as u8) {
        Some(index) => text.wrapping_add(index).cast_mut(),
        None => ptr::null_mut(),
    }
}

/// Where `strtok` continues.
static STRTOK_NEXT: AtomicPtr<c_char> = AtomicPtr::new(ptr::null_mut());

#[unsafe(no_mangle)]
unsafe extern "C" fn esp_sr_strtok(text: *mut c_char, delimiters: *const c_char) -> *mut c_char {
    let mut cursor = if text.is_null() {
        STRTOK_NEXT.load(Ordering::Relaxed)
    } else {
        text
    };
    if cursor.is_null() {
        return ptr::null_mut();
    }
    // SAFETY: C callers pass NUL-terminated strings, and `cursor` stays within the string.
    unsafe {
        let delimiters = CStr::from_ptr(delimiters).to_bytes();
        // `c_char` is `u8` on RISC-V and `i8` on some hosts.
        #[allow(clippy::unnecessary_cast)]
        let is_delimiter = |c: c_char| delimiters.contains(&(c as u8));
        while *cursor != 0 && is_delimiter(*cursor) {
            cursor = cursor.add(1);
        }
        if *cursor == 0 {
            STRTOK_NEXT.store(ptr::null_mut(), Ordering::Relaxed);
            return ptr::null_mut();
        }
        let token = cursor;
        while *cursor != 0 && !is_delimiter(*cursor) {
            cursor = cursor.add(1);
        }
        let next = if *cursor == 0 {
            ptr::null_mut()
        } else {
            *cursor = 0;
            cursor.add(1)
        };
        STRTOK_NEXT.store(next, Ordering::Relaxed);
        token
    }
}

/// Parses the longest number at the start of `text`, after white space, as C's `strtod` does for
/// decimal numbers. Returns the number and the bytes it took, including the white space.
pub fn parse_float(text: &[u8]) -> (f64, usize) {
    let start = text
        .iter()
        .take_while(|byte| byte.is_ascii_whitespace())
        .count();
    let mut end = start;
    let digits = |from: usize| {
        text[from..]
            .iter()
            .take_while(|b| b.is_ascii_digit())
            .count()
    };
    if matches!(text.get(end), Some(b'+' | b'-')) {
        end += 1;
    }
    let integer = digits(end);
    end += integer;
    let mut fraction = 0;
    if text.get(end) == Some(&b'.') {
        fraction = digits(end + 1);
        end += 1 + fraction;
    }
    if integer + fraction == 0 {
        return (0.0, 0);
    }
    if matches!(text.get(end), Some(b'e' | b'E')) {
        let mut exponent = end + 1;
        if matches!(text.get(exponent), Some(b'+' | b'-')) {
            exponent += 1;
        }
        if digits(exponent) > 0 {
            end = exponent + digits(exponent);
        }
    }
    let number = core::str::from_utf8(&text[start..end]).unwrap();
    // `f64` does not accept a fraction without digits, such as "1.".
    let number = number.strip_suffix('.').unwrap_or(number);
    (number.parse().unwrap_or(0.0), end)
}

#[unsafe(no_mangle)]
unsafe extern "C" fn esp_sr_atof(text: *const c_char) -> f64 {
    // SAFETY: C callers pass a NUL-terminated string.
    parse_float(unsafe { CStr::from_ptr(text) }.to_bytes()).0
}

#[unsafe(no_mangle)]
unsafe extern "C" fn esp_sr_strtof(text: *const c_char, end: *mut *mut c_char) -> f32 {
    // SAFETY: C callers pass a NUL-terminated string and a valid or null `end`.
    unsafe {
        let (number, length) = parse_float(CStr::from_ptr(text).to_bytes());
        if !end.is_null() {
            *end = text.add(length).cast_mut();
        }
        number as f32
    }
}

/// The state of `rand`, as picolibc keeps it.
static RAND_NEXT: Mutex<Cell<u64>> = Mutex::new(Cell::new(1));

#[unsafe(no_mangle)]
extern "C" fn esp_sr_srand(seed: u32) {
    critical_section::with(|cs| RAND_NEXT.borrow(cs).set(u64::from(seed)));
}

/// picolibc's generator, which the ESP-SR libraries were built with: a 64-bit linear congruential
/// generator that returns bits 32 to 62.
#[unsafe(no_mangle)]
extern "C" fn esp_sr_rand() -> c_int {
    critical_section::with(|cs| {
        let next = RAND_NEXT.borrow(cs);
        next.set(
            next.get()
                .wrapping_mul(0x5851_F42D_4C95_7F2D)
                .wrapping_add(1),
        );
        ((next.get() >> 32) as u32 & 0x7FFF_FFFF) as c_int
    })
}

/// There is no clock, so the time is always the epoch. Only the subword tokenizer reads it, to seed
/// `rand`, and the seed does not change what MultiNet recognizes.
#[unsafe(no_mangle)]
unsafe extern "C" fn esp_sr_time(time: *mut i64) -> i64 {
    if !time.is_null() {
        // SAFETY: C callers pass a valid or null pointer.
        unsafe { time.write(0) };
    }
    0
}

#[unsafe(no_mangle)]
extern "C" fn esp_sr_exit(status: c_int) -> ! {
    panic!("ESP-SR exited with status {status}")
}

#[unsafe(no_mangle)]
unsafe extern "C" fn esp_sr_assert_func(
    file: *const c_char,
    line: c_int,
    function: *const c_char,
    expression: *const c_char,
) -> ! {
    // SAFETY: C callers pass NUL-terminated strings, or null for the function.
    let text = |text: *const c_char| unsafe {
        if text.is_null() {
            c"?"
        } else {
            CStr::from_ptr(text)
        }
    };
    panic!(
        "ESP-SR assertion {:?} failed in {:?} at {:?}:{line}",
        text(expression),
        text(function),
        text(file)
    )
}

/// The libraries only call `longjmp` after a `setjmp`, which they never call.
#[unsafe(no_mangle)]
extern "C" fn esp_sr_longjmp(_environment: *mut c_void, _value: c_int) -> ! {
    panic!("ESP-SR called longjmp")
}

// Files. The models are in memory, so there are no files: opening fails, and the functions on a
// stream fail without one. `printf.rs` writes the standard streams.

/// A `FILE *`. The standard streams only need to differ from each other and from null.
#[repr(transparent)]
pub struct Stream(pub(crate) *const c_void);

// SAFETY: Nothing reads through the pointer.
unsafe impl Sync for Stream {}

static STREAM: u8 = 0;

#[unsafe(no_mangle)]
static esp_sr_stdin: Stream = Stream((&raw const STREAM).cast());
#[unsafe(no_mangle)]
static esp_sr_stdout: Stream = Stream((&raw const STREAM).wrapping_add(1).cast());
#[unsafe(no_mangle)]
pub(crate) static esp_sr_stderr: Stream = Stream((&raw const STREAM).wrapping_add(2).cast());

const EOF: c_int = -1;

#[unsafe(no_mangle)]
extern "C" fn esp_sr_fopen(_path: *const c_char, _mode: *const c_char) -> *mut c_void {
    ptr::null_mut()
}

#[unsafe(no_mangle)]
extern "C" fn esp_sr_fdopen(_fd: c_int, _mode: *const c_char) -> *mut c_void {
    ptr::null_mut()
}

#[unsafe(no_mangle)]
extern "C" fn esp_sr_fclose(_stream: *mut c_void) -> c_int {
    EOF
}

#[unsafe(no_mangle)]
extern "C" fn esp_sr_fgetc(_stream: *mut c_void) -> c_int {
    EOF
}

#[unsafe(no_mangle)]
extern "C" fn esp_sr_fgets(_line: *mut c_char, _size: c_int, _stream: *mut c_void) -> *mut c_char {
    ptr::null_mut()
}

#[unsafe(no_mangle)]
extern "C" fn esp_sr_fread(
    _data: *mut c_void,
    _size: usize,
    _count: usize,
    _stream: *mut c_void,
) -> usize {
    0
}

#[unsafe(no_mangle)]
extern "C" fn esp_sr_fseek(_stream: *mut c_void, _offset: c_long, _whence: c_int) -> c_int {
    -1
}

#[unsafe(no_mangle)]
extern "C" fn esp_sr_ftell(_stream: *mut c_void) -> c_long {
    -1
}

#[unsafe(no_mangle)]
extern "C" fn esp_sr_feof(_stream: *mut c_void) -> c_int {
    1
}

// FreeRTOS mutexes, which only need to exist.

static MUTEX: u8 = 0;

#[unsafe(no_mangle)]
extern "C" fn esp_sr_xQueueCreateMutex(_queue_type: u8) -> *const c_void {
    (&raw const MUTEX).cast()
}

#[unsafe(no_mangle)]
extern "C" fn esp_sr_xQueueGenericCreate(
    _length: u32,
    _item_size: u32,
    _queue_type: u8,
) -> *const c_void {
    (&raw const MUTEX).cast()
}

#[unsafe(no_mangle)]
extern "C" fn esp_sr_xQueueSemaphoreTake(_queue: *const c_void, _ticks_to_wait: u32) -> c_int {
    1
}

#[unsafe(no_mangle)]
extern "C" fn esp_sr_xQueueGenericSend(
    _queue: *const c_void,
    _item: *const c_void,
    _ticks_to_wait: u32,
    _copy_position: c_int,
) -> c_int {
    1
}

#[unsafe(no_mangle)]
extern "C" fn esp_sr_vQueueDelete(_queue: *const c_void) {}

#[unsafe(no_mangle)]
extern "C" fn esp_sr_esp_log_timestamp() -> u32 {
    0
}
