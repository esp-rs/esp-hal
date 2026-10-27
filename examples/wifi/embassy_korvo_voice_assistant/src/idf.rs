//! The ESP-IDF and C library functions that ESP-SR calls and picolibc leaves to the application.
//!
//! Only the second core runs ESP-SR, so the FreeRTOS mutexes do nothing.

use core::{
    alloc::{GlobalAlloc, Layout},
    ffi::{c_char, c_int, c_void},
    ptr,
};

use esp_alloc::{HEAP, MemoryCapability, export::enumset::EnumSet};
use esp_hal::system::Cpu;

// The C allocation functions of ESP-SR, the Wi-Fi driver and esp-rtos. They replace those of
// esp-alloc's `compat` feature, which align to 4 bytes only and cannot free the aligned allocations
// of ESP-SR. Any of the `free` functions frees any allocation.

/// ESP-IDF's `MALLOC_CAP_*` flags that select the memory.
const MALLOC_CAP_DMA: u32 = 1 << 3;
const MALLOC_CAP_SPIRAM: u32 = 1 << 10;
const MALLOC_CAP_INTERNAL: u32 = 1 << 11;

/// The size and the alignment of each allocation are stored in the 8 bytes before the returned
/// pointer.
const HEADER: usize = 8;

fn allocate(size: usize, align: usize, capabilities: EnumSet<MemoryCapability>) -> *mut c_void {
    // Only ESP-SR runs on the second core. Its memory is in PSRAM, so that it cannot take the
    // internal RAM that Wi-Fi needs.
    let capabilities = match Cpu::current() {
        Cpu::AppCpu => MemoryCapability::External.into(),
        Cpu::ProCpu => capabilities,
    };
    let align = align.max(4);
    let offset = align.max(HEADER);
    let Ok(layout) = Layout::from_size_align(size + offset, align) else {
        return ptr::null_mut();
    };
    // SAFETY: The layout is not empty.
    let allocation = unsafe { HEAP.alloc_caps(capabilities, layout) };
    if allocation.is_null() {
        return ptr::null_mut();
    }
    // SAFETY: The allocation has `offset` bytes before the returned pointer, enough for the header.
    unsafe {
        let ptr = allocation.add(offset);
        let header = ptr.cast::<usize>();
        header.sub(2).write(layout.size());
        header.sub(1).write(layout.align());
        ptr.cast()
    }
}

/// The layout and the start of the allocation behind a pointer from [`allocate`].
///
/// # Safety
///
/// `ptr` must come from [`allocate`] and must not have been freed.
unsafe fn allocation(ptr: *mut c_void) -> (Layout, *mut u8) {
    unsafe {
        let header = ptr.cast::<usize>();
        let layout = Layout::from_size_align_unchecked(header.sub(2).read(), header.sub(1).read());
        (layout, ptr.cast::<u8>().sub(layout.align().max(HEADER)))
    }
}

fn allocate_zeroed(
    count: usize,
    size: usize,
    capabilities: EnumSet<MemoryCapability>,
) -> *mut c_void {
    let Some(size) = count.checked_mul(size) else {
        return ptr::null_mut();
    };
    let ptr = allocate(size, 4, capabilities);
    if !ptr.is_null() {
        // SAFETY: The allocation has `size` bytes.
        unsafe { ptr.cast::<u8>().write_bytes(0, size) };
    }
    ptr
}

/// # Safety
///
/// `ptr` must be null or come from [`allocate`] and not have been freed.
unsafe fn reallocate(
    ptr: *mut c_void,
    size: usize,
    capabilities: EnumSet<MemoryCapability>,
) -> *mut c_void {
    let new = allocate(size, 4, capabilities);
    if !new.is_null() && !ptr.is_null() {
        unsafe {
            let (layout, start) = allocation(ptr);
            let old_size = layout.size() - (ptr.cast::<u8>().offset_from(start) as usize);
            ptr::copy_nonoverlapping(ptr.cast::<u8>(), new.cast(), old_size.min(size));
            free(ptr);
        }
    }
    new
}

/// The memory that ESP-IDF's `MALLOC_CAP_*` flags select.
fn capabilities(caps: u32) -> EnumSet<MemoryCapability> {
    if caps & MALLOC_CAP_SPIRAM != 0 {
        MemoryCapability::External.into()
    } else if caps & (MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA) != 0 {
        MemoryCapability::Internal.into()
    } else {
        EnumSet::empty()
    }
}

#[unsafe(no_mangle)]
extern "C" fn malloc(size: usize) -> *mut c_void {
    allocate(size, 4, EnumSet::empty())
}

#[unsafe(no_mangle)]
extern "C" fn malloc_internal(size: usize) -> *mut c_void {
    allocate(size, 4, MemoryCapability::Internal.into())
}

#[unsafe(no_mangle)]
extern "C" fn calloc(count: usize, size: usize) -> *mut c_void {
    allocate_zeroed(count, size, EnumSet::empty())
}

#[unsafe(no_mangle)]
extern "C" fn calloc_internal(count: usize, size: usize) -> *mut c_void {
    allocate_zeroed(count, size, MemoryCapability::Internal.into())
}

#[unsafe(no_mangle)]
unsafe extern "C" fn realloc(ptr: *mut c_void, size: usize) -> *mut c_void {
    unsafe { reallocate(ptr, size, EnumSet::empty()) }
}

#[unsafe(no_mangle)]
unsafe extern "C" fn realloc_internal(ptr: *mut c_void, size: usize) -> *mut c_void {
    unsafe { reallocate(ptr, size, MemoryCapability::Internal.into()) }
}

#[unsafe(no_mangle)]
unsafe extern "C" fn free(ptr: *mut c_void) {
    if ptr.is_null() {
        return;
    }
    // SAFETY: C callers pass pointers from these functions that they have not freed.
    unsafe {
        let (layout, start) = allocation(ptr);
        HEAP.dealloc(start, layout);
    }
}

#[unsafe(no_mangle)]
unsafe extern "C" fn free_internal(ptr: *mut c_void) {
    unsafe { free(ptr) }
}

#[unsafe(no_mangle)]
extern "C" fn get_free_internal_heap_size() -> usize {
    HEAP.free_caps(MemoryCapability::Internal.into())
}

#[unsafe(no_mangle)]
extern "C" fn heap_caps_malloc(size: usize, caps: u32) -> *mut c_void {
    allocate(size, 4, capabilities(caps))
}

#[unsafe(no_mangle)]
extern "C" fn heap_caps_calloc(count: usize, size: usize, caps: u32) -> *mut c_void {
    allocate_zeroed(count, size, capabilities(caps))
}

#[unsafe(no_mangle)]
extern "C" fn heap_caps_aligned_alloc(alignment: usize, size: usize, caps: u32) -> *mut c_void {
    allocate(size, alignment, capabilities(caps))
}

#[unsafe(no_mangle)]
unsafe extern "C" fn heap_caps_free(ptr: *mut c_void) {
    unsafe { free(ptr) }
}

/// A FreeRTOS queue handle. ESP-SR only creates mutexes, which need no state.
static MUTEX: u8 = 0;

#[unsafe(no_mangle)]
extern "C" fn xQueueCreateMutex(_queue_type: u8) -> *const c_void {
    (&raw const MUTEX).cast()
}

#[unsafe(no_mangle)]
extern "C" fn xQueueSemaphoreTake(_queue: *const c_void, _ticks_to_wait: u32) -> c_int {
    1
}

#[unsafe(no_mangle)]
extern "C" fn xQueueGenericSend(
    _queue: *const c_void,
    _item: *const c_void,
    _ticks_to_wait: u32,
    _copy_position: c_int,
) -> c_int {
    1
}

#[unsafe(no_mangle)]
extern "C" fn vQueueDelete(_queue: *const c_void) {}

/// `dl::base::dotprod(float *, float *, float *, int, int)` from ESP-DL, which the speech features
/// use. ESP-DL ignores `shift` for floats.
#[unsafe(export_name = "_ZN2dl4base7dotprodEPfS1_S1_ii")]
unsafe extern "C" fn dotprod(
    a: *const f32,
    b: *const f32,
    out: *mut f32,
    len: c_int,
    _shift: c_int,
) {
    // SAFETY: ESP-SR passes two vectors of `len` floats and one output float.
    unsafe {
        let a = core::slice::from_raw_parts(a, len as usize);
        let b = core::slice::from_raw_parts(b, len as usize);
        out.write(a.iter().zip(b).map(|(a, b)| a * b).sum());
    }
}

/// Writes the output of picolibc's `printf` and friends, see `c/stdio.c`.
#[unsafe(no_mangle)]
extern "C" fn esp_sr_putc(c: c_char, _stream: *mut c_void) -> c_int {
    esp_println::Printer::write_bytes(&[c]);
    c_int::from(c)
}

/// ESP-SR reads its models from memory, there is no file system.
#[unsafe(no_mangle)]
extern "C" fn fopen(_path: *const c_char, _mode: *const c_char) -> *mut c_void {
    ptr::null_mut()
}

#[unsafe(no_mangle)]
extern "C" fn exit(status: c_int) -> ! {
    panic!("ESP-SR exited with status {status}")
}

#[unsafe(no_mangle)]
extern "C" fn _exit(status: c_int) -> ! {
    exit(status)
}
