//! Memory for the ESP-SR libraries and the C structures this crate builds for them.
//!
//! On the chip, all of it is in PSRAM, so that the libraries cannot take the internal RAM other
//! drivers, such as Wi-Fi, need. The size and the alignment of each allocation are stored right
//! before the returned pointer, so that `free` needs nothing else.

use core::{alloc::Layout, ffi::c_char, ptr};

/// The size and the alignment, 8 bytes on the chip.
const HEADER: usize = 2 * size_of::<usize>();

#[cfg(target_arch = "riscv32")]
unsafe fn raw_allocate(layout: Layout) -> *mut u8 {
    use esp_alloc::{HEAP, MemoryCapability};
    unsafe { HEAP.alloc_caps(MemoryCapability::External.into(), layout) }
}

#[cfg(target_arch = "riscv32")]
unsafe fn raw_deallocate(start: *mut u8, layout: Layout) {
    use core::alloc::GlobalAlloc;
    unsafe { esp_alloc::HEAP.dealloc(start, layout) }
}

/// The free PSRAM.
#[cfg(target_arch = "riscv32")]
pub fn free_bytes() -> usize {
    use esp_alloc::{HEAP, MemoryCapability};
    HEAP.free_caps(MemoryCapability::External.into())
}

// Host tests use the system allocator.

#[cfg(not(target_arch = "riscv32"))]
unsafe fn raw_allocate(layout: Layout) -> *mut u8 {
    extern crate std;
    unsafe { std::alloc::alloc(layout) }
}

#[cfg(not(target_arch = "riscv32"))]
unsafe fn raw_deallocate(start: *mut u8, layout: Layout) {
    extern crate std;
    unsafe { std::alloc::dealloc(start, layout) }
}

#[cfg(not(target_arch = "riscv32"))]
pub fn free_bytes() -> usize {
    usize::MAX
}

/// Allocates `size` bytes aligned to `align`, or returns null.
pub fn allocate(size: usize, align: usize) -> *mut u8 {
    let align = align.max(align_of::<usize>());
    let offset = align.max(HEADER);
    let Some(total) = size.checked_add(offset) else {
        return ptr::null_mut();
    };
    let Ok(layout) = Layout::from_size_align(total, align) else {
        return ptr::null_mut();
    };
    // SAFETY: The layout is not empty.
    let start = unsafe { raw_allocate(layout) };
    if start.is_null() {
        return ptr::null_mut();
    }
    // SAFETY: The allocation has `offset` bytes before the returned pointer, enough for the header.
    unsafe {
        let ptr = start.add(offset);
        let header = ptr.cast::<usize>();
        header.sub(2).write(layout.size());
        header.sub(1).write(layout.align());
        ptr
    }
}

/// The layout and the start of the allocation behind `ptr`.
///
/// # Safety
///
/// `ptr` must come from [`allocate`] and not have been freed.
unsafe fn allocation(ptr: *mut u8) -> (Layout, *mut u8) {
    unsafe {
        let header = ptr.cast::<usize>();
        let layout = Layout::from_size_align_unchecked(header.sub(2).read(), header.sub(1).read());
        (layout, ptr.sub(layout.align().max(HEADER)))
    }
}

/// The bytes that [`allocate`] gave `ptr`, at least the size it asked for.
///
/// # Safety
///
/// `ptr` must come from [`allocate`] and not have been freed.
pub unsafe fn allocation_size(ptr: *mut u8) -> usize {
    unsafe {
        let (layout, start) = allocation(ptr);
        layout.size() - ptr.offset_from(start) as usize
    }
}

/// Frees memory from [`allocate`].
///
/// # Safety
///
/// `ptr` must be null or come from [`allocate`] and not have been freed.
pub unsafe fn deallocate(ptr: *mut u8) {
    if ptr.is_null() {
        return;
    }
    unsafe {
        let (layout, start) = allocation(ptr);
        raw_deallocate(start, layout);
    }
}

/// Allocates zeroed memory for `count` items of `size` bytes, or returns null.
pub fn allocate_zeroed(count: usize, size: usize) -> *mut u8 {
    let Some(size) = count.checked_mul(size) else {
        return ptr::null_mut();
    };
    let ptr = allocate(size, 4);
    if !ptr.is_null() {
        // SAFETY: The allocation has `size` bytes.
        unsafe { ptr.write_bytes(0, size) };
    }
    ptr
}

/// Copies `text` to a new NUL-terminated string, or returns null.
pub fn duplicate(text: &[u8]) -> *mut c_char {
    let copy = allocate(text.len() + 1, 1);
    if !copy.is_null() {
        // SAFETY: The allocation has room for the text and the NUL.
        unsafe {
            ptr::copy_nonoverlapping(text.as_ptr(), copy, text.len());
            copy.add(text.len()).write(0);
        }
    }
    copy.cast()
}

/// Allocates and initializes a `T`, or returns null.
pub fn boxed<T>(value: T) -> *mut T {
    let ptr = allocate(size_of::<T>(), align_of::<T>()).cast::<T>();
    if !ptr.is_null() {
        // SAFETY: The allocation fits a `T`.
        unsafe { ptr.write(value) };
    }
    ptr
}
