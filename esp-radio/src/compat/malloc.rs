//! These symbols are expected to be present.
//!
//! `esp-alloc` will provide them. The user is expected to provide implementation if
//! they don't want to use `esp-alloc`

#![allow(unused)]

unsafe extern "C" {
    pub fn malloc(size: usize) -> *mut u8;

    pub fn malloc_internal(size: usize) -> *mut u8;

    pub fn free(ptr: *mut u8);

    pub fn free_internal(ptr: *mut u8);

    pub fn realloc_internal(ptr: *mut u8, size: usize) -> *mut u8;

    pub fn calloc(number: u32, size: usize) -> *mut u8;

    pub fn calloc_internal(number: u32, size: usize) -> *mut u8;

    pub fn get_free_internal_heap_size() -> usize;
}

// Polyfill the InternalMemory allocator
#[cfg(not(feature = "esp-alloc"))]
mod esp_alloc {
    use core::{alloc::Layout, ptr::NonNull};

    use allocator_api2::alloc::{AllocError, Allocator};

    /// An allocator that uses internal memory only.
    pub struct InternalMemory;

    unsafe impl Allocator for InternalMemory {
        fn allocate(&self, layout: Layout) -> Result<NonNull<[u8]>, AllocError> {
            let raw_ptr = unsafe { super::malloc_internal(layout.size()) };
            let ptr = NonNull::new(raw_ptr).ok_or(AllocError)?;
            Ok(NonNull::slice_from_raw_parts(ptr, layout.size()))
        }

        unsafe fn deallocate(&self, ptr: NonNull<u8>, _layout: Layout) {
            unsafe { super::free_internal(ptr.as_ptr()) };
        }
    }
}

pub(crate) use esp_alloc::InternalMemory;
