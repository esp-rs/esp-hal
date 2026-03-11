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
            // We assume malloc returns a 4-byte aligned pointer. We can skip aligning types
            // that are already aligned to 4 bytes or less.
            let ptr = if layout.align() <= 4 {
                unsafe { super::malloc_internal(layout.size()) }
            } else {
                // We allocate extra memory so that we can store the number of prefix bytes in the
                // bytes before the actual allocation. We will then use this to
                // restore the pointer to the original allocation.

                // If we can get away with 0 padding bytes, let's do that. In this case, we need
                // space for the prefix length only.
                // We assume malloc returns a 4-byte aligned pointer. This means any higher
                // alignment requirements can be satisfied by at most align - 4
                // bytes of shift, and we can use the remaining 4 bytes for the prefix length.
                let extra = layout.align().max(4);

                let allocation = unsafe { super::malloc_internal(layout.size() + extra) };

                if allocation.is_null() {
                    return Err(AllocError);
                }

                // reserve at least 4 bytes for the prefix
                let ptr = allocation.wrapping_add(4);

                let align_offset = ptr.align_offset(layout.align());

                let data_ptr = ptr.wrapping_add(align_offset);
                let prefix_ptr = data_ptr.wrapping_sub(4);

                // Store the amount of padding bytes used for alignment.
                unsafe { prefix_ptr.cast::<usize>().write(align_offset) };

                data_ptr
            };

            let ptr = NonNull::new(ptr).ok_or(AllocError)?;
            Ok(NonNull::slice_from_raw_parts(ptr, layout.size()))
        }

        unsafe fn deallocate(&self, ptr: NonNull<u8>, layout: Layout) {
            // We assume malloc returns a 4-byte aligned pointer. In that case we don't have to
            // align, so we don't have a prefix.
            if layout.align() <= 4 {
                unsafe { super::free_internal(ptr.as_ptr()) };
            } else {
                // Retrieve the amount of padding bytes used for alignment.
                let prefix_ptr = ptr.as_ptr().wrapping_sub(4);
                let prefix_bytes = unsafe { prefix_ptr.cast::<usize>().read() };

                unsafe { super::free_internal(prefix_ptr.wrapping_sub(prefix_bytes)) };
            }
        }
    }
}

pub(crate) use esp_alloc::InternalMemory;
