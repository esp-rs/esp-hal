use core::{alloc::GlobalAlloc, ptr::NonNull};

use allocator_api2::alloc::{AllocError, Allocator, Layout};

use crate::EspHeap;

unsafe impl Allocator for EspHeap {
    fn allocate(&self, layout: Layout) -> Result<NonNull<[u8]>, AllocError> {
        let raw_ptr = unsafe { self.alloc(layout) };

        if raw_ptr.is_null() {
            return Err(AllocError);
        }

        let ptr = NonNull::new(raw_ptr).ok_or(AllocError)?;
        Ok(NonNull::slice_from_raw_parts(ptr, layout.size()))
    }

    unsafe fn deallocate(&self, ptr: NonNull<u8>, layout: Layout) {
        crate::HEAP.dealloc(ptr.as_ptr(), layout);
    }
}

/// An allocator that uses all configured, available memory.
pub struct AnyMemory;

unsafe impl Allocator for AnyMemory {
    fn allocate(&self, layout: Layout) -> Result<NonNull<[u8]>, AllocError> {
        let raw_ptr = unsafe { crate::HEAP.alloc(layout) };

        if raw_ptr.is_null() {
            return Err(AllocError);
        }

        let ptr = NonNull::new(raw_ptr).ok_or(AllocError)?;
        Ok(NonNull::slice_from_raw_parts(ptr, layout.size()))
    }

    unsafe fn deallocate(&self, ptr: NonNull<u8>, layout: Layout) {
        crate::HEAP.dealloc(ptr.as_ptr(), layout);
    }
}
