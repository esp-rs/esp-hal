#[cfg(feature = "nightly")]
use core::alloc::{AllocError as CoreAllocError, Allocator as CoreAllocator};
use core::{
    alloc::{GlobalAlloc, Layout},
    ptr::NonNull,
};

use allocator_api2::alloc::{AllocError, Allocator};
use enumset::EnumSet;

use crate::MemoryCapability;

fn allocate_caps(
    capabilities: EnumSet<MemoryCapability>,
    layout: Layout,
) -> Result<NonNull<[u8]>, AllocError> {
    let raw_ptr = unsafe { crate::HEAP.alloc_caps(capabilities, layout) };
    let ptr = NonNull::new(raw_ptr).ok_or(AllocError)?;
    Ok(NonNull::slice_from_raw_parts(ptr, layout.size()))
}

use crate::EspHeap;

unsafe impl Allocator for EspHeap {
    fn allocate(&self, layout: Layout) -> Result<NonNull<[u8]>, AllocError> {
        let raw_ptr = unsafe { self.alloc(layout) };
        let ptr = NonNull::new(raw_ptr).ok_or(AllocError)?;
        Ok(NonNull::slice_from_raw_parts(ptr, layout.size()))
    }

    unsafe fn deallocate(&self, ptr: NonNull<u8>, layout: Layout) {
        unsafe {
            crate::HEAP.dealloc(ptr.as_ptr(), layout);
        }
    }
}

#[cfg(feature = "nightly")]
unsafe impl CoreAllocator for EspHeap {
    fn allocate(&self, layout: Layout) -> Result<NonNull<[u8]>, CoreAllocError> {
        let raw_ptr = unsafe { self.alloc(layout) };
        let ptr = NonNull::new(raw_ptr).ok_or(CoreAllocError)?;
        Ok(NonNull::slice_from_raw_parts(ptr, layout.size()))
    }

    unsafe fn deallocate(&self, ptr: NonNull<u8>, layout: Layout) {
        unsafe {
            crate::HEAP.dealloc(ptr.as_ptr(), layout);
        }
    }
}

/// An allocator that uses all configured, available memory.
pub struct AnyMemory;

unsafe impl Allocator for AnyMemory {
    fn allocate(&self, layout: Layout) -> Result<NonNull<[u8]>, AllocError> {
        allocate_caps(EnumSet::empty(), layout)
    }

    unsafe fn deallocate(&self, ptr: NonNull<u8>, layout: Layout) {
        unsafe {
            crate::HEAP.dealloc(ptr.as_ptr(), layout);
        }
    }
}

#[cfg(feature = "nightly")]
unsafe impl CoreAllocator for AnyMemory {
    fn allocate(&self, layout: Layout) -> Result<NonNull<[u8]>, CoreAllocError> {
        allocate_caps(EnumSet::empty(), layout).map_err(|_| CoreAllocError)
    }

    unsafe fn deallocate(&self, ptr: NonNull<u8>, layout: Layout) {
        unsafe {
            crate::HEAP.dealloc(ptr.as_ptr(), layout);
        }
    }
}

/// An allocator that uses internal memory only.
pub struct InternalMemory;

unsafe impl Allocator for InternalMemory {
    fn allocate(&self, layout: Layout) -> Result<NonNull<[u8]>, AllocError> {
        allocate_caps(EnumSet::from(MemoryCapability::Internal), layout)
    }

    unsafe fn deallocate(&self, ptr: NonNull<u8>, layout: Layout) {
        unsafe {
            crate::HEAP.dealloc(ptr.as_ptr(), layout);
        }
    }
}

#[cfg(feature = "nightly")]
unsafe impl CoreAllocator for InternalMemory {
    fn allocate(&self, layout: Layout) -> Result<NonNull<[u8]>, CoreAllocError> {
        allocate_caps(EnumSet::from(MemoryCapability::Internal), layout).map_err(|_| CoreAllocError)
    }

    unsafe fn deallocate(&self, ptr: NonNull<u8>, layout: Layout) {
        unsafe {
            crate::HEAP.dealloc(ptr.as_ptr(), layout);
        }
    }
}

/// An allocator that uses external (PSRAM) memory only.
pub struct ExternalMemory;

unsafe impl Allocator for ExternalMemory {
    fn allocate(&self, layout: Layout) -> Result<NonNull<[u8]>, AllocError> {
        allocate_caps(EnumSet::from(MemoryCapability::External), layout)
    }

    unsafe fn deallocate(&self, ptr: NonNull<u8>, layout: Layout) {
        unsafe {
            crate::HEAP.dealloc(ptr.as_ptr(), layout);
        }
    }
}

#[cfg(feature = "nightly")]
unsafe impl CoreAllocator for ExternalMemory {
    fn allocate(&self, layout: Layout) -> Result<NonNull<[u8]>, CoreAllocError> {
        allocate_caps(EnumSet::from(MemoryCapability::External), layout).map_err(|_| CoreAllocError)
    }

    unsafe fn deallocate(&self, ptr: NonNull<u8>, layout: Layout) {
        unsafe {
            crate::HEAP.dealloc(ptr.as_ptr(), layout);
        }
    }
}
