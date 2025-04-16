#[unsafe(no_mangle)]
pub unsafe extern "C" fn malloc(size: usize) -> *mut u8 {
    trace!("alloc {}", size);

    unsafe extern "C" {
        fn esp_wifi_allocate_from_internal_ram(size: usize) -> *mut u8;
    }

    let ptr = unsafe { esp_wifi_allocate_from_internal_ram(size) };

    if ptr.is_null() {
        warn!("Unable to allocate {} bytes", size);
    }

    ptr
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn free(ptr: *mut u8) {
    unsafe {
        trace!("free {:?}", ptr);

        if ptr.is_null() {
            warn!("Attempt to free null pointer");
            return;
        }

        unsafe extern "C" {
            fn esp_wifi_deallocate_internal_ram(ptr: *mut u8);
        }

        esp_wifi_deallocate_internal_ram(ptr);
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn calloc(number: u32, size: usize) -> *mut u8 {
    unsafe {
        trace!("calloc {} {}", number, size);

        let total_size = number as usize * size;
        let ptr = malloc(total_size);

        if !ptr.is_null() {
            for i in 0..total_size as isize {
                ptr.offset(i).write_volatile(0);
            }
        }

        ptr
    }
}

#[unsafe(no_mangle)]
unsafe extern "C" fn realloc(ptr: *mut u8, new_size: usize) -> *mut u8 {
    trace!("realloc {:?} {}", ptr, new_size);

    unsafe extern "C" {
        fn memcpy(d: *mut u8, s: *const u8, l: usize);
    }

    unsafe {
        let p = malloc(new_size);
        if !p.is_null() && !ptr.is_null() {
            let len = usize::min(
                (ptr as *const u32).sub(1).read_volatile() as usize,
                new_size,
            );
            memcpy(p, ptr, len);
            free(ptr);
        }
        p
    }
}

#[cfg(feature = "esp-alloc")]
#[doc(hidden)]
#[unsafe(no_mangle)]
pub extern "C" fn esp_wifi_free_internal_heap() -> usize {
    esp_alloc::HEAP.free_caps(esp_alloc::MemoryCapability::Internal.into())
}

#[cfg(feature = "esp-alloc")]
#[doc(hidden)]
#[unsafe(no_mangle)]
pub extern "C" fn esp_wifi_allocate_from_internal_ram(size: usize) -> *mut u8 {
    let total_size = size + 4;
    unsafe {
        let ptr = esp_alloc::HEAP.alloc_caps(
            esp_alloc::MemoryCapability::Internal.into(),
            core::alloc::Layout::from_size_align_unchecked(total_size, 4),
        );

        if ptr.is_null() {
            return ptr;
        }

        *(ptr as *mut usize) = total_size;
        ptr.offset(4)
    }
}

#[cfg(feature = "esp-alloc")]
#[doc(hidden)]
#[unsafe(no_mangle)]
pub extern "C" fn esp_wifi_deallocate_internal_ram(ptr: *mut u8) {
    use core::alloc::GlobalAlloc;

    unsafe {
        let ptr = ptr.offset(-4);
        let total_size = *(ptr as *const usize);

        esp_alloc::HEAP.dealloc(
            ptr,
            core::alloc::Layout::from_size_align_unchecked(total_size, 4),
        )
    }
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
            unsafe extern "C" {
                fn esp_wifi_allocate_from_internal_ram(size: usize) -> *mut u8;
            }
            let raw_ptr = unsafe { esp_wifi_allocate_from_internal_ram(layout.size()) };
            let ptr = NonNull::new(raw_ptr).ok_or(AllocError)?;
            Ok(NonNull::slice_from_raw_parts(ptr, layout.size()))
        }

        unsafe fn deallocate(&self, ptr: NonNull<u8>, _layout: Layout) {
            unsafe {
                unsafe extern "C" {
                    fn esp_wifi_deallocate_internal_ram(ptr: *mut u8);
                }
                esp_wifi_deallocate_internal_ram(ptr.as_ptr());
            }
        }
    }
}

pub(crate) use esp_alloc::InternalMemory;
