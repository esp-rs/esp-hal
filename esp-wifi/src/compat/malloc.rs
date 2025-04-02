use core::alloc::Layout;

#[no_mangle]
pub unsafe extern "C" fn malloc(size: usize) -> *mut u8 {
    trace!("alloc {}", size);

    extern "C" {
        fn esp_wifi_allocate_from_internal_ram(size: usize) -> *mut u8;
    }

    let ptr = unsafe { esp_wifi_allocate_from_internal_ram(total_size) };

    if ptr.is_null() {
        warn!("Unable to allocate {} bytes", size);
    }

    ptr
}

#[no_mangle]
pub unsafe extern "C" fn free(ptr: *mut u8) {
    trace!("free {:?}", ptr);

    if ptr.is_null() {
        warn!("Attempt to free null pointer");
        return;
    }

    extern "C" {
        fn esp_wifi_deallocate_internal_ram(ptr: *mut u8);
    }

    esp_wifi_deallocate_internal_ram(ptr);
}

#[no_mangle]
pub unsafe extern "C" fn calloc(number: u32, size: usize) -> *mut u8 {
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

#[no_mangle]
unsafe extern "C" fn realloc(ptr: *mut u8, new_size: usize) -> *mut u8 {
    trace!("realloc {:?} {}", ptr, new_size);

    extern "C" {
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
#[no_mangle]
pub extern "C" fn esp_wifi_free_internal_heap() -> usize {
    esp_alloc::HEAP.free_caps(esp_alloc::MemoryCapability::Internal.into())
}

#[cfg(feature = "esp-alloc")]
#[doc(hidden)]
#[no_mangle]
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
#[no_mangle]
pub extern "C" fn esp_wifi_deallocate_internal_ram(ptr: *mut u8) {
    unsafe {
        let ptr = ptr.offset(-4);
        let total_size = *(ptr as *const usize);

        esp_alloc::HEAP.dealloc(
            esp_alloc::MemoryCapability::Internal.into(),
            core::alloc::Layout::from_size_align_unchecked(total_size, 4),
        )
    }
}
