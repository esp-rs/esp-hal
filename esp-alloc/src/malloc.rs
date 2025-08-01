#[unsafe(no_mangle)]
pub unsafe extern "C" fn malloc(size: usize) -> *mut u8 {
    let total_size = size + 4;

    unsafe {
        let ptr = crate::HEAP.alloc_caps(
            enumset::EnumSet::empty(),
            core::alloc::Layout::from_size_align_unchecked(total_size, 4),
        );

        if ptr.is_null() {
            return ptr;
        }

        *(ptr as *mut usize) = total_size;
        ptr.offset(4)
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn malloc_internal(size: usize) -> *mut u8 {
    let total_size = size + 4;

    unsafe {
        let ptr = crate::HEAP.alloc_caps(
            crate::MemoryCapability::Internal.into(),
            core::alloc::Layout::from_size_align_unchecked(total_size, 4),
        );

        if ptr.is_null() {
            return ptr;
        }

        *(ptr as *mut usize) = total_size;
        ptr.offset(4)
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn free(ptr: *mut u8) {
    if ptr.is_null() {
        return;
    }

    use core::alloc::GlobalAlloc;

    unsafe {
        let ptr = ptr.offset(-4);
        let total_size = *(ptr as *const usize);

        crate::HEAP.dealloc(
            ptr,
            core::alloc::Layout::from_size_align_unchecked(total_size, 4),
        )
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn calloc(number: u32, size: usize) -> *mut u8 {
    let total_size = number as usize * size;
    unsafe {
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
pub unsafe extern "C" fn calloc_internal(number: u32, size: usize) -> *mut u8 {
    let total_size = number as usize * size;
    unsafe {
        let ptr = malloc_internal(total_size);

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

#[unsafe(no_mangle)]
unsafe extern "C" fn get_free_internal_heap_size() -> usize {
    crate::HEAP.free_caps(crate::MemoryCapability::Internal.into())
}
