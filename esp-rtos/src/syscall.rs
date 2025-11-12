#[cfg(feature = "alloc")]
use core::ffi::c_void;

#[cfg(feature = "alloc")]
use esp_rom_sys::SYSCALL_TABLE;

#[cfg(feature = "alloc")]
extern "C" fn _getreent() -> *mut esp_rom_sys::_reent {
    use allocator_api2::boxed::Box;
    crate::task::with_current_task(|task| {
        task.thread_local
            .reent
            .get_or_insert_with(|| unsafe {
                // Safety: _reent is a C-type containing integers and pointers, it is safe to
                // zero-initialize it.
                Box::new_zeroed().assume_init()
            })
            .as_mut() as *mut _
    })
}

#[cfg(feature = "alloc")]
#[repr(C)]
struct AllocationHeader {
    size: usize,
    align: usize,
}

#[cfg(feature = "alloc")]
extern "C" fn _malloc_r(_reent: *mut esp_rom_sys::_reent, size: usize) -> *mut c_void {
    use core::{mem, ptr};

    use allocator_api2::alloc::{Layout, alloc};
    if size == 0 {
        return ptr::null_mut();
    }

    let user_align = mem::align_of::<usize>();

    let header_layout = Layout::new::<AllocationHeader>();
    let user_layout = match Layout::from_size_align(size, user_align) {
        Ok(l) => l,
        Err(_) => return ptr::null_mut(),
    };

    let (layout, user_offset) = match header_layout.extend(user_layout) {
        Ok(v) => v,
        Err(_) => return ptr::null_mut(),
    };

    unsafe {
        let p = alloc(layout);
        if p.is_null() {
            return ptr::null_mut();
        }

        let header_ptr = p as *mut AllocationHeader;
        header_ptr.write(AllocationHeader {
            size,
            align: user_align,
        });

        let user_data_ptr = p.add(user_offset);

        info!(
            "Alloc called for {} bytes, returned {:?}",
            size, user_data_ptr
        );
        user_data_ptr as *mut c_void
    }
}

#[cfg(feature = "alloc")]
extern "C" fn _free_r(_reent: *mut esp_rom_sys::_reent, ptr: *mut c_void) {
    use allocator_api2::alloc::{Layout, dealloc};

    if ptr.is_null() {
        return;
    }

    unsafe {
        let user_ptr = ptr as *mut u8;
        let header_layout = Layout::new::<AllocationHeader>();
        let header_ptr = user_ptr.sub(header_layout.size()) as *mut AllocationHeader;
        let header = header_ptr.read();

        let user_layout = Layout::from_size_align_unchecked(header.size, header.align);
        let (layout, _offset) = header_layout.extend(user_layout).unwrap();

        let p = header_ptr as *mut u8;
        dealloc(p, layout);
    }
}

pub fn setup_syscalls() {
    #[cfg(feature = "alloc")]
    unsafe {
        SYSCALL_TABLE.__getreent = Some(_getreent);
        SYSCALL_TABLE._malloc_r = Some(_malloc_r);
        SYSCALL_TABLE._free_r = Some(_free_r);
    }
}
