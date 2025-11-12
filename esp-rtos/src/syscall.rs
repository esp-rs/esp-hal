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

#[cfg(feature = "esp-alloc")]
#[repr(C)]
struct AllocationHeader {
    size: usize,
}

#[cfg(feature = "esp-alloc")]
extern "C" fn _malloc_r(_reent: *mut esp_rom_sys::_reent, size: usize) -> *mut c_void {
    use alloc::alloc::alloc;
    use core::{alloc::Layout, mem, ptr};

    let header_layout = Layout::new::<AllocationHeader>();
    let user_layout = match Layout::from_size_align(size, mem::align_of::<AllocationHeader>()) {
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
        header_ptr.write(AllocationHeader { size });

        let user_data_ptr = p.add(user_offset);

        debug!(
            "Alloc called for {} bytes, returned {:?}",
            size, user_data_ptr
        );
        user_data_ptr as *mut c_void
    }
}

#[cfg(feature = "esp-alloc")]
extern "C" fn _free_r(_reent: *mut esp_rom_sys::_reent, ptr: *mut c_void) {
    use alloc::alloc::dealloc;
    use core::{alloc::Layout, mem};

    if ptr.is_null() {
        return;
    }

    debug!("Freeing {:?}", ptr);
    unsafe {
        let user_ptr = ptr as *mut u8;
        let header_layout = Layout::new::<AllocationHeader>();
        let header_ptr = user_ptr.sub(header_layout.size()) as *mut AllocationHeader;
        let header = header_ptr.read();

        let user_layout =
            Layout::from_size_align_unchecked(header.size, mem::align_of::<AllocationHeader>());
        let (layout, _offset) = header_layout.extend(user_layout).unwrap();

        let p = header_ptr as *mut u8;
        dealloc(p, layout);
    }
}

#[cfg(all(feature = "alloc", not(feature = "esp-alloc")))]
unsafe extern "C" {
    fn malloc_internal(size: usize) -> *mut c_void;
}

#[cfg(all(feature = "alloc", not(feature = "esp-alloc")))]
unsafe extern "C" {
    fn free_internal(ptr: *mut c_void);
}

#[cfg(all(feature = "alloc", not(feature = "esp-alloc")))]
extern "C" fn _malloc_r(_reent: *mut esp_rom_sys::_reent, size: usize) -> *mut c_void {
    unsafe { malloc_internal(size) }
}

#[cfg(all(feature = "alloc", not(feature = "esp-alloc")))]
extern "C" fn _free_r(_reent: *mut esp_rom_sys::_reent, ptr: *mut c_void) {
    unsafe { free_internal(ptr) }
}

pub fn setup_syscalls() {
    #[cfg(feature = "alloc")]
    unsafe {
        SYSCALL_TABLE.__getreent = Some(_getreent);
        SYSCALL_TABLE._malloc_r = Some(_malloc_r);
        SYSCALL_TABLE._free_r = Some(_free_r);
    }
}
