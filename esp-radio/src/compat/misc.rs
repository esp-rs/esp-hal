use crate::compat::malloc::malloc;

// these are not called but needed for linking
#[unsafe(no_mangle)]
unsafe extern "C" fn __esp_radio_fwrite(
    ptr: *const (),
    size: usize,
    count: usize,
    stream: *const (),
) -> usize {
    todo!("fwrite {:?} {} {} {:?}", ptr, size, count, stream)
}

#[unsafe(no_mangle)]
unsafe extern "C" fn __esp_radio_fopen(filename: *const u8, mode: *const u8) -> *const () {
    todo!("fopen {:?} {:?}", filename, mode)
}

#[unsafe(no_mangle)]
unsafe extern "C" fn __esp_radio_fgets(str: *const u8, count: u32, file: *const ()) -> *const u8 {
    todo!("fgets {:?} {} {:?}", str, count, file)
}

#[unsafe(no_mangle)]
unsafe extern "C" fn __esp_radio_fclose(stream: *const ()) -> i32 {
    todo!("fclose {:?}", stream);
}

// We cannot just use the ROM function since it needs to allocate memory
#[unsafe(no_mangle)]
unsafe extern "C" fn __esp_radio_strdup(str: *const core::ffi::c_char) -> *const core::ffi::c_char {
    trace!("strdup {:?}", str);

    unsafe {
        let s = core::ffi::CStr::from_ptr(str);
        let len = s.count_bytes() + 1;
        let p = malloc(len);
        if !p.is_null() {
            core::ptr::copy_nonoverlapping(str, p.cast(), len);
        }
        p.cast()
    }
}
