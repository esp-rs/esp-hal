#![no_std]

/// This is needed by `libesp_rom.a` (if used)
#[unsafe(no_mangle)]
unsafe extern "C" fn __assert_func(
    file: *const core::ffi::c_char,
    line: u32,
    func: *const core::ffi::c_char,
    expr: *const core::ffi::c_char,
) {
    unsafe {
        panic!(
            "__assert_func in {}:{} ({}): {}",
            core::ffi::CStr::from_ptr(file).to_str().unwrap(),
            line,
            core::ffi::CStr::from_ptr(func).to_str().unwrap(),
            core::ffi::CStr::from_ptr(expr).to_str().unwrap(),
        );
    }
}
