#[cfg(feature = "binary-logs")]
mod binary_logs {
    use log::info;

    extern "C" {
        fn vsnprintf(dst: *mut u8, _n: u32, format: *const u8, ...) -> i32;
    }

    #[no_mangle]
    pub unsafe extern "C" fn phy_printf(_format: *const u8, _args: ...) {
        syslog(_format, _args);
    }

    #[no_mangle]
    pub unsafe extern "C" fn rtc_printf(_format: *const u8, _args: ...) {
        #[cfg(feature = "binary-logs")]
        syslog(_format, _args);
    }

    #[no_mangle]
    pub unsafe extern "C" fn coexist_printf(_format: *const u8, _args: ...) {
        #[cfg(feature = "binary-logs")]
        syslog(_format, _args);
    }

    pub unsafe extern "C" fn syslog(format: *const u8, args: core::ffi::VaListImpl) {
        let mut buf = [0u8; 512];
        vsnprintf(&mut buf as *mut u8, 511, format, args);
        let res_str = core::ffi::CStr::from_ptr(core::ptr::addr_of!(buf).cast())
            .to_str()
            .unwrap();
        info!("{}", res_str);
    }
}

#[cfg(not(feature = "binary-logs"))]
mod dummy {
    #[no_mangle]
    pub unsafe extern "C" fn phy_printf(_format: *const u8, _args: *const ()) {}
    #[no_mangle]
    pub unsafe extern "C" fn rtc_printf(_format: *const u8, _args: *const ()) {}
    #[no_mangle]
    pub unsafe extern "C" fn coexist_printf(_format: *const u8, _args: *const ()) {}
}
