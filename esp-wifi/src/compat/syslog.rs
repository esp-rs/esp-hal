use core::ffi::VaListImpl;

#[allow(unused)]
pub unsafe extern "C" fn syslog(_priority: u32, _format: *const u8, _args: VaListImpl) {
    #[cfg(feature = "wifi-logs")]
    cfg_if::cfg_if! {
        if #[cfg(any(target_arch = "riscv32", all(target_arch = "xtensa", xtensa_has_vaarg)))]
        {
            extern "C" {
                fn vsnprintf(buffer: *mut u8, len: usize, fmt: *const u8, args: VaListImpl);
            }

            let mut buf = [0u8; 512];
            vsnprintf(&mut buf as *mut u8, 512, _format, _args);
            let res_str = core::ffi::CStr::from_ptr(&buf as *const _ as *const i8);
            info!("{}", res_str.to_str().unwrap());
        }
        else
        {
            let res_str = core::ffi::CStr::from_ptr(_format as *const i8);
            info!("{}", res_str.to_str().unwrap());
        }
    }
}
