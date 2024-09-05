use core::ffi::VaListImpl;

#[allow(unused)]
pub unsafe extern "C" fn syslog(_priority: u32, _format: *const u8, _args: VaListImpl) {
    #[cfg(feature = "wifi-logs")]
    cfg_if::cfg_if! {
        if #[cfg(any(target_arch = "riscv32", all(target_arch = "xtensa", xtensa_has_vaarg)))]
        {
            let mut buf = [0u8; 512];
            vsnprintf(&mut buf as *mut u8, 512, _format, _args);
            let res_str = str_from_c(&buf as *const u8);
            info!("{}", res_str);
        }
        else
        {
            let res_str = str_from_c(_format);
            info!("{}", res_str);
        }
    }
}
