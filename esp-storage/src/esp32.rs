use crate::maybe_with_critical_section;

unsafe extern "C" {
    fn esp_rom_spiflash_read(src_addr: u32, data: *const u32, len: u32) -> i32;
    fn esp_rom_spiflash_unlock() -> i32;
    fn esp_rom_spiflash_erase_sector(sector_number: u32) -> i32;
    fn esp_rom_spiflash_write(dest_addr: u32, data: *const u32, len: u32) -> i32;
}

pub(crate) fn spiflash_read(src_addr: u32, data: *const u32, len: u32) -> i32 {
    maybe_with_critical_section(|| unsafe { esp_rom_spiflash_read(src_addr, data, len) })
}

pub(crate) fn spiflash_unlock() -> i32 {
    maybe_with_critical_section(|| unsafe { esp_rom_spiflash_unlock() })
}

pub(crate) fn spiflash_erase_sector(sector_number: u32) -> i32 {
    maybe_with_critical_section(|| unsafe { esp_rom_spiflash_erase_sector(sector_number) })
}

pub(crate) fn spiflash_write(dest_addr: u32, data: *const u32, len: u32) -> i32 {
    maybe_with_critical_section(|| unsafe { esp_rom_spiflash_write(dest_addr, data, len) })
}

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
