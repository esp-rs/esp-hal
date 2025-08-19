use esp_rom_sys as _;
use esp_rom_sys::rom::spiflash::*;

use crate::maybe_with_critical_section;

#[inline(never)]
#[cfg_attr(not(target_os = "macos"), unsafe(link_section = ".rwtext"))]
pub(crate) fn spiflash_read(src_addr: u32, data: *const u32, len: u32) -> i32 {
    maybe_with_critical_section(|| unsafe { esp_rom_spiflash_read(src_addr, data, len) })
}

#[inline(never)]
#[cfg_attr(not(target_os = "macos"), unsafe(link_section = ".rwtext"))]
pub(crate) fn spiflash_unlock() -> i32 {
    maybe_with_critical_section(|| unsafe { esp_rom_spiflash_unlock() })
}

#[inline(never)]
#[cfg_attr(not(target_os = "macos"), unsafe(link_section = ".rwtext"))]
pub(crate) fn spiflash_erase_sector(sector_number: u32) -> i32 {
    maybe_with_critical_section(|| unsafe { esp_rom_spiflash_erase_sector(sector_number) })
}

#[inline(never)]
#[cfg_attr(not(target_os = "macos"), unsafe(link_section = ".rwtext"))]
pub(crate) fn spiflash_write(dest_addr: u32, data: *const u32, len: u32) -> i32 {
    maybe_with_critical_section(|| unsafe { esp_rom_spiflash_write(dest_addr, data, len) })
}
