use crate::maybe_with_critical_section;

crate::rom_fn! {
    fn esp_rom_spiflash_read(src_addr: u32, data: *const u32, len: u32) -> i32 = 0x40000a20;
    fn esp_rom_spiflash_unlock() -> i32 = 0x40000a2c;
    fn esp_rom_spiflash_erase_sector(sector_number: u32) -> i32 = 0x400009fc;
    fn esp_rom_spiflash_write(dest_addr: u32, data: *const u32, len: u32) -> i32 = 0x40000a14;
}

#[inline(always)]
#[unsafe(link_section = ".rwtext")]
pub(crate) fn spiflash_read(src_addr: u32, data: *const u32, len: u32) -> i32 {
    maybe_with_critical_section(|| esp_rom_spiflash_read(src_addr, data, len))
}

#[inline(always)]
#[unsafe(link_section = ".rwtext")]
pub(crate) fn spiflash_unlock() -> i32 {
    maybe_with_critical_section(esp_rom_spiflash_unlock)
}

#[inline(always)]
#[unsafe(link_section = ".rwtext")]
pub(crate) fn spiflash_erase_sector(sector_number: u32) -> i32 {
    maybe_with_critical_section(|| esp_rom_spiflash_erase_sector(sector_number))
}

#[inline(always)]
#[unsafe(link_section = ".rwtext")]
pub(crate) fn spiflash_write(dest_addr: u32, data: *const u32, len: u32) -> i32 {
    maybe_with_critical_section(|| esp_rom_spiflash_write(dest_addr, data, len))
}
