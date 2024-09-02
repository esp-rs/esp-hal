use crate::maybe_with_critical_section;

crate::rom_fn! {
    fn esp_rom_spiflash_read(src_addr: u32, data: *const u32, len: u32) -> i32 = 0x40000130,
    fn esp_rom_spiflash_unlock() -> i32 = 0x40000140,
    fn esp_rom_spiflash_erase_sector(sector_number: u32) -> i32 = 0x40000128,
    fn esp_rom_spiflash_write(dest_addr: u32, data: *const u32, len: u32) -> i32 = 0x4000012c,
}

pub(crate) fn spiflash_read(src_addr: u32, data: *const u32, len: u32) -> i32 {
    maybe_with_critical_section(|| esp_rom_spiflash_read(src_addr, data, len))
}

pub(crate) fn spiflash_unlock() -> i32 {
    maybe_with_critical_section(esp_rom_spiflash_unlock)
}

pub(crate) fn spiflash_erase_sector(sector_number: u32) -> i32 {
    maybe_with_critical_section(|| esp_rom_spiflash_erase_sector(sector_number))
}

pub(crate) fn spiflash_write(dest_addr: u32, data: *const u32, len: u32) -> i32 {
    maybe_with_critical_section(|| esp_rom_spiflash_write(dest_addr, data, len))
}
