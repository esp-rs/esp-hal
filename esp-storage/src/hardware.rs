use esp_rom_sys as _;
use esp_rom_sys::rom::spiflash::*;
use procmacros::ram;

use crate::maybe_with_critical_section;

#[ram]
pub(crate) fn spiflash_read(src_addr: u32, data: *const u32, len: u32) -> i32 {
    maybe_with_critical_section(|| unsafe { esp_rom_spiflash_read(src_addr, data, len) })
}

#[ram]
pub(crate) fn spiflash_unlock() -> i32 {
    maybe_with_critical_section(|| unsafe { esp_rom_spiflash_unlock() })
}

#[ram]
pub(crate) fn spiflash_erase_sector(sector_number: u32) -> i32 {
    maybe_with_critical_section(|| unsafe { esp_rom_spiflash_erase_sector(sector_number) })
}

#[ram]
pub(crate) fn spiflash_erase_block(block_number: u32) -> i32 {
    maybe_with_critical_section(|| unsafe { esp_rom_spiflash_erase_block(block_number) })
}

#[ram]
pub(crate) fn spiflash_write(dest_addr: u32, data: *const u32, len: u32) -> i32 {
    maybe_with_critical_section(|| unsafe { esp_rom_spiflash_write(dest_addr, data, len) })
}

/// Detect flash size by reading via the dedicated hardware RDID command.
pub(crate) fn get_flash_size() -> u32 {
    // On ESP32 the hardware RDID mechanism does not work reliably, so we
    // read the flash size from the ROM global `g_rom_flashchip` instead.
    #[cfg(esp32)]
    {
        #[repr(C)]
        struct RomSpiflashChip {
            device_id: u32,
            chip_size: u32,
        }
        unsafe extern "C" {
            static g_rom_flashchip: RomSpiflashChip;
        }
        unsafe { g_rom_flashchip.chip_size }
    }

    #[cfg(not(esp32))]
    {
        let id = maybe_with_critical_section(|| {
            let spi1 = esp_hal::peripherals::SPI1::regs();
            spi1.cmd().write(|w| w.flash_rdid().set_bit());
            while spi1.cmd().read().flash_rdid().bit_is_set() {}
            spi1.w(0).read().buf().bits() & 0x00FF_FFFF
        });

        const KB: u32 = 1024;
        const MB: u32 = 1024 * KB;

        // https://github.com/espressif/esptool/blob/8363cae8eca42ec70e26edfe4d1727549d6ce578/esptool/cmds.py#L55-L98
        let [manufacturer, _, _, _] = id.to_le_bytes();
        if manufacturer == 0x1F {
            let [_, capacity, _, _] = id.to_le_bytes();
            match capacity & 0x1F {
                0x04 => 512 * KB,
                0x05 => MB,
                0x06 => 2 * MB,
                0x07 => 4 * MB,
                0x08 => 8 * MB,
                0x09 => 16 * MB,
                _ => 0,
            }
        } else {
            let [_, _, capacity, _] = id.to_le_bytes();
            match capacity {
                0x12 => 256 * KB,
                0x13 => 512 * KB,
                0x14 => MB,
                0x15 => 2 * MB,
                0x16 => 4 * MB,
                0x17 => 8 * MB,
                0x18 => 16 * MB,
                0x19 => 32 * MB,
                0x1A => 64 * MB,
                0x1B => 128 * MB,
                0x1C => 256 * MB,
                0x20 => 64 * MB,
                0x21 => 128 * MB,
                0x22 => 256 * MB,
                0x32 => 256 * KB,
                0x33 => 512 * KB,
                0x34 => MB,
                0x35 => 2 * MB,
                0x36 => 4 * MB,
                0x37 => 8 * MB,
                0x38 => 16 * MB,
                0x39 => 32 * MB,
                0x3A => 64 * MB,
                _ => 0,
            }
        }
    }
}
