//! ROM SPI flash wrappers and JEDEC capacity decode.

use procmacros::ram;

use super::{ConfigError, Error};

/// Direct-read chunk limit (ESP-IDF `MAX_READ_CHUNK`), not a ROM hard limit.
pub(super) const ROM_READ_CHUNK: usize = 16384;
/// Direct-write chunk limit (ESP-IDF `MAX_WRITE_CHUNK`), not a ROM hard limit.
pub(super) const ROM_WRITE_CHUNK: usize = 8192;

#[repr(C)]
struct RomSpiflashChip {
    device_id: u32,
}

/// ROM-cached JEDEC id (manufacturer in bits 23:16).
///
/// ESP32 and ESP32-S2 export `g_rom_flashchip` as a BSS symbol. Later chips
/// store the same fields behind `rom_spiflash_legacy_data` (IDF exposes the
/// old name as a macro).
pub(super) fn cached_device_id() -> u32 {
    cfg_select! {
        any(esp32, esp32s2) => {
            unsafe extern "C" {
                static g_rom_flashchip: RomSpiflashChip;
            }
            unsafe { g_rom_flashchip.device_id }
        }
        _ => {
            #[repr(C)]
            struct RomSpiflashLegacyData {
                chip: RomSpiflashChip,
            }
            unsafe extern "C" {
                static rom_spiflash_legacy_data: *const RomSpiflashLegacyData;
            }
            let data = unsafe { rom_spiflash_legacy_data };
            if data.is_null() {
                0
            } else {
                unsafe { (*data).chip.device_id }
            }
        }
    }
}

/// No-response sentinels and the ROM's static W25Q16 placeholder (`0x001540EF`).
///
/// `0xFFFF3F` is esptool's raw `RDID` no-response value; `0x3FFFFF` is that
/// word after the ROM's manufacturer-first swap.
fn is_unusable_id(id: u32) -> bool {
    matches!(
        id,
        0x0000_0000 | 0x00FF_FFFF | 0x00FF_FF3F | 0x003F_FFFF | 0x0015_40EF
    )
}

fn unknown_flash_chip(raw: u32) -> ConfigError {
    warn!("unknown cached flash id {:#010x}", raw);
    ConfigError::UnknownFlashChip
}

/// Decode flash density from a ROM-cached JEDEC id.
///
/// The ROM stores the id manufacturer-first, which is the reverse of a raw
/// `RDID` word. Adesto (`0x1F`) density nibble sits in bits 12:8 in both
/// layouts.
pub(super) fn capacity_from_cached_id(raw: u32) -> Result<usize, ConfigError> {
    let id = raw & 0x00FF_FFFF;
    if is_unusable_id(id) {
        return Err(unknown_flash_chip(raw));
    }

    const KB: u32 = 1024;
    const MB: u32 = 1024 * KB;

    let manufacturer = (id >> 16) as u8;
    let size = if manufacturer == 0x1F {
        // https://github.com/espressif/esptool/blob/8363cae8eca42ec70e26edfe4d1727549d6ce578/esptool/cmds.py#L55-L98
        match ((id >> 8) as u8) & 0x1F {
            0x04 => 512 * KB,
            0x05 => MB,
            0x06 => 2 * MB,
            0x07 => 4 * MB,
            0x08 => 8 * MB,
            0x09 => 16 * MB,
            _ => return Err(unknown_flash_chip(raw)),
        }
    } else {
        match id as u8 {
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
            _ => return Err(unknown_flash_chip(raw)),
        }
    };

    Ok(size as usize)
}

#[ram]
pub(super) fn check_rc(rc: i32) -> Result<(), Error> {
    match rc {
        0 => Ok(()),
        1 => Err(Error::IoError),
        2 => Err(Error::IoTimeout),
        other => {
            warn!("unexpected flash ROM status {}", other);
            Err(Error::Unknown)
        }
    }
}

#[ram]
pub(super) fn spiflash_read(src_addr: u32, data: *const u32, len: u32) -> i32 {
    unsafe { esp_rom_sys::rom::spiflash::esp_rom_spiflash_read(src_addr, data, len) }
}

#[ram]
pub(super) fn spiflash_write(dest_addr: u32, data: *const u32, len: u32) -> i32 {
    unsafe { esp_rom_sys::rom::spiflash::esp_rom_spiflash_write(dest_addr, data, len) }
}

#[ram]
pub(super) fn spiflash_unlock() -> i32 {
    unsafe { esp_rom_sys::rom::spiflash::esp_rom_spiflash_unlock() }
}

#[ram]
pub(super) fn spiflash_erase_sector(sector_number: u32) -> i32 {
    unsafe { esp_rom_sys::rom::spiflash::esp_rom_spiflash_erase_sector(sector_number) }
}

#[ram]
pub(super) fn spiflash_erase_block(block_number: u32) -> i32 {
    unsafe { esp_rom_sys::rom::spiflash::esp_rom_spiflash_erase_block(block_number) }
}
