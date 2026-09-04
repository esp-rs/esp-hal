//! ROM SPI flash wrappers and JEDEC capacity decode.

use procmacros::ram;

use super::{ConfigError, Error};

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

/// No-response sentinels.
///
/// `0xFFFF3F` is esptool's raw `RDID` no-response value; `0x3FFFFF` is that
/// word after the ROM's manufacturer-first swap.
///
/// `0x001540EF` is a valid Winbond W25Q16 JEDEC id. It is only rejected on
/// ESP32, where the ROM does not run `RDID` and leaves that word as a
/// static placeholder until a second-stage bootloader identifies the chip.
fn is_unusable_id(id: u32) -> bool {
    let no_response = matches!(id, 0x0000_0000 | 0x00FF_FFFF | 0x00FF_FF3F | 0x003F_FFFF);
    cfg_select! {
        esp32 => no_response || id == 0x0015_40EF,
        _ => no_response,
    }
}

#[inline(never)]
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

#[inline(always)]
fn check_rc(rc: i32) -> Result<(), Error> {
    if rc == 0 {
        Ok(())
    } else {
        decode_rom_error(rc)
    }
}

/// Must live in RAM: callers run with the flash cache suspended.
#[ram]
fn decode_rom_error(rc: i32) -> Result<(), Error> {
    match rc {
        1 => Err(Error::IoError),
        2 => Err(Error::IoTimeout),
        other => {
            warn!("unexpected flash ROM status {}", other);
            Err(Error::Unknown)
        }
    }
}

#[inline(always)]
pub(super) fn read(src_addr: u32, data: &mut [u32]) -> Result<(), Error> {
    check_rc(unsafe {
        esp_rom_sys::rom::spiflash::esp_rom_spiflash_read(
            src_addr,
            data.as_mut_ptr(),
            data.len() as u32 * super::WORD_SIZE,
        )
    })
}

#[inline(always)]
pub(super) fn write(dest_addr: u32, data: &[u32]) -> Result<(), Error> {
    check_rc(unsafe {
        esp_rom_sys::rom::spiflash::esp_rom_spiflash_write(
            dest_addr,
            data.as_ptr(),
            data.len() as u32 * super::WORD_SIZE,
        )
    })
}

#[inline(always)]
pub(super) fn unlock() -> Result<(), Error> {
    check_rc(unsafe { esp_rom_sys::rom::spiflash::esp_rom_spiflash_unlock() })
}

#[inline(always)]
pub(super) fn erase_sector(sector_number: u32) -> Result<(), Error> {
    check_rc(unsafe { esp_rom_sys::rom::spiflash::esp_rom_spiflash_erase_sector(sector_number) })
}

#[inline(always)]
pub(super) fn erase_block(block_number: u32) -> Result<(), Error> {
    check_rc(unsafe { esp_rom_sys::rom::spiflash::esp_rom_spiflash_erase_block(block_number) })
}
