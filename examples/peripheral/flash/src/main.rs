//! Writes and reads internal SPI flash via [`esp_hal::flash`].
//!
//! Assumes the default factory layout (app at `0x1_0000`, NVS at `0x9000`).
//! Write and erase are unsafe NOR operations: there is no auto-erase, and they
//! can overwrite the running application. This example only mutates the first
//! NVS sector. Buffers are word slices in internal RAM.

//% CHIP_FILTER: flash_driver_supported

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    flash::{Config, Flash},
    main,
};
use esp_println::println;

esp_bootloader_esp_idf::esp_app_desc!();

/// Default factory-app image: `0x1_0000` plus the 24-byte image header and
/// 8-byte first section header.
const APP_DESC_OFFSET: u32 = 0x10_020;
/// Default NVS data partition.
const NVS_OFFSET: u32 = 0x9000;

fn as_bytes(words: &[u32]) -> &[u8] {
    // SAFETY: inspecting the in-memory bytes of `u32` words.
    unsafe { core::slice::from_raw_parts(words.as_ptr().cast(), size_of_val(words)) }
}

fn as_bytes_mut(words: &mut [u32]) -> &mut [u8] {
    // SAFETY: mutating the in-memory bytes of `u32` words.
    unsafe { core::slice::from_raw_parts_mut(words.as_mut_ptr().cast(), size_of_val(words)) }
}

#[main]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let mut flash = Flash::new(peripherals.FLASH, Config::default()).unwrap();
    let info = flash.chip_info();
    println!(
        "Flash chip_id = {:#08x}, capacity = {}",
        info.chip_id, info.capacity
    );
    println!("Flash capacity  = {}", flash.capacity());
    println!();
    // The app descriptor sits in the first 256 bytes of an app image, after the
    // 24-byte image header and the 8-byte first section header.
    let mut app_desc = [0u32; 64];
    flash.read(APP_DESC_OFFSET, &mut app_desc).unwrap();
    println!("App descriptor dump {:02x?}", as_bytes(&app_desc));
    println!();

    let mut words = [0u32; 8];
    flash.read(NVS_OFFSET, &mut words).unwrap();
    println!("Read from {:x}:  {:02x?}", NVS_OFFSET, as_bytes(&words));

    {
        let bytes = as_bytes_mut(&mut words);
        bytes[0x00] = bytes[0x00].wrapping_add(1);
        bytes[0x01] = bytes[0x01].wrapping_add(2);
        bytes[0x02] = bytes[0x02].wrapping_add(3);
        bytes[0x03] = bytes[0x03].wrapping_add(4);
        bytes[0x04] = bytes[0x04].wrapping_add(1);
        bytes[0x05] = bytes[0x05].wrapping_add(2);
        bytes[0x06] = bytes[0x06].wrapping_add(3);
        bytes[0x07] = bytes[0x07].wrapping_add(4);
    }

    // NOR flash can only program 1→0; erase the sector first so the new pattern
    // can include 1-bits.
    // SAFETY: NVS is a data partition, not mapped firmware.
    unsafe {
        flash
            .erase(NVS_OFFSET, NVS_OFFSET + Flash::SECTOR_SIZE)
            .unwrap();
        flash.write(NVS_OFFSET, &words).unwrap();
    }
    println!("Written to {:x}: {:02x?}", NVS_OFFSET, as_bytes(&words));

    let mut reread = [0u32; 8];
    flash.read(NVS_OFFSET, &mut reread).unwrap();
    println!("Read from {:x}:  {:02x?}", NVS_OFFSET, as_bytes(&reread));

    println!();
    println!("Reset (CTRL-R in espflash) to re-read the persisted data.");

    loop {}
}
