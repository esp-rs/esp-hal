//! Encrypted flash access on a device with flash encryption enabled.
//!
//! ATTENTION!
//!
//! Requires burned flash-encryption eFuses, an encryption-enabled second-stage
//! bootloader, and a matching partition table. Do not run this via xtask /
//! devtool.
//!
//! An encryption-enabled bootloader is larger than the default one and does not
//! fit below the usual 0x8000 partition table, so that table has to be moved.
//! This binary assumes the application image starts at 0x20000. 0xf000 is an
//! unused sector in the gap that layout leaves; it is erased as scratch.
//!
//! IF YOU HAVE NO IDEA WHAT ALL THIS MEANS, STOP HERE!!!
//!
//! `Flash::write_encrypted` can only be validated on hardware whose flash
//! encryption eFuses are burned: the ROM encrypts on write either way, but
//! `Flash::read_encrypted` only returns plaintext when the hardware decrypts.
//! HIL runners do not use encrypted boards, so this is a manual test.
//!
//! On ESP32 the ROM programs a 32-byte row of two AES blocks that share an
//! address-derived tweak. A 16-byte write at a 16- but not 32-byte aligned
//! boundary therefore has to decrypt the adjacent block and re-encrypt it to
//! the same ciphertext. Every row size and both boundary alignments are
//! covered below so that bridge is exercised.

//% CHIP_FILTER: flash_driver_supported
//% FEATURES: unstable

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    Blocking,
    flash::{Config, Error, Flash},
    main,
};
use esp_println::println;

esp_bootloader_esp_idf::esp_app_desc!();

/// Unused sector below the application image (see the crate-level docs).
const SCRATCH: u32 = 0xf000;

/// `(offset into the sector, word count)` for every row size the driver
/// selects, at both a 32- and a 16-byte aligned offset.
const CASES: &[(u32, usize)] = &[(0, 4), (0, 8), (0, 12), (16, 4), (16, 8), (16, 12)];

#[main]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init(esp_hal::Config::default());

    assert!(
        esp_hal::efuse::flash_encryption(),
        "this device has no flash encryption key burned"
    );

    let mut flash = Flash::new(peripherals.FLASH, Config::default()).unwrap();
    let sector = flash.chip_info().sector_size;

    let mut plaintext = [0u32; 12];
    for (i, word) in plaintext.iter_mut().enumerate() {
        *word = 0x0102_0304u32.wrapping_mul(i as u32 + 1);
    }

    for &(offset, words) in CASES {
        let data = &plaintext[..words];
        let len = (words * 4) as u32;

        // SAFETY: scratch is below the application image at 0x20000.
        unsafe { flash.erase(SCRATCH, SCRATCH + sector).unwrap() };
        unsafe { flash.write_encrypted(SCRATCH + offset, data).unwrap() };

        let mut plain = [0u32; 12];
        flash
            .read_encrypted(SCRATCH + offset, &mut plain[..words])
            .unwrap();
        assert_eq!(data, &plain[..words], "{len} bytes at {offset} lost data");

        let mut raw = [0u32; 12];
        flash.read(SCRATCH + offset, &mut raw[..words]).unwrap();
        assert_ne!(data, &raw[..words], "{len} bytes at {offset} not encrypted");

        // Re-encrypting a bridged neighbor must reproduce erased flash.
        if offset > 0 {
            assert_erased(&mut flash, SCRATCH, offset);
        }
        assert_erased(&mut flash, SCRATCH + offset + len, 16);

        println!("ok: {len} bytes at offset {offset}");
    }

    assert_eq!(
        unsafe { flash.write_encrypted(SCRATCH + 4, &plaintext[..4]) },
        Err(Error::NotAligned)
    );
    assert_eq!(
        unsafe { flash.write_encrypted(SCRATCH, &plaintext[..3]) },
        Err(Error::NotAligned)
    );

    unsafe { flash.erase(SCRATCH, SCRATCH + sector).unwrap() };

    println!("Test passed");

    loop {}
}

/// Assert that up to 16 bytes of flash read back as erased.
fn assert_erased(flash: &mut Flash<'static, Blocking>, offset: u32, len: u32) {
    let mut buf = [0u32; 4];
    let words = (len / 4) as usize;
    flash.read(offset, &mut buf[..words]).unwrap();
    assert!(
        buf[..words].iter().all(|&word| word == 0xFFFF_FFFF),
        "{offset:#x} is not erased"
    );
}
