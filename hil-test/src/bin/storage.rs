//! Flash storage tests using `esp_hal::flash`.
//!
//! Assumes a certain (i.e. default) partition table layout.
//% CHIP_FILTER: flash_driver_supported
//% FEATURES: unstable
//% CARGO-CONFIG: target.'cfg(target_arch = "riscv32")'.rustflags = [ "--cfg=__test_flash" ]
//% CARGO-CONFIG: target.'cfg(target_arch = "xtensa")'.rustflags = [ "--cfg=__test_flash" ]

#![no_std]
#![no_main]

use esp_bootloader_esp_idf::EspAppDesc;
use esp_hal::{
    Blocking,
    flash::{Config, Flash},
};
use hil_test as _;

#[embedded_test::tests(default_timeout = 3)]
mod tests {
    use super::*;

    fn flash_from_peripherals(
        peripherals: esp_hal::peripherals::Peripherals,
    ) -> Flash<'static, Blocking> {
        Flash::new(peripherals.FLASH, Config::default()).unwrap()
    }

    // Test we place the app descriptor at the right position in the image and we
    // can read it
    #[test]
    fn test_can_read_app_desc() {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let mut words = [0u32; 64];

        let mut flash = flash_from_peripherals(peripherals);

        // esp-idf 2nd stage bootloader would expect the app-descriptor at the start of
        // DROM it also expects DROM segment to the the first page of the
        // app-image and we need to account for the image header - so we end up
        // with flash-address 0x10_000 + 0x20
        flash.read(0x10_020, &mut words).unwrap();

        assert_eq!(&words, unsafe {
            core::mem::transmute::<&EspAppDesc, &[u32; 64]>(&hil_test::ESP_APP_DESC)
        });
    }

    #[test]
    fn test_read_encrypted_same_as_unencrypted_wo_encryption_enabled() {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let mut words1 = [0u32; 64];
        let mut words2 = [0u32; 64];

        let mut flash = flash_from_peripherals(peripherals);

        for offset in (0x10_000..0x20_000).step_by(128) {
            flash.read(offset, &mut words1).unwrap();
            flash.read_encrypted(offset, &mut words2).unwrap();

            // if encryption is not enabled we should read the same plain text
            assert_eq!(&words1, &words2);
        }
    }

    #[test]
    fn test_write_encrypted_will_encrypt() {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let mut words1 = [0u32; 64];
        let mut words2 = [0u32; 64];

        let mut flash = flash_from_peripherals(peripherals);

        // SAFETY: the NVS partition is not mapped.
        unsafe { flash.erase(0x9000, 0xa000).unwrap() };
        unsafe { flash.write_encrypted(0x9000, &[0u32; 64]).unwrap() };

        flash.read(0x9000, &mut words1).unwrap();
        flash.read_encrypted(0x9000, &mut words2).unwrap();

        // if encryption is not enabled we should read the same bytes in both cases
        assert_eq!(&words1, &words2);

        // but encrypted write should do "something" to the data even w/o encryption actually
        // enabled
        assert_ne!(&words1, &[0u32; 64]);

        // leave NVS erased, the ciphertext is garbage for NVS
        unsafe { flash.erase(0x9000, 0xa000).unwrap() };
    }
}
