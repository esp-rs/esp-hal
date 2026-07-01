//! esp-storage tests
//!
//! Assumes a certain (i.e. default) partition table layout.
//% CHIP_FILTER: soc_has_flash
//% FEATURES: unstable esp-storage
//% CARGO-CONFIG: target.'cfg(target_arch = "riscv32")'.rustflags = [ "--cfg=__test_esp_storage" ]
//% CARGO-CONFIG: target.'cfg(target_arch = "xtensa")'.rustflags = [ "--cfg=__test_esp_storage" ]

#![no_std]
#![no_main]

use esp_bootloader_esp_idf::EspAppDesc;
use esp_storage::FlashStorage;
use hil_test as _;

#[embedded_test::tests(default_timeout = 3)]
mod tests {
    use super::*;

    // Test we place the app descriptor at the right position in the image and we
    // can read it
    #[test]
    fn test_can_read_app_desc() {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let mut bytes = [0u8; 256];

        let mut flash = FlashStorage::new(peripherals.FLASH);
        #[cfg(multi_core)]
        let mut flash = flash.multicore_auto_park();

        // esp-idf 2nd stage bootloader would expect the app-descriptor at the start of
        // DROM it also expects DROM segment to the the first page of the
        // app-image and we need to account for the image header - so we end up
        // with flash-address 0x10_000 + 0x20
        flash.read(0x10_020, &mut bytes).unwrap();

        assert_eq!(&bytes, unsafe {
            core::mem::transmute::<&EspAppDesc, &[u8; 256]>(&hil_test::ESP_APP_DESC)
        });
    }

    #[test]
    fn test_read_encrypted_same_as_unencrypted_wo_encryption_enabled() {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let mut bytes1 = [0u8; 256];
        let mut bytes2 = [0u8; 256];

        let mut flash = FlashStorage::new(peripherals.FLASH);
        #[cfg(multi_core)]
        let mut flash = flash.multicore_auto_park();

        for offset in (0x10_000..0x20_000).step_by(128) {
            flash.read(offset, &mut bytes1).unwrap();
            flash.read_encrypted(offset, &mut bytes2).unwrap();

            // if encryption is not enabled we should read the same plain text
            assert_eq!(&bytes1, &bytes2);
        }
    }

    #[test]
    fn test_write_encrypted_will_encrypt() {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let mut bytes1 = [0u8; 256];
        let mut bytes2 = [0u8; 256];

        let mut flash = FlashStorage::new(peripherals.FLASH);
        #[cfg(multi_core)]
        let mut flash = flash.multicore_auto_park();

        flash.write_encrypted(0x9000, &[0x0u8; 256]).unwrap();

        flash.read(0x9000, &mut bytes1).unwrap();
        flash.read_encrypted(0x9000, &mut bytes2).unwrap();

        // if encryption is not enabled we should read the same bytes in both cases
        assert_eq!(&bytes1, &bytes2);

        // but encrypted write should do "something" to the data even w/o encryption actually
        // enabled
        assert_ne!(&bytes1, &[0x0u8; 256]);

        // overwrite NVS so the next time the test runs it actually needs to overwrite the data
        flash.write(0x9000, &[0xffu8; 256]).unwrap();
    }
}
