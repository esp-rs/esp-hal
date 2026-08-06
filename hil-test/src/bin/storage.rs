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
    use esp_bootloader_esp_idf::partitions::{
        self,
        AppPartitionSubType,
        DataPartitionSubType,
        PartitionType,
    };
    use sha2::{Digest, Sha256};

    use super::*;

    fn flash_from_peripherals(
        peripherals: esp_hal::peripherals::Peripherals,
    ) -> FlashStorage<'static> {
        let flash = FlashStorage::new(peripherals.FLASH);
        #[cfg(multi_core)]
        let flash = flash.multicore_auto_park();
        flash
    }

    // Test we place the app descriptor at the right position in the image and we
    // can read it
    #[test]
    fn test_can_read_app_desc() {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let mut bytes = [0u8; 256];

        let mut flash = flash_from_peripherals(peripherals);

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

        let mut flash = flash_from_peripherals(peripherals);

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

        let mut flash = flash_from_peripherals(peripherals);

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

    #[test]
    fn test_sha256_of_data_partition() {
        let peripherals = esp_hal::init(esp_hal::Config::default());
        let mut flash = flash_from_peripherals(peripherals);

        let mut pt_mem = [0u8; partitions::PARTITION_TABLE_MAX_LEN];
        let pt = partitions::read_partition_table(&mut flash, &mut pt_mem).unwrap();
        let nvs = pt
            .find_partition(PartitionType::Data(DataPartitionSubType::Nvs))
            .unwrap()
            .unwrap();

        // Independent reference hash of the full partition contents
        let mut expected = Sha256::new();
        {
            let mut region = nvs.as_flash_region(&mut flash);
            let mut chunk = [0u8; 512];
            let mut offset = 0u32;
            while offset < nvs.len() {
                let n = ((nvs.len() - offset) as usize).min(chunk.len());
                region.read(offset, &mut chunk[..n]).unwrap();
                expected.update(&chunk[..n]);
                offset += n as u32;
            }
        }

        assert_eq!(nvs.sha256(&mut flash).unwrap(), expected.finalize().as_slice());
    }

    #[test]
    fn test_sha256_of_app_partition() {
        let peripherals = esp_hal::init(esp_hal::Config::default());
        let mut flash = flash_from_peripherals(peripherals);

        let mut pt_mem = [0u8; partitions::PARTITION_TABLE_MAX_LEN];
        let pt = partitions::read_partition_table(&mut flash, &mut pt_mem).unwrap();
        let factory = pt
            .find_partition(PartitionType::App(AppPartitionSubType::Factory))
            .unwrap()
            .unwrap();

        let digest = factory.sha256(&mut flash).unwrap();

        // Running image must produce a non-zero digest
        assert!(digest.iter().any(|b| *b != 0));
        assert_eq!(digest, factory.sha256(&mut flash).unwrap());
    }
}
