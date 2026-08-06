//! esp-bootloader-esp-idf tests
//!
//! Assumes a default partition table layout.
//% CHIP_FILTER: soc_has_flash
//% FEATURES: unstable esp-storage

#![no_std]
#![no_main]

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

        // Independent reference hash of the full partition contents.
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

        assert_eq!(
            nvs.sha256(&mut flash).unwrap(),
            expected.finalize().as_slice()
        );
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

        // Running image must produce a non-zero digest.
        assert!(digest.iter().any(|b| *b != 0));
        assert_eq!(digest, factory.sha256(&mut flash).unwrap());
    }
}
