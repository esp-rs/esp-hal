//! Regression test for https://github.com/esp-rs/esp-hal/issues/4094
//!
//! `FlashRegion::erase` takes an exclusive end address, so erasing up to the
//! end of the partition (e.g. the last sector, or the whole partition) must
//! not fail with `Error::OutOfBounds`.
//!
//! Assumes the device is flashed with a partition table containing an NVS
//! partition (which is the case for the default partition table).
//!
//! Also exercises `esp_storage::FlashStorage::erase` on the last sector of the
//! whole flash - an area nothing occupies with the default partition table.
//!
//! NOTE: This test erases the NVS partition and the last sector of the flash!

//% FEATURES: esp-storage

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_bootloader_esp_idf::partitions::{
    DataPartitionSubType,
    Error,
    PARTITION_TABLE_MAX_LEN,
    PartitionType,
    read_partition_table,
};
use esp_hal::main;
use esp_println::println;
use esp_storage::FlashStorageError;

esp_bootloader_esp_idf::esp_app_desc!();

const SECTOR_SIZE: u32 = 4096;

#[main]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let mut flash = esp_storage::FlashStorage::new(peripherals.FLASH);

    let mut buffer = [0u8; PARTITION_TABLE_MAX_LEN];
    let pt = read_partition_table(&mut flash, &mut buffer).unwrap();

    let nvs = pt
        .find_partition(PartitionType::Data(DataPartitionSubType::Nvs))
        .unwrap()
        .expect("No NVS partition found");

    let mut region = nvs.as_flash_region(&mut flash);
    let len = region.partition_size() as u32;
    println!("NVS partition: {} bytes", len);
    assert!(len >= 2 * SECTOR_SIZE);

    let mut sector = [0u8; SECTOR_SIZE as usize];

    // Erasing the whole partition must succeed (#4094)
    region.erase(0, len).unwrap();
    region.read(0, &mut sector).unwrap();
    assert!(sector.iter().all(|&b| b == 0xff));
    region.read(len - SECTOR_SIZE, &mut sector).unwrap();
    assert!(sector.iter().all(|&b| b == 0xff));

    // Erasing only the last sector must succeed and leave the rest untouched
    region
        .write(len - 2 * SECTOR_SIZE, &[0x5a; SECTOR_SIZE as usize])
        .unwrap();
    region
        .write(len - SECTOR_SIZE, &[0x5a; SECTOR_SIZE as usize])
        .unwrap();

    region.erase(len - SECTOR_SIZE, len).unwrap();

    region.read(len - 2 * SECTOR_SIZE, &mut sector).unwrap();
    assert!(sector.iter().all(|&b| b == 0x5a));
    region.read(len - SECTOR_SIZE, &mut sector).unwrap();
    assert!(sector.iter().all(|&b| b == 0xff));

    // Out-of-bounds and reversed ranges must still be rejected
    assert_eq!(region.erase(0, len + SECTOR_SIZE), Err(Error::OutOfBounds));
    assert_eq!(
        region.erase(len, len + SECTOR_SIZE),
        Err(Error::OutOfBounds)
    );
    assert_eq!(region.erase(SECTOR_SIZE, 0), Err(Error::OutOfBounds));

    // The same boundary conditions apply to `FlashStorage` itself: erasing the
    // last sector of the flash must succeed
    let capacity = flash.capacity() as u32;
    flash
        .write(capacity - SECTOR_SIZE, &[0xa5; SECTOR_SIZE as usize])
        .unwrap();
    flash.erase(capacity - SECTOR_SIZE, capacity).unwrap();
    flash.read(capacity - SECTOR_SIZE, &mut sector).unwrap();
    assert!(sector.iter().all(|&b| b == 0xff));

    assert_eq!(
        flash.erase(SECTOR_SIZE, 0),
        Err(FlashStorageError::OutOfBounds)
    );

    println!("Test passed");

    loop {}
}
