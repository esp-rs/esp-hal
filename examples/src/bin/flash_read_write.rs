//! Writes and reads flash memory.
//!
//! See https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/partition-tables.html#built-in-partition-tables

//% FEATURES: esp-storage esp-hal/unstable
//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use embedded_storage::{ReadStorage, Storage};
use esp_backtrace as _;
use esp_bootloader_esp_idf::partitions;
use esp_hal::main;
use esp_println::println;
use esp_storage::FlashStorage;

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    let _ = esp_hal::init(esp_hal::Config::default());

    let mut flash = FlashStorage::new();
    println!("Flash size = {}", flash.capacity());

    let mut pt_mem = [0u8; partitions::PARTITION_TABLE_MAX_LEN];
    let pt = partitions::read_partition_table(&mut flash, &mut pt_mem).unwrap();

    for i in 0..pt.len() {
        let raw = pt.get_partition(i).unwrap();
        println!("{:?}", raw);
    }
    println!();

    // The app descriptor (if present) is contained in the first 256 bytes
    // of an app image, right after the image header (24 bytes) and the first section header (8 bytes)
    let mut app_desc = [0u8; 256];
    pt.find_partition(partitions::PartitionType::App(
        partitions::AppPartitionSubType::Factory,
    ))
    .unwrap()
    .unwrap()
    .as_embedded_storage(&mut flash)
    .read(32, &mut app_desc)
    .unwrap();
    println!("App descriptor dump {:02x?}", app_desc);
    println!();

    let nvs = pt
        .find_partition(partitions::PartitionType::Data(
            partitions::DataPartitionSubType::Nvs,
        ))
        .unwrap()
        .unwrap();
    let mut nvs_partition = nvs.as_embedded_storage(&mut flash);

    let mut bytes = [0u8; 32];
    println!("NVS partition size = {}", nvs_partition.capacity());
    println!();

    nvs_partition.read(0, &mut bytes).unwrap();
    println!("Read from {:x}:  {:02x?}", 0, &bytes[..32]);

    bytes[0x00] = bytes[0x00].wrapping_add(1);
    bytes[0x01] = bytes[0x01].wrapping_add(2);
    bytes[0x02] = bytes[0x02].wrapping_add(3);
    bytes[0x03] = bytes[0x03].wrapping_add(4);
    bytes[0x04] = bytes[0x04].wrapping_add(1);
    bytes[0x05] = bytes[0x05].wrapping_add(2);
    bytes[0x06] = bytes[0x06].wrapping_add(3);
    bytes[0x07] = bytes[0x07].wrapping_add(4);

    nvs_partition.write(0, &bytes).unwrap();
    println!("Written to {:x}: {:02x?}", 0, &bytes[..32]);

    let mut reread_bytes = [0u8; 32];
    nvs_partition.read(0, &mut reread_bytes).unwrap();
    println!("Read from {:x}:  {:02x?}", 0, &reread_bytes[..32]);

    println!();
    println!("Reset (CTRL-R in espflash) to re-read the persisted data.");

    loop {}
}
