//! OTA Update Example
//!
//! This shows the basics of dealing with partitions and changing the active partition.
//! For simplicity it will flash an application image embedded into the binary.
//! In a real world application you can get the image via HTTP(S), UART or from an sd-card etc.
//!
//! Adjust the target and the chip in the following commands according to the chip used!
//!
//! - `cargo xtask build examples examples esp32 --example=gpio_interrupt`
//! - `espflash save-image --chip=esp32 examples/target/xtensa-esp32-none-elf/release/gpio_interrupt examples/target/ota_image`
//! - `cargo xtask build examples examples esp32 --example=ota_update`
//! - `espflash save-image --chip=esp32 examples/target/xtensa-esp32-none-elf/release/ota_update examples/target/ota_image`
//! - erase whole flash via `espflash erase-flash` (this is to make sure otadata is cleared and no code is flashed to any partition)
//! - run via `cargo xtask run example examples esp32 --example=ota_update`
//!
//! On first boot notice the firmware partition gets booted ("Loaded app from partition at offset 0x10000").
//! Press the BOOT button, once finished press the RESET button.
//!
//! Notice OTA0 gets booted ("Loaded app from partition at offset 0x110000").
//!
//! Once again press BOOT, when finished press RESET.
//! You will see the `gpio_interrupt` example gets booted from OTA1 ("Loaded app from partition at offset 0x210000")
//!
//! See https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/ota.html

//% FEATURES: esp-storage esp-hal/unstable
//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use embedded_storage::Storage;
use esp_backtrace as _;
use esp_bootloader_esp_idf::{
    ota::Slot,
    partitions::{self, AppPartitionSubType, DataPartitionSubType},
};
use esp_hal::{
    gpio::{Input, InputConfig, Pull},
    main,
};
use esp_println::println;

esp_bootloader_esp_idf::esp_app_desc!();

static OTA_IMAGE: &[u8] = include_bytes!("../../target/ota_image");

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let mut storage = esp_storage::FlashStorage::new();

    let mut buffer = [0u8; esp_bootloader_esp_idf::partitions::PARTITION_TABLE_MAX_LEN];
    let pt = esp_bootloader_esp_idf::partitions::read_partition_table(&mut storage, &mut buffer)
        .unwrap();

    // List all partitions - this is just FYI
    for i in 0..pt.len() {
        println!("{:?}", pt.get_partition(i));
    }

    // Find the OTA-data partition and show the currently active partition
    let ota_part = pt
        .find_partition(esp_bootloader_esp_idf::partitions::PartitionType::Data(
            DataPartitionSubType::Ota,
        ))
        .unwrap()
        .unwrap();
    let mut ota_part = ota_part.as_embedded_storage(&mut storage);
    println!("Found ota data");

    let mut ota = esp_bootloader_esp_idf::ota::Ota::new(&mut ota_part).unwrap();
    let current = ota.current_slot().unwrap();
    println!(
        "current image state {:?} (only relevant if the bootloader was built with auto-rollback support)",
        ota.current_ota_state()
    );
    println!("current {:?} - next {:?}", current, current.next());

    // Mark the current slot as VALID - this is only needed if the bootloader was built with auto-rollback support.
    // The default pre-compiled bootloader in espflash is NOT.
    if ota.current_slot().unwrap() != Slot::None
        && (ota.current_ota_state().unwrap() == esp_bootloader_esp_idf::ota::OtaImageState::New
            || ota.current_ota_state().unwrap()
                == esp_bootloader_esp_idf::ota::OtaImageState::PendingVerify)
    {
        println!("Changed state to VALID");
        ota.set_current_ota_state(esp_bootloader_esp_idf::ota::OtaImageState::Valid)
            .unwrap();
    }

    cfg_if::cfg_if! {
        if #[cfg(any(feature = "esp32", feature = "esp32s2", feature = "esp32s3"))] {
            let button = peripherals.GPIO0;
        } else {
            let button = peripherals.GPIO9;
        }
    }

    let boot_button = Input::new(button, InputConfig::default().with_pull(Pull::Up));

    println!("Press boot button to flash and switch to the next OTA slot");
    let mut done = false;
    loop {
        if boot_button.is_low() && !done {
            done = true;

            let next_slot = current.next();

            println!("Flashing image to {:?}", next_slot);

            // find the target app partition
            let next_app_partition = match next_slot {
                Slot::None => {
                    // None is FACTORY if present, OTA0 otherwise
                    pt.find_partition(partitions::PartitionType::App(AppPartitionSubType::Factory))
                        .or_else(|_| {
                            pt.find_partition(partitions::PartitionType::App(
                                AppPartitionSubType::Ota0,
                            ))
                        })
                        .unwrap()
                }
                Slot::Slot0 => pt
                    .find_partition(partitions::PartitionType::App(AppPartitionSubType::Ota0))
                    .unwrap(),
                Slot::Slot1 => pt
                    .find_partition(partitions::PartitionType::App(AppPartitionSubType::Ota1))
                    .unwrap(),
            }
            .unwrap();
            println!("Found partition: {:?}", next_app_partition);
            let mut next_app_partition = next_app_partition.as_embedded_storage(&mut storage);

            // write to the app partition
            for (sector, chunk) in OTA_IMAGE.chunks(4096).enumerate() {
                println!("Writing sector {sector}...");
                next_app_partition
                    .write((sector * 4096) as u32, chunk)
                    .unwrap();
            }

            println!("Changing OTA slot and setting the state to NEW");
            let ota_part = pt
                .find_partition(esp_bootloader_esp_idf::partitions::PartitionType::Data(
                    DataPartitionSubType::Ota,
                ))
                .unwrap()
                .unwrap();
            let mut ota_part = ota_part.as_embedded_storage(&mut storage);
            let mut ota = esp_bootloader_esp_idf::ota::Ota::new(&mut ota_part).unwrap();
            ota.set_current_slot(next_slot).unwrap();
            ota.set_current_ota_state(esp_bootloader_esp_idf::ota::OtaImageState::New)
                .unwrap();
        }
    }
}
