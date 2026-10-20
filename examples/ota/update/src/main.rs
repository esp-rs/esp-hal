//! OTA Update Example
//!
//! This shows the basics of dealing with partitions and changing the active
//! partition. For simplicity it will flash an application image embedded into
//! the binary. In a real world application you can get the image via HTTP(S),
//! UART or from an sd-card etc.
//!
//! Adjust the target and the chip in the following commands according to the
//! chip used!
//!
//! ```ignore,bash
//! cargo xtask build gpio esp32
//! espflash save-image --chip=esp32 target/xtensa-esp32-none-elf/release/gpio_interrupt target/ota_image
//! cargo xtask build ota/update esp32
//! espflash save-image --chip=esp32 target/xtensa-esp32-none-elf/release/ota_update target/ota_image
//! cargo xtask build ota/update esp32
//! espflash save-image --chip=esp32 target/xtensa-esp32-none-elf/release/ota_update target/ota_image
//! espflash erase-flash
//! cargo xtask run ota/update esp32
//! ```
//!
//! On first boot notice the firmware partition gets booted ("Loaded app from
//! partition at offset 0x10000"). Press the BOOT button, once finished press
//! the RESET button.
//!
//! Notice OTA0 gets booted ("Loaded app from partition at offset 0x110000").
//!
//! Once again press BOOT, when finished press RESET.
//! You will see the `gpio_interrupt` example gets booted from OTA1 ("Loaded app
//! from partition at offset 0x210000")
//!
//! See <https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/ota.html>

//% CHIP_FILTER: flash_driver_supported

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    flash::{Config, Flash},
    gpio::{Input, InputConfig, Pull},
    main,
};
use esp_println::println;

esp_bootloader_esp_idf::esp_app_desc!();

static OTA_IMAGE: &[u8] = include_bytes!("../../../../target/ota_image");

const SECTOR_SIZE: usize = 4096;

#[main]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let mut flash = Flash::new(peripherals.FLASH, Config::default()).unwrap();

    let mut buffer = [0u8; esp_bootloader_esp_idf::partitions::PARTITION_TABLE_MAX_LEN];
    let pt =
        esp_bootloader_esp_idf::partitions::read_partition_table(&mut flash, &mut buffer).unwrap();

    // List all partitions - this is just FYI
    for part in pt.iter() {
        println!("{:?}", part);
    }

    println!("Currently booted partition {:?}", pt.booted_partition());

    let mut ota =
        esp_bootloader_esp_idf::ota_updater::OtaUpdater::new(&mut flash, &mut buffer).unwrap();

    let current = if let Ok(current) = ota.selected_partition() {
        current
    } else {
        ota.reset_data().unwrap();
        esp_bootloader_esp_idf::partitions::AppPartitionSubType::Factory
    };

    println!(
        "current image state {:?} (only relevant if the bootloader was built with auto-rollback support)",
        ota.current_ota_state()
    );
    println!("currently selected partition {:?}", current);

    // Mark the current slot as VALID - this is only needed if the bootloader was
    // built with auto-rollback support. The default pre-compiled bootloader in
    // espflash is NOT.
    if let Ok(state) = ota.current_ota_state() {
        if state == esp_bootloader_esp_idf::ota::OtaImageState::New
            || state == esp_bootloader_esp_idf::ota::OtaImageState::PendingVerify
        {
            println!("Changed state to VALID");
            ota.set_current_ota_state(esp_bootloader_esp_idf::ota::OtaImageState::Valid)
                .unwrap();
        }
    }

    let button =
        cfg_select! {
            any(feature = "esp32", feature = "esp32s2", feature = "esp32s3") => peripherals.GPIO0,
            feature = "esp32c5" => peripherals.GPIO28,
            feature = "esp32p4" => peripherals.GPIO35,
            feature = "esp32h4" => peripherals.GPIO34,
            feature = "esp32s31" => peripherals.GPIO61,
            _ => peripherals.GPIO9,
        };

    let boot_button = Input::new(button, InputConfig::default().with_pull(Pull::Up));

    println!("Press boot button to flash and switch to the next OTA slot");
    let mut done = false;
    loop {
        if boot_button.is_low() && !done {
            done = true;

            let (mut next_app_partition, part_type) = ota.next_partition().unwrap();

            println!("Flashing image to {:?}", part_type);

            // Write to the app partition. The region is encrypted if flash
            // encryption is enabled.
            let mut sector_buffer = [0xff; SECTOR_SIZE];
            for (sector, chunk) in OTA_IMAGE.chunks(SECTOR_SIZE).enumerate() {
                println!("Writing sector {sector}...");

                let offset = (sector * SECTOR_SIZE) as u32;
                next_app_partition
                    .erase(offset, offset + SECTOR_SIZE as u32)
                    .unwrap();

                // Encrypted writes must be a multiple of 16 bytes, so pad the
                // last chunk with erased bytes.
                sector_buffer.fill(0xff);
                sector_buffer[..chunk.len()].copy_from_slice(chunk);
                let len = chunk.len().next_multiple_of(16);
                next_app_partition
                    .write(offset, &sector_buffer[..len])
                    .unwrap();
            }

            println!("Changing OTA slot and setting the state to NEW");

            ota.activate_next_partition().unwrap();
            ota.set_current_ota_state(esp_bootloader_esp_idf::ota::OtaImageState::New)
                .unwrap();
        }
    }
}
