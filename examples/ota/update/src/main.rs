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
//! cargo xtask build examples gpio --chip=esp32
//! espflash save-image --chip=esp32 target/xtensa-esp32-none-elf/release/gpio_interrupt examples/target/ota_image
//! cargo xtask build examples update --chip=esp32
//! espflash save-image --chip=esp32 target/xtensa-esp32-none-elf/release/ota_update examples/target/ota_image
//! cargo xtask build examples update --chip=esp32
//! espflash save-image --chip=esp32 target/xtensa-esp32-none-elf/release/ota_update examples/target/ota_image
//! espflash erase-flash
//! cargo xtask run example update --chip=esp32
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

#![no_std]
#![no_main]

use embedded_storage::Storage;
use esp_backtrace as _;
use esp_hal::{
    gpio::{Input, InputConfig, Pull},
    main,
};
use esp_println::println;
use esp_storage::FlashStorage;

esp_bootloader_esp_idf::esp_app_desc!();

static OTA_IMAGE: &[u8] = include_bytes!("../../../target/ota_image");

#[main]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let mut flash = FlashStorage::new(peripherals.FLASH);

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

    let current = ota.selected_partition().unwrap();
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

            ota.with_next_partition(|mut next_app_partition, part_type| {
                println!("Flashing image to {:?}", part_type);

                // write to the app partition
                for (sector, chunk) in OTA_IMAGE.chunks(4096).enumerate() {
                    println!("Writing sector {sector}...");

                    next_app_partition
                        .write((sector * 4096) as u32, chunk)
                        .unwrap();
                }
            })
            .ok();

            println!("Changing OTA slot and setting the state to NEW");

            ota.activate_next_partition().unwrap();
            ota.set_current_ota_state(esp_bootloader_esp_idf::ota::OtaImageState::New)
                .unwrap();
        }
    }
}
