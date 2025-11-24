//! Test we place the app descriptor at the right position in the image and we
//! can read it

//% CHIPS: esp32 esp32s2 esp32s3 esp32c2 esp32c3 esp32c6 esp32h2
//% FEATURES: unstable esp-storage

#![no_std]
#![no_main]

use embedded_storage::ReadStorage;
use esp_bootloader_esp_idf::EspAppDesc;
use esp_storage::FlashStorage;
use hil_test as _;

#[embedded_test::tests(default_timeout = 3)]
mod tests {
    use super::*;

    #[test]
    fn test_can_read_app_desc() {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let mut bytes = [0u8; 256];

        let mut flash = FlashStorage::new(peripherals.FLASH);

        // esp-idf 2nd stage bootloader would expect the app-descriptor at the start of
        // DROM it also expects DROM segment to the the first page of the
        // app-image and we need to account for the image header - so we end up
        // with flash-address 0x10_000 + 0x20
        flash.read(0x10_020, &mut bytes).unwrap();

        assert_eq!(&bytes, unsafe {
            core::mem::transmute::<&EspAppDesc, &[u8; 256]>(&hil_test::ESP_APP_DESC)
        });
    }
}
