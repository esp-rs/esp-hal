//! Tests flip_link

//% CHIPS: esp32c6 esp32h2
//% FEATURES: unstable defmt
//% ENV: ESP_HAL_CONFIG_FLIP_LINK = true

#![no_std]
#![no_main]

use hil_test as _;

esp_bootloader_esp_idf::esp_app_desc!();

#[cfg(test)]
#[embedded_test::tests(default_timeout = 3)]
mod tests {
    #[test]
    fn test() {
        let _p = esp_hal::init(Default::default());
        defmt::info!("Hello, world!");
        defmt::assert_eq!(1 + 1, 2);
    }
}
