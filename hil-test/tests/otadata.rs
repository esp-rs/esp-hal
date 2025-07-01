//! Tests parts of esp-bootloader-esp-idf's otadata related functionality not
//! testable on the host

#![no_std]
#![no_main]

use hil_test as _;

esp_bootloader_esp_idf::esp_app_desc!();

#[cfg(test)]
#[embedded_test::tests(default_timeout = 3)]
mod tests {
    #[test]
    fn test_crc_rom_function() {
        let crc = esp_bootloader_esp_idf::Crc32ForTesting::new();
        let res = crc.crc(&[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16]);
        assert_eq!(res, 436745307);
    }
}
