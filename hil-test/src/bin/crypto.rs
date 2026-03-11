//! Crypto hardware tests

//% CHIPS(quad): esp32s2
// The S3 dev kit in the HIL-tester has octal PSRAM.
//% CHIPS(octal): esp32s3
// ESP32 has no AES-DMA, no point in setting up PSRAM
// TODO: enable PSRAM for ESP32-C5
//% CHIPS(no_psram): esp32 esp32c2 esp32c3 esp32c5 esp32c6 esp32h2

//% ENV(octal): ESP_HAL_CONFIG_PSRAM_MODE=octal
//% FEATURES(quad, octal): psram
//% FEATURES: unstable esp-alloc/nightly

#![no_std]
#![no_main]

use hil_test as _;

#[macro_use]
extern crate esp_metadata_generated;

#[cfg(aes_driver_supported)]
#[path = "crypto/aes.rs"]
mod aes;

#[cfg(ecc_driver_supported)]
#[path = "crypto/ecc.rs"]
mod ecc;

#[cfg(rsa_driver_supported)]
#[path = "crypto/rsa.rs"]
mod rsa;

#[cfg(sha_driver_supported)]
#[path = "crypto/sha.rs"]
mod sha;
