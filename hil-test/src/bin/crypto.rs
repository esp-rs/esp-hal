//! Crypto hardware tests

#![no_std]
#![no_main]

// ESP32 has no AES-DMA, no point in setting up PSRAM.
#[rustfmt::skip]
//% CHIP_FILTER(psram):    (aes_driver_supported || ecc_driver_supported || rsa_driver_supported || sha_driver_supported || rng_driver_supported) && dma_can_access_psram
//% CHIP_FILTER(no_psram): (aes_driver_supported || ecc_driver_supported || rsa_driver_supported || sha_driver_supported || rng_driver_supported) && !dma_can_access_psram

//% FEATURES: unstable esp-alloc/nightly

use hil_test as _;

// Macros are only used by some driver modules (e.g. ECC), so the import is unused on chips
// that don't compile those modules.
#[allow(unused_imports)]
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

#[cfg(rng_driver_supported)]
#[path = "crypto/rng.rs"]
mod rng;
