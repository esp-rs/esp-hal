//! Crypto hardware tests

// Only build on chips that expose at least one crypto driver; otherwise every
// module below is `cfg`-ed out and the binary has no test harness (and thus no
// panic handler), which fails to link.
// ESP32 has no AES-DMA, no point in setting up PSRAM
//% CHIP_FILTER(psram):    dma_can_access_psram && (aes_driver_supported || ecc_driver_supported || rsa_driver_supported || sha_driver_supported || rng_driver_supported)
//% CHIP_FILTER(no_psram): !dma_can_access_psram && (aes_driver_supported || ecc_driver_supported || rsa_driver_supported || sha_driver_supported || rng_driver_supported)

//% FEATURES: unstable esp-alloc/nightly

#![no_std]
#![no_main]

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
