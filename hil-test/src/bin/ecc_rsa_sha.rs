//! ECC, RSA and SHA Tests

//% CHIPS: esp32 esp32c2 esp32c3 esp32c5 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: unstable

#![no_std]
#![no_main]

use hil_test as _;

#[cfg(ecc_driver_supported)]
#[path = "ecc_rsa_sha/ecc.rs"]
mod ecc;

#[cfg(rsa_driver_supported)]
#[path = "ecc_rsa_sha/rsa.rs"]
mod rsa;

#[cfg(sha_driver_supported)]
#[path = "ecc_rsa_sha/sha.rs"]
mod sha;
