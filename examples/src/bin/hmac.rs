//! Demonstrates the use of the HMAC peripheral and compares the speed of
//! hardware-accelerated and pure software hashing.
//!
//! # Writing key
//! Before using the HMAC accelerator in upstream mode, you first need to
//! prepare a secret 256-bit HMAC key and burn the key to an empty eFuse block.
//!
//! ## ⚠️ Before writing ⚠️
//! - From the factory, the eFuse keyblocks are programmed to be 32-byte 0x00.
//! - This example is programmed to use this value so you can skip this step if
//!   you don't want to burn an eFuse key.
//! - If you skip the skip burning a custom key, you still need to [burn the
//!   purpose](#burn-key-purpose).
//! - [Read more about eFuses](https://docs.espressif.com/projects/esptool/en/latest/esp32c3/espefuse/index.html)
//!
//! ## Burn key purpose
//! You first need to burn the efuse key purpose for the specified key below
//! (Default Key0). Purposes:
//!
//! | Purpose                        | Mode       | Value | Description                                   |
//! |--------------------------------|------------|-------|-----------------------------------------------|
//! | JTAG Re-enable                 | Downstream | 6     | EFUSE_KEY_PURPOSE_HMAC_DOWN_JTAG              |
//! | DS Key Derivation              | Downstream | 7     | EFUSE_KEY_PURPOSE_HMAC_DOWN_DIGITAL_SIGNATURE |
//! | HMAC Calculation               | Upstream   | 8     | EFUSE_KEY_PURPOSE_HMAC_UP                     |
//! | Both JTAG Re-enable and DS KDF | Downstream | 5     | EFUSE_KEY_PURPOSE_HMAC_DOWN_ALL               |
//!
//! To burn the efuse key purpose `HMAC_UP` to `Key0`:
//! ```sh
//! espefuse.py burn_efuse KEY_PURPOSE_0 8
//! ```
//!
//! ## Use a custom key
//! You can generate a custom key file from a string using the following command
//! ```sh
//! echo -n "<Your custom key>" | openssl dgst -sha256 -binary > key.bin
//! ```
//!
//! You can then write your key using the following command
//! - `BLOCK_KEY0` The keyblock to program the key to. By default this example
//!   uses key0.
//! - `HMAC_UP` The purpose for the key. Use HMAC_UP for upstream.
//! - `--no-read-protect` Allow to read the key from software, after writing it.
//! ```sh
//! espefuse.py burn_key BLOCK_KEY0 key.bin HMAC_UP --no-read-protect
//! ```
//! To see the key in bytes, you can do the following:
//! ```sh
//! echo -n "<Your custom key>" | openssl dgst -sha256 -binary | xxd -p
//! ```
//! or from the binary file
//! ```sh
//! xxd -p key.bin
//! ```

//% CHIPS: esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: esp-hal/unstable

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    hmac::{Hmac, HmacPurpose, KeyId},
    main,
    rng::Rng,
};
use esp_println::println;
use hmac::{Hmac as HmacSw, Mac};
use nb::block;
use sha2::Sha256;

esp_bootloader_esp_idf::esp_app_desc!();

type HmacSha256 = HmacSw<Sha256>;

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let mut rng = Rng::new(peripherals.RNG);

    // Set sw key
    let key = [0_u8; 32].as_ref();

    let mut hw_hmac = Hmac::new(peripherals.HMAC);

    let mut src = [0_u8; 1024];
    rng.read(src.as_mut_slice());

    let mut output = [0u8; 32];

    println!("Beginning stress tests...");
    println!("Testing length from 0 to {:?} bytes for HMAC...", src.len());
    for i in 0..src.len() + 1 {
        let (nsrc, _) = src.split_at(i);
        let mut remaining = nsrc;
        hw_hmac.init();
        block!(hw_hmac.configure(HmacPurpose::ToUser, KeyId::Key0)).expect("Key purpose mismatch");
        let pre_hw_hmac = esp_hal::time::Instant::now();
        while remaining.len() > 0 {
            remaining = block!(hw_hmac.update(remaining)).unwrap();
        }
        block!(hw_hmac.finalize(output.as_mut_slice())).unwrap();
        let hw_time = pre_hw_hmac.elapsed();

        let mut sw_hmac = HmacSha256::new_from_slice(key).expect("HMAC can take key of any size");
        let pre_sw_hash = esp_hal::time::Instant::now();
        sw_hmac.update(nsrc);
        let soft_result = sw_hmac.finalize().into_bytes();

        let soft_time = pre_sw_hash.elapsed();
        for (a, b) in output.iter().zip(soft_result) {
            assert_eq!(*a, b);
        }
        println!(
            "Testing for length: {:>4} | HW: {:>6} cycles, SW: {:>7} cycles (HW HMAC is {:>2}x faster)",
            i,
            hw_time,
            soft_time,
            soft_time / hw_time
        );
    }
    println!("Finished stress tests!");

    loop {}
}
