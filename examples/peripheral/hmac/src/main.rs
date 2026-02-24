//! Demonstrates the two possible ways to use hardware acceleration for HMAC operations and compares
//! the speed of hardware-accelerated and pure software hashing.
//!
//! - Use of the SHA peripheral with a software HMAC implementation
//! - Use of the HMAC peripheral (optional if required setup is skipped)
//!
//! # Writing key
//! Before using the HMAC accelerator in upstream mode, you first need to
//! prepare a secret 256-bit HMAC key and burn the key to an empty eFuse block.
//!
//! ## ⚠️ Before writing ⚠️
//! - From the factory, the eFuse keyblocks are programmed to be 32-byte 0x00.
//! - This example is programmed to use this value so you can skip this step if you don't want to
//!   burn an eFuse key.
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
//! - `BLOCK_KEY0` The keyblock to program the key to. By default this example uses key0.
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

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    hmac::{Hmac, HmacPurpose, KeyId},
    main,
    rng::Rng,
    sha::{Sha256Context, ShaBackend},
    time::Instant,
};
use esp_println::{print, println};
use hmac::{Hmac as HmacSw, Mac, SimpleHmac};
use nb::block;
use sha2::Sha256;

esp_bootloader_esp_idf::esp_app_desc!();

type HmacSha256 = HmacSw<Sha256>;

#[main]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    let mut peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));

    let mut sha_backend = ShaBackend::new(peripherals.SHA.reborrow());

    // Set sw key
    let key = [0_u8; 32].as_ref();

    let mut src = [0_u8; 1024];
    Rng::new().read(src.as_mut_slice());

    let mut output = [0u8; 32];

    println!("Beginning stress tests...");
    println!("Testing length from 0 to {:?} bytes for HMAC...", src.len());
    for i in 0..src.len() + 1 {
        print!("Testing for length: {:>4}", i);
        let (nsrc, _) = src.split_at(i);

        // Use a software algorithm to generate the expected value, and timing baseline
        let mut sw_hmac = HmacSha256::new_from_slice(key).expect("HMAC can take key of any size");
        let pre_sw_hash = Instant::now();
        sw_hmac.update(nsrc);
        let soft_result = sw_hmac.finalize().into_bytes();
        let soft_time = pre_sw_hash.elapsed();
        print!(" SW: {:>7}", soft_time);

        // Use the hardware to see if we can get the same result, and compare the timing
        let mut hw_hmac = Hmac::new(peripherals.HMAC.reborrow());
        hw_hmac.init();
        if block!(hw_hmac.configure(HmacPurpose::ToUser, KeyId::Key0)).is_ok() {
            let pre_hw_hmac = Instant::now();
            let mut remaining = nsrc;
            while !remaining.is_empty() {
                remaining = block!(hw_hmac.update(remaining)).unwrap();
            }
            block!(hw_hmac.finalize(output.as_mut_slice())).unwrap();

            let hw_time = pre_hw_hmac.elapsed();

            assert_eq!(output, soft_result.as_slice());

            print!(", HW: {:>6} ({:>2}x faster)", hw_time, soft_time / hw_time);
        } else {
            print!(", HW HMAC skipped (Key not burned)");
        };
        core::mem::drop(hw_hmac);

        // Now let's try a hybrid approach using the SHA peripheral with a software HMAC
        // implementation
        let _sha_backend = sha_backend.start();

        let mut hyb_hmac = SimpleHmac::<Sha256Context>::new_from_slice(key)
            .expect("HMAC can take key of any size");
        let pre_hyb_hash = Instant::now();
        hyb_hmac.update(nsrc);
        let hybrid_result = hyb_hmac.finalize().into_bytes();
        let hybrid_time = pre_hyb_hash.elapsed();
        print!(
            ", SW+HW: {:>7} ({:>2}x faster)",
            hybrid_time,
            soft_time / hybrid_time
        );
        assert_eq!(hybrid_result, soft_result);

        println!();
    }
    println!("Finished stress tests!");

    loop {}
}
