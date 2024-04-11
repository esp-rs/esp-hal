//! SHA Test

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use defmt_rtt as _;
use esp_backtrace as _;
use esp_hal::{
    peripherals::Peripherals,
    prelude::*,
    sha::{Sha, ShaMode},
};
use nb::block;

struct Context<'a> {
    sha: Sha<'a, esp_hal::Blocking>,
}

impl Context<'_> {
    pub fn init() -> Self {
        let peripherals = Peripherals::take();
        #[cfg(not(feature = "esp32"))]
        let mut sha = Sha::new(peripherals.SHA, ShaMode::SHA256, None);
        #[cfg(feature = "esp32")]
        let mut sha = Sha::new(peripherals.SHA, ShaMode::SHA256);

        Context { sha }
    }
}

#[cfg(test)]
#[embedded_test::tests]
mod tests {
    use defmt::assert_eq;

    use super::*;

    #[init]
    fn init() -> Context<'static> {
        Context::init()
    }

    #[test]
    fn test_sha_hashing(mut ctx: Context<'static>) {
        let source_data = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa".as_bytes();
        let mut remaining = source_data;
        let expected_output = [
            0x1e, 0xbb, 0xda, 0xb3, 0x35, 0xe0, 0x54, 0x01, 0x5f, 0x0f, 0xc1, 0x7f, 0x62, 0x77,
            0x06, 0x09, 0x72, 0x3d, 0x92, 0xc6, 0x40, 0xb6, 0x5b, 0xa9, 0x97, 0x4d, 0x66, 0x6c,
            0x36, 0x4a, 0x3a, 0x63,
        ];

        // Short hashes can be created by decreasing the output buffer to the desired
        // length
        let mut output = [0u8; 32];

        while remaining.len() > 0 {
            // Can add println to view progress, however println takes a few orders of
            // magnitude longer than the Sha function itself so not useful for
            // comparing processing time println!("Remaining len: {}",
            // remaining.len());

            // All the HW Sha functions are infallible so unwrap is fine to use if you use
            // block!
            remaining = block!(ctx.sha.update(remaining)).unwrap();
        }

        // Finish can be called as many times as desired to get mutliple copies of the
        // output.
        block!(ctx.sha.finish(output.as_mut_slice())).unwrap();

        assert_eq!(output, expected_output);
    }
}
