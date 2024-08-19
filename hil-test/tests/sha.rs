//! SHA Test

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use esp_hal::{
    peripherals::Peripherals,
    prelude::*,
    sha::{Sha, ShaMode},
};
use hil_test as _;
use nb::block;

#[cfg(test)]
#[embedded_test::tests]
mod tests {
    use defmt::assert_eq;

    use super::*;

    #[init]
    fn init() {}

    #[test]
    fn test_sha_1() {
        let peripherals = Peripherals::take();
        let mut sha = Sha::new(peripherals.SHA, ShaMode::SHA1);

        let source_data = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa".as_bytes();
        let mut remaining = source_data;
        let expected_output = [
            0x57, 0xf5, 0x3e, 0xd5, 0x59, 0x85, 0x24, 0x49, 0x3e, 0xc5, 0x76, 0x77, 0xa, 0xaf,
            0x3b, 0xb1, 0x0, 0x63, 0xe3, 0xce, 0xef, 0x5, 0xf8, 0xe3, 0xfe, 0x3d, 0x96, 0xa4, 0x63,
            0x29, 0xa5, 0x78,
        ];
        let mut output = [0u8; 32];

        while remaining.len() > 0 {
            remaining = block!(sha.update(remaining)).unwrap();
        }
        block!(sha.finish(output.as_mut_slice())).unwrap();

        assert_eq!(expected_output, output);
    }

    #[test]
    #[cfg(not(feature = "esp32"))]
    fn test_sha_224() {
        let peripherals = Peripherals::take();
        let mut sha = Sha::new(peripherals.SHA, ShaMode::SHA224);

        let source_data = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa".as_bytes();
        let mut remaining = source_data;
        let expected_output = [
            0x3b, 0x29, 0x33, 0xca, 0xfa, 0x6, 0xc0, 0x29, 0x68, 0x10, 0xa1, 0x3e, 0x54, 0x5f,
            0x25, 0x40, 0xa4, 0x35, 0x17, 0x3, 0x6d, 0xa2, 0xb, 0xeb, 0x8c, 0xbe, 0x79, 0x3b, 0xb6,
            0xa8, 0x8c, 0xff,
        ];
        let mut output = [0u8; 32];

        while remaining.len() > 0 {
            remaining = block!(sha.update(remaining)).unwrap();
        }
        block!(sha.finish(output.as_mut_slice())).unwrap();

        assert_eq!(expected_output, output);
    }

    #[test]
    fn test_sha_256() {
        let peripherals = Peripherals::take();
        let mut sha = Sha::new(peripherals.SHA, ShaMode::SHA256);

        let source_data = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa".as_bytes();
        let mut remaining = source_data;
        let expected_output = [
            0x1e, 0xbb, 0xda, 0xb3, 0x35, 0xe0, 0x54, 0x01, 0x5f, 0x0f, 0xc1, 0x7f, 0x62, 0x77,
            0x06, 0x09, 0x72, 0x3d, 0x92, 0xc6, 0x40, 0xb6, 0x5b, 0xa9, 0x97, 0x4d, 0x66, 0x6c,
            0x36, 0x4a, 0x3a, 0x63,
        ];
        let mut output = [0u8; 32];

        while remaining.len() > 0 {
            remaining = block!(sha.update(remaining)).unwrap();
        }
        block!(sha.finish(output.as_mut_slice())).unwrap();

        assert_eq!(expected_output, output);
    }

    #[test]
    #[cfg(any(feature = "esp32", feature = "esp32s2", feature = "esp32s3"))]
    fn test_sha_384() {
        let peripherals = Peripherals::take();
        let mut sha = Sha::new(peripherals.SHA, ShaMode::SHA384);

        let source_data = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa".as_bytes();
        let mut remaining = source_data;
        let expected_output = [
            0x8a, 0x1d, 0xe0, 0x7f, 0xa9, 0xc, 0x4c, 0xbb, 0xac, 0xe4, 0x62, 0xbd, 0xd9, 0x2f,
            0x90, 0x88, 0x61, 0x69, 0x40, 0xc0, 0x55, 0x6b, 0x80, 0x6, 0xaa, 0xfc, 0xd4, 0xff,
            0xc1, 0x8, 0xe9, 0xb2,
        ];
        let mut output = [0u8; 32];

        while remaining.len() > 0 {
            remaining = block!(sha.update(remaining)).unwrap();
        }
        block!(sha.finish(output.as_mut_slice())).unwrap();

        assert_eq!(expected_output, output);
    }

    #[test]
    #[cfg(any(feature = "esp32", feature = "esp32s2", feature = "esp32s3"))]
    fn test_sha_512() {
        let peripherals = Peripherals::take();
        let mut sha = Sha::new(peripherals.SHA, ShaMode::SHA512);

        let source_data = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa".as_bytes();
        let mut remaining = source_data;
        let expected_output = [
            0xee, 0x8d, 0xe, 0x15, 0xde, 0xdc, 0xd8, 0xc8, 0x86, 0xa2, 0xef, 0xb1, 0xac, 0x6a,
            0x49, 0xcf, 0xd8, 0x3f, 0x67, 0x65, 0x64, 0xb3, 0x0, 0xce, 0x48, 0x51, 0x5e, 0xce,
            0x5f, 0x4b, 0xee, 0x10,
        ];
        let mut output = [0u8; 32];

        while remaining.len() > 0 {
            remaining = block!(sha.update(remaining)).unwrap();
        }
        block!(sha.finish(output.as_mut_slice())).unwrap();

        assert_eq!(expected_output, output);
    }

    #[test]
    #[cfg(any(feature = "esp32s2", feature = "esp32s3"))]
    fn test_sha_512_224() {
        let peripherals = Peripherals::take();
        let mut sha = Sha::new(peripherals.SHA, ShaMode::SHA512_224);

        let source_data = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa".as_bytes();
        let mut remaining = source_data;
        let expected_output = [
            0x19, 0xf2, 0xb3, 0x88, 0x22, 0x86, 0x94, 0x38, 0xee, 0x24, 0xc1, 0xc3, 0xb0, 0xb1,
            0x21, 0x6a, 0xf4, 0x81, 0x14, 0x8f, 0x4, 0x34, 0xfd, 0xd7, 0x54, 0x3, 0x2b, 0x88, 0xa3,
            0xc1, 0xb8, 0x60,
        ];
        let mut output = [0u8; 32];

        while remaining.len() > 0 {
            remaining = block!(sha.update(remaining)).unwrap();
        }
        block!(sha.finish(output.as_mut_slice())).unwrap();

        assert_eq!(expected_output, output);
    }

    #[test]
    #[cfg(any(feature = "esp32s2", feature = "esp32s3"))]
    fn test_sha_512_256() {
        let peripherals = Peripherals::take();
        let mut sha = Sha::new(peripherals.SHA, ShaMode::SHA512_256);

        let source_data = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa".as_bytes();
        let mut remaining = source_data;
        let expected_output = [
            0xb7, 0x49, 0x4e, 0xe1, 0xdb, 0xcd, 0xe5, 0x47, 0x5a, 0x61, 0x25, 0xac, 0x27, 0xc2,
            0x1b, 0x53, 0xcd, 0x6b, 0x16, 0x33, 0xb4, 0x94, 0xac, 0xa4, 0x2a, 0xe6, 0x99, 0x2f,
            0xe7, 0xd, 0x83, 0x19,
        ];
        let mut output = [0u8; 32];

        while remaining.len() > 0 {
            remaining = block!(sha.update(remaining)).unwrap();
        }
        block!(sha.finish(output.as_mut_slice())).unwrap();

        assert_eq!(expected_output, output);
    }
}
