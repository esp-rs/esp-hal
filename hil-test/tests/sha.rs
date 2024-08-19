//! SHA Test

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use digest::Digest;
use esp_hal::{
    clock::ClockControl,
    peripherals::Peripherals,
    prelude::*,
    rng::Rng,
    sha::{Sha, Sha1, Sha256},
    system::SystemControl,
};
use hil_test as _;
use nb::block;
use sha1;
use sha2;

/// Dummy data used to feed the hasher.
static CHAR_ARRAY: [u8; 200] = [b'a'; 200];

/// Dummy random data used to feed the Sha1 hasher
static mut SHA1_RANDOM_ARRAY: [u8; 256] = [0u8; 256];

/// Dummy random data used to feed the Sha224 hasher
static mut SHA224_RANDOM_ARRAY: [u8; 256] = [0u8; 256];

/// Dummy random data used to feed the Sha256 hasher
static mut SHA256_RANDOM_ARRAY: [u8; 256] = [0u8; 256];

/// Dummy random data used to feed the Sha384 hasher
#[cfg(any(feature = "esp32", feature = "esp32s2", feature = "esp32s3"))]
static mut SHA384_RANDOM_ARRAY: [u8; 256] = [0u8; 256];

/// Dummy random data used to feed the Sha512 hasher
#[cfg(any(feature = "esp32", feature = "esp32s2", feature = "esp32s3"))]
static mut SHA512_RANDOM_ARRAY: [u8; 256] = [0u8; 256];

#[cfg(test)]
#[embedded_test::tests]
mod tests {
    use defmt::assert_eq;

    use super::*;

    #[init]
    fn init() {}

    #[test]
    fn test_sha_1() {
        let mut sha = Sha1::new();

        let source_data = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa".as_bytes();
        let mut remaining = source_data;
        let expected_output = [
            0x57, 0xf5, 0x3e, 0xd5, 0x59, 0x85, 0x24, 0x49, 0x3e, 0xc5, 0x76, 0x77, 0xa, 0xaf,
            0x3b, 0xb1, 0x0, 0x63, 0xe3, 0xce, 0xef, 0x5, 0xf8, 0xe3, 0xfe, 0x3d, 0x96, 0xa4, 0x63,
            0x29, 0xa5, 0x78,
        ];
        let mut output = [0u8; 32];

        while remaining.len() > 0 {
            remaining = block!(Sha::update(&mut sha, remaining)).unwrap();
        }
        block!(sha.finish(output.as_mut_slice())).unwrap();

        assert_eq!(expected_output, output);
    }

    #[test]
    fn test_sha_1_digest() {
        let mut sha: Sha1<esp_hal::Blocking> = digest::Digest::new();

        let source_data = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa".as_bytes();
        let expected_output = [
            0x57, 0xf5, 0x3e, 0xd5, 0x59, 0x85, 0x24, 0x49, 0x3e, 0xc5, 0x76, 0x77, 0xa, 0xaf,
            0x3b, 0xb1, 0x0, 0x63, 0xe3, 0xce,
        ];
        digest::Digest::update(&mut sha, source_data);
        let output: [u8; 20] = digest::Digest::finalize(sha).into();

        assert_eq!(expected_output, output);
    }

    #[test]
    #[cfg(not(feature = "esp32"))]
    fn test_sha_224() {
        let mut sha = esp_hal::sha::Sha224::new();

        let source_data = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa".as_bytes();
        let mut remaining = source_data;
        let expected_output = [
            0x3b, 0x29, 0x33, 0xca, 0xfa, 0x6, 0xc0, 0x29, 0x68, 0x10, 0xa1, 0x3e, 0x54, 0x5f,
            0x25, 0x40, 0xa4, 0x35, 0x17, 0x3, 0x6d, 0xa2, 0xb, 0xeb, 0x8c, 0xbe, 0x79, 0x3b,
        ];
        let mut output = [0u8; 28];

        while remaining.len() > 0 {
            remaining = block!(Sha::update(&mut sha, remaining)).unwrap();
        }
        block!(sha.finish(output.as_mut_slice())).unwrap();

        assert_eq!(expected_output, output);
    }

    #[test]
    #[cfg(not(feature = "esp32"))]
    fn test_sha_224_digest() {
        let mut sha: esp_hal::sha::Sha224<esp_hal::Blocking> = digest::Digest::new();

        let source_data = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa".as_bytes();
        let expected_output = [
            0x3b, 0x29, 0x33, 0xca, 0xfa, 0x6, 0xc0, 0x29, 0x68, 0x10, 0xa1, 0x3e, 0x54, 0x5f,
            0x25, 0x40, 0xa4, 0x35, 0x17, 0x3, 0x6d, 0xa2, 0xb, 0xeb, 0x8c, 0xbe, 0x79, 0x3b,
        ];

        digest::Digest::update(&mut sha, source_data);
        let output: [u8; 28] = digest::Digest::finalize(sha).into();

        assert_eq!(expected_output, output);
    }

    #[test]
    fn test_sha_256() {
        let mut sha = Sha256::new();

        let source_data = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa".as_bytes();
        let mut remaining = source_data;
        let expected_output = [
            0x1e, 0xbb, 0xda, 0xb3, 0x35, 0xe0, 0x54, 0x01, 0x5f, 0x0f, 0xc1, 0x7f, 0x62, 0x77,
            0x06, 0x09, 0x72, 0x3d, 0x92, 0xc6, 0x40, 0xb6, 0x5b, 0xa9, 0x97, 0x4d, 0x66, 0x6c,
            0x36, 0x4a, 0x3a, 0x63,
        ];
        let mut output = [0u8; 32];

        while remaining.len() > 0 {
            remaining = block!(Sha::update(&mut sha, remaining)).unwrap();
        }
        block!(sha.finish(output.as_mut_slice())).unwrap();

        assert_eq!(expected_output, output);
    }

    #[test]
    fn test_sha_256_digest() {
        let mut sha: Sha256<esp_hal::Blocking> = digest::Digest::new();

        let source_data = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa".as_bytes();
        let expected_output = [
            0x1e, 0xbb, 0xda, 0xb3, 0x35, 0xe0, 0x54, 0x01, 0x5f, 0x0f, 0xc1, 0x7f, 0x62, 0x77,
            0x06, 0x09, 0x72, 0x3d, 0x92, 0xc6, 0x40, 0xb6, 0x5b, 0xa9, 0x97, 0x4d, 0x66, 0x6c,
            0x36, 0x4a, 0x3a, 0x63,
        ];

        digest::Digest::update(&mut sha, source_data);
        let output: [u8; 32] = digest::Digest::finalize(sha).into();

        assert_eq!(expected_output, output);
    }

    #[test]
    #[cfg(any(feature = "esp32", feature = "esp32s2", feature = "esp32s3"))]
    fn test_sha_384() {
        let mut sha = esp_hal::sha::Sha384::new();

        let source_data = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa".as_bytes();
        let mut remaining = source_data;
        let expected_output = [
            0x8a, 0x1d, 0xe0, 0x7f, 0xa9, 0xc, 0x4c, 0xbb, 0xac, 0xe4, 0x62, 0xbd, 0xd9, 0x2f,
            0x90, 0x88, 0x61, 0x69, 0x40, 0xc0, 0x55, 0x6b, 0x80, 0x6, 0xaa, 0xfc, 0xd4, 0xff,
            0xc1, 0x8, 0xe9, 0xb2,
        ];
        let mut output = [0u8; 32];

        while remaining.len() > 0 {
            remaining = block!(Sha::update(&mut sha, remaining)).unwrap();
        }
        block!(sha.finish(output.as_mut_slice())).unwrap();

        assert_eq!(expected_output, output);
    }

    #[test]
    #[cfg(any(feature = "esp32", feature = "esp32s2", feature = "esp32s3"))]
    fn test_sha_384_digest() {
        let mut sha: esp_hal::sha::Sha384<esp_hal::Blocking> = digest::Digest::new();

        let source_data = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa".as_bytes();
        let expected_output = [
            0x8a, 0x1d, 0xe0, 0x7f, 0xa9, 0xc, 0x4c, 0xbb, 0xac, 0xe4, 0x62, 0xbd, 0xd9, 0x2f,
            0x90, 0x88, 0x61, 0x69, 0x40, 0xc0, 0x55, 0x6b, 0x80, 0x6, 0xaa, 0xfc, 0xd4, 0xff,
            0xc1, 0x8, 0xe9, 0xb2, 0xcd, 0xd8, 0xa9, 0x77, 0x36, 0x98, 0x2e, 0x36, 0x3f, 0x69,
            0xa0, 0x7a, 0x20, 0xfa, 0x1c, 0xeb,
        ];

        digest::Digest::update(&mut sha, source_data);
        let output: [u8; 48] = digest::Digest::finalize(sha).into();

        assert_eq!(expected_output, output);
    }

    #[test]
    #[cfg(any(feature = "esp32", feature = "esp32s2", feature = "esp32s3"))]
    fn test_sha_512() {
        let mut sha = esp_hal::sha::Sha512::new();

        let source_data = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa".as_bytes();
        let mut remaining = source_data;
        let expected_output = [
            0xee, 0x8d, 0xe, 0x15, 0xde, 0xdc, 0xd8, 0xc8, 0x86, 0xa2, 0xef, 0xb1, 0xac, 0x6a,
            0x49, 0xcf, 0xd8, 0x3f, 0x67, 0x65, 0x64, 0xb3, 0x0, 0xce, 0x48, 0x51, 0x5e, 0xce,
            0x5f, 0x4b, 0xee, 0x10,
        ];
        let mut output = [0u8; 32];

        while remaining.len() > 0 {
            remaining = block!(Sha::update(&mut sha, remaining)).unwrap();
        }
        block!(sha.finish(output.as_mut_slice())).unwrap();

        assert_eq!(expected_output, output);
    }

    #[test]
    #[cfg(any(feature = "esp32", feature = "esp32s2", feature = "esp32s3"))]
    fn test_sha_512_digest() {
        let mut sha: esp_hal::sha::Sha512<esp_hal::Blocking> = digest::Digest::new();

        let source_data = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa".as_bytes();
        let expected_output = [
            0xee, 0x8d, 0x0e, 0x15, 0xde, 0xdc, 0xd8, 0xc8, 0x86, 0xa2, 0xef, 0xb1, 0xac, 0x6a,
            0x49, 0xcf, 0xd8, 0x3f, 0x67, 0x65, 0x64, 0xb3, 0x00, 0xce, 0x48, 0x51, 0x5e, 0xce,
            0x5f, 0x4b, 0xee, 0x10, 0xe1, 0x1d, 0x89, 0xc2, 0x1c, 0x21, 0x81, 0x53, 0xc3, 0xb2,
            0x31, 0xab, 0x77, 0xca, 0xed, 0xc9, 0x6c, 0x24, 0xd7, 0xe5, 0x9a, 0x94, 0x86, 0x80,
            0xe1, 0x51, 0x00, 0x1a, 0xe1, 0x8c, 0xec, 0x80,
        ];

        digest::Digest::update(&mut sha, source_data);
        let output: [u8; 64] = digest::Digest::finalize(sha).into();

        assert_eq!(expected_output, output);
    }

    #[test]
    #[cfg(any(feature = "esp32s2", feature = "esp32s3"))]
    fn test_sha_512_224() {
        let mut sha = esp_hal::sha::Sha512_224::new();

        let source_data = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa".as_bytes();
        let mut remaining = source_data;
        let expected_output = [
            0x19, 0xf2, 0xb3, 0x88, 0x22, 0x86, 0x94, 0x38, 0xee, 0x24, 0xc1, 0xc3, 0xb0, 0xb1,
            0x21, 0x6a, 0xf4, 0x81, 0x14, 0x8f, 0x4, 0x34, 0xfd, 0xd7, 0x54, 0x3, 0x2b, 0x88, 0xa3,
            0xc1, 0xb8, 0x60,
        ];
        let mut output = [0u8; 32];

        while remaining.len() > 0 {
            remaining = block!(Sha::update(&mut sha, remaining)).unwrap();
        }
        block!(sha.finish(output.as_mut_slice())).unwrap();

        assert_eq!(expected_output, output);
    }

    #[test]
    #[cfg(any(feature = "esp32s2", feature = "esp32s3"))]
    fn test_sha_512_224_digest() {
        let mut sha: esp_hal::sha::Sha512_224<esp_hal::Blocking> = digest::Digest::new();

        let source_data = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa".as_bytes();
        let expected_output = [
            0x19, 0xf2, 0xb3, 0x88, 0x22, 0x86, 0x94, 0x38, 0xee, 0x24, 0xc1, 0xc3, 0xb0, 0xb1,
            0x21, 0x6a, 0xf4, 0x81, 0x14, 0x8f, 0x4, 0x34, 0xfd, 0xd7, 0x54, 0x3, 0x2b, 0x88,
        ];

        digest::Digest::update(&mut sha, source_data);
        let output: [u8; 28] = digest::Digest::finalize(sha).into();

        assert_eq!(expected_output, output);
    }

    #[test]
    #[cfg(any(feature = "esp32s2", feature = "esp32s3"))]
    fn test_sha_512_256() {
        let mut sha = esp_hal::sha::Sha512_256::new();

        let source_data = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa".as_bytes();
        let mut remaining = source_data;
        let expected_output = [
            0xb7, 0x49, 0x4e, 0xe1, 0xdb, 0xcd, 0xe5, 0x47, 0x5a, 0x61, 0x25, 0xac, 0x27, 0xc2,
            0x1b, 0x53, 0xcd, 0x6b, 0x16, 0x33, 0xb4, 0x94, 0xac, 0xa4, 0x2a, 0xe6, 0x99, 0x2f,
            0xe7, 0xd, 0x83, 0x19,
        ];
        let mut output = [0u8; 32];

        while remaining.len() > 0 {
            remaining = block!(Sha::update(&mut sha, remaining)).unwrap();
        }
        block!(sha.finish(output.as_mut_slice())).unwrap();

        assert_eq!(expected_output, output);
    }

    #[test]
    #[cfg(any(feature = "esp32s2", feature = "esp32s3"))]
    fn test_sha_512_256_digest() {
        let mut sha: esp_hal::sha::Sha512_256<esp_hal::Blocking> = digest::Digest::new();

        let source_data = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa".as_bytes();
        let expected_output = [
            0xb7, 0x49, 0x4e, 0xe1, 0xdb, 0xcd, 0xe5, 0x47, 0x5a, 0x61, 0x25, 0xac, 0x27, 0xc2,
            0x1b, 0x53, 0xcd, 0x6b, 0x16, 0x33, 0xb4, 0x94, 0xac, 0xa4, 0x2a, 0xe6, 0x99, 0x2f,
            0xe7, 0xd, 0x83, 0x19,
        ];

        digest::Digest::update(&mut sha, source_data);
        let output: [u8; 32] = digest::Digest::finalize(sha).into();

        assert_eq!(expected_output, output);
    }

    /// A test that runs a hashing on a digest of every size between 1 and 256
    /// inclusively.
    #[test]
    fn test_digest_of_size_1_to_200() {
        for i in 1..=200 {
            test_for_size::<esp_hal::sha::Sha1<esp_hal::Blocking>, 20>(i);
            #[cfg(not(feature = "esp32"))]
            test_for_size::<esp_hal::sha::Sha224<esp_hal::Blocking>, 28>(i);
            test_for_size::<esp_hal::sha::Sha256<esp_hal::Blocking>, 32>(i);
            #[cfg(any(feature = "esp32", feature = "esp32s2", feature = "esp32s3"))]
            test_for_size::<esp_hal::sha::Sha384<esp_hal::Blocking>, 48>(i);
            #[cfg(any(feature = "esp32", feature = "esp32s2", feature = "esp32s3"))]
            test_for_size::<esp_hal::sha::Sha512<esp_hal::Blocking>, 64>(i);
        }
    }

    /// A rolling test that loops between hasher for every step to test
    /// interleaving. This specifically test the Sha trait implementation
    #[test]
    fn test_sha_rolling() {
        let peripherals = Peripherals::take();
        let system = SystemControl::new(peripherals.SYSTEM);
        let _clocks = ClockControl::boot_defaults(system.clock_control).freeze();

        let mut rng = Rng::new(peripherals.RNG);

        // Fill source data with random data
        use core::ptr::addr_of_mut;
        for slice in unsafe {
            [
                addr_of_mut!(SHA1_RANDOM_ARRAY),
                addr_of_mut!(SHA224_RANDOM_ARRAY),
                addr_of_mut!(SHA256_RANDOM_ARRAY),
                #[cfg(any(feature = "esp32", feature = "esp32s2", feature = "esp32s3"))]
                addr_of_mut!(SHA384_RANDOM_ARRAY),
                #[cfg(any(feature = "esp32", feature = "esp32s2", feature = "esp32s3"))]
                addr_of_mut!(SHA512_RANDOM_ARRAY),
            ]
        } {
            rng.read(unsafe { &mut *slice });
        }

        for size in [1, 64, 128, 256] {
            // Use different random data for each hasher.
            let sha1_source_data =
                unsafe { core::slice::from_raw_parts(SHA1_RANDOM_ARRAY.as_ptr(), size) };
            #[cfg(not(feature = "esp32"))]
            let sha224_source_data =
                unsafe { core::slice::from_raw_parts(SHA224_RANDOM_ARRAY.as_ptr(), size) };
            let sha256_source_data =
                unsafe { core::slice::from_raw_parts(SHA256_RANDOM_ARRAY.as_ptr(), size) };
            #[cfg(any(feature = "esp32", feature = "esp32s2", feature = "esp32s3"))]
            let sha384_source_data =
                unsafe { core::slice::from_raw_parts(SHA384_RANDOM_ARRAY.as_ptr(), size) };
            #[cfg(any(feature = "esp32", feature = "esp32s2", feature = "esp32s3"))]
            let sha512_source_data =
                unsafe { core::slice::from_raw_parts(SHA512_RANDOM_ARRAY.as_ptr(), size) };

            let mut sha1_remaining = sha1_source_data;
            #[cfg(not(feature = "esp32"))]
            let mut sha224_remaining = sha224_source_data;
            let mut sha256_remaining = sha256_source_data;
            #[cfg(any(feature = "esp32", feature = "esp32s2", feature = "esp32s3"))]
            let mut sha384_remaining = sha384_source_data;
            #[cfg(any(feature = "esp32", feature = "esp32s2", feature = "esp32s3"))]
            let mut sha512_remaining = sha512_source_data;

            let mut sha1 = esp_hal::sha::Sha1::default();
            #[cfg(not(feature = "esp32"))]
            let mut sha224 = esp_hal::sha::Sha224::default();
            let mut sha256 = esp_hal::sha::Sha256::default();
            #[cfg(any(feature = "esp32", feature = "esp32s2", feature = "esp32s3"))]
            let mut sha384 = esp_hal::sha::Sha384::default();
            #[cfg(any(feature = "esp32", feature = "esp32s2", feature = "esp32s3"))]
            let mut sha512 = esp_hal::sha::Sha512::default();

            // All sources are the same length
            while sha1_remaining.len() > 0 {
                sha1_remaining = block!(Sha::update(&mut sha1, sha1_remaining)).unwrap();
                #[cfg(not(feature = "esp32"))]
                {
                    sha224_remaining = block!(Sha::update(&mut sha224, sha224_remaining)).unwrap();
                }
                sha256_remaining = block!(Sha::update(&mut sha256, sha256_remaining)).unwrap();
                #[cfg(any(feature = "esp32", feature = "esp32s2", feature = "esp32s3"))]
                {
                    sha384_remaining = block!(Sha::update(&mut sha384, sha384_remaining)).unwrap();
                    sha512_remaining = block!(Sha::update(&mut sha512, sha512_remaining)).unwrap();
                }
            }

            let mut sha1_output = [0u8; 20];
            block!(sha1.finish(sha1_output.as_mut_slice())).unwrap();
            #[cfg(not(feature = "esp32"))]
            let mut sha224_output = [0u8; 28];
            #[cfg(not(feature = "esp32"))]
            block!(sha224.finish(sha224_output.as_mut_slice())).unwrap();
            let mut sha256_output = [0u8; 32];
            block!(sha256.finish(sha256_output.as_mut_slice())).unwrap();
            #[cfg(any(feature = "esp32", feature = "esp32s2", feature = "esp32s3"))]
            let mut sha384_output = [0u8; 48];
            #[cfg(any(feature = "esp32", feature = "esp32s2", feature = "esp32s3"))]
            block!(sha384.finish(sha384_output.as_mut_slice())).unwrap();
            #[cfg(any(feature = "esp32", feature = "esp32s2", feature = "esp32s3"))]
            let mut sha512_output = [0u8; 64];
            #[cfg(any(feature = "esp32", feature = "esp32s2", feature = "esp32s3"))]
            block!(sha512.finish(sha512_output.as_mut_slice())).unwrap();

            // Calculate software result to compare against
            // Sha1
            let mut sha1_sw = sha1::Sha1::new();
            sha1_sw.update(sha1_source_data);
            let soft_result = sha1_sw.finalize();
            for (a, b) in sha1_output.iter().zip(soft_result) {
                assert_eq!(*a, b);
            }

            // Sha224
            #[cfg(not(feature = "esp32"))]
            {
                let mut sha224_sw = sha2::Sha224::new();
                sha224_sw.update(sha224_source_data);
                let soft_result = sha224_sw.finalize();
                for (a, b) in sha224_output.iter().zip(soft_result) {
                    assert_eq!(*a, b);
                }
            }

            // Sha256
            let mut sha256_sw = sha2::Sha256::new();
            sha256_sw.update(sha256_source_data);
            let soft_result = sha256_sw.finalize();
            for (a, b) in sha256_output.iter().zip(soft_result) {
                assert_eq!(*a, b);
            }

            // Sha384
            #[cfg(any(feature = "esp32", feature = "esp32s2", feature = "esp32s3"))]
            {
                let mut sha384_sw = sha2::Sha384::new();
                sha384_sw.update(sha384_source_data);
                let soft_result = sha384_sw.finalize();
                for (a, b) in sha384_output.iter().zip(soft_result) {
                    assert_eq!(*a, b);
                }
            }

            // Sha512
            #[cfg(any(feature = "esp32", feature = "esp32s2", feature = "esp32s3"))]
            {
                let mut sha512_sw = sha2::Sha512::new();
                sha512_sw.update(sha512_source_data);
                let soft_result = sha512_sw.finalize();
                for (a, b) in sha512_output.iter().zip(soft_result) {
                    assert_eq!(*a, b);
                }
            }
        }
    }

    /// A rolling test that loops between hasher for every step to test
    /// interleaving. This specifically test the Digest trait implementation
    #[test]
    fn test_for_digest_rolling() {
        let peripherals = Peripherals::take();
        let system = SystemControl::new(peripherals.SYSTEM);
        let _clocks = ClockControl::boot_defaults(system.clock_control).freeze();

        let mut rng = Rng::new(peripherals.RNG);

        // Fill source data with random data
        use core::ptr::addr_of_mut;
        for slice in unsafe {
            [
                addr_of_mut!(SHA1_RANDOM_ARRAY),
                addr_of_mut!(SHA224_RANDOM_ARRAY),
                addr_of_mut!(SHA256_RANDOM_ARRAY),
                #[cfg(any(feature = "esp32", feature = "esp32s2", feature = "esp32s3"))]
                addr_of_mut!(SHA384_RANDOM_ARRAY),
                #[cfg(any(feature = "esp32", feature = "esp32s2", feature = "esp32s3"))]
                addr_of_mut!(SHA512_RANDOM_ARRAY),
            ]
        } {
            rng.read(unsafe { &mut *slice });
        }

        for size in [1, 64, 128, 256] {
            // Use different random data for each hasher.
            let sha1_source_data =
                unsafe { core::slice::from_raw_parts(SHA1_RANDOM_ARRAY.as_ptr(), size) };
            #[cfg(not(feature = "esp32"))]
            let sha224_source_data =
                unsafe { core::slice::from_raw_parts(SHA224_RANDOM_ARRAY.as_ptr(), size) };
            let sha256_source_data =
                unsafe { core::slice::from_raw_parts(SHA256_RANDOM_ARRAY.as_ptr(), size) };
            #[cfg(any(feature = "esp32", feature = "esp32s2", feature = "esp32s3"))]
            let sha384_source_data =
                unsafe { core::slice::from_raw_parts(SHA384_RANDOM_ARRAY.as_ptr(), size) };
            #[cfg(any(feature = "esp32", feature = "esp32s2", feature = "esp32s3"))]
            let sha512_source_data =
                unsafe { core::slice::from_raw_parts(SHA512_RANDOM_ARRAY.as_ptr(), size) };

            let mut sha1 = esp_hal::sha::Sha1::default();
            #[cfg(not(feature = "esp32"))]
            let mut sha224 = esp_hal::sha::Sha224::default();
            let mut sha256 = esp_hal::sha::Sha256::default();
            #[cfg(any(feature = "esp32", feature = "esp32s2", feature = "esp32s3"))]
            let mut sha384 = esp_hal::sha::Sha384::default();
            #[cfg(any(feature = "esp32", feature = "esp32s2", feature = "esp32s3"))]
            let mut sha512 = esp_hal::sha::Sha512::default();

            // The Digest::update will consume the entirety of remaining. We don't need to
            // loop until remaining is fully consumed.
            Digest::update(&mut sha1, sha1_source_data);
            #[cfg(not(feature = "esp32"))]
            Digest::update(&mut sha224, sha224_source_data);
            Digest::update(&mut sha256, sha256_source_data);
            #[cfg(any(feature = "esp32", feature = "esp32s2", feature = "esp32s3"))]
            Digest::update(&mut sha384, sha384_source_data);
            #[cfg(any(feature = "esp32", feature = "esp32s2", feature = "esp32s3"))]
            Digest::update(&mut sha512, sha512_source_data);

            let sha1_output: [u8; 20] = Digest::finalize(sha1).into();
            #[cfg(not(feature = "esp32"))]
            let sha224_output: [u8; 28] = Digest::finalize(sha224).into();
            let sha256_output: [u8; 32] = Digest::finalize(sha256).into();
            #[cfg(any(feature = "esp32", feature = "esp32s2", feature = "esp32s3"))]
            let sha384_output: [u8; 48] = Digest::finalize(sha384).into();
            #[cfg(any(feature = "esp32", feature = "esp32s2", feature = "esp32s3"))]
            let sha512_output: [u8; 64] = Digest::finalize(sha512).into();

            // Calculate software result to compare against
            // Sha1
            let mut sha1_sw = sha1::Sha1::new();
            sha1_sw.update(sha1_source_data);
            let soft_result = sha1_sw.finalize();
            for (a, b) in sha1_output.iter().zip(soft_result) {
                assert_eq!(*a, b);
            }

            // Sha224
            #[cfg(not(feature = "esp32"))]
            {
                let mut sha224_sw = sha2::Sha224::new();
                sha224_sw.update(sha224_source_data);
                let soft_result = sha224_sw.finalize();
                for (a, b) in sha224_output.iter().zip(soft_result) {
                    assert_eq!(*a, b);
                }
            }

            // Sha256
            let mut sha256_sw = sha2::Sha256::new();
            sha256_sw.update(sha256_source_data);
            let soft_result = sha256_sw.finalize();
            for (a, b) in sha256_output.iter().zip(soft_result) {
                assert_eq!(*a, b);
            }

            // Sha384
            #[cfg(any(feature = "esp32", feature = "esp32s2", feature = "esp32s3"))]
            {
                let mut sha384_sw = sha2::Sha384::new();
                sha384_sw.update(sha384_source_data);
                let soft_result = sha384_sw.finalize();
                for (a, b) in sha384_output.iter().zip(soft_result) {
                    assert_eq!(*a, b);
                }
            }

            // Sha512
            #[cfg(any(feature = "esp32", feature = "esp32s2", feature = "esp32s3"))]
            {
                let mut sha512_sw = sha2::Sha512::new();
                sha512_sw.update(sha512_source_data);
                let soft_result = sha512_sw.finalize();
                for (a, b) in sha512_output.iter().zip(soft_result) {
                    assert_eq!(*a, b);
                }
            }
        }
    }
}

/// A simple test using [esp_hal::sha::Sha] trait to test hashing for an
/// algorithm against a specific size. This will compare the result with a
/// software implementation and return false if there's a mismatch
fn test_for_size<D: Digest + Default + Sha<esp_hal::Blocking>, const N: usize>(size: usize) {
    let source_data = unsafe { core::slice::from_raw_parts(CHAR_ARRAY.as_ptr(), size) };
    let mut remaining = source_data;
    let mut hasher = D::default();

    let mut output = [0u8; N];

    while remaining.len() > 0 {
        remaining = block!(Sha::update(&mut hasher, &remaining)).unwrap();
    }

    block!(hasher.finish(output.as_mut_slice())).unwrap();

    // Compare against Software result.
    match N {
        20 => {
            let mut hasher = sha1::Sha1::new();
            hasher.update(source_data);
            let soft_result = hasher.finalize();
            for (a, b) in output.iter().zip(soft_result) {
                assert_eq!(*a, b);
            }
        }
        28 => {
            let mut hasher = sha2::Sha224::new();
            hasher.update(source_data);
            let soft_result = hasher.finalize();
            for (a, b) in output.iter().zip(soft_result) {
                assert_eq!(*a, b);
            }
        }
        32 => {
            let mut hasher = sha2::Sha256::new();
            hasher.update(source_data);
            let soft_result = hasher.finalize();
            for (a, b) in output.iter().zip(soft_result) {
                assert_eq!(*a, b);
            }
        }
        48 => {
            let mut hasher = sha2::Sha384::new();
            hasher.update(source_data);
            let soft_result = hasher.finalize();
            for (a, b) in output.iter().zip(soft_result) {
                assert_eq!(*a, b);
            }
        }
        64 => {
            let mut hasher = sha2::Sha512::new();
            hasher.update(source_data);
            let soft_result = hasher.finalize();
            for (a, b) in output.iter().zip(soft_result) {
                assert_eq!(*a, b);
            }
        }
        _ => unreachable!(),
    };
}
