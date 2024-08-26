//! SHA Test

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use digest::Digest;
#[cfg(not(feature = "esp32"))]
use esp_hal::sha::Sha224;
#[cfg(any(feature = "esp32", feature = "esp32s2", feature = "esp32s3"))]
use esp_hal::sha::{Sha384, Sha512};
#[cfg(any(feature = "esp32s2", feature = "esp32s3"))]
use esp_hal::sha::{Sha512_224, Sha512_256};
use esp_hal::{
    peripherals::Peripherals,
    prelude::*,
    rng::Rng,
    sha::{Sha, Sha1, Sha256},
    Blocking,
};
use hil_test as _;
use nb::block;

/// Dummy data used to feed the hasher.
const SOURCE_DATA: &[u8] = &[b'a'; 258];

#[track_caller]
fn assert_sw_hash<D: Digest>(input: &[u8], expected_output: &[u8]) {
    let mut hasher = D::new();
    hasher.update(input);
    let soft_result = hasher.finalize();

    defmt::assert_eq!(expected_output, &soft_result[..]);
}

fn hash_sha<S: Sha<Blocking> + Default>(mut input: &[u8], output: &mut [u8]) {
    let mut hasher = S::default();
    while !input.is_empty() {
        input = block!(Sha::update(&mut hasher, input)).unwrap();
    }
    block!(hasher.finish(output)).unwrap();
}

fn hash_digest<S: Digest>(input: &[u8], output: &mut [u8]) {
    let mut hasher = S::new();
    digest::Digest::update(&mut hasher, input);
    output.copy_from_slice(&digest::Digest::finalize(hasher));
}

/// A simple test using the Sha trait. This will compare the result with a
/// software implementation.
#[track_caller]
fn assert_sha<S: Sha<Blocking> + Default, const N: usize>(input: &[u8]) {
    let mut output = [0u8; N];
    hash_sha::<S>(input, &mut output);

    // Compare against Software result.
    match N {
        20 => assert_sw_hash::<sha1::Sha1>(input, &output),
        28 => assert_sw_hash::<sha2::Sha224>(input, &output),
        32 => assert_sw_hash::<sha2::Sha256>(input, &output),
        48 => assert_sw_hash::<sha2::Sha384>(input, &output),
        64 => assert_sw_hash::<sha2::Sha512>(input, &output),
        _ => unreachable!(),
    }
}

/// A simple test using the Digest trait. This will compare the result with a
/// software implementation.
#[track_caller]
fn assert_digest<D: Digest, const N: usize>(input: &[u8]) {
    let mut output = [0u8; N];
    hash_digest::<D>(input, &mut output);

    // Compare against Software result.
    match N {
        20 => assert_sw_hash::<sha1::Sha1>(input, &output),
        28 => assert_sw_hash::<sha2::Sha224>(input, &output),
        32 => assert_sw_hash::<sha2::Sha256>(input, &output),
        48 => assert_sw_hash::<sha2::Sha384>(input, &output),
        64 => assert_sw_hash::<sha2::Sha512>(input, &output),
        _ => unreachable!(),
    }
}

fn with_random_data(
    mut rng: Rng,
    f: impl Fn(
        (&[u8], &mut [u8]),
        (&[u8], &mut [u8]),
        (&[u8], &mut [u8]),
        (&[u8], &mut [u8]),
        (&[u8], &mut [u8]),
    ),
) {
    const BUFFER_LEN: usize = 256;

    let mut sha1_random = [0u8; BUFFER_LEN];
    #[cfg_attr(feature = "esp32", allow(unused_mut))]
    let mut sha224_random = [0u8; BUFFER_LEN];
    let mut sha256_random = [0u8; BUFFER_LEN];
    let mut sha384_random = [0u8; BUFFER_LEN];
    let mut sha512_random = [0u8; BUFFER_LEN];

    // Fill source data with random data
    rng.read(&mut sha1_random);
    #[cfg(not(feature = "esp32"))]
    {
        rng.read(&mut sha224_random);
    }
    rng.read(&mut sha256_random);
    #[cfg(any(feature = "esp32", feature = "esp32s2", feature = "esp32s3"))]
    {
        rng.read(&mut sha384_random);
        rng.read(&mut sha512_random);
    }

    for size in [1, 64, 128, 256] {
        let mut sha1_output = [0u8; 20];
        let mut sha224_output = [0u8; 28];
        let mut sha256_output = [0u8; 32];
        let mut sha384_output = [0u8; 48];
        let mut sha512_output = [0u8; 64];
        f(
            (&sha1_random[..size], &mut sha1_output[..]),
            (&sha224_random[..size], &mut sha224_output[..]),
            (&sha256_random[..size], &mut sha256_output[..]),
            (&sha384_random[..size], &mut sha384_output[..]),
            (&sha512_random[..size], &mut sha512_output[..]),
        );

        // Calculate software result to compare against
        // Sha1
        assert_sw_hash::<sha1::Sha1>(&sha1_random[..size], &sha1_output);

        // Sha224
        #[cfg(not(feature = "esp32"))]
        {
            assert_sw_hash::<sha2::Sha224>(&sha224_random[..size], &sha224_output);
        }

        // Sha256
        assert_sw_hash::<sha2::Sha256>(&sha256_random[..size], &sha256_output);

        #[cfg(any(feature = "esp32", feature = "esp32s2", feature = "esp32s3"))]
        {
            // Sha384
            assert_sw_hash::<sha2::Sha384>(&sha384_random[..size], &sha384_output);

            // Sha512
            assert_sw_hash::<sha2::Sha512>(&sha512_random[..size], &sha512_output);
        }
    }
}

#[cfg(test)]
#[embedded_test::tests]
mod tests {
    use defmt::assert_eq;

    use super::*;

    #[init]
    fn init() -> Rng {
        cfg_if::cfg_if! {
            if #[cfg(feature = "esp32")] {
                // FIXME: max speed fails...?
                let config = Config::default();
            } else {
                let mut config = Config::default();
                config.cpu_clock = CpuClock::max();
            }
        }

        let (peripherals, _clocks) = esp_hal::init(config);

        Rng::new(peripherals.RNG)
    }

    #[test]
    #[cfg(any(feature = "esp32s2", feature = "esp32s3"))]
    fn test_sha_512_224() {
        let expected_output = [
            0x19, 0xf2, 0xb3, 0x88, 0x22, 0x86, 0x94, 0x38, 0xee, 0x24, 0xc1, 0xc3, 0xb0, 0xb1,
            0x21, 0x6a, 0xf4, 0x81, 0x14, 0x8f, 0x4, 0x34, 0xfd, 0xd7, 0x54, 0x3, 0x2b, 0x88,
        ];
        let mut output = [0u8; 28];
        hash_sha::<Sha512_224<Blocking>>(SOURCE_DATA, &mut output);
        assert_eq!(output, expected_output);

        let mut output = [0u8; 28];
        hash_digest::<Sha512_224<Blocking>>(SOURCE_DATA, &mut output);
        assert_eq!(output, expected_output);
    }

    #[test]
    #[cfg(any(feature = "esp32s2", feature = "esp32s3"))]
    fn test_sha_512_256() {
        let expected_output = [
            0xb7, 0x49, 0x4e, 0xe1, 0xdb, 0xcd, 0xe5, 0x47, 0x5a, 0x61, 0x25, 0xac, 0x27, 0xc2,
            0x1b, 0x53, 0xcd, 0x6b, 0x16, 0x33, 0xb4, 0x94, 0xac, 0xa4, 0x2a, 0xe6, 0x99, 0x2f,
            0xe7, 0xd, 0x83, 0x19,
        ];
        let mut output = [0u8; 32];
        hash_sha::<Sha512_256<Blocking>>(SOURCE_DATA, &mut output);
        assert_eq!(output, expected_output);

        let mut output = [0u8; 32];
        hash_digest::<Sha512_256<Blocking>>(SOURCE_DATA, &mut output);
        assert_eq!(output, expected_output);
    }

    /// A test that runs a hashing on a digest of every size between 1 and 200
    /// inclusively.
    #[test]
    fn test_digest_of_size_1_to_200() {
        for i in 1..=200 {
            assert_sha::<Sha1<Blocking>, 20>(&SOURCE_DATA[..i]);
            assert_digest::<Sha1<Blocking>, 20>(&SOURCE_DATA[..i]);

            #[cfg(not(feature = "esp32"))]
            {
                assert_sha::<Sha224<Blocking>, 28>(&SOURCE_DATA[..i]);
                assert_digest::<Sha224<Blocking>, 28>(&SOURCE_DATA[..i]);
            }

            assert_sha::<Sha256<Blocking>, 32>(&SOURCE_DATA[..i]);
            assert_digest::<Sha256<Blocking>, 32>(&SOURCE_DATA[..i]);

            #[cfg(any(feature = "esp32", feature = "esp32s2", feature = "esp32s3"))]
            {
                assert_sha::<Sha384<Blocking>, 48>(&SOURCE_DATA[..i]);
                assert_digest::<Sha384<Blocking>, 48>(&SOURCE_DATA[..i]);

                assert_sha::<Sha512<Blocking>, 64>(&SOURCE_DATA[..i]);
                assert_digest::<Sha512<Blocking>, 64>(&SOURCE_DATA[..i]);
            }
        }
    }

    /// A rolling test that loops between hasher for every step to test
    /// interleaving. This specifically test the Sha trait implementation
    #[test]
    fn test_sha_rolling(rng: Rng) {
        #[allow(unused)]
        with_random_data(rng, |sha1_p, sha224_p, sha256_p, sha384_p, sha512_p| {
            let mut sha1_remaining = sha1_p.0;
            #[cfg(not(feature = "esp32"))]
            let mut sha224_remaining = sha224_p.0;
            let mut sha256_remaining = sha256_p.0;
            #[cfg(any(feature = "esp32", feature = "esp32s2", feature = "esp32s3"))]
            let mut sha384_remaining = sha384_p.0;
            #[cfg(any(feature = "esp32", feature = "esp32s2", feature = "esp32s3"))]
            let mut sha512_remaining = sha512_p.0;

            let mut sha1 = esp_hal::sha::Sha1::default();
            #[cfg(not(feature = "esp32"))]
            let mut sha224 = esp_hal::sha::Sha224::default();
            let mut sha256 = esp_hal::sha::Sha256::default();
            #[cfg(any(feature = "esp32", feature = "esp32s2", feature = "esp32s3"))]
            let mut sha384 = esp_hal::sha::Sha384::default();
            #[cfg(any(feature = "esp32", feature = "esp32s2", feature = "esp32s3"))]
            let mut sha512 = esp_hal::sha::Sha512::default();

            loop {
                let mut all_done = true;
                if !sha1_remaining.is_empty() {
                    sha1_remaining = block!(Sha::update(&mut sha1, sha1_remaining)).unwrap();
                    all_done = false;
                }
                #[cfg(not(feature = "esp32"))]
                if !sha224_remaining.is_empty() {
                    sha224_remaining = block!(Sha::update(&mut sha224, sha224_remaining)).unwrap();
                    all_done = false;
                }

                if !sha256_remaining.is_empty() {
                    sha256_remaining = block!(Sha::update(&mut sha256, sha256_remaining)).unwrap();
                    all_done = false;
                }

                #[cfg(any(feature = "esp32", feature = "esp32s2", feature = "esp32s3"))]
                {
                    if !sha384_remaining.is_empty() {
                        sha384_remaining =
                            block!(Sha::update(&mut sha384, sha384_remaining)).unwrap();
                        all_done = false;
                    }

                    if !sha512_remaining.is_empty() {
                        sha512_remaining =
                            block!(Sha::update(&mut sha512, sha512_remaining)).unwrap();
                        all_done = false;
                    }
                }

                if all_done {
                    break;
                }
            }

            block!(sha1.finish(sha1_p.1)).unwrap();
            #[cfg(not(feature = "esp32"))]
            {
                block!(sha224.finish(sha224_p.1)).unwrap();
            }
            block!(sha256.finish(sha256_p.1)).unwrap();
            #[cfg(any(feature = "esp32", feature = "esp32s2", feature = "esp32s3"))]
            {
                block!(sha384.finish(sha384_p.1)).unwrap();
                block!(sha512.finish(sha512_p.1)).unwrap();
            }
        });
    }

    /// A rolling test that loops between hasher for every step to test
    /// interleaving. This specifically test the Digest trait implementation
    #[test]
    fn test_for_digest_rolling(rng: Rng) {
        #[allow(unused)]
        with_random_data(rng, |sha1_p, sha224_p, sha256_p, sha384_p, sha512_p| {
            // The Digest::update will consume the entirety of remaining. We don't need to
            // loop until remaining is fully consumed.

            let mut sha1 = esp_hal::sha::Sha1::default();
            Digest::update(&mut sha1, sha1_p.0);
            let sha1_output = Digest::finalize(sha1);
            sha1_p.1.copy_from_slice(&sha1_output);

            #[cfg(not(feature = "esp32"))]
            {
                let mut sha224 = esp_hal::sha::Sha224::default();
                Digest::update(&mut sha224, sha224_p.0);
                let sha224_output = Digest::finalize(sha224);
                sha224_p.1.copy_from_slice(&sha224_output);
            }

            let mut sha256 = esp_hal::sha::Sha256::default();
            Digest::update(&mut sha256, sha256_p.0);
            let sha256_output = Digest::finalize(sha256);
            sha256_p.1.copy_from_slice(&sha256_output);

            #[cfg(any(feature = "esp32", feature = "esp32s2", feature = "esp32s3"))]
            {
                let mut sha384 = esp_hal::sha::Sha384::default();
                Digest::update(&mut sha384, sha384_p.0);
                let sha384_output = Digest::finalize(sha384);
                sha384_p.1.copy_from_slice(&sha384_output);

                let mut sha512 = esp_hal::sha::Sha512::default();
                Digest::update(&mut sha512, sha512_p.0);
                let sha512_output = Digest::finalize(sha512);
                sha512_p.1.copy_from_slice(&sha512_output);
            }
        });
    }
}
