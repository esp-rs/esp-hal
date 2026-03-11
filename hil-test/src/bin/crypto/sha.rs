#[embedded_test::tests(default_timeout = 6)]
mod tests {
    use digest::{Digest, Update};
    #[cfg(rng_trng_supported)]
    use esp_hal::rng::TrngSource;
    #[cfg(not(esp32))]
    use esp_hal::sha::Sha224;
    #[cfg(any(esp32, esp32s2, esp32s3))]
    use esp_hal::sha::{Sha384, Sha512};
    #[cfg(any(esp32s2, esp32s3))]
    use esp_hal::sha::{Sha512_224, Sha512_256};
    use esp_hal::{
        clock::CpuClock,
        rng::Rng,
        sha::{Sha, Sha1, Sha256, ShaAlgorithm, ShaBackend, ShaDigest},
    };
    use nb::block;

    const SOURCE_DATA: &[u8] = &[b'a'; 258];

    pub struct Context {
        #[cfg(rng_trng_supported)]
        _rng_source: TrngSource<'static>,
        sha: Sha<'static>,
    }

    #[track_caller]
    fn assert_sw_hash<D: Digest>(algo: &str, input: &[u8], expected_output: &[u8]) {
        let mut hasher = D::new();
        hasher.update(input);
        let soft_result = hasher.finalize();

        hil_test::assert_eq!(
            expected_output,
            &soft_result[..],
            "Output mismatch with {}",
            algo
        );
    }

    fn hash_sha<S: ShaAlgorithm>(sha: &mut Sha<'static>, mut input: &[u8], output: &mut [u8]) {
        let mut digest = sha.start::<S>();
        while !input.is_empty() {
            input = block!(digest.update(input)).unwrap();
        }
        block!(digest.finish(output)).unwrap();
    }

    fn hash_digest<'a, S: ShaAlgorithm>(
        sha: &'a mut Sha<'static>,
        input: &[u8],
        output: &mut [u8],
    ) {
        let mut hasher = ShaDigest::<S, _>::new(sha);
        Update::update(&mut hasher, input);
        output.copy_from_slice(&digest::FixedOutput::finalize_fixed(hasher));
    }

    /// A simple test using the Sha trait. This will compare the result with a
    /// software implementation.
    #[track_caller]
    fn assert_sha<S: ShaAlgorithm, const N: usize>(sha: &mut Sha<'static>, input: &[u8]) {
        let mut output = [0u8; N];
        hash_sha::<S>(sha, input, &mut output);

        // Compare against Software result.
        match N {
            20 => assert_sw_hash::<sha1::Sha1>("SHA-1", input, &output),
            28 => assert_sw_hash::<sha2::Sha224>("SHA-224", input, &output),
            32 => assert_sw_hash::<sha2::Sha256>("SHA-256", input, &output),
            48 => assert_sw_hash::<sha2::Sha384>("SHA-384", input, &output),
            64 => assert_sw_hash::<sha2::Sha512>("SHA-512", input, &output),
            _ => unreachable!(),
        }
    }

    /// A simple test using the Digest trait. This will compare the result with a
    /// software implementation.
    #[track_caller]
    fn assert_digest<'a, S: ShaAlgorithm, const N: usize>(sha: &'a mut Sha<'static>, input: &[u8]) {
        let mut output = [0u8; N];
        hash_digest::<S>(sha, input, &mut output);

        // Compare against Software result.
        match N {
            20 => assert_sw_hash::<sha1::Sha1>("SHA-1", input, &output),
            28 => assert_sw_hash::<sha2::Sha224>("SHA-224", input, &output),
            32 => assert_sw_hash::<sha2::Sha256>("SHA-256", input, &output),
            48 => assert_sw_hash::<sha2::Sha384>("SHA-384", input, &output),
            64 => assert_sw_hash::<sha2::Sha512>("SHA-512", input, &output),
            _ => unreachable!(),
        }
    }

    #[allow(unused_mut)]
    fn with_random_data(
        mut f: impl FnMut(
            (&[u8], &mut [u8]),
            (&[u8], &mut [u8]),
            (&[u8], &mut [u8]),
            (&[u8], &mut [u8]),
            (&[u8], &mut [u8]),
        ),
    ) {
        // Make sure this is not a multiple of the block size
        const BUFFER_LEN: usize = 264;

        let mut sha1_random = [0u8; BUFFER_LEN];
        let mut sha224_random = [0u8; BUFFER_LEN];
        let mut sha256_random = [0u8; BUFFER_LEN];
        let mut sha384_random = [0u8; BUFFER_LEN];
        let mut sha512_random = [0u8; BUFFER_LEN];

        let rng = Rng::new();

        // Fill source data with random data
        rng.read(&mut sha1_random);
        #[cfg(not(esp32))]
        rng.read(&mut sha224_random);
        rng.read(&mut sha256_random);
        #[cfg(any(esp32, esp32s2, esp32s3))]
        rng.read(&mut sha384_random);
        #[cfg(any(esp32, esp32s2, esp32s3))]
        rng.read(&mut sha512_random);

        for size in [1, 64, 128, 256, BUFFER_LEN] {
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
            assert_sw_hash::<sha1::Sha1>("SHA-1", &sha1_random[..size], &sha1_output);

            #[cfg(not(esp32))]
            assert_sw_hash::<sha2::Sha224>("SHA-224", &sha224_random[..size], &sha224_output);

            assert_sw_hash::<sha2::Sha256>("SHA-256", &sha256_random[..size], &sha256_output);

            #[cfg(any(esp32, esp32s2, esp32s3))]
            assert_sw_hash::<sha2::Sha384>("SHA-384", &sha384_random[..size], &sha384_output);

            #[cfg(any(esp32, esp32s2, esp32s3))]
            assert_sw_hash::<sha2::Sha512>("SHA-512", &sha512_random[..size], &sha512_output);
        }
    }

    #[init]
    fn init() -> Context {
        let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
        let peripherals = esp_hal::init(config);

        Context {
            #[cfg(rng_trng_supported)]
            _rng_source: TrngSource::new(peripherals.RNG, peripherals.ADC1),
            sha: Sha::new(peripherals.SHA),
        }
    }

    #[test]
    #[cfg(any(esp32s2, esp32s3))]
    fn test_sha_512_224(mut ctx: Context) {
        let expected_output = [
            0x19, 0xf2, 0xb3, 0x88, 0x22, 0x86, 0x94, 0x38, 0xee, 0x24, 0xc1, 0xc3, 0xb0, 0xb1,
            0x21, 0x6a, 0xf4, 0x81, 0x14, 0x8f, 0x4, 0x34, 0xfd, 0xd7, 0x54, 0x3, 0x2b, 0x88,
        ];
        let mut output = [0u8; 28];
        hash_sha::<Sha512_224>(&mut ctx.sha, SOURCE_DATA, &mut output);
        assert_eq!(output, expected_output);

        let mut output = [0u8; 28];
        hash_digest::<Sha512_224>(&mut ctx.sha, SOURCE_DATA, &mut output);
        assert_eq!(output, expected_output);
    }

    #[test]
    #[cfg(any(esp32s2, esp32s3))]
    fn test_sha_512_256(mut ctx: Context) {
        let expected_output = [
            0xb7, 0x49, 0x4e, 0xe1, 0xdb, 0xcd, 0xe5, 0x47, 0x5a, 0x61, 0x25, 0xac, 0x27, 0xc2,
            0x1b, 0x53, 0xcd, 0x6b, 0x16, 0x33, 0xb4, 0x94, 0xac, 0xa4, 0x2a, 0xe6, 0x99, 0x2f,
            0xe7, 0xd, 0x83, 0x19,
        ];
        let mut output = [0u8; 32];
        hash_sha::<Sha512_256>(&mut ctx.sha, SOURCE_DATA, &mut output);
        assert_eq!(output, expected_output);

        let mut output = [0u8; 32];
        hash_digest::<Sha512_256>(&mut ctx.sha, SOURCE_DATA, &mut output);
        assert_eq!(output, expected_output);
    }

    /// A test that runs a hashing on a digest of every size between 1 and 200
    /// inclusively.
    #[test]
    #[timeout(15)]
    fn test_digest_of_size_1_to_200(mut ctx: Context) {
        for i in 1..=200 {
            assert_sha::<Sha1, 20>(&mut ctx.sha, &SOURCE_DATA[..i]);
            assert_digest::<Sha1, 20>(&mut ctx.sha, &SOURCE_DATA[..i]);

            #[cfg(not(esp32))]
            {
                assert_sha::<Sha224, 28>(&mut ctx.sha, &SOURCE_DATA[..i]);
                assert_digest::<Sha224, 28>(&mut ctx.sha, &SOURCE_DATA[..i]);
            }

            assert_sha::<Sha256, 32>(&mut ctx.sha, &SOURCE_DATA[..i]);
            assert_digest::<Sha256, 32>(&mut ctx.sha, &SOURCE_DATA[..i]);

            #[cfg(any(esp32, esp32s2, esp32s3))]
            {
                assert_sha::<Sha384, 48>(&mut ctx.sha, &SOURCE_DATA[..i]);
                assert_digest::<Sha384, 48>(&mut ctx.sha, &SOURCE_DATA[..i]);

                assert_sha::<Sha512, 64>(&mut ctx.sha, &SOURCE_DATA[..i]);
                assert_digest::<Sha512, 64>(&mut ctx.sha, &SOURCE_DATA[..i]);
            }
        }
    }

    #[cfg(not(esp32))]
    /// A rolling test that loops between hasher for every step to test
    /// interleaving. This specifically test the Sha trait implementation
    #[test]
    fn test_sha_rolling(mut ctx: Context) {
        #[allow(unused)]
        with_random_data(|sha1_p, sha224_p, sha256_p, sha384_p, sha512_p| {
            let mut sha1_remaining = sha1_p.0;
            let mut sha224_remaining = sha224_p.0;
            let mut sha256_remaining = sha256_p.0;
            #[cfg(any(esp32, esp32s2, esp32s3))]
            let mut sha384_remaining = sha384_p.0;
            #[cfg(any(esp32, esp32s2, esp32s3))]
            let mut sha512_remaining = sha512_p.0;

            let mut sha1 = esp_hal::sha::Context::<esp_hal::sha::Sha1>::new();
            let mut sha224 = esp_hal::sha::Context::<esp_hal::sha::Sha224>::new();
            let mut sha256 = esp_hal::sha::Context::<esp_hal::sha::Sha256>::new();
            #[cfg(any(esp32, esp32s2, esp32s3))]
            let mut sha384 = esp_hal::sha::Context::<esp_hal::sha::Sha384>::new();
            #[cfg(any(esp32, esp32s2, esp32s3))]
            let mut sha512 = esp_hal::sha::Context::<esp_hal::sha::Sha512>::new();

            loop {
                let mut all_done = true;
                if !sha1_remaining.is_empty() {
                    let mut digest = ShaDigest::restore(&mut ctx.sha, &mut sha1);
                    sha1_remaining = block!(digest.update(sha1_remaining)).unwrap();
                    block!(digest.save(&mut sha1));
                    all_done = false;
                }
                if !sha224_remaining.is_empty() {
                    let mut digest = ShaDigest::restore(&mut ctx.sha, &mut sha224);
                    sha224_remaining = block!(digest.update(sha224_remaining)).unwrap();
                    block!(digest.save(&mut sha224));
                    all_done = false;
                }

                if !sha256_remaining.is_empty() {
                    let mut digest = ShaDigest::restore(&mut ctx.sha, &mut sha256);
                    sha256_remaining = block!(digest.update(sha256_remaining)).unwrap();
                    block!(digest.save(&mut sha256));
                    all_done = false;
                }

                #[cfg(any(esp32, esp32s2, esp32s3))]
                {
                    if !sha384_remaining.is_empty() {
                        let mut digest = ShaDigest::restore(&mut ctx.sha, &mut sha384);
                        sha384_remaining = block!(digest.update(sha384_remaining)).unwrap();
                        block!(digest.save(&mut sha384));
                        all_done = false;
                    }

                    if !sha512_remaining.is_empty() {
                        let mut digest = ShaDigest::restore(&mut ctx.sha, &mut sha512);
                        sha512_remaining = block!(digest.update(sha512_remaining)).unwrap();
                        block!(digest.save(&mut sha512));
                        all_done = false;
                    }
                }

                if all_done {
                    break;
                }
            }

            let mut digest = ShaDigest::restore(&mut ctx.sha, &mut sha1);
            block!(digest.finish(sha1_p.1)).unwrap();
            let mut digest = ShaDigest::restore(&mut ctx.sha, &mut sha224);
            block!(digest.finish(sha224_p.1)).unwrap();
            let mut digest = ShaDigest::restore(&mut ctx.sha, &mut sha256);
            block!(digest.finish(sha256_p.1)).unwrap();
            #[cfg(any(esp32, esp32s2, esp32s3))]
            {
                let mut digest = ShaDigest::restore(&mut ctx.sha, &mut sha384);
                block!(digest.finish(sha384_p.1)).unwrap();
                let mut digest = ShaDigest::restore(&mut ctx.sha, &mut sha512);
                block!(digest.finish(sha512_p.1)).unwrap();
            }
        });
    }

    /// A rolling test that loops between hasher for every step to test
    /// interleaving. This specifically test the Digest trait implementation
    #[test]
    fn test_for_digest_rolling(mut ctx: Context) {
        #[allow(unused)]
        with_random_data(|sha1_p, sha224_p, sha256_p, sha384_p, sha512_p| {
            // The Digest::update will consume the entirety of remaining. We don't need to
            // loop until remaining is fully consumed.

            let mut sha1 = ctx.sha.start::<esp_hal::sha::Sha1>();
            Update::update(&mut sha1, sha1_p.0);
            let sha1_output = digest::FixedOutput::finalize_fixed(sha1);
            sha1_p.1.copy_from_slice(&sha1_output);

            #[cfg(not(esp32))]
            {
                let mut sha224 = ctx.sha.start::<esp_hal::sha::Sha224>();
                Update::update(&mut sha224, sha224_p.0);
                let sha224_output = digest::FixedOutput::finalize_fixed(sha224);
                sha224_p.1.copy_from_slice(&sha224_output);
            }

            let mut sha256 = ctx.sha.start::<esp_hal::sha::Sha256>();
            Update::update(&mut sha256, sha256_p.0);
            let sha256_output = digest::FixedOutput::finalize_fixed(sha256);
            sha256_p.1.copy_from_slice(&sha256_output);

            #[cfg(any(esp32, esp32s2, esp32s3))]
            {
                let mut sha384 = ctx.sha.start::<esp_hal::sha::Sha384>();
                Update::update(&mut sha384, sha384_p.0);
                let sha384_output = digest::FixedOutput::finalize_fixed(sha384);
                sha384_p.1.copy_from_slice(&sha384_output);

                let mut sha512 = ctx.sha.start::<esp_hal::sha::Sha512>();
                Update::update(&mut sha512, sha512_p.0);
                let sha512_output = digest::FixedOutput::finalize_fixed(sha512);
                sha512_p.1.copy_from_slice(&sha512_output);
            }
        });
    }

    /// Calling finalize repeatedly will first return the result of the first hashing operation,
    /// then return result for hashing 0 bytes.
    #[test]
    fn test_repeated_finalize_is_empty_hash(_ctx: Context) {
        let mut sha_backend = ShaBackend::new(unsafe { esp_hal::peripherals::SHA::steal() });
        let _sha_driver = sha_backend.start();

        let mut sha1 = esp_hal::sha::Sha1Context::new();
        let sha1 = &mut sha1; // Trick to not pick Digest methods erroneously

        let mut empty_result = [0; 20];
        let mut hash_result = [0; 20];
        let mut repeated_result = [0; 20];

        sha1.finalize(&mut empty_result).wait_blocking();

        sha1.update(&SOURCE_DATA).wait_blocking();
        sha1.finalize(&mut hash_result).wait_blocking();

        sha1.finalize(&mut repeated_result).wait_blocking();

        assert_sw_hash::<sha1::Sha1>("SHA-1", &SOURCE_DATA, &hash_result);
        assert_sw_hash::<sha1::Sha1>("SHA-1", &[], &empty_result);
        hil_test::assert_eq!(empty_result, repeated_result);
    }

    #[test]
    #[cfg(not(esp32))]
    fn test_clone_separates_state(_ctx: Context) {
        use esp_hal::sha::Sha1Context;

        let mut sha_backend = ShaBackend::new(unsafe { esp_hal::peripherals::SHA::steal() });
        let _sha_driver = sha_backend.start();

        let mut sha1 = Sha1Context::new();

        let mut hash_result = [0; 20];
        let mut cloned_result = [0; 20];

        Sha1Context::update(&mut sha1, &SOURCE_DATA).wait_blocking();

        let mut sha1_clone = sha1.clone();

        Sha1Context::finalize(&mut sha1, &mut hash_result).wait_blocking();
        Sha1Context::finalize(&mut sha1_clone, &mut cloned_result).wait_blocking();

        assert_sw_hash::<sha1::Sha1>("SHA-1", &SOURCE_DATA, &hash_result);
        hil_test::assert_eq!(hash_result, cloned_result);
    }

    /// A rolling test that loops between hasher for every step to test
    /// interleaving. This specifically tests the SHA backend implementation
    #[test]
    fn test_for_digest_rolling_context(_ctx: Context) {
        let mut sha_backend = ShaBackend::new(unsafe { esp_hal::peripherals::SHA::steal() });
        let _sha_driver = sha_backend.start();

        #[allow(unused)]
        with_random_data(|sha1_p, sha224_p, sha256_p, sha384_p, sha512_p| {
            {
                let mut sha1 = esp_hal::sha::Sha1Context::new();
                sha1.update(sha1_p.0).wait_blocking();
                sha1.finalize_into_slice(sha1_p.1).unwrap().wait_blocking();
            }

            #[cfg(not(esp32))]
            {
                let mut sha224 = esp_hal::sha::Sha224Context::new();
                sha224.update(sha224_p.0).wait_blocking();
                sha224
                    .finalize_into_slice(sha224_p.1)
                    .unwrap()
                    .wait_blocking();
            }

            {
                let mut sha256 = esp_hal::sha::Sha256Context::new();
                sha256.update(sha256_p.0).wait_blocking();
                sha256
                    .finalize_into_slice(sha256_p.1)
                    .unwrap()
                    .wait_blocking();
            }

            #[cfg(any(esp32, esp32s2, esp32s3))]
            {
                let mut sha384 = esp_hal::sha::Sha384Context::new();
                sha384.update(sha384_p.0).wait_blocking();
                sha384
                    .finalize_into_slice(sha384_p.1)
                    .unwrap()
                    .wait_blocking();
            }

            #[cfg(any(esp32, esp32s2, esp32s3))]
            {
                let mut sha512 = esp_hal::sha::Sha512Context::new();
                sha512.update(sha512_p.0).wait_blocking();
                sha512
                    .finalize_into_slice(sha512_p.1)
                    .unwrap()
                    .wait_blocking();
            }
        });
    }

    /// A rolling test that loops between hasher for every step to test
    /// interleaving. This specifically tests the SHA backend implementation
    #[test]
    fn test_for_digest_rolling_context_interleaved(_ctx: Context) {
        let mut sha_backend = ShaBackend::new(unsafe { esp_hal::peripherals::SHA::steal() });
        let _sha_driver = sha_backend.start();

        let mut sha1 = esp_hal::sha::Sha1Context::new();

        #[cfg(not(esp32))]
        let mut sha224 = esp_hal::sha::Sha224Context::new();

        let mut sha256 = esp_hal::sha::Sha256Context::new();

        #[cfg(any(esp32, esp32s2, esp32s3))]
        let mut sha384 = esp_hal::sha::Sha384Context::new();

        #[cfg(any(esp32, esp32s2, esp32s3))]
        let mut sha512 = esp_hal::sha::Sha512Context::new();

        #[allow(unused)]
        with_random_data(|sha1_p, sha224_p, sha256_p, sha384_p, sha512_p| {
            {
                sha1.update(sha1_p.0).wait_blocking();
                sha1.finalize_into_slice(sha1_p.1).unwrap().wait_blocking();
            }

            #[cfg(not(esp32))]
            {
                sha224.update(sha224_p.0).wait_blocking();
                sha224
                    .finalize_into_slice(sha224_p.1)
                    .unwrap()
                    .wait_blocking();
            }

            {
                sha256.update(sha256_p.0).wait_blocking();
                sha256
                    .finalize_into_slice(sha256_p.1)
                    .unwrap()
                    .wait_blocking();
            }

            #[cfg(any(esp32, esp32s2, esp32s3))]
            {
                sha384.update(sha384_p.0).wait_blocking();
                sha384
                    .finalize_into_slice(sha384_p.1)
                    .unwrap()
                    .wait_blocking();
            }

            #[cfg(any(esp32, esp32s2, esp32s3))]
            {
                sha512.update(sha512_p.0).wait_blocking();
                sha512
                    .finalize_into_slice(sha512_p.1)
                    .unwrap()
                    .wait_blocking();
            }
        });
    }

    /// Test the owned code path (start_owned) to ensure it works with BorrowMut
    #[test]
    fn test_sha_owned(ctx: Context) {
        let mut sha_digest = ctx.sha.start_owned::<Sha256>();

        let mut remaining = SOURCE_DATA;
        while !remaining.is_empty() {
            remaining = block!(sha_digest.update(remaining)).unwrap();
        }

        let mut output = [0u8; 32];
        block!(sha_digest.finish(&mut output)).unwrap();

        // Verify against software implementation
        assert_sw_hash::<sha2::Sha256>("SHA-256", SOURCE_DATA, &output);
    }
}
