//! Demonstrates the use of the SHA peripheral and compares the speed of
//! hardware-accelerated and pure software hashing.
//!
//! This also includes a series of testing to ensure the proper functioning of the peripheral
//!
//! For simplicity purpose, every hasher is declared using it's full path, to help differenciate
//! between Hardware acceleration and software. Hashers starting with `esp_hal` such as
//! [esp_hal::sha::Sha256] use the hardware accelerated peripheral, and starting with `sha2` such
//! as [sha2::Sha256] use the software implementation.

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl, entry, peripherals::Peripherals, prelude::*, sha::Sha,
    system::SystemControl,
};
use esp_println::println;
use nb::block;
use sha2::Digest;

/// Dummy data used to feed the hasher.
static CHAR_ARRAY: [u8; 4096] = [b'a'; 4096];

#[entry]
fn main() -> ! {
    esp_println::logger::init_logger(log::LevelFilter::Warn);
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let _clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    for i in 1..512 {
        println!("Testing with {} chars", i);

        if !rolling_with_digest_trait(i) {
            println!("Rolling digest {} Failed", i);
            // panic!("failed");
        }

        if !test_for_size::<esp_hal::sha::Sha1<esp_hal::Blocking>, 20>(i) {
            println!("SHA1 {} Failed", i);
            // panic!("failed");
        }
        #[cfg(not(feature = "esp32"))]
        if !test_for_size::<esp_hal::sha::Sha224<esp_hal::Blocking>, 28>(i) {
            println!("SHA224 {} Failed", i);
            // panic!("failed");
        }
        if !test_for_size::<esp_hal::sha::Sha256<esp_hal::Blocking>, 32>(i) {
            println!("SHA256 {} Failed", i);
            // panic!("failed");
        }

        // Notes: SHA512 and SHA384 are broken for now for certain lengths.
        if !test_for_size::<esp_hal::sha::Sha384<esp_hal::Blocking>, 48>(i) {
            println!("SHA384 {} Failed", i);
            // panic!("failed");
        }
        if !test_for_size::<esp_hal::sha::Sha512<esp_hal::Blocking>, 64>(i) {
            log::error!("SHA512 {} Failed", i);
            // panic!("failed");
        }
    }

    println!("Done!");

    loop {}
}

/// A rolling test that loops between hasher for every step to test interleaving.
///
/// Returns true if the test succeed, else false.
fn rolling_with_digest_trait(size: usize) -> bool {
    let source_data = unsafe { core::slice::from_raw_parts(CHAR_ARRAY.as_ptr(), size) };

    let mut sha1 = esp_hal::sha::Sha1::default();
    #[cfg(not(feature = "esp32"))]
    let mut sha224 = esp_hal::sha::Sha224::default();
    let mut sha256 = esp_hal::sha::Sha256::default();

    // The Digest::update will consume the entirety of remaining. We don't need to loop until
    // remaining is fully consumed.
    Digest::update(&mut sha1, source_data);
    #[cfg(not(feature = "esp32"))]
    Digest::update(&mut sha224, source_data);
    Digest::update(&mut sha256, source_data);

    let sha1_output: [u8; 20] = Digest::finalize(sha1).into();
    #[cfg(not(feature = "esp32"))]
    let sha224_output: [u8; 28] = Digest::finalize(sha224).into();
    let sha256_output: [u8; 32] = Digest::finalize(sha256).into();

    // Calculate software result to compare against
    // Sha1
    let mut sha1_sw = sha1::Sha1::new();
    sha1_sw.update(source_data);
    let soft_result = sha1_sw.finalize();
    for (a, b) in sha1_output.iter().zip(soft_result) {
        if *a != b {
            log::error!("HW: {:02x?}", sha1_output);
            log::error!("SW: {:02x?}", soft_result);
            return false;
        }
    }

    // Sha224
    #[cfg(not(feature = "esp32"))]
    {
        let mut sha224_sw = sha2::Sha224::new();
        sha224_sw.update(source_data);
        let soft_result = sha224_sw.finalize();
        for (a, b) in sha224_output.iter().zip(soft_result) {
            if *a != b {
                log::error!("HW: {:02x?}", sha224_output);
                log::error!("SW: {:02x?}", soft_result);
                return false;
            }
        }
    }

    // Sha256
    let mut sha256_sw = sha2::Sha256::new();
    sha256_sw.update(source_data);
    let soft_result = sha256_sw.finalize();
    for (a, b) in sha256_output.iter().zip(soft_result) {
        if *a != b {
            log::error!("HW: {:02x?}", sha256_output);
            log::error!("SW: {:02x?}", soft_result);
            return false;
        }
    }

    true
}

/// A simple test using [esp_hal::sha::Sha] trait to test hashing for an algorithm against a
/// specific size. This will compare the result with a software implementation and return false if
/// there's a mismatch
fn test_for_size<'a, D: Digest + Default + Sha<'a, esp_hal::Blocking>, const N: usize>(
    size: usize,
) -> bool {
    let source_data = unsafe { core::slice::from_raw_parts(CHAR_ARRAY.as_ptr(), size) };
    let mut remaining = source_data;
    let mut hasher = D::default();

    // Short hashes can be created by decreasing the output buffer to the desired
    // length
    let mut output = [0u8; N];

    // The hardware implementation takes a subslice of the input, and returns the
    // unprocessed parts The unprocessed parts can be input in the next
    // iteration, you can always add more data until finish() is called. After
    // finish() is called update()'s will contribute to a new hash which
    // can be extracted again with finish().
    while remaining.len() > 0 {
        // Can add println to view progress, however println takes a few orders of
        // magnitude longer than the Sha function itself so not useful for
        // comparing processing time println!("Remaining len: {}",
        // remaining.len());

        // All the HW Sha functions are infallible so unwrap is fine to use if you use
        // block!
        remaining = block!(Sha::update(&mut hasher, &remaining)).unwrap();
    }

    // Finish can be called as many times as desired to get multiple copies of the
    // output.
    block!(hasher.finish(output.as_mut_slice())).unwrap();

    // Compare against Software result.
    match N {
        20 => {
            let mut hasher = sha1::Sha1::new();
            hasher.update(source_data);
            let soft_result = hasher.finalize();
            for (a, b) in output.iter().zip(soft_result) {
                if *a != b {
                    return false;
                }
            }
        }
        28 => {
            let mut hasher = sha2::Sha224::new();
            hasher.update(source_data);
            let soft_result = hasher.finalize();
            for (a, b) in output.iter().zip(soft_result) {
                if *a != b {
                    return false;
                }
            }
        }
        32 => {
            let mut hasher = sha2::Sha256::new();
            hasher.update(source_data);
            let soft_result = hasher.finalize();
            for (a, b) in output.iter().zip(soft_result) {
                if *a != b {
                    return false;
                }
            }
        }
        48 => {
            let mut hasher = sha2::Sha384::new();
            hasher.update(source_data);
            let soft_result = hasher.finalize();
            for (a, b) in output.iter().zip(soft_result) {
                if *a != b {
                    return false;
                }
            }
        }
        64 => {
            let mut hasher = sha2::Sha512::new();
            hasher.update(source_data);
            let soft_result = hasher.finalize();
            for (a, b) in output.iter().zip(soft_result) {
                if *a != b {
                    log::error!("HW: {:02x?}", output);
                    log::error!("SW: {:02x?}", soft_result);
                    return false;
                }
            }
        }
        _ => todo!(),
    };

    true
}
