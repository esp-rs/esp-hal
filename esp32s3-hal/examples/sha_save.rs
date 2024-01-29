//! Demonstrates the use of the SHA peripheral and compares the speed of
//! hardware-accelerated and pure software hashing.

#![no_std]
#![no_main]

use esp32s3_hal::{
    clock::ClockControl,
    peripherals::Peripherals,
    prelude::*,
    sha::{Sha, ShaMode},
    xtensa_lx,
};
use esp_backtrace as _;
use esp_println::println;
use nb::block;
use sha2::{Digest, Sha256};

#[entry]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let _clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let source_data = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa".as_bytes();
    let mut remaining = source_data.clone();
    let mut hasher = Sha::new(peripherals.SHA, ShaMode::SHA256);

    // Short hashes can be created by decreasing the output buffer to the desired
    // length
    let mut output = [0u8; 32];

    let pre_calc = xtensa_lx::timer::get_cycle_count();
    // The hardware implementation takes a subslice of the input, and returns the
    // unprocessed parts The unprocessed parts can be input in the next
    // iteration, you can always add more data until finish() is called. After
    // finish() is called update()'s will contribute to a new hash which
    // can be extracted again with finish().

    // Number of runs of the hasher
    let mut number_of_runs = 0;

    // Saved state of the hasher, to restore from
    let mut saved_state = None;

    // Saved remaining slice, to resume with
    let mut saved_remaining = None;

    while remaining.len() > 0 {
        // Can add println to view progress, however println takes a few orders of
        // magnitude longer than the Sha function itself so not useful for
        // comparing processing time println!("Remaining len: {}",
        // remaining.len());

        // All the HW Sha functions are infallible so unwrap is fine to use if you use
        // block!
        remaining = block!(hasher.update(remaining)).unwrap();

        number_of_runs += 1;
        // Run 1 works
        // Run 2 works
        // Run 3 works
        // Run 4 fails
        println!("Run: {}", number_of_runs);
        if number_of_runs == 3 {
            println!("Run #{} saved", number_of_runs);
            saved_state = Some(hasher.save_digest::<32>());
            saved_remaining = Some(remaining);
        }

    }

    // Finish can be called as many times as desired to get mutliple copies of the
    // output.
    block!(hasher.finish(output.as_mut_slice())).unwrap();
    let post_calc = xtensa_lx::timer::get_cycle_count();
    let hw_time = post_calc - pre_calc;
    println!("Took {} cycles", hw_time);
    println!("SHA256 Hash output {:02x?}", output);

    // Restore SHA digest
    hasher.restore_digest(saved_state.unwrap());
    remaining = saved_remaining.unwrap();

    while remaining.len() > 0 {
        remaining = block!(hasher.update(remaining)).unwrap();
    }

    block!(hasher.finish(output.as_mut_slice())).unwrap();
    println!("SHA256 Hash output from saved state {:02x?}", output);

    let pre_calc = xtensa_lx::timer::get_cycle_count();
    let mut hasher = Sha256::new();
    hasher.update(source_data);
    let soft_result = hasher.finalize();
    let post_calc = xtensa_lx::timer::get_cycle_count();
    let soft_time = post_calc - pre_calc;
    println!("Took {} cycles", soft_time);
    println!("SHA256 Hash output {:02x?}", soft_result);

    println!("HW SHA is {}x faster", soft_time / hw_time);

    loop {}
}
