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
use sha2::{Digest, Sha512};

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let _clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let source_data = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa".as_bytes();
    let mut remaining = source_data;
    let mut hasher = Sha::new(peripherals.SHA, ShaMode::SHA512);

    // Short hashes can be created by decreasing the output buffer to the desired
    // length
    let mut output = [0u8; 64];

    let pre_calc = xtensa_lx::timer::get_cycle_count();
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
        remaining = block!(hasher.update(remaining)).unwrap();
    }

    // Finish can be called as many times as desired to get mutliple copies of the
    // output.
    block!(hasher.finish(output.as_mut_slice())).unwrap();
    let post_calc = xtensa_lx::timer::get_cycle_count();
    let hw_time = post_calc - pre_calc;
    println!("Took {} cycles", hw_time);
    println!("SHA512 Hash output {:02x?}", output);

    let pre_calc = xtensa_lx::timer::get_cycle_count();
    let mut hasher = Sha512::new();
    hasher.update(source_data);
    let soft_result = hasher.finalize();
    let post_calc = xtensa_lx::timer::get_cycle_count();
    let soft_time = post_calc - pre_calc;
    println!("Took {} cycles", soft_time);
    println!("SHA512 Hash output {:02x?}", soft_result);

    println!("HW SHA is {}x faster", soft_time / hw_time);

    loop {}
}
