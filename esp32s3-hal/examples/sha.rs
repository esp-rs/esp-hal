//! This shows how to read selected information from eFuses.
//! e.g. the MAC address

#![no_std]
#![no_main]

use esp32s3_hal::{
    clock::ClockControl,
    pac::Peripherals,
    prelude::*,
    timer::TimerGroup,
    Rtc,
    sha::{Sha, ShaMode},
};
use nb::block;
use esp_backtrace as _;
use esp_println::println;
use xtensa_lx_rt::entry;
use sha2::{Sha256, Sha512, Digest};

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take().unwrap();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut wdt = timer_group0.wdt;
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);

    // Disable MWDT and RWDT (Watchdog) flash boot protection
    wdt.disable();
    rtc.rwdt.disable();

    
    let source_data = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa".as_bytes();
    let mut remaining = source_data.clone();
    let mut hasher = Sha::new(peripherals.SHA, ShaMode::SHA512);
    let mut output = [0u8; 64];

    let pre_calc = xtensa_lx::timer::get_cycle_count();
    loop {
        //println!("Remaining len: {}", remaining.len());
        match block!(hasher.update(remaining)) {
            Ok(data) => remaining = data,
            _ => unreachable!()
        }
        if remaining.len() == 0 {
            break;
        }
    }

    block!(hasher.finish(output.as_mut_slice())).unwrap();
    let post_calc = xtensa_lx::timer::get_cycle_count();
    let hw_time = post_calc - pre_calc;
    println!("Took {} cycles", hw_time);
    println!("SHA512 Hash output {:02x?}", output);
    let _usha = hasher.free();


    let pre_calc = xtensa_lx::timer::get_cycle_count();
    let mut hasher = Sha512::new();
    hasher.update(source_data);
    let soft_result = hasher.finalize();
    let post_calc = xtensa_lx::timer::get_cycle_count();
    let soft_time = post_calc - pre_calc;
    println!("Took {} cycles", soft_time);
    println!("SHA512 Hash output {:02x?}", soft_result);

    println!("HW SHA is {}x faster", soft_time/hw_time);

    loop {}
}
