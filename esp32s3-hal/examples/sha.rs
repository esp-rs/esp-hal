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
    Delay
};
use nb::block;
use esp_backtrace as _;
use esp_println::println;
use xtensa_lx_rt::entry;

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

    let mut delay = Delay::new(&clocks);

    // $ echo "Test data" | sha1sum                                                                                                                                                                                                           (base) 
    // 094a36de27ee800c2c7d98544caa9b64e76f5d3c

    // pySHA512(128 * 'a') -> b73d1929aa615934e61a871596b3f3b33359f42b8175602e89f7e06e5f658a243667807ed300314b95cacdd579f3e33abdfbe351909519a846d465c59582f321
    // SHA512SUM(128 * 'a') -> b73d1929aa615934e61a871596b3f3b33359f42b8175602e89f7e06e5f658a243667807ed300314b95cacdd579f3e33abdfbe351909519a846d465c59582f321
    let source_data = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaac".as_bytes();
    let mut remaining = source_data.clone();
    let mut hasher = Sha::new(peripherals.SHA, ShaMode::SHA512);
    let mut output = [0u8; 64];

    loop {
        println!("Remaining len: {} first_run: {}, finished: {}", remaining.len(), hasher.first_run(), hasher.finished());
        match block!(hasher.update(remaining)) {
            Ok(data) => remaining = data,
            _ => unreachable!()
        }
        if remaining.len() == 0 {
            break;
        }
    }
    
    block!(hasher.finish(output.as_mut_slice())).unwrap();
    println!("Final len: {} first_run: {}, finished: {}", remaining.len(), hasher.first_run(), hasher.finished());
    println!("SHA512 Hash output {:02x?}", output);

    block!(hasher.finish(output.as_mut_slice())).unwrap();
    println!("SHA512 Hash output {:02x?}", output);
    let usha = hasher.free();


    let mut remaining = source_data.clone();
    let mut hasher = Sha::new(usha, ShaMode::SHA512);
    let mut output = [0u8; 64];

    loop {
        println!("Remaining len: {} first_run: {}, finished: {}", remaining.len(), hasher.first_run(), hasher.finished());
        match block!(hasher.update(remaining)) {
            Ok(data) => remaining = data,
            _ => unreachable!()
        }
        if remaining.len() == 0 {
            break;
        }
    }
    
    block!(hasher.finish(output.as_mut_slice())).unwrap();
    println!("Final len: {} first_run: {}, finished: {}", remaining.len(), hasher.first_run(), hasher.finished());
    println!("SHA512 Hash output {:02x?}", output);

    block!(hasher.finish(output.as_mut_slice())).unwrap();
    println!("SHA512 Hash output {:02x?}", output);
    hasher.free();

    loop {}
}
