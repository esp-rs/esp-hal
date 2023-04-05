//! Demonstrates the use of the HMAC peripheral and compares the speed of
//! hardware-accelerated and pure software hashing.

#![no_std]
#![no_main]

use esp32c3_hal::{
    clock::ClockControl,
    hmac::{Hmac, HmacPurpose, KeyId},
    peripherals::Peripherals,
    prelude::*,
    systimer::SystemTimer,
    timer::TimerGroup,
    Rng,
    Rtc,
};
use esp_backtrace as _;
use esp_println::{print, println};
use hmac::{Hmac as HmacSw, Mac};
use nb::block;
use sha2::Sha256;

type HmacSha256 = HmacSw<Sha256>;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let mut system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    // Disable the RTC and TIMG watchdog timers
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    let timer_group0 = TimerGroup::new(
        peripherals.TIMG0,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(
        peripherals.TIMG1,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut wdt1 = timer_group1.wdt;

    let mut rng = Rng::new(peripherals.RNG);

    // Disable MWDT and RWDT (Watchdog) flash boot protection
    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();

    // Set sw key
    let key = [0_u8; 32].as_ref();

    let mut hw_hmac = Hmac::new(peripherals.HMAC, &mut system.peripheral_clock_control);

    let mut src = [0_u8; 1024];
    rng.read(src.as_mut_slice()).unwrap();
    // println!("HMAC input {:02X?}", src);

    let mut output = [0u8; 32];

    println!("Beginning stress tests...");
    print!("Testing length from 0 to {:?} bytes for HMAC...", src.len());
    let mut delta_time = SystemTimer::now();
    for i in 0..src.len() + 1 {
        let (nsrc, _) = src.split_at(i);
        let mut remaining = nsrc;
        hw_hmac.init();
        block!(hw_hmac.configure(HmacPurpose::ToUser, KeyId::Key0)).expect("Key purpose mismatch");
        let pre_hw_hmac = SystemTimer::now();
        while remaining.len() > 0 {
            remaining = block!(hw_hmac.update(remaining)).unwrap();
        }
        block!(hw_hmac.finalize(output.as_mut_slice())).unwrap();
        let post_hw_hmac = SystemTimer::now();
        let mut sw_hmac = HmacSha256::new_from_slice(key).expect("HMAC can take key of any size");
        sw_hmac.update(nsrc);
        let soft_result = sw_hmac.finalize().into_bytes();
        for (a, b) in output.iter().zip(soft_result) {
            assert_eq!(*a, b);
        }
        delta_time = post_hw_hmac - pre_hw_hmac;
    }
    print!("ok");
    print!(
        " (it took {} cycles for hw hash with max length",
        delta_time
    );
    print!(" or {} cycles/byte", delta_time / (src.len() as u64));
    println!(")");
    println!("Finished stress tests!");

    loop {}
}
