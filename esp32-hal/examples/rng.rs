//! Demonstrates the use of the hardware Random Number Generator (RNG)

#![no_std]
#![no_main]

use esp32_hal::{
    clock::ClockControl,
    peripherals::Peripherals,
    prelude::*,
    timer::TimerGroup,
    Rng,
    Rtc,
};
use esp_backtrace as _;
use esp_println::println;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.DPORT.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut wdt = timer_group0.wdt;
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);

    // Disable MWDT and RWDT (Watchdog) flash boot protection:
    wdt.disable();
    rtc.rwdt.disable();

    // Instantiate the Random Number Generator peripheral:
    let mut rng = Rng::new(peripherals.RNG);

    // Generate a random word (u32):
    println!("Random u32:   {}", rng.random());

    // Fill a buffer with random bytes:
    let mut buf = [0u8; 16];
    rng.read(&mut buf).unwrap();
    println!("Random bytes: {:?}", buf);

    loop {}
}
