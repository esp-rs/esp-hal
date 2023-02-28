//! Demonstrates the use of the hardware Random Number Generator (RNG)

#![no_std]
#![no_main]

use esp32c6_hal::{
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
    let system = peripherals.PCR.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let mut rtc = Rtc::new(peripherals.LP_CLKRST);
    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(peripherals.TIMG1, &clocks);
    let mut wdt1 = timer_group1.wdt;

    // Disable watchdog timers:
    rtc.swd.disable();
    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();

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
