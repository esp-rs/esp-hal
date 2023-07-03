//! Prints time in milliseconds from the RTC Timer

#![no_std]
#![no_main]

use esp32c2_hal::{
    clock::ClockControl,
    peripherals::Peripherals,
    prelude::*,
    timer::TimerGroup,
    Delay,
    Rtc,
};
use esp_backtrace as _;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let mut system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    // Disable the watchdog timers. For the ESP32-C2, this includes the Super WDT,
    // the RTC WDT, and the TIMG WDTs.
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    let timer_group0 = TimerGroup::new(
        peripherals.TIMG0,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut wdt0 = timer_group0.wdt;

    rtc.swd.disable();
    rtc.rwdt.disable();
    wdt0.disable();

    let mut delay = Delay::new(&clocks);

    loop {
        esp_println::println!("rtc time in milliseconds is {}", rtc.get_time_ms());
        delay.delay_ms(1000u32);
    }
}
