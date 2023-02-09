//! Blinks an LED
//!
//! This assumes that a LED is connected to the pin assigned to `led`. (GPIO5)

#![no_std]
#![no_main]

use esp32c6_hal::{
    clock::ClockControl,
    gpio::IO,
    peripherals::Peripherals,
    prelude::*,
    timer::TimerGroup,
    Delay,
    Rtc,
};
use esp_backtrace as _;
use esp_riscv_rt::entry;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.PCR.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    // Disable the watchdog timers. For the ESP32-C6, this includes the Super WDT,
    // and the TIMG WDTs.
    let mut rtc = Rtc::new(peripherals.LP_CLKRST);
    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(peripherals.TIMG1, &clocks);
    let mut wdt1 = timer_group1.wdt;

    rtc.swd.disable();
    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();

    // Set GPIO5 as an output, and set its state high initially.
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let mut led = io.pins.gpio5.into_push_pull_output();

    led.set_high().unwrap();

    // Initialize the Delay peripheral, and use it to toggle the LED state in a
    // loop.
    let mut delay = Delay::new(&clocks);

    // just for testing ... remove this
    let rst_reason = unsafe { (0x600b0410 as *mut u32).read_volatile() & 0b11111 };
    esp_println::println!("raw rst reason {} 0x{:x}", rst_reason, rst_reason);

    loop {
        esp_println::println!("blinky");
        led.toggle().unwrap();
        delay.delay_ms(2000u32);
    }
}
