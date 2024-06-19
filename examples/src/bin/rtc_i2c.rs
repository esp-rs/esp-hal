//! Uses the RTC I2C peripheral from the main CPU

//% CHIPS: esp32s3

#![no_std]
#![no_main]

use core::time::Duration;

use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    delay::Delay,
    gpio::Io,
    i2c::rtc_i2c::{RtcI2c, Timing},
    peripherals::Peripherals,
    prelude::*,
    system::SystemControl,
};

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();
    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    let i2c = RtcI2c::new(
        peripherals.RTC_I2C,
        io.pins.gpio1,
        io.pins.gpio2,
        Timing::standard_mode(),
        Duration::from_micros(100),
    );

    let delay = Delay::new(&clocks);

    loop {
        let mut data = [0; 10];

        i2c.read(0x43, 5, &mut data).unwrap();

        esp_println::println!("I2C data {:?}", data);
        delay.delay_millis(1000);
    }
}
