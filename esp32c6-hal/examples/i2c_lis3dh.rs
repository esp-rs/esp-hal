//! I2C LIS3DH Example
//!
//! Pins used are:
//!
//! SDA     GPIO4
//! SCL     GPIO5
//!
//! Depending on your target and the board you are using you have to change the
//! pins.
//!
//! This example connects to an LIS3DH 3-axis accelerometer, creates an
//! orientation tracker using this device, and prints out the current
//! orientation of the device over UART.

#![no_std]
#![no_main]

use esp32c6_hal::{
    clock::ClockControl,
    i2c::I2C,
    peripherals::Peripherals,
    prelude::*,
    timer::TimerGroup,
    Delay,
    Rtc,
    IO,
};
use esp_backtrace as _;
use esp_println::println;
use lis3dh::{
    accelerometer::{RawAccelerometer, Tracker},
    Lis3dh,
    Range,
    SlaveAddr,
};

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let mut system = peripherals.PCR.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let mut rtc = Rtc::new(peripherals.LP_CLKRST);
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

    // Disable the watchdog timers:
    rtc.swd.disable();
    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();

    let mut delay = Delay::new(&clocks);
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    let i2c = I2C::new(
        peripherals.I2C0,
        io.pins.gpio4, // SDA
        io.pins.gpio5, // SCL
        400u32.kHz(),
        &mut system.peripheral_clock_control,
        &clocks,
    );

    let mut lis3dh = Lis3dh::new_i2c(i2c, SlaveAddr::Alternate).unwrap();
    lis3dh.set_range(Range::G8).unwrap();

    // Threshold value obtain experimentally, as suggested by the `accelerometer`
    // documentation:
    // https://docs.rs/accelerometer/latest/accelerometer/orientation/struct.Tracker.html
    let mut tracker = Tracker::new(3700.0);

    loop {
        let accel = lis3dh.accel_raw().unwrap();
        let orientation = tracker.update(accel);

        println!("{orientation:?}");

        delay.delay_ms(250u32);
    }
}
