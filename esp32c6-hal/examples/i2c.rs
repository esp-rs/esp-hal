//! Read calibration data from BMP180 sensor
//!
//! This example dumps the calibration data from a BMP180 sensor
//!
//! The following wiring is assumed:
//! - SDA => GPIO1
//! - SCL => GPIO2

#![no_std]
#![no_main]

use esp32c6_hal::{
    clock::ClockControl,
    gpio::IO,
    i2c::I2C,
    peripherals::Peripherals,
    prelude::*,
    timer::TimerGroup,
    Rtc,
};
use esp_backtrace as _;
use esp_println::println;
use esp_riscv_rt::entry;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let mut system = peripherals.PCR.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    // Disable the watchdog timers. For the ESP32-C6, this includes the Super WDT,
    // and the TIMG WDTs.
    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let _wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(peripherals.TIMG1, &clocks);
    let _wdt1 = timer_group1.wdt;

    let mut rtc = Rtc::new(peripherals.LP_CLKRST);

    // Disable MWDT and RWDT (Watchdog) flash boot protection
    rtc.rwdt.disable();
    rtc.swd.disable();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    // Create a new peripheral object with the described wiring
    // and standard I2C clock speed
    let mut i2c = I2C::new(
        peripherals.I2C0,
        io.pins.gpio1,
        io.pins.gpio2,
        100u32.kHz(),
        &mut system.peripheral_clock_control,
        &clocks,
    );

    loop {
        let mut data = [0u8; 1];
        i2c.write_read(0x76, &[0xD0], &mut data).ok();

        println!("{:02x?}", data);
    }
}
