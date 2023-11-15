//! This shows a very basic example of running code on the LP core.
//!
//! Code on LP core increments a counter and continuously toggles GPIO1. The
//! current value is printed by the HP core.
//!
//! Make sure to first compile the `esp-lp-hal/examples/blinky.rs` example

#![no_std]
#![no_main]

use esp32c6_hal::{
    clock::ClockControl, gpio::lp_gpio::IntoLowPowerPin, i2c::I2C, lp_core,
    peripherals::Peripherals, prelude::*, IO,
};
use esp_backtrace as _;
use esp_hal_common::system::PeripheralClockControl;
use esp_println::{print, println};

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    // configure GPIO 1 as LP output pin
    let lp_pin_sda = io.pins.gpio6.into_low_power().into_puinput_ppoutput();

    let lp_pin_scl = io.pins.gpio7.into_low_power().into_puinput_ppoutput();

    let mut lp_core = esp32c6_hal::lp_core::LpCore::new(peripherals.LP_CORE);
    lp_core.stop();
    println!("lp core stopped");

    // load code to LP core
    let lp_core_code =
        load_lp_code!("../esp-lp-hal/target/riscv32imac-unknown-none-elf/release/examples/blinky");

    // start LP core
    lp_core_code.run(
        &mut lp_core,
        lp_core::LpCoreWakeupSource::HpCpu,
        lp_pin_sda,
        lp_pin_scl,
    );
    println!("lpcore run");

    let data = (0x5000_2000) as *mut u32;
    loop {
        print!("Current {:x}           \u{000d}", unsafe {
            data.read_volatile()
        });
    }
}
