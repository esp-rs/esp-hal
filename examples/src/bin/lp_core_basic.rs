//! This shows a very basic example of running code on the LP core.
//!
//! Code on LP core increments a counter and continuously toggles LED. The
//! current value is printed by the HP core.
//!
//! Make sure to first compile the `esp-lp-hal/examples/blinky.rs` example
//!
//! The following wiring is assumed:
//! - LED => GPIO1

//% CHIPS: esp32c6

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    gpio::lp_io::LowPowerOutput,
    lp_core::{LpCore, LpCoreWakeupSource},
    prelude::*,
};
use esp_println::{print, println};

#[entry]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    // configure GPIO 1 as LP output pin

    let lp_pin = LowPowerOutput::new(peripherals.pins.gpio1);

    let mut lp_core = LpCore::new(peripherals.LP_CORE);
    lp_core.stop();
    println!("lp core stopped");

    // load code to LP core
    let lp_core_code =
        load_lp_code!("../esp-lp-hal/target/riscv32imac-unknown-none-elf/release/examples/blinky");

    // start LP core
    lp_core_code.run(&mut lp_core, LpCoreWakeupSource::HpCpu, lp_pin);
    println!("lpcore run");

    let data = (0x5000_2000) as *mut u32;
    loop {
        print!("Current {:x}           \u{000d}", unsafe {
            data.read_volatile()
        });
    }
}
