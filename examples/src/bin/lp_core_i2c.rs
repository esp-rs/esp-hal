//! This shows a very basic example of running code on the LP core.
//!
//! Code on LP core uses LP_I2C initialized on HP core. For more information
//! check `lp_core_i2c` example in the `esp-lp-hal`.
//!
//! Make sure to first compile the `esp-lp-hal/examples/i2c.rs` example
//!
//! The following wiring is assumed:
//! - SDA => GPIO6
//! - SCL => GPIO7

//% CHIPS: esp32c6

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    gpio::lp_io::LowPowerOutputOpenDrain,
    i2c::lp_i2c::LpI2c,
    lp_core::{LpCore, LpCoreWakeupSource},
    prelude::*,
};
use esp_println::println;

#[entry]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let lp_sda = LowPowerOutputOpenDrain::new(peripherals.GPIO6);
    let lp_scl = LowPowerOutputOpenDrain::new(peripherals.GPIO7);

    let lp_i2c = LpI2c::new(peripherals.LP_I2C0, lp_sda, lp_scl, 100.kHz());

    let mut lp_core = LpCore::new(peripherals.LP_CORE);
    lp_core.stop();
    println!("lp core stopped");

    // load code to LP core
    let lp_core_code =
        load_lp_code!("../esp-lp-hal/target/riscv32imac-unknown-none-elf/release/examples/i2c");

    // start LP core
    lp_core_code.run(&mut lp_core, LpCoreWakeupSource::HpCpu, lp_i2c);
    println!("lpcore run");

    loop {}
}
