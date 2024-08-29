//! This shows a very basic example of running code on the ULP RISCV core.
//!
//! Code on ULP core just increments a counter and blinks with LED. The current
//! value is printed by the HP core.
//!
//! The following wiring is assumed:
//! - LED => GPIO1

//% CHIPS: esp32s2 esp32s3

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    gpio::{rtc_io::*, Io},
    peripherals::Peripherals,
    prelude::*,
    ulp_core,
};
use esp_println::{print, println};

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    let pin = LowPowerOutput::new(io.pins.gpio1);

    let mut ulp_core = ulp_core::UlpCore::new(peripherals.ULP_RISCV_CORE);

    #[cfg(feature = "esp32s3")]
    {
        ulp_core.stop();
        println!("ulp core stopped");
    }

    // load code to LP core
    let lp_core_code =
        load_lp_code!("target/riscv32imc-unknown-none-elf/release/examples/lp_blinky");

    // start LP core
    lp_core_code.run(&mut ulp_core, ulp_core::UlpCoreWakeupSource::HpCpu, pin);
    println!("ulpcore run");

    let data = (0x5000_0400) as *mut u32;
    loop {
        print!("Current {:x}           \u{000d}", unsafe {
            data.read_volatile()
        });
    }
}
