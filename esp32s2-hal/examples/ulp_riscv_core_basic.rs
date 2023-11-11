//! This shows a very basic example of running code on the ULP RISCV core.
//!
//! Code on ULP core just increments a counter and blinks GPIO 1. The current
//! value is printed by the HP core.

#![no_std]
#![no_main]

use esp32s2_hal::{
    clock::ClockControl,
    gpio::rtc_io::*,
    peripherals::Peripherals,
    prelude::*,
    ulp_core,
    IO,
};
use esp_backtrace as _;
use esp_println::{print, println};

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let _clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let pin = io.pins.gpio1.into_low_power().into_push_pull_output();

    let mut ulp_core = ulp_core::UlpCore::new(peripherals.ULP_RISCV_CORE);

    // load code to LP core
    let lp_core_code = load_lp_code!(
        "../esp-ulp-riscv-hal/target/riscv32imc-unknown-none-elf/release/examples/blinky"
    );

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
