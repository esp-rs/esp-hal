//! This shows a very basic example of running code on the LP core.
//!
//! Code on LP core uses LP_UART initialized on HP core. For more information
//! check `lp_core_uart` example in the `esp-lp-hal.
//!
//! Make sure to first compile the `esp-lp-hal/examples/uart.rs` example
//!
//! The following wiring is assumed:
//! - TX => GPIO5
//! - RX => GPIO4

//% CHIPS: esp32c6

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    gpio::lp_io::{LowPowerInput, LowPowerOutput},
    lp_core::{LpCore, LpCoreWakeupSource},
    prelude::*,
    uart::{config::Config, lp_uart::LpUart, Uart},
};
use esp_println::println;

#[entry]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let io = peripherals.GPIO.pins();

    // Set up (HP) UART1:

    let mut uart1 =
        Uart::new_with_config(peripherals.UART1, Config::default(), io.gpio6, io.gpio7).unwrap();

    // Set up (LP) UART:
    let lp_tx = LowPowerOutput::new(io.gpio5);
    let lp_rx = LowPowerInput::new(io.gpio4);
    let lp_uart = LpUart::new(peripherals.LP_UART, lp_tx, lp_rx);

    let mut lp_core = LpCore::new(peripherals.LP_CORE);
    lp_core.stop();
    println!("lp core stopped");

    // Load code to LP core:
    let lp_core_code =
        load_lp_code!("../esp-lp-hal/target/riscv32imac-unknown-none-elf/release/examples/uart");

    // Start LP core:
    lp_core_code.run(&mut lp_core, LpCoreWakeupSource::HpCpu, lp_uart);
    println!("lpcore run");

    loop {
        let read = nb::block!(uart1.read_byte());

        match read {
            Ok(read) => println!("Read 0x{:02x}", read),
            Err(err) => println!("Error {:?}", err),
        }
    }
}
