//! This shows a very basic example of running code on the LP core.
//!
//! Code on LP core increments a counter and continuously toggles GPIO1. The
//! current value is printed by the HP core.
//!
//! Make sure to first compile the `esp32c6-lp-hal/examples/blinky.rs` example

#![no_std]
#![no_main]

use esp32c6_hal::{
    clock::ClockControl,
    gpio::lp_gpio::IntoLowPowerPin,
    lp_core,
    peripherals::Peripherals,
    prelude::*,
    uart::{
        config::{Config, DataBits, Parity, StopBits},
        TxRxPins,
    },
    Uart,
    IO,
};
use esp_backtrace as _;
use esp_println::{print, println};

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    // Set up (HP) UART1:

    let config = Config {
        baudrate: 115_200,
        data_bits: DataBits::DataBits8,
        parity: Parity::ParityNone,
        stop_bits: StopBits::STOP1,
    };

    let pins = TxRxPins::new_tx_rx(
        io.pins.gpio6.into_push_pull_output(),
        io.pins.gpio7.into_floating_input(),
    );

    let mut uart1 = Uart::new_with_config(peripherals.UART1, config, Some(pins), &clocks);

    // Set up (LP) UART:

    let lp_tx = io.pins.gpio5.into_low_power().into_push_pull_output();
    let lp_rx = io.pins.gpio4.into_low_power().into_floating_input();

    let mut lp_core = esp32c6_hal::lp_core::LpCore::new(peripherals.LP_CORE);
    lp_core.stop();
    println!("lp core stopped");

    // load code to LP core
    let lp_core_code = load_lp_code!(
        "../esp32c6-lp-hal/target/riscv32imac-unknown-none-elf/release/examples/uart"
    );

    // start LP core
    lp_core_code.run(
        &mut lp_core,
        lp_core::LpCoreWakeupSource::HpCpu,
        lp_tx,
        lp_rx,
    );
    println!("lpcore run");

    loop {
        let read = nb::block!(uart1.read());

        match read {
            Ok(read) => println!("Read 0x{:02x}", read),
            Err(err) => println!("Error {:?}", err),
        }
    }
}
