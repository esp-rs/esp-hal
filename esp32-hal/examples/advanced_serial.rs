//! This shows how to configure UART
//! You can short the TX and RX pin and see it reads what was written.
//! Additionally you can connect a logic analzyer to TX and see how the changes
//! of the configuration change the output signal.
//!
//! The following wiring is assumed:
//! - TX => GPIO16
//! - RX => GPIO17

#![no_std]
#![no_main]

use esp32_hal::{
    clock::ClockControl,
    gpio::IO,
    peripherals::Peripherals,
    prelude::*,
    uart::{
        config::{Config, DataBits, Parity, StopBits},
        TxRxPins,
    },
    Delay,
    Uart,
};
use esp_backtrace as _;
use esp_println::println;
use nb::block;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let mut system = peripherals.DPORT.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let config = Config {
        baudrate: 115200,
        data_bits: DataBits::DataBits8,
        parity: Parity::ParityNone,
        stop_bits: StopBits::STOP1,
    };

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let pins = TxRxPins::new_tx_rx(
        io.pins.gpio16.into_push_pull_output(),
        io.pins.gpio17.into_floating_input(),
    );

    let mut serial1 = Uart::new_with_config(
        peripherals.UART1,
        Some(config),
        Some(pins),
        &clocks,
        &mut system.peripheral_clock_control,
    );

    let mut delay = Delay::new(&clocks);

    println!("Start");
    loop {
        serial1.write(0x42).ok();
        let read = block!(serial1.read());

        match read {
            Ok(read) => println!("Read 0x{:02x}", read),
            Err(err) => println!("Error {:?}", err),
        }

        delay.delay_ms(250u32);
    }
}
