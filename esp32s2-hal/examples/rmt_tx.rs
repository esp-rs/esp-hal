//! Demonstrates generating pulse sequences with RMT
//! Connect a logic analyzer to GPIO4 to see the generated pulses.

#![no_std]
#![no_main]

use esp32s2_hal::{
    clock::ClockControl,
    gpio::IO,
    peripherals::Peripherals,
    prelude::*,
    rmt::{PulseCode, TxChannel, TxChannelConfig, TxChannelCreator},
    Delay,
    Rmt,
};
use esp_backtrace as _;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();
    let mut clock_control = system.peripheral_clock_control;

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let mut delay = Delay::new(&clocks);

    let rmt = Rmt::new(peripherals.RMT, 80u32.MHz(), &mut clock_control, &clocks).unwrap();

    let mut channel = rmt
        .channel2
        .configure(
            io.pins.gpio4,
            TxChannelConfig {
                clk_divider: 255,
                ..TxChannelConfig::default()
            },
        )
        .unwrap();

    let mut data = [PulseCode {
        level1: true,
        length1: 200,
        level2: false,
        length2: 50,
    }; 20];

    data[data.len() - 2] = PulseCode {
        level1: true,
        length1: 3000,
        level2: false,
        length2: 500,
    };
    data[data.len() - 1] = PulseCode::default();

    loop {
        let transaction = channel.transmit(&data);
        channel = transaction.wait().unwrap();
        delay.delay_ms(500u32);
    }
}
