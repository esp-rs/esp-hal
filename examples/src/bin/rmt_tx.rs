//! Demonstrates generating pulse sequences with RMT
//!
//! Connect a logic analyzer to GPIO4 to see the generated pulses.
//!
//! The following wiring is assumed:
//! - generated pulses => GPIO4

//% CHIPS: esp32 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    prelude::*,
    rmt::{PulseCode, Rmt, TxChannel, TxChannelConfig, TxChannelCreator},
};

#[entry]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    cfg_if::cfg_if! {
        if #[cfg(feature = "esp32h2")] {
            let freq = 32.MHz();
        } else {
            let freq = 80.MHz();
        }
    };

    let rmt = Rmt::new(peripherals.RMT, freq).unwrap();

    let tx_config = TxChannelConfig {
        clk_divider: 255,
        ..TxChannelConfig::default()
    };

    let mut channel = rmt
        .channel0
        .configure(peripherals.GPIO4, tx_config)
        .unwrap();

    let delay = Delay::new();

    let mut data = [PulseCode::new(true, 200, false, 50); 20];
    data[data.len() - 2] = PulseCode::new(true, 3000, false, 500);
    data[data.len() - 1] = PulseCode::empty();

    loop {
        let transaction = channel.transmit(&data).unwrap();
        channel = transaction.wait().unwrap();
        delay.delay_millis(500);
    }
}
