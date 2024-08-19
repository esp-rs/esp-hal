//! RMT Loopback Test
//!
//! It's assumed GPIO2 is connected to GPIO3

//% CHIPS: esp32 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use esp_hal::{
    gpio::Io,
    prelude::*,
    rmt::{PulseCode, Rmt, RxChannel, RxChannelConfig, TxChannel, TxChannelConfig},
};
use hil_test as _;

#[cfg(test)]
#[embedded_test::tests]
mod tests {
    use super::*;

    #[init]
    fn init() {}

    #[test]
    #[timeout(1)]
    fn rmt_loopback() {
        let System {
            peripherals,
            clocks,
            ..
        } = esp_hal::init(CpuClock::boot_default());

        let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

        cfg_if::cfg_if! {
            if #[cfg(feature = "esp32h2")] {
                let freq = 32.MHz();
            } else {
                let freq = 80.MHz();
            }
        };

        let rmt = Rmt::new(peripherals.RMT, freq, &clocks).unwrap();

        let tx_config = TxChannelConfig {
            clk_divider: 255,
            ..TxChannelConfig::default()
        };

        let tx_channel = {
            use esp_hal::rmt::TxChannelCreator;
            rmt.channel0.configure(io.pins.gpio2, tx_config).unwrap()
        };

        let rx_config = RxChannelConfig {
            clk_divider: 255,
            idle_threshold: 1000,
            ..RxChannelConfig::default()
        };

        cfg_if::cfg_if! {
            if #[cfg(any(feature = "esp32", feature = "esp32s2"))] {
                let  rx_channel = {
                    use esp_hal::rmt::RxChannelCreator;
                    rmt.channel1.configure(io.pins.gpio3, rx_config).unwrap()
                };
            } else if #[cfg(feature = "esp32s3")] {
                let  rx_channel = {
                    use esp_hal::rmt::RxChannelCreator;
                    rmt.channel7.configure(io.pins.gpio3, rx_config).unwrap()
                };
            } else {
                let  rx_channel = {
                    use esp_hal::rmt::RxChannelCreator;
                    rmt.channel2.configure(io.pins.gpio3, rx_config).unwrap()
                };
            }
        }

        let mut tx_data = [PulseCode {
            level1: true,
            length1: 200,
            level2: false,
            length2: 50,
        }; 20];

        tx_data[tx_data.len() - 2] = PulseCode {
            level1: true,
            length1: 3000,
            level2: false,
            length2: 500,
        };
        tx_data[tx_data.len() - 1] = PulseCode::default();

        let mut rcv_data = [PulseCode {
            level1: false,
            length1: 0,
            level2: false,
            length2: 0,
        }; 20];

        let rx_transaction = rx_channel.receive(&mut rcv_data).unwrap();
        let tx_transaction = tx_channel.transmit(&tx_data);
        tx_transaction.wait().unwrap();
        rx_transaction.wait().unwrap();

        // the last two pulse-codes are the ones which wait for the timeout so they
        // can't be equal
        assert_eq!(&tx_data[..18], &rcv_data[..18]);
    }
}
