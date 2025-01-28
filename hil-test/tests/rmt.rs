//! RMT Loopback Test

//% CHIPS: esp32 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: unstable

#![no_std]
#![no_main]

use esp_hal::{
    gpio::Level,
    rmt::{PulseCode, Rmt, RxChannel, RxChannelConfig, TxChannel, TxChannelConfig},
    time::RateExtU32,
};
use hil_test as _;

#[cfg(test)]
#[embedded_test::tests(default_timeout = 1)]
mod tests {
    use esp_hal::rmt::Error;

    use super::*;

    #[init]
    fn init() {}

    #[test]
    fn rmt_loopback() {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        cfg_if::cfg_if! {
            if #[cfg(feature = "esp32h2")] {
                let freq = 32.MHz();
            } else {
                let freq = 80.MHz();
            }
        };

        let rmt = Rmt::new(peripherals.RMT, freq).unwrap();

        let (rx, tx) = hil_test::common_test_pins!(peripherals);

        let tx_config = TxChannelConfig::default().with_clk_divider(255);

        let tx_channel = {
            use esp_hal::rmt::TxChannelCreator;
            rmt.channel0.configure(tx, tx_config).unwrap()
        };

        let rx_config = RxChannelConfig::default()
            .with_clk_divider(255)
            .with_idle_threshold(1000);

        cfg_if::cfg_if! {
            if #[cfg(feature = "esp32")] {
                let  rx_channel = {
                    use esp_hal::rmt::RxChannelCreator;
                    rmt.channel1.configure(rx, rx_config).unwrap()
                };
            } else if #[cfg(feature = "esp32s2")] {
                let rx_channel = {
                    use esp_hal::rmt::RxChannelCreator;
                    rmt.channel1.configure(rx, rx_config).unwrap()
                };
            } else if #[cfg(feature = "esp32s3")] {
                let  rx_channel = {
                    use esp_hal::rmt::RxChannelCreator;
                    rmt.channel7.configure(rx, rx_config).unwrap()
                };
            } else {
                let  rx_channel = {
                    use esp_hal::rmt::RxChannelCreator;
                    rmt.channel2.configure(rx, rx_config).unwrap()
                };
            }
        }

        let mut tx_data = [PulseCode::new(Level::High, 200, Level::Low, 50); 20];

        tx_data[tx_data.len() - 2] = PulseCode::new(Level::High, 3000, Level::Low, 500);
        tx_data[tx_data.len() - 1] = PulseCode::empty();

        let mut rcv_data: [u32; 20] = [PulseCode::empty(); 20];

        let rx_transaction = rx_channel.receive(&mut rcv_data).unwrap();
        let tx_transaction = tx_channel.transmit(&tx_data).unwrap();

        rx_transaction.wait().unwrap();
        tx_transaction.wait().unwrap();

        // the last two pulse-codes are the ones which wait for the timeout so
        // they can't be equal
        assert_eq!(&tx_data[..18], &rcv_data[..18]);
    }

    #[test]
    fn rmt_single_shot_fails_without_end_marker() {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        cfg_if::cfg_if! {
            if #[cfg(feature = "esp32h2")] {
                let freq = 32.MHz();
            } else {
                let freq = 80.MHz();
            }
        };

        let rmt = Rmt::new(peripherals.RMT, freq).unwrap();

        let (_, tx) = hil_test::common_test_pins!(peripherals);

        let tx_config = TxChannelConfig {
            clk_divider: 255,
            ..TxChannelConfig::default()
        };

        let tx_channel = {
            use esp_hal::rmt::TxChannelCreator;
            rmt.channel0.configure(tx, tx_config).unwrap()
        };

        let tx_data = [PulseCode::new(Level::High, 200, Level::Low, 50); 20];

        let tx_transaction = tx_channel.transmit(&tx_data);

        assert!(tx_transaction.is_err());
        assert!(matches!(tx_transaction, Err(Error::EndMarkerMissing)));
    }

    #[cfg(any(esp32s3, esp32c3, esp32c6, esp32h2))]
    #[test]
    fn rmt_loopback_fill_longer_buffer() {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        cfg_if::cfg_if! {
            if #[cfg(feature = "esp32h2")] {
                let freq = 32.MHz();
            } else {
                let freq = 80.MHz();
            }
        };

        let rmt = Rmt::new(peripherals.RMT, freq).unwrap();

        let (rx, tx) = hil_test::common_test_pins!(peripherals);

        let rx_config = RxChannelConfig::default()
            .with_clk_divider(10)
            .with_idle_threshold(1000);

        let tx_config = TxChannelConfig::default().with_clk_divider(10);

        let tx_channel = {
            use esp_hal::rmt::TxChannelCreator;
            rmt.channel0.configure(tx, tx_config).unwrap()
        };

        cfg_if::cfg_if! {
            if #[cfg(feature = "esp32s3")] {
                let rx_channel = {
                    use esp_hal::rmt::RxChannelCreator;
                    rmt.channel7.configure(rx, rx_config).unwrap()
                };
            } else {
                let rx_channel = {
                    use esp_hal::rmt::RxChannelCreator;
                    rmt.channel2.configure(rx, rx_config).unwrap()
                };
            }
        }

        let mut rcv_data: [u32; 105] = [PulseCode::empty(); 105];

        let mut tx_data = [PulseCode::empty(); 22];

        // Periodic data so we can be sure we aren't dropping bytes
        for i in 0..21 {
            tx_data[i] = PulseCode::new(Level::High, (i + 10) as u16, Level::Low, 50);
        }

        let rx_transaction = rx_channel.receive(&mut rcv_data).unwrap();

        // We use `transmit_continuously` here because the rx wait monopolizes
        // a core and this lets us transmit more than the RMT buffer's
        // worth of bytes using dedicated hardware
        let tx_transaction = tx_channel.transmit_continuously(&tx_data).unwrap();

        rx_transaction.wait().unwrap();
        tx_transaction.stop().unwrap();

        let mut expected_received = [PulseCode::empty(); 105];
        for i in 0..105 {
            expected_received[i] = tx_data[i % 21];
        }

        assert_eq!(rcv_data, expected_received);
    }
}
