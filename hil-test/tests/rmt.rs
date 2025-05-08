//! RMT Loopback Test

//% CHIPS: esp32 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: unstable

#![no_std]
#![no_main]

use core::fmt::Debug;

use esp_hal::{
    gpio::{Level, NoPin},
    rmt::{PulseCode, Rmt, RxChannel, RxChannelConfig, TxChannel, TxChannelConfig},
    time::Rate,
};
use hil_test as _;

fn setup(
    tx_config: TxChannelConfig,
    rx_config: RxChannelConfig,
) -> (impl TxChannel + Debug, impl RxChannel + Debug) {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    // Ensure identical RMT carrier frequency in each case to ease estimating
    // test runtime -> 500kHz -> 2us/cycle
    let (freq, div) = if cfg!(feature = "esp32h2") {
        (Rate::from_mhz(32), 255)
    } else {
        (Rate::from_mhz(80), 255)
    };

    let rmt = Rmt::new(peripherals.RMT, freq).unwrap();

    let (rx, tx) = hil_test::common_test_pins!(peripherals);

    let tx_channel = {
        use esp_hal::rmt::TxChannelCreator;
        rmt.channel0
            .configure(tx, tx_config.with_clk_divider(div))
            .unwrap()
    };

    cfg_if::cfg_if! {
        if #[cfg(any(feature = "esp32", feature = "esp32s3"))] {
            let rx_channel_creator = rmt.channel4;
        } else {
            let rx_channel_creator = rmt.channel2;
        }
    };
    let rx_channel = {
        use esp_hal::rmt::RxChannelCreator;
        rx_channel_creator
            .configure(rx, rx_config.with_clk_divider(div))
            .unwrap()
    };

    (tx_channel, rx_channel)
}

// All devices have RAM blocks of 48 or 64 codes. We use this below to hit
// different code paths. For example, 20 codes always fit one block, 80
// definitely require 2, etc.
//
// total test duration should be approx.
//     7ms + tx_len * 0.5ms
fn do_rmt_loopback<const TX_LEN: usize>(tx_memsize: u8, rx_memsize: u8, wait_tx_first: bool) {
    let tx_config = TxChannelConfig::default().with_memsize(tx_memsize);
    let rx_config = RxChannelConfig::default()
        .with_idle_threshold(1000)
        .with_memsize(rx_memsize);

    let (tx_channel, rx_channel) = setup(tx_config, rx_config);

    // (100 -- 300) * 2us/code = 200-600us/code
    let mut tx_data: [_; TX_LEN] = core::array::from_fn(|i| {
        PulseCode::new(Level::High, (100 + (i * 10) % 200) as u16, Level::Low, 50)
    });

    // 3500 * 2us/code = 7ms
    tx_data[TX_LEN - 2] = PulseCode::new(Level::High, 3000, Level::Low, 500);
    tx_data[TX_LEN - 1] = PulseCode::empty();

    let mut rcv_data: [u32; TX_LEN] = [PulseCode::empty(); TX_LEN];

    let rx_transaction = rx_channel.receive(&mut rcv_data).unwrap();
    let tx_transaction = tx_channel.transmit(&tx_data).unwrap();

    if wait_tx_first {
        tx_transaction.wait().unwrap();
        rx_transaction.wait().unwrap();
    } else {
        rx_transaction.wait().unwrap();
        tx_transaction.wait().unwrap();
    }

    // the last two pulse-codes are the ones which wait for the timeout so
    // they can't be equal
    assert_eq!(&tx_data[..TX_LEN - 2], &rcv_data[..TX_LEN - 2]);
}

#[cfg(test)]
#[embedded_test::tests(default_timeout = 1)]
mod tests {
    use esp_hal::rmt::Error;

    use super::*;

    #[init]
    fn init() {}

    #[test]
    fn rmt_loopback_simple() {
        // 20 codes fit a single RAM block
        do_rmt_loopback::<20>(1, 1, false);
    }

    #[test]
    fn rmt_loopback_extended_ram() {
        // 80 codes require two RAM blocks
        do_rmt_loopback::<80>(2, 2, false);
    }

    #[test]
    fn rmt_loopback_debug() {
        // Hopefully helpful to figure out why rmt_loopback_tx_wrap fails for esp32
        do_rmt_loopback::<80>(2, 2, true);
    }

    #[test]
    fn rmt_loopback_tx_wrap() {
        // 80 codes require two RAM blocks; thus a tx channel with only 1 block requires
        // wrapping. We need to .wait() on the tx transaction first to handle
        // this.
        do_rmt_loopback::<80>(1, 2, true);
    }

    // FIXME: This test can't work right now, because wrapping rx is not
    // implemented. #[test]
    // fn rmt_loopback_rx_wrap() {
    //     // 80 codes require two RAM blocks; thus an rx channel with only 1 block
    //     // requires wrapping
    //     do_rmt_loopback<80>(2, 1, false);
    // }

    #[test]
    fn rmt_single_shot_wrap() {
        const TX_LEN: usize = 80;

        let tx_config = TxChannelConfig::default()
            .with_clk_divider(255)
            .with_memsize(1);
        let (tx_channel, _) = setup(tx_config, Default::default());

        let mut tx_data = [PulseCode::new(Level::High, 200, Level::Low, 50); TX_LEN];
        tx_data[TX_LEN - 2] = PulseCode::new(Level::High, 3000, Level::Low, 500);
        tx_data[TX_LEN - 1] = PulseCode::empty();

        tx_channel.transmit(&tx_data).unwrap().wait().unwrap();
    }

    #[test]
    fn rmt_single_shot_extended() {
        const TX_LEN: usize = 80;

        let tx_config = TxChannelConfig::default()
            .with_clk_divider(255)
            .with_memsize(2);
        let (tx_channel, _) = setup(tx_config, Default::default());

        let mut tx_data = [PulseCode::new(Level::High, 200, Level::Low, 50); TX_LEN];
        tx_data[TX_LEN - 2] = PulseCode::new(Level::High, 3000, Level::Low, 500);
        tx_data[TX_LEN - 1] = PulseCode::empty();

        tx_channel.transmit(&tx_data).unwrap().wait().unwrap();
    }

    #[test]
    fn rmt_single_shot_extended_wrap() {
        const TX_LEN: usize = 150;

        let tx_config = TxChannelConfig::default()
            .with_clk_divider(255)
            .with_memsize(2);
        let (tx_channel, _) = setup(tx_config, Default::default());

        let mut tx_data = [PulseCode::new(Level::High, 200, Level::Low, 50); TX_LEN];
        tx_data[TX_LEN - 2] = PulseCode::new(Level::High, 3000, Level::Low, 500);
        tx_data[TX_LEN - 1] = PulseCode::empty();

        tx_channel.transmit(&tx_data).unwrap().wait().unwrap();
    }

    #[test]
    fn rmt_single_shot_fails_without_end_marker() {
        let tx_config = TxChannelConfig::default().with_clk_divider(255);
        let (tx_channel, _) = setup(tx_config, Default::default());

        let tx_data = [PulseCode::new(Level::High, 200, Level::Low, 50); 20];

        let tx_transaction = tx_channel.transmit(&tx_data);

        assert!(tx_transaction.is_err());
        assert!(matches!(tx_transaction, Err(Error::EndMarkerMissing)));
    }

    #[test]
    fn rmt_overlapping_ram_fails() {
        use esp_hal::rmt::TxChannelCreator;

        let peripherals = esp_hal::init(esp_hal::Config::default());

        let freq = if cfg!(feature = "esp32h2") {
            Rate::from_mhz(32)
        } else {
            Rate::from_mhz(80)
        };

        let rmt = Rmt::new(peripherals.RMT, freq).unwrap();

        let ch0 = rmt
            .channel0
            .configure(NoPin, TxChannelConfig::default().with_memsize(2));

        let ch1 = rmt.channel1.configure(NoPin, TxChannelConfig::default());

        assert!(ch0.is_ok());
        assert!(matches!(ch1, Err(Error::MemoryBlockNotAvailable)));
    }
}
