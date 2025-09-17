//! RMT Loopback Test

//% CHIPS: esp32 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: unstable

#![no_std]
#![no_main]

use esp_hal::{
    Blocking,
    DriverMode,
    gpio::{
        Flex,
        Level,
        NoPin,
        interconnect::{PeripheralInput, PeripheralOutput},
    },
    rmt::{
        Channel,
        Error,
        PulseCode,
        Rmt,
        Rx,
        RxChannelConfig,
        RxChannelCreator,
        Tx,
        TxChannelConfig,
        TxChannelCreator,
    },
    time::Rate,
};
use hil_test as _;

cfg_if::cfg_if! {
    if #[cfg(esp32h2)] {
        const FREQ: Rate = Rate::from_mhz(32);
        const DIV: u8 = 64;
    } else {
        const FREQ: Rate = Rate::from_mhz(80);
        const DIV: u8 = 160;
    }
}

fn setup<'a, Dm: DriverMode>(
    rmt: Rmt<'static, Dm>,
    rx: impl PeripheralInput<'a>,
    tx: impl PeripheralOutput<'a>,
    tx_config: TxChannelConfig,
    rx_config: RxChannelConfig,
) -> (Channel<Dm, Tx>, Channel<Dm, Rx>) {
    let tx_channel = rmt
        .channel0
        .configure_tx(tx, tx_config.with_clk_divider(DIV))
        .unwrap();

    cfg_if::cfg_if! {
        if #[cfg(any(esp32, esp32s3))] {
            let rx_channel_creator = rmt.channel4;
        } else {
            let rx_channel_creator = rmt.channel2;
        }
    };
    let rx_channel = rx_channel_creator
        .configure_rx(rx, rx_config.with_clk_divider(DIV))
        .unwrap();

    (tx_channel, rx_channel)
}

fn generate_tx_data<const TX_LEN: usize>(write_end_marker: bool) -> [PulseCode; TX_LEN] {
    let mut tx_data: [_; TX_LEN] = core::array::from_fn(|i| {
        PulseCode::new(Level::High, (100 + (i * 10) % 200) as u16, Level::Low, 50)
    });

    if write_end_marker {
        tx_data[TX_LEN - 2] = PulseCode::new(Level::High, 3000, Level::Low, 500);
        tx_data[TX_LEN - 1] = PulseCode::end_marker();
    }

    tx_data
}

fn do_rmt_loopback_inner<const TX_LEN: usize>(
    tx_channel: Channel<Blocking, Tx>,
    rx_channel: Channel<Blocking, Rx>,
) {
    let tx_data: [_; TX_LEN] = generate_tx_data(true);
    let mut rcv_data: [PulseCode; TX_LEN] = [PulseCode::default(); TX_LEN];

    let mut rx_transaction = rx_channel.receive(&mut rcv_data).unwrap();
    let mut tx_transaction = tx_channel.transmit(&tx_data).unwrap();

    loop {
        let tx_done = tx_transaction.poll();
        let rx_done = rx_transaction.poll();
        if tx_done && rx_done {
            break;
        }
    }

    tx_transaction.wait().unwrap();
    rx_transaction.wait().unwrap();

    // the last two pulse-codes are the ones which wait for the timeout so
    // they can't be equal
    assert_eq!(&tx_data[..TX_LEN - 2], &rcv_data[..TX_LEN - 2]);
}

// Run a test where some data is sent from one channel and looped back to
// another one for receive, and verify that the data matches.
fn do_rmt_loopback<const TX_LEN: usize>(tx_memsize: u8, rx_memsize: u8) {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let (_, pin) = hil_test::common_test_pins!(peripherals);
    let (rx, tx) = Flex::new(pin).split();
    let rmt = Rmt::new(peripherals.RMT, FREQ).unwrap();

    let tx_config = TxChannelConfig::default().with_memsize(tx_memsize);
    let rx_config = RxChannelConfig::default()
        .with_idle_threshold(1000)
        .with_memsize(rx_memsize);

    let (tx_channel, rx_channel) = setup(rmt, rx, tx, tx_config, rx_config);

    do_rmt_loopback_inner::<TX_LEN>(tx_channel, rx_channel);
}

// Run a test where some data is sent from one channel and looped back to
// another one for receive, and verify that the data matches.
async fn do_rmt_loopback_async<const TX_LEN: usize>(tx_memsize: u8, rx_memsize: u8) {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let (_, pin) = hil_test::common_test_pins!(peripherals);
    let (rx, tx) = Flex::new(pin).split();
    let rmt = Rmt::new(peripherals.RMT, FREQ).unwrap().into_async();

    let tx_config = TxChannelConfig::default().with_memsize(tx_memsize);
    let rx_config = RxChannelConfig::default()
        .with_idle_threshold(1000)
        .with_memsize(rx_memsize);

    let (mut tx_channel, mut rx_channel) = setup(rmt, rx, tx, tx_config, rx_config);

    let tx_data: [_; TX_LEN] = generate_tx_data(true);
    let mut rcv_data: [PulseCode; TX_LEN] = [PulseCode::default(); TX_LEN];

    let (rx_res, tx_res) = embassy_futures::join::join(
        rx_channel.receive(&mut rcv_data),
        tx_channel.transmit(&tx_data),
    )
    .await;

    tx_res.unwrap();
    rx_res.unwrap();

    // the last two pulse-codes are the ones which wait for the timeout so
    // they can't be equal
    assert_eq!(&tx_data[..TX_LEN - 2], &rcv_data[..TX_LEN - 2]);
}

// Run a test that just sends some data, without trying to recive it.
#[must_use = "Tests should fail on errors"]
fn do_rmt_single_shot<const TX_LEN: usize>(
    tx_memsize: u8,
    write_end_marker: bool,
) -> Result<(), Error> {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let (_, pin) = hil_test::common_test_pins!(peripherals);
    let (rx, tx) = Flex::new(pin).split();
    let rmt = Rmt::new(peripherals.RMT, FREQ).unwrap();

    let tx_config = TxChannelConfig::default()
        .with_clk_divider(DIV)
        .with_memsize(tx_memsize);
    let (tx_channel, _) = setup(rmt, rx, tx, tx_config, Default::default());

    let tx_data: [_; TX_LEN] = generate_tx_data(write_end_marker);

    tx_channel.transmit(&tx_data)?.wait().map_err(|(e, _)| e)?;

    Ok(())
}

#[cfg(test)]
#[embedded_test::tests(default_timeout = 1, executor = hil_test::Executor::new())]
mod tests {
    use super::*;

    #[init]
    fn init() {}

    #[test]
    fn rmt_loopback_simple() {
        // 20 codes fit a single RAM block
        do_rmt_loopback::<20>(1, 1);
    }

    #[test]
    async fn rmt_loopback_simple_async() {
        // 20 codes fit a single RAM block
        do_rmt_loopback_async::<20>(1, 1).await;
    }
    #[test]
    fn rmt_loopback_extended_ram() {
        // 80 codes require two RAM blocks
        do_rmt_loopback::<80>(2, 2);
    }

    // FIXME: This test currently fails on esp32 with an rmt::Error::ReceiverError,
    // which should imply a receiver overrun, which is unexpected (the buffer
    // should hold 2 * 64 codes, which is sufficient).
    // However, the problem already exists exists in the original code that added
    // support for extended channel RAM, so we can't bisect to find a
    // regression. Skip the test for now.
    #[cfg(not(esp32))]
    #[test]
    fn rmt_loopback_tx_wrap() {
        // 80 codes require two RAM blocks; thus a tx channel with only 1 block requires
        // wrapping.
        do_rmt_loopback::<80>(1, 2);
    }

    // FIXME: This test can't work right now, because wrapping rx is not
    // implemented.
    //
    // #[test]
    // fn rmt_loopback_rx_wrap() {
    //     // 80 codes require two RAM blocks; thus an rx channel with only 1 block
    //     // requires wrapping
    //     do_rmt_loopback<80>(2, 1);
    // }

    #[test]
    fn rmt_single_shot_wrap() {
        // Single RAM block (48 or 64 codes), requires wrapping
        do_rmt_single_shot::<80>(1, true).unwrap();
    }

    #[test]
    fn rmt_single_shot_extended() {
        // Two RAM blocks (96 or 128 codes), no wrapping
        do_rmt_single_shot::<80>(2, true).unwrap();
    }

    #[test]
    fn rmt_single_shot_extended_wrap() {
        // Two RAM blocks (96 or 128 codes), requires wrapping
        do_rmt_single_shot::<150>(2, true).unwrap();
    }

    #[test]
    fn rmt_single_shot_fails_without_end_marker() {
        let result = do_rmt_single_shot::<20>(1, false);

        assert!(matches!(result, Err(Error::EndMarkerMissing)));
    }

    #[test]
    fn rmt_overlapping_ram_fails() {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let rmt = Rmt::new(peripherals.RMT, FREQ).unwrap();

        let _ch0 = rmt
            .channel0
            .configure_tx(NoPin, TxChannelConfig::default().with_memsize(2))
            .unwrap();

        // Configuring channel 1 should fail, since channel 0 already uses its memory.
        let ch1 = rmt.channel1.configure_tx(NoPin, TxChannelConfig::default());

        assert!(matches!(ch1, Err(Error::MemoryBlockNotAvailable)));
    }

    #[test]
    fn rmt_overlapping_ram_release() {
        use esp_hal::rmt::TxChannelCreator;

        let peripherals = esp_hal::init(esp_hal::Config::default());

        let rmt = Rmt::new(peripherals.RMT, FREQ).unwrap();

        let ch0 = rmt
            .channel0
            .configure_tx(NoPin, TxChannelConfig::default().with_memsize(2))
            .unwrap();

        // After dropping channel 0, the memory that it reserved should become available
        // again such that channel 1 configuration succeeds.
        core::mem::drop(ch0);
        rmt.channel1
            .configure_tx(NoPin, TxChannelConfig::default())
            .unwrap();
    }

    macro_rules! test_channel_pair {
        (
            $peripherals:ident,
            $pin:ident,
            $tx_channel:ident,
            $rx_channel:ident
        ) => {
            // It's currently not possible to reborrow ChannelCreators and reconfigure channels, so
            // we reconfigure the whole peripheral for each sub-test.
            let rmt = Rmt::new($peripherals.RMT.reborrow(), FREQ).unwrap();

            let tx_config = TxChannelConfig::default();
            let rx_config = RxChannelConfig::default().with_idle_threshold(1000);

            let pin = Flex::new($pin.reborrow());
            let (rx_pin, tx_pin) = pin.split();

            let tx_channel = rmt.$tx_channel.configure_tx(tx_pin, tx_config).unwrap();

            let rx_channel = rmt.$rx_channel.configure_rx(rx_pin, rx_config).unwrap();

            do_rmt_loopback_inner::<20>(tx_channel, rx_channel);
        };
    }

    // A simple test that uses all channels: This is useful to verify that all channel/register
    // indexing in the driver is correct. The other tests are prone to hide related issues due to
    // using low channel numbers only (in particular 0 for the tx channel).
    #[test]
    fn rmt_use_all_channels() {
        let mut p = esp_hal::init(esp_hal::Config::default());
        let (_, mut pin) = hil_test::common_test_pins!(p);

        // FIXME: Find a way to implement these with less boilerplate and without a macro.
        // Maybe use ChannelCreator::steal() (doesn't help right now because it uses a const
        // generic channel number)?

        // Chips with combined RxTx channels
        #[cfg(esp32)]
        {
            test_channel_pair!(p, pin, channel0, channel1);
            test_channel_pair!(p, pin, channel0, channel2);
            test_channel_pair!(p, pin, channel0, channel3);
            test_channel_pair!(p, pin, channel0, channel4);
            test_channel_pair!(p, pin, channel0, channel5);
            test_channel_pair!(p, pin, channel0, channel6);
            test_channel_pair!(p, pin, channel0, channel7);

            test_channel_pair!(p, pin, channel1, channel0);
            test_channel_pair!(p, pin, channel2, channel0);
            test_channel_pair!(p, pin, channel3, channel0);
            test_channel_pair!(p, pin, channel4, channel0);
            test_channel_pair!(p, pin, channel5, channel0);
            test_channel_pair!(p, pin, channel6, channel0);
            test_channel_pair!(p, pin, channel7, channel0);
        }

        #[cfg(esp32s2)]
        {
            test_channel_pair!(p, pin, channel0, channel1);
            test_channel_pair!(p, pin, channel0, channel2);
            test_channel_pair!(p, pin, channel0, channel3);

            test_channel_pair!(p, pin, channel1, channel0);
            test_channel_pair!(p, pin, channel2, channel0);
            test_channel_pair!(p, pin, channel3, channel0);
        }

        // Chips with separate Rx and Tx channels
        #[cfg(esp32s3)]
        {
            test_channel_pair!(p, pin, channel0, channel4);
            test_channel_pair!(p, pin, channel1, channel5);
            test_channel_pair!(p, pin, channel2, channel6);
            test_channel_pair!(p, pin, channel3, channel7);
        }

        #[cfg(not(any(esp32, esp32s2, esp32s3)))]
        {
            test_channel_pair!(p, pin, channel0, channel2);
            test_channel_pair!(p, pin, channel1, channel3);
        }
    }
}
