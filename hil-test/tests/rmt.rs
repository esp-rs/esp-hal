//! RMT Loopback Test

//% CHIPS: esp32 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: embassy unstable

#![no_std]
#![no_main]

use esp_hal::{
    Blocking,
    DriverMode,
    gpio::{InputPin, Level, NoPin, OutputPin},
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

// RMT channel clock = 500kHz
cfg_if::cfg_if! {
    if #[cfg(esp32h2)] {
        const FREQ: Rate = Rate::from_mhz(32);
        const DIV: u8 = 64;
    } else {
        const FREQ: Rate = Rate::from_mhz(80);
        const DIV: u8 = 160;
    }
}

fn setup<Dm: DriverMode>(
    rmt: Rmt<'static, Dm>,
    rx: impl InputPin,
    tx: impl OutputPin,
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

// Pulses of H 100..300 L 50, i.e. 150..350 / 500kHz = 150..350 * 2us = 300..700us
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

// Most of the time, the codes received in tests match exactly, but every once in a while, a test
// fails with pulse lengths off by one. Allow for that here, there is no hardware synchronization
// between rx/tx that would guarantee them to be identical.
fn pulse_code_matches(left: PulseCode, right: PulseCode) -> bool {
    left.level1() == right.level1()
        && left.level2() == right.level2()
        && left.length1().abs_diff(right.length1()) <= 1
        && left.length2().abs_diff(right.length2()) <= 1
}

// Run tests with a large buffer like `DEFMT_RTT_BUFFER_SIZE=32768 xtask run ...` to avoid
// truncated output when running this with defmt!
// Note that probe-rs reading the buffer might mess up timing-sensitive tests.
fn check_data_eq(
    tx: &[PulseCode],
    rx: &[PulseCode],
    rx_res: Result<usize, Error>,
    rx_memsize: usize,
    rx_wrap: bool,
) {
    // Only checks the buffer sizes; the rx buffer might still contain garbage after a certain
    // index.
    assert_eq!(tx.len(), rx.len(), "tx and rx len mismatch");

    // Last tx code is the stop code, which won't be received
    let mut expected_rx_len = tx.len() - 1;

    // Some device don't support wrapping rx, and will terminate rx when the buffer is full.
    if !rx_wrap {
        expected_rx_len = expected_rx_len.min(rx_memsize)
    };

    let mut errors: usize = 0;
    for (idx, (&code_tx, &code_rx)) in core::iter::zip(tx, rx).enumerate() {
        let mut _msg = "";

        if idx < expected_rx_len {
            if idx == tx.len() - 2 {
                // The second-to-last pulse-code is the one which exceeds the idle threshold and
                // should be received as stop code.
                if !(code_rx.level1() == Level::High && code_rx.length1() == 0) {
                    _msg = "rx code not a stop code!";
                    errors += 1;
                }
            } else if !pulse_code_matches(code_tx, code_rx) {
                _msg = "rx/tx code mismatch!";
                errors += 1;
            }
        }

        #[cfg(feature = "defmt")]
        if _msg.len() > 0 {
            defmt::error!(
                "loopback @ idx {}: {:?} (tx) -> {:?} (rx): {}",
                idx,
                code_tx,
                code_rx,
                _msg
            );
        } else {
            defmt::info!(
                "loopback @ idx {}: {:?} (tx) -> {:?} (rx)",
                idx,
                code_tx,
                code_rx,
            );
        }
    }

    match rx_res {
        Ok(rx_count) => {
            assert_eq!(
                rx_count,
                expected_rx_len,
                "unexpected rx count (last: tx={:?}, rx={:?})",
                tx[rx_count - 1],
                rx[rx_count - 1],
            );
        }
        Err(Error::InvalidDataLength) if !rx_wrap => {
            assert!(tx.len() > rx_memsize);
            return;
        }
        Err(e) => {
            panic!("unexpected rx error {:?}", e);
        }
    }

    // The driver shouldn't have touched the rx buffer beyond expected_rx_len, and
    // we initialized the buffer to PulseCode::end_marker()
    assert!(
        rx[expected_rx_len..]
            .iter()
            .all(|c| *c == PulseCode::end_marker()),
        "rx buffer unexpectedly overwritten beyond rx end"
    );

    // FIXME: Show first mismatch, not first code!
    assert_eq!(
        errors,
        0,
        "rx/tx code mismatch at {}/{} indices (First: tx={:?}, rx={:?})",
        errors,
        tx.len() - 1,
        tx.first(),
        rx.first(),
    );
}

fn do_rmt_loopback_inner<const TX_LEN: usize>(
    tx_channel: Channel<Blocking, Tx>,
    rx_channel: Channel<Blocking, Rx>,
) {
    let tx_data: [_; TX_LEN] = generate_tx_data(true);
    let mut rcv_data: [PulseCode; TX_LEN] = [PulseCode::default(); TX_LEN];

    let supports_rx_wrap = rx_channel.supports_wrap();
    let rx_buffer_size = rx_channel.buffer_size();

    let rx_res = match rx_channel.receive(&mut rcv_data) {
        Ok(mut rx_transaction) => {
            let mut tx_transaction = tx_channel.transmit(&tx_data).unwrap();

            loop {
                let tx_done = tx_transaction.poll();
                let rx_done = rx_transaction.poll();
                if tx_done && rx_done {
                    break;
                }
            }

            tx_transaction.wait().unwrap();
            match rx_transaction.wait() {
                Ok((rx_count, _channel)) => Ok(rx_count),
                Err((err, _channel)) => Err(err),
            }
        }
        Err(e) => Err(e),
    };
    check_data_eq(
        &tx_data,
        &rcv_data,
        rx_res,
        rx_buffer_size,
        supports_rx_wrap,
    );
}

// Run a test where some data is sent from one channel and looped back to
// another one for receive, and verify that the data matches.
fn do_rmt_loopback<const TX_LEN: usize>(tx_memsize: u8, rx_memsize: u8) {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let (rx, tx) = hil_test::common_test_pins!(peripherals);
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
    let (rx, tx) = hil_test::common_test_pins!(peripherals);
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

    check_data_eq(
        &tx_data,
        &rcv_data,
        rx_res,
        rx_channel.buffer_size(),
        rx_channel.supports_wrap(),
    );
}

// Run a test that just sends some data, without trying to recive it.
#[must_use = "Tests should fail on errors"]
fn do_rmt_single_shot<const TX_LEN: usize>(
    tx_memsize: u8,
    write_end_marker: bool,
) -> Result<(), Error> {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let (rx, tx) = hil_test::common_test_pins!(peripherals);
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

    #[test]
    fn rmt_loopback_tx_wrap() {
        // 80 codes require two RAM blocks; thus a tx channel with only 1 block requires
        // wrapping.
        do_rmt_loopback::<80>(1, 2);
    }

    #[test]
    async fn rmt_loopback_tx_wrap_async() {
        do_rmt_loopback_async::<80>(1, 2).await;
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

        assert_eq!(result, Err(Error::EndMarkerMissing));
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
            $tx_pin:ident,
            $rx_pin:ident,
            $tx_channel:ident,
            $rx_channel:ident
        ) => {
            // It's currently not possible to reborrow ChannelCreators and reconfigure channels, so
            // we reconfigure the whole peripheral for each sub-test.
            let rmt = Rmt::new($peripherals.RMT.reborrow(), FREQ).unwrap();

            let tx_config = TxChannelConfig::default();
            let rx_config = RxChannelConfig::default().with_idle_threshold(1000);

            let tx_channel = rmt
                .$tx_channel
                .configure_tx($tx_pin.reborrow(), tx_config)
                .unwrap();

            let rx_channel = rmt
                .$rx_channel
                .configure_rx($rx_pin.reborrow(), rx_config)
                .unwrap();

            do_rmt_loopback_inner::<20>(tx_channel, rx_channel);
        };
    }

    // A simple test that uses all channels: This is useful to verify that all channel/register
    // indexing in the driver is correct. The other tests are prone to hide related issues due to
    // using low channel numbers only (in particular 0 for the tx channel).
    #[test]
    fn rmt_use_all_channels() {
        let mut p = esp_hal::init(esp_hal::Config::default());
        let (mut rx, mut tx) = hil_test::common_test_pins!(p);

        // FIXME: Find a way to implement these with less boilerplate and without a macro.
        // Maybe use ChannelCreator::steal() (doesn't help right now because it uses a const
        // generic channel number)?

        // Chips with combined RxTx channels
        #[cfg(esp32)]
        {
            test_channel_pair!(p, tx, rx, channel0, channel1);
            test_channel_pair!(p, tx, rx, channel0, channel2);
            test_channel_pair!(p, tx, rx, channel0, channel3);
            test_channel_pair!(p, tx, rx, channel0, channel4);
            test_channel_pair!(p, tx, rx, channel0, channel5);
            test_channel_pair!(p, tx, rx, channel0, channel6);
            test_channel_pair!(p, tx, rx, channel0, channel7);

            test_channel_pair!(p, tx, rx, channel1, channel0);
            test_channel_pair!(p, tx, rx, channel2, channel0);
            test_channel_pair!(p, tx, rx, channel3, channel0);
            test_channel_pair!(p, tx, rx, channel4, channel0);
            test_channel_pair!(p, tx, rx, channel5, channel0);
            test_channel_pair!(p, tx, rx, channel6, channel0);
            test_channel_pair!(p, tx, rx, channel7, channel0);
        }

        #[cfg(esp32s2)]
        {
            test_channel_pair!(p, tx, rx, channel0, channel1);
            test_channel_pair!(p, tx, rx, channel0, channel2);
            test_channel_pair!(p, tx, rx, channel0, channel3);

            test_channel_pair!(p, tx, rx, channel1, channel0);
            test_channel_pair!(p, tx, rx, channel2, channel0);
            test_channel_pair!(p, tx, rx, channel3, channel0);
        }

        // Chips with separate Rx and Tx channels
        #[cfg(esp32s3)]
        {
            test_channel_pair!(p, tx, rx, channel0, channel4);
            test_channel_pair!(p, tx, rx, channel1, channel5);
            test_channel_pair!(p, tx, rx, channel2, channel6);
            test_channel_pair!(p, tx, rx, channel3, channel7);
        }

        #[cfg(not(any(esp32, esp32s2, esp32s3)))]
        {
            test_channel_pair!(p, tx, rx, channel0, channel2);
            test_channel_pair!(p, tx, rx, channel1, channel3);
        }
    }

    // Test that dropping rx/tx futures returns the hardware to a predictable state from which
    // subsequent transactions are successful.
    #[test]
    async fn rmt_loopback_after_drop_async() {
        use embassy_time::Delay;
        use embedded_hal_async::delay::DelayNs;

        const TX_LEN: usize = 20;

        let peripherals = esp_hal::init(esp_hal::Config::default());

        hil_test::init_embassy!(peripherals, 2);
        let (rx, tx) = hil_test::common_test_pins!(peripherals);
        let rmt = Rmt::new(peripherals.RMT, FREQ).unwrap().into_async();

        let tx_config = TxChannelConfig::default()
            // If not enabling a defined idle_output level, the output might remain high after
            // dropping the tx future, which will lead to an extra edge being received.
            .with_idle_output(true);
        let rx_config = RxChannelConfig::default().with_idle_threshold(1000);

        let (mut tx_channel, mut rx_channel) = setup(rmt, rx, tx, tx_config, rx_config);

        let tx_data: [_; TX_LEN] = generate_tx_data(true);
        let mut rcv_data: [PulseCode; TX_LEN] = [PulseCode::end_marker(); TX_LEN];

        // Start the transactions...
        let rx_fut = rx_channel.receive(&mut rcv_data);
        let tx_fut = tx_channel.transmit(&tx_data);

        Delay.delay_ms(2).await;

        // ...poll them, but then drop them before completion...
        // (`poll_once` takes the future by value and drops it before returning)
        let rx_poll = embassy_futures::poll_once(rx_fut);
        let tx_poll = embassy_futures::poll_once(tx_fut);

        // The test should fail here when the the delay above is increased, e.g. to 100ms.
        assert!(rx_poll.is_pending());
        assert!(tx_poll.is_pending());

        // ...then start over and check that everything still works as expected (i.e. we
        // didn't leave the hardware in an unexpected state or lock up when
        // dropping the futures).
        let (rx_res, tx_res) = embassy_futures::join::join(
            rx_channel.receive(&mut rcv_data),
            tx_channel.transmit(&tx_data),
        )
        .await;

        tx_res.unwrap();

        check_data_eq(
            &tx_data,
            &rcv_data,
            rx_res,
            rx_channel.buffer_size(),
            rx_channel.supports_wrap(),
        );
    }
}
