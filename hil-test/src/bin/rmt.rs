//! RMT Loopback Test

//% CHIPS: esp32 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: embassy unstable

#![no_std]
#![no_main]

use esp_hal::{
    Blocking,
    DriverMode,
    delay::Delay,
    gpio::{
        Flex,
        InputConfig,
        Level,
        NoPin,
        OutputConfig,
        Pull,
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
#[allow(unused_imports)]
use hil_test::{assert, assert_eq};

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

fn setup<'a, Dm: DriverMode>(
    rmt: Rmt<'a, Dm>,
    rx: impl PeripheralInput<'a>,
    tx: impl PeripheralOutput<'a>,
    tx_config: TxChannelConfig,
    rx_config: RxChannelConfig,
) -> (Channel<'a, Dm, Tx>, Channel<'a, Dm, Rx>) {
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
fn generate_tx_data<const TX_LEN: usize>(
    write_stop_code: bool,
    write_end_marker: bool,
) -> [PulseCode; TX_LEN] {
    let mut tx_data: [_; TX_LEN] = core::array::from_fn(|i| {
        PulseCode::new(Level::High, (100 + (i * 10) % 200) as u16, Level::Low, 50)
    });

    let mut pos = TX_LEN - 1;
    if write_end_marker {
        tx_data[pos] = PulseCode::end_marker();
        pos -= 1;
    }
    if write_stop_code {
        tx_data[pos] = PulseCode::new(Level::High, 3000, Level::Low, 500);
    }

    tx_data
}

// Most of the time, the codes received in tests match exactly, but every once in a while, a test
// fails with pulse lengths off by one. Allow for that here, there is no hardware synchronization
// between rx/tx that would guarantee them to be identical.
fn pulse_code_matches(left: PulseCode, right: PulseCode, tolerance: u16) -> bool {
    left.level1() == right.level1()
        && left.level2() == right.level2()
        && left.length1().abs_diff(right.length1()) <= tolerance
        && left.length2().abs_diff(right.length2()) <= tolerance
}

// When running this with defmt, consider increasing embedded_test's default_timeout below to avoid
// timeouts while printing. Note that probe-rs reading the buffer might still mess up
// timing-sensitive tests! This doesn't apply to `check_data_eq` since it is used after the action,
// but adding any additional logging to the driver is likely to cause sporadic issues.
// FIXME: Add a variant that just check the data, and one that does additional verification for
// regular loopback transactions!
fn check_data_eq(
    tx: &[PulseCode],
    rx: &[PulseCode],
    idle_threshold: u16,
    tolerance: u16,
    rx_res: Result<usize, Error>,
    rx_memsize: usize,
    rx_wrap: bool,
) {
    let mut errors: usize = 0;
    let mut count: usize = 0;

    // The last tx code might be a stop code, which won't be received; but we skip that below
    let mut expected_rx_len = tx.len();

    for (idx, (&code_tx, &code_rx)) in core::iter::zip(tx, rx).enumerate() {
        let mut done = false;
        let mut _msg = "";
        if code_tx.length1() == 0 {
            // Sent an end marker
            break;
        } else if code_tx.length2() == 0 {
            // Sent an end marker in the second PulseCode field
            done = true;
            if !pulse_code_matches(code_tx, code_rx.with_length2(0).unwrap(), tolerance) {
                errors += 1;
                _msg = "rx/tx code mismatch!";
            }
        } else if code_tx.length1() > idle_threshold {
            // Should receive an end marker in the first PulseCode field
            done = true;
            if !code_rx.length1() == 0 && code_tx.level1() == code_rx.level1() {
                errors += 1;
                _msg = "rx code not a stop code!";
            }
        } else if code_tx.length2() > idle_threshold {
            // Should receive an end marker in the second PulseCode field
            done = true;
            if !pulse_code_matches(code_tx.with_length2(0).unwrap(), code_rx, tolerance) {
                errors += 1;
                _msg = "rx code not a stop code!";
            }
        } else if !pulse_code_matches(code_tx, code_rx, tolerance) {
            // Regular pulse code, should be received exactly
            errors += 1;
            _msg = "rx/tx code mismatch!";
        };

        count += 1;

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

        if done {
            expected_rx_len = idx + 1;
            break;
        }
    }

    // Some device don't support wrapping rx, and will terminate rx when the buffer is full.
    if !rx_wrap {
        expected_rx_len = expected_rx_len.min(rx_memsize)
    };

    match rx_res {
        Ok(rx_count) => {
            assert_eq!(
                // FIXME: This is now incorrect! At least for cont. tx
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
    assert!(
        errors == 0,
        "rx/tx code mismatch at {}/{} indices (First: tx={:?}, rx={:?})",
        errors,
        count,
        tx.first(),
        rx.first(),
    );
}

fn do_rmt_loopback_inner<const TX_LEN: usize>(
    tx_channel: Channel<Blocking, Tx>,
    rx_channel: Channel<Blocking, Rx>,
) {
    let supports_rx_wrap = rx_channel.supports_wrap();
    let rx_buffer_size = rx_channel.buffer_size();

    let tx_data: [_; TX_LEN] = generate_tx_data(true, true);
    let mut rcv_data: [PulseCode; TX_LEN] = [PulseCode::default(); TX_LEN];

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
        1000,
        1,
        rx_res,
        rx_buffer_size,
        supports_rx_wrap,
    );
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

    let tx_data: [_; TX_LEN] = generate_tx_data(true, true);
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
        1000,
        1,
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
    let (_, pin) = hil_test::common_test_pins!(peripherals);
    let (rx, tx) = Flex::new(pin).split();
    let rmt = Rmt::new(peripherals.RMT, FREQ).unwrap();

    let tx_config = TxChannelConfig::default()
        .with_clk_divider(DIV)
        .with_memsize(tx_memsize);
    let (tx_channel, _) = setup(rmt, rx, tx, tx_config, Default::default());

    let tx_data: [_; TX_LEN] = generate_tx_data(write_end_marker, write_end_marker);

    tx_channel.transmit(&tx_data)?.wait().map_err(|(e, _)| e)?;

    Ok(())
}

#[embedded_test::tests(default_timeout = 5, executor = hil_test::Executor::new())]
mod tests {
    #[allow(unused_imports)]
    use hil_test::{assert, assert_eq};

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

    // Test that the timing is at least roughly correct (i.e. the clock source isn't completely
    // misconfigured).
    // This test is almost guaranteed to fail when using defmt!
    #[test]
    fn rmt_clock_rate() {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let (_, pin) = hil_test::common_test_pins!(peripherals);
        let mut pin = Flex::new(pin);
        pin.set_input_enable(true);
        pin.set_output_enable(true);
        pin.apply_input_config(&InputConfig::default().with_pull(Pull::None));
        pin.apply_output_config(&OutputConfig::default());
        let rx_pin = pin.peripheral_input();
        let mut tx_pin = pin;

        tx_pin.set_low();

        let rmt = Rmt::new(peripherals.RMT, FREQ).unwrap();

        let tx_config = TxChannelConfig::default();
        // idle threshold 16383 cycles = ~33ms
        let rx_config = RxChannelConfig::default().with_idle_threshold(0x3FFF);

        let (_, rx_channel) = setup(rmt, rx_pin, NoPin, tx_config, rx_config);

        let supports_rx_wrap = rx_channel.supports_wrap();
        let rx_buffer_size = rx_channel.buffer_size();

        let mut rx_data = [PulseCode::default(); 6];
        let rx_transaction = rx_channel.receive(&mut rx_data).unwrap();

        let mut expected: [_; 5] = core::array::from_fn(|i| {
            let i = i as u16 + 1;
            // 1 cycle = 2us
            PulseCode::new(Level::High, 500 * i, Level::Low, 500 * i)
        });
        *expected.last_mut().unwrap() = PulseCode::end_marker();

        let delay = Delay::new();
        for i in 1..5 {
            tx_pin.set_high();
            delay.delay_millis(i);
            tx_pin.set_low();
            delay.delay_millis(i);
        }

        tx_pin.set_high();
        delay.delay_millis(100);
        tx_pin.set_low();

        let rx_res = rx_transaction.wait()
            .map(|(x, _)| x)
            .map_err(|(e, _)| e);

        // tolerance 25 cycles == 50us
        check_data_eq(
            &expected,
            &rx_data,
            0x3FFF,
            25,
            rx_res,
            rx_buffer_size,
            supports_rx_wrap,
        );
    }

    // Compile-only test to verify that reborrowing channel creators works.
    async fn _rmt_reborrow_channel_creator() {
        let mut peripherals = esp_hal::init(esp_hal::Config::default());

        let mut rmt = Rmt::new(peripherals.RMT.reborrow(), FREQ)
            .unwrap()
            .into_async();

        for _ in 0..2 {
            let mut ch0 = rmt
                .channel0
                .reborrow()
                .configure_tx(NoPin, TxChannelConfig::default())
                .unwrap();

            let tx_data: [_; 10] = generate_tx_data(true, true);

            ch0.transmit(&tx_data).await.unwrap();
        }
    }

    // Test that dropping rx/tx transactions returns the hardware to a predictable state from which
    // subsequent transactions are successful.
    #[test]
    fn rmt_loopback_after_drop_blocking() {
        const TX_LEN: usize = 40;

        let mut peripherals = esp_hal::init(esp_hal::Config::default());

        let rmt = Rmt::new(peripherals.RMT.reborrow(), FREQ).unwrap();

        let (_, pin) = hil_test::common_test_pins!(peripherals);
        let pin = Flex::new(pin);
        let (rx_pin, tx_pin) = pin.split();

        let tx_config = TxChannelConfig::default()
            // If not enabling a defined idle_output level, the output might remain high after
            // dropping the tx future, which will lead to an extra edge being received.
            .with_idle_output(true);
        let rx_config = RxChannelConfig::default().with_idle_threshold(1000);

        let (mut tx_channel, mut rx_channel) = setup(rmt, rx_pin, tx_pin, tx_config, rx_config);

        let tx_data: [_; TX_LEN] = generate_tx_data(true, true);
        let mut rcv_data: [PulseCode; TX_LEN] = [PulseCode::end_marker(); TX_LEN];

        // Start the transactions...
        let mut rx_transaction = rx_channel.reborrow().receive(&mut rcv_data).unwrap();
        let mut tx_transaction = tx_channel.reborrow().transmit(&tx_data).unwrap();

        Delay::new().delay_millis(2);

        // ...poll them for a while, but then drop them...
        // (`poll_once` takes the future by value and drops it before returning)
        let rx_done = rx_transaction.poll();
        let tx_done = tx_transaction.poll();

        // The test should fail here when the the delay above is increased, e.g. to 100ms.
        assert!(!rx_done);
        assert!(!tx_done);

        // Removing these lines should fail to compile!
        core::mem::drop(rx_transaction);
        core::mem::drop(tx_transaction);

        rcv_data.fill(PulseCode::default());
        let mut rx_transaction = rx_channel.reborrow().receive(&mut rcv_data).unwrap();
        let mut tx_transaction = tx_channel.reborrow().transmit(&tx_data).unwrap();

        loop {
            let tx_done = tx_transaction.poll();
            let rx_done = rx_transaction.poll();
            if tx_done && rx_done {
                break;
            }
        }

        tx_transaction.wait().unwrap();
        let rx_res = rx_transaction.wait()
            .map(|(x, _)| x)
            .map_err(|(e, _)| e);

        check_data_eq(
            &tx_data,
            &rcv_data,
            1000,
            1,
            rx_res,
            rx_channel.buffer_size(),
            rx_channel.supports_wrap(),
        );
    }

    // Test that completed blocking transactions leave the hardware in a predictable state from
    // which subsequent transactions are successful.
    #[test]
    fn rmt_loopback_repeat_blocking() {
        const TX_LEN: usize = 20;

        let mut peripherals = esp_hal::init(esp_hal::Config::default());

        let rmt = Rmt::new(peripherals.RMT.reborrow(), FREQ).unwrap();

        let (_, pin) = hil_test::common_test_pins!(peripherals);
        let pin = Flex::new(pin);
        let (rx_pin, tx_pin) = pin.split();

        let tx_config = TxChannelConfig::default()
            // If not enabling a defined idle_output level, the output might remain high after
            // dropping the tx future, which will lead to an extra edge being received.
            .with_idle_output(true);
        let rx_config = RxChannelConfig::default().with_idle_threshold(1000);

        let (mut tx_channel, mut rx_channel) = setup(rmt, rx_pin, tx_pin, tx_config, rx_config);

        let tx_data: [_; TX_LEN] = generate_tx_data(true, true);
        let mut rcv_data: [PulseCode; TX_LEN] = [PulseCode::end_marker(); TX_LEN];

        for _ in 0..3 {
            rcv_data.fill(PulseCode::default());
            let mut rx_transaction = rx_channel.reborrow().receive(&mut rcv_data).unwrap();
            let mut tx_transaction = tx_channel.reborrow().transmit(&tx_data).unwrap();

            loop {
                let tx_done = tx_transaction.poll();
                let rx_done = rx_transaction.poll();
                if tx_done && rx_done {
                    break;
                }
            }

            tx_transaction.wait().unwrap();
            let rx_res = rx_transaction.wait()
                .map(|(x, _)| x)
                .map_err(|(e, _)| e);

            check_data_eq(
                &tx_data,
                &rcv_data,
                1000,
                1,
                rx_res,
                rx_channel.buffer_size(),
                rx_channel.supports_wrap(),
            );
        }
    }

    // Test that dropping rx/tx futures returns the hardware to a predictable state from which
    // subsequent transactions are successful.
    #[test]
    async fn rmt_loopback_after_drop_async() {
        use embassy_time::Delay;
        use embedded_hal_async::delay::DelayNs;

        const TX_LEN: usize = 40;

        let peripherals = esp_hal::init(esp_hal::Config::default());

        hil_test::init_embassy!(peripherals, 2);

        let rmt = Rmt::new(peripherals.RMT, FREQ).unwrap().into_async();

        let (_, pin) = hil_test::common_test_pins!(peripherals);
        let pin = Flex::new(pin);
        let (rx_pin, tx_pin) = pin.split();

        let tx_config = TxChannelConfig::default()
            // If not enabling a defined idle_output level, the output might remain high after
            // dropping the tx future, which will lead to an extra edge being received.
            .with_idle_output(true);
        let rx_config = RxChannelConfig::default().with_idle_threshold(1000);

        let (mut tx_channel, mut rx_channel) = setup(rmt, rx_pin, tx_pin, tx_config, rx_config);

        let tx_data: [_; TX_LEN] = generate_tx_data(true, true);
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
        rcv_data.fill(PulseCode::default());
        let (rx_res, tx_res) = embassy_futures::join::join(
            rx_channel.receive(&mut rcv_data),
            tx_channel.transmit(&tx_data),
        )
        .await;

        tx_res.unwrap();

        check_data_eq(
            &tx_data,
            &rcv_data,
            1000,
            1,
            rx_res,
            rx_channel.buffer_size(),
            rx_channel.supports_wrap(),
        );
    }

    // Test that completed async transactions leave the hardware in a predictable state from which
    // subsequent transactions are successful.
    #[test]
    async fn rmt_loopback_repeat_async() {
        const TX_LEN: usize = 40;

        let peripherals = esp_hal::init(esp_hal::Config::default());

        hil_test::init_embassy!(peripherals, 2);

        let rmt = Rmt::new(peripherals.RMT, FREQ).unwrap().into_async();

        let (_, pin) = hil_test::common_test_pins!(peripherals);
        let pin = Flex::new(pin);
        let (rx_pin, tx_pin) = pin.split();

        let tx_config = TxChannelConfig::default()
            // If not enabling a defined idle_output level, the output might remain high after
            // dropping the tx future, which will lead to an extra edge being received.
            .with_idle_output(true);
        let rx_config = RxChannelConfig::default().with_idle_threshold(1000);

        let (mut tx_channel, mut rx_channel) = setup(rmt, rx_pin, tx_pin, tx_config, rx_config);

        let tx_data: [_; TX_LEN] = generate_tx_data(true, true);
        let mut rcv_data: [PulseCode; TX_LEN] = [PulseCode::end_marker(); TX_LEN];

        for _ in 0..3 {
            rcv_data.fill(PulseCode::default());
            let (rx_res, tx_res) = embassy_futures::join::join(
                rx_channel.receive(&mut rcv_data),
                tx_channel.transmit(&tx_data),
            )
            .await;

            tx_res.unwrap();

            check_data_eq(
                &tx_data,
                &rcv_data,
                1000,
                1,
                rx_res,
                rx_channel.buffer_size(),
                rx_channel.supports_wrap(),
            );
        }
    }

    #[cfg(not(esp32))]
    fn _rmt_loopback_continuous_tx_impl(loopstop: esp_hal::rmt::LoopStop) {
        use esp_hal::rmt::{LoopCount, LoopStop};

        const TX_COUNT: usize = 10;
        const MAX_LOOP_COUNT: usize = 3;
        const MAX_RX_LEN: usize = (MAX_LOOP_COUNT + 1) * TX_COUNT;

        let mut peripherals = esp_hal::init(esp_hal::Config::default());

        let rmt = Rmt::new(peripherals.RMT.reborrow(), FREQ).unwrap();

        let (_, pin) = hil_test::common_test_pins!(peripherals);
        let pin = Flex::new(pin);
        let (rx_pin, tx_pin) = pin.split();

        let tx_config = TxChannelConfig::default()
            .with_idle_output(true)
            .with_idle_output_level(Level::Low);
        let rx_config = RxChannelConfig::default()
            .with_idle_threshold(1000)
            .with_memsize(2);

        let (mut tx_channel, mut rx_channel) = setup(rmt, rx_pin, tx_pin, tx_config, rx_config);

        let tx_data: [_; TX_COUNT + 1] = generate_tx_data(false, true);
        let mut rx_data: [PulseCode; MAX_RX_LEN] = [Default::default(); MAX_RX_LEN];
        let mut expected_data: [PulseCode; MAX_RX_LEN] = [Default::default(); MAX_RX_LEN];

        for loopcount in 1..MAX_LOOP_COUNT {
            rx_data.fill(PulseCode::default());
            let rx_transaction = rx_channel.reborrow().receive(&mut rx_data).unwrap();
            let tx_transaction = tx_channel
                .reborrow()
                .transmit_continuously(
                    &tx_data,
                    LoopCount::Finite((loopcount as u16).try_into().unwrap()),
                    loopstop,
                )
                .unwrap();

            // All data is small enough to fit a single hardware buffer, so we don't need to poll
            // rx/tx concurrently.
            while !tx_transaction.is_loopcount_interrupt_set() {}

            let rx_res = if loopstop == LoopStop::Manual {
                tx_transaction.stop_next().unwrap();
                rx_transaction.wait()
            } else {
                // tx should stop automatically, so rx should also stop before we explicitly stop
                // tx
                let res = rx_transaction.wait();
                tx_transaction.stop_next().unwrap();
                res
            };
            let rx_res = rx_res
                .map(|(x, _)| x)
                .map_err(|(e, _)| e);

            // FIXME: Somehow verify that tx is really stopped!

            let rx_loopcount = if loopstop == LoopStop::Manual {
                loopcount + 1
            } else {
                loopcount
            };
            expected_data.fill(PulseCode::default());
            for i in 0..rx_loopcount {
                let i0 = i * TX_COUNT;
                expected_data[i0..i0 + TX_COUNT].copy_from_slice(&tx_data[..TX_COUNT]);
            }
            // We don't transmit an explicit code that will exceed the idle threshold, rather the
            // final code will simply be extended via the idle output level of the transmitter.
            expected_data[rx_loopcount * TX_COUNT - 1] = expected_data[rx_loopcount * TX_COUNT - 1]
                .with_length2(1100)
                .unwrap();

            check_data_eq(
                &expected_data,
                &rx_data,
                1000,
                1,
                rx_res,
                rx_channel.buffer_size(),
                rx_channel.supports_wrap(),
            );
        }
    }

    // Test that continuous tx with a finite loopcount works as expected when using manual
    // stopping (on devices that have the required hardware support).
    #[cfg(not(esp32))]
    #[test]
    fn rmt_loopback_continuous_tx_manual_stop() {
        _rmt_loopback_continuous_tx_impl(esp_hal::rmt::LoopStop::Manual);
    }

    // Test that continuous tx with a finite loopcount works as expected when using automatic
    // stopping (on devices that have the required hardware support).
    #[cfg(any(esp32c6, esp32h2, esp32s3))]
    #[test]
    fn rmt_loopback_continuous_tx_auto_stop() {
        _rmt_loopback_continuous_tx_impl(esp_hal::rmt::LoopStop::Auto);
    }
}
