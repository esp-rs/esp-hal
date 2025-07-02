//! RMT Loopback Test

//% CHIPS: esp32 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: unstable

#![no_std]
#![no_main]

use esp_hal::{
    DriverMode,
    gpio::{Input, InputPin, Level, NoPin, OutputPin},
    rmt::{
        Channel,
        Error,
        PulseCode,
        Rmt,
        Rx,
        RxChannelConfig,
        Tx,
        TxChannelConfig,
    },
    time::Rate,
};
use hil_test as _;

esp_bootloader_esp_idf::esp_app_desc!();

cfg_if::cfg_if! {
    if #[cfg(esp32h2)] {
        const FREQ: Rate = Rate::from_mhz(32);
        const DIV: u8 = 64;
    } else {
        const FREQ: Rate = Rate::from_mhz(80);
        const DIV: u8 = 160;
    }
}

fn setup<'a, Dm: DriverMode + 'static>(
    rmt: Rmt<'a, Dm>,
    rx: impl InputPin + 'a,
    tx: impl OutputPin + 'a,
    tx_config: TxChannelConfig,
    rx_config: RxChannelConfig,
) -> (Channel<'a, Dm, Tx>, Channel<'a, Dm, Rx>) {
    let tx_channel = rmt
        .channel0
        .configure_tx(tx, tx_config.with_clk_divider(DIV))
        .map_err(|(e, _c)| e)
        .unwrap();

    cfg_if::cfg_if! {
        if #[cfg(any(esp32, esp32s3))] {
            let rx_channel_creator = rmt.channel4.degrade();
        } else {
            let rx_channel_creator = rmt.channel2.degrade();
        }
    };
    let rx_channel = rx_channel_creator
        .configure_rx(rx, rx_config.with_clk_divider(DIV))
        .map_err(|(e, _c)| e)
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

struct TxDataIter {
    remaining: usize,
    i: u16,
    write_end_marker: bool,
}

impl Iterator for TxDataIter {
    type Item = PulseCode;

    fn next(&mut self) -> Option<Self::Item> {
        let code = match self.remaining {
            0 => return None,
            1 if self.write_end_marker => PulseCode::end_marker(),
            2 if self.write_end_marker => PulseCode::new(Level::High, 3000, Level::Low, 500),
            _ => PulseCode::new(
                Level::High,
                (100 + (self.i * 10) % 200) as u16,
                Level::Low,
                50,
            ),
        };

        self.i += 1;
        self.remaining -= 1;

        Some(code)
    }
}

fn check_data_eq(tx: &[PulseCode], rx: &[PulseCode], rx_count: Option<usize>, rx_memsize: usize) {
    // Only checks the buffers; the rx buffer might still contain garbage after a
    // certain index.
    assert_eq!(tx.len(), rx.len(), "tx and rx len mismatch");

    // Last tx code is the stop code, which won't be received
    let mut expected_rx_len = tx.len() - 1;
    // Some device don't support wrapping rx, and will terminate rx when the buffer
    // is full.
    if cfg!(any(esp32, esp32s2)) {
        expected_rx_len = expected_rx_len.min(rx_memsize)
    };

    if let Some(rx_count) = rx_count {
        assert_eq!(
            rx_count, expected_rx_len,
            "unexpected rx count {} != {}",
            rx_count, expected_rx_len,
        );
    }

    let mut errors: usize = 0;
    for (idx, (code_tx, code_rx)) in core::iter::zip(tx, rx).take(expected_rx_len).enumerate() {
        if idx == tx.len() - 2 {
            if !(code_rx.level1() == Level::High && code_rx.length1() == 0) {
                // The second-to-last pulse-code is the one which exceeds the idle threshold and
                // should be received as stop code.
                #[cfg(feature = "defmt")]
                defmt::error!(
                    "rx code not a stop code at index {}: {:?} (tx) -> {:?} (rx)",
                    idx,
                    code_tx,
                    code_rx
                );
                errors += 1;
            }
        } else {
            if code_tx != code_rx {
                #[cfg(feature = "defmt")]
                defmt::error!(
                    "rx code mismatch at index {}: {:?} (tx) != {:?} (rx)",
                    idx,
                    code_tx,
                    code_rx
                );
                errors += 1;
            }
        }
    }

    // The driver shouldn't have touched the rx buffer beyond expected_rx_len, and
    // we initialized the buffer to PulseCod::end_marker()
    assert!(
        rx[expected_rx_len..]
            .iter()
            .all(|c| *c == PulseCode::end_marker()),
        "rx buffer unexpectedly overwritten beyond rx end"
    );

    assert!(
        errors == 0,
        "rx/tx code mismatch at {}/{} indices (First: tx={:?}, rx={:?})",
        errors,
        tx.len() - 2,
        tx.first(),
        rx.first(),
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

    let (mut tx_channel, mut rx_channel) = setup(rmt, rx, tx, tx_config, rx_config);

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
    let rx_count = match rx_transaction.wait() {
        Ok(count) => Some(count),
        #[cfg(any(esp32, esp32s2))]
        Err(Error::ReceiverError) => None,
        Err(e) => {
            panic!("unexpected rx error {:?}", e);
        }
    };

    check_data_eq(&tx_data, &rcv_data, rx_count, rx_channel.buffer_size());
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

    let rx_count = match rx_res {
        Ok(count) => Some(count),
        #[cfg(any(esp32, esp32s2))]
        Err(Error::ReceiverError) => None,
        Err(e) => {
            panic!("unexpected rx error {:?}", e);
        }
    };

    check_data_eq(&tx_data, &rcv_data, rx_count, rx_channel.buffer_size());
}

// Run a test that just sends some data, without trying to receive it. This uses
// an Iterator of PulseCode instead of a slice.
#[must_use = "Tests should fail on errors"]
fn do_rmt_single_shot_iter(
    tx_len: usize,
    tx_memsize: u8,
    write_end_marker: bool,
) -> Result<(), Error> {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let (rx, tx) = hil_test::common_test_pins!(peripherals);
    let rmt = Rmt::new(peripherals.RMT, FREQ).unwrap();

    let tx_config = TxChannelConfig::default()
        .with_clk_divider(DIV)
        .with_memsize(tx_memsize);
    let (mut tx_channel, _) = setup(rmt, rx, tx, tx_config, Default::default());

    let mut tx_data = TxDataIter {
        remaining: tx_len,
        i: 0,
        write_end_marker,
    };
    tx_channel.transmit_iter(&mut tx_data)?.wait()
}

#[must_use = "Tests should fail on errors"]
async fn do_rmt_single_shot_iter_async(
    tx_len: usize,
    tx_memsize: u8,
    write_end_marker: bool,
) -> Result<(), Error> {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let (rx, tx) = hil_test::common_test_pins!(peripherals);
    let rmt = Rmt::new(peripherals.RMT, FREQ).unwrap().into_async();

    let tx_config = TxChannelConfig::default()
        .with_clk_divider(DIV)
        .with_memsize(tx_memsize);
    let (mut tx_channel, _) = setup(rmt, rx, tx, tx_config, Default::default());

    let mut tx_data = TxDataIter {
        remaining: tx_len,
        i: 0,
        write_end_marker,
    };
    tx_channel.transmit_iter(&mut tx_data).await
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
    async fn rmt_loopback_extended_ram_async() {
        // 80 codes require two RAM blocks
        do_rmt_loopback_async::<80>(2, 2).await;
    }

    // FIXME: This test currently fails on esp32 with an rmt::Error::ReceiverError,
    // which should imply a receiver overrun, which is unexpected (the buffer
    // should hold 2 * 64 codes, which is sufficient).
    // However, the problem already exists exists in the original code that added
    // support for extended channel RAM, so we can't bisect to find a
    // regression. Skip the test for now.
    // #[cfg(not(esp32))]
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

    #[test]
    fn rmt_loopback_rx_wrap() {
        // 80 codes require two RAM blocks; thus an rx channel with only 1 block
        // requires wrapping
        do_rmt_loopback::<80>(2, 1);
    }

    #[test]
    async fn rmt_loopback_rx_wrap_async() {
        // 80 codes require two RAM blocks; thus an rx channel with only 1 block
        // requires wrapping
        do_rmt_loopback_async::<80>(2, 1).await;
    }

    #[test]
    fn rmt_loopback_rx_tx_wrap() {
        // 80 codes require two RAM blocks; thus both the rx and tx channel will wrap
        do_rmt_loopback::<80>(1, 1);
    }

    #[test]
    async fn rmt_loopback_rx_tx_wrap_async() {
        // 80 codes require two RAM blocks; thus both the rx and tx channel will wrap
        do_rmt_loopback_async::<80>(1, 1).await;
    }

    #[test]
    async fn rmt_single_shot_simple_async() {
        // 20 codes fit a single RAM block
        do_rmt_single_shot_iter_async(20, 1, true).await.unwrap();
    }

    #[test]
    fn rmt_single_shot_wrap() {
        // Single RAM block (48 or 64 codes), requires wrapping
        do_rmt_single_shot_iter(80, 1, true).unwrap();
    }

    #[test]
    async fn rmt_single_shot_wrap_async() {
        // Single RAM block (48 or 64 codes), requires wrapping
        do_rmt_single_shot_iter_async(80, 1, true).await.unwrap();
    }

    #[test]
    fn rmt_single_shot_extended() {
        // Two RAM blocks (96 or 128 codes), no wrapping
        do_rmt_single_shot_iter(80, 2, true).unwrap();
    }

    #[test]
    fn rmt_single_shot_extended_wrap() {
        // Two RAM blocks (96 or 128 codes), requires wrapping
        do_rmt_single_shot_iter(150, 2, true).unwrap();
    }

    #[test]
    fn rmt_single_shot_fails_without_end_marker() {
        let result = do_rmt_single_shot_iter(20, 1, false);

        assert_eq!(result, Err(Error::EndMarkerMissing));
    }

    #[test]
    fn rmt_overlapping_ram_fail_and_release() {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let rmt = Rmt::new(peripherals.RMT, FREQ).unwrap();

        let ch0 = rmt
            .channel0
            .configure_tx(NoPin, TxChannelConfig::default().with_memsize(2))
            .unwrap();

        // Configuring channel 1 should fail, since channel 0 already uses its memory.
        let ch1 = rmt.channel1.configure_tx(NoPin, TxChannelConfig::default());

        let (err, ch1) = ch1.expect_err("channel configuration unexpectly suceeded");
        assert_eq!(err, Error::MemoryBlockNotAvailable);

        // After dropping channel 0, the memory that it reserved should become available
        // again such that channel 1 configuration succeeds.
        core::mem::drop(ch0);
        let ch1 = ch1
            .configure_tx(NoPin, TxChannelConfig::default())
            .unwrap();
    }

    #[test]
    async fn rmt_async_tx_invalid_args_slice() {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let rmt = Rmt::new(peripherals.RMT, FREQ).unwrap().into_async();

        let mut ch0 = rmt
            .channel0
            .configure_tx(NoPin, TxChannelConfig::default())
            .unwrap();

        let code = PulseCode::new(Level::High, 42, Level::Low, 42);

        let no_end = [code; 80];

        assert_eq!(ch0.transmit(&no_end[..0]).await, Err(Error::InvalidArgument));

        // No wrapping, error should already be detected before starting tx
        assert_eq!(
            ch0.transmit(&no_end[..3]).await,
            Err(Error::EndMarkerMissing)
        );

        // Requires wrapping, error can only be detected after starting tx
        assert_eq!(
            ch0.transmit(&no_end[..80]).await,
            Err(Error::EndMarkerMissing)
        );
    }

    #[test]
    async fn rmt_async_tx_invalid_args_iter() {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let rmt = Rmt::new(peripherals.RMT, FREQ).unwrap().into_async();

        let mut ch0 = rmt
            .channel0
            .configure_tx(NoPin, TxChannelConfig::default())
            .unwrap();

        let code = PulseCode::new(Level::High, 42, Level::Low, 42);

        assert_eq!(
            ch0.transmit_iter(core::iter::repeat(code).take(0)).await,
            Err(Error::InvalidArgument)
        );

        // No wrapping, error should already be detected before starting tx
        assert_eq!(
            ch0.transmit_iter(core::iter::repeat(code).take(3)).await,
            Err(Error::EndMarkerMissing)
        );

        // Requires wrapping, error can only be detected after starting tx
        assert_eq!(
            ch0.transmit_iter(core::iter::repeat(code).take(80)).await,
            Err(Error::EndMarkerMissing)
        );
    }

    #[test]
    async fn rmt_loopback_after_drop() {
        const TX_LEN: usize = 20;

        let peripherals = esp_hal::init(esp_hal::Config::default());
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

        // ...poll them for a while, but then drop them...
        // (`poll_once` takes the future by value and this drops it before returning)
        let _ = embassy_futures::poll_once(rx_fut);
        let _ = embassy_futures::poll_once(tx_fut);

        // ...then start over and check that everything still works as expected (i.e. we
        // didn't leave the hardware in an unexpected state or lock up when
        // dropping the futures).
        let (rx_res, tx_res) = embassy_futures::join::join(
            rx_channel.receive(&mut rcv_data),
            tx_channel.transmit(&tx_data),
        )
        .await;

        tx_res.unwrap();
        let rx_count = rx_res.unwrap();

        check_data_eq(
            &tx_data,
            &rcv_data,
            Some(rx_count),
            rx_channel.buffer_size(),
        );
    }

    #[test]
    async fn rmt_pin_reconfigure() {
        let mut peripherals = esp_hal::init(esp_hal::Config::default());
        let (_rx, mut tx) = hil_test::common_test_pins!(peripherals);

        let ch0 = {
            let mut rmt = Rmt::new(peripherals.RMT.reborrow(), FREQ)
                .unwrap()
                .into_async();

            let ch0 = {
                let mut ch0 = rmt
                    .channel0
                    .reborrow()
                    .configure_tx(tx.reborrow(), TxChannelConfig::default())
                    .unwrap();

                let tx_data: [_; 10] = generate_tx_data(true);

                ch0.transmit(&tx_data).await.unwrap();

                ch0
            };

            // Removing the drop should break compilation!
            core::mem::drop(ch0);
            let _input = Input::new(tx.reborrow(), Default::default());

            {
                // This time take the ChannelCreator by value
                let mut ch0 = rmt
                    .channel0
                    .configure_tx(tx.reborrow(), TxChannelConfig::default())
                    .unwrap();

                let tx_data: [_; 10] = generate_tx_data(true);

                ch0.transmit(&tx_data).await.unwrap();

                ch0
            }

            // let tx_data: [_; 10] = generate_tx_data(1, true);
            // ch0.transmit(&tx_data).await.unwrap();
        };

        // Note that we can keep around the channel longer than the Rmt! This is
        // intended and useful when e.g. creating the Rmt in the main task and
        // then sending several channels to different places.
        let _ = ch0;
    }
}
