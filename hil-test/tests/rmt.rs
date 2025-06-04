//! RMT Loopback Test

//% CHIPS: esp32 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: unstable

#![no_std]
#![no_main]

use esp_hal::{
    Async,
    Blocking,
    DriverMode,
    gpio::{Level, NoPin},
    rmt::{
        AnyRxChannel,
        AnyTxChannel,
        Error,
        PulseCode,
        Rmt,
        RxChannelConfig,
        RxChannelCreator,
        TxChannelConfig,
        TxChannelCreator,
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

trait WithMode<'rmt, Dm: DriverMode> {
    fn with_mode(self) -> Rmt<'rmt, Dm>;
}

impl<'rmt> WithMode<'rmt, Blocking> for Rmt<'rmt, Blocking> {
    fn with_mode(self) -> Rmt<'rmt, Blocking> {
        self
    }
}

impl<'rmt> WithMode<'rmt, Async> for Rmt<'rmt, Blocking> {
    fn with_mode(self) -> Rmt<'rmt, Async> {
        self.into_async()
    }
}

fn setup<Dm: DriverMode>(
    tx_config: TxChannelConfig,
    rx_config: RxChannelConfig,
) -> (AnyTxChannel<Dm>, AnyRxChannel<Dm>)
where
    for<'rmt> Rmt<'rmt, Blocking>: WithMode<'rmt, Dm>,
{
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let rmt = Rmt::new(peripherals.RMT, FREQ).unwrap();
    let rmt = <Rmt<Blocking> as WithMode<'_, Dm>>::with_mode(rmt);

    let (rx, tx) = hil_test::common_test_pins!(peripherals);

    let tx_channel = rmt
        .channel0
        .configure_tx(tx, tx_config.with_clk_divider(DIV))
        .unwrap()
        .degrade();

    cfg_if::cfg_if! {
        if #[cfg(any(esp32, esp32s3))] {
            let rx_channel_creator = rmt.channel4;
        } else {
            let rx_channel_creator = rmt.channel2;
        }
    };
    let rx_channel = rx_channel_creator
        .configure_rx(rx, rx_config.with_clk_divider(DIV))
        .unwrap()
        .degrade();

    (tx_channel, rx_channel)
}

fn generate_tx_data<const TX_LEN: usize>(
    scale: u16,
    write_end_marker: bool,
) -> [PulseCode; TX_LEN] {
    let mut tx_data: [_; TX_LEN] = core::array::from_fn(|i| {
        PulseCode::new(
            Level::High,
            scale * (100 + (i * 10) % 200) as u16,
            Level::Low,
            scale * 50,
        )
    });

    if write_end_marker {
        // Must not scale this, because that would mess up idle_threshold detection!
        tx_data[TX_LEN - 2] = PulseCode::new(Level::High, 3000, Level::Low, 500);
        tx_data[TX_LEN - 1] = PulseCode::end_marker();
    }

    tx_data
}

struct TxDataIter {
    remaining: usize,
    i: u16,
    write_end_marker: bool,
    scale: u16,
}

impl Iterator for TxDataIter {
    type Item = PulseCode;

    fn next(&mut self) -> Option<Self::Item> {
        let code = match self.remaining {
            0 => return None,
            1 if self.write_end_marker => PulseCode::end_marker(),
            2 if self.write_end_marker => {
                PulseCode::new(Level::High, self.scale * 3000, Level::Low, self.scale * 500)
            }
            _ => PulseCode::new(
                Level::High,
                self.scale * (100 + (self.i * 10) % 200) as u16,
                Level::Low,
                self.scale * 50,
            ),
        };

        self.i += 1;
        self.remaining -= 1;

        Some(code)
    }
}

fn check_data_eq(tx: &[PulseCode], rx: &[PulseCode], rx_count: usize, rx_memsize: usize) {
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

    assert_eq!(rx_count, expected_rx_len, "unexpected rx count");

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
        "rx/tx code mismatch at {}/{} indices",
        errors,
        tx.len() - 2
    );
}

// Run a test where some data is sent from one channel and looped back to
// another one for receive, and verify that the data matches.
fn do_rmt_loopback<const TX_LEN: usize>(tx_memsize: u8, rx_memsize: u8) {
    use esp_hal::rmt::{RxChannel, TxChannel};

    let tx_config = TxChannelConfig::default()
        .with_idle_output_level(Level::Low)
        .with_idle_output(true)
        .with_memsize(tx_memsize);
    let rx_config = RxChannelConfig::default()
        .with_idle_threshold(1000)
        .with_memsize(rx_memsize);

    let (mut tx_channel, mut rx_channel) = setup::<Blocking>(tx_config, rx_config);

    let tx_data: [_; TX_LEN] = generate_tx_data(1, true);
    let mut rcv_data: [PulseCode; TX_LEN] = [PulseCode::end_marker(); TX_LEN];

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
    let rx_count = rx_transaction.wait().unwrap();

    check_data_eq(&tx_data, &rcv_data, rx_count, rx_channel.buffer_size());
}

// Run a test where some data is sent from one channel and looped back to
// another one for receive, and verify that the data matches.
async fn do_rmt_loopback_async<const TX_LEN: usize>(tx_memsize: u8, rx_memsize: u8) {
    use esp_hal::rmt::{RxChannelAsync, TxChannelAsync};

    let tx_config = TxChannelConfig::default()
        .with_idle_output_level(Level::Low)
        .with_idle_output(true)
        .with_memsize(tx_memsize);
    let rx_config = RxChannelConfig::default()
        .with_idle_threshold(1000)
        .with_memsize(rx_memsize);

    let (mut tx_channel, mut rx_channel) = setup::<Async>(tx_config, rx_config);

    let tx_data: [_; TX_LEN] = generate_tx_data(1, true);
    let mut rcv_data: [PulseCode; TX_LEN] = [PulseCode::end_marker(); TX_LEN];

    let (rx_res, tx_res) = embassy_futures::join::join(
        rx_channel.receive(&mut rcv_data),
        tx_channel.transmit(&tx_data).unwrap(),
    )
    .await;

    tx_res.unwrap();
    let rx_count = rx_res.unwrap();

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
    use esp_hal::rmt::TxChannel;

    let tx_config = TxChannelConfig::default()
        .with_clk_divider(DIV)
        .with_memsize(tx_memsize);
    let (mut tx_channel, _) = setup::<Blocking>(tx_config, Default::default());

    let mut tx_data = TxDataIter {
        remaining: tx_len,
        i: 0,
        write_end_marker,
        scale: 1,
    };
    tx_channel.transmit(&mut tx_data)?.wait()
}

#[must_use = "Tests should fail on errors"]
async fn do_rmt_single_shot_iter_async(
    tx_len: usize,
    tx_memsize: u8,
    write_end_marker: bool,
) -> Result<(), Error> {
    use esp_hal::rmt::TxChannelAsync;

    let tx_config = TxChannelConfig::default()
        .with_clk_divider(DIV)
        .with_memsize(tx_memsize);
    let (mut tx_channel, _) = setup::<Async>(tx_config, Default::default());

    let mut tx_data = TxDataIter {
        remaining: tx_len,
        i: 0,
        write_end_marker,
        scale: 1,
    };
    tx_channel.transmit(&mut tx_data)?.await
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

        assert!(matches!(result, Err(Error::EndMarkerMissing)));
    }

    #[test]
    fn rmt_overlapping_ram_fails() {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let rmt = Rmt::new(peripherals.RMT, FREQ).unwrap();

        let ch0 = rmt
            .channel0
            .configure_tx(NoPin, TxChannelConfig::default().with_memsize(2));

        let ch1 = rmt.channel1.configure_tx(NoPin, TxChannelConfig::default());

        assert!(ch0.is_ok());
        assert!(matches!(ch1, Err(Error::MemoryBlockNotAvailable)));
    }
}
