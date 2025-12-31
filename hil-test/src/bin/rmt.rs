//! RMT Loopback Test
//!
//! When running this with defmt, consider increasing embedded_test's default_timeout below to
//! avoid timeouts while printing. Note that probe-rs reading the buffer might still mess up
//! timing-sensitive tests! This doesn't apply to `check_data_eq` since it is used after the
//! action, but adding any additional logging to the driver is likely to cause sporadic issues.

//% CHIPS: esp32 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: embassy esp-alloc unstable

#![no_std]
#![no_main]

extern crate alloc;

use core::task::Poll;

use allocator_api2::{vec, vec::Vec};
use embassy_futures::poll_once;
use esp_hal::{
    Async,
    Blocking,
    DriverMode,
    delay::Delay,
    gpio::{
        AnyPin,
        Flex,
        InputConfig,
        Level,
        NoPin,
        OutputConfig,
        Pin,
        Pull,
        interconnect::{PeripheralInput, PeripheralOutput},
    },
    peripherals::RMT,
    ram,
    rmt::{
        CHANNEL_RAM_SIZE,
        Channel,
        ConfigError,
        Error,
        HAS_RX_WRAP,
        LoopMode,
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
    timer::timg::TimerGroup,
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

enum EndMarkerConfig {
    None,
    Field1,
    Field2,
}

struct LoopbackConfig {
    tx_len: usize,
    idle_threshold: u16,
    idle_output: bool,
    tx_memsize: u8,
    rx_memsize: u8,
    write_stop_code: bool,
    end_marker: EndMarkerConfig,
    tolerance: u16,
    abort: bool,
    length1_base: u16,
    length1_step: u16,
    length2: u16,
}

impl Default for LoopbackConfig {
    fn default() -> Self {
        Self {
            tx_len: CHANNEL_RAM_SIZE / 2,
            idle_threshold: 1000,
            idle_output: false,
            tx_memsize: 1,
            rx_memsize: 1,
            write_stop_code: true,
            end_marker: EndMarkerConfig::Field1,
            tolerance: 1,
            abort: false,
            length1_base: 100,
            length1_step: 10,
            length2: 50,
        }
    }
}

impl LoopbackConfig {
    fn tx_config(&self) -> TxChannelConfig {
        TxChannelConfig::default()
            .with_clk_divider(DIV)
            .with_idle_output(self.idle_output)
            .with_idle_output_level(Level::Low)
            .with_memsize(self.tx_memsize)
    }

    fn rx_config(&self) -> RxChannelConfig {
        RxChannelConfig::default()
            .with_clk_divider(DIV)
            .with_idle_threshold(self.idle_threshold)
            .with_memsize(self.rx_memsize)
    }
}

// Pulses of H 100..330 L 50, i.e. 100..330 / 500kHz = 100..330 * 2us = 200..660us
//
// Note that there's no small common multiple of `CHANNEL_RAM_SIZE` (either 48 or 64) and the
// period 23, so we don't repeatedly write the same data to the same location in the hardware
// buffer in loopback tests with buffer wrapping. If we did, that might hide bugs.
// FIXME: Make a method on LoopbackConfig?
fn generate_tx_data(conf: &LoopbackConfig) -> Vec<PulseCode> {
    let mut tx_data: Vec<_> = (0..)
        .take(conf.tx_len)
        .map(|i| {
            PulseCode::new(
                Level::High,
                conf.length1_base + conf.length1_step * (i % 23),
                Level::Low,
                conf.length2,
            )
        })
        .collect();

    let mut pos = conf.tx_len - 1;
    match conf.end_marker {
        EndMarkerConfig::None => (),
        EndMarkerConfig::Field1 => {
            tx_data[pos] = PulseCode::end_marker();
            pos -= 1;
        }
        EndMarkerConfig::Field2 => {
            tx_data[pos] = tx_data[pos].with_length2(0).unwrap();
            pos -= 1;
        }
    }
    if conf.write_stop_code {
        tx_data[pos] = PulseCode::new(
            Level::High,
            2 * conf.idle_threshold,
            Level::Low,
            conf.length2,
        );
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

// Compare tx and rx data from a loopback transmission
//
// This takes into account whether the device does not support wrapping rx such that truncated
// output is expected.
fn check_data_eq(
    conf: &LoopbackConfig,
    tx: &[PulseCode],
    rx: &[PulseCode],
    rx_res: Result<usize, Error>,
) {
    let mut errors: usize = 0;
    let mut count: usize = 0;

    let mut expected_rx_len = conf.tx_len;

    // Iterate through pairs of sent/received `PulseCode`s
    let mut rx_done = false;
    for (idx, (&code_tx, &code_rx)) in core::iter::zip(tx, rx).enumerate() {
        count += if rx_done { 0 } else { 1 };

        let mut _msg = "";
        if rx_done {
            // The rx buffer should be zero-filled, we check that below.
        } else if code_tx.length1() > conf.idle_threshold {
            // Should receive an end marker in the first PulseCode field
            rx_done = true;
            expected_rx_len = expected_rx_len.min(idx + 1);
            if !(code_rx.length1() == 0 && code_tx.level1() == code_rx.level1()) {
                errors += 1;
                _msg = "rx code not a stop code!";
            }
        } else if code_tx.length2() > conf.idle_threshold {
            // Should receive an end marker in the second PulseCode field
            rx_done = true;
            expected_rx_len = expected_rx_len.min(idx + 1);
            if !pulse_code_matches(code_tx.with_length2(0).unwrap(), code_rx, conf.tolerance) {
                errors += 1;
                _msg = "rx code not a stop code!";
            }
        } else if !pulse_code_matches(code_tx, code_rx, conf.tolerance) {
            // Regular pulse code, should be received exactly
            errors += 1;
            _msg = "rx/tx code mismatch!";
        };

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

    // Some devices don't support wrapping rx, and will terminate rx when the buffer is full.
    let rx_memsize_codes = conf.rx_memsize as usize * CHANNEL_RAM_SIZE;
    if !HAS_RX_WRAP {
        expected_rx_len = expected_rx_len.min(rx_memsize_codes)
    };

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
        Err(Error::InvalidDataLength) if !HAS_RX_WRAP => {
            // In this case, rx wasn't even started, so we can't compare the partially received
            // data below!
            assert!(tx.len() > rx_memsize_codes);
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

    assert!(
        errors == 0,
        "rx/tx code mismatch at {}/{} indices",
        errors,
        count
    );
}

struct BitbangTransmitter {
    pin: Flex<'static>,
}

impl BitbangTransmitter {
    fn new(pin: impl Pin + 'static) -> Self {
        let mut pin = Flex::new(pin);
        pin.set_input_enable(true);
        pin.set_output_enable(true);
        pin.apply_input_config(&InputConfig::default().with_pull(Pull::None));
        pin.apply_output_config(&OutputConfig::default());

        pin.set_low();

        Self { pin }
    }

    fn rx_pin(&mut self) -> impl PeripheralInput<'static> {
        self.pin.peripheral_input()
    }

    fn transmit(&mut self, tx_data: &[PulseCode]) {
        let cycles_per_us = DIV as u32 / FREQ.as_mhz();

        let delay = Delay::new();

        for code in tx_data {
            if code.length1() == 0 {
                break;
            }
            self.pin.set_level(code.level1());
            delay.delay_micros(code.length1() as u32 * cycles_per_us);

            if code.length2() == 0 {
                break;
            }
            self.pin.set_level(code.level2());
            delay.delay_micros(code.length2() as u32 * cycles_per_us);
        }
    }
}

fn do_rmt_loopback_inner(
    conf: &LoopbackConfig,
    tx_channel: Channel<Blocking, Tx>,
    rx_channel: Channel<Blocking, Rx>,
) {
    let tx_data = generate_tx_data(conf);
    let mut rcv_data = vec![PulseCode::default(); conf.tx_len];

    let rx_transaction = rx_channel.receive(&mut rcv_data);
    let mut tx_transaction = tx_channel.transmit(&tx_data).unwrap();

    if conf.abort {
        // Start the transactions...
        let mut rx_transaction = rx_transaction.unwrap();

        Delay::new().delay_millis(2);

        // ...poll them for a while, but then drop them...
        // (`poll_once` takes the future by value and drops it before returning)
        let rx_done = rx_transaction.poll();
        let tx_done = tx_transaction.poll();

        // Drop the transactions
        core::mem::drop(rx_transaction);
        core::mem::drop(tx_transaction);

        // The test should fail here when the the delay above is increased, e.g. to 100ms.
        assert!(!rx_done);
        assert!(!tx_done);
    } else {
        let run = move || match rx_transaction {
            Ok(mut rx_transaction) => {
                // ... poll them until completion.
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
            Err((e, _)) => Err(e),
        };
        let rx_res = run();

        check_data_eq(conf, &tx_data, &rcv_data, rx_res);
    }
}

// Run a test where some data is sent from one channel and looped back to
// another one for receive, and verify that the data matches.
fn do_rmt_loopback(ctx: &mut Context, conf: &LoopbackConfig) {
    let (tx_channel, rx_channel) = ctx.setup_loopback(&conf);

    do_rmt_loopback_inner(&conf, tx_channel, rx_channel);
}

async fn do_rmt_loopback_async_inner(
    conf: &LoopbackConfig,
    tx_channel: &mut Channel<'_, Async, Tx>,
    rx_channel: &mut Channel<'_, Async, Rx>,
) {
    use embassy_time::Delay;
    use embedded_hal_async::delay::DelayNs;

    let tx_data = generate_tx_data(conf);
    let mut rcv_data = vec![PulseCode::default(); conf.tx_len];

    // Start the transactions...
    let rx_fut = rx_channel.receive(&mut rcv_data);
    let tx_fut = tx_channel.transmit(&tx_data);

    if conf.abort {
        Delay.delay_ms(2).await;

        // ...poll them, but then drop them before completion...
        // (`poll_once` takes the future by value and drops it before returning)
        let rx_poll = poll_once(rx_fut);
        let tx_poll = poll_once(tx_fut);

        // The test should fail here when the the delay above is increased, e.g. to 100ms.
        assert!(rx_poll.is_pending());
        assert!(tx_poll.is_pending());
    } else {
        let (rx_res, tx_res) = embassy_futures::join::join(rx_fut, tx_fut).await;

        tx_res.unwrap();

        check_data_eq(conf, &tx_data, &rcv_data, rx_res);
    }
}

// Run a test where some data is sent from one channel and looped back to
// another one for receive, and verify that the data matches.
async fn do_rmt_loopback_async(ctx: &mut Context, conf: &LoopbackConfig) {
    let (mut tx_channel, mut rx_channel) = ctx.setup_loopback_async(&conf);

    do_rmt_loopback_async_inner(&conf, &mut tx_channel, &mut rx_channel).await
}

macro_rules! pins {
    ($ctx:expr) => {
        Flex::new($ctx.pin.reborrow()).split()
    };
}

cfg_if::cfg_if!(
    if #[cfg(any(esp32, esp32s3))] {
        macro_rules! rx_channel_creator {
            ($rmt:expr) => {
                $rmt.channel4
            };
        }
    } else {
        macro_rules! rx_channel_creator {
            ($rmt:expr) => {
                $rmt.channel2
            };
        }
    }
);

struct Context {
    rmt: RMT<'static>,
    pin: AnyPin<'static>,
}

impl Context {
    fn setup_impl<'a, Dm: DriverMode>(
        rmt: Rmt<'a, Dm>,
        rx: impl PeripheralInput<'a>,
        tx: impl PeripheralOutput<'a>,
        conf: &LoopbackConfig,
    ) -> (Channel<'a, Dm, Tx>, Channel<'a, Dm, Rx>) {
        let tx_channel = rmt
            .channel0
            .configure_tx(&conf.tx_config())
            .unwrap()
            .with_pin(tx);

        let rx_channel = rx_channel_creator!(rmt)
            .configure_rx(&conf.rx_config())
            .unwrap()
            .with_pin(rx);

        (tx_channel, rx_channel)
    }

    fn setup_loopback(
        &mut self,
        conf: &LoopbackConfig,
    ) -> (Channel<'_, Blocking, Tx>, Channel<'_, Blocking, Rx>) {
        let rmt = Rmt::new(self.rmt.reborrow(), FREQ).unwrap();
        let (rx, tx) = pins!(self);
        Self::setup_impl(rmt, rx, tx, conf)
    }

    fn setup_loopback_async(
        &mut self,
        conf: &LoopbackConfig,
    ) -> (Channel<'_, Async, Tx>, Channel<'_, Async, Rx>) {
        let rmt = Rmt::new(self.rmt.reborrow(), FREQ).unwrap().into_async();
        let (rx, tx) = pins!(self);
        Self::setup_impl(rmt, rx, tx, conf)
    }
}

#[embedded_test::tests(default_timeout = 1, executor = hil_test::Executor::new())]
mod tests {
    #[allow(unused_imports)]
    use hil_test::{assert, assert_eq};

    use super::*;

    #[init]
    fn init() -> Context {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        esp_alloc::heap_allocator!(#[ram(reclaimed)] size: 64 * 1024);

        let software_interrupt =
            esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);

        let timg0 = TimerGroup::new(peripherals.TIMG0);

        esp_rtos::start(timg0.timer0, software_interrupt.software_interrupt0);

        let pin = AnyPin::from(hil_test::common_test_pins!(peripherals).1);

        Context {
            rmt: peripherals.RMT,
            pin,
        }
    }

    #[test]
    fn rmt_loopback_simple(mut ctx: Context) {
        // Fit both tx and rx data into a single memory block
        let conf = LoopbackConfig::default();
        do_rmt_loopback(&mut ctx, &conf);
    }

    #[test]
    async fn rmt_loopback_simple_async(mut ctx: Context) {
        // Fit both tx and rx data into a single memory block
        let conf = LoopbackConfig::default();
        do_rmt_loopback_async(&mut ctx, &conf).await;
    }

    #[test]
    fn rmt_loopback_end_marker_field2(mut ctx: Context) {
        // Fit both tx and rx data into a single memory block
        let conf = LoopbackConfig {
            end_marker: EndMarkerConfig::Field2,
            ..Default::default()
        };
        do_rmt_loopback(&mut ctx, &conf);
    }

    #[test]
    fn rmt_loopback_extended_ram(mut ctx: Context) {
        // require/use 2 memory blocks for both tx and rx data -> no wrapping
        let conf = LoopbackConfig {
            tx_len: CHANNEL_RAM_SIZE * 3 / 2,
            tx_memsize: 2,
            rx_memsize: 2,
            ..Default::default()
        };
        do_rmt_loopback(&mut ctx, &conf);
    }

    #[test]
    fn rmt_loopback_tx_wrap(mut ctx: Context) {
        // require two RAM blocks; thus a tx channel with only 1 block requires wrapping.
        let conf = LoopbackConfig {
            tx_len: CHANNEL_RAM_SIZE * 3 / 2,
            tx_memsize: 1,
            rx_memsize: 2,
            ..Default::default()
        };
        do_rmt_loopback(&mut ctx, &conf);
    }

    #[test]
    async fn rmt_loopback_tx_wrap_async(mut ctx: Context) {
        // require two RAM blocks; thus a tx channel with only 1 block requires wrapping.
        let conf = LoopbackConfig {
            tx_len: CHANNEL_RAM_SIZE * 3 / 2,
            tx_memsize: 1,
            rx_memsize: 2,
            ..Default::default()
        };
        do_rmt_loopback_async(&mut ctx, &conf).await;
    }

    #[test]
    fn rmt_loopback_rx_wrap(mut ctx: Context) {
        // require two RAM blocks; thus an rx channel with only 1 block requires wrapping.
        let conf = LoopbackConfig {
            tx_len: CHANNEL_RAM_SIZE * 3 / 2,
            tx_memsize: 2,
            rx_memsize: 1,
            ..Default::default()
        };
        do_rmt_loopback(&mut ctx, &conf);
    }

    #[test]
    async fn rmt_loopback_rx_wrap_async(mut ctx: Context) {
        // require two RAM blocks; thus an rx channel with only 1 block requires wrapping.
        let conf = LoopbackConfig {
            tx_len: CHANNEL_RAM_SIZE * 3 / 2,
            tx_memsize: 2,
            rx_memsize: 1,
            ..Default::default()
        };
        do_rmt_loopback_async(&mut ctx, &conf).await;
    }

    // Test that errors on invalid data are returned as expected (in particular, checks that the
    // methods don't hang executing a never-ending/never-started tranmission).
    #[test]
    fn rmt_tx_errors_blocking(mut ctx: Context) {
        let conf = LoopbackConfig {
            end_marker: EndMarkerConfig::None,
            ..Default::default()
        };

        let (mut tx_channel, _) = ctx.setup_loopback(&conf);

        let tx_data = generate_tx_data(&conf);

        assert!(
            matches!(
                tx_channel.reborrow().transmit(&tx_data),
                Err((Error::EndMarkerMissing, _))
            ),
            "Expected transmit to return an error without end marker"
        );

        assert!(
            matches!(
                tx_channel.reborrow().transmit(&tx_data[..0]),
                Err((Error::InvalidArgument, _))
            ),
            "Expected transmit to return an error on empty data"
        );

        assert!(
            matches!(
                tx_channel
                    .reborrow()
                    .transmit_continuously(&tx_data, LoopMode::Infinite),
                Err((Error::EndMarkerMissing, _))
            ),
            "Expected transmit_continuously to return an error without end marker"
        );

        assert!(
            matches!(
                tx_channel
                    .reborrow()
                    .transmit_continuously(&tx_data[..0], LoopMode::Infinite),
                Err((Error::InvalidArgument, _))
            ),
            "Expected transmit_continuously to return an error on empty data"
        );

        // Configuration that requires wrapping tx and is missing an end marker
        let conf = LoopbackConfig {
            tx_len: CHANNEL_RAM_SIZE * 3 / 2,
            end_marker: EndMarkerConfig::None,
            ..Default::default()
        };

        let tx_data = generate_tx_data(&conf);

        // Most importantly, this should not hang indefinitely due to the missing end marker and
        // re-transmitting the channel RAM content over and over again.
        assert!(
            matches!(
                tx_channel
                    .reborrow()
                    .transmit(&tx_data)
                    .map(|t| t.wait())
                    .flatten(),
                Err((Error::EndMarkerMissing, _))
            ),
            "Expected transmit to return an error without end marker when wrapping"
        );

        assert!(
            matches!(
                tx_channel
                    .reborrow()
                    .transmit_continuously(&tx_data, LoopMode::Infinite),
                Err((Error::Overflow, _))
            ),
            "Expected transmit_continuously to return an error on overflow"
        );
    }

    #[test]
    async fn rmt_tx_errors_async(mut ctx: Context) {
        let conf = LoopbackConfig {
            end_marker: EndMarkerConfig::None,
            ..Default::default()
        };

        let (mut tx_channel, _) = ctx.setup_loopback_async(&conf);

        let tx_data = generate_tx_data(&conf);

        assert!(
            matches!(
                poll_once(tx_channel.transmit(&tx_data)),
                Poll::Ready(Err(Error::EndMarkerMissing))
            ),
            "Expected transmit to return an error without end marker"
        );

        assert!(
            matches!(
                poll_once(tx_channel.transmit(&tx_data[..0])),
                Poll::Ready(Err(Error::InvalidArgument))
            ),
            "Expected transmit to return an error on empty data"
        );

        // Configuration that requires wrapping tx and is missing an end marker
        let conf = LoopbackConfig {
            tx_len: CHANNEL_RAM_SIZE * 3 / 2,
            end_marker: EndMarkerConfig::None,
            ..Default::default()
        };
        let tx_data = generate_tx_data(&conf);

        // Most importantly, this should not hang indefinitely due to the missing end marker and
        // re-transmitting the channel RAM content over and over again.
        assert!(
            matches!(
                tx_channel.transmit(&tx_data).await,
                Err(Error::EndMarkerMissing)
            ),
            "Expected transmit to return an error without end marker when wrapping"
        );
    }

    #[test]
    fn rmt_overlapping_ram_fails(mut ctx: Context) {
        let rmt = Rmt::new(ctx.rmt.reborrow(), FREQ).unwrap();

        let _ch0 = rmt
            .channel0
            .configure_tx(&TxChannelConfig::default().with_memsize(2))
            .unwrap();

        // Configuring channel 1 should fail, since channel 0 already uses its memory.
        let ch1 = rmt.channel1.configure_tx(&TxChannelConfig::default());

        assert!(matches!(ch1, Err(ConfigError::MemoryBlockNotAvailable)));
    }

    #[test]
    fn rmt_overlapping_ram_release(mut ctx: Context) {
        let rmt = Rmt::new(ctx.rmt.reborrow(), FREQ).unwrap();

        let ch0 = rmt
            .channel0
            .configure_tx(&TxChannelConfig::default().with_memsize(2))
            .unwrap();

        // After dropping channel 0, the memory that it reserved should become available
        // again such that channel 1 configuration succeeds.
        core::mem::drop(ch0);
        rmt.channel1
            .configure_tx(&TxChannelConfig::default())
            .unwrap();
    }

    #[test]
    fn rmt_overlapping_ram_reconfigure(mut ctx: Context) {
        let mut rmt = Rmt::new(ctx.rmt.reborrow(), FREQ).unwrap();

        let mut ch0 = rmt
            .channel0
            .configure_tx(&TxChannelConfig::default().with_memsize(1))
            .unwrap();

        let ch1 = rmt
            .channel1
            .reborrow()
            .configure_tx(&TxChannelConfig::default().with_memsize(1))
            .unwrap();

        let res = ch0.apply_config(&TxChannelConfig::default().with_memsize(2));
        assert!(matches!(res, Err(ConfigError::MemoryBlockNotAvailable)));

        core::mem::drop(ch1);

        ch0.apply_config(&TxChannelConfig::default().with_memsize(2))
            .unwrap();

        let res = rmt
            .channel1
            .configure_tx(&TxChannelConfig::default().with_memsize(1));
        assert!(matches!(res, Err(ConfigError::MemoryBlockNotAvailable)));
    }

    macro_rules! test_channel_pair {
        (
            $ctx:expr,
            $tx_channel:ident,
            $rx_channel:ident
        ) => {{
            let conf = LoopbackConfig::default();

            let (rx_pin, tx_pin) = pins!($ctx);
            let mut rmt = Rmt::new($ctx.rmt.reborrow(), FREQ).unwrap();

            let tx_channel = rmt
                .$tx_channel
                .reborrow()
                .configure_tx(&conf.tx_config())
                .unwrap()
                .with_pin(tx_pin);

            let rx_channel = rmt
                .$rx_channel
                .reborrow()
                .configure_rx(&conf.rx_config())
                .unwrap()
                .with_pin(rx_pin);

            do_rmt_loopback_inner(&conf, tx_channel, rx_channel);
        }};
    }

    // A simple test that uses all channels: This is useful to verify that all channel/register
    // indexing in the driver is correct. The other tests are prone to hide related issues due to
    // using low channel numbers only (in particular 0 for the tx channel).
    #[test]
    fn rmt_use_all_channels(mut ctx: Context) {
        // FIXME: Find a way to leverage esp-metadata to generate this.

        // Chips with combined RxTx channels
        #[cfg(esp32)]
        {
            test_channel_pair!(ctx, channel0, channel1);
            test_channel_pair!(ctx, channel0, channel2);
            test_channel_pair!(ctx, channel0, channel3);
            test_channel_pair!(ctx, channel0, channel4);
            test_channel_pair!(ctx, channel0, channel5);
            test_channel_pair!(ctx, channel0, channel6);
            test_channel_pair!(ctx, channel0, channel7);

            test_channel_pair!(ctx, channel1, channel0);
            test_channel_pair!(ctx, channel2, channel0);
            test_channel_pair!(ctx, channel3, channel0);
            test_channel_pair!(ctx, channel4, channel0);
            test_channel_pair!(ctx, channel5, channel0);
            test_channel_pair!(ctx, channel6, channel0);
            test_channel_pair!(ctx, channel7, channel0);
        }

        #[cfg(esp32s2)]
        {
            test_channel_pair!(ctx, channel0, channel1);
            test_channel_pair!(ctx, channel0, channel2);
            test_channel_pair!(ctx, channel0, channel3);

            test_channel_pair!(ctx, channel1, channel0);
            test_channel_pair!(ctx, channel2, channel0);
            test_channel_pair!(ctx, channel3, channel0);
        }

        // Chips with separate Rx and Tx channels
        #[cfg(esp32s3)]
        {
            test_channel_pair!(ctx, channel0, channel4);
            test_channel_pair!(ctx, channel1, channel5);
            test_channel_pair!(ctx, channel2, channel6);
            test_channel_pair!(ctx, channel3, channel7);
        }

        #[cfg(not(any(esp32, esp32s2, esp32s3)))]
        {
            test_channel_pair!(ctx, channel0, channel2);
            test_channel_pair!(ctx, channel1, channel3);
        }
    }

    // Test that the timing is at least roughly correct (i.e. the clock source isn't completely
    // misconfigured).
    // This test is almost guaranteed to fail when using defmt!
    #[test]
    fn rmt_clock_rate(ctx: Context) {
        // Increase pulse lengths and tolerance to deal with the imprecision of Delay
        let conf = LoopbackConfig {
            tx_len: 6,
            // idle threshold 16383 cycles = ~33ms
            idle_threshold: 0x3FFF,
            // tolerance 25 cycles == 50us
            tolerance: 25,
            length1_base: 500,
            length1_step: 500,
            length2: 500,
            ..Default::default()
        };

        let rmt = Rmt::new(ctx.rmt, FREQ).unwrap();

        let mut bitbang_tx = BitbangTransmitter::new(ctx.pin);
        let (_, rx_channel) = Context::setup_impl(rmt, bitbang_tx.rx_pin(), NoPin, &conf);

        let tx_data = generate_tx_data(&conf);
        let mut rx_data = [PulseCode::default(); 6];

        let rx_transaction = rx_channel.receive(&mut rx_data).unwrap();
        bitbang_tx.transmit(&tx_data);
        let rx_res = rx_transaction.wait().map(|(x, _)| x).map_err(|(e, _)| e);

        check_data_eq(&conf, &tx_data, &rx_data, rx_res);
    }

    // Compile-only test to verify that reborrowing channel creators works.
    async fn _rmt_reborrow_channel_creator() {
        let mut peripherals = esp_hal::init(esp_hal::Config::default());

        let mut rmt = Rmt::new(peripherals.RMT.reborrow(), FREQ)
            .unwrap()
            .into_async();

        let conf = LoopbackConfig::default();

        for _ in 0..2 {
            let mut ch0 = rmt
                .channel0
                .reborrow()
                .configure_tx(&conf.tx_config())
                .unwrap();

            let tx_data = generate_tx_data(&conf);

            ch0.transmit(&tx_data).await.unwrap();
        }
    }

    // Test that dropping rx/tx transactions returns the hardware to a predictable state from which
    // subsequent transactions are successful.
    #[test]
    fn rmt_loopback_after_drop_blocking(mut ctx: Context) {
        let mut conf = LoopbackConfig {
            // If not enabling a defined idle_output level, the output might remain high after
            // dropping the tx transaction, which will lead to an extra edge being received.
            idle_output: true,
            ..Default::default()
        };

        let (mut tx_channel, mut rx_channel) = ctx.setup_loopback(&conf);

        conf.abort = true;
        do_rmt_loopback_inner(&conf, tx_channel.reborrow(), rx_channel.reborrow());

        conf.abort = false;
        do_rmt_loopback_inner(&conf, tx_channel, rx_channel);
    }

    // Test that completed blocking transactions leave the hardware in a predictable state from
    // which subsequent transactions are successful.
    #[test]
    fn rmt_loopback_repeat_blocking(mut ctx: Context) {
        let conf = LoopbackConfig {
            // If not enabling a defined idle_output level, the output might remain high after
            // dropping the tx transaction, which will lead to an extra edge being received.
            idle_output: true,
            ..Default::default()
        };

        // Test that dropping & recreating Rmt works
        for _ in 0..3 {
            let mut rmt = Rmt::new(ctx.rmt.reborrow(), FREQ).unwrap();

            // Test that dropping & recreating ChannelCreator works
            for _ in 0..3 {
                let (rx, tx) = pins!(ctx);

                let mut tx_channel = rmt
                    .channel0
                    .reborrow()
                    .configure_tx(&conf.tx_config())
                    .unwrap()
                    .with_pin(tx);

                let mut rx_channel = rx_channel_creator!(rmt)
                    .reborrow()
                    .configure_rx(&conf.rx_config())
                    .unwrap()
                    .with_pin(rx);

                // Test that dropping & recreating Channel works
                for _ in 0..3 {
                    do_rmt_loopback_inner(&conf, tx_channel.reborrow(), rx_channel.reborrow());
                }
            }
        }
    }

    // Test that dropping rx/tx futures returns the hardware to a predictable state from which
    // subsequent transactions are successful.
    #[test]
    async fn rmt_loopback_after_drop_async(mut ctx: Context) {
        let mut conf = LoopbackConfig {
            // If not enabling a defined idle_output level, the output might remain high after
            // dropping the tx future, which will lead to an extra edge being received.
            idle_output: true,
            ..Default::default()
        };

        let (mut tx_channel, mut rx_channel) = ctx.setup_loopback_async(&conf);

        // Start loopback once, but abort before completion...
        conf.abort = true;
        do_rmt_loopback_async_inner(&conf, &mut tx_channel, &mut rx_channel).await;

        // ...then start over and check that everything still works as expected (i.e. we
        // didn't leave the hardware in an unexpected state or lock up when
        // dropping the futures).
        conf.abort = false;
        do_rmt_loopback_async_inner(&conf, &mut tx_channel, &mut rx_channel).await;
    }

    // Test that completed async transactions leave the hardware in a predictable state from which
    // subsequent transactions are successful.
    #[test]
    async fn rmt_loopback_repeat_async(mut ctx: Context) {
        let conf = LoopbackConfig {
            // If not enabling a defined idle_output level, the output might remain high after
            // dropping the tx future, which will lead to an extra edge being received.
            idle_output: true,
            ..Default::default()
        };

        // Test that dropping & recreating Rmt works
        for _ in 0..3 {
            let mut rmt = Rmt::new(ctx.rmt.reborrow(), FREQ).unwrap().into_async();

            // Test that dropping & recreating ChannelCreator works
            for _ in 0..3 {
                let (rx, tx) = pins!(ctx);

                let mut tx_channel = rmt
                    .channel0
                    .reborrow()
                    .configure_tx(&conf.tx_config())
                    .unwrap()
                    .with_pin(tx);

                let mut rx_channel = rx_channel_creator!(rmt)
                    .reborrow()
                    .configure_rx(&conf.rx_config())
                    .unwrap()
                    .with_pin(rx);

                // Test that dropping & recreating Channel works
                for _ in 0..3 {
                    do_rmt_loopback_async_inner(&conf, &mut tx_channel, &mut rx_channel).await;
                }
            }
        }
    }

    #[cfg(not(esp32))]
    fn rmt_loopback_continuous_tx_impl(mut ctx: Context, use_autostop: bool) {
        const TX_COUNT: usize = 10;
        const MAX_LOOP_COUNT: usize = 3;
        const MAX_RX_LEN: usize = (MAX_LOOP_COUNT + 1) * TX_COUNT;

        let mut conf = LoopbackConfig {
            tx_len: TX_COUNT + 1,
            write_stop_code: false,
            idle_output: true,
            rx_memsize: 2,
            ..Default::default()
        };

        let (mut tx_channel, mut rx_channel) = ctx.setup_loopback(&conf);

        let tx_data = generate_tx_data(&conf);
        let mut rx_data = vec![PulseCode::default(); MAX_RX_LEN];
        let mut expected_data = vec![PulseCode::default(); MAX_RX_LEN];

        for loopcount in 1..MAX_LOOP_COUNT {
            #[allow(unused_mut)]
            let mut loopmode = LoopMode::InfiniteWithInterrupt(loopcount as u16);
            #[cfg(any(esp32c6, esp32h2, esp32s3))]
            if use_autostop {
                loopmode = LoopMode::Finite(loopcount as u16);
            };
            rx_data.fill(PulseCode::default());
            let rx_transaction = rx_channel.reborrow().receive(&mut rx_data).unwrap();
            let tx_transaction = tx_channel
                .reborrow()
                .transmit_continuously(&tx_data, loopmode)
                .unwrap();

            // All data is small enough to fit a single hardware buffer, so we don't need to poll
            // rx/tx concurrently.
            while !tx_transaction.is_loopcount_interrupt_set() {}

            let rx_res = if use_autostop {
                // tx should stop automatically, so rx should also stop before we explicitly stop
                // tx
                let res = rx_transaction.wait();
                tx_transaction.stop_next().unwrap();
                res
            } else {
                tx_transaction.stop_next().unwrap();
                rx_transaction.wait()
            };

            let rx_res = rx_res.map(|(x, _)| x).map_err(|(e, _)| e);

            // FIXME: Somehow verify that tx is really stopped!

            let rx_loopcount = if use_autostop {
                loopcount
            } else {
                loopcount + 1
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

            conf.tx_len = expected_data.len();
            check_data_eq(&conf, &expected_data, &rx_data, rx_res);
        }
    }

    // Test that continuous tx with a finite loopcount works as expected when using manual
    // stopping (on devices that have the required hardware support).
    #[cfg(not(esp32))]
    #[test]
    fn rmt_loopback_continuous_tx_manual_stop(ctx: Context) {
        rmt_loopback_continuous_tx_impl(ctx, false);
    }

    // Test that continuous tx with a finite loopcount works as expected when using automatic
    // stopping (on devices that have the required hardware support).
    #[cfg(any(esp32c6, esp32h2, esp32s3))]
    #[test]
    fn rmt_loopback_continuous_tx_auto_stop(ctx: Context) {
        rmt_loopback_continuous_tx_impl(ctx, true);
    }

    // Test that using loopcount 0 doesn't hang, but returns success immediately.
    #[cfg(any(esp32c6, esp32h2, esp32s3))]
    #[test]
    fn rmt_continuous_tx_zero_loopcount(mut ctx: Context) {
        use esp_hal::time::Instant;

        let conf = LoopbackConfig {
            idle_output: true,
            write_stop_code: false,
            ..Default::default()
        };

        let (mut tx_channel, _) = ctx.setup_loopback(&conf);

        // Use long pulse codes, such that we can use a relatively low threshold in the assertion
        // below to ensure that this test is robust against small timing variations
        let tx_data: [_; 3] = [
            PulseCode::new(Level::High, 10_000, Level::Low, 10_000),
            PulseCode::new(Level::High, 10_000, Level::Low, 10_000),
            PulseCode::end_marker(),
        ];

        let start = Instant::now();

        let tx_transaction = tx_channel
            .reborrow()
            .transmit_continuously(&tx_data, LoopMode::Finite(0))
            .unwrap();

        while !tx_transaction.is_loopcount_interrupt_set() {}

        tx_transaction.stop_next().unwrap();

        // overall, this should complete in less time than sending a single code from `tx_data`
        // would take (see above, the codes are several 2 * 10ms long)
        assert!(
            start.elapsed().as_micros() < 1000,
            "tx with loopcount 0 did not complete immediately"
        );
    }
}
