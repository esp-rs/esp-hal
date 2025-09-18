//! RMT Loopback Test

//% CHIPS: esp32 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: embassy unstable

#![no_std]
#![no_main]

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
        Pull,
        interconnect::{PeripheralInput, PeripheralOutput},
    },
    peripherals::RMT,
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
fn check_data_eq(tx: &[PulseCode], rx: &[PulseCode], idle_threshold: u16, tolerance: u16) {
    let mut errors: usize = 0;
    let mut count: usize = 0;

    for (_idx, (&code_tx, &code_rx)) in core::iter::zip(tx, rx).enumerate() {
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
                _idx,
                code_tx,
                code_rx,
                _msg
            );
        } else {
            defmt::info!(
                "loopback @ idx {}: {:?} (tx) -> {:?} (rx)",
                _idx,
                code_tx,
                code_rx,
            );
        }

        if done {
            break;
        }
    }

    assert!(
        errors == 0,
        "rx/tx code mismatch at {}/{} indices",
        errors,
        count
    );
}

fn do_rmt_loopback_inner<const TX_LEN: usize>(
    tx_channel: Channel<Blocking, Tx>,
    rx_channel: Channel<Blocking, Rx>,
    abort: bool,
) {
    let tx_data: [_; TX_LEN] = generate_tx_data(true, true);
    let mut rcv_data: [PulseCode; TX_LEN] = [PulseCode::default(); TX_LEN];

    // Start the transactions...
    let mut rx_transaction = rx_channel.receive(&mut rcv_data).unwrap();
    let mut tx_transaction = tx_channel.transmit(&tx_data).unwrap();

    if abort {
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
        // ... poll them until completion.
        loop {
            let tx_done = tx_transaction.poll();
            let rx_done = rx_transaction.poll();
            if tx_done && rx_done {
                break;
            }
        }

        tx_transaction.wait().unwrap();
        rx_transaction.wait().unwrap();

        check_data_eq(&tx_data, &rcv_data, 1000, 1);
    }
}

// Run a test where some data is sent from one channel and looped back to
// another one for receive, and verify that the data matches.
fn do_rmt_loopback<const TX_LEN: usize>(ctx: &mut Context, tx_memsize: u8, rx_memsize: u8) {
    let tx_config = TxChannelConfig::default().with_memsize(tx_memsize);
    let rx_config = RxChannelConfig::default()
        .with_idle_threshold(1000)
        .with_memsize(rx_memsize);

    let (tx_channel, rx_channel) = ctx.setup_loopback(tx_config, rx_config);

    do_rmt_loopback_inner::<TX_LEN>(tx_channel, rx_channel, false);
}

async fn do_rmt_loopback_async_inner<const TX_LEN: usize>(
    tx_channel: &mut Channel<'_, Async, Tx>,
    rx_channel: &mut Channel<'_, Async, Rx>,
    abort: bool,
) {
    use embassy_time::Delay;
    use embedded_hal_async::delay::DelayNs;

    let tx_data: [_; TX_LEN] = generate_tx_data(true, true);
    let mut rcv_data: [PulseCode; TX_LEN] = [PulseCode::default(); TX_LEN];

    // Start the transactions...
    let rx_fut = rx_channel.receive(&mut rcv_data);
    let tx_fut = tx_channel.transmit(&tx_data);

    if abort {
        Delay.delay_ms(2).await;

        // ...poll them, but then drop them before completion...
        // (`poll_once` takes the future by value and drops it before returning)
        let rx_poll = embassy_futures::poll_once(rx_fut);
        let tx_poll = embassy_futures::poll_once(tx_fut);

        // The test should fail here when the the delay above is increased, e.g. to 100ms.
        assert!(rx_poll.is_pending());
        assert!(tx_poll.is_pending());
    } else {
        let (rx_res, tx_res) = embassy_futures::join::join(rx_fut, tx_fut).await;

        tx_res.unwrap();
        rx_res.unwrap();

        check_data_eq(&tx_data, &rcv_data, 1000, 1);
    }
}

// Run a test where some data is sent from one channel and looped back to
// another one for receive, and verify that the data matches.
async fn do_rmt_loopback_async<const TX_LEN: usize>(
    ctx: &mut Context,
    tx_memsize: u8,
    rx_memsize: u8,
) {
    let tx_config = TxChannelConfig::default().with_memsize(tx_memsize);
    let rx_config = RxChannelConfig::default()
        .with_idle_threshold(1000)
        .with_memsize(rx_memsize);

    let (mut tx_channel, mut rx_channel) = ctx.setup_loopback_async(tx_config, rx_config);

    do_rmt_loopback_async_inner::<TX_LEN>(&mut tx_channel, &mut rx_channel, false).await
}

// Run a test that just sends some data, without trying to recive it.
#[must_use = "Tests should fail on errors"]
fn do_rmt_single_shot<const TX_LEN: usize>(
    mut ctx: Context,
    tx_memsize: u8,
    write_end_marker: bool,
) -> Result<(), Error> {
    let tx_config = TxChannelConfig::default()
        .with_clk_divider(DIV)
        .with_memsize(tx_memsize);

    let (tx_channel, _) = ctx.setup_loopback(tx_config, Default::default());

    let tx_data: [_; TX_LEN] = generate_tx_data(write_end_marker, write_end_marker);

    tx_channel.transmit(&tx_data)?.wait().map_err(|(e, _)| e)?;

    Ok(())
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
        tx_config: TxChannelConfig,
        rx_config: RxChannelConfig,
    ) -> (Channel<'a, Dm, Tx>, Channel<'a, Dm, Rx>) {
        let tx_channel = rmt
            .channel0
            .configure_tx(tx, tx_config.with_clk_divider(DIV))
            .unwrap();

        let rx_channel = rx_channel_creator!(rmt)
            .configure_rx(rx, rx_config.with_clk_divider(DIV))
            .unwrap();

        (tx_channel, rx_channel)
    }

    fn setup_loopback(
        &mut self,
        tx_config: TxChannelConfig,
        rx_config: RxChannelConfig,
    ) -> (Channel<'_, Blocking, Tx>, Channel<'_, Blocking, Rx>) {
        let rmt = Rmt::new(self.rmt.reborrow(), FREQ).unwrap();
        let (rx, tx) = pins!(self);
        Self::setup_impl(rmt, rx, tx, tx_config, rx_config)
    }

    fn setup_loopback_async(
        &mut self,
        tx_config: TxChannelConfig,
        rx_config: RxChannelConfig,
    ) -> (Channel<'_, Async, Tx>, Channel<'_, Async, Rx>) {
        let rmt = Rmt::new(self.rmt.reborrow(), FREQ).unwrap().into_async();
        let (rx, tx) = pins!(self);
        Self::setup_impl(rmt, rx, tx, tx_config, rx_config)
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

        #[cfg(riscv)]
        let software_interrupt =
            esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);

        let timg0 = TimerGroup::new(peripherals.TIMG0);

        esp_rtos::start(
            timg0.timer0,
            #[cfg(riscv)]
            software_interrupt.software_interrupt0,
        );

        let pin = AnyPin::from(hil_test::common_test_pins!(peripherals).1);

        Context {
            rmt: peripherals.RMT,
            pin,
        }
    }

    #[test]
    fn rmt_loopback_simple(mut ctx: Context) {
        // 20 codes fit a single RAM block
        do_rmt_loopback::<20>(&mut ctx, 1, 1);
    }

    #[test]
    async fn rmt_loopback_simple_async(mut ctx: Context) {
        // 20 codes fit a single RAM block
        do_rmt_loopback_async::<20>(&mut ctx, 1, 1).await;
    }
    #[test]
    fn rmt_loopback_extended_ram(mut ctx: Context) {
        // 80 codes require two RAM blocks
        do_rmt_loopback::<80>(&mut ctx, 2, 2);
    }

    #[test]
    fn rmt_loopback_tx_wrap(mut ctx: Context) {
        // 80 codes require two RAM blocks; thus a tx channel with only 1 block requires
        // wrapping.
        do_rmt_loopback::<80>(&mut ctx, 1, 2);
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
    fn rmt_single_shot_wrap(ctx: Context) {
        // Single RAM block (48 or 64 codes), requires wrappin, falseg
        do_rmt_single_shot::<80>(ctx, 1, true).unwrap();
    }

    #[test]
    fn rmt_single_shot_extended(ctx: Context) {
        // Two RAM blocks (96 or 128 codes), no wrapping
        do_rmt_single_shot::<80>(ctx, 2, true).unwrap();
    }

    #[test]
    fn rmt_single_shot_extended_wrap(ctx: Context) {
        // Two RAM blocks (96 or 128 codes), requires wrapping
        do_rmt_single_shot::<150>(ctx, 2, true).unwrap();
    }

    #[test]
    fn rmt_single_shot_fails_without_end_marker(ctx: Context) {
        let result = do_rmt_single_shot::<20>(ctx, 1, false);

        assert_eq!(result, Err(Error::EndMarkerMissing));
    }

    #[test]
    fn rmt_overlapping_ram_fails(mut ctx: Context) {
        let rmt = Rmt::new(ctx.rmt.reborrow(), FREQ).unwrap();

        let _ch0 = rmt
            .channel0
            .configure_tx(NoPin, TxChannelConfig::default().with_memsize(2))
            .unwrap();

        // Configuring channel 1 should fail, since channel 0 already uses its memory.
        let ch1 = rmt.channel1.configure_tx(NoPin, TxChannelConfig::default());

        assert!(matches!(ch1, Err(Error::MemoryBlockNotAvailable)));
    }

    #[test]
    fn rmt_overlapping_ram_release(mut ctx: Context) {
        let rmt = Rmt::new(ctx.rmt.reborrow(), FREQ).unwrap();

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
            $ctx:expr,
            $tx_channel:ident,
            $rx_channel:ident
        ) => {{
            let tx_config = TxChannelConfig::default();
            let rx_config = RxChannelConfig::default().with_idle_threshold(1000);

            let (rx_pin, tx_pin) = pins!($ctx);
            let mut rmt = Rmt::new($ctx.rmt.reborrow(), FREQ).unwrap();

            let tx_channel = rmt
                .$tx_channel
                .reborrow()
                .configure_tx(tx_pin, tx_config)
                .unwrap();

            let rx_channel = rmt
                .$rx_channel
                .reborrow()
                .configure_rx(rx_pin, rx_config)
                .unwrap();

            do_rmt_loopback_inner::<20>(tx_channel, rx_channel, false);
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
        let rmt = Rmt::new(ctx.rmt, FREQ).unwrap();

        let mut pin = Flex::new(ctx.pin);
        pin.set_input_enable(true);
        pin.set_output_enable(true);
        pin.apply_input_config(&InputConfig::default().with_pull(Pull::None));
        pin.apply_output_config(&OutputConfig::default());
        let rx_pin = pin.peripheral_input();
        let mut tx_pin = pin;

        tx_pin.set_low();

        let tx_config = TxChannelConfig::default();
        // idle threshold 16383 cycles = ~33ms
        let rx_config = RxChannelConfig::default().with_idle_threshold(0x3FFF);

        let (_, rx_channel) = Context::setup_impl(rmt, rx_pin, NoPin, tx_config, rx_config);

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

        rx_transaction.wait().unwrap();

        // tolerance 25 cycles == 50us
        check_data_eq(&expected, &rx_data, 0x3FFF, 25);
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
    fn rmt_loopback_after_drop_blocking(mut ctx: Context) {
        const TX_LEN: usize = 40;

        let tx_config = TxChannelConfig::default()
            // If not enabling a defined idle_output level, the output might remain high after
            // dropping the tx transaction, which will lead to an extra edge being received.
            .with_idle_output(true);
        let rx_config = RxChannelConfig::default().with_idle_threshold(1000);

        let (mut tx_channel, mut rx_channel) = ctx.setup_loopback(tx_config, rx_config);

        do_rmt_loopback_inner::<TX_LEN>(tx_channel.reborrow(), rx_channel.reborrow(), true);

        do_rmt_loopback_inner::<TX_LEN>(tx_channel, rx_channel, false);
    }

    // Test that completed blocking transactions leave the hardware in a predictable state from
    // which subsequent transactions are successful.
    #[test]
    fn rmt_loopback_repeat_blocking(mut ctx: Context) {
        let tx_config = TxChannelConfig::default()
            // If not enabling a defined idle_output level, the output might remain high after
            // dropping the tx future, which will lead to an extra edge being received.
            .with_idle_output(true);
        let rx_config = RxChannelConfig::default().with_idle_threshold(1000);

        // Test that dropping & recreating Rmt works
        for _ in 0..3 {
            let mut rmt = Rmt::new(ctx.rmt.reborrow(), FREQ).unwrap();

            // Test that dropping & recreating ChannelCreator works
            for _ in 0..3 {
                let (rx, tx) = pins!(ctx);

                let mut tx_channel = rmt
                    .channel0
                    .reborrow()
                    .configure_tx(tx, tx_config.with_clk_divider(DIV))
                    .unwrap();

                let mut rx_channel = rx_channel_creator!(rmt)
                    .reborrow()
                    .configure_rx(rx, rx_config.with_clk_divider(DIV))
                    .unwrap();

                // Test that dropping & recreating Channel works
                for _ in 0..3 {
                    do_rmt_loopback_inner::<20>(
                        tx_channel.reborrow(),
                        rx_channel.reborrow(),
                        false,
                    );
                }
            }
        }
    }

    // Test that dropping rx/tx futures returns the hardware to a predictable state from which
    // subsequent transactions are successful.
    #[test]
    async fn rmt_loopback_after_drop_async(mut ctx: Context) {
        const TX_LEN: usize = 40;

        let tx_config = TxChannelConfig::default()
            // If not enabling a defined idle_output level, the output might remain high after
            // dropping the tx future, which will lead to an extra edge being received.
            .with_idle_output(true);
        let rx_config = RxChannelConfig::default().with_idle_threshold(1000);

        let (mut tx_channel, mut rx_channel) = ctx.setup_loopback_async(tx_config, rx_config);

        // Start loopback once, but abort before completion...
        do_rmt_loopback_async_inner::<TX_LEN>(&mut tx_channel, &mut rx_channel, true).await;

        // ...then start over and check that everything still works as expected (i.e. we
        // didn't leave the hardware in an unexpected state or lock up when
        // dropping the futures).
        do_rmt_loopback_async_inner::<TX_LEN>(&mut tx_channel, &mut rx_channel, false).await;
    }

    // Test that completed async transactions leave the hardware in a predictable state from which
    // subsequent transactions are successful.
    #[test]
    async fn rmt_loopback_repeat_async(mut ctx: Context) {
        let tx_config = TxChannelConfig::default()
            // If not enabling a defined idle_output level, the output might remain high after
            // dropping the tx future, which will lead to an extra edge being received.
            .with_idle_output(true);
        let rx_config = RxChannelConfig::default().with_idle_threshold(1000);

        // Test that dropping & recreating Rmt works
        for _ in 0..3 {
            let mut rmt = Rmt::new(ctx.rmt.reborrow(), FREQ).unwrap().into_async();

            // Test that dropping & recreating ChannelCreator works
            for _ in 0..3 {
                let (rx, tx) = pins!(ctx);

                let mut tx_channel = rmt
                    .channel0
                    .reborrow()
                    .configure_tx(tx, tx_config.with_clk_divider(DIV))
                    .unwrap();

                let mut rx_channel = rx_channel_creator!(rmt)
                    .reborrow()
                    .configure_rx(rx, rx_config.with_clk_divider(DIV))
                    .unwrap();

                // Test that dropping & recreating Channel works
                for _ in 0..3 {
                    do_rmt_loopback_async_inner::<20>(&mut tx_channel, &mut rx_channel, false)
                        .await;
                }
            }
        }
    }

    #[cfg(not(esp32))]
    fn rmt_loopback_continuous_tx_impl(mut ctx: Context, loopstop: esp_hal::rmt::LoopStop) {
        use esp_hal::rmt::{LoopCount, LoopStop};

        const TX_COUNT: usize = 10;
        const MAX_LOOP_COUNT: usize = 3;
        const MAX_RX_LEN: usize = (MAX_LOOP_COUNT + 1) * TX_COUNT;

        let tx_config = TxChannelConfig::default()
            .with_idle_output(true)
            .with_idle_output_level(Level::Low);
        let rx_config = RxChannelConfig::default()
            .with_idle_threshold(1000)
            .with_memsize(2);

        let (mut tx_channel, mut rx_channel) = ctx.setup_loopback(tx_config, rx_config);

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

            if loopstop == LoopStop::Manual {
                tx_transaction.stop_next().unwrap();
                rx_transaction.wait().unwrap();
            } else {
                // tx should stop automatically, so rx should also stop before we explicitly stop
                // tx
                rx_transaction.wait().unwrap();
                tx_transaction.stop_next().unwrap();
            }

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

            check_data_eq(&expected_data, &rx_data, 1000, 1);
        }
    }

    // Test that continuous tx with a finite loopcount works as expected when using manual
    // stopping (on devices that have the required hardware support).
    #[cfg(not(esp32))]
    #[test]
    fn rmt_loopback_continuous_tx_manual_stop(ctx: Context) {
        rmt_loopback_continuous_tx_impl(ctx, esp_hal::rmt::LoopStop::Manual);
    }

    // Test that continuous tx with a finite loopcount works as expected when using automatic
    // stopping (on devices that have the required hardware support).
    #[cfg(any(esp32c6, esp32h2, esp32s3))]
    #[test]
    fn rmt_loopback_continuous_tx_auto_stop(ctx: Context) {
        rmt_loopback_continuous_tx_impl(ctx, esp_hal::rmt::LoopStop::Auto);
    }
}
