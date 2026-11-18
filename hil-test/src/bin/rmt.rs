//! RMT Loopback Test
//!
//! When running this with defmt, consider increasing embedded_test's default_timeout below to
//! avoid timeouts while printing. Note that probe-rs reading the buffer might still mess up
//! timing-sensitive tests! This doesn't apply to `check_data_eq` since it is used after the
//! action, but adding any additional logging to the driver is likely to cause sporadic issues.
//!
//! The loopback tests receive on an async channel, which the executor polls while a blocking
//! transmission runs. A blocking transmission does not let the executor read the RX RAM, so its
//! frames fit in the RX RAM.

//% CHIP_FILTER: rmt_driver_supported
//% FEATURES: embassy esp-alloc unstable

#![no_std]
#![no_main]

extern crate alloc;

use core::{pin::pin, task::Poll};

use allocator_api2::{vec, vec::Vec};
use embassy_futures::{join::join, poll_once};
use esp_hal::{
    Async,
    Blocking,
    delay::Delay,
    gpio::{
        AnyPin,
        Flex,
        InputConfig,
        Level,
        OutputConfig,
        Pin,
        Pull,
        interconnect::{InputSignal, OutputSignal},
    },
    peripherals::RMT,
    ram,
    rmt::{
        CHANNEL_RAM_SIZE,
        ConfigError,
        HAS_RX_WRAP,
        LoopMode,
        PulseCode,
        Rmt,
        RmtConfig,
        RxChannel,
        RxConfig,
        RxError,
        RxInstance,
        TxChannel,
        TxConfig,
        TxError,
        TxInstance,
    },
    time::{Duration, Instant, Rate},
    timer::timg::TimerGroup,
};
#[allow(unused_imports)]
use hil_test::{assert, assert_eq, assert_matches};

// RMT channel clock = 500kHz
cfg_select! {
    any(esp32h2, esp32h4) => {
        const FREQ: Rate = Rate::from_mhz(32);
        const DIV: u32 = 64;
    }
    _ => {
        const FREQ: Rate = Rate::from_mhz(80);
        const DIV: u32 = 160;
    }
}

// The channel singletons. Each test starts from a fresh `esp_hal::init`, so stealing them is
// sound.
macro_rules! ch {
    ($name:ident) => {
        unsafe { esp_hal::peripherals::$name::steal() }
    };
}

macro_rules! tx_1 {
    () => {
        ch!(RMT_CH0)
    };
}

macro_rules! tx_2 {
    () => {
        (ch!(RMT_CH0), ch!(RMT_CH1))
    };
}

cfg_select! {
    any(esp32, esp32s3, esp32s31, esp32p4) => {
        macro_rules! rx_1 {
            () => {
                ch!(RMT_CH4)
            };
        }
        // All RX blocks after the TX channels of the tests.
        macro_rules! rx_all {
            () => {
                (ch!(RMT_CH4), ch!(RMT_CH5), ch!(RMT_CH6), ch!(RMT_CH7))
            };
        }
        const RX_ALL_CODES: usize = 4 * CHANNEL_RAM_SIZE;
    }
    _ => {
        macro_rules! rx_1 {
            () => {
                ch!(RMT_CH2)
            };
        }
        macro_rules! rx_all {
            () => {
                (ch!(RMT_CH2), ch!(RMT_CH3))
            };
        }
        const RX_ALL_CODES: usize = 2 * CHANNEL_RAM_SIZE;
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
    write_stop_code: bool,
    end_marker: EndMarkerConfig,
    tolerance: u16,
    length1_base: u16,
    length1_step: u16,
    length2: u16,
    sclk_div: u32, // 1 - 256
    ch_div: u32,
}

impl Default for LoopbackConfig {
    fn default() -> Self {
        Self {
            tx_len: CHANNEL_RAM_SIZE / 2,
            idle_threshold: 1000,
            write_stop_code: true,
            end_marker: EndMarkerConfig::Field1,
            tolerance: 1,
            length1_base: 100,
            length1_step: 10,
            length2: 50,
            sclk_div: 1,
            ch_div: DIV,
        }
    }
}

impl LoopbackConfig {
    fn sclk_freq(&self) -> Rate {
        FREQ / self.sclk_div
    }

    fn tick_rate(&self) -> Rate {
        self.sclk_freq() / self.ch_div
    }

    fn rmt_config(&self) -> RmtConfig {
        #[allow(unused_mut)]
        let mut config = RmtConfig::default();
        #[cfg(not(rmt_has_per_channel_clock))]
        {
            config = config.with_frequency(self.sclk_freq());
        }
        config
    }

    fn tx_config(&self) -> TxConfig {
        TxConfig::default().with_tick_rate(self.tick_rate())
    }

    fn rx_config(&self) -> RxConfig {
        let idle_ns =
            u64::from(self.idle_threshold) * 1_000_000_000 / u64::from(self.tick_rate().as_hz());
        RxConfig::default()
            .with_tick_rate(self.tick_rate())
            .with_idle_ns(idle_ns as u32)
    }
}

// Pulses of H 100..330 L 50, i.e. 100..330 / 500kHz = 100..330 * 2us = 200..660us
//
// Note that there's no small common multiple of `CHANNEL_RAM_SIZE` (either 48 or 64) and the
// period 23, so we don't repeatedly write the same data to the same location in the hardware
// buffer in loopback tests with buffer wrapping. If we did, that might hide bugs.
fn generate_tx_data(conf: &LoopbackConfig) -> Vec<PulseCode> {
    const WRAP_COUNT: u16 = 23;

    let mut tx_data: Vec<_> = (0..)
        .take(conf.tx_len)
        .map(|i| {
            PulseCode::try_new(
                Level::High,
                conf.length1_base + conf.length1_step * (i % WRAP_COUNT),
                Level::Low,
                conf.length2,
            )
            .expect("pulse code length out of range")
        })
        .collect();
    let mut i_max = WRAP_COUNT.min(conf.tx_len as u16);

    let mut pos = conf.tx_len - 1;
    match conf.end_marker {
        EndMarkerConfig::None => (),
        EndMarkerConfig::Field1 => {
            tx_data[pos] = PulseCode::end_marker();
            pos -= 1;
            i_max -= 1;
        }
        EndMarkerConfig::Field2 => {
            tx_data[pos] = tx_data[pos].with_length2(0).unwrap();
            pos -= 1;
            i_max -= 1;
        }
    }
    if conf.write_stop_code {
        i_max -= 1;
        assert!(conf.length1_base + i_max * conf.length1_step + conf.length2 < conf.idle_threshold);

        tx_data[pos] = PulseCode::try_new(
            Level::High,
            2 * conf.idle_threshold,
            Level::Low,
            conf.length2,
        )
        .expect("idle_threshold out of range");
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

// Compare tx and rx data from a loopback transmission.
fn check_data_eq(
    conf: &LoopbackConfig,
    tx: &[PulseCode],
    rx: &[PulseCode],
    rx_res: Result<usize, RxError>,
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
            // The rx buffer should be untouched, we check that below.
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
        }
    }

    match rx_res {
        Ok(rx_count) => {
            assert_eq!(
                rx_count,
                expected_rx_len,
                "unexpected rx count (last: tx={:?}, rx={:?})",
                tx[rx_count.max(1) - 1],
                rx[rx_count.max(1) - 1],
            );
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
    cycles_per_us: u32,
}

impl BitbangTransmitter {
    const SCALE: u32 = 1024;

    fn new(pin: impl Pin + 'static, conf: &LoopbackConfig) -> Self {
        let mut pin = Flex::new(pin);
        pin.set_input_enable(true);
        pin.set_output_enable(true);
        pin.apply_input_config(&InputConfig::default().with_pull(Pull::None));
        pin.apply_output_config(&OutputConfig::default());

        pin.set_low();

        Self {
            pin,
            cycles_per_us: conf.ch_div * 1000 * Self::SCALE / conf.sclk_freq().as_khz(),
        }
    }

    fn rx_pin(&mut self) -> InputSignal<'static> {
        self.pin.peripheral_input()
    }

    fn transmit(&mut self, tx_data: &[PulseCode]) {
        let delay = Delay::new();

        for code in tx_data {
            if code.length1() == 0 {
                break;
            }
            self.pin.set_level(code.level1());
            delay.delay_micros(code.length1() as u32 * self.cycles_per_us / Self::SCALE);

            if code.length2() == 0 {
                break;
            }
            self.pin.set_level(code.level2());
            delay.delay_micros(code.length2() as u32 * self.cycles_per_us / Self::SCALE);
        }
    }
}

// The transmit channel of a loopback, in either driver mode.
enum Tx<'r, 'c> {
    Blocking(&'r mut TxChannel<'c, Blocking>),
    Async(&'r mut TxChannel<'c, Async>),
}

// Sends data from `tx` and receives it on `rx`, and checks that the data matches.
async fn do_rmt_loopback(conf: &LoopbackConfig, tx: Tx<'_, '_>, rx: &mut RxChannel<'_, Async>) {
    let tx_data = generate_tx_data(conf);
    let mut rx_data = vec![PulseCode::end_marker(); conf.tx_len];

    // `join` polls the receive future first, so the reception starts before the transmission.
    let (rx_res, tx_res) = join(rx.receive_async(&mut rx_data), async {
        match tx {
            Tx::Blocking(tx) => tx.transmit(&tx_data),
            Tx::Async(tx) => tx.transmit_async(&tx_data).await,
        }
    })
    .await;

    tx_res.unwrap();
    check_data_eq(conf, &tx_data, &rx_data, rx_res);
}

fn pins<'a>(pin: &'a mut AnyPin<'static>) -> (InputSignal<'a>, OutputSignal<'a>) {
    Flex::new(pin.reborrow()).split()
}

// Creates the channels of a loopback from one `Rmt`.
fn setup<'a>(
    rmt: &'a Rmt<'_, Async>,
    pin: &'a mut AnyPin<'static>,
    conf: &LoopbackConfig,
    tx: impl TxInstance + 'a,
    rx: impl RxInstance + 'a,
) -> (TxChannel<'a, Blocking>, RxChannel<'a, Async>) {
    let (rx_pin, tx_pin) = pins(pin);
    let tx = rmt
        .tx_channel(tx, conf.tx_config())
        .unwrap()
        .with_pin(tx_pin);
    let rx = rmt
        .async_rx_channel(rx, conf.rx_config())
        .unwrap()
        .with_pin(rx_pin);
    (tx, rx)
}

fn setup_async<'a>(
    rmt: &'a Rmt<'_, Async>,
    pin: &'a mut AnyPin<'static>,
    conf: &LoopbackConfig,
    tx: impl TxInstance + 'a,
    rx: impl RxInstance + 'a,
) -> (TxChannel<'a, Async>, RxChannel<'a, Async>) {
    let (rx_pin, tx_pin) = pins(pin);
    let tx = rmt
        .async_tx_channel(tx, conf.tx_config())
        .unwrap()
        .with_pin(tx_pin);
    let rx = rmt
        .async_rx_channel(rx, conf.rx_config())
        .unwrap()
        .with_pin(rx_pin);
    (tx, rx)
}

// Time that the hardware needs to send `codes`, at 2 us per tick.
fn send_time(codes: &[PulseCode]) -> Duration {
    let ticks: u64 = codes
        .iter()
        .map(|c| u64::from(c.length1()) + u64::from(c.length2()))
        .sum();
    Duration::from_micros(ticks * 2)
}

// SCLK dividers to test; ESP32 and ESP32-S2 don't have an SCLK divider. On ESP32-P4 and
// ESP32-S31, 80 MHz comes from a 160 MHz source, so the largest integer divider is 128.
const SCLK_DIVIDERS: &[u32] = if cfg!(any(esp32, esp32s2)) {
    &[1]
} else if cfg!(any(esp32p4, esp32s31)) {
    &[1, 16, 128]
} else {
    &[1, 16, 256]
};

const CHANNEL_DIVIDERS: &[u32] = &[1, 16, 255];

fn divider_loopback_config(sclk_div: u32, ch_div: u32) -> LoopbackConfig {
    let div = sclk_div * ch_div;
    // the stop code is sent with length 2 * idle_threshold = 40 * step, cf. generate_tx_data
    let step = (32767 / (41 * div)).max(5) as u16;

    LoopbackConfig {
        sclk_div,
        ch_div,
        tx_len: 5,
        length1_base: 10 * step,
        length1_step: step,
        length2: 2 * step,
        idle_threshold: 20 * step,
        ..Default::default()
    }
}

// Starts a transmission longer than the channel RAM, refills it `refills_in_time` times, then
// stops polling. The hardware must stop at the driver's end marker, and the transmission must
// report the underrun. Then the channel must work again.
async fn do_rmt_underrun(ctx: &mut Context, tx_blocks: u8, refills_in_time: usize) {
    let m = tx_blocks as usize * CHANNEL_RAM_SIZE;
    let h = m / 2;
    // The hardware stops after the initial fill (all of RAM but the end marker) and the refills.
    let sent = m - 1 + refills_in_time * h;
    if !HAS_RX_WRAP && sent + 1 > RX_ALL_CODES {
        return;
    }

    let conf = LoopbackConfig {
        tx_len: sent + h + 4,
        ..Default::default()
    };
    let tx_data = generate_tx_data(&conf);
    let mut rx_data = vec![PulseCode::end_marker(); sent + 1];

    let rmt = Rmt::new(ctx.rmt.reborrow(), conf.rmt_config())
        .unwrap()
        .into_async();
    let (mut tx, mut rx) = if tx_blocks == 1 {
        setup_async(&rmt, &mut ctx.pin, &conf, tx_1!(), rx_all!())
    } else {
        setup_async(&rmt, &mut ctx.pin, &conf, tx_2!(), rx_all!())
    };

    let rx_res = {
        let mut rx_fut = pin!(rx.receive_async(&mut rx_data));
        let mut tx_fut = pin!(tx.transmit_async(&tx_data));

        assert!(poll_once(rx_fut.as_mut()).is_pending());
        let start = Instant::now();
        assert!(poll_once(tx_fut.as_mut()).is_pending());

        // Poll TX until half a refill period after the last in-time refill, then stop. RX is
        // polled until the hardware has stopped, so that RX does not overflow.
        let tx_stop = send_time(&tx_data[..refills_in_time * h + h / 2]);
        let rx_stop = send_time(&tx_data[..sent]) + Duration::from_millis(20);
        let mut rx_res = None;
        while start.elapsed() < rx_stop {
            if refills_in_time > 0 && start.elapsed() < tx_stop {
                assert!(poll_once(tx_fut.as_mut()).is_pending());
            }
            if rx_res.is_none()
                && let Poll::Ready(res) = poll_once(rx_fut.as_mut())
            {
                rx_res = Some(res);
            }
        }

        assert_matches!(
            poll_once(tx_fut.as_mut()),
            Poll::Ready(Err(TxError::Underrun))
        );
        match rx_res {
            Some(res) => res,
            None => rx_fut.await,
        }
    };

    let expected = LoopbackConfig {
        tx_len: sent,
        ..LoopbackConfig::default()
    };
    let mut expected_data = tx_data[..sent].to_vec();
    // The output stays low after the last code, so RX ends with it.
    expected_data[sent - 1] = expected_data[sent - 1].with_length2(1100).unwrap();
    check_data_eq(&expected, &expected_data, &rx_data, rx_res);

    // The channel works again after the underrun.
    let again = LoopbackConfig {
        tx_len: (m * 3 / 2).min(RX_ALL_CODES),
        ..Default::default()
    };
    do_rmt_loopback(&again, Tx::Async(&mut tx), &mut rx).await;
}

// Sends data with lengths around the RAM size and its halves.
async fn do_rmt_exact_fit(ctx: &mut Context, tx_blocks: u8) {
    let m = tx_blocks as usize * CHANNEL_RAM_SIZE;
    let h = m / 2;

    for tx_len in [m - 2, m - 1, m, m + 1, m + h - 1, m + h, m + h + 1, 2 * m] {
        if !HAS_RX_WRAP && tx_len > RX_ALL_CODES {
            continue;
        }
        let conf = LoopbackConfig {
            tx_len,
            ..Default::default()
        };

        let rmt = Rmt::new(ctx.rmt.reborrow(), conf.rmt_config())
            .unwrap()
            .into_async();
        let (mut tx, mut rx) = if tx_blocks == 1 {
            setup_async(&rmt, &mut ctx.pin, &conf, tx_1!(), rx_all!())
        } else {
            setup_async(&rmt, &mut ctx.pin, &conf, tx_2!(), rx_all!())
        };
        do_rmt_loopback(&conf, Tx::Async(&mut tx), &mut rx).await;
    }
}

// Receives on a blocking channel. A loop transmission is the stimulus, because it runs without
// the CPU: each repetition starts with a low level that is longer than the idle threshold, so RX
// receives exactly one repetition.
fn do_rmt_blocking_rx(ctx: &mut Context, pattern_len: usize, tx_blocks: u8, rx_blocks: u8) {
    let conf = LoopbackConfig {
        tx_len: pattern_len,
        write_stop_code: false,
        end_marker: EndMarkerConfig::None,
        ..Default::default()
    };
    let pattern = generate_tx_data(&conf);
    let mut tx_data = vec![PulseCode::new(Level::Low, 1500, Level::Low, 1500)];
    tx_data.extend_from_slice(&pattern);

    let rmt = Rmt::new(ctx.rmt.reborrow(), conf.rmt_config()).unwrap();
    let (rx_pin, tx_pin) = pins(&mut ctx.pin);
    let mut tx = match tx_blocks {
        1 => rmt.tx_channel(tx_1!(), conf.tx_config()),
        _ => rmt.tx_channel(tx_2!(), conf.tx_config()),
    }
    .unwrap()
    .with_pin(tx_pin);
    let mut rx = match rx_blocks {
        1 => rmt.rx_channel(rx_1!(), conf.rx_config()),
        _ => rmt.rx_channel(rx_all!(), conf.rx_config()),
    }
    .unwrap()
    .with_pin(rx_pin);

    let mut rx_data = vec![PulseCode::end_marker(); pattern_len];
    let tx_loop = tx.transmit_loop(&tx_data, LoopMode::Infinite).unwrap();
    let rx_res = rx.receive(&mut rx_data);
    tx_loop.stop().unwrap();

    let mut expected = pattern.clone();
    expected[pattern_len - 1] = expected[pattern_len - 1].with_length2(1100).unwrap();
    check_data_eq(&conf, &expected, &rx_data, rx_res);
}

struct Context {
    rmt: RMT<'static>,
    pin: AnyPin<'static>,

    #[cfg(esp32)]
    pin2: AnyPin<'static>,
}

#[embedded_test::tests(default_timeout = 1, executor = hil_test::Executor::new())]
mod tests {
    #[allow(unused_imports)]
    use hil_test::{assert, assert_eq, assert_matches};

    use super::*;

    #[init]
    fn init() -> Context {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        // The reclaimed region (dram2_seg) is only 56 KiB on the H4.
        esp_alloc::heap_allocator!(#[ram(reclaimed)] size: 48 * 1024);

        let timg0 = TimerGroup::new(peripherals.TIMG0);

        esp_rtos::start(timg0.timer0);

        let (pin, _pin2) = hil_test::common_test_pins!(peripherals);

        Context {
            rmt: peripherals.RMT,
            pin: AnyPin::from(pin),
            #[cfg(esp32)]
            pin2: AnyPin::from(_pin2),
        }
    }

    #[test]
    fn rmt_frequency_accessor_reports_configured_rate(mut ctx: Context) {
        let conf = LoopbackConfig::default();
        let rmt = Rmt::new(ctx.rmt.reborrow(), conf.rmt_config()).unwrap();
        assert_eq!(rmt.frequency(), FREQ);

        #[cfg(soc_has_clock_node_rmt_sclk)]
        assert_eq!(
            esp_hal::clock::ll::RmtInstance::Rmt.sclk_frequency(),
            FREQ.as_hz()
        );
    }

    // 3 MHz is not an integer fraction of the source clock: the group clock uses the nearest
    // integer divider, and the transmission runs at the reported frequency.
    #[cfg(not(rmt_has_per_channel_clock))]
    #[test]
    fn rmt_integer_group_divider(mut ctx: Context) {
        const REQUESTED: Rate = Rate::from_mhz(3);
        const CODES: u64 = 4;

        let rmt = Rmt::new(
            ctx.rmt.reborrow(),
            RmtConfig::default().with_frequency(REQUESTED),
        )
        .unwrap();
        let frequency = rmt.frequency();
        assert!(
            frequency.as_hz().abs_diff(REQUESTED.as_hz()) < REQUESTED.as_hz() / 30,
            "group clock {} Hz",
            frequency.as_hz()
        );

        let code = PulseCode::new(
            Level::High,
            PulseCode::MAX_LEN,
            Level::Low,
            PulseCode::MAX_LEN,
        );
        let tx_data = [code; CODES as usize];
        let expected_us =
            CODES * 2 * PulseCode::MAX_LEN as u64 * 1_000_000 / frequency.as_hz() as u64;

        let mut tx = rmt
            .tx_channel(tx_1!(), TxConfig::default())
            .unwrap()
            .with_pin(ctx.pin.reborrow());
        assert_eq!(tx.tick_rate(), frequency);

        let start = Instant::now();
        tx.transmit(&tx_data).unwrap();
        let elapsed_us = start.elapsed().as_micros();

        assert!(
            elapsed_us * 100 >= expected_us * 99 && elapsed_us * 100 <= expected_us * 101,
            "unexpected tx duration: {} us (expected {} us)",
            elapsed_us,
            expected_us
        );
    }

    #[test]
    #[cfg(esp32c6)]
    fn rmt_frequency_is_not_apb_on_default_esp32c6(mut ctx: Context) {
        // Reproducer for #5532: with the default ESP32-C6 clock configuration, APB runs at
        // 40 MHz, while the RMT driver configures the RMT counter clock to 80 MHz. Code that
        // derives pulse lengths from APB will therefore generate pulses that are too short.
        let rmt = Rmt::new(ctx.rmt.reborrow(), RmtConfig::default()).unwrap();

        let apb_frequency = Rate::from_hz(esp_hal::clock::ll::apb_clk_frequency());

        assert_eq!(apb_frequency.as_hz(), 40_000_000);
        assert_eq!(rmt.frequency().as_hz(), 80_000_000);
    }

    #[test]
    fn rmt_tick_rates(mut ctx: Context) {
        let conf = LoopbackConfig::default();
        let rmt = Rmt::new(ctx.rmt.reborrow(), conf.rmt_config()).unwrap();
        let group = rmt.frequency();

        // Channels of one `Rmt` have independent tick rates.
        let mut tx0 = rmt.tx_channel(ch!(RMT_CH0), TxConfig::default()).unwrap();
        let tx1 = rmt.tx_channel(ch!(RMT_CH1), conf.tx_config()).unwrap();
        assert_eq!(tx0.tick_rate(), group);
        assert_eq!(tx1.tick_rate(), conf.tick_rate());

        // The nearest tick period: 7/16 of the group clock gives divider 2.
        tx0.apply_config(&TxConfig::default().with_tick_rate(group * 7 / 16))
            .unwrap();
        assert_eq!(tx0.tick_rate(), group / 2);

        for rate in [
            Rate::from_hz(group.as_hz() + 1),
            group / 257,
            Rate::from_hz(0),
        ] {
            assert_eq!(
                tx0.apply_config(&TxConfig::default().with_tick_rate(rate)),
                Err(ConfigError::UnreachableTickRate)
            );
        }

        let rx = rmt.rx_channel(rx_1!(), conf.rx_config()).unwrap();
        assert_eq!(rx.tick_rate(), conf.tick_rate());
    }

    #[test]
    fn rmt_config_errors(mut ctx: Context) {
        let rmt = Rmt::new(ctx.rmt.reborrow(), LoopbackConfig::default().rmt_config()).unwrap();

        let mut tx = rmt.tx_channel(tx_1!(), TxConfig::default()).unwrap();
        let carrier = esp_hal::rmt::CarrierConfig::default();
        tx.apply_config(&TxConfig::default().with_carrier(carrier))
            .unwrap();
        for carrier in [
            carrier.with_duty_percent(0),
            carrier.with_duty_percent(101),
            carrier.with_frequency(Rate::from_hz(0)),
        ] {
            assert_eq!(
                tx.apply_config(&TxConfig::default().with_carrier(carrier)),
                Err(ConfigError::UnreachableCarrier)
            );
        }

        let mut rx = rmt.rx_channel(rx_1!(), RxConfig::default()).unwrap();
        for config in [
            // Longer than the largest idle threshold at the group clock.
            RxConfig::default().with_idle_ns(1_000_000_000),
            RxConfig::default().with_idle_ns(0),
            // The filter holds at most 255 group clock cycles.
            RxConfig::default().with_min_pulse_ns(1_000_000),
        ] {
            assert_eq!(
                rx.apply_config(&config),
                Err(ConfigError::RxLimitOutOfRange)
            );
        }
    }

    #[test]
    async fn rmt_loopback_simple(mut ctx: Context) {
        // Fit both tx and rx data into a single memory block
        let conf = LoopbackConfig::default();
        let rmt = Rmt::new(ctx.rmt.reborrow(), conf.rmt_config())
            .unwrap()
            .into_async();
        let (mut tx, mut rx) = setup(&rmt, &mut ctx.pin, &conf, tx_1!(), rx_1!());
        do_rmt_loopback(&conf, Tx::Blocking(&mut tx), &mut rx).await;
    }

    #[test]
    async fn rmt_loopback_simple_async(mut ctx: Context) {
        let conf = LoopbackConfig::default();
        let rmt = Rmt::new(ctx.rmt.reborrow(), conf.rmt_config())
            .unwrap()
            .into_async();
        let (mut tx, mut rx) = setup_async(&rmt, &mut ctx.pin, &conf, tx_1!(), rx_1!());
        do_rmt_loopback(&conf, Tx::Async(&mut tx), &mut rx).await;
    }

    #[test]
    async fn rmt_loopback_end_marker_field2(mut ctx: Context) {
        let conf = LoopbackConfig {
            end_marker: EndMarkerConfig::Field2,
            ..Default::default()
        };
        let rmt = Rmt::new(ctx.rmt.reborrow(), conf.rmt_config())
            .unwrap()
            .into_async();
        let (mut tx, mut rx) = setup(&rmt, &mut ctx.pin, &conf, tx_1!(), rx_1!());
        do_rmt_loopback(&conf, Tx::Blocking(&mut tx), &mut rx).await;
    }

    // The driver adds the end marker.
    #[test]
    async fn rmt_loopback_without_end_marker(mut ctx: Context) {
        let rmt = Rmt::new(ctx.rmt.reborrow(), LoopbackConfig::default().rmt_config())
            .unwrap()
            .into_async();
        for tx_len in [CHANNEL_RAM_SIZE / 2, CHANNEL_RAM_SIZE * 3 / 2] {
            let conf = LoopbackConfig {
                tx_len,
                end_marker: EndMarkerConfig::None,
                ..Default::default()
            };
            let (mut tx, mut rx) = setup(&rmt, &mut ctx.pin, &conf, tx_1!(), rx_all!());
            do_rmt_loopback(&conf, Tx::Blocking(&mut tx), &mut rx).await;
        }
    }

    #[test]
    async fn rmt_loopback_extended_ram(mut ctx: Context) {
        // 2 memory blocks for tx data -> no wrapping
        let conf = LoopbackConfig {
            tx_len: CHANNEL_RAM_SIZE * 3 / 2,
            ..Default::default()
        };
        let rmt = Rmt::new(ctx.rmt.reborrow(), conf.rmt_config())
            .unwrap()
            .into_async();
        let (mut tx, mut rx) = setup(&rmt, &mut ctx.pin, &conf, tx_2!(), rx_all!());
        do_rmt_loopback(&conf, Tx::Blocking(&mut tx), &mut rx).await;
    }

    #[test]
    async fn rmt_loopback_tx_wrap(mut ctx: Context) {
        // A tx channel with only 1 block requires wrapping.
        let conf = LoopbackConfig {
            tx_len: CHANNEL_RAM_SIZE * 3 / 2,
            ..Default::default()
        };
        let rmt = Rmt::new(ctx.rmt.reborrow(), conf.rmt_config())
            .unwrap()
            .into_async();
        let (mut tx, mut rx) = setup(&rmt, &mut ctx.pin, &conf, tx_1!(), rx_all!());
        do_rmt_loopback(&conf, Tx::Blocking(&mut tx), &mut rx).await;
    }

    #[test]
    async fn rmt_loopback_tx_wrap_async(mut ctx: Context) {
        let conf = LoopbackConfig {
            tx_len: CHANNEL_RAM_SIZE * 3 / 2,
            ..Default::default()
        };
        let rmt = Rmt::new(ctx.rmt.reborrow(), conf.rmt_config())
            .unwrap()
            .into_async();
        let (mut tx, mut rx) = setup_async(&rmt, &mut ctx.pin, &conf, tx_1!(), rx_all!());
        do_rmt_loopback(&conf, Tx::Async(&mut tx), &mut rx).await;
    }

    #[cfg(rmt_has_rx_wrap)]
    #[test]
    async fn rmt_loopback_rx_wrap_async(mut ctx: Context) {
        // An rx channel with only 1 block requires wrapping.
        let conf = LoopbackConfig {
            tx_len: CHANNEL_RAM_SIZE * 3 / 2,
            ..Default::default()
        };
        let rmt = Rmt::new(ctx.rmt.reborrow(), conf.rmt_config())
            .unwrap()
            .into_async();
        let (mut tx, mut rx) = setup_async(&rmt, &mut ctx.pin, &conf, tx_2!(), rx_1!());
        do_rmt_loopback(&conf, Tx::Async(&mut tx), &mut rx).await;
    }

    #[test]
    fn rmt_blocking_rx(mut ctx: Context) {
        do_rmt_blocking_rx(&mut ctx, CHANNEL_RAM_SIZE / 2, 1, 1);
    }

    // The blocking receive reads the RX RAM while the frame arrives.
    #[cfg(rmt_has_rx_wrap)]
    #[test]
    fn rmt_blocking_rx_wrap(mut ctx: Context) {
        do_rmt_blocking_rx(&mut ctx, CHANNEL_RAM_SIZE * 3 / 2, 2, 1);
    }

    #[test]
    async fn rmt_tx_errors(mut ctx: Context) {
        let conf = LoopbackConfig::default();
        let rmt = Rmt::new(ctx.rmt.reborrow(), conf.rmt_config())
            .unwrap()
            .into_async();

        let mut tx = rmt.tx_channel(ch!(RMT_CH0), conf.tx_config()).unwrap();
        let mut tx_async = rmt
            .async_tx_channel(ch!(RMT_CH1), conf.tx_config())
            .unwrap();

        // Empty data returns at once.
        assert_eq!(tx.transmit(&[]), Ok(()));
        assert_eq!(tx_async.transmit_async(&[]).await, Ok(()));
        tx.transmit_loop(&[], LoopMode::Infinite)
            .unwrap()
            .stop()
            .unwrap();

        // Loop mode does not refill: the data and the end marker must fit in the RAM.
        let data = [PulseCode::new(Level::High, 100, Level::Low, 100); CHANNEL_RAM_SIZE];
        assert_matches!(
            tx.transmit_loop(&data, LoopMode::Infinite),
            Err(TxError::DataTooLong)
        );

        #[cfg(rmt_has_tx_loop_count)]
        assert_matches!(
            tx.transmit_loop(
                &data[..1],
                LoopMode::InfiniteWithInterrupt(esp_hal::rmt::MAX_TX_LOOPCOUNT + 1)
            ),
            Err(TxError::LoopCountOutOfRange)
        );
    }

    // A code with a zero length ends the transmission, also after the first RAM fill.
    #[test]
    async fn rmt_zero_length_code_ends_early(mut ctx: Context) {
        let rmt = Rmt::new(ctx.rmt.reborrow(), LoopbackConfig::default().rmt_config())
            .unwrap()
            .into_async();
        for end in [10, CHANNEL_RAM_SIZE + 10] {
            let conf = LoopbackConfig {
                tx_len: end + 1,
                ..Default::default()
            };
            let (mut tx, mut rx) = setup(&rmt, &mut ctx.pin, &conf, tx_1!(), rx_all!());

            let mut tx_data = generate_tx_data(&conf);
            // Codes after the end marker must not be sent.
            tx_data.extend_from_slice(&[PulseCode::new(Level::High, 100, Level::Low, 100); 20]);
            let mut rx_data = vec![PulseCode::end_marker(); tx_data.len()];

            let (rx_res, tx_res) = join(rx.receive_async(&mut rx_data), async {
                tx.transmit(&tx_data)
            })
            .await;
            assert_eq!(tx_res, Ok(()));
            check_data_eq(&conf, &tx_data, &rx_data, rx_res);
        }
    }

    #[test]
    #[timeout(3)]
    async fn rmt_underrun_initial_1_block(mut ctx: Context) {
        do_rmt_underrun(&mut ctx, 1, 0).await;
    }

    #[test]
    #[timeout(3)]
    async fn rmt_underrun_after_refill_1_block(mut ctx: Context) {
        do_rmt_underrun(&mut ctx, 1, 1).await;
    }

    #[test]
    #[timeout(3)]
    async fn rmt_underrun_initial_2_blocks(mut ctx: Context) {
        do_rmt_underrun(&mut ctx, 2, 0).await;
    }

    #[test]
    #[timeout(3)]
    async fn rmt_underrun_after_refill_2_blocks(mut ctx: Context) {
        do_rmt_underrun(&mut ctx, 2, 1).await;
    }

    #[test]
    #[timeout(5)]
    async fn rmt_exact_fit_1_block(mut ctx: Context) {
        do_rmt_exact_fit(&mut ctx, 1).await;
    }

    #[test]
    #[timeout(5)]
    async fn rmt_exact_fit_2_blocks(mut ctx: Context) {
        do_rmt_exact_fit(&mut ctx, 2).await;
    }

    macro_rules! test_channel_pair {
        ($ctx:expr, $tx:ident, $rx:ident) => {{
            let conf = LoopbackConfig::default();
            let rmt = Rmt::new($ctx.rmt.reborrow(), conf.rmt_config())
                .unwrap()
                .into_async();
            let (mut tx, mut rx) = setup(&rmt, &mut $ctx.pin, &conf, ch!($tx), ch!($rx));
            do_rmt_loopback(&conf, Tx::Blocking(&mut tx), &mut rx).await;
        }};
    }

    // A simple test that uses all channels: This is useful to verify that all channel/register
    // indexing in the driver is correct. The other tests are prone to hide related issues due to
    // using low channel numbers only (in particular 0 for the tx channel).
    #[test]
    async fn rmt_use_all_channels(mut ctx: Context) {
        // Chips with combined RxTx channels
        #[cfg(esp32)]
        {
            test_channel_pair!(ctx, RMT_CH0, RMT_CH1);
            test_channel_pair!(ctx, RMT_CH0, RMT_CH2);
            test_channel_pair!(ctx, RMT_CH0, RMT_CH3);
            test_channel_pair!(ctx, RMT_CH0, RMT_CH4);
            test_channel_pair!(ctx, RMT_CH0, RMT_CH5);
            test_channel_pair!(ctx, RMT_CH0, RMT_CH6);
            test_channel_pair!(ctx, RMT_CH0, RMT_CH7);

            test_channel_pair!(ctx, RMT_CH1, RMT_CH0);
            test_channel_pair!(ctx, RMT_CH2, RMT_CH0);
            test_channel_pair!(ctx, RMT_CH3, RMT_CH0);
            test_channel_pair!(ctx, RMT_CH4, RMT_CH0);
            test_channel_pair!(ctx, RMT_CH5, RMT_CH0);
            test_channel_pair!(ctx, RMT_CH6, RMT_CH0);
            test_channel_pair!(ctx, RMT_CH7, RMT_CH0);
        }

        #[cfg(esp32s2)]
        {
            test_channel_pair!(ctx, RMT_CH0, RMT_CH1);
            test_channel_pair!(ctx, RMT_CH0, RMT_CH2);
            test_channel_pair!(ctx, RMT_CH0, RMT_CH3);

            test_channel_pair!(ctx, RMT_CH1, RMT_CH0);
            test_channel_pair!(ctx, RMT_CH2, RMT_CH0);
            test_channel_pair!(ctx, RMT_CH3, RMT_CH0);
        }

        // Chips with separate Rx and Tx channels
        #[cfg(any(esp32s3, esp32s31, esp32p4))]
        {
            test_channel_pair!(ctx, RMT_CH0, RMT_CH4);
            test_channel_pair!(ctx, RMT_CH1, RMT_CH5);
            test_channel_pair!(ctx, RMT_CH2, RMT_CH6);
            test_channel_pair!(ctx, RMT_CH3, RMT_CH7);
        }

        #[cfg(not(any(esp32, esp32s2, esp32s3, esp32s31, esp32p4)))]
        {
            test_channel_pair!(ctx, RMT_CH0, RMT_CH2);
            test_channel_pair!(ctx, RMT_CH1, RMT_CH3);
        }
    }

    // Test that the timing is at least roughly correct (i.e. the clock source isn't completely
    // misconfigured).
    // This test is almost guaranteed to fail when using defmt!
    #[test]
    async fn rmt_clock_rate(ctx: Context) {
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

        let rmt = Rmt::new(ctx.rmt, conf.rmt_config()).unwrap().into_async();

        let mut bitbang_tx = BitbangTransmitter::new(ctx.pin, &conf);
        let mut rx = rmt
            .async_rx_channel(rx_1!(), conf.rx_config())
            .unwrap()
            .with_pin(bitbang_tx.rx_pin());

        let tx_data = generate_tx_data(&conf);
        let mut rx_data = [PulseCode::end_marker(); 6];

        // The receive future is polled first, so the reception starts before the bitbanging.
        let (rx_res, ()) = join(rx.receive_async(&mut rx_data), async {
            bitbang_tx.transmit(&tx_data);
        })
        .await;

        check_data_eq(&conf, &tx_data, &rx_data, rx_res);
    }

    // Test that dropping rx/tx futures returns the hardware to a predictable state from which
    // subsequent transactions are successful.
    #[test]
    async fn rmt_loopback_after_drop_async(mut ctx: Context) {
        let conf = LoopbackConfig::default();
        let rmt = Rmt::new(ctx.rmt.reborrow(), conf.rmt_config())
            .unwrap()
            .into_async();
        let (mut tx, mut rx) = setup_async(&rmt, &mut ctx.pin, &conf, tx_1!(), rx_1!());

        // Start loopback once, but abort before completion...
        let tx_data = generate_tx_data(&conf);
        let mut rx_data = vec![PulseCode::end_marker(); conf.tx_len];
        {
            let mut rx_fut = pin!(rx.receive_async(&mut rx_data));
            let mut tx_fut = pin!(tx.transmit_async(&tx_data));
            assert!(poll_once(rx_fut.as_mut()).is_pending());
            assert!(poll_once(tx_fut.as_mut()).is_pending());

            Delay::new().delay_millis(2);

            // The test should fail here when the delay above is increased, e.g. to 100ms.
            assert!(poll_once(rx_fut.as_mut()).is_pending());
            assert!(poll_once(tx_fut.as_mut()).is_pending());
        }

        // ...then start over and check that everything still works as expected (i.e. we
        // didn't leave the hardware in an unexpected state or lock up when
        // dropping the futures).
        do_rmt_loopback(&conf, Tx::Async(&mut tx), &mut rx).await;
    }

    // Test that dropping a loop transmission stops it, so that the channel works again.
    #[test]
    async fn rmt_loopback_after_loop_drop(mut ctx: Context) {
        let conf = LoopbackConfig::default();
        let rmt = Rmt::new(ctx.rmt.reborrow(), conf.rmt_config())
            .unwrap()
            .into_async();
        let (mut tx, mut rx) = setup(&rmt, &mut ctx.pin, &conf, tx_1!(), rx_1!());

        let data = [PulseCode::new(Level::High, 100, Level::Low, 100); 4];
        let tx_loop = tx.transmit_loop(&data, LoopMode::Infinite).unwrap();
        Delay::new().delay_millis(2);
        core::mem::drop(tx_loop);

        do_rmt_loopback(&conf, Tx::Blocking(&mut tx), &mut rx).await;
    }

    // Test that completed transactions leave the hardware in a predictable state from which
    // subsequent transactions are successful.
    #[test]
    #[timeout(3)]
    async fn rmt_loopback_repeat(mut ctx: Context) {
        let conf = LoopbackConfig::default();

        // Test that dropping & recreating Rmt works
        for _ in 0..3 {
            let rmt = Rmt::new(ctx.rmt.reborrow(), conf.rmt_config())
                .unwrap()
                .into_async();

            // Test that dropping & recreating the channels works
            for _ in 0..3 {
                let (mut tx, mut rx) = setup(&rmt, &mut ctx.pin, &conf, tx_1!(), rx_1!());
                for _ in 0..3 {
                    do_rmt_loopback(&conf, Tx::Blocking(&mut tx), &mut rx).await;
                }
            }
            for _ in 0..3 {
                let (mut tx, mut rx) = setup_async(&rmt, &mut ctx.pin, &conf, tx_1!(), rx_1!());
                for _ in 0..3 {
                    do_rmt_loopback(&conf, Tx::Async(&mut tx), &mut rx).await;
                }
            }
        }
    }

    #[cfg(not(esp32))]
    async fn rmt_loopback_continuous_tx_impl(mut ctx: Context, use_autostop: bool) {
        const TX_COUNT: usize = 10;
        const MAX_LOOP_COUNT: usize = 3;
        const MAX_RX_LEN: usize = (MAX_LOOP_COUNT + 1) * TX_COUNT;

        let mut conf = LoopbackConfig {
            tx_len: TX_COUNT + 1,
            write_stop_code: false,
            ..Default::default()
        };

        let rmt = Rmt::new(ctx.rmt.reborrow(), conf.rmt_config())
            .unwrap()
            .into_async();
        let (mut tx, mut rx) = setup(&rmt, &mut ctx.pin, &conf, tx_1!(), rx_all!());

        let tx_data = generate_tx_data(&conf);
        let mut rx_data = vec![PulseCode::end_marker(); MAX_RX_LEN];
        let mut expected_data = vec![PulseCode::end_marker(); MAX_RX_LEN];

        for loopcount in 1..MAX_LOOP_COUNT {
            #[allow(unused_mut)]
            let mut loopmode = LoopMode::InfiniteWithInterrupt(loopcount as u16);
            #[cfg(rmt_has_tx_loop_auto_stop)]
            if use_autostop {
                loopmode = LoopMode::Finite(loopcount as u16);
            };
            rx_data.fill(PulseCode::end_marker());

            // `join` polls the receive future first, so the reception starts before the loop.
            let (rx_res, ()) = join(rx.receive_async(&mut rx_data), async {
                let tx_loop = tx.transmit_loop(&tx_data, loopmode).unwrap();
                while !tx_loop.is_loopcount_interrupt_set() {
                    embassy_futures::yield_now().await;
                }
                if !use_autostop {
                    tx_loop.stop_next().unwrap();
                }
            })
            .await;

            let rx_loopcount = if use_autostop {
                loopcount
            } else {
                loopcount + 1
            };
            expected_data.fill(PulseCode::end_marker());
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
            conf.tx_len = TX_COUNT + 1;
        }
    }

    // Test that continuous tx with a finite loopcount works as expected when using manual
    // stopping (on devices that have the required hardware support).
    #[cfg(not(esp32))]
    #[test]
    async fn rmt_loopback_continuous_tx_manual_stop(ctx: Context) {
        rmt_loopback_continuous_tx_impl(ctx, false).await;
    }

    // Test that continuous tx with a finite loopcount works as expected when using automatic
    // stopping (on devices that have the required hardware support).
    #[cfg(rmt_has_tx_loop_auto_stop)]
    #[test]
    async fn rmt_loopback_continuous_tx_auto_stop(ctx: Context) {
        rmt_loopback_continuous_tx_impl(ctx, true).await;
    }

    // Regression test for continuous tx with a slow SCLK, where the loop count interrupt often
    // never fired, see esp-rs/esp-hal#3930.
    #[cfg(all(rmt_has_tx_loop_count, not(rmt_has_per_channel_clock)))]
    #[test]
    #[timeout(3)]
    fn rmt_continuous_tx_slow_sclk(mut ctx: Context) {
        const LOOPS: u16 = 10;
        const CODE_LENGTH: u16 = 289;

        let sclk = FREQ / 256;

        let tx_data = [PulseCode::new(
            Level::High,
            CODE_LENGTH,
            Level::Low,
            CODE_LENGTH,
        )];

        let rmt = Rmt::new(
            ctx.rmt.reborrow(),
            RmtConfig::default().with_frequency(sclk),
        )
        .unwrap();
        let expected_us =
            (LOOPS as u64 * 2 * CODE_LENGTH as u64 * 1_000_000) / rmt.frequency().as_hz() as u64;

        // Recreate the channel for every transaction, as in the original report.
        for _ in 0..10 {
            let mut tx = rmt
                .tx_channel(tx_1!(), TxConfig::default())
                .unwrap()
                .with_pin(ctx.pin.reborrow());

            #[cfg(rmt_has_tx_loop_auto_stop)]
            let loopmode = LoopMode::Finite(LOOPS);
            #[cfg(not(rmt_has_tx_loop_auto_stop))]
            let loopmode = LoopMode::InfiniteWithInterrupt(LOOPS);

            let start = Instant::now();
            let tx_loop = tx.transmit_loop(&tx_data, loopmode).unwrap();

            while !tx_loop.is_loopcount_interrupt_set() {}
            let elapsed_us = start.elapsed().as_micros();

            tx_loop.stop_next().unwrap();

            assert!(
                elapsed_us * 10 >= expected_us * 9 && elapsed_us * 10 <= expected_us * 11,
                "unexpected tx duration: {} us (expected {} us)",
                elapsed_us,
                expected_us
            );
        }
    }

    // Test that using loopcount 0 doesn't hang, but returns success immediately.
    #[cfg(rmt_has_tx_loop_auto_stop)]
    #[test]
    fn rmt_continuous_tx_zero_loopcount(mut ctx: Context) {
        let conf = LoopbackConfig::default();
        let rmt = Rmt::new(ctx.rmt.reborrow(), conf.rmt_config()).unwrap();
        let mut tx = rmt
            .tx_channel(tx_1!(), conf.tx_config())
            .unwrap()
            .with_pin(ctx.pin.reborrow());

        // Use long pulse codes, such that we can use a relatively low threshold in the assertion
        // below to ensure that this test is robust against small timing variations
        let tx_data = [
            PulseCode::new(Level::High, 10_000, Level::Low, 10_000),
            PulseCode::new(Level::High, 10_000, Level::Low, 10_000),
        ];

        let start = Instant::now();

        let tx_loop = tx.transmit_loop(&tx_data, LoopMode::Finite(0)).unwrap();

        while !tx_loop.is_loopcount_interrupt_set() {}

        tx_loop.stop_next().unwrap();

        // overall, this should complete in less time than sending a single code from `tx_data`
        // would take (see above, the codes are several 2 * 10ms long)
        assert!(
            start.elapsed().as_micros() < 1000,
            "tx with loopcount 0 did not complete immediately"
        );
    }

    // Regression test for esp-rs/esp-hal#4697
    //
    // This tests ESP32-specific (mis-)behavior
    #[cfg(esp32)]
    #[test]
    async fn rmt_check_regression_4697(mut ctx: Context) {
        let rmt = Rmt::new(ctx.rmt.reborrow(), RmtConfig::default())
            .unwrap()
            .into_async();

        let (rx_pin, tx_pin) = (ctx.pin, ctx.pin2);

        // A tick of 255 source clock cycles and an idle threshold of 10000 ticks.
        let tick_rate = FREQ / 255;
        let idle_ns = (10_000 * 1_000_000_000 / u64::from(tick_rate.as_hz())) as u32;
        let mut rx = rmt
            .async_rx_channel(
                rx_1!(),
                RxConfig::default()
                    .with_tick_rate(tick_rate)
                    .with_idle_ns(idle_ns),
            )
            .unwrap()
            .with_pin(rx_pin);

        let mut spi = esp_hal::spi::master::Spi::new(
            unsafe { esp_hal::peripherals::SPI2::steal() },
            Default::default(),
        )
        .unwrap()
        .with_sck(tx_pin)
        .into_async();

        let mut buf = [0u8; 1000];

        // This code is to drive the hardware into the state that caused the
        // originally failed assertion.
        //
        // We are "spamming" RMT with pulses by feeding it the SPI SCK.
        //
        // While not really explainable from any documentation, this turned out to drive the
        // hardware into an unexpected state.
        //
        // With the previous assert in place, interestingly the first two receive calls failed as
        // expected but then we ran into the issue where the assert failed instead of seeing
        // the error bit set.
        join(
            async {
                let mut data = [PulseCode::default(); 10];
                for _ in 0..3 {
                    assert!(rx.receive_async(&mut data).await.is_err());
                }
            },
            async {
                spi.transfer_in_place_async(&mut buf).await.unwrap();
            },
        )
        .await;
    }

    // Repeated transactions on the same channels are required to trigger some of the issues with
    // slow SCLK.
    #[test]
    #[timeout(10)]
    async fn rmt_dividers(mut ctx: Context) {
        for &sclk_div in SCLK_DIVIDERS {
            for &ch_div in CHANNEL_DIVIDERS {
                let conf = divider_loopback_config(sclk_div, ch_div);

                let rmt = Rmt::new(ctx.rmt.reborrow(), conf.rmt_config())
                    .unwrap()
                    .into_async();
                let (mut tx, mut rx) = setup(&rmt, &mut ctx.pin, &conf, tx_1!(), rx_1!());
                for _ in 0..3 {
                    do_rmt_loopback(&conf, Tx::Blocking(&mut tx), &mut rx).await;
                }
            }
        }
    }

    #[test]
    #[timeout(10)]
    async fn rmt_dividers_async(mut ctx: Context) {
        for &sclk_div in SCLK_DIVIDERS {
            for &ch_div in CHANNEL_DIVIDERS {
                let conf = divider_loopback_config(sclk_div, ch_div);

                let rmt = Rmt::new(ctx.rmt.reborrow(), conf.rmt_config())
                    .unwrap()
                    .into_async();
                let (mut tx, mut rx) = setup_async(&rmt, &mut ctx.pin, &conf, tx_1!(), rx_1!());
                for _ in 0..3 {
                    do_rmt_loopback(&conf, Tx::Async(&mut tx), &mut rx).await;
                }
            }
        }
    }
}
