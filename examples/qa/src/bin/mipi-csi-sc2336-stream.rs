//! Streams RAW8 frames from the SC2336 camera of the ESP32-P4 Function EV Board
//! with `CsiStream`, into two PSRAM buffers that the DMA interrupt handler
//! alternates between, while the sensor streams throughout.
//!
//! Each buffer is followed by guard words, and both start filled with a
//! sentinel that the first frame in each buffer must fully overwrite. Frame
//! sequence numbers must increase, and the stream must report no errored frame.
//!
//! Selected at build time:
//! - `CSI_FRAMES` (default 100): frames to take.
//! - `CSI_ASYNC=1`: take the frames with `next_frame_async`, polled again only after its waker
//!   fired. Then drop a pending future, restart the stream, and capture one frame after
//!   `into_blocking`.
//! - `CSI_FAULTS=1`: after the frames, check the failure cases. Blocking only: a `next_frame`
//!   timeout with the sensor stopped, a consumer that holds a frame, and stop and start at varying
//!   points of the frame. Both modes: sensor stop and start in the middle of a frame while the
//!   stream stays armed.
//! - `CSI_ROUNDS` (default 1): repeat the run in one boot.
//! - `CSI_TEST_PATTERN=1`: the sensor test pattern. While the sensor streams, the pattern varies by
//!   one LSB in part of the image, so every frame must be within one LSB of the first.
//!
//! The run ends with a `QA_RESULT: PASS` or `QA_RESULT: FAIL` line.
//% CHIP_FILTER: mipi_csi_driver_supported
//% FEATURES: unstable

#![no_std]
#![no_main]

extern crate alloc;

use alloc::vec::Vec;
use core::{alloc::Layout, pin::pin};

use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{
    Async,
    delay::Delay,
    main,
    mipi_csi::{CsiCapture, CsiStream, MipiCsi, StreamError},
    peripherals::VDMA,
    psram,
    time::{Duration, Instant},
};
use esp_println::println;
use qa_test::camera::{
    self,
    FRAME_BYTES,
    GUARD_BYTES,
    PSRAM_DMA_ALIGNMENT,
    Sc2336,
    count_sentinel,
    fill_sentinel,
    is_pending,
    poll_on_wake,
    sentinel_words,
    woken_polls,
};

esp_bootloader_esp_idf::esp_app_desc!();

type Buffers = [&'static mut [u8]; 2];

const FRAME_TIMEOUT: Duration = Duration::from_millis(500);
/// Stop and start cycles of the restart case.
const RESTART_CYCLES: u32 = 20;
/// Sensor stop and start rounds while one stream stays armed.
const SENSOR_RESTART_ROUNDS: u32 = 5;
/// How long the slow consumer holds a frame, about nine sensor frames.
const HOLD_MS: u64 = 300;
/// The capture runs on `VDMA_CH1`.
const CSI_DMA_CHANNEL: usize = 1;

fn flag(value: Option<&str>) -> bool {
    value == Some("1")
}

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    esp_alloc::psram_allocator!(
        peripherals.PSRAM,
        esp_hal::psram,
        psram::PsramConfig::default()
    );

    let frames: u32 = option_env!("CSI_FRAMES").map_or(100, |n| n.parse().unwrap());
    let rounds: u32 = option_env!("CSI_ROUNDS").map_or(1, |n| n.parse().unwrap());
    let use_async = flag(option_env!("CSI_ASYNC"));
    let faults = flag(option_env!("CSI_FAULTS"));
    let test_pattern = flag(option_env!("CSI_TEST_PATTERN"));
    println!(
        "stream: frames={frames} rounds={rounds} async={use_async} faults={faults} test_pattern={test_pattern}"
    );

    let mut capture = MipiCsi::new(peripherals.MIPI_CSI, camera::csi_config())
        .unwrap()
        .with_dma(peripherals.VDMA_CH1, camera::capture_config())
        .unwrap();
    let mut sensor = Sc2336::new(
        peripherals.I2C0,
        peripherals.GPIO7,
        peripherals.GPIO8,
        test_pattern,
    );

    let len = 2 * (FRAME_BYTES + GUARD_BYTES);
    let layout = Layout::from_size_align(len, PSRAM_DMA_ALIGNMENT).unwrap();
    let region = unsafe { alloc::alloc::alloc(layout) };
    assert!(!region.is_null(), "stream buffer allocation failed");
    let region = unsafe { core::slice::from_raw_parts_mut(region, len) };
    let (first, second) = region.split_at_mut(FRAME_BYTES + GUARD_BYTES);
    let (buffer0, guard0) = first.split_at_mut(FRAME_BYTES);
    let (buffer1, guard1) = second.split_at_mut(FRAME_BYTES);
    let mut buffers: Buffers = [buffer0, buffer1];
    let mut guards: Buffers = [guard0, guard1];

    let mut failed_rounds = 0;
    for round in 1..=rounds {
        guards.iter_mut().for_each(|guard| fill_sentinel(guard));
        let mut ok;
        (capture, buffers, ok) = if use_async {
            stream_async(capture, buffers, &mut sensor, frames, test_pattern, faults)
        } else {
            stream_blocking(capture, buffers, &mut sensor, frames, test_pattern)
        };
        if faults && !use_async && ok {
            (capture, buffers, ok) = fault_cases(capture, buffers, &mut sensor);
        }
        let guards_clean = guards
            .iter_mut()
            .all(|guard| sentinel_words(guard) == GUARD_BYTES / 4);
        ok &= guards_clean;
        println!("round[{round}/{rounds}]: ok={ok} guards_clean={guards_clean}");
        failed_rounds += u32::from(!ok);
    }

    println!(
        "QA_RESULT: {} test=stream async={use_async} faults={faults} frames={frames} rounds={rounds} failed_rounds={failed_rounds}",
        if failed_rounds == 0 { "PASS" } else { "FAIL" }
    );
    loop {}
}

/// Checks of the frames of one stream run.
struct FrameLog {
    buffers: [usize; 2],
    first_fill_untouched: [Option<usize>; 2],
    last_sequence: Option<u32>,
    sequences_increase: bool,
    reference: Option<Vec<u8>>,
    pattern_max_delta: u8,
    delivered: u32,
}

impl FrameLog {
    /// Fills `buffers` with the sentinel, for the first-frame check.
    fn new(buffers: &mut Buffers, test_pattern: bool) -> Self {
        buffers.iter_mut().for_each(|buffer| fill_sentinel(buffer));
        Self {
            buffers: [buffers[0].as_ptr() as usize, buffers[1].as_ptr() as usize],
            first_fill_untouched: [None; 2],
            last_sequence: None,
            sequences_increase: true,
            reference: test_pattern.then(Vec::new),
            pattern_max_delta: 0,
            delivered: 0,
        }
    }

    fn record(&mut self, frame: &[u8], sequence: u32) {
        let slot = usize::from(frame.as_ptr() as usize == self.buffers[1]);
        self.first_fill_untouched[slot].get_or_insert_with(|| count_sentinel(frame));
        self.sequences_increase &= self.last_sequence.is_none_or(|last| sequence > last);
        self.last_sequence = Some(sequence);
        if let Some(reference) = &mut self.reference {
            if reference.is_empty() {
                reference.extend_from_slice(frame);
            }
            let delta = reference
                .iter()
                .zip(frame)
                .map(|(a, b)| a.abs_diff(*b))
                .max()
                .unwrap_or(0);
            self.pattern_max_delta = self.pattern_max_delta.max(delta);
        }
        self.delivered += 1;
    }

    fn passed(&self, label: &str, frames: u32) -> bool {
        let ok = self.delivered == frames
            && self.sequences_increase
            && self
                .first_fill_untouched
                .iter()
                .all(|untouched| untouched.is_none_or(|words| words == 0))
            && self.pattern_max_delta <= 1;
        println!(
            "{label}: delivered={}/{frames} sequences_increase={} first_fill_untouched_words={:?} test_pattern_max_delta={}",
            self.delivered,
            self.sequences_increase,
            self.first_fill_untouched,
            self.pattern_max_delta
        );
        ok
    }
}

fn stream_blocking<'d>(
    capture: CsiCapture<'d>,
    mut buffers: Buffers,
    sensor: &mut Sc2336<'_>,
    frames: u32,
    test_pattern: bool,
) -> (CsiCapture<'d>, Buffers, bool) {
    let mut log = FrameLog::new(&mut buffers, test_pattern);
    let mut stream = match capture.into_stream(buffers) {
        Ok(stream) => stream,
        Err((capture, buffers, error)) => {
            println!("stream: did not start: {error:?}");
            return (capture, buffers, false);
        }
    };
    let stream_on = sensor.set_stream(true);
    let start = Instant::now();
    let mut error = None;
    for _ in 0..frames {
        match stream.next_frame(FRAME_TIMEOUT) {
            Ok(frame) => log.record(&frame, frame.sequence()),
            Err(e) => {
                error = Some(e);
                break;
            }
        }
    }
    let elapsed_ms = start.elapsed().as_millis().max(1);
    let stats = stream.stats();
    let stream_off = sensor.set_stream(false);
    let (capture, buffers) = stream.stop();

    println!(
        "stream: error={error:?} {stats:?} delivered_frames_per_s_x10={}",
        log.delivered as u64 * 10_000 / elapsed_ms
    );
    let ok = log.passed("stream", frames)
        && error.is_none()
        && stats.errored == 0
        && stats.resyncs == 0
        && stream_on.is_ok()
        && stream_off.is_ok();
    (capture, buffers, ok)
}

/// Takes one frame with `next_frame_async`, polled only after a wake.
fn next_async(stream: &mut CsiStream<'_, Async>, log: Option<&mut FrameLog>) -> bool {
    match poll_on_wake(pin!(stream.next_frame_async()), FRAME_TIMEOUT) {
        Some(Ok(frame)) => {
            if let Some(log) = log {
                log.record(&frame, frame.sequence());
            }
            true
        }
        Some(Err(error)) => {
            println!("async: {error:?}");
            false
        }
        None => {
            println!("async: no wake within {FRAME_TIMEOUT:?}");
            false
        }
    }
}

fn stream_async<'d>(
    capture: CsiCapture<'d>,
    mut buffers: Buffers,
    sensor: &mut Sc2336<'_>,
    frames: u32,
    test_pattern: bool,
    faults: bool,
) -> (CsiCapture<'d>, Buffers, bool) {
    let mut log = FrameLog::new(&mut buffers, test_pattern);
    let mut stream = match capture.into_async().into_stream(buffers) {
        Ok(stream) => stream,
        Err((capture, buffers, error)) => {
            println!("async: did not start: {error:?}");
            return (capture.into_blocking(), buffers, false);
        }
    };
    let stream_on = sensor.set_stream(true);
    let woken_before = woken_polls();
    for _ in 0..frames {
        if !next_async(&mut stream, Some(&mut log)) {
            break;
        }
    }
    let woken = woken_polls() - woken_before;

    // The frame just taken was the newest, so a new future is pending until
    // the next frame completes. Dropping it must not lose that frame.
    let cancel_pending = is_pending(pin!(stream.next_frame_async()));
    let after_cancel = (0..5).filter(|_| next_async(&mut stream, None)).count();

    // The first block after a restart can be the tail of the frame the stop
    // cut, which the stream drops.
    let (capture, buffers) = stream.stop();
    let mut stream = match capture.into_stream(buffers) {
        Ok(stream) => stream,
        Err((capture, buffers, error)) => {
            println!("async: did not restart: {error:?}");
            let _ = sensor.set_stream(false);
            return (capture.into_blocking(), buffers, false);
        }
    };
    let after_restart = (0..5).filter(|_| next_async(&mut stream, None)).count();
    let sensor_restart_ok = !faults || sensor_restart(sensor, || next_async(&mut stream, None));
    let stats = stream.stats();
    let stream_off = sensor.set_stream(false);
    let (capture, buffers) = stream.stop();

    // The sensor stop can cut a frame, and the host then reports an error at
    // the next frame start. One whole sensor frame before arming lets that
    // error latch before the capture, which clears it.
    let mut capture = capture.into_blocking();
    let settle_on = sensor.set_stream(true);
    Delay::new().delay_millis(50);
    let [buffer0, buffer1] = buffers;
    let blocking = capture.receive_with(buffer0, || {});
    let settle_off = sensor.set_stream(false);

    println!(
        "async: woken_polls={woken} cancel_pending={cancel_pending} after_cancel={after_cancel}/5 after_restart={after_restart}/5 sensor_restart_ok={sensor_restart_ok} {stats:?} blocking_after_into_blocking={blocking:?}"
    );
    let ok = log.passed("async", frames)
        && woken > 0
        && cancel_pending
        && after_cancel == 5
        && after_restart == 5
        && sensor_restart_ok
        && stats.errored == stats.resyncs
        && stream_on.is_ok()
        && stream_off.is_ok()
        && settle_on.is_ok()
        && settle_off.is_ok()
        && blocking.is_ok();
    (capture, [buffer0, buffer1], ok)
}

/// Stops and restarts the sensor in the middle of a frame, at a different
/// point each round, while the stream stays armed. The stream must drop the
/// cut frame and deliver whole frames again. `next` takes one frame.
fn sensor_restart(sensor: &mut Sc2336<'_>, mut next: impl FnMut() -> bool) -> bool {
    let delay = Delay::new();
    let mut rounds_ok = 0;
    for round in 0..SENSOR_RESTART_ROUNDS {
        // `next` returns at a frame end, so this moves the stop into the next
        // frame.
        let stop_delay_ms = 10 + round * 5;
        delay.delay_millis(stop_delay_ms);
        let _ = sensor.set_stream(false);
        delay.delay_millis(100);
        let _ = sensor.set_stream(true);
        let frames = (0..5).filter(|_| next()).count();
        if frames == 5 {
            rounds_ok += 1;
        } else {
            println!(
                "sensor-restart: round {round} stop_delay_ms={stop_delay_ms} frames={frames}/5"
            );
        }
    }
    rounds_ok == SENSOR_RESTART_ROUNDS
}

fn fault_case(name: &str, ok: bool, detail: core::fmt::Arguments<'_>) -> bool {
    println!(
        "fault[{name}]: {} {detail}",
        if ok { "PASS" } else { "FAIL" }
    );
    ok
}

/// Failure cases of a blocking `CsiStream`, starting with the sensor stopped.
fn fault_cases<'d>(
    mut capture: CsiCapture<'d>,
    mut buffers: Buffers,
    sensor: &mut Sc2336<'_>,
) -> (CsiCapture<'d>, Buffers, bool) {
    macro_rules! start {
        ($capture:expr, $buffers:expr) => {
            match $capture.into_stream($buffers) {
                Ok(stream) => stream,
                Err((capture, buffers, error)) => {
                    println!("fault: stream did not start: {error:?}");
                    let _ = sensor.set_stream(false);
                    return (capture, buffers, false);
                }
            }
        };
    }
    let delay = Delay::new();
    let mut passed = true;

    // No sensor frames: `next_frame` times out, and stopping a stream whose DMA
    // is armed but never started writing leaves both buffers untouched.
    buffers.iter_mut().for_each(|buffer| fill_sentinel(buffer));
    let mut stream = start!(capture, buffers);
    let start = Instant::now();
    let result = stream.next_frame(Duration::from_millis(200)).map(|_| ());
    let waited_ms = start.elapsed().as_millis();
    let stats = stream.stats();
    (capture, buffers) = stream.stop();
    let untouched = buffers
        .iter_mut()
        .all(|buffer| sentinel_words(buffer) == FRAME_BYTES / 4);
    passed &= fault_case(
        "timeout",
        result == Err(StreamError::Timeout)
            && (200..=210).contains(&waited_ms)
            && stats.completed == 0
            && untouched,
        format_args!(
            "result={result:?} waited_ms={waited_ms} {stats:?} buffers_untouched={untouched}"
        ),
    );

    // Slow consumer: hold one frame while the sensor streams. The DMA
    // destination never enters the held buffer, and newer frames overwrite
    // each other in the other buffer.
    let mut stream = start!(capture, buffers);
    let stream_on = sensor.set_stream(true);
    // The first block can be the tail of the frame the previous stop cut.
    let warm_up = stream.next_frame(FRAME_TIMEOUT).map(|_| ());
    let before = stream.stats();
    let held = stream.next_frame(FRAME_TIMEOUT).map(|frame| {
        let range = frame.as_ptr() as u32..frame.as_ptr() as u32 + frame.len() as u32;
        let hold_start = Instant::now();
        let (mut samples, mut inside) = (0u32, 0u32);
        while hold_start.elapsed() < Duration::from_millis(HOLD_MS) {
            let dar = VDMA::regs().ch(CSI_DMA_CHANNEL).dar0().read().bits();
            samples += 1;
            inside += u32::from(range.contains(&dar));
        }
        (samples, inside)
    });
    let next = stream.next_frame(FRAME_TIMEOUT).map(|_| ());
    let after = stream.stats();
    let overwritten = after.overwritten - before.overwritten;
    passed &= fault_case(
        "slow-consumer",
        stream_on.is_ok()
            && warm_up.is_ok()
            && held.is_ok_and(|(samples, inside)| samples > 0 && inside == 0)
            && next.is_ok()
            && overwritten >= 5
            && after.errored == before.errored,
        format_args!(
            "(dar_samples, dar_inside_held)={held:?} next={next:?} overwritten_during_hold={overwritten}"
        ),
    );

    // Stop and start with the sensor streaming, each stop at a different point
    // of the frame, in 1.7 ms steps. A short first block after a start is
    // dropped and resynced.
    let mut cycles_ok = 0;
    let (mut errored, mut resyncs) = (0, 0);
    for cycle in 0..RESTART_CYCLES {
        let stop_delay_us = cycle * 1_700 % 33_000;
        delay.delay_micros(stop_delay_us);
        if cycle > 0 {
            let stats = stream.stats();
            errored += stats.errored;
            resyncs += stats.resyncs;
        }
        (capture, buffers) = stream.stop();
        stream = start!(capture, buffers);
        let frames = (0..2)
            .filter(|_| stream.next_frame(FRAME_TIMEOUT).is_ok())
            .count();
        if frames == 2 {
            cycles_ok += 1;
        } else {
            println!(
                "fault[restart]: cycle {cycle} stop_delay_us={stop_delay_us} frames={frames}/2"
            );
        }
    }
    let stats = stream.stats();
    errored += stats.errored;
    resyncs += stats.resyncs;
    (capture, buffers) = stream.stop();
    passed &= fault_case(
        "restart",
        cycles_ok == RESTART_CYCLES && errored == resyncs && errored <= RESTART_CYCLES,
        format_args!("cycles_ok={cycles_ok}/{RESTART_CYCLES} errored={errored} resyncs={resyncs}"),
    );

    let mut stream = start!(capture, buffers);
    let warm_up = stream.next_frame(FRAME_TIMEOUT).map(|_| ());
    let restart_ok = sensor_restart(sensor, || stream.next_frame(FRAME_TIMEOUT).is_ok());
    let stats = stream.stats();
    let stream_off = sensor.set_stream(false);
    (capture, buffers) = stream.stop();
    passed &= fault_case(
        "sensor-restart",
        warm_up.is_ok()
            && restart_ok
            && stats.errored == stats.resyncs
            && stats.errored <= SENSOR_RESTART_ROUNDS
            && stream_off.is_ok(),
        format_args!("rounds_ok={restart_ok} {stats:?}"),
    );

    (capture, buffers, passed)
}
