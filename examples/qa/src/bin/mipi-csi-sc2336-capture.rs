//! Captures RAW8 frames from the SC2336 camera of the ESP32-P4 Function EV
//! Board, one `CsiCapture` receive call per frame, into one PSRAM buffer.
//!
//! Every frame must overwrite the whole buffer and leave the guard words after
//! it untouched. In interrupt mode, the capture channel's own completion
//! interrupt must also end the wait.
//!
//! Selected at build time:
//! - `CSI_FRAMES=1..=1000` (default 1): frames to capture.
//! - `CSI_COMPLETION`: `poll` (default) waits with `receive_with`, `interrupt` with
//!   `receive_with_interrupt`.
//! - `CSI_SENSOR_CONTINUOUS=1`: start the sensor stream once, instead of around every frame.
//! - `CSI_TEST_PATTERN=1`: the sensor test pattern. With the sensor started for every frame, all
//!   frames must be equal.
//! - `CSI_FRAME_EXPORT=1`: print the last frame as `@@SC2336FRAME` text lines for
//!   `examples/qa/tools/csi_frame_extract.py`. About 860 KB of console output.
//!
//! The run ends with a `QA_RESULT: PASS` or `QA_RESULT: FAIL` line.
//% CHIP_FILTER: mipi_csi_driver_supported
//% FEATURES: unstable

#![no_std]
#![no_main]

extern crate alloc;

use alloc::vec::Vec;
use core::alloc::Layout;

use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{
    main,
    mipi_csi::{CaptureConfig, Config, MipiCsi},
    psram,
};
use esp_println::{print, println};
use qa_test::camera::{
    self,
    FRAME_BYTES,
    GUARD_BYTES,
    HEIGHT,
    PSRAM_DMA_ALIGNMENT,
    Sc2336,
    WIDTH,
    checksum32,
    fill_sentinel,
    sentinel_words,
};

esp_bootloader_esp_idf::esp_app_desc!();

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

    let frames: u32 = option_env!("CSI_FRAMES").map_or(1, |n| n.parse().unwrap());
    assert!((1..=1000).contains(&frames), "CSI_FRAMES must be 1..=1000");
    let interrupt = match option_env!("CSI_COMPLETION") {
        None | Some("poll") => false,
        Some("interrupt") => true,
        Some(other) => panic!("unknown CSI_COMPLETION '{other}', use poll or interrupt"),
    };
    let continuous = flag(option_env!("CSI_SENSOR_CONTINUOUS"));
    let test_pattern = flag(option_env!("CSI_TEST_PATTERN"));
    let export = flag(option_env!("CSI_FRAME_EXPORT"));
    println!(
        "capture: frames={frames} interrupt={interrupt} sensor_continuous={continuous} test_pattern={test_pattern}"
    );

    // Built with the defaults, so the run also covers `apply_config`.
    let mut csi = MipiCsi::new(peripherals.MIPI_CSI, Config::default()).unwrap();
    csi.apply_config(&camera::csi_config()).unwrap();
    let mut capture = csi
        .with_dma(peripherals.VDMA_CH1, CaptureConfig::default())
        .unwrap();
    capture.apply_config(&camera::capture_config()).unwrap();
    let mut sensor = Sc2336::new(
        peripherals.I2C0,
        peripherals.GPIO7,
        peripherals.GPIO8,
        test_pattern,
    );

    let layout = Layout::from_size_align(FRAME_BYTES + GUARD_BYTES, PSRAM_DMA_ALIGNMENT).unwrap();
    let buffer = unsafe { alloc::alloc::alloc(layout) };
    assert!(!buffer.is_null(), "frame buffer allocation failed");
    let buffer = unsafe { core::slice::from_raw_parts_mut(buffer, layout.size()) };
    let (frame, guard) = buffer.split_at_mut(FRAME_BYTES);

    if continuous {
        sensor.set_stream(true).unwrap();
    }
    let mut checksums = Vec::new();
    for index in 1..=frames {
        fill_sentinel(frame);
        fill_sentinel(guard);
        let mut stream_on = Ok(());
        let start_sensor = || {
            if !continuous {
                stream_on = sensor.set_stream(true);
            }
        };
        let result = if interrupt {
            capture.receive_with_interrupt(frame, start_sensor)
        } else {
            capture.receive_with(frame, start_sensor)
        };
        let stream_off = if continuous {
            Ok(())
        } else {
            sensor.set_stream(false)
        };

        let untouched = sentinel_words(frame);
        let guard_written = GUARD_BYTES / 4 - sentinel_words(guard);
        let report = capture.completion_interrupt_report();
        let irq_ok = !interrupt || (report.fired && report.own_source);
        let ok = result.is_ok()
            && untouched == 0
            && guard_written == 0
            && irq_ok
            && stream_on.is_ok()
            && stream_off.is_ok();
        let checksum = checksum32(frame);
        checksums.push(checksum);

        if !ok || frames <= 10 || index % 10 == 0 {
            // Not `{:08x}` with a `0x` prefix: the monitor resolves those as addresses.
            println!(
                "frame[{index}/{frames}]: ok={ok} untouched_words={untouched} guard_written_words={guard_written} checksum32={checksum:08x}"
            );
        }
        if !ok {
            println!(
                "frame[{index}/{frames}]: result={result:?} completion_irq={report:?} stream_on={stream_on:?} stream_off={stream_off:?}"
            );
            let _ = sensor.set_stream(false);
            qa_result(false, frames, interrupt);
        }
    }
    if continuous {
        sensor.set_stream(false).unwrap();
    }

    let mut distinct = checksums.clone();
    distinct.sort_unstable();
    distinct.dedup();
    println!(
        "capture: {frames} frames, distinct_checksums={}",
        distinct.len()
    );
    if test_pattern && !continuous && distinct.len() != 1 {
        println!("capture: test pattern frames differ");
        qa_result(false, frames, interrupt);
    }

    if export {
        export_frame(&mut sensor, frame, test_pattern);
    }
    qa_result(true, frames, interrupt)
}

/// The last line of a run, printed only after every check.
fn qa_result(pass: bool, frames: u32, interrupt: bool) -> ! {
    println!(
        "QA_RESULT: {} test=capture frames={frames} interrupt={interrupt}",
        if pass { "PASS" } else { "FAIL" }
    );
    loop {}
}

const EXPORT_PREFIX: &str = "@@SC2336FRAME";
const EXPORT_CHUNK_BYTES: usize = 512;
const EXPORT_BASE64_BYTES: usize = EXPORT_CHUNK_BYTES.div_ceil(3) * 4;
const EXPORT_FRAME_ID: u32 = 1;
const SC2336_REG_FLIP_MIRROR: u16 = 0x3221;
const BASE64_ALPHABET: &[u8; 64] =
    b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

/// CRC-32 (IEEE), the same as Python's `zlib.crc32`.
fn crc32(data: &[u8]) -> u32 {
    let mut crc = !0u32;
    for &byte in data {
        crc ^= byte as u32;
        for _ in 0..8 {
            crc = if crc & 1 != 0 {
                (crc >> 1) ^ 0xEDB8_8320
            } else {
                crc >> 1
            };
        }
    }
    !crc
}

fn base64_encode<'a>(input: &[u8], output: &'a mut [u8; EXPORT_BASE64_BYTES]) -> &'a str {
    let mut len = 0;
    for group in input.chunks(3) {
        let n = (group[0] as u32) << 16
            | (group.get(1).copied().unwrap_or(0) as u32) << 8
            | group.get(2).copied().unwrap_or(0) as u32;
        for (i, slot) in output[len..len + 4].iter_mut().enumerate() {
            *slot = if i <= group.len() {
                BASE64_ALPHABET[(n >> (18 - 6 * i)) as usize & 63]
            } else {
                b'='
            };
        }
        len += 4;
    }
    core::str::from_utf8(&output[..len]).unwrap()
}

/// Prints `frame` as text lines that `examples/qa/tools/csi_frame_extract.py`
/// turns back into the raw frame.
///
/// espflash decodes the console as text, so the payload is base64. Numbers are
/// hex without a `0x` prefix, which espflash would try to resolve as symbols.
fn export_frame(sensor: &mut Sc2336<'_>, frame: &[u8], test_pattern: bool) {
    // The Bayer order of the official mode table assumes mirror and flip are off.
    // `--` tells the host that the register could not be read.
    let crc = crc32(frame);
    let chunks = frame.len().div_ceil(EXPORT_CHUNK_BYTES);
    print!("{EXPORT_PREFIX} BEGIN v=1 id={EXPORT_FRAME_ID} ");
    match sensor.read(SC2336_REG_FLIP_MIRROR) {
        Ok(value) => print!("reg3221={value:02x} "),
        Err(_) => print!("reg3221=-- "),
    }
    println!(
        "width={WIDTH} height={HEIGHT} format=RAW8 bayer=BGGR bayer_source=esp-cam-sensor:MIPI_2lane_24Minput_RAW8_1024x600_30fps test_pattern={} buffer={:08x} len={} chunk_bytes={EXPORT_CHUNK_BYTES} chunks={chunks} crc32={crc:08x} checksum32={:08x}",
        test_pattern as u8,
        frame.as_ptr() as u32,
        frame.len(),
        checksum32(frame),
    );

    let mut encoded = [0u8; EXPORT_BASE64_BYTES];
    for (index, chunk) in frame.chunks(EXPORT_CHUNK_BYTES).enumerate() {
        println!(
            "{EXPORT_PREFIX} D {EXPORT_FRAME_ID} {index} {:x} {:08x} {}",
            index * EXPORT_CHUNK_BYTES,
            crc32(chunk),
            base64_encode(chunk, &mut encoded)
        );
    }

    println!(
        "{EXPORT_PREFIX} END id={EXPORT_FRAME_ID} len={} chunks={chunks} crc32={crc:08x}",
        frame.len()
    );
}
