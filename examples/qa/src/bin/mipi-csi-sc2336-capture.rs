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
use esp_println::println;
use qa_test::camera::{
    self,
    FRAME_BYTES,
    GUARD_BYTES,
    PSRAM_DMA_ALIGNMENT,
    Sc2336,
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
