//! Checks the `VDD_MIPI_DPHY` rail that the MIPI-CSI and MIPI-DSI drivers
//! share, on the ESP32-P4 Function EV Board with the SC2336 camera.
//!
//! Creates CSI alone, DSI alone, then both in either creation and drop order,
//! and repeats the combined cases. After every step the PMU rail must be on
//! exactly while a driver lives, a live CSI driver must capture a full RAW8
//! frame, and a live DSI driver must keep its PHY PLL locked.
//!
//! The run ends with a `QA_RESULT: PASS` or `QA_RESULT: FAIL` line.
//% CHIP_FILTER: mipi_csi_driver_supported && mipi_dsi_driver_supported
//% FEATURES: unstable

#![no_std]
#![no_main]

extern crate alloc;

use alloc::{format, string::String};
use core::alloc::Layout;

use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{
    clock::ll::{MipiDsiPhyPllRefclkConfig, MipiDsiPhyPllRefclkSclk},
    main,
    mipi_csi::{CsiCapture, MipiCsi},
    mipi_dsi::{Config as DsiConfig, DataLanes as DsiDataLanes, MipiDsi},
    peripherals::{MIPI_CSI, MIPI_DSI, MIPI_DSI_HOST, PMU, VDMA_CH0, VDMA_CH1},
    psram,
};
use esp_println::println;
use qa_test::camera::{self, FRAME_BYTES, PSRAM_DMA_ALIGNMENT, Sc2336};

esp_bootloader_esp_idf::esp_app_desc!();

/// Cycles of the combined cases after the first pass.
const REPEATS: u32 = 3;

struct Rig<'d> {
    csi: MIPI_CSI<'d>,
    csi_channel: VDMA_CH1<'d>,
    dsi: MIPI_DSI<'d>,
    dsi_channel: VDMA_CH0<'d>,
    sensor: Sc2336<'d>,
    buffer: &'static mut [u8],
}

fn new_csi<'a>(csi: &'a mut MIPI_CSI<'_>, channel: &'a mut VDMA_CH1<'_>) -> CsiCapture<'a> {
    MipiCsi::new(csi.reborrow(), camera::csi_config())
        .unwrap()
        .with_dma(channel.reborrow(), camera::capture_config())
        .unwrap()
}

fn new_dsi<'a>(dsi: &'a mut MIPI_DSI<'_>, channel: &'a mut VDMA_CH0<'_>) -> MipiDsi<'a> {
    MipiDsi::new(
        dsi.reborrow(),
        channel.reborrow(),
        DsiConfig::default()
            .with_num_data_lanes(DsiDataLanes::_2)
            .with_lane_bit_rate_mbps(1000.0)
            .with_phy_pll_refclk(MipiDsiPhyPllRefclkConfig::new(
                MipiDsiPhyPllRefclkSclk::Xtal,
                0,
            )),
    )
    .expect("MipiDsi init failed")
}

fn report(step: &str, what: &str, ok: bool) -> bool {
    println!(
        "dphy-lifecycle[{step}]: {what} {}",
        if ok { "ok" } else { "FAIL" }
    );
    ok
}

/// The PMU drives `VDD_MIPI_DPHY` (VO3 `xpd`, bit 8 of `ext_ldo_p0_0p2a`).
fn rail(step: &str, expect_on: bool) -> bool {
    let on = PMU::regs().ext_ldo_p0_0p2a().read().bits() & (1 << 8) != 0;
    report(
        step,
        &format!("rail_on={on} expected={expect_on}"),
        on == expect_on,
    )
}

fn frame(
    step: &str,
    capture: &mut CsiCapture<'_>,
    sensor: &mut Sc2336<'_>,
    buffer: &mut [u8],
) -> bool {
    let result = capture
        .receive_with(buffer, || sensor.set_stream(true).unwrap())
        .map(|frame| frame.received_bytes);
    let _ = sensor.set_stream(false);
    report(
        step,
        &format!("csi_frame={result:?}"),
        result.is_ok_and(|bytes| bytes == FRAME_BYTES),
    )
}

/// Only read while a `MipiDsi` lives, so that the DSI host is clocked.
fn dsi_locked(step: &str) -> bool {
    let locked = MIPI_DSI_HOST::regs()
        .phy_status()
        .read()
        .phy_lock()
        .bit_is_set();
    report(step, &format!("dsi_pll_locked={locked}"), locked)
}

/// Creates CSI and DSI in the order `csi_first`, and drops them in the order
/// `drop_csi_first`, checking the rail and both drivers after every step.
fn pair(label: &str, rig: &mut Rig<'_>, csi_first: bool, drop_csi_first: bool) -> bool {
    let Rig {
        csi,
        csi_channel,
        dsi,
        dsi_channel,
        sensor,
        buffer,
    } = rig;
    let mut ok = true;
    let (mut capture, bus) = if csi_first {
        let capture = new_csi(csi, csi_channel);
        ok &= rail(&format!("{label}/csi-up"), true);
        (capture, new_dsi(dsi, dsi_channel))
    } else {
        let bus = new_dsi(dsi, dsi_channel);
        ok &= rail(&format!("{label}/dsi-up"), true);
        (new_csi(csi, csi_channel), bus)
    };
    ok &= rail(&format!("{label}/both-up"), true);
    ok &= frame(&format!("{label}/both-up"), &mut capture, sensor, buffer);
    ok &= dsi_locked(&format!("{label}/both-up"));
    if drop_csi_first {
        drop(capture);
        ok &= rail(&format!("{label}/csi-dropped"), true);
        ok &= dsi_locked(&format!("{label}/csi-dropped"));
        drop(bus);
    } else {
        drop(bus);
        ok &= rail(&format!("{label}/dsi-dropped"), true);
        ok &= frame(
            &format!("{label}/dsi-dropped"),
            &mut capture,
            sensor,
            buffer,
        );
        drop(capture);
    }
    ok & rail(&format!("{label}/both-dropped"), false)
}

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    esp_alloc::psram_allocator!(
        peripherals.PSRAM,
        esp_hal::psram,
        psram::PsramConfig::default()
    );

    let layout = Layout::from_size_align(FRAME_BYTES, PSRAM_DMA_ALIGNMENT).unwrap();
    let buffer = unsafe { alloc::alloc::alloc(layout) };
    assert!(!buffer.is_null(), "frame buffer allocation failed");
    let mut rig = Rig {
        csi: peripherals.MIPI_CSI,
        csi_channel: peripherals.VDMA_CH1,
        dsi: peripherals.MIPI_DSI,
        dsi_channel: peripherals.VDMA_CH0,
        sensor: Sc2336::new(
            peripherals.I2C0,
            peripherals.GPIO7,
            peripherals.GPIO8,
            false,
        ),
        buffer: unsafe { core::slice::from_raw_parts_mut(buffer, FRAME_BYTES) },
    };

    let mut ok = rail("start", false);

    let mut capture = new_csi(&mut rig.csi, &mut rig.csi_channel);
    ok &= rail("1-csi-alone/up", true);
    ok &= frame("1-csi-alone/up", &mut capture, &mut rig.sensor, rig.buffer);
    drop(capture);
    ok &= rail("1-csi-alone/dropped", false);

    let bus = new_dsi(&mut rig.dsi, &mut rig.dsi_channel);
    ok &= rail("2-dsi-alone/up", true);
    ok &= dsi_locked("2-dsi-alone/up");
    drop(bus);
    ok &= rail("2-dsi-alone/dropped", false);

    let cases = [
        ("3-csi-dsi-drop-dsi", true, false),
        ("4-csi-dsi-drop-csi", true, true),
        ("5-dsi-csi-drop-csi", false, true),
        ("6-dsi-csi-drop-dsi", false, false),
    ];
    for cycle in 0..=REPEATS {
        for (name, csi_first, drop_csi_first) in cases {
            let label = if cycle == 0 {
                String::from(name)
            } else {
                format!("7-repeat{cycle}-{name}")
            };
            ok &= pair(&label, &mut rig, csi_first, drop_csi_first);
        }
    }

    println!(
        "QA_RESULT: {} test=dphy_lifecycle repeats={REPEATS}",
        if ok { "PASS" } else { "FAIL" }
    );
    loop {}
}
