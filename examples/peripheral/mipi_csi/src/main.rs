//! Minimal MIPI CSI-2 bring-up smoke test for ESP32-P4
//!
//! This only initializes the CSI HOST/BRIDGE blocks, which also powers the D-PHY rail.
//! It does not configure the camera sensor or start capture.
//!
//! Diagnostics output from `MipiCsi::init_diagnostics()` is enabled by the
//! `diagnostics` feature (on by default for this example package).

//% CHIP_FILTER: mipi_csi_driver_supported

#![no_std]
#![no_main]

use esp_backtrace as _;
#[cfg(feature = "diagnostics")]
use esp_hal::mipi_csi::PhyCfgClockSource;
use esp_hal::{
    main,
    mipi_csi::{Config, DataLanes, MipiCsi},
};
use esp_println::println;

esp_bootloader_esp_idf::esp_app_desc!();

#[cfg(feature = "diagnostics")]
fn phy_cfg_clk_source_name(source: PhyCfgClockSource) -> &'static str {
    match source {
        PhyCfgClockSource::PllF20m => "pll_f20m",
        PhyCfgClockSource::RcFast => "rc_fast",
        PhyCfgClockSource::PllF25m => "pll_f25m",
        PhyCfgClockSource::Unknown(_) => "unknown",
    }
}

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    esp_println::logger::init_logger_from_env();

    let config = Config::default()
        .with_num_data_lanes(DataLanes::_1)
        .with_lane_bit_rate_mbps(800);

    println!("Initializing MIPI CSI");
    #[cfg(feature = "diagnostics")]
    let mipi_csi = MipiCsi::new(peripherals.MIPI_CSI, config).unwrap();
    #[cfg(not(feature = "diagnostics"))]
    let _mipi_csi = MipiCsi::new(peripherals.MIPI_CSI, config).unwrap();
    println!("MIPI CSI initialized successfully");

    #[cfg(feature = "diagnostics")]
    {
        let diagnostics = mipi_csi.init_diagnostics();

        println!("--- Configuration Readback ---");
        println!(
            "host bus clock enabled: {}",
            diagnostics.config.host_bus_clock_enabled
        );
        println!(
            "bridge bus clock enabled: {}",
            diagnostics.config.bridge_bus_clock_enabled
        );
        println!(
            "dphy cfg clock enabled: {}",
            diagnostics.config.phy_cfg_clock_enabled
        );
        println!(
            "dphy cfg clock source: {}",
            phy_cfg_clk_source_name(diagnostics.config.phy_cfg_clock_source)
        );
        println!(
            "bridge internal clock enabled: {}",
            diagnostics.config.bridge_internal_clock_enabled
        );
        println!(
            "host bus reset asserted: {}",
            diagnostics.config.host_bus_reset_asserted
        );
        println!(
            "bridge bus reset asserted: {}",
            diagnostics.config.bridge_bus_reset_asserted
        );
        println!(
            "bridge soft reset asserted: {}",
            diagnostics.config.bridge_soft_reset_asserted
        );
        println!(
            "phy shutdown asserted (active-low): {}",
            diagnostics.config.phy_shutdown_asserted
        );
        println!(
            "dphy reset asserted (active-low): {}",
            diagnostics.config.dphy_reset_asserted
        );
        println!(
            "csi2 reset asserted (active-low): {}",
            diagnostics.config.csi2_controller_reset_asserted
        );
        println!(
            "configured data lanes: {}",
            diagnostics.config.configured_data_lanes
        );
        println!(
            "virtual channel extension enabled: {}",
            diagnostics.config.virtual_channel_extension_enabled
        );
        println!(
            "scrambling enabled: {}",
            diagnostics.config.scrambling_enabled
        );
        println!("bridge enabled: {}", diagnostics.config.bridge_enabled);

        println!("--- Hardware Status / Events ---");
        println!(
            "host any error/status bit set: {} (raw=0x{:08x})",
            diagnostics.status.host_errors.any_error(),
            diagnostics.status.host_errors.raw_main_status,
        );
        println!(
            "bridge any event/status bit set: {} (raw=0x{:08x})",
            diagnostics.status.bridge_events.any_event(),
            diagnostics.status.bridge_events.raw_status,
        );
        println!(
            "host status detail: phy_fatal={}, packet_fatal={}, bndry_frame_fatal={}, \
             seq_frame_fatal={}, crc_frame_fatal={}, payload_crc_fatal={}, data_id_error={}, \
             ecc_corrected={}, phy_error={}",
            diagnostics.status.host_errors.phy_fatal,
            diagnostics.status.host_errors.packet_fatal,
            diagnostics.status.host_errors.boundary_frame_fatal,
            diagnostics.status.host_errors.sequence_frame_fatal,
            diagnostics.status.host_errors.crc_frame_fatal,
            diagnostics.status.host_errors.payload_crc_fatal,
            diagnostics.status.host_errors.data_id_error,
            diagnostics.status.host_errors.ecc_corrected,
            diagnostics.status.host_errors.phy_error,
        );
        println!(
            "bridge status detail: vadr_num_gt={}, vadr_num_lt={}, discard={}, \
             csi_buf_overrun={}, csi_async_fifo_ovf={}, dma_cfg_has_updated={}",
            diagnostics.status.bridge_events.vadr_num_gt,
            diagnostics.status.bridge_events.vadr_num_lt,
            diagnostics.status.bridge_events.discard,
            diagnostics.status.bridge_events.csi_buf_overrun,
            diagnostics.status.bridge_events.csi_async_fifo_ovf,
            diagnostics.status.bridge_events.dma_cfg_has_updated,
        );
        println!("NOTE: register readback confirms software configuration only.");
        println!("NOTE: without a connected camera, CSI link/packet reception is not validated.");
    }

    #[cfg(not(feature = "diagnostics"))]
    println!(
        "CSI diagnostics disabled; enable the `diagnostics` feature to print readback/status."
    );

    loop {}
}
