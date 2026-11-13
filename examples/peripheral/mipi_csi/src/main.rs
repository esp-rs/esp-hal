//! Minimal MIPI CSI-2 bring-up smoke test for ESP32-P4
//!
//! This only initializes the CSI HOST/BRIDGE blocks, which also powers the D-PHY rail.
//! It does not configure the camera sensor or start capture.

//% CHIP_FILTER: mipi_csi_driver_supported

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    main,
    mipi_csi::{Config, DataLanes, MipiCsi},
};
use esp_println::println;

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    esp_println::logger::init_logger_from_env();

    let config = Config::default()
        .with_num_data_lanes(DataLanes::_1)
        .with_lane_bit_rate_mbps(800);

    println!("Initializing MIPI CSI");
    let _mipi_csi = MipiCsi::new(peripherals.MIPI_CSI, config).unwrap();
    println!("MIPI CSI initialized successfully");
    println!("NOTE: without a connected camera, CSI link/packet reception is not validated.");

    loop {}
}
