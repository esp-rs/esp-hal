//! No-peripheral DW-GDMA smoke test for ESP32-P4.
//!
//! Validates that configuring one VDMA channel does not disturb another
//! channel, without starting any DMA transfer.
//% CHIP_FILTER: mipi_dsi_driver_supported

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    Config,
    main,
    mipi_dsi::{__qa_dw_gdma_no_peripheral_smoke, DwGdmaChannelConfigReadback},
};
use esp_println::println;

esp_bootloader_esp_idf::esp_app_desc!();

fn print_check(label: &str, passed: bool) {
    println!("[{}] {}", if passed { "PASS" } else { "FAIL" }, label);
}

fn print_channel_config(label: &str, cfg: DwGdmaChannelConfigReadback) {
    println!(
        "{} cfg0=0x{:08x} cfg1=0x{:08x} src_mb={} dst_mb={} tt_fc={} src_sw_hs={} dst_sw_hs={} src_per={} dst_per={} prio={} src_osr={} dst_osr={}",
        label,
        cfg.cfg0_bits,
        cfg.cfg1_bits,
        cfg.src_multblk_type,
        cfg.dst_multblk_type,
        cfg.transfer_type_flow_control,
        cfg.src_uses_software_handshake,
        cfg.dst_uses_software_handshake,
        cfg.src_peripheral,
        cfg.dst_peripheral,
        cfg.priority,
        cfg.src_outstanding_limit,
        cfg.dst_outstanding_limit
    );
}

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(Config::default());
    esp_println::logger::init_logger_from_env();

    println!("Running DW-GDMA no-transfer smoke test");
    let report = __qa_dw_gdma_no_peripheral_smoke(peripherals.VDMA_CH0, peripherals.VDMA_CH1);

    println!("--- Controller ---");
    println!(
        "CFG0: 0x{:08x}, dmac_en={}, int_en={}",
        report.controller_cfg0_bits, report.controller_enabled, report.controller_interrupt_enabled
    );

    println!("--- Channel Readback ---");
    print_channel_config("ch0 after first config", report.channel0_after_first_config);
    print_channel_config(
        "ch0 after ch1 config",
        report.channel0_after_channel1_config,
    );
    print_channel_config("ch1 after config", report.channel1_after_config);

    println!("--- Checks ---");
    print_check("channel IDs are ch0/ch1", report.expected_channel_ids);
    print_check("channels are distinct", report.unique_channels);
    print_check("controller enabled", report.controller_enabled);
    print_check(
        "controller interrupts enabled",
        report.controller_interrupt_enabled,
    );
    print_check(
        "channel 0 matches expected config",
        report.channel0_matches_expected,
    );
    print_check(
        "channel 1 matches expected config",
        report.channel1_matches_expected,
    );
    print_check(
        "channel 0 unchanged after channel 1 configure",
        report.channel0_unchanged_after_channel1_config,
    );
    print_check(
        "channel 0 disabled before cleanup",
        !report.channel0_enabled_before_cleanup,
    );
    print_check(
        "channel 1 disabled before cleanup",
        !report.channel1_enabled_before_cleanup,
    );
    print_check(
        "channel enable register clear after cleanup",
        (report.channel_enable_bits_after_cleanup & 0x3) == 0,
    );
    println!(
        "CHEN0 before cleanup: 0x{:08x}",
        report.channel_enable_bits_before_cleanup
    );
    println!(
        "CHEN0 after cleanup : 0x{:08x}",
        report.channel_enable_bits_after_cleanup
    );

    let passed = report.passed();
    println!(
        "DW-GDMA no-transfer smoke RESULT: {}",
        if passed { "PASS" } else { "FAIL" }
    );

    if !passed {
        panic!("DW-GDMA no-transfer smoke failed");
    }

    loop {}
}
