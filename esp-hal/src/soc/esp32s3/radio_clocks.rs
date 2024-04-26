//! # Radio clocks driver (ESP32-S3)
//!
//! ## Overview
//!
//! The `Radio Clocks` module provides control and configuration functions for
//! various radio peripherals, such as `PHY`, `Bluetooth (BT)`, and `Wi-Fi`. The
//! module allows enabling and disabling these peripherals, resetting the `Media
//! Access Control (MAC)` and initializing clocks.
//!
//! The module defines a `RadioClockController` trait implemented by the
//! `RadioClockControl` struct. This trait provides methods to enable, disable,
//! reset the MAC, initialize clocks and perform other related operations.

use crate::system::{RadioClockController, RadioPeripherals};

// Note: this comment has been copied from esp-idf, including the mistake.
// Mask for clock bits used by both WIFI and Bluetooth, 0, 1, 2, 3, 7,
// 8, 9, 10, 19, 20, 21, 22, 23
const SYSTEM_WIFI_CLK_WIFI_BT_COMMON_M: u32 = 0x78078F;

// SYSTEM_WIFI_CLK_EN : R/W ;bitpos:[31:0] ;default: 32'hfffce030
const SYSTEM_WIFI_CLK_EN: u32 = 0x00FB9FCF;

impl RadioClockController for crate::peripherals::RADIO_CLK {
    fn enable(&mut self, peripheral: RadioPeripherals) {
        match peripheral {
            RadioPeripherals::Phy => enable_phy(),
            RadioPeripherals::Bt => common_wifi_bt_clock_enable(),
            RadioPeripherals::Wifi => common_wifi_bt_clock_enable(),
        }
    }

    fn disable(&mut self, peripheral: RadioPeripherals) {
        match peripheral {
            RadioPeripherals::Phy => disable_phy(),
            RadioPeripherals::Bt => common_wifi_bt_clock_disable(),
            RadioPeripherals::Wifi => common_wifi_bt_clock_disable(),
        }
    }

    fn reset_mac(&mut self) {
        reset_mac();
    }

    fn init_clocks(&mut self) {
        init_clocks();
    }

    fn ble_rtc_clk_init(&mut self) {
        // nothing for this target
    }

    fn reset_rpa(&mut self) {
        // nothing for this target
    }
}

fn enable_phy() {
    // `periph_ll_wifi_bt_module_enable_clk_clear_rst`
    let syscon = unsafe { &*esp32s3::APB_CTRL::PTR };
    syscon
        .wifi_clk_en()
        .modify(|r, w| unsafe { w.bits(r.bits() | SYSTEM_WIFI_CLK_WIFI_BT_COMMON_M) });
}

fn disable_phy() {
    // `periph_ll_wifi_bt_module_disable_clk_set_rst`
    let syscon = unsafe { &*esp32s3::APB_CTRL::PTR };
    syscon
        .wifi_clk_en()
        .modify(|r, w| unsafe { w.bits(r.bits() & !SYSTEM_WIFI_CLK_WIFI_BT_COMMON_M) });
}

fn common_wifi_bt_clock_enable() {
    // `periph_ll_wifi_module_enable_clk_clear_rst`. does nothing
}

fn common_wifi_bt_clock_disable() {
    // `periph_ll_wifi_module_disable_clk_set_rst`. does nothing
}

fn reset_mac() {
    const SYSTEM_MAC_RST: u32 = 1 << 2;
    let syscon = unsafe { &*esp32s3::APB_CTRL::PTR };
    syscon
        .wifi_rst_en()
        .modify(|r, w| unsafe { w.bits(r.bits() | SYSTEM_MAC_RST) });
    syscon
        .wifi_rst_en()
        .modify(|r, w| unsafe { w.bits(r.bits() & !SYSTEM_MAC_RST) });
}

fn init_clocks() {
    let syscon = unsafe { &*esp32s3::APB_CTRL::PTR };

    const SYSTEM_WIFI_CLK_I2C_CLK_EN: u32 = 1 << 5;
    const SYSTEM_WIFI_CLK_UNUSED_BIT12: u32 = 1 << 12;
    const SYSTEM_WIFI_CLK_SDIO_HOST_EN: u32 = 1 << 13;

    const WIFI_BT_SDIO_CLK: u32 =
        SYSTEM_WIFI_CLK_I2C_CLK_EN | SYSTEM_WIFI_CLK_UNUSED_BIT12 | SYSTEM_WIFI_CLK_SDIO_HOST_EN;

    syscon
        .wifi_clk_en()
        .modify(|r, w| unsafe { w.bits(r.bits() & !WIFI_BT_SDIO_CLK | SYSTEM_WIFI_CLK_EN) });
}
