//! # Radio clocks driver (ESP32-S2)
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

use crate::radio_clock_ctrl::{RadioClockController, RadioPeripherals};

// Mask for clock bits used by both WIFI and Bluetooth, bit 0, 3, 6, 7, 8, 9
const DPORT_WIFI_CLK_WIFI_BT_COMMON_M: u32 = 0x000003c9;
const DPORT_WIFI_CLK_WIFI_EN_M: u32 = 0x000007cf;

impl RadioClockController for crate::peripherals::RADIO_CLK {
    fn enable(&mut self, peripheral: RadioPeripherals) {
        match peripheral {
            RadioPeripherals::Phy => enable_phy(),
            RadioPeripherals::Wifi => wifi_clock_enable(),
        }
    }

    fn disable(&mut self, peripheral: RadioPeripherals) {
        match peripheral {
            RadioPeripherals::Phy => disable_phy(),
            RadioPeripherals::Wifi => wifi_clock_disable(),
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
    let dport = unsafe { &*esp32s2::SYSCON::PTR };
    dport
        .wifi_clk_en()
        .modify(|r, w| unsafe { w.bits(r.bits() | DPORT_WIFI_CLK_WIFI_BT_COMMON_M) });
}

fn disable_phy() {
    // `periph_ll_wifi_bt_module_disable_clk_set_rst`
    let syscon = unsafe { &*esp32s2::SYSCON::PTR };
    syscon
        .wifi_clk_en()
        .modify(|r, w| unsafe { w.bits(r.bits() & !DPORT_WIFI_CLK_WIFI_BT_COMMON_M) });
}

fn wifi_clock_enable() {
    // `periph_ll_wifi_module_enable_clk_clear_rst`
    let syscon = unsafe { &*esp32s2::SYSCON::PTR };
    syscon
        .wifi_clk_en()
        .modify(|r, w| unsafe { w.bits(r.bits() | DPORT_WIFI_CLK_WIFI_EN_M) });
}

fn wifi_clock_disable() {
    // `periph_ll_wifi_module_disable_clk_set_rst`
    let syscon = unsafe { &*esp32s2::SYSCON::PTR };
    syscon
        .wifi_clk_en()
        .modify(|r, w| unsafe { w.bits(r.bits() & !DPORT_WIFI_CLK_WIFI_EN_M) });
}

fn reset_mac() {
    const SYSTEM_MAC_RST: u32 = 1 << 2;
    let syscon = unsafe { &*esp32s2::SYSCON::PTR };
    syscon
        .wifi_rst_en()
        .modify(|r, w| unsafe { w.wifi_rst().bits(r.wifi_rst().bits() | SYSTEM_MAC_RST) });
    syscon
        .wifi_rst_en()
        .modify(|r, w| unsafe { w.wifi_rst().bits(r.wifi_rst().bits() & !SYSTEM_MAC_RST) });
}

fn init_clocks() {
    let syscon = unsafe { &*esp32s2::SYSCON::PTR };

    const DPORT_WIFI_CLK_WIFI_EN: u32 = 0x003807cf;
    const DPORT_WIFI_CLK_BT_EN_M: u32 = 0x61 << 11;
    const DPORT_WIFI_CLK_SDIOSLAVE_EN: u32 = 1 << 4;
    const DPORT_WIFI_CLK_UNUSED_BIT5: u32 = 1 << 5;
    const DPORT_WIFI_CLK_UNUSED_BIT12: u32 = 1 << 12;
    const DPORT_WIFI_CLK_SDIO_HOST_EN: u32 = 1 << 13;
    const DPORT_WIFI_CLK_EMAC_EN: u32 = 1 << 14;

    const WIFI_BT_SDIO_CLK: u32 = DPORT_WIFI_CLK_WIFI_EN
        | DPORT_WIFI_CLK_BT_EN_M
        | DPORT_WIFI_CLK_UNUSED_BIT5
        | DPORT_WIFI_CLK_UNUSED_BIT12
        | DPORT_WIFI_CLK_SDIOSLAVE_EN
        | DPORT_WIFI_CLK_SDIO_HOST_EN
        | DPORT_WIFI_CLK_EMAC_EN;

    syscon
        .wifi_clk_en()
        .modify(|r, w| unsafe { w.bits(r.bits() & !WIFI_BT_SDIO_CLK | DPORT_WIFI_CLK_WIFI_EN) });
}
