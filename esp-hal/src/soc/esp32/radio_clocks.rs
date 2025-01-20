//! # Radio clocks driver (ESP32)
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

use crate::{
    peripherals::DPORT,
    system::{RadioClockController, RadioPeripherals},
};

const DPORT_WIFI_CLK_WIFI_BT_COMMON_M: u32 = 0x000003c9;
const DPORT_WIFI_CLK_WIFI_EN_M: u32 = 0x00000406;
const DPORT_WIFI_CLK_BT_EN_M: u32 = 0x00030800;

impl RadioClockController for crate::peripherals::RADIO_CLK {
    fn enable(&mut self, peripheral: RadioPeripherals) {
        match peripheral {
            RadioPeripherals::Phy => enable_phy(),
            RadioPeripherals::Bt => bt_clock_enable(),
            RadioPeripherals::Wifi => wifi_clock_enable(),
        }
    }

    fn disable(&mut self, peripheral: RadioPeripherals) {
        match peripheral {
            RadioPeripherals::Phy => disable_phy(),
            RadioPeripherals::Bt => bt_clock_disable(),
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
    DPORT::regs()
        .wifi_clk_en()
        .modify(|r, w| unsafe { w.bits(r.bits() | DPORT_WIFI_CLK_WIFI_BT_COMMON_M) });
}

fn disable_phy() {
    // `periph_ll_wifi_bt_module_disable_clk_set_rst`
    DPORT::regs()
        .wifi_clk_en()
        .modify(|r, w| unsafe { w.bits(r.bits() & !DPORT_WIFI_CLK_WIFI_BT_COMMON_M) });
}

fn bt_clock_enable() {
    DPORT::regs()
        .wifi_clk_en()
        .modify(|r, w| unsafe { w.bits(r.bits() | DPORT_WIFI_CLK_BT_EN_M) });
}

fn bt_clock_disable() {
    DPORT::regs()
        .wifi_clk_en()
        .modify(|r, w| unsafe { w.bits(r.bits() & !DPORT_WIFI_CLK_BT_EN_M) });
}

fn wifi_clock_enable() {
    // `periph_ll_wifi_module_enable_clk_clear_rst`
    DPORT::regs()
        .wifi_clk_en()
        .modify(|r, w| unsafe { w.bits(r.bits() | DPORT_WIFI_CLK_WIFI_EN_M) });
}

fn wifi_clock_disable() {
    // `periph_ll_wifi_module_disable_clk_set_rst`
    DPORT::regs()
        .wifi_clk_en()
        .modify(|r, w| unsafe { w.bits(r.bits() & !DPORT_WIFI_CLK_WIFI_EN_M) });
}

fn reset_mac() {
    const SYSTEM_MAC_RST: u8 = 1 << 2;
    DPORT::regs()
        .core_rst_en()
        .modify(|r, w| unsafe { w.core_rst().bits(r.core_rst().bits() | SYSTEM_MAC_RST) });
    DPORT::regs()
        .core_rst_en()
        .modify(|r, w| unsafe { w.core_rst().bits(r.core_rst().bits() & !SYSTEM_MAC_RST) });
}

fn init_clocks() {
    // esp-idf assumes all clocks are enabled by default, and disables the following
    // bits:
    //
    // ```
    // const DPORT_WIFI_CLK_SDIOSLAVE_EN: u32 = 1 << 4;
    // const DPORT_WIFI_CLK_UNUSED_BIT5: u32 = 1 << 5;
    // const DPORT_WIFI_CLK_UNUSED_BIT12: u32 = 1 << 12;
    // const DPORT_WIFI_CLK_SDIO_HOST_EN: u32 = 1 << 13;
    // const DPORT_WIFI_CLK_EMAC_EN: u32 = 1 << 14;
    //
    // const WIFI_BT_SDIO_CLK: u32 = DPORT_WIFI_CLK_WIFI_EN_M
    //     | DPORT_WIFI_CLK_BT_EN_M
    //     | DPORT_WIFI_CLK_UNUSED_BIT5
    //     | DPORT_WIFI_CLK_UNUSED_BIT12
    //     | DPORT_WIFI_CLK_SDIOSLAVE_EN
    //     | DPORT_WIFI_CLK_SDIO_HOST_EN
    //     | DPORT_WIFI_CLK_EMAC_EN;
    // ```
    //
    // However, we can't do this because somehow our initialization process is
    // different, and disabling some bits, or not enabling them makes the BT
    // stack crash.

    DPORT::regs()
        .wifi_clk_en()
        .write(|w| unsafe { w.bits(u32::MAX) });
}
