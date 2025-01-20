//! # Radio clocks driver (ESP32-C2)
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
    peripherals::{APB_CTRL, MODEM_CLKRST},
    system::{RadioClockController, RadioPeripherals},
};

// Mask for clock bits used by both WIFI and Bluetooth, 0, 1, 2, 3, 7, 8, 9, 10,
// 19, 20, 21, 22, 23
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
        ble_rtc_clk_init();
    }

    fn reset_rpa(&mut self) {
        reset_rpa();
    }
}

fn enable_phy() {
    // `periph_ll_wifi_bt_module_enable_clk_clear_rst`
    APB_CTRL::regs()
        .wifi_clk_en()
        .modify(|r, w| unsafe { w.bits(r.bits() | SYSTEM_WIFI_CLK_WIFI_BT_COMMON_M) });
}

fn disable_phy() {
    // `periph_ll_wifi_bt_module_disable_clk_set_rst`
    APB_CTRL::regs()
        .wifi_clk_en()
        .modify(|r, w| unsafe { w.bits(r.bits() & !SYSTEM_WIFI_CLK_WIFI_BT_COMMON_M) });
}

fn common_wifi_bt_clock_enable() {
    // `periph_ll_wifi_module_enable_clk_clear_rst`, no-op
}

fn common_wifi_bt_clock_disable() {
    // `periph_ll_wifi_module_disable_clk_clear_rst`, no-op
}

fn reset_mac() {
    const SYSTEM_MAC_RST: u32 = 1 << 2;
    APB_CTRL::regs()
        .wifi_rst_en()
        .modify(|r, w| unsafe { w.wifi_rst().bits(r.wifi_rst().bits() | SYSTEM_MAC_RST) });
    APB_CTRL::regs()
        .wifi_rst_en()
        .modify(|r, w| unsafe { w.wifi_rst().bits(r.wifi_rst().bits() & !SYSTEM_MAC_RST) });
}

fn init_clocks() {
    // from `esp_perip_clk_init`
    const SYSTEM_WIFI_CLK_UNUSED_BIT5: u32 = 1 << 5;
    const SYSTEM_WIFI_CLK_UNUSED_BIT12: u32 = 1 << 12;
    const WIFI_BT_SDIO_CLK: u32 = SYSTEM_WIFI_CLK_UNUSED_BIT5 | SYSTEM_WIFI_CLK_UNUSED_BIT12;

    APB_CTRL::regs()
        .wifi_clk_en()
        .modify(|r, w| unsafe { w.bits(r.bits() & !WIFI_BT_SDIO_CLK | SYSTEM_WIFI_CLK_EN) });
}

fn ble_rtc_clk_init() {
    let modem_clkrst = MODEM_CLKRST::regs();
    modem_clkrst
        .modem_lp_timer_conf()
        .modify(|_, w| w.lp_timer_sel_xtal32k().clear_bit());
    modem_clkrst
        .modem_lp_timer_conf()
        .modify(|_, w| w.lp_timer_sel_xtal().set_bit());
    modem_clkrst
        .modem_lp_timer_conf()
        .modify(|_, w| w.lp_timer_sel_8m().clear_bit());
    modem_clkrst
        .modem_lp_timer_conf()
        .modify(|_, w| w.lp_timer_sel_rtc_slow().clear_bit());

    // assume 40MHz xtal
    modem_clkrst
        .modem_lp_timer_conf()
        .modify(|_, w| unsafe { w.lp_timer_clk_div_num().bits(249) });

    modem_clkrst
        .etm_clk_conf()
        .modify(|_, w| w.etm_clk_active().set_bit());
    modem_clkrst
        .etm_clk_conf()
        .modify(|_, w| w.etm_clk_sel().clear_bit());
}

fn reset_rpa() {
    const BLE_RPA_REST_BIT: u32 = 1 << 27;
    APB_CTRL::regs()
        .wifi_rst_en()
        .modify(|r, w| unsafe { w.bits(r.bits() | BLE_RPA_REST_BIT) });
    APB_CTRL::regs()
        .wifi_rst_en()
        .modify(|r, w| unsafe { w.bits(r.bits() & !BLE_RPA_REST_BIT) });
}
