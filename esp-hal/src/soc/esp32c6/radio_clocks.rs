//! # Radio clocks driver (ESP32-C6)
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

impl RadioClockController for crate::peripherals::RADIO_CLK {
    fn enable(&mut self, peripheral: RadioPeripherals) {
        match peripheral {
            RadioPeripherals::Phy => enable_phy(),
            RadioPeripherals::Wifi => wifi_clock_enable(),
            RadioPeripherals::Bt => ble_clock_enable(),
            RadioPeripherals::Ieee802154 => ieee802154_clock_enable(),
        }
    }

    fn disable(&mut self, peripheral: RadioPeripherals) {
        match peripheral {
            RadioPeripherals::Phy => disable_phy(),
            RadioPeripherals::Wifi => wifi_clock_disable(),
            RadioPeripherals::Bt => ble_clock_disable(),
            RadioPeripherals::Ieee802154 => ieee802154_clock_disable(),
        }
    }

    fn reset_mac(&mut self) {
        reset_mac();
    }

    fn init_clocks(&mut self) {
        init_clocks();
    }

    fn ble_rtc_clk_init(&mut self) {
        // nothing for this target (yet)
    }

    fn reset_rpa(&mut self) {
        // nothing for this target (yet)
    }
}

fn enable_phy() {
    let modem_lpcon = unsafe { &*esp32c6::MODEM_LPCON::PTR };
    modem_lpcon
        .clk_conf()
        .modify(|_, w| w.clk_i2c_mst_en().set_bit());
    modem_lpcon
        .i2c_mst_clk_conf()
        .modify(|_, w| w.clk_i2c_mst_sel_160m().set_bit());
}

fn disable_phy() {
    let modem_lpcon = unsafe { &*esp32c6::MODEM_LPCON::PTR };
    modem_lpcon
        .clk_conf()
        .modify(|_, w| w.clk_i2c_mst_en().clear_bit());
    modem_lpcon
        .i2c_mst_clk_conf()
        .modify(|_, w| w.clk_i2c_mst_sel_160m().clear_bit());
}

fn wifi_clock_enable() {
    let modem_syscon = unsafe { &*esp32c6::MODEM_SYSCON::PTR };
    let modem_lpcon = unsafe { &*esp32c6::MODEM_LPCON::PTR };

    modem_syscon.clk_conf1().modify(|_, w| {
        w.clk_wifi_apb_en()
            .set_bit()
            .clk_wifimac_en()
            .set_bit()
            .clk_fe_apb_en()
            .set_bit()
            .clk_fe_cal_160m_en()
            .set_bit()
            .clk_fe_160m_en()
            .set_bit()
            .clk_fe_80m_en()
            .set_bit()
            .clk_wifibb_160x1_en()
            .set_bit()
            .clk_wifibb_80x1_en()
            .set_bit()
            .clk_wifibb_40x1_en()
            .set_bit()
            .clk_wifibb_80x_en()
            .set_bit()
            .clk_wifibb_40x_en()
            .set_bit()
            .clk_wifibb_80m_en()
            .set_bit()
            .clk_wifibb_44m_en()
            .set_bit()
            .clk_wifibb_40m_en()
            .set_bit()
            .clk_wifibb_22m_en()
            .set_bit()
    });

    modem_lpcon
        .clk_conf()
        .modify(|_, w| w.clk_wifipwr_en().set_bit().clk_coex_en().set_bit());
}

fn wifi_clock_disable() {
    let modem_syscon = unsafe { &*esp32c6::MODEM_SYSCON::PTR };
    let modem_lpcon = unsafe { &*esp32c6::MODEM_LPCON::PTR };

    modem_syscon.clk_conf1().modify(|_, w| {
        w.clk_wifi_apb_en()
            .clear_bit()
            .clk_wifimac_en()
            .clear_bit()
            .clk_fe_apb_en()
            .clear_bit()
            .clk_fe_cal_160m_en()
            .clear_bit()
            .clk_fe_160m_en()
            .clear_bit()
            .clk_fe_80m_en()
            .clear_bit()
            .clk_wifibb_160x1_en()
            .clear_bit()
            .clk_wifibb_80x1_en()
            .clear_bit()
            .clk_wifibb_40x1_en()
            .clear_bit()
            .clk_wifibb_80x_en()
            .clear_bit()
            .clk_wifibb_40x_en()
            .clear_bit()
            .clk_wifibb_80m_en()
            .clear_bit()
            .clk_wifibb_44m_en()
            .clear_bit()
            .clk_wifibb_40m_en()
            .clear_bit()
            .clk_wifibb_22m_en()
            .clear_bit()
    });

    modem_lpcon
        .clk_conf()
        .modify(|_, w| w.clk_wifipwr_en().clear_bit().clk_coex_en().clear_bit());
}

fn ieee802154_clock_enable() {
    let modem_syscon = unsafe { &*esp32c6::MODEM_SYSCON::PTR };
    let modem_lpcon = unsafe { &*esp32c6::MODEM_LPCON::PTR };

    modem_syscon
        .clk_conf()
        .modify(|_, w| w.clk_zb_apb_en().set_bit().clk_zb_mac_en().set_bit());

    modem_syscon.clk_conf1().modify(|_, w| {
        w.clk_fe_apb_en()
            .set_bit()
            .clk_fe_cal_160m_en()
            .set_bit()
            .clk_fe_160m_en()
            .set_bit()
            .clk_fe_80m_en()
            .set_bit()
            .clk_bt_apb_en()
            .set_bit()
            .clk_bt_en()
            .set_bit()
            .clk_wifibb_160x1_en()
            .set_bit()
            .clk_wifibb_80x1_en()
            .set_bit()
            .clk_wifibb_40x1_en()
            .set_bit()
            .clk_wifibb_80x_en()
            .set_bit()
            .clk_wifibb_40x_en()
            .set_bit()
            .clk_wifibb_80m_en()
            .set_bit()
            .clk_wifibb_44m_en()
            .set_bit()
            .clk_wifibb_40m_en()
            .set_bit()
            .clk_wifibb_22m_en()
            .set_bit()
    });

    modem_lpcon
        .clk_conf()
        .modify(|_, w| w.clk_coex_en().set_bit());
}

fn ieee802154_clock_disable() {
    let modem_syscon = unsafe { &*esp32c6::MODEM_SYSCON::PTR };
    let modem_lpcon = unsafe { &*esp32c6::MODEM_LPCON::PTR };

    modem_syscon
        .clk_conf()
        .modify(|_, w| w.clk_zb_apb_en().clear_bit().clk_zb_mac_en().clear_bit());

    modem_syscon.clk_conf1().modify(|_, w| {
        w.clk_fe_apb_en()
            .clear_bit()
            .clk_fe_cal_160m_en()
            .clear_bit()
            .clk_fe_160m_en()
            .clear_bit()
            .clk_fe_80m_en()
            .clear_bit()
            .clk_bt_apb_en()
            .clear_bit()
            .clk_bt_en()
            .clear_bit()
            .clk_wifibb_160x1_en()
            .clear_bit()
            .clk_wifibb_80x1_en()
            .clear_bit()
            .clk_wifibb_40x1_en()
            .clear_bit()
            .clk_wifibb_80x_en()
            .clear_bit()
            .clk_wifibb_40x_en()
            .clear_bit()
            .clk_wifibb_80m_en()
            .clear_bit()
            .clk_wifibb_44m_en()
            .clear_bit()
            .clk_wifibb_40m_en()
            .clear_bit()
            .clk_wifibb_22m_en()
            .clear_bit()
    });

    modem_lpcon
        .clk_conf()
        .modify(|_, w| w.clk_coex_en().clear_bit());
}

fn ble_clock_enable() {
    let modem_syscon = unsafe { &*esp32c6::MODEM_SYSCON::PTR };
    let modem_lpcon = unsafe { &*esp32c6::MODEM_LPCON::PTR };

    modem_syscon.clk_conf().modify(|_, w| {
        w.clk_etm_en()
            .set_bit()
            .clk_modem_sec_en()
            .set_bit()
            .clk_modem_sec_ecb_en()
            .set_bit()
            .clk_modem_sec_ccm_en()
            .set_bit()
            .clk_modem_sec_bah_en()
            .set_bit()
            .clk_modem_sec_apb_en()
            .set_bit()
            .clk_ble_timer_en()
            .set_bit()
    });

    modem_syscon.clk_conf1().modify(|_, w| {
        w.clk_fe_apb_en()
            .set_bit()
            .clk_fe_cal_160m_en()
            .set_bit()
            .clk_fe_160m_en()
            .set_bit()
            .clk_fe_80m_en()
            .set_bit()
            .clk_bt_apb_en()
            .set_bit()
            .clk_bt_en()
            .set_bit()
    });

    modem_lpcon
        .clk_conf()
        .modify(|_, w| w.clk_coex_en().set_bit());
}

fn ble_clock_disable() {
    let modem_syscon = unsafe { &*esp32c6::MODEM_SYSCON::PTR };
    let modem_lpcon = unsafe { &*esp32c6::MODEM_LPCON::PTR };

    modem_syscon.clk_conf().modify(|_, w| {
        w.clk_etm_en()
            .clear_bit()
            .clk_modem_sec_en()
            .clear_bit()
            .clk_modem_sec_ecb_en()
            .clear_bit()
            .clk_modem_sec_ccm_en()
            .clear_bit()
            .clk_modem_sec_bah_en()
            .clear_bit()
            .clk_modem_sec_apb_en()
            .clear_bit()
            .clk_ble_timer_en()
            .clear_bit()
    });

    modem_syscon.clk_conf1().modify(|_, w| {
        w.clk_fe_apb_en()
            .clear_bit()
            .clk_fe_cal_160m_en()
            .clear_bit()
            .clk_fe_160m_en()
            .clear_bit()
            .clk_fe_80m_en()
            .clear_bit()
            .clk_bt_apb_en()
            .clear_bit()
            .clk_bt_en()
            .clear_bit()
    });

    modem_lpcon
        .clk_conf()
        .modify(|_, w| w.clk_coex_en().set_bit());
}

fn reset_mac() {
    // empty
}

fn init_clocks() {
    unsafe {
        let pmu = &*esp32c6::PMU::PTR;

        pmu.hp_sleep_icg_modem()
            .modify(|_, w| w.hp_sleep_dig_icg_modem_code().bits(0));
        pmu.hp_modem_icg_modem()
            .modify(|_, w| w.hp_modem_dig_icg_modem_code().bits(1));
        pmu.hp_active_icg_modem()
            .modify(|_, w| w.hp_active_dig_icg_modem_code().bits(2));
        pmu.imm_modem_icg()
            .write(|w| w.update_dig_icg_modem_en().set_bit());
        pmu.imm_sleep_sysclk()
            .write(|w| w.update_dig_icg_switch().set_bit());

        let modem_syscon = &*esp32c6::MODEM_SYSCON::PTR;
        modem_syscon.clk_conf_power_st().modify(|_, w| {
            w.clk_modem_apb_st_map()
                .bits(6)
                .clk_modem_peri_st_map()
                .bits(4)
                .clk_wifi_st_map()
                .bits(6)
                .clk_bt_st_map()
                .bits(6)
                .clk_fe_st_map()
                .bits(6)
                .clk_zb_st_map()
                .bits(6)
        });

        let modem_lpcon = &*esp32c6::MODEM_LPCON::PTR;
        modem_lpcon.clk_conf_power_st().modify(|_, w| {
            w.clk_lp_apb_st_map()
                .bits(6)
                .clk_i2c_mst_st_map()
                .bits(6)
                .clk_coex_st_map()
                .bits(6)
                .clk_wifipwr_st_map()
                .bits(6)
        });

        modem_lpcon.wifi_lp_clk_conf().modify(|_, w| {
            w.clk_wifipwr_lp_sel_osc_slow()
                .set_bit()
                .clk_wifipwr_lp_sel_osc_fast()
                .set_bit()
                .clk_wifipwr_lp_sel_xtal32k()
                .set_bit()
                .clk_wifipwr_lp_sel_xtal()
                .set_bit()
        });

        modem_lpcon
            .wifi_lp_clk_conf()
            .modify(|_, w| w.clk_wifipwr_lp_div_num().bits(0));

        modem_lpcon
            .clk_conf()
            .modify(|_, w| w.clk_wifipwr_en().set_bit());
    }
}
