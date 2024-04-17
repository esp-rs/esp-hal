//! # Radio clocks driver (ESP32-H2)
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

use crate::system::{RadioClockControl, RadioClockController, RadioPeripherals};

impl RadioClockController for RadioClockControl {
    fn enable(&mut self, peripheral: RadioPeripherals) {
        match peripheral {
            RadioPeripherals::Phy => enable_phy(),
            RadioPeripherals::Bt | RadioPeripherals::Ieee802154 => ble_ieee802154_clock_enable(),
        }
    }

    fn disable(&mut self, peripheral: RadioPeripherals) {
        match peripheral {
            RadioPeripherals::Phy => disable_phy(),
            RadioPeripherals::Bt | RadioPeripherals::Ieee802154 => ble_ieee802154_clock_disable(),
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
    unsafe { &*esp32h2::MODEM_LPCON::PTR }
        .clk_conf()
        .modify(|_, w| w.clk_i2c_mst_en().set_bit());
}

fn disable_phy() {
    unsafe { &*esp32h2::MODEM_LPCON::PTR }
        .clk_conf()
        .modify(|_, w| w.clk_i2c_mst_en().clear_bit());
}

fn ble_ieee802154_clock_enable() {
    let modem_syscon = unsafe { &*esp32h2::MODEM_SYSCON::PTR };

    modem_syscon
        .clk_conf()
        .modify(|_, w| w.clk_zb_apb_en().set_bit().clk_zb_mac_en().set_bit());

    modem_syscon.clk_conf1().modify(|_, w| {
        w.clk_bt_apb_en()
            .set_bit()
            .clk_bt_en()
            .set_bit()
            .clk_fe_16m_en()
            .set_bit()
            .clk_fe_32m_en()
            .set_bit()
            .clk_fe_adc_en()
            .set_bit()
            .clk_fe_apb_en()
            .set_bit()
            .clk_fe_sdm_en()
            .set_bit()
    });

    unsafe { &*esp32h2::MODEM_LPCON::PTR }
        .clk_conf()
        .modify(|_, w| w.clk_coex_en().set_bit());
}

fn ble_ieee802154_clock_disable() {
    let modem_lpcon = unsafe { &*esp32h2::MODEM_LPCON::PTR };
    let modem_syscon = unsafe { &*esp32h2::MODEM_SYSCON::PTR };

    modem_syscon
        .clk_conf()
        .modify(|_, w| w.clk_zb_apb_en().clear_bit().clk_zb_mac_en().clear_bit());

    modem_syscon.clk_conf1().modify(|_, w| {
        w.clk_bt_apb_en()
            .clear_bit()
            .clk_bt_en()
            .clear_bit()
            .clk_fe_16m_en()
            .clear_bit()
            .clk_fe_32m_en()
            .clear_bit()
            .clk_fe_adc_en()
            .clear_bit()
            .clk_fe_apb_en()
            .clear_bit()
            .clk_fe_sdm_en()
            .clear_bit()
    });

    modem_lpcon
        .clk_conf()
        .modify(|_, w| w.clk_coex_en().clear_bit());
}

fn reset_mac() {
    // empty
}

fn init_clocks() {
    unsafe {
        let pmu = &*esp32h2::PMU::PTR;

        pmu.hp_sleep_icg_modem()
            .modify(|_, w| w.hp_sleep_dig_icg_modem_code().bits(0));
        pmu.hp_modem_icg_modem()
            .modify(|_, w| w.hp_modem_dig_icg_modem_code().bits(1));
        pmu.hp_active_icg_modem()
            .modify(|_, w| w.hp_active_dig_icg_modem_code().bits(2));
        pmu.imm_modem_icg()
            .as_ptr()
            .write_volatile(pmu.imm_modem_icg().as_ptr().read_volatile() | 1 << 31);
        pmu.imm_sleep_sysclk()
            .as_ptr()
            .write_volatile(pmu.imm_sleep_sysclk().as_ptr().read_volatile() | 1 << 28);

        (*esp32h2::MODEM_LPCON::PTR).clk_conf().modify(|_, w| {
            w.clk_i2c_mst_en()
                .set_bit()
                .clk_coex_en()
                .set_bit()
                .clk_fe_mem_en()
                .set_bit()
        });
    }
}
