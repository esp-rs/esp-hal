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

use crate::{
    peripherals::{MODEM_LPCON, MODEM_SYSCON, PMU},
    system::{RadioClockController, RadioPeripherals},
};

impl RadioClockController for crate::peripherals::RADIO_CLK {
    fn enable(&mut self, peripheral: RadioPeripherals) {
        match peripheral {
            RadioPeripherals::Phy => enable_phy(true),
            RadioPeripherals::Bt | RadioPeripherals::Ieee802154 => {
                ble_ieee802154_clock_enable(true)
            }
        }
    }

    fn disable(&mut self, peripheral: RadioPeripherals) {
        match peripheral {
            RadioPeripherals::Phy => enable_phy(false),
            RadioPeripherals::Bt | RadioPeripherals::Ieee802154 => {
                ble_ieee802154_clock_enable(false)
            }
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

fn enable_phy(en: bool) {
    MODEM_LPCON::regs()
        .clk_conf()
        .modify(|_, w| w.clk_i2c_mst_en().bit(en));
}

fn ble_ieee802154_clock_enable(en: bool) {
    MODEM_SYSCON::regs().clk_conf().modify(|_, w| {
        w.clk_zb_apb_en().bit(en);
        w.clk_zb_mac_en().bit(en)
    });

    MODEM_SYSCON::regs().clk_conf1().modify(|_, w| {
        w.clk_bt_apb_en().bit(en);
        w.clk_bt_en().bit(en);
        w.clk_fe_16m_en().bit(en);
        w.clk_fe_32m_en().bit(en);
        w.clk_fe_adc_en().bit(en);
        w.clk_fe_apb_en().bit(en);
        w.clk_fe_sdm_en().bit(en)
    });

    MODEM_LPCON::regs()
        .clk_conf()
        .modify(|_, w| w.clk_coex_en().bit(en));
}

fn reset_mac() {
    // empty
}

fn init_clocks() {
    unsafe {
        let pmu = PMU::regs();

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

        MODEM_LPCON::regs().clk_conf().modify(|_, w| {
            w.clk_i2c_mst_en()
                .set_bit()
                .clk_coex_en()
                .set_bit()
                .clk_fe_mem_en()
                .set_bit()
        });
    }
}
