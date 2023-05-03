use crate::system::{RadioClockControl, RadioClockController, RadioPeripherals};

impl RadioClockController for RadioClockControl {
    fn enable(&mut self, peripheral: RadioPeripherals) {
        match peripheral {
            RadioPeripherals::Phy => enable_phy(),
            RadioPeripherals::Wifi => wifi_clock_enable(),
            RadioPeripherals::Bt => todo!("BLE not yet supported"),
            RadioPeripherals::Ieee802154 => ieee802154_clock_enable(),
        }
    }

    fn disable(&mut self, peripheral: RadioPeripherals) {
        match peripheral {
            RadioPeripherals::Phy => disable_phy(),
            RadioPeripherals::Wifi => wifi_clock_disable(),
            RadioPeripherals::Bt => todo!("BLE not yet supported"),
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
        .clk_conf
        .modify(|_, w| w.clk_i2c_mst_en().set_bit());
    modem_lpcon
        .i2c_mst_clk_conf
        .modify(|_, w| w.clk_i2c_mst_sel_160m().set_bit());
}

fn disable_phy() {
    let modem_lpcon = unsafe { &*esp32c6::MODEM_LPCON::PTR };
    modem_lpcon
        .clk_conf
        .modify(|_, w| w.clk_i2c_mst_en().clear_bit());
    modem_lpcon
        .i2c_mst_clk_conf
        .modify(|_, w| w.clk_i2c_mst_sel_160m().clear_bit());
}

fn wifi_clock_enable() {
    let modem_syscon = unsafe { &*esp32c6::MODEM_SYSCON::PTR };
    let modem_lpcon = unsafe { &*esp32c6::MODEM_LPCON::PTR };

    modem_syscon.clk_conf1.modify(|_, w| {
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
        .clk_conf
        .modify(|_, w| w.clk_wifipwr_en().set_bit().clk_coex_en().set_bit());
}

fn wifi_clock_disable() {
    let modem_syscon = unsafe { &*esp32c6::MODEM_SYSCON::PTR };
    let modem_lpcon = unsafe { &*esp32c6::MODEM_LPCON::PTR };

    modem_syscon.clk_conf1.modify(|_, w| {
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
        .clk_conf
        .modify(|_, w| w.clk_wifipwr_en().clear_bit().clk_coex_en().clear_bit());
}

fn ieee802154_clock_enable() {
    let modem_syscon = unsafe { &*esp32c6::MODEM_SYSCON::PTR };
    let modem_lpcon = unsafe { &*esp32c6::MODEM_LPCON::PTR };

    modem_syscon
        .clk_conf
        .modify(|_, w| w.clk_zb_apb_en().set_bit().clk_zb_mac_en().set_bit());

    modem_syscon.clk_conf1.modify(|_, w| {
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
        .clk_conf
        .modify(|_, w| w.clk_coex_en().set_bit());
}

fn ieee802154_clock_disable() {
    let modem_syscon = unsafe { &*esp32c6::MODEM_SYSCON::PTR };
    let modem_lpcon = unsafe { &*esp32c6::MODEM_LPCON::PTR };

    modem_syscon
        .clk_conf
        .modify(|_, w| w.clk_zb_apb_en().clear_bit().clk_zb_mac_en().clear_bit());

    modem_syscon.clk_conf1.modify(|_, w| {
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
        .clk_conf
        .modify(|_, w| w.clk_coex_en().clear_bit());
}

fn reset_mac() {
    // empty
}

fn init_clocks() {
    unsafe {
        let pmu = &*esp32c6::PMU::PTR;

        pmu.hp_sleep_icg_modem
            .modify(|_, w| w.hp_sleep_dig_icg_modem_code().variant(0));
        pmu.hp_modem_icg_modem
            .modify(|_, w| w.hp_modem_dig_icg_modem_code().variant(1));
        pmu.hp_active_icg_modem
            .modify(|_, w| w.hp_active_dig_icg_modem_code().variant(2));
        pmu.imm_modem_icg
            .as_ptr()
            .write_volatile(pmu.imm_modem_icg.as_ptr().read_volatile() | 1 << 31);
        pmu.imm_sleep_sysclk
            .as_ptr()
            .write_volatile(pmu.imm_sleep_sysclk.as_ptr().read_volatile() | 1 << 28);

        let modem_syscon = &*esp32c6::MODEM_SYSCON::PTR;
        modem_syscon.clk_conf_power_st.modify(|_, w| {
            w.clk_modem_apb_st_map()
                .variant(6)
                .clk_modem_peri_st_map()
                .variant(4)
                .clk_wifi_st_map()
                .variant(6)
                .clk_bt_st_map()
                .variant(6)
                .clk_fe_st_map()
                .variant(6)
                .clk_zb_st_map()
                .variant(6)
        });

        let modem_lpcon = &*esp32c6::MODEM_LPCON::PTR;
        modem_lpcon.clk_conf_power_st.modify(|_, w| {
            w.clk_lp_apb_st_map()
                .variant(6)
                .clk_i2c_mst_st_map()
                .variant(6)
                .clk_coex_st_map()
                .variant(6)
                .clk_wifipwr_st_map()
                .variant(6)
        });

        modem_lpcon.wifi_lp_clk_conf.modify(|_, w| {
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
            .wifi_lp_clk_conf
            .modify(|_, w| w.clk_wifipwr_lp_div_num().variant(0));

        modem_lpcon
            .clk_conf
            .modify(|_, w| w.clk_wifipwr_en().set_bit());
    }
}
