pub(crate) fn enable_phy(en: bool) {
    regs!(MODEM_LPCON)
        .clk_conf()
        .modify(|_, w| w.clk_i2c_mst_en().bit(en));
    regs!(MODEM_LPCON)
        .i2c_mst_clk_conf()
        .modify(|_, w| w.clk_i2c_mst_sel_160m().bit(en));
}

#[cfg_attr(not(feature = "unstable"), expect(unused))]
pub(crate) fn enable_wifi(en: bool) {
    regs!(MODEM_SYSCON).clk_conf1().modify(|_, w| {
        w.clk_wifi_apb_en().bit(en);
        w.clk_wifimac_en().bit(en);
        w.clk_fe_apb_en().bit(en);
        w.clk_fe_cal_160m_en().bit(en);
        w.clk_fe_160m_en().bit(en);
        w.clk_fe_80m_en().bit(en);
        w.clk_wifibb_160x1_en().bit(en);
        w.clk_wifibb_80x1_en().bit(en);
        w.clk_wifibb_40x1_en().bit(en);
        w.clk_wifibb_80x_en().bit(en);
        w.clk_wifibb_40x_en().bit(en);
        w.clk_wifibb_80m_en().bit(en);
        w.clk_wifibb_44m_en().bit(en);
        w.clk_wifibb_40m_en().bit(en);
        w.clk_wifibb_22m_en().bit(en)
    });

    regs!(MODEM_LPCON).clk_conf().modify(|_, w| {
        w.clk_wifipwr_en().bit(en);
        w.clk_coex_en().bit(en)
    });
}

#[cfg_attr(not(feature = "unstable"), expect(unused))]
pub(crate) fn enable_ieee802154(en: bool) {
    regs!(MODEM_SYSCON).clk_conf().modify(|_, w| {
        w.clk_zb_apb_en().bit(en);
        w.clk_zb_mac_en().bit(en)
    });

    regs!(MODEM_SYSCON).clk_conf1().modify(|_, w| {
        w.clk_fe_apb_en().bit(en);
        w.clk_fe_cal_160m_en().bit(en);
        w.clk_fe_160m_en().bit(en);
        w.clk_fe_80m_en().bit(en);
        w.clk_bt_apb_en().bit(en);
        w.clk_bt_en().bit(en);
        w.clk_wifibb_160x1_en().bit(en);
        w.clk_wifibb_80x1_en().bit(en);
        w.clk_wifibb_40x1_en().bit(en);
        w.clk_wifibb_80x_en().bit(en);
        w.clk_wifibb_40x_en().bit(en);
        w.clk_wifibb_80m_en().bit(en);
        w.clk_wifibb_44m_en().bit(en);
        w.clk_wifibb_40m_en().bit(en);
        w.clk_wifibb_22m_en().bit(en)
    });

    regs!(MODEM_LPCON)
        .clk_conf()
        .modify(|_, w| w.clk_coex_en().set_bit());
}

#[cfg_attr(not(feature = "unstable"), expect(unused))]
pub(crate) fn enable_bt(en: bool) {
    regs!(MODEM_SYSCON).clk_conf().modify(|_, w| {
        w.clk_etm_en().bit(en);
        w.clk_modem_sec_en().bit(en);
        w.clk_modem_sec_ecb_en().bit(en);
        w.clk_modem_sec_ccm_en().bit(en);
        w.clk_modem_sec_bah_en().bit(en);
        w.clk_modem_sec_apb_en().bit(en);
        w.clk_ble_timer_en().bit(en)
    });

    regs!(MODEM_SYSCON).clk_conf1().modify(|_, w| {
        w.clk_fe_apb_en().bit(en);
        w.clk_fe_cal_160m_en().bit(en);
        w.clk_fe_160m_en().bit(en);
        w.clk_fe_80m_en().bit(en);
        w.clk_bt_apb_en().bit(en);
        w.clk_bt_en().bit(en)
    });

    regs!(MODEM_LPCON)
        .clk_conf()
        .modify(|_, w| w.clk_coex_en().bit(en));
}

pub(crate) fn reset_wifi_mac() {
    // empty
}

pub(crate) fn init_clocks() {
    unsafe {
        regs!(PMU)
            .hp_sleep_icg_modem()
            .modify(|_, w| w.hp_sleep_dig_icg_modem_code().bits(0));
        regs!(PMU)
            .hp_modem_icg_modem()
            .modify(|_, w| w.hp_modem_dig_icg_modem_code().bits(1));
        regs!(PMU)
            .hp_active_icg_modem()
            .modify(|_, w| w.hp_active_dig_icg_modem_code().bits(2));
        regs!(PMU)
            .imm_modem_icg()
            .write(|w| w.update_dig_icg_modem_en().set_bit());
        regs!(PMU)
            .imm_sleep_sysclk()
            .write(|w| w.update_dig_icg_switch().set_bit());

        regs!(MODEM_SYSCON).clk_conf_power_st().modify(|_, w| {
            w.clk_modem_apb_st_map().bits(6);
            w.clk_modem_peri_st_map().bits(4);
            w.clk_wifi_st_map().bits(6);
            w.clk_bt_st_map().bits(6);
            w.clk_fe_st_map().bits(6);
            w.clk_zb_st_map().bits(6)
        });

        regs!(MODEM_LPCON).clk_conf_power_st().modify(|_, w| {
            w.clk_lp_apb_st_map().bits(6);
            w.clk_i2c_mst_st_map().bits(6);
            w.clk_coex_st_map().bits(6);
            w.clk_wifipwr_st_map().bits(6)
        });

        regs!(MODEM_LPCON).wifi_lp_clk_conf().modify(|_, w| {
            w.clk_wifipwr_lp_sel_osc_slow().set_bit();
            w.clk_wifipwr_lp_sel_osc_fast().set_bit();
            w.clk_wifipwr_lp_sel_xtal32k().set_bit();
            w.clk_wifipwr_lp_sel_xtal().set_bit()
        });

        regs!(MODEM_LPCON)
            .wifi_lp_clk_conf()
            .modify(|_, w| w.clk_wifipwr_lp_div_num().bits(0));

        regs!(MODEM_LPCON)
            .clk_conf()
            .modify(|_, w| w.clk_wifipwr_en().set_bit());
    }
}

pub(crate) fn ble_rtc_clk_init() {
    // nothing for this target (yet)
}

pub(crate) fn reset_rpa() {
    // nothing for this target (yet)
}
