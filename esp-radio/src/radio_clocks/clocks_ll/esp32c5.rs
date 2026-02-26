#[allow(unused)]
pub(crate) fn enable_phy(en: bool) {
    regs!(MODEM_SYSCON)
        .clk_conf_power_st()
        .modify(|_, w| unsafe {
            w.clk_modem_apb_st_map().bits(6);
            w.clk_modem_peri_st_map().bits(4);
            w.clk_wifi_st_map().bits(6);
            w.clk_bt_st_map().bits(4);
            w.clk_fe_st_map().bits(6);
            w.clk_zb_st_map().bits(4)
        });

    regs!(MODEM_LPCON)
        .clk_conf_power_st()
        .modify(|_, w| unsafe {
            w.clk_lp_apb_st_map().bits(6);
            w.clk_i2c_mst_st_map().bits(6);
            w.clk_coex_st_map().bits(6);
            w.clk_wifipwr_st_map().bits(6)
        });

    regs!(MODEM_LPCON)
        .clk_conf()
        .modify(|_, w| w.clk_coex_en().bit(en));

    regs!(MODEM_SYSCON).clk_conf1().modify(|_, w| {
        w.clk_wifi_apb_en().bit(en);
        w.clk_wifibb_22m_en().bit(en);
        w.clk_fe_40m_en().bit(en);
        w.clk_fe_80m_en().bit(en);
        w.clk_wifibb_44m_en().bit(en);
        w.clk_wifimac_en().bit(en)
    });

    regs!(MODEM_SYSCON)
        .clk_conf1()
        .modify(|r, w| unsafe { w.bits(r.bits() | 0x1fb) });

    regs!(MODEM_SYSCON).clk_conf1().modify(|_, w| {
        w.clk_fe_apb_en().bit(en);
        w.clk_fe_80m_en().bit(en);
        w.clk_fe_160m_en().bit(en);
        w.clk_fe_dac_en().bit(en);
        w.clk_fe_pwdet_adc_en().bit(en);
        w.clk_fe_adc_en().bit(en)
    });
}

#[allow(unused)]
pub(crate) fn enable_wifi(en: bool) {
    regs!(MODEM_SYSCON).clk_conf1().modify(|_, w| {
        w.clk_wifi_apb_en().bit(en);
        w.clk_wifibb_44m_en().bit(en);
        w.clk_wifimac_en().bit(en);
        w.clk_fe_apb_en().bit(en);
        w.clk_fe_80m_en().bit(en);
        w.clk_fe_160m_en().bit(en)
    });
}

#[allow(unused)]
pub(crate) fn enable_ieee802154(en: bool) {
    regs!(MODEM_SYSCON).clk_conf().modify(|r, w| {
        w.clk_etm_en().bit(en);
        w.clk_zb_apb_en().bit(en);
        w.clk_zbmac_en().bit(en);
        w.clk_modem_sec_en().bit(en);
        w.clk_modem_sec_ecb_en().bit(en);
        w.clk_modem_sec_ccm_en().bit(en);
        w.clk_modem_sec_bah_en().bit(en);
        w.clk_modem_sec_apb_en().bit(en);
        w.clk_ble_timer_en().bit(en)
    });

    regs!(MODEM_SYSCON).clk_conf1().modify(|_, w| {
        w.clk_fe_apb_en().bit(en);
        w.clk_bt_apb_en().bit(en);
        w.clk_btbb_en().bit(en);
        w.clk_btmac_en().bit(en);
        w.clk_fe_20m_en().bit(en);
        w.clk_fe_40m_en().bit(en);
        w.clk_fe_80m_en().bit(en);
        w.clk_fe_160m_en().bit(en)
    });
}

#[allow(unused)]
pub(crate) fn enable_bt(en: bool) {
    regs!(MODEM_SYSCON).clk_conf().modify(|r, w| {
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
        w.clk_bt_apb_en().bit(en);
        w.clk_btbb_en().bit(en);
        w.clk_btmac_en().bit(en);
        w.clk_fe_20m_en().bit(en);
        w.clk_fe_40m_en().bit(en);
        w.clk_fe_80m_en().bit(en);
        w.clk_fe_160m_en().bit(en)
    });
}

#[allow(unused)]
pub(crate) fn reset_wifi_mac() {
    // empty
}

// #[allow(unused)]
pub(crate) fn init_clocks() {
    regs!(MODEM_LPCON)
        .clk_conf()
        .modify(|_, w| w.clk_i2c_mst_en().set_bit().clk_wifipwr_en().set_bit());
}

#[allow(unused)]
pub(crate) fn ble_rtc_clk_init() {
    // nothing for this target (yet)
}

#[allow(unused)]
pub(crate) fn reset_rpa() {
    // nothing for this target (yet)
}
