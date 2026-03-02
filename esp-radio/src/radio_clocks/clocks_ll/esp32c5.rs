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
