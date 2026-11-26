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
        w.clk_fe_apb_en().bit(en);
        w.clk_fe_20m_en().bit(en);
        w.clk_fe_40m_en().bit(en);
        w.clk_fe_80m_en().bit(en);
        w.clk_fe_160m_en().bit(en);
        w.clk_fe_dac_en().bit(en);
        w.clk_fe_pwdet_adc_en().bit(en);
        w.clk_fe_adc_en().bit(en)
    });
}

/// `MODEM_SYSCON.clk_conf`: `clk_modem_sec_apb_en`.
pub(crate) const CALIBRATION_CLK_CONF_MASK: u32 = 1 << 28;
/// `MODEM_SYSCON.clk_conf1`: `clk_wifibb_*_en` (bits 0-8), `clk_wifi_apb_en`, `clk_bt_apb_en` and
/// `clk_btbb_en`.
pub(crate) const CALIBRATION_CLK_CONF1_MASK: u32 = 0x1ff | 1 << 10 | 1 << 16 | 1 << 17;
