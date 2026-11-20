pub(crate) fn enable_phy(en: bool) {
    // IDF `PHY_CLOCK_DEPS`: the common and private FE clocks and the analog I2C master.
    regs!(MODEM_SYSCON).clk_conf1().modify(|_, w| {
        w.clk_fe_apb_en().bit(en);
        w.clk_fe_32m_en().bit(en);
        w.clk_fe_16m_en().bit(en);
        w.clk_fe_adc_en().bit(en);
        w.clk_fe_sdm_en().bit(en)
    });

    regs!(MODEM_LPCON)
        .clk_conf()
        .modify(|_, w| w.clk_i2c_mst_en().bit(en));
}

/// `MODEM_SYSCON.clk_conf`: `clk_modem_sec_apb_en`.
pub(crate) const CALIBRATION_CLK_CONF_MASK: u32 = 1 << 28;
/// `MODEM_SYSCON.clk_conf1`: `clk_bt_apb_en` and `clk_btbb_en`.
pub(crate) const CALIBRATION_CLK_CONF1_MASK: u32 = 1 << 16 | 1 << 17;
