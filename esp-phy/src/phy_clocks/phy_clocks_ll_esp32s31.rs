pub(crate) fn enable_phy(en: bool) {
    if en {
        // IDF `modem_clock_hal_enable_soc_pll_source_cg` (0x3d). PHY calibration
        // talks to analog via I2C and needs the modem PLL, not XTAL-only 0x25.
        regs!(HP_SYS_CLKRST).modem_conf().modify(|_, w| {
            w.modem_apb_clk_en().set_bit();
            w.modem_rst_en().clear_bit();
            w.modem_clk_en().set_bit();
            w.modem_clk_source_sel().set_bit();
            w.modem_pll_clk_en().set_bit();
            w.modem_xtal_clk_en().set_bit()
        });
    }

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

    regs!(MODEM_SYSCON)
        .clk_conf()
        .modify(|_, w| w.clk_i2c_mst_sel_160m().bit(en));

    regs!(MODEM_LPCON).clk_conf().modify(|_, w| {
        w.clk_i2c_mst_en().bit(en);
        w.clk_coex_en().bit(en)
    });

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
