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
            w.clk_bt_st_map().bits(6);
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
    // `clk_wifipwr_en` stays on after the first enable. It drives the timer that wakes the modem
    // for the next beacon, and the PHY is disabled at every doze.
    regs!(MODEM_LPCON).clk_conf().modify(|_, w| {
        w.clk_coex_en().bit(en);
        if en {
            w.clk_wifipwr_en().set_bit();
        }
        w
    });

    regs!(MODEM_SYSCON).clk_conf1().modify(|_, w| {
        w.clk_fe_apb_en().bit(en);
        w.clk_fe_20m_en().bit(en);
        w.clk_fe_40m_en().bit(en);
        w.clk_fe_80m_en().bit(en);
        w.clk_fe_160m_en().bit(en);
        w.clk_fe_dac_en().bit(en);
        w.clk_fe_pwdet_adc_en().bit(en);
        w.clk_fe_adc_en().bit(en);

        // Treat this as shared, BLE needs it.
        // Must be kept enabled or many things fail.
        w.clk_wifi_apb_en().bit(true);
        w.clk_wifibb_80x1_en().bit(true);

        w.clk_wifibb_160x1_en().bit(en);
        w.clk_wifibb_40x1_en().bit(en);
        w.clk_wifibb_80x_en().bit(en);
        w.clk_wifibb_40x_en().bit(en);
        w.clk_wifibb_80m_en().bit(en);
        w.clk_wifibb_44m_en().bit(en);
        w.clk_wifibb_40m_en().bit(en);
        w.clk_wifibb_22m_en().bit(en)
    });
}
