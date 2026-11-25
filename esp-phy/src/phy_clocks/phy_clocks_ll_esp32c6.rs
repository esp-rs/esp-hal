pub(crate) fn enable_phy(en: bool) {
    // The PHY owns the front-end clocks, as in ESP-IDF's `PHY_CLOCK_DEPS`. Every radio needs them
    // while the PHY is on, and `phy_wakeup_init` needs them too, so a radio must not gate them
    // while it dozes. They stay on after the PHY turns off, as in ESP-IDF: the REGDMA saves the
    // front-end registers on the way into light sleep, when the PHY is already off.
    if en {
        regs!(MODEM_SYSCON).clk_conf1().modify(|_, w| {
            w.clk_fe_apb_en().set_bit();
            w.clk_fe_cal_160m_en().set_bit();
            w.clk_fe_160m_en().set_bit();
            w.clk_fe_80m_en().set_bit()
        });
    }
    regs!(MODEM_LPCON)
        .clk_conf()
        .modify(|_, w| w.clk_i2c_mst_en().bit(en));
    regs!(MODEM_LPCON)
        .i2c_mst_clk_conf()
        .modify(|_, w| w.clk_i2c_mst_sel_160m().bit(en));
}

/// `MODEM_SYSCON.clk_conf`: `clk_modem_sec_apb_en`.
pub(crate) const CALIBRATION_CLK_CONF_MASK: u32 = 1 << 28;
/// `MODEM_SYSCON.clk_conf1`: `clk_wifibb_*_en` (bits 0-8), `clk_bt_apb_en` and `clk_bt_en`.
pub(crate) const CALIBRATION_CLK_CONF1_MASK: u32 = 0x1ff | 1 << 17 | 1 << 18;
