pub(crate) fn enable_phy(en: bool) {
    regs!(MODEM_LPCON)
        .clk_conf()
        .modify(|_, w| w.clk_i2c_mst_en().bit(en));
}

/// `MODEM_SYSCON.clk_conf`: `clk_modem_sec_apb_en`.
pub(crate) const CALIBRATION_CLK_CONF_MASK: u32 = 1 << 27;
/// `MODEM_SYSCON.clk_conf1`: `clk_fe_*_en` (bits 12-16), `clk_bt_apb_en` and `clk_bt_en`.
pub(crate) const CALIBRATION_CLK_CONF1_MASK: u32 = 0x1f << 12 | 1 << 17 | 1 << 18;
