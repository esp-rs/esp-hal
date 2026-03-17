pub(crate) fn enable_phy(en: bool) {
    regs!(MODEM_LPCON)
        .clk_conf()
        .modify(|_, w| w.clk_i2c_mst_en().bit(en));
    regs!(MODEM_LPCON)
        .i2c_mst_clk_conf()
        .modify(|_, w| w.clk_i2c_mst_sel_160m().bit(en));
}
