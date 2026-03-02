pub(crate) fn enable_phy(en: bool) {
    esp_hal::peripherals::MODEM_LPCON::regs()
        .clk_conf()
        .modify(|_, w| w.clk_i2c_mst_en().bit(en));
    esp_hal::peripherals::MODEM_LPCON::regs()
        .i2c_mst_clk_conf()
        .modify(|_, w| w.clk_i2c_mst_sel_160m().bit(en));
}
