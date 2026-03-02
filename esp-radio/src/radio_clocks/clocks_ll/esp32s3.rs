// SYSTEM_WIFI_CLK_EN : R/W ;bitpos:[31:0] ;default: 32'hfffce030
const SYSTEM_WIFI_CLK_EN: u32 = 0x00FB9FCF;

pub(crate) fn enable_bt(_: bool) {
    // `periph_ll_wifi_module_enable_clk_clear_rst`. does nothing
    // `periph_ll_wifi_module_disable_clk_set_rst`. does nothing
}

pub(crate) fn enable_wifi(_: bool) {
    // `periph_ll_wifi_module_enable_clk_clear_rst`. does nothing
    // `periph_ll_wifi_module_disable_clk_set_rst`. does nothing
}

pub(crate) fn reset_wifi_mac() {
    regs!(APB_CTRL)
        .wifi_rst_en()
        .modify(|_, w| w.mac_rst().set_bit());
    regs!(APB_CTRL)
        .wifi_rst_en()
        .modify(|_, w| w.mac_rst().clear_bit());
}

pub(crate) fn init_clocks() {
    const SYSTEM_WIFI_CLK_I2C_CLK_EN: u32 = 1 << 5;
    const SYSTEM_WIFI_CLK_UNUSED_BIT12: u32 = 1 << 12;
    const SYSTEM_WIFI_CLK_SDIO_HOST_EN: u32 = 1 << 13;

    const WIFI_BT_SDIO_CLK: u32 =
        SYSTEM_WIFI_CLK_I2C_CLK_EN | SYSTEM_WIFI_CLK_UNUSED_BIT12 | SYSTEM_WIFI_CLK_SDIO_HOST_EN;

    regs!(APB_CTRL)
        .wifi_clk_en()
        .modify(|r, w| unsafe { w.bits(r.bits() & !WIFI_BT_SDIO_CLK | SYSTEM_WIFI_CLK_EN) });
}

pub(crate) fn ble_rtc_clk_init() {
    // nothing for this target
}

pub(crate) fn reset_rpa() {
    // nothing for this target
}
