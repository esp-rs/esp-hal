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
    regs!(APB_CTRL).wifi_clk_en().modify(|r, w| {
        unsafe { w.bits(r.bits() | SYSTEM_WIFI_CLK_EN) };
        w.i2c_clk_en().clear_bit();
        w.sdio_host_clk_en().clear_bit()
    });
}

pub(crate) fn deinit_clocks() {
    // Nothing to do: ESP-IDF's Wi-Fi disable mask
    // (`SYSTEM_WIFI_CLK_WIFI_EN_M`) is 0 on this chip — its
    // `periph_ll_wifi_module_disable_clk_set_rst` clears nothing — and
    // `disable_wifi_power_domain` powers the modem power domain off.
}

pub(crate) fn ble_rtc_clk_init() {
    // nothing for this target
}

pub(crate) fn reset_rpa() {
    // nothing for this target
}
