// SYSTEM_WIFI_CLK_EN : R/W ;bitpos:[31:0] ;default: 32'hfffce030
const SYSTEM_WIFI_CLK_EN: u32 = 0x00FB9FCF;

pub(crate) fn enable_wifi(_: bool) {
    // `periph_ll_wifi_module_enable_clk_clear_rst`, no-op
    // `periph_ll_wifi_module__clk_clear_rst`, no-op
}

pub(crate) fn enable_bt(_: bool) {
    // `periph_ll_wifi_module_enable_clk_clear_rst`, no-op
    // `periph_ll_wifi_module__clk_clear_rst`, no-op
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
    // undo the power down in base_settings (esp32c3_sleep)
    regs!(RTC_CNTL).dig_iso().modify(|_, w| {
        w.wifi_force_iso().clear_bit();
        w.bt_force_iso().clear_bit()
    });

    regs!(RTC_CNTL).dig_pwc().modify(|_, w| {
        w.wifi_force_pd().clear_bit();
        w.bt_force_pd().clear_bit()
    });

    // from `esp_perip_clk_init`
    const SYSTEM_WIFI_CLK_I2C_CLK_EN: u32 = 1 << 5;
    const SYSTEM_WIFI_CLK_UNUSED_BIT12: u32 = 1 << 12;
    const WIFI_BT_SDIO_CLK: u32 = SYSTEM_WIFI_CLK_I2C_CLK_EN | SYSTEM_WIFI_CLK_UNUSED_BIT12;

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
