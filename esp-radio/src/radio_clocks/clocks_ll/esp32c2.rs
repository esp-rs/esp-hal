// SYSTEM_WIFI_CLK_EN : R/W ;bitpos:[31:0] ;default: 32'hfffce030
const SYSTEM_WIFI_CLK_EN: u32 = 0x00FB9FCF;

pub(crate) fn enable_bt(_: bool) {
    // `periph_ll_wifi_module_enable_clk_clear_rst`, no-op
    // `periph_ll_wifi_module_disable_clk_clear_rst`, no-op
}

pub(crate) fn enable_wifi(_: bool) {
    // `periph_ll_wifi_module_enable_clk_clear_rst`, no-op
    // `periph_ll_wifi_module_disable_clk_clear_rst`, no-op
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
    // from `esp_perip_clk_init`
    const SYSTEM_WIFI_CLK_UNUSED_BIT5: u32 = 1 << 5;
    const SYSTEM_WIFI_CLK_UNUSED_BIT12: u32 = 1 << 12;
    const WIFI_BT_SDIO_CLK: u32 = SYSTEM_WIFI_CLK_UNUSED_BIT5 | SYSTEM_WIFI_CLK_UNUSED_BIT12;

    regs!(APB_CTRL)
        .wifi_clk_en()
        .modify(|r, w| unsafe { w.bits(r.bits() & !WIFI_BT_SDIO_CLK | SYSTEM_WIFI_CLK_EN) });
}

pub(crate) fn deinit_clocks() {
    // Nothing to do: when the last `PhyClockGuard` drops, esp-phy gates the
    // shared modem clocks (`SYSTEM_WIFI_CLK_WIFI_BT_COMMON_M`) — the same
    // state ESP-IDF leaves behind via `wifi_bt_common_module_disable`, and
    // its `periph_ll_wifi_module_disable_clk_set_rst` is a no-op on ESP32-C2.
    // Gating anything beyond that here breaks Wi-Fi re-initialization on
    // ESP32-C2.
}

pub(crate) fn ble_rtc_clk_init() {
    // `BLE_LP_CLK` is programmed by the clock tree.
}

pub(crate) fn reset_rpa() {
    regs!(APB_CTRL)
        .wifi_rst_en()
        .modify(|_, w| w.ble_rpa_rst().set_bit());
    regs!(APB_CTRL)
        .wifi_rst_en()
        .modify(|_, w| w.ble_rpa_rst().clear_bit());
}
