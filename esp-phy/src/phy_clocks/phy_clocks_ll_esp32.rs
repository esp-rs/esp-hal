const DPORT_WIFI_CLK_WIFI_BT_COMMON_M: u32 = 0x000003c9;

pub(crate) fn enable_phy(enable: bool) {
    // `periph_ll_wifi_bt_module_enable_clk_clear_rst`
    // `periph_ll_wifi_bt_module_disable_clk_set_rst`
    regs!(DPORT).wifi_clk_en().modify(|r, w| unsafe {
        if enable {
            w.bits(r.bits() | DPORT_WIFI_CLK_WIFI_BT_COMMON_M)
        } else {
            w.bits(r.bits() & !DPORT_WIFI_CLK_WIFI_BT_COMMON_M)
        }
    });
}
