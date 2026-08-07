const DPORT_WIFI_CLK_WIFI_EN_M: u32 = 0x000007cf;

pub(crate) fn enable_wifi(enable: bool) {
    // `periph_ll_wifi_module_enable_clk_clear_rst`
    // `periph_ll_wifi_module_disable_clk_set_rst`
    regs!(SYSCON).wifi_clk_en().modify(|r, w| unsafe {
        if enable {
            w.bits(r.bits() | DPORT_WIFI_CLK_WIFI_EN_M)
        } else {
            w.bits(r.bits() & !DPORT_WIFI_CLK_WIFI_EN_M)
        }
    });
}

pub(crate) fn reset_wifi_mac() {
    regs!(SYSCON)
        .wifi_rst_en()
        .modify(|_, w| w.mac_rst().set_bit());
    regs!(SYSCON)
        .wifi_rst_en()
        .modify(|_, w| w.mac_rst().clear_bit());
}

pub(crate) fn init_clocks() {
    const DPORT_WIFI_CLK_WIFI_EN: u32 = 0x003807cf;
    const DPORT_WIFI_CLK_BT_EN_M: u32 = 0x61 << 11;
    const DPORT_WIFI_CLK_SDIOSLAVE_EN: u32 = 1 << 4;
    const DPORT_WIFI_CLK_UNUSED_BIT5: u32 = 1 << 5;
    const DPORT_WIFI_CLK_UNUSED_BIT12: u32 = 1 << 12;
    const DPORT_WIFI_CLK_SDIO_HOST_EN: u32 = 1 << 13;
    const DPORT_WIFI_CLK_EMAC_EN: u32 = 1 << 14;

    const WIFI_BT_SDIO_CLK: u32 = DPORT_WIFI_CLK_WIFI_EN
        | DPORT_WIFI_CLK_BT_EN_M
        | DPORT_WIFI_CLK_UNUSED_BIT5
        | DPORT_WIFI_CLK_UNUSED_BIT12
        | DPORT_WIFI_CLK_SDIOSLAVE_EN
        | DPORT_WIFI_CLK_SDIO_HOST_EN
        | DPORT_WIFI_CLK_EMAC_EN;

    regs!(SYSCON)
        .wifi_clk_en()
        .modify(|r, w| unsafe { w.bits(r.bits() & !WIFI_BT_SDIO_CLK | DPORT_WIFI_CLK_WIFI_EN) });
}

pub(crate) fn deinit_clocks() {
    // Nothing to do: `enable_wifi(false)` (called by the Wi-Fi driver on
    // deinit) clears `DPORT_WIFI_CLK_WIFI_EN_M`. That mask (0x7cf) is smaller
    // than the value `init_clocks` writes (0x3807cf), so the upper bits stay
    // set — ESP-IDF's `periph_ll_wifi_module_disable_clk_set_rst` leaves them
    // set as well, and they only feed the modem, which
    // `disable_wifi_power_domain` powers off.
}
