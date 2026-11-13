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

    // from `esp_perip_clk_init`
    regs!(SYSCON).wifi_clk_en().modify(|r, w| unsafe {
        w.bits(r.bits() | DPORT_WIFI_CLK_WIFI_EN);
        w.sdioslave_clk_en().clear_bit();
        w.unused_bit5().clear_bit();
        w.bt_baseband_clk_en().clear_bit();
        w.unused_bit12().clear_bit();
        w.sdio_host_clk_en().clear_bit();
        w.emac_clk_en().clear_bit();
        w.bt_lc_clk_en().bits(0)
    });

    // The Wi-Fi light sleep clock: the RTC slow clock, undivided. The divider resets to 255.
    regs!(SYSTEM)
        .lpck_div_int()
        .modify(|_, w| unsafe { w.lpck_div_num().bits(0) });
    regs!(SYSTEM).bt_lpck_div_frac().modify(|_, w| {
        w.lpclk_sel_xtal32k().clear_bit();
        w.lpclk_sel_xtal().clear_bit();
        w.lpclk_sel_8m().clear_bit();
        w.lpclk_sel_rtc_slow().set_bit()
    });
}

pub(crate) fn deinit_clocks() {
    // Nothing to do: `enable_wifi(false)` (called by the Wi-Fi driver on
    // deinit) clears `DPORT_WIFI_CLK_WIFI_EN_M`. That mask (0x7cf) is smaller
    // than the value `init_clocks` writes (0x3807cf), so the upper bits stay
    // set — ESP-IDF's `periph_ll_wifi_module_disable_clk_set_rst` leaves them
    // set as well, and they only feed the modem, which
    // `disable_wifi_power_domain` powers off.
}
