// TODO: define these in the PAC
const DPORT_WIFI_CLK_WIFI_EN_M: u32 = 0x00000406;
const DPORT_WIFI_CLK_BT_EN_M: u32 = 0x00030800;

#[cfg_attr(not(feature = "unstable"), expect(unused))]
pub(crate) fn enable_bt(enable: bool) {
    regs!(DPORT).wifi_clk_en().modify(|r, w| unsafe {
        if enable {
            w.bits(r.bits() | DPORT_WIFI_CLK_BT_EN_M)
        } else {
            w.bits(r.bits() & !DPORT_WIFI_CLK_BT_EN_M)
        }
    });
}

#[cfg_attr(not(feature = "unstable"), expect(unused))]
pub(crate) fn enable_wifi(enable: bool) {
    // `periph_ll_wifi_module_enable_clk_clear_rst`
    // `periph_ll_wifi_module_disable_clk_set_rst`
    regs!(DPORT).wifi_clk_en().modify(|r, w| unsafe {
        if enable {
            w.bits(r.bits() | DPORT_WIFI_CLK_WIFI_EN_M)
        } else {
            w.bits(r.bits() & !DPORT_WIFI_CLK_WIFI_EN_M)
        }
    });
}

pub(crate) fn reset_wifi_mac() {
    regs!(DPORT)
        .wifi_rst_en()
        .modify(|_, w| w.mac_rst().set_bit());
    regs!(DPORT)
        .wifi_rst_en()
        .modify(|_, w| w.mac_rst().clear_bit());
}

pub(crate) fn init_clocks() {
    // esp-idf assumes all clocks are enabled by default, and disables the following
    // bits:
    //
    // ```
    // const DPORT_WIFI_CLK_SDIOSLAVE_EN: u32 = 1 << 4;
    // const DPORT_WIFI_CLK_UNUSED_BIT5: u32 = 1 << 5;
    // const DPORT_WIFI_CLK_UNUSED_BIT12: u32 = 1 << 12;
    // const DPORT_WIFI_CLK_SDIO_HOST_EN: u32 = 1 << 13;
    // const DPORT_WIFI_CLK_EMAC_EN: u32 = 1 << 14;
    //
    // const WIFI_BT_SDIO_CLK: u32 = DPORT_WIFI_CLK_WIFI_EN_M
    //     | DPORT_WIFI_CLK_BT_EN_M
    //     | DPORT_WIFI_CLK_UNUSED_BIT5
    //     | DPORT_WIFI_CLK_UNUSED_BIT12
    //     | DPORT_WIFI_CLK_SDIOSLAVE_EN
    //     | DPORT_WIFI_CLK_SDIO_HOST_EN
    //     | DPORT_WIFI_CLK_EMAC_EN;
    // ```
    //
    // However, we can't do this because somehow our initialization process is
    // different, and disabling some bits, or not enabling them makes the BT
    // stack crash.

    regs!(DPORT)
        .wifi_clk_en()
        .write(|w| unsafe { w.bits(u32::MAX) });
}

pub(crate) fn ble_rtc_clk_init() {
    // nothing for this target
}

pub(crate) fn reset_rpa() {
    // nothing for this target
}
