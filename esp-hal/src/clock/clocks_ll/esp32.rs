use crate::peripherals::DPORT;

// TODO: define these in the PAC
const DPORT_WIFI_CLK_WIFI_BT_COMMON_M: u32 = 0x000003c9;
const DPORT_WIFI_CLK_WIFI_EN_M: u32 = 0x00000406;
const DPORT_WIFI_CLK_BT_EN_M: u32 = 0x00030800;

pub(super) fn enable_phy(enable: bool) {
    // `periph_ll_wifi_bt_module_enable_clk_clear_rst`
    // `periph_ll_wifi_bt_module_disable_clk_set_rst`
    DPORT::regs().wifi_clk_en().modify(|r, w| unsafe {
        if enable {
            w.bits(r.bits() | DPORT_WIFI_CLK_WIFI_BT_COMMON_M)
        } else {
            w.bits(r.bits() & !DPORT_WIFI_CLK_WIFI_BT_COMMON_M)
        }
    });
}

#[cfg_attr(not(feature = "unstable"), expect(unused))]
pub(super) fn enable_bt(enable: bool) {
    DPORT::regs().wifi_clk_en().modify(|r, w| unsafe {
        if enable {
            w.bits(r.bits() | DPORT_WIFI_CLK_BT_EN_M)
        } else {
            w.bits(r.bits() & !DPORT_WIFI_CLK_BT_EN_M)
        }
    });
}

#[cfg_attr(not(feature = "unstable"), expect(unused))]
pub(super) fn enable_wifi(enable: bool) {
    // `periph_ll_wifi_module_enable_clk_clear_rst`
    // `periph_ll_wifi_module_disable_clk_set_rst`
    DPORT::regs().wifi_clk_en().modify(|r, w| unsafe {
        if enable {
            w.bits(r.bits() | DPORT_WIFI_CLK_WIFI_EN_M)
        } else {
            w.bits(r.bits() & !DPORT_WIFI_CLK_WIFI_EN_M)
        }
    });
}

pub(super) fn reset_wifi_mac() {
    DPORT::regs()
        .wifi_rst_en()
        .modify(|_, w| w.mac_rst().set_bit());
    DPORT::regs()
        .wifi_rst_en()
        .modify(|_, w| w.mac_rst().clear_bit());
}

pub(super) fn init_clocks() {
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

    DPORT::regs()
        .wifi_clk_en()
        .write(|w| unsafe { w.bits(u32::MAX) });
}

pub(super) fn ble_rtc_clk_init() {
    // nothing for this target
}

pub(super) fn reset_rpa() {
    // nothing for this target
}
