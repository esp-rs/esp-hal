use crate::peripherals::{APB_CTRL, LPWR};

// Mask for clock bits used by both WIFI and Bluetooth, 0, 1, 2, 3, 7, 8, 9, 10,
// 19, 20, 21, 22, 23
const SYSTEM_WIFI_CLK_WIFI_BT_COMMON_M: u32 = 0x78078F;
// SYSTEM_WIFI_CLK_EN : R/W ;bitpos:[31:0] ;default: 32'hfffce030
const SYSTEM_WIFI_CLK_EN: u32 = 0x00FB9FCF;

pub(super) fn enable_phy(enable: bool) {
    // `periph_ll_wifi_bt_module_enable_clk_clear_rst`
    // `periph_ll_wifi_bt_module_disable_clk_set_rst`
    APB_CTRL::regs().wifi_clk_en().modify(|r, w| unsafe {
        if enable {
            w.bits(r.bits() | SYSTEM_WIFI_CLK_WIFI_BT_COMMON_M)
        } else {
            w.bits(r.bits() & !SYSTEM_WIFI_CLK_WIFI_BT_COMMON_M)
        }
    });
}

#[cfg_attr(not(feature = "unstable"), expect(unused))]
pub(super) fn enable_wifi(_: bool) {
    // `periph_ll_wifi_module_enable_clk_clear_rst`, no-op
    // `periph_ll_wifi_module__clk_clear_rst`, no-op
}

#[cfg_attr(not(feature = "unstable"), expect(unused))]
pub(super) fn enable_bt(_: bool) {
    // `periph_ll_wifi_module_enable_clk_clear_rst`, no-op
    // `periph_ll_wifi_module__clk_clear_rst`, no-op
}

pub(super) fn reset_wifi_mac() {
    APB_CTRL::regs()
        .wifi_rst_en()
        .modify(|_, w| w.mac_rst().set_bit());
    APB_CTRL::regs()
        .wifi_rst_en()
        .modify(|_, w| w.mac_rst().clear_bit());
}

pub(super) fn init_clocks() {
    // undo the power down in base_settings (esp32c3_sleep)
    LPWR::regs().dig_iso().modify(|_, w| {
        w.wifi_force_iso().clear_bit();
        w.bt_force_iso().clear_bit()
    });

    LPWR::regs().dig_pwc().modify(|_, w| {
        w.wifi_force_pd().clear_bit();
        w.bt_force_pd().clear_bit()
    });

    // from `esp_perip_clk_init`
    const SYSTEM_WIFI_CLK_I2C_CLK_EN: u32 = 1 << 5;
    const SYSTEM_WIFI_CLK_UNUSED_BIT12: u32 = 1 << 12;
    const WIFI_BT_SDIO_CLK: u32 = SYSTEM_WIFI_CLK_I2C_CLK_EN | SYSTEM_WIFI_CLK_UNUSED_BIT12;

    APB_CTRL::regs()
        .wifi_clk_en()
        .modify(|r, w| unsafe { w.bits(r.bits() & !WIFI_BT_SDIO_CLK | SYSTEM_WIFI_CLK_EN) });
}

pub(super) fn ble_rtc_clk_init() {
    // nothing for this target
}

pub(super) fn reset_rpa() {
    // nothing for this target
}
