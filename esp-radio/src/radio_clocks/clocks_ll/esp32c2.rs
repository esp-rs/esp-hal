use crate::{
    clock::Clocks,
};

// Mask for clock bits used by both WIFI and Bluetooth, 0, 1, 2, 3, 7, 8, 9, 10,
// 19, 20, 21, 22, 23
const SYSTEM_WIFI_CLK_WIFI_BT_COMMON_M: u32 = 0x78078F;
// SYSTEM_WIFI_CLK_EN : R/W ;bitpos:[31:0] ;default: 32'hfffce030
const SYSTEM_WIFI_CLK_EN: u32 = 0x00FB9FCF;

pub(crate) fn enable_phy(enable: bool) {
    // `periph_ll_wifi_bt_module_enable_clk_clear_rst`
    // `periph_ll_wifi_bt_module_disable_clk_set_rst`
    regs!(APB_CTRL).wifi_clk_en().modify(|r, w| unsafe {
        if enable {
            w.bits(r.bits() | SYSTEM_WIFI_CLK_WIFI_BT_COMMON_M)
        } else {
            w.bits(r.bits() & !SYSTEM_WIFI_CLK_WIFI_BT_COMMON_M)
        }
    });
}

#[cfg_attr(not(feature = "unstable"), expect(unused))]
pub(crate) fn enable_bt(_: bool) {
    // `periph_ll_wifi_module_enable_clk_clear_rst`, no-op
    // `periph_ll_wifi_module_disable_clk_clear_rst`, no-op
}

#[cfg_attr(not(feature = "unstable"), expect(unused))]
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

pub(crate) fn ble_rtc_clk_init() {
    regs!(MODEM_CLKRST).modem_lp_timer_conf().modify(|_, w| {
        w.lp_timer_sel_xtal32k().clear_bit();
        w.lp_timer_sel_xtal().set_bit();
        w.lp_timer_sel_8m().clear_bit();
        w.lp_timer_sel_rtc_slow().clear_bit()
    });

    let xtal_frequency = Clocks::get().xtal_clock;
    let bt_lpclk_target = match xtal_frequency.as_mhz() {
        26 => 40000,
        // 40MHz
        _ => 32000,
    };
    let divider = xtal_frequency.as_hz() / (5 * bt_lpclk_target) - 1;

    regs!(MODEM_CLKRST)
        .modem_lp_timer_conf()
        .modify(|_, w| unsafe { w.lp_timer_clk_div_num().bits(divider as u8) });

    regs!(MODEM_CLKRST).etm_clk_conf().modify(|_, w| {
        w.etm_clk_active().set_bit();
        w.etm_clk_sel().clear_bit()
    });
}

pub(crate) fn reset_rpa() {
    regs!(APB_CTRL)
        .wifi_rst_en()
        .modify(|_, w| w.ble_rpa_rst().set_bit());
    regs!(APB_CTRL)
        .wifi_rst_en()
        .modify(|_, w| w.ble_rpa_rst().clear_bit());
}
