use crate::{
    clock::{Clock, CpuClock},
    peripherals::APB_CTRL,
    rom,
};

pub(crate) fn set_cpu_clock(cpu_clock_speed: CpuClock) {
    let system_control = crate::peripherals::SYSTEM::regs();

    unsafe {
        system_control
            .sysclk_conf()
            .modify(|_, w| w.soc_clk_sel().bits(1));
        system_control.cpu_per_conf().modify(|_, w| {
            w.pll_freq_sel()
                .set_bit()
                .cpuperiod_sel()
                .bits(match cpu_clock_speed {
                    CpuClock::_80MHz => 0,
                    CpuClock::_160MHz => 1,
                    CpuClock::_240MHz => 2,
                })
        });
    }

    rom::ets_update_cpu_frequency_rom(cpu_clock_speed.frequency().as_mhz());
}

// Note: this comment has been copied from esp-idf, including the mistake.
// Mask for clock bits used by both WIFI and Bluetooth, 0, 1, 2, 3, 7,
// 8, 9, 10, 19, 20, 21, 22, 23
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

pub(super) fn enable_bt(_: bool) {
    // `periph_ll_wifi_module_enable_clk_clear_rst`. does nothing
    // `periph_ll_wifi_module_disable_clk_set_rst`. does nothing
}

pub(super) fn enable_wifi(_: bool) {
    // `periph_ll_wifi_module_enable_clk_clear_rst`. does nothing
    // `periph_ll_wifi_module_disable_clk_set_rst`. does nothing
}

pub(super) fn reset_mac() {
    const SYSTEM_MAC_RST: u32 = 1 << 2;
    APB_CTRL::regs()
        .wifi_rst_en()
        .modify(|r, w| unsafe { w.bits(r.bits() | SYSTEM_MAC_RST) });
    APB_CTRL::regs()
        .wifi_rst_en()
        .modify(|r, w| unsafe { w.bits(r.bits() & !SYSTEM_MAC_RST) });
}

pub(super) fn init_clocks() {
    const SYSTEM_WIFI_CLK_I2C_CLK_EN: u32 = 1 << 5;
    const SYSTEM_WIFI_CLK_UNUSED_BIT12: u32 = 1 << 12;
    const SYSTEM_WIFI_CLK_SDIO_HOST_EN: u32 = 1 << 13;

    const WIFI_BT_SDIO_CLK: u32 =
        SYSTEM_WIFI_CLK_I2C_CLK_EN | SYSTEM_WIFI_CLK_UNUSED_BIT12 | SYSTEM_WIFI_CLK_SDIO_HOST_EN;

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
