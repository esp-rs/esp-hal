use crate::{
    clock::{Clock, CpuClock},
    peripherals::APB_CTRL,
    rom,
    system::{RadioClockController, RadioPeripherals},
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

impl RadioClockController for crate::peripherals::RADIO_CLK {
    fn enable(&mut self, peripheral: RadioPeripherals) {
        match peripheral {
            RadioPeripherals::Phy => enable_phy(),
            RadioPeripherals::Bt => common_wifi_bt_clock_enable(),
            RadioPeripherals::Wifi => common_wifi_bt_clock_enable(),
        }
    }

    fn disable(&mut self, peripheral: RadioPeripherals) {
        match peripheral {
            RadioPeripherals::Phy => disable_phy(),
            RadioPeripherals::Bt => common_wifi_bt_clock_disable(),
            RadioPeripherals::Wifi => common_wifi_bt_clock_disable(),
        }
    }

    fn reset_mac(&mut self) {
        reset_mac();
    }

    fn init_clocks(&mut self) {
        init_clocks();
    }

    fn ble_rtc_clk_init(&mut self) {
        // nothing for this target
    }

    fn reset_rpa(&mut self) {
        // nothing for this target
    }
}

fn enable_phy() {
    // `periph_ll_wifi_bt_module_enable_clk_clear_rst`
    APB_CTRL::regs()
        .wifi_clk_en()
        .modify(|r, w| unsafe { w.bits(r.bits() | SYSTEM_WIFI_CLK_WIFI_BT_COMMON_M) });
}

fn disable_phy() {
    // `periph_ll_wifi_bt_module_disable_clk_set_rst`
    APB_CTRL::regs()
        .wifi_clk_en()
        .modify(|r, w| unsafe { w.bits(r.bits() & !SYSTEM_WIFI_CLK_WIFI_BT_COMMON_M) });
}

fn common_wifi_bt_clock_enable() {
    // `periph_ll_wifi_module_enable_clk_clear_rst`. does nothing
}

fn common_wifi_bt_clock_disable() {
    // `periph_ll_wifi_module_disable_clk_set_rst`. does nothing
}

fn reset_mac() {
    const SYSTEM_MAC_RST: u32 = 1 << 2;
    APB_CTRL::regs()
        .wifi_rst_en()
        .modify(|r, w| unsafe { w.bits(r.bits() | SYSTEM_MAC_RST) });
    APB_CTRL::regs()
        .wifi_rst_en()
        .modify(|r, w| unsafe { w.bits(r.bits() & !SYSTEM_MAC_RST) });
}

fn init_clocks() {
    const SYSTEM_WIFI_CLK_I2C_CLK_EN: u32 = 1 << 5;
    const SYSTEM_WIFI_CLK_UNUSED_BIT12: u32 = 1 << 12;
    const SYSTEM_WIFI_CLK_SDIO_HOST_EN: u32 = 1 << 13;

    const WIFI_BT_SDIO_CLK: u32 =
        SYSTEM_WIFI_CLK_I2C_CLK_EN | SYSTEM_WIFI_CLK_UNUSED_BIT12 | SYSTEM_WIFI_CLK_SDIO_HOST_EN;

    APB_CTRL::regs()
        .wifi_clk_en()
        .modify(|r, w| unsafe { w.bits(r.bits() & !WIFI_BT_SDIO_CLK | SYSTEM_WIFI_CLK_EN) });
}
