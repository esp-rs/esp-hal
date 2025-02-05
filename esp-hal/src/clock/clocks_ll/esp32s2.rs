use crate::{
    clock::CpuClock,
    peripherals::SYSCON,
    system::{RadioClockController, RadioPeripherals},
};

const MHZ: u32 = 1000000;
const UINT16_MAX: u32 = 0xffff;

const RTC_CNTL_DBIAS_1V25: u32 = 7;

// when not running with 80MHz Flash frequency we could use RTC_CNTL_DBIAS_1V10
// for DIG_DBIAS_80M_160M - but RTC_CNTL_DBIAS_1V25 shouldn't hurt
const DIG_DBIAS_80M_160M: u32 = RTC_CNTL_DBIAS_1V25;
const DIG_DBIAS_240M: u32 = RTC_CNTL_DBIAS_1V25;

pub(crate) fn set_cpu_clock(cpu_clock_speed: CpuClock) {
    let system_control = crate::peripherals::SYSTEM::regs();
    let rtc_cntl = crate::peripherals::LPWR::regs();

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

        rtc_cntl.reg().modify(|_, w| {
            w.dig_reg_dbias_wak().bits(match cpu_clock_speed {
                CpuClock::_80MHz => DIG_DBIAS_80M_160M,
                CpuClock::_160MHz => DIG_DBIAS_80M_160M,
                CpuClock::_240MHz => DIG_DBIAS_240M,
            } as u8)
        });

        // FIXME untangle this
        let value = (((80 * MHZ) >> 12) & UINT16_MAX) | ((((80 * MHZ) >> 12) & UINT16_MAX) << 16);
        rtc_cntl.store5().modify(|_, w| w.scratch5().bits(value));
    }
}

// Mask for clock bits used by both WIFI and Bluetooth, bit 0, 3, 6, 7, 8, 9
const DPORT_WIFI_CLK_WIFI_BT_COMMON_M: u32 = 0x000003c9;
const DPORT_WIFI_CLK_WIFI_EN_M: u32 = 0x000007cf;

impl RadioClockController for crate::peripherals::RADIO_CLK {
    fn enable(&mut self, peripheral: RadioPeripherals) {
        match peripheral {
            RadioPeripherals::Phy => enable_phy(),
            RadioPeripherals::Wifi => wifi_clock_enable(),
        }
    }

    fn disable(&mut self, peripheral: RadioPeripherals) {
        match peripheral {
            RadioPeripherals::Phy => disable_phy(),
            RadioPeripherals::Wifi => wifi_clock_disable(),
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
    SYSCON::regs()
        .wifi_clk_en()
        .modify(|r, w| unsafe { w.bits(r.bits() | DPORT_WIFI_CLK_WIFI_BT_COMMON_M) });
}

fn disable_phy() {
    // `periph_ll_wifi_bt_module_disable_clk_set_rst`
    SYSCON::regs()
        .wifi_clk_en()
        .modify(|r, w| unsafe { w.bits(r.bits() & !DPORT_WIFI_CLK_WIFI_BT_COMMON_M) });
}

fn wifi_clock_enable() {
    // `periph_ll_wifi_module_enable_clk_clear_rst`
    SYSCON::regs()
        .wifi_clk_en()
        .modify(|r, w| unsafe { w.bits(r.bits() | DPORT_WIFI_CLK_WIFI_EN_M) });
}

fn wifi_clock_disable() {
    // `periph_ll_wifi_module_disable_clk_set_rst`
    SYSCON::regs()
        .wifi_clk_en()
        .modify(|r, w| unsafe { w.bits(r.bits() & !DPORT_WIFI_CLK_WIFI_EN_M) });
}

fn reset_mac() {
    const SYSTEM_MAC_RST: u32 = 1 << 2;
    SYSCON::regs()
        .wifi_rst_en()
        .modify(|r, w| unsafe { w.wifi_rst().bits(r.wifi_rst().bits() | SYSTEM_MAC_RST) });
    SYSCON::regs()
        .wifi_rst_en()
        .modify(|r, w| unsafe { w.wifi_rst().bits(r.wifi_rst().bits() & !SYSTEM_MAC_RST) });
}

fn init_clocks() {
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

    SYSCON::regs()
        .wifi_clk_en()
        .modify(|r, w| unsafe { w.bits(r.bits() & !WIFI_BT_SDIO_CLK | DPORT_WIFI_CLK_WIFI_EN) });
}
