use crate::{clock::CpuClock, peripherals::SYSCON};

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

pub(super) fn enable_phy(enable: bool) {
    // `periph_ll_wifi_bt_module_enable_clk_clear_rst`
    // `periph_ll_wifi_bt_module_disable_clk_set_rst`
    SYSCON::regs().wifi_clk_en().modify(|r, w| unsafe {
        if enable {
            w.bits(r.bits() | DPORT_WIFI_CLK_WIFI_BT_COMMON_M)
        } else {
            w.bits(r.bits() & !DPORT_WIFI_CLK_WIFI_BT_COMMON_M)
        }
    });
}

pub(super) fn enable_wifi(enable: bool) {
    // `periph_ll_wifi_module_enable_clk_clear_rst`
    // `periph_ll_wifi_module_disable_clk_set_rst`
    SYSCON::regs().wifi_clk_en().modify(|r, w| unsafe {
        if enable {
            w.bits(r.bits() | DPORT_WIFI_CLK_WIFI_EN_M)
        } else {
            w.bits(r.bits() & !DPORT_WIFI_CLK_WIFI_EN_M)
        }
    });
}

pub(super) fn reset_mac() {
    const SYSTEM_MAC_RST: u32 = 1 << 2;
    SYSCON::regs()
        .wifi_rst_en()
        .modify(|r, w| unsafe { w.wifi_rst().bits(r.wifi_rst().bits() | SYSTEM_MAC_RST) });
    SYSCON::regs()
        .wifi_rst_en()
        .modify(|r, w| unsafe { w.wifi_rst().bits(r.wifi_rst().bits() & !SYSTEM_MAC_RST) });
}

pub(super) fn init_clocks() {
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

pub(super) fn ble_rtc_clk_init() {
    // nothing for this target
}

pub(super) fn reset_rpa() {
    // nothing for this target
}
