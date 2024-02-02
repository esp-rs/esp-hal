use crate::clock::CpuClock;

const MHZ: u32 = 1000000;
const UINT16_MAX: u32 = 0xffff;

const RTC_CNTL_DBIAS_1V25: u32 = 7;

// when not running with 80MHz Flash frequency we could use RTC_CNTL_DBIAS_1V10
// for DIG_DBIAS_80M_160M - but RTC_CNTL_DBIAS_1V25 shouldn't hurt
const DIG_DBIAS_80M_160M: u32 = RTC_CNTL_DBIAS_1V25;
const DIG_DBIAS_240M: u32 = RTC_CNTL_DBIAS_1V25;

pub(crate) fn set_cpu_clock(cpu_clock_speed: CpuClock) {
    let system_control = unsafe { &*crate::peripherals::SYSTEM::PTR };
    let rtc_cntl = unsafe { &*crate::peripherals::RTC_CNTL::ptr() };

    unsafe {
        system_control
            .sysclk_conf()
            .modify(|_, w| w.soc_clk_sel().bits(1));
        system_control.cpu_per_conf().modify(|_, w| {
            w.pll_freq_sel()
                .set_bit()
                .cpuperiod_sel()
                .bits(match cpu_clock_speed {
                    CpuClock::Clock80MHz => 0,
                    CpuClock::Clock160MHz => 1,
                    CpuClock::Clock240MHz => 2,
                })
        });

        rtc_cntl.reg().modify(|_, w| {
            w.dig_reg_dbias_wak().bits(match cpu_clock_speed {
                CpuClock::Clock80MHz => DIG_DBIAS_80M_160M,
                CpuClock::Clock160MHz => DIG_DBIAS_80M_160M,
                CpuClock::Clock240MHz => DIG_DBIAS_240M,
            } as u8)
        });

        let value = (((80 * MHZ) >> 12) & UINT16_MAX) | ((((80 * MHZ) >> 12) & UINT16_MAX) << 16);
        rtc_cntl.store5().modify(|_, w| w.scratch5().bits(value));
    }
}
