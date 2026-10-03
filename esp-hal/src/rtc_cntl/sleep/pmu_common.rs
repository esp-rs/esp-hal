use crate::{
    clock::{RtcClock, calibrate_rtc_fast_clock, calibrate_rtc_slow_clock, rtc_slow_cal_period},
    peripherals::PMU,
    rtc_cntl::sleep::PowerDownFlags,
    soc::clocks,
};

#[derive(Clone, Copy)]
pub(super) struct SleepTimeConfig {
    pub sleep_time_adjustment: u32,
    pub slowclk_period: u32,
    pub fastclk_frequency: u32,
}

impl SleepTimeConfig {
    fn rtc_clk_cal_fast(deep: bool) -> u32 {
        if deep {
            calibrate_rtc_fast_clock();
        }

        clocks::rc_fast_clk_frequency()
    }

    fn rtc_clk_cal_slow(deep: bool) -> u32 {
        if deep {
            calibrate_rtc_slow_clock();
        }

        rtc_slow_cal_period()
    }

    pub fn new(deep: bool) -> Self {
        Self {
            sleep_time_adjustment: 0,
            slowclk_period: Self::rtc_clk_cal_slow(deep),
            fastclk_frequency: Self::rtc_clk_cal_fast(deep),
        }
    }

    pub fn light_sleep(pd_flags: PowerDownFlags) -> Self {
        let mut this = Self::new(false);

        let sw = Self::LIGHT_SLEEP_TIME_OVERHEAD_US; // TODO
        let hw = this.pmu_sleep_calculate_hw_wait_time(pd_flags);

        this.sleep_time_adjustment = sw + hw;

        this
    }

    pub fn deep_sleep() -> Self {
        let mut this = Self::new(true);

        this.sleep_time_adjustment = 250 + 100 * 240 / Self::CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;

        this
    }

    pub fn us_to_slowclk(&self, us: u32) -> u32 {
        (us << RtcClock::CAL_FRACT) / self.slowclk_period
    }

    pub fn slowclk_to_us(&self, rtc_cycles: u32) -> u32 {
        (rtc_cycles * self.slowclk_period) >> RtcClock::CAL_FRACT
    }

    pub fn us_to_fastclk(&self, us: u32) -> u32 {
        (us as u64 * self.fastclk_frequency as u64 / 1_000_000) as u32
    }
}

/// Writes the dirty lines of the level-one data cache back to memory.
///
/// The cache belongs to the CPU power domain, so a sleep that powers the domain down loses every
/// dirty line. The retention frames are among them, because the save writes them through the cache.
/// esp-idf writes the cache back in `pmu_sleep_start` (`esp32p4/pmu_sleep.c:410`), and it uses the
/// registers and not the ROM helper, because the return of a call dirties the cache again.
#[crate::ram]
#[cfg(soc_internal_memory_cached)]
fn writeback_data_cache() {
    // `CACHE_MAP_L1_DCACHE` of `esp32p4/rom/cache.h`.
    const L1_DATA_CACHE: u8 = 1 << 4;

    let cache = crate::peripherals::CACHE::regs();
    cache.sync_addr().write(|w| unsafe { w.bits(0) });
    cache.sync_size().write(|w| unsafe { w.bits(0) });
    cache
        .sync_map()
        .write(|w| unsafe { w.sync_map().bits(L1_DATA_CACHE) });
    cache.sync_ctrl().modify(|_, w| w.writeback_ena().set_bit());
    while !cache.sync_ctrl().read().sync_done().bit_is_set() {}
}

/// Requests the sleep and returns whether the hardware rejected the request.
///
/// The software retention path calls this through a function pointer after the critical frame is
/// saved.
#[inline(always)]
pub(crate) fn request_sleep() -> bool {
    #[cfg(soc_internal_memory_cached)]
    writeback_data_cache();

    PMU::regs()
        .slp_wakeup_cntl0()
        .write(|w| w.sleep_req().bit(true));

    super::wait_for_sleep_result()
}
