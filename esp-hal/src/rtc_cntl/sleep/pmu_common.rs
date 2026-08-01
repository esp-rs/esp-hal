use crate::{
    clock::{
        RtcClock,
        calibrate_rtc_fast_clock,
        calibrate_rtc_slow_clock,
        rtc_fast_cal_period,
        rtc_slow_cal_period,
    },
    rtc_cntl::sleep::PowerDownFlags,
    soc::clocks,
};

/// The clock period corresponding to `freq_hz`, in microseconds, as a fixed-point number with
/// [`RtcClock::CAL_FRACT`] fractional bits.
///
/// This is the same representation `RtcClock::calibrate` returns, and is used as a fallback when
/// calibration is not available.
fn nominal_cal_period(freq_hz: u32) -> u32 {
    // The point of this function is to avoid a division by zero, so it must not introduce one
    // itself if the clock frequency is not known either.
    let period = (1_000_000u64 << RtcClock::CAL_FRACT) / freq_hz.max(1) as u64;
    period.min(u32::MAX as u64) as u32
}

#[derive(Clone, Copy)]
pub(super) struct SleepTimeConfig {
    pub sleep_time_adjustment: u32,
    pub slowclk_period: u32,
    pub fastclk_period: u32,
}

impl SleepTimeConfig {
    fn rtc_clk_cal_fast(deep: bool) -> u32 {
        if deep {
            calibrate_rtc_fast_clock();
        }

        match rtc_fast_cal_period() {
            // Calibration timed out, or has not run yet. Fall back to the nominal RC_FAST
            // frequency: a slightly inaccurate sleep duration is better than a division by zero
            // in `us_to_fastclk`.
            0 => {
                warn!("RC_FAST calibration unavailable, assuming the nominal frequency");
                nominal_cal_period(clocks::rc_fast_clk_frequency())
            }
            period => period,
        }
    }

    fn rtc_clk_cal_slow(deep: bool) -> u32 {
        if deep {
            calibrate_rtc_slow_clock();
        }

        match rtc_slow_cal_period() {
            // See `rtc_clk_cal_fast`.
            0 => {
                warn!("RTC_SLOW calibration unavailable, assuming the nominal frequency");
                nominal_cal_period(RtcClock::slow_freq().as_hz())
            }
            period => period,
        }
    }

    pub fn new(deep: bool) -> Self {
        Self {
            sleep_time_adjustment: 0,
            slowclk_period: Self::rtc_clk_cal_slow(deep),
            fastclk_period: Self::rtc_clk_cal_fast(deep),
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
        (us << RtcClock::CAL_FRACT) / self.fastclk_period
    }
}
