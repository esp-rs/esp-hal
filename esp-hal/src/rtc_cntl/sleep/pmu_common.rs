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

    #[cfg(sleep_deep_sleep)]
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
#[procmacros::ram(unstable(checked))]
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
#[cfg_attr(cpu_retention = "software", procmacros::ram(unstable(checked)))]
#[cfg_attr(not(cpu_retention = "software"), inline(always))]
pub(crate) fn request_sleep() -> bool {
    #[cfg(soc_internal_memory_cached)]
    writeback_data_cache();

    PMU::regs()
        .slp_wakeup_cntl0()
        .write(|w| w.sleep_req().bit(true));

    super::wait_for_sleep_result()
}

#[cfg(sleep_usb_serial_jtag_in_light_sleep)]
pub(super) mod usb_serial_jtag {
    use core::sync::atomic::{AtomicBool, Ordering};

    use crate::peripherals::USB_DEVICE;

    /// Whether the current light sleep keeps USB Serial/JTAG running.
    static KEPT_IN_SLEEP: AtomicBool = AtomicBool::new(false);

    /// Returns whether USB Serial/JTAG is clocked and out of reset.
    pub(in crate::rtc_cntl::sleep) fn module_is_enabled() -> bool {
        cfg_select! {
            esp32p4 => {
                crate::peripherals::HP_SYS_CLKRST::regs()
                    .soc_clk_ctrl2()
                    .read()
                    .usb_device_apb_clk_en()
                    .bit_is_set()
                    && crate::peripherals::LP_AON_CLKRST::regs()
                        .lp_aonclkrst_hp_usb_clkrst_ctrl1()
                        .read()
                        .lp_aonclkrst_rst_en_usb_device()
                        .bit_is_clear()
            }
            _ => {
                let conf = crate::peripherals::PCR::regs().usb_device_conf().read();
                conf.usb_device_clk_en().bit_is_set() && conf.usb_device_rst_en().bit_is_clear()
            }
        }
    }

    /// Returns whether the USB PHY gets its clock.
    fn phy_clock_is_enabled() -> bool {
        cfg_select! {
            esp32p4 => crate::peripherals::LP_AON_CLKRST::regs()
                .lp_aonclkrst_hp_usb_clkrst_ctrl0()
                .read()
                .lp_aonclkrst_usb_device_48m_clk_en()
                .bit_is_set(),
            _ => true,
        }
    }

    /// Returns whether a USB host sent a start-of-frame packet since the previous call.
    ///
    /// A connected host that has not suspended the bus sends one every millisecond. The caller
    /// makes sure that USB Serial/JTAG is clocked and out of reset.
    fn host_sent_sof() -> bool {
        let regs = USB_DEVICE::regs();
        let sof = regs.int_raw().read().sof().bit_is_set();
        regs.int_clr().write(|w| w.sof().clear_bit_by_one());
        sof
    }

    /// Decides whether the coming sleep keeps USB Serial/JTAG running, and returns the decision.
    ///
    /// A light sleep keeps it running while it is in use and a USB host is connected.
    pub(in crate::rtc_cntl::sleep) fn decide_for_sleep(deep: bool) -> bool {
        let keep = !deep
            && module_is_enabled()
            && phy_clock_is_enabled()
            && USB_DEVICE::regs()
                .conf0()
                .read()
                .usb_pad_enable()
                .bit_is_set()
            && host_sent_sof();
        KEPT_IN_SLEEP.store(keep, Ordering::Relaxed);
        keep
    }

    /// Whether the current sleep keeps USB Serial/JTAG running.
    pub(in crate::rtc_cntl::sleep) fn kept_in_sleep() -> bool {
        KEPT_IN_SLEEP.load(Ordering::Relaxed)
    }
}

/// Waits until the eFuse controller has reloaded the eFuse memory after the sleep request.
///
/// An eFuse read before this point can return stale data.
#[cfg(not(esp32h2))]
#[inline(always)]
pub(crate) fn wait_for_efuse_idle() {
    // `EFUSE_CONTROLLER_STATE_IDLE`
    const IDLE: u8 = 1;

    while crate::peripherals::EFUSE::regs()
        .status()
        .read()
        .state()
        .bits()
        != IDLE
    {}
}
