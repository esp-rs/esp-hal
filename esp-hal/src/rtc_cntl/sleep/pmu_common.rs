use crate::{
    clock::{RtcClock, calibrate_rtc_fast_clock, rtc_slow_cal_period},
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

    pub fn new(deep: bool) -> Self {
        Self {
            sleep_time_adjustment: 0,
            slowclk_period: rtc_slow_cal_period(),
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

/// Selects the HP_SLEEP system clock and clock gates.
///
/// A sleep that keeps USB Serial/JTAG running opens the gates of USB Serial/JTAG, the HP bus and
/// IOMUX on top of `icg_func`, and runs the system clock from the PLL, which `pll_sysclk` selects.
///
/// See <https://github.com/espressif/esp-idf/blob/4dbfecac7e/components/soc/esp32c6/include/soc/pmu_icg_mapping.h>.
#[cfg(any(esp32c5, esp32c6, esp32c61, esp32h2))]
#[inline(always)]
pub(super) fn configure_hp_sleep_clocks(icg_func: u32, keep_usb_serial_jtag: bool, pll_sysclk: u8) {
    const HP_SYSCLK_XTAL: u8 = 0;
    const PMU_ICG_FUNC_ENA_USB_DEVICE: u32 = 1 << 6;
    const PMU_ICG_FUNC_ENA_HPBUS: u32 = 1 << 15;
    const PMU_ICG_FUNC_ENA_IOMUX: u32 = 1 << 28;
    const PMU_ICG_APB_ENA_USB_DEVICE: u32 = 1 << 17;
    const PMU_ICG_APB_ENA_IOMUX: u32 = 1 << 26;

    let (icg_func, icg_apb, sysclk) = if keep_usb_serial_jtag {
        (
            icg_func
                | PMU_ICG_FUNC_ENA_USB_DEVICE
                | PMU_ICG_FUNC_ENA_HPBUS
                | PMU_ICG_FUNC_ENA_IOMUX,
            PMU_ICG_APB_ENA_USB_DEVICE | PMU_ICG_APB_ENA_IOMUX,
            pll_sysclk,
        )
    } else {
        (icg_func, 0, HP_SYSCLK_XTAL)
    };

    let pmu = PMU::regs();
    pmu.hp_sleep_sysclk().modify(|_, w| unsafe {
        w.hp_sleep_icg_sys_clock_en().bit(icg_func != 0);
        w.hp_sleep_dig_sys_clk_sel().bits(sysclk)
    });
    pmu.hp_sleep_icg_hp_func()
        .write(|w| unsafe { w.hp_sleep_dig_icg_func_en().bits(icg_func) });
    pmu.hp_sleep_icg_hp_apb()
        .write(|w| unsafe { w.hp_sleep_dig_icg_apb_en().bits(icg_apb) });
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
/// The software retention path calls this after the critical frame is saved.
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
