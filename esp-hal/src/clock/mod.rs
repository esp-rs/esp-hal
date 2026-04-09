#![cfg_attr(docsrs, procmacros::doc_replace)]
//! # CPU Clock Control
//!
//! ## Overview
//!
//! Clocks are mainly sourced from oscillator (OSC), RC, and PLL circuits, and
//! then processed by the dividers or selectors, which allows most functional
//! modules to select their working clock according to their power consumption
//! and performance requirements.
//!
//! The clock subsystem  is used to source and distribute system/module clocks
//! from a range of root clocks. The clock tree driver maintains the basic
//! functionality of the system clock and the intricate relationship among
//! module clocks.
//!
//! ## Configuration
//!
//! During HAL initialization, specify a CPU clock speed to configure the
//! desired clock frequencies.
//!
//! The `CPU clock` is responsible for defining the speed at which the central
//! processing unit (CPU) operates. This driver provides predefined options for
//! different CPU clock speeds, such as
#![cfg_attr(not(esp32h2), doc = "* 80MHz")]
#![cfg_attr(esp32h2, doc = "* 96MHz")]
#![cfg_attr(esp32c2, doc = "* 120MHz")]
#![cfg_attr(not(any(esp32c2, esp32h2)), doc = "* 160MHz")]
#![cfg_attr(any(esp32c5, xtensa), doc = "* 240MHz")]
//! ### Frozen Clock Frequencies
//!
//! Once the clock configuration is applied, the clock frequencies become
//! `frozen` and cannot be changed.
//!
//! ## Examples
//!
//! ### Initialize With Different Clock Frequencies
//! ```rust, no_run
//! # {before_snippet}
//! use esp_hal::clock::CpuClock;
//!
//! // Initialize with the highest possible frequency for this chip
//! let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
//! let peripherals = esp_hal::init(config);
//! # {after_snippet}
//! ```
#![cfg_attr(not(feature = "rt"), expect(unused))]

#[cfg(soc_has_clock_node_lp_slow_clk)]
use clocks::LpSlowClkConfig;
#[cfg(all(not(esp32s2), soc_has_clock_node_rtc_slow_clk))]
use clocks::RtcSlowClkConfig;
#[cfg(soc_has_clock_node_timg_function_clock)]
use clocks::TimgFunctionClockConfig;

/// # Low-level clock control
///
/// <section class="warning">
/// This module provides experimental low-level clock control functionality. These functions
/// can render your device temporarily unusable. Use with caution.
/// </section>
#[doc = ""]
#[instability::unstable]
pub mod ll {
    #[instability::unstable]
    pub use crate::soc::clocks::*;
}

#[cfg(timergroup_rc_fast_calibration_divider)]
use crate::efuse::ChipRevision;
#[instability::unstable]
pub use crate::soc::clocks::ClockConfig;
pub use crate::soc::clocks::CpuClock;
use crate::{ESP_HAL_LOCK, soc::clocks, time::Rate};
#[cfg(soc_has_clock_node_timg_calibration_clock)]
use crate::{peripherals::TIMG0, soc::clocks::ClockTree};

impl CpuClock {
    #[procmacros::doc_replace]
    /// Use the highest possible frequency for a particular chip.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::clock::CpuClock;
    /// let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    /// let peripherals = esp_hal::init(config);
    /// # {after_snippet}
    /// ```
    pub const fn max() -> Self {
        cfg_if::cfg_if! {
            if #[cfg(esp32c2)] {
                Self::_120MHz
            } else if #[cfg(any(esp32c3, esp32c6, esp32c61))] {
                Self::_160MHz
            } else if #[cfg(esp32h2)] {
                Self::_96MHz
            } else {
                Self::_240MHz
            }
        }
    }
}

/// RTC Clocks.
#[instability::unstable]
pub struct RtcClock;

#[cfg(soc_has_clock_node_timg_calibration_clock)]
use crate::soc::clocks::TimgCalibrationClockConfig;

/// RTC Watchdog Timer driver.
impl RtcClock {
    const CAL_FRACT: u32 = 19;

    /// Get the nominal value of the RTC_SLOW_CLK source.
    #[instability::unstable]
    #[cfg(any(soc_has_clock_node_lp_slow_clk, soc_has_clock_node_rtc_slow_clk))]
    pub fn slow_freq() -> Rate {
        cfg_if::cfg_if! {
            if #[cfg(soc_has_clock_node_rtc_slow_clk)] {
                let getter = clocks::rtc_slow_clk_frequency;
            } else {
                let getter = clocks::lp_slow_clk_frequency;
            }
        }
        Rate::from_hz(ClockTree::with(getter))
    }

    /// Measure the frequency of one of the TIMG0 calibration clocks,
    /// using XTAL_CLK as the reference clock.
    ///
    /// This function will time out and return 0 if the time for the given
    /// number of cycles to be counted exceeds the expected time twice. This
    /// may happen if 32k XTAL is being calibrated, but the oscillator has
    /// not started up (due to incorrect loading capacitance, board design
    /// issue, or lack of 32 XTAL on board).
    #[cfg(soc_has_clock_node_timg_calibration_clock)]
    pub(crate) fn calibrate(cal_clk: TimgCalibrationClockConfig, slowclk_cycles: u32) -> u32 {
        ClockTree::with(|clocks| {
            let xtal_freq = Rate::from_hz(clocks::xtal_clk_frequency(clocks));

            let (xtal_cycles, _) = Clocks::measure_rtc_clock(
                clocks,
                cal_clk,
                #[cfg(soc_has_clock_node_timg_function_clock)]
                TimgFunctionClockConfig::XtalClk,
                slowclk_cycles,
            );

            if xtal_cycles == 0 {
                warn!("{:?} calibration failed", cal_clk);
            } else {
                debug!("Counted {} XTAL cycles", xtal_cycles);

                debug!(
                    "{:?} frequency: {}",
                    cal_clk,
                    (xtal_freq / xtal_cycles) * slowclk_cycles
                );
            }

            let divider = xtal_freq.as_mhz() as u64 * slowclk_cycles as u64;

            let period_64 =
                (((xtal_cycles as u64) << RtcClock::CAL_FRACT) + divider / 2u64 - 1u64) / divider;

            (period_64 & u32::MAX as u64) as u32
        })
    }
}

/// Clock frequencies.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
#[doc(hidden)]
#[instability::unstable]
pub struct Clocks {
    /// CPU clock frequency
    #[cfg(soc_has_clock_node_cpu_clk)]
    pub cpu_clock: Rate,

    /// APB clock frequency
    #[cfg(soc_has_clock_node_apb_clk)]
    pub apb_clock: Rate,

    /// XTAL clock frequency
    #[cfg(soc_has_clock_node_xtal_clk)]
    pub xtal_clock: Rate,
}

static mut ACTIVE_CLOCKS: Option<Clocks> = None;

impl Clocks {
    pub(crate) fn init(cpu_clock_config: ClockConfig) {
        ESP_HAL_LOCK.lock(|| {
            crate::rtc_cntl::rtc::init();

            let config = Self::configure(cpu_clock_config);
            unsafe { ACTIVE_CLOCKS = Some(config) };
        });

        Clocks::calibrate_rtc_slow_clock();
    }

    fn try_get<'a>() -> Option<&'a Clocks> {
        unsafe {
            // Safety: ACTIVE_CLOCKS is only set in `init` and never modified after that.
            let clocks = &*core::ptr::addr_of!(ACTIVE_CLOCKS);
            clocks.as_ref()
        }
    }

    /// Get the active clock configuration.
    pub fn get<'a>() -> &'a Clocks {
        unwrap!(Self::try_get())
    }
}

impl Clocks {
    /// Configure the CPU clock speed.
    pub(crate) fn configure(clock_config: ClockConfig) -> Self {
        use crate::soc::clocks::ClockTree;

        clock_config.configure();

        ClockTree::with(|clocks| {
            // FIXME: MCPWM clock configuration needs to know about the active clock source
            // frequency. In the future, we should turn the MCPWM config structs into
            // plain old data structures and remove this pre-configuration, otherwise we will not be
            // able to select a different clock source.
            #[cfg(soc_has_clock_node_mcpwm_function_clock)]
            {
                clocks::McpwmInstance::Mcpwm0.configure_function_clock(clocks, Default::default());
                #[cfg(soc_has_mcpwm1)]
                clocks::McpwmInstance::Mcpwm1.configure_function_clock(clocks, Default::default());
            }
            // Until we have every clock consumer modelled, we should manually keep clocks alive
            #[cfg(soc_has_clock_node_rc_fast_clk)]
            clocks::request_rc_fast_clk(clocks);
            #[cfg(soc_has_clock_node_rc_slow_clk)]
            clocks::request_rc_slow_clk(clocks);
            #[cfg(soc_has_clock_node_pll_clk)]
            clocks::request_pll_clk(clocks);
            #[cfg(soc_has_clock_node_pll_f96m_clk)]
            clocks::request_pll_f96m_clk(clocks);
            #[cfg(soc_has_clock_node_pll_f240m)]
            clocks::request_pll_f240m(clocks);
            #[cfg(soc_has_clock_node_rc_fast_div_clk)]
            clocks::request_rc_fast_div_clk(clocks);

            // TODO: this struct can be removed once everything uses the new internal clock tree
            // code
            Self {
                #[cfg(soc_has_clock_node_cpu_clk)]
                cpu_clock: Rate::from_hz(clocks::cpu_clk_frequency(clocks)),
                #[cfg(soc_has_clock_node_apb_clk)]
                apb_clock: Rate::from_hz(clocks::apb_clk_frequency(clocks)),
                #[cfg(soc_has_clock_node_xtal_clk)]
                xtal_clock: Rate::from_hz(clocks::xtal_clk_frequency(clocks)),
            }
        })
    }

    /// Uses a TIMG0 feature to count clock cycles of a high-frequency clock, for a period of time
    /// that is measured by a low-frequency clock. This function can be used to calibrate two
    /// clocks to each other, e.g. to determine a rough value of the XTAL clock, or to determine
    /// the current frequency of a low-precision RC oscillator.
    #[cfg(soc_has_clock_node_timg_calibration_clock)]
    pub(crate) fn measure_rtc_clock(
        clocks: &mut ClockTree,
        rtc_clock: TimgCalibrationClockConfig,
        // TODO: verify function clock is used, C6 TRM suggests fixed XTAL_CLK
        #[cfg(soc_has_clock_node_timg_function_clock)] function_clock: TimgFunctionClockConfig,
        slow_cycles: u32,
    ) -> (u32, Rate) {
        // There may already be another calibration process running when we call this function,
        // so we should wait until the previous process is finished.
        #[cfg(not(esp32))]
        if TIMG0::regs()
            .rtccalicfg()
            .read()
            .rtc_cali_start_cycling()
            .bit()
        {
            // Set a small timeout threshold to speed up timeout occurrence.
            // The internal circuit will be reset when the timeout occurs and will not affect
            // the next calibration.
            TIMG0::regs()
                .rtccalicfg2()
                .modify(|_, w| unsafe { w.rtc_cali_timeout_thres().bits(1) });

            loop {
                if TIMG0::regs()
                    .rtccalicfg()
                    .read()
                    .rtc_cali_rdy()
                    .bit_is_set()
                    || TIMG0::regs()
                        .rtccalicfg2()
                        .read()
                        .rtc_cali_timeout()
                        .bit_is_set()
                {
                    break;
                }
            }
        }
        TIMG0::regs()
            .rtccalicfg()
            .modify(|_, w| w.rtc_cali_start_cycling().clear_bit());

        use esp_rom_sys::rom::ets_delay_us;

        #[cfg(timergroup_rc_fast_calibration_divider)]
        let calibration_divider = if rtc_clock == TimgCalibrationClockConfig::RcFastDivClk
            && crate::soc::chip_revision_above(ChipRevision::from_combined(property!(
                "timergroup.rc_fast_calibration_divider_min_rev"
            ))) {
            property!("timergroup.rc_fast_calibration_divider")
        } else {
            1
        };
        #[cfg(not(timergroup_rc_fast_calibration_divider))]
        let calibration_divider = 1;

        // On some revisions calibration uses a divided RC_FAST tick.
        let calibration_cycles = (slow_cycles / calibration_divider).max(1);

        // By default the TIMG0 bus clock is running. Do not create a peripheral guard as dropping
        // it would reset the timer, and it would enable its WDT.

        // Make sure the process doesn't time out due to some spooky configuration.
        #[cfg(not(esp32))]
        TIMG0::regs().rtccalicfg2().reset();

        TIMG0::regs()
            .rtccalicfg()
            .modify(|_, w| w.rtc_cali_start().clear_bit());

        // Make sure we measure the crystal.
        cfg_if::cfg_if! {
            if #[cfg(soc_has_clock_node_timg_function_clock)] {
                let current_function_clock = clocks::TimgInstance::Timg0.function_clock_config(clocks);
                clocks::TimgInstance::Timg0.configure_function_clock(clocks, function_clock);
                clocks::TimgInstance::Timg0.request_function_clock(clocks);
            }
        }

        let current_calib_clock = clocks::timg_calibration_clock_config(clocks);
        clocks::configure_timg_calibration_clock(clocks, rtc_clock);
        clocks::request_timg_calibration_clock(clocks);

        let calibration_clock_frequency = clocks::timg_calibration_clock_frequency(clocks);

        let effective_calibration_clock_frequency =
            calibration_clock_frequency / calibration_divider;

        // Set up timeout based on the calibration clock frequency. This is counted in XTAL_CLK
        // cycles.
        #[cfg(not(esp32))]
        {
            let function_clk_freq =
                clocks::TimgInstance::Timg0.function_clock_frequency(clocks) as u64;
            let expected_function_clock_cycles = (function_clk_freq * slow_cycles as u64
                / effective_calibration_clock_frequency as u64)
                as u32;

            TIMG0::regs().rtccalicfg2().modify(|_, w| unsafe {
                let writer = w.rtc_cali_timeout_thres();
                let mask = (1 << writer.width()) - 1;
                writer.bits((expected_function_clock_cycles * 2).min(mask));
                w
            });
        }

        TIMG0::regs().rtccalicfg().modify(|_, w| unsafe {
            w.rtc_cali_max().bits(calibration_cycles as u16);
            w.rtc_cali_start_cycling().clear_bit();
            w.rtc_cali_start().set_bit()
        });

        // Delay, otherwise the CPU may read back the previous state of the completion flag and skip
        // waiting.
        let us_time_estimate = slow_cycles * 1_000_000 / effective_calibration_clock_frequency;
        ets_delay_us(us_time_estimate);

        #[cfg(esp32)]
        let mut timeout_us = us_time_estimate;

        // Wait for the calibration to finish
        let cali_value = loop {
            if TIMG0::regs()
                .rtccalicfg()
                .read()
                .rtc_cali_rdy()
                .bit_is_set()
            {
                break TIMG0::regs().rtccalicfg1().read().rtc_cali_value().bits();
            }

            #[cfg(not(esp32))]
            if TIMG0::regs()
                .rtccalicfg2()
                .read()
                .rtc_cali_timeout()
                .bit_is_set()
            {
                // Timed out waiting for calibration
                break 0;
            }

            #[cfg(esp32)]
            if timeout_us > 0 {
                timeout_us -= 1;
                ets_delay_us(1);
            } else {
                // Timed out waiting for calibration
                break 0;
            }
        };

        TIMG0::regs()
            .rtccalicfg()
            .modify(|_, w| w.rtc_cali_start().clear_bit());

        if let Some(calib_clock) = current_calib_clock
            && calib_clock != rtc_clock
        {
            clocks::configure_timg_calibration_clock(clocks, calib_clock);
        }
        clocks::release_timg_calibration_clock(clocks);

        #[cfg(soc_has_clock_node_timg_function_clock)]
        {
            if let Some(func_clock) = current_function_clock
                && func_clock != function_clock
            {
                clocks::TimgInstance::Timg0.configure_function_clock(clocks, func_clock);
            }
            clocks::TimgInstance::Timg0.release_function_clock(clocks);
        }

        (cali_value, Rate::from_hz(calibration_clock_frequency))
    }

    #[cfg(not(soc_has_clock_node_timg_calibration_clock))]
    pub(crate) fn calibrate_rtc_slow_clock() {
        // Do nothing until TIMG_CALIBRATION_CLOCK is added to device metadata.
    }

    #[cfg(soc_has_clock_node_timg_calibration_clock)]
    pub(crate) fn calibrate_rtc_slow_clock() {
        // Unfortunate device specific mapping.
        // TODO: fix it by generating cfgs for each mux input?
        cfg_if::cfg_if! {
            if #[cfg(esp32s2)] {
                // Can directly measure output of the RTC_SLOW mux
                let slow_clk = TimgCalibrationClockConfig::RtcClk;
            } else if #[cfg(soc_has_clock_node_rtc_slow_clk)] {
                let slow_clk = match unwrap!(ClockTree::with(clocks::rtc_slow_clk_config)) {
                    RtcSlowClkConfig::RcFast => TimgCalibrationClockConfig::RcFastDivClk,
                    RtcSlowClkConfig::RcSlow => TimgCalibrationClockConfig::RcSlowClk,
                    #[cfg(not(esp32c2))]
                    RtcSlowClkConfig::Xtal32k => TimgCalibrationClockConfig::Xtal32kClk,
                    #[cfg(esp32c2)]
                    RtcSlowClkConfig::OscSlow => TimgCalibrationClockConfig::Osc32kClk,
                };
            } else if #[cfg(soc_has_clock_node_lp_slow_clk)]{
                let slow_clk = match unwrap!(ClockTree::with(clocks::lp_slow_clk_config)) {
                    LpSlowClkConfig::OscSlow => TimgCalibrationClockConfig::Xtal32kClk, //?
                    LpSlowClkConfig::Xtal32k => TimgCalibrationClockConfig::Xtal32kClk,
                    LpSlowClkConfig::RcSlow => TimgCalibrationClockConfig::RcSlowClk,
                };
            }
        }

        let cal_val = RtcClock::calibrate(slow_clk, 1024);

        cfg_if::cfg_if! {
            if #[cfg(soc_has_lp_aon)] {
                use crate::peripherals::LP_AON;
            } else {
                use crate::peripherals::LPWR as LP_AON;
            }
        }

        LP_AON::regs()
            .store1()
            .write(|w| unsafe { w.bits(cal_val) });
    }
}

/// The CPU clock frequency.
pub fn cpu_clock() -> Rate {
    Clocks::get().cpu_clock
}

/// The XTAL clock frequency.
pub fn xtal_clock() -> Rate {
    Clocks::get().xtal_clock
}

/// Read the calibrated RTC slow clock period from the STORE1 register.
///
/// The period is in unit of microseconds, represented as a fixed-point number
/// with `RtcClock::CAL_FRACT` fractional bits.
///
/// Written by [`Clocks::calibrate_rtc_slow_clock`] during clock
/// initialization.
fn rtc_slow_cal_period() -> u64 {
    cfg_if::cfg_if! {
        if #[cfg(soc_has_lp_aon)] {
            use crate::peripherals::LP_AON;
        } else {
            use crate::peripherals::LPWR as LP_AON;
        }
    }

    LP_AON::regs().store1().read().bits() as u64
}

/// Convert RTC slow clock ticks to microseconds using the calibrated period.
#[cfg(lp_timer_driver_supported)]
pub(crate) fn rtc_ticks_to_us(ticks: u64) -> u64 {
    let period = rtc_slow_cal_period();

    // The LP timer is a 48-bit counter running from RTC_SLOW_CLK.
    // `period` is a fixed point number with 19 fractional bits, it may be a 24-bit value if
    // RTC_SLOW_CLK is as low as 32768 Hz. Just multiplying the two may cause an overflow on 64
    // bits. We use a calculation that prevents overflow unconditionally because the counter reaches
    // 2^19 pretty quickly. We could use a larger mask (up to 40 bits), but that would make the
    // calculation slower and more complex for perhaps too little benefit.

    const MASK: u64 = (1 << RtcClock::CAL_FRACT) - 1;
    let upper = (ticks & !MASK) >> RtcClock::CAL_FRACT;
    let lower = ticks & MASK;

    upper * period + ((lower * period) >> RtcClock::CAL_FRACT)
}

/// Convert microseconds to RTC slow clock ticks using the calibrated period.
pub(crate) fn us_to_rtc_ticks(time_in_us: u64) -> u64 {
    let period = rtc_slow_cal_period();

    if time_in_us > (u64::MAX >> RtcClock::CAL_FRACT) {
        ((time_in_us / period) << RtcClock::CAL_FRACT)
            + ((time_in_us % period) << RtcClock::CAL_FRACT) / period
    } else {
        (time_in_us << RtcClock::CAL_FRACT) / period
    }
}
