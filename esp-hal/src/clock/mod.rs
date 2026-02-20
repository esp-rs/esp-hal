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
#[cfg(soc_has_clock_node_timg0_function_clock)]
use clocks::Timg0FunctionClockConfig;
use esp_rom_sys::rom::ets_delay_us;

use crate::{
    ESP_HAL_LOCK,
    peripherals::TIMG0,
    soc::clocks::{self, ClockTree},
    time::Rate,
};

#[cfg_attr(esp32, path = "clocks_ll/esp32.rs")]
#[cfg_attr(esp32c2, path = "clocks_ll/esp32c2.rs")]
#[cfg_attr(esp32c3, path = "clocks_ll/esp32c3.rs")]
#[cfg_attr(esp32c5, path = "clocks_ll/esp32c5.rs")]
#[cfg_attr(esp32c6, path = "clocks_ll/esp32c6.rs")]
#[cfg_attr(esp32h2, path = "clocks_ll/esp32h2.rs")]
#[cfg_attr(esp32s2, path = "clocks_ll/esp32s2.rs")]
#[cfg_attr(esp32s3, path = "clocks_ll/esp32s3.rs")]
pub(crate) mod clocks_ll;

#[cfg(feature = "unstable")]
#[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
pub use crate::soc::clocks::ClockConfig;
#[cfg(not(feature = "unstable"))]
pub(crate) use crate::soc::clocks::ClockConfig;
pub use crate::soc::clocks::CpuClock;

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
            } else if #[cfg(any(esp32c3, esp32c6))] {
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

cfg_if::cfg_if! {
    if #[cfg(soc_has_clock_node_timg_calibration_clock)] {
        use crate::soc::clocks::TimgCalibrationClockConfig;
    } else if #[cfg(soc_has_clock_node_timg0_calibration_clock)] {
        use crate::soc::clocks::Timg0CalibrationClockConfig as TimgCalibrationClockConfig;
    }
}

/// RTC Watchdog Timer driver.
impl RtcClock {
    const CAL_FRACT: u32 = 19;

    /// Get the RTC_SLOW_CLK source.
    #[cfg_attr(all(not(feature = "unstable"), esp32c5), expect(dead_code))]
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
    pub(crate) fn calibrate(cal_clk: TimgCalibrationClockConfig, slowclk_cycles: u32) -> u32 {
        ClockTree::with(|clocks| {
            let xtal_freq = Rate::from_hz(clocks::xtal_clk_frequency(clocks));

            #[cfg(timergroup_rc_fast_calibration_divider)]
            let slowclk_cycles = if cal_clk == TimgCalibrationClockConfig::RcFastDivClk
                && crate::soc::chip_revision_above(property!(
                    "timergroup.rc_fast_calibration_divider_min_rev"
                )) {
                // Avoid excessive calibration time by dividing the cycles by the divider
                // that exists between FOSC and the calibration circuit.
                slowclk_cycles / property!("timergroup.rc_fast_calibration_divider")
            } else {
                slowclk_cycles
            };

            let (xtal_cycles, _) = Clocks::measure_rtc_clock(
                clocks,
                cal_clk,
                #[cfg(soc_has_clock_node_timg0_function_clock)]
                Timg0FunctionClockConfig::XtalClk,
                slowclk_cycles,
            );

            let divider = xtal_freq.as_mhz() as u64 * slowclk_cycles as u64;

            let period_64 =
                (((xtal_cycles as u64) << RtcClock::CAL_FRACT) + divider / 2u64 - 1u64) / divider;

            (period_64 & u32::MAX as u64) as u32
        })
    }

    /// Calculate the necessary RTC_SLOW_CLK cycles to complete 1 millisecond.
    pub(crate) fn cycles_to_1ms() -> u16 {
        cfg_if::cfg_if! {
            if #[cfg(soc_has_lp_aon)] {
                use crate::peripherals::LP_AON;
            } else {
                use crate::peripherals::LPWR as LP_AON;
            }
        }

        let period_13q19 = LP_AON::regs().store1().read().bits();

        // 100_000_000 is used to get rid of `float` calculations
        let period = (100_000_000 * period_13q19 as u64) / (1 << RtcClock::CAL_FRACT);

        (100_000_000 * 1000 / period) as u16
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
            #[cfg(soc_has_clock_node_mcpwm0_function_clock)]
            clocks::configure_mcpwm0_function_clock(clocks, Default::default());
            #[cfg(soc_has_clock_node_mcpwm1_function_clock)]
            clocks::configure_mcpwm1_function_clock(clocks, Default::default());

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
    pub(crate) fn measure_rtc_clock(
        clocks: &mut ClockTree,
        rtc_clock: TimgCalibrationClockConfig,
        // TODO: verify function clock is used, C6 TRM suggests fixed XTAL_CLK
        #[cfg(soc_has_clock_node_timg0_function_clock)] function_clock: Timg0FunctionClockConfig,
        slow_cycles: u32,
    ) -> (u32, Rate) {
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
            if #[cfg(soc_has_clock_node_timg0_function_clock)] {
                let current_function_clock = clocks::timg0_function_clock_config(clocks);
                clocks::configure_timg0_function_clock(clocks, function_clock);
                clocks::request_timg0_function_clock(clocks);
            }
        }
        cfg_if::cfg_if! {
            if #[cfg(soc_has_clock_node_timg_calibration_clock)] {
                let current_calib_clock = clocks::timg_calibration_clock_config(clocks);
                clocks::configure_timg_calibration_clock(clocks, rtc_clock);
                clocks::request_timg_calibration_clock(clocks);

                let calibration_clock_frequency = clocks::timg_calibration_clock_frequency(clocks);
            } else {
                let current_calib_clock = clocks::timg0_calibration_clock_config(clocks);
                clocks::configure_timg0_calibration_clock(clocks, rtc_clock);
                clocks::request_timg0_calibration_clock(clocks);

                let calibration_clock_frequency = clocks::timg0_calibration_clock_frequency(clocks);
            }
        }

        // Set up timeout based on the calibration clock frequency. This is counted in XTAL_CLK
        // cycles.
        #[cfg(not(esp32))]
        {
            let function_clk_freq = clocks::timg0_function_clock_frequency(clocks) as u64;
            let expected_function_clock_cycles = (function_clk_freq * slow_cycles as u64
                / calibration_clock_frequency as u64)
                as u32;

            TIMG0::regs().rtccalicfg2().modify(|_, w| unsafe {
                let writer = w.rtc_cali_timeout_thres();
                let mask = (1 << writer.width()) - 1;
                writer.bits((expected_function_clock_cycles * 2).min(mask));
                w
            });
        }

        TIMG0::regs().rtccalicfg().modify(|_, w| unsafe {
            w.rtc_cali_max().bits(slow_cycles as u16);
            w.rtc_cali_start_cycling().clear_bit();
            w.rtc_cali_start().set_bit()
        });

        // Delay, otherwise the CPU may read back the previous state of the completion flag and skip
        // waiting.
        let us_time_estimate = slow_cycles * 1_000_000 / calibration_clock_frequency;
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

        // TODO: this would be nicer if we had clock node objects instead of free-standing functions
        cfg_if::cfg_if! {
            if #[cfg(soc_has_clock_node_timg_calibration_clock)] {
                if let Some(calib_clock) = current_calib_clock
                    && calib_clock != rtc_clock
                {
                    clocks::configure_timg_calibration_clock(clocks, calib_clock);
                }
                clocks::release_timg_calibration_clock(clocks);
            } else {
                if let Some(calib_clock) = current_calib_clock
                    && calib_clock != rtc_clock
                {
                    clocks::configure_timg0_calibration_clock(clocks, calib_clock);
                }
                clocks::release_timg0_calibration_clock(clocks);
            }
        }

        #[cfg(soc_has_clock_node_timg0_function_clock)]
        {
            if let Some(func_clock) = current_function_clock
                && func_clock != function_clock
            {
                clocks::configure_timg0_function_clock(clocks, func_clock);
            }
            clocks::release_timg0_function_clock(clocks);
        }

        (cali_value, Rate::from_hz(calibration_clock_frequency))
    }

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
            } else {
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

#[cfg(any(
    soc_has_bt,
    all(feature = "unstable", soc_has_ieee802154),
    soc_has_wifi
))]
mod modem {
    use core::{cell::Cell, marker::PhantomData};

    use esp_sync::RawMutex;

    use super::*;
    #[cfg(soc_has_bt)]
    use crate::peripherals::BT;
    #[cfg(all(feature = "unstable", soc_has_ieee802154))]
    use crate::peripherals::IEEE802154;
    #[cfg(soc_has_wifi)]
    use crate::peripherals::WIFI;
    use crate::private::Sealed;

    /// Tracks the number of references to the PHY clock.
    static PHY_CLOCK_REF_COUNTER: embassy_sync::blocking_mutex::Mutex<RawMutex, Cell<u8>> =
        embassy_sync::blocking_mutex::Mutex::new(Cell::new(0));

    fn increase_phy_clock_ref_count_internal() {
        PHY_CLOCK_REF_COUNTER.lock(|phy_clock_ref_counter| {
            let phy_clock_ref_count = phy_clock_ref_counter.get();

            if phy_clock_ref_count == 0 {
                clocks_ll::enable_phy(true);
            }
            let new_phy_clock_ref_count = unwrap!(
                phy_clock_ref_count.checked_add(1),
                "PHY clock ref count overflowed."
            );

            phy_clock_ref_counter.set(new_phy_clock_ref_count);
        })
    }

    fn decrease_phy_clock_ref_count_internal() {
        PHY_CLOCK_REF_COUNTER.lock(|phy_clock_ref_counter| {
            let new_phy_clock_ref_count = unwrap!(
                phy_clock_ref_counter.get().checked_sub(1),
                "PHY clock ref count underflowed. Either you forgot a PhyClockGuard, or used ModemClockController::decrease_phy_clock_ref_count incorrectly."
            );

            if new_phy_clock_ref_count == 0 {
                clocks_ll::enable_phy(false);
            }

            phy_clock_ref_counter.set(new_phy_clock_ref_count);
        })
    }

    #[inline]
    #[instability::unstable]
    /// Do any common initial initialization needed for the radio clocks
    pub fn init_radio_clocks() {
        clocks_ll::init_clocks();
    }

    #[instability::unstable]
    #[derive(Debug)]
    /// Prevents the PHY clock from being disabled.
    ///
    /// As long as at least one [PhyClockGuard] exists, the PHY clock will remain
    /// active. To release this guard, you can either let it go out of scope or use
    /// [PhyClockGuard::release] to explicitly release it.
    pub struct PhyClockGuard<'d> {
        _phantom: PhantomData<&'d ()>,
    }

    impl PhyClockGuard<'_> {
        #[instability::unstable]
        #[inline]
        /// Release the clock guard.
        ///
        /// The PHY clock will be disabled, if this is the last clock guard.
        pub fn release(self) {}
    }

    impl Drop for PhyClockGuard<'_> {
        fn drop(&mut self) {
            decrease_phy_clock_ref_count_internal();
        }
    }

    #[instability::unstable]
    /// This trait provides common clock functionality for all modem peripherals.
    pub trait ModemClockController<'d>: Sealed + 'd {
        /// Enable the modem clock for this controller.
        fn enable_modem_clock(&mut self, enable: bool);

        /// Enable the PHY clock and acquire a [PhyClockGuard].
        ///
        /// The PHY clock will only be disabled, once all [PhyClockGuard]'s of all
        /// modems were dropped.
        fn enable_phy_clock(&self) -> PhyClockGuard<'d> {
            increase_phy_clock_ref_count_internal();
            PhyClockGuard {
                _phantom: PhantomData,
            }
        }

        /// Decreases the PHY clock reference count for this modem ignoring
        /// currently alive [PhyClockGuard]s.
        ///
        /// # Panics
        /// This function panics if the PHY clock is inactive. If the ref count is
        /// lower than the number of alive [PhyClockGuard]s, dropping a guard can
        /// now panic.
        fn decrease_phy_clock_ref_count(&self) {
            decrease_phy_clock_ref_count_internal();
        }
    }

    #[cfg(soc_has_wifi)]
    #[instability::unstable]
    impl<'d> ModemClockController<'d> for WIFI<'d> {
        fn enable_modem_clock(&mut self, enable: bool) {
            clocks_ll::enable_wifi(enable);
        }
    }

    #[cfg(soc_has_wifi)]
    impl WIFI<'_> {
        #[instability::unstable]
        /// Reset the Wi-Fi MAC.
        pub fn reset_wifi_mac(&mut self) {
            clocks_ll::reset_wifi_mac();
        }
    }

    #[cfg(soc_has_bt)]
    #[instability::unstable]
    impl<'d> ModemClockController<'d> for BT<'d> {
        fn enable_modem_clock(&mut self, enable: bool) {
            clocks_ll::enable_bt(enable);
        }
    }

    #[cfg(soc_has_bt)]
    impl BT<'_> {
        /// Reset the Bluetooth Resolvable Private Address (RPA).
        #[instability::unstable]
        #[inline]
        pub fn reset_rpa(&mut self) {
            clocks_ll::reset_rpa();
        }

        /// Initialize BLE RTC clocks
        #[instability::unstable]
        #[inline]
        pub fn ble_rtc_clk_init(&mut self) {
            clocks_ll::ble_rtc_clk_init();
        }
    }

    #[cfg(all(feature = "unstable", soc_has_ieee802154))]
    #[instability::unstable]
    impl<'d> ModemClockController<'d> for IEEE802154<'d> {
        fn enable_modem_clock(&mut self, enable: bool) {
            clocks_ll::enable_ieee802154(enable);
        }
    }
}

#[cfg(all(
    feature = "unstable",
    any(soc_has_bt, soc_has_ieee802154, soc_has_wifi)
))]
pub use modem::*;
