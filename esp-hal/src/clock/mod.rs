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
#![cfg_attr(xtensa, doc = "* 240MHz")]
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

use core::{cell::Cell, marker::PhantomData};

use esp_rom_sys::rom::ets_delay_us;
#[cfg(any(bt, ieee802154, wifi))]
use esp_sync::RawMutex;

#[cfg(esp32c6)]
use crate::efuse::Efuse;
#[cfg(bt)]
use crate::peripherals::BT;
#[cfg(all(feature = "unstable", ieee802154))]
use crate::peripherals::IEEE802154;
#[cfg(wifi)]
use crate::peripherals::WIFI;
use crate::{
    ESP_HAL_LOCK,
    peripherals::{LPWR, TIMG0},
    private::Sealed,
    rtc_cntl::{Rtc, RtcCalSel},
    time::Rate,
};

cfg_if::cfg_if! {
    if #[cfg(any(esp32c6, esp32h2))] {
        use crate::peripherals::LP_AON;
    } else {
        use crate::peripherals::LPWR as LP_AON;
    }
}

#[cfg_attr(esp32, path = "clocks_ll/esp32.rs")]
#[cfg_attr(esp32c2, path = "clocks_ll/esp32c2.rs")]
#[cfg_attr(esp32c3, path = "clocks_ll/esp32c3.rs")]
#[cfg_attr(esp32c6, path = "clocks_ll/esp32c6.rs")]
#[cfg_attr(esp32h2, path = "clocks_ll/esp32h2.rs")]
#[cfg_attr(esp32s2, path = "clocks_ll/esp32s2.rs")]
#[cfg_attr(esp32s3, path = "clocks_ll/esp32s3.rs")]
pub(crate) mod clocks_ll;

/// Clock properties
#[doc(hidden)]
pub trait Clock {
    /// Frequency of the clock in [Rate].
    fn frequency(&self) -> Rate;

    /// Frequency of the clock in Megahertz
    fn mhz(&self) -> u32 {
        self.frequency().as_mhz()
    }

    /// Frequency of the clock in Hertz
    fn hz(&self) -> u32 {
        self.frequency().as_hz()
    }
}

/// CPU clock speed
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(
    clippy::enum_variant_names,
    reason = "MHz suffix indicates physical unit."
)]
#[non_exhaustive]
pub enum CpuClock {
    /// 80MHz CPU clock
    #[cfg(not(esp32h2))]
    #[default]
    // FIXME: I don't think this is correct in general?
    _80MHz  = 80,

    /// 96MHz CPU clock
    #[cfg(esp32h2)]
    #[default]
    _96MHz  = 96,

    /// 120MHz CPU clock
    #[cfg(esp32c2)]
    _120MHz = 120,

    /// 160MHz CPU clock
    #[cfg(not(any(esp32c2, esp32h2)))]
    _160MHz = 160,

    /// 240MHz CPU clock
    #[cfg(xtensa)]
    _240MHz = 240,
}

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

impl Clock for CpuClock {
    fn frequency(&self) -> Rate {
        Rate::from_mhz(*self as u32)
    }
}

for_each_soc_xtal_options!(
    (all $( ($freq:literal) ),*) => {
        paste::paste! {
            /// XTAL clock speed
            #[instability::unstable]
            #[derive(Debug, Clone, Copy)]
            #[non_exhaustive]
            pub enum XtalClock {
                $(
                    #[doc = concat!(stringify!($freq), "MHz XTAL clock")]
                    [<_ $freq M>],
                )*
            }

            impl XtalClock {
                pub(crate) const fn closest_from_mhz(mhz: u32) -> Self {
                    let options = [
                        $( ($freq, XtalClock::[<_ $freq M>] ), )*
                    ];

                    let mut error = mhz.abs_diff(options[0].0);
                    let mut selected = options[0].1;

                    let mut idx = 1;
                    while idx < options.len() {
                        let e = mhz.abs_diff(options[idx].0);
                        if e < error {
                            selected = options[idx].1;
                            error = e;
                        }
                        idx += 1;
                    }

                    selected
                }
            }

            impl Clock for XtalClock {
                fn frequency(&self) -> Rate {
                    match self {
                        $(
                            XtalClock::[<_ $freq M>] => Rate::from_mhz($freq),
                        )*
                    }
                }
            }
        }
    };
);

#[allow(clippy::enum_variant_names, unused)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) enum PllClock {
    #[cfg(esp32h2)]
    Pll8MHz,
    #[cfg(any(esp32c6, esp32h2))]
    Pll48MHz,
    #[cfg(esp32h2)]
    Pll64MHz,
    #[cfg(esp32c6)]
    Pll80MHz,
    #[cfg(esp32h2)]
    Pll96MHz,
    #[cfg(esp32c6)]
    Pll120MHz,
    #[cfg(esp32c6)]
    Pll160MHz,
    #[cfg(esp32c6)]
    Pll240MHz,
    #[cfg(not(any(esp32c2, esp32c6, esp32h2)))]
    Pll320MHz,
    #[cfg(not(esp32h2))]
    Pll480MHz,
}

impl Clock for PllClock {
    fn frequency(&self) -> Rate {
        match self {
            #[cfg(esp32h2)]
            Self::Pll8MHz => Rate::from_mhz(8),
            #[cfg(any(esp32c6, esp32h2))]
            Self::Pll48MHz => Rate::from_mhz(48),
            #[cfg(esp32h2)]
            Self::Pll64MHz => Rate::from_mhz(64),
            #[cfg(esp32c6)]
            Self::Pll80MHz => Rate::from_mhz(80),
            #[cfg(esp32h2)]
            Self::Pll96MHz => Rate::from_mhz(96),
            #[cfg(esp32c6)]
            Self::Pll120MHz => Rate::from_mhz(120),
            #[cfg(esp32c6)]
            Self::Pll160MHz => Rate::from_mhz(160),
            #[cfg(esp32c6)]
            Self::Pll240MHz => Rate::from_mhz(240),
            #[cfg(not(any(esp32c2, esp32c6, esp32h2)))]
            Self::Pll320MHz => Rate::from_mhz(320),
            #[cfg(not(esp32h2))]
            Self::Pll480MHz => Rate::from_mhz(480),
        }
    }
}

#[allow(unused)]
#[derive(Debug, Clone, Copy)]
pub(crate) enum ApbClock {
    #[cfg(esp32h2)]
    ApbFreq32MHz,
    #[cfg(not(esp32h2))]
    ApbFreq40MHz,
    #[cfg(not(esp32h2))]
    ApbFreq80MHz,
    ApbFreqOther(u32),
}

impl Clock for ApbClock {
    fn frequency(&self) -> Rate {
        match self {
            #[cfg(esp32h2)]
            ApbClock::ApbFreq32MHz => Rate::from_mhz(32),
            #[cfg(not(esp32h2))]
            ApbClock::ApbFreq40MHz => Rate::from_mhz(40),
            #[cfg(not(esp32h2))]
            ApbClock::ApbFreq80MHz => Rate::from_mhz(80),
            ApbClock::ApbFreqOther(mhz) => Rate::from_mhz(*mhz),
        }
    }
}

/// RTC FAST_CLK frequency values
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RtcFastClock {
    /// Main XTAL, divided by 4
    #[cfg(not(any(esp32c6, esp32h2)))]
    XtalD4,

    /// Select XTAL_D2_CLK as RTC_FAST_CLK source
    #[cfg(any(esp32c6, esp32h2))]
    XtalD2,

    /// Internal fast RC oscillator
    RcFast,
}

impl Clock for RtcFastClock {
    fn frequency(&self) -> Rate {
        match self {
            #[cfg(not(any(esp32c6, esp32h2)))]
            RtcFastClock::XtalD4 => Rate::from_hz(40_000_000 / 4),
            #[cfg(any(esp32c6, esp32h2))]
            RtcFastClock::XtalD2 => Rate::from_hz(property!("soc.xtal_frequency") / 2),
            RtcFastClock::RcFast => Rate::from_hz(property!("soc.rc_fast_clk_default")),
        }
    }
}

/// RTC SLOW_CLK frequency values
#[cfg(not(any(esp32c6, esp32h2)))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum RtcSlowClock {
    /// Internal slow RC oscillator
    RcSlow   = 0,
    /// External 32 KHz XTAL
    _32kXtal = 1,
    /// Internal fast RC oscillator, divided by 256
    _8mD256  = 2,
}

/// RTC SLOW_CLK frequency values
#[cfg(any(esp32c6, esp32h2))]
#[derive(Debug, Clone, Copy)]
#[non_exhaustive]
pub enum RtcSlowClock {
    /// Select RC_SLOW_CLK as RTC_SLOW_CLK source
    RcSlow   = 0,
    /// Select XTAL32K_CLK as RTC_SLOW_CLK source
    _32kXtal = 1,
    /// Select RC32K_CLK as RTC_SLOW_CLK source
    _32kRc   = 2,
    /// Select OSC_SLOW_CLK (external slow clock) as RTC_SLOW_CLK source
    OscSlow  = 3,
}

impl Clock for RtcSlowClock {
    fn frequency(&self) -> Rate {
        match self {
            RtcSlowClock::RcSlow => Rate::from_hz(property!("soc.rc_slow_clock")),
            RtcSlowClock::_32kXtal => Rate::from_hz(32_768),
            #[cfg(any(esp32c6, esp32h2))]
            RtcSlowClock::_32kRc => Rate::from_hz(32_768),
            #[cfg(not(any(esp32c6, esp32h2)))]
            RtcSlowClock::_8mD256 => RtcFastClock::RcFast.frequency() / 256,
            #[cfg(any(esp32c6, esp32h2))]
            RtcSlowClock::OscSlow => Rate::from_hz(32_768),
        }
    }
}

/// RTC Clocks.
pub struct RtcClock;

/// RTC Watchdog Timer driver.
impl RtcClock {
    const CAL_FRACT: u32 = 19;

    pub(crate) fn update_xtal_freq_mhz(mhz: u32) {
        let half = mhz & 0xFFFF;
        let combined = half | half << 16;

        let xtal_freq_reg = LP_AON::regs().store4().read().bits();
        let disable_log_bit = xtal_freq_reg & Rtc::RTC_DISABLE_ROM_LOG;
        debug!("Storing {}MHz to retention register", mhz);

        LP_AON::regs()
            .store4()
            .write(|w| unsafe { w.bits(combined | disable_log_bit) });
    }

    pub(crate) fn read_xtal_freq_mhz() -> Option<u32> {
        let xtal_freq_reg = LP_AON::regs().store4().read().bits();

        // RTC_XTAL_FREQ is stored as two copies in lower and upper 16-bit halves
        // need to mask out the RTC_DISABLE_ROM_LOG bit which is also stored in the same
        // register
        let xtal_freq = (xtal_freq_reg & !Rtc::RTC_DISABLE_ROM_LOG) as u16;
        let xtal_freq_copy = (xtal_freq_reg >> 16) as u16;

        if xtal_freq == xtal_freq_copy && xtal_freq != 0 && xtal_freq != u16::MAX {
            debug!("Read stored {}MHz from retention register", xtal_freq);
            Some(xtal_freq as u32)
        } else {
            None
        }
    }

    /// Get main XTAL frequency.
    ///
    /// This is the value stored in RTC register RTC_XTAL_FREQ_REG during esp-hal startup.
    #[instability::unstable]
    pub fn xtal_freq() -> XtalClock {
        match Self::read_xtal_freq_mhz() {
            Some(mhz) => XtalClock::closest_from_mhz(mhz),
            None => panic!("Clocks have not been initialized yet"),
        }
    }

    /// Get the RTC_SLOW_CLK source.
    #[cfg(not(any(esp32c6, esp32h2)))]
    pub fn slow_freq() -> RtcSlowClock {
        match LPWR::regs().clk_conf().read().ana_clk_rtc_sel().bits() {
            0 => RtcSlowClock::RcSlow,
            1 => RtcSlowClock::_32kXtal,
            2 => RtcSlowClock::_8mD256,
            _ => unreachable!(),
        }
    }

    /// Select source for RTC_SLOW_CLK.
    #[cfg(not(any(esp32c6, esp32h2)))]
    pub(crate) fn set_slow_freq(slow_freq: RtcSlowClock) {
        LPWR::regs().clk_conf().modify(|_, w| {
            unsafe {
                w.ana_clk_rtc_sel().bits(slow_freq as u8);
            }

            // Why we need to connect this clock to digital?
            // Or maybe this clock should be connected to digital when
            // XTAL 32k clock is enabled instead?
            w.dig_xtal32k_en()
                .bit(matches!(slow_freq, RtcSlowClock::_32kXtal));

            // The clk_8m_d256 will be closed when rtc_state in SLEEP,
            // so if the slow_clk is 8md256, clk_8m must be force power on
            w.ck8m_force_pu()
                .bit(matches!(slow_freq, RtcSlowClock::_8mD256))
        });

        crate::rom::ets_delay_us(300u32);
    }

    /// Select source for RTC_FAST_CLK.
    #[cfg(not(any(esp32c6, esp32h2)))]
    pub(crate) fn set_fast_freq(fast_freq: RtcFastClock) {
        LPWR::regs().clk_conf().modify(|_, w| {
            w.fast_clk_rtc_sel().bit(match fast_freq {
                RtcFastClock::RcFast => true,
                RtcFastClock::XtalD4 => false,
            })
        });

        crate::rom::ets_delay_us(3u32);
    }

    // TODO: IDF-5781 Some of esp32c6 SOC_RTC_FAST_CLK_SRC_XTAL_D2 rtc_fast clock
    // has timing issue. Force to use SOC_RTC_FAST_CLK_SRC_RC_FAST since 2nd
    // stage bootloader https://github.com/espressif/esp-idf/blob/master/components/bootloader_support/src/bootloader_clock_init.c#L65-L67
    #[cfg(any(esp32h2, esp32c6))]
    pub(crate) fn set_fast_freq(fast_freq: RtcFastClock) {
        #[cfg(esp32h2)]
        LPWR::regs().lp_clk_conf().modify(|_, w| unsafe {
            w.fast_clk_sel().bits(match fast_freq {
                RtcFastClock::RcFast => 0b00,
                RtcFastClock::XtalD2 => 0b01,
            })
        });

        #[cfg(esp32c6)]
        LPWR::regs().lp_clk_conf().modify(|_, w| {
            w.fast_clk_sel().bit(match fast_freq {
                RtcFastClock::RcFast => false,
                RtcFastClock::XtalD2 => true,
            })
        });

        crate::rom::ets_delay_us(3);
    }

    #[cfg(any(esp32h2, esp32c6))]
    pub(crate) fn set_slow_freq(slow_freq: RtcSlowClock) {
        LPWR::regs()
            .lp_clk_conf()
            .modify(|_, w| unsafe { w.slow_clk_sel().bits(slow_freq as u8) });

        LPWR::regs().clk_to_hp().modify(|_, w| {
            w.icg_hp_xtal32k()
                .bit(matches!(slow_freq, RtcSlowClock::_32kXtal));
            w.icg_hp_osc32k()
                .bit(matches!(slow_freq, RtcSlowClock::_32kRc))
        });
    }

    /// Get the RTC_SLOW_CLK source
    #[cfg(any(esp32h2, esp32c6))]
    pub fn slow_freq() -> RtcSlowClock {
        match LPWR::regs().lp_clk_conf().read().slow_clk_sel().bits() {
            0 => RtcSlowClock::RcSlow,
            1 => RtcSlowClock::_32kXtal,
            2 => RtcSlowClock::_32kRc,
            3 => RtcSlowClock::OscSlow,
            _ => unreachable!(),
        }
    }

    /// Calibration of RTC_SLOW_CLK is performed using a special feature of
    /// TIMG0. This feature counts the number of XTAL clock cycles within a
    /// given number of RTC_SLOW_CLK cycles.
    #[cfg(not(any(esp32c6, esp32h2)))] // TODO: merge with C6/H2 impl
    fn calibrate_internal(cal_clk: RtcCalSel, slowclk_cycles: u32) -> u32 {
        // Except for ESP32, choosing RTC_CAL_RTC_MUX results in calibration of
        // the 150k RTC clock (90k on ESP32-S2) regardless of the currently selected
        // SLOW_CLK. On the ESP32, it uses the currently selected SLOW_CLK.
        // The following code emulates ESP32 behavior for the other chips:
        #[cfg(not(esp32))]
        let cal_clk = match cal_clk {
            RtcCalSel::RtcMux => match RtcClock::slow_freq() {
                RtcSlowClock::_32kXtal => RtcCalSel::_32kXtal,
                RtcSlowClock::_8mD256 => RtcCalSel::_8mD256,
                _ => cal_clk,
            },
            RtcCalSel::InternalOsc => RtcCalSel::RtcMux,
            _ => cal_clk,
        };
        let rtc_cntl = LPWR::regs();
        let timg0 = TIMG0::regs();

        // Enable requested clock (150k clock is always on)
        let dig_32k_xtal_enabled = rtc_cntl.clk_conf().read().dig_xtal32k_en().bit_is_set();
        let dig_clk8m_d256_enabled = rtc_cntl.clk_conf().read().dig_clk8m_d256_en().bit_is_set();

        rtc_cntl.clk_conf().modify(|_, w| {
            if matches!(cal_clk, RtcCalSel::_32kXtal) {
                w.dig_xtal32k_en().set_bit();
            }

            if matches!(cal_clk, RtcCalSel::_8mD256) {
                w.dig_clk8m_d256_en().set_bit();
            }

            w
        });

        // There may be another calibration process already running during we
        // call this function, so we should wait the last process is done.
        #[cfg(not(esp32))]
        if timg0
            .rtccalicfg()
            .read()
            .rtc_cali_start_cycling()
            .bit_is_set()
        {
            // Set a small timeout threshold to accelerate the generation of timeout.
            // The internal circuit will be reset when the timeout occurs and will not
            // affect the next calibration.
            timg0
                .rtccalicfg2()
                .modify(|_, w| unsafe { w.rtc_cali_timeout_thres().bits(1) });

            while timg0.rtccalicfg().read().rtc_cali_rdy().bit_is_clear()
                && timg0.rtccalicfg2().read().rtc_cali_timeout().bit_is_clear()
            {}
        }

        // Prepare calibration
        timg0.rtccalicfg().modify(|_, w| unsafe {
            w.rtc_cali_clk_sel().bits(cal_clk as u8);
            w.rtc_cali_start_cycling().clear_bit();
            w.rtc_cali_max().bits(slowclk_cycles as u16)
        });

        // Figure out how long to wait for calibration to finish
        // Set timeout reg and expect time delay
        let expected_freq = match cal_clk {
            RtcCalSel::_32kXtal => {
                #[cfg(not(esp32))]
                timg0.rtccalicfg2().modify(|_, w| unsafe {
                    w.rtc_cali_timeout_thres().bits(slowclk_cycles << 12)
                });
                RtcSlowClock::_32kXtal
            }
            RtcCalSel::_8mD256 => {
                #[cfg(not(esp32))]
                timg0.rtccalicfg2().modify(|_, w| unsafe {
                    w.rtc_cali_timeout_thres().bits(slowclk_cycles << 12)
                });
                RtcSlowClock::_8mD256
            }
            _ => {
                #[cfg(not(esp32))]
                timg0.rtccalicfg2().modify(|_, w| unsafe {
                    w.rtc_cali_timeout_thres().bits(slowclk_cycles << 10)
                });
                RtcSlowClock::RcSlow
            }
        };

        let us_time_estimate = Rate::from_mhz(slowclk_cycles) / expected_freq.frequency();

        // Start calibration
        timg0
            .rtccalicfg()
            .modify(|_, w| w.rtc_cali_start().clear_bit());
        timg0
            .rtccalicfg()
            .modify(|_, w| w.rtc_cali_start().set_bit());

        // Wait for calibration to finish up to another us_time_estimate
        crate::rom::ets_delay_us(us_time_estimate);

        #[cfg(esp32)]
        let mut timeout_us = us_time_estimate;

        let cal_val = loop {
            if timg0.rtccalicfg().read().rtc_cali_rdy().bit_is_set() {
                break timg0.rtccalicfg1().read().rtc_cali_value().bits();
            }

            #[cfg(not(esp32))]
            if timg0.rtccalicfg2().read().rtc_cali_timeout().bit_is_set() {
                // Timed out waiting for calibration
                break 0;
            }

            #[cfg(esp32)]
            if timeout_us > 0 {
                timeout_us -= 1;
                crate::rom::ets_delay_us(1);
            } else {
                // Timed out waiting for calibration
                break 0;
            }
        };

        timg0
            .rtccalicfg()
            .modify(|_, w| w.rtc_cali_start().clear_bit());

        rtc_cntl.clk_conf().modify(|_, w| {
            w.dig_xtal32k_en().bit(dig_32k_xtal_enabled);
            w.dig_clk8m_d256_en().bit(dig_clk8m_d256_enabled)
        });

        cal_val
    }

    /// Calibration of RTC_SLOW_CLK is performed using a special feature of
    /// TIMG0. This feature counts the number of XTAL clock cycles within a
    /// given number of RTC_SLOW_CLK cycles.
    #[cfg(any(esp32c6, esp32h2))]
    pub(crate) fn calibrate_internal(mut cal_clk: RtcCalSel, slowclk_cycles: u32) -> u32 {
        use crate::peripherals::{LP_CLKRST, PCR, PMU};

        #[derive(Clone, Copy)]
        enum RtcCaliClkSel {
            CaliClkRcSlow = 0,
            CaliClkRcFast = 1,
            CaliClk32k    = 2,
        }

        if cal_clk == RtcCalSel::RtcMux {
            cal_clk = match cal_clk {
                RtcCalSel::RtcMux => match RtcClock::slow_freq() {
                    RtcSlowClock::_32kXtal => RtcCalSel::_32kXtal,
                    RtcSlowClock::_32kRc => RtcCalSel::_32kRc,
                    _ => cal_clk,
                },
                RtcCalSel::_32kOscSlow => RtcCalSel::RtcMux,
                _ => cal_clk,
            };
        }

        let clk_src = RtcClock::slow_freq();

        if cal_clk == RtcCalSel::RtcMux {
            cal_clk = match clk_src {
                RtcSlowClock::RcSlow => RtcCalSel::RcSlow,
                RtcSlowClock::_32kXtal => RtcCalSel::_32kXtal,
                RtcSlowClock::_32kRc => RtcCalSel::_32kRc,
                RtcSlowClock::OscSlow => RtcCalSel::_32kOscSlow,
            };
        }

        let cali_clk_sel;
        if cal_clk == RtcCalSel::RtcMux {
            cal_clk = match clk_src {
                RtcSlowClock::RcSlow => RtcCalSel::RcSlow,
                RtcSlowClock::_32kXtal => RtcCalSel::_32kXtal,
                RtcSlowClock::_32kRc => RtcCalSel::_32kRc,
                RtcSlowClock::OscSlow => RtcCalSel::RcSlow,
            }
        }

        if cal_clk == RtcCalSel::RcFast {
            cali_clk_sel = RtcCaliClkSel::CaliClkRcFast;
        } else if cal_clk == RtcCalSel::RcSlow {
            cali_clk_sel = RtcCaliClkSel::CaliClkRcSlow;
        } else {
            cali_clk_sel = RtcCaliClkSel::CaliClk32k;

            match cal_clk {
                RtcCalSel::RtcMux | RtcCalSel::RcSlow | RtcCalSel::RcFast => {}
                RtcCalSel::_32kRc => {
                    PCR::regs()
                        .ctrl_32k_conf()
                        .modify(|_, w| unsafe { w.clk_32k_sel().bits(0) });
                }
                RtcCalSel::_32kXtal => {
                    PCR::regs()
                        .ctrl_32k_conf()
                        .modify(|_, w| unsafe { w.clk_32k_sel().bits(1) });
                }
                RtcCalSel::_32kOscSlow => {
                    PCR::regs()
                        .ctrl_32k_conf()
                        .modify(|_, w| unsafe { w.clk_32k_sel().bits(2) });
                }
            }
        }

        // Enable requested clock (150k is always on)
        // Some delay is required before the time is stable
        // Only enable if originaly was disabled
        // If clock is already on, do nothing

        let dig_32k_xtal_enabled = LP_CLKRST::regs()
            .clk_to_hp()
            .read()
            .icg_hp_xtal32k()
            .bit_is_set();

        if cal_clk == RtcCalSel::_32kXtal && !dig_32k_xtal_enabled {
            LP_CLKRST::regs()
                .clk_to_hp()
                .modify(|_, w| w.icg_hp_xtal32k().set_bit());
        }

        // TODO: very hacky - icg_hp_xtal32k is already set in the above condition?
        // in ESP-IDF these are not called in this function but the fields are set
        LP_CLKRST::regs()
            .clk_to_hp()
            .modify(|_, w| w.icg_hp_xtal32k().set_bit());
        PMU::regs().hp_sleep_lp_ck_power().modify(|_, w| {
            w.hp_sleep_xpd_xtal32k().set_bit();
            w.hp_sleep_xpd_rc32k().set_bit()
        });

        let rc_fast_enabled = PMU::regs()
            .hp_sleep_lp_ck_power()
            .read()
            .hp_sleep_xpd_fosc_clk()
            .bit_is_set();
        let dig_rc_fast_enabled = LP_CLKRST::regs()
            .clk_to_hp()
            .read()
            .icg_hp_fosc()
            .bit_is_set();

        if cal_clk == RtcCalSel::RcFast {
            if !rc_fast_enabled {
                PMU::regs()
                    .hp_sleep_lp_ck_power()
                    .modify(|_, w| w.hp_sleep_xpd_fosc_clk().set_bit());
                crate::rom::ets_delay_us(50);
            }

            if !dig_rc_fast_enabled {
                LP_CLKRST::regs()
                    .clk_to_hp()
                    .modify(|_, w| w.icg_hp_fosc().set_bit());
                crate::rom::ets_delay_us(5);
            }
        }

        let rc32k_enabled = PMU::regs()
            .hp_sleep_lp_ck_power()
            .read()
            .hp_sleep_xpd_rc32k()
            .bit_is_set();
        let dig_rc32k_enabled = LP_CLKRST::regs()
            .clk_to_hp()
            .read()
            .icg_hp_osc32k()
            .bit_is_set();

        if cal_clk == RtcCalSel::_32kRc {
            if !rc32k_enabled {
                PMU::regs()
                    .hp_sleep_lp_ck_power()
                    .modify(|_, w| w.hp_sleep_xpd_rc32k().set_bit());
                crate::rom::ets_delay_us(300);
            }

            if !dig_rc32k_enabled {
                LP_CLKRST::regs()
                    .clk_to_hp()
                    .modify(|_, w| w.icg_hp_osc32k().set_bit());
            }
        }

        // Check if there is already running calibration process
        // TODO: &mut TIMG0 for calibration
        let timg0 = TIMG0::regs();

        if timg0
            .rtccalicfg()
            .read()
            .rtc_cali_start_cycling()
            .bit_is_set()
        {
            timg0
                .rtccalicfg2()
                .modify(|_, w| unsafe { w.rtc_cali_timeout_thres().bits(1) });

            // Set small timeout threshold to accelerate the generation of timeot
            // Internal circuit will be reset when timeout occurs and will not affect the
            // next calibration
            while !timg0.rtccalicfg().read().rtc_cali_rdy().bit_is_set()
                && !timg0.rtccalicfg2().read().rtc_cali_timeout().bit_is_set()
            {}
        }

        // Prepare calibration
        timg0
            .rtccalicfg()
            .modify(|_, w| unsafe { w.rtc_cali_clk_sel().bits(cali_clk_sel as u8) });
        timg0
            .rtccalicfg()
            .modify(|_, w| w.rtc_cali_start_cycling().clear_bit());
        timg0
            .rtccalicfg()
            .modify(|_, w| unsafe { w.rtc_cali_max().bits(slowclk_cycles as u16) });

        let expected_frequency = match cali_clk_sel {
            RtcCaliClkSel::CaliClk32k => {
                timg0.rtccalicfg2().modify(|_, w| unsafe {
                    w.rtc_cali_timeout_thres().bits(slowclk_cycles << 12)
                });
                RtcSlowClock::_32kXtal.frequency()
            }
            RtcCaliClkSel::CaliClkRcFast => {
                timg0
                    .rtccalicfg2()
                    .modify(|_, w| unsafe { w.rtc_cali_timeout_thres().bits(0x01FFFFFF) });
                RtcFastClock::RcFast.frequency()
            }
            RtcCaliClkSel::CaliClkRcSlow => {
                timg0.rtccalicfg2().modify(|_, w| unsafe {
                    w.rtc_cali_timeout_thres().bits(slowclk_cycles << 10)
                });
                RtcSlowClock::RcSlow.frequency()
            }
        };

        let us_time_estimate = Rate::from_mhz(slowclk_cycles) / expected_frequency;

        // Start calibration
        timg0
            .rtccalicfg()
            .modify(|_, w| w.rtc_cali_start().clear_bit());
        timg0
            .rtccalicfg()
            .modify(|_, w| w.rtc_cali_start().set_bit());

        // Wait for calibration to finish up to another us_time_estimate
        crate::rom::ets_delay_us(us_time_estimate);

        let cal_val = loop {
            if timg0.rtccalicfg().read().rtc_cali_rdy().bit_is_set() {
                // The Fosc CLK of calibration circuit is divided by 32 for ECO1.
                // So we need to multiply the frequency of the Fosc for ECO1 and above chips by
                // 32 times. And ensure that this modification will not affect
                // ECO0.
                // https://github.com/espressif/esp-idf/commit/e3148369f32fdc6de62c35a67f7adb6f4faef4e3
                #[cfg(esp32c6)]
                if Efuse::chip_revision() > 0 && cal_clk == RtcCalSel::RcFast {
                    break timg0.rtccalicfg1().read().rtc_cali_value().bits() >> 5;
                }
                break timg0.rtccalicfg1().read().rtc_cali_value().bits();
            }

            if timg0.rtccalicfg2().read().rtc_cali_timeout().bit_is_set() {
                // Timed out waiting for calibration
                break 0;
            }
        };

        timg0
            .rtccalicfg()
            .modify(|_, w| w.rtc_cali_start().clear_bit());

        if cal_clk == RtcCalSel::_32kXtal && !dig_32k_xtal_enabled {
            LP_CLKRST::regs()
                .clk_to_hp()
                .modify(|_, w| w.icg_hp_xtal32k().clear_bit());
        }

        if cal_clk == RtcCalSel::RcFast {
            if rc_fast_enabled {
                PMU::regs()
                    .hp_sleep_lp_ck_power()
                    .modify(|_, w| w.hp_sleep_xpd_fosc_clk().set_bit());
                crate::rom::ets_delay_us(50);
            }

            if dig_rc_fast_enabled {
                LP_CLKRST::regs()
                    .clk_to_hp()
                    .modify(|_, w| w.icg_hp_fosc().set_bit());
                crate::rom::ets_delay_us(5);
            }
        }

        if cal_clk == RtcCalSel::_32kRc {
            if rc32k_enabled {
                PMU::regs()
                    .hp_sleep_lp_ck_power()
                    .modify(|_, w| w.hp_sleep_xpd_rc32k().set_bit());
                crate::rom::ets_delay_us(300);
            }
            if dig_rc32k_enabled {
                LP_CLKRST::regs()
                    .clk_to_hp()
                    .modify(|_, w| w.icg_hp_osc32k().set_bit());
            }
        }

        cal_val
    }

    /// Measure RTC slow clock's period, based on main XTAL frequency
    ///
    /// This function will time out and return 0 if the time for the given
    /// number of cycles to be counted exceeds the expected time twice. This
    /// may happen if 32k XTAL is being calibrated, but the oscillator has
    /// not started up (due to incorrect loading capacitance, board design
    /// issue, or lack of 32 XTAL on board).
    pub(crate) fn calibrate(cal_clk: RtcCalSel, slowclk_cycles: u32) -> u32 {
        let xtal_freq = RtcClock::xtal_freq();

        #[cfg(esp32c6)]
        let slowclk_cycles = if Efuse::chip_revision() > 0 && cal_clk == RtcCalSel::RcFast {
            // The Fosc CLK of calibration circuit is divided by 32 for ECO1.
            // So we need to divide the calibrate cycles of the FOSC for ECO1 and above
            // chips by 32 to avoid excessive calibration time.
            slowclk_cycles >> 5
        } else {
            slowclk_cycles
        };

        let xtal_cycles = RtcClock::calibrate_internal(cal_clk, slowclk_cycles) as u64;
        let divider = xtal_freq.mhz() as u64 * slowclk_cycles as u64;
        let period_64 = ((xtal_cycles << RtcClock::CAL_FRACT) + divider / 2u64 - 1u64) / divider;

        (period_64 & u32::MAX as u64) as u32
    }

    /// Calculate the necessary RTC_SLOW_CLK cycles to complete 1 millisecond.
    pub(crate) fn cycles_to_1ms() -> u16 {
        cfg_if::cfg_if! {
            if #[cfg(any(esp32c6, esp32h2))] {
                let calibration_clock = match RtcClock::slow_freq() {
                    RtcSlowClock::RcSlow => RtcCalSel::RtcMux,
                    RtcSlowClock::_32kXtal => RtcCalSel::_32kXtal,
                    RtcSlowClock::_32kRc => RtcCalSel::_32kRc,
                    RtcSlowClock::OscSlow => RtcCalSel::_32kOscSlow,
                    // RtcSlowClock::RcFast => RtcCalSel::RcFast,
                };
            } else {
                let calibration_clock = match RtcClock::slow_freq() {
                    RtcSlowClock::RcSlow => RtcCalSel::RtcMux,
                    RtcSlowClock::_32kXtal => RtcCalSel::_32kXtal,
                    RtcSlowClock::_8mD256 => RtcCalSel::_8mD256,
                };
            }
        }

        // TODO: store the result somewhere
        let period_13q19 = RtcClock::calibrate(calibration_clock, 1024);

        // 100_000_000 is used to get rid of `float` calculations
        let period = (100_000_000 * period_13q19 as u64) / (1 << RtcClock::CAL_FRACT);

        (100_000_000 * 1000 / period) as u16
    }

    /// Return estimated XTAL frequency in MHz.
    pub(crate) fn estimate_xtal_frequency() -> u32 {
        // TODO: this could reuse Self::calibrate_internal
        const SLOW_CLOCK_CYCLES: u32 = 100;

        let calibration_clock = RtcSlowClock::RcSlow;

        // Make sure the process doesn't time out due to some spooky configuration.
        #[cfg(not(esp32))]
        TIMG0::regs().rtccalicfg2().reset();

        TIMG0::regs()
            .rtccalicfg()
            .modify(|_, w| w.rtc_cali_start().clear_bit());

        TIMG0::regs().rtccalicfg().write(|w| unsafe {
            w.rtc_cali_clk_sel().bits(calibration_clock as u8);
            w.rtc_cali_max().bits(SLOW_CLOCK_CYCLES as u16);
            w.rtc_cali_start_cycling().clear_bit();
            w.rtc_cali_start().set_bit()
        });

        // Delay, otherwise the CPU may read back the previous state of the completion flag and skip
        // waiting.
        ets_delay_us(SLOW_CLOCK_CYCLES * 1_000_000 / calibration_clock.frequency().as_hz());

        // Wait for the calibration to finish
        while TIMG0::regs()
            .rtccalicfg()
            .read()
            .rtc_cali_rdy()
            .bit_is_clear()
        {}

        let cali_value = TIMG0::regs().rtccalicfg1().read().rtc_cali_value().bits();

        TIMG0::regs()
            .rtccalicfg()
            .modify(|_, w| w.rtc_cali_start().clear_bit());

        (cali_value * (calibration_clock.frequency().as_hz() / SLOW_CLOCK_CYCLES)) / 1_000_000
    }
}

/// Clock frequencies.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
#[doc(hidden)]
pub struct Clocks {
    /// CPU clock frequency
    pub cpu_clock: Rate,

    /// APB clock frequency
    pub apb_clock: Rate,

    /// XTAL clock frequency
    pub xtal_clock: Rate,

    /// I2C clock frequency
    #[cfg(esp32)]
    pub i2c_clock: Rate,

    /// PWM clock frequency
    #[cfg(esp32)]
    pub pwm_clock: Rate,

    /// Crypto PWM  clock frequency
    #[cfg(esp32s3)]
    pub crypto_pwm_clock: Rate,

    /// Crypto clock frequency
    #[cfg(any(esp32c6, esp32h2))]
    pub crypto_clock: Rate,

    /// PLL 48M clock frequency (fixed)
    #[cfg(esp32h2)]
    pub pll_48m_clock: Rate,

    /// PLL 96M clock frequency (fixed)
    #[cfg(esp32h2)]
    pub pll_96m_clock: Rate,
}

static mut ACTIVE_CLOCKS: Option<Clocks> = None;

impl Clocks {
    pub(crate) fn init(cpu_clock_speed: CpuClock) {
        ESP_HAL_LOCK.lock(|| {
            crate::rtc_cntl::rtc::init();

            let config = Self::configure(cpu_clock_speed);
            RtcClock::update_xtal_freq_mhz(config.xtal_clock.as_mhz());
            unsafe { ACTIVE_CLOCKS = Some(config) };
        })
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

    /// Returns the xtal frequency.
    ///
    /// This function will run the frequency estimation if called before
    /// [`crate::init()`].
    #[cfg(systimer)]
    #[inline]
    pub(crate) fn xtal_freq() -> Rate {
        if esp_config::esp_config_str!("ESP_HAL_CONFIG_XTAL_FREQUENCY") == "auto"
            && let Some(clocks) = Self::try_get()
        {
            return clocks.xtal_clock;
        }

        Self::measure_xtal_frequency().frequency()
    }

    const fn xtal_frequency_from_config() -> Option<XtalClock> {
        let frequency_conf = esp_config::esp_config_str!("ESP_HAL_CONFIG_XTAL_FREQUENCY");

        for_each_soc_xtal_options!(
            (all $( ($freq:literal) ),*) => {
                paste::paste! {
                    return match frequency_conf.as_bytes() {
                        b"auto" => None,

                        // If the frequency is a pre-set value for the chip, return the associated enum variant.
                        $( _ if esp_config::esp_config_int_parse!(u32, frequency_conf) == $freq => Some(XtalClock::[<_ $freq M>]), )*

                        _ => None,
                    };
                }
            };
        );
    }

    fn measure_xtal_frequency() -> XtalClock {
        if let Some(clock) = const { Self::xtal_frequency_from_config() } {
            // Use the configured frequency
            clock
        } else if esp_config::esp_config_str!("ESP_HAL_CONFIG_XTAL_FREQUENCY") == "auto" {
            // TODO: we should be able to read from a retention register, but probe-rs flashes a
            // bootloader that assumes a frequency, instead of choosing a matching one.
            let mhz = RtcClock::estimate_xtal_frequency();

            debug!("Working with a {}MHz crystal", mhz);

            // Try to guess the closest possible crystal value.
            XtalClock::closest_from_mhz(mhz)
        } else {
            unreachable!("Invalid crystal frequency configured, this should not be possible.")
        }
    }
}

#[cfg(esp32)]
impl Clocks {
    /// Configure the CPU clock speed.
    pub(crate) fn configure(cpu_clock_speed: CpuClock) -> Self {
        let xtal_freq = Self::measure_xtal_frequency();

        if cpu_clock_speed != CpuClock::default() {
            let pll_freq = match cpu_clock_speed {
                CpuClock::_80MHz => PllClock::Pll320MHz,
                CpuClock::_160MHz => PllClock::Pll320MHz,
                CpuClock::_240MHz => PllClock::Pll480MHz,
            };

            clocks_ll::esp32_rtc_update_to_xtal(xtal_freq, 1);
            clocks_ll::esp32_rtc_bbpll_enable();
            clocks_ll::esp32_rtc_bbpll_configure(xtal_freq, pll_freq);
            clocks_ll::set_cpu_freq(cpu_clock_speed);
        }

        Self {
            cpu_clock: cpu_clock_speed.frequency(),
            apb_clock: Rate::from_mhz(80),
            xtal_clock: Rate::from_mhz(xtal_freq.mhz()),
            i2c_clock: Rate::from_mhz(80),
            // The docs are unclear here. pwm_clock seems to be tied to clocks.apb_clock
            // while simultaneously being fixed at 160 MHz.
            // Testing showed 160 MHz to be correct for current clock configurations.
            pwm_clock: Rate::from_mhz(160),
        }
    }
}

#[cfg(esp32c2)]
impl Clocks {
    /// Configure the CPU clock speed.
    pub(crate) fn configure(cpu_clock_speed: CpuClock) -> Self {
        let xtal_freq = Self::measure_xtal_frequency();

        let apb_freq;
        if cpu_clock_speed != CpuClock::default() {
            let pll_freq = PllClock::Pll480MHz;

            if cpu_clock_speed.mhz() <= xtal_freq.mhz() {
                apb_freq = ApbClock::ApbFreqOther(cpu_clock_speed.mhz());
                clocks_ll::esp32c2_rtc_update_to_xtal(xtal_freq, 1);
                clocks_ll::esp32c2_rtc_apb_freq_update(apb_freq);
            } else {
                apb_freq = ApbClock::ApbFreq40MHz;
                clocks_ll::esp32c2_rtc_bbpll_enable();
                clocks_ll::esp32c2_rtc_bbpll_configure(xtal_freq, pll_freq);
                clocks_ll::esp32c2_rtc_freq_to_pll_mhz(cpu_clock_speed);
                clocks_ll::esp32c2_rtc_apb_freq_update(apb_freq);
            }
        } else {
            apb_freq = ApbClock::ApbFreq40MHz;
        }

        Self {
            cpu_clock: cpu_clock_speed.frequency(),
            apb_clock: apb_freq.frequency(),
            xtal_clock: xtal_freq.frequency(),
        }
    }
}

#[cfg(esp32c3)]
impl Clocks {
    /// Configure the CPU clock speed.
    pub(crate) fn configure(cpu_clock_speed: CpuClock) -> Self {
        let xtal_freq = Self::measure_xtal_frequency();

        let apb_freq;
        if cpu_clock_speed != CpuClock::default() {
            if cpu_clock_speed.mhz() <= xtal_freq.mhz() {
                apb_freq = ApbClock::ApbFreqOther(cpu_clock_speed.mhz());
                clocks_ll::esp32c3_rtc_update_to_xtal(xtal_freq, 1);
                clocks_ll::esp32c3_rtc_apb_freq_update(apb_freq);
            } else {
                let pll_freq = PllClock::Pll480MHz;
                apb_freq = ApbClock::ApbFreq80MHz;
                clocks_ll::esp32c3_rtc_bbpll_enable();
                clocks_ll::esp32c3_rtc_bbpll_configure(xtal_freq, pll_freq);
                clocks_ll::esp32c3_rtc_freq_to_pll_mhz(cpu_clock_speed);
                clocks_ll::esp32c3_rtc_apb_freq_update(apb_freq);
            }
        } else {
            apb_freq = ApbClock::ApbFreq80MHz;
        }

        Self {
            cpu_clock: cpu_clock_speed.frequency(),
            apb_clock: apb_freq.frequency(),
            xtal_clock: xtal_freq.frequency(),
        }
    }
}

#[cfg(esp32c6)]
impl Clocks {
    /// Configure the CPU clock speed.
    pub(crate) fn configure(cpu_clock_speed: CpuClock) -> Self {
        let xtal_freq = Self::measure_xtal_frequency();

        let apb_freq;
        if cpu_clock_speed != CpuClock::default() {
            if cpu_clock_speed.mhz() <= xtal_freq.mhz() {
                apb_freq = ApbClock::ApbFreqOther(cpu_clock_speed.mhz());
                clocks_ll::esp32c6_rtc_update_to_xtal(xtal_freq, 1);
                clocks_ll::esp32c6_rtc_apb_freq_update(apb_freq);
            } else {
                let pll_freq = PllClock::Pll480MHz;
                apb_freq = ApbClock::ApbFreq80MHz;
                clocks_ll::esp32c6_rtc_bbpll_enable();
                clocks_ll::esp32c6_rtc_bbpll_configure(xtal_freq, pll_freq);
                clocks_ll::esp32c6_rtc_freq_to_pll_mhz(cpu_clock_speed);
                clocks_ll::esp32c6_rtc_apb_freq_update(apb_freq);
            }
        } else {
            apb_freq = ApbClock::ApbFreq80MHz;
        }

        Self {
            cpu_clock: cpu_clock_speed.frequency(),
            apb_clock: apb_freq.frequency(),
            xtal_clock: xtal_freq.frequency(),
            crypto_clock: Rate::from_mhz(160),
        }
    }
}

#[cfg(esp32h2)]
impl Clocks {
    /// Configure the CPU clock speed.
    pub(crate) fn configure(cpu_clock_speed: CpuClock) -> Self {
        let xtal_freq = Self::measure_xtal_frequency();

        let apb_freq;
        if cpu_clock_speed != CpuClock::default() {
            if cpu_clock_speed.mhz() <= xtal_freq.mhz() {
                apb_freq = ApbClock::ApbFreqOther(cpu_clock_speed.mhz());
                clocks_ll::esp32h2_rtc_update_to_xtal(xtal_freq, 1);
                clocks_ll::esp32h2_rtc_apb_freq_update(apb_freq);
            } else {
                let pll_freq = PllClock::Pll96MHz;
                apb_freq = ApbClock::ApbFreq32MHz;
                clocks_ll::esp32h2_rtc_bbpll_enable();
                clocks_ll::esp32h2_rtc_bbpll_configure(xtal_freq, pll_freq);
                clocks_ll::esp32h2_rtc_freq_to_pll_mhz(cpu_clock_speed);
                clocks_ll::esp32h2_rtc_apb_freq_update(apb_freq);
            }
        } else {
            apb_freq = ApbClock::ApbFreq32MHz;
        }

        Self {
            cpu_clock: cpu_clock_speed.frequency(),
            apb_clock: apb_freq.frequency(),
            xtal_clock: xtal_freq.frequency(),
            pll_48m_clock: Rate::from_mhz(48),
            crypto_clock: Rate::from_mhz(96),
            pll_96m_clock: Rate::from_mhz(96),
        }
    }
}

#[cfg(esp32s2)]
impl Clocks {
    /// Configure the CPU clock speed.
    pub(crate) fn configure(cpu_clock_speed: CpuClock) -> Self {
        let xtal_freq = Self::measure_xtal_frequency();

        if cpu_clock_speed != CpuClock::default() {
            clocks_ll::set_cpu_clock(cpu_clock_speed);
        }

        Self {
            cpu_clock: cpu_clock_speed.frequency(),
            apb_clock: Rate::from_mhz(80),
            xtal_clock: xtal_freq.frequency(),
        }
    }
}

#[cfg(esp32s3)]
impl Clocks {
    /// Configure the CPU clock speed.
    pub(crate) fn configure(cpu_clock_speed: CpuClock) -> Self {
        let xtal_freq = Self::measure_xtal_frequency();

        if cpu_clock_speed != CpuClock::default() {
            clocks_ll::set_cpu_clock(cpu_clock_speed);
        }

        Self {
            cpu_clock: cpu_clock_speed.frequency(),
            apb_clock: Rate::from_mhz(80),
            xtal_clock: xtal_freq.frequency(),
            crypto_pwm_clock: Rate::from_mhz(160),
        }
    }
}

#[cfg(any(bt, ieee802154, wifi))]
/// Tracks the number of references to the PHY clock.
static PHY_CLOCK_REF_COUNTER: embassy_sync::blocking_mutex::Mutex<RawMutex, Cell<u8>> =
    embassy_sync::blocking_mutex::Mutex::new(Cell::new(0));

#[cfg(any(bt, ieee802154, wifi))]
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

#[cfg(any(bt, ieee802154, wifi))]
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
#[cfg(any(bt, ieee802154, wifi))]
#[derive(Debug)]
/// Prevents the PHY clock from being disabled.
///
/// As long as at least one [PhyClockGuard] exists, the PHY clock will remain
/// active. To release this guard, you can either let it go out of scope or use
/// [PhyClockGuard::release] to explicitly release it.
pub struct PhyClockGuard<'d> {
    _phantom: PhantomData<&'d ()>,
}

#[cfg(any(bt, ieee802154, wifi))]
impl PhyClockGuard<'_> {
    #[instability::unstable]
    #[inline]
    /// Release the clock guard.
    ///
    /// The PHY clock will be disabled, if this is the last clock guard.
    pub fn release(self) {}
}

#[cfg(any(bt, ieee802154, wifi))]
impl Drop for PhyClockGuard<'_> {
    fn drop(&mut self) {
        decrease_phy_clock_ref_count_internal();
    }
}

#[cfg(any(bt, ieee802154, wifi))]
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

#[cfg(wifi)]
#[instability::unstable]
impl<'d> ModemClockController<'d> for WIFI<'d> {
    fn enable_modem_clock(&mut self, enable: bool) {
        clocks_ll::enable_wifi(enable);
    }
}
#[cfg(wifi)]
impl WIFI<'_> {
    #[instability::unstable]
    /// Reset the Wi-Fi MAC.
    pub fn reset_wifi_mac(&mut self) {
        clocks_ll::reset_wifi_mac();
    }
}

#[cfg(bt)]
#[instability::unstable]
impl<'d> ModemClockController<'d> for BT<'d> {
    fn enable_modem_clock(&mut self, enable: bool) {
        clocks_ll::enable_bt(enable);
    }
}
#[cfg(bt)]
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

#[cfg(ieee802154)]
#[instability::unstable]
impl<'d> ModemClockController<'d> for IEEE802154<'d> {
    fn enable_modem_clock(&mut self, enable: bool) {
        clocks_ll::enable_ieee802154(enable);
    }
}
