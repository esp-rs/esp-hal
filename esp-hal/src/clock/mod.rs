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
//!
//!   * 80 MHz
//!   * 96 MHz
//!   * 120 MHz
//!   * 160 MHz
//!   * 240 MHz
//!
//! and others, depending on the microcontroller model.
//!
//! ### Frozen Clock Frequencies
//!
//! Once the clock configuration is applied, the clock frequencies become
//! `frozen` and cannot be changed. The `Clocks` struct is returned as part of
//! the `System` struct, providing read-only access to the configured clock
//! frequencies.
//!
//! ## Examples
//!
//! ### Initialize With Different Clock Frequencies
//! ```rust, no_run
//! # #![no_std]
//! # use esp_hal::prelude::*;
//! # #[panic_handler]
//! # fn panic(_ : &core::panic::PanicInfo) -> ! {
//! #     loop {}
//! # }
//! # fn main() {
//! // Initialize with the highest possible frequency for this chip
//! let (peripherals, clocks) = esp_hal::init({
//!     let mut config = esp_hal::Config::default();
//!     config.cpu_clock = CpuClock::max();
//!     config
//! });
//!
//! // Initialize with custom clock frequency
//! // let (peripherals, clocks) = esp_hal::init({
//! //    let mut config = esp_hal::Config::default();
#![cfg_attr(
    not(any(esp32c2, esp32h2)),
    doc = "//    config.cpu_clock = CpuClock::Clock160MHz;"
)]
#![cfg_attr(esp32c2, doc = "//    config.cpu_clock = CpuClock::Clock120MHz;")]
#![cfg_attr(esp32h2, doc = "//    config.cpu_clock = CpuClock::Clock96MHz;")]
//! //    config
//! // });
//! //
//! // Initialize with default clock frequency for this chip
//! // let (peripherals, clocks) = esp_hal::init(esp_hal::Config::default());
//! # }
//! ```

use core::marker::PhantomData;

use fugit::HertzU32;
#[cfg(esp32c2)]
use portable_atomic::{AtomicU32, Ordering};

#[cfg(any(esp32, esp32c2))]
use crate::rtc_cntl::RtcClock;

#[cfg_attr(esp32, path = "clocks_ll/esp32.rs")]
#[cfg_attr(esp32c2, path = "clocks_ll/esp32c2.rs")]
#[cfg_attr(esp32c3, path = "clocks_ll/esp32c3.rs")]
#[cfg_attr(esp32c6, path = "clocks_ll/esp32c6.rs")]
#[cfg_attr(esp32h2, path = "clocks_ll/esp32h2.rs")]
#[cfg_attr(esp32s2, path = "clocks_ll/esp32s2.rs")]
#[cfg_attr(esp32s3, path = "clocks_ll/esp32s3.rs")]
pub(crate) mod clocks_ll;

/// Clock properties
pub trait Clock {
    /// Frequency of the clock in [Hertz](fugit::HertzU32), using [fugit] types.
    fn frequency(&self) -> HertzU32;

    /// Frequency of the clock in Megahertz
    fn mhz(&self) -> u32 {
        self.frequency().to_MHz()
    }

    /// Frequency of the clock in Hertz
    fn hz(&self) -> u32 {
        self.frequency().to_Hz()
    }
}

/// CPU clock speed
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CpuClock {
    /// 80MHz CPU clock
    #[cfg(not(esp32h2))]
    Clock80MHz  = 80,

    /// 96MHz CPU clock
    #[cfg(esp32h2)]
    Clock96MHz  = 96,

    /// 120MHz CPU clock
    #[cfg(esp32c2)]
    Clock120MHz = 120,

    /// 160MHz CPU clock
    #[cfg(not(any(esp32c2, esp32h2)))]
    Clock160MHz = 160,

    /// 240MHz CPU clock
    #[cfg(xtensa)]
    Clock240MHz = 240,
}

impl Default for CpuClock {
    fn default() -> Self {
        cfg_if::cfg_if! {
            if #[cfg(esp32h2)] {
                Self::Clock96MHz
            } else {
                // FIXME: I don't think this is correct in general?
                Self::Clock80MHz
            }
        }
    }
}

impl CpuClock {
    /// Use the highest possible frequency for a particular chip.
    pub const fn max() -> Self {
        cfg_if::cfg_if! {
            if #[cfg(esp32c2)] {
                Self::Clock120MHz
            } else if #[cfg(any(esp32c3, esp32c6))] {
                Self::Clock160MHz
            } else if #[cfg(esp32h2)] {
                Self::Clock96MHz
            } else {
                Self::Clock240MHz
            }
        }
    }
}

impl Clock for CpuClock {
    fn frequency(&self) -> HertzU32 {
        HertzU32::MHz(*self as u32)
    }
}

/// XTAL clock speed
#[derive(Debug, Clone, Copy)]
#[non_exhaustive]
pub enum XtalClock {
    /// 26MHz XTAL clock
    #[cfg(any(esp32, esp32c2))]
    RtcXtalFreq26M,
    /// 32MHz XTAL clock
    #[cfg(any(esp32c3, esp32h2, esp32s3))]
    RtcXtalFreq32M,
    /// 40MHz XTAL clock
    #[cfg(not(esp32h2))]
    RtcXtalFreq40M,
    /// Other XTAL clock
    RtcXtalFreqOther(u32),
}

impl Clock for XtalClock {
    fn frequency(&self) -> HertzU32 {
        match self {
            #[cfg(any(esp32, esp32c2))]
            XtalClock::RtcXtalFreq26M => HertzU32::MHz(26),
            #[cfg(any(esp32c3, esp32h2, esp32s3))]
            XtalClock::RtcXtalFreq32M => HertzU32::MHz(32),
            #[cfg(not(esp32h2))]
            XtalClock::RtcXtalFreq40M => HertzU32::MHz(40),
            XtalClock::RtcXtalFreqOther(mhz) => HertzU32::MHz(*mhz),
        }
    }
}

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
    fn frequency(&self) -> HertzU32 {
        match self {
            #[cfg(esp32h2)]
            Self::Pll8MHz => HertzU32::MHz(8),
            #[cfg(any(esp32c6, esp32h2))]
            Self::Pll48MHz => HertzU32::MHz(48),
            #[cfg(esp32h2)]
            Self::Pll64MHz => HertzU32::MHz(64),
            #[cfg(esp32c6)]
            Self::Pll80MHz => HertzU32::MHz(80),
            #[cfg(esp32h2)]
            Self::Pll96MHz => HertzU32::MHz(96),
            #[cfg(esp32c6)]
            Self::Pll120MHz => HertzU32::MHz(120),
            #[cfg(esp32c6)]
            Self::Pll160MHz => HertzU32::MHz(160),
            #[cfg(esp32c6)]
            Self::Pll240MHz => HertzU32::MHz(240),
            #[cfg(not(any(esp32c2, esp32c6, esp32h2)))]
            Self::Pll320MHz => HertzU32::MHz(320),
            #[cfg(not(esp32h2))]
            Self::Pll480MHz => HertzU32::MHz(480),
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
    fn frequency(&self) -> HertzU32 {
        match self {
            #[cfg(esp32h2)]
            ApbClock::ApbFreq32MHz => HertzU32::MHz(32),
            #[cfg(not(esp32h2))]
            ApbClock::ApbFreq40MHz => HertzU32::MHz(40),
            #[cfg(not(esp32h2))]
            ApbClock::ApbFreq80MHz => HertzU32::MHz(80),
            ApbClock::ApbFreqOther(mhz) => HertzU32::MHz(*mhz),
        }
    }
}

/// Frozen clock frequencies
///
/// The instantiation of this type indicates that the clock configuration can no
/// longer be changed.
pub struct Clocks<'a> {
    _private: PhantomData<&'a ()>,
    rates: RawClocks,
}

impl<'a> Clocks<'a> {
    /// This should not be used in user code.
    /// The whole point this exists is make it possible to have other crates
    /// (i.e. esp-wifi) create `Clocks`
    #[doc(hidden)]
    pub(crate) fn from_raw_clocks(raw_clocks: RawClocks) -> Clocks<'a> {
        Self {
            _private: PhantomData,
            rates: raw_clocks,
        }
    }
}

impl core::ops::Deref for Clocks<'_> {
    type Target = RawClocks;

    fn deref(&self) -> &RawClocks {
        &self.rates
    }
}

/// The list of the clock frequencies that are used in the system.
pub struct RawClocks {
    /// CPU clock frequency
    pub cpu_clock: HertzU32,

    /// APB clock frequency
    pub apb_clock: HertzU32,

    /// XTAL clock frequency
    pub xtal_clock: HertzU32,

    /// I2C clock frequency
    #[cfg(esp32)]
    pub i2c_clock: HertzU32,

    /// PWM clock frequency
    #[cfg(esp32)]
    pub pwm_clock: HertzU32,

    /// Crypto PWM  clock frequency
    #[cfg(esp32s3)]
    pub crypto_pwm_clock: HertzU32,

    /// Crypto clock frequency
    #[cfg(any(esp32c6, esp32h2))]
    pub crypto_clock: HertzU32,

    /// PLL 48M clock frequency (fixed)
    #[cfg(esp32h2)]
    pub pll_48m_clock: HertzU32,

    /// PLL 96M clock frequency (fixed)
    #[cfg(esp32h2)]
    pub pll_96m_clock: HertzU32,
}

cfg_if::cfg_if! {
    if #[cfg(esp32c2)] {
        static XTAL_FREQ_MHZ: AtomicU32 = AtomicU32::new(40);

        pub(crate) fn xtal_freq_mhz() -> u32 {
            XTAL_FREQ_MHZ.load(Ordering::Relaxed)
        }
    } else if #[cfg(esp32h2)] {
        pub(crate) fn xtal_freq_mhz() -> u32 {
            32
        }
    } else if #[cfg(any(esp32, esp32s2))] {
        // Function would be unused
    } else {
        pub(crate) fn xtal_freq_mhz() -> u32 {
            40
        }
    }
}

/// Used to configure the frequencies of the clocks present in the chip.
///
/// After setting all frequencies, call the freeze function to apply the
/// configuration.
pub struct ClockControl {
    desired_rates: RawClocks,
}

impl ClockControl {
    pub(crate) fn new(clock: CpuClock) -> Self {
        Self::configure(clock)
    }

    /// Applies the clock configuration and returns a Clocks struct that
    /// signifies that the clocks are frozen, and contains the frequencies
    /// used. After this function is called, the clocks can not change
    pub fn freeze(self) -> Clocks<'static> {
        Clocks::from_raw_clocks(self.desired_rates)
    }
}

#[cfg(esp32)]
impl ClockControl {
    /// Configure the CPU clock speed.
    pub(crate) fn configure(cpu_clock_speed: CpuClock) -> ClockControl {
        let xtal_freq = if RtcClock::estimate_xtal_frequency() > 33 {
            XtalClock::RtcXtalFreq40M
        } else {
            XtalClock::RtcXtalFreq26M
        };

        if cpu_clock_speed != CpuClock::default() {
            let pll_freq = match cpu_clock_speed {
                CpuClock::Clock80MHz => PllClock::Pll320MHz,
                CpuClock::Clock160MHz => PllClock::Pll320MHz,
                CpuClock::Clock240MHz => PllClock::Pll480MHz,
            };

            clocks_ll::esp32_rtc_update_to_xtal(xtal_freq, 1);
            clocks_ll::esp32_rtc_bbpll_enable();
            clocks_ll::esp32_rtc_bbpll_configure(xtal_freq, pll_freq);
            clocks_ll::set_cpu_freq(cpu_clock_speed);
        }

        ClockControl {
            desired_rates: RawClocks {
                cpu_clock: cpu_clock_speed.frequency(),
                apb_clock: HertzU32::MHz(80),
                xtal_clock: HertzU32::MHz(xtal_freq.mhz()),
                i2c_clock: HertzU32::MHz(80),
                // The docs are unclear here. pwm_clock seems to be tied to clocks.apb_clock
                // while simultaneously being fixed at 160 MHz.
                // Testing showed 160 MHz to be correct for current clock configurations.
                pwm_clock: HertzU32::MHz(160),
            },
        }
    }
}

#[cfg(esp32c2)]
impl ClockControl {
    /// Configure the CPU clock speed.
    pub(crate) fn configure(cpu_clock_speed: CpuClock) -> ClockControl {
        let xtal_freq = if RtcClock::estimate_xtal_frequency() > 33 {
            XtalClock::RtcXtalFreq40M
        } else {
            XtalClock::RtcXtalFreq26M
        };
        XTAL_FREQ_MHZ.store(xtal_freq.mhz(), Ordering::Relaxed);

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

        ClockControl {
            desired_rates: RawClocks {
                cpu_clock: cpu_clock_speed.frequency(),
                apb_clock: apb_freq.frequency(),
                xtal_clock: xtal_freq.frequency(),
            },
        }
    }
}

#[cfg(esp32c3)]
impl ClockControl {
    /// Configure the CPU clock speed.
    pub(crate) fn configure(cpu_clock_speed: CpuClock) -> ClockControl {
        let xtal_freq = XtalClock::RtcXtalFreq40M;

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

        ClockControl {
            desired_rates: RawClocks {
                cpu_clock: cpu_clock_speed.frequency(),
                apb_clock: apb_freq.frequency(),
                xtal_clock: xtal_freq.frequency(),
            },
        }
    }
}

#[cfg(esp32c6)]
impl ClockControl {
    /// Configure the CPU clock speed.
    pub(crate) fn configure(cpu_clock_speed: CpuClock) -> ClockControl {
        let xtal_freq = XtalClock::RtcXtalFreq40M;

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

        ClockControl {
            desired_rates: RawClocks {
                cpu_clock: cpu_clock_speed.frequency(),
                apb_clock: apb_freq.frequency(),
                xtal_clock: xtal_freq.frequency(),
                crypto_clock: HertzU32::MHz(160),
            },
        }
    }
}

#[cfg(esp32h2)]
impl ClockControl {
    /// Configure the CPU clock speed.
    pub(crate) fn configure(cpu_clock_speed: CpuClock) -> ClockControl {
        let xtal_freq = XtalClock::RtcXtalFreq32M;

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

        ClockControl {
            desired_rates: RawClocks {
                cpu_clock: cpu_clock_speed.frequency(),
                apb_clock: apb_freq.frequency(),
                xtal_clock: xtal_freq.frequency(),
                pll_48m_clock: HertzU32::MHz(48),
                crypto_clock: HertzU32::MHz(96),
                pll_96m_clock: HertzU32::MHz(96),
            },
        }
    }
}

#[cfg(esp32s2)]
impl ClockControl {
    /// Configure the CPU clock speed.
    pub(crate) fn configure(cpu_clock_speed: CpuClock) -> ClockControl {
        if cpu_clock_speed != CpuClock::default() {
            clocks_ll::set_cpu_clock(cpu_clock_speed);
        }

        ClockControl {
            desired_rates: RawClocks {
                cpu_clock: cpu_clock_speed.frequency(),
                apb_clock: HertzU32::MHz(80),
                xtal_clock: HertzU32::MHz(40),
            },
        }
    }
}

#[cfg(esp32s3)]
impl ClockControl {
    /// Configure the CPU clock speed.
    pub(crate) fn configure(cpu_clock_speed: CpuClock) -> ClockControl {
        if cpu_clock_speed != CpuClock::default() {
            clocks_ll::set_cpu_clock(cpu_clock_speed);
        }

        ClockControl {
            desired_rates: RawClocks {
                cpu_clock: cpu_clock_speed.frequency(),
                apb_clock: HertzU32::MHz(80),
                xtal_clock: HertzU32::MHz(40),
                crypto_pwm_clock: HertzU32::MHz(160),
            },
        }
    }
}
