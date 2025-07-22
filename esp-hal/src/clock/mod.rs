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

#[cfg(bt)]
use crate::peripherals::BT;
#[cfg(all(feature = "unstable", ieee802154))]
use crate::peripherals::IEEE802154;
#[cfg(wifi)]
use crate::peripherals::WIFI;
#[cfg(any(esp32, esp32c2))]
use crate::rtc_cntl::RtcClock;
use crate::{private::Sealed, time::Rate};

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
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(
    clippy::enum_variant_names,
    reason = "MHz suffix indicates physical unit."
)]
#[non_exhaustive]
pub enum CpuClock {
    /// 80MHz CPU clock
    #[cfg(not(esp32h2))]
    _80MHz  = 80,

    /// 96MHz CPU clock
    #[cfg(esp32h2)]
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

impl Default for CpuClock {
    fn default() -> Self {
        cfg_if::cfg_if! {
            if #[cfg(esp32h2)] {
                Self::_96MHz
            } else {
                // FIXME: I don't think this is correct in general?
                Self::_80MHz
            }
        }
    }
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

/// XTAL clock speed
#[instability::unstable]
#[derive(Debug, Clone, Copy)]
#[non_exhaustive]
pub enum XtalClock {
    /// 26MHz XTAL clock
    #[cfg(any(esp32, esp32c2))]
    _26M,
    /// 32MHz XTAL clock
    #[cfg(any(esp32c3, esp32h2, esp32s3))]
    _32M,
    /// 40MHz XTAL clock
    #[cfg(not(esp32h2))]
    _40M,
    /// Other XTAL clock
    Other(u32),
}

impl Clock for XtalClock {
    fn frequency(&self) -> Rate {
        match self {
            #[cfg(any(esp32, esp32c2))]
            XtalClock::_26M => Rate::from_mhz(26),
            #[cfg(any(esp32c3, esp32h2, esp32s3))]
            XtalClock::_32M => Rate::from_mhz(32),
            #[cfg(not(esp32h2))]
            XtalClock::_40M => Rate::from_mhz(40),
            XtalClock::Other(mhz) => Rate::from_mhz(*mhz),
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
        critical_section::with(|_| {
            unsafe { ACTIVE_CLOCKS = Some(Self::configure(cpu_clock_speed)) };
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
}

#[cfg(esp32)]
impl Clocks {
    fn measure_xtal_frequency() -> XtalClock {
        if esp_config::esp_config_str!("ESP_HAL_CONFIG_XTAL_FREQUENCY") == "auto" {
            if RtcClock::estimate_xtal_frequency() > 33 {
                XtalClock::_40M
            } else {
                XtalClock::_26M
            }
        } else {
            const {
                let frequency_conf = esp_config::esp_config_str!("ESP_HAL_CONFIG_XTAL_FREQUENCY");
                match frequency_conf.as_bytes() {
                    b"auto" => XtalClock::Other(0), // Can't be `unreachable!` due to const eval.
                    b"26" => XtalClock::_26M,
                    b"40" => XtalClock::_40M,
                    _ => XtalClock::Other(esp_config::esp_config_int_parse!(u32, frequency_conf)),
                }
            }
        }
    }

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
    fn measure_xtal_frequency() -> XtalClock {
        if esp_config::esp_config_str!("ESP_HAL_CONFIG_XTAL_FREQUENCY") == "auto" {
            if RtcClock::estimate_xtal_frequency() > 33 {
                XtalClock::_40M
            } else {
                XtalClock::_26M
            }
        } else {
            const {
                let frequency_conf = esp_config::esp_config_str!("ESP_HAL_CONFIG_XTAL_FREQUENCY");
                match frequency_conf.as_bytes() {
                    b"auto" => XtalClock::Other(0), // Can't be `unreachable!` due to const eval.
                    b"26" => XtalClock::_26M,
                    b"40" => XtalClock::_40M,
                    _ => XtalClock::Other(esp_config::esp_config_int_parse!(u32, frequency_conf)),
                }
            }
        }
    }

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
    fn measure_xtal_frequency() -> XtalClock {
        XtalClock::_40M
    }

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
    fn measure_xtal_frequency() -> XtalClock {
        XtalClock::_40M
    }

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
    fn measure_xtal_frequency() -> XtalClock {
        XtalClock::_32M
    }

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
    fn measure_xtal_frequency() -> XtalClock {
        XtalClock::_40M
    }

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
    fn measure_xtal_frequency() -> XtalClock {
        XtalClock::_40M
    }

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
static PHY_CLOCK_REF_COUNTER: critical_section::Mutex<Cell<u8>> =
    critical_section::Mutex::new(Cell::new(0));
#[cfg(any(bt, ieee802154, wifi))]
fn increase_phy_clock_ref_count_internal() {
    critical_section::with(|cs| {
        let phy_clock_ref_counter = PHY_CLOCK_REF_COUNTER.borrow(cs);
        let phy_clock_ref_count = phy_clock_ref_counter.get();

        if phy_clock_ref_count == 0 {
            clocks_ll::enable_phy(true);
        }

        phy_clock_ref_counter.set(phy_clock_ref_count + 1);
    });
}
#[cfg(any(bt, ieee802154, wifi))]
fn decrease_phy_clock_ref_count_internal() {
    critical_section::with(|cs| {
        let phy_clock_ref_counter = PHY_CLOCK_REF_COUNTER.borrow(cs);

        let new_phy_clock_ref_count = unwrap!(
            phy_clock_ref_counter.get().checked_sub(1),
            "PHY clock ref count underflowed. Either you forgot a PhyClockGuard, or used ModemClockController::decrease_phy_clock_ref_count incorrectly."
        );

        if new_phy_clock_ref_count == 0 {
            clocks_ll::enable_phy(false);
        }
        phy_clock_ref_counter.set(new_phy_clock_ref_count);
    });
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
/// This trait provides common functionality for all
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
