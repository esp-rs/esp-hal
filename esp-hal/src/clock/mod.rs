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
#![doc = crate::before_snippet!()]
//! use esp_hal::clock::CpuClock;
//!
//! // Initialize with the highest possible frequency for this chip
//! let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
//! let peripherals = esp_hal::init(config);
//! # Ok(())
//! # }
//! ```

use core::{marker::PhantomData, sync::atomic::Ordering};

use portable_atomic::AtomicU8;

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
    /// Use the highest possible frequency for a particular chip.
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
        if esp_config::esp_config_str!("ESP_HAL_CONFIG_XTAL_FREQUENCY") == "auto" {
            if let Some(clocks) = Self::try_get() {
                return clocks.xtal_clock;
            }
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

/// Split modem clock controllers
#[cfg(any(bt, ieee802154, wifi))]
#[instability::unstable]
pub struct ModemClockControllers<'d> {
    /// Modem clock controller for Wi-Fi
    #[cfg(wifi)]
    pub wifi: WiFiClockController<'d>,
    /// Modem clock controller for Bluetooth
    #[cfg(bt)]
    pub bt: BtClockController<'d>,
    /// Modem clock controller for IEEE 802.15.4
    #[cfg(ieee802154)]
    pub ieee802154: Ieee802154ClockController<'d>
}

/// Control the radio peripheral clocks
#[cfg(any(bt, ieee802154, wifi))]
#[instability::unstable]
pub struct RadioClockController<'d> {
    _rcc: crate::peripherals::RADIO_CLK<'d>,
}

#[cfg(any(bt, ieee802154, wifi))]
impl<'d> RadioClockController<'d> {
    /// Create a new instance of the radio clock controller
    #[instability::unstable]
    pub fn new(rcc: crate::peripherals::RADIO_CLK<'d>) -> Self {
        Self { _rcc: rcc }
    }

    /// Enable the PHY clocks
    #[instability::unstable]
    #[cfg(phy)]
    #[inline]
    pub fn enable_phy(&mut self, enable: bool) {
        clocks_ll::enable_phy(enable);
    }

    /// Enable the Bluetooth clocks
    #[instability::unstable]
    #[cfg(bt)]
    #[inline]
    pub fn enable_bt(&mut self, enable: bool) {
        clocks_ll::enable_bt(enable);
    }

    /// Enable the WiFi clocks
    #[instability::unstable]
    #[cfg(wifi)]
    #[inline]
    pub fn enable_wifi(&mut self, enable: bool) {
        clocks_ll::enable_wifi(enable);
    }

    /// Enable the IEEE 802.15.4 peripheral clocks
    #[instability::unstable]
    #[cfg(ieee802154)]
    #[inline]
    pub fn enable_ieee802154(&mut self, enable: bool) {
        clocks_ll::enable_ieee802154(enable);
    }

    /// Reset the Wi-Fi MAC
    #[instability::unstable]
    #[inline]
    pub fn reset_wifi_mac(&mut self) {
        clocks_ll::reset_wifi_mac();
    }

    /// Do any common initial initialization needed
    #[instability::unstable]
    #[inline]
    pub fn init_clocks(&mut self) {
        clocks_ll::init_clocks();
    }

    /// Initialize BLE RTC clocks
    #[instability::unstable]
    #[inline]
    pub fn ble_rtc_clk_init(&mut self) {
        clocks_ll::ble_rtc_clk_init();
    }

    /// Reset the Bluetooth Resolvable Private Address (RPA).
    #[instability::unstable]
    #[inline]
    pub fn reset_rpa(&mut self) {
        clocks_ll::reset_rpa();
    }
    /// Split the radio clock controller into individual modem clock controllers.
    #[instability::unstable]
    #[inline]
    pub fn split(self) -> ModemClockControllers<'d> {
        ModemClockControllers {
            #[cfg(wifi)]
            wifi: WiFiClockController { _phantom: PhantomData },
            #[cfg(bt)]
            bt: BtClockController { _phantom: PhantomData },
            #[cfg(ieee802154)]
            ieee802154: Ieee802154ClockController { _phantom: PhantomData }
        }
    }
}

#[cfg(any(bt, ieee802154, wifi))]
#[instability::unstable]
#[derive(Clone, Copy)]
#[doc(hidden)]
/// These are all modems with bitmasks as their representation.
pub enum Modem {
    #[cfg(wifi)]
    WiFi       = 0,
    #[cfg(bt)]
    Bt         = 1,
    #[cfg(ieee802154)]
    Ieee802154 = 2,
}

#[cfg(any(bt, ieee802154, wifi))]
/// Tracks which modems currently request an active PHY clock.
static PHY_CLOCK_REF_COUNTERS: critical_section::Mutex<[AtomicU8; 3]> = critical_section::Mutex::new([const { AtomicU8::new(0) }; 3]);

// These functions are moved out of the trait to prevent monomorphization for
// every modem clock controller. If that doens't really make sense, this can be
// moved back.

#[cfg(any(bt, ieee802154, wifi))]
fn enable_phy_clock_internal(modem: Modem) {
    critical_section::with(|cs| {
        let phy_clock_refs = PHY_CLOCK_REF_COUNTERS.borrow(cs);
        if phy_clock_refs[modem as usize].fetch_add(1, Ordering::Relaxed) == 0 {
            clocks_ll::enable_phy(true);
        }
    });
}
#[cfg(any(bt, ieee802154, wifi))]
fn disable_phy_clock_internal(modem: Modem) {
    critical_section::with(|cs| {
        let mut phy_clock_refs_present = false;

        for (i, modem_phy_clock_ref_counter) in PHY_CLOCK_REF_COUNTERS.borrow(cs).iter().enumerate() {
            let mut modem_phy_clock_ref_count = modem_phy_clock_ref_counter.load(Ordering::Relaxed);

            if i == modem as usize {
                modem_phy_clock_ref_count = modem_phy_clock_ref_count.saturating_sub(1);
                modem_phy_clock_ref_counter.store(modem_phy_clock_ref_count, Ordering::Relaxed);
            }
            if modem_phy_clock_ref_count != 0 {
                phy_clock_refs_present = true;
            }
        }
        if !phy_clock_refs_present {
            clocks_ll::enable_phy(false);
        }
    });
}

#[cfg(any(bt, ieee802154, wifi))]
#[instability::unstable]
/// This trait provides common functionality for all
pub trait ModemClockController: Sealed {
    #[doc(hidden)]
    const MODEM: Modem;

    /// Enable the modem clock for this controller.
    fn enable_modem_clock(&mut self, enable: bool);

    /// Enable the PHY clock.
    ///
    /// If disabling the PHY clock is requested, but another modem currently
    /// relies on the PHY clock being active, this won't disable the clock.
    fn enable_phy_clock(&mut self, enable: bool) {
        if enable {
            enable_phy_clock_internal(Self::MODEM);
        } else {
            disable_phy_clock_internal(Self::MODEM);
        }
    }

    /// Do any common initial initialization needed
    #[instability::unstable]
    #[inline]
    fn init_clocks(&mut self) {
        clocks_ll::init_clocks();
    }
}
#[cfg(wifi)]
#[instability::unstable]
/// Controller for the Wi-Fi modem clock.
pub struct WiFiClockController<'d> {
    _phantom: PhantomData<&'d ()>,
}
#[cfg(wifi)]
impl Sealed for WiFiClockController<'_> {}
#[cfg(wifi)]
impl ModemClockController for WiFiClockController<'_> {
    const MODEM: Modem = Modem::WiFi;
    fn enable_modem_clock(&mut self, enable: bool) {
        clocks_ll::enable_wifi(enable);
    }
}
#[cfg(bt)]
#[instability::unstable]
impl WiFiClockController<'_> {
    /// Reset the Wi-Fi MAC
    pub fn reset_wifi_mac(&mut self) {
        clocks_ll::reset_wifi_mac();
    }
}

#[cfg(bt)]
#[instability::unstable]
/// Controller for the Bluetooth / BLE modem clock.
pub struct BtClockController<'d> {
    _phantom: PhantomData<&'d ()>,
}
#[cfg(bt)]
impl Sealed for BtClockController<'_> {}
#[cfg(bt)]
impl ModemClockController for BtClockController<'_> {
    const MODEM: Modem = Modem::Bt;
    fn enable_modem_clock(&mut self, enable: bool) {
        clocks_ll::enable_bt(enable);
    }
}
#[cfg(bt)]
impl BtClockController<'_> {
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
/// Controller for the IEEE 802.15.4 modem clock.
pub struct Ieee802154ClockController<'d> {
    _phantom: PhantomData<&'d ()>,
}
#[cfg(ieee802154)]
impl Sealed for Ieee802154ClockController<'_> {}
#[cfg(ieee802154)]
impl ModemClockController for Ieee802154ClockController<'_> {
    const MODEM: Modem = Modem::Ieee802154;
    fn enable_modem_clock(&mut self, enable: bool) {
        clocks_ll::enable_ieee802154(enable);
    }
}
