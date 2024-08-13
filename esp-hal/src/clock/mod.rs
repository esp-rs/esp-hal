//! # Clock Control
//!
//! ## Overview
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
//! Proper clock configuration is essential for the correct functioning of the
//! microcontroller and its peripherals.
//!
//! The `Clock` driver supports configuring multiple clocks, including:
//!   * CPU clock
//!   * APB (Advanced Peripheral Bus) clock
//!   * XTAL (External Crystal) clock
//!   * PLL (Phase Lock Loop) clock
//!
//! and other specific clocks based on the ESP microcontroller's architecture.
//!
//! The `CPU clock` is responsible for defining the speed at which the central
//! processing unit (CPU) operates. This driver provides predefined options for
//! different CPU clock speeds, such
//!   * 80 MHz
//!   * 96 MHz
//!   * 120 MHz
//!   * 160 MHz
//!   * 240 MHz
//!
//! and others, depending on the microcontroller model.
//!
//! ### Clock Control
//! The `ClockControl` struct allows users to configure the desired clock
//! frequencies before applying them. It offers flexibility in selecting
//! appropriate clock frequencies based on specific application requirements.
//!
//! ### Frozen Clock Frequencies
//! Once the clock configuration is applied using the `freeze` function of the
//! ClockControl struct, the clock frequencies become `frozen` and cannot be
//! changed. The `Clocks` struct is returned after freezing, providing read-only
//! access to the configured clock frequencies.
//!
//! ## Examples
//! ### Initialize With Different Clock Frequencies
//! ```rust, no_run
//! # #![no_std]
//! # use esp_hal::peripherals::Peripherals;
//! # use esp_hal::clock::ClockControl;
//! # use esp_hal::system::SystemControl;
//! # #[panic_handler]
//! # fn panic(_ : &core::panic::PanicInfo) -> ! {
//! #     loop {}
//! # }
//! # fn main() {
//! #   let peripherals = Peripherals::take();
//! #   let system = SystemControl::new(peripherals.SYSTEM);
//! // Initialize with the highest possible frequency for this chip
//! let clocks = ClockControl::max(system.clock_control).freeze();
//!
//! // Initialize with custom clock frequency
//! // let clocks = ClockControl::configure(system.clock_control, CpuClock::Clock160MHz).freeze();
//! //
//! // Initialize with default clock frequency for this chip
//! // let clocks = ClockControl::boot_defaults(system.clock_control).freeze();
//! # }
//! ```

use fugit::HertzU32;

#[cfg(any(esp32, esp32c2))]
use crate::rtc_cntl::RtcClock;
use crate::{
    peripheral::{Peripheral, PeripheralRef},
    system::SystemClockControl,
};

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
#[derive(Debug, Clone, Copy)]
pub enum CpuClock {
    /// 80MHz CPU clock
    #[cfg(not(esp32h2))]
    Clock80MHz,
    /// 96MHz CPU clock
    #[cfg(esp32h2)]
    Clock96MHz,
    /// 120MHz CPU clock
    #[cfg(esp32c2)]
    Clock120MHz,
    /// 160MHz CPU clock
    #[cfg(not(any(esp32c2, esp32h2)))]
    Clock160MHz,
    /// 240MHz CPU clock
    #[cfg(xtensa)]
    Clock240MHz,
}

#[allow(dead_code)]
impl Clock for CpuClock {
    fn frequency(&self) -> HertzU32 {
        match self {
            #[cfg(not(esp32h2))]
            CpuClock::Clock80MHz => HertzU32::MHz(80),
            #[cfg(esp32h2)]
            CpuClock::Clock96MHz => HertzU32::MHz(96),
            #[cfg(esp32c2)]
            CpuClock::Clock120MHz => HertzU32::MHz(120),
            #[cfg(not(any(esp32c2, esp32h2)))]
            CpuClock::Clock160MHz => HertzU32::MHz(160),
            #[cfg(xtensa)]
            CpuClock::Clock240MHz => HertzU32::MHz(240),
        }
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
/// longer be changed
pub struct Clocks<'d> {
    _private: PeripheralRef<'d, SystemClockControl>,
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

#[doc(hidden)]
impl<'d> Clocks<'d> {
    /// This should not be used in user code.
    /// The whole point this exists is make it possible to have other crates
    /// (i.e. esp-wifi) create `Clocks`
    #[doc(hidden)]
    pub fn from_raw_clocks(
        system_clock_control: PeripheralRef<'d, SystemClockControl>,
        raw_clocks: RawClocks,
    ) -> Clocks<'d> {
        Self {
            _private: system_clock_control,
            cpu_clock: raw_clocks.cpu_clock,
            apb_clock: raw_clocks.apb_clock,
            xtal_clock: raw_clocks.xtal_clock,
            #[cfg(esp32)]
            i2c_clock: raw_clocks.i2c_clock,
            #[cfg(esp32)]
            pwm_clock: raw_clocks.pwm_clock,
            #[cfg(esp32s3)]
            crypto_pwm_clock: raw_clocks.crypto_pwm_clock,
            #[cfg(any(esp32c6, esp32h2))]
            crypto_clock: raw_clocks.crypto_clock,
            #[cfg(esp32h2)]
            pll_48m_clock: raw_clocks.pll_48m_clock,
            #[cfg(esp32h2)]
            pll_96m_clock: raw_clocks.pll_96m_clock,
        }
    }
}

#[doc(hidden)]
pub struct RawClocks {
    pub cpu_clock: HertzU32,
    pub apb_clock: HertzU32,
    pub xtal_clock: HertzU32,
    #[cfg(esp32)]
    pub i2c_clock: HertzU32,
    #[cfg(esp32)]
    pub pwm_clock: HertzU32,
    #[cfg(esp32s3)]
    pub crypto_pwm_clock: HertzU32,
    #[cfg(any(esp32c6, esp32h2))]
    pub crypto_clock: HertzU32,
    #[cfg(esp32h2)]
    pub pll_48m_clock: HertzU32,
    #[cfg(esp32h2)]
    pub pll_96m_clock: HertzU32,
}

/// Used to configure the frequencies of the clocks present in the chip.
///
/// After setting all frequencies, call the freeze function to apply the
/// configuration.
pub struct ClockControl<'d> {
    _private: PeripheralRef<'d, SystemClockControl>,
    desired_rates: RawClocks,
}

impl<'d> ClockControl<'d> {
    /// Applies the clock configuration and returns a Clocks struct that
    /// signifies that the clocks are frozen, and contains the frequencies
    /// used. After this function is called, the clocks can not change
    pub fn freeze(self) -> Clocks<'d> {
        Clocks::from_raw_clocks(self._private, self.desired_rates)
    }
}

#[cfg(esp32)]
impl<'d> ClockControl<'d> {
    /// Use what is considered the default settings after boot.
    pub fn boot_defaults(
        clock_control: impl Peripheral<P = SystemClockControl> + 'd,
    ) -> ClockControl<'d> {
        let xtal_freq = if RtcClock::estimate_xtal_frequency() > 33 {
            40
        } else {
            26
        };

        ClockControl {
            _private: clock_control.into_ref(),
            desired_rates: RawClocks {
                cpu_clock: HertzU32::MHz(80),
                apb_clock: HertzU32::MHz(80),
                xtal_clock: HertzU32::MHz(xtal_freq),
                i2c_clock: HertzU32::MHz(80),
                pwm_clock: HertzU32::MHz(160),
            },
        }
    }

    /// Configure the CPU clock speed.
    pub fn configure(
        clock_control: impl Peripheral<P = SystemClockControl> + 'd,
        cpu_clock_speed: CpuClock,
    ) -> ClockControl<'d> {
        let xtal_freq = if RtcClock::estimate_xtal_frequency() > 33 {
            XtalClock::RtcXtalFreq40M
        } else {
            XtalClock::RtcXtalFreq26M
        };

        let pll_freq = match cpu_clock_speed {
            CpuClock::Clock80MHz => PllClock::Pll320MHz,
            CpuClock::Clock160MHz => PllClock::Pll320MHz,
            CpuClock::Clock240MHz => PllClock::Pll480MHz,
        };

        clocks_ll::esp32_rtc_update_to_xtal(xtal_freq, 1);
        clocks_ll::esp32_rtc_bbpll_enable();
        clocks_ll::esp32_rtc_bbpll_configure(xtal_freq, pll_freq);
        clocks_ll::set_cpu_freq(cpu_clock_speed);

        ClockControl {
            _private: clock_control.into_ref(),
            desired_rates: RawClocks {
                cpu_clock: cpu_clock_speed.frequency(),
                apb_clock: HertzU32::MHz(80),
                xtal_clock: HertzU32::MHz(40),
                i2c_clock: HertzU32::MHz(80),
                // The docs are unclear here. pwm_clock seems to be tied to clocks.apb_clock
                // while simultaneously being fixed at 160 MHz.
                // Testing showed 160 MHz to be correct for current clock configurations.
                pwm_clock: HertzU32::MHz(160),
            },
        }
    }

    /// Use the highest possible frequency for a particular chip
    pub fn max(clock_control: impl Peripheral<P = SystemClockControl> + 'd) -> ClockControl<'d> {
        Self::configure(clock_control, CpuClock::Clock240MHz)
    }
}

#[cfg(esp32c2)]
impl<'d> ClockControl<'d> {
    /// Use what is considered the default settings after boot.
    pub fn boot_defaults(
        clock_control: impl Peripheral<P = SystemClockControl> + 'd,
    ) -> ClockControl<'d> {
        let xtal_freq = if RtcClock::estimate_xtal_frequency() > 33 {
            40
        } else {
            26
        };

        ClockControl {
            _private: clock_control.into_ref(),
            desired_rates: RawClocks {
                cpu_clock: HertzU32::MHz(80),
                apb_clock: HertzU32::MHz(40),
                xtal_clock: HertzU32::MHz(xtal_freq),
            },
        }
    }

    /// Configure the CPU clock speed.
    pub fn configure(
        clock_control: impl Peripheral<P = SystemClockControl> + 'd,
        cpu_clock_speed: CpuClock,
    ) -> ClockControl<'d> {
        let apb_freq;

        let xtal_freq = if RtcClock::estimate_xtal_frequency() > 33 {
            XtalClock::RtcXtalFreq40M
        } else {
            XtalClock::RtcXtalFreq26M
        };

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

        ClockControl {
            _private: clock_control.into_ref(),
            desired_rates: RawClocks {
                cpu_clock: cpu_clock_speed.frequency(),
                apb_clock: apb_freq.frequency(),
                xtal_clock: xtal_freq.frequency(),
            },
        }
    }

    /// Use the highest possible frequency for a particular chip
    pub fn max(clock_control: impl Peripheral<P = SystemClockControl> + 'd) -> ClockControl<'d> {
        Self::configure(clock_control, CpuClock::Clock120MHz)
    }
}

#[cfg(esp32c3)]
impl<'d> ClockControl<'d> {
    /// Use what is considered the default settings after boot.
    pub fn boot_defaults(
        clock_control: impl Peripheral<P = SystemClockControl> + 'd,
    ) -> ClockControl<'d> {
        ClockControl {
            _private: clock_control.into_ref(),
            desired_rates: RawClocks {
                cpu_clock: HertzU32::MHz(80),
                apb_clock: HertzU32::MHz(80),
                xtal_clock: HertzU32::MHz(40),
            },
        }
    }

    /// Configure the CPU clock speed.
    pub fn configure(
        clock_control: impl Peripheral<P = SystemClockControl> + 'd,
        cpu_clock_speed: CpuClock,
    ) -> ClockControl<'d> {
        let apb_freq;
        let xtal_freq = XtalClock::RtcXtalFreq40M;
        let pll_freq = PllClock::Pll480MHz;

        if cpu_clock_speed.mhz() <= xtal_freq.mhz() {
            apb_freq = ApbClock::ApbFreqOther(cpu_clock_speed.mhz());
            clocks_ll::esp32c3_rtc_update_to_xtal(xtal_freq, 1);
            clocks_ll::esp32c3_rtc_apb_freq_update(apb_freq);
        } else {
            apb_freq = ApbClock::ApbFreq80MHz;
            clocks_ll::esp32c3_rtc_bbpll_enable();
            clocks_ll::esp32c3_rtc_bbpll_configure(xtal_freq, pll_freq);
            clocks_ll::esp32c3_rtc_freq_to_pll_mhz(cpu_clock_speed);
            clocks_ll::esp32c3_rtc_apb_freq_update(apb_freq);
        }

        ClockControl {
            _private: clock_control.into_ref(),
            desired_rates: RawClocks {
                cpu_clock: cpu_clock_speed.frequency(),
                apb_clock: apb_freq.frequency(),
                xtal_clock: xtal_freq.frequency(),
            },
        }
    }

    /// Use the highest possible frequency for a particular chip
    pub fn max(clock_control: impl Peripheral<P = SystemClockControl> + 'd) -> ClockControl<'d> {
        Self::configure(clock_control, CpuClock::Clock160MHz)
    }
}

#[cfg(esp32c6)]
impl<'d> ClockControl<'d> {
    /// Use what is considered the default settings after boot.
    pub fn boot_defaults(
        clock_control: impl Peripheral<P = SystemClockControl> + 'd,
    ) -> ClockControl<'d> {
        ClockControl {
            _private: clock_control.into_ref(),
            desired_rates: RawClocks {
                cpu_clock: HertzU32::MHz(80),
                apb_clock: HertzU32::MHz(80),
                xtal_clock: HertzU32::MHz(40),
                crypto_clock: HertzU32::MHz(160),
            },
        }
    }

    /// Configure the CPU clock speed.
    pub fn configure(
        clock_control: impl Peripheral<P = SystemClockControl> + 'd,
        cpu_clock_speed: CpuClock,
    ) -> ClockControl<'d> {
        let apb_freq;
        let xtal_freq = XtalClock::RtcXtalFreq40M;
        let pll_freq = PllClock::Pll480MHz;

        if cpu_clock_speed.mhz() <= xtal_freq.mhz() {
            apb_freq = ApbClock::ApbFreqOther(cpu_clock_speed.mhz());
            clocks_ll::esp32c6_rtc_update_to_xtal(xtal_freq, 1);
            clocks_ll::esp32c6_rtc_apb_freq_update(apb_freq);
        } else {
            apb_freq = ApbClock::ApbFreq80MHz;
            clocks_ll::esp32c6_rtc_bbpll_enable();
            clocks_ll::esp32c6_rtc_bbpll_configure(xtal_freq, pll_freq);
            clocks_ll::esp32c6_rtc_freq_to_pll_mhz(cpu_clock_speed);
            clocks_ll::esp32c6_rtc_apb_freq_update(apb_freq);
        }

        ClockControl {
            _private: clock_control.into_ref(),
            desired_rates: RawClocks {
                cpu_clock: cpu_clock_speed.frequency(),
                apb_clock: apb_freq.frequency(),
                xtal_clock: xtal_freq.frequency(),
                crypto_clock: HertzU32::MHz(160),
            },
        }
    }

    /// Use the highest possible frequency for a particular chip
    pub fn max(clock_control: impl Peripheral<P = SystemClockControl> + 'd) -> ClockControl<'d> {
        Self::configure(clock_control, CpuClock::Clock160MHz)
    }
}

#[cfg(esp32h2)]
impl<'d> ClockControl<'d> {
    /// Use what is considered the default settings after boot.
    pub fn boot_defaults(
        clock_control: impl Peripheral<P = SystemClockControl> + 'd,
    ) -> ClockControl<'d> {
        ClockControl {
            _private: clock_control.into_ref(),
            desired_rates: RawClocks {
                cpu_clock: HertzU32::MHz(96),
                apb_clock: HertzU32::MHz(32),
                xtal_clock: HertzU32::MHz(32),
                pll_48m_clock: HertzU32::MHz(48),
                crypto_clock: HertzU32::MHz(96),
                pll_96m_clock: HertzU32::MHz(96),
            },
        }
    }

    /// Configure the CPU clock speed.
    pub fn configure(
        clock_control: impl Peripheral<P = SystemClockControl> + 'd,
        cpu_clock_speed: CpuClock,
    ) -> ClockControl<'d> {
        let apb_freq;
        let xtal_freq = XtalClock::RtcXtalFreq32M;
        let pll_freq = PllClock::Pll96MHz;

        if cpu_clock_speed.mhz() <= xtal_freq.mhz() {
            apb_freq = ApbClock::ApbFreqOther(cpu_clock_speed.mhz());
            clocks_ll::esp32h2_rtc_update_to_xtal(xtal_freq, 1);
            clocks_ll::esp32h2_rtc_apb_freq_update(apb_freq);
        } else {
            apb_freq = ApbClock::ApbFreq32MHz;
            clocks_ll::esp32h2_rtc_bbpll_enable();
            clocks_ll::esp32h2_rtc_bbpll_configure(xtal_freq, pll_freq);
            clocks_ll::esp32h2_rtc_freq_to_pll_mhz(cpu_clock_speed);
            clocks_ll::esp32h2_rtc_apb_freq_update(apb_freq);
        }

        ClockControl {
            _private: clock_control.into_ref(),
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

    /// Use the highest possible frequency for a particular chip
    pub fn max(clock_control: impl Peripheral<P = SystemClockControl> + 'd) -> ClockControl<'d> {
        Self::configure(clock_control, CpuClock::Clock96MHz)
    }
}

#[cfg(esp32s2)]
impl<'d> ClockControl<'d> {
    /// Use what is considered the default settings after boot.
    pub fn boot_defaults(
        clock_control: impl Peripheral<P = SystemClockControl> + 'd,
    ) -> ClockControl<'d> {
        ClockControl {
            _private: clock_control.into_ref(),
            desired_rates: RawClocks {
                cpu_clock: HertzU32::MHz(80),
                apb_clock: HertzU32::MHz(80),
                xtal_clock: HertzU32::MHz(40),
            },
        }
    }

    /// Configure the CPU clock speed.
    pub fn configure(
        clock_control: impl Peripheral<P = SystemClockControl> + 'd,
        cpu_clock_speed: CpuClock,
    ) -> ClockControl<'d> {
        clocks_ll::set_cpu_clock(cpu_clock_speed);

        ClockControl {
            _private: clock_control.into_ref(),
            desired_rates: RawClocks {
                cpu_clock: cpu_clock_speed.frequency(),
                apb_clock: HertzU32::MHz(80),
                xtal_clock: HertzU32::MHz(40),
            },
        }
    }

    /// Use the highest possible frequency for a particular chip
    pub fn max(clock_control: impl Peripheral<P = SystemClockControl> + 'd) -> ClockControl<'d> {
        Self::configure(clock_control, CpuClock::Clock240MHz)
    }
}

#[cfg(esp32s3)]
impl<'d> ClockControl<'d> {
    /// Use what is considered the default settings after boot.
    pub fn boot_defaults(
        clock_control: impl Peripheral<P = SystemClockControl> + 'd,
    ) -> ClockControl<'d> {
        ClockControl {
            _private: clock_control.into_ref(),
            desired_rates: RawClocks {
                cpu_clock: HertzU32::MHz(80),
                apb_clock: HertzU32::MHz(80),
                xtal_clock: HertzU32::MHz(40),
                crypto_pwm_clock: HertzU32::MHz(160),
            },
        }
    }

    /// Configure the CPU clock speed.
    pub fn configure(
        clock_control: impl Peripheral<P = SystemClockControl> + 'd,
        cpu_clock_speed: CpuClock,
    ) -> ClockControl<'d> {
        clocks_ll::set_cpu_clock(cpu_clock_speed);

        ClockControl {
            _private: clock_control.into_ref(),
            desired_rates: RawClocks {
                cpu_clock: cpu_clock_speed.frequency(),
                apb_clock: HertzU32::MHz(80),
                xtal_clock: HertzU32::MHz(40),
                crypto_pwm_clock: HertzU32::MHz(160),
            },
        }
    }

    /// Use the highest possible frequency for a particular chip
    pub fn max(clock_control: impl Peripheral<P = SystemClockControl> + 'd) -> ClockControl<'d> {
        Self::configure(clock_control, CpuClock::Clock240MHz)
    }
}
