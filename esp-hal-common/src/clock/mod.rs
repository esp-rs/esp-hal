//! # Clock Control
use fugit::HertzU32;

use crate::{
    peripheral::{Peripheral, PeripheralRef},
    system::SystemClockControl,
};

#[cfg_attr(esp32, path = "clocks_ll/esp32.rs")]
#[cfg_attr(esp32c2, path = "clocks_ll/esp32c2.rs")]
#[cfg_attr(esp32c3, path = "clocks_ll/esp32c3.rs")]
#[cfg_attr(esp32s2, path = "clocks_ll/esp32s2.rs")]
#[cfg_attr(esp32s3, path = "clocks_ll/esp32s3.rs")]
mod clocks_ll;

pub trait Clock {
    fn frequency(&self) -> HertzU32;

    fn mhz(&self) -> u32 {
        self.frequency().to_MHz()
    }

    fn hz(&self) -> u32 {
        self.frequency().to_Hz()
    }
}

/// CPU clock speed
#[derive(Debug, Clone, Copy)]
pub enum CpuClock {
    Clock80MHz,
    #[cfg(esp32c2)]
    Clock120MHz,
    #[cfg(not(esp32c2))]
    Clock160MHz,
    #[cfg(not(any(esp32c2, esp32c3)))]
    Clock240MHz,
}

#[allow(dead_code)]
impl Clock for CpuClock {
    fn frequency(&self) -> HertzU32 {
        match self {
            CpuClock::Clock80MHz => HertzU32::MHz(80),
            #[cfg(esp32c2)]
            CpuClock::Clock120MHz => HertzU32::MHz(120),
            #[cfg(not(esp32c2))]
            CpuClock::Clock160MHz => HertzU32::MHz(160),
            #[cfg(not(any(esp32c2, esp32c3)))]
            CpuClock::Clock240MHz => HertzU32::MHz(240),
        }
    }
}

#[allow(unused)]
#[derive(Debug, Clone, Copy)]
pub(crate) enum XtalClock {
    #[cfg(esp32)]
    RtcXtalFreq24M,
    #[cfg(any(esp32, esp32c2))]
    RtcXtalFreq26M,
    #[cfg(any(esp32c3, esp32s3))]
    RtcXtalFreq32M,
    RtcXtalFreq40M,
    RtcXtalFreqOther(u32),
}

impl Clock for XtalClock {
    fn frequency(&self) -> HertzU32 {
        match self {
            #[cfg(esp32)]
            XtalClock::RtcXtalFreq24M => HertzU32::MHz(24),
            #[cfg(any(esp32, esp32c2))]
            XtalClock::RtcXtalFreq26M => HertzU32::MHz(26),
            #[cfg(any(esp32c3, esp32s3))]
            XtalClock::RtcXtalFreq32M => HertzU32::MHz(32),
            XtalClock::RtcXtalFreq40M => HertzU32::MHz(40),
            XtalClock::RtcXtalFreqOther(mhz) => HertzU32::MHz(*mhz),
        }
    }
}

#[allow(unused)]
#[derive(Debug, Clone, Copy)]
pub(crate) enum PllClock {
    #[cfg(not(esp32c2))]
    Pll320MHz,
    Pll480MHz,
}

#[allow(unused)]
#[derive(Debug, Clone, Copy)]
pub(crate) enum ApbClock {
    ApbFreq40MHz,
    ApbFreq80MHz,
    ApbFreqOther(u32),
}

impl Clock for ApbClock {
    fn frequency(&self) -> HertzU32 {
        match self {
            ApbClock::ApbFreq40MHz => HertzU32::MHz(40),
            ApbClock::ApbFreq80MHz => HertzU32::MHz(80),
            ApbClock::ApbFreqOther(mhz) => HertzU32::MHz(*mhz),
        }
    }
}

/// Frozen clock frequencies
///
/// The existence of this value indicates that the clock configuration can no
/// longer be changed
pub struct Clocks<'d> {
    _private: PeripheralRef<'d, SystemClockControl>,
    pub cpu_clock: HertzU32,
    pub apb_clock: HertzU32,
    pub xtal_clock: HertzU32,
    pub i2c_clock: HertzU32,
    #[cfg(esp32)]
    pub pwm_clock: HertzU32,
    #[cfg(esp32s3)]
    pub crypto_pwm_clock: HertzU32,
    // TODO chip specific additional ones as needed
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
            i2c_clock: raw_clocks.i2c_clock,
            #[cfg(esp32)]
            pwm_clock: raw_clocks.pwm_clock,
            #[cfg(esp32s3)]
            crypto_pwm_clock: raw_clocks.crypto_pwm_clock,
        }
    }
}

#[doc(hidden)]
pub struct RawClocks {
    pub cpu_clock: HertzU32,
    pub apb_clock: HertzU32,
    pub xtal_clock: HertzU32,
    pub i2c_clock: HertzU32,
    #[cfg(esp32)]
    pub pwm_clock: HertzU32,
    #[cfg(esp32s3)]
    pub crypto_pwm_clock: HertzU32,
    // TODO chip specific additional ones as needed
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
    #[allow(unused)]
    pub fn boot_defaults(
        clock_control: impl Peripheral<P = SystemClockControl> + 'd,
    ) -> ClockControl<'d> {
        #[cfg(feature = "esp32_40mhz")]
        return ClockControl {
            _private: clock_control.into_ref(),
            desired_rates: RawClocks {
                cpu_clock: HertzU32::MHz(80),
                apb_clock: HertzU32::MHz(80),
                xtal_clock: HertzU32::MHz(40),
                i2c_clock: HertzU32::MHz(80),
                pwm_clock: HertzU32::MHz(160),
            },
        };

        #[cfg(feature = "esp32_26mhz")]
        return ClockControl {
            _private: clock_control.into_ref(),
            desired_rates: RawClocks {
                cpu_clock: HertzU32::MHz(80),
                apb_clock: HertzU32::MHz(80),
                xtal_clock: HertzU32::MHz(26),
                i2c_clock: HertzU32::MHz(80),
                pwm_clock: HertzU32::MHz(160),
            },
        };
    }

    /// Configure the CPU clock speed.
    #[allow(unused)]
    pub fn configure(
        clock_control: impl Peripheral<P = SystemClockControl> + 'd,
        cpu_clock_speed: CpuClock,
    ) -> ClockControl<'d> {
        // like NuttX use 40M hardcoded - if it turns out to be a problem
        // we will take care then
        #[cfg(feature = "esp32_40mhz")]
        let xtal_freq = XtalClock::RtcXtalFreq40M;
        #[cfg(feature = "esp32_26mhz")]
        let xtal_freq = XtalClock::RtcXtalFreq26M;
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
                i2c_clock: HertzU32::MHz(40),
                // The docs are unclear here. pwm_clock seems to be tied to clocks.apb_clock
                // while simultaneously being fixed at 160 MHz.
                // Testing showed 160 MHz to be correct for current clock configurations.
                pwm_clock: HertzU32::MHz(160),
            },
        }
    }
}

#[cfg(esp32c2)]
impl<'d> ClockControl<'d> {
    /// Use what is considered the default settings after boot.
    #[allow(unused)]
    pub fn boot_defaults(
        clock_control: impl Peripheral<P = SystemClockControl> + 'd,
    ) -> ClockControl<'d> {
        #[cfg(feature = "esp32c2_40mhz")]
        return ClockControl {
            _private: clock_control.into_ref(),
            desired_rates: RawClocks {
                cpu_clock: HertzU32::MHz(80),
                apb_clock: HertzU32::MHz(40),
                xtal_clock: HertzU32::MHz(40),
                i2c_clock: HertzU32::MHz(40),
            },
        };

        #[cfg(feature = "esp32c2_26mhz")]
        return ClockControl {
            _private: clock_control.into_ref(),
            desired_rates: RawClocks {
                cpu_clock: HertzU32::MHz(80),
                apb_clock: HertzU32::MHz(40),
                xtal_clock: HertzU32::MHz(26),
                i2c_clock: HertzU32::MHz(26),
            },
        };
    }

    /// Configure the CPU clock speed.
    #[allow(unused)]
    pub fn configure(
        clock_control: impl Peripheral<P = SystemClockControl> + 'd,
        cpu_clock_speed: CpuClock,
    ) -> ClockControl<'d> {
        let apb_freq;
        #[cfg(feature = "esp32c2_40mhz")]
        let xtal_freq = XtalClock::RtcXtalFreq40M;
        #[cfg(feature = "esp32c2_26mhz")]
        let xtal_freq = XtalClock::RtcXtalFreq26M;
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
                i2c_clock: HertzU32::MHz(40),
            },
        }
    }
}

#[cfg(esp32c3)]
impl<'d> ClockControl<'d> {
    /// Use what is considered the default settings after boot.
    #[allow(unused)]
    pub fn boot_defaults(
        clock_control: impl Peripheral<P = SystemClockControl> + 'd,
    ) -> ClockControl<'d> {
        ClockControl {
            _private: clock_control.into_ref(),
            desired_rates: RawClocks {
                cpu_clock: HertzU32::MHz(80),
                apb_clock: HertzU32::MHz(80),
                xtal_clock: HertzU32::MHz(40),
                i2c_clock: HertzU32::MHz(40),
            },
        }
    }

    /// Configure the CPU clock speed.
    #[allow(unused)]
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
                i2c_clock: HertzU32::MHz(40),
            },
        }
    }
}

#[cfg(esp32s2)]
impl<'d> ClockControl<'d> {
    /// Use what is considered the default settings after boot.
    #[allow(unused)]
    pub fn boot_defaults(
        clock_control: impl Peripheral<P = SystemClockControl> + 'd,
    ) -> ClockControl<'d> {
        ClockControl {
            _private: clock_control.into_ref(),
            desired_rates: RawClocks {
                cpu_clock: HertzU32::MHz(80),
                apb_clock: HertzU32::MHz(80),
                xtal_clock: HertzU32::MHz(40),
                i2c_clock: HertzU32::MHz(80),
            },
        }
    }

    /// Configure the CPU clock speed.
    #[allow(unused)]
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
                i2c_clock: HertzU32::MHz(40),
            },
        }
    }
}

#[cfg(esp32s3)]
impl<'d> ClockControl<'d> {
    /// Use what is considered the default settings after boot.
    #[allow(unused)]
    pub fn boot_defaults(
        clock_control: impl Peripheral<P = SystemClockControl> + 'd,
    ) -> ClockControl<'d> {
        ClockControl {
            _private: clock_control.into_ref(),
            desired_rates: RawClocks {
                cpu_clock: HertzU32::MHz(80),
                apb_clock: HertzU32::MHz(80),
                xtal_clock: HertzU32::MHz(40),
                i2c_clock: HertzU32::MHz(40),
                crypto_pwm_clock: HertzU32::MHz(160),
            },
        }
    }

    /// Configure the CPU clock speed.
    #[allow(unused)]
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
                i2c_clock: HertzU32::MHz(40),
                crypto_pwm_clock: HertzU32::MHz(160),
            },
        }
    }
}
