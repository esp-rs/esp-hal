//! # Clock Control
use fugit::HertzU32;

use crate::system::SystemClockControl;

#[cfg_attr(esp32, path = "clocks_ll/esp32.rs")]
#[cfg_attr(esp32c2, path = "clocks_ll/esp32c2.rs")]
#[cfg_attr(esp32c3, path = "clocks_ll/esp32c3.rs")]
#[cfg_attr(esp32s2, path = "clocks_ll/esp32s2.rs")]
#[cfg_attr(esp32s3, path = "clocks_ll/esp32s3.rs")]
#[cfg_attr(feature = "esp32", path = "clocks_ll/esp32.rs")]
#[cfg_attr(feature = "esp32c2", path = "clocks_ll/esp32c2.rs")]
#[cfg_attr(feature = "esp32c3", path = "clocks_ll/esp32c3.rs")]
#[cfg_attr(feature = "esp32s2", path = "clocks_ll/esp32s2.rs")]
#[cfg_attr(feature = "esp32s3", path = "clocks_ll/esp32s3.rs")]
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
    Clock160MHz,
    #[cfg(not(any(esp32c2, esp32c3)))]
    #[cfg(not(any(feature = "esp32c2", feature = "esp32c3")))]
    Clock240MHz,
}

#[allow(dead_code)]
impl Clock for CpuClock {
    fn frequency(&self) -> HertzU32 {
        match self {
            CpuClock::Clock80MHz => HertzU32::MHz(80),
            CpuClock::Clock160MHz => HertzU32::MHz(160),
            #[cfg(not(any(esp32c2, esp32c3)))]
            #[cfg(not(any(feature = "esp32c2", feature = "esp32c3")))]
            CpuClock::Clock240MHz => HertzU32::MHz(240),
        }
    }
}

#[allow(unused)]
#[derive(Debug, Clone, Copy)]
pub(crate) enum XtalClock {
    RtcXtalFreq40M,
    #[cfg(esp32)]
    RtcXtalFreq26M,
    #[cfg(esp32)]
    RtcXtalFreq24M,
    #[cfg(any(esp32c2, esp32c3, esp32s3))]
    #[cfg(any(feature = "esp32c2", feature = "esp32c3", feature = "esp32s3"))]
    RtcXtalFreq32M,
    RtcXtalFreqOther(u32),
}

impl Clock for XtalClock {
    fn frequency(&self) -> HertzU32 {
        match self {
            XtalClock::RtcXtalFreq40M => HertzU32::MHz(40),
            #[cfg(esp32)]
            XtalClock::RtcXtalFreq26M => HertzU32::MHz(26),
            #[cfg(esp32)]
            XtalClock::RtcXtalFreq24M => HertzU32::MHz(24),
            #[cfg(any(esp32c2, esp32c3, esp32s3))]
            #[cfg(any(feature = "esp32c2", feature = "esp32c3", feature = "esp32s3"))]
            XtalClock::RtcXtalFreq32M => HertzU32::MHz(32),
            XtalClock::RtcXtalFreqOther(mhz) => HertzU32::MHz(*mhz),
        }
    }
}

#[allow(unused)]
#[derive(Debug, Clone, Copy)]
pub(crate) enum PllClock {
    Pll320MHz,
    Pll480MHz,
}

#[allow(unused)]
#[derive(Debug, Clone, Copy)]
pub(crate) enum ApbClock {
    ApbFreq80MHz,
    ApbFreqOther(u32),
}

impl Clock for ApbClock {
    fn frequency(&self) -> HertzU32 {
        match self {
            ApbClock::ApbFreq80MHz => HertzU32::MHz(80),
            ApbClock::ApbFreqOther(mhz) => HertzU32::MHz(*mhz),
        }
    }
}

/// Frozen clock frequencies
///
/// The existence of this value indicates that the clock configuration can no
/// longer be changed
pub struct Clocks {
    _private: (),
    pub cpu_clock: HertzU32,
    pub apb_clock: HertzU32,
    pub xtal_clock: HertzU32,
    pub i2c_clock: HertzU32,
    // TODO chip specific additional ones as needed
}

#[doc(hidden)]
impl Clocks {
    /// This should not be used in user code.
    /// The whole point this exists is make it possible to have other crates
    /// (i.e. esp-wifi) create `Clocks`
    #[doc(hidden)]
    pub fn from_raw_clocks(raw_clocks: RawClocks) -> Clocks {
        Self {
            _private: (),
            cpu_clock: raw_clocks.cpu_clock,
            apb_clock: raw_clocks.apb_clock,
            xtal_clock: raw_clocks.xtal_clock,
            i2c_clock: raw_clocks.i2c_clock,
        }
    }
}

#[doc(hidden)]
pub struct RawClocks {
    pub cpu_clock: HertzU32,
    pub apb_clock: HertzU32,
    pub xtal_clock: HertzU32,
    pub i2c_clock: HertzU32,
    // TODO chip specific additional ones as needed
}

/// Used to configure the frequencies of the clocks present in the chip.
///
/// After setting all frequencies, call the freeze function to apply the
/// configuration.
pub struct ClockControl {
    _private: (),
    desired_rates: RawClocks,
}

impl ClockControl {
    /// Applies the clock configuration and returns a Clocks struct that
    /// signifies that the clocks are frozen, and contains the frequencies
    /// used. After this function is called, the clocks can not change
    pub fn freeze(self) -> Clocks {
        Clocks::from_raw_clocks(self.desired_rates)
    }
}

#[cfg(esp32)]
impl ClockControl {
    /// Use what is considered the default settings after boot.
    #[allow(unused)]
    pub fn boot_defaults(clock_control: SystemClockControl) -> ClockControl {
        ClockControl {
            _private: (),
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
    pub fn configure(clock_control: SystemClockControl, cpu_clock_speed: CpuClock) -> ClockControl {
        // like NuttX use 40M hardcoded - if it turns out to be a problem
        // we will take care then
        let xtal_freq = XtalClock::RtcXtalFreq40M;
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
            _private: (),
            desired_rates: RawClocks {
                cpu_clock: cpu_clock_speed.frequency(),
                apb_clock: HertzU32::MHz(80),
                xtal_clock: HertzU32::MHz(40),
                i2c_clock: HertzU32::MHz(40),
            },
        }
    }
}

#[cfg(esp32c2)]
impl ClockControl {
    /// Use what is considered the default settings after boot.
    #[allow(unused)]
    pub fn boot_defaults(clock_control: SystemClockControl) -> ClockControl {
        ClockControl {
            _private: (),
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
    pub fn configure(clock_control: SystemClockControl, cpu_clock_speed: CpuClock) -> ClockControl {
        let apb_freq;
        let xtal_freq = XtalClock::RtcXtalFreq40M;
        let pll_freq = PllClock::Pll480MHz;

        if cpu_clock_speed.mhz() <= xtal_freq.mhz() {
            apb_freq = ApbClock::ApbFreqOther(cpu_clock_speed.mhz());
            clocks_ll::esp32c2_rtc_update_to_xtal(xtal_freq, 1);
            clocks_ll::esp32c2_rtc_apb_freq_update(apb_freq);
        } else {
            apb_freq = ApbClock::ApbFreq80MHz;
            clocks_ll::esp32c2_rtc_bbpll_enable();
            clocks_ll::esp32c2_rtc_bbpll_configure(xtal_freq, pll_freq);
            clocks_ll::esp32c2_rtc_freq_to_pll_mhz(cpu_clock_speed);
            clocks_ll::esp32c2_rtc_apb_freq_update(apb_freq);
        }

        ClockControl {
            _private: (),
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
impl ClockControl {
    /// Use what is considered the default settings after boot.
    #[allow(unused)]
    pub fn boot_defaults(clock_control: SystemClockControl) -> ClockControl {
        ClockControl {
            _private: (),
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
    pub fn configure(clock_control: SystemClockControl, cpu_clock_speed: CpuClock) -> ClockControl {
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
            _private: (),
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
impl ClockControl {
    /// Use what is considered the default settings after boot.
    #[allow(unused)]
    pub fn boot_defaults(clock_control: SystemClockControl) -> ClockControl {
        ClockControl {
            _private: (),
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
    pub fn configure(clock_control: SystemClockControl, cpu_clock_speed: CpuClock) -> ClockControl {
        clocks_ll::set_cpu_clock(cpu_clock_speed);

        ClockControl {
            _private: (),
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
impl ClockControl {
    /// Use what is considered the default settings after boot.
    #[allow(unused)]
    pub fn boot_defaults(clock_control: SystemClockControl) -> ClockControl {
        ClockControl {
            _private: (),
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
    pub fn configure(clock_control: SystemClockControl, cpu_clock_speed: CpuClock) -> ClockControl {
        clocks_ll::set_cpu_clock(cpu_clock_speed);

        ClockControl {
            _private: (),
            desired_rates: RawClocks {
                cpu_clock: cpu_clock_speed.frequency(),
                apb_clock: HertzU32::MHz(80),
                xtal_clock: HertzU32::MHz(40),
                i2c_clock: HertzU32::MHz(40),
            },
        }
    }
}
