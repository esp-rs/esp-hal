//! # Clock Control
use fugit::MegahertzU32;

use crate::system::SystemClockControl;

/// Frozen clock frequencies
///
/// The existence of this value indicates that the clock configuration can no
/// longer be changed
pub struct Clocks {
    _private: (),
    pub cpu_clock: MegahertzU32,
    pub apb_clock: MegahertzU32,
    pub xtal_clock: MegahertzU32,
    pub i2c_clock: MegahertzU32,
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
    pub cpu_clock: MegahertzU32,
    pub apb_clock: MegahertzU32,
    pub xtal_clock: MegahertzU32,
    pub i2c_clock: MegahertzU32,
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
    /// Use what is considered the default settings after boot.
    #[cfg(feature = "esp32c3")]
    #[allow(unused)]
    pub fn boot_defaults(clock_control: SystemClockControl) -> ClockControl {
        ClockControl {
            _private: (),
            desired_rates: RawClocks {
                cpu_clock: MegahertzU32::MHz(80),
                apb_clock: MegahertzU32::MHz(80),
                xtal_clock: MegahertzU32::MHz(40),
                i2c_clock: MegahertzU32::MHz(40),
            },
        }
    }

    #[cfg(feature = "esp32")]
    #[allow(unused)]
    pub fn boot_defaults(clock_control: SystemClockControl) -> ClockControl {
        ClockControl {
            _private: (),
            desired_rates: RawClocks {
                cpu_clock: MegahertzU32::MHz(80),
                apb_clock: MegahertzU32::MHz(80),
                xtal_clock: MegahertzU32::MHz(40),
                i2c_clock: MegahertzU32::MHz(80),
            },
        }
    }

    #[cfg(feature = "esp32s2")]
    #[allow(unused)]
    pub fn boot_defaults(clock_control: SystemClockControl) -> ClockControl {
        ClockControl {
            _private: (),
            desired_rates: RawClocks {
                cpu_clock: MegahertzU32::MHz(80),
                apb_clock: MegahertzU32::MHz(80),
                xtal_clock: MegahertzU32::MHz(40),
                i2c_clock: MegahertzU32::MHz(80),
            },
        }
    }

    #[cfg(feature = "esp32s3")]
    #[allow(unused)]
    pub fn boot_defaults(clock_control: SystemClockControl) -> ClockControl {
        ClockControl {
            _private: (),
            desired_rates: RawClocks {
                cpu_clock: MegahertzU32::MHz(80),
                apb_clock: MegahertzU32::MHz(80),
                xtal_clock: MegahertzU32::MHz(40),
                i2c_clock: MegahertzU32::MHz(40),
            },
        }
    }

    /// Applies the clock configuration and returns a Clocks struct that
    /// signifies that the clocks are frozen, and contains the frequencies
    /// used. After this function is called, the clocks can not change
    pub fn freeze(self) -> Clocks {
        Clocks::from_raw_clocks(self.desired_rates)
    }
}
