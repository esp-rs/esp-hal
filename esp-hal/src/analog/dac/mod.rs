#![cfg_attr(docsrs, procmacros::doc_replace(
    "dac1_pin" => gpio_for_signal!(DAC1),
    "write_value" => {
        cfg(dac_version = "1") => "dac1.write(128);",
        cfg(dac_version = "2") => "dac1.write(2048)?;",
    },
))]
//! # Digital to Analog Converter (DAC)
//!
//! ## Overview
//! Two DAC channels convert digital codes into analog voltages from 0 V to the
//! reference supply voltage. [`Dac::resolution`] gives the code width in bits.
//!
//! ## Configuration
//! Each DAC channel uses a fixed GPIO pin.
//!
//! ## Examples
//! ### Write a value to a DAC channel
//! ```rust, no_run
//! # {before_snippet}
//! # use esp_hal::analog::dac::Dac;
//! let mut dac1 = Dac::new(peripherals.DAC1, peripherals.__dac1_pin__);
//!
//! __write_value__
//! # {after_snippet}
//! ```

use super::ADC_DAC_LOCK;
use crate::gpio::AnalogPin;

#[cfg_attr(dac_version = "2", path = "v2.rs")]
#[cfg_attr(dac_version = "1", path = "v1.rs")]
mod ll;

/// DAC output error.
#[cfg(dac_version = "2")]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
#[instability::unstable]
pub enum Error {
    /// The output code is out of range.
    ValueOutOfRange,
}

#[cfg(dac_version = "2")]
impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::ValueOutOfRange => f.write_str("DAC code is out of range"),
        }
    }
}

#[cfg(dac_version = "2")]
impl core::error::Error for Error {}

/// A digital-to-analog converter (DAC) channel.
///
/// Dropping the driver disables its analog output.
#[cfg_attr(
    any(esp32s2, dac_version = "2"),
    doc = "The shared DAC clock is disabled after both channels are powered down."
)]
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Dac<'d> {
    channel: AnyDacChannel<'d>,
    #[cfg(dac_version = "2")]
    _guard: crate::system::GenericPeripheralGuard<{ crate::system::Peripheral::Dac as u8 }>,
}

impl<'d> Dac<'d> {
    /// Creates a new DAC channel.
    pub fn new<T: Instance + 'd>(dac: T, pin: T::Pin) -> Self
    where
        T::Pin: AnalogPin + 'd,
    {
        let channel = dac.degrade();

        // The DAC function clock also gates access to its registers.
        // See <https://github.com/espressif/esp-idf/blob/d445e947156f276dfe0db5804e4edf6ae90fe6fb/components/esp_hal_ana_conv/esp32s31/include/hal/dac_ll.h#L62-L75>.
        #[cfg(dac_version = "2")]
        let guard = crate::system::GenericPeripheralGuard::new();

        pin.set_analog(crate::private::Internal);
        ADC_DAC_LOCK.lock(|| ll::enable(channel.index()));

        Self {
            channel,
            #[cfg(dac_version = "2")]
            _guard: guard,
        }
    }

    /// Returns the DAC resolution in bits.
    #[instability::unstable]
    pub const fn resolution(&self) -> u8 {
        property!("dac.resolution")
    }

    core::cfg_select! {
        dac_version = "1" => {
            /// Writes a raw DAC code.
            ///
            /// The nominal output voltage is `Vref * value / 255`.
            #[cfg_attr(esp32, doc = "The reference supply `Vref` is `VDD3P3_RTC`.")]
            #[cfg_attr(esp32s2, doc = "The reference supply `Vref` is `VDD3P3_RTC_IO`.")]
            pub fn write(&mut self, value: u8) {
                ADC_DAC_LOCK.lock(|| ll::write(self.channel.index(), value));
            }
        }
        dac_version = "2" => {
            /// Writes a raw DAC code.
            ///
            /// The valid range is `0..(1 << self.resolution())`. The nominal output
            /// voltage is `Vref * value / ((1 << self.resolution()) - 1)`.
            /// The DAC uses Low-speed mode with the output buffer bypassed.
            ///
            /// # Errors
            ///
            /// [`Error::ValueOutOfRange`] if `value` is outside the valid range.
            /// The output remains unchanged on error.
            pub fn write(&mut self, value: u16) -> Result<(), Error> {
                if u32::from(value) >= 1 << self.resolution() {
                    return Err(Error::ValueOutOfRange);
                }
                ADC_DAC_LOCK.lock(|| ll::write(self.channel.index(), value));
                Ok(())
            }
        }
    }
}

impl Drop for Dac<'_> {
    fn drop(&mut self) {
        ADC_DAC_LOCK.lock(|| ll::disable(self.channel.index()));
    }
}

crate::any_peripheral! {
    /// Type-erased ownership of a DAC channel.
    #[doc(hidden)]
    pub peripheral AnyDacChannel<'d> {
        #[cfg(soc_has_dac1)]
        Dac1(crate::peripherals::DAC1<'d>),
        #[cfg(soc_has_dac2)]
        Dac2(crate::peripherals::DAC2<'d>),
    }
}

// Public because it appears in Dac::new's bounds.
#[doc(hidden)]
pub trait Instance: crate::private::Sealed + any::Degrade {
    type Pin;

    fn index(&self) -> usize;
}

impl AnyDacChannel<'_> {
    fn index(&self) -> usize {
        any::delegate!(self, channel => { channel.index() })
    }
}

for_each_analog_function! {
    (DACn $((($channel:ident, DACn, $number:literal), $gpio:ident)),*) => {
        const _: () = core::assert!(
            [$($number),*].len() == property!("dac.channel_count"),
            "DAC channel count does not match its analog pin mappings"
        );
    };

    (($channel:ident, DACn, $number:literal), $gpio:ident) => {
        impl<'d> Instance for crate::peripherals::$channel<'d> {
            type Pin = crate::peripherals::$gpio<'d>;

            fn index(&self) -> usize {
                $number - 1
            }
        }
    };
}
