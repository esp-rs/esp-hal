#![cfg_attr(docsrs, procmacros::doc_replace(
    "dac1_pin" => gpio_for_signal!(DAC1),
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
//! dac1.write(128);
//! # {after_snippet}
//! ```

use super::ADC_DAC_LOCK;
use crate::gpio::AnalogPin;

#[cfg_attr(dac_version = "1", path = "v1.rs")]
mod ll;

/// A digital-to-analog converter (DAC) channel.
///
/// Dropping the driver disables its analog output.
#[cfg_attr(
    esp32s2,
    doc = "The shared DAC clock is disabled after both channels are powered down."
)]
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Dac<'d> {
    channel: AnyDacChannel<'d>,
}

impl<'d> Dac<'d> {
    /// Creates a new DAC channel.
    pub fn new<T: Instance + 'd>(dac: T, pin: T::Pin) -> Self
    where
        T::Pin: AnalogPin + 'd,
    {
        let channel = dac.degrade();

        pin.set_analog(crate::private::Internal);
        ADC_DAC_LOCK.lock(|| ll::enable(channel.index()));

        Self { channel }
    }

    /// Returns the DAC resolution in bits.
    #[instability::unstable]
    pub const fn resolution(&self) -> u8 {
        property!("dac.resolution")
    }

    /// Writes a raw DAC code.
    ///
    /// The nominal output voltage is `Vref * value / 255`.
    #[cfg_attr(esp32, doc = "The reference supply `Vref` is `VDD3P3_RTC`.")]
    #[cfg_attr(esp32s2, doc = "The reference supply `Vref` is `VDD3P3_RTC_IO`.")]
    pub fn write(&mut self, value: u8) {
        ADC_DAC_LOCK.lock(|| ll::write(self.channel.index(), value));
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
