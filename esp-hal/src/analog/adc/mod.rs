#![cfg_attr(docsrs, procmacros::doc_replace(
    "analog_pin" => gpio_for_signal!(ADC1_CH0),
))]
//! # Analog to Digital Converter (ADC)
//!
//! ## Overview
//!
//! The ADC is integrated on the chip, and is capable of measuring analog
//! signals from specific analog I/O pins. One or more ADC units are available,
//! depending on the device being used.
//!
//! ## Configuration
//!
//! The ADC can be configured to measure analog signals from specific pins. The
//! configuration includes the resolution of the ADC, the attenuation of the
//! input signal, and the pins to be measured.
//!
//! Some targets also support ADC calibration via different schemes like
//! basic calibration, curve fitting or linear interpolation. The calibration
//! schemes can be used to improve the accuracy of the ADC readings.
//!
//! ## Examples
//!
//! ### Read an analog signal from a pin
//!
//! ```rust, no_run
//! # {before_snippet}
//! # use esp_hal::analog::adc::AdcConfig;
//! # use esp_hal::peripherals::ADC1;
//! # use esp_hal::analog::adc::Attenuation;
//! # use esp_hal::analog::adc::Adc;
//! # use esp_hal::delay::Delay;
//! let mut adc1_config = AdcConfig::new();
//! let mut pin = adc1_config.enable_pin(peripherals.__analog_pin__, Attenuation::_11dB);
//! let mut adc1 = Adc::new(peripherals.ADC1, adc1_config);
//!
//! let mut delay = Delay::new();
//!
//! loop {
//!     let pin_value = nb::block!(adc1.read_oneshot(&mut pin))?;
//!
//!     delay.delay_millis(1500);
//! }
//! # }
//! ```
//!
//! ## Implementation State
//!
//!  - [ADC calibration is not implemented for all targets].
//!  - The ESP32-S31 has no calibration scheme. ESP-IDF does not define the calibration eFuses or
//!    the curve fitting coefficients for this chip yet.
//!  - The ESP32-S31 SAR ADC has one attenuation setting, so the attenuation given to
//!    [`AdcConfig::enable_pin`] has no effect
//!  - The ESP32-S31 SAR is differential and its result is the weighted sum of 17 redundant
//!    comparator bits. Readings run from 0 to `FULL_SCALE` (4393), and an input tied to ground
//!    reads about `ZERO_DIFF_CODE` (2198), so a single-ended measurement only uses the codes above
//!    that point.
//!
//! [ADC calibration is not implemented for all targets]: https://github.com/esp-rs/esp-hal/issues/326
use core::marker::PhantomData;

use crate::gpio::AnalogPin;

#[cfg_attr(esp32, path = "esp32.rs")]
#[cfg_attr(esp32p4, path = "p4.rs")]
#[cfg_attr(esp32s31, path = "s31.rs")]
#[cfg_attr(all(riscv, not(any(esp32p4, esp32s31))), path = "riscv.rs")]
#[cfg_attr(any(esp32s2, esp32s3), path = "xtensa.rs")]
#[cfg(feature = "unstable")]
mod implementation;

#[cfg(feature = "unstable")]
pub use self::implementation::*;

/// The approximate attenuation of the ADC pin.
///
/// The effective measurement range for a given attenuation is dependent on the
/// device being targeted. Refer to the "ADC Characteristics" section of the
/// device datasheet for more information.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(clippy::enum_variant_names, reason = "unit of measurement")]
pub enum Attenuation {
    /// About 0dB attenuation.
    _0dB   = 0b00,
    /// About 2.5dB attenuation.
    _2p5dB = 0b01,
    /// About 6dB attenuation.
    _6dB   = 0b10,
    /// About 11dB attenuation.
    _11dB  = 0b11,
}

/// Calibration source of the ADC.
#[cfg(not(esp32))]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum AdcCalSource {
    /// Use Ground as the calibration source
    Gnd,
    /// Use Vref as the calibration source
    Ref,
}

/// An I/O pin which can be read using the ADC.
pub struct AdcPin<PIN, ADCX, CS = ()> {
    /// The underlying GPIO pin.
    pub pin: PIN,
    /// Calibration scheme used for the configured ADC pin
    pub cal_scheme: CS,
    _phantom: PhantomData<ADCX>,
}

impl<PIN: core::fmt::Debug, ADCX, CS: core::fmt::Debug> core::fmt::Debug for AdcPin<PIN, ADCX, CS> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("AdcPin")
            .field("pin", &self.pin)
            .field("cal_scheme", &self.cal_scheme)
            .finish()
    }
}

#[cfg(feature = "defmt")]
impl<PIN: defmt::Format, ADCX, CS: defmt::Format> defmt::Format for AdcPin<PIN, ADCX, CS> {
    fn format(&self, fmt: defmt::Formatter<'_>) {
        defmt::write!(
            fmt,
            "AdcPin {{ pin: {}, cal_scheme: {} }}",
            self.pin,
            self.cal_scheme
        );
    }
}

/// Configuration for the ADC.
#[cfg(feature = "unstable")]
pub struct AdcConfig<ADCX> {
    #[cfg(esp32)]
    resolution: Resolution,
    attenuations: [Option<Attenuation>; NUM_ATTENS],
    _phantom: PhantomData<ADCX>,
}

#[cfg(feature = "unstable")]
impl<ADCX> AdcConfig<ADCX> {
    /// Creates a new configuration struct with its default values.
    pub fn new() -> Self {
        Self::default()
    }

    /// Enables the specified pin with the given attenuation.
    pub fn enable_pin<PIN>(&mut self, pin: PIN, attenuation: Attenuation) -> AdcPin<PIN, ADCX>
    where
        PIN: AdcChannel + AnalogPin,
    {
        // TODO revert this on drop
        pin.set_analog(crate::private::Internal);
        self.attenuations[pin.adc_channel() as usize] = Some(attenuation);

        AdcPin {
            pin,
            cal_scheme: AdcCalScheme::<()>::new_cal(attenuation),
            _phantom: PhantomData,
        }
    }

    /// Enables the specified pin with the given attenuation and calibration
    /// scheme.
    #[cfg(feature = "unstable")]
    pub fn enable_pin_with_cal<PIN, CS>(
        &mut self,
        pin: PIN,
        attenuation: Attenuation,
    ) -> AdcPin<PIN, ADCX, CS>
    where
        PIN: AdcChannel + AnalogPin,
        CS: AdcCalScheme<ADCX>,
    {
        // TODO revert this on drop
        pin.set_analog(crate::private::Internal);
        let channel = pin.adc_channel();
        self.attenuations[channel as usize] = Some(attenuation);

        AdcPin {
            pin,
            cal_scheme: CS::new_cal_with_channel(attenuation, channel),
            _phantom: PhantomData,
        }
    }
}

#[cfg(feature = "unstable")]
impl<ADCX> Default for AdcConfig<ADCX> {
    fn default() -> Self {
        Self {
            #[cfg(esp32)]
            resolution: Resolution::default(),
            attenuations: [None; NUM_ATTENS],
            _phantom: PhantomData,
        }
    }
}

#[cfg(not(esp32))]
#[doc(hidden)]
#[cfg(feature = "unstable")]
pub trait CalibrationAccess: RegisterAccess {
    const ADC_CAL_CNT_MAX: u16;
    const ADC_CAL_CHANNEL: u16;
    const ADC_VAL_MASK: u16;

    fn enable_vdef(enable: bool);

    /// Enables internal calibration voltage source.
    fn connect_cal(source: AdcCalSource, enable: bool);
}

/// A helper trait to get the ADC channel of a compatible GPIO pin.
pub trait AdcChannel {
    /// Channel number used by the ADC
    fn adc_channel(&self) -> u8;
}

/// A trait abstracting over calibration methods.
///
/// The methods in this trait are mostly for internal use. Call
/// `enable_pin_with_cal` with an implementor of this trait to get calibrated
/// ADC reads.
pub trait AdcCalScheme<ADCX>: Sized + crate::private::Sealed {
    /// Creates a new calibration scheme for the given attenuation.
    fn new_cal(atten: Attenuation) -> Self;

    /// Creates a new calibration scheme for the given attenuation and ADC
    /// channel.
    ///
    /// The default implementation ignores `channel` and calls [`Self::new_cal`].
    fn new_cal_with_channel(atten: Attenuation, _channel: u8) -> Self {
        Self::new_cal(atten)
    }

    /// Returns the basic ADC bias value.
    fn adc_cal(&self) -> u16 {
        0
    }

    /// Converts ADC value.
    fn adc_val(&self, val: u16) -> u16 {
        val
    }
}

impl crate::private::Sealed for () {}

impl<ADCX> AdcCalScheme<ADCX> for () {
    fn new_cal(_atten: Attenuation) -> Self {}
}

/// A helper trait to get access to ADC calibration efuses.
#[cfg(not(any(esp32, esp32s31)))]
trait AdcCalEfuse {
    /// Returns the ADC calibration init code.
    ///
    /// Returns digital value for zero voltage for a given attenuation.
    fn init_code(atten: Attenuation) -> Option<u16>;

    /// Returns the ADC calibration reference point voltage.
    ///
    /// Returns reference voltage (millivolts) for a given attenuation.
    fn cal_mv(atten: Attenuation) -> u16;

    /// Returns the ADC calibration reference point digital value.
    ///
    /// Returns digital value for reference voltage for a given attenuation.
    fn cal_code(atten: Attenuation) -> Option<u16>;

    /// Returns the ADC channel specific calibration.
    ///
    /// Returns digital per channel offset from reference voltage.
    #[cfg(any(esp32c5, esp32c6, esp32c61, esp32h2))]
    fn cal_chan_compens(atten: Attenuation, channel: u8) -> Option<i32>;
}

for_each_analog_function! {
    (($ch_name:ident, ADCn_CHm, $adc:literal, $ch:literal), $gpio:ident) => {
        impl $crate::analog::adc::AdcChannel for $crate::peripherals::$gpio<'_> {
            fn adc_channel(&self) -> u8 {
                $ch
            }
        }
    };
}
