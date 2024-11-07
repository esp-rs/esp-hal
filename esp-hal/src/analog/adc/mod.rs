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
//! ## Usage
//!
//! The ADC driver implements the `embedded-hal@0.2.x` ADC traits.
//!
//! ## Example
//!
//! ### Read an analog signal from a pin
//!
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! # use esp_hal::analog::adc::AdcConfig;
//! # use esp_hal::peripherals::ADC1;
//! # use esp_hal::analog::adc::Attenuation;
//! # use esp_hal::analog::adc::Adc;
//! # use esp_hal::delay::Delay;
//! # use esp_hal::gpio::Io;
//! # let io = Io::new(peripherals.IO_MUX);
#![cfg_attr(esp32, doc = "let analog_pin = peripherals.pins.gpio32;")]
#![cfg_attr(any(esp32s2, esp32s3), doc = "let analog_pin = peripherals.pins.gpio3;")]
#![cfg_attr(
    not(any(esp32, esp32s2, esp32s3)),
    doc = "let analog_pin = peripherals.pins.gpio2;"
)]
//! let mut adc1_config = AdcConfig::new();
//! let mut pin = adc1_config.enable_pin(
//!     analog_pin,
//!     Attenuation::Attenuation11dB,
//! );
//! let mut adc1 = Adc::new(peripherals.ADC1, adc1_config);
//!
//! let mut delay = Delay::new();
//!
//! loop {
//!     let pin_value: u16 = nb::block!(adc1.read_oneshot(&mut pin)).unwrap();
//!
//!     delay.delay_millis(1500);
//! }
//! # }
//! ```
//! 
//! ## Implementation State
//!
//!  - [ADC calibration is not implemented for all targets].
//!
//! [ADC calibration is not implemented for all targets]: https://github.com/esp-rs/esp-hal/issues/326
use core::marker::PhantomData;

pub use self::implementation::*;
use crate::gpio::AnalogPin;

#[cfg_attr(esp32, path = "esp32.rs")]
#[cfg_attr(riscv, path = "riscv.rs")]
#[cfg_attr(any(esp32s2, esp32s3), path = "xtensa.rs")]
mod implementation;

/// The attenuation of the ADC pin.
///
/// The effective measurement range for a given attenuation is dependent on the
/// device being targeted. Please refer to "ADC Characteristics" section of your
/// device's datasheet for more information.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Attenuation {
    /// 0dB attenuation
    Attenuation0dB   = 0b00,
    /// 2.5dB attenuation
    #[cfg(not(esp32c2))]
    Attenuation2p5dB = 0b01,
    /// 6dB attenuation
    #[cfg(not(esp32c2))]
    Attenuation6dB   = 0b10,
    /// 11dB attenuation
    Attenuation11dB  = 0b11,
}

/// Calibration source of the ADC.
#[cfg(not(esp32))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AdcCalSource {
    /// Use Ground as the calibration source
    Gnd,
    /// Use Vref as the calibration source
    Ref,
}

/// An I/O pin which can be read using the ADC.
pub struct AdcPin<PIN, ADCI, CS = ()> {
    /// The underlying GPIO pin
    pub pin: PIN,
    /// Calibration scheme used for the configured ADC pin
    #[cfg_attr(esp32, allow(unused))]
    pub cal_scheme: CS,
    _phantom: PhantomData<ADCI>,
}

impl<PIN, ADCI, CS> embedded_hal_02::adc::Channel<ADCI> for AdcPin<PIN, ADCI, CS>
where
    PIN: embedded_hal_02::adc::Channel<ADCI, ID = u8>,
{
    type ID = u8;

    fn channel() -> Self::ID {
        PIN::channel()
    }
}

/// Configuration for the ADC.
pub struct AdcConfig<ADCI> {
    #[cfg_attr(not(esp32), allow(unused))]
    resolution: Resolution,
    attenuations: [Option<Attenuation>; NUM_ATTENS],
    _phantom: PhantomData<ADCI>,
}

impl<ADCI> AdcConfig<ADCI> {
    /// Create a new configuration struct with its default values
    pub fn new() -> Self {
        Self::default()
    }

    /// Enable the specified pin with the given attenuation
    pub fn enable_pin<PIN>(&mut self, pin: PIN, attenuation: Attenuation) -> AdcPin<PIN, ADCI>
    where
        PIN: AdcChannel + AnalogPin,
    {
        // TODO revert this on drop
        pin.set_analog(crate::private::Internal);
        self.attenuations[PIN::CHANNEL as usize] = Some(attenuation);

        AdcPin {
            pin,
            cal_scheme: AdcCalScheme::<()>::new_cal(attenuation),
            _phantom: PhantomData,
        }
    }

    /// Enable the specified pin with the given attenuation and calibration
    /// scheme
    #[cfg(not(esp32))]
    pub fn enable_pin_with_cal<PIN, CS>(
        &mut self,
        pin: PIN,
        attenuation: Attenuation,
    ) -> AdcPin<PIN, ADCI, CS>
    where
        ADCI: CalibrationAccess,
        PIN: AdcChannel + AnalogPin,
        CS: AdcCalScheme<ADCI>,
    {
        // TODO revert this on drop
        pin.set_analog(crate::private::Internal);
        self.attenuations[PIN::CHANNEL as usize] = Some(attenuation);

        AdcPin {
            pin,
            cal_scheme: CS::new_cal(attenuation),
            _phantom: PhantomData,
        }
    }
}

impl<ADCI> Default for AdcConfig<ADCI> {
    fn default() -> Self {
        Self {
            resolution: Resolution::default(),
            attenuations: [None; NUM_ATTENS],
            _phantom: PhantomData,
        }
    }
}

#[cfg(not(esp32))]
#[doc(hidden)]
pub trait CalibrationAccess: RegisterAccess {
    const ADC_CAL_CNT_MAX: u16;
    const ADC_CAL_CHANNEL: u16;
    const ADC_VAL_MASK: u16;

    fn enable_vdef(enable: bool);

    /// Enable internal calibration voltage source
    fn connect_cal(source: AdcCalSource, enable: bool);
}

/// A helper trait to get the ADC channel of a compatible GPIO pin.
pub trait AdcChannel {
    /// Channel number used by the ADC
    const CHANNEL: u8;
}

/// A trait abstracting over calibration methods.
///
/// The methods in this trait are mostly for internal use. To get
/// calibrated ADC reads, all you need to do is call `enable_pin_with_cal`
/// and specify some implementor of this trait.
pub trait AdcCalScheme<ADCI>: Sized + crate::private::Sealed {
    /// Create a new calibration scheme for the given attenuation.
    fn new_cal(atten: Attenuation) -> Self;

    /// Return the basic ADC bias value.
    fn adc_cal(&self) -> u16 {
        0
    }

    /// Convert ADC value.
    fn adc_val(&self, val: u16) -> u16 {
        val
    }
}

impl crate::private::Sealed for () {}

impl<ADCI> AdcCalScheme<ADCI> for () {
    fn new_cal(_atten: Attenuation) -> Self {}
}

/// A helper trait to get access to ADC calibration efuses.
#[cfg(not(any(esp32, esp32s2, esp32h2)))]
trait AdcCalEfuse {
    /// Get ADC calibration init code
    ///
    /// Returns digital value for zero voltage for a given attenuation
    fn get_init_code(atten: Attenuation) -> Option<u16>;

    /// Get ADC calibration reference point voltage
    ///
    /// Returns reference voltage (millivolts) for a given attenuation
    fn get_cal_mv(atten: Attenuation) -> u16;

    /// Get ADC calibration reference point digital value
    ///
    /// Returns digital value for reference voltage for a given attenuation
    fn get_cal_code(atten: Attenuation) -> Option<u16>;
}

macro_rules! impl_adc_interface {
    ($adc:ident [
        $( (GpioPin<$pin:literal>, $channel:expr) ,)+
    ]) => {
        $(
            impl $crate::analog::adc::AdcChannel for crate::gpio::GpioPin<$pin> {
                const CHANNEL: u8 = $channel;
            }

            impl embedded_hal_02::adc::Channel<crate::peripherals::$adc> for crate::gpio::GpioPin<$pin> {
                type ID = u8;

                fn channel() -> u8 { $channel }
            }
        )+
    }
}

pub(crate) use impl_adc_interface;
