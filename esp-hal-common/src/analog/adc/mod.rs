//! # Analog to Digital Converter (ADC)
//!
//! The Analog to Digital Converter (ADC) is integrated on the chip, and is
//! capable of measuring analog signals from specific analog I/O pins. One or
//! more ADC units are available, depending on the device being used.
//!
//! ## Example
//!
//! ```no_run
//! let mut adc1_config = AdcConfig::new();
//! let mut adc1 = ADC::<ADC1>::adc(peripherals.ADC1, adc1_config).unwrap();
//! let mut pin = adc1_config.enable_pin(io.pins.gpio2.into_analog(), Attenuation::Attenuation11dB);
//!
//! let mut delay = Delay::new(&clocks);
//!
//! loop {
//!     let pin_value: u16 = nb::block!(adc1.read(&mut pin)).unwrap();
//!     println!("PIN2 ADC reading = {}", pin_value);
//!
//!     delay.delay_ms(1500u32);
//! }
//! ```

pub use self::implementation::*;

#[cfg_attr(esp32, path = "esp32.rs")]
#[cfg_attr(riscv, path = "riscv.rs")]
#[cfg_attr(any(esp32s2, esp32s3), path = "xtensa.rs")]
mod implementation;

mod private {
    pub trait Sealed {}
}

/// The attenuation of the ADC pin.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Attenuation {
    /// 0dB attenuation, measurement range: 0-800mV
    Attenuation0dB   = 0b00,
    /// 2.5dB attenuation, measurement range: 0-1100mV
    #[cfg(not(esp32c2))]
    Attenuation2p5dB = 0b01,
    /// 6dB attenuation, measurement range: 0-1350mV
    #[cfg(not(esp32c2))]
    Attenuation6dB   = 0b10,
    /// 11dB attenuation, measurement range: 0-2600mV
    Attenuation11dB  = 0b11,
}

/// Calibration source of the ADC.
#[cfg(not(esp32))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AdcCalSource {
    Gnd,
    Ref,
}

/// A helper trait to get the ADC channel of a compatible GPIO pin.
pub trait AdcChannel {
    const CHANNEL: u8;
}

/// A trait abstracting over calibration methods.
///
/// The methods in this trait are mostly for internal use. To get
/// calibrated ADC reads, all you need to do is call `enable_pin_with_cal`
/// and specify some implementor of this trait.
pub trait AdcCalScheme<ADCI>: Sized + private::Sealed {
    /// Create a new calibration scheme for the given attenuation.
    fn new_cal(atten: Attenuation) -> Self;

    /// Return the basic ADC bias value. See [`AdcCalBasic`] for
    /// details.
    fn adc_cal(&self) -> u16 {
        0
    }

    /// Convert ADC value
    fn adc_val(&self, val: u16) -> u16 {
        val
    }
}

impl private::Sealed for () {}

impl<ADCI> AdcCalScheme<ADCI> for () {
    fn new_cal(_atten: Attenuation) -> Self {
        ()
    }
}

/// A helper trait to get access to ADC calibration efuses.
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
