//! # Digital to Analog Converter (DAC)
//!
//! ## Overview
//! Espressif devices usually have multiple DAC channels. Each DAC channel can
//! convert the digital value 0~255 to the analog voltage 0~Vref (The reference
//! voltage 'Vref' here is input from an input pin)
//!
//! The DAC peripheral supports outputting analog signal in the multiple ways.
//!
//! Two 8-bit DAC channels are available.
//!
//! ## Configuration
//! Developers can choose the  DAC channel they want to use based on the GPIO
//! pin assignments for each channel.
//!
//! ## Examples
//! ### Write a value to a DAC channel
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! # use esp_hal::analog::dac::Dac;
//! # use esp_hal::delay::Delay;
//! # use embedded_hal::delay::DelayNs;
#![cfg_attr(esp32, doc = "let dac1_pin = peripherals.GPIO25;")]
#![cfg_attr(esp32s2, doc = "let dac1_pin = peripherals.GPIO17;")]
//! let mut dac1 = Dac::new(peripherals.DAC1, dac1_pin);
//!
//! let mut delay = Delay::new();
//!
//! let mut voltage_dac1 = 200u8;
//!
//! // Change voltage on the pins using write function:
//! loop {
//!     voltage_dac1 = voltage_dac1.wrapping_add(1);
//!     dac1.write(voltage_dac1);
//!
//!     delay.delay_ms(50u32);
//! }
//! # }
//! ```

use crate::gpio::{self, AnalogPin};

// Only specific pins can be used with each DAC peripheral, and of course
// these pins are different depending on which chip you are using; for this
// reason, we will type alias the pins for ease of use later in this module:
cfg_if::cfg_if! {
    if #[cfg(esp32)] {
        type Dac1Gpio<'d> = gpio::GpioPin<'d, 25>;
        type Dac2Gpio<'d> = gpio::GpioPin<'d, 26>;
    } else if #[cfg(esp32s2)] {
        type Dac1Gpio<'d> = gpio::GpioPin<'d, 17>;
        type Dac2Gpio<'d> = gpio::GpioPin<'d, 18>;
    }
}

/// Digital-to-Analog Converter (DAC) Channel
pub struct Dac<'d, T>
where
    T: Instance + 'd,
    T::Pin: AnalogPin + 'd,
{
    _inner: T,
    _lifetime: core::marker::PhantomData<&'d mut ()>,
}

impl<'d, T> Dac<'d, T>
where
    T: Instance + 'd,
    T::Pin: AnalogPin + 'd,
{
    /// Construct a new instance of [`Dac`].
    pub fn new(dac: T, pin: T::Pin) -> Self {
        // TODO: Revert on drop.
        pin.set_analog(crate::private::Internal);

        #[cfg(esp32s2)]
        crate::peripherals::SENS::regs()
            .sar_dac_ctrl1()
            .modify(|_, w| w.dac_clkgate_en().set_bit());

        T::enable_xpd();

        Self {
            _inner: dac,
            _lifetime: core::marker::PhantomData,
        }
    }

    /// Writes the given value.
    ///
    /// For each DAC channel, the output analog voltage can be calculated as
    /// follows: DACn_OUT = VDD3P3_RTC * PDACn_DAC/256
    pub fn write(&mut self, value: u8) {
        T::set_pad_source();
        T::write_byte(value);
    }
}

#[doc(hidden)]
pub trait Instance: crate::private::Sealed {
    const INDEX: usize;

    type Pin;

    fn enable_xpd() {
        crate::peripherals::RTC_IO::regs()
            .pad_dac(Self::INDEX)
            .modify(|_, w| w.dac_xpd_force().set_bit().xpd_dac().set_bit());
    }

    fn set_pad_source() {
        crate::peripherals::SENS::regs()
            .sar_dac_ctrl2()
            .modify(|_, w| w.dac_cw_en(Self::INDEX as u8).clear_bit());
    }

    fn write_byte(value: u8) {
        crate::peripherals::RTC_IO::regs()
            .pad_dac(Self::INDEX)
            .modify(|_, w| unsafe { w.dac().bits(value) });
    }
}

impl<'d> Instance for crate::peripherals::DAC1<'d> {
    const INDEX: usize = 0;

    type Pin = Dac1Gpio<'d>;
}

impl<'d> Instance for crate::peripherals::DAC2<'d> {
    const INDEX: usize = 1;

    type Pin = Dac2Gpio<'d>;
}
