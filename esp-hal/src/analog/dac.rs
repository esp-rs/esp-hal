//! # Digital to Analog Converter (DAC)
//!
//! The `dac` module enables users to generate analog output signals with
//! precise control over voltage levels using one of the onboard
//! digital-to-analog converters (DAC).
//!
//! Two 8-bit DAC channels are available. Each DAC channel can convert the
//! digital value 0-255 to the analog voltage 0-3.3v. Developers can choose the
//! DAC channel they want to use based on the GPIO pin assignments for each
//! channel.
//!
//! ## Example
//!
//! ```no_run
//! let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
//! let gpio25 = io.pins.gpio25;
//! let gpio26 = io.pins.gpio26;
//!
//! let mut dac1 = Dac1::new(peripherals.DAC1, gpio25);
//! let mut dac2 = Dac2::new(peripherals.DAC2, gpio26);
//!
//! let mut delay = Delay::new(&clocks);
//!
//! let mut voltage_dac1 = 200u8;
//! let mut voltage_dac2 = 255u8;
//!
//! // Change voltage on the pins using write function:
//! loop {
//!     voltage_dac1 = voltage_dac1.wrapping_add(1);
//!     dac1.write(voltage_dac1);
//!
//!     voltage_dac2 = voltage_dac2.wrapping_sub(1);
//!     dac2.write(voltage_dac2);
//!
//!     delay.delay_ms(50u32);
//! }
//! ```

use crate::{
    gpio::{self, AnalogPin},
    peripheral::{Peripheral, PeripheralRef},
    peripherals,
};

cfg_if::cfg_if! {
    if #[cfg(esp32)] {
        type Dac1Gpio = gpio::Gpio25;
        type Dac2Gpio = gpio::Gpio26;
    } else if #[cfg(esp32s2)] {
        type Dac1Gpio = gpio::Gpio17;
        type Dac2Gpio = gpio::Gpio18;
    }
}

/// Digital-to-Analog Converter (DAC) Channel 1
pub struct Dac1<'d> {
    _inner: PeripheralRef<'d, peripherals::DAC1>,
}

impl<'d> Dac1<'d> {
    /// Constructs a new DAC instance.
    pub fn new(dac: impl Peripheral<P = peripherals::DAC1> + 'd, pin: Dac1Gpio) -> Self {
        crate::into_ref!(dac);
        // TODO revert this on drop
        pin.set_analog(crate::private::Internal);

        #[cfg(esp32s2)]
        unsafe { &*peripherals::SENS::PTR }
            .sar_dac_ctrl1()
            .modify(|_, w| w.dac_clkgate_en().set_bit());

        unsafe { &*peripherals::RTC_IO::PTR }
            .pad_dac1()
            .modify(|_, w| w.pdac1_dac_xpd_force().set_bit().pdac1_xpd_dac().set_bit());

        Self { _inner: dac }
    }

    /// Writes the given value.
    ///
    /// For each DAC channel, the output analog voltage can be calculated as
    /// follows: DACn_OUT = VDD3P3_RTC * PDACn_DAC/256
    pub fn write(&mut self, value: u8) {
        unsafe { &*crate::peripherals::SENS::PTR }
            .sar_dac_ctrl2()
            .modify(|_, w| w.dac_cw_en1().clear_bit());

        unsafe { &*crate::peripherals::RTC_IO::PTR }
            .pad_dac1()
            .modify(|_, w| unsafe { w.pdac1_dac().bits(value) });
    }
}

/// Digital-to-Analog Converter (DAC) Channel 2
pub struct Dac2<'d> {
    _inner: PeripheralRef<'d, peripherals::DAC2>,
}

impl<'d> Dac2<'d> {
    /// Constructs a new DAC instance.
    pub fn new(dac: impl Peripheral<P = peripherals::DAC2> + 'd, pin: Dac2Gpio) -> Self {
        crate::into_ref!(dac);
        // TODO revert this on drop
        pin.set_analog(crate::private::Internal);

        #[cfg(esp32s2)]
        unsafe { &*peripherals::SENS::PTR }
            .sar_dac_ctrl1()
            .modify(|_, w| w.dac_clkgate_en().set_bit());

        unsafe { &*peripherals::RTC_IO::PTR }
            .pad_dac2()
            .modify(|_, w| w.pdac2_dac_xpd_force().set_bit().pdac2_xpd_dac().set_bit());

        Self { _inner: dac }
    }

    /// Writes the given value.
    ///
    /// For each DAC channel, the output analog voltage can be calculated as
    /// follows: DACn_OUT = VDD3P3_RTC * PDACn_DAC/256
    pub fn write(&mut self, value: u8) {
        unsafe { &*crate::peripherals::SENS::PTR }
            .sar_dac_ctrl2()
            .modify(|_, w| w.dac_cw_en2().clear_bit());

        unsafe { &*crate::peripherals::RTC_IO::PTR }
            .pad_dac2()
            .modify(|_, w| unsafe { w.pdac2_dac().bits(value) });
    }
}
