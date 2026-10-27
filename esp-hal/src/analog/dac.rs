#![cfg_attr(docsrs, procmacros::doc_replace(
    "dac1_pin" => gpio_for_signal!(DAC1),
))]
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
//! # {before_snippet}
//! # use esp_hal::analog::dac::Dac;
//! # use esp_hal::delay::Delay;
//! # use embedded_hal::delay::DelayNs;
//! let mut dac1 = Dac::new(peripherals.DAC1, peripherals.__dac1_pin__);
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

use crate::gpio::AnalogPin;

static DAC_LOCK: esp_sync::RawMutex = esp_sync::RawMutex::new();

// Only specific pins can be used with each DAC peripheral, and of course
// these pins are different depending on which chip you are using; for this
// reason, we will type alias the pins for ease of use later in this module:
cfg_select! {
    esp32 => {
        type Dac1Gpio<'d> = crate::peripherals::GPIO25<'d>;
        type Dac2Gpio<'d> = crate::peripherals::GPIO26<'d>;
    }
    esp32s2 => {
        type Dac1Gpio<'d> = crate::peripherals::GPIO17<'d>;
        type Dac2Gpio<'d> = crate::peripherals::GPIO18<'d>;
    }
    _ => {}
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
    /// Creates a new instance of [`Dac`].
    pub fn new(dac: T, pin: T::Pin) -> Self {
        // TODO: Revert on drop.
        pin.set_analog(crate::private::Internal);

        DAC_LOCK.lock(|| {
            #[cfg(esp32s2)]
            crate::peripherals::SENS::regs()
                .sar_dac_ctrl1()
                .modify(|_, w| w.dac_clkgate_en().set_bit());

            select_direct_source(T::INDEX);

            crate::peripherals::RTC_IO::regs()
                .pad_dac(T::INDEX)
                .modify(|_, w| w.dac_xpd_force().set_bit().xpd_dac().set_bit());
        });

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
        DAC_LOCK.lock(|| {
            select_direct_source(T::INDEX);
            crate::peripherals::RTC_IO::regs()
                .pad_dac(T::INDEX)
                .modify(|_, w| unsafe { w.dac().bits(value) });
        });
    }
}

// Public because it appears in the bounds of Dac.
#[doc(hidden)]
pub trait Instance: crate::private::Sealed {
    const INDEX: usize;

    type Pin;
}

fn select_direct_source(index: usize) {
    // Select the channel source before releasing the shared DMA override.
    // https://github.com/espressif/esp-idf/blob/4d80c1b29aeabc8e735ffc6e1be9bb23e95d1d5d/components/esp_hal_ana_conv/esp32s2/include/hal/dac_ll.h#L66-L81
    crate::peripherals::SENS::regs()
        .sar_dac_ctrl2()
        .modify(|_, w| w.dac_cw_en(index as u8).clear_bit());
    crate::peripherals::SENS::regs()
        .sar_dac_ctrl1()
        .modify(|_, w| w.dac_dig_force().clear_bit());
    #[cfg(esp32s2)]
    crate::peripherals::APB_SARADC::regs()
        .apb_dac_ctrl()
        .modify(|_, w| w.trans().clear_bit());
}

#[cfg(dac_dac1)]
impl<'d> Instance for crate::peripherals::DAC1<'d> {
    const INDEX: usize = 0;

    type Pin = Dac1Gpio<'d>;
}

#[cfg(dac_dac2)]
impl<'d> Instance for crate::peripherals::DAC2<'d> {
    const INDEX: usize = 1;

    type Pin = Dac2Gpio<'d>;
}
