#![cfg_attr(docsrs, procmacros::doc_replace(
    "dac1_pin" => gpio_for_signal!(DAC1),
))]
//! # Digital to Analog Converter (DAC)
//!
//! ## Overview
//! Two 8-bit DAC channels convert digital values from 0 to 255 into analog
//! voltages from 0 V to the reference supply voltage.
//!
//! ## Configuration
//! Each DAC channel uses a fixed GPIO pin.
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
//! loop {
//!     voltage_dac1 = voltage_dac1.wrapping_add(1);
//!     dac1.write(voltage_dac1);
//!
//!     delay.delay_ms(50u32);
//! }
//! # }
//! ```

use super::ADC_DAC_LOCK;
use crate::gpio::AnalogPin;

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

/// Digital-to-Analog Converter (DAC) Channel.
///
/// Dropping the driver disables its analog output.
#[cfg_attr(
    esp32s2,
    doc = "The shared DAC clock is disabled after both channels are powered down."
)]
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
        pin.set_analog(crate::private::Internal);

        ADC_DAC_LOCK.lock(|| {
            #[cfg(esp32s2)]
            crate::peripherals::SENS::regs()
                .sar_dac_ctrl1()
                .modify(|_, w| w.dac_clkgate_en().set_bit());

            select_direct_source(T::INDEX);

            // Prevent ADC sampling from powering the DAC back up after drop.
            // https://github.com/espressif/esp-idf/blob/4d80c1b29aeabc8e735ffc6e1be9bb23e95d1d5d/components/esp_hal_ana_conv/esp32/include/hal/dac_ll.h#L94-L104
            let sensors = crate::peripherals::SENS::regs();
            let control = cfg_select! {
                esp32 => sensors.sar_meas_ctrl2(),
                esp32s2 => sensors.sar_amp_ctrl3(),
            };
            control.modify(|_, w| unsafe { w.sar1_dac_xpd_fsm().bits(0) });

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
    /// The nominal output voltage is `Vref * value / 255`.
    #[cfg_attr(esp32, doc = "The reference supply `Vref` is `VDD3P3_RTC`.")]
    #[cfg_attr(esp32s2, doc = "The reference supply `Vref` is `VDD3P3_RTC_IO`.")]
    pub fn write(&mut self, value: u8) {
        ADC_DAC_LOCK.lock(|| {
            select_direct_source(T::INDEX);
            crate::peripherals::RTC_IO::regs()
                .pad_dac(T::INDEX)
                .modify(|_, w| unsafe { w.dac().bits(value) });
        });
    }
}

impl<T: Instance> Drop for Dac<'_, T>
where
    T::Pin: AnalogPin,
{
    fn drop(&mut self) {
        ADC_DAC_LOCK.lock(|| {
            let rtc_io = crate::peripherals::RTC_IO::regs();
            rtc_io
                .pad_dac(T::INDEX)
                .modify(|_, w| w.xpd_dac().clear_bit().dac_xpd_force().clear_bit());

            // Both channels share the clock gate.
            // https://github.com/espressif/esp-idf/blob/4d80c1b29aeabc8e735ffc6e1be9bb23e95d1d5d/components/esp_hal_ana_conv/esp32s2/include/hal/dac_ll.h#L49-L56
            #[cfg(esp32s2)]
            if rtc_io.pad_dac(0).read().xpd_dac().bit_is_clear()
                && rtc_io.pad_dac(1).read().xpd_dac().bit_is_clear()
            {
                crate::peripherals::SENS::regs()
                    .sar_dac_ctrl1()
                    .modify(|_, w| w.dac_clkgate_en().clear_bit());
            }
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
