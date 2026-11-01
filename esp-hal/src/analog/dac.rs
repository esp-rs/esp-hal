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

        ADC_DAC_LOCK.lock(|| {
            #[cfg(esp32s2)]
            crate::peripherals::SENS::regs()
                .sar_dac_ctrl1()
                .modify(|_, w| w.dac_clkgate_en().set_bit());

            select_direct_source(channel.index());

            // Prevent ADC sampling from powering the DAC back up after drop.
            // https://github.com/espressif/esp-idf/blob/4d80c1b29aeabc8e735ffc6e1be9bb23e95d1d5d/components/esp_hal_ana_conv/esp32/include/hal/dac_ll.h#L94-L104
            let sensors = crate::peripherals::SENS::regs();
            let control = cfg_select! {
                esp32 => sensors.sar_meas_ctrl2(),
                esp32s2 => sensors.sar_amp_ctrl3(),
            };
            control.modify(|_, w| unsafe { w.sar1_dac_xpd_fsm().bits(0) });

            crate::peripherals::RTC_IO::regs()
                .pad_dac(channel.index())
                .modify(|_, w| w.dac_xpd_force().set_bit().xpd_dac().set_bit());
        });

        Self { channel }
    }

    /// Writes the given value.
    ///
    /// The nominal output voltage is `Vref * value / 255`.
    #[cfg_attr(esp32, doc = "The reference supply `Vref` is `VDD3P3_RTC`.")]
    #[cfg_attr(esp32s2, doc = "The reference supply `Vref` is `VDD3P3_RTC_IO`.")]
    pub fn write(&mut self, value: u8) {
        ADC_DAC_LOCK.lock(|| {
            select_direct_source(self.channel.index());
            crate::peripherals::RTC_IO::regs()
                .pad_dac(self.channel.index())
                .modify(|_, w| unsafe { w.dac().bits(value) });
        });
    }
}

impl Drop for Dac<'_> {
    fn drop(&mut self) {
        ADC_DAC_LOCK.lock(|| {
            let rtc_io = crate::peripherals::RTC_IO::regs();
            rtc_io
                .pad_dac(self.channel.index())
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
