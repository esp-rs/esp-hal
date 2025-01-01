//! # Temperature Sensor (tsens)
//!
//! ## Overview
//!
//! The Temperature Sensor peripheral is used to measure the internal
//! temperature inside the chip. The voltage is internally converted via an ADC
//! into a digital value, and has a measuring range of –40 °C to 125 °C.
//! The temperature value depends on factors like microcontroller clock
//! frequency or I/O load. Generally, the chip’s internal temperature is higher
//! than the operating ambient temperature.
//!
//! ## Configuration
//!
//! Currently, it does not support any configuration, but some future
//! configuration options are listed below in the Implementation State list of
//! pending items
//!
//! ## Examples
//!
//! The following example will measure the internal chip temperature every
//! second, and print it
//!
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! # use esp_hal::tsens::TemperatureSensor;
//! # use esp_hal::delay::Delay;
//!
//! let tsens = TemperatureSensor::new(peripherals.TSENS);
//! let delay = Delay::new();
//!
//! loop {
//!   let temp = tsens.get_celsius();
//!   println!("Temperature: {:.2}°C", temp);
//!   delay.delay_millis(1_000);
//! }
//! # }
//! ```
//! 
//! ## Implementation State
//!
//! - Source clock selection is not supported
//! - Temperature calibration range is not supported
//! - Interrupts are not supported

use crate::{
    peripheral::{Peripheral, PeripheralRef},
    peripherals::TSENS,
    system::{GenericPeripheralGuard, PeripheralClockControl},
};

/// Temperature sensor driver
// #[derive(Clone, Copy)]
pub struct TemperatureSensor<'d> {
    _peripheral: PeripheralRef<'d, TSENS>,
    _guard: GenericPeripheralGuard<{ crate::system::Peripheral::Tsens as u8 }>,
}

impl<'d> TemperatureSensor<'d> {
    /// Create a new temperature sensor instance
    pub fn new(peripheral: impl Peripheral<P = TSENS> + 'd) -> Self {
        crate::into_ref!(peripheral);

        let guard = GenericPeripheralGuard::new();

        // We need to enable ApbSarAdc clock before trying to write on the tsens_ctrl
        // register
        PeripheralClockControl::enable(crate::system::Peripheral::ApbSarAdc);

        let apb_saradc = unsafe { &*crate::peripherals::APB_SARADC::PTR };

        // Power Up
        apb_saradc.tsens_ctrl().write(|w| w.pu().set_bit());

        // Default to XTAL_CLK source, as it works out of the box on both esp32c6 and
        // esp32c3
        apb_saradc.tsens_ctrl2().write(|w| w.clk_sel().set_bit());

        Self {
            _guard: guard,
            _peripheral: peripheral,
        }
    }

    /// Disable the temperature sensor
    pub fn disable(&self) {
        let abp_saradc = unsafe { &*crate::peripherals::APB_SARADC::PTR };
        abp_saradc.tsens_ctrl().write(|w| w.pu().clear_bit());

        PeripheralClockControl::disable(crate::system::Peripheral::Tsens);
    }

    /// Get the temperature in Celsius
    #[inline]
    pub fn get_celsius(&self) -> f32 {
        let abp_saradc = unsafe { &*crate::peripherals::APB_SARADC::PTR };

        let measurement = abp_saradc.tsens_ctrl().read().out().bits();

        // TODO Address multiple temperature ranges and offsets
        let offset = -1f32;
        (measurement as f32) * 0.4386 - offset * 27.88 - 20.52
    }

    /// Get the temperature in Fahrenheit
    #[inline]
    pub fn get_fahrenheit(&self) -> f32 {
        let celsius = self.get_celsius();
        (celsius * 1.8) + 32.0
    }

    /// Get the temperature in Kelvin
    #[inline]
    pub fn get_kelvin(&self) -> f32 {
        let celsius = self.get_celsius();
        celsius + 273.15
    }
}
