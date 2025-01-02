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
//! It is recommended to wait a few hundred microseconds after turning it on
//! before measuring, in order to allow the sensor to stabilize.
//!
//! ## Configuration
//!
//! The temperature sensor can be configured with different clock sources.
//!
//! ## Examples
//!
//! The following example will measure the internal chip temperature every
//! second, and print it
//!
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! # use esp_hal::tsens::{TemperatureSensor, Config};
//! # use esp_hal::delay::Delay;
//!
//! let temperature_sensor = TemperatureSensor::new(
//!         peripherals.TSENS,
//!         Config::default()
//!     ).unwrap();
//! let delay = Delay::new();
//!
//! loop {
//!   let temp = temperature_sensor.get_celsius();
//!   println!("Temperature: {:.2}°C", temp);
//!   delay.delay_millis(1_000);
//! }
//! # }
//! ```
//! 
//! ## Implementation State
//!
//! - Temperature calibration range is not supported
//! - Interrupts are not supported

use crate::{
    peripheral::{Peripheral, PeripheralRef},
    peripherals::TSENS,
    system::{GenericPeripheralGuard, PeripheralClockControl},
};

/// Clock source for the temperature sensor
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ClockSource {
    /// Use RC_FAST clock source
    RcFast,
    /// Use XTAL clock source
    #[default]
    Xtal,
}

/// Temperature sensor configuration
#[derive(Debug, Clone, Default, PartialEq, Eq, Copy, procmacros::BuilderLite)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config {
    clock_source: ClockSource,
}

/// Temperature sensor configuration error
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConfigError {}

/// Temperature sensor driver
pub struct TemperatureSensor<'d> {
    _peripheral: PeripheralRef<'d, TSENS>,
    _guard: GenericPeripheralGuard<{ crate::system::Peripheral::Tsens as u8 }>,
}

impl<'d> TemperatureSensor<'d> {
    /// Create a new temperature sensor instance with configuration
    pub fn new(
        peripheral: impl Peripheral<P = TSENS> + 'd,
        config: Config,
    ) -> Result<Self, ConfigError> {
        crate::into_ref!(peripheral);
        let guard = GenericPeripheralGuard::new();

        // We need to enable ApbSarAdc clock before trying to write on the tsens_ctrl
        // register
        PeripheralClockControl::enable(crate::system::Peripheral::ApbSarAdc);
        let apb_saradc = unsafe { &*crate::peripherals::APB_SARADC::PTR };

        // Power Up
        apb_saradc.tsens_ctrl().write(|w| w.pu().set_bit());

        let mut tsens = Self {
            _guard: guard,
            _peripheral: peripheral,
        };
        tsens.apply_config(&config)?;

        Ok(tsens)
    }

    /// Change the temperature sensor configuration
    pub fn apply_config(&mut self, config: &Config) -> Result<(), ConfigError> {
        let apb_saradc = unsafe { &*crate::peripherals::APB_SARADC::PTR };

        // Set clock source
        apb_saradc.tsens_ctrl2().write(|w| {
            w.clk_sel()
                .bit(matches!(config.clock_source, ClockSource::Xtal))
        });

        Ok(())
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
