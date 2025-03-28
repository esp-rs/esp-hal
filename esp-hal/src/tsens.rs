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
//!         Config::default())?;
//! let delay = Delay::new();
//! delay.delay_micros(200);
//! loop {
//!   let temp = temperature_sensor.get_temperature();
//!   println!("Temperature: {:.2}°C", temp.to_celcius());
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
    peripherals::{APB_SARADC, TSENS},
    system::GenericPeripheralGuard,
};

/// Clock source for the temperature sensor
#[derive(Debug, Clone, Default, PartialEq, Eq, Copy, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum ClockSource {
    /// Use RC_FAST clock source
    RcFast,
    /// Use XTAL clock source
    #[default]
    Xtal,
}

/// Temperature sensor configuration
#[derive(Debug, Clone, Default, PartialEq, Eq, Copy, Hash, procmacros::BuilderLite)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct Config {
    /// Clock source for the temperature sensor
    clock_source: ClockSource,
}

/// Temperature sensor configuration error
#[derive(Debug, Clone, PartialEq, Eq, Copy, Hash)]
#[non_exhaustive]
pub enum ConfigError {}

/// Temperature value
/// This struct stores the raw ADC value, and can be used to calculate the
/// temperature in Celsius using the formula:
/// `(raw_value * 0.4386) - (offset * 27.88) - 20.52`
#[derive(Debug)]
pub struct Temperature {
    /// Raw ADC value
    pub raw_value: u8,

    /// Offset value - depends on the temperature range configured
    pub offset: i8,
}

impl Temperature {
    /// Create a new temperature value
    #[inline]
    pub fn new(raw_value: u8, offset: i8) -> Self {
        Self { raw_value, offset }
    }

    /// Get the temperature in Celsius
    #[inline]
    pub fn to_celsius(&self) -> f32 {
        (self.raw_value as f32) * 0.4386 - (self.offset as f32) * 27.88 - 20.52
    }

    /// Get the temperature in Fahrenheit
    #[inline]
    pub fn to_fahrenheit(&self) -> f32 {
        let celsius = self.to_celsius();
        (celsius * 1.8) + 32.0
    }

    /// Get the temperature in Kelvin
    #[inline]
    pub fn to_kelvin(&self) -> f32 {
        let celsius = self.to_celsius();
        celsius + 273.15
    }
}

/// Temperature sensor driver
#[derive(Debug)]
pub struct TemperatureSensor<'d> {
    _peripheral: TSENS<'d>,
    _tsens_guard: GenericPeripheralGuard<{ crate::system::Peripheral::Tsens as u8 }>,
    _abp_saradc_guard: GenericPeripheralGuard<{ crate::system::Peripheral::ApbSarAdc as u8 }>,
}

impl<'d> TemperatureSensor<'d> {
    /// Create a new temperature sensor instance with configuration
    /// The sensor will be automatically powered up
    pub fn new(peripheral: TSENS<'d>, config: Config) -> Result<Self, ConfigError> {
        // NOTE: We need enable ApbSarAdc before enabling Tsens
        let apb_saradc_guard = GenericPeripheralGuard::new();
        let tsens_guard = GenericPeripheralGuard::new();

        let mut tsens = Self {
            _peripheral: peripheral,
            _tsens_guard: tsens_guard,
            _abp_saradc_guard: apb_saradc_guard,
        };
        tsens.apply_config(&config)?;

        tsens.power_up();

        Ok(tsens)
    }

    /// Power up the temperature sensor
    pub fn power_up(&self) {
        debug!("Power up");
        APB_SARADC::regs()
            .tsens_ctrl()
            .modify(|_, w| w.pu().set_bit());
    }

    /// Power down the temperature sensor - useful if you want to save power
    pub fn power_down(&self) {
        APB_SARADC::regs()
            .tsens_ctrl()
            .modify(|_, w| w.pu().clear_bit());
    }

    /// Change the temperature sensor configuration
    pub fn apply_config(&mut self, config: &Config) -> Result<(), ConfigError> {
        // Set clock source
        APB_SARADC::regs().tsens_ctrl2().write(|w| {
            w.clk_sel()
                .bit(matches!(config.clock_source, ClockSource::Xtal))
        });

        Ok(())
    }

    /// Get the raw temperature value
    #[inline]
    pub fn get_temperature(&self) -> Temperature {
        let raw_value = APB_SARADC::regs().tsens_ctrl().read().out().bits();

        // TODO Address multiple temperature ranges and offsets
        let offset = -1i8;

        Temperature::new(raw_value, offset)
    }
}
