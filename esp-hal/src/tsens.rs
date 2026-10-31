#![cfg_attr(docsrs, procmacros::doc_replace)]
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
//! [`Config`] selects the measurement range and clock source. Available clock
//! sources depend on the chip; see [`ClockSource`].
//!
//! ## Examples
//!
//! The following example will measure the internal chip temperature every
//! second, and print it
//!
//! ```rust, no_run
//! # {before_snippet}
//! # use esp_hal::tsens::{TemperatureSensor, Config, MeasurementRange};
//! # use esp_hal::delay::Delay;
//!
//! let config = Config::default().with_range(MeasurementRange::Minus10To80C);
//! let mut temperature_sensor = TemperatureSensor::new(peripherals.TSENS, config)?;
//! let delay = Delay::new();
//! loop {
//!     let temp = temperature_sensor.get_temperature();
//!     println!("Temperature: {:.2}°C", temp.to_celsius());
//!     delay.delay_millis(1_000);
//! }
//! # }
//! ```
//!
//! ## Implementation State
//!
//! - Interrupts are not supported

mod clocks;

#[cfg(temp_sensor_register_block_apb_saradc)]
use crate::peripherals::APB_SARADC;
#[cfg(temp_sensor_register_block_sens)]
use crate::peripherals::SENS;
/// Clock source for the temperature sensor.
pub use crate::soc::clocks::TsensSclkConfig as ClockSource;
use crate::{
    clock::ll::{ClockTree, TsensInstance},
    peripherals::TSENS,
    system::GenericPeripheralGuard,
    time::{Duration, Instant},
};

/// Temperature sensor configuration
#[derive(Debug, Clone, Default, PartialEq, Eq, Copy, Hash, procmacros::BuilderLite)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct Config {
    /// Clock source for the temperature sensor.
    clock_source: ClockSource,

    /// Measurement range of the temperature sensor.
    /// Defaults to [`MeasurementRange::Minus10To80C`].
    range: MeasurementRange,
}

/// Measurement range of the temperature sensor.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum MeasurementRange {
    /// -40 °C to 20 °C.
    Minus40To20C,
    /// -30 °C to 50 °C.
    Minus30To50C,
    /// -10 °C to 80 °C.
    #[default]
    Minus10To80C,
    /// 20 °C to 100 °C.
    Plus20To100C,
    /// 50 °C to 125 °C.
    Plus50To125C,
}

struct RangeSettings {
    dac: u8,
    offset: i8,
}

impl MeasurementRange {
    fn settings(self) -> RangeSettings {
        // See <https://github.com/espressif/esp-idf/blob/d445e947156f276dfe0db5804e4edf6ae90fe6fb/components/esp_hal_ana_conv/esp32s3/temperature_sensor_periph.c#L9-L16>.
        let (dac, offset) = match self {
            Self::Minus40To20C => (10, 2),
            Self::Minus30To50C => (11, 1),
            Self::Minus10To80C => (15, 0),
            Self::Plus20To100C => (7, -1),
            Self::Plus50To125C => (5, -2),
        };
        RangeSettings { dac, offset }
    }
}

/// Temperature sensor configuration error.
#[derive(Debug, Clone, PartialEq, Eq, Copy, Hash)]
#[non_exhaustive]
pub enum ConfigError {}

/// Stores the raw ADC value and can be used to calculate the temperature in Celsius using the
/// formula: `(raw_value * 0.4386) - (offset * 27.88) - 20.52`.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Temperature {
    /// Raw ADC value.
    pub raw_value: u8,

    /// Offset value - depends on the temperature range configured.
    pub offset: i8,
}

impl Temperature {
    /// Creates a new temperature value.
    #[inline]
    pub fn new(raw_value: u8, offset: i8) -> Self {
        Self { raw_value, offset }
    }

    /// Returns the temperature in Celsius.
    #[inline]
    pub fn to_celsius(&self) -> f32 {
        (self.raw_value as f32) * 0.4386 - (self.offset as f32) * 27.88 - 20.52
    }

    /// Returns the temperature in Fahrenheit.
    #[inline]
    pub fn to_fahrenheit(&self) -> f32 {
        let celsius = self.to_celsius();
        (celsius * 1.8) + 32.0
    }

    /// Returns the temperature in Kelvin.
    #[inline]
    pub fn to_kelvin(&self) -> f32 {
        let celsius = self.to_celsius();
        celsius + 273.15
    }
}

/// Temperature sensor driver
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TemperatureSensor<'d> {
    range: MeasurementRange,
    stabilization: Option<(Instant, Duration)>,
    _peripheral: TSENS<'d>,
    _clock_guard: TsensClockGuard,
    _tsens_guard: GenericPeripheralGuard<{ crate::system::Peripheral::Tsens as u8 }>,
    #[cfg(temp_sensor_register_block_apb_saradc)]
    _apb_saradc_guard: GenericPeripheralGuard<{ crate::system::Peripheral::ApbSarAdc as u8 }>,
}

impl<'d> TemperatureSensor<'d> {
    /// Creates and powers up the temperature sensor with the given configuration.
    pub fn new(peripheral: TSENS<'d>, config: Config) -> Result<Self, ConfigError> {
        // The APB_SARADC clock must be enabled before TSENS.
        #[cfg(temp_sensor_register_block_apb_saradc)]
        let apb_saradc_guard = GenericPeripheralGuard::new();
        let tsens_guard = GenericPeripheralGuard::new();

        let mut tsens = Self {
            range: config.range,
            stabilization: None,
            _peripheral: peripheral,
            _clock_guard: TsensClockGuard::new(config.clock_source),
            _tsens_guard: tsens_guard,
            #[cfg(temp_sensor_register_block_apb_saradc)]
            _apb_saradc_guard: apb_saradc_guard,
        };

        let dac = config.range.settings().dac;
        #[cfg(any(temp_sensor_register_block_sens, temp_sensor_register_block_lp_tsens))]
        crate::soc::regi2c::I2C_SARADC_TSENS_DAC.write_field(dac);

        tsens.power_up();
        #[cfg(temp_sensor_register_block_apb_saradc)]
        crate::soc::regi2c::I2C_SARADC_TSENS_DAC.write_field(dac);

        tsens.start_stabilization();

        Ok(tsens)
    }

    fn start_stabilization(&mut self) {
        const STABILIZATION_CYCLES: u64 = 20_000;
        let delay_us = (STABILIZATION_CYCLES * 1_000_000)
            .div_ceil(u64::from(TsensInstance::Tsens.sclk_frequency()));
        self.stabilization = Some((Instant::now(), Duration::from_micros(delay_us)));
    }

    fn power_up(&mut self) {
        debug!("Power up");
        cfg_select! {
            temp_sensor_register_block_apb_saradc => {
                APB_SARADC::regs()
                    .tsens_ctrl()
                    .modify(|_, w| w.pu().set_bit());
            }
            temp_sensor_register_block_sens => {
                // The request bit can survive a digital-system reset.
                SENS::regs().sar_tsens_ctrl().modify(|_, w| {
                    w.sar_tsens_dump_out().clear_bit();
                    w.sar_tsens_power_up_force().set_bit()
                });
                SENS::regs()
                    .sar_tsens_ctrl2()
                    .modify(|_, w| unsafe { w.sar_tsens_xpd_force().bits(1) });
                SENS::regs()
                    .sar_tsens_ctrl()
                    .modify(|_, w| w.sar_tsens_power_up().set_bit());
            }
            temp_sensor_register_block_lp_tsens => {
                TSENS::regs()
                    .ctrl()
                    .modify(|_, w| w.power_up().set_bit().sample_en().set_bit());
                TSENS::regs()
                    .wakeup_ctrl()
                    .modify(|_, w| w.wakeup_en().set_bit());
            }
        }
    }

    fn power_down(&mut self) {
        cfg_select! {
            temp_sensor_register_block_apb_saradc => {
                APB_SARADC::regs()
                    .tsens_ctrl()
                    .modify(|_, w| w.pu().clear_bit());
            }
            temp_sensor_register_block_sens => {
                SENS::regs()
                    .sar_tsens_ctrl()
                    .modify(|_, w| w.sar_tsens_power_up_force().clear_bit());
                SENS::regs()
                    .sar_tsens_ctrl2()
                    .modify(|_, w| unsafe { w.sar_tsens_xpd_force().bits(0) });
                SENS::regs()
                    .sar_tsens_ctrl()
                    .modify(|_, w| w.sar_tsens_power_up().clear_bit());
            }
            temp_sensor_register_block_lp_tsens => {
                TSENS::regs()
                    .wakeup_ctrl()
                    .modify(|_, w| w.wakeup_en().clear_bit());
                TSENS::regs()
                    .ctrl()
                    .modify(|_, w| w.sample_en().clear_bit().power_up().clear_bit());
            }
        }
    }

    /// Changes the temperature sensor configuration.
    pub fn apply_config(&mut self, config: &Config) -> Result<(), ConfigError> {
        let restart_stabilization = cfg_select! {
            soc_clock_node_tsens_sclk_is_configurable => ClockTree::with(|clocks| {
                let tsens = TsensInstance::Tsens;
                let restart = tsens.sclk_config(clocks) != Some(config.clock_source)
                    && self
                        .stabilization
                        .is_some_and(|(start, duration)| start.elapsed() < duration);
                tsens.configure_sclk(clocks, config.clock_source);
                restart
            }),
            _ => false,
        };

        let range_changed = self.range != config.range;
        if range_changed {
            crate::soc::regi2c::I2C_SARADC_TSENS_DAC.write_field(config.range.settings().dac);
            self.range = config.range;
        }
        if range_changed || restart_stabilization {
            self.start_stabilization();
        }

        Ok(())
    }

    /// Returns the raw temperature value.
    ///
    /// Waits for any remaining stabilization time after power-up or a range change.
    #[inline]
    pub fn get_temperature(&mut self) -> Temperature {
        if let Some((start, duration)) = self.stabilization.take() {
            while start.elapsed() < duration {}
        }

        let raw_value = cfg_select! {
            temp_sensor_register_block_apb_saradc => {
                APB_SARADC::regs().tsens_ctrl().read().out().bits()
            }
            temp_sensor_register_block_sens => {{
                let ctrl = SENS::regs().sar_tsens_ctrl();
                ctrl.modify(|_, w| w.sar_tsens_dump_out().set_bit());
                while ctrl.read().sar_tsens_ready().bit_is_clear() {}
                ctrl.modify(|_, w| w.sar_tsens_dump_out().clear_bit());
                ctrl.read().sar_tsens_out().bits()
            }}
            temp_sensor_register_block_lp_tsens => TSENS::regs().ctrl().read().out().bits(),
        };

        Temperature::new(raw_value, self.range.settings().offset)
    }
}

impl Drop for TemperatureSensor<'_> {
    fn drop(&mut self) {
        self.power_down();
    }
}

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
struct TsensClockGuard;

impl TsensClockGuard {
    fn new(_source: ClockSource) -> Self {
        ClockTree::with(|clocks| {
            #[cfg(soc_clock_node_tsens_sclk_is_configurable)]
            TsensInstance::Tsens.configure_sclk(clocks, _source);
            TsensInstance::Tsens.request_sclk(clocks);
        });
        Self
    }
}

impl Drop for TsensClockGuard {
    fn drop(&mut self) {
        ClockTree::with(|clocks| TsensInstance::Tsens.release_sclk(clocks));
    }
}
