//! # Coordinate Rotation Digital Computer (CORDIC)
//!
//! ## Overview
//! The CORDIC peripheral computes trigonometric and hyperbolic functions,
//! square roots, and natural logarithms. It also converts Cartesian vectors
//! to polar coordinates.
//!
//! The driver uses signed Q1.15 and Q1.31 values in `[-1, 1)`.
//! Angle formulas use radians.
//!
//! ## Configuration
//! The default [`Config`] uses the 160 MHz PLL without division.
//!
//! ## Implementation State
//! The driver uses blocking register polling.
//!
//! ## Examples
//!
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! use esp_hal::{cordic::{ClockSource, Config, Cordic, Precision}, time::Rate};
//! let config = Config::default()
//!     .with_clock_source(ClockSource::Xtal)
//!     .with_frequency(Rate::from_mhz(24))
//!     .with_precision(Precision::Iterations16);
//! let mut cordic = Cordic::new(peripherals.CORDIC, config)?;
//! let (sin, cos) = cordic.sin_cos_q15(0x2000)?; // π/4
//! # Ok(())
//! # }
//! ```

use core::marker::PhantomData;

/// Source of the CORDIC calculation clock.
pub use crate::soc::clocks::CordicFunctionClockSource as ClockSource;
use crate::{
    Blocking,
    DriverMode,
    clock::{
        dividers::FractionalDivider,
        ll::{ClockTree, CordicFunctionClockConfig, CordicInstance},
    },
    peripherals::CORDIC,
    system::{GenericPeripheralGuard, Peripheral, PeripheralClockControl},
    time::{Duration, Instant, Rate},
};

/// Configuration for the CORDIC accelerator.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Hash, procmacros::BuilderLite)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct Config {
    /// Calculation clock source.
    clock_source: ClockSource,
    /// Target clock rate. Uses the source rate when unset.
    ///
    /// See [`Cordic::frequency`] for the rate after divider rounding.
    frequency: Option<Rate>,
    /// Iteration count. Defaults to [`Precision::Maximum`].
    precision: Precision,
}

#[derive(Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
struct ClockRequest {
    source: ClockSource,
    target_frequency: Option<Rate>,
    source_hz: u32,
}

impl ClockRequest {
    fn new(config: &Config) -> Self {
        let source = config.clock_source;
        let target_frequency = config.frequency;
        Self {
            source,
            target_frequency,
            source_hz: target_frequency.map_or(0, |_| {
                CordicInstance::function_clock_source_frequency(source)
            }),
        }
    }

    fn resolve(&self) -> Result<CordicFunctionClockConfig, ConfigError> {
        let Some(target_frequency) = self.target_frequency else {
            return Ok(CordicFunctionClockConfig::new(self.source, 1, 0, 1));
        };
        let target_hz = target_frequency.as_hz();
        if target_hz == 0 || target_hz > self.source_hz {
            return Err(ConfigError::UnreachableClockRate);
        }
        let (_, max_divider) = property!("clock_tree.cordic.function_clock.divider");
        let (_, max_denominator) = property!("clock_tree.cordic.function_clock.denominator");
        let divider = FractionalDivider::new(self.source_hz, target_hz, max_denominator);
        if divider.integer > max_divider {
            return Err(ConfigError::UnreachableClockRate);
        }
        Ok(CordicFunctionClockConfig::new(
            self.source,
            divider.integer,
            divider.numerator,
            divider.denominator.max(1),
        ))
    }
}

/// Number of CORDIC iterations per calculation.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
#[repr(u8)]
pub enum Precision {
    /// Highest iteration count supported by the operation.
    #[default]
    Maximum,
    /// 4 iterations.
    Iterations4  = 4,
    /// 8 iterations.
    Iterations8  = 8,
    /// 12 iterations.
    Iterations12 = 12,
    /// 16 iterations.
    Iterations16 = 16,
    /// 20 iterations.
    Iterations20 = 20,
    /// 24 iterations.
    Iterations24 = 24,
}

/// A CORDIC configuration error.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, strum::Display)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum ConfigError {
    /// The requested rate is zero or outside the frequency range for the selected source.
    UnreachableClockRate,
}

impl core::error::Error for ConfigError {}

/// An error returned by a CORDIC calculation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, strum::Display)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    /// The input is outside the supported range of the operation.
    InvalidInput,
    /// The operation does not support the configured iteration count.
    UnsupportedPrecision,
    /// The peripheral did not report completion within the polling timeout.
    ///
    /// The driver resets the peripheral before returning this error.
    Timeout,
}

impl core::error::Error for Error {}

/// A blocking CORDIC driver.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Cordic<'d, Dm: DriverMode = Blocking> {
    peripheral: CORDIC<'d>,
    _clock_guard: CordicClockGuard,
    _guard: GenericPeripheralGuard<{ Peripheral::Cordic as u8 }>,
    clock_request: ClockRequest,
    precision: Precision,
    _mode: PhantomData<Dm>,
}

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
struct CordicClockGuard;

impl CordicClockGuard {
    fn new(config: CordicFunctionClockConfig) -> Self {
        ClockTree::with(|clocks| {
            CordicInstance::Cordic.configure_function_clock(clocks, config);
            CordicInstance::Cordic.request_function_clock(clocks);
        });
        Self
    }
}

impl Drop for CordicClockGuard {
    fn drop(&mut self) {
        ClockTree::with(|clocks| CordicInstance::Cordic.release_function_clock(clocks));
    }
}

#[derive(Clone, Copy)]
#[repr(u8)]
enum Function {
    Cos     = 0,
    Sin     = 1,
    Phase   = 2,
    Modulus = 3,
    Atan    = 4,
    Cosh    = 5,
    Sinh    = 6,
    Atanh   = 7,
    Ln      = 8,
    Sqrt    = 9,
}

// These hardware input limits apply with SCALE = 1.
// See <https://github.com/espressif/esp-idf/blob/0e6b480f858c69862ffb6001d96e9eef74599a6e/docs/en/api-reference/peripherals/cordic.rst#L79-L98>.
const SINH_COSH_MAX_INPUT_Q31: u32 = (559 * (1_u64 << 31) / 1000) as u32;
const HYPERBOLIC_VECTOR_MAX_INPUT_Q31: u32 = (403 * (1_u64 << 31) / 1000) as u32;

impl Function {
    const fn scale_exponent(self) -> u8 {
        match self {
            Self::Cosh | Self::Sinh | Self::Atanh | Self::Ln => 1,
            _ => 0,
        }
    }
}

struct Arguments {
    arg1_word: u32,
    arg2_word: u32,
    two_inputs: bool,
    q31: bool,
}

impl Arguments {
    fn q15(arg1: i16, arg2: Option<i16>) -> Self {
        Self {
            arg1_word: u32::from(arg1 as u16) | (u32::from(arg2.unwrap_or(0) as u16) << 16),
            arg2_word: 0,
            two_inputs: arg2.is_some(),
            q31: false,
        }
    }

    fn q31(arg1: i32, arg2: Option<i32>) -> Self {
        Self {
            arg1_word: arg1 as u32,
            arg2_word: arg2.unwrap_or(0) as u32,
            two_inputs: arg2.is_some(),
            q31: true,
        }
    }
}

enum Outputs {
    Single,
    Pair,
}

fn validate_vector(
    x: i32,
    y: i32,
    fractional_bits: u32,
    requires_phase: bool,
) -> Result<(), Error> {
    let x = i64::from(x);
    let y = i64::from(y);
    let squared_modulus = (x * x) as u64 + (y * y) as u64;
    let one_squared = 1 << (2 * fractional_bits);
    if squared_modulus >= one_squared || (requires_phase && squared_modulus == 0) {
        return Err(Error::InvalidInput);
    }
    Ok(())
}

fn validate_magnitude(input: i32, limit: u32) -> Result<(), Error> {
    if input.unsigned_abs() > limit {
        return Err(Error::InvalidInput);
    }
    Ok(())
}

impl<'d> Cordic<'d, Blocking> {
    /// Creates a new CORDIC driver.
    ///
    /// # Errors
    ///
    /// [`ConfigError::UnreachableClockRate`] when the requested clock rate is unsupported.
    pub fn new(peripheral: CORDIC<'d>, config: Config) -> Result<Self, ConfigError> {
        let clock_request = ClockRequest::new(&config);
        // Keep the calculation clock running during the peripheral reset.
        let clock_guard = CordicClockGuard::new(clock_request.resolve()?);
        Ok(Self {
            peripheral,
            _clock_guard: clock_guard,
            _guard: GenericPeripheralGuard::new(),
            clock_request,
            precision: config.precision,
            _mode: PhantomData,
        })
    }

    /// Applies a new clock and precision configuration.
    ///
    /// # Errors
    ///
    /// [`ConfigError::UnreachableClockRate`] when the requested clock rate is unsupported.
    /// The current configuration is retained in that case.
    pub fn apply_config(&mut self, config: &Config) -> Result<(), ConfigError> {
        let clock_request = ClockRequest::new(config);
        if self.clock_request != clock_request {
            let clock_config = clock_request.resolve()?;
            ClockTree::with(|clocks| {
                let instance = CordicInstance::Cordic;
                if instance.function_clock_config(clocks) != Some(clock_config) {
                    instance.configure_function_clock(clocks, clock_config);
                }
            });
            self.clock_request = clock_request;
        }
        self.precision = config.precision;
        Ok(())
    }

    /// Returns the calculation clock rate derived from the selected source and divider.
    ///
    /// The actual rate can differ when using the internal RC oscillator.
    pub fn frequency(&self) -> Rate {
        Rate::from_hz(CordicInstance::Cordic.function_clock_frequency())
    }

    /// Returns `(sin(θ), cos(θ))` in Q1.15 for input `θ / π`.
    ///
    /// A result of +1 is represented by [`i16::MAX`].
    ///
    /// # Errors
    ///
    /// [`Error::Timeout`] when the calculation does not complete.
    pub fn sin_cos_q15(&mut self, angle: i16) -> Result<(i16, i16), Error> {
        self.calculate_q15_pair(Function::Sin, angle, None)
    }

    /// Returns `(sin(θ), cos(θ))` in Q1.31 for input `θ / π`.
    ///
    /// A result of +1 is represented by [`i32::MAX`].
    ///
    /// # Errors
    ///
    /// [`Error::Timeout`] when the calculation does not complete.
    pub fn sin_cos_q31(&mut self, angle: i32) -> Result<(i32, i32), Error> {
        self.calculate_q31_pair(Function::Sin, angle, None)
    }

    /// Returns `sin(θ)` in Q1.15 for input `θ / π`.
    ///
    /// A result of +1 is represented by [`i16::MAX`].
    /// [`Self::sin_cos_q15`] computes sine and cosine in one operation.
    ///
    /// # Errors
    ///
    /// [`Error::Timeout`] when the calculation does not complete.
    pub fn sin_q15(&mut self, angle: i16) -> Result<i16, Error> {
        self.calculate_q15(Function::Sin, angle, None)
    }

    /// Returns `sin(θ)` in Q1.31 for input `θ / π`.
    ///
    /// A result of +1 is represented by [`i32::MAX`].
    /// [`Self::sin_cos_q31`] computes sine and cosine in one operation.
    ///
    /// # Errors
    ///
    /// [`Error::Timeout`] when the calculation does not complete.
    pub fn sin_q31(&mut self, angle: i32) -> Result<i32, Error> {
        self.calculate_q31(Function::Sin, angle, None)
    }

    /// Returns `cos(θ)` in Q1.15 for input `θ / π`.
    ///
    /// A result of +1 is represented by [`i16::MAX`].
    /// [`Self::sin_cos_q15`] computes sine and cosine in one operation.
    ///
    /// # Errors
    ///
    /// [`Error::Timeout`] when the calculation does not complete.
    pub fn cos_q15(&mut self, angle: i16) -> Result<i16, Error> {
        self.calculate_q15(Function::Cos, angle, None)
    }

    /// Returns `cos(θ)` in Q1.31 for input `θ / π`.
    ///
    /// A result of +1 is represented by [`i32::MAX`].
    /// [`Self::sin_cos_q31`] computes sine and cosine in one operation.
    ///
    /// # Errors
    ///
    /// [`Error::Timeout`] when the calculation does not complete.
    pub fn cos_q31(&mut self, angle: i32) -> Result<i32, Error> {
        self.calculate_q31(Function::Cos, angle, None)
    }

    /// Returns `(atan2(y, x) / π, hypot(x, y))` in Q1.15.
    ///
    /// For `x < 0` and `y = 0`, the phase approximates +1.
    ///
    /// # Errors
    ///
    /// - [`Error::InvalidInput`] when the vector is zero or has a modulus of at least one.
    /// - [`Error::Timeout`] when the calculation does not complete.
    pub fn phase_modulus_q15(&mut self, x: i16, y: i16) -> Result<(i16, i16), Error> {
        validate_vector(i32::from(x), i32::from(y), 15, true)?;
        self.calculate_q15_pair(Function::Phase, x, Some(y))
    }

    /// Returns `(atan2(y, x) / π, hypot(x, y))` in Q1.31.
    ///
    /// For `x < 0` and `y = 0`, the phase approximates +1.
    ///
    /// # Errors
    ///
    /// - [`Error::InvalidInput`] when the vector is zero or has a modulus of at least one.
    /// - [`Error::Timeout`] when the calculation does not complete.
    pub fn phase_modulus_q31(&mut self, x: i32, y: i32) -> Result<(i32, i32), Error> {
        validate_vector(x, y, 31, true)?;
        self.calculate_q31_pair(Function::Phase, x, Some(y))
    }

    /// Returns `atan2(y, x) / π` in Q1.15.
    ///
    /// For `x < 0` and `y = 0`, the phase approximates +1.
    /// [`Self::phase_modulus_q15`] computes the phase and modulus in one operation.
    ///
    /// # Errors
    ///
    /// - [`Error::InvalidInput`] when the vector is zero or has a modulus of at least one.
    /// - [`Error::Timeout`] when the calculation does not complete.
    pub fn phase_q15(&mut self, x: i16, y: i16) -> Result<i16, Error> {
        validate_vector(i32::from(x), i32::from(y), 15, true)?;
        self.calculate_q15(Function::Phase, x, Some(y))
    }

    /// Returns `atan2(y, x) / π` in Q1.31.
    ///
    /// For `x < 0` and `y = 0`, the phase approximates +1.
    /// [`Self::phase_modulus_q31`] computes the phase and modulus in one operation.
    ///
    /// # Errors
    ///
    /// - [`Error::InvalidInput`] when the vector is zero or has a modulus of at least one.
    /// - [`Error::Timeout`] when the calculation does not complete.
    pub fn phase_q31(&mut self, x: i32, y: i32) -> Result<i32, Error> {
        validate_vector(x, y, 31, true)?;
        self.calculate_q31(Function::Phase, x, Some(y))
    }

    /// Returns `hypot(x, y)` in Q1.15.
    ///
    /// [`Self::phase_modulus_q15`] computes the phase and modulus in one operation.
    ///
    /// # Errors
    ///
    /// - [`Error::InvalidInput`] when the vector has a modulus of at least one.
    /// - [`Error::Timeout`] when the calculation does not complete.
    pub fn modulus_q15(&mut self, x: i16, y: i16) -> Result<i16, Error> {
        validate_vector(i32::from(x), i32::from(y), 15, false)?;
        self.calculate_q15(Function::Modulus, x, Some(y))
    }

    /// Returns `hypot(x, y)` in Q1.31.
    ///
    /// [`Self::phase_modulus_q31`] computes the phase and modulus in one operation.
    ///
    /// # Errors
    ///
    /// - [`Error::InvalidInput`] when the vector has a modulus of at least one.
    /// - [`Error::Timeout`] when the calculation does not complete.
    pub fn modulus_q31(&mut self, x: i32, y: i32) -> Result<i32, Error> {
        validate_vector(x, y, 31, false)?;
        self.calculate_q31(Function::Modulus, x, Some(y))
    }

    /// Returns `atan(x) / π` in Q1.15.
    ///
    /// # Errors
    ///
    /// [`Error::Timeout`] when the calculation does not complete.
    pub fn atan_q15(&mut self, x: i16) -> Result<i16, Error> {
        self.calculate_q15(Function::Atan, x, None)
    }

    /// Returns `atan(x) / π` in Q1.31.
    ///
    /// # Errors
    ///
    /// [`Error::Timeout`] when the calculation does not complete.
    pub fn atan_q31(&mut self, x: i32) -> Result<i32, Error> {
        self.calculate_q31(Function::Atan, x, None)
    }

    /// Returns `(sinh(x) / 2, cosh(x) / 2)` in Q1.15 for input `x / 2`.
    ///
    /// # Errors
    ///
    /// - [`Error::InvalidInput`] when `|x / 2| > 0.559`.
    /// - [`Error::Timeout`] when the calculation does not complete.
    pub fn sinh_cosh_q15(&mut self, x_over_two: i16) -> Result<(i16, i16), Error> {
        validate_magnitude(i32::from(x_over_two), SINH_COSH_MAX_INPUT_Q31 >> 16)?;
        self.calculate_q15_pair(Function::Sinh, x_over_two, None)
    }

    /// Returns `(sinh(x) / 2, cosh(x) / 2)` in Q1.31 for input `x / 2`.
    ///
    /// # Errors
    ///
    /// - [`Error::InvalidInput`] when `|x / 2| > 0.559`.
    /// - [`Error::Timeout`] when the calculation does not complete.
    pub fn sinh_cosh_q31(&mut self, x_over_two: i32) -> Result<(i32, i32), Error> {
        validate_magnitude(x_over_two, SINH_COSH_MAX_INPUT_Q31)?;
        self.calculate_q31_pair(Function::Sinh, x_over_two, None)
    }

    /// Returns `sinh(x) / 2` in Q1.15 for input `x / 2`.
    ///
    /// [`Self::sinh_cosh_q15`] computes sinh and cosh in one operation.
    ///
    /// # Errors
    ///
    /// - [`Error::InvalidInput`] when `|x / 2| > 0.559`.
    /// - [`Error::Timeout`] when the calculation does not complete.
    pub fn sinh_q15(&mut self, x_over_two: i16) -> Result<i16, Error> {
        validate_magnitude(i32::from(x_over_two), SINH_COSH_MAX_INPUT_Q31 >> 16)?;
        self.calculate_q15(Function::Sinh, x_over_two, None)
    }

    /// Returns `sinh(x) / 2` in Q1.31 for input `x / 2`.
    ///
    /// [`Self::sinh_cosh_q31`] computes sinh and cosh in one operation.
    ///
    /// # Errors
    ///
    /// - [`Error::InvalidInput`] when `|x / 2| > 0.559`.
    /// - [`Error::Timeout`] when the calculation does not complete.
    pub fn sinh_q31(&mut self, x_over_two: i32) -> Result<i32, Error> {
        validate_magnitude(x_over_two, SINH_COSH_MAX_INPUT_Q31)?;
        self.calculate_q31(Function::Sinh, x_over_two, None)
    }

    /// Returns `cosh(x) / 2` in Q1.15 for input `x / 2`.
    ///
    /// [`Self::sinh_cosh_q15`] computes sinh and cosh in one operation.
    ///
    /// # Errors
    ///
    /// - [`Error::InvalidInput`] when `|x / 2| > 0.559`.
    /// - [`Error::Timeout`] when the calculation does not complete.
    pub fn cosh_q15(&mut self, x_over_two: i16) -> Result<i16, Error> {
        validate_magnitude(i32::from(x_over_two), SINH_COSH_MAX_INPUT_Q31 >> 16)?;
        self.calculate_q15(Function::Cosh, x_over_two, None)
    }

    /// Returns `cosh(x) / 2` in Q1.31 for input `x / 2`.
    ///
    /// [`Self::sinh_cosh_q31`] computes sinh and cosh in one operation.
    ///
    /// # Errors
    ///
    /// - [`Error::InvalidInput`] when `|x / 2| > 0.559`.
    /// - [`Error::Timeout`] when the calculation does not complete.
    pub fn cosh_q31(&mut self, x_over_two: i32) -> Result<i32, Error> {
        validate_magnitude(x_over_two, SINH_COSH_MAX_INPUT_Q31)?;
        self.calculate_q31(Function::Cosh, x_over_two, None)
    }

    /// Returns `atanh(x) / 2` in Q1.15 for input `x / 2`.
    ///
    /// # Errors
    ///
    /// - [`Error::InvalidInput`] when `|x / 2| > 0.403`.
    /// - [`Error::Timeout`] when the calculation does not complete.
    pub fn atanh_q15(&mut self, x_over_two: i16) -> Result<i16, Error> {
        validate_magnitude(i32::from(x_over_two), HYPERBOLIC_VECTOR_MAX_INPUT_Q31 >> 16)?;
        self.calculate_q15(Function::Atanh, x_over_two, None)
    }

    /// Returns `atanh(x) / 2` in Q1.31 for input `x / 2`.
    ///
    /// # Errors
    ///
    /// - [`Error::InvalidInput`] when `|x / 2| > 0.403`.
    /// - [`Error::Timeout`] when the calculation does not complete.
    pub fn atanh_q31(&mut self, x_over_two: i32) -> Result<i32, Error> {
        validate_magnitude(x_over_two, HYPERBOLIC_VECTOR_MAX_INPUT_Q31)?;
        self.calculate_q31(Function::Atanh, x_over_two, None)
    }

    /// Returns `ln(x) / 4` from the ratio `(x - 1) / (2 * (x + 1))`
    /// for positive `x`, with the input and result in Q1.15.
    ///
    /// # Errors
    ///
    /// - [`Error::InvalidInput`] when `|ratio| > 0.403`.
    /// - [`Error::Timeout`] when the calculation does not complete.
    pub fn ln_from_ratio_q15(&mut self, ratio: i16) -> Result<i16, Error> {
        validate_magnitude(i32::from(ratio), HYPERBOLIC_VECTOR_MAX_INPUT_Q31 >> 16)?;
        self.calculate_q15(Function::Ln, ratio, None)
    }

    /// Returns `ln(x) / 4` from the ratio `(x - 1) / (2 * (x + 1))`
    /// for positive `x`, with the input and result in Q1.31.
    ///
    /// # Errors
    ///
    /// - [`Error::InvalidInput`] when `|ratio| > 0.403`.
    /// - [`Error::Timeout`] when the calculation does not complete.
    pub fn ln_from_ratio_q31(&mut self, ratio: i32) -> Result<i32, Error> {
        validate_magnitude(ratio, HYPERBOLIC_VECTOR_MAX_INPUT_Q31)?;
        self.calculate_q31(Function::Ln, ratio, None)
    }

    /// Returns `sqrt(x)` in Q1.15.
    ///
    /// # Errors
    ///
    /// - [`Error::InvalidInput`] when `x` is negative.
    /// - [`Error::UnsupportedPrecision`] when the iteration count exceeds 12.
    /// - [`Error::Timeout`] when the calculation does not complete.
    pub fn sqrt_q15(&mut self, x: i16) -> Result<i16, Error> {
        let Some(root_shift) = self.sqrt_shift(i32::from(x), x.leading_zeros())? else {
            return Ok(0);
        };
        let root = self.calculate_q15(Function::Sqrt, x << (2 * root_shift), None)?;
        Ok(root >> root_shift)
    }

    /// Returns `sqrt(x)` in Q1.31.
    ///
    /// # Errors
    ///
    /// - [`Error::InvalidInput`] when `x` is negative.
    /// - [`Error::UnsupportedPrecision`] when the iteration count exceeds 12.
    /// - [`Error::Timeout`] when the calculation does not complete.
    pub fn sqrt_q31(&mut self, x: i32) -> Result<i32, Error> {
        let Some(root_shift) = self.sqrt_shift(x, x.leading_zeros())? else {
            return Ok(0);
        };
        let root = self.calculate_q31(Function::Sqrt, x << (2 * root_shift), None)?;
        Ok(root >> root_shift)
    }

    fn sqrt_shift(&self, x: i32, leading_zeros: u32) -> Result<Option<u32>, Error> {
        match x {
            ..0 => Err(Error::InvalidInput),
            0 => {
                self.iterations(Function::Sqrt)?;
                Ok(None)
            }
            // Normalize to [0.25, 1) for the hardware input range of [0.1069, 1).
            // See <https://github.com/espressif/esp-idf/blob/0e6b480f858c69862ffb6001d96e9eef74599a6e/docs/en/api-reference/peripherals/cordic.rst#L99-L103>.
            _ => Ok(Some((leading_zeros - 1) / 2)),
        }
    }

    fn iterations(&self, function: Function) -> Result<u8, Error> {
        let max = match function {
            Function::Sqrt => Precision::Iterations12,
            _ => Precision::Iterations24,
        } as u8;
        let iterations = match self.precision {
            Precision::Maximum => max,
            precision => precision as u8,
        };
        if iterations > max {
            return Err(Error::UnsupportedPrecision);
        }
        Ok(iterations)
    }

    fn calculate_q15(
        &mut self,
        function: Function,
        arg1: i16,
        arg2: Option<i16>,
    ) -> Result<i16, Error> {
        let (first, _) =
            self.calculate_raw(function, Arguments::q15(arg1, arg2), Outputs::Single)?;
        Ok(first as i16)
    }

    fn calculate_q15_pair(
        &mut self,
        function: Function,
        arg1: i16,
        arg2: Option<i16>,
    ) -> Result<(i16, i16), Error> {
        let (packed, _) =
            self.calculate_raw(function, Arguments::q15(arg1, arg2), Outputs::Pair)?;
        Ok((packed as i16, (packed >> 16) as i16))
    }

    fn calculate_q31(
        &mut self,
        function: Function,
        arg1: i32,
        arg2: Option<i32>,
    ) -> Result<i32, Error> {
        let (first, _) =
            self.calculate_raw(function, Arguments::q31(arg1, arg2), Outputs::Single)?;
        Ok(first as i32)
    }

    fn calculate_q31_pair(
        &mut self,
        function: Function,
        arg1: i32,
        arg2: Option<i32>,
    ) -> Result<(i32, i32), Error> {
        let (first, second) =
            self.calculate_raw(function, Arguments::q31(arg1, arg2), Outputs::Pair)?;
        Ok((first as i32, second as i32))
    }

    fn calculate_raw(
        &mut self,
        function: Function,
        arguments: Arguments,
        outputs: Outputs,
    ) -> Result<(u32, u32), Error> {
        const FAST_POLL_ATTEMPTS: usize = 1024;
        const SLOW_POLL_TIMEOUT: Duration = Duration::from_millis(2);

        let iterations = self.iterations(function)?;
        let regs = self.peripheral.register_block();
        let Arguments {
            arg1_word,
            arg2_word,
            two_inputs,
            q31,
        } = arguments;
        let pair = matches!(outputs, Outputs::Pair);
        regs.csr_cfg().write(|w| unsafe {
            w.func().bits(function as u8);
            w.pres().bits(iterations / 4 - 1);
            w.scale().bits(function.scale_exponent());
            w.arg_num().bit(two_inputs);
            w.res_num().bit(pair);
            w.arg_size().bit(q31);
            w.res_size().bit(q31)
        });
        regs.arg1().write(|w| unsafe { w.data().bits(arg1_word) });
        if q31 && two_inputs {
            regs.arg2().write(|w| unsafe { w.data().bits(arg2_word) });
        }
        regs.csr_cfg().modify(|_, w| w.update_flag().set_bit());

        let is_ready = || regs.csr_cfg().read().res_rdy_flag().bit_is_set();
        if !(0..FAST_POLL_ATTEMPTS).any(|_| is_ready()) {
            let start = Instant::now();
            while !is_ready() {
                if start.elapsed() >= SLOW_POLL_TIMEOUT {
                    PeripheralClockControl::reset(Peripheral::Cordic);
                    return Err(Error::Timeout);
                }
            }
        }

        let first = regs.res1().read().data().bits();
        let second = if q31 && pair {
            regs.res2().read().data().bits()
        } else {
            0
        };
        Ok((first, second))
    }
}
