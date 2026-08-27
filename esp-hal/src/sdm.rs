#![cfg_attr(docsrs, procmacros::doc_replace)]

//! Sigma-delta modulation peripheral.
//!
//! The sigma-delta modulator produces a pulse-density modulated output on a
//! GPIO matrix signal. Each channel is a peripheral singleton (`SDM_CH0`,
//! `SDM_CH1`, …). Configure a channel with a carrier frequency and pulse
//! density, then connect it to one output pin.
//!
//! ## Examples
//!
//! Generate a sigma-delta output signal on a GPIO pin.
//!
//! ```rust, no_run
//! # {before_snippet}
//! use esp_hal::{
//!     sdm::{Channel, ChannelConfig},
//!     time::Rate,
//! };
//!
//! let config = ChannelConfig::new()
//!     // Select the prescaler that produces the closest available frequency.
//!     .with_frequency(Rate::from_khz(500))?
//!     .with_duty(128);
//! let mut channel = Channel::new(peripherals.SDM_CH0, peripherals.GPIO2, config);
//!
//! channel.set_duty(192); // duty ranges from 0 to 255
//! channel.set_pulse_density(0); // pulse density ranges from -128 to 127
//!
//! # {after_snippet}
//! ```
//!
//! ## Clock source
#![cfg_attr(
    not(soc_has_clock_node_iomux_function_clock),
    doc = r#"
The SDM function clock is derived from the APB clock.
"#
)]
#![cfg_attr(
    soc_has_clock_node_iomux_function_clock,
    doc = r#"
The SDM function clock is derived from the global `IOMUX_FUNCTION_CLOCK`. Its
source is shared by every consumer of that clock and is therefore configured
globally instead of through [`Channel`]. The available sources and the default for
the selected target are listed by
[`IomuxFunctionClockConfig`](crate::clock::ll::IomuxFunctionClockConfig).

The source can be selected as part of the global clock configuration before initializing the HAL.
"#
)]
//! Each channel's prescaler divides this function clock to produce its output
//! frequency.

use core::fmt;

use crate::{
    gpio::{
        OutputConfig,
        OutputSignal,
        PinGuard,
        interconnect::{OutputSignal as GpioOutputSignal, PeripheralOutput},
    },
    peripherals::GPIO_SD,
    soc::clocks::{ClockTree, SdmInstance},
    system::{GenericPeripheralGuard, Peripheral},
    time::Rate,
};

/// Immutable per-channel metadata owned by each `SDM_CH*` singleton.
#[doc(hidden)]
pub struct ChannelInfo {
    /// Hardware channel index used to select the register bank.
    channel: usize,
    /// GPIO matrix output signal for this channel.
    signal: OutputSignal,
}

/// A peripheral singleton compatible with the sigma-delta driver.
#[doc(hidden)]
pub trait Instance: crate::private::Sealed + any::Degrade {
    /// Returns the metadata for this channel.
    fn info(&self) -> &'static ChannelInfo;
}

impl Instance for AnySdmChannel<'_> {
    fn info(&self) -> &'static ChannelInfo {
        any::delegate!(self, channel => { channel.info() })
    }
}

for_each_sdm_channel! {
    (channels $(($num:literal, $peri:ident, $variant:ident, $signal:ident)),*) => {
        crate::any_peripheral! {
            /// Any SDM channel.
            pub peripheral AnySdmChannel<'d> {
                $(
                    $variant(crate::peripherals::$peri<'d>),
                )*
            }
        }
    };

    ($num:literal, $peri:ident, $variant:ident, $signal:ident) => {
        impl crate::sdm::Instance for crate::peripherals::$peri<'_> {
            fn info(&self) -> &'static ChannelInfo {
                static INFO: ChannelInfo = ChannelInfo {
                    channel: $num,
                    signal: OutputSignal::$signal,
                };
                &INFO
            }
        }
    };
}

/// Sigma-delta configuration or runtime error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    /// The requested frequency cannot be represented by the hardware prescaler.
    UnreachableTargetFrequency,
    /// The prescaler is outside the supported range.
    PrescalerOutOfRange,
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::UnreachableTargetFrequency => f.write_str("Unreachable target frequency"),
            Self::PrescalerOutOfRange => f.write_str("Prescaler out of range"),
        }
    }
}

impl core::error::Error for Error {}

/// Sigma-delta channel configuration.
///
/// The hardware stores the prescaler and pulse density in the same register,
/// so applying a complete channel configuration can update both fields with a
/// single register write.
#[derive(Default, Debug, Clone, Copy, PartialEq, Eq, procmacros::BuilderLite)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct ChannelConfig {
    /// Raw hardware prescaler value.
    #[builder_lite(skip)]
    raw_prescaler: u8,

    /// Pulse density in the hardware range `-128..=127`.
    pulse_density: i8,
}

impl ChannelConfig {
    /// Creates a new channel configuration with the default prescaler and pulse
    /// density.
    pub const fn new() -> Self {
        Self {
            raw_prescaler: 0,
            pulse_density: 0,
        }
    }

    /// Sets the requested output frequency.
    ///
    /// Selects the prescaler that produces the closest output frequency using
    /// the currently configured SDM clock source.
    ///
    /// # Errors
    ///
    /// [`Error::UnreachableTargetFrequency`] when no hardware
    /// prescaler can represent the requested frequency.
    pub fn with_frequency(mut self, frequency: Rate) -> Result<Self, Error> {
        self.raw_prescaler = raw_prescaler(prescaler_from_frequency(frequency)?);
        Ok(self)
    }

    /// Sets the hardware prescaler.
    ///
    /// The hardware divider range is `1..=256`.
    ///
    /// # Errors
    ///
    /// [`Error::PrescalerOutOfRange`] when `prescaler` is not in
    /// `1..=256`.
    pub fn with_prescaler(mut self, prescaler: u16) -> Result<Self, Error> {
        check_prescaler(prescaler)?;
        self.raw_prescaler = raw_prescaler(prescaler);
        Ok(self)
    }

    /// Sets duty cycle. `0` maps to the minimum density and `255` maps to the
    /// maximum density.
    pub const fn with_duty(mut self, duty: u8) -> Self {
        self.pulse_density = duty_to_density(duty);
        self
    }
}

/// A connected sigma-delta channel.
///
/// Dropping a channel disconnects its output pin and releases the SDM clock
/// guard.
#[derive(Debug)]
pub struct Channel<'d> {
    channel: AnySdmChannel<'d>,
    _pin_guard: PinGuard,
    _clock_guard: SdmClockGuard,
}

impl<'d> Channel<'d> {
    /// Creates a new connected sigma-delta channel.
    pub fn new(
        channel: impl Instance + 'd,
        pin: impl PeripheralOutput<'d>,
        config: ChannelConfig,
    ) -> Self {
        let info = channel.info();
        let clock_guard = SdmClockGuard::new();

        let mut this = Self {
            channel: channel.degrade(),
            _pin_guard: connect_pin(info.signal, pin),
            _clock_guard: clock_guard,
        };
        this.apply_config(&config);
        this
    }

    fn index(&self) -> usize {
        self.channel.info().channel
    }

    /// Applies a new channel configuration.
    pub fn apply_config(&mut self, config: &ChannelConfig) {
        GPIO_SD::regs().sigmadelta(self.index()).write(|w| unsafe {
            cfg_select! {
                esp32c5 => {
                    w.sd_in().bits(config.pulse_density as u8);
                    w.sd_prescale().bits(config.raw_prescaler)
                }
                _ => {
                    w.in_().bits(config.pulse_density as u8);
                    w.prescale().bits(config.raw_prescaler)
                }
            }
        });
    }

    /// Sets raw pulse density.
    ///
    /// The value ranges from `-128` to `127`.
    pub fn set_pulse_density(&mut self, density: i8) {
        GPIO_SD::regs()
            .sigmadelta(self.index())
            .modify(|_, w| unsafe {
                cfg_select! {
                    esp32c5 => w.sd_in().bits(density as u8),
                    _ => w.in_().bits(density as u8),
                }
            });
    }

    /// Sets duty cycle. `0` maps to the minimum density and `255` maps to the
    /// maximum density.
    pub fn set_duty(&mut self, duty: u8) {
        self.set_pulse_density(duty_to_density(duty))
    }

    /// Reads the hardware prescaler.
    ///
    /// The returned value is in the hardware divider range `1..=256`.
    pub fn prescaler(&self) -> u16 {
        let reg = GPIO_SD::regs().sigmadelta(self.index()).read();
        let bits = cfg_select! {
            esp32c5 => reg.sd_prescale().bits(),
            _ => reg.prescale().bits(),
        };

        bits as u16 + 1
    }

    /// Reads the raw pulse density.
    ///
    /// The returned value is in the hardware range `-128..=127`.
    pub fn pulse_density(&self) -> i8 {
        let reg = GPIO_SD::regs().sigmadelta(self.index()).read();
        let bits = cfg_select! {
            esp32c5 => reg.sd_in().bits(),
            _ => reg.in_().bits(),
        };

        bits as i8
    }
}

fn connect_pin<'d>(signal: OutputSignal, pin: impl PeripheralOutput<'d>) -> PinGuard {
    let pin: GpioOutputSignal<'d> = pin.into();
    pin.apply_output_config(&OutputConfig::default());
    pin.set_output_enable(true);
    pin.connect_with_guard(signal)
}

#[derive(Debug)]
struct SdmClockGuard {
    // Fields are dropped in declaration order. Release the function clock
    // while the GPIO_SD register clock is still available.
    _function_clock: SdmFunctionClockGuard,
    _peripheral: GenericPeripheralGuard<{ Peripheral::GpioSd as u8 }>,
}

impl SdmClockGuard {
    fn new() -> Self {
        let peripheral = GenericPeripheralGuard::new();
        let function_clock = SdmFunctionClockGuard::new();
        Self {
            _function_clock: function_clock,
            _peripheral: peripheral,
        }
    }
}

#[derive(Debug)]
struct SdmFunctionClockGuard;

impl SdmFunctionClockGuard {
    fn new() -> Self {
        ClockTree::with(|clocks| SdmInstance::GpioSd.request_function_clock(clocks));
        Self
    }
}

impl Drop for SdmFunctionClockGuard {
    fn drop(&mut self) {
        ClockTree::with(|clocks| SdmInstance::GpioSd.release_function_clock(clocks));
    }
}

fn prescaler_from_frequency(frequency: Rate) -> Result<u16, Error> {
    let source_frequency = SdmInstance::GpioSd.function_clock_frequency() as u64;
    let requested_frequency = frequency.as_hz() as u64;

    if requested_frequency == 0
        || requested_frequency > source_frequency
        || requested_frequency * 256 < source_frequency
    {
        return Err(Error::UnreachableTargetFrequency);
    }

    // The closest output must be produced by one of the two integers around
    // the ideal prescaler.
    let lower = source_frequency / requested_frequency;
    let upper = lower + 1;
    if upper > 256 {
        return Ok(lower as u16);
    }

    // Compare |source / prescaler - requested| without truncating either
    // resulting frequency.
    let lower_error = (source_frequency - requested_frequency * lower) * upper;
    let upper_error = (requested_frequency * upper - source_frequency) * lower;

    Ok(if lower_error <= upper_error {
        lower as u16
    } else {
        upper as u16
    })
}

fn check_prescaler(prescaler: u16) -> Result<(), Error> {
    if (1..=256).contains(&prescaler) {
        Ok(())
    } else {
        Err(Error::PrescalerOutOfRange)
    }
}

fn raw_prescaler(prescaler: u16) -> u8 {
    (prescaler - 1) as u8
}

const fn duty_to_density(duty: u8) -> i8 {
    duty.wrapping_sub(128) as i8
}
