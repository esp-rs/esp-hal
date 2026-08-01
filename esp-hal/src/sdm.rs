#![cfg_attr(docsrs, procmacros::doc_replace)]

//! Sigma-delta modulation peripheral.
//!
//! The sigma-delta modulator produces a pulse-density modulated output on a
//! GPIO matrix signal. Each channel can be configured with a carrier frequency
//! and pulse density, then routed to one output pin.
//!
//! ## Examples
//!
//! Generate a sigma-delta output signal on a GPIO pin.
//!
//! ```rust, no_run
//! # {before_snippet}
//! use esp_hal::{sdm::Sdm, time::Rate};
//!
//! let mut sdm = Sdm::new(peripherals.GPIO_SD);
//! let config = sdm
//!     .channel_config()
//!     // Select the prescaler that produces the closest available frequency.
//!     .with_frequency(Rate::from_khz(500))?
//!     .with_duty(128);
//! let mut channel = sdm.channel0.connect(peripherals.GPIO2, config);
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
globally instead of through [`Sdm`]. The available sources and the default for
the selected target are listed by
[`IomuxFunctionClockConfig`](crate::clock::ll::IomuxFunctionClockConfig).

The source can be selected as part of the global clock configuration before initializing the HAL:

```rust, no_run
use esp_hal::{
    Config,
    clock::{ClockConfig, ll::IomuxFunctionClockConfig},
};

let clock_config = ClockConfig {
    iomux_function_clock: Some(IomuxFunctionClockConfig::XtalClk),
    ..ClockConfig::default()
};
let peripherals = esp_hal::init(Config::default().with_cpu_clock(clock_config));
```

The low-level clock-tree API can also change the source at runtime:

```rust, no_run
use esp_hal::clock::ll::{
    ClockTree,
    IomuxFunctionClockConfig,
    configure_iomux_function_clock,
};

ClockTree::with(|clocks| {
    configure_iomux_function_clock(clocks, IomuxFunctionClockConfig::XtalClk);
});
```

Changing this global source affects every active consumer of `IOMUX_FUNCTION_CLOCK`. Existing peripheral dividers are not automatically recalculated.
"#
)]
//! Each channel's prescaler divides this function clock to produce its output
//! frequency.

use core::{fmt, marker::PhantomData};

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

for_each_sdm_channel!(
    (channels $(($ch:literal, $signal:ident)),*) => {
        paste::paste! {
            /// Sigma-delta peripheral.
            ///
            /// This type only owns the SDM peripheral token and exposes the hardware
            /// channel creators. Moving individual channel creators out of this
            /// collection is supported.
            #[derive(Debug)]
            #[non_exhaustive]
            pub struct Sdm<'d> {
                _instance: GPIO_SD<'d>,
                $(
                    #[doc = concat!("Channel ", stringify!($ch), " creator.")]
                    pub [<channel $ch>]: ChannelCreator<'d>,
                )*
            }

            impl<'d> Sdm<'d> {
                /// Creates a new sigma-delta peripheral driver.
                pub fn new(instance: GPIO_SD<'d>) -> Self {
                    Self {
                        _instance: instance,
                        $(
                            [<channel $ch>]: ChannelCreator::new($ch),
                        )*
                    }
                }

                /// Creates a channel configuration builder.
                pub const fn channel_config(&self) -> ChannelConfigBuilder {
                    // This ensures that channel configs can only be created after SDM has
                    // been initialized. `with_frequency` relies on the function clock that
                    // was configured during system initialization.
                    ChannelConfigBuilder::new()
                }
            }
        }
    };
);

for_each_sdm_channel!(
    (channels $(($ch:literal, $signal:ident)),*) => {
        fn output_signal(channel: usize) -> OutputSignal {
            match channel {
                $(
                    $ch => OutputSignal::$signal,
                )*
                _ => unreachable!("SDM channel index out of range"),
            }
        }
    };
);

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
            Self::UnreachableTargetFrequency => f.write_str("unreachable target frequency"),
            Self::PrescalerOutOfRange => f.write_str("prescaler out of range"),
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
    /// Sets the hardware prescaler.
    ///
    /// The hardware divider range is `1..=256`.
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

/// Builds a sigma-delta channel configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct ChannelConfigBuilder {
    config: ChannelConfig,
}

impl ChannelConfigBuilder {
    const fn new() -> Self {
        // see `Sdm::channel_config()` for why this is not public
        Self {
            config: ChannelConfig {
                raw_prescaler: 0,
                pulse_density: 0,
            },
        }
    }

    /// Sets the requested output frequency.
    ///
    /// Selects the prescaler that produces the closest output frequency using
    /// the currently configured SDM clock source.
    pub fn with_frequency(mut self, frequency: Rate) -> Result<ChannelConfig, Error> {
        self.config.raw_prescaler = raw_prescaler(prescaler_from_frequency(frequency)?);
        Ok(self.config)
    }

    /// Sets the hardware prescaler.
    ///
    /// The hardware divider range is `1..=256`.
    pub fn with_prescaler(mut self, prescaler: u16) -> Result<ChannelConfig, Error> {
        self.config = self.config.with_prescaler(prescaler)?;
        Ok(self.config)
    }

    /// Returns the default channel configuration.
    pub const fn build(self) -> ChannelConfig {
        self.config
    }
}

/// Creates a connected sigma-delta channel.
///
/// Channel creators are exposed by [`Sdm`]. Calling [`connect`](Self::connect)
/// mutably borrows the creator for the lifetime of the active [`Channel`],
/// which prevents accidentally connecting the same channel twice.
#[derive(Debug)]
pub struct ChannelCreator<'d> {
    channel: usize,
    // Conceptually retains the main driver even when this creator is moved out
    // of it, preventing SDM from being reinitialized while the creator exists.
    _sdm: PhantomData<Sdm<'d>>,
}

impl ChannelCreator<'_> {
    const fn new(channel: usize) -> Self {
        Self {
            channel,
            _sdm: PhantomData,
        }
    }

    /// Configures this channel and connects it to an output pin.
    pub fn connect<'a, 'd>(
        &'a mut self,
        pin: impl PeripheralOutput<'d>,
        config: ChannelConfig,
    ) -> Channel<'a> {
        let clock_guard = SdmClockGuard::new();
        write_config_raw(self.channel, config);

        Channel {
            channel: self.channel,
            _creator: PhantomData,
            _pin_guard: connect_pin(self.channel, pin),
            _clock_guard: clock_guard,
        }
    }
}

/// A connected sigma-delta channel.
///
/// Dropping a channel disconnects its output pin, releases the SDM clock guard,
/// and makes its [`ChannelCreator`] available again.
#[derive(Debug)]
pub struct Channel<'a> {
    channel: usize,
    _creator: PhantomData<&'a mut ChannelCreator<'a>>,
    _pin_guard: PinGuard,
    _clock_guard: SdmClockGuard,
}

impl Channel<'_> {
    /// Applies a new channel configuration.
    pub fn apply_config(&mut self, config: &ChannelConfig) {
        write_config_raw(self.channel, *config);
    }

    /// Sets raw pulse density.
    ///
    /// The value ranges from `-128` to `127`.
    pub fn set_pulse_density(&mut self, density: i8) {
        modify_pulse_density_raw(self.channel, density);
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
        prescaler_raw(self.channel) + 1
    }

    /// Reads the raw pulse density.
    ///
    /// The returned value is in the hardware range `-128..=127`.
    pub fn pulse_density(&self) -> i8 {
        pulse_density_raw(self.channel)
    }
}

fn connect_pin<'d>(channel: usize, pin: impl PeripheralOutput<'d>) -> PinGuard {
    let pin: GpioOutputSignal<'d> = pin.into();
    pin.apply_output_config(&OutputConfig::default());
    pin.set_output_enable(true);
    pin.connect_with_guard(output_signal(channel))
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

fn write_config_raw(channel: usize, config: ChannelConfig) {
    // ESP32-C5's PAC names these fields `sd_in`/`sd_prescale`; the other
    // supported PACs name them `in`/`prescale`.
    let prescaler = config.raw_prescaler as _;
    let density = config.pulse_density as _;
    let sd = GPIO_SD::regs();

    #[cfg(esp32c5)]
    sd.sigmadelta(channel)
        .write(|w| unsafe { w.sd_in().bits(density).sd_prescale().bits(prescaler) });

    #[cfg(not(esp32c5))]
    sd.sigmadelta(channel)
        .write(|w| unsafe { w.in_().bits(density).prescale().bits(prescaler) });
}

fn modify_pulse_density_raw(channel: usize, density: i8) {
    // ESP32-C5's PAC names this field `sd_in`; the other supported PACs name it `in`.
    let sd = GPIO_SD::regs();

    #[cfg(esp32c5)]
    sd.sigmadelta(channel)
        .modify(|_, w| unsafe { w.sd_in().bits(density as _) });

    #[cfg(not(esp32c5))]
    sd.sigmadelta(channel)
        .modify(|_, w| unsafe { w.in_().bits(density as _) });
}

fn prescaler_raw(channel: usize) -> u16 {
    let sd = GPIO_SD::regs();

    #[cfg(esp32c5)]
    let bits = sd.sigmadelta(channel).read().sd_prescale().bits();

    #[cfg(not(esp32c5))]
    let bits = sd.sigmadelta(channel).read().prescale().bits();

    bits as u16
}

fn pulse_density_raw(channel: usize) -> i8 {
    let sd = GPIO_SD::regs();

    #[cfg(esp32c5)]
    let bits = sd.sigmadelta(channel).read().sd_in().bits();

    #[cfg(not(esp32c5))]
    let bits = sd.sigmadelta(channel).read().in_().bits();

    bits as i8
}
