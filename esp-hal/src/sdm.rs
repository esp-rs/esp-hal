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
//! use esp_hal::{
//!     sdm::{Sdm, SdmConfig},
//!     time::Rate,
//! };
//!
//! let mut sdm = Sdm::new(peripherals.GPIO_SD, SdmConfig::default());
//! let config = sdm
//!     .channel_config()
//!     .with_frequency(Rate::from_khz(500))?
//!     .with_duty(128);
//! let mut channel = sdm.channel0.connect(peripherals.GPIO2, config);
//!
//! channel.set_duty(192); // duty ranges from 0 to 255
//! channel.set_pulse_density(0); // pulse density ranges from -128 to 127
//! //
//! # {after_snippet}
//! ```

use core::{fmt, marker::PhantomData};

#[cfg(soc_has_clock_node_iomux_function_clock)]
use crate::soc::clocks::{self, ClockTree, IomuxFunctionClockConfig};
use crate::{
    gpio::{
        OutputConfig,
        OutputSignal,
        PinGuard,
        interconnect::{OutputSignal as GpioOutputSignal, PeripheralOutput},
    },
    peripherals::GPIO_SD,
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
                    pub [<channel $ch>]: ChannelCreator,
                )*
            }

            impl<'d> Sdm<'d> {
                /// Creates a new sigma-delta peripheral driver.
                ///
                /// The SDM clock source is shared by all SDM channels through the IO_MUX
                /// clock, so it is selected here instead of being configurable per channel.
                pub fn new(instance: GPIO_SD<'d>, config: SdmConfig) -> Self {
                    #[cfg(soc_has_clock_node_iomux_function_clock)]
                    ClockTree::with(|clocks| {
                        clocks::configure_iomux_function_clock(
                            clocks,
                            config.clock_source.into(),
                        );
                    });
                    #[cfg(not(soc_has_clock_node_iomux_function_clock))]
                    let _ = config;

                    Self {
                        _instance: instance,
                        $(
                            [<channel $ch>]: ChannelCreator::new($ch),
                        )*
                    }
                }

                /// Creates a channel configuration builder.
                pub const fn channel_config(&self) -> ChannelConfigBuilder {
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

/// Sigma-delta peripheral configuration.
#[derive(Default, Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct SdmConfig {
    /// Clock source used by the shared SDM/IO_MUX clock.
    pub clock_source: ClockSource,
}

impl SdmConfig {
    /// Selects the source clock used by all SDM channels.
    pub const fn with_clock_source(mut self, clock_source: ClockSource) -> Self {
        self.clock_source = clock_source;
        self
    }
}

/// Source clock for the shared SDM/IO_MUX clock.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum ClockSource {
    /// APB clock.
    #[cfg(any(esp32, esp32c3, esp32s2, esp32s3))]
    #[default]
    Apb,
    /// XTAL clock.
    #[cfg(any(esp32c5, esp32c6, esp32h2, esp32p4))]
    Xtal,
    /// Fixed 80 MHz PLL clock.
    #[cfg(any(esp32c5, esp32c6, esp32p4))]
    #[cfg_attr(any(esp32c5, esp32c6, esp32p4), default)]
    PllF80m,
    /// Fixed 48 MHz PLL clock.
    #[cfg(esp32h2)]
    #[cfg_attr(esp32h2, default)]
    PllF48m,
}

#[cfg(any(esp32c5, esp32c6, esp32p4))]
// C5/C6 hardware also routes RC_FAST (called FOSC in some register descriptions) through the
// IO MUX. ESP-IDF does not expose it as an SDM clock source, so neither do we. It should be
// usable in principle, but has not been included in the supported SDM API.
impl From<ClockSource> for IomuxFunctionClockConfig {
    fn from(source: ClockSource) -> Self {
        match source {
            ClockSource::Xtal => Self::XtalClk,
            ClockSource::PllF80m => Self::PllF80m,
        }
    }
}

#[cfg(esp32h2)]
impl From<ClockSource> for IomuxFunctionClockConfig {
    fn from(source: ClockSource) -> Self {
        match source {
            ClockSource::Xtal => Self::XtalClk,
            ClockSource::PllF48m => Self::PllF48m,
        }
    }
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
#[derive(Default, Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct ChannelConfig {
    raw_prescaler: u8,
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

    /// Sets the pulse density.
    ///
    /// The value ranges is `-128..=127`.
    pub const fn with_pulse_density(mut self, density: i8) -> Self {
        self.pulse_density = density;
        self
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
        Self {
            config: ChannelConfig {
                raw_prescaler: 0,
                pulse_density: 0,
            },
        }
    }

    /// Sets the requested output frequency.
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
pub struct ChannelCreator {
    channel: usize,
}

impl ChannelCreator {
    const fn new(channel: usize) -> Self {
        Self { channel }
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
    _creator: PhantomData<&'a mut ChannelCreator>,
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
    _peripheral: GenericPeripheralGuard<{ Peripheral::GpioSd as u8 }>,
    _iomux: IomuxClockGuard,
}

impl SdmClockGuard {
    fn new() -> Self {
        let iomux = IomuxClockGuard::new();
        Self {
            _peripheral: GenericPeripheralGuard::new(),
            _iomux: iomux,
        }
    }
}

#[derive(Debug)]
struct IomuxClockGuard;

impl IomuxClockGuard {
    fn new() -> Self {
        #[cfg(soc_has_clock_node_iomux_function_clock)]
        ClockTree::with(clocks::request_iomux_function_clock);
        Self
    }
}

impl Drop for IomuxClockGuard {
    fn drop(&mut self) {
        #[cfg(soc_has_clock_node_iomux_function_clock)]
        ClockTree::with(clocks::release_iomux_function_clock);
    }
}

fn prescaler_from_frequency(frequency: Rate) -> Result<u16, Error> {
    #[cfg(soc_has_clock_node_iomux_function_clock)]
    let source_frequency = clocks::iomux_function_clock_frequency();
    #[cfg(not(soc_has_clock_node_iomux_function_clock))]
    let source_frequency = crate::soc::clocks::apb_clk_frequency();
    let requested_frequency = frequency.as_hz();

    if requested_frequency == 0 || requested_frequency > source_frequency {
        return Err(Error::UnreachableTargetFrequency);
    }

    let prescaler =
        (source_frequency as u64 + requested_frequency as u64 / 2) / requested_frequency as u64;
    if prescaler == 0 || prescaler > 256 {
        return Err(Error::UnreachableTargetFrequency);
    }

    Ok(prescaler as u16)
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
