//! Sigma-delta modulation peripheral.
//!
//! The sigma-delta modulator produces a pulse-density modulated output on a
//! GPIO matrix signal. Each channel can be configured with a carrier frequency
//! and pulse density, then routed to one output pin.

use core::fmt;

use esp_sync::NonReentrantMutex;

use crate::{
    gpio::{
        OutputConfig, OutputSignal, PinGuard,
        interconnect::{OutputSignal as GpioOutputSignal, PeripheralOutput},
    },
    peripherals::GPIO_SD,
    system::{GenericPeripheralGuard, Peripheral},
    time::Rate,
};

static CLOCK_STATE: NonReentrantMutex<SdmClockState> = NonReentrantMutex::new(SdmClockState::new());

struct SdmClockState {
    users: usize,
    source: Option<ClockSource>,
}

impl SdmClockState {
    const fn new() -> Self {
        Self {
            users: 0,
            source: None,
        }
    }
}

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
                clock_source: ClockSource,
                $(
                    #[doc = concat!("Channel ", stringify!($ch), " creator.")]
                    pub [<channel $ch>]: ChannelCreator<$ch>,
                )*
            }

            impl<'d> Sdm<'d> {
                /// Creates a new sigma-delta peripheral driver.
                pub fn new(instance: GPIO_SD<'d>) -> Self {
                    Self::new_with_config(instance, SdmConfig::default())
                }

                /// Creates a new sigma-delta peripheral driver with the given
                /// peripheral-wide configuration.
                ///
                /// The SDM clock source is shared by all SDM channels through the IO_MUX
                /// clock, so it is selected here instead of being configurable per channel.
                pub fn new_with_config(instance: GPIO_SD<'d>, config: SdmConfig) -> Self {
                    Self {
                        _instance: instance,
                        clock_source: config.clock_source,
                        $(
                            [<channel $ch>]: ChannelCreator::new(config.clock_source),
                        )*
                    }
                }

                /// Creates a channel configuration builder using this peripheral's
                /// clock source.
                pub const fn channel_config(&self) -> ChannelConfigBuilder {
                    ChannelConfigBuilder::new(self.clock_source)
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
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
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

impl Default for SdmConfig {
    fn default() -> Self {
        Self {
            clock_source: ClockSource::default(),
        }
    }
}

/// Source clock for the shared SDM/IO_MUX clock.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum ClockSource {
    /// APB clock.
    #[cfg(any(esp32, esp32c3, esp32s2, esp32s3))]
    Apb,
    /// XTAL clock.
    #[cfg(any(esp32c5, esp32c6, esp32h2, esp32p4))]
    Xtal,
    /// Fixed 80 MHz PLL clock.
    #[cfg(any(esp32c5, esp32c6, esp32p4))]
    PllF80m,
    /// Fixed 48 MHz PLL clock.
    #[cfg(esp32h2)]
    PllF48m,
}

impl Default for ClockSource {
    fn default() -> Self {
        cfg_if::cfg_if! {
            if #[cfg(any(esp32, esp32c3, esp32s2, esp32s3))] {
                Self::Apb
            } else if #[cfg(any(esp32c5, esp32c6, esp32p4))] {
                Self::PllF80m
            } else if #[cfg(esp32h2)] {
                Self::PllF48m
            }
        }
    }
}

impl ClockSource {
    fn frequency(self) -> Rate {
        match self {
            #[cfg(any(esp32, esp32c3, esp32s2, esp32s3))]
            Self::Apb => Rate::from_hz(crate::soc::clocks::apb_clk_frequency()),
            #[cfg(any(esp32c5, esp32c6, esp32h2, esp32p4))]
            Self::Xtal => crate::clock::xtal_clock(),
            #[cfg(any(esp32c5, esp32c6, esp32p4))]
            Self::PllF80m => Rate::from_mhz(80),
            #[cfg(esp32h2)]
            Self::PllF48m => Rate::from_mhz(48),
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
    /// Another SDM channel is already using a different clock source.
    ClockSourceConflict,
    /// The prescaler is outside the supported range.
    PrescalerOutOfRange,
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::UnreachableTargetFrequency => f.write_str("unreachable target frequency"),
            Self::ClockSourceConflict => f.write_str("SDM clock source conflict"),
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
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
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
///
/// This builder carries the peripheral-wide clock source so frequencies can be
/// converted to raw prescaler values without storing clock selection in the
/// channel configuration itself.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct ChannelConfigBuilder {
    clock_source: ClockSource,
    config: ChannelConfig,
}

impl ChannelConfigBuilder {
    const fn new(clock_source: ClockSource) -> Self {
        Self {
            clock_source,
            config: ChannelConfig {
                raw_prescaler: 0,
                pulse_density: 0,
            },
        }
    }

    /// Sets the requested output frequency.
    pub fn with_frequency(mut self, frequency: Rate) -> Result<ChannelConfig, Error> {
        self.config.raw_prescaler =
            raw_prescaler(prescaler_from_frequency(frequency, self.clock_source)?);
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

impl Default for ChannelConfig {
    fn default() -> Self {
        Self {
            raw_prescaler: 0,
            pulse_density: 0,
        }
    }
}

/// Creates a connected sigma-delta channel.
///
/// Channel creators are exposed by [`Sdm`]. Calling [`connect`](Self::connect)
/// consumes the creator and returns an active [`Channel`], which prevents
/// accidentally connecting the same channel twice.
#[derive(Debug)]
pub struct ChannelCreator<const CHANNEL: usize> {
    // This is copied from the Sdm-level configuration so partially moved
    // channel creators still use the shared clock source selected at construction time.
    // It is intentionally not configurable per channel.
    clock_source: ClockSource,
}

impl<const CHANNEL: usize> ChannelCreator<CHANNEL> {
    const fn new(clock_source: ClockSource) -> Self {
        Self { clock_source }
    }

    /// Configures this channel and connects it to an output pin.
    pub fn connect<'d>(
        self,
        pin: impl PeripheralOutput<'d>,
        config: ChannelConfig,
    ) -> Result<Channel<CHANNEL>, Error> {
        let clock_guard = SdmClockGuard::new(self.clock_source)?;
        write_config_raw(CHANNEL, config);

        Ok(Channel {
            clock_source: self.clock_source,
            config,
            _pin_guard: connect_pin(CHANNEL, pin),
            _clock_guard: clock_guard,
        })
    }
}

/// A connected sigma-delta channel.
///
/// Dropping a channel disconnects its output pin and releases the SDM clock
/// guard. Use [`disconnect`](Self::disconnect) to explicitly release the
/// channel and recover its [`ChannelCreator`].
#[derive(Debug)]
pub struct Channel<const CHANNEL: usize> {
    clock_source: ClockSource,
    config: ChannelConfig,
    _pin_guard: PinGuard,
    _clock_guard: SdmClockGuard,
}

impl<const CHANNEL: usize> Channel<CHANNEL> {
    /// Applies a new channel configuration.
    pub fn apply_config(&mut self, config: &ChannelConfig) {
        write_config_raw(CHANNEL, *config);
        self.config = *config;
    }

    /// Sets raw pulse density.
    ///
    /// The value ranges from `-128` to `127`.
    pub fn set_pulse_density(&mut self, density: i8) {
        let config = self.config.with_pulse_density(density);
        self.apply_config(&config);
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
        self.config.raw_prescaler as u16 + 1
    }

    /// Reads the raw pulse density.
    ///
    /// The returned value is in the hardware range `-128..=127`.
    pub fn pulse_density(&self) -> i8 {
        self.config.pulse_density
    }

    /// Disconnects this channel and returns its channel creator.
    pub fn disconnect(self) -> ChannelCreator<CHANNEL> {
        let clock_source = self.clock_source;
        drop(self);
        ChannelCreator::new(clock_source)
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
    clock_source: ClockSource,
}

impl SdmClockGuard {
    fn new(clock_source: ClockSource) -> Result<Self, Error> {
        CLOCK_STATE.with(|state| {
            if state.users == usize::MAX {
                panic!("SDM clock reference counter overflowed");
            }

            if state.users == 0 {
                configure_clock_source(clock_source);
                state.source = Some(clock_source);
            } else if state.source != Some(clock_source) {
                return Err(Error::ClockSourceConflict);
            }

            state.users += 1;
            Ok(())
        })?;

        Ok(Self {
            _peripheral: GenericPeripheralGuard::new(),
            clock_source,
        })
    }
}

impl Drop for SdmClockGuard {
    fn drop(&mut self) {
        CLOCK_STATE.with(|state| match state.users {
            0 => panic!("SDM clock reference counter underflowed"),
            1 => {
                debug_assert_eq!(state.source, Some(self.clock_source));
                state.users = 0;
                state.source = None;
            }
            _ => {
                debug_assert_eq!(state.source, Some(self.clock_source));
                state.users -= 1;
            }
        });
    }
}

fn prescaler_from_frequency(frequency: Rate, clock_source: ClockSource) -> Result<u16, Error> {
    let source_frequency = clock_source.frequency().as_hz();
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

fn configure_clock_source(clock_source: ClockSource) {
    // Newer chips expose this selector in different clock-control blocks:
    // - C5/C6/H2 use PCR.iomux_clk_conf.iomux_func_clk_sel, but the selector
    //   encodings differ by chip.
    // - P4 uses HP_SYS_CLKRST.peri_clk_ctrl26.iomux_clk_src_sel, a one-bit
    //   XTAL/PLL_F80M selector.
    // - ESP32/C3/S2/S3 use APB for this driver path and do not need a source
    //   selector write here.
    cfg_if::cfg_if! {
        if #[cfg(esp32c5)] {
            crate::peripherals::PCR::regs().iomux_clk_conf().modify(|_, w| unsafe {
                w.iomux_func_clk_sel()
                    .bits(match clock_source {
                        ClockSource::Xtal => 0,
                        ClockSource::PllF80m => 2,
                    })
                    .iomux_func_clk_en()
                    .set_bit()
            });
        } else if #[cfg(esp32c6)] {
            crate::peripherals::PCR::regs().iomux_clk_conf().modify(|_, w| unsafe {
                w.iomux_func_clk_sel()
                    .bits(match clock_source {
                        ClockSource::PllF80m => 1,
                        ClockSource::Xtal => 3,
                    })
                    .iomux_func_clk_en()
                    .set_bit()
            });
        } else if #[cfg(esp32h2)] {
            crate::peripherals::PCR::regs().iomux_clk_conf().modify(|_, w| unsafe {
                w.iomux_func_clk_sel()
                    .bits(match clock_source {
                        ClockSource::Xtal => 0,
                        ClockSource::PllF48m => 2,
                    })
                    .iomux_func_clk_en()
                    .set_bit()
            });
        } else if #[cfg(esp32p4)] {
            crate::peripherals::HP_SYS_CLKRST::regs()
                .peri_clk_ctrl26()
                .modify(|_, w| {
                    match clock_source {
                        ClockSource::Xtal => w.iomux_clk_src_sel().clear_bit(),
                        ClockSource::PllF80m => w.iomux_clk_src_sel().set_bit(),
                    }
                    .iomux_clk_en()
                    .set_bit()
                });
        } else {
            let _ = clock_source;
        }
    }
}

fn write_config_raw(channel: usize, config: ChannelConfig) {
    // ESP32-C5's PAC names these fields `sd_in`/`sd_prescale`; the other
    // supported PACs name them `in`/`prescale`.
    let prescaler = config.raw_prescaler as _;
    let density = config.pulse_density as _;
    let sd = GPIO_SD::regs();

    #[cfg(esp32c5)]
    sd.sigmadelta(channel).write(|w| unsafe {
        w.sd_in()
            .bits(density)
            .sd_prescale()
            .bits(prescaler)
    });

    #[cfg(not(esp32c5))]
    sd.sigmadelta(channel).write(|w| unsafe {
        w.in_()
            .bits(density)
            .prescale()
            .bits(prescaler)
    });
}
