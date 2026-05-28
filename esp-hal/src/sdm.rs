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
                        $(
                            [<channel $ch>]: ChannelCreator::new(config.clock_source),
                        )*
                    }
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
    /// The raw prescaler is outside the hardware range.
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
    prescaler: u16,
    pulse_density: i8,
}

impl<const CHANNEL: usize> ChannelCreator<CHANNEL> {
    const fn new(clock_source: ClockSource) -> Self {
        Self {
            clock_source,
            prescaler: 1,
            pulse_density: 0,
        }
    }

    const fn with_raw_config(clock_source: ClockSource, prescaler: u16, pulse_density: i8) -> Self {
        Self {
            clock_source,
            prescaler,
            pulse_density,
        }
    }

    /// Sets the requested output frequency.
    ///
    /// The frequency is converted immediately using the SDM clock source
    /// selected in [`SdmConfig`].
    pub fn with_frequency(mut self, frequency: Rate) -> Result<Self, Error> {
        self.prescaler = prescaler_from_frequency(frequency, self.clock_source)?;
        Ok(self)
    }

    /// Sets the raw hardware prescaler.
    ///
    /// The hardware divider range is `1..=256`.
    pub fn with_prescaler(mut self, prescaler: u16) -> Result<Self, Error> {
        check_prescaler(prescaler)?;
        self.prescaler = prescaler;
        Ok(self)
    }

    /// Sets the pulse density.
    ///
    /// The value ranges is `-128..=127`
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

    /// Configures this channel and connects it to an output pin.
    pub fn connect<'d>(self, pin: impl PeripheralOutput<'d>) -> Result<Channel<CHANNEL>, Error> {
        let clock_guard = SdmClockGuard::new(self.clock_source)?;
        set_prescaler_raw(CHANNEL, self.prescaler);
        set_pulse_density_raw(CHANNEL, self.pulse_density);

        Ok(Channel {
            clock_source: self.clock_source,
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
    _pin_guard: PinGuard,
    _clock_guard: SdmClockGuard,
}

impl<const CHANNEL: usize> Channel<CHANNEL> {
    /// Sets raw pulse density.
    ///
    /// The value ranges from `-128` to `127`.
    pub fn set_pulse_density(&mut self, density: i8) {
        set_pulse_density_raw(CHANNEL, density);
    }

    /// Sets duty cycle. `0` maps to the minimum density and `255` maps to the
    /// maximum density.
    pub fn set_duty(&mut self, duty: u8) {
        self.set_pulse_density(duty_to_density(duty))
    }

    /// Sets raw prescaler.
    ///
    /// The hardware divider range is `1..=256`.
    pub fn set_prescaler(&mut self, prescaler: u16) -> Result<(), Error> {
        check_prescaler(prescaler)?;
        set_prescaler_raw(CHANNEL, prescaler);
        Ok(())
    }

    /// Sets the output frequency.
    pub fn set_frequency(&mut self, frequency: Rate) -> Result<(), Error> {
        let prescaler = prescaler_from_frequency(frequency, self.clock_source)?;
        self.set_prescaler(prescaler)
    }

    /// Reads the raw hardware prescaler.
    ///
    /// The returned value is in the hardware divider range `1..=256`.
    pub fn prescaler(&mut self) -> Result<u16, Error> {
        Ok(prescaler_raw(CHANNEL))
    }

    /// Reads the raw pulse density.
    ///
    /// The returned value is in the hardware range `-128..=127`.
    pub fn pulse_density(&mut self) -> Result<i8, Error> {
        Ok(pulse_density_raw(CHANNEL))
    }

    /// Disconnects this channel and returns its channel creator.
    pub fn disconnect(self) -> ChannelCreator<CHANNEL> {
        let clock_source = self.clock_source;
        let prescaler = prescaler_raw(CHANNEL);
        let pulse_density = pulse_density_raw(CHANNEL);
        drop(self);
        ChannelCreator::with_raw_config(clock_source, prescaler, pulse_density)
    }
}

impl<const CHANNEL: usize> Drop for Channel<CHANNEL> {
    fn drop(&mut self) {
        set_pulse_density_raw(CHANNEL, 0);
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

const fn duty_to_density(duty: u8) -> i8 {
    (duty as i16 - 128) as i8
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

fn set_pulse_density_raw(channel: usize, density: i8) {
    // The register layout is the same conceptually, but ESP32-C5's PAC names
    // the field `sd_in`; the other supported PACs name it `in`.
    let sd = GPIO_SD::regs();

    #[cfg(esp32c5)]
    sd.sigmadelta(channel)
        .modify(|_, w| unsafe { w.sd_in().bits(density as _) });

    #[cfg(not(esp32c5))]
    sd.sigmadelta(channel)
        .modify(|_, w| unsafe { w.in_().bits(density as _) });
}

fn set_prescaler_raw(channel: usize, prescaler: u16) {
    // Hardware stores prescaler - 1. ESP32-C5's PAC names the field
    // `sd_prescale`; the other supported PACs name it `prescale`.
    let bits = (prescaler - 1) as _;
    let sd = GPIO_SD::regs();

    #[cfg(esp32c5)]
    sd.sigmadelta(channel)
        .modify(|_, w| unsafe { w.sd_prescale().bits(bits) });

    #[cfg(not(esp32c5))]
    sd.sigmadelta(channel)
        .modify(|_, w| unsafe { w.prescale().bits(bits) });
}

fn pulse_density_raw(channel: usize) -> i8 {
    let sd = GPIO_SD::regs();

    #[cfg(esp32c5)]
    return sd.sigmadelta(channel).read().sd_in().bits() as i8;

    #[cfg(not(esp32c5))]
    return sd.sigmadelta(channel).read().in_().bits() as i8;
}

fn prescaler_raw(channel: usize) -> u16 {
    let sd = GPIO_SD::regs();

    #[cfg(esp32c5)]
    let bits = sd.sigmadelta(channel).read().sd_prescale().bits();

    #[cfg(not(esp32c5))]
    let bits = sd.sigmadelta(channel).read().prescale().bits();

    bits as u16 + 1
}
