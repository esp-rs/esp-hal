#![cfg_attr(docsrs, procmacros::doc_replace(
    "rx_channel" => {
        cfg(any(esp32, esp32s2)) => "RMT_CH1",
        cfg(any(esp32s3, esp32s31, esp32p4)) => "RMT_CH4",
        _ => "RMT_CH2"
    },
    "channels_desc" => {
        cfg(esp32) => "8 channels, each of them can be either receiver or transmitter",
        cfg(esp32s2) => "4 channels, each of them can be either receiver or transmitter",
        cfg(any(esp32s3, esp32s31, esp32p4)) => "8 channels, `RMT_CH0`-`RMT_CH3` for transmitting signals and `RMT_CH4`-`RMT_CH7` for receiving signals",
        cfg(any(esp32c3, esp32c5, esp32c6, esp32h2)) => "4 channels, `RMT_CH0` and `RMT_CH1` for transmitting signals and `RMT_CH2` and `RMT_CH3` for receiving signals",
    }
))]
//! # Remote Control Peripheral (RMT)
//!
//! ## Overview
//! The RMT (Remote Control) module is designed to send and receive infrared
//! remote control signals. A variety of remote control protocols can be
//! encoded/decoded via software based on the RMT module. The RMT module
//! converts pulse codes stored in the module’s built-in RAM into output
//! signals, or converts input signals into pulse codes and stores them in RAM.
//! In addition, the RMT module optionally modulates its output signals with a
//! carrier wave, or optionally demodulates and filters its input signals.
//!
//! Typically, the RMT peripheral can be used in the following scenarios:
//! - Transmit or receive infrared signals, with any IR protocols, e.g., NEC
//! - General-purpose sequence generator
//! - Transmit signals in a hardware-controlled loop, with a finite or infinite number of times
//! - Modulate the carrier to the output signal or demodulate the carrier from the input signal
//!
//! ### Channels
//!
//! There are __channels_desc__. Each channel is a peripheral singleton (`RMT_CH0`, …).
//!
//! A channel can use the RAM of the channels that follow it: pass a tuple of adjacent channel
//! singletons, such as `(RMT_CH0, RMT_CH1)`. See [`TxInstance`] and [`RxInstance`] for the
//! combinations of the chip.
//!
//! For more information, please refer to the
#![doc = concat!("[ESP-IDF documentation](https://docs.espressif.com/projects/esp-idf/en/latest/", chip!(), "/api-reference/peripherals/rmt.html)")]
//! ## Configuration
//!
//! [`Rmt`] holds the settings that all channels share: the clock source and the group clock
//! ([`RmtConfig`]). The channels come from [`Rmt::tx_channel`] and [`Rmt::rx_channel`], and
//! each channel selects its own tick rate ([`TxConfig`], [`RxConfig`]).
//!
//! ## Examples
//!
//! ### TX operation
//! ```rust, no_run
//! # {before_snippet}
//! # use esp_hal::delay::Delay;
//! # use esp_hal::gpio::Level;
//! # use esp_hal::rmt::{PulseCode, Rmt, RmtConfig, TxConfig};
//! let rmt = Rmt::new(peripherals.RMT, RmtConfig::default())?;
//!
//! let tx_config = TxConfig::default().with_tick_rate(Rate::from_mhz(1));
//! let mut channel = rmt
//!     .tx_channel(peripherals.RMT_CH0, tx_config)?
//!     .with_pin(peripherals.GPIO4);
//!
//! let delay = Delay::new();
//!
//! let mut data = [PulseCode::new(Level::High, 200, Level::Low, 50); 20];
//! data[data.len() - 1] = PulseCode::new(Level::High, 3000, Level::Low, 500);
//!
//! loop {
//!     channel.transmit(&data)?;
//!     delay.delay_millis(500);
//! }
//! # }
//! ```
//!
//! ### RX operation
//! ```rust, no_run
//! # {before_snippet}
//! # use esp_hal::rmt::{PulseCode, Rmt, RmtConfig, RxConfig};
//! let rmt = Rmt::new(peripherals.RMT, RmtConfig::default())?;
//!
//! let rx_config = RxConfig::default()
//!     .with_tick_rate(Rate::from_mhz(1))
//!     .with_idle_ns(10_000_000);
//! let mut channel = rmt
//!     .rx_channel(peripherals.__rx_channel__, rx_config)?
//!     .with_pin(peripherals.GPIO4);
//!
//! let mut data = [PulseCode::default(); 48];
//! let count = channel.receive(&mut data)?;
//! for code in &data[..count] {
//!     println!("{:?}", code);
//! }
//! # {after_snippet}
//! ```

use core::{
    marker::PhantomData,
    pin::Pin,
    task::{Context, Poll},
};

use enumset::{EnumSet, EnumSetType};
use esp_sync::RawMutex;
#[cfg(place_rmt_driver_in_ram)]
use procmacros::ram;

use crate::{
    Async,
    Blocking,
    DriverMode,
    asynch::AtomicWaker,
    clock::CpuFrequencyLock,
    gpio::{
        self,
        InputConfig,
        Level,
        OutputConfig,
        PinGuard,
        interconnect::{PeripheralInput, PeripheralOutput},
    },
    peripherals::{Interrupt, RMT},
    soc::clocks,
    system::GenericPeripheralGuard,
    time::Rate,
};

mod reader;
use reader::{ReaderState, RmtReader};
mod writer;
use writer::{RmtWriter, WriterState};

/// Errors of [`Rmt`] and of the channel constructors.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum ConfigError {
    /// The tick rate is above the group clock, or below the group clock divided by 256.
    UnreachableTickRate,

    /// The carrier frequency or duty cycle cannot be reached.
    UnreachableCarrier,

    /// A receive limit is out of range at the actual clock rates.
    RxLimitOutOfRange,
}

impl core::error::Error for ConfigError {}

impl core::fmt::Display for ConfigError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::UnreachableTickRate => write!(f, "The tick rate cannot be reached"),
            Self::UnreachableCarrier => write!(f, "The carrier cannot be reached"),
            Self::RxLimitOutOfRange => write!(f, "A receive limit is out of range"),
        }
    }
}

/// Errors of a transmission.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum TxError {
    /// The transmission ended before the driver wrote all data to channel RAM.
    Underrun,

    /// The data and the end marker do not fit in the channel RAM (loop mode).
    DataTooLong,

    /// The loop count is above [`MAX_TX_LOOPCOUNT`].
    #[cfg(rmt_has_tx_loop_count)]
    LoopCountOutOfRange,
}

impl core::error::Error for TxError {}

impl core::fmt::Display for TxError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Underrun => write!(f, "The transmission ended before all data was written"),
            Self::DataTooLong => write!(f, "The data does not fit in the channel RAM"),
            #[cfg(rmt_has_tx_loop_count)]
            Self::LoopCountOutOfRange => write!(f, "The loop count is out of range"),
        }
    }
}

/// Errors of a reception.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum RxError {
    /// The frame has more codes than the buffer holds.
    BufferTooSmall,

    /// The driver did not read the channel RAM in time, or the frame does not fit in the
    /// channel RAM on a chip without RX wrap.
    Overrun,
}

impl core::error::Error for RxError {}

impl core::fmt::Display for RxError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::BufferTooSmall => write!(f, "The frame does not fit in the buffer"),
            Self::Overrun => write!(f, "The channel RAM overflowed"),
        }
    }
}

/// Error of [`PulseCode::try_new`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum PulseCodeError {
    /// `length1` does not convert to `u16`, or is above [`PulseCode::MAX_LEN`].
    Length1OutOfRange,

    /// `length2` does not convert to `u16`, or is above [`PulseCode::MAX_LEN`]. Reported only if
    /// `length1` is in range.
    Length2OutOfRange,
}

impl core::error::Error for PulseCodeError {}

impl core::fmt::Display for PulseCodeError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Length1OutOfRange => write!(f, "length1 is out of range"),
            Self::Length2OutOfRange => write!(f, "length2 is out of range"),
        }
    }
}

/// Convenience newtype to work with pulse codes.
///
/// A [`PulseCode`] is represented as `u32`, with fields laid out as follows:
///
/// | Bit 31   | Bits 30-16 | Bit 15   | Bits 14-0 |
/// |----------|------------|----------|-----------|
/// | `level2` | `length2`  | `level1` | `length1` |
///
/// Here, `level1` / `length1` correspond to the signal that is send/received first,
/// and the signal with `level2` / `length2` is send/received afterwards.
///
/// If `length1` or `length2` are zero, this implies an end marker and transmission will
/// stop with the corresponding signal.
#[derive(Clone, Copy, Default, Eq, PartialEq)]
#[repr(transparent)]
pub struct PulseCode(u32);

// Pre-compute some constants to make it obvious that the code below doesn't mix up both halves of
// a PulseCode.
const LENGTH1_SHIFT: usize = 0;
const LEVEL1_SHIFT: usize = 15;
const LENGTH2_SHIFT: usize = 16;
const LEVEL2_SHIFT: usize = 31;

const LENGTH_MASK: u32 = 0x7FFF;
const LENGTH1_MASK: u32 = LENGTH_MASK << LENGTH1_SHIFT;
const LENGTH2_MASK: u32 = LENGTH_MASK << LENGTH2_SHIFT;

const LEVEL1_MASK: u32 = 1 << LEVEL1_SHIFT;
const LEVEL2_MASK: u32 = 1 << LEVEL2_SHIFT;

impl PulseCode {
    /// Maximum value for the `length1` and `length2` fields.
    pub const MAX_LEN: u16 = 0x7FFF;

    /// Creates a new instance.
    ///
    /// Panics if `length1` or `length2` exceed the maximum representable range.
    #[inline]
    pub const fn new(level1: Level, length1: u16, level2: Level, length2: u16) -> Self {
        if length1 > Self::MAX_LEN || length2 > Self::MAX_LEN {
            // defmt::panic! fails const eval
            core::panic!("PulseCode length out of range");
        };

        // SAFETY:
        // - We just checked that length1 and length2 are in range
        unsafe { Self::new_unchecked(level1, length1, level2, length2) }
    }

    /// Creates a new instance.
    ///
    /// If `length1` or `length2` exceed the maximum representable range, they
    /// will be clamped to `Self::MAX_LEN`.
    #[inline]
    pub const fn new_clamped(level1: Level, length1: u16, level2: Level, length2: u16) -> Self {
        // Can't use lengthX.min(Self::MAX_LEN) since it is not const
        let length1 = if length1 > Self::MAX_LEN {
            Self::MAX_LEN
        } else {
            length1
        };
        let length2 = if length2 > Self::MAX_LEN {
            Self::MAX_LEN
        } else {
            length2
        };

        // SAFETY:
        // - We just clamped length1 and length2 to the required intervals
        unsafe { Self::new_unchecked(level1, length1, level2, length2) }
    }

    /// Creates a new instance, attempting to convert lengths to `u16` first.
    ///
    /// This is slightly more convenient when passing in longer integers (e.g. `u32`) resulting from
    /// a preceding calculation.
    ///
    /// # Errors
    ///
    /// [`PulseCodeError::Length1OutOfRange`] or [`PulseCodeError::Length2OutOfRange`] if a length
    /// does not convert to `u16` or is above [`PulseCode::MAX_LEN`].
    #[inline]
    pub fn try_new(
        level1: Level,
        length1: impl TryInto<u16>,
        level2: Level,
        length2: impl TryInto<u16>,
    ) -> Result<Self, PulseCodeError> {
        let length1 = length1
            .try_into()
            .ok()
            .filter(|&l| l <= Self::MAX_LEN)
            .ok_or(PulseCodeError::Length1OutOfRange)?;
        let length2 = length2
            .try_into()
            .ok()
            .filter(|&l| l <= Self::MAX_LEN)
            .ok_or(PulseCodeError::Length2OutOfRange)?;

        // SAFETY:
        // - We just checked that length1 and length2 are in range
        Ok(unsafe { Self::new_unchecked(level1, length1, level2, length2) })
    }

    /// Creates a new instance without checking that code lengths are in range.
    ///
    /// # Safety
    ///
    /// `length1` and `length2` must be 15-bit wide, i.e. their MSB must be cleared.
    #[inline]
    pub const unsafe fn new_unchecked(
        level1: Level,
        length1: u16,
        level2: Level,
        length2: u16,
    ) -> Self {
        Self(
            (level1.const_into() as u32) << LEVEL1_SHIFT
                | (level2.const_into() as u32) << LEVEL2_SHIFT
                | (length1 as u32) << LENGTH1_SHIFT
                | (length2 as u32) << LENGTH2_SHIFT,
        )
    }

    /// Creates a new instance that is an end marker with `Level::Low`.
    ///
    /// This corresponds to the all-zero [`PulseCode`], i.e. with both level and
    /// length fields set to zero, equivalent to (but more semantic than)
    /// `PulseCode::from(0u32)` and [`PulseCode::default()`].
    // FIXME: Consider adding a variant with `level1`, `length1` and `level2` arguments
    // which sets `length2 = 0` so that it is still guaranteed to return an end
    // marker.
    #[inline]
    pub const fn end_marker() -> Self {
        Self(0)
    }

    /// Sets all levels and lengths to 0.
    ///
    /// In other words, assigns the value of [`PulseCode::end_marker()`] to `self`.
    #[inline]
    pub fn reset(&mut self) {
        self.0 = 0
    }

    /// Logical output level in the first pulse code interval
    #[inline]
    pub const fn level1(self) -> Level {
        let level = (self.0 >> LEVEL1_SHIFT) & 1;
        Level::const_from(0 != level)
    }

    /// Logical output level in the second pulse code interval
    #[inline]
    pub const fn level2(self) -> Level {
        let level = (self.0 >> LEVEL2_SHIFT) & 1;
        Level::const_from(0 != level)
    }

    /// Length of the first pulse code interval (in clock cycles)
    #[inline]
    pub const fn length1(self) -> u16 {
        ((self.0 >> LENGTH1_SHIFT) & LENGTH_MASK) as u16
    }

    /// Length of the second pulse code interval (in clock cycles)
    #[inline]
    pub const fn length2(self) -> u16 {
        ((self.0 >> LENGTH2_SHIFT) & LENGTH_MASK) as u16
    }

    /// Sets `level1` and returns the modified [`PulseCode`].
    #[inline]
    pub const fn with_level1(mut self, level: Level) -> Self {
        self.0 &= !LEVEL1_MASK;
        self.0 |= (level.const_into() as u32) << LEVEL1_SHIFT;
        self
    }

    /// Sets `level2` and returns the modified [`PulseCode`].
    #[inline]
    pub const fn with_level2(mut self, level: Level) -> Self {
        self.0 &= !LEVEL2_MASK;
        self.0 |= (level.const_into() as u32) << LEVEL2_SHIFT;
        self
    }

    /// Sets `length1` and returns the modified [`PulseCode`].
    ///
    /// Returns `None` if `length` exceeds the representable range.
    #[inline]
    pub const fn with_length1(mut self, length: u16) -> Option<Self> {
        if length > Self::MAX_LEN {
            return None;
        }

        self.0 &= !LENGTH1_MASK;
        self.0 |= (length as u32) << LENGTH1_SHIFT;
        Some(self)
    }

    /// Sets `length2` and returns the modified [`PulseCode`].
    ///
    /// Returns `None` if `length` exceeds the representable range.
    #[inline]
    pub const fn with_length2(mut self, length: u16) -> Option<Self> {
        if length > Self::MAX_LEN {
            return None;
        }

        self.0 &= !LENGTH2_MASK;
        self.0 |= (length as u32) << LENGTH2_SHIFT;
        Some(self)
    }

    /// Returns whether this pulse code contains an end marker.
    ///
    /// Equivalent to `self.length1() == 0 || self.length2() == 0`.
    #[inline]
    pub const fn is_end_marker(self) -> bool {
        self.length1() == 0 || self.length2() == 0
    }

    #[inline]
    fn symbol1(self) -> char {
        if self.level1().into() { 'H' } else { 'L' }
    }

    #[inline]
    fn symbol2(self) -> char {
        if self.level2().into() { 'H' } else { 'L' }
    }
}

impl core::fmt::Debug for PulseCode {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(
            f,
            "PulseCode({} {}, {} {})",
            self.symbol1(),
            self.length1(),
            self.symbol2(),
            self.length2(),
        )
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for PulseCode {
    fn format(&self, fmt: defmt::Formatter<'_>) {
        defmt::write!(
            fmt,
            "PulseCode({} {}, {} {})",
            self.symbol1(),
            self.length1(),
            self.symbol2(),
            self.length2(),
        )
    }
}

impl From<u32> for PulseCode {
    #[inline]
    fn from(value: u32) -> Self {
        Self(value)
    }
}

impl From<PulseCode> for u32 {
    #[inline]
    fn from(code: PulseCode) -> u32 {
        code.0
    }
}

/// Memory size associated to a channel.
///
/// This is a newtype around the number of blocks as u8, thus not requiring any
/// extra space. However, it is useful to abstract memory sizes into their own
/// type to make explicit whether they refer to a number of RAM blocks or a
/// number of pulse codes and to centralize conversion between both.
#[derive(Copy, Clone)]
struct MemSize(u8);

impl MemSize {
    /// Creates from the given number of RMT RAM blocks.
    #[inline]
    const fn from_blocks(blocks: u8) -> Self {
        Self(blocks)
    }

    /// Returns the number of RMT RAM blocks specified by this `MemSize`.
    #[inline]
    const fn blocks(self) -> u8 {
        self.0
    }

    /// Returns the number of RMT pulse codes specified by this `MemSize`.
    #[inline]
    const fn codes(self) -> usize {
        self.0 as usize * property!("rmt.channel_ram_size")
    }
}

/// Marker for a channel configured for transmit operations.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
struct Tx;

/// Marker for a channel configured for receive operations.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
struct Rx;

trait Direction: Copy + Clone + core::fmt::Debug + Unpin {
    #[cfg_attr(
        any(esp32, esp32s2),
        expect(dead_code, reason = "these chips use the same registers for TX and RX")
    )]
    const IS_TX: bool;
}

impl Direction for Tx {
    const IS_TX: bool = true;
}

impl Direction for Rx {
    const IS_TX: bool = false;
}

#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
struct DynChannelAccess<Dir: Direction> {
    ch_idx: ChannelIndex,
    _direction: PhantomData<Dir>,
}

impl<Dir: Direction> DynChannelAccess<Dir> {
    #[inline]
    unsafe fn conjure(ch_idx: ChannelIndex) -> Self {
        Self {
            ch_idx,
            _direction: PhantomData,
        }
    }
}

/// RMT clock source.
pub use crate::soc::clocks::RmtSclkConfig as ClockSource;

/// Configuration of [`Rmt`]: the settings that all channels share.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Hash, procmacros::BuilderLite)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct RmtConfig {
    /// Source of the group clock.
    clock_source: ClockSource,

    /// Frequency of the group clock. `None` means the source frequency.
    ///
    /// A value selects the integer divider (1 to 256) that gives the nearest clock period.
    /// [`Rmt::frequency`] returns the actual frequency.
    #[cfg(not(rmt_has_per_channel_clock))]
    frequency: Option<Rate>,
}

/// Carrier of a transmit channel, or the carrier that a receive channel removes.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, procmacros::BuilderLite)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct CarrierConfig {
    /// Carrier frequency.
    frequency: Rate,

    /// High time in percent of the carrier period.
    duty_percent: u8,

    /// The output level that the carrier modulates.
    level: Level,
}

impl Default for CarrierConfig {
    fn default() -> Self {
        Self {
            frequency: Rate::from_khz(38),
            duty_percent: 50,
            level: Level::High,
        }
    }
}

/// Configuration of a transmit channel.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, procmacros::BuilderLite)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct TxConfig {
    /// Tick rate of the channel. `None` means the group clock.
    ///
    /// The driver uses the nearest tick period that it can reach. The actual period differs
    /// from the requested one by at most `r / (2 * g)` of it, with `r` the requested rate and
    /// `g` the group clock. It is exact when `g / r` is an integer.
    /// [`TxChannel::tick_rate`] returns the actual rate.
    tick_rate: Option<Rate>,

    /// Output level when the channel does not transmit.
    idle_level: Level,

    /// Carrier that modulates the output. `None` means no carrier.
    carrier: Option<CarrierConfig>,
}

impl Default for TxConfig {
    fn default() -> Self {
        Self {
            tick_rate: None,
            idle_level: Level::Low,
            carrier: None,
        }
    }
}

/// Configuration of a receive channel.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, procmacros::BuilderLite)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct RxConfig {
    /// Tick rate of the channel. `None` means the group clock. See [`TxConfig::tick_rate`].
    tick_rate: Option<Rate>,

    /// Pulses shorter than this are noise, in nanoseconds. `None` turns the filter off.
    ///
    /// The filter counts group clock cycles and is at most 255 of them.
    min_pulse_ns: Option<u32>,

    /// A level that lasts longer than this ends the reception, in nanoseconds.
    ///
    /// At most [`MAX_RX_IDLE_THRESHOLD`] ticks.
    idle_ns: u32,

    /// Carrier to remove from the input. `None` means no demodulation.
    #[cfg(rmt_has_rx_demodulation)]
    carrier: Option<CarrierConfig>,
}

impl Default for RxConfig {
    fn default() -> Self {
        Self {
            tick_rate: None,
            min_pulse_ns: None,
            idle_ns: 100_000,
            #[cfg(rmt_has_rx_demodulation)]
            carrier: None,
        }
    }
}

// Returns the channel divider that gives the nearest tick period.
fn channel_divider(group: Rate, tick_rate: Option<Rate>) -> Result<u16, ConfigError> {
    let Some(tick_rate) = tick_rate else {
        return Ok(1);
    };
    let (group, rate) = (group.as_hz() as u64, tick_rate.as_hz() as u64);
    if rate == 0 || rate > group || rate * 256 < group {
        return Err(ConfigError::UnreachableTickRate);
    }
    Ok(((group + rate / 2) / rate).clamp(1, 256) as u16)
}

// Returns the high and low times of a carrier in group clock cycles.
fn carrier_cycles(group: Rate, carrier: &CarrierConfig) -> Result<(u16, u16), ConfigError> {
    let frequency = carrier.frequency.as_hz();
    if frequency == 0 || carrier.duty_percent > 100 {
        return Err(ConfigError::UnreachableCarrier);
    }
    let total = group.as_hz() / frequency;
    let high = total * carrier.duty_percent as u32 / 100;
    let low = total - high;
    match (u16::try_from(high), u16::try_from(low)) {
        (Ok(high @ 1..), Ok(low @ 1..)) => Ok((high, low)),
        _ => Err(ConfigError::UnreachableCarrier),
    }
}

// Converts a duration to cycles of `rate`, rounded to nearest.
fn ns_to_cycles(ns: u32, rate: Rate) -> u64 {
    (ns as u64 * rate.as_hz() as u64 + 500_000_000) / 1_000_000_000
}

// Returns the channel divider.
#[inline(never)]
fn apply_tx_config(
    raw: DynChannelAccess<Tx>,
    group: Rate,
    config: &TxConfig,
) -> Result<u16, ConfigError> {
    let divider = channel_divider(group, config.tick_rate)?;
    let carrier = config
        .carrier
        .map(|carrier| carrier_cycles(group, &carrier).map(|cycles| (carrier.level, cycles)))
        .transpose()?;

    raw.set_divider(divider as u8);
    match carrier {
        Some((level, (high, low))) => raw.set_tx_carrier(true, high, low, level),
        None => raw.set_tx_carrier(false, 0, 0, Level::Low),
    }
    raw.set_tx_idle_output(true, config.idle_level);

    Ok(divider)
}

// Returns the channel divider.
#[inline(never)]
fn apply_rx_config(
    raw: DynChannelAccess<Rx>,
    group: Rate,
    config: &RxConfig,
) -> Result<u16, ConfigError> {
    let divider = channel_divider(group, config.tick_rate)?;

    let filter = match config.min_pulse_ns {
        None => 0,
        Some(ns) => match ns_to_cycles(ns, group) {
            cycles @ 1..=255 => cycles as u8,
            _ => return Err(ConfigError::RxLimitOutOfRange),
        },
    };

    let tick_rate = Rate::from_hz(group.as_hz() / divider as u32);
    let idle = match ns_to_cycles(config.idle_ns, tick_rate) {
        ticks @ 1.. if ticks <= MAX_RX_IDLE_THRESHOLD as u64 => ticks as u16,
        _ => return Err(ConfigError::RxLimitOutOfRange),
    };

    #[cfg(rmt_has_rx_demodulation)]
    let carrier = config
        .carrier
        .map(|carrier| carrier_cycles(group, &carrier).map(|cycles| (carrier.level, cycles)))
        .transpose()?;

    raw.set_divider(divider as u8);
    #[cfg(rmt_has_rx_demodulation)]
    match carrier {
        Some((level, (high, low))) => raw.set_rx_carrier(true, high, low, level),
        None => raw.set_rx_carrier(false, 0, 0, Level::Low),
    }
    raw.set_rx_filter_threshold(filter);
    raw.set_rx_idle_threshold(idle);

    Ok(divider)
}

// Channel specification

for_each_rmt_channel!(
    (all $(($num:literal)),+) => {
        #[allow(clippy::no_effect)]
        const NUM_CHANNELS: usize = const { 0 $( + {$num; 1} )+ };
    };

    // Looping channel indices: Declare input/output signals and ChannelIndex
    // The number of Rx and Tx channels is identical for all chips.
    (tx $(($num:literal, $idx:literal)),+) => {
        paste::paste! {
            // Enum of valid channel indices: For the given chip, tx/rx channels for all of these indices
            // exist. (Note that channel index == channel number for esp32 and esp32s2, but not for other
            // chips.)
            //
            // This type is useful to inform the compiler of possible values of an u8 (i.e. we use this as
            // homemade refinement type) which allows it to elide bounds checks in register/field
            // accessors of the PAC even when using DynChannelAccess.
            //
            // Cf. https://github.com/rust-lang/rust/issues/109958 regarding rustc's capabilities here.
            #[doc(hidden)]
            #[derive(Clone, Copy, Debug, PartialEq, Eq)]
            #[cfg_attr(feature = "defmt", derive(defmt::Format))]
            #[repr(u8)]
            #[allow(unused)]
            pub enum ChannelIndex {
                $(
                    [<Ch $idx>] = $idx,
                )+
            }

            impl ChannelIndex {
                #[allow(clippy::no_effect)]
                const MAX: u8 = const { 0 $( + {$idx; 1} )+ };
            }

            const OUTPUT_SIGNALS: [gpio::OutputSignal; ChannelIndex::MAX as usize] = [
                $(
                    gpio::OutputSignal::[<RMT_SIG_ $idx>],
                )+
            ];
        }
    };

    (rx $(($num:literal, $idx:literal)),+) => {
        paste::paste! {
            const INPUT_SIGNALS: [gpio::InputSignal; ChannelIndex::MAX as usize] = [
                $(
                    gpio::InputSignal::[<RMT_SIG_ $idx>],
                )+
            ];
        }
    };
);

struct ChannelIndexIter(u8);

impl Iterator for ChannelIndexIter {
    type Item = ChannelIndex;

    fn next(&mut self) -> Option<Self::Item> {
        let ch_idx = self.0;
        if ch_idx < ChannelIndex::MAX {
            self.0 = ch_idx + 1;
            Some(unsafe { ChannelIndex::from_u8_unchecked(ch_idx) })
        } else {
            None
        }
    }
}

impl ChannelIndex {
    fn iter_all() -> ChannelIndexIter {
        ChannelIndexIter(0)
    }

    unsafe fn from_u8_unchecked(ch_idx: u8) -> Self {
        debug_assert!(ch_idx < ChannelIndex::MAX);
        unsafe { core::mem::transmute(ch_idx) }
    }
}

/// Immutable data of a channel in one direction.
#[doc(hidden)]
#[derive(Debug)]
pub struct Info {
    ch_idx: ChannelIndex,
}

mod sealed {
    pub trait Tx {}
    pub trait Rx {}
}

/// A transmit channel, alone or with the RAM blocks of the channels that follow it.
///
/// A tuple of adjacent channel singletons, such as `(RMT_CH0, RMT_CH1)`, gives the first channel
/// the RAM blocks of the others. Each block holds [`CHANNEL_RAM_SIZE`] pulse codes. The
/// channels that lend their RAM cannot be used while the tuple is in use, also if they are
/// receive channels.
///
/// The implementations list the singletons and tuples that this chip supports.
pub trait TxInstance: sealed::Tx {
    /// Number of RAM blocks that the channel uses.
    // Hidden, so that the RAM size is not part of the type in the public API.
    #[doc(hidden)]
    const BLOCKS: u8;

    // Public, because the generic channel constructor reads it.
    #[doc(hidden)]
    fn info(&self) -> &'static Info;
}

/// A receive channel, alone or with the RAM blocks of the channels that follow it.
///
/// A tuple of adjacent channel singletons, such as `(RMT_CH2, RMT_CH3)`, gives the first channel
/// the RAM blocks of the others. Each block holds [`CHANNEL_RAM_SIZE`] pulse codes.
///
/// The implementations list the singletons and tuples that this chip supports.
pub trait RxInstance: sealed::Rx {
    /// Number of RAM blocks that the channel uses.
    // Hidden, so that the RAM size is not part of the type in the public API.
    #[doc(hidden)]
    const BLOCKS: u8;

    // Public, because the generic channel constructor reads it.
    #[doc(hidden)]
    fn info(&self) -> &'static Info;
}

for_each_rmt_channel!(
    (tx $(($num:literal, $idx:literal)),+) => {
        paste::paste! {
            $(
                impl sealed::Tx for crate::peripherals::[<RMT_CH $num>]<'_> {}

                #[doc = concat!("`RMT_CH", $num, "` with 1 RAM block.")]
                impl TxInstance for crate::peripherals::[<RMT_CH $num>]<'_> {
                    const BLOCKS: u8 = 1;

                    fn info(&self) -> &'static Info {
                        static INFO: Info = Info {
                            ch_idx: ChannelIndex::[<Ch $idx>],
                        };
                        &INFO
                    }
                }
            )+

            crate::any_peripheral! {
                /// Any RMT channel that can transmit.
                pub peripheral AnyRmtTxChannel<'d> {
                    $(
                        [<RmtCh $num>](crate::peripherals::[<RMT_CH $num>]<'d>),
                    )+
                }
            }
        }
    };

    (rx $(($num:literal, $idx:literal)),+) => {
        paste::paste! {
            $(
                impl sealed::Rx for crate::peripherals::[<RMT_CH $num>]<'_> {}

                #[doc = concat!("`RMT_CH", $num, "` with 1 RAM block.")]
                impl RxInstance for crate::peripherals::[<RMT_CH $num>]<'_> {
                    const BLOCKS: u8 = 1;

                    fn info(&self) -> &'static Info {
                        static INFO: Info = Info {
                            ch_idx: ChannelIndex::[<Ch $idx>],
                        };
                        &INFO
                    }
                }
            )+

            mod rx_any {
                use super::{Info, RxInstance, sealed};

                crate::any_peripheral! {
                    /// Any RMT channel that can receive.
                    pub peripheral AnyRmtRxChannel<'d> {
                        $(
                            [<RmtCh $num>](crate::peripherals::[<RMT_CH $num>]<'d>),
                        )+
                    }
                }

                impl sealed::Rx for AnyRmtRxChannel<'_> {}

                /// A type-erased channel uses one RAM block: its type cannot show that the
                /// channels of a tuple are adjacent.
                impl RxInstance for AnyRmtRxChannel<'_> {
                    const BLOCKS: u8 = 1;

                    fn info(&self) -> &'static Info {
                        any::delegate!(self, channel => { RxInstance::info(channel) })
                    }
                }
            }

            pub use rx_any::AnyRmtRxChannel;
        }
    };

    (tx_tuples $(($blocks:literal, ($first:ident $(, $peri:ident)+))),*) => {
        $(
            impl sealed::Tx for (
                crate::peripherals::$first<'_>,
                $(crate::peripherals::$peri<'_>,)+
            ) {}

            #[doc = concat!("`", stringify!($first), "` with ", $blocks, " RAM blocks.")]
            impl TxInstance for (
                crate::peripherals::$first<'_>,
                $(crate::peripherals::$peri<'_>,)+
            ) {
                const BLOCKS: u8 = $blocks;

                fn info(&self) -> &'static Info {
                    TxInstance::info(&self.0)
                }
            }
        )*
    };

    (rx_tuples $(($blocks:literal, ($first:ident $(, $peri:ident)+))),*) => {
        $(
            impl sealed::Rx for (
                crate::peripherals::$first<'_>,
                $(crate::peripherals::$peri<'_>,)+
            ) {}

            #[doc = concat!("`", stringify!($first), "` with ", $blocks, " RAM blocks.")]
            impl RxInstance for (
                crate::peripherals::$first<'_>,
                $(crate::peripherals::$peri<'_>,)+
            ) {
                const BLOCKS: u8 = $blocks;

                fn info(&self) -> &'static Info {
                    RxInstance::info(&self.0)
                }
            }
        )*
    };
);

impl sealed::Tx for AnyRmtTxChannel<'_> {}

/// A type-erased channel uses one RAM block: its type cannot show that the channels of a tuple
/// are adjacent.
impl TxInstance for AnyRmtTxChannel<'_> {
    const BLOCKS: u8 = 1;

    fn info(&self) -> &'static Info {
        any::delegate!(self, channel => { TxInstance::info(channel) })
    }
}

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
struct RmtClockGuard;

impl RmtClockGuard {
    fn new() -> Self {
        #[cfg(soc_has_clock_node_rmt_sclk)]
        clocks::ClockTree::with(|clocks| {
            if clocks::RmtInstance::Rmt.sclk_config(clocks).is_none() {
                clocks::RmtInstance::Rmt.configure_sclk(clocks, ClockSource::default());
            }

            clocks::RmtInstance::Rmt.request_sclk(clocks);
        });

        // The RMT stops if APB_CLK is less than about 1/10 of the RMT clock.
        #[cfg(soc_has_apb_frequency_reduction)]
        clocks::ClockTree::with(clocks::request_apb_clk);

        Self
    }
}

impl Drop for RmtClockGuard {
    fn drop(&mut self) {
        #[cfg(soc_has_clock_node_rmt_sclk)]
        clocks::ClockTree::with(|clocks| clocks::RmtInstance::Rmt.release_sclk(clocks));
        #[cfg(soc_has_apb_frequency_reduction)]
        clocks::ClockTree::with(clocks::release_apb_clk);
    }
}

/// Per-channel size of the RMT hardware buffer (number of `PulseCode`s).
pub const CHANNEL_RAM_SIZE: usize = property!("rmt.channel_ram_size");

/// Whether the channel supports wrapping rx (wrapping tx is supported on all devices)
pub const HAS_RX_WRAP: bool = property!("rmt.has_rx_wrap");

/// Shared RMT setup: clock source and group clock.
///
/// Channels come from [`Rmt::tx_channel`] and [`Rmt::rx_channel`] and borrow the `Rmt`. The
/// driver mode decides who owns the RMT interrupt: the user in blocking mode
/// ([`Rmt::set_interrupt_handler`]), the driver in async mode.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Rmt<'d, Dm: DriverMode = Blocking> {
    _peripheral: RMT<'d>,
    frequency: Rate,
    _peripheral_guard: GenericPeripheralGuard<{ crate::system::Peripheral::Rmt as u8 }>,
    _clock_guard: RmtClockGuard,
    _mode: PhantomData<Dm>,
}

impl<'d> Rmt<'d, Blocking> {
    /// Creates the RMT driver and configures the group clock.
    ///
    /// # Errors
    ///
    /// None at the moment: an unreachable group frequency selects the nearest one.
    pub fn new(peripheral: RMT<'d>, config: RmtConfig) -> Result<Self, ConfigError> {
        let (divider, frequency) = chip_specific::group_clock(&config);

        let this = Self {
            _peripheral: peripheral,
            frequency,
            _peripheral_guard: GenericPeripheralGuard::new(),
            _clock_guard: RmtClockGuard::new(),
            _mode: PhantomData,
        };

        chip_specific::configure_clock(config.clock_source, divider, frequency);
        Ok(this)
    }

    /// Binds the driver's interrupt handler, which serves the async channels.
    pub fn into_async(mut self) -> Rmt<'d, Async> {
        self.set_interrupt_handler(chip_specific::async_interrupt_handler);
        Rmt {
            _peripheral: self._peripheral,
            frequency: self.frequency,
            _peripheral_guard: self._peripheral_guard,
            _clock_guard: self._clock_guard,
            _mode: PhantomData,
        }
    }

    /// Registers an interrupt handler for the RMT peripheral.
    ///
    /// Replaces any previously registered interrupt handlers. Needs `&mut self`, so it is only
    /// possible while no channel borrows this `Rmt`.
    pub fn set_interrupt_handler(&mut self, handler: crate::interrupt::InterruptHandler) {
        for core in crate::system::Cpu::other() {
            crate::interrupt::disable(core, Interrupt::RMT);
        }
        crate::interrupt::bind_handler(Interrupt::RMT, handler);
    }
}

impl crate::private::Sealed for Rmt<'_, Blocking> {}

impl crate::interrupt::InterruptConfigurable for Rmt<'_, Blocking> {
    fn set_interrupt_handler(&mut self, handler: crate::interrupt::InterruptHandler) {
        self.set_interrupt_handler(handler);
    }
}

impl<'d> Rmt<'d, Async> {
    /// Unbinds the driver's interrupt handler.
    pub fn into_blocking(self) -> Rmt<'d, Blocking> {
        for core in crate::system::Cpu::all() {
            crate::interrupt::disable(core, Interrupt::RMT);
        }
        Rmt {
            _peripheral: self._peripheral,
            frequency: self.frequency,
            _peripheral_guard: self._peripheral_guard,
            _clock_guard: self._clock_guard,
            _mode: PhantomData,
        }
    }

    /// Creates an async transmit channel. It also has the blocking methods.
    ///
    /// # Errors
    ///
    /// See [`Rmt::tx_channel`].
    pub fn async_tx_channel<'a>(
        &'a self,
        channel: impl TxInstance + 'a,
        config: TxConfig,
    ) -> Result<TxChannel<'a, Async>, ConfigError> {
        TxChannel::new(
            channel.info(),
            channel_blocks(&channel),
            self.frequency,
            &config,
        )
    }

    /// Creates an async receive channel. It also has the blocking methods.
    ///
    /// # Errors
    ///
    /// See [`Rmt::rx_channel`].
    pub fn async_rx_channel<'a>(
        &'a self,
        channel: impl RxInstance + 'a,
        config: RxConfig,
    ) -> Result<RxChannel<'a, Async>, ConfigError> {
        RxChannel::new(
            channel.info(),
            rx_channel_blocks(&channel),
            self.frequency,
            &config,
        )
    }
}

fn channel_blocks<T: TxInstance>(_: &T) -> u8 {
    T::BLOCKS
}

fn rx_channel_blocks<T: RxInstance>(_: &T) -> u8 {
    T::BLOCKS
}

impl<'d, Dm: DriverMode> Rmt<'d, Dm> {
    /// Changes the shared settings.
    ///
    /// Needs `&mut self`, so it is only possible while no channel borrows this `Rmt`.
    ///
    /// # Errors
    ///
    /// None at the moment, see [`Rmt::new`].
    pub fn apply_config(&mut self, config: &RmtConfig) -> Result<(), ConfigError> {
        let (divider, frequency) = chip_specific::group_clock(config);
        chip_specific::configure_clock(config.clock_source, divider, frequency);
        self.frequency = frequency;
        Ok(())
    }

    /// Returns the actual group clock frequency.
    ///
    /// The channel tick rates are this frequency divided by the channel dividers.
    pub fn frequency(&self) -> Rate {
        self.frequency
    }

    /// Creates a blocking transmit channel that uses the group clock of this `Rmt`.
    ///
    /// `channel` is a channel singleton, or a tuple of adjacent singletons for more RAM (see
    /// [`TxInstance`]).
    ///
    /// # Errors
    ///
    /// [`ConfigError::UnreachableTickRate`] or [`ConfigError::UnreachableCarrier`].
    pub fn tx_channel<'a>(
        &'a self,
        channel: impl TxInstance + 'a,
        config: TxConfig,
    ) -> Result<TxChannel<'a, Blocking>, ConfigError> {
        TxChannel::new(
            channel.info(),
            channel_blocks(&channel),
            self.frequency,
            &config,
        )
    }

    /// Creates a blocking receive channel that uses the group clock of this `Rmt`.
    ///
    /// `channel` is a channel singleton, or a tuple of adjacent singletons for more RAM (see
    /// [`RxInstance`]).
    ///
    /// # Errors
    ///
    /// [`ConfigError::UnreachableTickRate`], [`ConfigError::UnreachableCarrier`] or
    /// [`ConfigError::RxLimitOutOfRange`].
    pub fn rx_channel<'a>(
        &'a self,
        channel: impl RxInstance + 'a,
        config: RxConfig,
    ) -> Result<RxChannel<'a, Blocking>, ConfigError> {
        RxChannel::new(
            channel.info(),
            rx_channel_blocks(&channel),
            self.frequency,
            &config,
        )
    }
}

/// A transmit channel.
///
/// Created by [`Rmt::tx_channel`] or [`Rmt::async_tx_channel`]. Dropping the channel
/// disconnects the pin and frees the channel singletons.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TxChannel<'a, Dm: DriverMode> {
    raw: DynChannelAccess<Tx>,
    group_frequency: Rate,
    divider: u16,
    _pin_guard: PinGuard,
    _phantom: PhantomData<(&'a (), Dm)>,
}

impl<'a, Dm: DriverMode> TxChannel<'a, Dm> {
    fn new(
        info: &'static Info,
        blocks: u8,
        group_frequency: Rate,
        config: &TxConfig,
    ) -> Result<Self, ConfigError> {
        let raw = unsafe { DynChannelAccess::conjure(info.ch_idx) };
        let divider = apply_tx_config(raw, group_frequency, config)?;
        raw.set_memsize(MemSize::from_blocks(blocks));

        Ok(Self {
            raw,
            group_frequency,
            divider,
            _pin_guard: PinGuard::new_unconnected(),
            _phantom: PhantomData,
        })
    }

    /// Connects a pin to the channel's output signal.
    ///
    /// This will replace previous pin assignments for this signal.
    pub fn with_pin(mut self, pin: impl PeripheralOutput<'a>) -> Self {
        let pin = pin.into();

        // Make sure to not cause a pulse between enabling the output and connecting it.
        let (_, level) = self.raw.tx_idle_output();
        pin.set_output_high(level.into());
        pin.apply_output_config(&OutputConfig::default());
        pin.set_output_enable(true);

        self._pin_guard = pin.connect_with_guard(self.raw.output_signal());

        self
    }

    /// Changes the configuration.
    ///
    /// # Errors
    ///
    /// [`ConfigError::UnreachableTickRate`] or [`ConfigError::UnreachableCarrier`].
    pub fn apply_config(&mut self, config: &TxConfig) -> Result<(), ConfigError> {
        self.divider = apply_tx_config(self.raw, self.group_frequency, config)?;
        Ok(())
    }

    /// Returns the actual tick rate.
    pub fn tick_rate(&self) -> Rate {
        Rate::from_hz(self.group_frequency.as_hz() / self.divider as u32)
    }

    /// Sends `data` and blocks until the last code is out.
    ///
    /// The driver adds the end marker. A code with a zero length ends the transmission early:
    /// the codes after it are not sent. Empty data returns at once.
    ///
    /// # Errors
    ///
    /// [`TxError::Underrun`] if the transmission ended before the driver wrote all data to the
    /// channel RAM.
    #[cfg_attr(place_rmt_driver_in_ram, ram)]
    pub fn transmit(&mut self, data: &[PulseCode]) -> Result<(), TxError> {
        let Some(mut transmission) = Transmission::start(self.raw, data, false) else {
            return Ok(());
        };
        loop {
            if let Some(result) = transmission.poll() {
                return result;
            }
        }
    }

    /// Sends `data` repeatedly until the returned [`TxLoop`] stops.
    ///
    /// The driver adds the end marker that ends each repetition. All codes and the end marker
    /// must fit in the channel RAM. The `mode` argument decides whether the transmission
    /// repeats until it is stopped or for a number of repetitions, see [`LoopMode`].
    ///
    /// # Errors
    ///
    /// [`TxError::DataTooLong`], or `TxError::LoopCountOutOfRange` for a loop count above
    /// `MAX_TX_LOOPCOUNT`.
    #[cfg_attr(place_rmt_driver_in_ram, ram)]
    pub fn transmit_loop(
        &mut self,
        mut data: &[PulseCode],
        mode: LoopMode,
    ) -> Result<TxLoop<'_>, TxError> {
        let raw = self.raw;

        #[cfg(rmt_has_tx_loop_count)]
        if mode.get_count() > MAX_TX_LOOPCOUNT {
            return Err(TxError::LoopCountOutOfRange);
        }

        let mut guard = TxGuard::new(raw);
        #[cfg(rmt_has_tx_loop_auto_stop)]
        let zero_loops = mode == LoopMode::Finite(0);
        #[cfg(not(rmt_has_tx_loop_auto_stop))]
        let zero_loops = false;

        if data.is_empty() || zero_loops {
            guard.set_completed();
        } else {
            let mut writer = RmtWriter::new();
            writer.write(&mut data, raw, true);
            if writer.state() == WriterState::Active {
                return Err(TxError::DataTooLong);
            }

            raw.clear_tx_interrupts(EnumSet::all());
            raw.start_send(Some(mode), raw.memsize());
        }

        Ok(TxLoop {
            raw,
            _guard: guard,
            _channel: PhantomData,
        })
    }
}

impl<'a> TxChannel<'a, Async> {
    /// Sends `data` and returns when the last code is out.
    ///
    /// Dropping the future stops the transmission. See [`TxChannel::transmit`].
    ///
    /// # Errors
    ///
    /// [`TxError::Underrun`] if the task did not refill the channel RAM in time.
    #[cfg_attr(place_rmt_driver_in_ram, ram)]
    pub async fn transmit_async(&mut self, data: &[PulseCode]) -> Result<(), TxError> {
        let Some(transmission) = Transmission::start(self.raw, data, true) else {
            return Ok(());
        };
        TxFuture { transmission }.await
    }
}

impl<Dm: DriverMode> Drop for TxChannel<'_, Dm> {
    fn drop(&mut self) {
        self.raw.set_memsize(MemSize::from_blocks(0));
    }
}

// A transmission in progress: refills the channel RAM and checks the result.
struct Transmission<'d> {
    raw: DynChannelAccess<Tx>,
    writer: RmtWriter,
    // Data that has not yet been written to channel RAM. May be empty.
    data: &'d [PulseCode],
    guard: TxGuard,
}

impl<'d> Transmission<'d> {
    // Returns `None` for empty data: there is nothing to send.
    #[inline(always)]
    fn start(raw: DynChannelAccess<Tx>, mut data: &'d [PulseCode], listen: bool) -> Option<Self> {
        if data.is_empty() {
            return None;
        }

        let mut writer = RmtWriter::new();
        writer.write(&mut data, raw, true);

        raw.clear_tx_interrupts(EnumSet::all());
        if listen {
            let mut events = Event::End | Event::Error;
            if writer.state() == WriterState::Active {
                events |= Event::Threshold;
            }
            raw.listen_tx_interrupt(events);
        }
        raw.start_send(None, raw.memsize());

        Some(Self {
            raw,
            writer,
            data,
            guard: TxGuard::new(raw),
        })
    }

    // Refills the channel RAM if needed. Returns the result once the transmission has ended.
    #[cfg_attr(place_rmt_driver_in_ram, ram)]
    fn poll(&mut self) -> Option<Result<(), TxError>> {
        let raw = self.raw;
        let result = match raw.get_tx_status() {
            Some(Event::Error) => Err(TxError::Underrun),
            Some(Event::End) => self.writer.finish(raw),
            Some(Event::Threshold) => {
                raw.clear_tx_interrupts(Event::Threshold);
                self.writer.write(&mut self.data, raw, false);
                return None;
            }
            _ => return None,
        };

        self.guard.set_completed();
        Some(result)
    }
}

#[must_use = "futures do nothing unless you `.await` or poll them"]
struct TxFuture<'d> {
    transmission: Transmission<'d>,
}

impl core::future::Future for TxFuture<'_> {
    type Output = Result<(), TxError>;

    #[cfg_attr(place_rmt_driver_in_ram, ram)]
    fn poll(self: Pin<&mut Self>, ctx: &mut Context<'_>) -> Poll<Self::Output> {
        let transmission = &mut self.get_mut().transmission;
        let raw = transmission.raw;

        WAKER[raw.channel() as usize].register(ctx.waker());

        match transmission.poll() {
            Some(result) => Poll::Ready(result),
            None => {
                // The interrupt handler disables the threshold interrupt when it fires.
                if transmission.writer.state() == WriterState::Active {
                    raw.listen_tx_interrupt(Event::Threshold);
                }
                Poll::Pending
            }
        }
    }
}

/// Loop mode of [`TxChannel::transmit_loop`].
///
/// Depending on hardware support, the `loopcount` interrupt and automatic stopping of the
/// transmission upon reaching a specified loop count may not be available.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum LoopMode {
    /// Repeats until explicitly stopped.
    Infinite,

    /// Repeats until explicitly stopped, and asserts the loop count interrupt upon completing the
    /// given number of iterations.
    ///
    /// Loop counts larger than [`MAX_TX_LOOPCOUNT`] will result in an error.
    #[cfg(rmt_has_tx_loop_count)]
    InfiniteWithInterrupt(u16),

    /// Repeats for the given number of iterations, and also sets the loop count interrupt flag upon
    /// completion.
    ///
    /// If the iteration count is 0, the transaction will complete immediately without
    /// starting the transmitter.
    ///
    /// Loop counts larger than [`MAX_TX_LOOPCOUNT`] will result in an error.
    #[cfg(rmt_has_tx_loop_auto_stop)]
    Finite(u16),
}

impl LoopMode {
    #[allow(unused)]
    fn get_count(self) -> u16 {
        match self {
            Self::Infinite => 0,
            #[cfg(rmt_has_tx_loop_count)]
            Self::InfiniteWithInterrupt(count) => count,
            #[cfg(rmt_has_tx_loop_auto_stop)]
            Self::Finite(count) => count,
        }
    }
}

/// A loop transmission in progress, see [`TxChannel::transmit_loop`].
///
/// Dropping it stops the transmission.
#[must_use = "dropping the loop stops the transmission"]
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TxLoop<'c> {
    raw: DynChannelAccess<Tx>,
    _guard: TxGuard,
    _channel: PhantomData<&'c mut ()>,
}

impl TxLoop<'_> {
    /// Stops the transmission when the current repetition ends, and waits for that.
    #[cfg_attr(place_rmt_driver_in_ram, inline(always))]
    pub fn stop_next(self) -> Result<(), TxError> {
        self.stop_impl(false)
    }

    /// Stops the transmission as soon as possible.
    #[cfg_attr(place_rmt_driver_in_ram, inline(always))]
    pub fn stop(self) -> Result<(), TxError> {
        self.stop_impl(true)
    }

    #[cfg_attr(place_rmt_driver_in_ram, ram)]
    fn stop_impl(mut self, immediate: bool) -> Result<(), TxError> {
        let raw = self.raw;

        let mut result = Ok(());
        if self._guard.is_active() {
            // If rmt_has_tx_loop_auto_stop and the engine is stopped already, this is not
            // necessary. However, explicitly stopping unconditionally makes the logic
            // here much simpler and shouldn't create much overhead.
            raw.set_tx_continuous(false);
            if immediate {
                raw.stop_tx()
            }
            raw.update();

            if !immediate || !cfg!(rmt_has_tx_immediate_stop) {
                loop {
                    match raw.get_tx_status() {
                        Some(Event::Error) => {
                            result = Err(TxError::Underrun);
                            break;
                        }
                        Some(Event::End) => break,
                        Some(Event::LoopCount) if cfg!(rmt_has_tx_loop_auto_stop) => break,
                        _ => continue,
                    }
                }
            }
        }

        self._guard.set_completed();
        result
    }

    /// Returns whether the `loopcount` interrupt bit is set.
    ///
    /// Whether this implies that the transmission has stopped depends on the [`LoopMode`] value
    /// provided when starting it.
    #[cfg(rmt_has_tx_loop_count)]
    pub fn is_loopcount_interrupt_set(&self) -> bool {
        !self._guard.is_active() || self.raw.is_tx_loopcount_interrupt_set()
    }
}

/// A receive channel.
///
/// Created by [`Rmt::rx_channel`] or [`Rmt::async_rx_channel`].
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RxChannel<'a, Dm: DriverMode> {
    raw: DynChannelAccess<Rx>,
    group_frequency: Rate,
    divider: u16,
    _phantom: PhantomData<(&'a (), Dm)>,
}

impl<'a, Dm: DriverMode> RxChannel<'a, Dm> {
    fn new(
        info: &'static Info,
        blocks: u8,
        group_frequency: Rate,
        config: &RxConfig,
    ) -> Result<Self, ConfigError> {
        let raw = unsafe { DynChannelAccess::conjure(info.ch_idx) };
        let divider = apply_rx_config(raw, group_frequency, config)?;
        raw.set_memsize(MemSize::from_blocks(blocks));

        Ok(Self {
            raw,
            group_frequency,
            divider,
            _phantom: PhantomData,
        })
    }

    /// Connects a pin to the channel's input signal.
    ///
    /// This will replace previous pin assignments for this signal.
    pub fn with_pin(self, pin: impl PeripheralInput<'a>) -> Self {
        let pin = pin.into();
        pin.apply_input_config(&InputConfig::default());
        pin.set_input_enable(true);

        self.raw.input_signal().connect_to(&pin);

        self
    }

    /// Changes the configuration.
    ///
    /// # Errors
    ///
    /// [`ConfigError::UnreachableTickRate`], [`ConfigError::UnreachableCarrier`] or
    /// [`ConfigError::RxLimitOutOfRange`].
    pub fn apply_config(&mut self, config: &RxConfig) -> Result<(), ConfigError> {
        self.divider = apply_rx_config(self.raw, self.group_frequency, config)?;
        Ok(())
    }

    /// Returns the actual tick rate.
    pub fn tick_rate(&self) -> Rate {
        Rate::from_hz(self.group_frequency.as_hz() / self.divider as u32)
    }

    #[procmacros::doc_replace(
        "rx_size_limit" => {
            cfg(any(esp32, esp32s2)) => "A frame cannot be longer than the channel RAM on this chip.",
            _ => ""
        }
    )]
    /// Receives one frame into `buf` and returns the number of codes.
    ///
    /// Blocks until the input stays at one level for longer than the idle threshold.
    ///
    /// # {rx_size_limit}
    ///
    /// # Errors
    ///
    /// [`RxError::BufferTooSmall`] or [`RxError::Overrun`].
    #[cfg_attr(place_rmt_driver_in_ram, ram)]
    pub fn receive(&mut self, buf: &mut [PulseCode]) -> Result<usize, RxError> {
        let mut reception = Reception::start(self.raw, buf, false);
        loop {
            if let Some(result) = reception.poll(false) {
                return result;
            }
        }
    }

    /// Starts receiving one frame into `buf`.
    ///
    /// Returns an [`RxReception`] that can be used to read the codes while the frame arrives and to
    /// stop the reception early.
    #[cfg_attr(
        any(esp32, esp32s2),
        doc = "",
        doc = "A frame cannot be longer than the channel RAM on this chip.",
        doc = ""
    )]
    #[cfg_attr(place_rmt_driver_in_ram, ram)]
    #[instability::unstable]
    pub fn start_receive<'b>(&'b mut self, buf: &'b mut [PulseCode]) -> RxReception<'b> {
        RxReception {
            reception: Reception::start(self.raw, buf, false),
        }
    }
}

/// A reception in progress, see [`RxChannel::start_receive`].
///
/// Dropping it stops the reception.
#[must_use = "dropping the reception stops it"]
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub struct RxReception<'d> {
    reception: Reception<'d>,
}

impl RxReception<'_> {
    /// Copies the received codes into the buffer and returns the result once the reception has
    /// ended.
    ///
    /// Returns `None` while the reception continues. Use [`Self::received`] to read the codes.
    ///
    /// The hardware makes a [`PulseCode`] available only when both of its pulses are complete.
    /// Thus, the last pulse can be missing until the next edge on the input. When the reception
    /// has ended, the buffer contains all codes, including the end marker.
    ///
    /// # Errors
    ///
    /// [`RxError::BufferTooSmall`] or [`RxError::Overrun`].
    #[cfg_attr(place_rmt_driver_in_ram, ram)]
    #[instability::unstable]
    pub fn poll(&mut self) -> Option<Result<usize, RxError>> {
        self.reception.poll(true)
    }

    /// Returns the pulse codes in the buffer.
    ///
    /// [`Self::poll`] copies new codes from the hardware into the buffer.
    #[inline]
    #[instability::unstable]
    pub fn received(&self) -> &[PulseCode] {
        &self.reception.data[..self.reception.reader.total]
    }

    /// Stops the reception and returns the number of received pulse codes.
    ///
    /// The reception stops immediately, before the idle threshold. The codes that the hardware
    /// completed before this call are copied into the buffer. The buffer contains an end marker
    /// only if the reception ended before this call.
    #[cfg_attr(place_rmt_driver_in_ram, ram)]
    #[instability::unstable]
    pub fn stop(mut self) -> usize {
        let reception = &mut self.reception;
        if reception.guard.is_active() && reception.poll(true).is_none() {
            let raw = reception.raw;
            raw.stop_rx(true);
            raw.update();
            raw.unlisten_rx_interrupt(EnumSet::all());
            reception.guard.set_completed();
        }
        reception.reader.total
    }
}

impl<'a> RxChannel<'a, Async> {
    /// Receives one frame into `buf` and returns the number of codes.
    ///
    /// Dropping the future stops the reception. See [`RxChannel::receive`].
    ///
    /// # Errors
    ///
    /// [`RxError::BufferTooSmall`] or [`RxError::Overrun`].
    #[cfg_attr(place_rmt_driver_in_ram, ram)]
    pub async fn receive_async(&mut self, buf: &mut [PulseCode]) -> Result<usize, RxError> {
        RxFuture {
            reception: Reception::start(self.raw, buf, true),
        }
        .await
    }
}

impl<Dm: DriverMode> Drop for RxChannel<'_, Dm> {
    fn drop(&mut self) {
        self.raw.set_memsize(MemSize::from_blocks(0));
    }
}

// A reception in progress: reads the channel RAM and checks the result.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
struct Reception<'d> {
    raw: DynChannelAccess<Rx>,
    reader: RmtReader,
    // `data[..reader.total]` holds the received codes.
    data: &'d mut [PulseCode],
    guard: RxGuard,
}

impl<'d> Reception<'d> {
    #[inline(always)]
    fn start(raw: DynChannelAccess<Rx>, data: &'d mut [PulseCode], listen: bool) -> Self {
        raw.clear_rx_interrupts(EnumSet::all());
        if listen {
            raw.listen_rx_interrupt(Event::End | Event::Error | Event::Threshold);
        }
        raw.start_receive(true, raw.memsize());

        Self {
            raw,
            reader: RmtReader::new(),
            data,
            guard: RxGuard::new(raw),
        }
    }

    #[inline(always)]
    fn read(&mut self, final_: bool) -> bool {
        let mut rest = &mut self.data[self.reader.total..];
        self.reader.read(&mut rest, self.raw, final_)
    }

    // Reads the channel RAM if needed, or always if `live` is set. Returns the result once the
    // reception has ended.
    #[cfg_attr(place_rmt_driver_in_ram, ram)]
    fn poll(&mut self, live: bool) -> Option<Result<usize, RxError>> {
        let raw = self.raw;
        let result = match raw.get_rx_status() {
            // Read all available data also on error
            Some(event @ (Event::End | Event::Error)) => {
                // Rx is stopped already, but we do need to clear the rx enable flag and
                // apply that right away! Otherwise the next `raw.update` call would do so
                // while starting the next transaction, which breaks it when SCLK is slow.
                raw.stop_rx(false);
                raw.update();

                // `RmtReader::read()` returns immediately if the reader is already done.
                if !self.read(true) || event == Event::Error {
                    Err(RxError::Overrun)
                } else if let ReaderState::Error(err) = self.reader.state {
                    Err(err)
                } else {
                    Ok(self.reader.total)
                }
            }
            #[cfg(rmt_has_rx_wrap)]
            Some(Event::Threshold) => {
                raw.clear_rx_interrupts(Event::Threshold);
                self.read(false);
                return None;
            }
            _ => {
                if live {
                    self.read(false);
                }
                return None;
            }
        };

        self.guard.set_completed();
        Some(result)
    }
}

#[must_use = "futures do nothing unless you `.await` or poll them"]
struct RxFuture<'d> {
    reception: Reception<'d>,
}

impl core::future::Future for RxFuture<'_> {
    type Output = Result<usize, RxError>;

    #[cfg_attr(place_rmt_driver_in_ram, ram)]
    fn poll(self: Pin<&mut Self>, ctx: &mut Context<'_>) -> Poll<Self::Output> {
        let reception = &mut self.get_mut().reception;
        let raw = reception.raw;

        WAKER[raw.channel() as usize].register(ctx.waker());

        match reception.poll(false) {
            Some(result) => Poll::Ready(result),
            None => {
                // The interrupt handler disables the threshold interrupt when it fires.
                if reception.reader.state == ReaderState::Active {
                    raw.listen_rx_interrupt(Event::Threshold);
                }
                Poll::Pending
            }
        }
    }
}

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
struct TxGuard {
    raw: Option<DynChannelAccess<Tx>>,
    // The CPU refills the channel RAM. At a low CPU frequency, the refill can be too late.
    _cpu_frequency_lock: Option<CpuFrequencyLock>,
}

impl TxGuard {
    fn new(raw: DynChannelAccess<Tx>) -> Self {
        Self {
            raw: Some(raw),
            _cpu_frequency_lock: Some(CpuFrequencyLock::new()),
        }
    }

    /// Indicates that the transaction has completed (or was never started) and does not require
    /// explicit stopping on drop.
    fn set_completed(&mut self) {
        self.raw = None;
        self._cpu_frequency_lock = None;
    }

    fn is_active(&self) -> bool {
        self.raw.is_some()
    }
}

impl Drop for TxGuard {
    fn drop(&mut self) {
        // The guard is still active if the transmission was aborted or loops. A loop raises
        // `End` at every repetition on some chips, so the status does not tell whether the
        // channel still runs.
        if let Some(raw) = self.raw {
            // In continuous mode, an end marker restarts the loop instead of ending it.
            raw.set_tx_continuous(false);
            raw.stop_tx();
            raw.update();

            // Block until the channel is safe to use again.
            #[cfg(not(rmt_has_tx_immediate_stop))]
            while !raw.is_tx_idle() {}

            raw.unlisten_tx_interrupt(EnumSet::all());
        }
    }
}

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
struct RxGuard {
    raw: Option<DynChannelAccess<Rx>>,
    // The CPU empties the channel RAM. At a low CPU frequency, the read can be too late.
    _cpu_frequency_lock: Option<CpuFrequencyLock>,
}

impl RxGuard {
    fn new(raw: DynChannelAccess<Rx>) -> Self {
        Self {
            raw: Some(raw),
            _cpu_frequency_lock: Some(CpuFrequencyLock::new()),
        }
    }

    /// Indicates that the transaction has completed and does not require explicit stopping on
    /// drop.
    fn set_completed(&mut self) {
        self.raw = None;
        self._cpu_frequency_lock = None;
    }

    fn is_active(&self) -> bool {
        self.raw.is_some()
    }
}

impl Drop for RxGuard {
    fn drop(&mut self) {
        if let Some(raw) = self.raw {
            raw.stop_rx(true);
            raw.update();

            raw.unlisten_rx_interrupt(EnumSet::all());
        }
    }
}

static WAKER: [AtomicWaker; NUM_CHANNELS] = [const { AtomicWaker::new() }; NUM_CHANNELS];

// Lock to synchronize access to registers that are shared between channels.
static RMT_LOCK: RawMutex = RawMutex::new();

// Blocking channels move to other cores and tasks, and all channels borrow `Rmt`. Async drivers
// are not `Send`, see `crate::Async`. Keep these bounds until the stable API decides which ones
// it promises.
const _: () = {
    const fn assert_send<T: Send>() {}
    const fn assert_sync<T: Sync>() {}

    assert_send::<TxChannel<'static, Blocking>>();
    assert_send::<RxChannel<'static, Blocking>>();
    assert_sync::<Rmt<'static, Blocking>>();
    assert_sync::<Rmt<'static, Async>>();
};

#[derive(Debug, EnumSetType)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
enum Event {
    Error,
    Threshold,
    End,
    LoopCount,
}

impl<Dir: Direction> DynChannelAccess<Dir> {
    #[inline]
    fn channel_ram_start_offset(self) -> usize {
        usize::from(self.channel()) * property!("rmt.channel_ram_size")
    }

    #[inline]
    fn channel_ram_start(self) -> *mut PulseCode {
        unsafe {
            (property!("rmt.ram_start") as *mut PulseCode).add(self.channel_ram_start_offset())
        }
    }
}

impl DynChannelAccess<Tx> {
    #[inline(always)]
    fn output_signal(self) -> gpio::OutputSignal {
        OUTPUT_SIGNALS[self.ch_idx as usize]
    }

    // We could obtain `memsize` via `self.memsize()` here. However, it is already known at all
    // call sites, and passing it as argument avoids a volatile read that the compiler wouldn't be
    // able to deduplicate.
    #[inline(always)]
    fn start_send(self, loopmode: Option<LoopMode>, memsize: MemSize) {
        self.set_tx_threshold((memsize.codes() / 2) as u8);
        self.set_tx_continuous(loopmode.is_some());
        #[cfg(rmt_has_tx_loop_count)]
        self.set_loopmode(loopmode);
        self.set_tx_wrap_mode(loopmode.is_none());
        self.update();
        self.start_tx();
        self.update();
    }

    #[inline(always)]
    fn listen_tx_interrupt(self, event: impl Into<EnumSet<Event>>) {
        self.set_tx_interrupt(event.into(), true);
    }

    #[inline(always)]
    fn unlisten_tx_interrupt(self, event: impl Into<EnumSet<Event>>) {
        self.set_tx_interrupt(event.into(), false);
    }
}

impl DynChannelAccess<Rx> {
    #[inline(always)]
    fn input_signal(self) -> gpio::InputSignal {
        INPUT_SIGNALS[self.ch_idx as usize]
    }

    #[inline(always)]
    fn start_receive(self, _wrap: bool, _memsize: MemSize) {
        #[cfg(rmt_has_rx_wrap)]
        {
            self.set_rx_threshold((_memsize.codes() / 2) as u16);
            self.set_rx_wrap_mode(_wrap);
            self.update();
        }

        self.start_rx();
        self.update();
    }

    #[inline(always)]
    fn listen_rx_interrupt(self, event: impl Into<EnumSet<Event>>) {
        self.set_rx_interrupt(event.into(), true);
    }

    #[inline(always)]
    fn unlisten_rx_interrupt(self, event: impl Into<EnumSet<Event>>) {
        self.set_rx_interrupt(event.into(), false);
    }
}

// Obtain maximum value for a register field from the PAC's register spec.
macro_rules! max_from_register_spec {
    ($typ:ty, $reg:ident, $spec:ident, $w:ident) => {{
        use crate::soc::pac::rmt::$reg as reg;
        type Spec = reg::$spec;
        const WIDTH: u8 = reg::$w::<Spec>::WIDTH;

        const _: () = if WIDTH as u32 > <$typ>::BITS {
            core::panic!("Unexpectedly large register WIDTH");
        };

        // Fits into $ty according to the assertion above
        ((1u32 << WIDTH) - 1) as $typ
    }};
}

#[cfg(not(any(esp32, esp32s2)))]
mod chip_specific {
    use enumset::EnumSet;
    use portable_atomic::{AtomicU32, Ordering};
    #[cfg(place_rmt_driver_in_ram)]
    use procmacros::ram;

    use super::{
        ChannelIndex,
        ClockSource,
        Direction,
        DynChannelAccess,
        Event,
        Level,
        LoopMode,
        MemSize,
        RMT_LOCK,
        RmtConfig,
        Rx,
        Tx,
        WAKER,
    };
    use crate::{peripherals::RMT, soc::clocks, time::Rate};

    // Time to wait after a `conf_update`, see `DynChannelAccess::update`.
    static CONF_UPDATE_DELAY_US: AtomicU32 = AtomicU32::new(0);

    cfg_select! {
        esp32p4 => {
            use crate::soc::pac::hp_sys_clkrst::peri_clk_ctrl22 as sclk_conf;
            type Spec = sclk_conf::PERI_CLK_CTRL22_SPEC;
            type DivNumW<'a> = sclk_conf::RMT_CLK_DIV_NUM_W<'a, Spec>;
        }
        esp32s31 => {
            use crate::soc::pac::hp_sys_clkrst::rmt_ctrl0 as sclk_conf;
            type Spec = sclk_conf::RMT_CTRL0_SPEC;
            type DivNumW<'a> = sclk_conf::CLK_DIV_NUM_W<'a, Spec>;
        }
        soc_has_pcr => {
            use crate::soc::pac::pcr::rmt_sclk_conf as sclk_conf;
            type Spec = sclk_conf::RMT_SCLK_CONF_SPEC;
            type DivNumW<'a> = sclk_conf::SCLK_DIV_NUM_W<'a, Spec>;
        }
        _ => {
            use crate::soc::pac::rmt::sys_conf as sclk_conf;
            type Spec = sclk_conf::SYS_CONF_SPEC;
            type DivNumW<'a> = sclk_conf::SCLK_DIV_NUM_W<'a, Spec>;
        }
    }

    // The SCLK divider is `div_num + 1` (the fractional part stays 0).
    const MAX_DIVIDER: u32 = 1 << DivNumW::WIDTH;

    // Returns the integer group divider that gives the nearest clock period, and the actual
    // group clock frequency.
    pub(super) fn group_clock(config: &RmtConfig) -> (u32, Rate) {
        let src_clock = clocks::RmtInstance::sclk_source_frequency(config.clock_source);

        let divider = match config.frequency {
            None => 1,
            Some(frequency) => match frequency.as_hz() {
                0 => MAX_DIVIDER,
                f => ((src_clock + f / 2) / f).clamp(1, MAX_DIVIDER),
            },
        };

        (divider, Rate::from_hz(src_clock / divider))
    }

    pub(super) fn configure_clock(source: ClockSource, divider: u32, frequency: Rate) {
        // Synchronizing a `conf_update` into the SCLK domain takes about one SCLK cycle, wait for
        // two to leave some margin. For a fast SCLK, this is shorter than the register accesses
        // following the update, so don't delay at all.
        let delay_us = if frequency < Rate::from_mhz(10) {
            2_000_000u32.div_ceil(frequency.as_hz())
        } else {
            0
        };
        CONF_UPDATE_DELAY_US.store(delay_us, Ordering::Relaxed);

        clocks::ClockTree::with(|clocks| {
            clocks::RmtInstance::Rmt.configure_sclk(clocks, source);
        });

        let div_num = (divider - 1) as u8;
        // An integral divider is described as `0 / 1`, not as `0 / 0`.
        let numerator = 0;
        let denominator = 1;

        // TODO: configure dividers through clock tree
        // `div_a` is the numerator and `div_b` the denominator, as the RMT clock sections of the
        // TRMs state. Some register descriptions have them swapped.
        cfg_select! {
            esp32p4 => {
                use crate::peripherals::HP_SYS_CLKRST;
                HP_SYS_CLKRST::regs()
                    .peri_clk_ctrl22()
                    .modify(|_, w| unsafe {
                        w.rmt_clk_div_num().bits(div_num);
                        w.rmt_clk_div_numerator().bits(numerator);
                        w.rmt_clk_div_denominator().bits(denominator)
                    });
            }
            esp32s31 => {
                use crate::peripherals::HP_SYS_CLKRST;
                HP_SYS_CLKRST::regs().rmt_ctrl0().modify(|_, w| unsafe {
                    w.clk_div_num().bits(div_num);
                    w.clk_div_numerator().bits(numerator);
                    w.clk_div_denominator().bits(denominator)
                });
            }
            soc_has_pcr => {
                use crate::peripherals::PCR;
                PCR::regs().rmt_sclk_conf().modify(|_, w| unsafe {
                    w.sclk_div_num().bits(div_num);
                    w.sclk_div_a().bits(numerator);
                    w.sclk_div_b().bits(denominator)
                });
            }
            _ => {
                RMT::regs().sys_conf().modify(|_, w| unsafe {
                    w.sclk_div_num().bits(div_num);
                    w.sclk_div_a().bits(numerator);
                    w.sclk_div_b().bits(denominator)
                });
            }
        }

        RMT::regs()
            .sys_conf()
            .modify(|_, w| w.apb_fifo_mask().set_bit());
    }

    #[crate::handler]
    #[cfg_attr(place_rmt_driver_in_ram, ram)]
    pub(super) fn async_interrupt_handler() {
        let st = RMT::regs().int_st().read();

        for ch_idx in ChannelIndex::iter_all() {
            let raw_tx = unsafe { DynChannelAccess::<Tx>::conjure(ch_idx) };
            let raw_rx = unsafe { DynChannelAccess::<Rx>::conjure(ch_idx) };
            let ch_idx = ch_idx as u8;

            let channel = if st.ch_tx_end(ch_idx).bit() || st.ch_tx_err(ch_idx).bit() {
                raw_tx.unlisten_tx_interrupt(EnumSet::all());
                raw_tx.channel()
            } else if st.ch_tx_thr_event(ch_idx).bit() {
                // TxFuture will enable the interrupt again if required.
                raw_tx.unlisten_tx_interrupt(Event::Threshold);
                raw_tx.channel()
            } else if st.ch_rx_end(ch_idx).bit() || st.ch_rx_err(ch_idx).bit() {
                raw_rx.unlisten_rx_interrupt(EnumSet::all());
                raw_rx.channel()
            } else if st.ch_rx_thr_event(ch_idx).bit() {
                // RxFuture will enable the interrupt again if required.
                raw_rx.unlisten_rx_interrupt(Event::Threshold);
                raw_rx.channel()
            } else {
                continue;
            };

            WAKER[channel as usize].wake();
            return;
        }
    }

    impl<Dir: Direction> DynChannelAccess<Dir> {
        pub fn channel(self) -> u8 {
            if Dir::IS_TX {
                self.ch_idx as u8
            } else {
                self.ch_idx as u8 + ChannelIndex::MAX
            }
        }

        pub fn update(self) {
            let rmt = crate::peripherals::RMT::regs();
            let ch_idx = self.ch_idx as usize;

            if Dir::IS_TX {
                rmt.ch_tx_conf0(ch_idx)
                    .modify(|_, w| w.conf_update().set_bit());
            } else {
                rmt.ch_rx_conf1(ch_idx)
                    .modify(|_, w| w.conf_update().set_bit());
            }

            // The update needs to be synchronized into the SCLK domain before it takes effect.
            // Register writes or another update during that time may be lost, which breaks
            // transactions in unexpected ways when SCLK is slow.
            let delay_us = CONF_UPDATE_DELAY_US.load(Ordering::Relaxed);
            if delay_us > 0 {
                crate::rom::ets_delay_us(delay_us);
            }
        }

        pub fn set_divider(self, divider: u8) {
            let rmt = crate::peripherals::RMT::regs();
            let ch_idx = self.ch_idx as usize;

            if Dir::IS_TX {
                rmt.ch_tx_conf0(ch_idx)
                    .modify(|_, w| unsafe { w.div_cnt().bits(divider) });
            } else {
                rmt.ch_rx_conf0(ch_idx)
                    .modify(|_, w| unsafe { w.div_cnt().bits(divider) });
            }
        }

        pub fn memsize(self) -> MemSize {
            let rmt = RMT::regs();
            let ch_idx = self.ch_idx as usize;

            let blocks = if Dir::IS_TX {
                rmt.ch_tx_conf0(ch_idx).read().mem_size().bits()
            } else {
                rmt.ch_rx_conf0(ch_idx).read().mem_size().bits()
            };

            MemSize::from_blocks(blocks)
        }

        pub fn set_memsize(self, value: MemSize) {
            let blocks = value.blocks();
            let rmt = RMT::regs();
            let ch_idx = self.ch_idx as usize;

            if Dir::IS_TX {
                rmt.ch_tx_conf0(ch_idx)
                    .modify(|_, w| unsafe { w.mem_size().bits(blocks) });
            } else {
                rmt.ch_rx_conf0(ch_idx)
                    .modify(|_, w| unsafe { w.mem_size().bits(blocks) });
            }
        }

        #[inline(always)]
        pub fn reset_channel_clock_divider(self) {
            #[cfg(esp32c5)]
            {
                let mask = 1u32 << self.channel();
                let rmt = RMT::regs();

                rmt.ref_cnt_rst().write(|w| unsafe { w.bits(mask) });
                rmt.ref_cnt_rst().write(|w| unsafe { w.bits(0) });
            }
        }
    }

    // documented in re-export below
    #[allow(missing_docs)]
    pub const MAX_TX_LOOPCOUNT: u16 =
        max_from_register_spec!(u16, ch_tx_lim, CH_TX_LIM_SPEC, TX_LOOP_NUM_W);

    impl DynChannelAccess<Tx> {
        #[inline(always)]
        pub fn set_loopmode(self, mode: Option<LoopMode>) {
            let rmt = crate::peripherals::RMT::regs();
            let ch_idx = self.ch_idx as usize;

            if let Some(mode) = mode {
                rmt.ch_tx_lim(ch_idx).modify(|_, w| unsafe {
                    w.loop_count_reset().set_bit();
                    w.tx_loop_cnt_en().bit(!matches!(mode, LoopMode::Infinite));
                    w.tx_loop_num().bits(mode.get_count());

                    #[cfg(rmt_has_tx_loop_auto_stop)]
                    w.loop_stop_en().bit(matches!(mode, LoopMode::Finite(_)));

                    w
                });

                // FIXME: Is this required? This is a WT field for esp32c6 at least
                rmt.ch_tx_lim(ch_idx)
                    .modify(|_, w| w.loop_count_reset().clear_bit());
            } else {
                rmt.ch_tx_lim(ch_idx)
                    .modify(|_, w| w.tx_loop_cnt_en().clear_bit());
            }
        }

        #[inline(always)]
        pub fn clear_tx_interrupts(self, events: impl Into<EnumSet<Event>>) {
            let rmt = crate::peripherals::RMT::regs();
            let events = events.into();

            rmt.int_clr().write(
                #[inline(always)]
                |w| {
                    let ch_idx = self.ch_idx as u8;

                    w.ch_tx_end(ch_idx).bit(events.contains(Event::End));
                    w.ch_tx_err(ch_idx).bit(events.contains(Event::Error));
                    w.ch_tx_loop(ch_idx).bit(events.contains(Event::LoopCount));
                    w.ch_tx_thr_event(ch_idx)
                        .bit(events.contains(Event::Threshold));

                    w
                },
            );
        }

        pub fn set_tx_continuous(self, continuous: bool) {
            let rmt = crate::peripherals::RMT::regs();
            let ch_idx = self.ch_idx as usize;

            rmt.ch_tx_conf0(ch_idx)
                .modify(|_, w| w.tx_conti_mode().bit(continuous));
        }

        pub fn set_tx_wrap_mode(self, wrap: bool) {
            let rmt = crate::peripherals::RMT::regs();
            let ch_idx = self.ch_idx as usize;

            rmt.ch_tx_conf0(ch_idx)
                .modify(|_, w| w.mem_tx_wrap_en().bit(wrap));
        }

        pub fn set_tx_carrier(self, carrier: bool, high: u16, low: u16, level: Level) {
            let rmt = crate::peripherals::RMT::regs();
            let ch_idx = self.ch_idx as usize;

            rmt.chcarrier_duty(ch_idx)
                .write(|w| unsafe { w.carrier_high().bits(high).carrier_low().bits(low) });

            rmt.ch_tx_conf0(ch_idx).modify(|_, w| {
                w.carrier_en().bit(carrier);
                w.carrier_eff_en().set_bit();
                w.carrier_out_lv().bit(level.into())
            });
        }

        pub fn tx_idle_output(self) -> (bool, Level) {
            let rmt = crate::peripherals::RMT::regs();
            let ch_idx = self.ch_idx as usize;

            let reg = rmt.ch_tx_conf0(ch_idx).read();

            (reg.idle_out_en().bit(), reg.idle_out_lv().bit().into())
        }

        pub fn set_tx_idle_output(self, enable: bool, level: Level) {
            let rmt = crate::peripherals::RMT::regs();
            let ch_idx = self.ch_idx as usize;

            rmt.ch_tx_conf0(ch_idx)
                .modify(|_, w| w.idle_out_en().bit(enable).idle_out_lv().bit(level.into()));
        }

        pub fn start_tx(self) {
            let rmt = crate::peripherals::RMT::regs();
            let ch_idx = self.ch_idx as usize;

            self.reset_channel_clock_divider();

            rmt.ch_tx_conf0(ch_idx).modify(|_, w| {
                w.mem_rd_rst().set_bit();
                w.apb_mem_rst().set_bit();
                w.tx_start().set_bit()
            });
        }

        // Return the first flag that is set of, in order of decreasing priority,
        // Event::Error, Event::End, Event::LoopCount, Event::Threshold
        #[inline(always)]
        pub fn get_tx_status(self) -> Option<Event> {
            let rmt = crate::peripherals::RMT::regs();
            let reg = rmt.int_raw().read();
            let ch_idx = self.ch_idx as u8;

            if reg.ch_tx_end(ch_idx).bit() {
                Some(Event::End)
            } else if reg.ch_tx_err(ch_idx).bit() {
                Some(Event::Error)
            } else if reg.ch_tx_loop(ch_idx).bit() {
                Some(Event::LoopCount)
            } else if reg.ch_tx_thr_event(ch_idx).bit() {
                Some(Event::Threshold)
            } else {
                None
            }
        }

        pub fn set_tx_threshold(self, threshold: u8) {
            let rmt = crate::peripherals::RMT::regs();
            let ch_idx = self.ch_idx as usize;

            rmt.ch_tx_lim(ch_idx)
                .modify(|_, w| unsafe { w.tx_lim().bits(threshold as u16) });
        }

        pub fn is_tx_loopcount_interrupt_set(self) -> bool {
            let rmt = crate::peripherals::RMT::regs();
            let ch_idx = self.ch_idx as u8;

            rmt.int_raw().read().ch_tx_loop(ch_idx).bit()
        }

        // Requires an update() call
        pub fn stop_tx(self) {
            let rmt = crate::peripherals::RMT::regs();
            let ch_idx = self.ch_idx as usize;

            rmt.ch_tx_conf0(ch_idx).modify(|_, w| w.tx_stop().set_bit());
        }

        #[inline(always)]
        pub fn set_tx_interrupt(self, events: EnumSet<Event>, enable: bool) {
            let rmt = crate::peripherals::RMT::regs();

            RMT_LOCK.lock(|| {
                rmt.int_ena().modify(
                    #[inline(always)]
                    |_, w| {
                        let ch_idx = self.ch_idx as u8;

                        if events.contains(Event::Error) {
                            w.ch_tx_err(ch_idx).bit(enable);
                        }
                        if events.contains(Event::End) {
                            w.ch_tx_end(ch_idx).bit(enable);
                        }
                        if events.contains(Event::Threshold) {
                            w.ch_tx_thr_event(ch_idx).bit(enable);
                        }
                        w
                    },
                )
            });
        }

        // Position in the channel RAM after the code that the hardware read last.
        pub fn hw_offset(self) -> usize {
            let rmt = crate::peripherals::RMT::regs();
            let ch_idx = self.ch_idx as usize;

            let offset = rmt.ch_tx_status(ch_idx).read().mem_raddr_ex().bits();
            usize::from(offset).wrapping_sub(self.channel_ram_start_offset())
        }
    }

    // documented in re-export below
    #[allow(missing_docs)]
    pub const MAX_RX_IDLE_THRESHOLD: u16 = {
        // TODO: unify naming in PAC
        cfg_select! {
            any(esp32p4, esp32s31) => {
                // 15-bit IDLE_THRES in RX CONF0 (PAC aliases that register to TX CONF0).
                32767
            }
            _ => {
                max_from_register_spec!(u16, ch_rx_conf0, CH_RX_CONF0_SPEC, IDLE_THRES_W)
            }
        }
    };

    impl DynChannelAccess<Rx> {
        #[inline(always)]
        pub fn clear_rx_interrupts(self, events: impl Into<EnumSet<Event>>) {
            let rmt = crate::peripherals::RMT::regs();
            let events = events.into();

            rmt.int_clr().write(
                #[inline(always)]
                |w| {
                    let ch_idx = self.ch_idx as u8;

                    w.ch_rx_end(ch_idx).bit(events.contains(Event::End));
                    w.ch_rx_err(ch_idx).bit(events.contains(Event::Error));
                    w.ch_rx_thr_event(ch_idx)
                        .bit(events.contains(Event::Threshold));
                    w
                },
            );
        }

        pub fn set_rx_wrap_mode(self, wrap: bool) {
            let rmt = crate::peripherals::RMT::regs();
            let ch_idx = self.ch_idx as usize;

            rmt.ch_rx_conf1(ch_idx)
                .modify(|_, w| w.mem_rx_wrap_en().bit(wrap));
        }

        pub fn set_rx_carrier(self, carrier: bool, high: u16, low: u16, level: Level) {
            let rmt = crate::peripherals::RMT::regs();
            let ch_idx = self.ch_idx as usize;

            rmt.ch_rx_carrier_rm(ch_idx).write(|w| unsafe {
                w.carrier_high_thres().bits(high);
                w.carrier_low_thres().bits(low)
            });

            rmt.ch_rx_conf0(ch_idx).modify(|_, w| {
                w.carrier_en()
                    .bit(carrier)
                    .carrier_out_lv()
                    .bit(level.into())
            });
        }

        pub fn start_rx(self) {
            let rmt = crate::peripherals::RMT::regs();
            let ch_idx = self.ch_idx as u8;

            self.reset_channel_clock_divider();

            for i in 1..self.memsize().blocks() {
                rmt.ch_rx_conf1((ch_idx + i).into())
                    .modify(|_, w| w.mem_owner().set_bit());
            }
            rmt.ch_rx_conf1(ch_idx.into()).modify(|_, w| {
                w.mem_owner().set_bit();
                w.mem_wr_rst().set_bit();
                w.apb_mem_rst().set_bit();
                w.rx_en().set_bit()
            });
        }

        // Return the first flag that is set of, in order of decreasing priority,
        // Event::Error, Event::End, Event::Threshold
        #[inline(always)]
        pub fn get_rx_status(self) -> Option<Event> {
            let rmt = crate::peripherals::RMT::regs();
            let reg = rmt.int_raw().read();
            let ch_idx = self.ch_idx as u8;

            if reg.ch_rx_end(ch_idx).bit() {
                Some(Event::End)
            } else if reg.ch_rx_err(ch_idx).bit() {
                Some(Event::Error)
            } else if reg.ch_rx_thr_event(ch_idx).bit() {
                Some(Event::Threshold)
            } else {
                None
            }
        }

        pub fn set_rx_threshold(self, threshold: u16) {
            let rmt = crate::peripherals::RMT::regs();
            let ch_idx = self.ch_idx as usize;

            rmt.ch_rx_lim(ch_idx)
                .modify(|_, w| unsafe { w.rx_lim().bits(threshold) });
        }

        // This is immediate and does not update state flags; do not poll on get_rx_status()
        // afterwards!
        //
        // Requires an update() call
        pub fn stop_rx(self, _force: bool) {
            let rmt = crate::peripherals::RMT::regs();
            let ch_idx = self.ch_idx as usize;

            rmt.ch_rx_conf1(ch_idx).modify(|_, w| w.rx_en().clear_bit());
        }

        pub fn set_rx_filter_threshold(self, value: u8) {
            let rmt = crate::peripherals::RMT::regs();
            let ch_idx = self.ch_idx as usize;

            rmt.ch_rx_conf1(ch_idx).modify(|_, w| unsafe {
                w.rx_filter_en().bit(value > 0);
                w.rx_filter_thres().bits(value)
            });
        }

        pub fn set_rx_idle_threshold(self, value: u16) {
            debug_assert!(value <= MAX_RX_IDLE_THRESHOLD);

            let rmt = crate::peripherals::RMT::regs();
            let ch_idx = self.ch_idx as usize;

            rmt.ch_rx_conf0(ch_idx)
                .modify(|_, w| unsafe { w.idle_thres().bits(value) });
        }

        #[inline(always)]
        pub fn set_rx_interrupt(self, events: EnumSet<Event>, enable: bool) {
            let rmt = crate::peripherals::RMT::regs();

            RMT_LOCK.lock(|| {
                rmt.int_ena().modify(
                    #[inline(always)]
                    |_, w| {
                        let ch_idx = self.ch_idx as u8;

                        if events.contains(Event::Error) {
                            w.ch_rx_err(ch_idx).bit(enable);
                        }
                        if events.contains(Event::End) {
                            w.ch_rx_end(ch_idx).bit(enable);
                        }
                        if events.contains(Event::Threshold) {
                            w.ch_rx_thr_event(ch_idx).bit(enable);
                        }
                        w
                    },
                )
            });
        }

        pub fn hw_offset(self) -> usize {
            let rmt = crate::peripherals::RMT::regs();
            let ch_idx = self.ch_idx as usize;

            let offset = rmt.ch_rx_status(ch_idx).read().mem_waddr_ex().bits();
            usize::from(offset) - self.channel_ram_start_offset()
        }
    }
}

#[cfg(any(esp32, esp32s2))]
mod chip_specific {
    use enumset::EnumSet;
    #[cfg(place_rmt_driver_in_ram)]
    use procmacros::ram;

    use super::{
        ChannelIndex,
        ClockSource,
        Direction,
        DynChannelAccess,
        Event,
        Level,
        MemSize,
        RMT_LOCK,
        RmtConfig,
        Rx,
        Tx,
        WAKER,
    };
    use crate::{peripherals::RMT, soc::clocks, time::Rate};

    // These chips have no group divider: the group clock is the source clock.
    pub(super) fn group_clock(config: &RmtConfig) -> (u32, Rate) {
        let frequency = clocks::RmtInstance::sclk_source_frequency(config.clock_source);
        (1, Rate::from_hz(frequency))
    }

    pub(super) fn configure_clock(source: ClockSource, _divider: u32, _frequency: Rate) {
        let rmt = RMT::regs();

        clocks::ClockTree::with(|clocks| {
            clocks::RmtInstance::Rmt.configure_sclk(clocks, source);
        });

        rmt.apb_conf().modify(|_, w| w.apb_fifo_mask().set_bit());
    }

    #[crate::handler]
    #[cfg_attr(place_rmt_driver_in_ram, ram)]
    pub(super) fn async_interrupt_handler() {
        let st = RMT::regs().int_st().read();

        for ch_idx in ChannelIndex::iter_all() {
            let raw_tx = unsafe { DynChannelAccess::<Tx>::conjure(ch_idx) };
            let raw_rx = unsafe { DynChannelAccess::<Rx>::conjure(ch_idx) };
            let ch_idx = ch_idx as u8;

            if st.ch_tx_end(ch_idx).bit() {
                raw_tx.unlisten_tx_interrupt(EnumSet::all());
            } else if st.ch_rx_end(ch_idx).bit() {
                // While `rx_en` is set, the receiver starts a new frame at the next edge. Clear
                // it now, not when the task polls the future.
                raw_rx.stop_rx(false);
                raw_rx.unlisten_rx_interrupt(EnumSet::all());
            } else if st.ch_err(ch_idx).bit() {
                // On error interrupts, don't bother whether the channel is in Rx or Tx mode, just
                // unlisten all interrupts and wake.
                raw_tx.unlisten_tx_interrupt(EnumSet::all());
                raw_rx.unlisten_rx_interrupt(EnumSet::all());
            } else if st.ch_tx_thr_event(ch_idx).bit() {
                raw_tx.unlisten_tx_interrupt(Event::Threshold);
            } else {
                continue;
            }

            // raw_tx.channel() == raw_rx.channel() == ch_idx for these devices
            WAKER[ch_idx as usize].wake();
            return;
        }
    }

    impl<Dir: Direction> DynChannelAccess<Dir> {
        pub fn channel(self) -> u8 {
            self.ch_idx as u8
        }

        pub fn update(self) {
            // no-op
        }

        pub fn set_divider(self, divider: u8) {
            let rmt = crate::peripherals::RMT::regs();
            let ch = self.ch_idx as usize;

            rmt.chconf0(ch)
                .modify(|_, w| unsafe { w.div_cnt().bits(divider) });
        }

        pub fn memsize(self) -> MemSize {
            let rmt = crate::peripherals::RMT::regs();
            let ch = self.ch_idx as usize;

            let blocks = rmt.chconf0(ch).read().mem_size().bits();
            MemSize::from_blocks(blocks)
        }

        pub fn set_memsize(self, value: MemSize) {
            let rmt = crate::peripherals::RMT::regs();
            let ch = self.ch_idx as usize;

            rmt.chconf0(ch)
                .modify(|_, w| unsafe { w.mem_size().bits(value.blocks()) });
        }
    }

    // documented in re-export below
    #[allow(missing_docs)]
    #[cfg(rmt_has_tx_loop_count)]
    pub const MAX_TX_LOOPCOUNT: u16 =
        max_from_register_spec!(u16, ch_tx_lim, CH_TX_LIM_SPEC, TX_LOOP_NUM_W);

    impl DynChannelAccess<Tx> {
        #[cfg(rmt_has_tx_loop_count)]
        #[inline(always)]
        pub fn set_loopmode(self, mode: Option<super::LoopMode>) {
            let rmt = crate::peripherals::RMT::regs();
            let ch = self.ch_idx as usize;

            if let Some(mode) = mode {
                rmt.ch_tx_lim(ch).modify(|_, w| unsafe {
                    w.loop_count_reset().set_bit();
                    w.tx_loop_cnt_en()
                        .bit(!matches!(mode, super::LoopMode::Infinite));
                    w.tx_loop_num().bits(mode.get_count())
                });

                // FIXME: Is this required?
                rmt.ch_tx_lim(ch)
                    .modify(|_, w| w.loop_count_reset().clear_bit());
            } else {
                rmt.ch_tx_lim(ch)
                    .modify(|_, w| w.tx_loop_cnt_en().clear_bit());
            }
        }

        #[inline(always)]
        pub fn clear_tx_interrupts(self, events: impl Into<EnumSet<Event>>) {
            let rmt = crate::peripherals::RMT::regs();
            let events = events.into();

            rmt.int_clr().write(
                #[inline(always)]
                |w| {
                    let ch = self.ch_idx as u8;

                    w.ch_tx_end(ch).bit(events.contains(Event::End));
                    w.ch_err(ch).bit(events.contains(Event::Error));
                    #[cfg(rmt_has_tx_loop_count)]
                    w.ch_tx_loop(ch).bit(events.contains(Event::LoopCount));
                    w.ch_tx_thr_event(ch).bit(events.contains(Event::Threshold));

                    w
                },
            );
        }

        pub fn set_tx_continuous(self, continuous: bool) {
            let rmt = crate::peripherals::RMT::regs();
            let ch = self.ch_idx as usize;

            rmt.chconf1(ch)
                .modify(|_, w| w.tx_conti_mode().bit(continuous));
        }

        pub fn set_tx_wrap_mode(self, wrap: bool) {
            let rmt = crate::peripherals::RMT::regs();

            // this is "okay", because we use all TX channels always in wrap mode
            rmt.apb_conf().modify(|_, w| w.mem_tx_wrap_en().bit(wrap));
        }

        pub fn set_tx_carrier(self, carrier: bool, high: u16, low: u16, level: Level) {
            let rmt = crate::peripherals::RMT::regs();
            let ch = self.ch_idx as usize;

            rmt.chcarrier_duty(ch)
                .write(|w| unsafe { w.carrier_high().bits(high).carrier_low().bits(low) });

            rmt.chconf0(ch).modify(|_, w| {
                w.carrier_en()
                    .bit(carrier)
                    .carrier_out_lv()
                    .bit(level.into())
            });
        }

        pub fn tx_idle_output(self) -> (bool, Level) {
            let rmt = crate::peripherals::RMT::regs();
            let ch = self.ch_idx as usize;

            let reg = rmt.chconf1(ch).read();

            (reg.idle_out_en().bit(), reg.idle_out_lv().bit().into())
        }

        pub fn set_tx_idle_output(self, enable: bool, level: Level) {
            let rmt = crate::peripherals::RMT::regs();
            let ch = self.ch_idx as usize;

            rmt.chconf1(ch)
                .modify(|_, w| w.idle_out_en().bit(enable).idle_out_lv().bit(level.into()));
        }

        pub fn start_tx(self) {
            let rmt = crate::peripherals::RMT::regs();
            let ch = self.ch_idx as u8;

            for i in 1..self.memsize().blocks() {
                rmt.chconf1((ch + i).into())
                    .modify(|_, w| w.mem_owner().clear_bit());
            }

            rmt.chconf1(ch as usize).modify(|_, w| {
                w.mem_owner().clear_bit();
                w.mem_rd_rst().set_bit();
                w.apb_mem_rst().set_bit();
                w.tx_start().set_bit()
            });
        }

        // Return the first flag that is set of, in order of decreasing priority,
        // Event::Error, Event::End, Event::LoopCount, Event::Threshold
        #[inline(always)]
        pub fn get_tx_status(self) -> Option<Event> {
            let rmt = crate::peripherals::RMT::regs();
            let reg = rmt.int_raw().read();
            let ch = self.ch_idx as u8;

            if reg.ch_tx_end(ch).bit() {
                return Some(Event::End);
            }
            if reg.ch_err(ch).bit() {
                return Some(Event::Error);
            }
            #[cfg(rmt_has_tx_loop_count)]
            if reg.ch_tx_loop(ch).bit() {
                return Some(Event::LoopCount);
            }
            if reg.ch_tx_thr_event(ch).bit() {
                return Some(Event::Threshold);
            }

            None
        }

        pub fn set_tx_threshold(self, threshold: u8) {
            let rmt = crate::peripherals::RMT::regs();
            let ch = self.ch_idx as usize;

            rmt.ch_tx_lim(ch)
                .modify(|_, w| unsafe { w.tx_lim().bits(threshold as u16) });
        }

        #[cfg(rmt_has_tx_loop_count)]
        pub fn is_tx_loopcount_interrupt_set(self) -> bool {
            let rmt = crate::peripherals::RMT::regs();
            let ch = self.ch_idx as u8;

            rmt.int_raw().read().ch_tx_loop(ch).bit()
        }

        // It would be better to simply not define this method if not supported, but that would
        // make the implementation of ContinuousTxTransaction::stop_impl much more awkward.
        #[cfg(not(rmt_has_tx_loop_count))]
        #[allow(unused)]
        pub fn is_tx_loopcount_interrupt_set(self) -> bool {
            false
        }

        #[cfg(rmt_has_tx_immediate_stop)]
        pub fn stop_tx(self) {
            let rmt = crate::peripherals::RMT::regs();
            let ch = self.ch_idx as usize;

            rmt.chconf1(ch).modify(|_, w| w.tx_stop().set_bit());
        }

        #[cfg(not(rmt_has_tx_immediate_stop))]
        pub fn stop_tx(self) {
            let ptr = self.channel_ram_start();
            for idx in 0..self.memsize().codes() {
                unsafe {
                    ptr.add(idx).write_volatile(super::PulseCode::end_marker());
                }
            }
        }

        // Whether the channel state machine is idle.
        #[cfg(not(rmt_has_tx_immediate_stop))]
        pub fn is_tx_idle(self) -> bool {
            let rmt = crate::peripherals::RMT::regs();
            rmt.chstatus(self.ch_idx as usize).read().state().bits() == 0
        }

        #[inline(always)]
        pub fn set_tx_interrupt(self, events: EnumSet<Event>, enable: bool) {
            let rmt = crate::peripherals::RMT::regs();

            RMT_LOCK.lock(|| {
                rmt.int_ena().modify(
                    #[inline(always)]
                    |_, w| {
                        let ch = self.ch_idx as u8;

                        if events.contains(Event::Error) {
                            w.ch_err(ch).bit(enable);
                        }
                        if events.contains(Event::End) {
                            w.ch_tx_end(ch).bit(enable);
                        }
                        if events.contains(Event::Threshold) {
                            w.ch_tx_thr_event(ch).bit(enable);
                        }
                        w
                    },
                )
            });
        }

        // Position in the channel RAM after the code that the hardware read last.
        pub fn hw_offset(self) -> usize {
            let rmt = crate::peripherals::RMT::regs();
            let ch = self.ch_idx as usize;

            let offset = rmt.chstatus(ch).read().mem_raddr_ex().bits();
            usize::from(offset).wrapping_sub(self.channel_ram_start_offset())
        }
    }

    // documented in re-export below
    #[allow(missing_docs)]
    pub const MAX_RX_IDLE_THRESHOLD: u16 =
        max_from_register_spec!(u16, chconf0, CHCONF0_SPEC, IDLE_THRES_W);

    impl DynChannelAccess<Rx> {
        #[inline(always)]
        pub fn clear_rx_interrupts(self, events: impl Into<EnumSet<Event>>) {
            let rmt = crate::peripherals::RMT::regs();
            let events = events.into();

            rmt.int_clr().write(
                #[inline(always)]
                |w| {
                    let ch = self.ch_idx as u8;

                    w.ch_rx_end(ch).bit(events.contains(Event::End));
                    w.ch_err(ch).bit(events.contains(Event::Error));

                    w
                },
            );
        }

        #[cfg(rmt_has_rx_demodulation)]
        pub fn set_rx_carrier(self, carrier: bool, high: u16, low: u16, level: Level) {
            let rmt = crate::peripherals::RMT::regs();
            let ch = self.ch_idx as usize;

            rmt.chcarrier_duty(ch)
                .write(|w| unsafe { w.carrier_high().bits(high).carrier_low().bits(low) });

            rmt.chconf0(ch).modify(|_, w| {
                w.carrier_en()
                    .bit(carrier)
                    .carrier_out_lv()
                    .bit(level.into())
            });
        }

        pub fn start_rx(self) {
            let rmt = crate::peripherals::RMT::regs();
            let ch = self.ch_idx as u8;

            for i in 1..self.memsize().blocks() {
                rmt.chconf1((ch + i).into())
                    .modify(|_, w| w.mem_owner().set_bit());
            }

            rmt.chconf1(ch as usize).modify(|_, w| {
                w.mem_owner().set_bit();
                w.mem_wr_rst().set_bit();
                w.apb_mem_rst().set_bit();
                w.rx_en().set_bit()
            });
        }

        // Return the first flag that is set of, in order of decreasing priority,
        // Event::Error, Event::End, Event::Threshold
        #[inline(always)]
        pub fn get_rx_status(self) -> Option<Event> {
            let rmt = crate::peripherals::RMT::regs();
            let reg = rmt.int_raw().read();
            let ch = self.ch_idx as u8;

            if reg.ch_rx_end(ch).bit() {
                Some(Event::End)
            } else if reg.ch_err(ch).bit() {
                Some(Event::Error)
            } else {
                None
            }
        }

        pub fn stop_rx(self, force: bool) {
            let rmt = crate::peripherals::RMT::regs();
            let ch = self.ch_idx as usize;

            // There's no direct hardware support on these chips for stopping the receiver once it
            // started: It will only stop when it runs into an error or when it detects that the
            // data ended (buffer end or idle threshold). Depending on the current channel
            // settings, this might take a looong time.
            //
            // However, we do need to reliably stop the receiver on `Channel` drop to avoid
            // subsequent transactions receiving some initial garbage.
            //
            // This code attempts to work around this limitation by
            //
            // 1) setting the mem_owner to an invalid value. This doesn't seem to trigger an error
            //    immediately, so presumably the error only occurs when a new pulse code would be
            //    written.
            // 2) lowering the idle threshold and change other settings to make exceeding the
            //    threshold more likely (lower clock divider, enable and set an input filter that is
            //    longer than the idle threshold). The latter should have the same effect as
            //    reconnecting the pin to a constant level, but that tricky to do since we'd also
            //    need restore it afterwards.
            //
            // On its own, 2) should be sufficient and quickly result in an `End` condition. If
            // this is overlooking anything and a new code does happen to be received, 1) should
            // ensure that an `Error` condition occurs and in any case, we never block here for
            // long.
            if force {
                if !rmt.chconf1(ch).read().rx_en().bit() {
                    // Don't lock up trying to stop rx when we didn't start in the first place
                    return;
                }

                // With `rx_en` cleared, the receiver cannot start a frame. If it has not started
                // one yet, it never raises `End` or `Error`.
                rmt.chconf1(ch).modify(|_, w| w.rx_en().clear_bit());
                if rmt.chstatus(ch).read().state().bits() == 0 {
                    return;
                }

                let (old_idle_thres, old_div) = rmt.chconf0(ch).from_modify(|r, w| {
                    let old = (r.idle_thres().bits(), r.div_cnt().bits());
                    unsafe {
                        w.idle_thres().bits(1);
                        w.div_cnt().bits(1);
                    }
                    old
                });

                let (old_filter_en, old_filter_thres) = rmt.chconf1(ch).from_modify(|r, w| {
                    let old = (r.rx_filter_en().bit(), r.rx_filter_thres().bits());
                    w.rx_en().clear_bit();
                    w.rx_filter_en().bit(true);
                    unsafe { w.rx_filter_thres().bits(0xFF) };
                    w.mem_owner().clear_bit();
                    old
                });

                while !matches!(self.get_rx_status(), Some(Event::Error | Event::End)) {}

                // Restore settings

                rmt.chconf0(ch).modify(|_, w| unsafe {
                    w.idle_thres().bits(old_idle_thres);
                    w.div_cnt().bits(old_div)
                });

                rmt.chconf1(ch).modify(|_, w| {
                    w.rx_filter_en().bit(old_filter_en);
                    unsafe { w.rx_filter_thres().bits(old_filter_thres) };
                    w.mem_owner().set_bit()
                });
            } else {
                // Only disable, don't abort.
                rmt.chconf1(ch).modify(|_, w| w.rx_en().clear_bit());
            }
        }

        pub fn set_rx_filter_threshold(self, value: u8) {
            let rmt = crate::peripherals::RMT::regs();
            let ch = self.ch_idx as usize;

            rmt.chconf1(ch).modify(|_, w| unsafe {
                w.rx_filter_en().bit(value > 0);
                w.rx_filter_thres().bits(value)
            });
        }

        pub fn set_rx_idle_threshold(self, value: u16) {
            #[allow(clippy::absurd_extreme_comparisons)]
            {
                debug_assert!(value <= MAX_RX_IDLE_THRESHOLD);
            }

            let rmt = crate::peripherals::RMT::regs();
            let ch = self.ch_idx as usize;

            rmt.chconf0(ch)
                .modify(|_, w| unsafe { w.idle_thres().bits(value) });
        }

        #[inline(always)]
        pub fn set_rx_interrupt(self, events: EnumSet<Event>, enable: bool) {
            let rmt = crate::peripherals::RMT::regs();

            RMT_LOCK.lock(|| {
                rmt.int_ena().modify(
                    #[inline(always)]
                    |_, w| {
                        let ch = self.ch_idx as u8;

                        if events.contains(Event::Error) {
                            w.ch_err(ch).bit(enable);
                        }
                        if events.contains(Event::End) {
                            w.ch_rx_end(ch).bit(enable);
                        }
                        w
                    },
                )
            });
        }

        pub fn hw_offset(self) -> usize {
            let rmt = crate::peripherals::RMT::regs();
            let ch = self.ch_idx as usize;

            let offset = rmt.chstatus(ch).read().mem_waddr_ex().bits();
            usize::from(offset) - self.channel_ram_start_offset()
        }
    }
}

/// The largest idle threshold of a receive channel, in ticks.
pub use chip_specific::MAX_RX_IDLE_THRESHOLD;
/// The largest valid value for loopcounts in [`LoopMode`].
#[cfg(rmt_has_tx_loop_count)]
pub use chip_specific::MAX_TX_LOOPCOUNT;
