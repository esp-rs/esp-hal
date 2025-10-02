#![cfg_attr(docsrs, procmacros::doc_replace(
    "freq" => {
        cfg(esp32h2) => "let freq = Rate::from_mhz(32);",
        _ => "let freq = Rate::from_mhz(80);"
    },
    "channel" => {
        cfg(any(esp32, esp32s2)) => "let mut channel = rmt.channel0.configure_rx(peripherals.GPIO4, rx_config)?;",
        cfg(esp32s3) => "let mut channel = rmt.channel7.configure_rx(peripherals.GPIO4, rx_config)?;",
        _ => "let mut channel = rmt.channel2.configure_rx(peripherals.GPIO4, rx_config)?;"
    },
    "channels_desc" => {
        cfg(esp32) => "8 channels, each of them can be either receiver or transmitter.",
        cfg(esp32s2) => "4 channels, each of them can be either receiver or transmitter.",
        cfg(esp32s3) => "8 channels, `Channel<0>`-`Channel<3>` hardcoded for transmitting signals and `Channel<4>`-`Channel<7>` hardcoded for receiving signals.",
        cfg(any(esp32c3, esp32c6, esp32h2)) => "4 channels, `Channel<0>` and `Channel<1>` hardcoded for transmitting signals and `Channel<2>` and `Channel<3>` hardcoded for receiving signals.",
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
//! There are
//! # {channels_desc}
//!
//! For more information, please refer to the
#![doc = concat!("[ESP-IDF documentation](https://docs.espressif.com/projects/esp-idf/en/latest/", chip!(), "/api-reference/peripherals/rmt.html)")]
//! ## Configuration
//! Each TX/RX channel has the same functionality controlled by a dedicated set
//! of registers and is able to independently transmit or receive data. TX
//! channels are indicated by n which is used as a placeholder for the channel
//! number, and by m for RX channels.
//!
//! ## Examples
//!
//! ### Initialization
//!
//! ```rust, no_run
//! # {before_snippet}
//! # use esp_hal::gpio::Level;
//! # use esp_hal::peripherals::Peripherals;
//! # use esp_hal::rmt::TxChannelConfig;
//! # use esp_hal::rmt::Rmt;
//! # use crate::esp_hal::rmt::TxChannelCreator;
//! # {freq}
//! let rmt = Rmt::new(peripherals.RMT, freq)?;
//! let mut channel = rmt.channel0.configure_tx(
//!     peripherals.GPIO1,
//!     TxChannelConfig::default()
//!         .with_clk_divider(1)
//!         .with_idle_output_level(Level::Low)
//!         .with_idle_output(false)
//!         .with_carrier_modulation(false)
//!         .with_carrier_high(1)
//!         .with_carrier_low(1)
//!         .with_carrier_level(Level::Low),
//! )?;
//! # {after_snippet}
//! ```
//!
//! ### TX operation
//! ```rust, no_run
//! # {before_snippet}
//! # use esp_hal::delay::Delay;
//! # use esp_hal::gpio::Level;
//! # use esp_hal::rmt::{PulseCode, Rmt, TxChannelConfig, TxChannelCreator};
//!
//! // Configure frequency based on chip type
//! # {freq}
//! let rmt = Rmt::new(peripherals.RMT, freq)?;
//!
//! let tx_config = TxChannelConfig::default().with_clk_divider(255);
//!
//! let mut channel = rmt.channel0.configure_tx(peripherals.GPIO4, tx_config)?;
//!
//! let delay = Delay::new();
//!
//! let mut data = [PulseCode::new(Level::High, 200, Level::Low, 50); 20];
//! data[data.len() - 2] = PulseCode::new(Level::High, 3000, Level::Low, 500);
//! data[data.len() - 1] = PulseCode::end_marker();
//!
//! loop {
//!     let transaction = channel.transmit(&data)?;
//!     channel = transaction.wait()?;
//!     delay.delay_millis(500);
//! }
//! # }
//! ```
//!
//! ### RX operation
//! ```rust, no_run
//! # {before_snippet}
//! # use esp_hal::rmt::{PulseCode, Rmt, RxChannelConfig, RxChannelCreator};
//! # use esp_hal::delay::Delay;
//! # use esp_hal::gpio::{Level, Output, OutputConfig};
//!
//! const WIDTH: usize = 80;
//!
//! let mut out = Output::new(peripherals.GPIO5, Level::Low, OutputConfig::default());
//!
//! // Configure frequency based on chip type
//! # {freq}
//! let rmt = Rmt::new(peripherals.RMT, freq)?;
//!
//! let rx_config = RxChannelConfig::default()
//!     .with_clk_divider(1)
//!     .with_idle_threshold(10000);
//! # {channel}
//! let delay = Delay::new();
//! let mut data: [PulseCode; 48] = [PulseCode::default(); 48];
//!
//! loop {
//!     for x in data.iter_mut() {
//!         x.reset()
//!     }
//!
//!     let transaction = channel.receive(&mut data)?;
//!
//!     // Simulate input
//!     for i in 0u32..5u32 {
//!         out.set_high();
//!         delay.delay_micros(i * 10 + 20);
//!         out.set_low();
//!         delay.delay_micros(i * 20 + 20);
//!     }
//!
//!     match transaction.wait() {
//!         Ok(channel_res) => {
//!             channel = channel_res;
//!             let mut total = 0usize;
//!             for entry in &data[..data.len()] {
//!                 if entry.length1() == 0 {
//!                     break;
//!                 }
//!                 total += entry.length1() as usize;
//!
//!                 if entry.length2() == 0 {
//!                     break;
//!                 }
//!                 total += entry.length2() as usize;
//!             }
//!
//!             for entry in &data[..data.len()] {
//!                 if entry.length1() == 0 {
//!                     break;
//!                 }
//!
//!                 let count = WIDTH / (total / entry.length1() as usize);
//!                 let c = match entry.level1() {
//!                     Level::High => '-',
//!                     Level::Low => '_',
//!                 };
//!                 for _ in 0..count + 1 {
//!                     print!("{}", c);
//!                 }
//!
//!                 if entry.length2() == 0 {
//!                     break;
//!                 }
//!
//!                 let count = WIDTH / (total / entry.length2() as usize);
//!                 let c = match entry.level2() {
//!                     Level::High => '-',
//!                     Level::Low => '_',
//!                 };
//!                 for _ in 0..count + 1 {
//!                     print!("{}", c);
//!                 }
//!             }
//!
//!             println!();
//!         }
//!         Err((_err, channel_res)) => {
//!             channel = channel_res;
//!         }
//!     }
//!
//!     delay.delay_millis(1500);
//! }
//! # }
//! ```
//!
//! > Note: on ESP32 and ESP32-S2 you cannot specify a base frequency other than 80 MHz

use core::{
    default::Default,
    marker::PhantomData,
    num::NonZeroU16,
    pin::Pin,
    task::{Context, Poll},
};

use enumset::{EnumSet, EnumSetType};
use portable_atomic::Ordering;
#[cfg(place_rmt_driver_in_ram)]
use procmacros::ram;

use crate::{
    Async,
    Blocking,
    asynch::AtomicWaker,
    clock::Clocks,
    gpio::{
        self,
        InputConfig,
        Level,
        OutputConfig,
        interconnect::{PeripheralInput, PeripheralOutput},
    },
    handler,
    peripherals::{Interrupt, RMT},
    system::{self, GenericPeripheralGuard},
    time::Rate,
};

mod reader;
use reader::RmtReader;
mod writer;
use writer::RmtWriter;

/// Errors
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(clippy::enum_variant_names, reason = "peripheral is unstable")]
#[non_exhaustive]
pub enum Error {
    /// The desired frequency is impossible to reach
    UnreachableTargetFrequency,
    /// The amount of pulses exceeds the size of the FIFO
    Overflow,
    /// An argument is invalid
    InvalidArgument,
    /// An error occurred during transmission
    TransmissionError,
    /// No transmission end marker found
    EndMarkerMissing,
    /// Memsize is not correct,
    InvalidMemsize,
    /// The data length is invalid
    InvalidDataLength,
    /// Receiver error most likely RMT memory overflow
    ReceiverError,
    /// Memory block is not available for channel
    MemoryBlockNotAvailable,
}

impl core::error::Error for Error {}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Error::UnreachableTargetFrequency => {
                write!(f, "The desired frequency is impossible to reach")
            }
            Error::Overflow => write!(f, "The amount of pulses exceeds the size of the FIFO"),
            Error::InvalidArgument => write!(f, "An argument is invalid"),
            Error::TransmissionError => write!(f, "An error occurred during transmission"),
            Error::EndMarkerMissing => write!(f, "No transmission end marker found"),
            Error::InvalidMemsize => write!(f, "Memsize is not correct,"),
            Error::InvalidDataLength => write!(f, "The data length is invalid"),
            Error::ReceiverError => write!(f, "Receiver error most likely RMT memory overflow"),
            Error::MemoryBlockNotAvailable => {
                write!(f, "Memory block is not available for channel")
            }
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
pub struct PulseCode(pub u32);

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

    /// Create a new instance.
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

    /// Create a new instance.
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

    /// Create a new instance, attempting to convert lengths to `u16` first.
    ///
    /// This is slightly more convenient when passing in longer integers (e.g. `u32`) resulting from
    /// a preceding calculation.
    ///
    /// If `length1` or `length2` fail to convert to `u16` or exceed the maximum representable
    /// range, this will return `None`.
    #[inline]
    pub fn try_new(
        level1: Level,
        length1: impl TryInto<u16>,
        level2: Level,
        length2: impl TryInto<u16>,
    ) -> Option<Self> {
        let (Ok(length1), Ok(length2)) = (length1.try_into(), length2.try_into()) else {
            return None;
        };
        if length1 > Self::MAX_LEN || length2 >= Self::MAX_LEN {
            return None;
        }

        // SAFETY:
        // - We just checked that length1 and length2 are in range
        Some(unsafe { Self::new_unchecked(level1, length1, level2, length2) })
    }

    /// Create a new instance without checking that code lengths are in range.
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

    /// Create a new instance that is an end marker with `Level::Low`.
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

    /// Set all levels and lengths to 0.
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

    /// Set `level1` and return the modified [`PulseCode`].
    #[inline]
    pub const fn with_level1(mut self, level: Level) -> Self {
        self.0 &= !LEVEL1_MASK;
        self.0 |= (level.const_into() as u32) << LEVEL1_SHIFT;
        self
    }

    /// Set `level2` and return the modified [`PulseCode`].
    #[inline]
    pub const fn with_level2(mut self, level: Level) -> Self {
        self.0 &= !LEVEL2_MASK;
        self.0 |= (level.const_into() as u32) << LEVEL2_SHIFT;
        self
    }

    /// Set `length1` and return the modified [`PulseCode`].
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

    /// Set `length2` and return the modified [`PulseCode`].
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

    /// Return whether this pulse code contains an end marker.
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
    /// Create from the given number of RMT RAM blocks.
    #[inline]
    const fn from_blocks(blocks: u8) -> Self {
        Self(blocks)
    }

    /// Return the number of RMT RAM blocks specified by this `MemSize`.
    #[inline]
    const fn blocks(self) -> u8 {
        self.0
    }

    /// Return the number of RMT pulse codes specified by this `MemSize`.
    #[inline]
    const fn codes(self) -> usize {
        self.0 as usize * property!("rmt.channel_ram_size")
    }
}

/// Marker for a channel capable of/configured for transmit operations
#[derive(Clone, Copy, Debug)]
pub struct Tx;

/// Marker for a channel capable of/configured for receive operations
#[derive(Clone, Copy, Debug)]
pub struct Rx;

/// A trait implemented by the `Rx` and `Tx` marker structs.
///
/// For internal use by the driver.
pub trait Direction: Copy + Clone + core::fmt::Debug + crate::private::Sealed {
    #[doc(hidden)]
    const IS_TX: bool;
}

impl crate::private::Sealed for Tx {}

impl crate::private::Sealed for Rx {}

impl Direction for Tx {
    const IS_TX: bool = true;
}

impl Direction for Rx {
    const IS_TX: bool = false;
}

#[derive(Clone, Copy, Debug)]
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

/// Channel configuration for TX channels
#[derive(Debug, Copy, Clone, procmacros::BuilderLite)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TxChannelConfig {
    /// Channel's clock divider
    clk_divider: u8,
    /// Set the idle output level to low/high
    idle_output_level: Level,
    /// Enable idle output
    idle_output: bool,
    /// Enable carrier modulation
    carrier_modulation: bool,
    /// Carrier high phase in ticks
    carrier_high: u16,
    /// Carrier low phase in ticks
    carrier_low: u16,
    /// Level of the carrier
    carrier_level: Level,
    /// The amount of memory blocks allocated to this channel
    memsize: u8,
}

impl Default for TxChannelConfig {
    fn default() -> Self {
        Self {
            clk_divider: Default::default(),
            idle_output_level: Level::Low,
            idle_output: Default::default(),
            carrier_modulation: Default::default(),
            carrier_high: Default::default(),
            carrier_low: Default::default(),
            carrier_level: Level::Low,
            memsize: 1,
        }
    }
}

/// Channel configuration for RX channels
#[derive(Debug, Copy, Clone, procmacros::BuilderLite)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RxChannelConfig {
    /// Channel's clock divider
    clk_divider: u8,
    /// Enable carrier demodulation
    carrier_modulation: bool,
    /// Carrier high phase in ticks
    carrier_high: u16,
    /// Carrier low phase in ticks
    carrier_low: u16,
    /// Level of the carrier
    carrier_level: Level,
    /// Filter threshold in ticks
    filter_threshold: u8,
    /// Idle threshold in ticks
    idle_threshold: u16,
    /// The amount of memory blocks allocted to this channel
    memsize: u8,
}

impl Default for RxChannelConfig {
    fn default() -> Self {
        Self {
            clk_divider: Default::default(),
            carrier_modulation: Default::default(),
            carrier_high: Default::default(),
            carrier_low: Default::default(),
            carrier_level: Level::Low,
            filter_threshold: Default::default(),
            idle_threshold: Default::default(),
            memsize: 1,
        }
    }
}

// Channel specification

// Looping all channels: Build up the Rmt struct and compute NUM_CHANNELS
for_each_rmt_channel!(
    (all $(($num:literal)),+) => {
        paste::paste! {
            /// RMT Instance
            pub struct Rmt<'d, Dm>
            where
                Dm: crate::DriverMode,
            {
                pub(super) peripheral: RMT<'d>,
                $(
                    #[doc = concat!("RMT Channel ", $num)]
                    pub [<channel $num>]: ChannelCreator<Dm, $num>,
                )+
                _mode: PhantomData<Dm>,
            }

            impl<'d, Dm> Rmt<'d, Dm>
            where
                Dm: crate::DriverMode,
            {
                fn create(peripheral: RMT<'d>) -> Self {
                    Self {
                        peripheral,
                        $(
                            [<channel $num>]: ChannelCreator::conjure(),
                        )+
                        _mode: PhantomData,
                    }
                }
            }

            #[allow(clippy::no_effect)]
            const NUM_CHANNELS: usize = const { 0 $( + {$num; 1} )+ };
        }
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
            #[repr(u8)]
            #[allow(unused)]
            pub enum ChannelIndex {
                $(
                    [<Ch $idx>] = $idx,
                )+
            }

            #[allow(clippy::no_effect)]
            const CHANNEL_INDEX_COUNT: u8 = const { 0 $( + {$idx; 1} )+ };

            const OUTPUT_SIGNALS: [gpio::OutputSignal; CHANNEL_INDEX_COUNT as usize] = [
                $(
                    gpio::OutputSignal::[<RMT_SIG_ $idx>],
                )+
            ];

            $(
                impl<'d, Dm> TxChannelCreator<'d, Dm> for ChannelCreator<Dm, $num>
                where
                    Dm: crate::DriverMode,
                {
                    fn configure_tx(
                        self,
                        pin: impl PeripheralOutput<'d>,
                        config: TxChannelConfig,
                    ) -> Result<Channel<Dm, Tx>, Error>
                    where
                        Self: Sized,
                    {
                        unsafe { Channel::configure_tx(ChannelIndex::[<Ch $idx>], pin.into(), config) }
                    }
                }
            )+
        }
    };

    (rx $(($num:literal, $idx:literal)),+) => {
        paste::paste! {
            const INPUT_SIGNALS: [gpio::InputSignal; CHANNEL_INDEX_COUNT as usize] = [
                $(
                    gpio::InputSignal::[<RMT_SIG_ $idx>],
                )+
            ];

            $(
                impl<'d, Dm> RxChannelCreator<'d, Dm> for ChannelCreator<Dm, $num>
                where
                    Dm: crate::DriverMode,
                {
                    fn configure_rx(
                        self,
                        pin: impl PeripheralInput<'d>,
                        config: RxChannelConfig,
                    ) -> Result<Channel<Dm, Rx>, Error>
                    where
                        Self: Sized,
                    {
                        unsafe { Channel::configure_rx(ChannelIndex::[<Ch $idx>], pin.into(), config) }
                    }
                }
            )+
        }
    };
);

struct ChannelIndexIter(u8);

impl Iterator for ChannelIndexIter {
    type Item = ChannelIndex;

    fn next(&mut self) -> Option<Self::Item> {
        let ch_idx = self.0;
        if ch_idx < CHANNEL_INDEX_COUNT {
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
        debug_assert!(ch_idx < CHANNEL_INDEX_COUNT);
        unsafe { core::mem::transmute(ch_idx) }
    }
}

impl<'d> Rmt<'d, Blocking> {
    /// Create a new RMT instance
    pub fn new(peripheral: RMT<'d>, frequency: Rate) -> Result<Self, Error> {
        let this = Rmt::create(peripheral);
        self::chip_specific::configure_clock(ClockSource::default(), frequency)?;
        Ok(this)
    }

    /// Reconfigures the driver for asynchronous operation.
    pub fn into_async(mut self) -> Rmt<'d, Async> {
        self.set_interrupt_handler(async_interrupt_handler);
        Rmt::create(self.peripheral)
    }

    /// Registers an interrupt handler for the RMT peripheral.
    ///
    /// Note that this will replace any previously registered interrupt
    /// handlers.
    #[instability::unstable]
    pub fn set_interrupt_handler(&mut self, handler: crate::interrupt::InterruptHandler) {
        for core in crate::system::Cpu::other() {
            crate::interrupt::disable(core, Interrupt::RMT);
        }
        unsafe { crate::interrupt::bind_interrupt(Interrupt::RMT, handler.handler()) };
        unwrap!(crate::interrupt::enable(Interrupt::RMT, handler.priority()));
    }
}

impl crate::private::Sealed for Rmt<'_, Blocking> {}

#[instability::unstable]
impl crate::interrupt::InterruptConfigurable for Rmt<'_, Blocking> {
    fn set_interrupt_handler(&mut self, handler: crate::interrupt::InterruptHandler) {
        self.set_interrupt_handler(handler);
    }
}

// Mark the channel as used and reserve the channel RAM.
//
// If this is not possible (because a preceding channel is using the RAM, or
// because subsequent channels are in use so that we can't reserve the RAM),
// restore all state and return with an error.
fn reserve_channel(channel: u8, state: RmtState, memsize: MemSize) -> Result<(), Error> {
    if memsize.blocks() == 0 || memsize.blocks() > NUM_CHANNELS as u8 - channel {
        return Err(Error::InvalidMemsize);
    }

    let mut next_state = state;
    for cur_channel in channel..channel + memsize.blocks() {
        if RmtState::compare_exchange(
            cur_channel,
            RmtState::Unconfigured,
            next_state,
            Ordering::Acquire,
            Ordering::Relaxed,
        )
        .is_err()
        {
            RmtState::store_range_rev(
                RmtState::Unconfigured,
                channel,
                cur_channel,
                Ordering::Relaxed,
            );

            return Err(Error::MemoryBlockNotAvailable);
        }

        // Set the first channel to `state` (`Rx`|`Tx`), the remaining (if any) to
        // `Reserved`
        next_state = RmtState::Reserved;
    }

    Ok(())
}

// We store values of type `RmtState` in the global `STATE`. However, we also need atomic access,
// thus the enum needs to be represented as AtomicU8. Thus, we end up with unsafe conversions
// between `RmtState` and `u8` to avoid constant range checks.
// This submodule wraps all accesses in a safe API which ensures that only valid enum values can
// be stored to `STATE` such that other code in this driver does not need to be concerned with
// these details.
mod state {
    use portable_atomic::{AtomicU8, Ordering};

    use super::{Direction, DynChannelAccess, NUM_CHANNELS};

    static STATE: [AtomicU8; NUM_CHANNELS] =
        [const { AtomicU8::new(RmtState::Unconfigured as u8) }; NUM_CHANNELS];

    #[derive(Copy, Clone, Debug, PartialEq, Eq)]
    #[repr(u8)]
    pub(super) enum RmtState {
        // The channel is not configured for either rx or tx, and its memory is available
        Unconfigured,

        // The channels is not in use, but one of the preceding channels is using its memory
        Reserved,

        // The channel is configured for rx
        Rx,

        // The channel is configured for tx
        Tx,
    }

    impl RmtState {
        /// Check whether this state corresponds to a rx, tx, or other configuration.
        #[allow(unused)]
        #[inline]
        pub(super) fn is_tx(&self) -> Option<bool> {
            match self {
                Self::Rx => Some(false),
                Self::Tx => Some(true),
                _ => None,
            }
        }

        /// Convert a `u8` to `Self` without checking that it has a valid value for the enum.
        ///
        /// # Safety
        ///
        /// - Must only be called with valid values of the RmtState discrimiant
        #[allow(unused)]
        #[inline]
        unsafe fn from_u8_unchecked(value: u8) -> Self {
            unsafe { core::mem::transmute::<_, Self>(value) }
        }

        /// Load channel state from the global `STATE` by channel index.
        ///
        /// # Safety:
        ///
        /// - the `channel` number must be in 0..NUM_CHANNELS
        #[allow(unused)]
        #[inline]
        pub(super) unsafe fn load_by_channel_number(channel: u8, ordering: Ordering) -> Self {
            unsafe { Self::from_u8_unchecked(STATE[channel as usize].load(ordering)) }
        }

        /// Store the given state to all channel states for an index range in reverse order.
        #[inline]
        pub(super) fn store_range_rev(self, start: u8, end: u8, ordering: Ordering) {
            for ch_num in (start as usize..end as usize).rev() {
                STATE[ch_num].store(self as u8, ordering);
            }
        }

        /// Store channel state to the global `STATE` given a `DynChannelAccess`.
        #[allow(unused)]
        #[inline]
        pub(super) fn store<Dir: Direction>(self, raw: DynChannelAccess<Dir>, ordering: Ordering) {
            STATE[raw.channel() as usize].store(self as u8, ordering);
        }

        /// Load channel state from the global `STATE` given a `DynChannelAccess`.
        #[allow(unused)]
        #[inline]
        pub(super) fn load<Dir: Direction>(raw: DynChannelAccess<Dir>, ordering: Ordering) -> Self {
            // SAFETY: The only implementations of RawChannelAccess are in this module and can only
            // be (safely) constructed using valid channel numbers.
            unsafe { Self::load_by_channel_number(raw.channel(), ordering) }
        }

        /// Perform a compare_exchange on the global `STATE` by channel index.
        #[inline]
        pub(super) fn compare_exchange(
            ch_num: u8,
            current: Self,
            new: Self,
            success: Ordering,
            failure: Ordering,
        ) -> Result<Self, Self> {
            STATE[ch_num as usize]
                .compare_exchange(current as u8, new as u8, success, failure)
                .map(|prev| unsafe { Self::from_u8_unchecked(prev) })
                .map_err(|prev| unsafe { Self::from_u8_unchecked(prev) })
        }
    }
}

use state::RmtState;

/// RMT Channel
#[derive(Debug)]
#[non_exhaustive]
pub struct Channel<Dm, Dir>
where
    Dm: crate::DriverMode,
    Dir: Direction,
{
    raw: DynChannelAccess<Dir>,
    _mode: PhantomData<Dm>,
    _guard: GenericPeripheralGuard<{ system::Peripheral::Rmt as u8 }>,
}

impl<Dm> Channel<Dm, Tx>
where
    Dm: crate::DriverMode,
{
    unsafe fn configure_tx<'d>(
        ch_idx: ChannelIndex,
        pin: gpio::interconnect::OutputSignal<'d>,
        config: TxChannelConfig,
    ) -> Result<Self, Error> {
        let raw = unsafe { DynChannelAccess::conjure(ch_idx) };

        let _guard = GenericPeripheralGuard::new();

        let memsize = MemSize::from_blocks(config.memsize);
        reserve_channel(raw.channel(), RmtState::Tx, memsize)?;

        pin.apply_output_config(&OutputConfig::default());
        pin.set_output_enable(true);

        raw.output_signal().connect_to(&pin);

        raw.set_divider(config.clk_divider);
        raw.set_tx_carrier(
            config.carrier_modulation,
            config.carrier_high,
            config.carrier_low,
            config.carrier_level,
        );
        raw.set_tx_idle_output(config.idle_output, config.idle_output_level);
        raw.set_memsize(memsize);

        Ok(Self {
            raw,
            _mode: core::marker::PhantomData,
            _guard,
        })
    }
}

impl<Dm> Channel<Dm, Rx>
where
    Dm: crate::DriverMode,
{
    unsafe fn configure_rx<'d>(
        ch_idx: ChannelIndex,
        pin: gpio::interconnect::InputSignal<'d>,
        config: RxChannelConfig,
    ) -> Result<Self, Error> {
        let raw = unsafe { DynChannelAccess::conjure(ch_idx) };

        let _guard = GenericPeripheralGuard::new();

        if config.idle_threshold > property!("rmt.max_idle_threshold") {
            return Err(Error::InvalidArgument);
        }

        let memsize = MemSize::from_blocks(config.memsize);
        reserve_channel(raw.channel(), RmtState::Rx, memsize)?;

        pin.apply_input_config(&InputConfig::default());
        pin.set_input_enable(true);

        raw.input_signal().connect_to(&pin);

        raw.set_divider(config.clk_divider);
        raw.set_rx_carrier(
            config.carrier_modulation,
            config.carrier_high,
            config.carrier_low,
            config.carrier_level,
        );
        raw.set_rx_filter_threshold(config.filter_threshold);
        raw.set_rx_idle_threshold(config.idle_threshold);
        raw.set_memsize(memsize);

        Ok(Self {
            raw,
            _mode: core::marker::PhantomData,
            _guard,
        })
    }
}

impl<Dm, Dir> Drop for Channel<Dm, Dir>
where
    Dm: crate::DriverMode,
    Dir: Direction,
{
    fn drop(&mut self) {
        let memsize = self.raw.memsize();

        // This isn't really necessary, but be extra sure that this channel can't
        // interfere with others.
        self.raw.set_memsize(MemSize::from_blocks(0));

        // Existence of this `Channel` struct implies exclusive access to these hardware
        // channels, thus simply store the new state.
        RmtState::store_range_rev(
            RmtState::Unconfigured,
            self.raw.channel(),
            self.raw.channel() + memsize.blocks(),
            Ordering::Release,
        );
    }
}

/// Creates a TX channel
pub trait TxChannelCreator<'d, Dm>
where
    Dm: crate::DriverMode,
{
    /// Configure the TX channel
    fn configure_tx(
        self,
        pin: impl PeripheralOutput<'d>,
        config: TxChannelConfig,
    ) -> Result<Channel<Dm, Tx>, Error>
    where
        Self: Sized;
}

/// Creates a RX channel
pub trait RxChannelCreator<'d, Dm>
where
    Dm: crate::DriverMode,
{
    /// Configure the RX channel
    fn configure_rx(
        self,
        pin: impl PeripheralInput<'d>,
        config: RxChannelConfig,
    ) -> Result<Channel<Dm, Rx>, Error>
    where
        Self: Sized;
}

/// An in-progress transaction for a single shot TX transaction.
///
/// If the data size exceeds the size of the internal buffer, `.poll()` or
/// `.wait()` needs to be called before the entire buffer has been sent to avoid
/// underruns.
pub struct SingleShotTxTransaction<'a, T>
where
    T: Into<PulseCode> + Copy,
{
    channel: Channel<Blocking, Tx>,

    writer: RmtWriter,

    // Remaining data that has not yet been written to channel RAM. May be empty.
    remaining_data: &'a [T],
}

impl<T> SingleShotTxTransaction<'_, T>
where
    T: Into<PulseCode> + Copy,
{
    #[cfg_attr(place_rmt_driver_in_ram, ram)]
    fn poll_internal(&mut self) -> Option<Event> {
        let raw = self.channel.raw;

        let status = raw.get_tx_status();
        if status == Some(Event::Threshold) {
            raw.reset_tx_threshold_set();

            // `RmtWriter::write()` is safe to call even if `poll_internal` is called repeatedly
            // after the data is exhausted since it returns immediately if already done.
            self.writer.write(&mut self.remaining_data, raw, false);
        }

        status
    }

    /// Check transmission status and write new data to the hardware if
    /// necessary.
    ///
    /// Returns whether transmission has ended (whether successfully or with an
    /// error). In that case, a subsequent call to `wait()` returns immediately.
    #[cfg_attr(place_rmt_driver_in_ram, inline(always))]
    pub fn poll(&mut self) -> bool {
        match self.poll_internal() {
            Some(Event::Error | Event::End) => true,
            Some(Event::Threshold) | None => false,
        }
    }

    /// Wait for the transaction to complete
    #[cfg_attr(place_rmt_driver_in_ram, inline(always))]
    pub fn wait(mut self) -> Result<Channel<Blocking, Tx>, (Error, Channel<Blocking, Tx>)> {
        // Not sure that all the error cases below can happen. However, it's best to
        // handle them to be sure that we don't lock up here in case they can happen.
        loop {
            match self.poll_internal() {
                Some(Event::Error) => break Err((Error::TransmissionError, self.channel)),
                Some(Event::End) => {
                    if !self.remaining_data.is_empty() {
                        // Unexpectedly done, even though we have data left: For example, this could
                        // happen if there is a stop code inside the data and not just at the end.
                        break Err((Error::TransmissionError, self.channel));
                    } else {
                        break Ok(self.channel);
                    }
                }
                _ => continue,
            }
        }
    }
}

/// An in-progress continuous TX transaction
pub struct ContinuousTxTransaction {
    channel: Channel<Blocking, Tx>,
}

impl ContinuousTxTransaction {
    /// Stop transaction when the current iteration ends.
    #[cfg_attr(place_rmt_driver_in_ram, inline(always))]
    pub fn stop_next(self) -> Result<Channel<Blocking, Tx>, (Error, Channel<Blocking, Tx>)> {
        self.stop_impl(false)
    }

    /// Stop transaction as soon as possible.
    #[cfg_attr(place_rmt_driver_in_ram, inline(always))]
    pub fn stop(self) -> Result<Channel<Blocking, Tx>, (Error, Channel<Blocking, Tx>)> {
        self.stop_impl(true)
    }

    #[cfg_attr(place_rmt_driver_in_ram, ram)]
    fn stop_impl(
        self,
        immediate: bool,
    ) -> Result<Channel<Blocking, Tx>, (Error, Channel<Blocking, Tx>)> {
        let raw = self.channel.raw;

        raw.set_tx_continuous(false);
        let immediate = if immediate { raw.stop_tx() } else { false };
        raw.update();

        if immediate {
            return Ok(self.channel);
        }

        loop {
            match raw.get_tx_status() {
                Some(Event::Error) => break Err((Error::TransmissionError, self.channel)),
                Some(Event::End) => break Ok(self.channel),
                _ => continue,
            }
        }
    }

    /// Check if the `loopcount` interrupt bit is set
    pub fn is_loopcount_interrupt_set(&self) -> bool {
        self.channel.raw.is_tx_loopcount_interrupt_set()
    }
}

/// RMT Channel Creator
pub struct ChannelCreator<Dm, const CHANNEL: u8>
where
    Dm: crate::DriverMode,
{
    _mode: PhantomData<Dm>,
    _guard: GenericPeripheralGuard<{ crate::system::Peripheral::Rmt as u8 }>,
}

impl<Dm: crate::DriverMode, const CHANNEL: u8> ChannelCreator<Dm, CHANNEL> {
    fn conjure() -> ChannelCreator<Dm, CHANNEL> {
        ChannelCreator {
            _mode: PhantomData,
            _guard: GenericPeripheralGuard::new(),
        }
    }

    /// Unsafely steal a channel creator instance.
    ///
    /// # Safety
    ///
    /// Circumvents HAL ownership and safety guarantees and allows creating
    /// multiple handles to the same peripheral structure.
    #[inline]
    pub unsafe fn steal() -> ChannelCreator<Dm, CHANNEL> {
        ChannelCreator {
            _mode: PhantomData,
            _guard: GenericPeripheralGuard::new(),
        }
    }
}

/// Loopcount for continuous transmission
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum LoopCount {
    /// Repeat until explicitly stopped.
    Infinite,

    /// Repeat the given number of times.
    Finite(NonZeroU16),
}

// LoopCount::Infinite should be represented as 0u16, which corresponds to the value that needs to
// be written to the tx_lim register.
const _: () = if core::mem::size_of::<LoopCount>() != 2 {
    // This must not be defmt::panic!, which doesn't work in const context.
    core::panic!("Niche optimization unexpectedly not working!");
};

/// Channel in TX mode
impl Channel<Blocking, Tx> {
    /// Start transmitting the given pulse code sequence.
    /// This returns a [`SingleShotTxTransaction`] which can be used to wait for
    /// the transaction to complete and get back the channel for further
    /// use.
    #[cfg_attr(place_rmt_driver_in_ram, ram)]
    pub fn transmit<T>(self, mut data: &[T]) -> Result<SingleShotTxTransaction<'_, T>, Error>
    where
        T: Into<PulseCode> + Copy,
    {
        let raw = self.raw;
        let memsize = raw.memsize();

        match data.last() {
            None => return Err(Error::InvalidArgument),
            Some(&code) if code.into().is_end_marker() => (),
            Some(_) => return Err(Error::EndMarkerMissing),
        }

        let mut writer = RmtWriter::new();
        writer.write(&mut data, raw, true);

        raw.start_send(None, memsize);

        Ok(SingleShotTxTransaction {
            channel: self,
            writer,
            remaining_data: data,
        })
    }

    /// Start transmitting the given pulse code continuously.
    ///
    /// This returns a [`ContinuousTxTransaction`] which can be used to stop the
    /// ongoing transmission and get back the channel for further use.
    /// When setting a finite `loopcount`, [`ContinuousTxTransaction::is_loopcount_interrupt_set`]
    /// can be used to check if the loop count is reached.
    ///
    /// The length of sequence cannot exceed the size of the allocated RMT RAM.
    #[cfg_attr(place_rmt_driver_in_ram, ram)]
    pub fn transmit_continuously<T>(
        self,
        mut data: &[T],
        loopcount: LoopCount,
    ) -> Result<ContinuousTxTransaction, Error>
    where
        T: Into<PulseCode> + Copy,
    {
        let raw = self.raw;
        let memsize = raw.memsize();

        if data.is_empty() {
            return Err(Error::InvalidArgument);
        } else if data.len() > memsize.codes() {
            return Err(Error::Overflow);
        }

        let mut writer = RmtWriter::new();
        writer.write(&mut data, raw, true);

        self.raw.start_send(Some(loopcount), memsize);

        Ok(ContinuousTxTransaction { channel: self })
    }
}

/// RX transaction instance
pub struct RxTransaction<'a, T>
where
    T: From<PulseCode>,
{
    channel: Channel<Blocking, Rx>,

    reader: RmtReader,

    data: &'a mut [T],
}

impl<T> RxTransaction<'_, T>
where
    T: From<PulseCode>,
{
    #[cfg_attr(place_rmt_driver_in_ram, ram)]
    fn poll_internal(&mut self) -> Option<Event> {
        let raw = self.channel.raw;

        let status = raw.get_rx_status();

        if status == Some(Event::End) {
            // Do not clear the interrupt flags here: Subsequent calls of wait() must
            // be able to observe them if this is currently called via poll()
            raw.stop_rx();
            raw.update();

            // `RmtReader::read()` is safe to call even if `poll_internal` is called repeatedly
            // after the receiver finished since it returns immediately if already done.
            self.reader.read(&mut self.data, raw, true);
        }

        status
    }

    /// Check receive status
    ///
    /// Returns whether reception has ended (whether successfully or with an
    /// error). In that case, a subsequent call to `wait()` returns immediately.
    #[cfg_attr(place_rmt_driver_in_ram, inline(always))]
    pub fn poll(&mut self) -> bool {
        match self.poll_internal() {
            Some(Event::Error | Event::End) => true,
            Some(Event::Threshold) | None => false,
        }
    }

    /// Wait for the transaction to complete
    #[cfg_attr(place_rmt_driver_in_ram, inline(always))]
    pub fn wait(mut self) -> Result<Channel<Blocking, Rx>, (Error, Channel<Blocking, Rx>)> {
        let raw = self.channel.raw;

        let result = loop {
            match self.poll_internal() {
                Some(Event::Error) => break Err((Error::ReceiverError, self.channel)),
                Some(Event::End) => break Ok(self.channel),
                _ => continue,
            }
        };

        raw.clear_rx_interrupts();

        result
    }
}

/// Channel is RX mode
impl Channel<Blocking, Rx> {
    /// Start receiving pulse codes into the given buffer.
    /// This returns a [RxTransaction] which can be used to wait for receive to
    /// complete and get back the channel for further use.
    /// The length of the received data cannot exceed the allocated RMT RAM.
    #[cfg_attr(place_rmt_driver_in_ram, ram)]
    pub fn receive<T>(self, data: &mut [T]) -> Result<RxTransaction<'_, T>, Error>
    where
        Self: Sized,
        T: From<PulseCode>,
    {
        let raw = self.raw;
        let memsize = raw.memsize();

        if data.len() > memsize.codes() {
            return Err(Error::InvalidDataLength);
        }

        let reader = RmtReader::new();

        raw.start_receive();

        Ok(RxTransaction {
            channel: self,
            reader,
            data,
        })
    }
}

static WAKER: [AtomicWaker; NUM_CHANNELS] = [const { AtomicWaker::new() }; NUM_CHANNELS];
#[must_use = "futures do nothing unless you `.await` or poll them"]
struct RmtTxFuture {
    raw: DynChannelAccess<Tx>,
}

impl core::future::Future for RmtTxFuture {
    type Output = Result<(), Error>;

    #[cfg_attr(place_rmt_driver_in_ram, ram)]
    fn poll(self: Pin<&mut Self>, ctx: &mut Context<'_>) -> Poll<Self::Output> {
        WAKER[self.raw.channel() as usize].register(ctx.waker());

        match self.raw.get_tx_status() {
            Some(Event::Error) => Poll::Ready(Err(Error::TransmissionError)),
            Some(Event::End) => Poll::Ready(Ok(())),
            _ => Poll::Pending,
        }
    }
}

/// TX channel in async mode
impl Channel<Async, Tx> {
    /// Start transmitting the given pulse code sequence.
    /// The length of sequence cannot exceed the size of the allocated RMT
    /// RAM.
    #[cfg_attr(place_rmt_driver_in_ram, ram)]
    pub async fn transmit<T>(&mut self, mut data: &[T]) -> Result<(), Error>
    where
        Self: Sized,
        T: Into<PulseCode> + Copy,
    {
        let raw = self.raw;
        let memsize = raw.memsize();

        match data.last() {
            None => return Err(Error::InvalidArgument),
            Some(&code) if code.into().is_end_marker() => (),
            Some(_) => return Err(Error::EndMarkerMissing),
        }

        if data.len() > memsize.codes() {
            return Err(Error::InvalidDataLength);
        }

        let mut writer = RmtWriter::new();
        writer.write(&mut data, raw, true);

        raw.clear_tx_interrupts();
        raw.listen_tx_interrupt(Event::End | Event::Error);
        raw.start_send(None, memsize);

        (RmtTxFuture { raw }).await
    }
}

#[must_use = "futures do nothing unless you `.await` or poll them"]
struct RmtRxFuture {
    raw: DynChannelAccess<Rx>,
}

impl core::future::Future for RmtRxFuture {
    type Output = Result<(), Error>;

    #[cfg_attr(place_rmt_driver_in_ram, ram)]
    fn poll(self: Pin<&mut Self>, ctx: &mut Context<'_>) -> Poll<Self::Output> {
        WAKER[self.raw.channel() as usize].register(ctx.waker());

        match self.raw.get_rx_status() {
            Some(Event::Error) => Poll::Ready(Err(Error::ReceiverError)),
            Some(Event::End) => Poll::Ready(Ok(())),
            _ => Poll::Pending,
        }
    }
}

/// RX channel in async mode
impl Channel<Async, Rx> {
    /// Start receiving a pulse code sequence.
    /// The length of sequence cannot exceed the size of the allocated RMT
    /// RAM.
    #[cfg_attr(place_rmt_driver_in_ram, ram)]
    pub async fn receive<T>(&mut self, mut data: &mut [T]) -> Result<(), Error>
    where
        Self: Sized,
        T: From<PulseCode>,
    {
        let raw = self.raw;
        let memsize = raw.memsize();

        if data.len() > memsize.codes() {
            return Err(Error::InvalidDataLength);
        }

        let mut reader = RmtReader::new();

        raw.clear_rx_interrupts();
        raw.listen_rx_interrupt(Event::End | Event::Error);
        raw.start_receive();

        let result = (RmtRxFuture { raw }).await;

        if result.is_ok() {
            raw.stop_rx();
            raw.clear_rx_interrupts();
            raw.update();

            reader.read(&mut data, raw, true);
        }

        result
    }
}

#[handler]
fn async_interrupt_handler() {
    fn on_tx(raw: DynChannelAccess<Tx>) {
        raw.unlisten_tx_interrupt(Event::End | Event::Error);

        WAKER[raw.channel() as usize].wake();
    }

    fn on_rx(raw: DynChannelAccess<Rx>) {
        raw.unlisten_rx_interrupt(Event::End | Event::Error);

        WAKER[raw.channel() as usize].wake();
    }

    chip_specific::handle_channel_interrupts(on_tx, on_rx);
}

#[derive(Debug, EnumSetType)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
enum Event {
    Error,
    Threshold,
    End,
}

impl<Dir: Direction> DynChannelAccess<Dir> {
    #[inline]
    fn channel_ram_start(&self) -> *mut PulseCode {
        unsafe {
            (property!("rmt.ram_start") as *mut PulseCode)
                .add(usize::from(self.channel()) * property!("rmt.channel_ram_size"))
        }
    }
}

impl DynChannelAccess<Tx> {
    fn output_signal(&self) -> gpio::OutputSignal {
        OUTPUT_SIGNALS[self.ch_idx as usize]
    }

    // We could obtain `memsize` via `self.memsize()` here. However, it is already known at all
    // call sites, and passing it as argument avoids a volatile read that the compiler wouldn't be
    // able to deduplicate.
    #[inline]
    fn start_send(&self, loopcount: Option<LoopCount>, memsize: MemSize) {
        self.clear_tx_interrupts();
        self.set_tx_threshold((memsize.codes() / 2) as u8);
        self.set_tx_continuous(loopcount.is_some());
        self.set_generate_repeat_interrupt(loopcount);
        self.set_tx_wrap_mode(true);
        self.update();
        self.start_tx();
        self.update();
    }

    #[inline]
    fn listen_tx_interrupt(&self, event: impl Into<EnumSet<Event>>) {
        self.set_tx_interrupt(event.into(), true);
    }

    #[inline]
    fn unlisten_tx_interrupt(&self, event: impl Into<EnumSet<Event>>) {
        self.set_tx_interrupt(event.into(), false);
    }
}

impl DynChannelAccess<Rx> {
    fn input_signal(&self) -> gpio::InputSignal {
        INPUT_SIGNALS[self.ch_idx as usize]
    }

    fn start_receive(&self) {
        self.clear_rx_interrupts();
        self.set_rx_wrap_mode(false);
        self.start_rx();
        self.update();
    }

    #[inline]
    fn listen_rx_interrupt(&self, event: impl Into<EnumSet<Event>>) {
        self.set_rx_interrupt(event.into(), true);
    }

    #[inline]
    fn unlisten_rx_interrupt(&self, event: impl Into<EnumSet<Event>>) {
        self.set_rx_interrupt(event.into(), false);
    }
}

for_each_rmt_clock_source!(
    (all $(($name:ident, $bits:literal)),+) => {
        #[derive(Clone, Copy, Debug)]
        #[repr(u8)]
        enum ClockSource {
            $(
                #[allow(unused)]
                $name = $bits,
            )+
        }
    };

    (is_boolean) => {
        impl ClockSource {
            fn bit(self) -> bool {
                match (self as u8) {
                    0 => false,
                    1 => true,
                    _ => unreachable!("should be removed by the compiler!"),
                }
            }
        }
    };

    (default ($name:ident)) => {
        impl ClockSource {
            fn default() -> Self {
                Self::$name
            }

            fn bits(self) -> u8 {
                self as u8
            }

            fn freq(self) -> crate::time::Rate {
                match self {
                    #[cfg(rmt_supports_apb_clock)]
                    ClockSource::Apb => Clocks::get().apb_clock,

                    #[cfg(rmt_supports_rcfast_clock)]
                    ClockSource::RcFast => todo!(),

                    #[cfg(rmt_supports_xtal_clock)]
                    ClockSource::Xtal => Clocks::get().xtal_clock,

                    #[cfg(rmt_supports_pll80mhz_clock)]
                    ClockSource::Pll80MHz => Rate::from_mhz(80),

                    #[cfg(rmt_supports_reftick_clock)]
                    ClockSource::RefTick => todo!(),
                }
            }
        }
    };
);

#[cfg(not(any(esp32, esp32s2)))]
mod chip_specific {
    use enumset::EnumSet;

    use super::{
        CHANNEL_INDEX_COUNT,
        ChannelIndex,
        ClockSource,
        Direction,
        DynChannelAccess,
        Error,
        Event,
        Level,
        LoopCount,
        MemSize,
        Rx,
        Tx,
    };
    use crate::{peripherals::RMT, time::Rate};

    pub(super) fn configure_clock(source: ClockSource, frequency: Rate) -> Result<(), Error> {
        let src_clock = source.freq();

        if frequency > src_clock {
            return Err(Error::UnreachableTargetFrequency);
        }

        let Ok(div) = u8::try_from((src_clock / frequency) - 1) else {
            return Err(Error::UnreachableTargetFrequency);
        };

        #[cfg(not(soc_has_pcr))]
        {
            RMT::regs().sys_conf().modify(|_, w| unsafe {
                w.clk_en().clear_bit();
                w.sclk_sel().bits(source.bits());
                w.sclk_div_num().bits(div);
                w.sclk_div_a().bits(0);
                w.sclk_div_b().bits(0);
                w.apb_fifo_mask().set_bit()
            });
        }

        #[cfg(soc_has_pcr)]
        {
            use crate::peripherals::PCR;
            PCR::regs().rmt_sclk_conf().modify(|_, w| unsafe {
                w.sclk_div_num().bits(div);
                w.sclk_div_a().bits(0);
                w.sclk_div_b().bits(0)
            });

            #[cfg(esp32c6)]
            PCR::regs()
                .rmt_sclk_conf()
                .modify(|_, w| unsafe { w.sclk_sel().bits(source.bits()) });
            #[cfg(not(esp32c6))]
            PCR::regs()
                .rmt_sclk_conf()
                .modify(|_, w| w.sclk_sel().bit(source.bit()));

            RMT::regs()
                .sys_conf()
                .modify(|_, w| w.apb_fifo_mask().set_bit());
        }

        Ok(())
    }

    #[allow(unused)]
    #[inline]
    pub(super) fn handle_channel_interrupts(
        on_tx: fn(DynChannelAccess<Tx>),
        on_rx: fn(DynChannelAccess<Rx>),
    ) {
        let st = RMT::regs().int_st().read();

        for ch_idx in ChannelIndex::iter_all() {
            if st.ch_tx_end(ch_idx as u8).bit() || st.ch_tx_err(ch_idx as u8).bit() {
                let raw = unsafe { DynChannelAccess::<Tx>::conjure(ch_idx) };
                on_tx(raw);
                return;
            }
            if st.ch_rx_end(ch_idx as u8).bit() || st.ch_rx_err(ch_idx as u8).bit() {
                let raw = unsafe { DynChannelAccess::<Rx>::conjure(ch_idx) };
                on_rx(raw);
                return;
            }
        }
    }

    impl<Dir: Direction> DynChannelAccess<Dir> {
        #[inline]
        pub fn channel(&self) -> u8 {
            if Dir::IS_TX {
                self.ch_idx as u8
            } else {
                self.ch_idx as u8 + CHANNEL_INDEX_COUNT
            }
        }

        #[inline]
        pub fn update(&self) {
            let rmt = crate::peripherals::RMT::regs();
            let ch_idx = self.ch_idx as usize;

            if Dir::IS_TX {
                rmt.ch_tx_conf0(ch_idx)
                    .modify(|_, w| w.conf_update().set_bit());
            } else {
                rmt.ch_rx_conf1(ch_idx)
                    .modify(|_, w| w.conf_update().set_bit());
            }
        }

        pub fn set_divider(&self, divider: u8) {
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

        #[inline]
        pub fn memsize(&self) -> MemSize {
            let rmt = RMT::regs();
            let ch_idx = self.ch_idx as usize;

            let blocks = if Dir::IS_TX {
                rmt.ch_tx_conf0(ch_idx).read().mem_size().bits()
            } else {
                rmt.ch_rx_conf0(ch_idx).read().mem_size().bits()
            };

            MemSize::from_blocks(blocks)
        }

        pub fn set_memsize(&self, value: MemSize) {
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
    }

    impl DynChannelAccess<Tx> {
        #[inline]
        pub fn set_generate_repeat_interrupt(&self, loopcount: Option<LoopCount>) {
            let rmt = crate::peripherals::RMT::regs();

            rmt.ch_tx_lim(self.channel().into()).modify(|_, w| unsafe {
                w.loop_count_reset().set_bit();

                match loopcount {
                    Some(LoopCount::Finite(repeats)) if repeats.get() > 1 => {
                        w.tx_loop_cnt_en().set_bit();
                        w.tx_loop_num().bits(repeats.get())
                    }
                    _ => {
                        w.tx_loop_cnt_en().clear_bit();
                        w.tx_loop_num().bits(0)
                    }
                }
            });

            rmt.ch_tx_lim(self.channel().into())
                .modify(|_, w| w.loop_count_reset().clear_bit());
        }

        #[inline]
        pub fn clear_tx_interrupts(&self) {
            let rmt = crate::peripherals::RMT::regs();

            rmt.int_clr().write(|w| {
                w.ch_tx_end(self.channel()).set_bit();
                w.ch_tx_err(self.channel()).set_bit();
                w.ch_tx_loop(self.channel()).set_bit();
                w.ch_tx_thr_event(self.channel()).set_bit()
            });
        }

        #[inline]
        pub fn set_tx_continuous(&self, continuous: bool) {
            let rmt = crate::peripherals::RMT::regs();

            rmt.ch_tx_conf0(self.channel().into())
                .modify(|_, w| w.tx_conti_mode().bit(continuous));
        }

        #[inline]
        pub fn set_tx_wrap_mode(&self, wrap: bool) {
            let rmt = crate::peripherals::RMT::regs();

            rmt.ch_tx_conf0(self.channel().into())
                .modify(|_, w| w.mem_tx_wrap_en().bit(wrap));
        }

        pub fn set_tx_carrier(&self, carrier: bool, high: u16, low: u16, level: Level) {
            let rmt = crate::peripherals::RMT::regs();

            rmt.chcarrier_duty(self.channel().into())
                .write(|w| unsafe { w.carrier_high().bits(high).carrier_low().bits(low) });

            rmt.ch_tx_conf0(self.channel().into()).modify(|_, w| {
                w.carrier_en().bit(carrier);
                w.carrier_eff_en().set_bit();
                w.carrier_out_lv().bit(level.into())
            });
        }

        pub fn set_tx_idle_output(&self, enable: bool, level: Level) {
            let rmt = crate::peripherals::RMT::regs();
            rmt.ch_tx_conf0(self.channel().into())
                .modify(|_, w| w.idle_out_en().bit(enable).idle_out_lv().bit(level.into()));
        }

        #[inline]
        pub fn start_tx(&self) {
            let rmt = crate::peripherals::RMT::regs();

            rmt.ch_tx_conf0(self.channel().into()).modify(|_, w| {
                w.mem_rd_rst().set_bit();
                w.apb_mem_rst().set_bit();
                w.tx_start().set_bit()
            });
        }

        // Return the first flag that is set of, in order of decreasing priority,
        // Event::Error, Event::End, Event::Threshold
        #[inline]
        pub fn get_tx_status(&self) -> Option<Event> {
            let rmt = crate::peripherals::RMT::regs();
            let reg = rmt.int_raw().read();
            let ch = self.channel();

            if reg.ch_tx_end(ch).bit() {
                Some(Event::End)
            } else if reg.ch_tx_err(ch).bit() {
                Some(Event::Error)
            } else if reg.ch_tx_thr_event(ch).bit() {
                Some(Event::Threshold)
            } else {
                None
            }
        }

        #[inline]
        pub fn reset_tx_threshold_set(&self) {
            let rmt = crate::peripherals::RMT::regs();
            rmt.int_clr()
                .write(|w| w.ch_tx_thr_event(self.channel()).set_bit());
        }

        #[inline]
        pub fn set_tx_threshold(&self, threshold: u8) {
            let rmt = crate::peripherals::RMT::regs();
            rmt.ch_tx_lim(self.channel().into())
                .modify(|_, w| unsafe { w.tx_lim().bits(threshold as u16) });
        }

        #[inline]
        pub fn is_tx_loopcount_interrupt_set(&self) -> bool {
            let rmt = crate::peripherals::RMT::regs();
            rmt.int_raw().read().ch_tx_loop(self.channel()).bit()
        }

        // Returns whether stopping was immediate, or needs to wait for tx end.
        // Due to inlining, the compiler should be able to eliminate code in the caller that
        // depends on this.
        //
        // Requires an update() call
        #[inline]
        pub fn stop_tx(&self) -> bool {
            let rmt = crate::peripherals::RMT::regs();
            rmt.ch_tx_conf0(self.channel().into())
                .modify(|_, w| w.tx_stop().set_bit());
            true
        }

        #[inline]
        pub fn set_tx_interrupt(&self, events: EnumSet<Event>, enable: bool) {
            let rmt = crate::peripherals::RMT::regs();
            rmt.int_ena().modify(|_, w| {
                if events.contains(Event::Error) {
                    w.ch_tx_err(self.channel()).bit(enable);
                }
                if events.contains(Event::End) {
                    w.ch_tx_end(self.channel()).bit(enable);
                }
                if events.contains(Event::Threshold) {
                    w.ch_tx_thr_event(self.channel()).bit(enable);
                }
                w
            });
        }
    }

    impl DynChannelAccess<Rx> {
        #[inline]
        pub fn clear_rx_interrupts(&self) {
            let rmt = crate::peripherals::RMT::regs();
            let ch_idx = self.ch_idx as u8;

            rmt.int_clr().write(|w| {
                w.ch_rx_end(ch_idx).set_bit();
                w.ch_rx_err(ch_idx).set_bit();
                w.ch_rx_thr_event(ch_idx).set_bit()
            });
        }

        #[inline]
        pub fn set_rx_wrap_mode(&self, wrap: bool) {
            let rmt = crate::peripherals::RMT::regs();
            let ch_idx = self.ch_idx as usize;

            rmt.ch_rx_conf1(ch_idx)
                .modify(|_, w| w.mem_rx_wrap_en().bit(wrap));
        }

        pub fn set_rx_carrier(&self, carrier: bool, high: u16, low: u16, level: Level) {
            let ch_idx = self.ch_idx as usize;
            let rmt = crate::peripherals::RMT::regs();

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

        #[inline]
        pub fn start_rx(&self) {
            let rmt = crate::peripherals::RMT::regs();
            let ch_idx = self.ch_idx as u8;

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
        #[inline]
        pub fn get_rx_status(&self) -> Option<Event> {
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

        #[inline]
        pub fn stop_rx(&self) {
            let rmt = crate::peripherals::RMT::regs();
            let ch_idx = self.ch_idx as usize;
            rmt.ch_rx_conf1(ch_idx).modify(|_, w| w.rx_en().clear_bit());
        }

        pub fn set_rx_filter_threshold(&self, value: u8) {
            let rmt = crate::peripherals::RMT::regs();
            let ch_idx = self.ch_idx as usize;

            rmt.ch_rx_conf1(ch_idx).modify(|_, w| unsafe {
                w.rx_filter_en().bit(value > 0);
                w.rx_filter_thres().bits(value)
            });
        }

        pub fn set_rx_idle_threshold(&self, value: u16) {
            let rmt = crate::peripherals::RMT::regs();
            let ch_idx = self.ch_idx as usize;

            rmt.ch_rx_conf0(ch_idx)
                .modify(|_, w| unsafe { w.idle_thres().bits(value) });
        }

        #[inline]
        pub fn set_rx_interrupt(&self, events: EnumSet<Event>, enable: bool) {
            let rmt = crate::peripherals::RMT::regs();
            let ch_idx = self.ch_idx as u8;

            rmt.int_ena().modify(|_, w| {
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
            });
        }
    }
}

#[cfg(any(esp32, esp32s2))]
mod chip_specific {
    use enumset::EnumSet;
    use portable_atomic::Ordering;

    use super::{
        ChannelIndex,
        ClockSource,
        Direction,
        DynChannelAccess,
        Error,
        Event,
        Level,
        LoopCount,
        MemSize,
        NUM_CHANNELS,
        RmtState,
        Rx,
        Tx,
    };
    use crate::{peripherals::RMT, time::Rate};

    pub(super) fn configure_clock(source: ClockSource, frequency: Rate) -> Result<(), Error> {
        if frequency != source.freq() {
            return Err(Error::UnreachableTargetFrequency);
        }

        let rmt = RMT::regs();

        for ch_num in 0..NUM_CHANNELS {
            rmt.chconf1(ch_num)
                .modify(|_, w| w.ref_always_on().bit(source.bit()));
        }

        rmt.apb_conf().modify(|_, w| w.apb_fifo_mask().set_bit());

        #[cfg(not(esp32))]
        rmt.apb_conf().modify(|_, w| w.clk_en().set_bit());

        Ok(())
    }

    #[allow(unused)]
    #[inline]
    pub(super) fn handle_channel_interrupts(
        on_tx: fn(DynChannelAccess<Tx>),
        on_rx: fn(DynChannelAccess<Rx>),
    ) {
        let st = RMT::regs().int_st().read();

        for ch_idx in ChannelIndex::iter_all() {
            if st.ch_rx_end(ch_idx as u8).bit() {
                let raw = unsafe { DynChannelAccess::<Rx>::conjure(ch_idx) };
                on_rx(raw);
                return;
            }
            if st.ch_tx_end(ch_idx as u8).bit() {
                let raw = unsafe { DynChannelAccess::<Tx>::conjure(ch_idx) };
                on_tx(raw);
                return;
            }
            if st.ch_err(ch_idx as u8).bit() {
                // SAFETY: channel number == ch_idx for these chips, so the argument is correct and
                // in range.
                let state =
                    unsafe { RmtState::load_by_channel_number(ch_idx as u8, Ordering::Relaxed) };
                if let Some(is_tx) = state.is_tx() {
                    if is_tx {
                        let raw = unsafe { DynChannelAccess::<Tx>::conjure(ch_idx) };
                        on_tx(raw);
                    } else {
                        let raw = unsafe { DynChannelAccess::<Rx>::conjure(ch_idx) };
                        on_rx(raw);
                    }
                    return;
                }
            }
        }
    }

    impl<Dir: Direction> DynChannelAccess<Dir> {
        #[inline]
        pub fn channel(&self) -> u8 {
            self.ch_idx as u8
        }

        #[inline]
        pub fn update(&self) {
            // no-op
        }

        pub fn set_divider(&self, divider: u8) {
            let rmt = crate::peripherals::RMT::regs();
            rmt.chconf0(self.ch_idx as usize)
                .modify(|_, w| unsafe { w.div_cnt().bits(divider) });
        }

        #[inline]
        pub fn memsize(&self) -> MemSize {
            let rmt = crate::peripherals::RMT::regs();
            let blocks = rmt.chconf0(self.ch_idx as usize).read().mem_size().bits();
            MemSize::from_blocks(blocks)
        }

        pub fn set_memsize(&self, value: MemSize) {
            let rmt = crate::peripherals::RMT::regs();
            rmt.chconf0(self.ch_idx as usize)
                .modify(|_, w| unsafe { w.mem_size().bits(value.blocks()) });
        }
    }

    impl DynChannelAccess<Tx> {
        #[cfg(rmt_has_tx_loop_count)]
        #[inline]
        pub fn set_generate_repeat_interrupt(&self, loopcount: Option<LoopCount>) {
            let rmt = crate::peripherals::RMT::regs();

            let repeats = match loopcount {
                Some(LoopCount::Finite(repeats)) if repeats.get() > 1 => repeats.get(),
                _ => 0,
            };

            rmt.ch_tx_lim(self.ch_idx as usize)
                .modify(|_, w| unsafe { w.tx_loop_num().bits(repeats) });
        }

        #[cfg(not(rmt_has_tx_loop_count))]
        #[inline]
        pub fn set_generate_repeat_interrupt(&self, _loopcount: Option<LoopCount>) {
            // unsupported
        }

        #[inline]
        pub fn clear_tx_interrupts(&self) {
            let rmt = crate::peripherals::RMT::regs();
            let ch = self.ch_idx as u8;

            rmt.int_clr().write(|w| {
                w.ch_err(ch).set_bit();
                w.ch_tx_end(ch).set_bit();
                w.ch_tx_thr_event(ch).set_bit()
            });
        }

        #[inline]
        pub fn set_tx_continuous(&self, continuous: bool) {
            let rmt = crate::peripherals::RMT::regs();

            rmt.chconf1(self.ch_idx as usize)
                .modify(|_, w| w.tx_conti_mode().bit(continuous));
        }

        #[inline]
        pub fn set_tx_wrap_mode(&self, wrap: bool) {
            let rmt = crate::peripherals::RMT::regs();
            // this is "okay", because we use all TX channels always in wrap mode
            rmt.apb_conf().modify(|_, w| w.mem_tx_wrap_en().bit(wrap));
        }

        pub fn set_tx_carrier(&self, carrier: bool, high: u16, low: u16, level: Level) {
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

        pub fn set_tx_idle_output(&self, enable: bool, level: Level) {
            let rmt = crate::peripherals::RMT::regs();
            rmt.chconf1(self.ch_idx as usize)
                .modify(|_, w| w.idle_out_en().bit(enable).idle_out_lv().bit(level.into()));
        }

        #[inline]
        pub fn start_tx(&self) {
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
        // Event::Error, Event::End, Event::Threshold
        #[inline]
        pub fn get_tx_status(&self) -> Option<Event> {
            let rmt = crate::peripherals::RMT::regs();
            let reg = rmt.int_raw().read();
            let ch = self.ch_idx as u8;

            if reg.ch_tx_end(ch).bit() {
                Some(Event::End)
            } else if reg.ch_err(ch).bit() {
                Some(Event::Error)
            } else if reg.ch_tx_thr_event(ch).bit() {
                Some(Event::Threshold)
            } else {
                None
            }
        }

        #[inline]
        pub fn reset_tx_threshold_set(&self) {
            let rmt = crate::peripherals::RMT::regs();
            rmt.int_clr()
                .write(|w| w.ch_tx_thr_event(self.ch_idx as u8).set_bit());
        }

        #[inline]
        pub fn set_tx_threshold(&self, threshold: u8) {
            let rmt = crate::peripherals::RMT::regs();
            rmt.ch_tx_lim(self.ch_idx as usize)
                .modify(|_, w| unsafe { w.tx_lim().bits(threshold as u16) });
        }

        #[inline]
        pub fn is_tx_loopcount_interrupt_set(&self) -> bool {
            // no-op
            false
        }

        // Returns whether stopping was immediate, or needs to wait for tx end
        // Due to inlining, the compiler should be able to eliminate code in the caller that
        // depends on this.
        #[cfg(rmt_has_tx_immediate_stop)]
        #[inline]
        pub fn stop_tx(&self) -> bool {
            let rmt = crate::peripherals::RMT::regs();
            rmt.chconf1(self.ch_idx as usize)
                .modify(|_, w| w.tx_stop().set_bit());
            true
        }

        #[cfg(not(rmt_has_tx_immediate_stop))]
        #[inline]
        pub fn stop_tx(&self) -> bool {
            let ptr = self.channel_ram_start();
            for idx in 0..self.memsize().codes() {
                unsafe {
                    ptr.add(idx).write_volatile(super::PulseCode::end_marker());
                }
            }
            false
        }

        #[inline]
        pub fn set_tx_interrupt(&self, events: EnumSet<Event>, enable: bool) {
            let rmt = crate::peripherals::RMT::regs();
            let ch = self.ch_idx as u8;
            rmt.int_ena().modify(|_, w| {
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
            });
        }
    }

    impl DynChannelAccess<Rx> {
        #[inline]
        pub fn clear_rx_interrupts(&self) {
            let rmt = crate::peripherals::RMT::regs();
            let ch = self.ch_idx as u8;

            rmt.int_clr().write(|w| {
                w.ch_rx_end(ch).set_bit();
                w.ch_err(ch).set_bit()
            });
        }

        #[inline]
        pub fn set_rx_wrap_mode(&self, _wrap: bool) {
            // no-op
        }

        pub fn set_rx_carrier(&self, carrier: bool, high: u16, low: u16, level: Level) {
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

        #[inline]
        pub fn start_rx(&self) {
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
        #[inline]
        pub fn get_rx_status(&self) -> Option<Event> {
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

        #[inline]
        pub fn stop_rx(&self) {
            let rmt = crate::peripherals::RMT::regs();
            rmt.chconf1(self.ch_idx as usize)
                .modify(|_, w| w.rx_en().clear_bit());
        }

        pub fn set_rx_filter_threshold(&self, value: u8) {
            let rmt = crate::peripherals::RMT::regs();
            rmt.chconf1(self.ch_idx as usize).modify(|_, w| unsafe {
                w.rx_filter_en().bit(value > 0);
                w.rx_filter_thres().bits(value)
            });
        }

        pub fn set_rx_idle_threshold(&self, value: u16) {
            let rmt = crate::peripherals::RMT::regs();
            rmt.chconf0(self.ch_idx as usize)
                .modify(|_, w| unsafe { w.idle_thres().bits(value) });
        }

        #[inline]
        pub fn set_rx_interrupt(&self, events: EnumSet<Event>, enable: bool) {
            let rmt = crate::peripherals::RMT::regs();
            let ch = self.ch_idx as u8;
            rmt.int_ena().modify(|_, w| {
                if events.contains(Event::Error) {
                    w.ch_err(ch).bit(enable);
                }
                if events.contains(Event::End) {
                    w.ch_rx_end(ch).bit(enable);
                }
                w
            });
        }
    }
}
