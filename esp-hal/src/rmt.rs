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
    },
    "rx_size_limit" => {
        cfg(any(esp32, esp32s2)) => "The length of the received data cannot exceed the allocated RMT RAM.",
        _ => ""
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
//!         Ok((symbol_count, channel_res)) => {
//!             channel = channel_res;
//!             let mut total = 0usize;
//!             for entry in &data[..symbol_count] {
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
//!             for entry in &data[..symbol_count] {
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
    mem::ManuallyDrop,
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
    peripherals::{Interrupt, RMT},
    system::{self, GenericPeripheralGuard},
    time::Rate,
};

mod reader;
use reader::{ReaderState, RmtReader};
mod writer;
use writer::{RmtWriter, WriterState};

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
pub trait Direction: Copy + Clone + core::fmt::Debug + crate::private::Sealed + Unpin {
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
    /// Idle threshold in ticks, must not exceed [`MAX_RX_IDLE_THRESHOLD`]
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
            pub struct Rmt<'rmt, Dm>
            where
                Dm: crate::DriverMode,
            {
                peripheral: RMT<'rmt>,
                $(
                    #[doc = concat!("RMT Channel ", $num)]
                    pub [<channel $num>]: ChannelCreator<'rmt, Dm, $num>,
                )+
                _mode: PhantomData<Dm>,
            }

            impl<'rmt, Dm> Rmt<'rmt, Dm>
            where
                Dm: crate::DriverMode,
            {
                fn create(peripheral: RMT<'rmt>) -> Self {
                    Self {
                        peripheral,
                        $(
                            [<channel $num>]: ChannelCreator::conjure(),
                        )+
                        _mode: PhantomData,
                    }
                }
            }

            impl<'rmt> Rmt<'rmt, Blocking> {
                /// Reconfigures the driver for asynchronous operation.
                pub fn into_async(mut self) -> Rmt<'rmt, Async> {
                    self.set_interrupt_handler(chip_specific::async_interrupt_handler);

                    Rmt {
                        peripheral: self.peripheral,
                        $(
                            [<channel $num>]: unsafe { self.[<channel $num>].into_async() },
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

            impl ChannelIndex {
                #[allow(clippy::no_effect)]
                const MAX: u8 = const { 0 $( + {$idx; 1} )+ };
            }

            const OUTPUT_SIGNALS: [gpio::OutputSignal; ChannelIndex::MAX as usize] = [
                $(
                    gpio::OutputSignal::[<RMT_SIG_ $idx>],
                )+
            ];

            $(
                impl<'ch, Dm> TxChannelCreator<'ch, Dm> for ChannelCreator<'ch, Dm, $num>
                where
                    Dm: crate::DriverMode,
                {
                    fn configure_tx<P>(
                        self,
                        pin: P,
                        config: TxChannelConfig,
                    ) -> Result<Channel<'ch, Dm, Tx>, (Error, Self, P)>
                    where
                        Self: Sized,
                        P: PeripheralOutput<'ch>,
                    {
                        let raw = unsafe { DynChannelAccess::conjure(ChannelIndex::[<Ch $idx>]) };

                        let memsize = MemSize::from_blocks(config.memsize);
                        if let Err(e) = reserve_channel(raw.channel(), RmtState::Tx, memsize) {
                            return Err((e, self, pin));
                        };

                        Ok(unsafe { Channel::configure_tx(
                            raw,
                            memsize,
                            pin.into(),
                            config,
                            self._guard,
                        ) })
                    }
                }
            )+
        }
    };

    (rx $(($num:literal, $idx:literal)),+) => {
        paste::paste! {
            const INPUT_SIGNALS: [gpio::InputSignal; ChannelIndex::MAX as usize] = [
                $(
                    gpio::InputSignal::[<RMT_SIG_ $idx>],
                )+
            ];

            $(
                impl<'ch, Dm> RxChannelCreator<'ch, Dm> for ChannelCreator<'ch, Dm, $num>
                where
                    Dm: crate::DriverMode,
                {
                    fn configure_rx<P>(
                        self,
                        pin: P,
                        config: RxChannelConfig,
                    ) -> Result<Channel<'ch, Dm, Rx>, (Error, Self, P)>
                    where
                        Self: Sized,
                        P: PeripheralInput<'ch>,
                    {
                        let raw = unsafe { DynChannelAccess::conjure(ChannelIndex::[<Ch $idx>]) };

                        #[cfg_attr(any(esp32, esp32s2), allow(clippy::absurd_extreme_comparisons))]
                        if config.idle_threshold > MAX_RX_IDLE_THRESHOLD {
                            return Err((Error::InvalidArgument, self, pin));
                        }

                        let memsize = MemSize::from_blocks(config.memsize);
                        if let Err(e) = reserve_channel(raw.channel(), RmtState::Rx, memsize) {
                            return Err((e, self, pin));
                        };

                        Ok(unsafe { Channel::configure_rx(
                            raw,
                            memsize,
                            pin.into(),
                            config,
                            self._guard,
                        ) })
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

impl<'rmt> Rmt<'rmt, Blocking> {
    /// Create a new RMT instance
    pub fn new(peripheral: RMT<'rmt>, frequency: Rate) -> Result<Self, Error> {
        let this = Rmt::create(peripheral);
        self::chip_specific::configure_clock(ClockSource::default(), frequency)?;
        Ok(this)
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
#[inline(never)]
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
pub struct Channel<'ch, Dm, Dir>
where
    Dm: crate::DriverMode,
    Dir: Direction,
{
    raw: DynChannelAccess<Dir>,

    // Holds the lifetime for which have unique access to both the channel and the pin its
    // configured for. Conceptually, for 'ch, we keep the Rmt peripheral alive.
    _rmt: PhantomData<Rmt<'ch, Dm>>,

    // Only the "outermost" Channel/ChannelCreator holds the GenericPeripheralGuard, which avoids
    // constant inc/dec of the reference count on reborrow and drop.
    _guard: DropState,
}

/// Per-channel size of the RMT hardware buffer (number of `PulseCode`s).
pub const CHANNEL_RAM_SIZE: usize = property!("rmt.channel_ram_size");

/// Whether the channel supports wrapping rx (wrapping tx is supported on all devices)
pub const HAS_RX_WRAP: bool = property!("rmt.has_rx_wrap");

impl<'ch, Dm> Channel<'ch, Dm, Tx>
where
    Dm: crate::DriverMode,
{
    #[inline(never)]
    unsafe fn configure_tx(
        raw: DynChannelAccess<Tx>,
        memsize: MemSize,
        pin: gpio::interconnect::OutputSignal<'ch>,
        config: TxChannelConfig,
        guard: Option<GenericPeripheralGuard<{ system::Peripheral::Rmt as u8 }>>,
    ) -> Self {
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

        let _guard = match guard {
            Some(g) => DropState::MemAndGuard(g),
            None => DropState::MemoryOnly,
        };

        Self {
            raw,
            _rmt: core::marker::PhantomData,
            _guard,
        }
    }
}

impl<'ch, Dm> Channel<'ch, Dm, Rx>
where
    Dm: crate::DriverMode,
{
    #[inline(never)]
    unsafe fn configure_rx(
        raw: DynChannelAccess<Rx>,
        memsize: MemSize,
        pin: gpio::interconnect::InputSignal<'ch>,
        config: RxChannelConfig,
        guard: Option<GenericPeripheralGuard<{ system::Peripheral::Rmt as u8 }>>,
    ) -> Self {
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

        let _guard = match guard {
            Some(g) => DropState::MemAndGuard(g),
            None => DropState::MemoryOnly,
        };

        Self {
            raw,
            _rmt: core::marker::PhantomData,
            _guard,
        }
    }
}

#[derive(Debug)]
enum DropState {
    None,
    MemoryOnly,
    MemAndGuard(GenericPeripheralGuard<{ system::Peripheral::Rmt as u8 }>),
}

impl<Dm, Dir> Channel<'_, Dm, Dir>
where
    Dm: crate::DriverMode,
    Dir: Direction,
{
    /// Reborrow this channel for a shorter lifetime `'a`.
    pub fn reborrow<'a>(&'a mut self) -> Channel<'a, Dm, Dir> {
        Channel {
            raw: self.raw,
            _rmt: self._rmt,
            // Resources must only be released once the parent is dropped.
            _guard: DropState::None,
        }
    }
}

impl<Dm, Dir> Drop for Channel<'_, Dm, Dir>
where
    Dm: crate::DriverMode,
    Dir: Direction,
{
    fn drop(&mut self) {
        if !matches!(self._guard, DropState::None) {
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
}

/// Creates a TX channel
pub trait TxChannelCreator<'ch, Dm>
where
    Dm: crate::DriverMode,
{
    /// Configure the TX channel
    fn configure_tx<P>(
        self,
        pin: P,
        config: TxChannelConfig,
    ) -> Result<Channel<'ch, Dm, Tx>, (Error, Self, P)>
    where
        Self: Sized,
        P: PeripheralOutput<'ch>;
}

/// Creates a RX channel
pub trait RxChannelCreator<'ch, Dm>
where
    Dm: crate::DriverMode,
{
    /// Configure the RX channel
    fn configure_rx<P>(
        self,
        pin: P,
        config: RxChannelConfig,
    ) -> Result<Channel<'ch, Dm, Rx>, (Error, Self, P)>
    where
        Self: Sized,
        P: PeripheralInput<'ch>;
}

/// An in-progress transaction for a single shot TX transaction.
///
/// If the data size exceeds the size of the internal buffer, `.poll()` or
/// `.wait()` needs to be called before the entire buffer has been sent to avoid
/// underruns.
#[must_use = "transactions need to be `poll()`ed / `wait()`ed for to ensure progress"]
#[derive(Debug)]
pub struct TxTransaction<'ch, 'data, T>
where
    T: Into<PulseCode> + Copy,
{
    channel: Channel<'ch, Blocking, Tx>,

    writer: RmtWriter,

    // Remaining data that has not yet been written to channel RAM. May be empty.
    remaining_data: &'data [T],
}

impl<'ch, T> TxTransaction<'ch, '_, T>
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
        matches!(self.poll_internal(), Some(Event::Error | Event::End))
    }

    /// Wait for the transaction to complete
    #[cfg_attr(place_rmt_driver_in_ram, inline(always))]
    pub fn wait(
        mut self,
    ) -> Result<Channel<'ch, Blocking, Tx>, (Error, Channel<'ch, Blocking, Tx>)> {
        let raw = self.channel.raw;

        // Not sure that all the error cases below can happen. However, it's best to
        // handle them to be sure that we don't lock up here in case they can happen.
        let result = loop {
            match self.poll_internal() {
                Some(Event::Error) => break Err(Error::TransmissionError),
                Some(Event::End) => {
                    if !self.remaining_data.is_empty() {
                        // Unexpectedly done, even though we have data left: For example, this could
                        // happen if there is a stop code inside the data and not just at the end.
                        break Err(Error::TransmissionError);
                    } else {
                        break Ok(());
                    }
                }
                _ => continue,
            }
        };

        // Disable Drop handler since the transaction is stopped already.
        let this = ManuallyDrop::new(self);
        // Rust has no safe API to take values out of ManuallyDrop,
        // cf. https://github.com/rust-lang/rfcs/pull/3466
        // This is safe since we own `this`, and don't access it below.
        let channel = unsafe { core::ptr::read(&this.channel) };

        raw.clear_tx_interrupts();

        match result {
            Ok(()) => Ok(channel),
            Err(err) => Err((err, channel)),
        }
    }
}

impl<T> Drop for TxTransaction<'_, '_, T>
where
    T: Into<PulseCode> + Copy,
{
    fn drop(&mut self) {
        let raw = self.channel.raw;

        if !matches!(raw.get_tx_status(), Some(Event::Error | Event::End)) {
            let immediate = raw.stop_tx();
            raw.update();

            // Block until the channel is safe to use again.
            if !immediate {
                while !matches!(raw.get_tx_status(), Some(Event::Error | Event::End)) {}
            }
        }

        raw.clear_tx_interrupts();
    }
}

/// An in-progress continuous TX transaction
#[must_use = "transactions will be aborted when dropped"]
#[derive(Debug)]
pub struct ContinuousTxTransaction<'ch> {
    channel: Channel<'ch, Blocking, Tx>,
    is_running: bool,
}

impl<'ch> ContinuousTxTransaction<'ch> {
    // FIXME: This interface isn't great, since one cannot use the waiting time until tx is stopped
    // for other things! Implement a poll-like interface similar to TxTransaction!
    /// Stop transaction when the current iteration ends.
    #[cfg_attr(place_rmt_driver_in_ram, inline(always))]
    pub fn stop_next(
        self,
    ) -> Result<Channel<'ch, Blocking, Tx>, (Error, Channel<'ch, Blocking, Tx>)> {
        self.stop_impl(false)
    }

    /// Stop transaction as soon as possible.
    #[cfg_attr(place_rmt_driver_in_ram, inline(always))]
    pub fn stop(self) -> Result<Channel<'ch, Blocking, Tx>, (Error, Channel<'ch, Blocking, Tx>)> {
        self.stop_impl(true)
    }

    #[cfg_attr(place_rmt_driver_in_ram, ram)]
    fn stop_impl(
        self,
        immediate: bool,
    ) -> Result<Channel<'ch, Blocking, Tx>, (Error, Channel<'ch, Blocking, Tx>)> {
        let raw = self.channel.raw;

        let mut result = Ok(());
        if self.is_running {
            // If rmt_has_tx_loop_auto_stop and the engine is stopped already, this is not
            // necessary. However, explicitly stopping unconditionally makes the logic
            // here much simpler and shouldn't create much overhead.
            raw.set_tx_continuous(false);
            let needs_wait = if immediate { !raw.stop_tx() } else { true };
            raw.update();

            if needs_wait {
                loop {
                    match raw.get_tx_status() {
                        Some(Event::Error) => {
                            result = Err(Error::TransmissionError);
                            break;
                        }
                        Some(Event::End) => break,
                        Some(Event::LoopCount) if cfg!(rmt_has_tx_loop_auto_stop) => break,
                        _ => continue,
                    }
                }
            }

            raw.clear_tx_interrupts();
        }

        // Disable Drop handler since the transaction is stopped already.
        let this = ManuallyDrop::new(self);
        // Rust has no safe API to take values out of ManuallyDrop,
        // cf. https://github.com/rust-lang/rfcs/pull/3466
        // This is safe since we own `this`, and don't access it below.
        let channel = unsafe { core::ptr::read(&this.channel) };

        match result {
            Ok(()) => Ok(channel),
            Err(err) => Err((err, channel)),
        }
    }

    /// Check if the `loopcount` interrupt bit is set.
    ///
    /// Whether this implies that the transmission has stopped depends on the [`LoopMode`] value
    /// provided when starting it.
    #[cfg(rmt_has_tx_loop_count)]
    pub fn is_loopcount_interrupt_set(&self) -> bool {
        !self.is_running || self.channel.raw.is_tx_loopcount_interrupt_set()
    }
}

impl<'ch> Drop for ContinuousTxTransaction<'ch> {
    fn drop(&mut self) {
        let raw = self.channel.raw;

        if self.is_running && !matches!(raw.get_tx_status(), Some(Event::Error | Event::End)) {
            let immediate = raw.stop_tx();
            raw.update();

            // Block until the channel is safe to use again.
            if !immediate {
                while !matches!(raw.get_tx_status(), Some(Event::Error | Event::End)) {}
            }
        }

        raw.clear_tx_interrupts();
    }
}

/// RMT Channel Creator
#[derive(Debug)]
pub struct ChannelCreator<'ch, Dm, const CHANNEL: u8>
where
    Dm: crate::DriverMode,
{
    // Conceptually, this retains a reference to the main driver struct, even when the
    // `ChannelCreator` is taken out of it and the `Rmt` dropped. This prevents re-inititalizing
    // the `Rmt` and obtaining a duplicate `ChannelCreator`.
    _rmt: PhantomData<Rmt<'ch, Dm>>,

    // We need to keep the peripheral clocked since the following sequence of events is possible:
    //
    // ```
    // let cc = {
    //     let rmt = Rmt::new(peripheral, freq);  // 1
    //     rmt.channel0  // 2
    // };  // 3 (drop other ChannelCreators and Rmt.peripheral)
    // let ch0 = cc.configure_tx(pin, config).unwrap();  // 4 (create Channel._guard)
    // ch0.transmit(...);  // 5
    // ```
    //
    // If there was no _guard in ChannelCreator, the peripheral would be disabled in step 3, and
    // re-enabled in step 4, losing the clock configuration that was set in step 1.
    _guard: Option<GenericPeripheralGuard<{ crate::system::Peripheral::Rmt as u8 }>>,
}

impl<'ch, Dm, const CHANNEL: u8> ChannelCreator<'ch, Dm, CHANNEL>
where
    Dm: crate::DriverMode,
{
    fn conjure() -> Self {
        Self {
            _rmt: PhantomData,
            _guard: Some(GenericPeripheralGuard::new()),
        }
    }

    unsafe fn into_async(self) -> ChannelCreator<'ch, Async, CHANNEL> {
        ChannelCreator {
            _rmt: PhantomData,
            _guard: self._guard,
        }
    }

    /// Reborrow this channel creator for a shorter lifetime `'a`.
    pub fn reborrow<'a>(&'a mut self) -> ChannelCreator<'a, Dm, CHANNEL> {
        Self {
            _rmt: PhantomData,
            _guard: None,
        }
    }

    /// Unsafely steal a channel creator instance.
    ///
    /// # Safety
    ///
    /// Circumvents HAL ownership and safety guarantees and allows creating
    /// multiple handles to the same peripheral structure.
    #[inline]
    pub unsafe fn steal() -> Self {
        Self {
            _rmt: PhantomData,
            _guard: Some(GenericPeripheralGuard::new()),
        }
    }
}

/// Loop mode for continuous transmission
///
/// Depending on hardware support, the `loopcount` interrupt and automatic stopping of the
/// transmission upon reaching a specified loop count may not be available.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum LoopMode {
    /// Repeat until explicitly stopped.
    Infinite,

    // FIXME: Does continuous tx trigger the End interrupt on each repetition such that it could
    // be used to emulate the LoopCount interrupt on devices that lack it?
    /// Repeat until explicitly stopped, and assert the loop count interrupt upon completing the
    /// given number of iterations.
    ///
    /// Loop counts larger than [`MAX_TX_LOOPCOUNT`] will result in an error.
    #[cfg(rmt_has_tx_loop_count)]
    InfiniteWithInterrupt(u16),

    /// Repeat for the given number of iterations, and also set the loop count interrupt flag upon
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

/// Channel in TX mode
impl<'ch> Channel<'ch, Blocking, Tx> {
    /// Start transmitting the given pulse code sequence.
    /// This returns a [`TxTransaction`] which can be used to wait for
    /// the transaction to complete and get back the channel for further
    /// use.
    #[cfg_attr(place_rmt_driver_in_ram, ram)]
    pub fn transmit<'data, T>(
        self,
        mut data: &'data [T],
    ) -> Result<TxTransaction<'ch, 'data, T>, (Error, Self)>
    where
        T: Into<PulseCode> + Copy,
    {
        let raw = self.raw;
        let memsize = raw.memsize();

        match data.last() {
            None => return Err((Error::InvalidArgument, self)),
            Some(&code) if code.into().is_end_marker() => (),
            Some(_) => return Err((Error::EndMarkerMissing, self)),
        }

        let mut writer = RmtWriter::new();
        writer.write(&mut data, raw, true);

        raw.clear_tx_interrupts();
        raw.start_send(None, memsize);

        Ok(TxTransaction {
            channel: self,
            writer,
            remaining_data: data,
        })
    }

    /// Start transmitting the given pulse code continuously.
    ///
    /// This returns a [`ContinuousTxTransaction`] which can be used to stop the
    /// ongoing transmission and get back the channel for further use.
    ///
    /// The `mode` argument determines whether transmission will continue until explicitly stopped
    /// or for a fixed number of iterations; see [`LoopMode`] for more details.
    #[cfg_attr(
        rmt_has_tx_loop_count,
        doc = "When using a loop `mode` other than [`LoopMode::Infinite`], [`ContinuousTxTransaction::is_loopcount_interrupt_set`] can be used to check if the loop count is reached."
    )]
    /// The length of `data` cannot exceed the size of the allocated RMT RAM.
    #[cfg_attr(place_rmt_driver_in_ram, ram)]
    pub fn transmit_continuously<T>(
        self,
        mut data: &[T],
        mode: LoopMode,
    ) -> Result<ContinuousTxTransaction<'ch>, (Error, Self)>
    where
        T: Into<PulseCode> + Copy,
    {
        let raw = self.raw;
        let memsize = raw.memsize();

        #[cfg(rmt_has_tx_loop_count)]
        if mode.get_count() > MAX_TX_LOOPCOUNT {
            return Err((Error::InvalidArgument, self));
        }

        if data.is_empty() {
            return Err((Error::InvalidArgument, self));
        } else if data.len() > memsize.codes() {
            return Err((Error::Overflow, self));
        }

        // We need a separate flag to track whether we actually started the transmitter: The
        // ContinuousTxTransaction looks at the interrupt status to determine this (and would lock
        // up if it's not running, and will never set them), but unfortunately, it is not possible
        // (according to the TRM) to manually set the raw interrupt flags here, e.g. do something
        // like `raw.set_tx_status(Event::End, Event::LoopCount)`.
        #[cfg(rmt_has_tx_loop_auto_stop)]
        let is_running = mode != LoopMode::Finite(0);
        #[cfg(not(rmt_has_tx_loop_auto_stop))]
        let is_running = true;

        if is_running {
            let mut writer = RmtWriter::new();
            writer.write(&mut data, raw, true);

            raw.clear_tx_interrupts();
            raw.start_send(Some(mode), memsize);
        }

        Ok(ContinuousTxTransaction {
            channel: self,
            is_running,
        })
    }
}

/// RX transaction instance
#[must_use = "transactions need to be `poll()`ed / `wait()`ed for to ensure progress"]
#[derive(Debug)]
pub struct RxTransaction<'ch, 'data, T>
where
    T: From<PulseCode>,
{
    channel: Channel<'ch, Blocking, Rx>,

    reader: RmtReader,

    data: &'data mut [T],
}

impl<'ch, T> RxTransaction<'ch, '_, T>
where
    T: From<PulseCode>,
{
    #[cfg_attr(place_rmt_driver_in_ram, ram)]
    fn poll_internal(&mut self) -> Option<Event> {
        let raw = self.channel.raw;

        let status = raw.get_rx_status();

        match status {
            // Read all available data also on error
            Some(Event::End | Event::Error) => {
                // Do not clear the interrupt flags here: Subsequent calls of wait() must
                // be able to observe them if this is currently called via poll()
                // FIXME: rx should be stopped here, attempt removing this
                raw.stop_rx(false);
                raw.update();

                // `RmtReader::read()` is safe to call even if `poll_internal` is called repeatedly
                // after the receiver finished since it returns immediately if already done.
                self.reader.read(&mut self.data, raw, true);
            }
            #[cfg(rmt_has_rx_wrap)]
            Some(Event::Threshold) => {
                raw.reset_rx_threshold_set();

                if self.reader.state == ReaderState::Active {
                    self.reader.read(&mut self.data, raw, false);
                }
            }
            _ => (),
        }

        status
    }

    /// Check receive status
    ///
    /// Returns whether reception has ended (whether successfully or with an
    /// error). In that case, a subsequent call to `wait()` returns immediately.
    #[cfg_attr(place_rmt_driver_in_ram, inline(always))]
    pub fn poll(&mut self) -> bool {
        matches!(self.poll_internal(), Some(Event::Error | Event::End))
    }

    /// Wait for the transaction to complete
    #[cfg_attr(place_rmt_driver_in_ram, inline(always))]
    // The return type isn't nice, but the blocking API needs a broader redesign anyway.
    #[allow(clippy::type_complexity)]
    pub fn wait(
        mut self,
    ) -> Result<(usize, Channel<'ch, Blocking, Rx>), (Error, Channel<'ch, Blocking, Rx>)> {
        let raw = self.channel.raw;

        let result = loop {
            match self.poll_internal() {
                Some(Event::Error) => break Err(Error::ReceiverError),
                Some(Event::End) => break Ok(self.reader.total),
                _ => continue,
            }
        };

        // Disable Drop handler since the transaction is stopped already.
        let this = ManuallyDrop::new(self);
        // Rust has no safe API to take values out of ManuallyDrop,
        // cf. https://github.com/rust-lang/rfcs/pull/3466
        // This is safe since we own `this`, and don't access it below.
        let channel = unsafe { core::ptr::read(&this.channel) };

        raw.clear_rx_interrupts();

        match result {
            Ok(total) => Ok((total, channel)),
            Err(err) => Err((err, channel)),
        }
    }
}

impl<T> Drop for RxTransaction<'_, '_, T>
where
    T: From<PulseCode>,
{
    fn drop(&mut self) {
        let raw = self.channel.raw;

        raw.stop_rx(true);
        raw.update();

        raw.clear_rx_interrupts();
    }
}

/// Channel is RX mode
impl<'ch> Channel<'ch, Blocking, Rx> {
    /// Start receiving pulse codes into the given buffer.
    /// This returns a [RxTransaction] which can be used to wait for receive to
    /// complete and get back the channel for further use.
    ///
    /// # {rx_size_limit}
    #[cfg_attr(place_rmt_driver_in_ram, ram)]
    pub fn receive<'data, T>(
        self,
        data: &'data mut [T],
    ) -> Result<RxTransaction<'ch, 'data, T>, (Error, Self)>
    where
        Self: Sized,
        T: From<PulseCode>,
    {
        let raw = self.raw;
        let memsize = raw.memsize();

        if !property!("rmt.has_rx_wrap") && data.len() > memsize.codes() {
            return Err((Error::InvalidDataLength, self));
        }

        let reader = RmtReader::new();

        raw.clear_rx_interrupts();
        raw.start_receive(true, memsize);

        Ok(RxTransaction {
            channel: self,
            reader,
            data,
        })
    }
}

static WAKER: [AtomicWaker; NUM_CHANNELS] = [const { AtomicWaker::new() }; NUM_CHANNELS];

#[must_use = "futures do nothing unless you `.await` or poll them"]
struct TxFuture<'a, T>
where
    T: Into<PulseCode> + Copy,
{
    raw: DynChannelAccess<Tx>,
    _phantom: PhantomData<Channel<'a, Async, Tx>>,
    writer: RmtWriter,

    // Remaining data that has not yet been written to channel RAM. May be empty.
    data: &'a [T],
}

impl<T> core::future::Future for TxFuture<'_, T>
where
    T: Into<PulseCode> + Copy,
{
    type Output = Result<(), Error>;

    #[cfg_attr(place_rmt_driver_in_ram, ram)]
    fn poll(self: Pin<&mut Self>, ctx: &mut Context<'_>) -> Poll<Self::Output> {
        let this = self.get_mut();
        let raw = this.raw;

        if let WriterState::Error(err) = this.writer.state {
            return Poll::Ready(Err(err));
        }

        WAKER[raw.channel() as usize].register(ctx.waker());

        let result = match raw.get_tx_status() {
            Some(Event::Error) => Err(Error::TransmissionError),
            Some(Event::End) => {
                if this.writer.state == WriterState::Active {
                    // Unexpectedly done, even though we have data left.
                    Err(Error::TransmissionError)
                } else {
                    Ok(())
                }
            }
            Some(Event::Threshold) => {
                raw.reset_tx_threshold_set();

                this.writer.write(&mut this.data, raw, false);

                if this.writer.state == WriterState::Active {
                    raw.listen_tx_interrupt(Event::Threshold);
                }

                return Poll::Pending;
            }
            _ => return Poll::Pending,
        };

        Poll::Ready(result)
    }
}

impl<T> Drop for TxFuture<'_, T>
where
    T: Into<PulseCode> + Copy,
{
    fn drop(&mut self) {
        let raw = self.raw;

        if !matches!(raw.get_tx_status(), Some(Event::Error | Event::End)) {
            let immediate = raw.stop_tx();
            raw.update();

            // Block until the channel is safe to use again.
            if !immediate {
                while !matches!(raw.get_tx_status(), Some(Event::Error | Event::End)) {}
            }
        }

        raw.clear_tx_interrupts();
    }
}

/// TX channel in async mode
impl Channel<'_, Async, Tx> {
    /// Start transmitting the given pulse code sequence.
    /// The length of sequence cannot exceed the size of the allocated RMT
    /// RAM.
    #[cfg_attr(place_rmt_driver_in_ram, ram)]
    pub fn transmit<T>(&mut self, mut data: &[T]) -> impl Future<Output = Result<(), Error>>
    where
        Self: Sized,
        T: Into<PulseCode> + Copy,
    {
        let raw = self.raw;
        let memsize = raw.memsize();

        let mut writer = RmtWriter::new();

        match data.last() {
            None => {
                writer.state = WriterState::Error(Error::InvalidArgument);
            }
            Some(&code) if code.into().is_end_marker() => (),
            Some(_) => {
                writer.state = WriterState::Error(Error::EndMarkerMissing);
            }
        }

        if !matches!(writer.state, WriterState::Error(_)) {
            writer.write(&mut data, raw, true);

            let wrap = match writer.state {
                WriterState::Error(_) => false,
                WriterState::Active => true,
                WriterState::Done => false,
            };

            raw.clear_tx_interrupts();
            let mut events = Event::End | Event::Error;
            if wrap {
                events |= Event::Threshold;
            }
            raw.listen_tx_interrupt(events);
            raw.start_send(None, memsize);
        }

        TxFuture {
            raw,
            _phantom: PhantomData,
            writer,
            data,
        }
    }
}

#[must_use = "futures do nothing unless you `.await` or poll them"]
struct RxFuture<'a, T>
where
    T: From<PulseCode> + Unpin,
{
    raw: DynChannelAccess<Rx>,
    _phantom: PhantomData<Channel<'a, Async, Rx>>,
    reader: RmtReader,
    data: &'a mut [T],
}

impl<T> core::future::Future for RxFuture<'_, T>
where
    T: From<PulseCode> + Unpin,
{
    type Output = Result<usize, Error>;

    #[cfg_attr(place_rmt_driver_in_ram, ram)]
    fn poll(self: Pin<&mut Self>, ctx: &mut Context<'_>) -> Poll<Self::Output> {
        let this = self.get_mut();
        let raw = this.raw;

        if let ReaderState::Error(err) = this.reader.state {
            return Poll::Ready(Err(err));
        }

        WAKER[raw.channel() as usize].register(ctx.waker());

        let result = match raw.get_rx_status() {
            // Read all available data also on error
            Some(ev @ (Event::End | Event::Error)) => {
                this.reader.read(&mut this.data, raw, true);

                match ev {
                    Event::Error => Err(Error::ReceiverError),
                    _ => Ok(this.reader.total),
                }
            }
            #[cfg(rmt_has_rx_wrap)]
            Some(Event::Threshold) => {
                raw.reset_rx_threshold_set();

                this.reader.read(&mut this.data, raw, false);

                if this.reader.state == ReaderState::Active {
                    raw.listen_rx_interrupt(Event::Threshold);
                }

                return Poll::Pending;
            }
            _ => return Poll::Pending,
        };

        Poll::Ready(result)
    }
}

impl<T> Drop for RxFuture<'_, T>
where
    T: From<PulseCode> + Unpin,
{
    fn drop(&mut self) {
        let raw = self.raw;

        raw.stop_rx(true);
        raw.update();

        raw.clear_rx_interrupts();
    }
}

/// RX channel in async mode
impl Channel<'_, Async, Rx> {
    /// Start receiving a pulse code sequence.
    ///
    /// # {rx_size_limit}
    #[cfg_attr(place_rmt_driver_in_ram, ram)]
    pub fn receive<T>(&mut self, data: &mut [T]) -> impl Future<Output = Result<usize, Error>>
    where
        Self: Sized,
        T: From<PulseCode> + Unpin,
    {
        let raw = self.raw;
        let memsize = raw.memsize();

        let mut reader = RmtReader::new();

        if !property!("rmt.has_rx_wrap") && data.len() > memsize.codes() {
            reader.state = ReaderState::Error(Error::InvalidDataLength);
        } else {
            raw.clear_rx_interrupts();
            raw.listen_rx_interrupt(Event::End | Event::Error | Event::Threshold);
            raw.start_receive(true, memsize);
        }

        RxFuture {
            raw,
            reader,
            data,
            _phantom: PhantomData,
        }
    }
}

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
            #[inline(always)]
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
            #[inline(always)]
            fn default() -> Self {
                Self::$name
            }

            #[inline(always)]
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

    use super::{
        ChannelIndex,
        ClockSource,
        Direction,
        DynChannelAccess,
        Error,
        Event,
        Level,
        LoopMode,
        MemSize,
        Rx,
        Tx,
        WAKER,
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
                cfg_if::cfg_if!(
                    if #[cfg(esp32c6)] {
                        w.sclk_sel().bits(source.bits())
                    } else {
                        w.sclk_sel().bit(source.bit())
                    }
                );
                w.sclk_div_num().bits(div);
                w.sclk_div_a().bits(0);
                w.sclk_div_b().bits(0)
            });

            RMT::regs()
                .sys_conf()
                .modify(|_, w| w.apb_fifo_mask().set_bit());
        }

        Ok(())
    }

    #[crate::handler]
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
        #[inline(always)]
        pub fn channel(self) -> u8 {
            if Dir::IS_TX {
                self.ch_idx as u8
            } else {
                self.ch_idx as u8 + ChannelIndex::MAX
            }
        }

        #[inline(always)]
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
        }

        #[inline(always)]
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

        #[inline(always)]
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

        #[inline(always)]
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
        pub fn clear_tx_interrupts(self) {
            let rmt = crate::peripherals::RMT::regs();

            rmt.int_clr().write(|w| {
                let ch_idx = self.ch_idx as u8;

                w.ch_tx_end(ch_idx).set_bit();
                w.ch_tx_err(ch_idx).set_bit();
                w.ch_tx_loop(ch_idx).set_bit();
                w.ch_tx_thr_event(ch_idx).set_bit()
            });
        }

        #[inline(always)]
        pub fn set_tx_continuous(self, continuous: bool) {
            let rmt = crate::peripherals::RMT::regs();
            let ch_idx = self.ch_idx as usize;

            rmt.ch_tx_conf0(ch_idx)
                .modify(|_, w| w.tx_conti_mode().bit(continuous));
        }

        #[inline(always)]
        pub fn set_tx_wrap_mode(self, wrap: bool) {
            let rmt = crate::peripherals::RMT::regs();
            let ch_idx = self.ch_idx as usize;

            rmt.ch_tx_conf0(ch_idx)
                .modify(|_, w| w.mem_tx_wrap_en().bit(wrap));
        }

        #[inline(always)]
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

        #[inline(always)]
        pub fn set_tx_idle_output(self, enable: bool, level: Level) {
            let rmt = crate::peripherals::RMT::regs();
            let ch_idx = self.ch_idx as usize;

            rmt.ch_tx_conf0(ch_idx)
                .modify(|_, w| w.idle_out_en().bit(enable).idle_out_lv().bit(level.into()));
        }

        #[inline(always)]
        pub fn start_tx(self) {
            let rmt = crate::peripherals::RMT::regs();
            let ch_idx = self.ch_idx as usize;

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

        #[inline(always)]
        pub fn reset_tx_threshold_set(self) {
            let rmt = crate::peripherals::RMT::regs();

            rmt.int_clr().write(|w| {
                let ch_idx = self.ch_idx as u8;

                w.ch_tx_thr_event(ch_idx).set_bit()
            });
        }

        #[inline(always)]
        pub fn set_tx_threshold(self, threshold: u8) {
            let rmt = crate::peripherals::RMT::regs();
            let ch_idx = self.ch_idx as usize;

            rmt.ch_tx_lim(ch_idx)
                .modify(|_, w| unsafe { w.tx_lim().bits(threshold as u16) });
        }

        #[inline(always)]
        pub fn is_tx_loopcount_interrupt_set(self) -> bool {
            let rmt = crate::peripherals::RMT::regs();
            let ch_idx = self.ch_idx as u8;

            rmt.int_raw().read().ch_tx_loop(ch_idx).bit()
        }

        // Returns whether stopping was immediate, or needs to wait for tx end.
        // Due to inlining, the compiler should be able to eliminate code in the caller that
        // depends on this.
        //
        // Requires an update() call
        #[inline(always)]
        pub fn stop_tx(self) -> bool {
            let rmt = crate::peripherals::RMT::regs();
            let ch_idx = self.ch_idx as usize;

            rmt.ch_tx_conf0(ch_idx).modify(|_, w| w.tx_stop().set_bit());
            true
        }

        #[inline(always)]
        pub fn set_tx_interrupt(self, events: EnumSet<Event>, enable: bool) {
            let rmt = crate::peripherals::RMT::regs();

            rmt.int_ena().modify(|_, w| {
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
            });
        }

        #[allow(unused)]
        #[inline(always)]
        pub fn hw_offset(self) -> usize {
            let rmt = crate::peripherals::RMT::regs();
            let ch_idx = self.ch_idx as usize;

            let offset = rmt.ch_tx_status(ch_idx).read().mem_raddr_ex().bits();
            usize::from(offset) - self.channel_ram_start_offset()
        }
    }

    // documented in re-export below
    #[allow(missing_docs)]
    pub const MAX_RX_IDLE_THRESHOLD: u16 =
        max_from_register_spec!(u16, ch_rx_conf0, CH_RX_CONF0_SPEC, IDLE_THRES_W);

    impl DynChannelAccess<Rx> {
        #[inline(always)]
        pub fn clear_rx_interrupts(self) {
            let rmt = crate::peripherals::RMT::regs();

            rmt.int_clr().write(|w| {
                let ch_idx = self.ch_idx as u8;

                w.ch_rx_end(ch_idx).set_bit();
                w.ch_rx_err(ch_idx).set_bit();
                w.ch_rx_thr_event(ch_idx).set_bit()
            });
        }

        #[inline(always)]
        pub fn set_rx_wrap_mode(self, wrap: bool) {
            let rmt = crate::peripherals::RMT::regs();
            let ch_idx = self.ch_idx as usize;

            rmt.ch_rx_conf1(ch_idx)
                .modify(|_, w| w.mem_rx_wrap_en().bit(wrap));
        }

        #[inline(always)]
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

        #[inline(always)]
        pub fn start_rx(self) {
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

        #[inline(always)]
        pub fn reset_rx_threshold_set(self) {
            let rmt = crate::peripherals::RMT::regs();

            rmt.int_clr().write(|w| {
                let ch_idx = self.ch_idx as u8;

                w.ch_rx_thr_event(ch_idx).set_bit()
            });
        }

        #[inline(always)]
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
        #[inline(always)]
        pub fn stop_rx(self, _force: bool) {
            let rmt = crate::peripherals::RMT::regs();
            let ch_idx = self.ch_idx as usize;

            rmt.ch_rx_conf1(ch_idx).modify(|_, w| w.rx_en().clear_bit());
        }

        #[inline(always)]
        pub fn set_rx_filter_threshold(self, value: u8) {
            let rmt = crate::peripherals::RMT::regs();
            let ch_idx = self.ch_idx as usize;

            rmt.ch_rx_conf1(ch_idx).modify(|_, w| unsafe {
                w.rx_filter_en().bit(value > 0);
                w.rx_filter_thres().bits(value)
            });
        }

        #[inline(always)]
        pub fn set_rx_idle_threshold(self, value: u16) {
            let rmt = crate::peripherals::RMT::regs();
            let ch_idx = self.ch_idx as usize;

            rmt.ch_rx_conf0(ch_idx)
                .modify(|_, w| unsafe { w.idle_thres().bits(value) });
        }

        #[inline(always)]
        pub fn set_rx_interrupt(self, events: EnumSet<Event>, enable: bool) {
            let rmt = crate::peripherals::RMT::regs();

            rmt.int_ena().modify(|_, w| {
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
            });
        }

        #[inline(always)]
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

    use super::{
        ChannelIndex,
        ClockSource,
        Direction,
        DynChannelAccess,
        Error,
        Event,
        Level,
        MemSize,
        NUM_CHANNELS,
        Rx,
        Tx,
        WAKER,
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

    #[crate::handler]
    pub(super) fn async_interrupt_handler() {
        let st = RMT::regs().int_st().read();

        for ch_idx in ChannelIndex::iter_all() {
            let raw_tx = unsafe { DynChannelAccess::<Tx>::conjure(ch_idx) };
            let raw_rx = unsafe { DynChannelAccess::<Rx>::conjure(ch_idx) };
            let ch_idx = ch_idx as u8;

            if st.ch_tx_end(ch_idx).bit() {
                raw_tx.unlisten_tx_interrupt(EnumSet::all());
            } else if st.ch_rx_end(ch_idx).bit() {
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
        #[inline(always)]
        pub fn channel(self) -> u8 {
            self.ch_idx as u8
        }

        #[inline(always)]
        pub fn update(self) {
            // no-op
        }

        #[inline(always)]
        pub fn set_divider(self, divider: u8) {
            let rmt = crate::peripherals::RMT::regs();
            let ch = self.ch_idx as usize;

            rmt.chconf0(ch)
                .modify(|_, w| unsafe { w.div_cnt().bits(divider) });
        }

        #[inline(always)]
        pub fn memsize(self) -> MemSize {
            let rmt = crate::peripherals::RMT::regs();
            let ch = self.ch_idx as usize;

            let blocks = rmt.chconf0(ch).read().mem_size().bits();
            MemSize::from_blocks(blocks)
        }

        #[inline(always)]
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
        pub fn clear_tx_interrupts(self) {
            let rmt = crate::peripherals::RMT::regs();

            rmt.int_clr().write(|w| {
                let ch = self.ch_idx as u8;

                w.ch_err(ch).set_bit();
                w.ch_tx_end(ch).set_bit();
                #[cfg(rmt_has_tx_loop_count)]
                w.ch_tx_loop(ch).set_bit();
                w.ch_tx_thr_event(ch).set_bit()
            });
        }

        #[inline(always)]
        pub fn set_tx_continuous(self, continuous: bool) {
            let rmt = crate::peripherals::RMT::regs();
            let ch = self.ch_idx as usize;

            rmt.chconf1(ch)
                .modify(|_, w| w.tx_conti_mode().bit(continuous));
        }

        #[inline(always)]
        pub fn set_tx_wrap_mode(self, wrap: bool) {
            let rmt = crate::peripherals::RMT::regs();

            // this is "okay", because we use all TX channels always in wrap mode
            rmt.apb_conf().modify(|_, w| w.mem_tx_wrap_en().bit(wrap));
        }

        #[inline(always)]
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

        #[inline(always)]
        pub fn set_tx_idle_output(self, enable: bool, level: Level) {
            let rmt = crate::peripherals::RMT::regs();
            let ch = self.ch_idx as usize;

            rmt.chconf1(ch)
                .modify(|_, w| w.idle_out_en().bit(enable).idle_out_lv().bit(level.into()));
        }

        #[inline(always)]
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

        #[inline(always)]
        pub fn reset_tx_threshold_set(self) {
            let rmt = crate::peripherals::RMT::regs();

            rmt.int_clr().write(|w| {
                let ch = self.ch_idx as u8;

                w.ch_tx_thr_event(ch).set_bit()
            });
        }

        #[inline(always)]
        pub fn set_tx_threshold(self, threshold: u8) {
            let rmt = crate::peripherals::RMT::regs();
            let ch = self.ch_idx as usize;

            rmt.ch_tx_lim(ch)
                .modify(|_, w| unsafe { w.tx_lim().bits(threshold as u16) });
        }

        #[cfg(rmt_has_tx_loop_count)]
        #[inline(always)]
        pub fn is_tx_loopcount_interrupt_set(self) -> bool {
            let rmt = crate::peripherals::RMT::regs();
            let ch = self.ch_idx as u8;

            rmt.int_raw().read().ch_tx_loop(ch).bit()
        }

        // It would be better to simply not define this method if not supported, but that would
        // make the implementation of ContinuousTxTransaction::stop_impl much more awkward.
        #[cfg(not(rmt_has_tx_loop_count))]
        #[allow(unused)]
        #[inline(always)]
        pub fn is_tx_loopcount_interrupt_set(self) -> bool {
            false
        }

        // Returns whether stopping was immediate, or needs to wait for tx end
        // Due to inlining, the compiler should be able to eliminate code in the caller that
        // depends on this.
        #[cfg(rmt_has_tx_immediate_stop)]
        #[inline(always)]
        pub fn stop_tx(self) -> bool {
            let rmt = crate::peripherals::RMT::regs();
            let ch = self.ch_idx as usize;

            rmt.chconf1(ch).modify(|_, w| w.tx_stop().set_bit());
            true
        }

        #[cfg(not(rmt_has_tx_immediate_stop))]
        #[inline(always)]
        pub fn stop_tx(self) -> bool {
            let ptr = self.channel_ram_start();
            for idx in 0..self.memsize().codes() {
                unsafe {
                    ptr.add(idx).write_volatile(super::PulseCode::end_marker());
                }
            }
            false
        }

        #[inline(always)]
        pub fn set_tx_interrupt(self, events: EnumSet<Event>, enable: bool) {
            let rmt = crate::peripherals::RMT::regs();

            rmt.int_ena().modify(|_, w| {
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
            });
        }

        #[allow(unused)]
        #[inline(always)]
        pub fn hw_offset(self) -> usize {
            let rmt = crate::peripherals::RMT::regs();
            let ch = self.ch_idx as usize;

            let offset = rmt.chstatus(ch).read().mem_raddr_ex().bits();
            usize::from(offset) - self.channel_ram_start_offset()
        }
    }

    // documented in re-export below
    #[allow(missing_docs)]
    pub const MAX_RX_IDLE_THRESHOLD: u16 =
        max_from_register_spec!(u16, chconf0, CHCONF0_SPEC, IDLE_THRES_W);

    impl DynChannelAccess<Rx> {
        #[inline(always)]
        pub fn clear_rx_interrupts(self) {
            let rmt = crate::peripherals::RMT::regs();

            rmt.int_clr().write(|w| {
                let ch = self.ch_idx as u8;

                w.ch_rx_end(ch).set_bit();
                w.ch_err(ch).set_bit()
            });
        }

        #[inline(always)]
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

        #[inline(always)]
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

        #[inline(always)]
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
            // 2) lowering the idle treshold and change other settings to make exceeding the
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

        #[inline(always)]
        pub fn set_rx_filter_threshold(self, value: u8) {
            let rmt = crate::peripherals::RMT::regs();
            let ch = self.ch_idx as usize;

            rmt.chconf1(ch).modify(|_, w| unsafe {
                w.rx_filter_en().bit(value > 0);
                w.rx_filter_thres().bits(value)
            });
        }

        #[inline(always)]
        pub fn set_rx_idle_threshold(self, value: u16) {
            let rmt = crate::peripherals::RMT::regs();
            let ch = self.ch_idx as usize;

            rmt.chconf0(ch)
                .modify(|_, w| unsafe { w.idle_thres().bits(value) });
        }

        #[inline(always)]
        pub fn set_rx_interrupt(self, events: EnumSet<Event>, enable: bool) {
            let rmt = crate::peripherals::RMT::regs();

            rmt.int_ena().modify(|_, w| {
                let ch = self.ch_idx as u8;

                if events.contains(Event::Error) {
                    w.ch_err(ch).bit(enable);
                }
                if events.contains(Event::End) {
                    w.ch_rx_end(ch).bit(enable);
                }
                w
            });
        }

        #[inline(always)]
        pub fn hw_offset(self) -> usize {
            let rmt = crate::peripherals::RMT::regs();
            let ch = self.ch_idx as usize;

            let offset = rmt.chstatus(ch).read().mem_waddr_ex().bits();
            usize::from(offset) - self.channel_ram_start_offset()
        }
    }
}

/// The largest valid value for [`RxChannelConfig::with_idle_threshold`].
pub use chip_specific::MAX_RX_IDLE_THRESHOLD;
/// The largest valid value for loopcounts in [`LoopMode`].
#[cfg(rmt_has_tx_loop_count)]
pub use chip_specific::MAX_TX_LOOPCOUNT;
