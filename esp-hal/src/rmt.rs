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
//! - Transmit signals in a hardware-controlled loop, with a finite or infinite
//!   number of times
//! - Modulate the carrier to the output signal or demodulate the carrier from
//!   the input signal
//!
//! ### Channels
//!
//! There are
#![cfg_attr(
    esp32,
    doc = "8 channels, each of them can be either receiver or transmitter."
)]
#![cfg_attr(
    esp32s2,
    doc = "4 channels, each of them can be either receiver or transmitter."
)]
#![cfg_attr(
    esp32s3,
    doc = "8 channels, `Channel<0>`-`Channel<3>` hardcoded for transmitting signals and `Channel<4>`-`Channel<7>` hardcoded for receiving signals."
)]
#![cfg_attr(
    any(esp32c3, esp32c6, esp32h2),
    doc = "4 channels, `Channel<0>` and `Channel<1>` hardcoded for transmitting signals and `Channel<2>` and `Channel<3>` hardcoded for receiving signals."
)]
#![doc = ""]
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
#![doc = crate::before_snippet!()]
//! # use esp_hal::gpio::Level;
//! # use esp_hal::peripherals::Peripherals;
//! # use esp_hal::rmt::TxChannelConfig;
//! # use esp_hal::rmt::Rmt;
//! # use crate::esp_hal::rmt::TxChannelCreator;
#![cfg_attr(esp32h2, doc = "let freq = Rate::from_mhz(32);")]
#![cfg_attr(not(esp32h2), doc = "let freq = Rate::from_mhz(80);")]
//! let rmt = Rmt::new(peripherals.RMT, freq)?;
//! let mut channel = rmt
//!     .channel0
//!     .configure_tx(
//!         peripherals.GPIO1,
//!         TxChannelConfig::default()
//!             .with_clk_divider(1)
//!             .with_idle_output_level(Level::Low)
//!             .with_idle_output(false)
//!             .with_carrier_modulation(false)
//!             .with_carrier_high(1)
//!             .with_carrier_low(1)
//!             .with_carrier_level(Level::Low),
//!     )?;
//! # Ok(())
//! # }
//! ```
//! 
//! ### TX operation
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! # use esp_hal::delay::Delay;
//! # use esp_hal::gpio::Level;
//! # use esp_hal::rmt::{PulseCode, Rmt, TxChannel, TxChannelConfig, TxChannelCreator};
//!
//! // Configure frequency based on chip type
#![cfg_attr(esp32h2, doc = "let freq = Rate::from_mhz(32);")]
#![cfg_attr(not(esp32h2), doc = "let freq = Rate::from_mhz(80);")]
//! let rmt = Rmt::new(peripherals.RMT, freq)?;
//!
//! let tx_config = TxChannelConfig::default().with_clk_divider(255);
//!
//! let mut channel = rmt
//!     .channel0
//!     .configure_tx(peripherals.GPIO4, tx_config)?;
//!
//! let delay = Delay::new();
//!
//! let mut data = [PulseCode::new(Level::High, 200, Level::Low, 50); 20];
//! data[data.len() - 2] = PulseCode::new(Level::High, 3000, Level::Low, 500);
//! data[data.len() - 1] = PulseCode::end_marker();
//!
//! loop {
//!     let transaction = channel.transmit(&data)?;
//!     transaction.wait()?;
//!     delay.delay_millis(500);
//! }
//! # }
//! ```
//! 
//! ### RX operation
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! # use esp_hal::rmt::{PulseCode, Rmt, RxChannel, RxChannelConfig, RxChannelCreator};
//! # use esp_hal::delay::Delay;
//! # use esp_hal::gpio::{Level, Output, OutputConfig};
//!
//! const WIDTH: usize = 80;
//!
//! let mut out = Output::new(
//!     peripherals.GPIO5,
//!     Level::Low,
//!     OutputConfig::default(),
//! );
//!
//! // Configure frequency based on chip type
#![cfg_attr(esp32h2, doc = "let freq = Rate::from_mhz(32);")]
#![cfg_attr(not(esp32h2), doc = "let freq = Rate::from_mhz(80);")]
//! let rmt = Rmt::new(peripherals.RMT, freq)?;
//!
//! let rx_config = RxChannelConfig::default()
//!     .with_clk_divider(1)
//!     .with_idle_threshold(10000);
#![cfg_attr(
    any(esp32, esp32s2),
    doc = "let mut channel = rmt.channel0.configure_rx(peripherals.GPIO4, rx_config)?;"
)]
#![cfg_attr(
    esp32s3,
    doc = "let mut channel = rmt.channel7.configure_rx(peripherals.GPIO4, rx_config)?;"
)]
#![cfg_attr(
    not(any(esp32, esp32s2, esp32s3)),
    doc = "let mut channel = rmt.channel2.configure_rx(peripherals.GPIO4, rx_config)?;"
)]
//! let delay = Delay::new();
//! let mut data: [PulseCode; 48] = [PulseCode::end_marker(); 48];
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
//!         Ok(count) => {
//!             let mut total = 0usize;
//!             for entry in &data[..count] {
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
//!             for entry in &data[..count] {
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
//!         Err(_err) => {}
//!     }
//!
//!     delay.delay_millis(1500);
//! }
//! # }
//! ```
//! 
//! > Note: on ESP32 and ESP32-S2 you cannot specify a base frequency other than 80 MHz

use core::{
    borrow::Borrow,
    default::Default,
    marker::PhantomData,
    mem::ManuallyDrop,
    pin::Pin,
    task::{Context, Poll},
};

use enumset::{EnumSet, EnumSetType};
use portable_atomic::{AtomicU8, Ordering};

use crate::{
    Async,
    Blocking,
    asynch::AtomicWaker,
    gpio::{
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

/// Errors
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(clippy::enum_variant_names, reason = "peripheral is unstable")]
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
    /// An endmarker was encounted in the middle of the data
    UnexpectedEndMarker,
}

///  Convenience newtype to work with pulse codes.
// FIXME: Add More derives
#[derive(Copy, Clone, PartialEq, Eq, Default)]
#[repr(transparent)]
pub struct PulseCode(u32);

const LEVEL2_SHIFT: usize = 16;
const LEVEL1_SHIFT: usize = 0;

impl PulseCode {
    /// Create a new instance.
    ///
    /// SAFETY:
    /// - `length1` and `length2` must be 15-bit wide, i.e. their MSB must be
    ///   cleared.
    #[inline]
    const unsafe fn new_unchecked(
        level1: Level,
        length1: u16,
        level2: Level,
        length2: u16,
    ) -> Self {
        let mut code = 0;

        if matches!(level2, Level::High) {
            code |= 1 << (LEVEL2_SHIFT + 15);
        }
        code |= (length2 as u32) << LEVEL2_SHIFT;

        if matches!(level1, Level::High) {
            code |= 1 << (LEVEL1_SHIFT + 15);
        }
        code |= (length1 as u32) << LEVEL1_SHIFT;

        Self(code)
    }

    /// Create a new instance.
    ///
    /// If `length1` or `length2` exceed the maximum representable range, they
    /// will be clamped.
    #[inline]
    pub const fn new(level1: Level, length1: u16, level2: Level, length2: u16) -> Self {
        let length1 = if 0 != (length1 & 0x8000) {
            0x7FFF
        } else {
            length1
        };
        let length2 = if 0 != (length2 & 0x8000) {
            0x7FFF
        } else {
            length2
        };
        // SAFETY:
        // - We just clamped length1 and length2 to the required intervals
        unsafe { Self::new_unchecked(level1, length1, level2, length2) }
    }

    /// Create a new instance.
    ///
    /// If `length1` or `length2` exceed the maximum representable range, this
    /// will return `None`.
    #[inline]
    pub const fn try_new(level1: Level, length1: u16, level2: Level, length2: u16) -> Option<Self> {
        if 0 != (length1 & 0x8000) || 0 != (length2 & 0x8000) {
            return None;
        }

        // SAFETY:
        // - We just checked that length1 and length2 have their MSB cleared.
        Some(unsafe { Self::new_unchecked(level1, length1, level2, length2) })
    }

    /// Create a new empty instance
    // FIXME: add level argument
    #[inline]
    pub const fn end_marker() -> Self {
        Self(0)
    }

    /// Set all levels and lengths to 0
    #[inline]
    pub fn reset(&mut self) {
        self.0 = 0
    }

    /// Logical output level in the second pulse code interval
    #[inline]
    pub const fn level2(&self) -> Level {
        // Can't use Level::from(bool) since it is non-const
        if 0 != (self.0 & (1 << (LEVEL2_SHIFT + 15))) {
            Level::High
        } else {
            Level::Low
        }
    }

    /// Length of the second pulse code interval (in clock cycles)
    #[inline]
    pub const fn length2(&self) -> u16 {
        ((self.0 >> LEVEL2_SHIFT) & 0x7FFF) as u16
    }

    /// Logical output level in the first pulse code interval
    #[inline]
    pub const fn level1(&self) -> Level {
        // Can't use Level::from(bool) since it is non-const
        if 0 != (self.0 & (1 << (LEVEL1_SHIFT + 15))) {
            Level::High
        } else {
            Level::Low
        }
    }

    /// Length of the first pulse code interval (in clock cycles)
    #[inline]
    pub const fn length1(&self) -> u16 {
        ((self.0 >> LEVEL1_SHIFT) & 0x7FFF) as u16
    }

    /// Set level2
    #[inline]
    pub const fn with_level2(mut self, level: Level) -> Self {
        if matches!(level, Level::High) {
            self.0 |= 1 << (LEVEL2_SHIFT + 15);
        } else {
            self.0 &= !(1 << (LEVEL2_SHIFT + 15));
        }
        self
    }

    /// Set level1
    #[inline]
    pub const fn with_level1(mut self, level: Level) -> Self {
        if matches!(level, Level::High) {
            self.0 |= 1 << (LEVEL1_SHIFT + 15);
        } else {
            self.0 &= !(1 << (LEVEL1_SHIFT + 15));
        }
        self
    }

    /// Set length2
    #[inline]
    pub const fn with_length2(mut self, length: u16) -> Option<Self> {
        if 0 != (length & 0x8000) {
            None
        } else {
            self.0 &= !(0x7FFF << LEVEL2_SHIFT);
            self.0 |= (length as u32) << LEVEL2_SHIFT;
            Some(self)
        }
    }

    /// Set length1
    #[inline]
    pub const fn with_length1(mut self, length: u16) -> Option<Self> {
        if 0 != (length & 0x8000) {
            None
        } else {
            self.0 &= !(0x7FFF << LEVEL1_SHIFT);
            self.0 |= (length as u32) << LEVEL1_SHIFT;
            Some(self)
        }
    }

    /// Convert to u32
    #[inline]
    pub const fn as_u32(&self) -> u32 {
        self.0
    }

    /// Whether this pulse code contains an end marker
    #[inline]
    pub const fn is_end_marker(&self) -> bool {
        self.length1() == 0 || self.length2() == 0
    }
}

impl core::fmt::Debug for PulseCode {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("PulseCode")
            .field("level1", &self.level1())
            .field("length1", &self.length1())
            .field("level2", &self.level2())
            .field("length2", &self.length2())
            .finish()
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for PulseCode {
    fn format(&self, fmt: defmt::Formatter<'_>) {
        defmt::write!(
            fmt,
            "PulseCode({} {}, {} {})",
            if self.level1().into() { 'H' } else { 'L' },
            self.length1(),
            if self.level2().into() { 'H' } else { 'L' },
            self.length2(),
        )
    }
}

impl From<u32> for PulseCode {
    fn from(value: u32) -> Self {
        Self(value)
    }
}

impl From<PulseCode> for u32 {
    fn from(code: PulseCode) -> u32 {
        code.as_u32()
    }
}

/// Memory size associated to a channel.
#[doc(hidden)]
#[derive(Copy, Clone)]
pub struct MemSize(u8);

impl MemSize {
    #[inline]
    const fn from_blocks(blocks: u8) -> Self {
        Self(blocks)
    }

    #[inline]
    const fn blocks(self) -> u8 {
        self.0
    }

    #[inline]
    const fn codes(self) -> usize {
        self.0 as usize * property!("rmt.channel_ram_size")
    }
}

#[derive(Copy, Clone, PartialEq)]
enum WriterState {
    // Ready to continue writing data to the hardware buffer
    Active,

    // Data exhausted, and last code was a stop code
    Done,

    // Data exhausted, but no stop code encountered
    DoneNoEnd,
    // Data not exhausted, but encountered a stop code
    // DoneEarly,
}

/// docs
struct RmtWriter {
    // Offset in RMT RAM section to start writing from (number of PulseCode!)
    // u16 is sufficient to store this for all devices, and should be small enough such that
    // size_of::<RmtWriter>() == 2 * size_of::<PulseCode>()
    offset: u16,

    // ...
    written: usize,

    // ...
    state: WriterState,
}

impl RmtWriter {
    fn new() -> Self {
        Self {
            offset: 0,
            written: 0,
            state: WriterState::Active,
        }
    }

    // TODO: better check that hw ptr matches expectation when done! that also helps
    // with underruns
    fn write(
        &mut self,
        data: &mut impl Iterator<Item: Borrow<PulseCode>>,
        raw: &impl RawChannelAccess<Dir = Tx>,
        initial: bool,
    ) {
        if !matches!(self.state, WriterState::Active) {
            // Don't call next() on data again!
            return;
        }

        let start = raw.channel_ram_start();
        let memsize = raw.memsize().codes();
        let mut offset = self.offset as usize;

        // FIXME: debug_assert that the current hw read addr is in the part of RAM that
        // we don't overwrite

        // This is only used to fill the entire RAM from its start, or to refill
        // either the first or second half. The code below may rely on this.
        // In particular, this implies that the offset might only need to be wrapped at
        // the end.
        debug_assert!(!initial || offset == 0);

        let count = if initial {
            // Leave space for extra end marker
            memsize - 1
        } else {
            memsize / 2
        };

        let mut last_code = PulseCode::end_marker();
        let mut written = 0;
        while let Some(code) = data.next() {
            last_code = *code.borrow();

            unsafe { start.add(offset).write_volatile(last_code) }
            offset += 1;
            written += 1;
            if offset == memsize {
                offset = 0;
            }

            if written == count {
                break;
            }
        }

        if written < count {
            self.state = if !last_code.is_end_marker() {
                WriterState::DoneNoEnd
            } else {
                WriterState::Done
            };
        }

        // Write an extra end marker to detect underruns.
        // Do not increment the offset afterwards since we want to overwrite it in the
        // next call
        unsafe { start.add(offset).write_volatile(PulseCode::end_marker()) }

        self.written += written;
        self.offset = offset as u16;
    }
}

#[derive(Copy, Clone, PartialEq)]
enum ReaderState {
    // Ready to continue reading data from the hardware buffer
    Active,

    // Encountered a stop code
    Done,

    // Data exhausted, but no stop code encountered
    Overflow,
}

/// docs
struct RmtReader {
    // Offset in RMT RAM section to continue reading from (number of PulseCode!)
    // u16 is sufficient to store this for all devices, and should be small enough such that
    // size_of::<RmtReader>() == 2 * size_of::<PulseCode>()
    offset: u16,

    // ...
    total: usize,

    // ...
    state: ReaderState,
}

impl RmtReader {
    fn new() -> Self {
        Self {
            offset: 0,
            total: 0,
            state: ReaderState::Active,
        }
    }

    fn read<'a>(
        &mut self,
        data: &mut impl Iterator<Item = &'a mut PulseCode>,
        raw: &impl RawChannelAccess<Dir = Rx>,
        count: usize,
    ) {
        if !matches!(self.state, ReaderState::Active) {
            // Don't call next() on data again!
            return;
        }

        let start = raw.channel_ram_start();
        let memsize = raw.memsize().codes();
        let mut offset = self.offset as usize;

        // This is only used to read the entire RAM from its start, or to read
        // either the first or second half. The code below may rely on this.
        // In particular, this implies that the offset might only need to be wrapped at
        // the end.
        debug_assert!(count == memsize && offset == 0 || count == memsize / 2);
        // Offset might take a different value only if there's not data; but in that
        // case we already returned above.
        debug_assert!(offset == 0 || offset == memsize / 2);

        let initial_offset = offset;
        while offset < initial_offset + count {
            match data.next() {
                Some(code) => {
                    *code = unsafe { start.add(offset).read_volatile() };
                    offset += 1;

                    // FIXME: Maybe remove this for more efficiency?
                    if code.is_end_marker() {
                        self.state = ReaderState::Done;
                        break;
                    }
                }
                None => {
                    self.state = ReaderState::Overflow;
                    break;
                }
            }
        }

        debug_assert!(offset <= memsize);
        self.total += offset - initial_offset;
        if offset == memsize {
            // Wrap around
            offset = 0;
        }
        self.offset = offset as u16;
    }
}

/// Marker for a channel capable of/configured for transmit operations
#[derive(Debug)]
pub struct Tx;
/// Marker for a channel capable of/configured for receive operations
#[derive(Debug)]
pub struct Rx;
/// Marker for a channel capable of transmit and receive operations
#[cfg(any(esp32, esp32s2))]
#[derive(Debug)]
pub struct RxTx;

/// FIXME: docs
pub trait Direction: core::fmt::Debug + Unpin {
    /// FIXME: docs
    fn is_tx() -> bool;
}

impl Direction for Tx {
    #[inline]
    fn is_tx() -> bool {
        true
    }
}

impl Direction for Rx {
    #[inline]
    fn is_tx() -> bool {
        false
    }
}

/// docs
pub trait RawChannelAccess: crate::private::Sealed + Unpin {
    // Tx or Rx or Unconfigured
    #[doc(hidden)]
    type Dir: Direction;

    #[doc(hidden)]
    fn channel(&self) -> u8;
}

/// docs
///
/// This is a ZST.
#[derive(Debug)]
pub struct ConstChannelAccess<Dir, const CHANNEL: u8> {
    _direction: PhantomData<Dir>,
}

/// Type-erased equivalent of ConstChannelAccess.
#[derive(Debug)]
pub struct DynChannelAccess<Dir> {
    channel: u8,
    _direction: PhantomData<Dir>,
}

impl<Dir: Direction, const CHANNEL: u8> crate::private::Sealed
    for ConstChannelAccess<Dir, CHANNEL>
{
}
impl<Dir: Direction> crate::private::Sealed for DynChannelAccess<Dir> {}

impl<Dir: Direction, const CHANNEL: u8> ConstChannelAccess<Dir, CHANNEL>
where
    ConstChannelAccess<Dir, CHANNEL>: RawChannelAccess,
{
    unsafe fn conjure() -> Self {
        Self {
            _direction: PhantomData,
        }
    }
}

impl<Dir: Direction> DynChannelAccess<Dir> {
    unsafe fn conjure(channel: u8) -> Self {
        Self {
            channel,
            _direction: PhantomData,
        }
    }
}

impl<const CHANNEL: u8> RawChannelAccess for ConstChannelAccess<Tx, CHANNEL> {
    type Dir = Tx;

    fn channel(&self) -> u8 {
        CHANNEL
    }
}

impl<const CHANNEL: u8> RawChannelAccess for ConstChannelAccess<Rx, CHANNEL> {
    type Dir = Rx;

    #[inline]
    fn channel(&self) -> u8 {
        CHANNEL
    }
}

impl<Dir: Direction> RawChannelAccess for DynChannelAccess<Dir> {
    type Dir = Dir;

    #[inline]
    fn channel(&self) -> u8 {
        self.channel
    }
}

// FIXME: Function to degrade Channels to their type-erased variants
/// docs
pub type AnyTxChannel<Dm> = Channel<Dm, DynChannelAccess<Tx>>;
/// docs
pub type AnyRxChannel<Dm> = Channel<Dm, DynChannelAccess<Rx>>;

// FIXME: Consider removing ChannelCreator in favor of Channel<Unconfigured,
// ...> -> This might not be possible, because ChannelCreator: 'rmt whereas
//    Channel: 'rmt + 'pin
// pub type ChannelCreator<const CHANNEL: u8> = Channel<Unconfigured,
// ConstChannelAccess<Unconfigured, CHANNEL>>;

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

// Declare input/output signals and define the main Rmt struct, holding
// channels.
macro_rules! declare_signals {
    ($name:ident, $type:ident, [$($entry:ident $(,)?)*; $count:expr]) => {
        const $name: [crate::gpio::$type; $count] = [
            $(crate::gpio::$type::$entry,)*
        ];
    };
}

macro_rules! declare_channels {
    (@define_rmt {
        () -> (
            [$($field_decl:tt)*],
            [$($field_init:tt)*]
        )
    }) => {
        /// RMT Instance
        pub struct Rmt<'rmt, Dm>
        where
            Dm: $crate::DriverMode,
        {
            pub(super) peripheral: $crate::peripherals::RMT<'rmt>,
            $($field_decl)+
            _mode: ::core::marker::PhantomData<Dm>,
        }

        impl<'rmt, Dm> Rmt<'rmt, Dm>
        where
            Dm: crate::DriverMode,
        {
            fn create(peripheral: crate::peripherals::RMT<'rmt>) -> Self {
                Self {
                    peripheral,
                    $($field_init)+
                    _mode: ::core::marker::PhantomData,
                }
            }
        }
    };

    (@define_rmt {
        ([$name:ident, $num:literal, $cap:ident] $(, $($ch:tt),+)?)
        -> (
            [$($field_decl:tt)*],
            [$($field_init:tt)*]
        )
    }) => {
        declare_channels! (@define_rmt {
            ($($($ch),+)?) -> ([
                $($field_decl)*
                #[doc = concat!("RMT Channel ", $num)]
                // FIXME: This should probably carry the 'rmt lifetime, i.e.
                // ChannelCreator<'rmt, $num, $cap>
                // Otherwise, one could move the channel out of this struct
                // (since the struct doesn't implement Drop), drop the Rmt struct,
                // and use it beyond the lifetime of Rmt.peripheral
                pub $name: ChannelCreator<Dm, $num, $cap>,
            ], [
                $($field_init)*
                $name: $crate::rmt::ChannelCreator {
                    _mode: ::core::marker::PhantomData,
                    _capabilities: ::core::marker::PhantomData,
                    _guard: $crate::system::GenericPeripheralGuard::new(),
                },
            ])
        });
    };

    ($($ch:tt),+ $(,)?) => {
        declare_channels! (@define_rmt { ($($ch),+) -> ([], []) });
    };
}

const NUM_CHANNELS: usize = if cfg!(any(esp32, esp32s3)) { 8 } else { 4 };

cfg_if::cfg_if! {
    if #[cfg(esp32)] {
        declare_channels!(
            [channel0, 0, RxTx],
            [channel1, 1, RxTx],
            [channel2, 2, RxTx],
            [channel3, 3, RxTx],
            [channel4, 4, RxTx],
            [channel5, 5, RxTx],
            [channel6, 6, RxTx],
            [channel7, 7, RxTx],
        );
        declare_signals!(
            OUTPUT_SIGNALS,
            OutputSignal,
            [RMT_SIG_0, RMT_SIG_1, RMT_SIG_2, RMT_SIG_3, RMT_SIG_4, RMT_SIG_5, RMT_SIG_6, RMT_SIG_7; NUM_CHANNELS]
        );
        declare_signals!(
            INPUT_SIGNALS,
            InputSignal,
            [RMT_SIG_0, RMT_SIG_1, RMT_SIG_2, RMT_SIG_3, RMT_SIG_4, RMT_SIG_5, RMT_SIG_6, RMT_SIG_7; NUM_CHANNELS]
        );
    } else if #[cfg(esp32s2)] {
        declare_channels!(
            [channel0, 0, RxTx],
            [channel1, 1, RxTx],
            [channel2, 2, RxTx],
            [channel3, 3, RxTx],
        );
        declare_signals!(
            OUTPUT_SIGNALS,
            OutputSignal,
            [RMT_SIG_0, RMT_SIG_1, RMT_SIG_2, RMT_SIG_3; NUM_CHANNELS]
        );
        declare_signals!(
            INPUT_SIGNALS,
            InputSignal,
            [RMT_SIG_0, RMT_SIG_1, RMT_SIG_2, RMT_SIG_3; NUM_CHANNELS]
        );
    } else if #[cfg(esp32s3)] {
        declare_channels!(
            [channel0, 0, Tx],
            [channel1, 1, Tx],
            [channel2, 2, Tx],
            [channel3, 3, Tx],
            [channel4, 4, Rx],
            [channel5, 5, Rx],
            [channel6, 6, Rx],
            [channel7, 7, Rx],
        );
        declare_signals!(
            OUTPUT_SIGNALS,
            OutputSignal,
            [RMT_SIG_0, RMT_SIG_1, RMT_SIG_2, RMT_SIG_3; const { NUM_CHANNELS / 2 }]
        );
        declare_signals!(
            INPUT_SIGNALS,
            InputSignal,
            [RMT_SIG_0, RMT_SIG_1, RMT_SIG_2, RMT_SIG_3; const { NUM_CHANNELS / 2 }]
        );
    } else {
        declare_channels!(
            [channel0, 0, Tx],
            [channel1, 1, Tx],
            [channel2, 2, Rx],
            [channel3, 3, Rx],
        );
        declare_signals!(
            OUTPUT_SIGNALS,
            OutputSignal,
            [RMT_SIG_0, RMT_SIG_1; const { NUM_CHANNELS / 2 }]
        );
        declare_signals!(
            INPUT_SIGNALS,
            InputSignal,
            [RMT_SIG_0, RMT_SIG_1; const { NUM_CHANNELS / 2 }]
        );
    }
}

/// RMT Channel Creator
pub struct ChannelCreator<Dm, const CHANNEL: u8, Cap>
where
    Dm: crate::DriverMode,
{
    _mode: PhantomData<Dm>,
    _capabilities: PhantomData<Cap>,
    _guard: GenericPeripheralGuard<{ crate::system::Peripheral::Rmt as u8 }>,
}

impl<Dm, const CHANNEL: u8, Cap> ChannelCreator<Dm, CHANNEL, Cap>
where
    Dm: crate::DriverMode,
{
    // FIXME: This interface isn't great. Come up with a safe alternative that uses
    // STATE tracking.
    /// Unsafely steal a channel creator instance.
    ///
    /// # Safety
    ///
    /// Circumvents HAL ownership and safety guarantees and allows creating
    /// multiple handles to the same peripheral structure.
    pub unsafe fn steal() -> ChannelCreator<Dm, CHANNEL, Cap> {
        ChannelCreator {
            _mode: PhantomData,
            _capabilities: PhantomData,
            _guard: GenericPeripheralGuard::new(),
        }
    }
}

/// FIXME: docs
pub trait TxChannelCreator<Dm: crate::DriverMode> {
    /// FIXME: docs
    type Raw: TxChannelInternal;

    /// FIXME: docs
    fn configure_tx<'pin>(
        self,
        pin: impl PeripheralOutput<'pin>,
        config: TxChannelConfig,
    ) -> Result<Channel<Dm, Self::Raw>, Error>;
}

/// FIXME: docs
pub trait RxChannelCreator<Dm: crate::DriverMode> {
    /// FIXME: docs
    type Raw: RxChannelInternal;

    /// FIXME: docs
    fn configure_rx<'pin>(
        self,
        pin: impl PeripheralInput<'pin>,
        config: RxChannelConfig,
    ) -> Result<Channel<Dm, Self::Raw>, Error>;
}

// TODO:
// - RxTx devices: always allow reconfigure + unconfigure
// - Rx/Tx devices: allow Unconfigure and reconfigure with the same direction

#[cfg(not(any(esp32, esp32s2)))]
impl<Dm, const CHANNEL: u8> TxChannelCreator<Dm> for ChannelCreator<Dm, CHANNEL, Tx>
where
    Dm: crate::DriverMode,
{
    type Raw = ConstChannelAccess<Tx, CHANNEL>;

    /// Configure the TX channel
    fn configure_tx<'pin>(
        self,
        pin: impl PeripheralOutput<'pin>,
        config: TxChannelConfig,
    ) -> Result<Channel<Dm, Self::Raw>, Error> {
        let raw = unsafe { ConstChannelAccess::conjure() };
        configure_tx_channel(&raw, pin, config)?;
        Ok(Channel::new(raw))
    }
}

#[cfg(not(any(esp32, esp32s2)))]
impl<Dm, const CHANNEL: u8> RxChannelCreator<Dm> for ChannelCreator<Dm, CHANNEL, Rx>
where
    Dm: crate::DriverMode,
{
    type Raw = ConstChannelAccess<Rx, CHANNEL>;

    /// Configure the RX channel
    fn configure_rx<'pin>(
        self,
        pin: impl PeripheralInput<'pin>,
        config: RxChannelConfig,
    ) -> Result<Channel<Dm, Self::Raw>, Error> {
        let raw = unsafe { ConstChannelAccess::conjure() };
        configure_rx_channel(&raw, pin, config)?;
        Ok(Channel::new(raw))
    }
}

#[cfg(any(esp32, esp32s2))]
impl<Dm, const CHANNEL: u8> TxChannelCreator<Dm> for ChannelCreator<Dm, CHANNEL, RxTx>
where
    Dm: crate::DriverMode,
{
    type Raw = ConstChannelAccess<Tx, CHANNEL>;

    /// Configure the TX channel
    fn configure_tx<'pin>(
        self,
        pin: impl PeripheralOutput<'pin>,
        config: TxChannelConfig,
    ) -> Result<Channel<Dm, Self::Raw>, Error> {
        let raw = unsafe { ConstChannelAccess::conjure() };
        configure_tx_channel(&raw, pin, config)?;
        Ok(Channel::new(raw))
    }
}

#[cfg(any(esp32, esp32s2))]
impl<Dm, const CHANNEL: u8> RxChannelCreator<Dm> for ChannelCreator<Dm, CHANNEL, RxTx>
where
    Dm: crate::DriverMode,
{
    type Raw = ConstChannelAccess<Rx, CHANNEL>;

    /// Configure the RX channel
    fn configure_rx<'pin>(
        self,
        pin: impl PeripheralInput<'pin>,
        config: RxChannelConfig,
    ) -> Result<Channel<Dm, Self::Raw>, Error> {
        let raw = unsafe { ConstChannelAccess::conjure() };
        configure_rx_channel(&raw, pin, config)?;
        Ok(Channel::new(raw))
    }
}

impl<'rmt, Dm> Rmt<'rmt, Dm>
where
    Dm: crate::DriverMode,
{
    pub(crate) fn new_internal(peripheral: RMT<'rmt>, frequency: Rate) -> Result<Self, Error> {
        let me = Rmt::create(peripheral);
        self::chip_specific::configure_clock(frequency)?;
        Ok(me)
    }
}

impl<'rmt> Rmt<'rmt, Blocking> {
    /// Create a new RMT instance
    pub fn new(peripheral: RMT<'rmt>, frequency: Rate) -> Result<Self, Error> {
        Self::new_internal(peripheral, frequency)
    }

    /// Reconfigures the driver for asynchronous operation.
    pub fn into_async(mut self) -> Rmt<'rmt, Async> {
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
        if STATE[cur_channel as usize]
            .compare_exchange(
                RmtState::Unconfigured as u8,
                next_state as u8,
                Ordering::Acquire,
                Ordering::Relaxed,
            )
            .is_err()
        {
            for i in (channel..cur_channel).rev() {
                STATE[i as usize].store(RmtState::Unconfigured as u8, Ordering::Release);
            }

            return Err(Error::MemoryBlockNotAvailable);
        }

        // Set the first channel to `state` (`Rx`|`Tx`), the remaining (if any) to
        // `Reserved`
        next_state = RmtState::Reserved;
    }

    Ok(())
}

fn configure_rx_channel<'pin, const CHANNEL: u8>(
    raw: &ConstChannelAccess<Rx, CHANNEL>,
    pin: impl PeripheralInput<'pin>,
    config: RxChannelConfig,
) -> Result<(), Error> {
    let threshold = if cfg!(any(esp32, esp32s2)) {
        0b111_1111_1111_1111
    } else {
        0b11_1111_1111_1111
    };

    if config.idle_threshold > threshold {
        return Err(Error::InvalidArgument);
    }

    let memsize = MemSize::from_blocks(config.memsize);
    reserve_channel(raw.channel(), RmtState::RxIdle, memsize)?;

    let pin = pin.into();

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

    Ok(())
}

fn configure_tx_channel<'pin, const CHANNEL: u8>(
    raw: &ConstChannelAccess<Tx, CHANNEL>,
    pin: impl PeripheralOutput<'pin>,
    config: TxChannelConfig,
) -> Result<(), Error> {
    let memsize = MemSize::from_blocks(config.memsize);
    reserve_channel(raw.channel(), RmtState::TxIdle, memsize)?;

    let pin = pin.into();

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

    Ok(())
}

#[derive(Copy, Clone)]
#[repr(u8)]
enum RmtState {
    // The channel is not configured for either rx or tx, and its memory is available
    Unconfigured,

    // The channels is not in use, but one of the preceding channels is using its memory
    Reserved,

    // The channel is configured for rx and currently idle.
    RxIdle,

    // The channel is configured for tx and currently idle.
    TxIdle,

    // The channel is configured for rx and currently performing an async transaction
    RxAsync,

    // The channel is configured for tx and currently performing an async transaction
    TxAsync,
}

impl RmtState {
    #[allow(unused)]
    fn is_tx(&self) -> Option<bool> {
        match self {
            Self::RxIdle | Self::RxAsync => Some(false),
            Self::TxIdle | Self::TxAsync => Some(true),
            _ => None,
        }
    }

    // Safety: Must only be called with valid values of the RmtState discrimiant
    #[allow(unused)]
    unsafe fn from_u8_unchecked(value: u8) -> Self {
        unsafe { core::mem::transmute::<_, Self>(value) }
    }

    #[allow(unused)]
    unsafe fn load_unchecked(channel: u8, ordering: Ordering) -> Self {
        unsafe { Self::from_u8_unchecked(STATE[channel as usize].load(ordering)) }
    }
}

static WAKER: [AtomicWaker; NUM_CHANNELS] = [const { AtomicWaker::new() }; NUM_CHANNELS];
// This must only holds value of RmtState. However, we need atomic access, thus
// represent as AtomicU8.
static STATE: [AtomicU8; NUM_CHANNELS] =
    [const { AtomicU8::new(RmtState::Unconfigured as u8) }; NUM_CHANNELS];

type RmtPeripheralGuard = GenericPeripheralGuard<{ system::Peripheral::Rmt as u8 }>;

/// RMT Channel
#[derive(Debug)]
#[non_exhaustive]
pub struct Channel<Dm, Raw>
where
    Dm: crate::DriverMode,
    Raw: ChannelInternal,
{
    raw: Raw,
    _mode: PhantomData<Dm>,
    _guard: RmtPeripheralGuard,
}

impl<Dm, Raw> Channel<Dm, Raw>
where
    Dm: crate::DriverMode,
    Raw: ChannelInternal,
{
    fn new(raw: Raw) -> Self {
        Self {
            raw,
            _mode: core::marker::PhantomData,
            _guard: GenericPeripheralGuard::new(),
        }
    }
}

// Note that this is intentionally implemented even if Raw is DynChannelAccess
// already for convenience!
impl<Dm, Dir, Raw> Channel<Dm, Raw>
where
    Dm: crate::DriverMode,
    Dir: Direction,
    Raw: RawChannelAccess<Dir = Dir>,
{
    /// Consume the channel and return a type-erased version
    pub fn degrade(self) -> Channel<Dm, DynChannelAccess<Dir>> {
        use core::mem::ManuallyDrop;
        // Disable Drop handler on self
        let old = ManuallyDrop::new(self);
        Channel {
            raw: DynChannelAccess {
                channel: old.raw.channel(),
                _direction: PhantomData,
            },
            _mode: PhantomData,
            // FIXME: Don't clone, but move old._guard
            _guard: old._guard.clone(),
        }
    }
}

impl<Dm, Raw> Drop for Channel<Dm, Raw>
where
    Dm: crate::DriverMode,
    Raw: ChannelInternal,
{
    fn drop(&mut self) {
        let memsize = self.raw.memsize();

        // This isn't really necessary, but be extra sure that this channel can't
        // interfere with others.
        self.raw.set_memsize(MemSize::from_blocks(0));

        for s in STATE[usize::from(self.raw.channel())..][..usize::from(memsize.blocks())]
            .iter()
            .rev()
        {
            // Existence of this `Channel` struct implies exclusive access to these hardware
            // channels, thus simply store the new state.
            s.store(RmtState::Unconfigured as u8, Ordering::Release);
        }
    }
}

// Blocking interfaces
// ----------------------------------------------------------

/// An in-progress transaction for a single shot TX transaction.
pub struct SingleShotTxTransaction<'ch, C, D>
where
    C: TxChannel,
    D: Iterator,
    D::Item: Borrow<PulseCode>,
{
    channel: &'ch mut C,

    writer: RmtWriter,

    // Remaining data that has not yet been written to channel RAM. May be empty.
    data: D,
}

impl<C, D> SingleShotTxTransaction<'_, C, D>
where
    C: TxChannel,
    D: Iterator,
    D::Item: Borrow<PulseCode>,
{
    fn poll_internal(&mut self) -> Option<Event> {
        let raw = self.channel.raw();

        let status = raw.get_tx_status();
        if status == Some(Event::Threshold) {
            raw.reset_tx_threshold_set();

            if self.writer.state == WriterState::Active {
                // re-fill TX RAM
                self.writer.write(&mut self.data, raw, false);
                // FIXME: Return if writer.state indicates an error (ensure
                // to stop transmission first)
            }
        }

        status
    }

    /// Check transmission status and write new data to the hardware if
    /// necessary.
    ///
    /// Returns whether transmission has ended (whether successfully or with an
    /// error). In that case, a subsequent call to `wait()` should return
    /// immediately.
    pub fn poll(&mut self) -> bool {
        match self.poll_internal() {
            Some(Event::Error | Event::End) => true,
            Some(Event::Threshold) | None => false,
        }
    }

    /// Wait for the transaction to complete
    pub fn wait(mut self) -> Result<(), Error> {
        // Not sure that all the error cases below can happen. However, it's best to
        // handle them to be sure that we don't lock up here in case they can happen.
        let result = loop {
            match self.poll_internal() {
                Some(Event::Error) => break Err(Error::TransmissionError),
                Some(Event::End) => {
                    if self.writer.state == WriterState::Active {
                        // Unexpectedly done, even though we have data left.
                        break Err(Error::TransmissionError);
                    } else {
                        break Ok(());
                    }
                }
                _ => continue,
            }
        };

        // Disable the Drop handler since the transaction is properly stopped
        // already.
        let _ = ManuallyDrop::new(self);
        result
    }
}
impl<C, D> Drop for SingleShotTxTransaction<'_, C, D>
where
    C: TxChannel,
    D: Iterator,
    D::Item: Borrow<PulseCode>,
{
    fn drop(&mut self) {
        // If this is dropped, that implies that the transaction was not properly
        // `wait()`ed for. Thus, attempt to stop it as quickly as possible and
        // block in the meantime, such that subsequent uses of the channel are
        // safe (i.e. start from a state where the hardware is stopped).
        let raw = self.channel.raw();
        raw.stop_tx();

        while !matches!(raw.get_tx_status(), Some(Event::Error | Event::End)) {}
    }
}

/// An in-progress continuous TX transaction
pub struct ContinuousTxTransaction<'ch, C: TxChannel> {
    channel: &'ch mut C,
}

impl<C: TxChannel> ContinuousTxTransaction<'_, C> {
    /// Stop transaction when the current iteration ends.
    pub fn stop_next(self) -> Result<(), Error> {
        let raw = &self.channel.raw();

        raw.set_tx_continuous(false);
        raw.update();

        let result = loop {
            match raw.get_tx_status() {
                Some(Event::Error) => break Err(Error::TransmissionError),
                Some(Event::End) => break Ok(()),
                _ => continue,
            }
        };

        // Disable Drop handler since the transaction is stopped already.
        let _ = ManuallyDrop::new(self);
        result
    }

    /// Stop transaction as soon as possible.
    pub fn stop(self) -> Result<(), Error> {
        let raw = &self.channel.raw();

        raw.set_tx_continuous(false);
        raw.update();

        // FIXME: Check TRM on whether this requires an update() call.
        raw.stop_tx();

        let result = loop {
            match raw.get_tx_status() {
                Some(Event::Error) => break Err(Error::TransmissionError),
                Some(Event::End) => break Ok(()),
                _ => continue,
            }
        };

        // Disable Drop handler since the transaction is stopped already.
        let _ = ManuallyDrop::new(self);
        result
    }

    /// Check if the `loopcount` interrupt bit is set
    pub fn is_loopcount_interrupt_set(&self) -> bool {
        self.channel.raw().is_tx_loopcount_interrupt_set()
    }
}

impl<C: TxChannel> Drop for ContinuousTxTransaction<'_, C> {
    fn drop(&mut self) {
        // If this is dropped, that implies that the transaction was not manually
        // stopped with `stop()` or `stop_next()`.
        // Thus, attempt to stop it as quickly as possible and block in the meantime,
        // such that subsequent uses of the channel are safe (i.e. start from a
        // state where the hardware is stopped).
        let raw = self.channel.raw();

        raw.set_tx_continuous(false);
        raw.update();

        raw.stop_tx();

        while !matches!(raw.get_tx_status(), Some(Event::Error | Event::End)) {}
    }
}

/// Channel in TX mode
pub trait TxChannel: Sized {
    // Currently required such that `impl TxChannel` is all that is needed to use
    // `*TxTransaction`.
    #[doc(hidden)]
    fn raw(&self) -> &(impl RawChannelAccess<Dir = Tx> + TxChannelInternal);

    /// Start transmitting the given pulse code sequence.
    /// This returns a [`SingleShotTxTransaction`] which can be used to wait for
    /// the transaction to complete and get back the channel for further
    /// use.
    fn transmit<D>(
        &mut self,
        data: D,
    ) -> Result<SingleShotTxTransaction<'_, Self, <D as IntoIterator>::IntoIter>, Error>
    where
        D: IntoIterator,
        D::Item: Borrow<PulseCode>;

    /// Start transmitting the given pulse code continuously.
    /// This returns a [`ContinuousTxTransaction`] which can be used to stop the
    /// ongoing transmission and get back the channel for further use.
    /// The length of sequence cannot exceed the size of the allocated RMT RAM.
    fn transmit_continuously<D>(
        &mut self,
        data: D,
    ) -> Result<ContinuousTxTransaction<'_, Self>, Error>
    where
        D: IntoIterator,
        D::Item: Borrow<PulseCode>;

    /// Like [`Self::transmit_continuously`] but also sets a loop count.
    /// [`ContinuousTxTransaction`] can be used to check if the loop count is
    /// reached.
    fn transmit_continuously_with_loopcount<D>(
        &mut self,
        loopcount: u16,
        data: D,
    ) -> Result<ContinuousTxTransaction<'_, Self>, Error>
    where
        D: IntoIterator,
        D::Item: Borrow<PulseCode>;
}

impl<Raw> TxChannel for Channel<Blocking, Raw>
where
    Raw: RawChannelAccess<Dir = Tx> + TxChannelInternal,
{
    fn raw(&self) -> &(impl RawChannelAccess<Dir = Tx> + TxChannelInternal) {
        &self.raw
    }

    /// Start transmitting the given pulse code sequence.
    /// This returns a [`SingleShotTxTransaction`] which can be used to wait for
    /// the transaction to complete and get back the channel for further use.
    fn transmit<D>(
        &mut self,
        data: D,
    ) -> Result<SingleShotTxTransaction<'_, Self, <D as IntoIterator>::IntoIter>, Error>
    where
        D: IntoIterator,
        D::Item: Borrow<PulseCode>,
    {
        let raw = &self.raw;

        let mut data = data.into_iter();
        let mut writer = RmtWriter::new();
        writer.write(&mut data, raw, true);

        if writer.written == 0 {
            return Err(Error::InvalidArgument);
        }

        match writer.state {
            WriterState::DoneNoEnd => return Err(Error::EndMarkerMissing),
            // WriterState::DoneEarly => return Err(Error::UnexpectedEndMarker),
            _ => (),
        };

        raw.start_send(false, 0);

        Ok(SingleShotTxTransaction {
            channel: self,
            writer,
            data,
        })
    }

    /// Start transmitting the given pulse code continuously.
    /// This returns a [`ContinuousTxTransaction`] which can be used to stop the
    /// ongoing transmission and get back the channel for further use.
    /// The length of sequence cannot exceed the size of the allocated RMT RAM.
    fn transmit_continuously<D>(
        &mut self,
        data: D,
    ) -> Result<ContinuousTxTransaction<'_, Self>, Error>
    where
        D: IntoIterator,
        D::Item: Borrow<PulseCode>,
    {
        self.transmit_continuously_with_loopcount(0, data)
    }

    /// Like [`Self::transmit_continuously`] but also sets a loop count.
    /// [`ContinuousTxTransaction`] can be used to check if the loop count is
    /// reached.
    fn transmit_continuously_with_loopcount<D>(
        &mut self,
        loopcount: u16,
        data: D,
    ) -> Result<ContinuousTxTransaction<'_, Self>, Error>
    where
        D: IntoIterator,
        D::Item: Borrow<PulseCode>,
    {
        let raw = &self.raw;

        let mut data = data.into_iter();
        let mut writer = RmtWriter::new();
        writer.write(&mut data, raw, true);

        if writer.written == 0 {
            return Err(Error::InvalidArgument);
        }

        match writer.state {
            WriterState::Active => return Err(Error::Overflow),
            WriterState::DoneNoEnd => return Err(Error::EndMarkerMissing),
            // WriterState::DoneEarly => return Err(Error::UnexpectedEndMarker),
            WriterState::Done => (),
        };

        raw.start_send(true, loopcount);

        Ok(ContinuousTxTransaction { channel: self })
    }
}

/// RX transaction instance
pub struct RxTransaction<'ch, 'a, C, D>
where
    C: RxChannel,
    D: Iterator<Item = &'a mut PulseCode>,
{
    channel: &'ch mut C,

    reader: RmtReader,

    data: D,
}

impl<'a, C, D> RxTransaction<'_, 'a, C, D>
where
    C: RxChannel,
    D: Iterator<Item = &'a mut PulseCode>,
{
    fn poll_internal(&mut self) -> Option<Event> {
        let raw = self.channel.raw();

        let status = raw.get_rx_status();
        match status {
            Some(Event::End) => {
                if self.reader.state != ReaderState::Done {
                    // Do not clear the interrupt flags here: Subsequent calls of wait() must still
                    // be able to observe them if this is currently called via
                    // poll()
                    raw.stop_rx();
                    raw.update();

                    // read() does not wrap around, so we need to call it twice to handle the case
                    // where the current read offset is memsize / 2 (i.e. the second half of RMT RAM
                    // is read first).
                    // FIXME: This probably isn't correct: We must not read beyond the HW pointer.
                    let memsize = raw.memsize().codes();
                    self.reader.read(&mut self.data, raw, memsize / 2);
                    self.reader.read(&mut self.data, raw, memsize / 2);

                    // Ensure that no further data will be read if this is called repeatedly.
                    self.reader.state = ReaderState::Done;
                }
            }
            Some(Event::Threshold) if C::Raw::supports_rx_wrap() => {
                raw.reset_rx_threshold_set();

                let memsize = raw.memsize().codes();
                self.reader.read(&mut self.data, raw, memsize / 2);
            }
            _ => (),
        }

        status
    }

    /// Check receive status
    ///
    /// Returns whether reception has ended (whether successfully or with an
    /// error). In that case, a subsequent call to `wait()` should return
    /// immediately.
    pub fn poll(&mut self) -> bool {
        match self.poll_internal() {
            Some(Event::Error | Event::End) => true,
            Some(Event::Threshold) | None => false,
        }
    }

    /// Wait for the transaction to complete
    pub fn wait(mut self) -> Result<usize, Error> {
        let result = loop {
            match self.poll_internal() {
                Some(Event::Error) => break Err(Error::ReceiverError),
                Some(Event::End) => break Ok(self.reader.total),
                _ => continue,
            }
        };

        self.channel.raw().clear_rx_interrupts();

        // Disable Drop handler since the receiver is stopped already.
        let _ = ManuallyDrop::new(self);
        result
    }
}

impl<'a, C, D> Drop for RxTransaction<'_, 'a, C, D>
where
    C: RxChannel,
    D: Iterator<Item = &'a mut PulseCode>,
{
    fn drop(&mut self) {
        // If this is dropped, that implies that the transaction was not properly
        // `wait()`ed for. Thus, attempt to stop it as quickly as possible and
        // block in the meantime, such that subsequent uses of the channel are
        // safe (i.e. start from a state where the hardware is stopped).
        let raw = self.channel.raw();

        raw.stop_rx();
        raw.update();

        while !matches!(raw.get_rx_status(), Some(Event::Error | Event::End)) {}
    }
}

/// Channel is RX mode
pub trait RxChannel: Sized {
    // Currently required such that `impl RxChannel` is all that is needed to use
    // `RxTransaction`.
    #[doc(hidden)]
    type Raw: RawChannelAccess<Dir = Rx> + RxChannelInternal;

    #[doc(hidden)]
    fn raw(&self) -> &Self::Raw;

    /// Start receiving pulse codes into the given buffer.
    /// This returns a [RxTransaction] which can be used to wait for receive to
    /// complete and get back the channel for further use.
    /// The length of the received data cannot exceed the allocated RMT RAM.
    fn receive<'ch, 'a, D>(
        &'ch mut self,
        data: D,
    ) -> Result<RxTransaction<'ch, 'a, Self, D::IntoIter>, Error>
    where
        D: IntoIterator<Item = &'a mut PulseCode>;
}

impl<Raw> RxChannel for Channel<Blocking, Raw>
where
    Raw: RawChannelAccess<Dir = Rx> + RxChannelInternal,
{
    type Raw = Raw;

    fn raw(&self) -> &Raw {
        &self.raw
    }

    /// Start receiving pulse codes into the given buffer.
    /// This returns a [RxTransaction] which can be used to wait for receive to
    /// complete and get back the channel for further use.
    fn receive<'ch, 'a, D>(
        &'ch mut self,
        data: D,
    ) -> Result<RxTransaction<'ch, 'a, Self, D::IntoIter>, Error>
    where
        D: IntoIterator<Item = &'a mut PulseCode>,
    {
        self.raw.start_receive(true);

        Ok(RxTransaction {
            channel: self,
            reader: RmtReader::new(),
            data: data.into_iter(),
        })
    }
}

// Async interfaces ----------------------------------------------------------

// FIXME: This is essentially the same as SingleShotTxTransaction. Is it
// possible to share most of the code?
/// TODO: docs
#[must_use = "futures do nothing unless you `.await` or poll them"]
pub struct RmtTxFuture<'ch, Raw, D>
where
    Raw: RawChannelAccess<Dir = Tx> + TxChannelInternal,
    D: Iterator + Unpin,
    D::Item: Borrow<PulseCode>,
{
    raw: &'ch mut Raw,

    writer: RmtWriter,

    // Remaining data that has not yet been written to channel RAM. May be empty.
    data: D,
}

impl<Raw, D> core::future::Future for RmtTxFuture<'_, Raw, D>
where
    Raw: RawChannelAccess<Dir = Tx> + TxChannelInternal,
    D: Iterator + Unpin,
    D::Item: Borrow<PulseCode>,
{
    type Output = Result<(), Error>;

    fn poll(self: Pin<&mut Self>, ctx: &mut Context<'_>) -> Poll<Self::Output> {
        // Note that STATE is only accessed from a single core and no ISR here, so
        // Relaxed access is sufficient.
        let this = self.get_mut();
        let raw: &mut Raw = this.raw;

        WAKER[raw.channel() as usize].register(ctx.waker());

        let result = match raw.get_tx_status() {
            Some(Event::Error) => Poll::Ready(Err(Error::TransmissionError)),
            Some(Event::End) => {
                let result = if this.writer.state == WriterState::Active {
                    // Unexpectedly done, even though we have data left.
                    Err(Error::TransmissionError)
                } else {
                    Ok(())
                };

                Poll::Ready(result)
            }
            Some(Event::Threshold) => {
                raw.reset_tx_threshold_set();

                if this.writer.state == WriterState::Active {
                    // re-fill TX RAM
                    this.writer.write(&mut this.data, raw, false);
                    // FIXME: Return if writer.state indicates an error (ensure
                    // to stop transmission first)
                }
                if this.writer.state == WriterState::Done {
                    raw.unlisten_tx_interrupt(Event::Threshold);
                }

                Poll::Pending
            }
            _ => Poll::Pending,
        };

        if matches!(result, Poll::Ready(_)) {
            raw.unlisten_tx_interrupt(Event::Error | Event::End | Event::Threshold);
            raw.clear_tx_interrupts();
            STATE[raw.channel() as usize].store(RmtState::TxIdle as u8, Ordering::Relaxed);
        }

        result
    }
}

impl<Raw, D> Drop for RmtTxFuture<'_, Raw, D>
where
    Raw: RawChannelAccess<Dir = Tx> + TxChannelInternal,
    D: Iterator + Unpin,
    D::Item: Borrow<PulseCode>,
{
    fn drop(&mut self) {
        let raw: &mut Raw = self.raw;

        // STATE should be TxIdle if the future was polled to completion
        if STATE[raw.channel() as usize].load(Ordering::Relaxed) == RmtState::TxAsync as u8 {
            raw.unlisten_tx_interrupt(Event::Error | Event::End | Event::Threshold);
            raw.stop_tx();

            // block until the channel is safe to use again
            while !matches!(raw.get_tx_status(), Some(Event::Error | Event::End)) {}

            raw.clear_tx_interrupts();
            STATE[raw.channel() as usize].store(RmtState::TxIdle as u8, Ordering::Relaxed);
        }
    }
}

/// TX channel in async mode
pub trait TxChannelAsync {
    #[doc(hidden)]
    type Raw: RawChannelAccess<Dir = Tx> + TxChannelInternal;

    /// Start transmitting the given pulse code sequence.
    /// The length of sequence cannot exceed the size of the allocated RMT
    /// RAM.
    fn transmit<D>(&mut self, data: D) -> Result<RmtTxFuture<'_, Self::Raw, D::IntoIter>, Error>
    where
        Self: Sized,
        D: IntoIterator,
        D::IntoIter: Unpin,
        D::Item: Borrow<PulseCode>;
}

impl<Raw> TxChannelAsync for Channel<Async, Raw>
where
    Raw: RawChannelAccess<Dir = Tx> + TxChannelInternal,
{
    type Raw = Raw;

    /// Start transmitting the given pulse code sequence.
    /// The length of sequence cannot exceed the size of the allocated RMT
    /// RAM.
    fn transmit<D>(&mut self, data: D) -> Result<RmtTxFuture<'_, Self::Raw, D::IntoIter>, Error>
    where
        Self: Sized,
        D: IntoIterator,
        // FIXME: Is being Unpin a significant restriction for the iterator?
        // If so, we could use it in a pinned way by using pin-projection (e.g. via
        // pin-project-lite) for the RmtTxFuture.
        D::IntoIter: Unpin,
        D::Item: Borrow<PulseCode>,
    {
        let raw = &mut self.raw;

        let mut data = data.into_iter();
        let mut writer = RmtWriter::new();
        writer.write(&mut data, raw, true);

        if writer.written == 0 {
            return Err(Error::InvalidArgument);
        }

        let wrap = match writer.state {
            WriterState::DoneNoEnd => return Err(Error::EndMarkerMissing),
            // WriterState::DoneEarly => return Err(Error::UnexpectedEndMarker),
            WriterState::Active => true,
            WriterState::Done => false,
        };

        STATE[raw.channel() as usize].store(RmtState::TxAsync as u8, Ordering::Relaxed);

        raw.clear_tx_interrupts();
        let mut events = Event::End | Event::Error;
        if wrap {
            events |= Event::Threshold
        }
        raw.listen_tx_interrupt(events);
        raw.start_send(false, 0);

        Ok(RmtTxFuture { raw, writer, data })
    }
}

#[must_use = "futures do nothing unless you `.await` or poll them"]
struct RmtRxFuture<'a, Raw>
where
    Raw: RxChannelInternal,
{
    raw: &'a Raw,
}

impl<'a, Raw> RmtRxFuture<'a, Raw>
where
    Raw: RxChannelInternal,
{
    pub fn new(raw: &'a Raw) -> Self {
        Self { raw }
    }
}

impl<Raw> core::future::Future for RmtRxFuture<'_, Raw>
where
    Raw: RxChannelInternal,
{
    type Output = Event;

    fn poll(self: Pin<&mut Self>, ctx: &mut Context<'_>) -> Poll<Self::Output> {
        let raw = &self.raw;
        WAKER[raw.channel() as usize].register(ctx.waker());

        let result = match raw.get_rx_status() {
            Some(ev @ (Event::Error | Event::End)) => Poll::Ready(ev),
            _ => Poll::Pending,
        };

        if matches!(result, Poll::Ready(_)) {
            raw.clear_rx_interrupts();
            STATE[raw.channel() as usize].store(RmtState::RxIdle as u8, Ordering::Relaxed);
        }

        result
    }
}

impl<Raw> Drop for RmtRxFuture<'_, Raw>
where
    Raw: RxChannelInternal,
{
    fn drop(&mut self) {
        let raw = self.raw;

        // STATE should be RxIdle if the future was polled to completion
        if STATE[raw.channel() as usize].load(Ordering::Relaxed) == RmtState::RxAsync as u8 {
            raw.stop_rx();
            raw.update();

            // block until the channel is safe to use again
            while !matches!(raw.get_rx_status(), Some(Event::Error | Event::End)) {}

            raw.clear_rx_interrupts();
            STATE[raw.channel() as usize].store(RmtState::RxIdle as u8, Ordering::Relaxed);
        }
    }
}

/// RX channel in async mode
pub trait RxChannelAsync {
    /// Start receiving a pulse code sequence.
    /// The length of sequence cannot exceed the size of the allocated RMT
    /// RAM.
    async fn receive<T: From<PulseCode> + Copy>(&mut self, data: &mut [T]) -> Result<(), Error>
    where
        Self: Sized;
}

impl<Raw> RxChannelAsync for Channel<Async, Raw>
where
    Raw: RxChannelInternal,
{
    async fn receive<T: From<PulseCode> + Copy>(&mut self, data: &mut [T]) -> Result<(), Error>
    where
        Self: Sized,
    {
        let raw = &self.raw;

        if data.len() > raw.memsize().codes() {
            return Err(Error::InvalidDataLength);
        }

        STATE[raw.channel() as usize].store(RmtState::RxAsync as u8, Ordering::Relaxed);

        raw.clear_rx_interrupts();
        raw.listen_rx_interrupt(Event::End | Event::Error);
        raw.start_receive(false);

        match RmtRxFuture::new(raw).await {
            Event::Error => Err(Error::ReceiverError),
            Event::End => {
                raw.stop_rx();
                raw.clear_rx_interrupts();
                raw.update();

                let ptr = raw.channel_ram_start();
                let len = data.len();
                for (idx, entry) in data.iter_mut().take(len).enumerate() {
                    *entry = unsafe { ptr.add(idx).read_volatile().into() };
                }

                Ok(())
            }
            Event::Threshold => unreachable!(),
        }
    }
}

enum AnyChannelAccess {
    Tx(DynChannelAccess<Tx>),
    Rx(DynChannelAccess<Rx>),
}

impl AnyChannelAccess {
    #[inline]
    fn conjure(ch_num: u8, is_tx: bool) -> Self {
        if is_tx {
            Self::Tx(unsafe { DynChannelAccess::<Tx>::conjure(ch_num) })
        } else {
            Self::Rx(unsafe { DynChannelAccess::<Rx>::conjure(ch_num) })
        }
    }

    #[inline]
    fn channel(self) -> u8 {
        match self {
            Self::Tx(raw) => raw.channel(),
            Self::Rx(raw) => raw.channel(),
        }
    }
}

#[handler]
fn async_interrupt_handler() {
    for (raw, event) in chip_specific::pending_interrupt_for_channel() {
        match event {
            Event::End | Event::Error => match raw {
                AnyChannelAccess::Tx(ref raw) => {
                    raw.unlisten_tx_interrupt(Event::End | Event::Error)
                }
                AnyChannelAccess::Rx(ref raw) => {
                    raw.unlisten_rx_interrupt(Event::End | Event::Error)
                }
            },
            Event::Threshold => {
                // Just wake, but don't disable the interrupt
            }
        }
        WAKER[raw.channel() as usize].wake();
    }
}

#[derive(Debug, EnumSetType)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[doc(hidden)]
pub enum Event {
    Error,
    Threshold,
    End,
}

#[doc(hidden)]
pub trait ChannelInternal: RawChannelAccess {
    fn update(&self);

    fn set_divider(&self, divider: u8);

    fn memsize(&self) -> MemSize;

    fn set_memsize(&self, value: MemSize);

    fn is_error(&self) -> bool;

    #[inline]
    fn channel_ram_start(&self) -> *mut PulseCode {
        unsafe {
            (property!("rmt.ram_start") as *mut PulseCode)
                .add(usize::from(self.channel()) * property!("rmt.channel_ram_size"))
        }
    }
}

#[doc(hidden)]
pub trait TxChannelInternal: ChannelInternal {
    fn output_signal(&self) -> crate::gpio::OutputSignal {
        OUTPUT_SIGNALS[self.channel() as usize]
    }

    fn set_generate_repeat_interrupt(&self, repeats: u16);

    fn clear_tx_interrupts(&self);

    fn set_tx_continuous(&self, continuous: bool);

    fn set_tx_wrap_mode(&self, wrap: bool);

    fn set_tx_carrier(&self, carrier: bool, high: u16, low: u16, level: Level);

    fn set_tx_idle_output(&self, enable: bool, level: Level);

    fn start_tx(&self);

    // Return the first flag that is set of, in order of decreasing priority,
    // Event::Error, Event::End, Event::Threshold
    fn get_tx_status(&self) -> Option<Event>;

    fn is_tx_done(&self) -> bool;

    fn is_tx_threshold_set(&self) -> bool;

    fn reset_tx_threshold_set(&self);

    fn set_tx_threshold(&self, threshold: u8);

    fn is_tx_loopcount_interrupt_set(&self) -> bool;

    fn start_send(&self, continuous: bool, repeat: u16) {
        self.clear_tx_interrupts();

        self.set_tx_threshold((self.memsize().codes() / 2) as u8);
        self.set_tx_continuous(continuous);
        self.set_generate_repeat_interrupt(repeat);
        self.set_tx_wrap_mode(true);
        self.update();
        self.start_tx();
        self.update();
    }

    fn stop_tx(&self);

    fn set_tx_interrupt(&self, event: EnumSet<Event>, enable: bool);

    fn listen_tx_interrupt(&self, event: impl Into<EnumSet<Event>>) {
        self.set_tx_interrupt(event.into(), true);
    }

    fn unlisten_tx_interrupt(&self, event: impl Into<EnumSet<Event>>) {
        self.set_tx_interrupt(event.into(), false);
    }
}

#[doc(hidden)]
pub trait RxChannelInternal: ChannelInternal {
    fn input_signal(&self) -> crate::gpio::InputSignal {
        INPUT_SIGNALS[self.channel() as usize]
    }

    fn clear_rx_interrupts(&self);

    fn supports_rx_wrap() -> bool;

    fn set_rx_wrap_mode(&self, wrap: bool);

    fn set_rx_carrier(&self, carrier: bool, high: u16, low: u16, level: Level);

    fn start_rx(&self);

    // Return the first flag that is set of, in order of decreasing priority,
    // Event::Error, Event::End, Event::Threshold
    fn get_rx_status(&self) -> Option<Event>;

    fn is_rx_done(&self) -> bool;

    fn is_rx_threshold_set(&self) -> bool;

    fn reset_rx_threshold_set(&self);

    fn set_rx_threshold(&self, threshold: u8);

    fn start_receive(&self, wrap: bool) {
        self.clear_rx_interrupts();

        self.set_rx_threshold((self.memsize().codes() / 2) as u8);
        self.set_rx_wrap_mode(wrap);
        self.update();
        self.start_rx();
        self.update();
    }

    fn stop_rx(&self);

    fn set_rx_filter_threshold(&self, value: u8);

    fn set_rx_idle_threshold(&self, value: u16);

    fn set_rx_interrupt(&self, event: EnumSet<Event>, enable: bool);

    fn listen_rx_interrupt(&self, event: impl Into<EnumSet<Event>>) {
        self.set_rx_interrupt(event.into(), true);
    }

    fn unlisten_rx_interrupt(&self, event: impl Into<EnumSet<Event>>) {
        self.set_rx_interrupt(event.into(), false);
    }
}

#[cfg(not(any(esp32, esp32s2)))]
mod chip_specific {
    use enumset::EnumSet;

    use super::{
        AnyChannelAccess,
        ChannelInternal,
        Direction,
        Error,
        Event,
        Level,
        MemSize,
        NUM_CHANNELS,
        RawChannelAccess,
        Rx,
        RxChannelInternal,
        Tx,
        TxChannelInternal,
    };
    use crate::{peripherals::RMT, time::Rate};

    pub(super) fn configure_clock(frequency: Rate) -> Result<(), Error> {
        let src_clock = crate::soc::constants::RMT_CLOCK_SRC_FREQ;

        if frequency > src_clock {
            return Err(Error::UnreachableTargetFrequency);
        }

        let Ok(div) = u8::try_from((src_clock / frequency) - 1) else {
            return Err(Error::UnreachableTargetFrequency);
        };

        #[cfg(not(pcr))]
        {
            RMT::regs().sys_conf().modify(|_, w| unsafe {
                w.clk_en().clear_bit();
                w.sclk_sel().bits(crate::soc::constants::RMT_CLOCK_SRC);
                w.sclk_div_num().bits(div);
                w.sclk_div_a().bits(0);
                w.sclk_div_b().bits(0);
                w.apb_fifo_mask().set_bit()
            });
        }

        #[cfg(pcr)]
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
                .modify(|_, w| unsafe { w.sclk_sel().bits(crate::soc::constants::RMT_CLOCK_SRC) });
            #[cfg(not(esp32c6))]
            PCR::regs()
                .rmt_sclk_conf()
                .modify(|_, w| w.sclk_sel().bit(crate::soc::constants::RMT_CLOCK_SRC));

            RMT::regs()
                .sys_conf()
                .modify(|_, w| w.apb_fifo_mask().set_bit());
        }

        Ok(())
    }

    #[allow(unused)]
    pub(super) fn pending_interrupt_for_channel() -> impl Iterator<Item = (AnyChannelAccess, Event)>
    {
        let st = RMT::regs().int_st().read();

        (0..NUM_CHANNELS as u8 / 2).filter_map(move |ch_idx| {
            let (is_tx, event) = if st.ch_tx_err(ch_idx).bit() {
                (true, Event::Error)
            } else if st.ch_rx_err(ch_idx).bit() {
                (false, Event::Error)
            } else if st.ch_tx_end(ch_idx).bit() {
                (true, Event::End)
            } else if st.ch_rx_end(ch_idx).bit() {
                (false, Event::End)
            } else if st.ch_tx_thr_event(ch_idx).bit() {
                (true, Event::Threshold)
            } else if st.ch_rx_thr_event(ch_idx).bit() {
                (false, Event::Threshold)
            } else {
                return None;
            };

            let ch_num = if is_tx {
                // The first half of all channels support tx...
                ch_idx
            } else {
                // ...whereas the second half of channels support rx.
                NUM_CHANNELS as u8 / 2 + ch_idx
            };

            Some((AnyChannelAccess::conjure(ch_num, is_tx), event))
        })
    }

    // The index of this channel among all Tx/Rx channels (which may be different
    // from the channel number).
    #[inline]
    fn ch_idx<Dir: Direction>(raw: &impl RawChannelAccess<Dir = Dir>) -> u8 {
        if Dir::is_tx() {
            raw.channel()
        } else {
            raw.channel() - NUM_CHANNELS as u8 / 2
        }
    }

    // FIXME: Consider implementing directly on RawChannelAccess instead of the
    // extension trait?
    impl<A> ChannelInternal for A
    where
        A: RawChannelAccess<Dir: Direction>,
    {
        fn update(&self) {
            let rmt = crate::peripherals::RMT::regs();
            let ch_idx = ch_idx(self) as usize;

            if A::Dir::is_tx() {
                rmt.ch_tx_conf0(ch_idx)
                    .modify(|_, w| w.conf_update().set_bit());
            } else {
                rmt.ch_rx_conf1(ch_idx)
                    .modify(|_, w| w.conf_update().set_bit());
            }
        }

        fn set_divider(&self, divider: u8) {
            let rmt = crate::peripherals::RMT::regs();
            let ch_idx = ch_idx(self) as usize;

            if A::Dir::is_tx() {
                rmt.ch_tx_conf0(ch_idx)
                    .modify(|_, w| unsafe { w.div_cnt().bits(divider) });
            } else {
                rmt.ch_rx_conf0(ch_idx)
                    .modify(|_, w| unsafe { w.div_cnt().bits(divider) });
            }
        }

        fn memsize(&self) -> MemSize {
            let rmt = RMT::regs();
            let ch_idx = ch_idx(self) as usize;

            let blocks = if A::Dir::is_tx() {
                rmt.ch_tx_conf0(ch_idx).read().mem_size().bits()
            } else {
                rmt.ch_rx_conf0(ch_idx).read().mem_size().bits()
            };

            MemSize::from_blocks(blocks)
        }

        fn set_memsize(&self, value: MemSize) {
            let blocks = value.blocks();
            let rmt = RMT::regs();
            let ch_idx = ch_idx(self) as usize;

            if A::Dir::is_tx() {
                rmt.ch_tx_conf0(ch_idx)
                    .modify(|_, w| unsafe { w.mem_size().bits(blocks) });
            } else {
                rmt.ch_rx_conf0(ch_idx)
                    .modify(|_, w| unsafe { w.mem_size().bits(blocks) });
            }
        }

        fn is_error(&self) -> bool {
            let rmt = crate::peripherals::RMT::regs();
            let int_raw = rmt.int_raw().read();
            let ch_idx = ch_idx(self);

            if A::Dir::is_tx() {
                int_raw.ch_tx_err(ch_idx).bit()
            } else {
                int_raw.ch_rx_err(ch_idx).bit()
            }
        }
    }

    impl<A> TxChannelInternal for A
    where
        A: RawChannelAccess<Dir = Tx>,
    {
        fn set_generate_repeat_interrupt(&self, repeats: u16) {
            let rmt = crate::peripherals::RMT::regs();
            if repeats > 1 {
                rmt.ch_tx_lim(self.channel().into()).modify(|_, w| unsafe {
                    w.loop_count_reset().set_bit();
                    w.tx_loop_cnt_en().set_bit();
                    w.tx_loop_num().bits(repeats)
                });
            } else {
                rmt.ch_tx_lim(self.channel().into()).modify(|_, w| unsafe {
                    w.loop_count_reset().set_bit();
                    w.tx_loop_cnt_en().clear_bit();
                    w.tx_loop_num().bits(0)
                });
            }

            rmt.ch_tx_lim(self.channel().into())
                .modify(|_, w| w.loop_count_reset().clear_bit());
        }

        fn clear_tx_interrupts(&self) {
            let rmt = crate::peripherals::RMT::regs();

            rmt.int_clr().write(|w| {
                w.ch_tx_end(self.channel()).set_bit();
                w.ch_tx_err(self.channel()).set_bit();
                w.ch_tx_loop(self.channel()).set_bit();
                w.ch_tx_thr_event(self.channel()).set_bit()
            });
        }

        fn set_tx_continuous(&self, continuous: bool) {
            let rmt = crate::peripherals::RMT::regs();

            rmt.ch_tx_conf0(self.channel().into())
                .modify(|_, w| w.tx_conti_mode().bit(continuous));
        }

        fn set_tx_wrap_mode(&self, wrap: bool) {
            let rmt = crate::peripherals::RMT::regs();

            rmt.ch_tx_conf0(self.channel().into())
                .modify(|_, w| w.mem_tx_wrap_en().bit(wrap));
        }

        fn set_tx_carrier(&self, carrier: bool, high: u16, low: u16, level: Level) {
            let rmt = crate::peripherals::RMT::regs();

            rmt.chcarrier_duty(self.channel().into())
                .write(|w| unsafe { w.carrier_high().bits(high).carrier_low().bits(low) });

            rmt.ch_tx_conf0(self.channel().into()).modify(|_, w| {
                w.carrier_en().bit(carrier);
                w.carrier_eff_en().set_bit();
                w.carrier_out_lv().bit(level.into())
            });
        }

        fn set_tx_idle_output(&self, enable: bool, level: Level) {
            let rmt = crate::peripherals::RMT::regs();
            rmt.ch_tx_conf0(self.channel().into())
                .modify(|_, w| w.idle_out_en().bit(enable).idle_out_lv().bit(level.into()));
        }

        fn start_tx(&self) {
            let rmt = crate::peripherals::RMT::regs();

            rmt.ref_cnt_rst()
                .write(|w| unsafe { w.bits(self.channel().into()) });
            self.update();

            rmt.ch_tx_conf0(self.channel().into()).modify(|_, w| {
                w.mem_rd_rst().set_bit();
                w.apb_mem_rst().set_bit();
                w.tx_start().set_bit()
            });
            self.update();
        }

        fn get_tx_status(&self) -> Option<Event> {
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

        fn is_tx_done(&self) -> bool {
            let rmt = crate::peripherals::RMT::regs();
            rmt.int_raw().read().ch_tx_end(self.channel()).bit()
        }

        fn is_tx_threshold_set(&self) -> bool {
            let rmt = crate::peripherals::RMT::regs();
            rmt.int_raw().read().ch_tx_thr_event(self.channel()).bit()
        }

        fn reset_tx_threshold_set(&self) {
            let rmt = crate::peripherals::RMT::regs();
            rmt.int_clr()
                .write(|w| w.ch_tx_thr_event(self.channel()).set_bit());
        }

        fn set_tx_threshold(&self, threshold: u8) {
            let rmt = crate::peripherals::RMT::regs();
            rmt.ch_tx_lim(self.channel().into())
                .modify(|_, w| unsafe { w.tx_lim().bits(threshold as u16) });
        }

        fn is_tx_loopcount_interrupt_set(&self) -> bool {
            let rmt = crate::peripherals::RMT::regs();
            rmt.int_raw().read().ch_tx_loop(self.channel()).bit()
        }

        fn stop_tx(&self) {
            let rmt = crate::peripherals::RMT::regs();
            rmt.ch_tx_conf0(self.channel().into())
                .modify(|_, w| w.tx_stop().set_bit());
            self.update();
        }

        // Ensure that this is always inlined in (un)listen_tx_interrupt
        #[inline]
        fn set_tx_interrupt(&self, events: EnumSet<Event>, enable: bool) {
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

    impl<A> RxChannelInternal for A
    where
        A: RawChannelAccess<Dir = Rx>,
    {
        fn input_signal(&self) -> crate::gpio::InputSignal {
            let ch_idx = ch_idx(self) as usize;
            super::INPUT_SIGNALS[ch_idx]
        }

        fn clear_rx_interrupts(&self) {
            let rmt = crate::peripherals::RMT::regs();
            let ch_idx = ch_idx(self);

            rmt.int_clr().write(|w| {
                w.ch_rx_end(ch_idx).set_bit();
                w.ch_rx_err(ch_idx).set_bit();
                w.ch_rx_thr_event(ch_idx).set_bit()
            });
        }

        fn supports_rx_wrap() -> bool {
            true
        }

        fn set_rx_wrap_mode(&self, wrap: bool) {
            let rmt = crate::peripherals::RMT::regs();
            let ch_idx = ch_idx(self) as usize;

            rmt.ch_rx_conf1(ch_idx)
                .modify(|_, w| w.mem_rx_wrap_en().bit(wrap));
        }

        fn set_rx_carrier(&self, carrier: bool, high: u16, low: u16, level: Level) {
            let ch_idx = ch_idx(self) as usize;
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

        fn start_rx(&self) {
            let rmt = crate::peripherals::RMT::regs();
            let ch_idx = ch_idx(self);

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

        fn get_rx_status(&self) -> Option<Event> {
            let rmt = crate::peripherals::RMT::regs();
            let reg = rmt.int_raw().read();
            let ch_idx = ch_idx(self);

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

        fn is_rx_done(&self) -> bool {
            let rmt = crate::peripherals::RMT::regs();
            let ch_idx = ch_idx(self);
            rmt.int_raw().read().ch_rx_end(ch_idx).bit()
        }

        fn is_rx_threshold_set(&self) -> bool {
            let rmt = crate::peripherals::RMT::regs();
            let ch_idx = ch_idx(self);
            rmt.int_raw().read().ch_rx_thr_event(ch_idx).bit()
        }

        fn reset_rx_threshold_set(&self) {
            let rmt = crate::peripherals::RMT::regs();
            let ch_idx = ch_idx(self);
            rmt.int_clr().write(|w| w.ch_rx_thr_event(ch_idx).set_bit());
        }

        fn set_rx_threshold(&self, threshold: u8) {
            let rmt = crate::peripherals::RMT::regs();
            let ch_idx = ch_idx(self) as usize;
            rmt.ch_rx_lim(ch_idx).modify(|_, w| unsafe {
                cfg_if::cfg_if!(
                    if #[cfg(any(esp32c6, esp32h2))] {
                        w.rmt_rx_lim().bits(threshold as u16)
                    } else {
                        w.rx_lim().bits(threshold as u16)
                    }
                );
                w
            });
        }

        fn stop_rx(&self) {
            let rmt = crate::peripherals::RMT::regs();
            let ch_idx = ch_idx(self) as usize;
            rmt.ch_rx_conf1(ch_idx).modify(|_, w| w.rx_en().clear_bit());
        }

        fn set_rx_filter_threshold(&self, value: u8) {
            let rmt = crate::peripherals::RMT::regs();
            let ch_idx = ch_idx(self) as usize;

            rmt.ch_rx_conf1(ch_idx).modify(|_, w| unsafe {
                w.rx_filter_en().bit(value > 0);
                w.rx_filter_thres().bits(value)
            });
        }

        fn set_rx_idle_threshold(&self, value: u16) {
            let rmt = crate::peripherals::RMT::regs();
            let ch_idx = ch_idx(self) as usize;

            rmt.ch_rx_conf0(ch_idx)
                .modify(|_, w| unsafe { w.idle_thres().bits(value) });
        }

        // Ensure that this is always inlined in (un)listen_rx_interrupt
        #[inline]
        fn set_rx_interrupt(&self, events: EnumSet<Event>, enable: bool) {
            let rmt = crate::peripherals::RMT::regs();
            let ch_idx = ch_idx(self);

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
        AnyChannelAccess,
        ChannelInternal,
        Direction,
        Error,
        Event,
        Level,
        MemSize,
        NUM_CHANNELS,
        RawChannelAccess,
        RmtState,
        Rx,
        RxChannelInternal,
        Tx,
        TxChannelInternal,
    };
    use crate::{peripherals::RMT, time::Rate};

    pub fn configure_clock(frequency: Rate) -> Result<(), Error> {
        if frequency != Rate::from_mhz(80) {
            return Err(Error::UnreachableTargetFrequency);
        }

        let rmt = RMT::regs();

        for ch_num in 0..NUM_CHANNELS {
            rmt.chconf1(ch_num)
                .modify(|_, w| w.ref_always_on().set_bit());
        }

        rmt.apb_conf().modify(|_, w| w.apb_fifo_mask().set_bit());

        #[cfg(not(esp32))]
        rmt.apb_conf().modify(|_, w| w.clk_en().set_bit());

        Ok(())
    }

    #[allow(unused)]
    pub(super) fn pending_interrupt_for_channel() -> impl Iterator<Item = (AnyChannelAccess, Event)>
    {
        let st = RMT::regs().int_st().read();

        (0..NUM_CHANNELS as u8).filter_map(move |ch_num| {
            let (is_tx, event) = if st.ch_err(ch_num).bit() {
                if let Some(is_tx) =
                    unsafe { RmtState::load_unchecked(ch_num, Ordering::Relaxed) }.is_tx()
                {
                    (is_tx, Event::Error)
                } else {
                    // Shouldn't happen: The channel isn't configured for rx or tx, but an error
                    // interrupt occured. Ignore it.
                    return None;
                }
            } else if st.ch_tx_end(ch_num).bit() {
                (true, Event::End)
            } else if st.ch_rx_end(ch_num).bit() {
                (false, Event::End)
            } else if st.ch_tx_thr_event(ch_num).bit() {
                (true, Event::Threshold)
            } else {
                return None;
            };

            Some((AnyChannelAccess::conjure(ch_num, is_tx), event))
        })
    }

    impl<A> ChannelInternal for A
    where
        A: RawChannelAccess<Dir: Direction>,
    {
        fn update(&self) {
            // no-op
        }

        fn set_divider(&self, divider: u8) {
            let rmt = crate::peripherals::RMT::regs();
            rmt.chconf0(self.channel() as usize)
                .modify(|_, w| unsafe { w.div_cnt().bits(divider) });
        }

        fn memsize(&self) -> MemSize {
            let rmt = crate::peripherals::RMT::regs();
            let blocks = rmt
                .chconf0(self.channel() as usize)
                .read()
                .mem_size()
                .bits();
            MemSize::from_blocks(blocks)
        }

        fn set_memsize(&self, value: MemSize) {
            let rmt = crate::peripherals::RMT::regs();
            rmt.chconf0(self.channel() as usize)
                .modify(|_, w| unsafe { w.mem_size().bits(value.blocks()) });
        }

        fn is_error(&self) -> bool {
            let rmt = crate::peripherals::RMT::regs();
            rmt.int_raw().read().ch_err(self.channel()).bit()
        }
    }

    impl<A> TxChannelInternal for A
    where
        A: RawChannelAccess<Dir = Tx>,
    {
        #[cfg(not(esp32))]
        fn set_generate_repeat_interrupt(&self, repeats: u16) {
            let rmt = crate::peripherals::RMT::regs();
            if repeats > 1 {
                rmt.ch_tx_lim(self.channel() as usize)
                    .modify(|_, w| unsafe { w.tx_loop_num().bits(repeats) });
            } else {
                rmt.ch_tx_lim(self.channel() as usize)
                    .modify(|_, w| unsafe { w.tx_loop_num().bits(0) });
            }
        }

        #[cfg(esp32)]
        fn set_generate_repeat_interrupt(&self, _repeats: u16) {
            // unsupported
        }

        fn clear_tx_interrupts(&self) {
            let rmt = crate::peripherals::RMT::regs();
            let ch = self.channel();

            rmt.int_clr().write(|w| {
                w.ch_err(ch).set_bit();
                w.ch_tx_end(ch).set_bit();
                w.ch_tx_thr_event(ch).set_bit()
            });
        }

        fn set_tx_continuous(&self, continuous: bool) {
            let rmt = crate::peripherals::RMT::regs();

            rmt.chconf1(self.channel() as usize)
                .modify(|_, w| w.tx_conti_mode().bit(continuous));
        }

        fn set_tx_wrap_mode(&self, wrap: bool) {
            let rmt = crate::peripherals::RMT::regs();
            // this is "okay", because we use all TX channels always in wrap mode
            rmt.apb_conf().modify(|_, w| w.mem_tx_wrap_en().bit(wrap));
        }

        fn set_tx_carrier(&self, carrier: bool, high: u16, low: u16, level: Level) {
            let rmt = crate::peripherals::RMT::regs();
            let ch = self.channel() as usize;

            rmt.chcarrier_duty(ch)
                .write(|w| unsafe { w.carrier_high().bits(high).carrier_low().bits(low) });

            rmt.chconf0(ch).modify(|_, w| {
                w.carrier_en()
                    .bit(carrier)
                    .carrier_out_lv()
                    .bit(level.into())
            });
        }

        fn set_tx_idle_output(&self, enable: bool, level: Level) {
            let rmt = crate::peripherals::RMT::regs();
            rmt.chconf1(self.channel() as usize)
                .modify(|_, w| w.idle_out_en().bit(enable).idle_out_lv().bit(level.into()));
        }

        fn start_tx(&self) {
            let rmt = crate::peripherals::RMT::regs();
            let ch = self.channel();

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

        fn get_tx_status(&self) -> Option<Event> {
            let rmt = crate::peripherals::RMT::regs();
            let reg = rmt.int_raw().read();
            let ch = self.channel();

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

        fn is_tx_done(&self) -> bool {
            let rmt = crate::peripherals::RMT::regs();
            rmt.int_raw().read().ch_tx_end(self.channel()).bit()
        }

        fn is_tx_threshold_set(&self) -> bool {
            let rmt = crate::peripherals::RMT::regs();
            rmt.int_raw().read().ch_tx_thr_event(self.channel()).bit()
        }

        fn reset_tx_threshold_set(&self) {
            let rmt = crate::peripherals::RMT::regs();
            rmt.int_clr()
                .write(|w| w.ch_tx_thr_event(self.channel()).set_bit());
        }

        fn set_tx_threshold(&self, threshold: u8) {
            let rmt = crate::peripherals::RMT::regs();
            rmt.ch_tx_lim(self.channel() as usize)
                .modify(|_, w| unsafe { w.tx_lim().bits(threshold as u16) });
        }

        fn is_tx_loopcount_interrupt_set(&self) -> bool {
            // no-op
            false
        }

        #[cfg(esp32s2)]
        fn stop_tx(&self) {
            let rmt = crate::peripherals::RMT::regs();
            rmt.chconf1(self.channel() as usize)
                .modify(|_, w| w.tx_stop().set_bit());
        }

        #[cfg(esp32)]
        fn stop_tx(&self) {
            let ptr = self.channel_ram_start();
            for idx in 0..self.memsize().codes() {
                unsafe {
                    ptr.add(idx).write_volatile(super::PulseCode::end_marker());
                }
            }
        }

        // Ensure that this is always inlined in (un)listen_tx_interrupt
        #[inline]
        fn set_tx_interrupt(&self, events: EnumSet<Event>, enable: bool) {
            let rmt = crate::peripherals::RMT::regs();
            let ch = self.channel();
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

    impl<A> RxChannelInternal for A
    where
        A: RawChannelAccess<Dir = Rx>,
    {
        fn clear_rx_interrupts(&self) {
            let rmt = crate::peripherals::RMT::regs();
            let ch = self.channel();

            rmt.int_clr().write(|w| {
                w.ch_rx_end(ch).set_bit();
                w.ch_err(ch).set_bit();
                w.ch_tx_thr_event(ch).set_bit()
            });
        }

        fn supports_rx_wrap() -> bool {
            false
        }

        fn set_rx_wrap_mode(&self, _wrap: bool) {
            // no-op
        }

        fn set_rx_carrier(&self, carrier: bool, high: u16, low: u16, level: Level) {
            let rmt = crate::peripherals::RMT::regs();
            let ch = self.channel() as usize;

            rmt.chcarrier_duty(ch)
                .write(|w| unsafe { w.carrier_high().bits(high).carrier_low().bits(low) });

            rmt.chconf0(ch).modify(|_, w| {
                w.carrier_en()
                    .bit(carrier)
                    .carrier_out_lv()
                    .bit(level.into())
            });
        }

        fn start_rx(&self) {
            let rmt = crate::peripherals::RMT::regs();
            let ch = self.channel();

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

        fn get_rx_status(&self) -> Option<Event> {
            let rmt = crate::peripherals::RMT::regs();
            let reg = rmt.int_raw().read();
            let ch = self.channel();

            if reg.ch_rx_end(ch).bit() {
                Some(Event::End)
            } else if reg.ch_err(ch).bit() {
                Some(Event::Error)
            } else {
                None
            }
        }

        fn is_rx_done(&self) -> bool {
            let rmt = crate::peripherals::RMT::regs();
            rmt.int_raw().read().ch_rx_end(self.channel()).bit()
        }

        fn is_rx_threshold_set(&self) -> bool {
            // no-op
            false
        }

        fn reset_rx_threshold_set(&self) {
            // no-op
        }

        fn set_rx_threshold(&self, _threshold: u8) {
            // no-op
        }

        fn stop_rx(&self) {
            let rmt = crate::peripherals::RMT::regs();
            rmt.chconf1(self.channel() as usize)
                .modify(|_, w| w.rx_en().clear_bit());
        }

        fn set_rx_filter_threshold(&self, value: u8) {
            let rmt = crate::peripherals::RMT::regs();
            rmt.chconf1(self.channel() as usize).modify(|_, w| unsafe {
                w.rx_filter_en().bit(value > 0);
                w.rx_filter_thres().bits(value)
            });
        }

        fn set_rx_idle_threshold(&self, value: u16) {
            let rmt = crate::peripherals::RMT::regs();
            rmt.chconf0(self.channel() as usize)
                .modify(|_, w| unsafe { w.idle_thres().bits(value) });
        }

        // Ensure that this is always inlined in (un)listen_rx_interrupt
        #[inline]
        fn set_rx_interrupt(&self, events: EnumSet<Event>, enable: bool) {
            let rmt = crate::peripherals::RMT::regs();
            let ch = self.channel();
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
