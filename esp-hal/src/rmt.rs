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
//! # use esp_hal::rmt::{PulseCode, Rmt, TxChannelConfig};
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
//!     transaction.wait()?;
//!     delay.delay_millis(500);
//! }
//! # }
//! ```
//!
//! ### RX operation
//! ```rust, no_run
//! # {before_snippet}
//! # use esp_hal::rmt::{PulseCode, Rmt, RxChannelConfig};
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
    default::Default,
    marker::PhantomData,
    mem::ManuallyDrop,
    num::NonZeroU16,
    ops::ControlFlow,
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
    time::{Instant, Rate},
};

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
    /// An endmarker was encounted in the middle of the data
    UnexpectedEndMarker,
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
    /// Create a new instance.
    ///
    /// If `length1` or `length2` exceed the maximum representable range, they
    /// will be clamped to `0x7FFF`.
    #[inline]
    pub const fn new(level1: Level, length1: u16, level2: Level, length2: u16) -> Self {
        // Can't use lengthX.min(0x7FFF) since it is not const
        let length1 = if length1 >= 0x8000 { 0x7FFF } else { length1 };
        let length2 = if length2 >= 0x8000 { 0x7FFF } else { length2 };

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
        if length1 >= 0x8000 || length2 >= 0x8000 {
            return None;
        }

        // SAFETY:
        // - We just checked that length1 and length2 have their MSB cleared.
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

    /// Total length of the pulse code (in clock cycles)
    #[inline]
    pub const fn length(&self) -> u16 {
        self.length1() + self.length2()
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
        if length >= 0x8000 {
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
        if length >= 0x8000 {
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
// This currently needs to be `pub` because it is accessible via the *ChannelInternal traits.
#[doc(hidden)]
#[derive(Copy, Clone)]
pub struct MemSize(u8);

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

/// FIXME: Docs
pub struct RmtSlot<'a> {
    writer: &'a mut RmtWriterInner,
}

impl<'a> RmtSlot<'a> {
    /// Append a `PulseCode` to the RMT hardware buffer.
    ///
    /// An `RmtSlot` indicates a guarantee that there's a free slot in memory
    /// that can immediately be written to. If this slot isn't used, the
    /// next call to `RmtWriterInner.next` will use the same slot again (i.e.
    /// there will be no garbage data due to skipping unused slots).
    // For maximum efficiency (this needs to run in the ISR!), be extra sure that this will be
    // inlined in Encoder.encode
    #[inline(always)]
    pub fn write(self, code: PulseCode) {
        let writer = self.writer;

        debug_assert!(writer.ptr < writer.end);

        unsafe { writer.ptr.write_volatile(code) }

        writer.ptr = unsafe { writer.ptr.add(1) };
        writer.last_code = code;
    }
}

/// FIXME: Docs
pub struct RmtWriterInner {
    last_code: PulseCode,
    ptr: *mut PulseCode,
    start: *mut PulseCode,
    end: *mut PulseCode,
}

#[cfg(feature = "defmt")]
impl defmt::Format for RmtWriterInner {
    fn format(&self, fmt: defmt::Formatter<'_>) {
        fn get_ch(ptr: *mut PulseCode) -> i32 {
            let diff = unsafe { ptr.offset_from(property!("rmt.ram_start") as *mut PulseCode) };
            (diff / property!("rmt.channel_ram_size") as isize) as i32
        }

        fn get_offset(ptr: *mut PulseCode) -> i32 {
            let diff = unsafe { ptr.offset_from(property!("rmt.ram_start") as *mut PulseCode) };
            (diff % property!("rmt.channel_ram_size") as isize) as i32
        }

        defmt::write!(
            fmt,
            "RmtWriterInner {{ last_code: {:?}, ptr: CH{} + {:#x}, start: CH{} + {:#x}, end: CH{} + {:#x} }}",
            self.last_code,
            get_ch(self.ptr),
            get_offset(self.ptr),
            get_ch(self.start),
            get_offset(self.start),
            get_ch(self.end),
            get_offset(self.end),
        )
    }
}

impl RmtWriterInner {
    /// FIXME: docs
    #[inline(always)]
    pub fn next<'s>(&'s mut self) -> Option<RmtSlot<'s>>
    // where 's: 'a
    {
        if self.ptr >= self.end {
            return None;
        }

        Some(RmtSlot { writer: self })
    }

    /// FIXME: docs
    // Note that this explicitly supports count = 0, thus callers can avoid that check
    #[inline(always)]
    pub fn write_many<F>(&mut self, count: usize, mut f: F) -> usize
    where
        F: FnMut() -> PulseCode,
    {
        let count = count.min(unsafe { self.end.offset_from(self.ptr) } as usize);

        let mut last_code = self.last_code;
        let mut ptr = self.ptr;
        let end = unsafe { self.ptr.add(count) };

        while ptr < end {
            last_code = f();
            unsafe { ptr.write_volatile(last_code) };
            ptr = unsafe { ptr.add(1) };
        }

        debug_assert!(self.ptr <= self.end);

        self.ptr = ptr;
        self.last_code = last_code;

        count
    }
}

/// FIXME: Docs
pub trait Encoder {
    /// FIXME: docs
    fn encode(&mut self, writer: &mut RmtWriterInner) -> ControlFlow<()>;
}

#[derive(Debug)]
#[derive(Clone)]
#[doc(hidden)]
pub struct SliceEncoder<'a, T>
where
    T: Into<PulseCode> + Copy,
{
    data: &'a [T],
}

impl<'a, T> SliceEncoder<'a, T>
where
    T: Into<PulseCode> + Copy,
{
    fn new(data: &'a [T]) -> Self {
        Self { data }
    }
}

// FIXME: Use write_many
impl<'a, T> Encoder for SliceEncoder<'a, T>
where
    T: Into<PulseCode> + Copy,
{
    #[inline(always)]
    fn encode(&mut self, writer: &mut RmtWriterInner) -> ControlFlow<()> {
        let count = self
            .data
            .len()
            .min(unsafe { writer.end.offset_from(writer.ptr) } as usize);
        if count > 0 {
            let mut ptr = writer.ptr;

            // FIXME: The generated loop still isn't as tight as possible, since it uses an
            // iteration coutner instead of simply comparing the currentl slice pointer to its end.
            let mut last_code = PulseCode::end_marker();
            for code in &self.data[..count] {
                last_code = (*code).into();
                unsafe { ptr.write_volatile(last_code) };
                ptr = unsafe { ptr.add(1) };
            }

            debug_assert!(writer.ptr <= writer.end);

            writer.ptr = ptr;
            writer.last_code = last_code;
            self.data.split_off(..count).unwrap();
        }

        if self.data.is_empty() { ControlFlow::Break(()) } else { ControlFlow::Continue(()) }
    }
}

#[derive(Debug)]
#[doc(hidden)]
pub struct IterEncoder<D>
where
    D: Iterator<Item = PulseCode>,
{
    data: D,
}

impl<D> Clone for IterEncoder<D>
where
    D: Iterator<Item = PulseCode> + Clone,
{
    fn clone(&self) -> Self {
        Self {
            data: self.data.clone(),
        }
    }
}

impl<D> IterEncoder<D>
where
    D: Iterator<Item = PulseCode>,
{
    fn new(data: impl IntoIterator<IntoIter = D>) -> Self {
        Self {
            data: data.into_iter(),
        }
    }
}

impl<D> Encoder for IterEncoder<D>
where
    D: Iterator<Item = PulseCode>,
{
    #[inline(always)]
    fn encode(&mut self, writer: &mut RmtWriterInner) -> ControlFlow<()> {
        while let Some(slot) = writer.next() {
            if let Some(code) = self.data.next() {
                slot.write(code);
            } else {
                return ControlFlow::Break(());
            }
        }

        return ControlFlow::Continue(())
    }
}

#[derive(Copy, Clone, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
enum WriterState {
    // The provided data was empty
    Empty,

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
struct RmtWriterOuter {
    // Offset in RMT RAM section to start writing from (number of PulseCode!)
    // u16 is sufficient to store this for all devices, and should be small enough such that
    // size_of::<RmtWriterOuter>() == 2 * size_of::<u32>()
    offset: u16,

    // ...
    written: usize,

    // ...
    state: WriterState,
}

impl RmtWriterOuter {
    fn new() -> Self {
        Self {
            offset: 0,
            written: 0,
            state: WriterState::Active,
        }
    }

    // TODO: better check that hw ptr matches expectation when done! that also helps
    // with underruns
    // Always inline the wrapper function...
    #[inline]
    fn write(&mut self, data: &mut impl Encoder, raw: impl TxChannelInternal) {
        // ...which calls the inner funciton with a type-erased DynChannelAccess, such
        // that it is only monomorphized for different `data` types: There
        // should be no significant benefit from statically knowing the channel
        // number here.
        #[cfg_attr(not(place_rmt_driver_in_ram), inline(never))]
        #[cfg_attr(place_rmt_driver_in_ram, ram)]
        fn inner(
            this: &mut RmtWriterOuter,
            data: &mut impl Encoder,
            raw: DynChannelAccess<Tx>,
        ) {
            if this.state != WriterState::Active {
                // Don't call next() on data again!
                return;
            }

            let ram_start = raw.channel_ram_start();
            let memsize = raw.memsize().codes();
            // FIXME: Somehow tell the compiler that this is > 0 without having panicking
            // code here! assert!(memsize > 0);
            let ram_end = unsafe { ram_start.add(memsize) };

            // FIXME: debug_assert that the current hw read addr is in the part of RAM that
            // we don't overwrite

            let offset = usize::from(this.offset);
            debug_assert!(offset == 0 || offset == memsize / 2 - 1 || offset == memsize - 1);

            // This is only used to fill the entire RAM from its start, or to refill
            // either the first or second half. The code below may rely on this.
            // In particular, this implies that the offset might only need to be wrapped at
            // the end.

            // FIXME: instead of the initial flag, initialize offset to a special value that will
            // behave correctly with the below calculation! (also, note that we can replace initial
            // with self.total == 0)
            let (count0, mut count1) = {
                // Generate sltu in RISC-V -> branchless
                let mut s = if offset > memsize / 2 { 1 } else { 0 };
                s += if offset == 0 { 1 } else { 0 };
                let diff = offset as isize - memsize as isize / 2;
                (
                    // offset = 0               -> count0 = memsize - 1
                    // offset = memsize - 1     -> count0 = 1
                    // offset = memsize / 2 - 1 -> count0 = memsize / 2
                    memsize - (offset + 1),
                    // offset = 0               -> count1 = 0
                    // offset = memsize - 1     -> count1 = memsize / 2 - 1
                    // offset = memsize / 2 - 1 -> count1 = 0
                    diff.max(0) as usize
                )
            };

            #[cfg(debug_assertions)]
            {
                let (count0_dbg, count1_dbg) = if this.written == 0 {
                    debug_assert!(offset == 0);
                    // Leave space for extra end marker
                    (memsize - 1, 0)
                } else if offset == memsize / 2 - 1 {
                    // Overwrite previous extra end marker, then leave space for new extra end
                    // marker
                    (memsize / 2, 0)
                } else if offset == memsize - 1 {
                    // Overwrite previous extra end marker, then leave space for new extra end
                    // marker
                    (1, memsize / 2 - 1)
                } else {
                    panic!();
                };

                assert!(count0 == count0_dbg, "count0: o={}, w={}: {} != {}", offset, this.written, count0, count0_dbg);
                assert!(count1 == count1_dbg, "count1: o={}, w={}: {} != {}", offset, this.written, count1, count1_dbg);
            }

            let start0 = unsafe { ram_start.add(this.offset as usize) };
            let mut internal = RmtWriterInner {
                last_code: PulseCode::end_marker(),
                ptr: start0,
                start: start0,
                end: unsafe { start0.add(count0) },
            };

            loop {
                let done = data.encode(&mut internal);

                debug_assert!(internal.ptr.addr() >= internal.start.addr());
                debug_assert!(internal.ptr.addr() <= internal.end.addr());

                let written = unsafe { internal.ptr.offset_from(internal.start) } as usize;
                this.written += written;

                if done.is_break() || internal.ptr < internal.end {
                    // Data iterator is exhausted (either it stated so by returning false, or it
                    // didn't fill the entire requested buffer).
                    // FIXME: The is_end_marker check should be in slot.write, maybe?
                    this.state = if written > 0 && internal.last_code.is_end_marker() {
                        WriterState::Done
                    } else if written == 0 && this.written == 0 {
                        WriterState::Empty
                    } else {
                        // - written > 0 and no end marker, or
                        // - written == 0 but previously written some data
                        WriterState::DoneNoEnd
                    };

                    break;
                }

                if count1 == 0 {
                    break;
                }

                debug_assert!(internal.ptr == ram_end);
                internal.ptr = ram_start;
                internal.start = ram_start;
                internal.end = unsafe { ram_start.add(count1) };
                count1 = 0;
            }

            if this.state == WriterState::Active {
                debug_assert!((this.written + 1) % (memsize / 2) == 0);
            }

            // Write an extra end marker to detect underruns.
            // Do not increment the offset or written afterwards since we want to overwrite
            // it in the next call
            debug_assert!(internal.ptr.addr() < ram_end.addr());
            // FIXME: Maybe, this can be moved to the RmtWriterInner/RmtSlot and
            // SliceEncoder to only write it if really needed?
            unsafe { internal.ptr.write_volatile(PulseCode::end_marker()) }

            this.offset = unsafe { internal.ptr.offset_from(ram_start) } as u16;

            // When we're done, the pointer can point anywhere depending on the data length.
            // Otherwise, it should point at the last entry of either half of the channel
            // RAM.
            debug_assert!(
                this.state != WriterState::Active
                    || usize::from(this.offset) == memsize / 2 - 1
                    || usize::from(this.offset) == memsize - 1
            );
        }

        inner(self, data, raw.degrade())
    }
}

/// FIXME: Docs
pub struct RmtReaderInner {
    has_end: bool,
    ptr: *mut PulseCode,
    start: *mut PulseCode,
    end: *mut PulseCode,
}

#[cfg(feature = "defmt")]
impl defmt::Format for RmtReaderInner {
    fn format(&self, fmt: defmt::Formatter<'_>) {
        fn get_ch(ptr: *mut PulseCode) -> i32 {
            let diff = unsafe { ptr.offset_from(property!("rmt.ram_start") as *mut PulseCode) };
            (diff / property!("rmt.channel_ram_size") as isize) as i32
        }

        fn get_offset(ptr: *mut PulseCode) -> i32 {
            let diff = unsafe { ptr.offset_from(property!("rmt.ram_start") as *mut PulseCode) };
            (diff % property!("rmt.channel_ram_size") as isize) as i32
        }

        defmt::write!(
            fmt,
            "RmtReaderInner {{ has_end: {}, ptr: CH{} + {:#x}, start: CH{} + {:#x}, end: CH{} + {:#x} }}",
            self.has_end,
            get_ch(self.ptr),
            get_offset(self.ptr),
            get_ch(self.start),
            get_offset(self.start),
            get_ch(self.end),
            get_offset(self.end),
        )
    }
}

impl RmtReaderInner {
    /// FIXME: docs
    #[inline(always)]
    pub fn next(&mut self) -> Option<PulseCode>
    {
        if self.has_end || self.ptr >= self.end {
            return None;
        }

        let code = unsafe { self.ptr.read_volatile() };
        self.ptr = unsafe { self.ptr.add(1) };

        if code.is_end_marker() {
            // Maybe instead set self.end = self.ptr?
            self.has_end = true;
        }

        Some(code)
    }
}

/// FIXME: Docs
pub trait Decoder {
    /// FIXME: Docs
    fn decode(&mut self, reader: &mut RmtReaderInner) -> ControlFlow<()>;
}

#[derive(Debug)]
#[doc(hidden)]
pub struct SliceDecoder<'a, T>
where
    T: From<PulseCode>,
{
    data: &'a mut [T],
}

impl<'a, T> SliceDecoder<'a, T>
where
    T: From<PulseCode>,
{
    fn new(data: &'a mut [T]) -> Self {
        Self { data }
    }
}

impl<'a, T> Decoder for SliceDecoder<'a, T>
where
    T: From<PulseCode>,
{
    #[inline(always)]
    fn decode(&mut self, reader: &mut RmtReaderInner) -> ControlFlow<()> {
        let count = self
            .data
            .len()
            .min(unsafe { reader.end.offset_from(reader.ptr) } as usize);

        if count > 0 {
            let mut ptr = reader.ptr;

            for slot in &mut self.data[..count] {
                let code = unsafe { ptr.read_volatile() };
                *slot = code.into();
                ptr = unsafe { ptr.add(1) };

                if code.is_end_marker() {
                    reader.has_end = true;
                    break;
                }
            }

            debug_assert!(reader.ptr <= reader.end);

            reader.ptr = ptr;
            self.data.split_off_mut(..count).unwrap();
        }

        if reader.has_end || self.data.is_empty() { ControlFlow::Break(()) } else { ControlFlow::Continue(()) }
    }
}

#[derive(Debug)]
#[doc(hidden)]
pub struct IterDecoder<'a, D, T>
where
    D: Iterator<Item = &'a mut T>,
    T: From<PulseCode> + 'a,
{
    data: D,
}

impl<'a, D, T> IterDecoder<'a, D, T>
where
    D: Iterator<Item = &'a mut T>,
    T: From<PulseCode> + 'a,
{
    fn new(data: impl IntoIterator<IntoIter = D>) -> Self {
        Self {
            data: data.into_iter(),
        }
    }
}

impl<'a, D, T> Decoder for IterDecoder<'a, D, T>
where
    D: Iterator<Item = &'a mut T>,
    T: From<PulseCode> + 'a,
{
    #[inline(always)]
    fn decode(&mut self, reader: &mut RmtReaderInner) -> ControlFlow<()> {
        while let Some(slot) = self.data.next() {
            if let Some(code) = reader.next() {
                *slot = code.into();
            } else {
                return ControlFlow::Continue(());
            }
        }

        // -> ReaderState::Overflow
        return ControlFlow::Break(())
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
struct RmtReaderOuter {
    // Offset in RMT RAM section to continue reading from (number of u32!)
    // u16 is sufficient to store this for all devices, and should be small enough such that
    // size_of::<RmtReaderOuter>() == 2 * size_of::<u32>()
    offset: u16,

    // ...
    total: usize,

    // ...
    state: ReaderState,
}

impl RmtReaderOuter {
    fn new() -> Self {
        Self {
            offset: 0,
            total: 0,
            state: ReaderState::Active,
        }
    }

    fn read(
        &mut self,
        data: &mut impl Decoder,
        raw: impl RxChannelInternal,
        final_: bool,
    ) {
        if self.state != ReaderState::Active {
            // Don't call next() on data again!
            return;
        }

        let ram_start = raw.channel_ram_start();
        let memsize = raw.memsize().codes();
        let ram_end = unsafe { ram_start.add(memsize) };
        let initial_offset = self.offset as usize;

        // This is only used to read the entire RAM from its start, or to read
        // either the first or second half. The code below may rely on this.
        // In particular, this implies that the offset might only need to be wrapped at
        // the end.
        debug_assert!(initial_offset == memsize / 2 || initial_offset == 0);

        // Read in up to 2 chunks
        let count = if final_ { memsize } else { memsize / 2 };
        let count0 = count.min(memsize - initial_offset);
        let mut count1 = count - count0;

        let start0 = unsafe { ram_start.add(initial_offset) };
        let mut internal = RmtReaderInner {
            has_end: false,
            ptr: start0,
            start: start0,
            end: unsafe { start0.add(count0) },
        };
        loop {
            let done = data.decode(&mut internal);

            debug_assert!(internal.ptr.addr() >= internal.start.addr());
            debug_assert!(internal.ptr.addr() <= internal.end.addr());

            let read_count = unsafe { internal.ptr.offset_from(internal.start) } as usize;
            self.total += read_count;

            if internal.has_end {
                self.state = ReaderState::Done;
                break;
            } else if done.is_break() || internal.ptr < internal.end {
                self.state = ReaderState::Overflow;
                break;
            }

            if count1 == 0 {
                // Wrap around such that self.offset update below is correct.
                if internal.ptr == ram_end {
                    internal.ptr = ram_start;
                }
                break;
            }

            debug_assert!(internal.ptr == ram_end);
            internal.ptr = ram_start;
            internal.start = ram_start;
            internal.end = unsafe { ram_start.add(count1) };
            count1 = 0;
        }

        if self.state == ReaderState::Active {
            debug_assert!(self.total % (memsize / 2) == 0);
        }

        self.offset = unsafe { internal.ptr.offset_from(ram_start) } as u16;
        debug_assert!(self.state != ReaderState::Active
            || self.offset == 0
            || usize::from(self.offset) == memsize / 2);
    }
}

/// Marker for a channel capable of/configured for transmit operations
#[derive(Clone, Copy, Debug)]
pub struct Tx;

/// Marker for a channel capable of/configured for receive operations
#[derive(Clone, Copy, Debug)]
pub struct Rx;

/// Marker for a channel capable of transmit and receive operations
#[cfg(any(esp32, esp32s2))]
#[derive(Clone, Copy, Debug)]
pub struct RxTx;

impl crate::private::Sealed for Tx {}

impl crate::private::Sealed for Rx {}

#[cfg(any(esp32, esp32s2))]
impl crate::private::Sealed for RxTx {}

/// Marker for a channel that is not currently configured.
#[derive(Debug)]
pub struct Unconfigured<Dir> {
    _dir: PhantomData<Dir>,
}

/// A trait implemented by the `Rx` and `Tx` marker structs.
///
/// For internal use by the driver.
pub trait Direction: Capability {}

/// FIXME: Docs
pub trait Capability: Copy + Clone + core::fmt::Debug + Unpin + crate::private::Sealed {
    #[doc(hidden)]
    const SUPPORTS_TX: bool;

    #[doc(hidden)]
    fn is_tx() -> bool {
        Self::SUPPORTS_TX
    }
}

impl Direction for Tx {}
impl Direction for Rx {}

impl Capability for Tx {
    const SUPPORTS_TX: bool = true;
}
impl Capability for Rx {
    const SUPPORTS_TX: bool = false;
}
#[cfg(any(esp32, esp32s2))]
impl Capability for RxTx {
    const SUPPORTS_TX: bool = true;
}

// struct ChannelInfo<Cap, const CH_IDX: usize>

/// A sealed marker trait that indicates whether a channel with the given
/// `Capability` can be configured for a `Direction`.
///
/// In other words, this links the `Capability` and `Direction` marker structs
/// in the obvious way.
pub trait Supports<Dir: Direction>: Capability + crate::private::Sealed {}

impl Supports<Tx> for Tx {}
impl Supports<Rx> for Rx {}
#[cfg(any(esp32, esp32s2))]
impl Supports<Tx> for RxTx {}
#[cfg(any(esp32, esp32s2))]
impl Supports<Rx> for RxTx {}

/// An identifier for one channel of the RMT peripherial.
pub trait RawChannelAccess: Clone + Copy + core::fmt::Debug + crate::private::Sealed + Unpin {
    // Tx or Rx
    #[doc(hidden)]
    type Dir: Capability;

    #[doc(hidden)]
    #[inline]
    fn ch_idx(&self) -> u8 {
        self.ch_idx_enum() as u8
    }

    #[doc(hidden)]
    fn ch_idx_enum(&self) -> ChannelIndex;

    #[doc(hidden)]
    fn degrade(&self) -> DynChannelAccess<Self::Dir> {
        DynChannelAccess {
            ch_idx: self.ch_idx_enum(),
            _direction: PhantomData,
        }
    }
}

trait RawChannelAccessExt: RawChannelAccess {
    fn channel(&self) -> u8;

    fn configure<Dir: Direction>(self) -> DynChannelAccess<Dir>
    where
        Self::Dir: Supports<Dir>;
}

impl<Raw> RawChannelAccessExt for Raw
where
    Raw: RawChannelAccess<Dir: Capability>,
{
    #[inline]
    fn channel(&self) -> u8 {
        chip_specific::channel_for_idx::<Raw::Dir>(self.ch_idx_enum())
    }

    fn configure<Dir: Direction>(self) -> DynChannelAccess<Dir>
    where
        Self::Dir: Supports<Dir>
    {
        let ch_idx = self.ch_idx_enum();
        unsafe { DynChannelAccess::conjure(ch_idx) }
    }
}

/// A compile-time constant identifier for one channel of the RMT peripherial.
///
/// This is a ZST.
#[derive(Clone, Copy, Debug)]
pub struct ConstChannelAccess<Dir: Capability, const CH_IDX: u8> {
    _direction: PhantomData<Dir>,
}

/// Type-erased equivalent of ConstChannelAccess.
#[derive(Clone, Copy, Debug)]
pub struct DynChannelAccess<Dir: Capability> {
    ch_idx: ChannelIndex,
    _direction: PhantomData<Dir>,
}

impl<Dir: Capability, const CH_IDX: u8> crate::private::Sealed for ConstChannelAccess<Dir, CH_IDX> {}
impl<Dir: Capability> crate::private::Sealed for DynChannelAccess<Dir> {}

impl<Dir: Capability, const CH_IDX: u8> ConstChannelAccess<Dir, CH_IDX> {
    #[allow(unused)]
    const unsafe fn conjure() -> Self {
        Self {
            _direction: PhantomData,
        }
    }
}

impl<Dir: Capability> DynChannelAccess<Dir> {
    #[inline]
    unsafe fn conjure(ch_idx: ChannelIndex) -> Self {
        Self {
            ch_idx,
            _direction: PhantomData,
        }
    }
}

impl<const CH_IDX: u8, Dir: Capability> RawChannelAccess for ConstChannelAccess<Dir, CH_IDX> {
    type Dir = Dir;

    #[inline]
    fn ch_idx_enum(&self) -> ChannelIndex {
        debug_assert!(CH_IDX < CHANNEL_INDEX_COUNT);
        unsafe { ChannelIndex::from_u8_unchecked(CH_IDX) }
    }
}

impl<Dir: Capability> RawChannelAccess for DynChannelAccess<Dir> {
    type Dir = Dir;

    #[inline]
    fn ch_idx_enum(&self) -> ChannelIndex {
        self.ch_idx
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
macro_rules! declare_channels {
    ($([$num:literal, $idx:literal, $cap:ident]),+ $(,)?) => {
        paste::paste! {
            /// RMT Instance
            pub struct Rmt<'d, Dm>
            where
                Dm: crate::DriverMode,
            {
                pub(super) peripheral: RMT<'d>,
                $(
                    #[doc = concat!("RMT Channel ", $num)]
                    pub [<channel $num>]: ChannelCreator<'d, Dm, ConstChannelAccess<$cap, $idx>>,
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
                            [<channel $num>]: ChannelCreator::new(unsafe { ConstChannelAccess::conjure() }),
                        )+
                        _mode: PhantomData,
                    }
                }
            }

            #[allow(clippy::no_effect)]
            const NUM_CHANNELS: usize = const { 0 $( + {$num; 1} )+ };
        }
    };
}

// Looping channel indices: Declare input/output signals and ChannelIndex
// The number of Rx and Tx channels is identical for all chips.
macro_rules! declare_channel_index {
    ($($idx:literal),+ $(,)?) => {
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
                    /// FIXME: docs
                    [< Ch $idx>] = $idx,
                )+
            }

            #[allow(clippy::no_effect)]
            const CHANNEL_INDEX_COUNT: u8 = const { 0 $( + {$idx; 1} )+ };

            const OUTPUT_SIGNALS: [gpio::OutputSignal; CHANNEL_INDEX_COUNT as usize] = [
                $(
                    gpio::OutputSignal::[< RMT_SIG_ $idx >],
                )+
            ];

            const INPUT_SIGNALS: [gpio::InputSignal; CHANNEL_INDEX_COUNT as usize] = [
                $(
                    gpio::InputSignal::[< RMT_SIG_ $idx >],
                )+
            ];
        }
    };
}

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

cfg_if::cfg_if! {
    if #[cfg(esp32)] {
        declare_channels!(
            [0, 0, RxTx],
            [1, 1, RxTx],
            [2, 2, RxTx],
            [3, 3, RxTx],
            [4, 4, RxTx],
            [5, 5, RxTx],
            [6, 6, RxTx],
            [7, 7, RxTx],
        );
        declare_channel_index!(0, 1, 2, 3, 4, 5, 6, 7);
    } else if #[cfg(esp32s2)] {
        declare_channels!(
            [0, 0, RxTx],
            [1, 1, RxTx],
            [2, 2, RxTx],
            [3, 3, RxTx],
        );
        declare_channel_index!(0, 1, 2, 3);
    } else if #[cfg(esp32s3)] {
        declare_channels!(
            [0, 0, Tx],
            [1, 1, Tx],
            [2, 2, Tx],
            [3, 3, Tx],
            [4, 0, Rx],
            [5, 1, Rx],
            [6, 2, Rx],
            [7, 3, Rx],
        );
        declare_channel_index!(0, 1, 2, 3);
    } else {
        declare_channels!(
            [0, 0, Tx],
            [1, 1, Tx],
            [2, 0, Rx],
            [3, 1, Rx],
        );
        declare_channel_index!(0, 1);
    }
}

/// RMT Channel Creator
#[derive(Debug)]
pub struct ChannelCreator<'rmt, Dm, Raw>
where
    Dm: crate::DriverMode,
    Raw: RawChannelAccess<Dir: Capability>,
{
    raw: Raw,
    _rmt: PhantomData<Rmt<'rmt, Dm>>,
    _guard: GenericPeripheralGuard<{ crate::system::Peripheral::Rmt as u8 }>,
}

impl<'rmt, Dm, Raw> ChannelCreator<'rmt, Dm, Raw>
where
    Dm: crate::DriverMode,
    Raw: RawChannelAccess<Dir: Capability>,
{
    fn new(raw: Raw) -> Self {
        Self {
            raw,
            _rmt: PhantomData,
            _guard: GenericPeripheralGuard::new(),
        }
    }

    /// FIXME: Docs
    pub fn degrade(self) -> ChannelCreator<'rmt, Dm, DynChannelAccess<Raw::Dir>> {
        // FIXME: Share code with Channel::degrade?
        ChannelCreator {
            raw: self.raw.degrade(),
            _rmt: self._rmt,
            _guard: self._guard,
        }
    }

    /// FIXME: Docs
    pub fn reborrow(&mut self) -> ChannelCreator<'_, Dm, Raw> {
        ChannelCreator {
            raw: self.raw,
            _rmt: PhantomData,
            _guard: self._guard.clone(),
        }
    }
}

impl<'rmt, Dm, Dir, const CH_IDX: u8>
    ChannelCreator<'rmt, Dm, ConstChannelAccess<Dir, CH_IDX>>
where
    Dir: Capability,
    Dm: crate::DriverMode,
{
    // FIXME: This interface isn't great. Come up with a safe alternative that uses
    // STATE tracking, and verifies that the RMT is configured for the expected
    // DriverMode.
    /// Unsafely steal a channel creator instance.
    ///
    /// # Safety
    ///
    /// Circumvents HAL ownership and safety guarantees and allows creating
    /// multiple handles to the same peripheral structure.
    #[inline]
    pub unsafe fn steal() -> Option<Self> {
        if CH_IDX >= CHANNEL_INDEX_COUNT {
            return None;
        }

        let raw = unsafe { ConstChannelAccess::conjure() };

        // panic if the channel is currently running a transaction, or if another
        // channel is using its memory
        if !matches!(
            RmtState::load(raw, Ordering::Relaxed),
            RmtState::Unconfigured,
        ) {
            return None;
        };

        Some(ChannelCreator::new(raw))
    }
}

// TODO:
// - RxTx devices: always allow reconfigure + unconfigure
// - Rx/Tx devices: allow Unconfigure and reconfigure with the same direction

impl<'rmt, Dm, Raw> ChannelCreator<'rmt, Dm, Raw>
where
    Dm: crate::DriverMode,
    Raw: RawChannelAccess,
{
    /// Configure the TX channel
    pub fn configure_tx<'pin>(
        self,
        pin: impl PeripheralOutput<'pin>,
        config: TxChannelConfig,
    ) -> Result<Channel<'pin, Dm, Tx>, (Error, Self)>
    where
        Self: 'pin,
        Raw::Dir: Supports<Tx>,
    {
        let raw = self.raw.configure().degrade();
        if let Err(e) = configure_tx_channel(raw, pin.into(), config) {
            return Err((e, self));
        };
        Ok(Channel::new(raw))
    }

    /// Configure the RX channel
    pub fn configure_rx<'pin>(
        self,
        pin: impl PeripheralInput<'pin>,
        config: RxChannelConfig,
    ) -> Result<Channel<'pin, Dm, Rx>, (Error, Self)>
    where
        Self: 'pin,
        Raw::Dir: Supports<Rx>,
    {
        let raw = self.raw.configure().degrade();
        if let Err(e) = configure_rx_channel(raw, pin.into(), config) {
            return Err((e, self));
        };
        Ok(Channel::new(raw))
    }
}

impl<'d> Rmt<'d, Blocking> {
    /// Create a new RMT instance
    pub fn new(peripheral: RMT<'d>, frequency: Rate) -> Result<Self, Error> {
        let this = Rmt::create(peripheral);
        self::chip_specific::configure_clock(frequency)?;
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

fn configure_rx_channel<'d>(
    raw: DynChannelAccess<Rx>,
    pin: gpio::interconnect::InputSignal<'d>,
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

fn configure_tx_channel<'d>(
    raw: DynChannelAccess<Tx>,
    pin: gpio::interconnect::OutputSignal<'d>,
    config: TxChannelConfig,
) -> Result<(), Error> {
    let memsize = MemSize::from_blocks(config.memsize);
    reserve_channel(raw.channel(), RmtState::TxIdle, memsize)?;

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

// We store values of type `RmtState` in the global `STATE`. However, we also need atomic access,
// thus the enum needs to be represented as AtomicU8. Thus, we end up with unsafe conversions
// between `RmtState` and `u8` to avoid constant range checks.
// This submodule wraps all accesses in a safe API which ensures that only valid enum values can
// be stored to `STATE` such that other code in this driver does not need to be concerned with
// these details.
mod state {
    use portable_atomic::{AtomicU8, Ordering};
    use super::{NUM_CHANNELS, Capability, RawChannelAccess, RawChannelAccessExt};

    static STATE: [AtomicU8; NUM_CHANNELS] =
        [const { AtomicU8::new(RmtState::Unconfigured as u8) }; NUM_CHANNELS];

    // Allow implementing RmtState::is_tx via simple bit tests
    // TODO: Check that the compiler actually optimizes this accordingly.
    const RX_STATE_BASE: u8 = 0x40;
    const TX_STATE_BASE: u8 = 0x80;

    #[derive(Copy, Clone, Debug, PartialEq, Eq)]
    #[repr(u8)]
    pub(super) enum RmtState {
        // The channel is not configured for either rx or tx, and its memory is available
        Unconfigured = 0,

        // The channels is not in use, but one of the preceding channels is using its memory
        Reserved     = 1,

        // The channel is configured for rx and currently idle.
        RxIdle       = RX_STATE_BASE,

        // The channel is configured for tx and currently idle.
        TxIdle       = TX_STATE_BASE,

        // The channel is configured for rx and currently performing an async transaction
        RxAsync      = RX_STATE_BASE + 1,

        // The channel is configured for rx and currently performing a blocking transaction
        RxBlocking   = RX_STATE_BASE + 2,

        // The channel is configured for tx and currently performing an async transaction
        TxAsync      = TX_STATE_BASE + 1,

        // The channel is configured for tx and currently performing a blocking transaction
        TxBlocking   = TX_STATE_BASE + 2,
    }

    impl RmtState {
        /// Check whether this state corresponds to a rx, tx, or other configuration.
        #[allow(unused)]
        #[inline]
        pub(super) fn is_tx(&self) -> Option<bool> {
            match self {
                Self::RxIdle | Self::RxAsync | Self::RxBlocking => Some(false),
                Self::TxIdle | Self::TxAsync => Some(true),
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
        #[allow(unused)]
        #[inline]
        pub(super) fn load_by_idx(channel: u8, ordering: Ordering) -> Self {
            unsafe { Self::from_u8_unchecked(STATE[channel as usize].load(ordering)) }
        }

        /// Store the given state to all channel states for an index range in reverse order.
        #[inline]
        pub(super) fn store_range_rev(self, start: u8, end: u8, ordering: Ordering) {
            for ch_num in (start as usize..end as usize).rev() {
                STATE[ch_num].store(self as u8, ordering);
            }
        }

        /// Store channel state to the global `STATE` given a `RawChannelAccess`.
        #[allow(unused)]
        #[inline]
        pub(super) fn store(self, raw: impl RawChannelAccess<Dir: Capability>, ordering: Ordering) {
            STATE[raw.channel() as usize].store(self as u8, ordering);
        }

        /// Load channel state from the global `STATE` given a `RawChannelAccess`.
        #[allow(unused)]
        #[inline]
        pub(super) fn load(
            raw: impl RawChannelAccess<Dir: Capability>,
            ordering: Ordering,
        ) -> Self {
            Self::load_by_idx(raw.channel(), ordering)
        }

        /// Perform a compare_exchange for the channel state for a `RawChannelAccess`.
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
    _rmt: PhantomData<Rmt<'ch, Dm>>,
    _guard: GenericPeripheralGuard<{ system::Peripheral::Rmt as u8 }>,
}

impl<Dm, Dir> Channel<'_, Dm, Dir>
where
    Dm: crate::DriverMode,
    Dir: Direction
{
    fn new(raw: DynChannelAccess<Dir>) -> Self {
        Self {
            raw,
            _rmt: core::marker::PhantomData,
            _guard: GenericPeripheralGuard::new(),
        }
    }

    // FIXME: Implement, regain full Capabilitis of Raw!
    // fn unconfigure(self) -> ChannelCreator<'rmt, Dm, Raw> {
    //     // requires ChannelCreator to take a RawChannelAccess such that
    // type-erased channels can     // also be unconfigured
    //     todo!("implement, then do the same on drop")
    // }

    /// Get size of this channel's hardware buffer (number of `PulseCode`s).
    pub fn buffer_size(&self) -> usize {
        self.raw.memsize().codes()
    }
}

impl<Dm, Dir> Drop for Channel<'_, Dm, Dir>
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

/// An in-progress transaction for a single shot TX transaction.
///
/// If the data size exceeds the size of the internal buffer, `.poll()` or
/// `.wait()` needs to be called before the entire buffer has been sent to avoid
/// underruns.
#[must_use = "transactions need to be `poll()`ed / `wait()`ed for to ensure progress"]
pub struct SingleShotTxTransaction<'a, E>
where
    E: Encoder,
{
    raw: DynChannelAccess<Tx>,
    _phantom: PhantomData<&'a mut DynChannelAccess<Tx>>,

    writer: RmtWriterOuter,

    // Remaining data that has not yet been written to channel RAM. May be empty.
    // FIXME: Maybe store by reference &'a mut encoder?
    data: E,
}

impl<E> SingleShotTxTransaction<'_, E>
where
    E: Encoder,
{
    #[cfg_attr(place_rmt_driver_in_ram, ram)]
    fn poll_internal(&mut self) -> Option<Event> {
        let raw = self.raw;

        let status = raw.get_tx_status();
        if status == Some(Event::Threshold) {
            raw.reset_tx_threshold_set();

            if self.writer.state == WriterState::Active {
                // re-fill TX RAM
                self.writer.write(&mut self.data, raw);
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
    pub fn wait(mut self) -> Result<(), Error> {
        // Not sure that all the error cases below can happen. However, it's best to
        // handle them to be sure that we don't lock up here in case they can happen.
        let result = loop {
            match self.poll_internal() {
                Some(Event::Error) => break Err(Error::TransmissionError),
                Some(Event::End) => {
                    if self.writer.state == WriterState::Active {
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

        RmtState::store(RmtState::TxIdle, self.raw, Ordering::Relaxed);

        // Disable the Drop handler since the transaction is properly stopped
        // already.
        let _ = ManuallyDrop::new(self);
        result
    }
}

impl<E> Drop for SingleShotTxTransaction<'_, E>
where
    E: Encoder,
{
    fn drop(&mut self) {
        // If this is dropped, that implies that the transaction was not properly
        // `wait()`ed for. Thus, attempt to stop it as quickly as possible and
        // block in the meantime, such that subsequent uses of the channel are
        // safe (i.e. start from a state where the hardware is stopped).
        let raw = self.raw;

        let immediate = raw.stop_tx();
        raw.update();

        if !immediate {
            while !matches!(raw.get_tx_status(), Some(Event::Error | Event::End)) {}
        }

        RmtState::store(RmtState::TxIdle, raw, Ordering::Relaxed);
    }
}

/// An in-progress continuous TX transaction
pub struct ContinuousTxTransaction<'a> {
    raw: DynChannelAccess<Tx>,
    _phantom: PhantomData<&'a mut DynChannelAccess<Tx>>,
}

impl ContinuousTxTransaction<'_> {
    /// Stop transaction when the current iteration ends.
    #[cfg_attr(place_rmt_driver_in_ram, ram)]
    pub fn stop_next(self) -> Result<(), Error> {
        let raw = self.raw;

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
    #[cfg_attr(place_rmt_driver_in_ram, ram)]
    pub fn stop(self) -> Result<(), Error> {
        let raw = self.raw;

        raw.set_tx_continuous(false);
        let immediate = raw.stop_tx();
        raw.update();

        let result = if immediate {
            Ok(())
        } else {
            loop {
                match raw.get_tx_status() {
                    // FIXME: Is it really possible for an error to happen here?
                    Some(Event::Error) => break Err(Error::TransmissionError),
                    Some(Event::End) => break Ok(()),
                    _ => continue,
                }
            }
        };

        // Disable Drop handler since the transaction is stopped already.
        let _ = ManuallyDrop::new(self);
        result
    }

    /// Check if the `loopcount` interrupt bit is set
    pub fn is_loopcount_interrupt_set(&self) -> bool {
        self.raw.is_tx_loopcount_interrupt_set()
    }
}

impl Drop for ContinuousTxTransaction<'_> {
    fn drop(&mut self) {
        // If this is dropped, that implies that the transaction was not manually
        // stopped with `stop()` or `stop_next()`.
        // Thus, attempt to stop it as quickly as possible and block in the meantime,
        // such that subsequent uses of the channel are safe (i.e. start from a
        // state where the hardware is stopped).
        let raw = self.raw;

        raw.set_tx_continuous(false);
        let immediate = raw.stop_tx();
        raw.update();

        if !immediate {
            while !matches!(raw.get_tx_status(), Some(Event::Error | Event::End)) {}
        }
    }
}

/// Docs
#[derive(Clone, Debug, Default)]
pub struct BenchmarkResult {
    written: u64,
    duration_nanos: u64,
    iterations: u64,
    code_duration_nanos: u64,
}

impl core::fmt::Display for BenchmarkResult {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let duration = self.duration_nanos / self.iterations;
        let cpu_clock = Clocks::get().cpu_clock;
        writeln!(f, "RMT BenchmarkResult:")?;
        writeln!(f, "\tIterations: {}", self.iterations)?;
        writeln!(f, "\tCodes written: {}", self.written)?;
        writeln!(
            f,
            "\tEncoding time: {}us ({}% of {}us tx time)",
            duration / 1000,
            (100 * duration)
                .checked_div(self.code_duration_nanos)
                .unwrap_or(0),
            self.code_duration_nanos / 1000,
        )?;
        write!(
            f,
            "\tEncoding time / code: {}ns ~ {} cycles",
            duration / self.written,
            duration * cpu_clock.as_mhz() as u64 / (1000 * self.written),
        )?;

        Ok(())
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for BenchmarkResult {
    fn format(&self, fmt: defmt::Formatter<'_>) {
        let duration = self.duration_nanos / self.iterations;
        let cpu_clock = Clocks::get().cpu_clock;
        defmt::write!(fmt, "RMT BenchmarkResult {{\n");
        defmt::write!(fmt, "\tIterations: {}\n", self.iterations);
        defmt::write!(fmt, "\tCodes written: {}\n", self.written);
        defmt::write!(
            fmt,
            "\tEncoding time: {}us ({}% of {}us tx time)\n",
            duration / 1000,
            (100 * duration)
                .checked_div(self.code_duration_nanos)
                .unwrap_or(0),
            self.code_duration_nanos / 1000,
        );
        defmt::write!(
            fmt,
            "\tEncoding time / code: {}ns ~ {} cycles\n",
            duration / self.written,
            duration * cpu_clock.as_mhz() as u64 / (1000 * self.written),
        );
        defmt::write!(fmt, "}}");
    }
}

impl<Dm> Channel<'_, Dm, Tx>
where
    Dm: crate::DriverMode,
{
    /// FIXME: docs
    /// FIXME: Add an example of how to use these
    pub fn bench<T>(&mut self, data: &[T]) -> BenchmarkResult
    where
        T: Into<PulseCode> + Copy,
    {
        let mut total = 0;
        let mut total_length = 0;
        for code in data {
            let code = (*code).into();
            total += 1;
            // FIXME: What about the final stop code?
            if code.is_end_marker() {
                break;
            }
            total_length += code.length() as u64;
        }

        self.bench_inner(SliceEncoder::new(data), total_length, total)
    }

    /// FIXME: docs
    pub fn bench_iter<I>(&mut self, data: I) -> BenchmarkResult
    where
        I: IntoIterator<Item=PulseCode>,
        I::IntoIter: Clone,
    {
        let enc = IterEncoder::new(data);

        let mut total = 0;
        let mut total_length = 0;
        for code in enc.data.clone() {
            total += 1;
            // FIXME: What about the final stop code?
            if code.is_end_marker() {
                break;
            }
            total_length += code.length() as u64;
        }

        self.bench_inner(enc, total_length, total)
    }

    /// FIXME: docs
    pub fn bench_enc<E>(&mut self, data: E) -> BenchmarkResult
    where
        E: Encoder + Clone,
    {
        let raw = self.raw;

        let ram_start = raw.channel_ram_start();
        let mut ptr = ram_start;
        let memsize = raw.memsize().codes();
        let ram_end = unsafe { ram_start.add(memsize) };
        let mut code_length = 0;
        let mut read = 0;

        let mut encoder = data.clone();
        let mut writer = RmtWriterOuter::new();

        let mut sum_codes = |writer: &RmtWriterOuter| {
            // -1 to not count the artifial end markers
            while read < writer.written - 1 {
                let code = unsafe { ptr.read_volatile() };
                code_length += code.length() as u64;
                ptr = unsafe { ptr.add(1) };
                read += 1;
                if ptr == ram_end {
                    ptr = ram_start;
                }
            }
        };

        // FIXME: Add WriterState.is_done() for this check!
        // while !writer.is_done() {
        while writer.state == WriterState::Active {
            writer.write(&mut encoder, raw.degrade());
            sum_codes(&writer);
        }

        self.bench_inner(data, code_length, writer.written)
    }

    fn bench_inner<E>(&mut self, data: E, total_length: u64, total: usize) -> BenchmarkResult
    where
        E: Encoder + Clone,
    {
        let raw = self.raw;

        let clock = chip_specific::get_clock().as_hz() as u64;
        let div = raw.divider() as u64;
        let code_duration_nanos = total_length * div * 1_000_000_000 / clock;

        let mut iterations = 0;
        let start = Instant::now();
        let mut writer;
        loop {
            let mut encoder = data.clone();
            writer = RmtWriterOuter::new();

            // while !writer.is_done() {
            while writer.state == WriterState::Active {
                writer.write(&mut encoder, raw.degrade());
            }
            assert!(writer.written == total);
            iterations += 1;

            // Target >= 5ms
            if start.elapsed().as_micros() > 5_000 {
                break;
            }
        }
        let duration = start.elapsed();

        BenchmarkResult {
            written: writer.written as u64,
            duration_nanos: duration.as_micros() * 1000,
            iterations,
            code_duration_nanos,
        }
    }
}

/// Channel in TX mode
impl Channel<'_, Blocking, Tx> {
    /// Start transmitting the given pulse code sequence.
    /// This returns a [`SingleShotTxTransaction`] which can be used to wait for
    /// the transaction to complete and get back the channel for further
    /// use.
    #[cfg_attr(place_rmt_driver_in_ram, inline(always))]
    pub fn transmit<'a, T>(
        &mut self,
        data: &'a [T],
    ) -> Result<SingleShotTxTransaction<'_, SliceEncoder<'a, T>>, Error>
    where
        T: Into<PulseCode> + Copy,
    {
        self.transmit_enc(SliceEncoder::new(data))
    }

    /// FIXME: docs
    #[cfg_attr(place_rmt_driver_in_ram, inline(always))]
    pub fn transmit_iter<I>(
        &mut self,
        data: I,
    ) -> Result<SingleShotTxTransaction<'_, IterEncoder<I::IntoIter>>, Error>
    where
        I: IntoIterator<Item = PulseCode>,
    {
        self.transmit_enc(IterEncoder::new(data))
    }

    /// FIXME: docs
    #[cfg_attr(place_rmt_driver_in_ram, ram)]
    pub fn transmit_enc<E>(
        &mut self,
        mut data: E,
    ) -> Result<SingleShotTxTransaction<'_, E>, Error>
    where
        E: Encoder,
    {
        let raw = self.raw;

        let mut writer = RmtWriterOuter::new();
        writer.write(&mut data, raw);

        match writer.state {
            WriterState::Empty => return Err(Error::InvalidArgument),
            WriterState::DoneNoEnd => return Err(Error::EndMarkerMissing),
            // WriterState::DoneEarly => return Err(Error::UnexpectedEndMarker),
            _ => (),
        };

        RmtState::store(RmtState::TxBlocking, raw, Ordering::Relaxed);

        raw.clear_tx_interrupts();
        raw.start_send(false, None);

        Ok(SingleShotTxTransaction {
            raw,
            _phantom: PhantomData,
            writer,
            data,
        })
    }

    /// Start transmitting the given pulse code continuously.
    /// This returns a [`ContinuousTxTransaction`] which can be used to stop the
    /// ongoing transmission and get back the channel for further use.
    /// The length of sequence cannot exceed the size of the allocated RMT RAM.
    #[cfg_attr(place_rmt_driver_in_ram, inline(always))]
    pub fn transmit_continuously<T>(
        &mut self,
        loopcount: Option<NonZeroU16>,
        data: &'_ [T]
    ) -> Result<ContinuousTxTransaction<'_>, Error>
    where
        T: Into<PulseCode> + Copy,
    {
        self.transmit_continuously_enc(loopcount, SliceEncoder::new(data))
    }

    /// FIXME: docs
    #[cfg_attr(place_rmt_driver_in_ram, inline(always))]
    pub fn transmit_continuously_iter<I>(
        &mut self,
        loopcount: Option<NonZeroU16>,
        data: I,
    ) -> Result<ContinuousTxTransaction<'_>, Error>
    where
        I: IntoIterator<Item = PulseCode>,
    {
        self.transmit_continuously_enc(loopcount, IterEncoder::new(data))
    }

    /// FIXME: docs
    #[cfg_attr(place_rmt_driver_in_ram, ram)]
    pub fn transmit_continuously_enc<E>(
        &mut self,
        loopcount: Option<NonZeroU16>,
        mut data: E,
    ) -> Result<ContinuousTxTransaction<'_>, Error>
    where
        E: Encoder,
    {
        let raw = self.raw;

        let mut writer = RmtWriterOuter::new();
        writer.write(&mut data, raw);

        match writer.state {
            WriterState::Empty => return Err(Error::InvalidArgument),
            WriterState::Active => return Err(Error::Overflow),
            WriterState::DoneNoEnd => return Err(Error::EndMarkerMissing),
            // WriterState::DoneEarly => return Err(Error::UnexpectedEndMarker),
            WriterState::Done => (),
        };

        raw.start_send(true, loopcount);

        Ok(ContinuousTxTransaction { raw: self.raw, _phantom: PhantomData })
    }
}

/// RX transaction instance
#[must_use = "transactions need to be `poll()`ed / `wait()`ed for to ensure progress"]
pub struct RxTransaction<'ch, D>
where
    D: Decoder,
{
    raw: DynChannelAccess<Rx>,
    _phantom: PhantomData<&'ch mut DynChannelAccess<Rx>>,

    reader: RmtReaderOuter,

    data: D,
}

impl<D> RxTransaction<'_, D>
where
    D: Decoder,
{
    #[cfg_attr(place_rmt_driver_in_ram, ram)]
    fn poll_internal(&mut self) -> Option<Event> {
        let raw = self.raw;

        let status = raw.get_rx_status();
        match status {
            Some(Event::End) | Some(Event::Error) => {
                if self.reader.state != ReaderState::Done {
                    // Do not clear the interrupt flags here: Subsequent calls of wait() must
                    // be able to observe them if this is currently called via poll()
                    raw.stop_rx();
                    raw.update();

                    // TODO: Read at most up to the HW pointer!
                    self.reader.read(&mut self.data, raw, true);

                    // Ensure that no further data will be read if this is called repeatedly.
                    self.reader.state = ReaderState::Done;
                }
            }
            Some(Event::Threshold) if raw.supports_rx_wrap() => {
                raw.reset_rx_threshold_set();

                self.reader.read(&mut self.data, raw, false);
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
        match self.poll_internal() {
            Some(Event::Error | Event::End) => true,
            Some(Event::Threshold) | None => false,
        }
    }

    /// Wait for the transaction to complete
    #[cfg_attr(place_rmt_driver_in_ram, inline(always))]
    pub fn wait(mut self) -> Result<usize, Error> {
        let result = loop {
            match self.poll_internal() {
                Some(Event::Error) => break Err(Error::ReceiverError),
                Some(Event::End) => break Ok(self.reader.total),
                _ => continue,
            }
        };

        let raw = self.raw;

        raw.clear_rx_interrupts();
        RmtState::store(RmtState::RxIdle, raw, Ordering::Relaxed);

        // Disable Drop handler since the receiver is stopped already.
        let _ = ManuallyDrop::new(self);
        result
    }
}

impl<D> Drop for RxTransaction<'_, D>
where
    D: Decoder,
{
    fn drop(&mut self) {
        // If this is dropped, that implies that the transaction was not properly
        // `wait()`ed for. Thus, attempt to stop it as quickly as possible and
        // block in the meantime, such that subsequent uses of the channel are
        // safe (i.e. start from a state where the hardware is stopped).
        let raw = self.raw;

        // STATE should be RxIdle if the transaction was polled to completion
        if RmtState::load(raw, Ordering::Relaxed) == RmtState::RxBlocking {
            raw.stop_rx();
            raw.update();

            // block until the channel is safe to use again
            // -> Actually, stop_rx() appears to be immediate and does not update state
            // flags, so this would lock up!
            // while !matches!(raw.get_rx_status(), Some(Event::Error | Event::End)) {}

            raw.clear_rx_interrupts();
            RmtState::store(RmtState::RxIdle, raw, Ordering::Relaxed);
        }
    }
}

/// Channel is RX mode
impl Channel<'_, Blocking, Rx> {
    /// Start receiving pulse codes into the given buffer.
    /// This returns a [RxTransaction] which can be used to wait for receive to
    /// complete and get back the channel for further use.
    #[cfg_attr(place_rmt_driver_in_ram, inline(always))]
    pub fn receive<'ch, 'a, T>(
        &'ch mut self,
        data: &'a mut [T],
    ) -> Result<RxTransaction<'ch, SliceDecoder<'a, T>>, Error>
    where
        T: From<PulseCode> + 'a
    {
        self.receive_dec(SliceDecoder::new(data))
    }

    /// FIXME: docs
    #[cfg_attr(place_rmt_driver_in_ram, inline(always))]
    pub fn receive_iter<'ch, 'a, D, T>(
        &'ch mut self,
        data: D,
    ) -> Result<RxTransaction<'ch, IterDecoder<'a, D::IntoIter, T>>, Error>
    where
        D: IntoIterator<Item = &'a mut T>,
        T: From<PulseCode> + 'a
    {
        self.receive_dec(IterDecoder::new(data))
    }

    /// FIXME: Docs
    #[cfg_attr(place_rmt_driver_in_ram, ram)]
    pub fn receive_dec<D>(&mut self, data: D) -> Result<RxTransaction<'_, D>, Error>
    where
        D: Decoder,
    {
        let raw = self.raw;

        let reader = RmtReaderOuter::new();

        RmtState::store(RmtState::RxBlocking, raw, Ordering::Relaxed);

        raw.clear_rx_interrupts();
        raw.start_receive(true);

        Ok(RxTransaction {
            raw,
            _phantom: PhantomData,
            reader,
            data,
        })
    }
}

static WAKER: [AtomicWaker; NUM_CHANNELS] = [const { AtomicWaker::new() }; NUM_CHANNELS];

// FIXME: This is essentially the same as SingleShotTxTransaction. Is it
// possible to share most of the code?
#[must_use = "futures do nothing unless you `.await` or poll them"]
struct RmtTxFuture<'a, E>
where
    E: Encoder,
{
    raw: DynChannelAccess<Tx>,
    _phantom: PhantomData<(&'a mut DynChannelAccess<Tx>, &'a mut E)>,

    writer: RmtWriterOuter,

    // Remaining data that has not yet been written to channel RAM. May be empty.
    data: E,
}

impl<E> Future for RmtTxFuture<'_, E>
where
    E: Encoder + Unpin,
{
    type Output = Result<(), Error>;

    #[cfg_attr(place_rmt_driver_in_ram, ram)]
    fn poll(self: Pin<&mut Self>, ctx: &mut Context<'_>) -> Poll<Self::Output> {
        // Note that STATE is only accessed from a single core and no ISR here, so
        // Relaxed access is sufficient.
        let this = self.get_mut();
        let raw = this.raw;

        match this.writer.state {
            WriterState::Empty => {
                // In this case, tx was never started
                return Poll::Ready(Err(Error::InvalidArgument));
            }
            WriterState::DoneNoEnd => {
                // In this case, either tx was never started or it was stopped already below in
                // a previous call to poll.
                return Poll::Ready(Err(Error::EndMarkerMissing));
            }
            // WriterState::DoneEarly => {
            //     return Poll::Ready(Err(Error::UnexpectedEndMarker));
            // }
            _ => (),
        }

        WAKER[raw.channel() as usize].register(ctx.waker());

        let result = match raw.get_tx_status() {
            Some(Event::Error) => Err(Error::TransmissionError),
            Some(Event::End) => {
                if this.writer.state == WriterState::Active {
                    // Unexpectedly done, even though we have data left.
                    Err(Error::UnexpectedEndMarker)
                } else {
                    Ok(())
                }
            }
            Some(Event::Threshold) => {
                raw.reset_tx_threshold_set();

                if this.writer.state == WriterState::Active {
                    // re-fill TX RAM
                    this.writer.write(&mut this.data, raw);
                    // FIXME: Return if writer.state indicates an error (ensure
                    // to stop transmission first)
                }

                match this.writer.state {
                    WriterState::Active => {
                        raw.listen_tx_interrupt(Event::Threshold);
                    }
                    WriterState::DoneNoEnd => {
                        // We have written an extra end marker, so will stop
                        // automatically after
                        // all data has been transmitted. If we stop tx manually
                        // here, this will prevent the
                        // end interrupt from triggering, and we will lock up
                        // polling. raw.stop_tx();
                    }
                    _ => (),
                }

                return Poll::Pending;
            }
            _ => return Poll::Pending,
        };

        raw.unlisten_tx_interrupt(Event::Error | Event::End | Event::Threshold);
        raw.clear_tx_interrupts();
        RmtState::store(RmtState::TxIdle, raw, Ordering::Relaxed);

        Poll::Ready(result)
    }
}

impl<E> Drop for RmtTxFuture<'_, E>
where
    E: Encoder,
{
    fn drop(&mut self) {
        let raw = self.raw;

        // STATE should be TxIdle if the future was polled to completion
        if RmtState::load(raw, Ordering::Relaxed) == RmtState::TxAsync {
            raw.unlisten_tx_interrupt(Event::Error | Event::End | Event::Threshold);
            let immediate = raw.stop_tx();
            raw.update();

            // block until the channel is safe to use again
            if !immediate {
                while !matches!(raw.get_tx_status(), Some(Event::Error | Event::End)) {}
            }

            raw.clear_tx_interrupts();
            RmtState::store(RmtState::TxIdle, raw, Ordering::Relaxed);
        }
    }
}

/// TX channel in async mode
impl Channel<'_, Async, Tx> {
    /// Start transmitting the given pulse code sequence.
    /// The length of sequence cannot exceed the size of the allocated RMT
    /// RAM.
    #[cfg_attr(place_rmt_driver_in_ram, inline(always))]
    pub async fn transmit<T>(&mut self, data: &[T]) -> Result<(), Error>
    where
        Self: Sized,
        T: Into<PulseCode> + Copy,
    {
        self.transmit_inner(SliceEncoder::new(data)).await
    }

    /// FIXME: docs
    #[cfg_attr(place_rmt_driver_in_ram, inline(always))]
    pub async fn transmit_iter<I>(&mut self, data: I) -> Result<(), Error>
    where
        Self: Sized,
        I: IntoIterator<Item=PulseCode>,
        I::IntoIter: Unpin,
    {
        self.transmit_inner(IterEncoder::new(data)).await
    }

    /// FIXME: docs
    #[cfg_attr(place_rmt_driver_in_ram, inline(always))]
    pub async fn transmit_enc<'a, E>(&'a mut self, data: E) -> Result<(), Error>
    where
        Self: Sized,
        E: Encoder + Unpin + 'a
    {
        self.transmit_inner(data).await
    }

    #[cfg_attr(place_rmt_driver_in_ram, ram)]
    fn transmit_inner<'a, E>(&'a mut self, data: E) -> impl Future<Output = Result<(), Error>>
    where
        Self: Sized,
        E: Encoder + Unpin + 'a,
        // FIXME: Is being Unpin a significant restriction for the iterator?
        // If so, we could use it in a pinned way by using pin-projection (e.g. via
        // pin-project-lite) for the RmtTxFuture.
    {
        let raw = self.raw;

        let mut fut = RmtTxFuture {
            raw,
            _phantom: PhantomData,
            writer: RmtWriterOuter::new(),
            data,
        };

        fut.writer.write(&mut fut.data, raw);

        // Error cases; the future will return the error on the first call to poll()
        if matches!(fut.writer.state, WriterState::Empty | WriterState::DoneNoEnd) {
            return fut;
        }

        RmtState::store(RmtState::TxAsync, fut.raw, Ordering::Relaxed);

        fut.raw.clear_tx_interrupts();
        let mut events = Event::End | Event::Error;
        if fut.writer.state == WriterState::Active {
            // Needs wrapping
            events |= Event::Threshold;
        }
        fut.raw.listen_tx_interrupt(events);
        fut.raw.start_send(false, None);

        fut
    }
}

#[must_use = "futures do nothing unless you `.await` or poll them"]
struct RmtRxFuture<'a, D>
where
    D: Decoder + Unpin,
{
    raw: DynChannelAccess<Rx>,
    _phantom: PhantomData<&'a mut DynChannelAccess<Rx>>,

    reader: RmtReaderOuter,

    data: D,
}

impl<'a, D> Future for RmtRxFuture<'a, D>
where
    D: Decoder + Unpin,
{
    type Output = Result<usize, Error>;

    #[cfg_attr(place_rmt_driver_in_ram, ram)]
    fn poll(self: Pin<&mut Self>, ctx: &mut Context<'_>) -> Poll<Self::Output> {
        let this = self.get_mut();
        let raw = this.raw;

        WAKER[raw.channel() as usize].register(ctx.waker());

        let result = match raw.get_rx_status() {
            Some(Event::End) | Some(Event::Error) => {
                raw.stop_rx();
                raw.update();

                // TODO: Read at most up to the HW pointer!
                this.reader.read(&mut this.data, raw, true);

                Ok(this.reader.total)
            }
            Some(Event::Threshold) if raw.supports_rx_wrap() => {
                raw.reset_rx_threshold_set();

                if this.reader.state == ReaderState::Active {
                    this.reader.read(&mut this.data, raw, false);
                    // FIXME: Return if reader.state indicates an error (ensure
                    // to stop rx first)
                }
                if this.reader.state == ReaderState::Active {
                    raw.listen_rx_interrupt(Event::Threshold);
                }

                return Poll::Pending;
            }
            _ => return Poll::Pending,
        };

        raw.unlisten_rx_interrupt(Event::Error | Event::End | Event::Threshold);
        raw.clear_rx_interrupts();
        RmtState::store(RmtState::RxIdle, raw, Ordering::Relaxed);

        Poll::Ready(result)
    }
}

impl<'a, D> Drop for RmtRxFuture<'a, D>
where
    D: Decoder + Unpin,
{
    fn drop(&mut self) {
        let raw = self.raw;

        // STATE should be RxIdle if the future was polled to completion
        if RmtState::load(raw, Ordering::Relaxed) == RmtState::RxAsync {
            raw.stop_rx();
            raw.update();

            // block until the channel is safe to use again
            // -> Actually, stop_rx() appears to be immediate and does not update state
            // flags, so this would lock up!
            // while !matches!(raw.get_rx_status(), Some(Event::Error | Event::End)) {}

            raw.clear_rx_interrupts();
            RmtState::store(RmtState::RxIdle, raw, Ordering::Relaxed);
        }
    }
}

/// RX channel in async mode
impl Channel<'_, Async, Rx> {
    /// Start receiving a pulse code sequence.
    /// The length of sequence cannot exceed the size of the allocated RMT
    /// RAM.
    #[cfg_attr(place_rmt_driver_in_ram, inline(always))]
    pub async fn receive<'a, T>(&'a mut self, data: &'a mut [T]) -> Result<usize, Error>
    where
        Self: Sized,
        T: From<PulseCode> + 'a
    {
        self.receive_inner(SliceDecoder::new(data)).await
    }

    /// FIXME: docs
    #[cfg_attr(place_rmt_driver_in_ram, inline(always))]
    pub async fn receive_iter<'a, D, T>(&'a mut self, data: D) -> Result<usize, Error>
    where
        Self: Sized,
        D: IntoIterator<Item = &'a mut T>,
        D::IntoIter: Unpin,
        T: From<PulseCode> + 'a
    {
        self.receive_inner(IterDecoder::new(data)).await
    }

    /// FIXME: docs
    #[cfg_attr(place_rmt_driver_in_ram, inline(always))]
    pub async fn receive_dec<'a, D>(&'a mut self, data: D) -> Result<usize, Error>
    where
        Self: Sized,
        D: Decoder + Unpin
    {
        self.receive_inner(data).await
    }

    #[cfg_attr(place_rmt_driver_in_ram, ram)]
    fn receive_inner<'a, D>(&'a mut self, data: D) -> impl Future<Output = Result<usize, Error>>
    where
        Self: Sized,
        D: Decoder + Unpin,
    {
        let raw = self.raw;

        let reader = RmtReaderOuter::new();

        RmtState::store(RmtState::RxAsync, raw, Ordering::Relaxed);

        raw.clear_rx_interrupts();
        raw.listen_rx_interrupt(Event::End | Event::Error | Event::Threshold);
        raw.start_receive(true);

        RmtRxFuture {
            raw,
            _phantom: PhantomData,
            reader,
            data,
        }
    }
}

// #[cfg(any(esp32, esp32s2))]
#[handler]
fn async_interrupt_handler() {
    fn on_tx(raw: DynChannelAccess<Tx>, event: Event) {
        let events_to_unlisten = match event {
            Event::End | Event::Error => Event::End | Event::Error | Event::Threshold,
            // The RmtTxFuture will wnable the threshold interrupt again if required
            Event::Threshold => Event::Threshold.into(),
        };
        raw.unlisten_tx_interrupt(events_to_unlisten);

        WAKER[raw.channel() as usize].wake();
    }

    fn on_rx(raw: DynChannelAccess<Rx>, event: Event) {
        let events_to_unlisten = match event {
            Event::End | Event::Error => Event::End | Event::Error | Event::Threshold,
            // The RmtRxFuture will wnable the threshold interrupt again if required
            Event::Threshold => Event::Threshold.into(),
        };
        raw.unlisten_rx_interrupt(events_to_unlisten);

        WAKER[raw.channel() as usize].wake();
    }

    chip_specific::handle_channel_interrupts(on_tx, on_rx);
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
pub trait ChannelInternal: RawChannelAccess<Dir: Direction> {
    fn update(&self);

    fn divider(&self) -> u8;

    fn set_divider(&self, divider: u8);

    fn memsize(&self) -> MemSize;

    fn set_memsize(&self, value: MemSize);

    #[inline]
    fn channel_ram_start(&self) -> *mut PulseCode {
        unsafe {
            (property!("rmt.ram_start") as *mut PulseCode)
                .add(usize::from(self.channel()) * property!("rmt.channel_ram_size"))
        }
    }
}

#[doc(hidden)]
pub trait TxChannelInternal: ChannelInternal + RawChannelAccess<Dir = Tx> {
    fn output_signal(&self) -> gpio::OutputSignal {
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

    fn reset_tx_threshold_set(&self);

    fn set_tx_threshold(&self, threshold: u8);

    fn is_tx_loopcount_interrupt_set(&self) -> bool;

    #[inline]
    fn start_send(&self, continuous: bool, repeat: Option<NonZeroU16>) {
        self.clear_tx_interrupts();

        self.set_tx_threshold((self.memsize().codes() / 2) as u8);
        self.set_tx_continuous(continuous);
        self.set_generate_repeat_interrupt(repeat.map_or(0, |r| r.get()));
        self.set_tx_wrap_mode(true);
        self.update();
        self.start_tx();
        self.update();
    }

    // Return whether stopping was immediate, or needs to wait for tx end
    fn stop_tx(&self) -> bool;

    fn set_tx_interrupt(&self, event: EnumSet<Event>, enable: bool);

    #[inline]
    fn listen_tx_interrupt(&self, event: impl Into<EnumSet<Event>>) {
        self.set_tx_interrupt(event.into(), true);
    }

    #[inline]
    fn unlisten_tx_interrupt(&self, event: impl Into<EnumSet<Event>>) {
        self.set_tx_interrupt(event.into(), false);
    }
}

#[doc(hidden)]
pub trait RxChannelInternal: ChannelInternal + RawChannelAccess<Dir = Rx> {
    fn input_signal(&self) -> gpio::InputSignal {
        INPUT_SIGNALS[self.channel() as usize]
    }

    fn clear_rx_interrupts(&self);

    fn supports_rx_wrap(&self) -> bool;

    fn set_rx_wrap_mode(&self, wrap: bool);

    fn set_rx_carrier(&self, carrier: bool, high: u16, low: u16, level: Level);

    fn start_rx(&self);

    // Return the first flag that is set of, in order of decreasing priority,
    // Event::Error, Event::End, Event::Threshold
    fn get_rx_status(&self) -> Option<Event>;

    fn is_rx_threshold_set(&self) -> bool;

    fn reset_rx_threshold_set(&self);

    fn set_rx_threshold(&self, threshold: u8);

    fn start_receive(&self, wrap: bool) {
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

    #[inline]
    fn listen_rx_interrupt(&self, event: impl Into<EnumSet<Event>>) {
        self.set_rx_interrupt(event.into(), true);
    }

    #[inline]
    fn unlisten_rx_interrupt(&self, event: impl Into<EnumSet<Event>>) {
        self.set_rx_interrupt(event.into(), false);
    }
}

#[cfg(not(any(esp32, esp32s2)))]
mod chip_specific {
    use enumset::EnumSet;

    use super::{
        Capability,
        ChannelIndex,
        ChannelInternal,
        Direction,
        DynChannelAccess,
        Error,
        Event,
        Level,
        MemSize,
        NUM_CHANNELS,
        RawChannelAccess,
        RawChannelAccessExt,
        Rx,
        RxChannelInternal,
        Tx,
        TxChannelInternal,
    };
    use crate::{gpio, peripherals::RMT, time::Rate};

    pub(super) fn configure_clock(frequency: Rate) -> Result<(), Error> {
        let src_clock = crate::soc::constants::RMT_CLOCK_SRC_FREQ;

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
                w.sclk_sel().bits(crate::soc::constants::RMT_CLOCK_SRC);
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

    pub fn get_clock() -> Rate {
        let src_clock = crate::soc::constants::RMT_CLOCK_SRC_FREQ;

        let div: u8;

        #[cfg(not(soc_has_pcr))]
        {
            let r = RMT::regs().sys_conf().read();

            div = r.sclk_div_num().bits();

            assert_eq!(0, r.sclk_div_a().bits());
            assert_eq!(0, r.sclk_div_b().bits());
        }

        #[cfg(soc_has_pcr)]
        {
            use crate::peripherals::PCR;
            let r = PCR::regs().rmt_sclk_conf().read();

            div = r.sclk_div_num().bits();

            assert_eq!(0, r.sclk_div_a().bits());
            assert_eq!(0, r.sclk_div_b().bits());
        }

        Rate::from_hz(src_clock.as_hz() / (div as u32 + 1))
    }

    #[allow(unused)]
    #[inline]
    pub(super) fn handle_channel_interrupts(
        on_tx: fn(DynChannelAccess<Tx>, Event),
        on_rx: fn(DynChannelAccess<Rx>, Event),
    ) {
        let st = RMT::regs().int_st().read();

        for ch_idx in ChannelIndex::iter_all() {
            let event = if st.ch_tx_err(ch_idx as u8).bit() {
                Event::Error
            } else if st.ch_tx_end(ch_idx as u8).bit() {
                Event::End
            } else if st.ch_tx_thr_event(ch_idx as u8).bit() {
                Event::Threshold
            } else {
                continue;
            };

            let raw = unsafe { DynChannelAccess::<Tx>::conjure(ch_idx) };
            on_tx(raw, event);
        }

        for ch_idx in ChannelIndex::iter_all() {
            let event = if st.ch_rx_err(ch_idx as u8).bit() {
                Event::Error
            } else if st.ch_rx_end(ch_idx as u8).bit() {
                Event::End
            } else if st.ch_rx_thr_event(ch_idx as u8).bit() {
                Event::Threshold
            } else {
                continue;
            };

            let raw = unsafe { DynChannelAccess::<Rx>::conjure(ch_idx) };
            on_rx(raw, event);
        }
    }

    #[inline]
    pub(super) const fn channel_for_idx<Dir: Capability>(idx: ChannelIndex) -> u8 {
        if Dir::SUPPORTS_TX {
            idx as u8
        } else {
            idx as u8 + NUM_CHANNELS as u8 / 2
        }
    }

    impl<A> ChannelInternal for A
    where
        A: RawChannelAccess<Dir: Direction>,
    {
        #[inline]
        fn update(&self) {
            let rmt = crate::peripherals::RMT::regs();
            let ch_idx = self.ch_idx() as usize;

            if A::Dir::is_tx() {
                rmt.ch_tx_conf0(ch_idx)
                    .modify(|_, w| w.conf_update().set_bit());
            } else {
                rmt.ch_rx_conf1(ch_idx)
                    .modify(|_, w| w.conf_update().set_bit());
            }
        }

        fn divider(&self) -> u8 {
            let rmt = crate::peripherals::RMT::regs();
            let ch_idx = self.ch_idx() as usize;

            if A::Dir::is_tx() {
                rmt.ch_tx_conf0(ch_idx).read().div_cnt().bits()
            } else {
                rmt.ch_rx_conf0(ch_idx).read().div_cnt().bits()
            }
        }

        fn set_divider(&self, divider: u8) {
            let rmt = crate::peripherals::RMT::regs();
            let ch_idx = self.ch_idx() as usize;

            if A::Dir::is_tx() {
                rmt.ch_tx_conf0(ch_idx)
                    .modify(|_, w| unsafe { w.div_cnt().bits(divider) });
            } else {
                rmt.ch_rx_conf0(ch_idx)
                    .modify(|_, w| unsafe { w.div_cnt().bits(divider) });
            }
        }

        #[inline]
        fn memsize(&self) -> MemSize {
            let rmt = RMT::regs();
            let ch_idx = self.ch_idx() as usize;

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
            let ch_idx = self.ch_idx() as usize;

            if A::Dir::is_tx() {
                rmt.ch_tx_conf0(ch_idx)
                    .modify(|_, w| unsafe { w.mem_size().bits(blocks) });
            } else {
                rmt.ch_rx_conf0(ch_idx)
                    .modify(|_, w| unsafe { w.mem_size().bits(blocks) });
            }
        }
    }

    impl<A> TxChannelInternal for A
    where
        A: RawChannelAccess<Dir = Tx>,
    {
        #[inline]
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

        #[inline]
        fn clear_tx_interrupts(&self) {
            let rmt = crate::peripherals::RMT::regs();

            rmt.int_clr().write(|w| {
                w.ch_tx_end(self.channel()).set_bit();
                w.ch_tx_err(self.channel()).set_bit();
                w.ch_tx_loop(self.channel()).set_bit();
                w.ch_tx_thr_event(self.channel()).set_bit()
            });
        }

        #[inline]
        fn set_tx_continuous(&self, continuous: bool) {
            let rmt = crate::peripherals::RMT::regs();

            rmt.ch_tx_conf0(self.channel().into())
                .modify(|_, w| w.tx_conti_mode().bit(continuous));
        }

        #[inline]
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

        #[inline]
        fn start_tx(&self) {
            let rmt = crate::peripherals::RMT::regs();

            rmt.ch_tx_conf0(self.channel().into()).modify(|_, w| {
                w.mem_rd_rst().set_bit();
                w.apb_mem_rst().set_bit();
                w.tx_start().set_bit()
            });
        }

        // Inlining this should allow significant optimization at the call site.
        #[inline]
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

        #[inline]
        fn reset_tx_threshold_set(&self) {
            let rmt = crate::peripherals::RMT::regs();
            rmt.int_clr()
                .write(|w| w.ch_tx_thr_event(self.channel()).set_bit());
        }

        #[inline]
        fn set_tx_threshold(&self, threshold: u8) {
            let rmt = crate::peripherals::RMT::regs();
            rmt.ch_tx_lim(self.channel().into())
                .modify(|_, w| unsafe { w.tx_lim().bits(threshold as u16) });
        }

        #[inline]
        fn is_tx_loopcount_interrupt_set(&self) -> bool {
            let rmt = crate::peripherals::RMT::regs();
            rmt.int_raw().read().ch_tx_loop(self.channel()).bit()
        }

        #[inline]
        fn stop_tx(&self) -> bool {
            let rmt = crate::peripherals::RMT::regs();
            rmt.ch_tx_conf0(self.channel().into())
                .modify(|_, w| w.tx_stop().set_bit());
            true
        }

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
        fn input_signal(&self) -> gpio::InputSignal {
            let ch_idx = self.ch_idx() as usize;
            super::INPUT_SIGNALS[ch_idx]
        }

        #[inline]
        fn clear_rx_interrupts(&self) {
            let rmt = crate::peripherals::RMT::regs();
            let ch_idx = self.ch_idx();

            rmt.int_clr().write(|w| {
                w.ch_rx_end(ch_idx).set_bit();
                w.ch_rx_err(ch_idx).set_bit();
                w.ch_rx_thr_event(ch_idx).set_bit()
            });
        }

        #[inline]
        fn supports_rx_wrap(&self) -> bool {
            true
        }

        #[inline]
        fn set_rx_wrap_mode(&self, wrap: bool) {
            let rmt = crate::peripherals::RMT::regs();
            let ch_idx = self.ch_idx() as usize;

            rmt.ch_rx_conf1(ch_idx)
                .modify(|_, w| w.mem_rx_wrap_en().bit(wrap));
        }

        fn set_rx_carrier(&self, carrier: bool, high: u16, low: u16, level: Level) {
            let ch_idx = self.ch_idx() as usize;
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
        fn start_rx(&self) {
            let rmt = crate::peripherals::RMT::regs();
            let ch_idx = self.ch_idx();

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

        // Inlining this should allow significant optimization at the call site.
        #[inline]
        fn get_rx_status(&self) -> Option<Event> {
            let rmt = crate::peripherals::RMT::regs();
            let reg = rmt.int_raw().read();
            let ch_idx = self.ch_idx();

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
        fn is_rx_threshold_set(&self) -> bool {
            let rmt = crate::peripherals::RMT::regs();
            let ch_idx = self.ch_idx();
            rmt.int_raw().read().ch_rx_thr_event(ch_idx).bit()
        }

        #[inline]
        fn reset_rx_threshold_set(&self) {
            let rmt = crate::peripherals::RMT::regs();
            let ch_idx = self.ch_idx();
            rmt.int_clr().write(|w| w.ch_rx_thr_event(ch_idx).set_bit());
        }

        #[inline]
        fn set_rx_threshold(&self, threshold: u8) {
            let rmt = crate::peripherals::RMT::regs();
            let ch_idx = self.ch_idx() as usize;
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

        #[inline]
        fn stop_rx(&self) {
            let rmt = crate::peripherals::RMT::regs();
            let ch_idx = self.ch_idx() as usize;
            rmt.ch_rx_conf1(ch_idx).modify(|_, w| w.rx_en().clear_bit());
        }

        fn set_rx_filter_threshold(&self, value: u8) {
            let rmt = crate::peripherals::RMT::regs();
            let ch_idx = self.ch_idx() as usize;

            rmt.ch_rx_conf1(ch_idx).modify(|_, w| unsafe {
                w.rx_filter_en().bit(value > 0);
                w.rx_filter_thres().bits(value)
            });
        }

        fn set_rx_idle_threshold(&self, value: u16) {
            let rmt = crate::peripherals::RMT::regs();
            let ch_idx = self.ch_idx() as usize;

            rmt.ch_rx_conf0(ch_idx)
                .modify(|_, w| unsafe { w.idle_thres().bits(value) });
        }

        #[inline]
        fn set_rx_interrupt(&self, events: EnumSet<Event>, enable: bool) {
            let rmt = crate::peripherals::RMT::regs();
            let ch_idx = self.ch_idx();

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
        Capability,
        ChannelIndex,
        ChannelInternal,
        Direction,
        DynChannelAccess,
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

    pub(super) fn configure_clock(frequency: Rate) -> Result<(), Error> {
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

    pub fn get_clock() -> Rate {
        Rate::from_mhz(80)
    }

    #[allow(unused)]
    #[inline]
    pub(super) fn handle_channel_interrupts(
        on_tx: fn(DynChannelAccess<Tx>, Event),
        on_rx: fn(DynChannelAccess<Rx>, Event),
    ) {
        let st = RMT::regs().int_st().read();

        for ch_idx in ChannelIndex::iter_all() {
            let (is_tx, event) = if st.ch_err(ch_idx as u8).bit() {
                // FIXME: Is it really necessary to determine tx/rx here?
                // Ultimately, we just need to signal the waker, so it doesn't really matter.
                if let Some(is_tx) = RmtState::load_by_idx(ch_idx as u8, Ordering::Relaxed).is_tx() {
                    (is_tx, Event::Error)
                } else {
                    // Shouldn't happen: The channel isn't configured for rx or tx, but an error
                    // interrupt occured. Ignore it.
                    continue;
                }
            } else if st.ch_tx_end(ch_idx as u8).bit() {
                (true, Event::End)
            } else if st.ch_rx_end(ch_idx as u8).bit() {
                (false, Event::End)
            } else if st.ch_tx_thr_event(ch_idx as u8).bit() {
                (true, Event::Threshold)
            } else {
                continue;
            };

            if is_tx {
                let raw = unsafe { DynChannelAccess::<Tx>::conjure(ch_idx) };
                on_tx(raw, event);
            } else {
                let raw = unsafe { DynChannelAccess::<Rx>::conjure(ch_idx) };
                on_rx(raw, event);
            }
        }
    }

    #[inline]
    #[allow(clippy::extra_unused_type_parameters)]
    pub(super) const fn channel_for_idx<Dir: Capability>(idx: ChannelIndex) -> u8 {
        idx as u8
    }

    impl<A> ChannelInternal for A
    where
        A: RawChannelAccess<Dir: Direction>,
    {
        #[inline]
        fn update(&self) {
            // no-op
        }

        fn divider(&self) -> u8 {
            let rmt = crate::peripherals::RMT::regs();
            rmt.chconf0(self.ch_idx() as usize).read().div_cnt().bits()
        }

        fn set_divider(&self, divider: u8) {
            let rmt = crate::peripherals::RMT::regs();
            rmt.chconf0(self.ch_idx() as usize)
                .modify(|_, w| unsafe { w.div_cnt().bits(divider) });
        }

        #[inline]
        fn memsize(&self) -> MemSize {
            let rmt = crate::peripherals::RMT::regs();
            let blocks = rmt.chconf0(self.ch_idx() as usize).read().mem_size().bits();
            MemSize::from_blocks(blocks)
        }

        fn set_memsize(&self, value: MemSize) {
            let rmt = crate::peripherals::RMT::regs();
            rmt.chconf0(self.ch_idx() as usize)
                .modify(|_, w| unsafe { w.mem_size().bits(value.blocks()) });
        }
    }

    impl<A> TxChannelInternal for A
    where
        A: RawChannelAccess<Dir = Tx>,
    {
        #[cfg(not(esp32))]
        #[inline]
        fn set_generate_repeat_interrupt(&self, repeats: u16) {
            let rmt = crate::peripherals::RMT::regs();
            if repeats > 1 {
                rmt.ch_tx_lim(self.ch_idx() as usize)
                    .modify(|_, w| unsafe { w.tx_loop_num().bits(repeats) });
            } else {
                rmt.ch_tx_lim(self.ch_idx() as usize)
                    .modify(|_, w| unsafe { w.tx_loop_num().bits(0) });
            }
        }

        #[cfg(esp32)]
        #[inline]
        fn set_generate_repeat_interrupt(&self, _repeats: u16) {
            // unsupported
        }

        #[inline]
        fn clear_tx_interrupts(&self) {
            let rmt = crate::peripherals::RMT::regs();
            let ch = self.ch_idx();

            rmt.int_clr().write(|w| {
                w.ch_err(ch).set_bit();
                w.ch_tx_end(ch).set_bit();
                w.ch_tx_thr_event(ch).set_bit()
            });
        }

        #[inline]
        fn set_tx_continuous(&self, continuous: bool) {
            let rmt = crate::peripherals::RMT::regs();

            rmt.chconf1(self.ch_idx() as usize)
                .modify(|_, w| w.tx_conti_mode().bit(continuous));
        }

        #[inline]
        fn set_tx_wrap_mode(&self, wrap: bool) {
            let rmt = crate::peripherals::RMT::regs();
            // this is "okay", because we use all TX channels always in wrap mode
            rmt.apb_conf().modify(|_, w| w.mem_tx_wrap_en().bit(wrap));
        }

        fn set_tx_carrier(&self, carrier: bool, high: u16, low: u16, level: Level) {
            let rmt = crate::peripherals::RMT::regs();
            let ch = self.ch_idx() as usize;

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
            rmt.chconf1(self.ch_idx() as usize)
                .modify(|_, w| w.idle_out_en().bit(enable).idle_out_lv().bit(level.into()));
        }

        #[inline]
        fn start_tx(&self) {
            let rmt = crate::peripherals::RMT::regs();
            let ch = self.ch_idx();

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

        // Inlining this should allow significant optimization at the call site.
        #[inline]
        fn get_tx_status(&self) -> Option<Event> {
            let rmt = crate::peripherals::RMT::regs();
            let reg = rmt.int_raw().read();
            let ch = self.ch_idx();

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
        fn reset_tx_threshold_set(&self) {
            let rmt = crate::peripherals::RMT::regs();
            rmt.int_clr()
                .write(|w| w.ch_tx_thr_event(self.ch_idx()).set_bit());
        }

        #[inline]
        fn set_tx_threshold(&self, threshold: u8) {
            let rmt = crate::peripherals::RMT::regs();
            rmt.ch_tx_lim(self.ch_idx() as usize)
                .modify(|_, w| unsafe { w.tx_lim().bits(threshold as u16) });
        }

        #[inline]
        fn is_tx_loopcount_interrupt_set(&self) -> bool {
            // no-op
            false
        }

        #[cfg(esp32s2)]
        #[inline]
        fn stop_tx(&self) -> bool {
            let rmt = crate::peripherals::RMT::regs();
            rmt.chconf1(self.ch_idx() as usize)
                .modify(|_, w| w.tx_stop().set_bit());
            true
        }

        #[cfg(esp32)]
        #[inline]
        fn stop_tx(&self) -> bool {
            let ptr = self.channel_ram_start();
            for idx in 0..self.memsize().codes() {
                unsafe {
                    ptr.add(idx).write_volatile(super::PulseCode::end_marker());
                }
            }
            false
        }

        #[inline]
        fn set_tx_interrupt(&self, events: EnumSet<Event>, enable: bool) {
            let rmt = crate::peripherals::RMT::regs();
            let ch = self.ch_idx();
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
        #[inline]
        fn clear_rx_interrupts(&self) {
            let rmt = crate::peripherals::RMT::regs();
            let ch = self.ch_idx();

            rmt.int_clr().write(|w| {
                w.ch_rx_end(ch).set_bit();
                w.ch_err(ch).set_bit();
                w.ch_tx_thr_event(ch).set_bit()
            });
        }

        #[inline]
        fn supports_rx_wrap(&self) -> bool {
            false
        }

        #[inline]
        fn set_rx_wrap_mode(&self, _wrap: bool) {
            // no-op
        }

        fn set_rx_carrier(&self, carrier: bool, high: u16, low: u16, level: Level) {
            let rmt = crate::peripherals::RMT::regs();
            let ch = self.ch_idx() as usize;

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
        fn start_rx(&self) {
            let rmt = crate::peripherals::RMT::regs();
            let ch = self.ch_idx();

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

        // Inlining this should allow significant optimization at the call site.
        #[inline]
        fn get_rx_status(&self) -> Option<Event> {
            let rmt = crate::peripherals::RMT::regs();
            let reg = rmt.int_raw().read();
            let ch = self.ch_idx();

            if reg.ch_rx_end(ch).bit() {
                Some(Event::End)
            } else if reg.ch_err(ch).bit() {
                Some(Event::Error)
            } else {
                None
            }
        }

        #[inline]
        fn is_rx_threshold_set(&self) -> bool {
            // no-op
            false
        }

        #[inline]
        fn reset_rx_threshold_set(&self) {
            // no-op
        }

        #[inline]
        fn set_rx_threshold(&self, _threshold: u8) {
            // no-op
        }

        #[inline]
        fn stop_rx(&self) {
            let rmt = crate::peripherals::RMT::regs();
            rmt.chconf1(self.ch_idx() as usize)
                .modify(|_, w| w.rx_en().clear_bit());
        }

        fn set_rx_filter_threshold(&self, value: u8) {
            let rmt = crate::peripherals::RMT::regs();
            rmt.chconf1(self.ch_idx() as usize).modify(|_, w| unsafe {
                w.rx_filter_en().bit(value > 0);
                w.rx_filter_thres().bits(value)
            });
        }

        fn set_rx_idle_threshold(&self, value: u16) {
            let rmt = crate::peripherals::RMT::regs();
            rmt.chconf0(self.ch_idx() as usize)
                .modify(|_, w| unsafe { w.idle_thres().bits(value) });
        }

        #[inline]
        fn set_rx_interrupt(&self, events: EnumSet<Event>, enable: bool) {
            let rmt = crate::peripherals::RMT::regs();
            let ch = self.ch_idx();
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
