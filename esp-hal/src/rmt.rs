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
    borrow::{Borrow, BorrowMut},
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
}

/// FIXME: Docs
pub trait Encoder {
    /// FIXME: docs
    fn encode(&mut self, writer: &mut RmtWriterInner) -> bool;
}

/// FIXME: Docs
pub struct SliceEncoder<'a> {
    data: &'a [PulseCode],
}

impl<'a> SliceEncoder<'a> {
    fn new(data: &'a [PulseCode]) -> Self {
        Self { data }
    }
}

impl<'a> Encoder for SliceEncoder<'a> {
    #[inline(always)]
    fn encode(&mut self, writer: &mut RmtWriterInner) -> bool {
        let count = self.data.len().min(unsafe { writer.end.offset_from(writer.ptr) } as usize);
        if count > 0 {
            let mut ptr = writer.ptr;

            for code in &self.data[..count] {
                unsafe { ptr.write_volatile(*code) }
                ptr = unsafe { ptr.add(1) };
            }

            debug_assert!(writer.ptr <= writer.end);

            writer.ptr = ptr;
            writer.last_code = self.data[count - 1];
            self.data = &self.data[count..];
        }

        self.data.is_empty()
    }
}

/// FIXME: Docs
pub struct IterEncoder<D>
where
    D: Iterator,
    D::Item: Borrow<PulseCode>,
{
    data: D
}

impl<D> IterEncoder<D>
where
    D: Iterator,
    D::Item: Borrow<PulseCode>,
{
    fn new(data: impl IntoIterator<IntoIter = D>) -> Self {
        Self {
            data: data.into_iter()
        }
    }
}

impl<D> Encoder for IterEncoder<D>
where
    D: Iterator,
    D::Item: Borrow<PulseCode>,
{
    #[inline(always)]
    fn encode(&mut self, writer: &mut RmtWriterInner) -> bool {
        while let Some(slot) = writer.next() {
            if let Some(code) = self.data.next() {
                slot.write(*code.borrow());
            } else {
                return true;
            }
        }

        false
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
    // #[crate::ram]
    fn write(
        &mut self,
        data: &mut impl Encoder,
        raw: impl RawChannelAccess<Dir = Tx>,
        initial: bool,
    ) {
        // ...which calls the inner funciton with a type-erased DynChannelAccess, such that it is
        // only monomorphized for different `data` types: There should be no significant benefit
        // from statically knowing the channel number here.
        #[inline(never)]
        fn inner(
            this: &mut RmtWriterOuter,
            data: &mut impl Encoder,
            raw: DynChannelAccess<Tx>,
            initial: bool,
        ) {
            if this.state != WriterState::Active {
                // Don't call next() on data again!
                return;
            }

            let ram_start = raw.channel_ram_start();
            let memsize = raw.memsize().codes();
            // FIXME: Somehow tell the compiler that this is > 0 without having panicking code here!
            // assert!(memsize > 0);
            let ram_end = unsafe { ram_start.add(memsize) };

            // FIXME: debug_assert that the current hw read addr is in the part of RAM that
            // we don't overwrite

            // This is only used to fill the entire RAM from its start, or to refill
            // either the first or second half. The code below may rely on this.
            // In particular, this implies that the offset might only need to be wrapped at
            // the end.
            let (count0, mut count1) = if initial {
                debug_assert!(this.offset == 0);
                // Leave space for extra end marker
                (memsize - 1, 0)
            } else if usize::from(this.offset) == memsize / 2 - 1 {
                // Overwrite previous extra end marker, then leave space for new extra end marker
                (memsize / 2, 0)
            } else if usize::from(this.offset) == memsize - 1 {
                // Overwrite previous extra end marker, then leave space for new extra end marker
                (1, memsize / 2 - 1)
            } else {
                debug_assert!(false);
                unreachable!();
            };

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

                if done || internal.ptr < internal.end {
                    // Data iterator is exhausted (either it stated so by returning false, or it didn't
                    // fill the entire requested buffer).
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
            // FIXME: Maybe, this can be moved to the RmtWriterInner/RmtSlot and SliceEncoder to
            // only write it if really needed?
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

        inner(self, data, raw.degrade(), initial)
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

    fn read<'a, T: From<PulseCode> + 'static>(
        &mut self,
        data: &mut impl Iterator<Item = &'a mut T>,
        raw: impl RawChannelAccess<Dir = Rx>,
        final_: bool,
    ) {
        if self.state != ReaderState::Active {
            // Don't call next() on data again!
            return;
        }

        let start = raw.channel_ram_start();
        let memsize = raw.memsize().codes();
        let end = unsafe { start.add(memsize) };
        let initial_offset = self.offset as usize;

        // This is only used to read the entire RAM from its start, or to read
        // either the first or second half. The code below may rely on this.
        // In particular, this implies that the offset might only need to be wrapped at
        // the end.
        debug_assert!(initial_offset == memsize / 2 || initial_offset == 0);

        // Read in up to 2 chunks
        let count = if final_ { memsize } else { memsize / 2 };
        let mut count0 = count.min(memsize - initial_offset);
        let mut count1 = count - count0;

        let mut ptr = unsafe { start.add(initial_offset) };
        'outer: loop {
            while count0 > 0 {
                if let Some(value) = data.next() {
                    let code = unsafe { ptr.read_volatile() };
                    *value = code.into();
                    ptr = unsafe { ptr.add(1) };
                    debug_assert!(ptr.addr() <= end.addr());

                    count0 -= 1;

                    if code.is_end_marker() {
                        self.state = ReaderState::Done;
                        break 'outer;
                    }
                } else {
                    self.state = ReaderState::Overflow;
                    break 'outer;
                }
            }

            if count1 == 0 {
                break;
            }

            count0 = count1;
            count1 = 0;
            ptr = start;
        }

        if ptr == end {
            // Wrap around
            ptr = start;
        }

        debug_assert!(ptr.addr() >= start.addr() && ptr.addr() < end.addr());

        self.total += count - count0 - count1;
        self.offset = unsafe { ptr.offset_from(start) } as u16;
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

/// FIXME: docs
pub trait Direction: Capability {}

/// FIXME: Docs
pub trait Capability: Clone + Copy + core::fmt::Debug + Unpin + crate::private::Sealed {
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
/// In other words, this links the `Capability` and `Direction` marker structs in the obvious
/// way.
pub trait Supports<Dir: Direction>: Capability + crate::private::Sealed {}

impl Supports<Tx> for Tx {}
impl Supports<Rx> for Rx {}
#[cfg(any(esp32, esp32s2))]
impl Supports<Tx> for RxTx {}
#[cfg(any(esp32, esp32s2))]
impl Supports<Rx> for RxTx {}

/// FIXME: Docs
pub trait Configure<Dir>: RawChannelAccess<Dir: Capability> {
    /// FIXME: Docs
    type Configured: RawChannelAccess<Dir=Dir>;

    #[doc(hidden)]
    fn configure(&self) -> Self::Configured;
}

impl<Cap, Dir, const CH_IDX: u8> Configure<Dir> for ConstChannelAccess<Cap, CH_IDX>
where
    Cap: Capability + Supports<Dir>,
    Dir: Direction,
{
    type Configured = ConstChannelAccess<Dir, CH_IDX>;

    fn configure(&self) -> Self::Configured {
        ConstChannelAccess {
            _direction: PhantomData,
        }
    }
}

impl<Cap, Dir> Configure<Dir> for DynChannelAccess<Cap>
where
    Cap: Capability + Supports<Dir>,
    Dir: Direction,
{
    type Configured = DynChannelAccess<Dir>;

    fn configure(&self) -> Self::Configured {
        DynChannelAccess {
            ch_idx: self.ch_idx,
            _direction: PhantomData,
        }
    }
}

/// docs
pub trait RawChannelAccess: Clone + Copy + crate::private::Sealed + Unpin {
    // Tx or Rx or Unconfigured
    /// docs
    type Dir: Copy + Clone;

    #[doc(hidden)]
    fn ch_idx(&self) -> u8;

    #[doc(hidden)]
    fn degrade(&self) -> DynChannelAccess<Self::Dir> {
        DynChannelAccess {
            ch_idx: unsafe { ChannelIndex::from_u8_unchecked(self.ch_idx()) },
            _direction: PhantomData,
        }
    }
}

trait RawChannelAccessExt {
    fn channel(&self) -> u8;
}

impl<Raw> RawChannelAccessExt for Raw
where
    Raw: RawChannelAccess<Dir: Capability>
{
    #[inline]
    fn channel(&self) -> u8 {
        chip_specific::channel_for_idx::<Raw::Dir>(self.ch_idx())
    }
}

/// docs
///
/// This is a ZST.
#[derive(Clone, Copy, Debug)]
pub struct ConstChannelAccess<Dir, const CH_IDX: u8>
where
    Dir: Copy + Clone
{
    _direction: PhantomData<Dir>,
}

/// Type-erased equivalent of ConstChannelAccess.
#[derive(Clone, Copy, Debug)]
pub struct DynChannelAccess<Dir>
where
    Dir: Copy + Clone
{
    ch_idx: ChannelIndex,
    _direction: PhantomData<Dir>,
}

impl<Dir, const CH_IDX: u8> crate::private::Sealed for ConstChannelAccess<Dir, CH_IDX>
where
    Dir: Copy + Clone
{}
impl<Dir> crate::private::Sealed for DynChannelAccess<Dir>
where
    Dir: Copy + Clone
{}

impl<Dir, const CH_IDX: u8> ConstChannelAccess<Dir, CH_IDX>
where
    Dir: Copy + Clone,
    ConstChannelAccess<Dir, CH_IDX>: RawChannelAccess,
{
    unsafe fn conjure() -> Self {
        Self {
            _direction: PhantomData,
        }
    }
}

impl<Dir> DynChannelAccess<Dir>
where
    Dir: Copy + Clone,
{
    unsafe fn conjure(ch_idx: ChannelIndex) -> Self {
        Self {
            ch_idx,
            _direction: PhantomData,
        }
    }
}

impl<const CH_IDX: u8, Dir: Unpin> RawChannelAccess for ConstChannelAccess<Dir, CH_IDX>
where
    Dir: Copy + Clone,
{
    type Dir = Dir;

    #[inline]
    fn ch_idx(&self) -> u8 {
        CH_IDX
    }
}

impl<Dir: Unpin> RawChannelAccess for DynChannelAccess<Dir>
where
    Dir: Copy + Clone,
{
    type Dir = Dir;

    #[inline]
    fn ch_idx(&self) -> u8 {
        self.ch_idx as u8
    }
}

/// docs
pub type AnyTxChannel<'ch, Dm> = Channel<'ch, Dm, DynChannelAccess<Tx>>;
/// docs
pub type AnyRxChannel<'ch, Dm> = Channel<'ch, Dm, DynChannelAccess<Rx>>;

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
        ([$name:ident, $num:literal, $idx:literal, $cap:ident] $(, $($ch:tt),+)?)
        -> (
            [$($field_decl:tt)*],
            [$($field_init:tt)*]
        )
    }) => {
        paste::paste! {
            declare_channels! (@define_rmt {
                ($($($ch),+)?) -> ([
                    $($field_decl)*
                    #[doc = concat!("RMT Channel ", $num)]
                    pub [<channel $num>]: ChannelCreator<'rmt, Dm, ConstChannelAccess<$cap, $idx>>,
                ], [
                    $($field_init)*
                    [<channel $num>]: $crate::rmt::ChannelCreator {
                        raw: unsafe { ConstChannelAccess::conjure() },
                        _rmt: ::core::marker::PhantomData,
                        _guard: $crate::system::GenericPeripheralGuard::new(),
                    },
                ])
            });
        }
    };

    (@define_channel_index {
        ()
        -> (
            [$($field_decl:tt)*],
            $sum:expr,
        )
    }) => {
        // Enum of valid channel indices: For the given chip, tx/rx channels for all of these indices
        // exist. (Note that channel index == channel number for esp32 and esp32s2, but not for other
        // chips.)
        //
        // This type is useful to inform the compiler of possible values of an u8 (i.e. we use this as
        // homemade refinement type) which allows it to elide bounds checks in register/field
        // accessors of the PAC even when using DynChannelAccess.
        //
        // Cf. https://github.com/rust-lang/rust/issues/109958 regarding rustc's capabilities here.
        #[derive(Clone, Copy, Debug, PartialEq, Eq)]
        #[repr(u8)]
        #[allow(unused)]
        enum ChannelIndex {
            $($field_decl)*
        }

        const CHANNEL_INDEX_COUNT: u8 = const { $sum };
    };

    // Arbitrarily ignore Rx and only count Tx/RxTx
    (@define_channel_index {
        ([$name:ident, $num:literal, $idx:literal, Rx] $(, $($ch:tt),+)?)
        -> (
            [$($field_decl:tt)*],
            $sum:expr,
        )
    }) => {
        declare_channels! (@define_channel_index {
            ($($($ch),+)?) -> (
                [$($field_decl)*],
                $sum,
            )
        });
    };

    (@define_channel_index {
        ([$name:ident, $num:literal, $idx:literal, Tx] $(, $($ch:tt),+)?)
        -> (
            [$($field_decl:tt)*],
            $sum:expr,
        )
    }) => {
        paste::paste! {
            declare_channels! (@define_channel_index {
                ($($($ch),+)?) -> (
                    [
                        $($field_decl)*
                        [< Ch $idx>] = $idx,
                    ],
                    $sum + 1,
                )
            });
        }
    };

    (@define_channel_index {
        ([$name:ident, $num:literal, $idx:literal, RxTx] $(, $($ch:tt),+)?)
        -> (
            [$($field_decl:tt)*],
            $sum:expr,
        )
    }) => {
        paste::paste! {
            declare_channels! (@define_channel_index {
                ($($($ch),+)?) -> (
                    [
                        $($field_decl)*
                        [< Ch $idx>] = $idx,
                    ],
                    $sum + 1,
                )
            });
        }
    };

    (@channel_count { () -> { $sum:expr }}) => {
        const NUM_CHANNELS: usize = const { $sum };
    };

    (@channel_count {
        (
            [$name:ident, $num:literal, $idx:literal, $cap:ident]
            $(, $($remaining:tt),+)?
        )
        ->
        { $sum:expr }
    }) => {
        declare_channels! (@channel_count {
            ($($($remaining),+)?)
            ->
            { $sum + 1 }
        });
    };

    ($($ch:tt),+ $(,)?) => {
        declare_channels! (@define_rmt { ($($ch),+) -> ([], []) });

        declare_channels! (@define_channel_index { ($($ch),+) -> ([], 0, ) });

        declare_channels! (@channel_count { ($($ch),+) -> { 0 } });
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
        unsafe { core::mem::transmute(ch_idx) }
    }
}

cfg_if::cfg_if! {
    if #[cfg(esp32)] {
        declare_channels!(
            [channel0, 0, 0, RxTx],
            [channel1, 1, 1, RxTx],
            [channel2, 2, 2, RxTx],
            [channel3, 3, 3, RxTx],
            [channel4, 4, 4, RxTx],
            [channel5, 5, 5, RxTx],
            [channel6, 6, 6, RxTx],
            [channel7, 7, 7, RxTx],
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
            [channel0, 0, 0, RxTx],
            [channel1, 1, 1, RxTx],
            [channel2, 2, 2, RxTx],
            [channel3, 3, 3, RxTx],
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
            [channel0, 0, 0, Tx],
            [channel1, 1, 1, Tx],
            [channel2, 2, 2, Tx],
            [channel3, 3, 3, Tx],
            [channel4, 4, 0, Rx],
            [channel5, 5, 1, Rx],
            [channel6, 6, 2, Rx],
            [channel7, 7, 3, Rx],
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
            [channel0, 0, 0, Tx],
            [channel1, 1, 1, Tx],
            [channel2, 2, 0, Rx],
            [channel3, 3, 1, Rx],
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
    /// FIXME: Docs
    pub fn degrade(self) -> ChannelCreator<'rmt, Dm, DynChannelAccess<Raw::Dir>> {
        // FIXME: Share code with Channel::degrade?
        use core::mem::ManuallyDrop;
        // Disable Drop handler on self
        let old = ManuallyDrop::new(self);
        ChannelCreator {
            raw: DynChannelAccess {
                ch_idx: unsafe { ChannelIndex::from_u8_unchecked(old.raw.ch_idx()) },
                _direction: PhantomData,
            },
            _rmt: PhantomData,
            // FIXME: Don't clone, but move old._guard
            _guard: old._guard.clone(),
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

impl<'rmt, Dm, Dir: Capability, const CH_IDX: u8> ChannelCreator<'rmt, Dm, ConstChannelAccess<Dir, CH_IDX>>
where
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
    pub unsafe fn steal() -> ChannelCreator<'rmt, Dm, ConstChannelAccess<Dir, CH_IDX>> {
        assert!(CH_IDX < CHANNEL_INDEX_COUNT);

        let raw = unsafe { ConstChannelAccess::conjure() };

        // panic if the channel is currently running a transaction, or if another
        // channel is using its memory
        assert!(matches!(
            RmtState::load(raw, Ordering::Relaxed),
            RmtState::Unconfigured | RmtState::TxIdle | RmtState::RxIdle
        ));

        ChannelCreator {
            raw,
            _rmt: PhantomData,
            _guard: GenericPeripheralGuard::new(),
        }
    }
}

/// FIXME: docs
pub trait TxChannelCreator<Dm: crate::DriverMode>: Sized {
    /// FIXME: docs
    type Raw: TxChannelInternal;

    /// FIXME: docs
    fn configure_tx<'pin>(
        self,
        pin: impl PeripheralOutput<'pin>,
        config: TxChannelConfig,
    ) -> Result<Channel<'pin, Dm, Self::Raw>, (Error, Self)>
    where
        Self: 'pin ;
}

/// FIXME: docs
pub trait RxChannelCreator<Dm: crate::DriverMode>: Sized {
    /// FIXME: docs
    type Raw: RxChannelInternal;

    /// FIXME: docs
    fn configure_rx<'pin>(
        self,
        pin: impl PeripheralInput<'pin>,
        config: RxChannelConfig,
    ) -> Result<Channel<'pin, Dm, Self::Raw>, (Error, Self)>
    where
        Self: 'pin ;
}

/// FIXME: docs
pub trait AnyTxChannelAccess: RawChannelAccess<Dir = Tx> {}
/// FIXME: docs
pub trait AnyRxChannelAccess: RawChannelAccess<Dir = Rx> {}

// TODO:
// - RxTx devices: always allow reconfigure + unconfigure
// - Rx/Tx devices: allow Unconfigure and reconfigure with the same direction

impl<'rmt, Dm, Raw> TxChannelCreator<Dm> for ChannelCreator<'rmt, Dm, Raw>
where
    Dm: crate::DriverMode,
    Raw: Configure<Tx>,
{
    type Raw = <Raw as Configure<Tx>>::Configured;

    /// Configure the TX channel
    fn configure_tx<'pin>(
        self,
        pin: impl PeripheralOutput<'pin>,
        config: TxChannelConfig,
    ) -> Result<Channel<'pin, Dm, Self::Raw>, (Error, Self)>
    where
        Self: 'pin
    {
        let raw = self.raw.configure();
        if let Err(e) = configure_tx_channel(&raw, pin, config) {
            return Err((e, self));
        };
        Ok(Channel::new(raw))
    }
}

impl<'rmt, Dm, Raw> RxChannelCreator<Dm> for ChannelCreator<'rmt, Dm, Raw>
where
    Dm: crate::DriverMode,
    Raw: Configure<Rx>,
{
    type Raw = <Raw as Configure<Rx>>::Configured;

    /// Configure the RX channel
    fn configure_rx<'pin>(
        self,
        pin: impl PeripheralInput<'pin>,
        config: RxChannelConfig,
    ) -> Result<Channel<'pin, Dm, Self::Raw>, (Error, Self)>
    where
        Self: 'pin
    {
        let raw = self.raw.configure();
        if let Err(e) = configure_rx_channel(&raw, pin, config) {
            return Err((e, self));
        };
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
// FIXME: Pass channel as ChannelIndex to avoid bounds checks
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
            RmtState::store_range_rev(RmtState::Unconfigured, channel, cur_channel, Ordering::Relaxed);

            return Err(Error::MemoryBlockNotAvailable);
        }

        // Set the first channel to `state` (`Rx`|`Tx`), the remaining (if any) to
        // `Reserved`
        next_state = RmtState::Reserved;
    }

    Ok(())
}

fn configure_rx_channel<'pin>(
    raw: &impl RawChannelAccess<Dir = Rx>,
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

fn configure_tx_channel<'pin>(
    raw: &impl RawChannelAccess<Dir = Tx>,
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


mod state {
    use super::*;

    // This must only holds value of RmtState. However, we need atomic access, thus
    // represent as AtomicU8.
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
        #[allow(unused)]
        pub(super) fn is_tx(&self) -> Option<bool> {
            match self {
                Self::RxIdle | Self::RxAsync | Self::RxBlocking => Some(false),
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
        pub(super) fn load_by_idx(channel: u8, ordering: Ordering) -> Self {
            unsafe { Self::from_u8_unchecked(STATE[channel as usize].load(ordering)) }
        }

        #[inline]
        // FIXME: Pass start, end as ChannelIndex to avoid bounds checks
        pub(super) fn store_range_rev(self, start: u8, end: u8, ordering: Ordering) {
            for ch_num in (start as usize..end as usize).rev() {
                STATE[ch_num].store(self as u8, ordering);
            }
        }

        #[inline]
        pub(super) fn store(self, raw: impl RawChannelAccess<Dir: Capability>, ordering: Ordering) {
            STATE[raw.channel() as usize].store(self as u8, ordering);
        }

        #[inline]
        pub(super) fn load(raw: impl RawChannelAccess<Dir: Capability>, ordering: Ordering) -> Self {
            Self::load_by_idx(raw.channel(), ordering)
        }

        #[inline]
        pub(super) fn compare_exchange(
            ch_num: u8,
            current: Self,
            new: Self,
            success: Ordering,
            failure: Ordering,
        ) -> Result<Self, Self> {
            STATE[ch_num as usize].compare_exchange(current as u8, new as u8, success, failure)
                .map(|prev| unsafe { Self::from_u8_unchecked(prev) })
                .map_err(|prev| unsafe { Self::from_u8_unchecked(prev) })
        }
    }
}

use state::RmtState;

static WAKER: [AtomicWaker; NUM_CHANNELS] = [const { AtomicWaker::new() }; NUM_CHANNELS];

type RmtPeripheralGuard = GenericPeripheralGuard<{ system::Peripheral::Rmt as u8 }>;

/// RMT Channel
#[derive(Debug)]
#[non_exhaustive]
pub struct Channel<'ch, Dm, Raw>
where
    Dm: crate::DriverMode,
    Raw: ChannelInternal,
{
    raw: Raw,
    _rmt: PhantomData<Rmt<'ch, Dm>>,
    _guard: RmtPeripheralGuard,
}

impl<Dm, Raw> Channel<'_, Dm, Raw>
where
    Dm: crate::DriverMode,
    Raw: ChannelInternal,
{
    fn new(raw: Raw) -> Self {
        Self {
            raw,
            _rmt: core::marker::PhantomData,
            _guard: GenericPeripheralGuard::new(),
        }
    }

    // FIXME: Implement, regain full Capabilitis of Raw!
    // fn unconfigure(self) -> ChannelCreator<'rmt, Dm, Raw> {
    //     // requires ChannelCreator to take a RawChannelAccess such that type-erased channels can
    //     // also be unconfigured
    //     todo!("implement, then do the same on drop")
    // }

    /// Get size of this channel's hardware buffer (number of `PulseCode`s).
    pub fn buffer_size(&self) -> usize {
        self.raw.memsize().codes()
    }
}

// Note that this is intentionally implemented even if Raw is DynChannelAccess
// already for convenience!
impl<'ch, Dm, Dir, Raw> Channel<'ch, Dm, Raw>
where
    Dm: crate::DriverMode,
    Dir: Direction,
    Raw: RawChannelAccess<Dir = Dir>,
{
    /// Consume the channel and return a type-erased version
    pub fn degrade(self) -> Channel<'ch, Dm, DynChannelAccess<Dir>> {
        use core::mem::ManuallyDrop;
        // Disable Drop handler on self
        let old = ManuallyDrop::new(self);
        Channel {
            raw: DynChannelAccess {
                ch_idx: unsafe { ChannelIndex::from_u8_unchecked(old.raw.ch_idx()) },
                _direction: PhantomData,
            },
            _rmt: PhantomData,
            // FIXME: Don't clone, but move old._guard
            _guard: old._guard.clone(),
        }
    }
}

impl<Dm, Raw> Drop for Channel<'_, Dm, Raw>
where
    Dm: crate::DriverMode,
    Raw: ChannelInternal,
{
    fn drop(&mut self) {
        let memsize = self.raw.memsize();

        // This isn't really necessary, but be extra sure that this channel can't
        // interfere with others.
        self.raw.set_memsize(MemSize::from_blocks(0));

        // Existence of this `Channel` struct implies exclusive access to these hardware
        // channels, thus simply store the new state.
        // FIXME: Check whether this elides bounds checks
        RmtState::store_range_rev(
            RmtState::Unconfigured,
            self.raw.channel(),
            self.raw.channel() + memsize.blocks(),
            Ordering::Release
        );
    }
}

// Blocking interfaces
// ----------------------------------------------------------

/// An in-progress transaction for a single shot TX transaction.
pub struct SingleShotTxTransaction<'ch, Raw, E>
where
    Raw: TxChannelInternal,
    E: Encoder,
{
    raw: Raw,
    _phantom: PhantomData<&'ch mut Raw>,

    writer: RmtWriterOuter,

    // Remaining data that has not yet been written to channel RAM. May be empty.
    // FIXME: Maybe store by reference &'a mut encoder?
    data: E,
}

impl<Raw, E> SingleShotTxTransaction<'_, Raw, E>
where
    Raw: TxChannelInternal,
    E: Encoder,
{
    fn poll_internal(&mut self) -> Option<Event> {
        let raw = self.raw;

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

        RmtState::store(RmtState::TxIdle, self.raw, Ordering::Relaxed);

        // Disable the Drop handler since the transaction is properly stopped
        // already.
        let _ = ManuallyDrop::new(self);
        result
    }
}
impl<Raw, E> Drop for SingleShotTxTransaction<'_, Raw, E>
where
    Raw: TxChannelInternal,
    E: Encoder,
{
    fn drop(&mut self) {
        // If this is dropped, that implies that the transaction was not properly
        // `wait()`ed for. Thus, attempt to stop it as quickly as possible and
        // block in the meantime, such that subsequent uses of the channel are
        // safe (i.e. start from a state where the hardware is stopped).
        let raw = self.raw;
        let immediate = raw.stop_tx();

        if !immediate {
            while !matches!(raw.get_tx_status(), Some(Event::Error | Event::End)) {}
        }

        RmtState::store(RmtState::TxIdle, raw, Ordering::Relaxed);
    }
}

/// An in-progress continuous TX transaction
pub struct ContinuousTxTransaction<'ch, Raw: TxChannelInternal> {
    raw: Raw,
    _phantom: PhantomData<&'ch mut Raw>,
}

impl<Raw: TxChannelInternal> ContinuousTxTransaction<'_, Raw> {
    /// Stop transaction when the current iteration ends.
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
    pub fn stop(self) -> Result<(), Error> {
        let raw = self.raw;

        raw.set_tx_continuous(false);
        raw.update();

        // FIXME: Check TRM on whether this requires an update() call.
        let immediate = raw.stop_tx();

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

impl<Raw: TxChannelInternal> Drop for ContinuousTxTransaction<'_, Raw> {
    fn drop(&mut self) {
        // If this is dropped, that implies that the transaction was not manually
        // stopped with `stop()` or `stop_next()`.
        // Thus, attempt to stop it as quickly as possible and block in the meantime,
        // such that subsequent uses of the channel are safe (i.e. start from a
        // state where the hardware is stopped).
        let raw = self.raw;

        raw.set_tx_continuous(false);
        raw.update();

        let immediate = raw.stop_tx();

        if !immediate {
            while !matches!(raw.get_tx_status(), Some(Event::Error | Event::End)) {}
        }
    }
}

impl<Dm, Raw> Channel<'_, Dm, Raw>
where
    Dm: crate::DriverMode,
    Raw: TxChannelInternal,
{
    /// FIXME: docs
    /// FIXME: Add an example of how to use these
    pub fn bench(&mut self, data: &[PulseCode]) -> usize {
        let mut data = SliceEncoder::new(data);
        let mut writer = RmtWriterOuter::new();
        writer.write(&mut data, self.raw, true);

        while writer.state == WriterState::Active {
            writer.write(&mut data, self.raw, false);
        }

        writer.written
    }

    /// FIXME: docs
    pub fn bench_iter<D>(&mut self, data: D) -> usize
    where
        D: IntoIterator,
        D::Item: Borrow<PulseCode>
    {
        let mut data = IterEncoder::new(data);
        let mut writer = RmtWriterOuter::new();
        writer.write(&mut data, self.raw, true);

        while writer.state == WriterState::Active {
            writer.write(&mut data, self.raw, false);
        }

        writer.written
    }

    /// FIXME: docs
    pub fn bench_enc<E: Encoder>(&mut self, mut data: E) -> usize {
        let mut writer = RmtWriterOuter::new();
        writer.write(&mut data, self.raw, true);

        while writer.state == WriterState::Active {
            writer.write(&mut data, self.raw, false);
        }

        writer.written
    }
}

/// Channel in TX mode
pub trait TxChannel: Sized {
    /// FIXME: Docs
    type Raw: TxChannelInternal;

    /// Start transmitting the given pulse code sequence.
    /// This returns a [`SingleShotTxTransaction`] which can be used to wait for
    /// the transaction to complete and get back the channel for further
    /// use.
    fn transmit<'d>(
        &mut self,
        data: &'d [PulseCode],
        // FIXME: Opaque return type
    ) -> Result<SingleShotTxTransaction<'_, Self::Raw, SliceEncoder<'d>>, Error>;

    /// FIXME: docs
    fn transmit_iter<D>(
        &mut self,
        data: D,
    ) -> Result<SingleShotTxTransaction<'_, Self::Raw, IterEncoder<<D as IntoIterator>::IntoIter>>, Error>
    where
        D: IntoIterator,
        D::Item: Borrow<PulseCode>;

    /// FIXME: docs
    fn transmit_enc<E>(
        &mut self,
        data: E,
    ) -> Result<SingleShotTxTransaction<'_, Self::Raw, E>, Error>
    where
        E: Encoder;

    /// Start transmitting the given pulse code continuously.
    /// This returns a [`ContinuousTxTransaction`] which can be used to stop the
    /// ongoing transmission and get back the channel for further use.
    /// The length of sequence cannot exceed the size of the allocated RMT RAM.
    fn transmit_continuously<D>(
        &mut self,
        data: D,
    ) -> Result<ContinuousTxTransaction<'_, Self::Raw>, Error>
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
    ) -> Result<ContinuousTxTransaction<'_, Self::Raw>, Error>
    where
        D: IntoIterator,
        D::Item: Borrow<PulseCode>;
}

impl<Raw> TxChannel for Channel<'_, Blocking, Raw>
where
    Raw: TxChannelInternal,
{
    type Raw = Raw;

    fn transmit<'d>(
        &mut self,
        data: &'d [PulseCode],
    ) -> Result<SingleShotTxTransaction<'_, Raw, SliceEncoder<'d>>, Error> {
        let raw = self.raw;

        // FIXME: For slices, verify more things beforehand (non-empty, has end marker)

        let mut data = SliceEncoder::new(data);
        let mut writer = RmtWriterOuter::new();
        writer.write(&mut data, raw, true);

        match writer.state {
            WriterState::Empty => return Err(Error::InvalidArgument),
            WriterState::DoneNoEnd => return Err(Error::EndMarkerMissing),
            // WriterState::DoneEarly => return Err(Error::UnexpectedEndMarker),
            _ => (),
        };

        RmtState::store(RmtState::TxBlocking, raw, Ordering::Relaxed);

        raw.clear_tx_interrupts();
        raw.start_send(false, 0);

        Ok(SingleShotTxTransaction {raw, _phantom: PhantomData, writer, data})
    }

    /// Start transmitting the given pulse code sequence.
    /// This returns a [`SingleShotTxTransaction`] which can be used to wait for
    /// the transaction to complete and get back the channel for further use.
    fn transmit_iter<D>(
        &mut self,
        data: D,
    ) -> Result<SingleShotTxTransaction<'_, Raw, IterEncoder<<D as IntoIterator>::IntoIter>>, Error>
    where
        D: IntoIterator,
        D::Item: Borrow<PulseCode>,
    {
        let raw = self.raw;

        let mut data = IterEncoder::new(data);
        let mut writer = RmtWriterOuter::new();
        writer.write(&mut data, raw, true);

        match writer.state {
            WriterState::Empty => return Err(Error::InvalidArgument),
            WriterState::DoneNoEnd => return Err(Error::EndMarkerMissing),
            // WriterState::DoneEarly => return Err(Error::UnexpectedEndMarker),
            _ => (),
        };

        RmtState::store(RmtState::TxBlocking, raw, Ordering::Relaxed);

        raw.clear_tx_interrupts();
        raw.start_send(false, 0);

        Ok(SingleShotTxTransaction {raw, _phantom: PhantomData, writer, data})
    }

    fn transmit_enc<E>(
        &mut self,
        mut data: E,
    ) -> Result<SingleShotTxTransaction<'_, Raw, E>, Error>
    where
        E: Encoder
    {
        let raw = self.raw;

        let mut writer = RmtWriterOuter::new();
        writer.write(&mut data, raw, true);

        match writer.state {
            WriterState::Empty => return Err(Error::InvalidArgument),
            WriterState::DoneNoEnd => return Err(Error::EndMarkerMissing),
            // WriterState::DoneEarly => return Err(Error::UnexpectedEndMarker),
            _ => (),
        };

        RmtState::store(RmtState::TxBlocking, raw, Ordering::Relaxed);

        raw.clear_tx_interrupts();
        raw.start_send(false, 0);

        Ok(SingleShotTxTransaction {raw, _phantom: PhantomData, writer, data})
    }

    /// Start transmitting the given pulse code continuously.
    /// This returns a [`ContinuousTxTransaction`] which can be used to stop the
    /// ongoing transmission and get back the channel for further use.
    /// The length of sequence cannot exceed the size of the allocated RMT RAM.
    fn transmit_continuously<D>(
        &mut self,
        data: D,
    ) -> Result<ContinuousTxTransaction<'_, Raw>, Error>
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
    ) -> Result<ContinuousTxTransaction<'_, Raw>, Error>
    where
        D: IntoIterator,
        D::Item: Borrow<PulseCode>,
    {
        let raw = self.raw;

        let mut data = IterEncoder::new(data);
        let mut writer = RmtWriterOuter::new();
        writer.write(&mut data, raw, true);

        match writer.state {
            WriterState::Empty => return Err(Error::InvalidArgument),
            WriterState::Active => return Err(Error::Overflow),
            WriterState::DoneNoEnd => return Err(Error::EndMarkerMissing),
            // WriterState::DoneEarly => return Err(Error::UnexpectedEndMarker),
            WriterState::Done => (),
        };

        raw.start_send(true, loopcount);

        Ok(ContinuousTxTransaction { raw, _phantom: PhantomData })
    }
}

/// RX transaction instance
pub struct RxTransaction<'ch, 'a, Raw, D, T>
where
    Raw: RxChannelInternal,
    D: Iterator<Item = &'a mut T>,
    T: From<PulseCode> + 'static,
{
    raw: Raw,
    _phantom: PhantomData<&'ch mut Raw>,

    reader: RmtReader,

    data: D,
}

impl<'a, Raw, D, T> RxTransaction<'_, 'a, Raw, D, T>
where
    Raw: RxChannelInternal,
    D: Iterator<Item = &'a mut T>,
    T: From<PulseCode> + 'static,
{
    fn poll_internal(&mut self) -> Option<Event> {
        let raw = self.raw;

        let status = raw.get_rx_status();
        match status {
            Some(Event::End) => {
                if self.reader.state != ReaderState::Done {
                    // Do not clear the interrupt flags here: Subsequent calls of wait() must still
                    // be able to observe them if this is currently called via
                    // poll()
                    raw.stop_rx();
                    raw.update();

                    self.reader.read(&mut self.data, raw, true);

                    // Ensure that no further data will be read if this is called repeatedly.
                    self.reader.state = ReaderState::Done;
                }
            }
            Some(Event::Threshold) if Raw::supports_rx_wrap() => {
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

        let raw = self.raw;

        raw.clear_rx_interrupts();
        RmtState::store(RmtState::RxIdle, raw, Ordering::Relaxed);

        // Disable Drop handler since the receiver is stopped already.
        let _ = ManuallyDrop::new(self);
        result
    }
}

impl<'a, Raw, D, T> Drop for RxTransaction<'_, 'a, Raw, D, T>
where
    Raw: RxChannelInternal,
    D: Iterator<Item = &'a mut T>,
    T: From<PulseCode> + 'static,
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
            // -> Actually, stop_rx() appears to be immediate and does not update state flags, so
            // this would lock up!
            // while !matches!(raw.get_rx_status(), Some(Event::Error | Event::End)) {}

            raw.clear_rx_interrupts();
            RmtState::store(RmtState::RxIdle, raw, Ordering::Relaxed);
        }
    }
}

/// Channel is RX mode
pub trait RxChannel: Sized {
    /// FIXME: docs
    type Raw: RxChannelInternal;

    /// Start receiving pulse codes into the given buffer.
    /// This returns a [RxTransaction] which can be used to wait for receive to
    /// complete and get back the channel for further use.
    /// The length of the received data cannot exceed the allocated RMT RAM.
    fn receive<'ch, 'a, D, T>(
        &'ch mut self,
        data: D,
    ) -> Result<RxTransaction<'ch, 'a, Self::Raw, D::IntoIter, T>, Error>
    where
        D: IntoIterator<Item = &'a mut T>,
        T: From<PulseCode> + 'static;
}

impl<Raw> RxChannel for Channel<'_, Blocking, Raw>
where
    Raw: RxChannelInternal,
{
    type Raw = Raw;

    /// Start receiving pulse codes into the given buffer.
    /// This returns a [RxTransaction] which can be used to wait for receive to
    /// complete and get back the channel for further use.
    fn receive<'ch, 'a, D, T>(
        &'ch mut self,
        data: D,
    ) -> Result<RxTransaction<'ch, 'a, Raw, D::IntoIter, T>, Error>
    where
        D: IntoIterator<Item = &'a mut T>,
        T: From<PulseCode> + 'static,
    {
        let raw = self.raw;

        let data = data.into_iter();
        let reader = RmtReader::new();

        RmtState::store(RmtState::RxBlocking, raw, Ordering::Relaxed);

        raw.clear_rx_interrupts();
        raw.start_receive(true);

        Ok(RxTransaction {raw, _phantom: PhantomData, reader, data})
    }
}

// Async interfaces ----------------------------------------------------------

// FIXME: This is essentially the same as SingleShotTxTransaction. Is it
// possible to share most of the code?
#[must_use = "futures do nothing unless you `.await` or poll them"]
struct RmtTxFuture<'a, Raw, D, E>
where
    Raw: TxChannelInternal,
    D: BorrowMut<E>,
    E: Encoder,
{
    raw: Raw,
    _phantom: PhantomData<(&'a mut Raw, E)>,

    writer: RmtWriterOuter,

    // Remaining data that has not yet been written to channel RAM. May be empty.
    data: D,
}

impl<Raw, D, E> Future for RmtTxFuture<'_, Raw, D, E>
where
    Raw: TxChannelInternal,
    D: BorrowMut<E> + Unpin,
    E: Encoder + Unpin,
{
    type Output = Result<(), Error>;

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
                // In this case, either tx was never started or it was stopped already below in a
                // previous call to poll.
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
                    this.writer.write(this.data.borrow_mut(), raw, false);
                    // FIXME: Return if writer.state indicates an error (ensure
                    // to stop transmission first)
                }

                match this.writer.state {
                    WriterState::Active => {
                        raw.listen_tx_interrupt(Event::Threshold);
                    }
                    WriterState::DoneNoEnd => {
                        // We have written an extra end marker, so will stop automatically after
                        // all data has been transmitted. If we stop tx manually here, this will
                        // prevent the end interrupt from triggering, and we will lock up polling.
                        // raw.stop_tx();
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

impl<Raw, D, E> Drop for RmtTxFuture<'_, Raw, D, E>
where
    Raw: TxChannelInternal,
    D: BorrowMut<E>,
    E: Encoder,
{
    fn drop(&mut self) {
        let raw = self.raw;

        // STATE should be TxIdle if the future was polled to completion
        if RmtState::load(raw, Ordering::Relaxed) == RmtState::TxAsync {
            raw.unlisten_tx_interrupt(Event::Error | Event::End | Event::Threshold);
            let immediate = raw.stop_tx();

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
pub trait TxChannelAsync {
    /// Start transmitting the given pulse code sequence.
    /// The length of sequence cannot exceed the size of the allocated RMT
    /// RAM.
    async fn transmit<'a>(&'a mut self, data: &'a [PulseCode]) -> Result<(), Error>
    where
        Self: Sized;

    /// FIXME: docs
    async fn transmit_iter<D>(&mut self, data: D) -> Result<(), Error>
    where
        Self: Sized,
        D: IntoIterator,
        D::IntoIter: Unpin,
        D::Item: Borrow<PulseCode>;

    /// FIXME: docs
    async fn transmit_enc<'a, E>(&'a mut self, data: &'a mut E) -> Result<(), Error>
    where
        Self: Sized,
        E: Encoder + Unpin;
}

impl<Raw> TxChannelAsync for Channel<'_, Async, Raw>
where
    Raw: TxChannelInternal,
{
    // fn transmit(&mut self, data: &[PulseCode]) -> impl Future<Output = Result<(), Error>>
    fn transmit<'a>(&'a mut self, data: &'a [PulseCode]) -> impl Future<Output = Result<(), Error>>
    where
        Self: Sized
    {
        let raw = self.raw;

        // FIXME: copy over the modified validation code from the blocking code

        let mut fut = RmtTxFuture {
            raw,
            _phantom: PhantomData,
            writer: RmtWriterOuter::new(),
            data: SliceEncoder::new(data),
        };

        fut.writer.write(fut.data.borrow_mut(), raw, true);
        
        let wrap = match fut.writer.state {
            WriterState::Empty | WriterState::DoneNoEnd => {
                // Error cases; the future will return the error on the first call to poll()
                return fut;
            }
            WriterState::Active => true,
            WriterState::Done => false,
        };

        RmtState::store(RmtState::TxAsync, fut.raw, Ordering::Relaxed);

        fut.raw.clear_tx_interrupts();
        let mut events = Event::End | Event::Error;
        if wrap {
            events |= Event::Threshold;
        }
        fut.raw.listen_tx_interrupt(events);
        fut.raw.start_send(false, 0);

        fut
    }

    fn transmit_iter<D>(&mut self, data: D) -> impl Future<Output = Result<(), Error>>
    where
        Self: Sized,
        D: IntoIterator,
        // FIXME: Is being Unpin a significant restriction for the iterator?
        // If so, we could use it in a pinned way by using pin-projection (e.g. via
        // pin-project-lite) for the RmtTxFuture.
        D::IntoIter: Unpin,
        D::Item: Borrow<PulseCode>,
    {
        let raw = self.raw;

        let mut fut = RmtTxFuture {
            raw,
            _phantom: PhantomData,
            writer: RmtWriterOuter::new(),
            data: IterEncoder::new(data),
        };

        fut.writer.write(fut.data.borrow_mut(), raw, true);
        
        let wrap = match fut.writer.state {
            WriterState::Empty | WriterState::DoneNoEnd => {
                // Error cases; the future will return the error on the first call to poll()
                return fut;
            }
            WriterState::Active => true,
            WriterState::Done => false,
        };

        RmtState::store(RmtState::TxAsync, fut.raw, Ordering::Relaxed);

        fut.raw.clear_tx_interrupts();
        let mut events = Event::End | Event::Error;
        if wrap {
            events |= Event::Threshold;
        }
        fut.raw.listen_tx_interrupt(events);
        fut.raw.start_send(false, 0);

        fut
    }

    fn transmit_enc<'a, E>(&'a mut self, data: &'a mut E) -> impl Future<Output = Result<(), Error>>
    where
        Self: Sized,
        E: Encoder + Unpin,
    {
        let raw = self.raw;

        let mut fut = RmtTxFuture::<_, &mut E, E> {
            raw,
            _phantom: PhantomData,
            writer: RmtWriterOuter::new(),
            data,
        };

        fut.writer.write(fut.data.borrow_mut(), raw, true);
        
        let wrap = match fut.writer.state {
            WriterState::Empty | WriterState::DoneNoEnd => {
                // Error cases; the future will return the error on the first call to poll()
                return fut;
            }
            WriterState::Active => true,
            WriterState::Done => false,
        };

        RmtState::store(RmtState::TxAsync, fut.raw, Ordering::Relaxed);

        fut.raw.clear_tx_interrupts();
        let mut events = Event::End | Event::Error;
        if wrap {
            events |= Event::Threshold;
        }
        fut.raw.listen_tx_interrupt(events);
        fut.raw.start_send(false, 0);

        fut
    }
}

#[must_use = "futures do nothing unless you `.await` or poll them"]
struct RmtRxFuture<'ch, 'a, Raw, D, T>
where
    Raw: RxChannelInternal,
    D: Iterator<Item = &'a mut T> + Unpin,
    T: From<PulseCode> + 'static,
{
    raw: Raw,
    _phantom: PhantomData<&'ch mut Raw>,

    reader: RmtReader,

    data: D,
}

impl<'a, Raw, D, T> Future for RmtRxFuture<'_, 'a, Raw, D, T>
where
    Raw: RxChannelInternal,
    D: Iterator<Item = &'a mut T> + Unpin,
    T: From<PulseCode> + 'static,
{
    type Output = Result<usize, Error>;

    fn poll(self: Pin<&mut Self>, ctx: &mut Context<'_>) -> Poll<Self::Output> {
        let this = self.get_mut();
        let raw = this.raw;

        WAKER[raw.channel() as usize].register(ctx.waker());

        let result = match raw.get_rx_status() {
            Some(Event::Error) => Err(Error::ReceiverError),
            Some(Event::End) => {
                raw.stop_rx();
                raw.update();

                this.reader.read(&mut this.data, raw, true);

                Ok(this.reader.total)
            }
            Some(Event::Threshold) if Raw::supports_rx_wrap() => {
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

impl<'a, Raw, D, T> Drop for RmtRxFuture<'_, 'a, Raw, D, T>
where
    Raw: RxChannelInternal,
    D: Iterator<Item = &'a mut T> + Unpin,
    T: From<PulseCode> + 'static,
{
    fn drop(&mut self) {
        let raw = self.raw;

        // STATE should be RxIdle if the future was polled to completion
        if RmtState::load(raw, Ordering::Relaxed) == RmtState::RxAsync {
            raw.stop_rx();
            raw.update();

            // block until the channel is safe to use again
            // -> Actually, stop_rx() appears to be immediate and does not update state flags, so
            // this would lock up!
            // while !matches!(raw.get_rx_status(), Some(Event::Error | Event::End)) {}

            raw.clear_rx_interrupts();
            RmtState::store(RmtState::RxIdle, raw, Ordering::Relaxed);
        }
    }
}

/// RX channel in async mode
pub trait RxChannelAsync {
    /// Start receiving a pulse code sequence.
    /// The length of sequence cannot exceed the size of the allocated RMT
    /// RAM.
    async fn receive<'a, D, T>(&'a mut self, data: D) -> Result<usize, Error>
    where
        Self: Sized,
        D: IntoIterator<Item = &'a mut T>,
        D::IntoIter: Unpin,
        T: From<PulseCode> + 'static;
}

impl<Raw> RxChannelAsync for Channel<'_, Async, Raw>
where
    Raw: RxChannelInternal,
{
    fn receive<'a, D, T>(&'a mut self, data: D) -> impl Future<Output = Result<usize, Error>>
    where
        Self: Sized,
        D: IntoIterator<Item = &'a mut T>,
        D::IntoIter: Unpin,
        T: From<PulseCode> + 'static,
    {
        let raw = self.raw;

        let data = data.into_iter();
        let reader = RmtReader::new();

        RmtState::store(RmtState::RxAsync, raw, Ordering::Relaxed);

        raw.clear_rx_interrupts();
        raw.listen_rx_interrupt(Event::End | Event::Error | Event::Threshold);
        raw.start_receive(true);

        RmtRxFuture { raw, _phantom: PhantomData, reader, data }
    }
}

#[inline(never)]
fn wake(ch_num: u8) {
    WAKER[ch_num as usize].wake();
}

// #[cfg(any(esp32, esp32s2))]
#[handler]
fn async_interrupt_handler() {
    fn on_tx(raw: &impl RawChannelAccess<Dir = Tx>, event: Event) {
        let events_to_unlisten = match event {
            Event::End | Event::Error => Event::End | Event::Error | Event::Threshold,
            // The RmtTxFuture will wnable the threshold interrupt again if required
            Event::Threshold => Event::Threshold.into(),
        };
        raw.unlisten_tx_interrupt(events_to_unlisten);

        wake(raw.channel());
    }

    fn on_rx(raw: &impl RawChannelAccess<Dir = Rx>, event: Event) {
        let events_to_unlisten = match event {
            Event::End | Event::Error => Event::End | Event::Error | Event::Threshold,
            // The RmtRxFuture will wnable the threshold interrupt again if required
            Event::Threshold => Event::Threshold.into(),
        };
        raw.unlisten_rx_interrupt(events_to_unlisten);

        wake(raw.channel());
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
pub trait TxChannelInternal: ChannelInternal + RawChannelAccess<Dir = Tx> {
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
    pub(super) fn handle_channel_interrupts(
        on_tx: fn(&DynChannelAccess<Tx>, Event),
        on_rx: fn(&DynChannelAccess<Rx>, Event),
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
            on_tx(&raw, event);
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
            on_rx(&raw, event);
        }
    }

    #[inline]
    pub(super) const fn channel_for_idx<Dir: Capability>(idx: u8) -> u8 {
        if Dir::SUPPORTS_TX {
            idx
        } else {
            idx + NUM_CHANNELS as u8 / 2
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
            let ch_idx = self.ch_idx() as usize;

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
            let ch_idx = self.ch_idx() as usize;

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

        fn is_error(&self) -> bool {
            let rmt = crate::peripherals::RMT::regs();
            let int_raw = rmt.int_raw().read();
            let ch_idx = self.ch_idx();

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

        fn stop_tx(&self) -> bool {
            let rmt = crate::peripherals::RMT::regs();
            rmt.ch_tx_conf0(self.channel().into())
                .modify(|_, w| w.tx_stop().set_bit());
            self.update();
            true
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
            let ch_idx = self.ch_idx() as usize;
            super::INPUT_SIGNALS[ch_idx]
        }

        fn clear_rx_interrupts(&self) {
            let rmt = crate::peripherals::RMT::regs();
            let ch_idx = self.ch_idx();

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

        fn is_rx_done(&self) -> bool {
            let rmt = crate::peripherals::RMT::regs();
            let ch_idx = self.ch_idx();
            rmt.int_raw().read().ch_rx_end(ch_idx).bit()
        }

        fn is_rx_threshold_set(&self) -> bool {
            let rmt = crate::peripherals::RMT::regs();
            let ch_idx = self.ch_idx();
            rmt.int_raw().read().ch_rx_thr_event(ch_idx).bit()
        }

        fn reset_rx_threshold_set(&self) {
            let rmt = crate::peripherals::RMT::regs();
            let ch_idx = self.ch_idx();
            rmt.int_clr().write(|w| w.ch_rx_thr_event(ch_idx).set_bit());
        }

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

        // Ensure that this is always inlined in (un)listen_rx_interrupt
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
        RawChannelAccessExt,
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
    pub(super) fn handle_channel_interrupts(
        on_tx: fn(&DynChannelAccess<Tx>, Event),
        on_rx: fn(&DynChannelAccess<Rx>, Event),
    ) {
        let st = RMT::regs().int_st().read();

        for ch_idx in ChannelIndex::iter_all() {
            let (is_tx, event) = if st.ch_err(ch_idx as u8).bit() {
                // FIXME: Is it really necessary to determine tx/rx here?
                // Ultimately, we just need to signal the waker, so it doesn't really matter.
                if let Some(is_tx) =
                    RmtState::load_by_idx(ch_idx as u8, Ordering::Relaxed).is_tx()
                {
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
                on_tx(&raw, event);
            } else {
                let raw = unsafe { DynChannelAccess::<Rx>::conjure(ch_idx) };
                on_rx(&raw, event);
            }
        }
    }

    #[inline]
    #[allow(clippy::extra_unused_type_parameters)]
    pub(super) const fn channel_for_idx<Dir: Capability>(idx: u8) -> u8 {
        idx
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
            rmt.chconf0(self.ch_idx() as usize)
                .modify(|_, w| unsafe { w.div_cnt().bits(divider) });
        }

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

        fn is_error(&self) -> bool {
            let rmt = crate::peripherals::RMT::regs();
            rmt.int_raw().read().ch_err(self.ch_idx()).bit()
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
                rmt.ch_tx_lim(self.ch_idx() as usize)
                    .modify(|_, w| unsafe { w.tx_loop_num().bits(repeats) });
            } else {
                rmt.ch_tx_lim(self.ch_idx() as usize)
                    .modify(|_, w| unsafe { w.tx_loop_num().bits(0) });
            }
        }

        #[cfg(esp32)]
        fn set_generate_repeat_interrupt(&self, _repeats: u16) {
            // unsupported
        }

        fn clear_tx_interrupts(&self) {
            let rmt = crate::peripherals::RMT::regs();
            let ch = self.ch_idx();

            rmt.int_clr().write(|w| {
                w.ch_err(ch).set_bit();
                w.ch_tx_end(ch).set_bit();
                w.ch_tx_thr_event(ch).set_bit()
            });
        }

        fn set_tx_continuous(&self, continuous: bool) {
            let rmt = crate::peripherals::RMT::regs();

            rmt.chconf1(self.ch_idx() as usize)
                .modify(|_, w| w.tx_conti_mode().bit(continuous));
        }

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

        fn is_tx_done(&self) -> bool {
            let rmt = crate::peripherals::RMT::regs();
            rmt.int_raw().read().ch_tx_end(self.ch_idx()).bit()
        }

        fn is_tx_threshold_set(&self) -> bool {
            let rmt = crate::peripherals::RMT::regs();
            rmt.int_raw().read().ch_tx_thr_event(self.ch_idx()).bit()
        }

        fn reset_tx_threshold_set(&self) {
            let rmt = crate::peripherals::RMT::regs();
            rmt.int_clr()
                .write(|w| w.ch_tx_thr_event(self.ch_idx()).set_bit());
        }

        fn set_tx_threshold(&self, threshold: u8) {
            let rmt = crate::peripherals::RMT::regs();
            rmt.ch_tx_lim(self.ch_idx() as usize)
                .modify(|_, w| unsafe { w.tx_lim().bits(threshold as u16) });
        }

        fn is_tx_loopcount_interrupt_set(&self) -> bool {
            // no-op
            false
        }

        #[cfg(esp32s2)]
        fn stop_tx(&self) -> bool {
            let rmt = crate::peripherals::RMT::regs();
            rmt.chconf1(self.ch_idx() as usize)
                .modify(|_, w| w.tx_stop().set_bit());
            true
        }

        #[cfg(esp32)]
        fn stop_tx(&self) -> bool {
            let ptr = self.channel_ram_start();
            for idx in 0..self.memsize().codes() {
                unsafe {
                    ptr.add(idx).write_volatile(super::PulseCode::end_marker());
                }
            }
            false
        }

        // Ensure that this is always inlined in (un)listen_tx_interrupt
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
        fn clear_rx_interrupts(&self) {
            let rmt = crate::peripherals::RMT::regs();
            let ch = self.ch_idx();

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

        fn is_rx_done(&self) -> bool {
            let rmt = crate::peripherals::RMT::regs();
            rmt.int_raw().read().ch_rx_end(self.ch_idx()).bit()
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

        // Ensure that this is always inlined in (un)listen_rx_interrupt
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
