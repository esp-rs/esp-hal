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
//! # use esp_hal::rmt::{PulseCode, Rmt, TxChannel, TxChannelConfig, TxChannelCreator};
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
//! data[data.len() - 1] = PulseCode::empty();
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
//! # use esp_hal::rmt::{PulseCode, Rmt, RxChannel, RxChannelConfig, RxChannelCreator};
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
//! let mut data: [u32; 48] = [PulseCode::empty(); 48];
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
    pin::Pin,
    task::{Context, Poll},
};

use enumset::{EnumSet, EnumSetType};
use portable_atomic::{AtomicU8, Ordering};
#[cfg(place_rmt_driver_in_ram)]
use procmacros::ram;

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
}

///  Convenience trait to work with pulse codes.
pub trait PulseCode: crate::private::Sealed {
    /// Create a new instance
    fn new(level1: Level, length1: u16, level2: Level, length2: u16) -> Self;

    /// Create a new empty instance
    fn empty() -> Self;

    /// Set all levels and lengths to 0
    fn reset(&mut self);

    /// Logical output level in the first pulse code interval
    fn level1(&self) -> Level;

    /// Length of the first pulse code interval (in clock cycles)
    fn length1(&self) -> u16;

    /// Logical output level in the second pulse code interval
    fn level2(&self) -> Level;

    /// Length of the second pulse code interval (in clock cycles)
    fn length2(&self) -> u16;
}

impl PulseCode for u32 {
    fn new(level1: Level, length1: u16, level2: Level, length2: u16) -> Self {
        let level1 = ((bool::from(level1) as u32) << 15) | (length1 as u32 & 0b111_1111_1111_1111);
        let level2 = ((bool::from(level2) as u32) << 15) | (length2 as u32 & 0b111_1111_1111_1111);
        level1 | (level2 << 16)
    }

    fn empty() -> Self {
        0
    }

    fn reset(&mut self) {
        *self = 0
    }

    fn level1(&self) -> Level {
        (self & (1 << 15) != 0).into()
    }

    fn length1(&self) -> u16 {
        (self & 0b111_1111_1111_1111) as u16
    }

    fn level2(&self) -> Level {
        (self & (1 << 31) != 0).into()
    }

    fn length2(&self) -> u16 {
        ((self >> 16) & 0b111_1111_1111_1111) as u16
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
    fn is_tx() -> bool;
}

impl crate::private::Sealed for Tx {}

impl crate::private::Sealed for Rx {}

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

/// An identifier for one channel of the RMT peripherial.
pub trait RawChannelAccess: Clone + Copy + core::fmt::Debug + crate::private::Sealed {
    // Tx or Rx
    #[doc(hidden)]
    type Dir: Direction;

    #[doc(hidden)]
    fn channel(&self) -> u8;
}

/// A compile-time constant identifier for one channel of the RMT peripherial.
///
/// This is a ZST.
#[derive(Clone, Copy, Debug)]
pub struct ConstChannelAccess<Dir: Direction, const CHANNEL: u8> {
    _direction: PhantomData<Dir>,
}

/// Type-erased equivalent of ConstChannelAccess.
#[derive(Clone, Copy, Debug)]
pub struct DynChannelAccess<Dir: Direction> {
    channel: u8,
    _direction: PhantomData<Dir>,
}

impl<Dir: Direction, const CHANNEL: u8> crate::private::Sealed
    for ConstChannelAccess<Dir, CHANNEL>
{
}
impl<Dir: Direction> crate::private::Sealed for DynChannelAccess<Dir> {}

impl<Dir, const CHANNEL: u8> ConstChannelAccess<Dir, CHANNEL>
where
    Dir: Direction,
    ConstChannelAccess<Dir, CHANNEL>: RawChannelAccess,
{
    const unsafe fn conjure() -> Self {
        Self {
            _direction: PhantomData,
        }
    }
}

impl<Dir: Direction> DynChannelAccess<Dir> {
    #[inline]
    unsafe fn conjure(channel: u8) -> Self {
        Self {
            channel,
            _direction: PhantomData,
        }
    }
}

impl<const CHANNEL: u8> RawChannelAccess for ConstChannelAccess<Tx, CHANNEL> {
    type Dir = Tx;

    #[inline]
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

/// Alias for a type-erased channels configured for tx.
pub type AnyTxChannel<Dm> = Channel<Dm, DynChannelAccess<Tx>>;
/// Alias for a type-erased channels configured for rx.
pub type AnyRxChannel<Dm> = Channel<Dm, DynChannelAccess<Rx>>;
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

// Declare input/output signals in a const array.
macro_rules! declare_signals {
    ($name:ident, $type:ident, [$($entry:ident $(,)?)*; $count:expr]) => {
        pub(crate) const $name: [crate::gpio::$type; $count] = [
            $(crate::gpio::$type::$entry,)*
        ];
    };
}

macro_rules! declare_channels {
    // Final step of the macro, when all input has been iterated over: Declares the Rmt struct and
    // its constructor.
    (@define_rmt {
        () -> (
            [$($field_decl:tt)*],
            [$($field_init:tt)*]
        )
    }) => {
        /// RMT Instance
        pub struct Rmt<'d, Dm>
        where
            Dm: $crate::DriverMode,
        {
            pub(super) peripheral: $crate::peripherals::RMT<'d>,
            $($field_decl)+
            _mode: ::core::marker::PhantomData<Dm>,
        }

        impl<'d, Dm> Rmt<'d, Dm>
        where
            Dm: crate::DriverMode,
        {
            fn create(peripheral: crate::peripherals::RMT<'d>) -> Self {
                Self {
                    peripheral,
                    $($field_init)+
                    _mode: ::core::marker::PhantomData,
                }
            }
        }
    };

    // Iteration step of the macro when going through channels: Takes one entry from the channel
    // declaration and builds up field definitions for the Rmt struct as well as corresponding
    // initializers.
    (@define_rmt {
        ([$name:ident, $num:literal] $(, $($ch:tt),+)?)
        -> (
            [$($field_decl:tt)*],
            [$($field_init:tt)*]
        )
    }) => {
        declare_channels! (@define_rmt {
            ($($($ch),+)?) -> ([
                $($field_decl)*
                #[doc = concat!("RMT Channel ", $num)]
                pub $name: ChannelCreator<Dm, $num>,
            ], [
                $($field_init)*
                $name: $crate::rmt::ChannelCreator {
                    _mode: ::core::marker::PhantomData,
                    _guard: $crate::system::GenericPeripheralGuard::new(),
                },
            ])
        });
    };

    // The main entry-point of the macro: takes a comma-separated list of channels,
    // allowing for a trailing comma.
    ($($ch:tt),+ $(,)?) => {
        declare_channels! (@define_rmt { ($($ch),+) -> ([], []) });
    };
}

const NUM_CHANNELS: usize = if cfg!(any(esp32, esp32s3)) { 8 } else { 4 };

cfg_if::cfg_if! {
    if #[cfg(esp32)] {
        declare_channels!(
            [channel0, 0],
            [channel1, 1],
            [channel2, 2],
            [channel3, 3],
            [channel4, 4],
            [channel5, 5],
            [channel6, 6],
            [channel7, 7],
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
            [channel0, 0],
            [channel1, 1],
            [channel2, 2],
            [channel3, 3],
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
            [channel0, 0],
            [channel1, 1],
            [channel2, 2],
            [channel3, 3],
            [channel4, 4],
            [channel5, 5],
            [channel6, 6],
            [channel7, 7],
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
            [channel0, 0],
            [channel1, 1],
            [channel2, 2],
            [channel3, 3],
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

impl<'d, Dm> Rmt<'d, Dm>
where
    Dm: crate::DriverMode,
{
    pub(crate) fn new_internal(peripheral: RMT<'d>, frequency: Rate) -> Result<Self, Error> {
        let me = Rmt::create(peripheral);
        self::chip_specific::configure_clock(frequency)?;
        Ok(me)
    }
}

impl<'d> Rmt<'d, Blocking> {
    /// Create a new RMT instance
    pub fn new(peripheral: RMT<'d>, frequency: Rate) -> Result<Self, Error> {
        Self::new_internal(peripheral, frequency)
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

fn configure_rx_channel<'d>(
    raw: impl RxChannelInternal,
    pin: impl PeripheralInput<'d>,
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
    reserve_channel(raw.channel(), RmtState::Rx, memsize)?;

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

fn configure_tx_channel<'d>(
    raw: impl TxChannelInternal,
    pin: impl PeripheralOutput<'d>,
    config: TxChannelConfig,
) -> Result<(), Error> {
    let memsize = MemSize::from_blocks(config.memsize);
    reserve_channel(raw.channel(), RmtState::Tx, memsize)?;

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
#[repr(u8)]
enum RmtState {
    // The channel is not configured for either rx or tx, and its memory is available
    Unconfigured,

    // The channels is not in use, but one of the preceding channels is using its memory
    Reserved,

    // The channel is configured for rx
    Rx,

    // The channel is configured for tx
    Tx,
}

// This must only holds value of RmtState. However, we need atomic access, thus
// represent as AtomicU8.
static STATE: [AtomicU8; NUM_CHANNELS] =
    [const { AtomicU8::new(RmtState::Unconfigured as u8) }; NUM_CHANNELS];

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
    _guard: GenericPeripheralGuard<{ system::Peripheral::Rmt as u8 }>,
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

/// Creates a TX channel
pub trait TxChannelCreator<'d, Dm>
where
    Dm: crate::DriverMode,
{
    /// Type of the raw channel access token
    type Raw: TxChannelInternal;

    #[doc(hidden)]
    const RAW: Self::Raw;

    /// Configure the TX channel
    fn configure_tx(
        self,
        pin: impl PeripheralOutput<'d>,
        config: TxChannelConfig,
    ) -> Result<Channel<Dm, Self::Raw>, Error>
    where
        Self: Sized,
    {
        configure_tx_channel(Self::RAW, pin, config)?;
        Ok(Channel::new(Self::RAW))
    }
}

/// Creates a RX channel
pub trait RxChannelCreator<'d, Dm>
where
    Dm: crate::DriverMode,
{
    /// Type of the raw channel access token
    type Raw: RxChannelInternal;

    #[doc(hidden)]
    const RAW: Self::Raw;

    /// Configure the RX channel
    fn configure_rx(
        self,
        pin: impl PeripheralInput<'d>,
        config: RxChannelConfig,
    ) -> Result<Channel<Dm, Self::Raw>, Error>
    where
        Self: Sized,
    {
        configure_rx_channel(Self::RAW, pin, config)?;
        Ok(Channel::new(Self::RAW))
    }
}

/// An in-progress transaction for a single shot TX transaction.
pub struct SingleShotTxTransaction<'a, Raw>
where
    Raw: TxChannelInternal,
{
    channel: Channel<Blocking, Raw>,

    // The position in channel RAM to continue writing at; must be either
    // 0 or half the available RAM size if there's further data.
    // The position may be invalid if there's no data left.
    ram_index: usize,

    // Remaining data that has not yet been written to channel RAM. May be empty.
    remaining_data: &'a [u32],
}

impl<Raw> SingleShotTxTransaction<'_, Raw>
where
    Raw: TxChannelInternal,
{
    /// Wait for the transaction to complete
    #[cfg_attr(place_rmt_driver_in_ram, ram)]
    pub fn wait(mut self) -> Result<Channel<Blocking, Raw>, (Error, Channel<Blocking, Raw>)> {
        let raw = self.channel.raw;
        let memsize = raw.memsize().codes();

        while !self.remaining_data.is_empty() {
            // wait for TX-THR
            while !raw.is_tx_threshold_set() {
                if raw.is_tx_done() {
                    // Unexpectedly done, even though we have data left: For example, this could
                    // happen if there is a stop code inside the data and not just at the end.
                    return Err((Error::TransmissionError, self.channel));
                }
                if raw.is_error() {
                    // Not sure that this can happen? In any case, be sure that we don't lock up
                    // here in case it can.
                    return Err((Error::TransmissionError, self.channel));
                }
            }
            raw.reset_tx_threshold_set();

            // re-fill TX RAM
            let ptr = unsafe { raw.channel_ram_start().add(self.ram_index) };
            let count = self.remaining_data.len().min(memsize / 2);
            let (chunk, remaining) = self.remaining_data.split_at(count);
            for (idx, entry) in chunk.iter().enumerate() {
                unsafe {
                    ptr.add(idx).write_volatile(*entry);
                }
            }

            // If count == memsize / 2 codes were written, update ram_index as
            // - 0 -> memsize / 2
            // - memsize / 2 -> 0
            // Otherwise, for count < memsize / 2, the new position is invalid but the new
            // slice is empty and we won't use ram_index again.
            self.ram_index = memsize / 2 - self.ram_index;
            self.remaining_data = remaining;
            debug_assert!(
                self.ram_index == 0
                    || self.ram_index == memsize / 2
                    || self.remaining_data.is_empty()
            );
        }

        loop {
            if raw.is_error() {
                return Err((Error::TransmissionError, self.channel));
            }

            if raw.is_tx_done() {
                break;
            }
        }

        Ok(self.channel)
    }
}

/// An in-progress continuous TX transaction
pub struct ContinuousTxTransaction<Raw: TxChannelInternal> {
    channel: Channel<Blocking, Raw>,
}

impl<Raw: TxChannelInternal> ContinuousTxTransaction<Raw> {
    /// Stop transaction when the current iteration ends.
    #[cfg_attr(place_rmt_driver_in_ram, ram)]
    pub fn stop_next(self) -> Result<Channel<Blocking, Raw>, (Error, Channel<Blocking, Raw>)> {
        let raw = self.channel.raw;

        raw.set_tx_continuous(false);
        raw.update();

        loop {
            if raw.is_error() {
                return Err((Error::TransmissionError, self.channel));
            }

            if raw.is_tx_done() {
                break;
            }
        }

        Ok(self.channel)
    }

    /// Stop transaction as soon as possible.
    #[cfg_attr(place_rmt_driver_in_ram, ram)]
    pub fn stop(self) -> Result<Channel<Blocking, Raw>, (Error, Channel<Blocking, Raw>)> {
        let raw = self.channel.raw;

        raw.set_tx_continuous(false);
        raw.update();

        let ptr = raw.channel_ram_start();
        for idx in 0..raw.memsize().codes() {
            unsafe {
                ptr.add(idx).write_volatile(0);
            }
        }

        loop {
            if raw.is_error() {
                return Err((Error::TransmissionError, self.channel));
            }

            if raw.is_tx_done() {
                break;
            }
        }

        Ok(self.channel)
    }

    /// Check if the `loopcount` interrupt bit is set
    pub fn is_loopcount_interrupt_set(&self) -> bool {
        self.channel.raw.is_tx_loopcount_interrupt_set()
    }
}

macro_rules! impl_tx_channel_creator {
    ($channel:literal) => {
        impl<'d, Dm> $crate::rmt::TxChannelCreator<'d, Dm>
            for $crate::rmt::ChannelCreator<Dm, $channel>
        where
            Dm: $crate::DriverMode,
        {
            type Raw = $crate::rmt::ConstChannelAccess<$crate::rmt::Tx, $channel>;
            const RAW: Self::Raw = unsafe { $crate::rmt::ConstChannelAccess::conjure() };
        }
    };
}

macro_rules! impl_rx_channel_creator {
    ($channel:literal) => {
        impl<'d, Dm> $crate::rmt::RxChannelCreator<'d, Dm>
            for $crate::rmt::ChannelCreator<Dm, $channel>
        where
            Dm: $crate::DriverMode,
        {
            type Raw = $crate::rmt::ConstChannelAccess<$crate::rmt::Rx, $channel>;
            const RAW: Self::Raw = unsafe { $crate::rmt::ConstChannelAccess::conjure() };
        }
    };
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

#[cfg(not(any(esp32, esp32s2, esp32s3)))]
mod impl_for_chip {
    impl_tx_channel_creator!(0);
    impl_tx_channel_creator!(1);

    impl_rx_channel_creator!(2);
    impl_rx_channel_creator!(3);
}

#[cfg(esp32)]
mod impl_for_chip {
    impl_tx_channel_creator!(0);
    impl_tx_channel_creator!(1);
    impl_tx_channel_creator!(2);
    impl_tx_channel_creator!(3);
    impl_tx_channel_creator!(4);
    impl_tx_channel_creator!(5);
    impl_tx_channel_creator!(6);
    impl_tx_channel_creator!(7);

    impl_rx_channel_creator!(0);
    impl_rx_channel_creator!(1);
    impl_rx_channel_creator!(2);
    impl_rx_channel_creator!(3);
    impl_rx_channel_creator!(4);
    impl_rx_channel_creator!(5);
    impl_rx_channel_creator!(6);
    impl_rx_channel_creator!(7);
}

#[cfg(esp32s2)]
mod impl_for_chip {
    impl_tx_channel_creator!(0);
    impl_tx_channel_creator!(1);
    impl_tx_channel_creator!(2);
    impl_tx_channel_creator!(3);

    impl_rx_channel_creator!(0);
    impl_rx_channel_creator!(1);
    impl_rx_channel_creator!(2);
    impl_rx_channel_creator!(3);
}

#[cfg(esp32s3)]
mod impl_for_chip {
    impl_tx_channel_creator!(0);
    impl_tx_channel_creator!(1);
    impl_tx_channel_creator!(2);
    impl_tx_channel_creator!(3);

    impl_rx_channel_creator!(4);
    impl_rx_channel_creator!(5);
    impl_rx_channel_creator!(6);
    impl_rx_channel_creator!(7);
}

/// Channel in TX mode
pub trait TxChannel: Sized {
    /// Channel identifier of the implementing channel.
    type Raw: TxChannelInternal;

    /// Start transmitting the given pulse code sequence.
    /// This returns a [`SingleShotTxTransaction`] which can be used to wait for
    /// the transaction to complete and get back the channel for further
    /// use.
    fn transmit(self, data: &[u32]) -> Result<SingleShotTxTransaction<'_, Self::Raw>, Error>;

    /// Start transmitting the given pulse code continuously.
    /// This returns a [`ContinuousTxTransaction`] which can be used to stop the
    /// ongoing transmission and get back the channel for further use.
    /// The length of sequence cannot exceed the size of the allocated RMT RAM.
    fn transmit_continuously(
        self,
        data: &[u32],
    ) -> Result<ContinuousTxTransaction<Self::Raw>, Error>;

    /// Like [`Self::transmit_continuously`] but also sets a loop count.
    /// [`ContinuousTxTransaction`] can be used to check if the loop count is
    /// reached.
    fn transmit_continuously_with_loopcount(
        self,
        loopcount: u16,
        data: &[u32],
    ) -> Result<ContinuousTxTransaction<Self::Raw>, Error>;
}

impl<Raw> TxChannel for Channel<Blocking, Raw>
where
    Raw: TxChannelInternal,
{
    type Raw = Raw;

    #[cfg_attr(place_rmt_driver_in_ram, ram)]
    fn transmit(self, data: &[u32]) -> Result<SingleShotTxTransaction<'_, Raw>, Error> {
        let index = self.raw.start_send(data, false, 0)?;
        Ok(SingleShotTxTransaction {
            channel: self,
            // Either, remaining_data is empty, or we filled the entire buffer.
            ram_index: 0,
            remaining_data: &data[index..],
        })
    }

    #[inline]
    fn transmit_continuously(self, data: &[u32]) -> Result<ContinuousTxTransaction<Raw>, Error> {
        self.transmit_continuously_with_loopcount(0, data)
    }

    #[cfg_attr(place_rmt_driver_in_ram, ram)]
    fn transmit_continuously_with_loopcount(
        self,
        loopcount: u16,
        data: &[u32],
    ) -> Result<ContinuousTxTransaction<Raw>, Error> {
        if data.len() > self.raw.memsize().codes() {
            return Err(Error::Overflow);
        }

        let _index = self.raw.start_send(data, true, loopcount)?;
        Ok(ContinuousTxTransaction { channel: self })
    }
}

/// RX transaction instance
pub struct RxTransaction<'a, Raw: RxChannelInternal> {
    channel: Channel<Blocking, Raw>,
    data: &'a mut [u32],
}

impl<Raw: RxChannelInternal> RxTransaction<'_, Raw> {
    /// Wait for the transaction to complete
    #[cfg_attr(place_rmt_driver_in_ram, ram)]
    pub fn wait(self) -> Result<Channel<Blocking, Raw>, (Error, Channel<Blocking, Raw>)> {
        let raw = self.channel.raw;

        loop {
            if raw.is_error() {
                return Err((Error::ReceiverError, self.channel));
            }

            if raw.is_rx_done() {
                break;
            }
        }

        raw.stop_rx();
        raw.clear_rx_interrupts();
        raw.update();

        let ptr = raw.channel_ram_start();
        // SAFETY: RxChannel.receive() verifies that the length of self.data does not
        // exceed the channel RAM size.
        for (idx, entry) in self.data.iter_mut().enumerate() {
            *entry = unsafe { ptr.add(idx).read_volatile() };
        }

        Ok(self.channel)
    }
}

/// Channel is RX mode
pub trait RxChannel: Sized {
    /// Channel identifier of the implementing channel.
    type Raw: RxChannelInternal;

    /// Start receiving pulse codes into the given buffer.
    /// This returns a [RxTransaction] which can be used to wait for receive to
    /// complete and get back the channel for further use.
    /// The length of the received data cannot exceed the allocated RMT RAM.
    fn receive(self, data: &mut [u32]) -> Result<RxTransaction<'_, Self::Raw>, Error>;
}

impl<Raw> RxChannel for Channel<Blocking, Raw>
where
    Raw: RxChannelInternal,
{
    type Raw = Raw;

    #[cfg_attr(place_rmt_driver_in_ram, ram)]
    fn receive(self, data: &mut [u32]) -> Result<RxTransaction<'_, Self::Raw>, Error>
    where
        Self: Sized,
    {
        if data.len() > self.raw.memsize().codes() {
            return Err(Error::InvalidDataLength);
        }

        self.raw.start_receive();

        Ok(RxTransaction {
            channel: self,
            data,
        })
    }
}

static WAKER: [AtomicWaker; NUM_CHANNELS] = [const { AtomicWaker::new() }; NUM_CHANNELS];
#[must_use = "futures do nothing unless you `.await` or poll them"]
pub(crate) struct RmtTxFuture<Raw>
where
    Raw: TxChannelInternal,
{
    raw: Raw,
}

impl<Raw> core::future::Future for RmtTxFuture<Raw>
where
    Raw: TxChannelInternal,
{
    type Output = ();

    #[cfg_attr(place_rmt_driver_in_ram, ram)]
    fn poll(self: Pin<&mut Self>, ctx: &mut Context<'_>) -> Poll<Self::Output> {
        WAKER[self.raw.channel() as usize].register(ctx.waker());

        if self.raw.is_error() || self.raw.is_tx_done() {
            Poll::Ready(())
        } else {
            Poll::Pending
        }
    }
}

/// TX channel in async mode
pub trait TxChannelAsync {
    /// Start transmitting the given pulse code sequence.
    /// The length of sequence cannot exceed the size of the allocated RMT
    /// RAM.
    async fn transmit(&mut self, data: &[u32]) -> Result<(), Error>
    where
        Self: Sized;
}

impl<Raw> TxChannelAsync for Channel<Async, Raw>
where
    Raw: TxChannelInternal,
{
    #[cfg_attr(place_rmt_driver_in_ram, ram)]
    async fn transmit(&mut self, data: &[u32]) -> Result<(), Error>
    where
        Self: Sized,
    {
        let raw = self.raw;

        if data.len() > raw.memsize().codes() {
            return Err(Error::InvalidDataLength);
        }

        raw.clear_tx_interrupts();
        raw.listen_tx_interrupt(Event::End | Event::Error);
        raw.start_send(data, false, 0)?;

        (RmtTxFuture { raw }).await;

        if raw.is_error() {
            Err(Error::TransmissionError)
        } else {
            Ok(())
        }
    }
}

#[must_use = "futures do nothing unless you `.await` or poll them"]
pub(crate) struct RmtRxFuture<Raw>
where
    Raw: RxChannelInternal,
{
    raw: Raw,
}

impl<Raw> core::future::Future for RmtRxFuture<Raw>
where
    Raw: RxChannelInternal,
{
    type Output = ();

    #[cfg_attr(place_rmt_driver_in_ram, ram)]
    fn poll(self: Pin<&mut Self>, ctx: &mut Context<'_>) -> Poll<Self::Output> {
        WAKER[self.raw.channel() as usize].register(ctx.waker());
        if self.raw.is_error() || self.raw.is_rx_done() {
            Poll::Ready(())
        } else {
            Poll::Pending
        }
    }
}

/// RX channel in async mode
pub trait RxChannelAsync {
    /// Start receiving a pulse code sequence.
    /// The length of sequence cannot exceed the size of the allocated RMT
    /// RAM.
    async fn receive<T: From<u32> + Copy>(&mut self, data: &mut [T]) -> Result<(), Error>
    where
        Self: Sized;
}

impl<Raw> RxChannelAsync for Channel<Async, Raw>
where
    Raw: RxChannelInternal,
{
    #[cfg_attr(place_rmt_driver_in_ram, ram)]
    async fn receive<T: From<u32> + Copy>(&mut self, data: &mut [T]) -> Result<(), Error>
    where
        Self: Sized,
    {
        let raw = self.raw;

        if data.len() > raw.memsize().codes() {
            return Err(Error::InvalidDataLength);
        }

        raw.clear_rx_interrupts();
        raw.listen_rx_interrupt(Event::End | Event::Error);
        raw.start_receive();

        (RmtRxFuture { raw }).await;

        if raw.is_error() {
            Err(Error::ReceiverError)
        } else {
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
    }
}

#[cfg(not(any(esp32, esp32s2)))]
#[handler]
fn async_interrupt_handler() {
    let Some((channel, is_tx)) = chip_specific::pending_interrupt_for_channel() else {
        return;
    };
    if is_tx {
        unsafe { DynChannelAccess::<Tx>::conjure(channel) }
            .unlisten_tx_interrupt(Event::End | Event::Error);
    } else {
        unsafe { DynChannelAccess::<Rx>::conjure(channel) }
            .unlisten_rx_interrupt(Event::End | Event::Error);
    }

    WAKER[channel as usize].wake();
}

#[cfg(any(esp32, esp32s2))]
#[handler]
fn async_interrupt_handler() {
    let Some(channel) = chip_specific::pending_interrupt_for_channel() else {
        return;
    };

    unsafe { DynChannelAccess::<Tx>::conjure(channel) }
        .unlisten_tx_interrupt(Event::End | Event::Error);
    unsafe { DynChannelAccess::<Rx>::conjure(channel) }
        .unlisten_rx_interrupt(Event::End | Event::Error);

    WAKER[channel as usize].wake();
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
    fn channel_ram_start(&self) -> *mut u32 {
        unsafe {
            (property!("rmt.ram_start") as *mut u32)
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

    fn is_tx_done(&self) -> bool;

    fn is_tx_threshold_set(&self) -> bool;

    fn reset_tx_threshold_set(&self);

    fn set_tx_threshold(&self, threshold: u8);

    fn is_tx_loopcount_interrupt_set(&self) -> bool;

    #[inline]
    fn start_send(&self, data: &[u32], continuous: bool, repeat: u16) -> Result<usize, Error> {
        self.clear_tx_interrupts();

        if let Some(last) = data.last() {
            if !continuous && last.length2() != 0 && last.length1() != 0 {
                return Err(Error::EndMarkerMissing);
            }
        } else {
            return Err(Error::InvalidArgument);
        }

        let ptr = self.channel_ram_start();
        let memsize = self.memsize().codes();
        for (idx, entry) in data.iter().take(memsize).enumerate() {
            unsafe {
                ptr.add(idx).write_volatile(*entry);
            }
        }

        self.set_tx_threshold((memsize / 2) as u8);
        self.set_tx_continuous(continuous);
        self.set_generate_repeat_interrupt(repeat);
        self.set_tx_wrap_mode(true);
        self.update();
        self.start_tx();
        self.update();

        Ok(data.len().min(memsize))
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

    fn set_rx_wrap_mode(&self, wrap: bool);

    fn set_rx_carrier(&self, carrier: bool, high: u16, low: u16, level: Level);

    fn start_rx(&self);

    fn is_rx_done(&self) -> bool;

    fn start_receive(&self) {
        self.clear_rx_interrupts();
        self.set_rx_wrap_mode(false);
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

    #[allow(unused)]
    #[inline]
    pub(super) fn pending_interrupt_for_channel() -> Option<(u8, bool)> {
        let st = RMT::regs().int_st().read();

        for ch_idx in 0..NUM_CHANNELS as u8 / 2 {
            if st.ch_tx_end(ch_idx).bit() || st.ch_tx_err(ch_idx).bit() {
                // The first half of all channels support tx...
                let ch_num = ch_idx;
                return Some((ch_num, true));
            }
            if st.ch_rx_end(ch_idx).bit() || st.ch_rx_err(ch_idx).bit() {
                // ...whereas the second half of channels support rx.
                let ch_num = NUM_CHANNELS as u8 / 2 + ch_idx;
                return Some((ch_num, false));
            }
        }

        None
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

    impl<A> ChannelInternal for A
    where
        A: RawChannelAccess<Dir: Direction>,
    {
        #[inline]
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

        #[inline]
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

        #[inline]
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
            self.update();
        }

        #[inline]
        fn is_tx_done(&self) -> bool {
            let rmt = crate::peripherals::RMT::regs();
            rmt.int_raw().read().ch_tx_end(self.channel()).bit()
        }

        #[inline]
        fn is_tx_threshold_set(&self) -> bool {
            let rmt = crate::peripherals::RMT::regs();
            rmt.int_raw().read().ch_tx_thr_event(self.channel()).bit()
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
        fn stop_tx(&self) {
            let rmt = crate::peripherals::RMT::regs();
            rmt.ch_tx_conf0(self.channel().into())
                .modify(|_, w| w.tx_stop().set_bit());
            self.update();
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
        fn input_signal(&self) -> crate::gpio::InputSignal {
            let ch_idx = ch_idx(self) as usize;
            super::INPUT_SIGNALS[ch_idx]
        }

        #[inline]
        fn clear_rx_interrupts(&self) {
            let rmt = crate::peripherals::RMT::regs();
            let ch_idx = ch_idx(self);

            rmt.int_clr().write(|w| {
                w.ch_rx_end(ch_idx).set_bit();
                w.ch_rx_err(ch_idx).set_bit();
                w.ch_rx_thr_event(ch_idx).set_bit()
            });
        }

        #[inline]
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

        #[inline]
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

        #[inline]
        fn is_rx_done(&self) -> bool {
            let rmt = crate::peripherals::RMT::regs();
            let ch_idx = ch_idx(self);
            rmt.int_raw().read().ch_rx_end(ch_idx).bit()
        }

        #[inline]
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

    use super::{
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
    #[inline]
    pub(super) fn pending_interrupt_for_channel() -> Option<u8> {
        let rmt = RMT::regs();
        let st = rmt.int_st().read();

        (0..NUM_CHANNELS as u8).find(|&ch_num| {
            st.ch_rx_end(ch_num).bit() || st.ch_tx_end(ch_num).bit() || st.ch_err(ch_num).bit()
        })
    }

    impl<A> ChannelInternal for A
    where
        A: RawChannelAccess<Dir: Direction>,
    {
        #[inline]
        fn update(&self) {
            // no-op
        }

        fn set_divider(&self, divider: u8) {
            let rmt = crate::peripherals::RMT::regs();
            rmt.chconf0(self.channel() as usize)
                .modify(|_, w| unsafe { w.div_cnt().bits(divider) });
        }

        #[inline]
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

        #[inline]
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
        #[inline]
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
        #[inline]
        fn set_generate_repeat_interrupt(&self, _repeats: u16) {
            // unsupported
        }

        #[inline]
        fn clear_tx_interrupts(&self) {
            let rmt = crate::peripherals::RMT::regs();
            let ch = self.channel();

            rmt.int_clr().write(|w| {
                w.ch_err(ch).set_bit();
                w.ch_tx_end(ch).set_bit();
                w.ch_tx_thr_event(ch).set_bit()
            });
        }

        #[inline]
        fn set_tx_continuous(&self, continuous: bool) {
            let rmt = crate::peripherals::RMT::regs();

            rmt.chconf1(self.channel() as usize)
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

        #[inline]
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

        #[inline]
        fn is_tx_done(&self) -> bool {
            let rmt = crate::peripherals::RMT::regs();
            rmt.int_raw().read().ch_tx_end(self.channel()).bit()
        }

        #[inline]
        fn is_tx_threshold_set(&self) -> bool {
            let rmt = crate::peripherals::RMT::regs();
            rmt.int_raw().read().ch_tx_thr_event(self.channel()).bit()
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
            rmt.ch_tx_lim(self.channel() as usize)
                .modify(|_, w| unsafe { w.tx_lim().bits(threshold as u16) });
        }

        #[inline]
        fn is_tx_loopcount_interrupt_set(&self) -> bool {
            // no-op
            false
        }

        #[cfg(esp32s2)]
        #[inline]
        fn stop_tx(&self) {
            let rmt = crate::peripherals::RMT::regs();
            rmt.chconf1(self.channel() as usize)
                .modify(|_, w| w.tx_stop().set_bit());
        }

        #[cfg(esp32)]
        #[inline]
        fn stop_tx(&self) {}

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
        #[inline]
        fn clear_rx_interrupts(&self) {
            let rmt = crate::peripherals::RMT::regs();
            let ch = self.channel();

            rmt.int_clr().write(|w| {
                w.ch_rx_end(ch).set_bit();
                w.ch_err(ch).set_bit();
                w.ch_tx_thr_event(ch).set_bit()
            });
        }

        #[inline]
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

        #[inline]
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

        #[inline]
        fn is_rx_done(&self) -> bool {
            let rmt = crate::peripherals::RMT::regs();
            rmt.int_raw().read().ch_rx_end(self.channel()).bit()
        }

        #[inline]
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
