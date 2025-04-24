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
#![doc = concat!("[ESP-IDF documentation](https://docs.espressif.com/projects/esp-idf/en/latest/", crate::chip!(), "/api-reference/peripherals/rmt.html)")]
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
//!     .configure(
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
//!     .configure(peripherals.GPIO4, tx_config)?;
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
    doc = "let mut channel = rmt.channel0.configure(peripherals.GPIO4, rx_config)?;"
)]
#![cfg_attr(
    esp32s3,
    doc = "let mut channel = rmt.channel7.configure(peripherals.GPIO4, rx_config)?;"
)]
#![cfg_attr(
    not(any(esp32, esp32s2, esp32s3)),
    doc = "let mut channel = rmt.channel2.configure(peripherals.GPIO4, rx_config)?;"
)]
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

use crate::{
    Async,
    Blocking,
    asynch::AtomicWaker,
    gpio::{
        Level,
        interconnect::{PeripheralInput, PeripheralOutput},
    },
    handler,
    peripherals::{Interrupt, RMT},
    soc::constants,
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
            memsize: 1u8,
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

pub use impl_for_chip::{ChannelCreator, Rmt};

impl<'d, Dm> Rmt<'d, Dm>
where
    Dm: crate::DriverMode,
{
    pub(crate) fn new_internal(peripheral: RMT<'d>, frequency: Rate) -> Result<Self, Error> {
        let me = Rmt::create(peripheral);
        me.configure_clock(frequency)?;
        me.clear_memsizes();
        Ok(me)
    }

    #[cfg(any(esp32, esp32s2))]
    fn configure_clock(&self, frequency: Rate) -> Result<(), Error> {
        if frequency != Rate::from_mhz(80) {
            return Err(Error::UnreachableTargetFrequency);
        }

        self::chip_specific::configure_clock();

        Ok(())
    }

    // A memsize = 0 indicates that the channel memory is available
    #[cfg(any(esp32, esp32s2))]
    fn clear_memsizes(&self) {
        let rmt = crate::peripherals::RMT::regs();
        for i in 0..NUM_CHANNELS {
            rmt.chconf0(i)
                .modify(|_, w| unsafe { w.mem_size().bits(0) });
        }
    }

    // A memsize = 0 indicates that the channel memory is available
    #[cfg(not(any(esp32, esp32s2)))]
    fn clear_memsizes(&self) {
        let rmt = crate::peripherals::RMT::regs();
        for i in 0..NUM_CHANNELS / 2 {
            rmt.ch_tx_conf0(i)
                .modify(|_, w| unsafe { w.mem_size().bits(0) });
        }
        for i in 0..NUM_CHANNELS / 2 {
            rmt.ch_rx_conf0(i)
                .modify(|_, w| unsafe { w.mem_size().bits(0) });
        }
    }

    #[cfg(not(any(esp32, esp32s2)))]
    fn configure_clock(&self, frequency: Rate) -> Result<(), Error> {
        let src_clock = crate::soc::constants::RMT_CLOCK_SRC_FREQ;

        if frequency > src_clock {
            return Err(Error::UnreachableTargetFrequency);
        }

        let div = (src_clock / frequency) - 1;

        if div > u8::MAX as u32 {
            return Err(Error::UnreachableTargetFrequency);
        }

        self::chip_specific::configure_clock(div);

        Ok(())
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

fn configure_rx_channel<'d, T: RxChannelInternal>(
    pin: impl PeripheralInput<'d>,
    config: RxChannelConfig,
) -> Result<T, Error> {
    cfg_if::cfg_if! {
        if #[cfg(any(esp32, esp32s2))] {
            let threshold = 0b111_1111_1111_1111;
        } else {
            let threshold = 0b11_1111_1111_1111;
        }
    }

    if config.idle_threshold > threshold {
        return Err(Error::InvalidArgument);
    }

    if config.memsize > NUM_CHANNELS as u8 {
        return Err(Error::InvalidMemsize);
    }

    if config.memsize > NUM_CHANNELS as u8 - T::CHANNEL {
        return Err(Error::InvalidMemsize);
    }

    // If a channel's memory block is available then its memory size will be 0
    if !T::is_memory_blocks_available(config.memsize) {
        return Err(Error::MemoryBlockNotAvailable);
    }

    // If configured to use extended memory blocks then set the extended memory
    // blocks to 1 to indicate they are unavailable. Setting the memsize for
    // this channel will be set further down in the code
    T::set_memory_blocks_unavailable(config.memsize);

    let pin = pin.into();
    pin.init_input(crate::gpio::Pull::None);
    T::input_signal().connect_to(&pin);

    T::set_divider(config.clk_divider);
    T::set_carrier(
        config.carrier_modulation,
        config.carrier_high,
        config.carrier_low,
        config.carrier_level,
    );
    T::set_filter_threshold(config.filter_threshold);
    T::set_idle_threshold(config.idle_threshold);
    T::set_memsize(config.memsize);

    Ok(T::new())
}

fn configure_tx_channel<'d, T: TxChannelInternal>(
    pin: impl PeripheralOutput<'d>,
    config: TxChannelConfig,
) -> Result<T, Error> {
    if config.memsize > NUM_CHANNELS as u8 {
        return Err(Error::InvalidMemsize);
    }

    if config.memsize > NUM_CHANNELS as u8 - T::CHANNEL {
        return Err(Error::InvalidMemsize);
    }

    // If a channel's memory block is available then its memory size will be 0
    if !T::is_memory_blocks_available(config.memsize) {
        return Err(Error::MemoryBlockNotAvailable);
    }

    // If configured to use extended memory blocks then set the extended memory
    // blocks to 1 to indicate they are unavailable. Setting the memsize for
    // this channel will be set further down in the code
    T::set_memory_blocks_unavailable(config.memsize);

    let pin = pin.into();
    pin.set_to_push_pull_output();
    T::output_signal().connect_to(&pin);

    T::set_divider(config.clk_divider);
    T::set_carrier(
        config.carrier_modulation,
        config.carrier_high,
        config.carrier_low,
        config.carrier_level,
    );
    T::set_idle_output(config.idle_output, config.idle_output_level);
    T::set_memsize(config.memsize);

    Ok(T::new())
}

/// Creates a TX channel
pub trait TxChannelCreator<'d, T>
where
    T: TxChannel,
{
    /// Configure the TX channel
    fn configure(self, pin: impl PeripheralOutput<'d>, config: TxChannelConfig) -> Result<T, Error>
    where
        Self: Sized,
    {
        configure_tx_channel(pin, config)
    }
}

/// Creates a TX channel in async mode
pub trait TxChannelCreatorAsync<'d, T>
where
    T: TxChannelAsync,
{
    /// Configure the TX channel
    fn configure(self, pin: impl PeripheralOutput<'d>, config: TxChannelConfig) -> Result<T, Error>
    where
        Self: Sized,
    {
        configure_tx_channel(pin, config)
    }
}

/// Creates a RX channel
pub trait RxChannelCreator<'d, T>
where
    T: RxChannel,
{
    /// Configure the RX channel
    fn configure(self, pin: impl PeripheralInput<'d>, config: RxChannelConfig) -> Result<T, Error>
    where
        Self: Sized,
    {
        configure_rx_channel(pin, config)
    }
}

/// Creates a RX channel in async mode
pub trait RxChannelCreatorAsync<'d, T>
where
    T: RxChannelAsync,
{
    /// Configure the RX channel
    fn configure(self, pin: impl PeripheralInput<'d>, config: RxChannelConfig) -> Result<T, Error>
    where
        Self: Sized,
    {
        configure_rx_channel(pin, config)
    }
}

/// An in-progress transaction for a single shot TX transaction.
pub struct SingleShotTxTransaction<'a, C>
where
    C: TxChannel,
{
    channel: C,
    index: usize,
    data: &'a [u32],
}

impl<C> SingleShotTxTransaction<'_, C>
where
    C: TxChannel,
{
    /// Wait for the transaction to complete
    pub fn wait(mut self) -> Result<C, (Error, C)> {
        loop {
            if <C as TxChannelInternal>::is_error() {
                return Err((Error::TransmissionError, self.channel));
            }

            if self.index >= self.data.len() {
                break;
            }

            // wait for TX-THR
            while !<C as TxChannelInternal>::is_threshold_set() {}
            <C as TxChannelInternal>::reset_threshold_set();

            // re-fill TX RAM
            let ram_index = (((self.index
                - constants::RMT_CHANNEL_RAM_SIZE * <C as TxChannelInternal>::memsize() as usize)
                / (constants::RMT_CHANNEL_RAM_SIZE
                    * <C as TxChannelInternal>::memsize() as usize
                    / 2))
                % 2)
                * (constants::RMT_CHANNEL_RAM_SIZE * <C as TxChannelInternal>::memsize() as usize
                    / 2);

            let ptr = (constants::RMT_RAM_START
                + C::CHANNEL as usize * constants::RMT_CHANNEL_RAM_SIZE * 4
                + ram_index * 4) as *mut u32;
            for (idx, entry) in self.data[self.index..]
                .iter()
                .take(constants::RMT_CHANNEL_RAM_SIZE / 2)
                .enumerate()
            {
                unsafe {
                    ptr.add(idx).write_volatile(*entry);
                }
            }

            self.index +=
                constants::RMT_CHANNEL_RAM_SIZE * <C as TxChannelInternal>::memsize() as usize / 2;
        }

        loop {
            if <C as TxChannelInternal>::is_error() {
                return Err((Error::TransmissionError, self.channel));
            }

            if <C as TxChannelInternal>::is_done() {
                break;
            }
        }

        Ok(self.channel)
    }
}

/// An in-progress continuous TX transaction
pub struct ContinuousTxTransaction<C>
where
    C: TxChannel,
{
    channel: C,
}

impl<C> ContinuousTxTransaction<C>
where
    C: TxChannel,
{
    /// Stop transaction when the current iteration ends.
    pub fn stop_next(self) -> Result<C, (Error, C)> {
        <C as TxChannelInternal>::set_continuous(false);
        <C as TxChannelInternal>::update();

        loop {
            if <C as TxChannelInternal>::is_error() {
                return Err((Error::TransmissionError, self.channel));
            }

            if <C as TxChannelInternal>::is_done() {
                break;
            }
        }

        Ok(self.channel)
    }

    /// Stop transaction as soon as possible.
    pub fn stop(self) -> Result<C, (Error, C)> {
        <C as TxChannelInternal>::set_continuous(false);
        <C as TxChannelInternal>::update();

        let ptr = (constants::RMT_RAM_START
            + C::CHANNEL as usize * constants::RMT_CHANNEL_RAM_SIZE * 4)
            as *mut u32;
        for idx in 0..constants::RMT_CHANNEL_RAM_SIZE * <C as TxChannelInternal>::memsize() as usize
        {
            unsafe {
                ptr.add(idx).write_volatile(0);
            }
        }

        loop {
            if <C as TxChannelInternal>::is_error() {
                return Err((Error::TransmissionError, self.channel));
            }

            if <C as TxChannelInternal>::is_done() {
                break;
            }
        }

        Ok(self.channel)
    }

    /// Check if the `loopcount` interrupt bit is set
    pub fn is_loopcount_interrupt_set(&self) -> bool {
        <C as TxChannelInternal>::is_loopcount_interrupt_set()
    }
}

macro_rules! impl_tx_channel_creator {
    ($channel:literal) => {
        impl<'d> $crate::rmt::TxChannelCreator<'d, $crate::rmt::Channel<$crate::Blocking, $channel>>
            for ChannelCreator<$crate::Blocking, $channel>
        {
        }

        impl $crate::rmt::TxChannel for $crate::rmt::Channel<$crate::Blocking, $channel> {}

        impl<'d>
            $crate::rmt::TxChannelCreatorAsync<'d, $crate::rmt::Channel<$crate::Async, $channel>>
            for ChannelCreator<$crate::Async, $channel>
        {
        }

        impl $crate::rmt::TxChannelAsync for $crate::rmt::Channel<$crate::Async, $channel> {}
    };
}

macro_rules! impl_rx_channel_creator {
    ($channel:literal) => {
        impl<'d> $crate::rmt::RxChannelCreator<'d, $crate::rmt::Channel<$crate::Blocking, $channel>>
            for ChannelCreator<$crate::Blocking, $channel>
        {
        }

        impl $crate::rmt::RxChannel for $crate::rmt::Channel<$crate::Blocking, $channel> {}

        impl<'d>
            $crate::rmt::RxChannelCreatorAsync<'d, $crate::rmt::Channel<$crate::Async, $channel>>
            for ChannelCreator<$crate::Async, $channel>
        {
        }

        impl $crate::rmt::RxChannelAsync for $crate::rmt::Channel<$crate::Async, $channel> {}
    };
}

#[cfg(not(any(esp32, esp32s2, esp32s3)))]
mod impl_for_chip {
    use core::marker::PhantomData;

    use crate::system::GenericPeripheralGuard;

    /// RMT Instance
    pub struct Rmt<'d, Dm>
    where
        Dm: crate::DriverMode,
    {
        pub(super) peripheral: crate::peripherals::RMT<'d>,
        /// RMT Channel 0.
        pub channel0: ChannelCreator<Dm, 0>,
        /// RMT Channel 1.
        pub channel1: ChannelCreator<Dm, 1>,
        /// RMT Channel 2.
        pub channel2: ChannelCreator<Dm, 2>,
        /// RMT Channel 3.
        pub channel3: ChannelCreator<Dm, 3>,
        phantom: PhantomData<Dm>,
    }

    impl<'d, Dm> Rmt<'d, Dm>
    where
        Dm: crate::DriverMode,
    {
        pub(super) fn create(peripheral: crate::peripherals::RMT<'d>) -> Self {
            Self {
                peripheral,
                channel0: ChannelCreator {
                    phantom: PhantomData,
                    _guard: GenericPeripheralGuard::new(),
                },
                channel1: ChannelCreator {
                    phantom: PhantomData,
                    _guard: GenericPeripheralGuard::new(),
                },
                channel2: ChannelCreator {
                    phantom: PhantomData,
                    _guard: GenericPeripheralGuard::new(),
                },
                channel3: ChannelCreator {
                    phantom: PhantomData,
                    _guard: GenericPeripheralGuard::new(),
                },
                phantom: PhantomData,
            }
        }
    }

    /// RMT Channel Creator
    pub struct ChannelCreator<Dm, const CHANNEL: u8>
    where
        Dm: crate::DriverMode,
    {
        phantom: PhantomData<Dm>,
        _guard: GenericPeripheralGuard<{ crate::system::Peripheral::Rmt as u8 }>,
    }

    impl_tx_channel_creator!(0);
    impl_tx_channel_creator!(1);

    impl_rx_channel_creator!(2);
    impl_rx_channel_creator!(3);

    super::chip_specific::impl_tx_channel!(RMT_SIG_0, 0);
    super::chip_specific::impl_tx_channel!(RMT_SIG_1, 1);

    super::chip_specific::impl_rx_channel!(RMT_SIG_0, 2, 0);
    super::chip_specific::impl_rx_channel!(RMT_SIG_1, 3, 1);
}

#[cfg(esp32)]
mod impl_for_chip {
    use core::marker::PhantomData;

    use crate::{peripherals::RMT, system::GenericPeripheralGuard};

    /// RMT Instance
    pub struct Rmt<'d, Dm>
    where
        Dm: crate::DriverMode,
    {
        pub(super) peripheral: RMT<'d>,
        /// RMT Channel 0.
        pub channel0: ChannelCreator<Dm, 0>,
        /// RMT Channel 1.
        pub channel1: ChannelCreator<Dm, 1>,
        /// RMT Channel 2.
        pub channel2: ChannelCreator<Dm, 2>,
        /// RMT Channel 3.
        pub channel3: ChannelCreator<Dm, 3>,
        /// RMT Channel 4.
        pub channel4: ChannelCreator<Dm, 4>,
        /// RMT Channel 5.
        pub channel5: ChannelCreator<Dm, 5>,
        /// RMT Channel 6.
        pub channel6: ChannelCreator<Dm, 6>,
        /// RMT Channel 7.
        pub channel7: ChannelCreator<Dm, 7>,
        phantom: PhantomData<Dm>,
    }

    impl<'d, Dm> Rmt<'d, Dm>
    where
        Dm: crate::DriverMode,
    {
        pub(super) fn create(peripheral: RMT<'d>) -> Self {
            Self {
                peripheral,
                channel0: ChannelCreator {
                    phantom: PhantomData,
                    _guard: GenericPeripheralGuard::new(),
                },
                channel1: ChannelCreator {
                    phantom: PhantomData,
                    _guard: GenericPeripheralGuard::new(),
                },
                channel2: ChannelCreator {
                    phantom: PhantomData,
                    _guard: GenericPeripheralGuard::new(),
                },
                channel3: ChannelCreator {
                    phantom: PhantomData,
                    _guard: GenericPeripheralGuard::new(),
                },
                channel4: ChannelCreator {
                    phantom: PhantomData,
                    _guard: GenericPeripheralGuard::new(),
                },
                channel5: ChannelCreator {
                    phantom: PhantomData,
                    _guard: GenericPeripheralGuard::new(),
                },
                channel6: ChannelCreator {
                    phantom: PhantomData,
                    _guard: GenericPeripheralGuard::new(),
                },
                channel7: ChannelCreator {
                    phantom: PhantomData,
                    _guard: GenericPeripheralGuard::new(),
                },
                phantom: PhantomData,
            }
        }
    }

    /// RMT Channel Creator
    pub struct ChannelCreator<Dm, const CHANNEL: u8>
    where
        Dm: crate::DriverMode,
    {
        phantom: PhantomData<Dm>,
        _guard: GenericPeripheralGuard<{ crate::system::Peripheral::Rmt as u8 }>,
    }

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

    super::chip_specific::impl_tx_channel!(RMT_SIG_0, 0);
    super::chip_specific::impl_tx_channel!(RMT_SIG_1, 1);
    super::chip_specific::impl_tx_channel!(RMT_SIG_2, 2);
    super::chip_specific::impl_tx_channel!(RMT_SIG_3, 3);
    super::chip_specific::impl_tx_channel!(RMT_SIG_4, 4);
    super::chip_specific::impl_tx_channel!(RMT_SIG_5, 5);
    super::chip_specific::impl_tx_channel!(RMT_SIG_6, 6);
    super::chip_specific::impl_tx_channel!(RMT_SIG_7, 7);

    super::chip_specific::impl_rx_channel!(RMT_SIG_0, 0);
    super::chip_specific::impl_rx_channel!(RMT_SIG_1, 1);
    super::chip_specific::impl_rx_channel!(RMT_SIG_2, 2);
    super::chip_specific::impl_rx_channel!(RMT_SIG_3, 3);
    super::chip_specific::impl_rx_channel!(RMT_SIG_4, 4);
    super::chip_specific::impl_rx_channel!(RMT_SIG_5, 5);
    super::chip_specific::impl_rx_channel!(RMT_SIG_6, 6);
    super::chip_specific::impl_rx_channel!(RMT_SIG_7, 7);
}

#[cfg(esp32s2)]
mod impl_for_chip {
    use core::marker::PhantomData;

    use crate::{peripherals::RMT, system::GenericPeripheralGuard};

    /// RMT Instance
    pub struct Rmt<'d, Dm>
    where
        Dm: crate::DriverMode,
    {
        pub(super) peripheral: RMT<'d>,
        /// RMT Channel 0.
        pub channel0: ChannelCreator<Dm, 0>,
        /// RMT Channel 1.
        pub channel1: ChannelCreator<Dm, 1>,
        /// RMT Channel 2.
        pub channel2: ChannelCreator<Dm, 2>,
        /// RMT Channel 3.
        pub channel3: ChannelCreator<Dm, 3>,
        phantom: PhantomData<Dm>,
    }

    impl<'d, Dm> Rmt<'d, Dm>
    where
        Dm: crate::DriverMode,
    {
        pub(super) fn create(peripheral: RMT<'d>) -> Self {
            Self {
                peripheral,
                channel0: ChannelCreator {
                    phantom: PhantomData,
                    _guard: GenericPeripheralGuard::new(),
                },
                channel1: ChannelCreator {
                    phantom: PhantomData,
                    _guard: GenericPeripheralGuard::new(),
                },
                channel2: ChannelCreator {
                    phantom: PhantomData,
                    _guard: GenericPeripheralGuard::new(),
                },
                channel3: ChannelCreator {
                    phantom: PhantomData,
                    _guard: GenericPeripheralGuard::new(),
                },
                phantom: PhantomData,
            }
        }
    }

    /// RMT Channel Creator
    pub struct ChannelCreator<Dm, const CHANNEL: u8>
    where
        Dm: crate::DriverMode,
    {
        phantom: PhantomData<Dm>,
        _guard: GenericPeripheralGuard<{ crate::system::Peripheral::Rmt as u8 }>,
    }

    impl_tx_channel_creator!(0);
    impl_tx_channel_creator!(1);
    impl_tx_channel_creator!(2);
    impl_tx_channel_creator!(3);

    impl_rx_channel_creator!(0);
    impl_rx_channel_creator!(1);
    impl_rx_channel_creator!(2);
    impl_rx_channel_creator!(3);

    super::chip_specific::impl_tx_channel!(RMT_SIG_0, 0);
    super::chip_specific::impl_tx_channel!(RMT_SIG_1, 1);
    super::chip_specific::impl_tx_channel!(RMT_SIG_2, 2);
    super::chip_specific::impl_tx_channel!(RMT_SIG_3, 3);

    super::chip_specific::impl_rx_channel!(RMT_SIG_0, 0);
    super::chip_specific::impl_rx_channel!(RMT_SIG_1, 1);
    super::chip_specific::impl_rx_channel!(RMT_SIG_2, 2);
    super::chip_specific::impl_rx_channel!(RMT_SIG_3, 3);
}

#[cfg(esp32s3)]
mod impl_for_chip {
    use core::marker::PhantomData;

    use crate::{peripherals::RMT, system::GenericPeripheralGuard};

    /// RMT Instance
    pub struct Rmt<'d, Dm>
    where
        Dm: crate::DriverMode,
    {
        pub(super) peripheral: RMT<'d>,
        /// RMT Channel 0.
        pub channel0: ChannelCreator<Dm, 0>,
        /// RMT Channel 1.
        pub channel1: ChannelCreator<Dm, 1>,
        /// RMT Channel 2.
        pub channel2: ChannelCreator<Dm, 2>,
        /// RMT Channel 3.
        pub channel3: ChannelCreator<Dm, 3>,
        /// RMT Channel 4.
        pub channel4: ChannelCreator<Dm, 4>,
        /// RMT Channel 5.
        pub channel5: ChannelCreator<Dm, 5>,
        /// RMT Channel 6.
        pub channel6: ChannelCreator<Dm, 6>,
        /// RMT Channel 7.
        pub channel7: ChannelCreator<Dm, 7>,
        phantom: PhantomData<Dm>,
    }

    impl<'d, Dm> Rmt<'d, Dm>
    where
        Dm: crate::DriverMode,
    {
        pub(super) fn create(peripheral: RMT<'d>) -> Self {
            Self {
                peripheral,
                channel0: ChannelCreator {
                    phantom: PhantomData,
                    _guard: GenericPeripheralGuard::new(),
                },
                channel1: ChannelCreator {
                    phantom: PhantomData,
                    _guard: GenericPeripheralGuard::new(),
                },
                channel2: ChannelCreator {
                    phantom: PhantomData,
                    _guard: GenericPeripheralGuard::new(),
                },
                channel3: ChannelCreator {
                    phantom: PhantomData,
                    _guard: GenericPeripheralGuard::new(),
                },
                channel4: ChannelCreator {
                    phantom: PhantomData,
                    _guard: GenericPeripheralGuard::new(),
                },
                channel5: ChannelCreator {
                    phantom: PhantomData,
                    _guard: GenericPeripheralGuard::new(),
                },
                channel6: ChannelCreator {
                    phantom: PhantomData,
                    _guard: GenericPeripheralGuard::new(),
                },
                channel7: ChannelCreator {
                    phantom: PhantomData,
                    _guard: GenericPeripheralGuard::new(),
                },
                phantom: PhantomData,
            }
        }
    }

    /// RMT Channel Creator
    pub struct ChannelCreator<Dm, const CHANNEL: u8>
    where
        Dm: crate::DriverMode,
    {
        phantom: PhantomData<Dm>,
        _guard: GenericPeripheralGuard<{ crate::system::Peripheral::Rmt as u8 }>,
    }

    impl_tx_channel_creator!(0);
    impl_tx_channel_creator!(1);
    impl_tx_channel_creator!(2);
    impl_tx_channel_creator!(3);

    impl_rx_channel_creator!(4);
    impl_rx_channel_creator!(5);
    impl_rx_channel_creator!(6);
    impl_rx_channel_creator!(7);

    super::chip_specific::impl_tx_channel!(RMT_SIG_0, 0);
    super::chip_specific::impl_tx_channel!(RMT_SIG_1, 1);
    super::chip_specific::impl_tx_channel!(RMT_SIG_2, 2);
    super::chip_specific::impl_tx_channel!(RMT_SIG_3, 3);

    super::chip_specific::impl_rx_channel!(RMT_SIG_0, 4, 0);
    super::chip_specific::impl_rx_channel!(RMT_SIG_1, 5, 1);
    super::chip_specific::impl_rx_channel!(RMT_SIG_2, 6, 2);
    super::chip_specific::impl_rx_channel!(RMT_SIG_3, 7, 3);
}

/// RMT Channel
#[derive(Debug)]
#[non_exhaustive]
pub struct Channel<Dm, const CHANNEL: u8>
where
    Dm: crate::DriverMode,
{
    phantom: PhantomData<Dm>,
    _guard: GenericPeripheralGuard<{ system::Peripheral::Rmt as u8 }>,
}

/// Channel in TX mode
pub trait TxChannel: TxChannelInternal {
    /// Start transmitting the given pulse code sequence.
    /// This returns a [`SingleShotTxTransaction`] which can be used to wait for
    /// the transaction to complete and get back the channel for further
    /// use.
    fn transmit(self, data: &[u32]) -> Result<SingleShotTxTransaction<'_, Self>, Error>
    where
        Self: Sized,
    {
        let index = Self::send_raw(data, false, 0)?;
        Ok(SingleShotTxTransaction {
            channel: self,
            index,
            data,
        })
    }

    /// Start transmitting the given pulse code continuously.
    /// This returns a [`ContinuousTxTransaction`] which can be used to stop the
    /// ongoing transmission and get back the channel for further use.
    /// The length of sequence cannot exceed the size of the allocated RMT RAM.
    fn transmit_continuously(self, data: &[u32]) -> Result<ContinuousTxTransaction<Self>, Error>
    where
        Self: Sized,
    {
        self.transmit_continuously_with_loopcount(0, data)
    }

    /// Like [`Self::transmit_continuously`] but also sets a loop count.
    /// [`ContinuousTxTransaction`] can be used to check if the loop count is
    /// reached.
    fn transmit_continuously_with_loopcount(
        self,
        loopcount: u16,
        data: &[u32],
    ) -> Result<ContinuousTxTransaction<Self>, Error>
    where
        Self: Sized,
    {
        if data.len() > constants::RMT_CHANNEL_RAM_SIZE * Self::memsize() as usize {
            return Err(Error::Overflow);
        }

        let _index = Self::send_raw(data, true, loopcount)?;
        Ok(ContinuousTxTransaction { channel: self })
    }
}

/// RX transaction instance
pub struct RxTransaction<'a, C>
where
    C: RxChannel,
{
    channel: C,
    data: &'a mut [u32],
}

impl<C> RxTransaction<'_, C>
where
    C: RxChannel,
{
    /// Wait for the transaction to complete
    pub fn wait(self) -> Result<C, (Error, C)> {
        loop {
            if <C as RxChannelInternal>::is_error() {
                return Err((Error::ReceiverError, self.channel));
            }

            if <C as RxChannelInternal>::is_done() {
                break;
            }
        }

        <C as RxChannelInternal>::stop();
        <C as RxChannelInternal>::clear_interrupts();
        <C as RxChannelInternal>::update();

        let ptr = (constants::RMT_RAM_START
            + C::CHANNEL as usize * constants::RMT_CHANNEL_RAM_SIZE * 4)
            as *mut u32;
        let len = self.data.len();
        for (idx, entry) in self.data.iter_mut().take(len).enumerate() {
            *entry = unsafe { ptr.add(idx).read_volatile() };
        }

        Ok(self.channel)
    }
}

/// Channel is RX mode
pub trait RxChannel: RxChannelInternal {
    /// Start receiving pulse codes into the given buffer.
    /// This returns a [RxTransaction] which can be used to wait for receive to
    /// complete and get back the channel for further use.
    /// The length of the received data cannot exceed the allocated RMT RAM.
    fn receive(self, data: &mut [u32]) -> Result<RxTransaction<'_, Self>, Error>
    where
        Self: Sized,
    {
        if data.len() > constants::RMT_CHANNEL_RAM_SIZE * Self::memsize() as usize {
            return Err(Error::InvalidDataLength);
        }

        Self::start_receive_raw();

        Ok(RxTransaction {
            channel: self,
            data,
        })
    }
}

#[cfg(any(esp32, esp32s3))]
const NUM_CHANNELS: usize = 8;
#[cfg(not(any(esp32, esp32s3)))]
const NUM_CHANNELS: usize = 4;

static WAKER: [AtomicWaker; NUM_CHANNELS] = [const { AtomicWaker::new() }; NUM_CHANNELS];

#[must_use = "futures do nothing unless you `.await` or poll them"]
pub(crate) struct RmtTxFuture<T>
where
    T: TxChannelAsync,
{
    _phantom: PhantomData<T>,
}

impl<T> RmtTxFuture<T>
where
    T: TxChannelAsync,
{
    pub fn new(_instance: &T) -> Self {
        Self {
            _phantom: PhantomData,
        }
    }
}

impl<T> core::future::Future for RmtTxFuture<T>
where
    T: TxChannelAsync,
{
    type Output = ();

    fn poll(self: Pin<&mut Self>, ctx: &mut Context<'_>) -> Poll<Self::Output> {
        WAKER[T::CHANNEL as usize].register(ctx.waker());

        if T::is_error() || T::is_done() {
            Poll::Ready(())
        } else {
            Poll::Pending
        }
    }
}

/// TX channel in async mode
pub trait TxChannelAsync: TxChannelInternal {
    /// Start transmitting the given pulse code sequence.
    /// The length of sequence cannot exceed the size of the allocated RMT
    /// RAM.
    async fn transmit(&mut self, data: &[u32]) -> Result<(), Error>
    where
        Self: Sized,
    {
        if data.len() > constants::RMT_CHANNEL_RAM_SIZE * Self::memsize() as usize {
            return Err(Error::InvalidDataLength);
        }

        Self::clear_interrupts();
        Self::listen_interrupt(Event::End | Event::Error);
        Self::send_raw(data, false, 0)?;

        RmtTxFuture::new(self).await;

        if Self::is_error() {
            Err(Error::TransmissionError)
        } else {
            Ok(())
        }
    }
}

#[must_use = "futures do nothing unless you `.await` or poll them"]
pub(crate) struct RmtRxFuture<T>
where
    T: RxChannelAsync,
{
    _phantom: PhantomData<T>,
}

impl<T> RmtRxFuture<T>
where
    T: RxChannelAsync,
{
    pub fn new(_instance: &T) -> Self {
        Self {
            _phantom: PhantomData,
        }
    }
}

impl<T> core::future::Future for RmtRxFuture<T>
where
    T: RxChannelAsync,
{
    type Output = ();

    fn poll(self: Pin<&mut Self>, ctx: &mut Context<'_>) -> Poll<Self::Output> {
        WAKER[T::CHANNEL as usize].register(ctx.waker());
        if T::is_error() || T::is_done() {
            Poll::Ready(())
        } else {
            Poll::Pending
        }
    }
}

/// RX channel in async mode
pub trait RxChannelAsync: RxChannelInternal {
    /// Start receiving a pulse code sequence.
    /// The length of sequence cannot exceed the size of the allocated RMT
    /// RAM.
    async fn receive<T: From<u32> + Copy>(&mut self, data: &mut [T]) -> Result<(), Error>
    where
        Self: Sized,
    {
        if data.len() > constants::RMT_CHANNEL_RAM_SIZE * Self::memsize() as usize {
            return Err(Error::InvalidDataLength);
        }

        Self::clear_interrupts();
        Self::listen_interrupt(Event::End | Event::Error);
        Self::start_receive_raw();

        RmtRxFuture::new(self).await;

        if Self::is_error() {
            Err(Error::ReceiverError)
        } else {
            Self::stop();
            Self::clear_interrupts();
            Self::update();

            let ptr = (constants::RMT_RAM_START
                + Self::CHANNEL as usize * constants::RMT_CHANNEL_RAM_SIZE * 4)
                as *mut u32;
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
    let Some(channel) = chip_specific::pending_interrupt_for_channel() else {
        return;
    };
    match channel {
        0 => Channel::<Async, 0>::unlisten_interrupt(Event::End | Event::Error),
        1 => Channel::<Async, 1>::unlisten_interrupt(Event::End | Event::Error),
        2 => Channel::<Async, 2>::unlisten_interrupt(Event::End | Event::Error),
        3 => Channel::<Async, 3>::unlisten_interrupt(Event::End | Event::Error),

        #[cfg(any(esp32, esp32s3))]
        4 => Channel::<Async, 4>::unlisten_interrupt(Event::End | Event::Error),
        #[cfg(any(esp32, esp32s3))]
        5 => Channel::<Async, 5>::unlisten_interrupt(Event::End | Event::Error),
        #[cfg(any(esp32, esp32s3))]
        6 => Channel::<Async, 6>::unlisten_interrupt(Event::End | Event::Error),
        #[cfg(any(esp32, esp32s3))]
        7 => Channel::<Async, 7>::unlisten_interrupt(Event::End | Event::Error),

        _ => unreachable!(),
    }

    WAKER[channel].wake();
}

#[cfg(any(esp32, esp32s2))]
#[handler]
fn async_interrupt_handler() {
    let Some(channel) = chip_specific::pending_interrupt_for_channel() else {
        return;
    };
    match channel {
        0 => {
            <Channel<Async, 0> as TxChannelInternal>::unlisten_interrupt(Event::End | Event::Error);
            <Channel<Async, 0> as RxChannelInternal>::unlisten_interrupt(Event::End | Event::Error);
        }
        1 => {
            <Channel<Async, 1> as TxChannelInternal>::unlisten_interrupt(Event::End | Event::Error);
            <Channel<Async, 1> as RxChannelInternal>::unlisten_interrupt(Event::End | Event::Error);
        }
        2 => {
            <Channel<Async, 2> as TxChannelInternal>::unlisten_interrupt(Event::End | Event::Error);
            <Channel<Async, 2> as RxChannelInternal>::unlisten_interrupt(Event::End | Event::Error);
        }
        3 => {
            <Channel<Async, 3> as TxChannelInternal>::unlisten_interrupt(Event::End | Event::Error);
            <Channel<Async, 3> as RxChannelInternal>::unlisten_interrupt(Event::End | Event::Error);
        }
        #[cfg(esp32)]
        4 => {
            <Channel<Async, 4> as TxChannelInternal>::unlisten_interrupt(Event::End | Event::Error);
            <Channel<Async, 4> as RxChannelInternal>::unlisten_interrupt(Event::End | Event::Error);
        }
        #[cfg(any(esp32, esp32s3))]
        5 => {
            <Channel<Async, 5> as TxChannelInternal>::unlisten_interrupt(Event::End | Event::Error);
            <Channel<Async, 5> as RxChannelInternal>::unlisten_interrupt(Event::End | Event::Error);
        }
        #[cfg(any(esp32, esp32s3))]
        6 => {
            <Channel<Async, 6> as TxChannelInternal>::unlisten_interrupt(Event::End | Event::Error);
            <Channel<Async, 6> as RxChannelInternal>::unlisten_interrupt(Event::End | Event::Error);
        }
        #[cfg(any(esp32, esp32s3))]
        7 => {
            <Channel<Async, 7> as TxChannelInternal>::unlisten_interrupt(Event::End | Event::Error);
            <Channel<Async, 7> as RxChannelInternal>::unlisten_interrupt(Event::End | Event::Error);
        }

        _ => unreachable!(),
    }

    WAKER[channel].wake();
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
pub trait TxChannelInternal {
    const CHANNEL: u8;

    fn new() -> Self;

    fn output_signal() -> crate::gpio::OutputSignal;

    fn set_divider(divider: u8);

    fn update();

    fn set_generate_repeat_interrupt(repeats: u16);

    fn clear_interrupts();

    fn set_continuous(continuous: bool);

    fn set_wrap_mode(wrap: bool);

    fn set_carrier(carrier: bool, high: u16, low: u16, level: Level);

    fn set_idle_output(enable: bool, level: Level);

    fn is_memory_blocks_available(memory_blocks_requested: u8) -> bool;

    fn set_memory_blocks_unavailable(num_memory_blocks: u8);

    fn set_memsize(memsize: u8);

    fn memsize() -> u8;

    fn start_tx();

    fn is_done() -> bool;

    fn is_error() -> bool;

    fn is_threshold_set() -> bool;

    fn reset_threshold_set();

    fn set_threshold(threshold: u8);

    fn is_loopcount_interrupt_set() -> bool;

    fn send_raw(data: &[u32], continuous: bool, repeat: u16) -> Result<usize, Error> {
        Self::clear_interrupts();

        if let Some(last) = data.last() {
            if !continuous && last.length2() != 0 && last.length1() != 0 {
                return Err(Error::EndMarkerMissing);
            }
        } else {
            return Err(Error::InvalidArgument);
        }

        let ptr = (constants::RMT_RAM_START
            + Self::CHANNEL as usize * constants::RMT_CHANNEL_RAM_SIZE * 4)
            as *mut u32;
        for (idx, entry) in data
            .iter()
            .take(constants::RMT_CHANNEL_RAM_SIZE * Self::memsize() as usize)
            .enumerate()
        {
            unsafe {
                ptr.add(idx).write_volatile(*entry);
            }
        }

        Self::set_threshold((constants::RMT_CHANNEL_RAM_SIZE * Self::memsize() as usize / 2) as u8);
        Self::set_continuous(continuous);
        Self::set_generate_repeat_interrupt(repeat);
        Self::set_wrap_mode(true);
        Self::update();
        Self::start_tx();
        Self::update();

        if data.len() >= constants::RMT_CHANNEL_RAM_SIZE * Self::memsize() as usize {
            Ok(constants::RMT_CHANNEL_RAM_SIZE * Self::memsize() as usize)
        } else {
            Ok(data.len())
        }
    }

    fn stop();

    fn enable_listen_interrupt(event: EnumSet<Event>, enable: bool);

    fn listen_interrupt(event: impl Into<EnumSet<Event>>) {
        Self::enable_listen_interrupt(event.into(), true);
    }

    fn unlisten_interrupt(event: impl Into<EnumSet<Event>>) {
        Self::enable_listen_interrupt(event.into(), false);
    }
}

#[doc(hidden)]
pub trait RxChannelInternal {
    const CHANNEL: u8;

    fn new() -> Self;

    fn input_signal() -> crate::gpio::InputSignal;

    fn set_divider(divider: u8);

    fn update();

    fn clear_interrupts();

    fn set_wrap_mode(wrap: bool);

    fn set_carrier(carrier: bool, high: u16, low: u16, level: Level);

    fn is_memory_blocks_available(memory_blocks_requested: u8) -> bool;

    fn set_memory_blocks_unavailable(num_memory_blocks: u8);

    fn set_memsize(value: u8);

    fn memsize() -> u8;

    fn start_rx();

    fn is_done() -> bool;

    fn is_error() -> bool;

    fn start_receive_raw() {
        Self::clear_interrupts();
        Self::set_wrap_mode(false);
        Self::start_rx();
        Self::update();
    }

    fn stop();

    fn set_filter_threshold(value: u8);

    fn set_idle_threshold(value: u16);

    fn enable_listen_interrupt(event: EnumSet<Event>, enable: bool);

    fn listen_interrupt(event: impl Into<EnumSet<Event>>) {
        Self::enable_listen_interrupt(event.into(), true);
    }

    fn unlisten_interrupt(event: impl Into<EnumSet<Event>>) {
        Self::enable_listen_interrupt(event.into(), false);
    }
}

#[cfg(not(any(esp32, esp32s2)))]
mod chip_specific {
    use crate::peripherals::RMT;

    pub fn configure_clock(div: u32) {
        #[cfg(not(pcr))]
        {
            RMT::regs().sys_conf().modify(|_, w| unsafe {
                w.clk_en().clear_bit();
                w.sclk_sel().bits(crate::soc::constants::RMT_CLOCK_SRC);
                w.sclk_div_num().bits(div as u8);
                w.sclk_div_a().bits(0);
                w.sclk_div_b().bits(0);
                w.apb_fifo_mask().set_bit()
            });
        }

        #[cfg(pcr)]
        {
            use crate::peripherals::PCR;
            PCR::regs().rmt_sclk_conf().modify(|_, w| unsafe {
                w.sclk_div_num().bits(div as u8);
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
    }

    #[allow(unused)]
    #[cfg(not(esp32s3))]
    pub fn pending_interrupt_for_channel() -> Option<usize> {
        let st = RMT::regs().int_st().read();

        if st.ch0_tx_end().bit() || st.ch0_tx_err().bit() {
            Some(0)
        } else if st.ch1_tx_end().bit() || st.ch1_tx_err().bit() {
            Some(1)
        } else if st.ch2_rx_end().bit() || st.ch2_rx_err().bit() {
            Some(2)
        } else if st.ch3_rx_end().bit() || st.ch3_rx_err().bit() {
            Some(3)
        } else {
            None
        }
    }

    #[allow(unused)]
    #[cfg(esp32s3)]
    pub fn pending_interrupt_for_channel() -> Option<usize> {
        let st = RMT::regs().int_st().read();

        if st.ch0_tx_end().bit() || st.ch0_tx_err().bit() {
            Some(0)
        } else if st.ch1_tx_end().bit() || st.ch1_tx_err().bit() {
            Some(1)
        } else if st.ch2_tx_end().bit() || st.ch2_tx_err().bit() {
            Some(2)
        } else if st.ch3_tx_end().bit() || st.ch3_tx_err().bit() {
            Some(3)
        } else if st.ch4_rx_end().bit() || st.ch4_rx_err().bit() {
            Some(4)
        } else if st.ch5_rx_end().bit() || st.ch5_rx_err().bit() {
            Some(5)
        } else if st.ch6_rx_end().bit() || st.ch6_rx_err().bit() {
            Some(6)
        } else if st.ch7_rx_end().bit() || st.ch7_rx_err().bit() {
            Some(7)
        } else {
            None
        }
    }

    macro_rules! impl_tx_channel {
        ($signal:ident, $ch_num:literal) => {
            impl<Dm> $crate::rmt::TxChannelInternal for $crate::rmt::Channel<Dm, $ch_num>
            where
                Dm: $crate::DriverMode,
            {
                const CHANNEL: u8 = $ch_num;

                fn new() -> Self {
                    let guard = GenericPeripheralGuard::new();
                    Self {
                        phantom: core::marker::PhantomData,
                        _guard: guard,
                    }
                }

                fn output_signal() -> crate::gpio::OutputSignal {
                    crate::gpio::OutputSignal::$signal
                }

                fn set_divider(divider: u8) {
                    let rmt = crate::peripherals::RMT::regs();
                    rmt.ch_tx_conf0($ch_num)
                        .modify(|_, w| unsafe { w.div_cnt().bits(divider) });
                }

                fn update() {
                    let rmt = crate::peripherals::RMT::regs();
                    rmt.ch_tx_conf0($ch_num)
                        .modify(|_, w| w.conf_update().set_bit());
                }

                fn set_generate_repeat_interrupt(repeats: u16) {
                    let rmt = crate::peripherals::RMT::regs();
                    if repeats > 1 {
                        rmt.ch_tx_lim($ch_num).modify(|_, w| unsafe {
                            w.loop_count_reset().set_bit();
                            w.tx_loop_cnt_en().set_bit();
                            w.tx_loop_num().bits(repeats)
                        });
                    } else {
                        rmt.ch_tx_lim($ch_num).modify(|_, w| unsafe {
                            w.loop_count_reset().set_bit();
                            w.tx_loop_cnt_en().clear_bit();
                            w.tx_loop_num().bits(0)
                        });
                    }

                    rmt.ch_tx_lim($ch_num)
                        .modify(|_, w| w.loop_count_reset().clear_bit());
                }

                fn clear_interrupts() {
                    let rmt = crate::peripherals::RMT::regs();

                    rmt.int_clr().write(|w| {
                        w.ch_tx_end($ch_num).set_bit();
                        w.ch_tx_err($ch_num).set_bit();
                        w.ch_tx_loop($ch_num).set_bit();
                        w.ch_tx_thr_event($ch_num).set_bit()
                    });
                }

                fn set_continuous(continuous: bool) {
                    let rmt = crate::peripherals::RMT::regs();

                    rmt.ch_tx_conf0($ch_num)
                        .modify(|_, w| w.tx_conti_mode().bit(continuous));
                }

                fn set_wrap_mode(wrap: bool) {
                    let rmt = crate::peripherals::RMT::regs();

                    rmt.ch_tx_conf0($ch_num)
                        .modify(|_, w| w.mem_tx_wrap_en().bit(wrap));
                }

                fn set_carrier(carrier: bool, high: u16, low: u16, level: $crate::gpio::Level) {
                    let rmt = crate::peripherals::RMT::regs();

                    rmt.chcarrier_duty($ch_num)
                        .write(|w| unsafe { w.carrier_high().bits(high).carrier_low().bits(low) });

                    rmt.ch_tx_conf0($ch_num).modify(|_, w| {
                        w.carrier_en().bit(carrier);
                        w.carrier_eff_en().set_bit();
                        w.carrier_out_lv().bit(level.into())
                    });
                }

                fn set_idle_output(enable: bool, level: $crate::gpio::Level) {
                    let rmt = crate::peripherals::RMT::regs();
                    rmt.ch_tx_conf0($ch_num)
                        .modify(|_, w| w.idle_out_en().bit(enable).idle_out_lv().bit(level.into()));
                }

                fn is_memory_blocks_available(memory_blocks_requested: u8) -> bool {
                    let rmt = crate::peripherals::RMT::regs();
                    let mut memory_block_is_available = true;

                    if memory_blocks_requested > 1 {
                        for i in $ch_num..$ch_num + memory_blocks_requested {
                            if rmt.ch_tx_conf0(i as usize).read().mem_size().bits() != 0 {
                                memory_block_is_available = false;
                                break;
                            }
                        }
                    } else {
                        memory_block_is_available =
                            rmt.ch_tx_conf0($ch_num).read().mem_size().bits() == 0;
                    }

                    memory_block_is_available
                }

                fn set_memory_blocks_unavailable(num_memory_blocks: u8) {
                    let rmt = crate::peripherals::RMT::regs();
                    if num_memory_blocks > 1 {
                        for i in $ch_num + 1..$ch_num + num_memory_blocks {
                            rmt.ch_tx_conf0(i as usize)
                                .modify(|_, w| unsafe { w.mem_size().bits(1) });
                        }
                    }
                }

                fn set_memsize(value: u8) {
                    let rmt = crate::peripherals::RMT::regs();

                    rmt.ch_tx_conf0($ch_num)
                        .modify(|_, w| unsafe { w.mem_size().bits(value) });
                }

                fn memsize() -> u8 {
                    let rmt = crate::peripherals::RMT::regs();
                    rmt.ch_tx_conf0($ch_num).read().mem_size().bits()
                }

                fn start_tx() {
                    let rmt = crate::peripherals::RMT::regs();

                    rmt.ref_cnt_rst().write(|w| unsafe { w.bits(1 << $ch_num) });
                    Self::update();

                    rmt.ch_tx_conf0($ch_num).modify(|_, w| {
                        w.mem_rd_rst().set_bit();
                        w.apb_mem_rst().set_bit();
                        w.tx_start().set_bit()
                    });
                    Self::update();
                }

                fn is_done() -> bool {
                    let rmt = crate::peripherals::RMT::regs();
                    rmt.int_raw().read().ch_tx_end($ch_num).bit()
                }

                fn is_error() -> bool {
                    let rmt = crate::peripherals::RMT::regs();
                    rmt.int_raw().read().ch_tx_err($ch_num).bit()
                }

                fn is_threshold_set() -> bool {
                    let rmt = crate::peripherals::RMT::regs();
                    rmt.int_raw().read().ch_tx_thr_event($ch_num).bit()
                }

                fn reset_threshold_set() {
                    let rmt = crate::peripherals::RMT::regs();
                    rmt.int_clr()
                        .write(|w| w.ch_tx_thr_event($ch_num).set_bit());
                }

                fn set_threshold(threshold: u8) {
                    let rmt = crate::peripherals::RMT::regs();
                    rmt.ch_tx_lim($ch_num)
                        .modify(|_, w| unsafe { w.tx_lim().bits(threshold as u16) });
                }

                fn is_loopcount_interrupt_set() -> bool {
                    let rmt = crate::peripherals::RMT::regs();
                    rmt.int_raw().read().ch_tx_loop($ch_num).bit()
                }

                fn stop() {
                    let rmt = crate::peripherals::RMT::regs();
                    rmt.ch_tx_conf0($ch_num)
                        .modify(|_, w| w.tx_stop().set_bit());
                    Self::update();
                }

                fn enable_listen_interrupt(
                    events: enumset::EnumSet<$crate::rmt::Event>,
                    enable: bool,
                ) {
                    let rmt = crate::peripherals::RMT::regs();
                    rmt.int_ena().modify(|_, w| {
                        if events.contains($crate::rmt::Event::Error) {
                            w.ch_tx_err($ch_num).bit(enable);
                        }
                        if events.contains($crate::rmt::Event::End) {
                            w.ch_tx_end($ch_num).bit(enable);
                        }
                        if events.contains($crate::rmt::Event::Threshold) {
                            w.ch_tx_thr_event($ch_num).bit(enable);
                        }
                        w
                    });
                }
            }
        };
    }

    macro_rules! impl_rx_channel {
        ($signal:ident, $ch_num:literal, $ch_index:literal) => {
            impl<Dm> $crate::rmt::RxChannelInternal for $crate::rmt::Channel<Dm, $ch_num>
            where
                Dm: $crate::DriverMode,
            {
                const CHANNEL: u8 = $ch_num;

                fn new() -> Self {
                    let guard = GenericPeripheralGuard::new();
                    Self {
                        phantom: core::marker::PhantomData,
                        _guard: guard,
                    }
                }

                fn input_signal() -> crate::gpio::InputSignal {
                    crate::gpio::InputSignal::$signal
                }

                fn set_divider(divider: u8) {
                    let rmt = crate::peripherals::RMT::regs();
                    rmt.ch_rx_conf0($ch_index)
                        .modify(|_, w| unsafe { w.div_cnt().bits(divider) });
                }

                fn update() {
                    let rmt = crate::peripherals::RMT::regs();
                    rmt.ch_rx_conf1($ch_index)
                        .modify(|_, w| w.conf_update().set_bit());
                }

                fn clear_interrupts() {
                    let rmt = crate::peripherals::RMT::regs();

                    rmt.int_clr().write(|w| {
                        w.ch_rx_end($ch_index).set_bit();
                        w.ch_rx_err($ch_index).set_bit();
                        w.ch_rx_thr_event($ch_index).set_bit()
                    });
                }

                fn set_wrap_mode(wrap: bool) {
                    let rmt = crate::peripherals::RMT::regs();
                    rmt.ch_rx_conf1($ch_index)
                        .modify(|_, w| w.mem_rx_wrap_en().bit(wrap));
                }

                fn set_carrier(carrier: bool, high: u16, low: u16, level: $crate::gpio::Level) {
                    let rmt = crate::peripherals::RMT::regs();

                    rmt.ch_rx_carrier_rm($ch_index).write(|w| unsafe {
                        w.carrier_high_thres().bits(high);
                        w.carrier_low_thres().bits(low)
                    });

                    rmt.ch_rx_conf0($ch_index).modify(|_, w| {
                        w.carrier_en()
                            .bit(carrier)
                            .carrier_out_lv()
                            .bit(level.into())
                    });
                }

                fn is_memory_blocks_available(memory_blocks_requested: u8) -> bool {
                    let rmt = crate::peripherals::RMT::regs();
                    let mut memory_block_is_available = true;

                    if memory_blocks_requested > 1 {
                        for i in $ch_index..$ch_index + memory_blocks_requested {
                            if rmt.ch_rx_conf0(i as usize).read().mem_size().bits() != 0 {
                                memory_block_is_available = false;
                                break;
                            }
                        }
                    } else {
                        memory_block_is_available =
                            rmt.ch_rx_conf0($ch_index).read().mem_size().bits() == 0;
                    }

                    memory_block_is_available
                }

                fn set_memory_blocks_unavailable(num_memory_blocks: u8) {
                    let rmt = crate::peripherals::RMT::regs();
                    if num_memory_blocks > 1 {
                        for i in $ch_index + 1..$ch_index + num_memory_blocks {
                            rmt.ch_rx_conf0(i as usize)
                                .modify(|_, w| unsafe { w.mem_size().bits(1) });
                        }
                    }
                }

                fn set_memsize(value: u8) {
                    let rmt = crate::peripherals::RMT::regs();
                    rmt.ch_rx_conf0($ch_index)
                        .modify(|_, w| unsafe { w.mem_size().bits(value) });
                }

                fn memsize() -> u8 {
                    let rmt = crate::peripherals::RMT::regs();
                    rmt.ch_rx_conf0($ch_index).read().mem_size().bits()
                }

                fn start_rx() {
                    let rmt = crate::peripherals::RMT::regs();

                    for i in 0..Self::memsize() {
                        rmt.ch_rx_conf1(($ch_index + i).into())
                            .modify(|_, w| w.mem_owner().set_bit());
                    }
                    rmt.ch_rx_conf1($ch_index).modify(|_, w| {
                        w.mem_wr_rst().set_bit();
                        w.apb_mem_rst().set_bit();
                        w.rx_en().set_bit()
                    });
                }

                fn is_done() -> bool {
                    let rmt = crate::peripherals::RMT::regs();
                    rmt.int_raw().read().ch_rx_end($ch_index).bit()
                }

                fn is_error() -> bool {
                    let rmt = crate::peripherals::RMT::regs();
                    rmt.int_raw().read().ch_rx_err($ch_index).bit()
                }

                fn stop() {
                    let rmt = crate::peripherals::RMT::regs();
                    rmt.ch_rx_conf1($ch_index)
                        .modify(|_, w| w.rx_en().clear_bit());
                }

                fn set_filter_threshold(value: u8) {
                    let rmt = crate::peripherals::RMT::regs();

                    rmt.ch_rx_conf1($ch_index).modify(|_, w| unsafe {
                        w.rx_filter_en().bit(value > 0);
                        w.rx_filter_thres().bits(value)
                    });
                }

                fn set_idle_threshold(value: u16) {
                    let rmt = crate::peripherals::RMT::regs();

                    rmt.ch_rx_conf0($ch_index)
                        .modify(|_, w| unsafe { w.idle_thres().bits(value) });
                }

                fn enable_listen_interrupt(
                    events: enumset::EnumSet<$crate::rmt::Event>,
                    enable: bool,
                ) {
                    let rmt = crate::peripherals::RMT::regs();
                    rmt.int_ena().modify(|_, w| {
                        if events.contains($crate::rmt::Event::Error) {
                            w.ch_rx_err($ch_index).bit(enable);
                        }
                        if events.contains($crate::rmt::Event::End) {
                            w.ch_rx_end($ch_index).bit(enable);
                        }
                        if events.contains($crate::rmt::Event::Threshold) {
                            w.ch_rx_thr_event($ch_index).bit(enable);
                        }
                        w
                    });
                }
            }
        };
    }

    pub(crate) use impl_rx_channel;
    pub(crate) use impl_tx_channel;
}

#[cfg(any(esp32, esp32s2))]
mod chip_specific {
    use crate::peripherals::RMT;

    pub fn configure_clock() {
        let rmt = RMT::regs();

        rmt.ch0conf1().modify(|_, w| w.ref_always_on().set_bit());
        rmt.ch1conf1().modify(|_, w| w.ref_always_on().set_bit());
        rmt.ch2conf1().modify(|_, w| w.ref_always_on().set_bit());
        rmt.ch3conf1().modify(|_, w| w.ref_always_on().set_bit());
        #[cfg(esp32)]
        {
            rmt.ch4conf1().modify(|_, w| w.ref_always_on().set_bit());
            rmt.ch5conf1().modify(|_, w| w.ref_always_on().set_bit());
            rmt.ch6conf1().modify(|_, w| w.ref_always_on().set_bit());
            rmt.ch7conf1().modify(|_, w| w.ref_always_on().set_bit());
        }

        rmt.apb_conf().modify(|_, w| w.apb_fifo_mask().set_bit());

        #[cfg(not(esp32))]
        rmt.apb_conf().modify(|_, w| w.clk_en().set_bit());
    }

    #[allow(unused)]
    #[cfg(esp32)]
    pub fn pending_interrupt_for_channel() -> Option<usize> {
        let rmt = RMT::regs();
        let st = rmt.int_st().read();

        if st.ch0_rx_end().bit() || st.ch0_tx_end().bit() || st.ch0_err().bit() {
            Some(0)
        } else if st.ch1_rx_end().bit() || st.ch1_tx_end().bit() || st.ch1_err().bit() {
            Some(1)
        } else if st.ch2_rx_end().bit() || st.ch2_tx_end().bit() || st.ch2_err().bit() {
            Some(2)
        } else if st.ch3_rx_end().bit() || st.ch3_tx_end().bit() || st.ch3_err().bit() {
            Some(3)
        } else if st.ch4_rx_end().bit() || st.ch4_tx_end().bit() || st.ch4_err().bit() {
            Some(4)
        } else if st.ch5_rx_end().bit() || st.ch5_tx_end().bit() || st.ch5_err().bit() {
            Some(5)
        } else if st.ch6_rx_end().bit() || st.ch6_tx_end().bit() || st.ch6_err().bit() {
            Some(6)
        } else if st.ch7_rx_end().bit() || st.ch7_tx_end().bit() || st.ch7_err().bit() {
            Some(7)
        } else {
            None
        }
    }

    #[allow(unused)]
    #[cfg(esp32s2)]
    pub fn pending_interrupt_for_channel() -> Option<usize> {
        let rmt = RMT::regs();
        let st = rmt.int_st().read();

        if st.ch0_rx_end().bit() || st.ch0_tx_end().bit() || st.ch0_err().bit() {
            Some(0)
        } else if st.ch1_rx_end().bit() || st.ch1_tx_end().bit() || st.ch1_err().bit() {
            Some(1)
        } else if st.ch2_rx_end().bit() || st.ch2_tx_end().bit() || st.ch2_err().bit() {
            Some(2)
        } else if st.ch3_rx_end().bit() || st.ch3_tx_end().bit() || st.ch3_err().bit() {
            Some(3)
        } else {
            None
        }
    }

    macro_rules! impl_tx_channel {
        ($signal:ident, $ch_num:literal) => {
            impl<Dm> super::TxChannelInternal for $crate::rmt::Channel<Dm, $ch_num>
            where
                Dm: $crate::DriverMode,
            {
                const CHANNEL: u8 = $ch_num;

                fn new() -> Self {
                    let guard = GenericPeripheralGuard::new();
                    Self {
                        phantom: core::marker::PhantomData,
                        _guard: guard,
                    }
                }

                fn output_signal() -> crate::gpio::OutputSignal {
                    crate::gpio::OutputSignal::$signal
                }

                fn set_divider(divider: u8) {
                    let rmt = crate::peripherals::RMT::regs();
                    rmt.chconf0($ch_num)
                        .modify(|_, w| unsafe { w.div_cnt().bits(divider) });
                }

                fn update() {
                    // no-op
                }

                #[cfg(not(esp32))]
                fn set_generate_repeat_interrupt(repeats: u16) {
                    let rmt = crate::peripherals::RMT::regs();
                    if repeats > 1 {
                        rmt.ch_tx_lim($ch_num)
                            .modify(|_, w| unsafe { w.tx_loop_num().bits(repeats) });
                    } else {
                        rmt.ch_tx_lim($ch_num)
                            .modify(|_, w| unsafe { w.tx_loop_num().bits(0) });
                    }
                }

                #[cfg(esp32)]
                fn set_generate_repeat_interrupt(_repeats: u16) {
                    // unsupported
                }

                fn clear_interrupts() {
                    let rmt = crate::peripherals::RMT::regs();

                    rmt.int_clr().write(|w| {
                        w.ch_err($ch_num).set_bit();
                        w.ch_tx_end($ch_num).set_bit();
                        w.ch_tx_thr_event($ch_num).set_bit()
                    });
                }

                fn set_continuous(continuous: bool) {
                    let rmt = crate::peripherals::RMT::regs();

                    rmt.chconf1($ch_num)
                        .modify(|_, w| w.tx_conti_mode().bit(continuous));
                }

                fn set_wrap_mode(wrap: bool) {
                    let rmt = crate::peripherals::RMT::regs();
                    // this is "okay", because we use all TX channels always in wrap mode
                    rmt.apb_conf().modify(|_, w| w.mem_tx_wrap_en().bit(wrap));
                }

                fn set_carrier(carrier: bool, high: u16, low: u16, level: $crate::gpio::Level) {
                    let rmt = crate::peripherals::RMT::regs();

                    rmt.chcarrier_duty($ch_num)
                        .write(|w| unsafe { w.carrier_high().bits(high).carrier_low().bits(low) });

                    rmt.chconf0($ch_num).modify(|_, w| {
                        w.carrier_en()
                            .bit(carrier)
                            .carrier_out_lv()
                            .bit(level.into())
                    });
                }

                fn set_idle_output(enable: bool, level: $crate::gpio::Level) {
                    let rmt = crate::peripherals::RMT::regs();
                    rmt.chconf1($ch_num)
                        .modify(|_, w| w.idle_out_en().bit(enable).idle_out_lv().bit(level.into()));
                }

                fn is_memory_blocks_available(memory_blocks_requested: u8) -> bool {
                    let rmt = crate::peripherals::RMT::regs();
                    let mut memory_block_is_available = true;

                    if memory_blocks_requested > 1 {
                        for i in $ch_num..$ch_num + memory_blocks_requested {
                            if rmt.chconf0(i as usize).read().mem_size().bits() != 0 {
                                memory_block_is_available = false;
                                break;
                            }
                        }
                    } else {
                        memory_block_is_available =
                            rmt.chconf0($ch_num).read().mem_size().bits() == 0;
                    }

                    memory_block_is_available
                }

                fn set_memory_blocks_unavailable(num_memory_blocks: u8) {
                    let rmt = crate::peripherals::RMT::regs();
                    if num_memory_blocks > 1 {
                        for i in $ch_num + 1..$ch_num + num_memory_blocks {
                            rmt.chconf0(i as usize)
                                .modify(|_, w| unsafe { w.mem_size().bits(1) });
                        }
                    }
                }

                fn set_memsize(value: u8) {
                    let rmt = crate::peripherals::RMT::regs();

                    rmt.chconf0($ch_num)
                        .modify(|_, w| unsafe { w.mem_size().bits(value) });
                }

                fn memsize() -> u8 {
                    let rmt = crate::peripherals::RMT::regs();
                    rmt.chconf0($ch_num).read().mem_size().bits()
                }

                fn start_tx() {
                    let rmt = crate::peripherals::RMT::regs();

                    rmt.chconf1($ch_num).modify(|_, w| {
                        w.mem_rd_rst().set_bit();
                        w.apb_mem_rst().set_bit();
                        w.tx_start().set_bit()
                    });
                }

                fn is_done() -> bool {
                    let rmt = crate::peripherals::RMT::regs();
                    rmt.int_raw().read().ch_tx_end($ch_num).bit()
                }

                fn is_error() -> bool {
                    let rmt = crate::peripherals::RMT::regs();
                    rmt.int_raw().read().ch_err($ch_num).bit()
                }

                fn is_threshold_set() -> bool {
                    let rmt = crate::peripherals::RMT::regs();
                    rmt.int_raw().read().ch_tx_thr_event($ch_num).bit()
                }

                fn reset_threshold_set() {
                    let rmt = crate::peripherals::RMT::regs();
                    rmt.int_clr()
                        .write(|w| w.ch_tx_thr_event($ch_num).set_bit());
                }

                fn set_threshold(threshold: u8) {
                    let rmt = crate::peripherals::RMT::regs();
                    rmt.ch_tx_lim($ch_num)
                        .modify(|_, w| unsafe { w.tx_lim().bits(threshold as u16) });
                }

                fn is_loopcount_interrupt_set() -> bool {
                    // no-op
                    false
                }

                fn stop() {
                    #[cfg(esp32s2)]
                    {
                        let rmt = crate::peripherals::RMT::regs();
                        rmt.chconf1($ch_num).modify(|_, w| w.tx_stop().set_bit());
                    }
                }

                fn enable_listen_interrupt(
                    events: enumset::EnumSet<$crate::rmt::Event>,
                    enable: bool,
                ) {
                    let rmt = crate::peripherals::RMT::regs();
                    rmt.int_ena().modify(|_, w| {
                        if events.contains($crate::rmt::Event::Error) {
                            w.ch_err($ch_num).bit(enable);
                        }
                        if events.contains($crate::rmt::Event::End) {
                            w.ch_tx_end($ch_num).bit(enable);
                        }
                        if events.contains($crate::rmt::Event::Threshold) {
                            w.ch_tx_thr_event($ch_num).bit(enable);
                        }
                        w
                    });
                }
            }
        };
    }

    macro_rules! impl_rx_channel {
        ($signal:ident, $ch_num:literal) => {
            impl<Dm> super::RxChannelInternal for $crate::rmt::Channel<Dm, $ch_num>
            where
                Dm: $crate::DriverMode,
            {
                const CHANNEL: u8 = $ch_num;

                fn new() -> Self {
                    let guard = GenericPeripheralGuard::new();
                    Self {
                        phantom: core::marker::PhantomData,
                        _guard: guard,
                    }
                }

                fn input_signal() -> crate::gpio::InputSignal {
                    crate::gpio::InputSignal::$signal
                }

                fn set_divider(divider: u8) {
                    let rmt = crate::peripherals::RMT::regs();
                    rmt.chconf0($ch_num)
                        .modify(|_, w| unsafe { w.div_cnt().bits(divider) });
                }

                fn update() {
                    // no-op
                }

                fn clear_interrupts() {
                    let rmt = crate::peripherals::RMT::regs();

                    rmt.chconf1($ch_num).modify(|_, w| {
                        w.mem_wr_rst().set_bit();
                        w.apb_mem_rst().set_bit();
                        w.mem_owner().set_bit();
                        w.rx_en().clear_bit()
                    });
                    Self::update();

                    rmt.int_clr().write(|w| {
                        w.ch_rx_end($ch_num).set_bit();
                        w.ch_err($ch_num).set_bit();
                        w.ch_tx_thr_event($ch_num).set_bit()
                    });
                }

                fn set_wrap_mode(_wrap: bool) {
                    // no-op
                }

                fn set_carrier(carrier: bool, high: u16, low: u16, level: $crate::gpio::Level) {
                    let rmt = crate::peripherals::RMT::regs();

                    rmt.chcarrier_duty($ch_num)
                        .write(|w| unsafe { w.carrier_high().bits(high).carrier_low().bits(low) });

                    rmt.chconf0($ch_num).modify(|_, w| {
                        w.carrier_en()
                            .bit(carrier)
                            .carrier_out_lv()
                            .bit(level.into())
                    });
                }

                fn is_memory_blocks_available(memory_blocks_requested: u8) -> bool {
                    let rmt = crate::peripherals::RMT::regs();
                    let mut memory_block_is_available = true;

                    if memory_blocks_requested > 1 {
                        for i in $ch_num..$ch_num + memory_blocks_requested {
                            if rmt.chconf0(i as usize).read().mem_size().bits() != 0 {
                                memory_block_is_available = false;
                                break;
                            }
                        }
                    } else {
                        memory_block_is_available =
                            rmt.chconf0($ch_num).read().mem_size().bits() == 0;
                    }

                    memory_block_is_available
                }

                fn set_memory_blocks_unavailable(num_memory_blocks: u8) {
                    let rmt = crate::peripherals::RMT::regs();
                    if num_memory_blocks > 1 {
                        for i in $ch_num + 1..$ch_num + num_memory_blocks {
                            rmt.chconf0(i as usize)
                                .modify(|_, w| unsafe { w.mem_size().bits(1) });
                        }
                    }
                }

                fn set_memsize(value: u8) {
                    let rmt = crate::peripherals::RMT::regs();

                    rmt.chconf0($ch_num)
                        .modify(|_, w| unsafe { w.mem_size().bits(value) });
                }

                fn memsize() -> u8 {
                    let rmt = crate::peripherals::RMT::regs();
                    rmt.chconf0($ch_num).read().mem_size().bits()
                }

                fn start_rx() {
                    let rmt = crate::peripherals::RMT::regs();

                    for i in 0..Self::memsize() {
                        rmt.chconf1(($ch_num + i).into())
                            .modify(|_, w| w.mem_owner().set_bit());
                    }

                    rmt.chconf1($ch_num).modify(|_, w| {
                        w.mem_wr_rst().set_bit();
                        w.apb_mem_rst().set_bit();
                        w.rx_en().set_bit()
                    });
                }

                fn is_done() -> bool {
                    let rmt = crate::peripherals::RMT::regs();
                    rmt.int_raw().read().ch_rx_end($ch_num).bit()
                }

                fn is_error() -> bool {
                    let rmt = crate::peripherals::RMT::regs();
                    rmt.int_raw().read().ch_err($ch_num).bit()
                }

                fn stop() {
                    let rmt = crate::peripherals::RMT::regs();
                    rmt.chconf1($ch_num).modify(|_, w| w.rx_en().clear_bit());
                }

                fn set_filter_threshold(value: u8) {
                    let rmt = crate::peripherals::RMT::regs();
                    rmt.chconf1($ch_num).modify(|_, w| unsafe {
                        w.rx_filter_en().bit(value > 0);
                        w.rx_filter_thres().bits(value)
                    });
                }

                fn set_idle_threshold(value: u16) {
                    let rmt = crate::peripherals::RMT::regs();
                    rmt.chconf0($ch_num)
                        .modify(|_, w| unsafe { w.idle_thres().bits(value) });
                }

                fn enable_listen_interrupt(
                    events: enumset::EnumSet<$crate::rmt::Event>,
                    enable: bool,
                ) {
                    let rmt = crate::peripherals::RMT::regs();
                    rmt.int_ena().modify(|_, w| {
                        if events.contains($crate::rmt::Event::Error) {
                            w.ch_err($ch_num).bit(enable);
                        }
                        if events.contains($crate::rmt::Event::End) {
                            w.ch_rx_end($ch_num).bit(enable);
                        }
                        if events.contains($crate::rmt::Event::Threshold) {
                            w.ch_tx_thr_event($ch_num).bit(enable);
                        }
                        w
                    });
                }
            }
        };
    }

    pub(crate) use impl_rx_channel;
    pub(crate) use impl_tx_channel;
}
