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
//! There are
#![cfg_attr(
    esp32,
    doc = "8 channels, each of them can be either receiver or transmitter"
)]
#![cfg_attr(
    esp32s2,
    doc = "4 channels, each of them can be either receiver or transmitter"
)]
#![cfg_attr(
    esp32s3,
    doc = "8 channels, `Channel<0>`-`Channel<3>` hardcoded for transmitting signals and `Channel<4>`-`Channel<7>` hardcoded for receiving signals"
)]
#![cfg_attr(
    any(esp32c3, esp32c6, esp32h2),
    doc = "4 channels, `Channel<0>` and `Channel<1>` hardcoded for transmitting signals and `Channel<2>` and `Channel<3>` hardcoded for receiving signals."
)]
#![doc = "  "]
//! For more information, please refer to the
#![doc = concat!("[ESP-IDF documentation](https://docs.espressif.com/projects/esp-idf/en/latest/", crate::soc::chip!(), "/api-reference/peripherals/rmt.html)")]
//! ## Configuration
//! Each TX/RX channel has the same functionality controlled by a dedicated set
//! of registers and is able to independently transmit or receive data. TX
//! channels are indicated by n which is used as a placeholder for the channel
//! number, and by m for RX channels.
//!
//! ## Examples
//! ### Initialization
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! # use esp_hal::peripherals::Peripherals;
//! # use esp_hal::rmt::TxChannelConfig;
//! # use esp_hal::rmt::Rmt;
//! # use esp_hal::gpio::Io;
//! # use esp_hal::clock::ClockControl;
//! # use crate::esp_hal::rmt::TxChannelCreator;
//! # use crate::esp_hal::prelude::_fugit_RateExtU32;
//! # let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
#![cfg_attr(esp32h2, doc = "let freq = 32.MHz();")]
#![cfg_attr(not(esp32h2), doc = "let freq = 80.MHz();")]
//! let rmt = Rmt::new(peripherals.RMT, freq, &clocks).unwrap();
//! let mut channel = rmt
//!     .channel0
//!     .configure(
//!         io.pins.gpio1,
//!         TxChannelConfig {
//!             clk_divider: 1,
//!             idle_output_level: false,
//!             idle_output: false,
//!             carrier_modulation: false,
//!             carrier_high: 1,
//!             carrier_low: 1,
//!             carrier_level: false,
//!         },
//!     )
//!     .unwrap();
//! # }
//! ```
//! (on ESP32 and ESP32-S2 you cannot specify a base frequency other than 80
//! MHz)

#![warn(missing_docs)]

use core::marker::PhantomData;

use fugit::HertzU32;

use crate::{
    clock::Clocks,
    gpio::{InputPin, OutputPin},
    interrupt::InterruptHandler,
    peripheral::Peripheral,
    rmt::private::CreateInstance,
    soc::constants,
    system::PeripheralClockControl,
    InterruptConfigurable,
};

/// Errors
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// The desired frequency is impossible to reach
    UnreachableTargetFrequency,
    /// The amount of pulses exceeds the size of the FIFO
    Overflow,
    /// An argument is invalid
    InvalidArgument,
    /// An error occurred during transmission
    TransmissionError,
}

/// Convenience representation of a pulse code entry.
///
/// Allows for the assignment of two levels and their lengths
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PulseCode {
    /// Logical output level in the first pulse code interval
    pub level1: bool,
    /// Length of the first pulse code interval (in clock cycles)
    pub length1: u16,
    /// Logical output level in the second pulse code interval
    pub level2: bool,
    /// Length of the second pulse code interval (in clock cycles)
    pub length2: u16,
}

impl From<u32> for PulseCode {
    fn from(value: u32) -> Self {
        Self {
            level1: value & (1 << 15) != 0,
            length1: (value & 0b111_1111_1111_1111) as u16,
            level2: value & (1 << 31) != 0,
            length2: ((value >> 16) & 0b111_1111_1111_1111) as u16,
        }
    }
}

/// Convert a pulse code structure into a u32 value that can be written
/// into the data registers
impl From<PulseCode> for u32 {
    #[inline(always)]
    fn from(p: PulseCode) -> u32 {
        // The length1 value resides in bits [14:0]
        let mut entry: u32 = p.length1 as u32;

        // If level1 is high, set bit 15, otherwise clear it
        if p.level1 {
            entry |= 1 << 15;
        } else {
            entry &= !(1 << 15);
        }

        // If level2 is high, set bit 31, otherwise clear it
        if p.level2 {
            entry |= 1 << 31;
        } else {
            entry &= !(1 << 31);
        }

        // The length2 value resides in bits [30:16]
        entry |= (p.length2 as u32) << 16;

        entry
    }
}

/// Channel configuration for TX channels
#[derive(Debug, Copy, Clone, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TxChannelConfig {
    /// Channel's clock divider
    pub clk_divider: u8,
    /// Set the idle output level to low/high
    pub idle_output_level: bool,
    /// Enable idle output
    pub idle_output: bool,
    /// Enable carrier modulation
    pub carrier_modulation: bool,
    /// Carrier high phase in ticks
    pub carrier_high: u16,
    /// Carrier low phase in ticks
    pub carrier_low: u16,
    /// Level of the carrier
    pub carrier_level: bool,
}

/// Channel configuration for RX channels
#[derive(Debug, Copy, Clone, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RxChannelConfig {
    /// Channel's clock divider
    pub clk_divider: u8,
    /// Enable carrier demodulation
    pub carrier_modulation: bool,
    /// Carrier high phase in ticks
    pub carrier_high: u16,
    /// Carrier low phase in ticks
    pub carrier_low: u16,
    /// Level of the carrier
    pub carrier_level: bool,
    /// Filter threshold in ticks
    pub filter_threshold: u8,
    /// Idle threshold in ticks
    pub idle_threshold: u16,
}

pub use impl_for_chip::{ChannelCreator, Rmt};

#[cfg(feature = "async")]
use self::asynch::{RxChannelAsync, TxChannelAsync};

impl<'d, M> Rmt<'d, M>
where
    M: crate::Mode,
{
    pub(crate) fn new_internal(
        peripheral: impl Peripheral<P = crate::peripherals::RMT> + 'd,
        frequency: HertzU32,
        _clocks: &Clocks<'d>,
    ) -> Result<Self, Error> {
        let me = Rmt::create(peripheral);

        #[cfg(any(esp32, esp32s2))]
        if frequency != HertzU32::MHz(80) {
            return Err(Error::UnreachableTargetFrequency);
        }

        PeripheralClockControl::enable(crate::system::Peripheral::Rmt);

        #[cfg(not(any(esp32, esp32s2)))]
        me.configure_clock(frequency, _clocks)?;

        #[cfg(any(esp32, esp32s2))]
        self::chip_specific::configure_clock();

        Ok(me)
    }

    pub(crate) fn internal_set_interrupt_handler(&mut self, handler: InterruptHandler) {
        unsafe {
            crate::interrupt::bind_interrupt(crate::peripherals::Interrupt::RMT, handler.handler());
            crate::interrupt::enable(crate::peripherals::Interrupt::RMT, handler.priority())
                .unwrap();
        }
    }

    #[cfg(not(any(esp32, esp32s2)))]
    fn configure_clock(&self, frequency: HertzU32, _clocks: &Clocks<'d>) -> Result<(), Error> {
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

impl<'d> Rmt<'d, crate::Blocking> {
    /// Create a new RMT instance
    pub fn new(
        peripheral: impl Peripheral<P = crate::peripherals::RMT> + 'd,
        frequency: HertzU32,
        _clocks: &Clocks<'d>,
    ) -> Result<Self, Error> {
        Self::new_internal(peripheral, frequency, _clocks)
    }
}

impl<'d> crate::private::Sealed for Rmt<'d, crate::Blocking> {}

impl<'d> InterruptConfigurable for Rmt<'d, crate::Blocking> {
    fn set_interrupt_handler(&mut self, handler: crate::interrupt::InterruptHandler) {
        self.internal_set_interrupt_handler(handler);
    }
}

#[cfg(feature = "async")]
impl<'d> Rmt<'d, crate::Async> {
    /// Create a new RMT instance
    pub fn new_async(
        peripheral: impl Peripheral<P = crate::peripherals::RMT> + 'd,
        frequency: HertzU32,
        _clocks: &Clocks<'d>,
    ) -> Result<Self, Error> {
        let mut this = Self::new_internal(peripheral, frequency, _clocks)?;
        this.internal_set_interrupt_handler(asynch::async_interrupt_handler);
        Ok(this)
    }
}

/// Creates a TX channel
pub trait TxChannelCreator<'d, T, P>
where
    P: OutputPin,
    T: TxChannel,
{
    /// Configure the TX channel
    fn configure(
        self,
        pin: impl Peripheral<P = P> + 'd,
        config: TxChannelConfig,
    ) -> Result<T, Error>
    where
        Self: Sized,
    {
        crate::into_ref!(pin);
        pin.set_to_push_pull_output(crate::private::Internal);
        pin.connect_peripheral_to_output(T::output_signal(), crate::private::Internal);
        T::set_divider(config.clk_divider);
        T::set_carrier(
            config.carrier_modulation,
            config.carrier_high,
            config.carrier_low,
            config.carrier_level,
        );
        T::set_idle_output(config.idle_output, config.idle_output_level);

        Ok(T::new())
    }
}

/// Creates a TX channel in async mode
#[cfg(feature = "async")]
pub trait TxChannelCreatorAsync<'d, T, P>
where
    P: OutputPin,
    T: TxChannelAsync,
{
    /// Configure the TX channel
    fn configure(
        self,
        pin: impl Peripheral<P = P> + 'd,
        config: TxChannelConfig,
    ) -> Result<T, Error>
    where
        Self: Sized,
    {
        crate::into_ref!(pin);
        pin.set_to_push_pull_output(crate::private::Internal);
        pin.connect_peripheral_to_output(T::output_signal(), crate::private::Internal);
        T::set_divider(config.clk_divider);
        T::set_carrier(
            config.carrier_modulation,
            config.carrier_high,
            config.carrier_low,
            config.carrier_level,
        );
        T::set_idle_output(config.idle_output, config.idle_output_level);

        Ok(T::new())
    }
}

/// Creates a RX channel
pub trait RxChannelCreator<'d, T, P>
where
    P: InputPin,
    T: RxChannel,
{
    /// Configure the RX channel
    fn configure(
        self,
        pin: impl Peripheral<P = P> + 'd,
        config: RxChannelConfig,
    ) -> Result<T, Error>
    where
        Self: Sized,
    {
        if config.filter_threshold > 0b111_1111 {
            return Err(Error::InvalidArgument);
        }

        #[cfg(any(esp32, esp32s2))]
        if config.idle_threshold > 0b111_1111_1111_1111 {
            return Err(Error::InvalidArgument);
        }

        #[cfg(not(any(esp32, esp32s2)))]
        if config.idle_threshold > 0b11_1111_1111_1111 {
            return Err(Error::InvalidArgument);
        }

        crate::into_ref!(pin);
        pin.set_to_input(crate::private::Internal);
        pin.connect_input_to_peripheral(T::input_signal(), crate::private::Internal);
        T::set_divider(config.clk_divider);
        T::set_carrier(
            config.carrier_modulation,
            config.carrier_high,
            config.carrier_low,
            config.carrier_level,
        );
        T::set_filter_threshold(config.filter_threshold);
        T::set_idle_threshold(config.idle_threshold);

        Ok(T::new())
    }
}

/// Creates a RX channel in async mode
#[cfg(feature = "async")]
pub trait RxChannelCreatorAsync<'d, T, P>
where
    P: InputPin,
    T: RxChannelAsync,
{
    /// Configure the RX channel
    fn configure(
        self,
        pin: impl Peripheral<P = P> + 'd,
        config: RxChannelConfig,
    ) -> Result<T, Error>
    where
        Self: Sized,
    {
        if config.filter_threshold > 0b111_1111 {
            return Err(Error::InvalidArgument);
        }

        #[cfg(any(esp32, esp32s2))]
        if config.idle_threshold > 0b111_1111_1111_1111 {
            return Err(Error::InvalidArgument);
        }

        #[cfg(not(any(esp32, esp32s2)))]
        if config.idle_threshold > 0b11_1111_1111_1111 {
            return Err(Error::InvalidArgument);
        }

        crate::into_ref!(pin);
        pin.set_to_input(crate::private::Internal);
        pin.connect_input_to_peripheral(T::input_signal(), crate::private::Internal);
        T::set_divider(config.clk_divider);
        T::set_carrier(
            config.carrier_modulation,
            config.carrier_high,
            config.carrier_low,
            config.carrier_level,
        );
        T::set_filter_threshold(config.filter_threshold);
        T::set_idle_threshold(config.idle_threshold);

        Ok(T::new())
    }
}

/// An in-progress transaction for a single shot TX transaction.
pub struct SingleShotTxTransaction<'a, C, T: Into<u32> + Copy>
where
    C: TxChannel,
{
    channel: C,
    index: usize,
    data: &'a [T],
}

impl<'a, C, T: Into<u32> + Copy> SingleShotTxTransaction<'a, C, T>
where
    C: TxChannel,
{
    /// Wait for the transaction to complete
    pub fn wait(mut self) -> Result<C, (Error, C)> {
        loop {
            if <C as private::TxChannelInternal<crate::Blocking>>::is_error() {
                return Err((Error::TransmissionError, self.channel));
            }

            if self.index < self.data.len() {
                // wait for TX-THR
                loop {
                    if <C as private::TxChannelInternal<crate::Blocking>>::is_threshold_set() {
                        break;
                    }
                }
                <C as private::TxChannelInternal<crate::Blocking>>::reset_threshold_set();

                // re-fill TX RAM
                let ram_index = (((self.index - constants::RMT_CHANNEL_RAM_SIZE)
                    / (constants::RMT_CHANNEL_RAM_SIZE / 2))
                    % 2)
                    * (constants::RMT_CHANNEL_RAM_SIZE / 2);

                let ptr = (constants::RMT_RAM_START
                    + C::CHANNEL as usize * constants::RMT_CHANNEL_RAM_SIZE * 4
                    + ram_index * 4) as *mut u32;
                for (idx, entry) in self.data[self.index..]
                    .iter()
                    .take(constants::RMT_CHANNEL_RAM_SIZE / 2)
                    .enumerate()
                {
                    unsafe {
                        ptr.add(idx).write_volatile((*entry).into());
                    }
                }

                self.index += constants::RMT_CHANNEL_RAM_SIZE / 2;
            } else {
                break;
            }
        }

        loop {
            if <C as private::TxChannelInternal<crate::Blocking>>::is_error() {
                return Err((Error::TransmissionError, self.channel));
            }

            if <C as private::TxChannelInternal<crate::Blocking>>::is_done() {
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
        <C as private::TxChannelInternal<crate::Blocking>>::set_continuous(false);
        <C as private::TxChannelInternal<crate::Blocking>>::update();

        loop {
            if <C as private::TxChannelInternal<crate::Blocking>>::is_error() {
                return Err((Error::TransmissionError, self.channel));
            }

            if <C as private::TxChannelInternal<crate::Blocking>>::is_done() {
                break;
            }
        }

        Ok(self.channel)
    }

    /// Stop transaction as soon as possible.
    pub fn stop(self) -> Result<C, (Error, C)> {
        <C as private::TxChannelInternal<crate::Blocking>>::set_continuous(false);
        <C as private::TxChannelInternal<crate::Blocking>>::update();

        let ptr = (constants::RMT_RAM_START
            + C::CHANNEL as usize * constants::RMT_CHANNEL_RAM_SIZE * 4)
            as *mut u32;
        for idx in 0..constants::RMT_CHANNEL_RAM_SIZE {
            unsafe {
                ptr.add(idx).write_volatile(0);
            }
        }

        loop {
            if <C as private::TxChannelInternal<crate::Blocking>>::is_error() {
                return Err((Error::TransmissionError, self.channel));
            }

            if <C as private::TxChannelInternal<crate::Blocking>>::is_done() {
                break;
            }
        }

        Ok(self.channel)
    }

    /// Check if the `loopcount` interrupt bit is set
    pub fn is_loopcount_interrupt_set(&self) -> bool {
        <C as private::TxChannelInternal<crate::Blocking>>::is_loopcount_interrupt_set()
    }
}

macro_rules! impl_tx_channel_creator {
    ($channel:literal) => {
        impl<'d, P> $crate::rmt::TxChannelCreator<'d, $crate::rmt::Channel<$crate::Blocking, $channel>, P>
            for ChannelCreator<$crate::Blocking, $channel>
        where
            P: $crate::gpio::OutputPin,
        {
        }

        impl $crate::rmt::TxChannel for $crate::rmt::Channel<$crate::Blocking, $channel> {}

        #[cfg(feature = "async")]
        impl<'d, P> $crate::rmt::TxChannelCreatorAsync<'d, $crate::rmt::Channel<$crate::Async, $channel>, P>
            for ChannelCreator<$crate::Async, $channel>
        where
            P: $crate::gpio::OutputPin,
        {
        }

        #[cfg(feature = "async")]
        impl $crate::rmt::asynch::TxChannelAsync for $crate::rmt::Channel<$crate::Async, $channel> {}
    };
}

macro_rules! impl_rx_channel_creator {
    ($channel:literal) => {
        impl<'d, P> $crate::rmt::RxChannelCreator<'d, $crate::rmt::Channel<$crate::Blocking, $channel>, P>
            for ChannelCreator<$crate::Blocking, $channel>
        where
            P: $crate::gpio::InputPin,
        {
        }

        impl $crate::rmt::RxChannel for $crate::rmt::Channel<$crate::Blocking, $channel> {}

        #[cfg(feature = "async")]
        impl<'d, P> $crate::rmt::RxChannelCreatorAsync<'d, $crate::rmt::Channel<$crate::Async, $channel>, P>
        for ChannelCreator<$crate::Async, $channel>
        where
            P: $crate::gpio::InputPin,
        {
        }

        #[cfg(feature = "async")]
        impl $crate::rmt::asynch::RxChannelAsync for $crate::rmt::Channel<$crate::Async, $channel> {}
    };
}

#[cfg(not(any(esp32, esp32s2, esp32s3)))]
mod impl_for_chip {
    use core::marker::PhantomData;

    use super::private::CreateInstance;
    use crate::peripheral::{Peripheral, PeripheralRef};

    /// RMT Instance
    #[allow(missing_docs)]
    pub struct Rmt<'d, M>
    where
        M: crate::Mode,
    {
        _peripheral: PeripheralRef<'d, crate::peripherals::RMT>,
        pub channel0: ChannelCreator<M, 0>,
        pub channel1: ChannelCreator<M, 1>,
        pub channel2: ChannelCreator<M, 2>,
        pub channel3: ChannelCreator<M, 3>,
        phantom: PhantomData<M>,
    }

    impl<'d, M> CreateInstance<'d> for Rmt<'d, M>
    where
        M: crate::Mode,
    {
        fn create(peripheral: impl Peripheral<P = crate::peripherals::RMT> + 'd) -> Self {
            crate::into_ref!(peripheral);

            Self {
                _peripheral: peripheral,
                channel0: ChannelCreator {
                    phantom: PhantomData,
                },
                channel1: ChannelCreator {
                    phantom: PhantomData,
                },
                channel2: ChannelCreator {
                    phantom: PhantomData,
                },
                channel3: ChannelCreator {
                    phantom: PhantomData,
                },
                phantom: PhantomData,
            }
        }
    }

    /// RMT Channel Creator
    pub struct ChannelCreator<M, const CHANNEL: u8>
    where
        M: crate::Mode,
    {
        phantom: PhantomData<M>,
    }

    impl_tx_channel_creator!(0);
    impl_tx_channel_creator!(1);

    impl_rx_channel_creator!(2);
    impl_rx_channel_creator!(3);

    super::chip_specific::impl_tx_channel!(Channel, RMT_SIG_0, 0);
    super::chip_specific::impl_tx_channel!(Channel, RMT_SIG_1, 1);

    super::chip_specific::impl_rx_channel!(Channel, RMT_SIG_0, 2, 0);
    super::chip_specific::impl_rx_channel!(Channel, RMT_SIG_1, 3, 1);
}

#[cfg(esp32)]
mod impl_for_chip {
    use core::marker::PhantomData;

    use super::private::CreateInstance;
    use crate::peripheral::{Peripheral, PeripheralRef};

    /// RMT Instance
    #[allow(missing_docs)]
    pub struct Rmt<'d, M>
    where
        M: crate::Mode,
    {
        _peripheral: PeripheralRef<'d, crate::peripherals::RMT>,
        pub channel0: ChannelCreator<M, 0>,
        pub channel1: ChannelCreator<M, 1>,
        pub channel2: ChannelCreator<M, 2>,
        pub channel3: ChannelCreator<M, 3>,
        pub channel4: ChannelCreator<M, 4>,
        pub channel5: ChannelCreator<M, 5>,
        pub channel6: ChannelCreator<M, 6>,
        pub channel7: ChannelCreator<M, 7>,
        phantom: PhantomData<M>,
    }

    impl<'d, M> CreateInstance<'d> for Rmt<'d, M>
    where
        M: crate::Mode,
    {
        fn create(peripheral: impl Peripheral<P = crate::peripherals::RMT> + 'd) -> Self {
            crate::into_ref!(peripheral);

            Self {
                _peripheral: peripheral,
                channel0: ChannelCreator {
                    phantom: PhantomData,
                },
                channel1: ChannelCreator {
                    phantom: PhantomData,
                },
                channel2: ChannelCreator {
                    phantom: PhantomData,
                },
                channel3: ChannelCreator {
                    phantom: PhantomData,
                },
                channel4: ChannelCreator {
                    phantom: PhantomData,
                },
                channel5: ChannelCreator {
                    phantom: PhantomData,
                },
                channel6: ChannelCreator {
                    phantom: PhantomData,
                },
                channel7: ChannelCreator {
                    phantom: PhantomData,
                },
                phantom: PhantomData,
            }
        }
    }

    /// RMT Channel Creator
    pub struct ChannelCreator<M, const CHANNEL: u8>
    where
        M: crate::Mode,
    {
        phantom: PhantomData<M>,
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

    super::chip_specific::impl_tx_channel!(Channel, RMT_SIG_0, 0);
    super::chip_specific::impl_tx_channel!(Channel, RMT_SIG_1, 1);
    super::chip_specific::impl_tx_channel!(Channel, RMT_SIG_2, 2);
    super::chip_specific::impl_tx_channel!(Channel, RMT_SIG_3, 3);
    super::chip_specific::impl_tx_channel!(Channel, RMT_SIG_4, 4);
    super::chip_specific::impl_tx_channel!(Channel, RMT_SIG_5, 5);
    super::chip_specific::impl_tx_channel!(Channel, RMT_SIG_6, 6);
    super::chip_specific::impl_tx_channel!(Channel, RMT_SIG_7, 7);

    super::chip_specific::impl_rx_channel!(Channel, RMT_SIG_0, 0);
    super::chip_specific::impl_rx_channel!(Channel, RMT_SIG_1, 1);
    super::chip_specific::impl_rx_channel!(Channel, RMT_SIG_2, 2);
    super::chip_specific::impl_rx_channel!(Channel, RMT_SIG_3, 3);
    super::chip_specific::impl_rx_channel!(Channel, RMT_SIG_4, 4);
    super::chip_specific::impl_rx_channel!(Channel, RMT_SIG_5, 5);
    super::chip_specific::impl_rx_channel!(Channel, RMT_SIG_6, 6);
    super::chip_specific::impl_rx_channel!(Channel, RMT_SIG_7, 7);
}

#[cfg(esp32s2)]
mod impl_for_chip {
    use core::marker::PhantomData;

    use super::private::CreateInstance;
    use crate::peripheral::{Peripheral, PeripheralRef};

    /// RMT Instance
    #[allow(missing_docs)]
    pub struct Rmt<'d, M>
    where
        M: crate::Mode,
    {
        _peripheral: PeripheralRef<'d, crate::peripherals::RMT>,
        pub channel0: ChannelCreator<M, 0>,
        pub channel1: ChannelCreator<M, 1>,
        pub channel2: ChannelCreator<M, 2>,
        pub channel3: ChannelCreator<M, 3>,
        phantom: PhantomData<M>,
    }

    impl<'d, M> CreateInstance<'d> for Rmt<'d, M>
    where
        M: crate::Mode,
    {
        fn create(peripheral: impl Peripheral<P = crate::peripherals::RMT> + 'd) -> Self {
            crate::into_ref!(peripheral);

            Self {
                _peripheral: peripheral,
                channel0: ChannelCreator {
                    phantom: PhantomData,
                },
                channel1: ChannelCreator {
                    phantom: PhantomData,
                },
                channel2: ChannelCreator {
                    phantom: PhantomData,
                },
                channel3: ChannelCreator {
                    phantom: PhantomData,
                },
                phantom: PhantomData,
            }
        }
    }

    /// RMT Channel Creator
    pub struct ChannelCreator<M, const CHANNEL: u8>
    where
        M: crate::Mode,
    {
        phantom: PhantomData<M>,
    }

    impl_tx_channel_creator!(0);
    impl_tx_channel_creator!(1);
    impl_tx_channel_creator!(2);
    impl_tx_channel_creator!(3);

    impl_rx_channel_creator!(0);
    impl_rx_channel_creator!(1);
    impl_rx_channel_creator!(2);
    impl_rx_channel_creator!(3);

    super::chip_specific::impl_tx_channel!(Channel, RMT_SIG_0, 0);
    super::chip_specific::impl_tx_channel!(Channel, RMT_SIG_1, 1);
    super::chip_specific::impl_tx_channel!(Channel, RMT_SIG_2, 2);
    super::chip_specific::impl_tx_channel!(Channel, RMT_SIG_3, 3);

    super::chip_specific::impl_rx_channel!(Channel, RMT_SIG_0, 0);
    super::chip_specific::impl_rx_channel!(Channel, RMT_SIG_1, 1);
    super::chip_specific::impl_rx_channel!(Channel, RMT_SIG_2, 2);
    super::chip_specific::impl_rx_channel!(Channel, RMT_SIG_3, 3);
}

#[cfg(esp32s3)]
mod impl_for_chip {
    use core::marker::PhantomData;

    use super::private::CreateInstance;
    use crate::peripheral::{Peripheral, PeripheralRef};

    /// RMT Instance
    #[allow(missing_docs)]
    pub struct Rmt<'d, M>
    where
        M: crate::Mode,
    {
        _peripheral: PeripheralRef<'d, crate::peripherals::RMT>,
        pub channel0: ChannelCreator<M, 0>,
        pub channel1: ChannelCreator<M, 1>,
        pub channel2: ChannelCreator<M, 2>,
        pub channel3: ChannelCreator<M, 3>,
        pub channel4: ChannelCreator<M, 4>,
        pub channel5: ChannelCreator<M, 5>,
        pub channel6: ChannelCreator<M, 6>,
        pub channel7: ChannelCreator<M, 7>,
        phantom: PhantomData<M>,
    }

    impl<'d, M> CreateInstance<'d> for Rmt<'d, M>
    where
        M: crate::Mode,
    {
        fn create(peripheral: impl Peripheral<P = crate::peripherals::RMT> + 'd) -> Self {
            crate::into_ref!(peripheral);

            Self {
                _peripheral: peripheral,
                channel0: ChannelCreator {
                    phantom: PhantomData,
                },
                channel1: ChannelCreator {
                    phantom: PhantomData,
                },
                channel2: ChannelCreator {
                    phantom: PhantomData,
                },
                channel3: ChannelCreator {
                    phantom: PhantomData,
                },
                channel4: ChannelCreator {
                    phantom: PhantomData,
                },
                channel5: ChannelCreator {
                    phantom: PhantomData,
                },
                channel6: ChannelCreator {
                    phantom: PhantomData,
                },
                channel7: ChannelCreator {
                    phantom: PhantomData,
                },
                phantom: PhantomData,
            }
        }
    }

    /// RMT Channel Creator
    pub struct ChannelCreator<M, const CHANNEL: u8>
    where
        M: crate::Mode,
    {
        phantom: PhantomData<M>,
    }

    impl_tx_channel_creator!(0);
    impl_tx_channel_creator!(1);
    impl_tx_channel_creator!(2);
    impl_tx_channel_creator!(3);

    impl_rx_channel_creator!(4);
    impl_rx_channel_creator!(5);
    impl_rx_channel_creator!(6);
    impl_rx_channel_creator!(7);

    super::chip_specific::impl_tx_channel!(Channel, RMT_SIG_0, 0);
    super::chip_specific::impl_tx_channel!(Channel, RMT_SIG_1, 1);
    super::chip_specific::impl_tx_channel!(Channel, RMT_SIG_2, 2);
    super::chip_specific::impl_tx_channel!(Channel, RMT_SIG_3, 3);

    super::chip_specific::impl_rx_channel!(Channel, RMT_SIG_0, 4, 0);
    super::chip_specific::impl_rx_channel!(Channel, RMT_SIG_1, 5, 1);
    super::chip_specific::impl_rx_channel!(Channel, RMT_SIG_2, 6, 2);
    super::chip_specific::impl_rx_channel!(Channel, RMT_SIG_3, 7, 3);
}

/// RMT Channel
#[non_exhaustive]
#[derive(Debug)]
pub struct Channel<M, const CHANNEL: u8>
where
    M: crate::Mode,
{
    phantom: PhantomData<M>,
}

/// Channel in TX mode
pub trait TxChannel: private::TxChannelInternal<crate::Blocking> {
    /// Start transmitting the given pulse code sequence.
    /// This returns a [`SingleShotTxTransaction`] which can be used to wait for
    /// the transaction to complete and get back the channel for further
    /// use.
    fn transmit<T: Into<u32> + Copy>(self, data: &[T]) -> SingleShotTxTransaction<'_, Self, T>
    where
        Self: Sized,
    {
        let index = Self::send_raw(data, false, 0);
        SingleShotTxTransaction {
            channel: self,
            index,
            data,
        }
    }

    /// Start transmitting the given pulse code continuously.
    /// This returns a [`ContinuousTxTransaction`] which can be used to stop the
    /// ongoing transmission and get back the channel for further use.
    /// The length of sequence cannot exceed the size of the allocated RMT RAM.
    fn transmit_continuously<T: Into<u32> + Copy>(
        self,
        data: &[T],
    ) -> Result<ContinuousTxTransaction<Self>, Error>
    where
        Self: Sized,
    {
        self.transmit_continuously_with_loopcount(0, data)
    }

    /// Like [`Self::transmit_continuously`] but also sets a loop count.
    /// [`ContinuousTxTransaction`] can be used to check if the loop count is
    /// reached.
    fn transmit_continuously_with_loopcount<T: Into<u32> + Copy>(
        self,
        loopcount: u16,
        data: &[T],
    ) -> Result<ContinuousTxTransaction<Self>, Error>
    where
        Self: Sized,
    {
        if data.len() > constants::RMT_CHANNEL_RAM_SIZE {
            return Err(Error::Overflow);
        }

        let _index = Self::send_raw(data, true, loopcount);
        Ok(ContinuousTxTransaction { channel: self })
    }
}

/// RX transaction instance
pub struct RxTransaction<'a, C, T: From<u32> + Copy>
where
    C: RxChannel,
{
    channel: C,
    data: &'a mut [T],
}

impl<'a, C, T: From<u32> + Copy> RxTransaction<'a, C, T>
where
    C: RxChannel,
{
    /// Wait for the transaction to complete
    pub fn wait(self) -> Result<C, (Error, C)> {
        loop {
            if <C as private::RxChannelInternal<crate::Blocking>>::is_error() {
                return Err((Error::TransmissionError, self.channel));
            }

            if <C as private::RxChannelInternal<crate::Blocking>>::is_done() {
                break;
            }
        }

        <C as private::RxChannelInternal<crate::Blocking>>::stop();
        <C as private::RxChannelInternal<crate::Blocking>>::clear_interrupts();
        <C as private::RxChannelInternal<crate::Blocking>>::update();

        let ptr = (constants::RMT_RAM_START
            + C::CHANNEL as usize * constants::RMT_CHANNEL_RAM_SIZE * 4)
            as *mut u32;
        let len = self.data.len();
        for (idx, entry) in self.data.iter_mut().take(len).enumerate() {
            *entry = unsafe { ptr.add(idx).read_volatile().into() };
        }

        Ok(self.channel)
    }
}

/// Channel is RX mode
pub trait RxChannel: private::RxChannelInternal<crate::Blocking> {
    /// Start receiving pulse codes into the given buffer.
    /// This returns a [RxTransaction] which can be used to wait for receive to
    /// complete and get back the channel for further use.
    /// The length of the received data cannot exceed the allocated RMT RAM.
    fn receive<T: From<u32> + Copy>(
        self,
        data: &mut [T],
    ) -> Result<RxTransaction<'_, Self, T>, Error>
    where
        Self: Sized,
    {
        if data.len() > constants::RMT_CHANNEL_RAM_SIZE {
            return Err(Error::InvalidArgument);
        }

        Self::start_receive_raw();

        Ok(RxTransaction {
            channel: self,
            data,
        })
    }
}

/// Async functionality
#[cfg(feature = "async")]
pub mod asynch {
    use core::{
        pin::Pin,
        task::{Context, Poll},
    };

    use embassy_sync::waitqueue::AtomicWaker;
    use procmacros::handler;

    use super::{private::Event, *};

    #[cfg(any(esp32, esp32s3))]
    const NUM_CHANNELS: usize = 8;
    #[cfg(not(any(esp32, esp32s3)))]
    const NUM_CHANNELS: usize = 4;

    #[allow(clippy::declare_interior_mutable_const)]
    const INIT: AtomicWaker = AtomicWaker::new();
    static WAKER: [AtomicWaker; NUM_CHANNELS] = [INIT; NUM_CHANNELS];

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
    pub trait TxChannelAsync: private::TxChannelInternal<crate::Async> {
        /// Start transmitting the given pulse code sequence.
        /// The length of sequence cannot exceed the size of the allocated RMT
        /// RAM.
        async fn transmit<'a, T: Into<u32> + Copy>(&mut self, data: &'a [T]) -> Result<(), Error>
        where
            Self: Sized,
        {
            if data.len() > constants::RMT_CHANNEL_RAM_SIZE {
                return Err(Error::InvalidArgument);
            }

            Self::clear_interrupts();
            Self::listen_interrupt(super::private::Event::End);
            Self::listen_interrupt(super::private::Event::Error);
            Self::send_raw(data, false, 0);

            RmtTxFuture::new(self).await;

            if Self::is_error() {
                Err(Error::TransmissionError)
            } else {
                Ok(())
            }
        }
    }

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
    pub trait RxChannelAsync: private::RxChannelInternal<crate::Async> {
        /// Start receiving a pulse code sequence.
        /// The length of sequence cannot exceed the size of the allocated RMT
        /// RAM.
        async fn receive<'a, T: From<u32> + Copy>(&mut self, data: &'a mut [T]) -> Result<(), Error>
        where
            Self: Sized,
        {
            if data.len() > constants::RMT_CHANNEL_RAM_SIZE {
                return Err(Error::InvalidArgument);
            }

            Self::clear_interrupts();
            Self::listen_interrupt(super::private::Event::End);
            Self::listen_interrupt(super::private::Event::Error);
            Self::start_receive_raw();

            RmtRxFuture::new(self).await;

            if Self::is_error() {
                Err(Error::TransmissionError)
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
    pub(super) fn async_interrupt_handler() {
        if let Some(channel) = super::chip_specific::pending_interrupt_for_channel() {
            use crate::rmt::private::{RxChannelInternal, TxChannelInternal};

            match channel {
                0 => {
                    super::Channel::<crate::Async, 0>::unlisten_interrupt(Event::End);
                    super::Channel::<crate::Async, 0>::unlisten_interrupt(Event::Error);
                }
                1 => {
                    super::Channel::<crate::Async, 1>::unlisten_interrupt(Event::End);
                    super::Channel::<crate::Async, 1>::unlisten_interrupt(Event::Error);
                }
                2 => {
                    super::Channel::<crate::Async, 2>::unlisten_interrupt(Event::End);
                    super::Channel::<crate::Async, 2>::unlisten_interrupt(Event::Error);
                }
                3 => {
                    super::Channel::<crate::Async, 3>::unlisten_interrupt(Event::End);
                    super::Channel::<crate::Async, 3>::unlisten_interrupt(Event::Error);
                }

                #[cfg(any(esp32, esp32s3))]
                4 => {
                    super::Channel::<crate::Async, 4>::unlisten_interrupt(Event::End);
                    super::Channel::<crate::Async, 4>::unlisten_interrupt(Event::Error);
                }
                #[cfg(any(esp32, esp32s3))]
                5 => {
                    super::Channel::<crate::Async, 5>::unlisten_interrupt(Event::End);
                    super::Channel::<crate::Async, 5>::unlisten_interrupt(Event::Error);
                }
                #[cfg(any(esp32, esp32s3))]
                6 => {
                    super::Channel::<crate::Async, 6>::unlisten_interrupt(Event::End);
                    super::Channel::<crate::Async, 6>::unlisten_interrupt(Event::Error);
                }
                #[cfg(any(esp32, esp32s3))]
                7 => {
                    super::Channel::<crate::Async, 7>::unlisten_interrupt(Event::End);
                    super::Channel::<crate::Async, 7>::unlisten_interrupt(Event::Error);
                }

                _ => unreachable!(),
            }

            WAKER[channel].wake();
        }
    }

    #[cfg(any(esp32, esp32s2))]
    #[handler]
    pub(super) fn async_interrupt_handler() {
        if let Some(channel) = super::chip_specific::pending_interrupt_for_channel() {
            match channel {
                0 => {
                    <Channel<crate::Async,0> as super::private::TxChannelInternal<crate::Async>>::unlisten_interrupt(
                        Event::End,
                    );
                    <Channel<crate::Async,0> as super::private::TxChannelInternal<crate::Async>>::unlisten_interrupt(
                        Event::Error,
                    );
                    <Channel<crate::Async,0> as super::private::RxChannelInternal<crate::Async>>::unlisten_interrupt(
                        Event::End,
                    );
                    <Channel<crate::Async,0> as super::private::RxChannelInternal<crate::Async>>::unlisten_interrupt(
                        Event::Error,
                    );
                }
                1 => {
                    <Channel<crate::Async,1> as super::private::TxChannelInternal<crate::Async>>::unlisten_interrupt(
                        Event::End,
                    );
                    <Channel<crate::Async,1> as super::private::TxChannelInternal<crate::Async>>::unlisten_interrupt(
                        Event::Error,
                    );
                    <Channel<crate::Async,1> as super::private::RxChannelInternal<crate::Async>>::unlisten_interrupt(
                        Event::End,
                    );
                    <Channel<crate::Async,1> as super::private::RxChannelInternal<crate::Async>>::unlisten_interrupt(
                        Event::Error,
                    );
                }
                2 => {
                    <Channel<crate::Async,2> as super::private::TxChannelInternal<crate::Async>>::unlisten_interrupt(
                        Event::End,
                    );
                    <Channel<crate::Async,2> as super::private::TxChannelInternal<crate::Async>>::unlisten_interrupt(
                        Event::Error,
                    );
                    <Channel<crate::Async,2> as super::private::RxChannelInternal<crate::Async>>::unlisten_interrupt(
                        Event::End,
                    );
                    <Channel<crate::Async,2> as super::private::RxChannelInternal<crate::Async>>::unlisten_interrupt(
                        Event::Error,
                    );
                }
                3 => {
                    <Channel<crate::Async,3> as super::private::TxChannelInternal<crate::Async>>::unlisten_interrupt(
                        Event::End,
                    );
                    <Channel<crate::Async,3> as super::private::TxChannelInternal<crate::Async>>::unlisten_interrupt(
                        Event::Error,
                    );
                    <Channel<crate::Async,3> as super::private::RxChannelInternal<crate::Async>>::unlisten_interrupt(
                        Event::End,
                    );
                    <Channel<crate::Async,3> as super::private::RxChannelInternal<crate::Async>>::unlisten_interrupt(
                        Event::Error,
                    );
                }
                #[cfg(esp32)]
                4 => {
                    <Channel<crate::Async,4> as super::private::TxChannelInternal<crate::Async>>::unlisten_interrupt(
                        Event::End,
                    );
                    <Channel<crate::Async,4> as super::private::TxChannelInternal<crate::Async>>::unlisten_interrupt(
                        Event::Error,
                    );
                    <Channel<crate::Async,4> as super::private::RxChannelInternal<crate::Async>>::unlisten_interrupt(
                        Event::End,
                    );
                    <Channel<crate::Async,4> as super::private::RxChannelInternal<crate::Async>>::unlisten_interrupt(
                        Event::Error,
                    );
                }
                #[cfg(any(esp32, esp32s3))]
                5 => {
                    <Channel<crate::Async,5> as super::private::TxChannelInternal<crate::Async>>::unlisten_interrupt(
                        Event::End,
                    );
                    <Channel<crate::Async,5> as super::private::TxChannelInternal<crate::Async>>::unlisten_interrupt(
                        Event::Error,
                    );
                    <Channel<crate::Async,5> as super::private::RxChannelInternal<crate::Async>>::unlisten_interrupt(
                        Event::End,
                    );
                    <Channel<crate::Async,5> as super::private::RxChannelInternal<crate::Async>>::unlisten_interrupt(
                        Event::Error,
                    );
                }
                #[cfg(any(esp32, esp32s3))]
                6 => {
                    <Channel<crate::Async,6> as super::private::TxChannelInternal<crate::Async>>::unlisten_interrupt(
                        Event::End,
                    );
                    <Channel<crate::Async,6> as super::private::TxChannelInternal<crate::Async>>::unlisten_interrupt(
                        Event::Error,
                    );
                    <Channel<crate::Async,6> as super::private::RxChannelInternal<crate::Async>>::unlisten_interrupt(
                        Event::End,
                    );
                    <Channel<crate::Async,6> as super::private::RxChannelInternal<crate::Async>>::unlisten_interrupt(
                        Event::Error,
                    );
                }
                #[cfg(any(esp32, esp32s3))]
                7 => {
                    <Channel<crate::Async,7> as super::private::TxChannelInternal<crate::Async>>::unlisten_interrupt(
                        Event::End,
                    );
                    <Channel<crate::Async,7> as super::private::TxChannelInternal<crate::Async>>::unlisten_interrupt(
                        Event::Error,
                    );
                    <Channel<crate::Async,7> as super::private::RxChannelInternal<crate::Async>>::unlisten_interrupt(
                        Event::End,
                    );
                    <Channel<crate::Async,7> as super::private::RxChannelInternal<crate::Async>>::unlisten_interrupt(
                        Event::Error,
                    );
                }

                _ => unreachable!(),
            }

            WAKER[channel].wake();
        }
    }
}

mod private {
    use crate::{peripheral::Peripheral, soc::constants};

    pub enum Event {
        Error,
        Threshold,
        End,
    }

    pub trait CreateInstance<'d> {
        fn create(peripheral: impl Peripheral<P = crate::peripherals::RMT> + 'd) -> Self;
    }

    pub trait TxChannelInternal<M>
    where
        M: crate::Mode,
    {
        const CHANNEL: u8;

        fn new() -> Self;

        fn output_signal() -> crate::gpio::OutputSignal;

        fn set_divider(divider: u8);

        fn update();

        fn set_generate_repeat_interrupt(repeats: u16);

        fn clear_interrupts();

        fn set_continuous(continuous: bool);

        fn set_wrap_mode(wrap: bool);

        fn set_carrier(carrier: bool, high: u16, low: u16, level: bool);

        fn set_idle_output(enable: bool, level: bool);

        fn set_memsize(memsize: u8);

        fn start_tx();

        fn is_done() -> bool;

        fn is_error() -> bool;

        fn is_threshold_set() -> bool;

        fn reset_threshold_set();

        fn set_threshold(threshold: u8);

        fn is_loopcount_interrupt_set() -> bool;

        fn send_raw<T: Into<u32> + Copy>(data: &[T], continuous: bool, repeat: u16) -> usize {
            Self::clear_interrupts();

            let ptr = (constants::RMT_RAM_START
                + Self::CHANNEL as usize * constants::RMT_CHANNEL_RAM_SIZE * 4)
                as *mut u32;
            for (idx, entry) in data
                .iter()
                .take(constants::RMT_CHANNEL_RAM_SIZE)
                .enumerate()
            {
                unsafe {
                    ptr.add(idx).write_volatile((*entry).into());
                }
            }

            Self::set_threshold((constants::RMT_CHANNEL_RAM_SIZE / 2) as u8);
            Self::set_continuous(continuous);
            Self::set_generate_repeat_interrupt(repeat);
            Self::set_wrap_mode(true);
            Self::set_memsize(1);
            Self::update();
            Self::start_tx();
            Self::update();

            if data.len() >= constants::RMT_CHANNEL_RAM_SIZE {
                constants::RMT_CHANNEL_RAM_SIZE
            } else {
                data.len()
            }
        }

        fn stop();

        fn listen_interrupt(event: Event);

        fn unlisten_interrupt(event: Event);
    }

    pub trait RxChannelInternal<M>
    where
        M: crate::Mode,
    {
        const CHANNEL: u8;

        fn new() -> Self;

        fn input_signal() -> crate::gpio::InputSignal;

        fn set_divider(divider: u8);

        fn update();

        fn clear_interrupts();

        fn set_wrap_mode(wrap: bool);

        fn set_carrier(carrier: bool, high: u16, low: u16, level: bool);

        fn set_memsize(memsize: u8);

        fn start_rx();

        fn is_done() -> bool;

        fn is_error() -> bool;

        fn start_receive_raw() {
            Self::clear_interrupts();
            Self::set_wrap_mode(false);
            Self::set_memsize(1);
            Self::start_rx();
            Self::update();
        }

        fn stop();

        fn set_filter_threshold(value: u8);

        fn set_idle_threshold(value: u16);

        fn listen_interrupt(event: Event);

        fn unlisten_interrupt(event: Event);
    }
}

#[cfg(not(any(esp32, esp32s2)))]
mod chip_specific {
    pub fn configure_clock(div: u32) {
        #[cfg(not(pcr))]
        {
            let rmt = unsafe { &*crate::peripherals::RMT::PTR };
            rmt.sys_conf().modify(|_, w| unsafe {
                w.clk_en()
                    .clear_bit()
                    .sclk_sel()
                    .bits(crate::soc::constants::RMT_CLOCK_SRC)
                    .sclk_div_num()
                    .bits(div as u8)
                    .sclk_div_a()
                    .bits(0)
                    .sclk_div_b()
                    .bits(0)
                    .apb_fifo_mask()
                    .set_bit()
            });
        }

        #[cfg(pcr)]
        {
            let pcr = unsafe { &*crate::peripherals::PCR::PTR };
            pcr.rmt_sclk_conf().modify(|_, w| unsafe {
                w.sclk_div_num()
                    .bits(div as u8)
                    .sclk_div_a()
                    .bits(0)
                    .sclk_div_b()
                    .bits(0)
            });

            #[cfg(esp32c6)]
            pcr.rmt_sclk_conf()
                .modify(|_, w| unsafe { w.sclk_sel().bits(crate::soc::constants::RMT_CLOCK_SRC) });
            #[cfg(not(esp32c6))]
            pcr.rmt_sclk_conf()
                .modify(|_, w| w.sclk_sel().bit(crate::soc::constants::RMT_CLOCK_SRC));

            let rmt = unsafe { &*crate::peripherals::RMT::PTR };
            rmt.sys_conf().modify(|_, w| w.apb_fifo_mask().set_bit());
        }
    }

    #[allow(unused)]
    #[cfg(not(esp32s3))]
    pub fn pending_interrupt_for_channel() -> Option<usize> {
        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
        let st = rmt.int_st().read();

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
        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
        let st = rmt.int_st().read();

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
        ($channel:ident, $signal:ident, $ch_num:literal) => {
            paste::paste! {
                impl<M> $crate::rmt::private::TxChannelInternal<M> for $crate::rmt::$channel<M, $ch_num> where M: $crate::Mode {
                    const CHANNEL: u8 = $ch_num;

                    fn new() -> Self {
                        Self {
                            phantom: core::marker::PhantomData,
                        }
                    }

                    fn output_signal() -> crate::gpio::OutputSignal {
                        crate::gpio::OutputSignal::$signal
                    }

                    fn set_divider(divider: u8) {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        rmt.ch_tx_conf0($ch_num).modify(|_, w| unsafe { w.div_cnt().bits(divider) });
                    }

                    fn update() {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        rmt.ch_tx_conf0($ch_num).modify(|_, w| w.conf_update().set_bit());
                    }

                    fn set_generate_repeat_interrupt(repeats: u16) {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        if repeats > 1 {
                            rmt.ch_tx_lim($ch_num).modify(|_, w| unsafe {
                                w.loop_count_reset()
                                    .set_bit()
                                    .tx_loop_cnt_en()
                                    .set_bit()
                                    .tx_loop_num()
                                    .bits(repeats)
                            });
                        } else {
                            rmt.ch_tx_lim($ch_num).modify(|_, w| unsafe {
                                w.loop_count_reset()
                                    .set_bit()
                                    .tx_loop_cnt_en()
                                    .clear_bit()
                                    .tx_loop_num()
                                    .bits(0)
                            });
                        }

                        rmt.ch_tx_lim($ch_num).modify(|_, w| w.loop_count_reset().clear_bit());
                    }

                    fn clear_interrupts() {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };

                        rmt.int_clr().write(|w| {
                            w.[< ch $ch_num _tx_end >]()
                                .set_bit()
                                .[< ch $ch_num _tx_err >]()
                                .set_bit()
                                .[< ch $ch_num _tx_loop >]()
                                .set_bit()
                                .[< ch $ch_num _tx_thr_event >]()
                                .set_bit()
                        });
                    }

                    fn set_continuous(continuous: bool) {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };

                        rmt.ch_tx_conf0($ch_num).modify(|_, w| w.tx_conti_mode().bit(continuous));
                    }

                    fn set_wrap_mode(wrap: bool) {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };

                        rmt.ch_tx_conf0($ch_num).modify(|_, w| w.mem_tx_wrap_en().bit(wrap));
                    }

                    fn set_carrier(carrier: bool, high: u16, low: u16, level: bool) {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };

                        rmt.chcarrier_duty($ch_num)
                            .write(|w| unsafe { w.carrier_high().bits(high).carrier_low().bits(low) });

                        rmt.ch_tx_conf0($ch_num).modify(|_, w| {
                            w.carrier_en()
                                .bit(carrier)
                                .carrier_eff_en()
                                .set_bit()
                                .carrier_out_lv()
                                .bit(level)
                        });
                    }

                    fn set_idle_output(enable: bool, level: bool) {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        rmt.ch_tx_conf0($ch_num).modify(|_, w| w.idle_out_en().bit(enable).idle_out_lv().bit(level));
                    }

                    fn set_memsize(memsize: u8) {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };

                        rmt.ch_tx_conf0($ch_num).modify(|_, w| unsafe { w.mem_size().bits(memsize) });
                    }

                    fn start_tx() {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };

                        rmt.ref_cnt_rst().write(|w| unsafe { w.bits(1 << $ch_num) });
                        Self::update();

                        rmt.ch_tx_conf0($ch_num).modify(|_, w| {
                            w.mem_rd_rst()
                                .set_bit()
                                .apb_mem_rst()
                                .set_bit()
                                .tx_start()
                                .set_bit()
                        });
                        Self::update();
                    }

                    fn is_done() -> bool {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        rmt.int_raw().read().[< ch $ch_num _tx_end >]().bit()
                    }

                    fn is_error() -> bool {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        rmt.int_raw().read().[< ch $ch_num _tx_err >]().bit()
                    }

                    fn is_threshold_set() -> bool {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        rmt.int_raw().read().[< ch $ch_num _tx_thr_event >]().bit()
                    }

                    fn reset_threshold_set() {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        rmt.int_clr()
                            .write(|w| w.[< ch $ch_num _tx_thr_event >]().set_bit());
                    }

                    fn set_threshold(threshold: u8) {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        rmt.ch_tx_lim($ch_num).modify(|_, w| unsafe { w.tx_lim().bits(threshold as u16) });
                    }

                    fn is_loopcount_interrupt_set() -> bool {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        rmt.int_raw().read().[< ch $ch_num _tx_loop >]().bit()
                    }

                    fn stop() {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        rmt.ch_tx_conf0($ch_num).modify(|_, w| w.tx_stop().set_bit());
                        Self::update();
                    }

                    fn listen_interrupt(event: $crate::rmt::private::Event) {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        match event {
                            $crate::rmt::private::Event::Error => {
                                rmt.int_ena().modify(|_,w| w.[< ch $ch_num _tx_err >]().set_bit());
                            }
                            $crate::rmt::private::Event::End => {
                                rmt.int_ena().modify(|_,w| w.[< ch $ch_num _tx_end >]().set_bit());
                            }
                            $crate::rmt::private::Event::Threshold => {
                                rmt.int_ena().modify(|_,w| w.[< ch $ch_num _tx_thr_event >]().set_bit());
                            }
                        }
                    }

                    fn unlisten_interrupt(event: $crate::rmt::private::Event) {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        match event {
                            $crate::rmt::private::Event::Error => {
                                rmt.int_ena().modify(|_,w| w.[< ch $ch_num _tx_err >]().clear_bit());
                            }
                            $crate::rmt::private::Event::End => {
                                rmt.int_ena().modify(|_,w| w.[< ch $ch_num _tx_end >]().clear_bit());
                            }
                            $crate::rmt::private::Event::Threshold => {
                                rmt.int_ena().modify(|_,w| w.[< ch $ch_num _tx_thr_event >]().clear_bit());
                            }
                        }
                    }
                }
            }
        }
    }

    macro_rules! impl_rx_channel {
        ($channel:ident, $signal:ident, $ch_num:literal, $ch_index:literal) => {
            paste::paste! {
                impl<M> $crate::rmt::private::RxChannelInternal<M> for $crate::rmt::$channel<M, $ch_num> where M: $crate::Mode {
                    const CHANNEL: u8 = $ch_num;

                    fn new() -> Self {
                        Self {
                            phantom: core::marker::PhantomData,
                        }
                    }

                    fn input_signal() -> crate::gpio::InputSignal {
                        crate::gpio::InputSignal::$signal
                    }

                    fn set_divider(divider: u8) {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        rmt.[< ch $ch_num _rx_conf0 >]().modify(|_, w| unsafe { w.div_cnt().bits(divider) });
                    }

                    fn update() {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        rmt.[< ch $ch_num _rx_conf1 >]().modify(|_, w| w.conf_update().set_bit());
                    }

                    fn clear_interrupts() {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };

                        rmt.int_clr().write(|w| {
                            w.[< ch $ch_num _rx_end >]()
                                .set_bit()
                                .[< ch $ch_num _rx_err >]()
                                .set_bit()
                                .[< ch $ch_num _rx_thr_event >]()
                                .set_bit()
                        });
                    }

                    fn set_wrap_mode(wrap: bool) {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        rmt.[< ch $ch_num _rx_conf1 >]().modify(|_, w| w.mem_rx_wrap_en().bit(wrap));
                    }

                    fn set_carrier(carrier: bool, high: u16, low: u16, level: bool) {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };

                        rmt.ch_rx_carrier_rm($ch_index).write(|w| unsafe {
                            w.carrier_high_thres()
                                .bits(high)
                                .carrier_low_thres()
                                .bits(low)
                        });

                        rmt.[< ch $ch_num _rx_conf0 >]()
                            .modify(|_, w| w.carrier_en().bit(carrier).carrier_out_lv().bit(level));
                    }

                    fn set_memsize(memsize: u8) {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        rmt.[< ch $ch_num _rx_conf0 >]().modify(|_, w| unsafe { w.mem_size().bits(memsize) });
                    }

                    fn start_rx() {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        rmt.[< ch $ch_num _rx_conf1 >]().modify(|_, w| {
                            w.mem_wr_rst()
                                .set_bit()
                                .apb_mem_rst()
                                .set_bit()
                                .mem_owner()
                                .set_bit()
                                .rx_en()
                                .set_bit()
                        });
                    }

                    fn is_done() -> bool {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        rmt.int_raw().read().[< ch $ch_num _rx_end >]().bit()
                    }

                    fn is_error() -> bool {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        rmt.int_raw().read().[< ch $ch_num _rx_err >]().bit()
                    }

                    fn stop() {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        rmt.[< ch $ch_num _rx_conf1 >]().modify(|_, w| w.rx_en().clear_bit());
                    }

                    fn set_filter_threshold(value: u8) {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };

                        rmt.[< ch $ch_num _rx_conf1 >]().modify(|_, w| unsafe {
                            w.rx_filter_en()
                                .bit(value > 0)
                                .rx_filter_thres()
                                .bits(value)
                        });
                    }

                    fn set_idle_threshold(value: u16) {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };

                        rmt.[< ch $ch_num _rx_conf0 >]().modify(|_, w| unsafe { w.idle_thres().bits(value) });
                    }

                    fn listen_interrupt(event: $crate::rmt::private::Event) {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        match event {
                            $crate::rmt::private::Event::Error => {
                                rmt.int_ena().modify(|_,w| w.[< ch $ch_num _rx_err >]().set_bit());
                            }
                            $crate::rmt::private::Event::End => {
                                rmt.int_ena().modify(|_,w| w.[< ch $ch_num _rx_end >]().set_bit());
                            }
                            $crate::rmt::private::Event::Threshold => {
                                rmt.int_ena().modify(|_,w| w.[< ch $ch_num _rx_thr_event >]().set_bit());
                            }
                        }
                    }

                    fn unlisten_interrupt(event: $crate::rmt::private::Event) {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        match event {
                            $crate::rmt::private::Event::Error => {
                                rmt.int_ena().modify(|_,w| w.[< ch $ch_num _rx_err >]().clear_bit());
                            }
                            $crate::rmt::private::Event::End => {
                                rmt.int_ena().modify(|_,w| w.[< ch $ch_num _rx_end >]().clear_bit());
                            }
                            $crate::rmt::private::Event::Threshold => {
                                rmt.int_ena().modify(|_,w| w.[< ch $ch_num _rx_thr_event >]().clear_bit());
                            }
                        }
                    }
                }
            }
        }
    }

    pub(crate) use impl_rx_channel;
    pub(crate) use impl_tx_channel;
}

#[cfg(any(esp32, esp32s2))]
mod chip_specific {
    pub fn configure_clock() {
        let rmt = unsafe { &*crate::peripherals::RMT::PTR };

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
        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
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
        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
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
        ($channel:ident, $signal:ident, $ch_num:literal) => {
            paste::paste! {
                impl<M> super::private::TxChannelInternal<M> for super::$channel<M, $ch_num> where M: $crate::Mode {
                    const CHANNEL: u8 = $ch_num;

                    fn new() -> Self {
                        Self {
                            phantom: core::marker::PhantomData,
                        }
                    }

                    fn output_signal() -> crate::gpio::OutputSignal {
                        crate::gpio::OutputSignal::$signal
                    }

                    fn set_divider(divider: u8) {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        rmt.[< ch $ch_num conf0 >]().modify(|_, w| unsafe { w.div_cnt().bits(divider) });
                    }

                    fn update() {
                        // no-op
                    }

                    #[cfg(not(esp32))]
                    fn set_generate_repeat_interrupt(repeats: u16) {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        if repeats > 1 {
                            rmt.ch_tx_lim($ch_num).modify(|_, w| unsafe { w.tx_loop_num().bits(repeats) });
                        } else {
                            rmt.ch_tx_lim($ch_num).modify(|_, w| unsafe { w.tx_loop_num().bits(0) });
                        }
                    }

                    #[cfg(esp32)]
                    fn set_generate_repeat_interrupt(_repeats: u16) {
                        // unsupported
                    }

                    fn clear_interrupts() {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };

                        rmt.int_clr().write(|w| {
                            w.[< ch $ch_num _tx_end >]()
                                .set_bit()
                                .[< ch $ch_num _err >]()
                                .set_bit()
                                .[< ch $ch_num _tx_thr_event >]()
                                .set_bit()
                        });
                    }

                    fn set_continuous(continuous: bool) {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };

                        rmt.[< ch $ch_num conf1 >]().modify(|_, w| w.tx_conti_mode().bit(continuous));
                    }

                    fn set_wrap_mode(wrap: bool) {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        // this is "okay", because we use all TX channels always in wrap mode
                        rmt.apb_conf().modify(|_, w| w.mem_tx_wrap_en().bit(wrap));
                    }

                    fn set_carrier(carrier: bool, high: u16, low: u16, level: bool) {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };

                        rmt.chcarrier_duty($ch_num)
                            .write(|w| unsafe { w.carrier_high().bits(high).carrier_low().bits(low) });

                        rmt.[< ch $ch_num conf0 >]()
                            .modify(|_, w| w.carrier_en().bit(carrier).carrier_out_lv().bit(level));
                    }

                    fn set_idle_output(enable: bool, level: bool) {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        rmt.[< ch $ch_num conf1 >]()
                            .modify(|_, w| w.idle_out_en().bit(enable).idle_out_lv().bit(level));
                    }

                    fn set_memsize(memsize: u8) {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };

                        rmt.[< ch $ch_num conf0 >]().modify(|_, w| unsafe { w.mem_size().bits(memsize) });
                    }

                    fn start_tx() {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };

                        rmt.[< ch $ch_num conf1 >]().modify(|_, w| {
                            w.mem_rd_rst()
                                .set_bit()
                                .apb_mem_rst()
                                .set_bit()
                                .tx_start()
                                .set_bit()
                        });
                    }

                    fn is_done() -> bool {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        rmt.int_raw().read().[< ch $ch_num _tx_end >]().bit()
                    }

                    fn is_error() -> bool {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        rmt.int_raw().read().[< ch $ch_num _err >]().bit()
                    }

                    fn is_threshold_set() -> bool {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        rmt.int_raw().read().[< ch $ch_num _tx_thr_event >]().bit()
                    }

                    fn reset_threshold_set() {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        rmt.int_clr()
                            .write(|w| w.[< ch $ch_num _tx_thr_event >]().set_bit());
                    }

                    fn set_threshold(threshold: u8) {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        rmt.ch_tx_lim($ch_num).modify(|_, w| unsafe { w.tx_lim().bits(threshold as u16) });
                    }

                    fn is_loopcount_interrupt_set() -> bool {
                        // no-op
                        false
                    }

                    fn stop() {
                        #[cfg(esp32s2)]
                        {
                            let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                            rmt.[< ch $ch_num conf1 >]().modify(|_, w| w.tx_stop().set_bit());
                        }
                    }

                    fn listen_interrupt(event: $crate::rmt::private::Event) {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        match event {
                            $crate::rmt::private::Event::Error => {
                                rmt.int_ena().modify(|_,w| w.[< ch $ch_num _err >]().set_bit());
                            }
                            $crate::rmt::private::Event::End => {
                                rmt.int_ena().modify(|_,w| w.[< ch $ch_num _tx_end >]().set_bit());
                            }
                            $crate::rmt::private::Event::Threshold => {
                                rmt.int_ena().modify(|_,w| w.[< ch $ch_num _tx_thr_event >]().set_bit());
                            }
                        }
                    }

                    fn unlisten_interrupt(event: $crate::rmt::private::Event) {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        match event {
                            $crate::rmt::private::Event::Error => {
                                rmt.int_ena().modify(|_,w| w.[< ch $ch_num _err >]().clear_bit());
                            }
                            $crate::rmt::private::Event::End => {
                                rmt.int_ena().modify(|_,w| w.[< ch $ch_num _tx_end >]().clear_bit());
                            }
                            $crate::rmt::private::Event::Threshold => {
                                rmt.int_ena().modify(|_,w| w.[< ch $ch_num _tx_thr_event >]().clear_bit());
                            }
                        }
                    }
                }
            }
        }
    }

    macro_rules! impl_rx_channel {
        ($channel:ident, $signal:ident, $ch_num:literal) => {
            paste::paste! {
                impl<M> super::private::RxChannelInternal<M> for super::$channel<M, $ch_num> where M: $crate::Mode {
                    const CHANNEL: u8 = $ch_num;

                    fn new() -> Self {
                        Self {
                            phantom: core::marker::PhantomData,
                        }
                    }

                    fn input_signal() -> crate::gpio::InputSignal {
                        crate::gpio::InputSignal::$signal
                    }

                    fn set_divider(divider: u8) {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        rmt.[< ch $ch_num conf0 >]().modify(|_, w| unsafe { w.div_cnt().bits(divider) });
                    }

                    fn update() {
                        // no-op
                    }

                    fn clear_interrupts() {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };

                        rmt.[< ch $ch_num conf1 >]().modify(|_, w| {
                            w.mem_wr_rst()
                                .set_bit()
                                .apb_mem_rst()
                                .set_bit()
                                .mem_owner()
                                .set_bit()
                                .rx_en()
                                .clear_bit()
                        });
                        Self::update();

                        rmt.int_clr().write(|w| {
                            w.[< ch $ch_num _rx_end >]()
                                .set_bit()
                                .[< ch $ch_num _err >]()
                                .set_bit()
                                .[< ch $ch_num _tx_thr_event >]()
                                .set_bit()
                        });
                    }

                    fn set_wrap_mode(_wrap: bool) {
                        // no-op
                    }

                    fn set_carrier(carrier: bool, high: u16, low: u16, level: bool) {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };

                        rmt.chcarrier_duty($ch_num)
                            .write(|w| unsafe { w.carrier_high().bits(high).carrier_low().bits(low) });

                        rmt.[< ch $ch_num conf0 >]()
                            .modify(|_, w| w.carrier_en().bit(carrier).carrier_out_lv().bit(level));
                    }

                    fn set_memsize(memsize: u8) {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };

                        rmt.[< ch $ch_num conf0 >]().modify(|_, w| unsafe { w.mem_size().bits(memsize) });
                    }

                    fn start_rx() {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };

                        rmt.[< ch $ch_num conf1 >]().modify(|_, w| {
                            w.mem_wr_rst()
                                .set_bit()
                                .apb_mem_rst()
                                .set_bit()
                                .mem_owner()
                                .set_bit()
                                .rx_en()
                                .set_bit()
                        });
                    }

                    fn is_done() -> bool {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        rmt.int_raw().read().[< ch $ch_num _rx_end >]().bit()
                    }

                    fn is_error() -> bool {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        rmt.int_raw().read().[< ch $ch_num _err >]().bit()
                    }

                    fn stop() {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        rmt.[< ch $ch_num conf1 >]().modify(|_, w| w.rx_en().clear_bit());
                    }

                    fn set_filter_threshold(value: u8) {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        rmt.[< ch $ch_num conf1 >]().modify(|_, w| unsafe {
                            w.rx_filter_en()
                                .bit(value > 0)
                                .rx_filter_thres()
                                .bits(value)
                        });
                    }

                    fn set_idle_threshold(value: u16) {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };

                        rmt.[< ch $ch_num conf0 >]().modify(|_, w| unsafe { w.idle_thres().bits(value) });
                    }

                    fn listen_interrupt(event: $crate::rmt::private::Event) {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        match event {
                            $crate::rmt::private::Event::Error => {
                                rmt.int_ena().modify(|_,w| w.[< ch $ch_num _err >]().set_bit());
                            }
                            $crate::rmt::private::Event::End => {
                                rmt.int_ena().modify(|_,w| w.[< ch $ch_num _rx_end >]().set_bit());
                            }
                            $crate::rmt::private::Event::Threshold => {
                                rmt.int_ena().modify(|_,w| w.[< ch $ch_num _tx_thr_event >]().set_bit());
                            }
                        }
                    }

                    fn unlisten_interrupt(event: $crate::rmt::private::Event) {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        match event {
                            $crate::rmt::private::Event::Error => {
                                rmt.int_ena().modify(|_,w| w.[< ch $ch_num _err >]().clear_bit());
                            }
                            $crate::rmt::private::Event::End => {
                                rmt.int_ena().modify(|_,w| w.[< ch $ch_num _rx_end >]().clear_bit());
                            }
                            $crate::rmt::private::Event::Threshold => {
                                rmt.int_ena().modify(|_,w| w.[< ch $ch_num _tx_thr_event >]().clear_bit());
                            }
                        }
                    }
                }
            }
        }
    }

    pub(crate) use impl_rx_channel;
    pub(crate) use impl_tx_channel;
}
