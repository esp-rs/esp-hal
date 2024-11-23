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
#![doc = concat!("[ESP-IDF documentation](https://docs.espressif.com/projects/esp-idf/en/latest/", crate::soc::chip!(), "/api-reference/peripherals/rmt.html)")]
//! ## Configuration
//! Each TX/RX channel has the same functionality controlled by a dedicated set
//! of registers and is able to independently transmit or receive data. TX
//! channels are indicated by n which is used as a placeholder for the channel
//! number, and by m for RX channels.
//!
//! ## Example
//!
//! ### Initialization
//!
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! # use esp_hal::peripherals::Peripherals;
//! # use esp_hal::rmt::TxChannelConfig;
//! # use esp_hal::rmt::Rmt;
//! # use crate::esp_hal::rmt::TxChannelCreator;
#![cfg_attr(esp32h2, doc = "let freq = 32.MHz();")]
#![cfg_attr(not(esp32h2), doc = "let freq = 80.MHz();")]
//! let rmt = Rmt::new(peripherals.RMT, freq).unwrap();
//! let mut channel = rmt
//!     .channel0
//!     .configure(
//!         peripherals.GPIO1,
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
//! 
//! > Note: on ESP32 and ESP32-S2 you cannot specify a base frequency other than 80 MHz

use core::{
    marker::PhantomData,
    pin::Pin,
    task::{Context, Poll},
};

use enumset::{EnumSet, EnumSetType};
use fugit::HertzU32;

use crate::{
    asynch::AtomicWaker,
    gpio::interconnect::{PeripheralInput, PeripheralOutput},
    macros::handler,
    peripheral::Peripheral,
    peripherals::Interrupt,
    soc::constants,
    system::{self, GenericPeripheralGuard},
    Async,
    Blocking,
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
    /// No transmission end marker found
    EndMarkerMissing,
}

///  Convenience trait to work with pulse codes.
pub trait PulseCode: crate::private::Sealed {
    /// Create a new instance
    fn new(level1: bool, length1: u16, level2: bool, length2: u16) -> Self;

    /// Create a new empty instance
    fn empty() -> Self;

    /// Set all levels and lengths to 0
    fn reset(&mut self);

    /// Logical output level in the first pulse code interval
    fn level1(&self) -> bool;

    /// Length of the first pulse code interval (in clock cycles)
    fn length1(&self) -> u16;

    /// Logical output level in the second pulse code interval
    fn level2(&self) -> bool;

    /// Length of the second pulse code interval (in clock cycles)
    fn length2(&self) -> u16;
}

impl PulseCode for u32 {
    fn new(level1: bool, length1: u16, level2: bool, length2: u16) -> Self {
        (((level1 as u32) << 15) | length1 as u32 & 0b111_1111_1111_1111)
            | (((level2 as u32) << 15) | length2 as u32 & 0b111_1111_1111_1111) << 16
    }

    fn empty() -> Self {
        0
    }

    fn reset(&mut self) {
        *self = 0
    }

    fn level1(&self) -> bool {
        self & (1 << 15) != 0
    }

    fn length1(&self) -> u16 {
        (self & 0b111_1111_1111_1111) as u16
    }

    fn level2(&self) -> bool {
        self & (1 << 31) != 0
    }

    fn length2(&self) -> u16 {
        ((self >> 16) & 0b111_1111_1111_1111) as u16
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

impl<'d, M> Rmt<'d, M>
where
    M: crate::Mode,
{
    pub(crate) fn new_internal(
        peripheral: impl Peripheral<P = crate::peripherals::RMT> + 'd,
        frequency: HertzU32,
    ) -> Result<Self, Error> {
        let me = Rmt::create(peripheral);

        #[cfg(any(esp32, esp32s2))]
        if frequency != HertzU32::MHz(80) {
            return Err(Error::UnreachableTargetFrequency);
        }

        cfg_if::cfg_if! {
            if #[cfg(any(esp32, esp32s2))] {
                self::chip_specific::configure_clock();
            } else {
                me.configure_clock(frequency)?;
            }
        }

        Ok(me)
    }

    #[cfg(not(any(esp32, esp32s2)))]
    fn configure_clock(&self, frequency: HertzU32) -> Result<(), Error> {
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
    pub fn new(
        peripheral: impl Peripheral<P = crate::peripherals::RMT> + 'd,
        frequency: HertzU32,
    ) -> Result<Self, Error> {
        Self::new_internal(peripheral, frequency)
    }

    /// Reconfigures the driver for asynchronous operation.
    pub fn into_async(mut self) -> Rmt<'d, Async> {
        self.set_interrupt_handler(async_interrupt_handler);
        Rmt::create(self.peripheral)
    }
}

impl crate::private::Sealed for Rmt<'_, Blocking> {}

impl InterruptConfigurable for Rmt<'_, Blocking> {
    fn set_interrupt_handler(&mut self, handler: crate::interrupt::InterruptHandler) {
        for core in crate::Cpu::other() {
            crate::interrupt::disable(core, Interrupt::RMT);
        }
        unsafe { crate::interrupt::bind_interrupt(Interrupt::RMT, handler.handler()) };
        unwrap!(crate::interrupt::enable(Interrupt::RMT, handler.priority()));
    }
}

fn configure_rx_channel<'d, P: PeripheralInput, T: RxChannelInternal<M>, M: crate::Mode>(
    pin: impl Peripheral<P = P> + 'd,
    config: RxChannelConfig,
) -> Result<T, Error> {
    if config.filter_threshold > 0b111_1111 {
        return Err(Error::InvalidArgument);
    }

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

    crate::into_mapped_ref!(pin);
    pin.init_input(crate::gpio::Pull::None, crate::private::Internal);
    T::input_signal().connect_to(pin);

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

fn configure_tx_channel<'d, P: PeripheralOutput, T: TxChannelInternal<M>, M: crate::Mode>(
    pin: impl Peripheral<P = P> + 'd,
    config: TxChannelConfig,
) -> Result<T, Error> {
    crate::into_mapped_ref!(pin);
    pin.set_to_push_pull_output(crate::private::Internal);
    T::output_signal().connect_to(pin);

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

/// Creates a TX channel
pub trait TxChannelCreator<'d, T, P>
where
    P: PeripheralOutput,
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
        configure_tx_channel(pin, config)
    }
}

/// Creates a TX channel in async mode
pub trait TxChannelCreatorAsync<'d, T, P>
where
    P: PeripheralOutput,
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
        configure_tx_channel(pin, config)
    }
}

/// Creates a RX channel
pub trait RxChannelCreator<'d, T, P>
where
    P: PeripheralInput,
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
        configure_rx_channel(pin, config)
    }
}

/// Creates a RX channel in async mode
pub trait RxChannelCreatorAsync<'d, T, P>
where
    P: PeripheralInput,
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
            if <C as TxChannelInternal<Blocking>>::is_error() {
                return Err((Error::TransmissionError, self.channel));
            }

            if self.index >= self.data.len() {
                break;
            }

            // wait for TX-THR
            while !<C as TxChannelInternal<Blocking>>::is_threshold_set() {}
            <C as TxChannelInternal<Blocking>>::reset_threshold_set();

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
                    ptr.add(idx).write_volatile(*entry);
                }
            }

            self.index += constants::RMT_CHANNEL_RAM_SIZE / 2;
        }

        loop {
            if <C as TxChannelInternal<Blocking>>::is_error() {
                return Err((Error::TransmissionError, self.channel));
            }

            if <C as TxChannelInternal<Blocking>>::is_done() {
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
        <C as TxChannelInternal<Blocking>>::set_continuous(false);
        <C as TxChannelInternal<Blocking>>::update();

        loop {
            if <C as TxChannelInternal<Blocking>>::is_error() {
                return Err((Error::TransmissionError, self.channel));
            }

            if <C as TxChannelInternal<Blocking>>::is_done() {
                break;
            }
        }

        Ok(self.channel)
    }

    /// Stop transaction as soon as possible.
    pub fn stop(self) -> Result<C, (Error, C)> {
        <C as TxChannelInternal<Blocking>>::set_continuous(false);
        <C as TxChannelInternal<Blocking>>::update();

        let ptr = (constants::RMT_RAM_START
            + C::CHANNEL as usize * constants::RMT_CHANNEL_RAM_SIZE * 4)
            as *mut u32;
        for idx in 0..constants::RMT_CHANNEL_RAM_SIZE {
            unsafe {
                ptr.add(idx).write_volatile(0);
            }
        }

        loop {
            if <C as TxChannelInternal<Blocking>>::is_error() {
                return Err((Error::TransmissionError, self.channel));
            }

            if <C as TxChannelInternal<Blocking>>::is_done() {
                break;
            }
        }

        Ok(self.channel)
    }

    /// Check if the `loopcount` interrupt bit is set
    pub fn is_loopcount_interrupt_set(&self) -> bool {
        <C as TxChannelInternal<Blocking>>::is_loopcount_interrupt_set()
    }
}

macro_rules! impl_tx_channel_creator {
    ($channel:literal) => {
        impl<'d, P>
            $crate::rmt::TxChannelCreator<'d, $crate::rmt::Channel<$crate::Blocking, $channel>, P>
            for ChannelCreator<$crate::Blocking, $channel>
        where
            P: $crate::gpio::interconnect::PeripheralOutput,
        {
        }

        impl $crate::rmt::TxChannel for $crate::rmt::Channel<$crate::Blocking, $channel> {}

        impl<'d, P>
            $crate::rmt::TxChannelCreatorAsync<'d, $crate::rmt::Channel<$crate::Async, $channel>, P>
            for ChannelCreator<$crate::Async, $channel>
        where
            P: $crate::gpio::interconnect::PeripheralOutput,
        {
        }

        impl $crate::rmt::TxChannelAsync for $crate::rmt::Channel<$crate::Async, $channel> {}
    };
}

macro_rules! impl_rx_channel_creator {
    ($channel:literal) => {
        impl<'d, P>
            $crate::rmt::RxChannelCreator<'d, $crate::rmt::Channel<$crate::Blocking, $channel>, P>
            for ChannelCreator<$crate::Blocking, $channel>
        where
            P: $crate::gpio::interconnect::PeripheralInput,
        {
        }

        impl $crate::rmt::RxChannel for $crate::rmt::Channel<$crate::Blocking, $channel> {}

        impl<'d, P>
            $crate::rmt::RxChannelCreatorAsync<'d, $crate::rmt::Channel<$crate::Async, $channel>, P>
            for ChannelCreator<$crate::Async, $channel>
        where
            P: $crate::gpio::interconnect::PeripheralInput,
        {
        }

        impl $crate::rmt::RxChannelAsync for $crate::rmt::Channel<$crate::Async, $channel> {}
    };
}

#[cfg(not(any(esp32, esp32s2, esp32s3)))]
mod impl_for_chip {
    use core::marker::PhantomData;

    use crate::{
        peripheral::{Peripheral, PeripheralRef},
        system::GenericPeripheralGuard,
    };

    /// RMT Instance
    pub struct Rmt<'d, M>
    where
        M: crate::Mode,
    {
        pub(super) peripheral: PeripheralRef<'d, crate::peripherals::RMT>,
        /// RMT Channel 0.
        pub channel0: ChannelCreator<M, 0>,
        /// RMT Channel 1.
        pub channel1: ChannelCreator<M, 1>,
        /// RMT Channel 2.
        pub channel2: ChannelCreator<M, 2>,
        /// RMT Channel 3.
        pub channel3: ChannelCreator<M, 3>,
        phantom: PhantomData<M>,
    }

    impl<'d, M> Rmt<'d, M>
    where
        M: crate::Mode,
    {
        pub(super) fn create(
            peripheral: impl Peripheral<P = crate::peripherals::RMT> + 'd,
        ) -> Self {
            crate::into_ref!(peripheral);

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
    pub struct ChannelCreator<M, const CHANNEL: u8>
    where
        M: crate::Mode,
    {
        phantom: PhantomData<M>,
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

    use crate::{
        peripheral::{Peripheral, PeripheralRef},
        system::GenericPeripheralGuard,
    };

    /// RMT Instance
    pub struct Rmt<'d, M>
    where
        M: crate::Mode,
    {
        pub(super) peripheral: PeripheralRef<'d, crate::peripherals::RMT>,
        /// RMT Channel 0.
        pub channel0: ChannelCreator<M, 0>,
        /// RMT Channel 1.
        pub channel1: ChannelCreator<M, 1>,
        /// RMT Channel 2.
        pub channel2: ChannelCreator<M, 2>,
        /// RMT Channel 3.
        pub channel3: ChannelCreator<M, 3>,
        /// RMT Channel 4.
        pub channel4: ChannelCreator<M, 4>,
        /// RMT Channel 5.
        pub channel5: ChannelCreator<M, 5>,
        /// RMT Channel 6.
        pub channel6: ChannelCreator<M, 6>,
        /// RMT Channel 7.
        pub channel7: ChannelCreator<M, 7>,
        phantom: PhantomData<M>,
    }

    impl<'d, M> Rmt<'d, M>
    where
        M: crate::Mode,
    {
        pub(super) fn create(
            peripheral: impl Peripheral<P = crate::peripherals::RMT> + 'd,
        ) -> Self {
            crate::into_ref!(peripheral);
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
    pub struct ChannelCreator<M, const CHANNEL: u8>
    where
        M: crate::Mode,
    {
        phantom: PhantomData<M>,
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

    use crate::{
        peripheral::{Peripheral, PeripheralRef},
        system::GenericPeripheralGuard,
    };

    /// RMT Instance
    pub struct Rmt<'d, M>
    where
        M: crate::Mode,
    {
        pub(super) peripheral: PeripheralRef<'d, crate::peripherals::RMT>,
        /// RMT Channel 0.
        pub channel0: ChannelCreator<M, 0>,
        /// RMT Channel 1.
        pub channel1: ChannelCreator<M, 1>,
        /// RMT Channel 2.
        pub channel2: ChannelCreator<M, 2>,
        /// RMT Channel 3.
        pub channel3: ChannelCreator<M, 3>,
        phantom: PhantomData<M>,
    }

    impl<'d, M> Rmt<'d, M>
    where
        M: crate::Mode,
    {
        pub(super) fn create(
            peripheral: impl Peripheral<P = crate::peripherals::RMT> + 'd,
        ) -> Self {
            crate::into_ref!(peripheral);

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
    pub struct ChannelCreator<M, const CHANNEL: u8>
    where
        M: crate::Mode,
    {
        phantom: PhantomData<M>,
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

    use crate::{
        peripheral::{Peripheral, PeripheralRef},
        system::GenericPeripheralGuard,
    };

    /// RMT Instance
    pub struct Rmt<'d, M>
    where
        M: crate::Mode,
    {
        pub(super) peripheral: PeripheralRef<'d, crate::peripherals::RMT>,
        /// RMT Channel 0.
        pub channel0: ChannelCreator<M, 0>,
        /// RMT Channel 1.
        pub channel1: ChannelCreator<M, 1>,
        /// RMT Channel 2.
        pub channel2: ChannelCreator<M, 2>,
        /// RMT Channel 3.
        pub channel3: ChannelCreator<M, 3>,
        /// RMT Channel 4.
        pub channel4: ChannelCreator<M, 4>,
        /// RMT Channel 5.
        pub channel5: ChannelCreator<M, 5>,
        /// RMT Channel 6.
        pub channel6: ChannelCreator<M, 6>,
        /// RMT Channel 7.
        pub channel7: ChannelCreator<M, 7>,
        phantom: PhantomData<M>,
    }

    impl<'d, M> Rmt<'d, M>
    where
        M: crate::Mode,
    {
        pub(super) fn create(
            peripheral: impl Peripheral<P = crate::peripherals::RMT> + 'd,
        ) -> Self {
            crate::into_ref!(peripheral);

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
    pub struct ChannelCreator<M, const CHANNEL: u8>
    where
        M: crate::Mode,
    {
        phantom: PhantomData<M>,
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
pub struct Channel<M, const CHANNEL: u8>
where
    M: crate::Mode,
{
    phantom: PhantomData<M>,
    _guard: GenericPeripheralGuard<{ system::Peripheral::Rmt as u8 }>,
}

/// Channel in TX mode
pub trait TxChannel: TxChannelInternal<Blocking> {
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
        if data.len() > constants::RMT_CHANNEL_RAM_SIZE {
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
            if <C as RxChannelInternal<Blocking>>::is_error() {
                return Err((Error::TransmissionError, self.channel));
            }

            if <C as RxChannelInternal<Blocking>>::is_done() {
                break;
            }
        }

        <C as RxChannelInternal<Blocking>>::stop();
        <C as RxChannelInternal<Blocking>>::clear_interrupts();
        <C as RxChannelInternal<Blocking>>::update();

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
pub trait RxChannel: RxChannelInternal<Blocking> {
    /// Start receiving pulse codes into the given buffer.
    /// This returns a [RxTransaction] which can be used to wait for receive to
    /// complete and get back the channel for further use.
    /// The length of the received data cannot exceed the allocated RMT RAM.
    fn receive(self, data: &mut [u32]) -> Result<RxTransaction<'_, Self>, Error>
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
pub trait TxChannelAsync: TxChannelInternal<Async> {
    /// Start transmitting the given pulse code sequence.
    /// The length of sequence cannot exceed the size of the allocated RMT
    /// RAM.
    async fn transmit(&mut self, data: &[u32]) -> Result<(), Error>
    where
        Self: Sized,
    {
        if data.len() > constants::RMT_CHANNEL_RAM_SIZE {
            return Err(Error::InvalidArgument);
        }

        Self::clear_interrupts();
        Self::listen_interrupt(Event::End);
        Self::listen_interrupt(Event::Error);
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
pub trait RxChannelAsync: RxChannelInternal<Async> {
    /// Start receiving a pulse code sequence.
    /// The length of sequence cannot exceed the size of the allocated RMT
    /// RAM.
    async fn receive<T: From<u32> + Copy>(&mut self, data: &mut [T]) -> Result<(), Error>
    where
        Self: Sized,
    {
        if data.len() > constants::RMT_CHANNEL_RAM_SIZE {
            return Err(Error::InvalidArgument);
        }

        Self::clear_interrupts();
        Self::listen_interrupt(Event::End);
        Self::listen_interrupt(Event::Error);
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
            <Channel<Async, 0> as TxChannelInternal<Async>>::unlisten_interrupt(
                Event::End | Event::Error,
            );
            <Channel<Async, 0> as RxChannelInternal<Async>>::unlisten_interrupt(
                Event::End | Event::Error,
            );
        }
        1 => {
            <Channel<Async, 1> as TxChannelInternal<Async>>::unlisten_interrupt(
                Event::End | Event::Error,
            );
            <Channel<Async, 1> as RxChannelInternal<Async>>::unlisten_interrupt(
                Event::End | Event::Error,
            );
        }
        2 => {
            <Channel<Async, 2> as TxChannelInternal<Async>>::unlisten_interrupt(
                Event::End | Event::Error,
            );
            <Channel<Async, 2> as RxChannelInternal<Async>>::unlisten_interrupt(
                Event::End | Event::Error,
            );
        }
        3 => {
            <Channel<Async, 3> as TxChannelInternal<Async>>::unlisten_interrupt(
                Event::End | Event::Error,
            );
            <Channel<Async, 3> as RxChannelInternal<Async>>::unlisten_interrupt(
                Event::End | Event::Error,
            );
        }
        #[cfg(esp32)]
        4 => {
            <Channel<Async, 4> as TxChannelInternal<Async>>::unlisten_interrupt(
                Event::End | Event::Error,
            );
            <Channel<Async, 4> as RxChannelInternal<Async>>::unlisten_interrupt(
                Event::End | Event::Error,
            );
        }
        #[cfg(any(esp32, esp32s3))]
        5 => {
            <Channel<Async, 5> as TxChannelInternal<Async>>::unlisten_interrupt(
                Event::End | Event::Error,
            );
            <Channel<Async, 5> as RxChannelInternal<Async>>::unlisten_interrupt(
                Event::End | Event::Error,
            );
        }
        #[cfg(any(esp32, esp32s3))]
        6 => {
            <Channel<Async, 6> as TxChannelInternal<Async>>::unlisten_interrupt(
                Event::End | Event::Error,
            );
            <Channel<Async, 6> as RxChannelInternal<Async>>::unlisten_interrupt(
                Event::End | Event::Error,
            );
        }
        #[cfg(any(esp32, esp32s3))]
        7 => {
            <Channel<Async, 7> as TxChannelInternal<Async>>::unlisten_interrupt(
                Event::End | Event::Error,
            );
            <Channel<Async, 7> as RxChannelInternal<Async>>::unlisten_interrupt(
                Event::End | Event::Error,
            );
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
            .take(constants::RMT_CHANNEL_RAM_SIZE)
            .enumerate()
        {
            unsafe {
                ptr.add(idx).write_volatile(*entry);
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
            Ok(constants::RMT_CHANNEL_RAM_SIZE)
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
    pub fn configure_clock(div: u32) {
        #[cfg(not(pcr))]
        {
            let rmt = unsafe { &*crate::peripherals::RMT::PTR };
            rmt.sys_conf().modify(|_, w| unsafe {
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
            let pcr = unsafe { &*crate::peripherals::PCR::PTR };
            pcr.rmt_sclk_conf().modify(|_, w| unsafe {
                w.sclk_div_num().bits(div as u8);
                w.sclk_div_a().bits(0);
                w.sclk_div_b().bits(0)
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
        ($signal:ident, $ch_num:literal) => {
            paste::paste! {
                impl<M> $crate::rmt::TxChannelInternal<M> for $crate::rmt::Channel<M, $ch_num> where M: $crate::Mode {
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

                        rmt.ch_tx_lim($ch_num).modify(|_, w| w.loop_count_reset().clear_bit());
                    }

                    fn clear_interrupts() {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };

                        rmt.int_clr().write(|w| {
                            w.[< ch $ch_num _tx_end >]().set_bit();
                            w.[< ch $ch_num _tx_err >]().set_bit();
                            w.[< ch $ch_num _tx_loop >]().set_bit();
                            w.[< ch $ch_num _tx_thr_event >]().set_bit()
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
                            w.carrier_en().bit(carrier);
                            w.carrier_eff_en().set_bit();
                            w.carrier_out_lv().bit(level)
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
                            w.mem_rd_rst().set_bit();
                            w.apb_mem_rst().set_bit();
                            w.tx_start().set_bit()
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

                    fn enable_listen_interrupt(events: enumset::EnumSet<$crate::rmt::Event>, enable: bool) {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        rmt.int_ena().modify(|_, w| {
                            for event in events {
                                match event {
                                    $crate::rmt::Event::Error => w.[< ch $ch_num _tx_err >]().bit(enable),
                                    $crate::rmt::Event::End => w.[< ch $ch_num _tx_end >]().bit(enable),
                                    $crate::rmt::Event::Threshold => w.[< ch $ch_num _tx_thr_event >]().bit(enable),
                                };
                            }
                            w
                        });
                    }
                }
            }
        }
    }

    macro_rules! impl_rx_channel {
        ($signal:ident, $ch_num:literal, $ch_index:literal) => {
            paste::paste! {
                impl<M> $crate::rmt::RxChannelInternal<M> for $crate::rmt::Channel<M, $ch_num> where M: $crate::Mode {
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
                            w.[< ch $ch_num _rx_end >]().set_bit();
                            w.[< ch $ch_num _rx_err >]().set_bit();
                            w.[< ch $ch_num _rx_thr_event >]().set_bit()
                        });
                    }

                    fn set_wrap_mode(wrap: bool) {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        rmt.[< ch $ch_num _rx_conf1 >]().modify(|_, w| w.mem_rx_wrap_en().bit(wrap));
                    }

                    fn set_carrier(carrier: bool, high: u16, low: u16, level: bool) {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };

                        rmt.ch_rx_carrier_rm($ch_index).write(|w| unsafe {
                            w.carrier_high_thres().bits(high);
                            w.carrier_low_thres().bits(low)
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
                            w.mem_wr_rst().set_bit();
                            w.apb_mem_rst().set_bit();
                            w.mem_owner().set_bit();
                            w.rx_en().set_bit()
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
                            w.rx_filter_en().bit(value > 0);
                            w.rx_filter_thres().bits(value)
                        });
                    }

                    fn set_idle_threshold(value: u16) {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };

                        rmt.[< ch $ch_num _rx_conf0 >]().modify(|_, w| unsafe { w.idle_thres().bits(value) });
                    }

                    fn enable_listen_interrupt(events: enumset::EnumSet<$crate::rmt::Event>, enable: bool) {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        rmt.int_ena().modify(|_, w| {
                            for event in events {
                                match event {
                                    $crate::rmt::Event::Error => w.[< ch $ch_num _rx_err >]().bit(enable),
                                    $crate::rmt::Event::End => w.[< ch $ch_num _rx_end >]().bit(enable),
                                    $crate::rmt::Event::Threshold => w.[< ch $ch_num _rx_thr_event >]().bit(enable),
                                };
                            }
                            w
                        });
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
        ($signal:ident, $ch_num:literal) => {
            paste::paste! {
                impl<M> super::TxChannelInternal<M> for $crate::rmt::Channel<M, $ch_num> where M: $crate::Mode {
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
                            w.[< ch $ch_num _err >]().set_bit();
                            w.[< ch $ch_num _tx_end >]().set_bit();
                            w.[< ch $ch_num _tx_thr_event >]().set_bit()
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
                            w.mem_rd_rst().set_bit();
                            w.apb_mem_rst().set_bit();
                            w.tx_start().set_bit()
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

                    fn enable_listen_interrupt(events: enumset::EnumSet<$crate::rmt::Event>, enable: bool) {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        rmt.int_ena().modify(|_,w| {
                            for event in events {
                                match event {
                                    $crate::rmt::Event::Error => w.[< ch $ch_num _err >]().bit(enable),
                                    $crate::rmt::Event::End => w.[< ch $ch_num _tx_end >]().bit(enable),
                                    $crate::rmt::Event::Threshold => w.[< ch $ch_num _tx_thr_event >]().bit(enable),
                                };
                            }
                            w
                        });
                    }
                }
            }
        }
    }

    macro_rules! impl_rx_channel {
        ($signal:ident, $ch_num:literal) => {
            paste::paste! {
                impl<M> super::RxChannelInternal<M> for $crate::rmt::Channel<M, $ch_num> where M: $crate::Mode {
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
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        rmt.[< ch $ch_num conf0 >]().modify(|_, w| unsafe { w.div_cnt().bits(divider) });
                    }

                    fn update() {
                        // no-op
                    }

                    fn clear_interrupts() {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };

                        rmt.[< ch $ch_num conf1 >]().modify(|_, w| {
                            w.mem_wr_rst().set_bit();
                            w.apb_mem_rst().set_bit();
                            w.mem_owner().set_bit();
                            w.rx_en().clear_bit()
                        });
                        Self::update();

                        rmt.int_clr().write(|w| {
                            w.[< ch $ch_num _rx_end >]().set_bit();
                            w.[< ch $ch_num _err >]().set_bit();
                            w.[< ch $ch_num _tx_thr_event >]().set_bit()
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
                            w.mem_wr_rst().set_bit();
                            w.apb_mem_rst().set_bit();
                            w.mem_owner().set_bit();
                            w.rx_en().set_bit()
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
                            w.rx_filter_en().bit(value > 0);
                            w.rx_filter_thres().bits(value)
                        });
                    }

                    fn set_idle_threshold(value: u16) {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };

                        rmt.[< ch $ch_num conf0 >]().modify(|_, w| unsafe { w.idle_thres().bits(value) });
                    }

                    fn enable_listen_interrupt(events: enumset::EnumSet<$crate::rmt::Event>, enable: bool) {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        rmt.int_ena().modify(|_, w| {
                            for event in events {
                                match event {
                                    $crate::rmt::Event::Error => w.[< ch $ch_num _err >]().bit(enable),
                                    $crate::rmt::Event::End => w.[< ch $ch_num _rx_end >]().bit(enable),
                                    $crate::rmt::Event::Threshold => w.[< ch $ch_num _tx_thr_event >]().bit(enable),
                                };
                            }
                            w
                        });
                    }
                }
            }
        }
    }

    pub(crate) use impl_rx_channel;
    pub(crate) use impl_tx_channel;
}
