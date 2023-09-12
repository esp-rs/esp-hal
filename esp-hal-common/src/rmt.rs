//! # Remote Control Peripheral (RMT)
//!
//! ## Overview
//! Some ESP32 variants include a remote control peripheral (RMT) that
//! is designed to handle infrared remote control signals. For that
//! purpose, it can convert bitstreams of data (from the RAM) into
//! pulse codes and even modulate those codes into a carrier wave.
//!
//! It can also convert received pulse codes (again, with carrier
//! wave support) into data bits.
//!
//! A secondary use case for this peripheral is to drive RGB(W) LEDs
//! that bear an internal IC and use a pulse code protocol.
//!
//! ## Channels
//! The RMT peripheral has the following channels available
//! on individual chips:
//!
//! * The **ESP32** has 8 channels, each of them can be either receiver or
//!   transmitter
//! * The **ESP32-C3** has 4 channels, `Channel0` and `Channel1` hardcoded for
//!   transmitting signals and `Channel2` and `Channel3` hardcoded for receiving
//!   signals.
//! * The **ESP32-C6** has 4 channels, `Channel0` and `Channel1` hardcoded for
//!   transmitting signals and `Channel2` and `Channel3` hardcoded for receiving
//!   signals.
//! * The **ESP32-H2** has 4 channels, `Channel0` and `Channel1` hardcoded for
//!   transmitting signals and `Channel2` and `Channel3` hardcoded for receiving
//!   signals.
//! * The **ESP32-S2** has 4 channels, each of them can be either receiver or
//!   transmitter.
//! * The **ESP32-S3** has 8 channels, `Channel0`-`Channel3` hardcoded for
//!   transmitting signals and `Channel4`-`Channel7` hardcoded for receiving
//!   signals.
//!
//! For more information, please refer to the ESP-IDF documentation:
//! <https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/rmt.html>
//!
//! ## Examples
//!
//! ### Initialization
//!
//! ```no_run
//! let rmt = Rmt::new(peripherals.RMT, 80u32.MHz(), &mut clock_control, &clocks).unwrap();
//! let mut channel = rmt
//!     .channel0
//!     .configure(
//!         io.pins.gpio1.into_push_pull_output(),
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
//! ```
//! (on ESP32 and ESP32-S2 you cannot specify a base frequency other than 80
//! MHz)
//!
//! ### Sending a pulse sequence
//!
//! ```no_run
//! let data = [
//!     PulseCode {
//!         level1: true,
//!         length1: 100,
//!         level2: false,
//!         length2: 300,
//!     },
//!     PulseCode {
//!         level1: true,
//!         length1: 100,
//!         level2: false,
//!         length2: 0, // a zero-length pulse marks the end of the sequence
//!     },
//! ];
//!
//! let transaction = channel.transmit(&data);
//! channel = transaction.wait().unwrap();
//! ```

use fugit::HertzU32;

use crate::{
    clock::Clocks,
    gpio::{InputPin, OutputPin},
    peripheral::Peripheral,
    rmt::private::CreateInstance,
    soc::constants,
    system::PeripheralClockControl,
};

#[derive(Debug, Copy, Clone)]
pub enum Error {
    UnreachableTargetFrequency,
    Overflow,
    InvalidArgument,
    TransmissionError,
}

/// Convenience representation of a pulse code entry.
///
/// Allows for the assignment of two levels and their lengths
#[derive(Clone, Copy, Debug, Default)]
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

pub use impl_for_chip::Rmt;

impl<'d> Rmt<'d> {
    /// Create a new RMT instance
    pub fn new(
        peripheral: impl Peripheral<P = crate::peripherals::RMT> + 'd,
        frequency: HertzU32,
        peripheral_clock_control: &mut PeripheralClockControl,
        _clocks: &Clocks,
    ) -> Result<Self, Error> {
        let me = Rmt::create(peripheral);

        #[cfg(any(esp32, esp32s2))]
        if frequency != HertzU32::MHz(80) {
            return Err(Error::UnreachableTargetFrequency);
        }

        peripheral_clock_control.enable(crate::system::Peripheral::Rmt);

        #[cfg(not(any(esp32, esp32s2)))]
        me.configure_clock(frequency, _clocks)?;

        #[cfg(any(esp32, esp32s2))]
        self::chip_specific::configure_clock();

        Ok(me)
    }

    #[cfg(not(any(esp32, esp32s2)))]
    fn configure_clock(&self, frequency: HertzU32, _clocks: &Clocks) -> Result<(), Error> {
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

pub trait TxChannelCreator<'d, T, P, const CHANNEL: u8>
where
    P: OutputPin,
    T: TxChannel<CHANNEL>,
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
        pin.set_to_push_pull_output()
            .connect_peripheral_to_output(T::output_signal());
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

pub trait RxChannelCreator<'d, T, P, const CHANNEL: u8>
where
    P: InputPin,
    T: RxChannel<CHANNEL>,
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
        pin.set_to_input()
            .connect_input_to_peripheral(T::input_signal());
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
pub struct SingleShotTxTransaction<'a, C, T: Into<u32> + Copy, const CHANNEL: u8>
where
    C: TxChannel<CHANNEL>,
{
    channel: C,
    index: usize,
    data: &'a [T],
}

impl<'a, C, T: Into<u32> + Copy, const CHANNEL: u8> SingleShotTxTransaction<'a, C, T, CHANNEL>
where
    C: TxChannel<CHANNEL>,
{
    /// Wait for the transaction to complete
    pub fn wait(mut self) -> Result<C, (Error, C)> {
        loop {
            if <C as private::TxChannelInternal<CHANNEL>>::is_error() {
                return Err((Error::TransmissionError, self.channel));
            }

            if self.index < self.data.len() {
                // wait for TX-THR
                loop {
                    if <C as private::TxChannelInternal<CHANNEL>>::is_threshold_set() {
                        break;
                    }
                }
                <C as private::TxChannelInternal<CHANNEL>>::reset_threshold_set();

                // re-fill TX RAM
                let ram_index = (((self.index - constants::RMT_CHANNEL_RAM_SIZE)
                    / (constants::RMT_CHANNEL_RAM_SIZE / 2))
                    % 2)
                    * (constants::RMT_CHANNEL_RAM_SIZE / 2);

                let ptr = (constants::RMT_RAM_START
                    + CHANNEL as usize * constants::RMT_CHANNEL_RAM_SIZE * 4
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

                self.index = self.index + constants::RMT_CHANNEL_RAM_SIZE / 2;
            } else {
                break;
            }
        }

        loop {
            if <C as private::TxChannelInternal<CHANNEL>>::is_error() {
                return Err((Error::TransmissionError, self.channel));
            }

            if <C as private::TxChannelInternal<CHANNEL>>::is_done() {
                break;
            }
        }

        Ok(self.channel)
    }
}

/// An in-progress continuous TX transaction
pub struct ContinuousTxTransaction<C, const CHANNEL: u8>
where
    C: TxChannel<CHANNEL>,
{
    channel: C,
}

impl<C, const CHANNEL: u8> ContinuousTxTransaction<C, CHANNEL>
where
    C: TxChannel<CHANNEL>,
{
    /// Stop transaction when the current iteration ends.
    pub fn stop_next(self) -> Result<C, (Error, C)> {
        <C as private::TxChannelInternal<CHANNEL>>::set_continuous(false);
        <C as private::TxChannelInternal<CHANNEL>>::update();

        loop {
            if <C as private::TxChannelInternal<CHANNEL>>::is_error() {
                return Err((Error::TransmissionError, self.channel));
            }

            if <C as private::TxChannelInternal<CHANNEL>>::is_done() {
                break;
            }
        }

        Ok(self.channel)
    }

    /// Stop transaction as soon as possible.
    pub fn stop(self) -> Result<C, (Error, C)> {
        <C as private::TxChannelInternal<CHANNEL>>::set_continuous(false);
        <C as private::TxChannelInternal<CHANNEL>>::update();

        let ptr = (constants::RMT_RAM_START
            + CHANNEL as usize * constants::RMT_CHANNEL_RAM_SIZE * 4) as *mut u32;
        for idx in 0..constants::RMT_CHANNEL_RAM_SIZE {
            unsafe {
                ptr.add(idx).write_volatile(0);
            }
        }

        loop {
            if <C as private::TxChannelInternal<CHANNEL>>::is_error() {
                return Err((Error::TransmissionError, self.channel));
            }

            if <C as private::TxChannelInternal<CHANNEL>>::is_done() {
                break;
            }
        }

        Ok(self.channel)
    }

    pub fn is_loopcount_interrupt_set(&self) -> bool {
        <C as private::TxChannelInternal<CHANNEL>>::is_loopcount_interrupt_set()
    }
}

macro_rules! impl_tx_channel_creator {
    ($channel:literal) => {
        paste::paste! {
            impl<'d, P, const CHANNEL: u8> $crate::rmt::TxChannelCreator<'d, $crate::rmt::[< Channel $channel >] <CHANNEL>, P, CHANNEL>
            for [< Channel $channel Creator >]<CHANNEL>
            where
                P: $crate::gpio::OutputPin,
            {
            }

            impl<const CHANNEL: u8> $crate::rmt::TxChannel<CHANNEL> for $crate::rmt::[< Channel $channel >]<CHANNEL> {}

            #[cfg(feature = "async")]
            impl<const CHANNEL: u8> $crate::rmt::asynch::TxChannelAsync<CHANNEL> for $crate::rmt::[< Channel $channel >]<CHANNEL> {}
        }
    }
}

macro_rules! impl_rx_channel_creator {
    ($channel:literal) => {
        paste::paste! {
            impl<'d, P, const CHANNEL: u8> $crate::rmt::RxChannelCreator<'d, $crate::rmt::[< Channel $channel >] <CHANNEL>, P, CHANNEL>
            for [< Channel $channel Creator >]<CHANNEL>
            where
                P: $crate::gpio::InputPin,
            {
            }

            impl<const CHANNEL: u8> $crate::rmt::RxChannel<CHANNEL> for $crate::rmt::[< Channel $channel >]<CHANNEL> {}

            #[cfg(feature = "async")]
            impl<const CHANNEL: u8> $crate::rmt::asynch::RxChannelAsync<CHANNEL> for $crate::rmt::[< Channel $channel >]<CHANNEL> {}
        }
    }
}

#[cfg(not(any(esp32, esp32s2, esp32s3)))]
mod impl_for_chip {
    use super::private::CreateInstance;
    use crate::peripheral::{Peripheral, PeripheralRef};

    /// RMT Instance
    pub struct Rmt<'d> {
        _peripheral: PeripheralRef<'d, crate::peripherals::RMT>,
        pub channel0: Channel0Creator<0>,
        pub channel1: Channel1Creator<1>,
        pub channel2: Channel2Creator<2>,
        pub channel3: Channel3Creator<3>,
    }

    impl<'d> CreateInstance<'d> for Rmt<'d> {
        fn create(peripheral: impl Peripheral<P = crate::peripherals::RMT> + 'd) -> Self {
            crate::into_ref!(peripheral);

            Self {
                _peripheral: peripheral,
                channel0: Channel0Creator {},
                channel1: Channel1Creator {},
                channel2: Channel2Creator {},
                channel3: Channel3Creator {},
            }
        }
    }

    pub struct Channel0Creator<const CHANNEL: u8> {}

    pub struct Channel1Creator<const CHANNEL: u8> {}

    pub struct Channel2Creator<const CHANNEL: u8> {}

    pub struct Channel3Creator<const CHANNEL: u8> {}

    impl_tx_channel_creator!(0);
    impl_tx_channel_creator!(1);

    impl_rx_channel_creator!(2);
    impl_rx_channel_creator!(3);

    super::chip_specific::impl_tx_channel!(Channel0, RMT_SIG_0, 0);
    super::chip_specific::impl_tx_channel!(Channel1, RMT_SIG_1, 1);

    super::chip_specific::impl_rx_channel!(Channel2, RMT_SIG_0, 2, 0);
    super::chip_specific::impl_rx_channel!(Channel3, RMT_SIG_1, 3, 1);
}

#[cfg(any(esp32))]
mod impl_for_chip {
    use super::private::CreateInstance;
    use crate::peripheral::{Peripheral, PeripheralRef};

    /// RMT Instance
    pub struct Rmt<'d> {
        _peripheral: PeripheralRef<'d, crate::peripherals::RMT>,
        pub channel0: Channel0Creator<0>,
        pub channel1: Channel1Creator<1>,
        pub channel2: Channel2Creator<2>,
        pub channel3: Channel3Creator<3>,
        pub channel4: Channel4Creator<4>,
        pub channel5: Channel5Creator<5>,
        pub channel6: Channel6Creator<6>,
        pub channel7: Channel7Creator<7>,
    }

    impl<'d> CreateInstance<'d> for Rmt<'d> {
        fn create(peripheral: impl Peripheral<P = crate::peripherals::RMT> + 'd) -> Self {
            crate::into_ref!(peripheral);

            Self {
                _peripheral: peripheral,
                channel0: Channel0Creator {},
                channel1: Channel1Creator {},
                channel2: Channel2Creator {},
                channel3: Channel3Creator {},
                channel4: Channel4Creator {},
                channel5: Channel5Creator {},
                channel6: Channel6Creator {},
                channel7: Channel7Creator {},
            }
        }
    }

    pub struct Channel0Creator<const CHANNEL: u8> {}

    pub struct Channel1Creator<const CHANNEL: u8> {}

    pub struct Channel2Creator<const CHANNEL: u8> {}

    pub struct Channel3Creator<const CHANNEL: u8> {}

    pub struct Channel4Creator<const CHANNEL: u8> {}

    pub struct Channel5Creator<const CHANNEL: u8> {}

    pub struct Channel6Creator<const CHANNEL: u8> {}

    pub struct Channel7Creator<const CHANNEL: u8> {}

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

    super::chip_specific::impl_tx_channel!(Channel0, RMT_SIG_0, 0);
    super::chip_specific::impl_tx_channel!(Channel1, RMT_SIG_1, 1);
    super::chip_specific::impl_tx_channel!(Channel2, RMT_SIG_2, 2);
    super::chip_specific::impl_tx_channel!(Channel3, RMT_SIG_3, 3);
    super::chip_specific::impl_tx_channel!(Channel4, RMT_SIG_4, 4);
    super::chip_specific::impl_tx_channel!(Channel5, RMT_SIG_5, 5);
    super::chip_specific::impl_tx_channel!(Channel6, RMT_SIG_6, 6);
    super::chip_specific::impl_tx_channel!(Channel7, RMT_SIG_7, 7);

    super::chip_specific::impl_rx_channel!(Channel0, RMT_SIG_0, 0);
    super::chip_specific::impl_rx_channel!(Channel1, RMT_SIG_1, 1);
    super::chip_specific::impl_rx_channel!(Channel2, RMT_SIG_2, 2);
    super::chip_specific::impl_rx_channel!(Channel3, RMT_SIG_3, 3);
    super::chip_specific::impl_rx_channel!(Channel4, RMT_SIG_4, 4);
    super::chip_specific::impl_rx_channel!(Channel5, RMT_SIG_5, 5);
    super::chip_specific::impl_rx_channel!(Channel6, RMT_SIG_6, 6);
    super::chip_specific::impl_rx_channel!(Channel7, RMT_SIG_7, 7);
}

#[cfg(any(esp32s2))]
mod impl_for_chip {
    use super::private::CreateInstance;
    use crate::peripheral::{Peripheral, PeripheralRef};

    /// RMT Instance
    pub struct Rmt<'d> {
        _peripheral: PeripheralRef<'d, crate::peripherals::RMT>,
        pub channel0: Channel0Creator<0>,
        pub channel1: Channel1Creator<1>,
        pub channel2: Channel2Creator<2>,
        pub channel3: Channel3Creator<3>,
    }

    impl<'d> CreateInstance<'d> for Rmt<'d> {
        fn create(peripheral: impl Peripheral<P = crate::peripherals::RMT> + 'd) -> Self {
            crate::into_ref!(peripheral);

            Self {
                _peripheral: peripheral,
                channel0: Channel0Creator {},
                channel1: Channel1Creator {},
                channel2: Channel2Creator {},
                channel3: Channel3Creator {},
            }
        }
    }

    pub struct Channel0Creator<const CHANNEL: u8> {}

    pub struct Channel1Creator<const CHANNEL: u8> {}

    pub struct Channel2Creator<const CHANNEL: u8> {}

    pub struct Channel3Creator<const CHANNEL: u8> {}

    impl_tx_channel_creator!(0);
    impl_tx_channel_creator!(1);
    impl_tx_channel_creator!(2);
    impl_tx_channel_creator!(3);

    impl_rx_channel_creator!(0);
    impl_rx_channel_creator!(1);
    impl_rx_channel_creator!(2);
    impl_rx_channel_creator!(3);

    super::chip_specific::impl_tx_channel!(Channel0, RMT_SIG_0, 0);
    super::chip_specific::impl_tx_channel!(Channel1, RMT_SIG_1, 1);
    super::chip_specific::impl_tx_channel!(Channel2, RMT_SIG_2, 2);
    super::chip_specific::impl_tx_channel!(Channel3, RMT_SIG_3, 3);

    super::chip_specific::impl_rx_channel!(Channel0, RMT_SIG_0, 0);
    super::chip_specific::impl_rx_channel!(Channel1, RMT_SIG_1, 1);
    super::chip_specific::impl_rx_channel!(Channel2, RMT_SIG_2, 2);
    super::chip_specific::impl_rx_channel!(Channel3, RMT_SIG_3, 3);
}

#[cfg(any(esp32s3))]
mod impl_for_chip {
    use super::private::CreateInstance;
    use crate::peripheral::{Peripheral, PeripheralRef};

    /// RMT Instance
    pub struct Rmt<'d> {
        _peripheral: PeripheralRef<'d, crate::peripherals::RMT>,
        pub channel0: Channel0Creator<0>,
        pub channel1: Channel1Creator<1>,
        pub channel2: Channel2Creator<2>,
        pub channel3: Channel3Creator<3>,
        pub channel4: Channel4Creator<4>,
        pub channel5: Channel5Creator<5>,
        pub channel6: Channel6Creator<6>,
        pub channel7: Channel7Creator<7>,
    }

    impl<'d> CreateInstance<'d> for Rmt<'d> {
        fn create(peripheral: impl Peripheral<P = crate::peripherals::RMT> + 'd) -> Self {
            crate::into_ref!(peripheral);

            Self {
                _peripheral: peripheral,
                channel0: Channel0Creator {},
                channel1: Channel1Creator {},
                channel2: Channel2Creator {},
                channel3: Channel3Creator {},
                channel4: Channel4Creator {},
                channel5: Channel5Creator {},
                channel6: Channel6Creator {},
                channel7: Channel7Creator {},
            }
        }
    }

    pub struct Channel0Creator<const CHANNEL: u8> {}

    pub struct Channel1Creator<const CHANNEL: u8> {}

    pub struct Channel2Creator<const CHANNEL: u8> {}

    pub struct Channel3Creator<const CHANNEL: u8> {}

    pub struct Channel4Creator<const CHANNEL: u8> {}

    pub struct Channel5Creator<const CHANNEL: u8> {}

    pub struct Channel6Creator<const CHANNEL: u8> {}

    pub struct Channel7Creator<const CHANNEL: u8> {}

    impl_tx_channel_creator!(0);
    impl_tx_channel_creator!(1);
    impl_tx_channel_creator!(2);
    impl_tx_channel_creator!(3);

    impl_rx_channel_creator!(4);
    impl_rx_channel_creator!(5);
    impl_rx_channel_creator!(6);
    impl_rx_channel_creator!(7);

    super::chip_specific::impl_tx_channel!(Channel0, RMT_SIG_0, 0);
    super::chip_specific::impl_tx_channel!(Channel1, RMT_SIG_1, 1);
    super::chip_specific::impl_tx_channel!(Channel2, RMT_SIG_2, 2);
    super::chip_specific::impl_tx_channel!(Channel3, RMT_SIG_3, 3);

    super::chip_specific::impl_rx_channel!(Channel4, RMT_SIG_0, 4, 0);
    super::chip_specific::impl_rx_channel!(Channel5, RMT_SIG_1, 5, 1);
    super::chip_specific::impl_rx_channel!(Channel6, RMT_SIG_2, 6, 2);
    super::chip_specific::impl_rx_channel!(Channel7, RMT_SIG_3, 7, 3);
}

/// RMT Channel 0
#[non_exhaustive]
#[derive(Debug)]
pub struct Channel0<const CHANNEL: u8> {}

/// RMT Channel 1
#[non_exhaustive]
#[derive(Debug)]
pub struct Channel1<const CHANNEL: u8> {}

/// RMT Channel 2
#[non_exhaustive]
#[derive(Debug)]
pub struct Channel2<const CHANNEL: u8> {}

/// RMT Channel 3
#[non_exhaustive]
#[derive(Debug)]
pub struct Channel3<const CHANNEL: u8> {}

/// RMT Channel 4
#[cfg(any(esp32, esp32s3))]
#[non_exhaustive]
#[derive(Debug)]
pub struct Channel4<const CHANNEL: u8> {}

/// RMT Channel 5
#[cfg(any(esp32, esp32s3))]
#[non_exhaustive]
#[derive(Debug)]
pub struct Channel5<const CHANNEL: u8> {}

/// RMT Channel 6
#[cfg(any(esp32, esp32s3))]
#[non_exhaustive]
#[derive(Debug)]
pub struct Channel6<const CHANNEL: u8> {}

/// RMT Channel 7
#[cfg(any(esp32, esp32s3))]
#[non_exhaustive]
#[derive(Debug)]
pub struct Channel7<const CHANNEL: u8> {}

pub trait TxChannel<const CHANNEL: u8>: private::TxChannelInternal<CHANNEL> {
    /// Start transmitting the given pulse code sequence.
    /// This returns a [`SingleShotTxTransaction`] which can be used to wait for
    /// the transaction to complete and get back the channel for further
    /// use.
    fn transmit<'a, T: Into<u32> + Copy>(
        self,
        data: &'a [T],
    ) -> SingleShotTxTransaction<'a, Self, T, CHANNEL>
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
    fn transmit_continuously<'a, T: Into<u32> + Copy>(
        self,
        data: &'a [T],
    ) -> Result<ContinuousTxTransaction<Self, CHANNEL>, Error>
    where
        Self: Sized,
    {
        self.transmit_continuously_with_loopcount(0, data)
    }

    /// Like [`Self::transmit_continuously`] but also sets a loop count.
    /// [`ContinuousTxTransaction`] can be used to check if the loop count is
    /// reached.
    fn transmit_continuously_with_loopcount<'a, T: Into<u32> + Copy>(
        self,
        loopcount: u16,
        data: &'a [T],
    ) -> Result<ContinuousTxTransaction<Self, CHANNEL>, Error>
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
pub struct RxTransaction<'a, C, T: From<u32> + Copy, const CHANNEL: u8>
where
    C: RxChannel<CHANNEL>,
{
    channel: C,
    data: &'a mut [T],
}

impl<'a, C, T: From<u32> + Copy, const CHANNEL: u8> RxTransaction<'a, C, T, CHANNEL>
where
    C: RxChannel<CHANNEL>,
{
    /// Wait for the transaction to complete
    pub fn wait(self) -> Result<C, (Error, C)> {
        loop {
            if <C as private::RxChannelInternal<CHANNEL>>::is_error() {
                return Err((Error::TransmissionError, self.channel));
            }

            if <C as private::RxChannelInternal<CHANNEL>>::is_done() {
                break;
            }
        }

        <C as private::RxChannelInternal<CHANNEL>>::stop();
        <C as private::RxChannelInternal<CHANNEL>>::clear_interrupts();
        <C as private::RxChannelInternal<CHANNEL>>::update();

        let ptr = (constants::RMT_RAM_START
            + CHANNEL as usize * constants::RMT_CHANNEL_RAM_SIZE * 4) as *mut u32;
        let len = self.data.len();
        for (idx, entry) in self.data.iter_mut().take(len).enumerate() {
            *entry = unsafe { ptr.add(idx).read_volatile().into() };
        }

        Ok(self.channel)
    }
}

pub trait RxChannel<const CHANNEL: u8>: private::RxChannelInternal<CHANNEL> {
    /// Start receiving pulse codes into the given buffer.
    /// This returns a [RxTransaction] which can be used to wait for receive to
    /// complete and get back the channel for further use.
    /// The length of the received data cannot exceed the allocated RMT RAM.
    fn receive<'a, T: From<u32> + Copy>(
        self,
        data: &'a mut [T],
    ) -> Result<RxTransaction<'a, Self, T, CHANNEL>, Error>
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

#[cfg(feature = "async")]
pub mod asynch {
    use core::{
        marker::PhantomData,
        pin::Pin,
        task::{Context, Poll},
    };

    use embassy_sync::waitqueue::AtomicWaker;
    use procmacros::interrupt;

    use super::{private::Event, *};

    #[cfg(any(esp32, esp32s3))]
    const NUM_CHANNELS: usize = 8;
    #[cfg(not(any(esp32, esp32s3)))]
    const NUM_CHANNELS: usize = 4;

    const INIT: AtomicWaker = AtomicWaker::new();
    static WAKER: [AtomicWaker; NUM_CHANNELS] = [INIT; NUM_CHANNELS];

    pub(crate) struct RmtTxFuture<T, const CHANNEL: u8>
    where
        T: TxChannelAsync<CHANNEL>,
    {
        _phantom: PhantomData<T>,
    }

    impl<T, const CHANNEL: u8> RmtTxFuture<T, CHANNEL>
    where
        T: TxChannelAsync<CHANNEL>,
    {
        pub fn new(_instance: &T) -> Self {
            Self {
                _phantom: PhantomData,
            }
        }
    }

    impl<T, const CHANNEL: u8> core::future::Future for RmtTxFuture<T, CHANNEL>
    where
        T: TxChannelAsync<CHANNEL>,
    {
        type Output = ();

        fn poll(self: Pin<&mut Self>, ctx: &mut Context<'_>) -> Poll<Self::Output> {
            WAKER[CHANNEL as usize].register(ctx.waker());

            if T::is_error() || T::is_done() {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        }
    }

    pub trait TxChannelAsync<const CHANNEL: u8>: private::TxChannelInternal<CHANNEL> {
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

    pub(crate) struct RmtRxFuture<T, const CHANNEL: u8>
    where
        T: RxChannelAsync<CHANNEL>,
    {
        _phantom: PhantomData<T>,
    }

    impl<T, const CHANNEL: u8> RmtRxFuture<T, CHANNEL>
    where
        T: RxChannelAsync<CHANNEL>,
    {
        pub fn new(_instance: &T) -> Self {
            Self {
                _phantom: PhantomData,
            }
        }
    }

    impl<T, const CHANNEL: u8> core::future::Future for RmtRxFuture<T, CHANNEL>
    where
        T: RxChannelAsync<CHANNEL>,
    {
        type Output = ();

        fn poll(self: Pin<&mut Self>, ctx: &mut Context<'_>) -> Poll<Self::Output> {
            WAKER[CHANNEL as usize].register(ctx.waker());
            if T::is_error() || T::is_done() {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        }
    }

    pub trait RxChannelAsync<const CHANNEL: u8>: private::RxChannelInternal<CHANNEL> {
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
                    + CHANNEL as usize * constants::RMT_CHANNEL_RAM_SIZE * 4)
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
    #[interrupt]
    fn RMT() {
        if let Some(channel) = super::chip_specific::pending_interrupt_for_channel() {
            use crate::rmt::private::{RxChannelInternal, TxChannelInternal};

            match channel {
                0 => {
                    super::Channel0::<0>::unlisten_interrupt(Event::End);
                    super::Channel0::<0>::unlisten_interrupt(Event::Error);
                }
                1 => {
                    super::Channel1::<1>::unlisten_interrupt(Event::End);
                    super::Channel1::<1>::unlisten_interrupt(Event::Error);
                }
                2 => {
                    super::Channel2::<2>::unlisten_interrupt(Event::End);
                    super::Channel2::<2>::unlisten_interrupt(Event::Error);
                }
                3 => {
                    super::Channel3::<3>::unlisten_interrupt(Event::End);
                    super::Channel3::<3>::unlisten_interrupt(Event::Error);
                }

                // TODO ... how to handle chips which can use a channel for tx AND rx?
                #[cfg(any(esp32, esp32s3))]
                4 => {
                    super::Channel4::<4>::unlisten_interrupt(Event::End);
                    super::Channel4::<4>::unlisten_interrupt(Event::Error);
                }
                #[cfg(any(esp32, esp32s3))]
                5 => {
                    super::Channel5::<5>::unlisten_interrupt(Event::End);
                    super::Channel5::<5>::unlisten_interrupt(Event::Error);
                }
                #[cfg(any(esp32, esp32s3))]
                6 => {
                    super::Channel6::<6>::unlisten_interrupt(Event::End);
                    super::Channel6::<6>::unlisten_interrupt(Event::Error);
                }
                #[cfg(any(esp32, esp32s3))]
                7 => {
                    super::Channel7::<7>::unlisten_interrupt(Event::End);
                    super::Channel7::<7>::unlisten_interrupt(Event::Error);
                }

                _ => unreachable!(),
            }

            let rmt = unsafe { &*crate::peripherals::RMT::PTR };
            rmt.int_ena.write(|w| unsafe { w.bits(0) });

            WAKER[channel].wake();
        }
    }

    #[cfg(any(esp32, esp32s2))]
    #[interrupt]
    fn RMT() {
        if let Some(channel) = super::chip_specific::pending_interrupt_for_channel() {
            match channel {
                0 => {
                    <Channel0<0> as super::private::TxChannelInternal<0>>::unlisten_interrupt(
                        Event::End,
                    );
                    <Channel0<0> as super::private::TxChannelInternal<0>>::unlisten_interrupt(
                        Event::Error,
                    );
                    <Channel0<0> as super::private::RxChannelInternal<0>>::unlisten_interrupt(
                        Event::End,
                    );
                    <Channel0<0> as super::private::RxChannelInternal<0>>::unlisten_interrupt(
                        Event::Error,
                    );
                }
                1 => {
                    <Channel1<1> as super::private::TxChannelInternal<1>>::unlisten_interrupt(
                        Event::End,
                    );
                    <Channel1<1> as super::private::TxChannelInternal<1>>::unlisten_interrupt(
                        Event::Error,
                    );
                    <Channel1<1> as super::private::RxChannelInternal<1>>::unlisten_interrupt(
                        Event::End,
                    );
                    <Channel1<1> as super::private::RxChannelInternal<1>>::unlisten_interrupt(
                        Event::Error,
                    );
                }
                2 => {
                    <Channel2<2> as super::private::TxChannelInternal<2>>::unlisten_interrupt(
                        Event::End,
                    );
                    <Channel2<2> as super::private::TxChannelInternal<2>>::unlisten_interrupt(
                        Event::Error,
                    );
                    <Channel2<2> as super::private::RxChannelInternal<2>>::unlisten_interrupt(
                        Event::End,
                    );
                    <Channel2<2> as super::private::RxChannelInternal<2>>::unlisten_interrupt(
                        Event::Error,
                    );
                }
                3 => {
                    <Channel3<3> as super::private::TxChannelInternal<3>>::unlisten_interrupt(
                        Event::End,
                    );
                    <Channel3<3> as super::private::TxChannelInternal<3>>::unlisten_interrupt(
                        Event::Error,
                    );
                    <Channel3<3> as super::private::RxChannelInternal<3>>::unlisten_interrupt(
                        Event::End,
                    );
                    <Channel3<3> as super::private::RxChannelInternal<3>>::unlisten_interrupt(
                        Event::Error,
                    );
                }
                #[cfg(any(esp32))]
                4 => {
                    <Channel4<4> as super::private::TxChannelInternal<4>>::unlisten_interrupt(
                        Event::End,
                    );
                    <Channel4<4> as super::private::TxChannelInternal<4>>::unlisten_interrupt(
                        Event::Error,
                    );
                    <Channel4<4> as super::private::RxChannelInternal<4>>::unlisten_interrupt(
                        Event::End,
                    );
                    <Channel4<4> as super::private::RxChannelInternal<4>>::unlisten_interrupt(
                        Event::Error,
                    );
                }
                #[cfg(any(esp32, esp32s3))]
                5 => {
                    <Channel5<5> as super::private::TxChannelInternal<5>>::unlisten_interrupt(
                        Event::End,
                    );
                    <Channel5<5> as super::private::TxChannelInternal<5>>::unlisten_interrupt(
                        Event::Error,
                    );
                    <Channel5<5> as super::private::RxChannelInternal<5>>::unlisten_interrupt(
                        Event::End,
                    );
                    <Channel5<5> as super::private::RxChannelInternal<5>>::unlisten_interrupt(
                        Event::Error,
                    );
                }
                #[cfg(any(esp32, esp32s3))]
                6 => {
                    <Channel6<6> as super::private::TxChannelInternal<6>>::unlisten_interrupt(
                        Event::End,
                    );
                    <Channel6<6> as super::private::TxChannelInternal<6>>::unlisten_interrupt(
                        Event::Error,
                    );
                    <Channel6<6> as super::private::RxChannelInternal<6>>::unlisten_interrupt(
                        Event::End,
                    );
                    <Channel6<6> as super::private::RxChannelInternal<6>>::unlisten_interrupt(
                        Event::Error,
                    );
                }
                #[cfg(any(esp32, esp32s3))]
                7 => {
                    <Channel7<7> as super::private::TxChannelInternal<7>>::unlisten_interrupt(
                        Event::End,
                    );
                    <Channel7<7> as super::private::TxChannelInternal<7>>::unlisten_interrupt(
                        Event::Error,
                    );
                    <Channel7<7> as super::private::RxChannelInternal<7>>::unlisten_interrupt(
                        Event::End,
                    );
                    <Channel7<7> as super::private::RxChannelInternal<7>>::unlisten_interrupt(
                        Event::Error,
                    );
                }

                _ => unreachable!(),
            }

            let rmt = unsafe { &*crate::peripherals::RMT::PTR };
            rmt.int_ena.write(|w| unsafe { w.bits(0) });

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

    pub trait TxChannelInternal<const CHANNEL: u8> {
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
                + CHANNEL as usize * constants::RMT_CHANNEL_RAM_SIZE * 4)
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

    pub trait RxChannelInternal<const CHANNEL: u8> {
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
            rmt.sys_conf.modify(|_, w| {
                w.clk_en()
                    .clear_bit()
                    .sclk_sel()
                    .variant(crate::soc::constants::RMT_CLOCK_SRC)
                    .sclk_div_num()
                    .variant(div as u8)
                    .sclk_div_a()
                    .variant(0)
                    .sclk_div_b()
                    .variant(0)
                    .apb_fifo_mask()
                    .set_bit()
            });
        }

        #[cfg(pcr)]
        {
            let pcr = unsafe { &*crate::peripherals::PCR::PTR };
            pcr.rmt_sclk_conf.modify(|_, w| {
                w.sclk_sel()
                    .variant(crate::soc::constants::RMT_CLOCK_SRC)
                    .sclk_div_num()
                    .variant(div as u8)
                    .sclk_div_a()
                    .variant(0)
                    .sclk_div_b()
                    .variant(0)
            });

            let rmt = unsafe { &*crate::peripherals::RMT::PTR };
            rmt.sys_conf.modify(|_, w| w.apb_fifo_mask().set_bit());
        }
    }

    #[allow(unused)]
    #[cfg(not(esp32s3))]
    pub fn pending_interrupt_for_channel() -> Option<usize> {
        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
        let st = rmt.int_st.read();

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
        let st = rmt.int_st.read();

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
                impl<const CHANNEL: u8> $crate::rmt::private::TxChannelInternal<CHANNEL> for $crate::rmt:: $channel <CHANNEL> {
                    fn new() -> Self {
                        Self {}
                    }

                    fn output_signal() -> crate::gpio::OutputSignal {
                        crate::gpio::OutputSignal::$signal
                    }

                    fn set_divider(divider: u8) {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        rmt.ch_tx_conf0[$ch_num].modify(|_, w| w.div_cnt().variant(divider));
                    }

                    fn update() {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        rmt.ch_tx_conf0[$ch_num].modify(|_, w| w.conf_update().set_bit());
                    }

                    fn set_generate_repeat_interrupt(repeats: u16) {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        if repeats > 1 {
                            rmt.ch_tx_lim[$ch_num].modify(|_, w| {
                                w.loop_count_reset()
                                    .set_bit()
                                    .tx_loop_cnt_en()
                                    .set_bit()
                                    .tx_loop_num()
                                    .variant(repeats)
                            });
                        } else {
                            rmt.ch_tx_lim[$ch_num].modify(|_, w| {
                                w.loop_count_reset()
                                    .set_bit()
                                    .tx_loop_cnt_en()
                                    .clear_bit()
                                    .tx_loop_num()
                                    .variant(0)
                            });
                        }

                        rmt.ch_tx_lim[$ch_num].modify(|_, w| w.loop_count_reset().clear_bit());
                    }

                    fn clear_interrupts() {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };

                        rmt.int_clr.write(|w| {
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

                        rmt.ch_tx_conf0[$ch_num].modify(|_, w| w.tx_conti_mode().bit(continuous));
                    }

                    fn set_wrap_mode(wrap: bool) {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };

                        rmt.ch_tx_conf0[$ch_num].modify(|_, w| w.mem_tx_wrap_en().bit(wrap));
                    }

                    fn set_carrier(carrier: bool, high: u16, low: u16, level: bool) {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };

                        rmt.chcarrier_duty[$ch_num]
                            .write(|w| w.carrier_high().variant(high).carrier_low().variant(low));

                        rmt.ch_tx_conf0[$ch_num].modify(|_, w| {
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
                        rmt.ch_tx_conf0[$ch_num].modify(|_, w| w.idle_out_en().bit(enable).idle_out_lv().bit(level));
                    }

                    fn set_memsize(memsize: u8) {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };

                        rmt.ch_tx_conf0[$ch_num].modify(|_, w| w.mem_size().variant(memsize));
                    }

                    fn start_tx() {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };

                        rmt.ref_cnt_rst.write(|w| unsafe { w.bits(1 << CHANNEL) });
                        Self::update();

                        rmt.ch_tx_conf0[$ch_num].modify(|_, w| {
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
                        rmt.int_raw.read().[< ch $ch_num _tx_end >]().bit()
                    }

                    fn is_error() -> bool {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        rmt.int_raw.read().[< ch $ch_num _tx_err >]().bit()
                    }

                    fn is_threshold_set() -> bool {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        rmt.int_raw.read().[< ch $ch_num _tx_thr_event >]().bit()
                    }

                    fn reset_threshold_set() {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        rmt.int_clr
                            .write(|w| w.[< ch $ch_num _tx_thr_event >]().set_bit());
                    }

                    fn set_threshold(threshold: u8) {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        rmt.ch_tx_lim[$ch_num].modify(|_, w| w.tx_lim().variant(threshold as u16));
                    }

                    fn is_loopcount_interrupt_set() -> bool {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        rmt.int_raw.read().[< ch $ch_num _tx_loop >]().bit()
                    }

                    fn stop() {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        rmt.ch_tx_conf0[$ch_num].modify(|_, w| w.tx_stop().set_bit());
                        Self::update();
                    }

                    fn listen_interrupt(event: $crate::rmt::private::Event) {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        match event {
                            $crate::rmt::private::Event::Error => {
                                rmt.int_ena.modify(|_,w| w.[< ch $ch_num _tx_err >]().set_bit());
                            }
                            $crate::rmt::private::Event::End => {
                                rmt.int_ena.modify(|_,w| w.[< ch $ch_num _tx_end >]().set_bit());
                            }
                            $crate::rmt::private::Event::Threshold => {
                                rmt.int_ena.modify(|_,w| w.[< ch $ch_num _tx_thr_event >]().set_bit());
                            }
                        }
                    }

                    fn unlisten_interrupt(event: $crate::rmt::private::Event) {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        match event {
                            $crate::rmt::private::Event::Error => {
                                rmt.int_ena.modify(|_,w| w.[< ch $ch_num _tx_err >]().clear_bit());
                            }
                            $crate::rmt::private::Event::End => {
                                rmt.int_ena.modify(|_,w| w.[< ch $ch_num _tx_end >]().clear_bit());
                            }
                            $crate::rmt::private::Event::Threshold => {
                                rmt.int_ena.modify(|_,w| w.[< ch $ch_num _tx_thr_event >]().clear_bit());
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
                impl<const CHANNEL: u8> $crate::rmt::private::RxChannelInternal<CHANNEL> for $crate::rmt:: $channel <CHANNEL> {
                    fn new() -> Self {
                        Self {}
                    }

                    fn input_signal() -> crate::gpio::InputSignal {
                        crate::gpio::InputSignal::$signal
                    }

                    fn set_divider(divider: u8) {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        rmt.[< ch $ch_num _rx_conf0 >].modify(|_, w| w.div_cnt().variant(divider));
                    }

                    fn update() {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        rmt.[< ch $ch_num _rx_conf1 >].modify(|_, w| w.conf_update().set_bit());
                    }

                    fn clear_interrupts() {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };

                        rmt.int_clr.write(|w| {
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
                        rmt.[< ch $ch_num _rx_conf1 >].modify(|_, w| w.mem_rx_wrap_en().bit(wrap));
                    }

                    fn set_carrier(carrier: bool, high: u16, low: u16, level: bool) {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };

                        rmt.ch_rx_carrier_rm[$ch_index].write(|w| {
                            w.carrier_high_thres()
                                .variant(high)
                                .carrier_low_thres()
                                .variant(low)
                        });

                        rmt.[< ch $ch_num _rx_conf0 >]
                            .modify(|_, w| w.carrier_en().bit(carrier).carrier_out_lv().bit(level));
                    }

                    fn set_memsize(memsize: u8) {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        rmt.[< ch $ch_num _rx_conf0 >].modify(|_, w| w.mem_size().variant(memsize));
                    }

                    fn start_rx() {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        rmt.[< ch $ch_num _rx_conf1 >].modify(|_, w| {
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
                        rmt.int_raw.read().[< ch $ch_num _rx_end >]().bit()
                    }

                    fn is_error() -> bool {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        rmt.int_raw.read().[< ch $ch_num _rx_err >]().bit()
                    }

                    fn stop() {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        rmt.[< ch $ch_num _rx_conf1 >].modify(|_, w| w.rx_en().clear_bit());
                    }

                    fn set_filter_threshold(value: u8) {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };

                        rmt.[< ch $ch_num _rx_conf1 >].modify(|_, w| {
                            w.rx_filter_en()
                                .bit(value > 0)
                                .rx_filter_thres()
                                .variant(value)
                        });
                    }

                    fn set_idle_threshold(value: u16) {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };

                        rmt.[< ch $ch_num _rx_conf0 >].modify(|_, w| w.idle_thres().variant(value));
                    }

                    fn listen_interrupt(event: $crate::rmt::private::Event) {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        match event {
                            $crate::rmt::private::Event::Error => {
                                rmt.int_ena.modify(|_,w| w.[< ch $ch_num _rx_err >]().set_bit());
                            }
                            $crate::rmt::private::Event::End => {
                                rmt.int_ena.modify(|_,w| w.[< ch $ch_num _rx_end >]().set_bit());
                            }
                            $crate::rmt::private::Event::Threshold => {
                                rmt.int_ena.modify(|_,w| w.[< ch $ch_num _rx_thr_event >]().set_bit());
                            }
                        }
                    }

                    fn unlisten_interrupt(event: $crate::rmt::private::Event) {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        match event {
                            $crate::rmt::private::Event::Error => {
                                rmt.int_ena.modify(|_,w| w.[< ch $ch_num _rx_err >]().clear_bit());
                            }
                            $crate::rmt::private::Event::End => {
                                rmt.int_ena.modify(|_,w| w.[< ch $ch_num _rx_end >]().clear_bit());
                            }
                            $crate::rmt::private::Event::Threshold => {
                                rmt.int_ena.modify(|_,w| w.[< ch $ch_num _rx_thr_event >]().clear_bit());
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

        rmt.ch0conf1.modify(|_, w| w.ref_always_on().set_bit());
        rmt.ch1conf1.modify(|_, w| w.ref_always_on().set_bit());
        rmt.ch2conf1.modify(|_, w| w.ref_always_on().set_bit());
        rmt.ch3conf1.modify(|_, w| w.ref_always_on().set_bit());
        #[cfg(esp32)]
        {
            rmt.ch4conf1.modify(|_, w| w.ref_always_on().set_bit());
            rmt.ch5conf1.modify(|_, w| w.ref_always_on().set_bit());
            rmt.ch6conf1.modify(|_, w| w.ref_always_on().set_bit());
            rmt.ch7conf1.modify(|_, w| w.ref_always_on().set_bit());
        }

        rmt.apb_conf.modify(|_, w| w.apb_fifo_mask().set_bit());

        #[cfg(not(esp32))]
        rmt.apb_conf.modify(|_, w| w.clk_en().set_bit());
    }

    #[allow(unused)]
    #[cfg(esp32)]
    pub fn pending_interrupt_for_channel() -> Option<usize> {
        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
        let st = rmt.int_st.read();

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
        let st = rmt.int_st.read();

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
                impl<const CHANNEL: u8> super::private::TxChannelInternal<CHANNEL> for super::$channel<CHANNEL> {
                    fn new() -> Self {
                        Self {}
                    }

                    fn output_signal() -> crate::gpio::OutputSignal {
                        crate::gpio::OutputSignal::$signal
                    }

                    fn set_divider(divider: u8) {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        rmt.[< ch $ch_num conf0 >].modify(|_, w| w.div_cnt().variant(divider));
                    }

                    fn update() {
                        // no-op
                    }

                    #[cfg(not(esp32))]
                    fn set_generate_repeat_interrupt(repeats: u16) {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        if repeats > 1 {
                            rmt.ch_tx_lim[$ch_num].modify(|_, w| w.tx_loop_num().variant(repeats));
                        } else {
                            rmt.ch_tx_lim[$ch_num].modify(|_, w| w.tx_loop_num().variant(0));
                        }
                    }

                    #[cfg(esp32)]
                    fn set_generate_repeat_interrupt(_repeats: u16) {
                        // unsupported
                    }

                    fn clear_interrupts() {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };

                        rmt.int_clr.write(|w| {
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

                        rmt.[< ch $ch_num conf1 >].modify(|_, w| w.tx_conti_mode().bit(continuous));
                    }

                    fn set_wrap_mode(wrap: bool) {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        // this is "okay", because we use all TX channels always in wrap mode
                        rmt.apb_conf.modify(|_, w| w.mem_tx_wrap_en().bit(wrap));
                    }

                    fn set_carrier(carrier: bool, high: u16, low: u16, level: bool) {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };

                        rmt.chcarrier_duty[$ch_num]
                            .write(|w| w.carrier_high().variant(high).carrier_low().variant(low));

                        rmt.[< ch $ch_num conf0 >]
                            .modify(|_, w| w.carrier_en().bit(carrier).carrier_out_lv().bit(level));
                    }

                    fn set_idle_output(enable: bool, level: bool) {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        rmt.[< ch $ch_num conf1 >]
                            .modify(|_, w| w.idle_out_en().bit(enable).idle_out_lv().bit(level));
                    }

                    fn set_memsize(memsize: u8) {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };

                        rmt.[< ch $ch_num conf0 >].modify(|_, w| w.mem_size().variant(memsize));
                    }

                    fn start_tx() {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };

                        rmt.[< ch $ch_num conf1 >].modify(|_, w| {
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
                        rmt.int_raw.read().[< ch $ch_num _tx_end >]().bit()
                    }

                    fn is_error() -> bool {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        rmt.int_raw.read().[< ch $ch_num _err >]().bit()
                    }

                    fn is_threshold_set() -> bool {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        rmt.int_raw.read().[< ch $ch_num _tx_thr_event >]().bit()
                    }

                    fn reset_threshold_set() {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        rmt.int_clr
                            .write(|w| w.[< ch $ch_num _tx_thr_event >]().set_bit());
                    }

                    fn set_threshold(threshold: u8) {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        rmt.ch_tx_lim[$ch_num].modify(|_, w| w.tx_lim().variant(threshold as u16));
                    }

                    fn is_loopcount_interrupt_set() -> bool {
                        // no-op
                        false
                    }

                    fn stop() {
                        #[cfg(esp32s2)]
                        {
                            let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                            rmt.[< ch $ch_num conf1 >].modify(|_, w| w.tx_stop().set_bit());
                        }
                    }

                    fn listen_interrupt(event: $crate::rmt::private::Event) {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        match event {
                            $crate::rmt::private::Event::Error => {
                                rmt.int_ena.modify(|_,w| w.[< ch $ch_num _err >]().set_bit());
                            }
                            $crate::rmt::private::Event::End => {
                                rmt.int_ena.modify(|_,w| w.[< ch $ch_num _tx_end >]().set_bit());
                            }
                            $crate::rmt::private::Event::Threshold => {
                                rmt.int_ena.modify(|_,w| w.[< ch $ch_num _tx_thr_event >]().set_bit());
                            }
                        }
                    }

                    fn unlisten_interrupt(event: $crate::rmt::private::Event) {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        match event {
                            $crate::rmt::private::Event::Error => {
                                rmt.int_ena.modify(|_,w| w.[< ch $ch_num _err >]().clear_bit());
                            }
                            $crate::rmt::private::Event::End => {
                                rmt.int_ena.modify(|_,w| w.[< ch $ch_num _tx_end >]().clear_bit());
                            }
                            $crate::rmt::private::Event::Threshold => {
                                rmt.int_ena.modify(|_,w| w.[< ch $ch_num _tx_thr_event >]().clear_bit());
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
                impl<const CHANNEL: u8> super::private::RxChannelInternal<CHANNEL> for super::$channel<CHANNEL> {
                    fn new() -> Self {
                        Self {}
                    }

                    fn input_signal() -> crate::gpio::InputSignal {
                        crate::gpio::InputSignal::$signal
                    }

                    fn set_divider(divider: u8) {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        rmt.[< ch $ch_num conf0 >].modify(|_, w| w.div_cnt().variant(divider));
                    }

                    fn update() {
                        // no-op
                    }

                    fn clear_interrupts() {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };

                        rmt.[< ch $ch_num conf1 >].modify(|_, w| {
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

                        rmt.int_clr.write(|w| {
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

                        rmt.chcarrier_duty[$ch_num]
                            .write(|w| w.carrier_high().variant(high).carrier_low().variant(low));

                        rmt.[< ch $ch_num conf0 >]
                            .modify(|_, w| w.carrier_en().bit(carrier).carrier_out_lv().bit(level));
                    }

                    fn set_memsize(memsize: u8) {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };

                        rmt.[< ch $ch_num conf0 >].modify(|_, w| w.mem_size().variant(memsize));
                    }

                    fn start_rx() {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };

                        rmt.[< ch $ch_num conf1 >].modify(|_, w| {
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
                        rmt.int_raw.read().[< ch $ch_num _rx_end >]().bit()
                    }

                    fn is_error() -> bool {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        rmt.int_raw.read().[< ch $ch_num _err >]().bit()
                    }

                    fn stop() {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        rmt.[< ch $ch_num conf1 >].modify(|_, w| w.rx_en().clear_bit());
                    }

                    fn set_filter_threshold(value: u8) {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        rmt.[< ch $ch_num conf1 >].modify(|_, w| {
                            w.rx_filter_en()
                                .bit(value > 0)
                                .rx_filter_thres()
                                .variant(value)
                        });
                    }

                    fn set_idle_threshold(value: u16) {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };

                        rmt.[< ch $ch_num conf0 >].modify(|_, w| w.idle_thres().variant(value));
                    }

                    fn listen_interrupt(event: $crate::rmt::private::Event) {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        match event {
                            $crate::rmt::private::Event::Error => {
                                rmt.int_ena.modify(|_,w| w.[< ch $ch_num _err >]().set_bit());
                            }
                            $crate::rmt::private::Event::End => {
                                rmt.int_ena.modify(|_,w| w.[< ch $ch_num _rx_end >]().set_bit());
                            }
                            $crate::rmt::private::Event::Threshold => {
                                rmt.int_ena.modify(|_,w| w.[< ch $ch_num _tx_thr_event >]().set_bit());
                            }
                        }
                    }

                    fn unlisten_interrupt(event: $crate::rmt::private::Event) {
                        let rmt = unsafe { &*crate::peripherals::RMT::PTR };
                        match event {
                            $crate::rmt::private::Event::Error => {
                                rmt.int_ena.modify(|_,w| w.[< ch $ch_num _err >]().clear_bit());
                            }
                            $crate::rmt::private::Event::End => {
                                rmt.int_ena.modify(|_,w| w.[< ch $ch_num _rx_end >]().clear_bit());
                            }
                            $crate::rmt::private::Event::Threshold => {
                                rmt.int_ena.modify(|_,w| w.[< ch $ch_num _tx_thr_event >]().clear_bit());
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
