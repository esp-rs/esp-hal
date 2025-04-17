//! # Serial Peripheral Interface - Master Mode
//!
//! ## Overview
//!
//! In this mode, the SPI acts as master and initiates the SPI transactions.
//!
//! ## Configuration
//!
//! The peripheral can be used in full-duplex and half-duplex mode and can
//! leverage DMA for data transfers. It can also be used in blocking or async.
//!
//! ### Exclusive access to the SPI bus
//!
//! If all you want to do is to communicate to a single device, and you initiate
//! transactions yourself, there are a number of ways to achieve this:
//!
//! - Use the [`SpiBus`](embedded_hal::spi::SpiBus) trait and its associated
//!   functions to initiate transactions with simultaneous reads and writes, or
//! - Use the `ExclusiveDevice` struct from [`embedded-hal-bus`] or `SpiDevice`
//!   from [`embassy-embedded-hal`].
//!
//! ### Shared SPI access
//!
//! If you have multiple devices on the same SPI bus that each have their own CS
//! line (and optionally, configuration), you may want to have a look at the
//! implementations provided by [`embedded-hal-bus`] and
//! [`embassy-embedded-hal`].
//!
//! ## Usage
//!
//! The module implements several third-party traits from embedded-hal@1.x.x
//! and [`embassy-embedded-hal`].
//!
//! [`embedded-hal-bus`]: https://docs.rs/embedded-hal-bus/latest/embedded_hal_bus/spi/index.html
//! [`embassy-embedded-hal`]: embassy_embedded_hal::shared_bus

#[cfg(esp32)]
use core::cell::Cell;
use core::marker::PhantomData;

#[instability::unstable]
pub use dma::*;
use enumset::{EnumSet, EnumSetType};
#[cfg(place_spi_driver_in_ram)]
use procmacros::ram;

use super::{BitOrder, DataMode, DmaError, Error, Mode};
use crate::{
    Async,
    Blocking,
    DriverMode,
    asynch::AtomicWaker,
    clock::Clocks,
    dma::{DmaChannelFor, DmaEligible, DmaRxBuffer, DmaTxBuffer},
    gpio::{
        InputSignal,
        NoPin,
        OutputSignal,
        PinGuard,
        interconnect::{PeripheralInput, PeripheralOutput},
    },
    interrupt::InterruptHandler,
    pac::spi2::RegisterBlock,
    private::{self, OnDrop, Sealed},
    spi::AnySpi,
    system::{Cpu, PeripheralGuard},
    time::Rate,
};

/// Enumeration of possible SPI interrupt events.
#[derive(Debug, Hash, EnumSetType)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
#[instability::unstable]
pub enum SpiInterrupt {
    /// Indicates that the SPI transaction has completed successfully.
    ///
    /// This interrupt is triggered when an SPI transaction has finished
    /// transmitting and receiving data.
    TransferDone,

    /// Triggered at the end of configurable segmented transfer.
    #[cfg(any(esp32s2, gdma))]
    DmaSegmentedTransferDone,

    /// Used and triggered by software. Only used for user defined function.
    #[cfg(gdma)]
    App2,

    /// Used and triggered by software. Only used for user defined function.
    #[cfg(gdma)]
    App1,
}

/// The size of the FIFO buffer for SPI
const FIFO_SIZE: usize = if cfg!(esp32s2) { 72 } else { 64 };

/// Padding byte for empty write transfers
const EMPTY_WRITE_PAD: u8 = 0x00;

const MAX_DMA_SIZE: usize = 32736;

/// SPI commands, each consisting of a 16-bit command value and a data mode.
///
/// Used to define specific commands sent over the SPI bus.
/// Can be [Command::None] if command phase should be suppressed.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub enum Command {
    /// No command is sent.
    None,
    /// A 1-bit command.
    _1Bit(u16, DataMode),
    /// A 2-bit command.
    _2Bit(u16, DataMode),
    /// A 3-bit command.
    _3Bit(u16, DataMode),
    /// A 4-bit command.
    _4Bit(u16, DataMode),
    /// A 5-bit command.
    _5Bit(u16, DataMode),
    /// A 6-bit command.
    _6Bit(u16, DataMode),
    /// A 7-bit command.
    _7Bit(u16, DataMode),
    /// A 8-bit command.
    _8Bit(u16, DataMode),
    /// A 9-bit command.
    _9Bit(u16, DataMode),
    /// A 10-bit command.
    _10Bit(u16, DataMode),
    /// A 11-bit command.
    _11Bit(u16, DataMode),
    /// A 12-bit command.
    _12Bit(u16, DataMode),
    /// A 13-bit command.
    _13Bit(u16, DataMode),
    /// A 14-bit command.
    _14Bit(u16, DataMode),
    /// A 15-bit command.
    _15Bit(u16, DataMode),
    /// A 16-bit command.
    _16Bit(u16, DataMode),
}

impl Command {
    fn width(&self) -> usize {
        match self {
            Command::None => 0,
            Command::_1Bit(_, _) => 1,
            Command::_2Bit(_, _) => 2,
            Command::_3Bit(_, _) => 3,
            Command::_4Bit(_, _) => 4,
            Command::_5Bit(_, _) => 5,
            Command::_6Bit(_, _) => 6,
            Command::_7Bit(_, _) => 7,
            Command::_8Bit(_, _) => 8,
            Command::_9Bit(_, _) => 9,
            Command::_10Bit(_, _) => 10,
            Command::_11Bit(_, _) => 11,
            Command::_12Bit(_, _) => 12,
            Command::_13Bit(_, _) => 13,
            Command::_14Bit(_, _) => 14,
            Command::_15Bit(_, _) => 15,
            Command::_16Bit(_, _) => 16,
        }
    }

    fn value(&self) -> u16 {
        match self {
            Command::None => 0,
            Command::_1Bit(value, _)
            | Command::_2Bit(value, _)
            | Command::_3Bit(value, _)
            | Command::_4Bit(value, _)
            | Command::_5Bit(value, _)
            | Command::_6Bit(value, _)
            | Command::_7Bit(value, _)
            | Command::_8Bit(value, _)
            | Command::_9Bit(value, _)
            | Command::_10Bit(value, _)
            | Command::_11Bit(value, _)
            | Command::_12Bit(value, _)
            | Command::_13Bit(value, _)
            | Command::_14Bit(value, _)
            | Command::_15Bit(value, _)
            | Command::_16Bit(value, _) => *value,
        }
    }

    fn mode(&self) -> DataMode {
        match self {
            Command::None => DataMode::SingleTwoDataLines,
            Command::_1Bit(_, mode)
            | Command::_2Bit(_, mode)
            | Command::_3Bit(_, mode)
            | Command::_4Bit(_, mode)
            | Command::_5Bit(_, mode)
            | Command::_6Bit(_, mode)
            | Command::_7Bit(_, mode)
            | Command::_8Bit(_, mode)
            | Command::_9Bit(_, mode)
            | Command::_10Bit(_, mode)
            | Command::_11Bit(_, mode)
            | Command::_12Bit(_, mode)
            | Command::_13Bit(_, mode)
            | Command::_14Bit(_, mode)
            | Command::_15Bit(_, mode)
            | Command::_16Bit(_, mode) => *mode,
        }
    }

    fn is_none(&self) -> bool {
        matches!(self, Command::None)
    }
}

/// SPI address, ranging from 1 to 32 bits, paired with a data mode.
///
/// This can be used to specify the address phase of SPI transactions.
/// Can be [Address::None] if address phase should be suppressed.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub enum Address {
    /// No address phase.
    None,
    /// A 1-bit address.
    _1Bit(u32, DataMode),
    /// A 2-bit address.
    _2Bit(u32, DataMode),
    /// A 3-bit address.
    _3Bit(u32, DataMode),
    /// A 4-bit address.
    _4Bit(u32, DataMode),
    /// A 5-bit address.
    _5Bit(u32, DataMode),
    /// A 6-bit address.
    _6Bit(u32, DataMode),
    /// A 7-bit address.
    _7Bit(u32, DataMode),
    /// A 8-bit address.
    _8Bit(u32, DataMode),
    /// A 9-bit address.
    _9Bit(u32, DataMode),
    /// A 10-bit address.
    _10Bit(u32, DataMode),
    /// A 11-bit address.
    _11Bit(u32, DataMode),
    /// A 12-bit address.
    _12Bit(u32, DataMode),
    /// A 13-bit address.
    _13Bit(u32, DataMode),
    /// A 14-bit address.
    _14Bit(u32, DataMode),
    /// A 15-bit address.
    _15Bit(u32, DataMode),
    /// A 16-bit address.
    _16Bit(u32, DataMode),
    /// A 17-bit address.
    _17Bit(u32, DataMode),
    /// A 18-bit address.
    _18Bit(u32, DataMode),
    /// A 19-bit address.
    _19Bit(u32, DataMode),
    /// A 20-bit address.
    _20Bit(u32, DataMode),
    /// A 21-bit address.
    _21Bit(u32, DataMode),
    /// A 22-bit address.
    _22Bit(u32, DataMode),
    /// A 23-bit address.
    _23Bit(u32, DataMode),
    /// A 24-bit address.
    _24Bit(u32, DataMode),
    /// A 25-bit address.
    _25Bit(u32, DataMode),
    /// A 26-bit address.
    _26Bit(u32, DataMode),
    /// A 27-bit address.
    _27Bit(u32, DataMode),
    /// A 28-bit address.
    _28Bit(u32, DataMode),
    /// A 29-bit address.
    _29Bit(u32, DataMode),
    /// A 30-bit address.
    _30Bit(u32, DataMode),
    /// A 31-bit address.
    _31Bit(u32, DataMode),
    /// A 32-bit address.
    _32Bit(u32, DataMode),
}

impl Address {
    fn width(&self) -> usize {
        match self {
            Address::None => 0,
            Address::_1Bit(_, _) => 1,
            Address::_2Bit(_, _) => 2,
            Address::_3Bit(_, _) => 3,
            Address::_4Bit(_, _) => 4,
            Address::_5Bit(_, _) => 5,
            Address::_6Bit(_, _) => 6,
            Address::_7Bit(_, _) => 7,
            Address::_8Bit(_, _) => 8,
            Address::_9Bit(_, _) => 9,
            Address::_10Bit(_, _) => 10,
            Address::_11Bit(_, _) => 11,
            Address::_12Bit(_, _) => 12,
            Address::_13Bit(_, _) => 13,
            Address::_14Bit(_, _) => 14,
            Address::_15Bit(_, _) => 15,
            Address::_16Bit(_, _) => 16,
            Address::_17Bit(_, _) => 17,
            Address::_18Bit(_, _) => 18,
            Address::_19Bit(_, _) => 19,
            Address::_20Bit(_, _) => 20,
            Address::_21Bit(_, _) => 21,
            Address::_22Bit(_, _) => 22,
            Address::_23Bit(_, _) => 23,
            Address::_24Bit(_, _) => 24,
            Address::_25Bit(_, _) => 25,
            Address::_26Bit(_, _) => 26,
            Address::_27Bit(_, _) => 27,
            Address::_28Bit(_, _) => 28,
            Address::_29Bit(_, _) => 29,
            Address::_30Bit(_, _) => 30,
            Address::_31Bit(_, _) => 31,
            Address::_32Bit(_, _) => 32,
        }
    }

    fn value(&self) -> u32 {
        match self {
            Address::None => 0,
            Address::_1Bit(value, _)
            | Address::_2Bit(value, _)
            | Address::_3Bit(value, _)
            | Address::_4Bit(value, _)
            | Address::_5Bit(value, _)
            | Address::_6Bit(value, _)
            | Address::_7Bit(value, _)
            | Address::_8Bit(value, _)
            | Address::_9Bit(value, _)
            | Address::_10Bit(value, _)
            | Address::_11Bit(value, _)
            | Address::_12Bit(value, _)
            | Address::_13Bit(value, _)
            | Address::_14Bit(value, _)
            | Address::_15Bit(value, _)
            | Address::_16Bit(value, _)
            | Address::_17Bit(value, _)
            | Address::_18Bit(value, _)
            | Address::_19Bit(value, _)
            | Address::_20Bit(value, _)
            | Address::_21Bit(value, _)
            | Address::_22Bit(value, _)
            | Address::_23Bit(value, _)
            | Address::_24Bit(value, _)
            | Address::_25Bit(value, _)
            | Address::_26Bit(value, _)
            | Address::_27Bit(value, _)
            | Address::_28Bit(value, _)
            | Address::_29Bit(value, _)
            | Address::_30Bit(value, _)
            | Address::_31Bit(value, _)
            | Address::_32Bit(value, _) => *value,
        }
    }

    fn is_none(&self) -> bool {
        matches!(self, Address::None)
    }

    fn mode(&self) -> DataMode {
        match self {
            Address::None => DataMode::SingleTwoDataLines,
            Address::_1Bit(_, mode)
            | Address::_2Bit(_, mode)
            | Address::_3Bit(_, mode)
            | Address::_4Bit(_, mode)
            | Address::_5Bit(_, mode)
            | Address::_6Bit(_, mode)
            | Address::_7Bit(_, mode)
            | Address::_8Bit(_, mode)
            | Address::_9Bit(_, mode)
            | Address::_10Bit(_, mode)
            | Address::_11Bit(_, mode)
            | Address::_12Bit(_, mode)
            | Address::_13Bit(_, mode)
            | Address::_14Bit(_, mode)
            | Address::_15Bit(_, mode)
            | Address::_16Bit(_, mode)
            | Address::_17Bit(_, mode)
            | Address::_18Bit(_, mode)
            | Address::_19Bit(_, mode)
            | Address::_20Bit(_, mode)
            | Address::_21Bit(_, mode)
            | Address::_22Bit(_, mode)
            | Address::_23Bit(_, mode)
            | Address::_24Bit(_, mode)
            | Address::_25Bit(_, mode)
            | Address::_26Bit(_, mode)
            | Address::_27Bit(_, mode)
            | Address::_28Bit(_, mode)
            | Address::_29Bit(_, mode)
            | Address::_30Bit(_, mode)
            | Address::_31Bit(_, mode)
            | Address::_32Bit(_, mode) => *mode,
        }
    }
}

/// SPI clock source.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub enum ClockSource {
    /// Use the APB clock.
    Apb,
    // #[cfg(any(esp32c2, esp32c3, esp32s3))]
    // Xtal,
}

/// SPI peripheral configuration
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, procmacros::BuilderLite)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct Config {
    /// The precomputed clock configuration register value.
    ///
    /// Clock divider calculations are relatively expensive, and the SPI
    /// peripheral is commonly expected to be used in a shared bus
    /// configuration, where different devices may need different bus clock
    /// frequencies. To reduce the time required to reconfigure the bus, we
    /// cache clock register's value here, for each configuration.
    ///
    /// This field is not intended to be set by the user, and is only used
    /// internally.
    #[builder_lite(skip)]
    reg: Result<u32, ConfigError>,

    /// The target frequency
    #[builder_lite(skip_setter)]
    frequency: Rate,

    /// The clock source
    #[builder_lite(unstable)]
    #[builder_lite(skip_setter)]
    clock_source: ClockSource,

    /// SPI sample/shift mode.
    mode: Mode,

    /// Bit order of the read data.
    read_bit_order: BitOrder,

    /// Bit order of the written data.
    write_bit_order: BitOrder,
}

impl Default for Config {
    fn default() -> Self {
        let mut this = Config {
            reg: Ok(0),
            frequency: Rate::from_mhz(1),
            clock_source: ClockSource::Apb,
            mode: Mode::_0,
            read_bit_order: BitOrder::MsbFirst,
            write_bit_order: BitOrder::MsbFirst,
        };

        this.reg = this.recalculate();

        this
    }
}

impl Config {
    /// Set the frequency of the SPI bus clock.
    pub fn with_frequency(mut self, frequency: Rate) -> Self {
        self.frequency = frequency;
        self.reg = self.recalculate();

        self
    }

    /// Set the clock source of the SPI bus.
    #[instability::unstable]
    pub fn with_clock_source(mut self, clock_source: ClockSource) -> Self {
        self.clock_source = clock_source;
        self.reg = self.recalculate();

        self
    }

    fn clock_source_freq_hz(&self) -> Rate {
        let clocks = Clocks::get();

        // FIXME: take clock source into account
        cfg_if::cfg_if! {
            if #[cfg(esp32h2)] {
                // ESP32-H2 is using PLL_48M_CLK source instead of APB_CLK
                clocks.pll_48m_clock
            } else {
                clocks.apb_clock
            }
        }
    }

    fn recalculate(&self) -> Result<u32, ConfigError> {
        // taken from https://github.com/apache/incubator-nuttx/blob/8267a7618629838231256edfa666e44b5313348e/arch/risc-v/src/esp32c3/esp32c3_spi.c#L496
        let source_freq = self.clock_source_freq_hz();

        let reg_val: u32;
        let duty_cycle = 128;

        // In HW, n, h and l fields range from 1 to 64, pre ranges from 1 to 8K.
        // The value written to register is one lower than the used value.

        if self.frequency > ((source_freq / 4) * 3) {
            // Using APB frequency directly will give us the best result here.
            reg_val = 1 << 31;
        } else {
            // For best duty cycle resolution, we want n to be as close to 32 as
            // possible, but we also need a pre/n combo that gets us as close as
            // possible to the intended frequency. To do this, we bruteforce n and
            // calculate the best pre to go along with that. If there's a choice
            // between pre/n combos that give the same result, use the one with the
            // higher n.

            let mut pre: i32;
            let mut bestn: i32 = -1;
            let mut bestpre: i32 = -1;
            let mut besterr: i32 = 0;
            let mut errval: i32;

            let target_freq_hz = self.frequency.as_hz() as i32;
            let source_freq_hz = source_freq.as_hz() as i32;

            // Start at n = 2. We need to be able to set h/l so we have at least
            // one high and one low pulse.

            for n in 2..=64 {
                // Effectively, this does:
                // pre = round((APB_CLK_FREQ / n) / frequency)

                pre = ((source_freq_hz / n) + (target_freq_hz / 2)) / target_freq_hz;

                if pre <= 0 {
                    pre = 1;
                }

                if pre > 16 {
                    pre = 16;
                }

                errval = (source_freq_hz / (pre * n) - target_freq_hz).abs();
                if bestn == -1 || errval <= besterr {
                    besterr = errval;
                    bestn = n;
                    bestpre = pre;
                }
            }

            let n: i32 = bestn;
            pre = bestpre;
            let l: i32 = n;

            // Effectively, this does:
            // h = round((duty_cycle * n) / 256)

            let mut h: i32 = (duty_cycle * n + 127) / 256;
            if h <= 0 {
                h = 1;
            }

            reg_val = (l as u32 - 1)
                | ((h as u32 - 1) << 6)
                | ((n as u32 - 1) << 12)
                | ((pre as u32 - 1) << 18);
        }

        Ok(reg_val)
    }

    fn raw_clock_reg_value(&self) -> Result<u32, ConfigError> {
        self.reg
    }

    fn validate(&self) -> Result<(), ConfigError> {
        let source_freq = self.clock_source_freq_hz();
        let min_divider = 1;
        // FIXME: while ESP32 and S2 support pre dividers as large as 8192,
        // those values are not currently supported by the divider calculation.
        let max_divider = 16 * 64; // n * pre

        if self.frequency < source_freq / max_divider || self.frequency > source_freq / min_divider
        {
            return Err(ConfigError::UnsupportedFrequency);
        }

        Ok(())
    }
}

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
struct SpiPinGuard {
    mosi_pin: PinGuard,
    sclk_pin: PinGuard,
    cs_pin: PinGuard,
    sio1_pin: PinGuard,
    sio2_pin: Option<PinGuard>,
    sio3_pin: Option<PinGuard>,
}

/// Configuration errors.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ConfigError {
    /// The requested frequency is not supported.
    UnsupportedFrequency,
}

impl core::error::Error for ConfigError {}

impl core::fmt::Display for ConfigError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            ConfigError::UnsupportedFrequency => {
                write!(f, "The requested frequency is not supported")
            }
        }
    }
}

/// SPI peripheral driver
///
/// ### SPI Initialization
/// ```rust, no_run
#[doc = crate::before_snippet!()]
/// # use esp_hal::spi::Mode;
/// # use esp_hal::spi::master::{Config, Spi};
/// let mut spi = Spi::new(
///     peripherals.SPI2,
///     Config::default()
///         .with_frequency(Rate::from_khz(100))
///         .with_mode(Mode::_0),
/// )?
/// .with_sck(peripherals.GPIO0)
/// .with_mosi(peripherals.GPIO1)
/// .with_miso(peripherals.GPIO2);
/// # Ok(())
/// # }
/// ```
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Spi<'d, Dm: DriverMode> {
    spi: AnySpi<'d>,
    _mode: PhantomData<Dm>,
    guard: PeripheralGuard,
    pins: SpiPinGuard,
}

impl<Dm: DriverMode> Sealed for Spi<'_, Dm> {}

impl<'d> Spi<'d, Blocking> {
    /// Constructs an SPI instance in 8bit dataframe mode.
    ///
    /// # Errors
    ///
    /// See [`Spi::apply_config`].
    pub fn new(spi: impl Instance + 'd, config: Config) -> Result<Self, ConfigError> {
        let guard = PeripheralGuard::new(spi.info().peripheral);

        let mosi_pin = PinGuard::new_unconnected(spi.info().mosi);
        let sclk_pin = PinGuard::new_unconnected(spi.info().sclk);
        let cs_pin = PinGuard::new_unconnected(spi.info().cs[0]);
        let sio1_pin = PinGuard::new_unconnected(spi.info().sio1_output);
        let sio2_pin = spi.info().sio2_output.map(PinGuard::new_unconnected);
        let sio3_pin = spi.info().sio3_output.map(PinGuard::new_unconnected);

        let mut this = Spi {
            spi: spi.degrade(),
            _mode: PhantomData,
            guard,
            pins: SpiPinGuard {
                mosi_pin,
                sclk_pin,
                cs_pin,
                sio1_pin,
                sio2_pin,
                sio3_pin,
            },
        };

        this.driver().init();
        this.apply_config(&config)?;

        let this = this
            .with_sio0(NoPin)
            .with_sio1(NoPin)
            .with_sck(NoPin)
            .with_cs(NoPin);

        let is_qspi = this.driver().info.sio2_input.is_some();
        if is_qspi {
            unwrap!(this.driver().info.sio2_input).connect_to(&NoPin);
            unwrap!(this.driver().info.sio2_output).connect_to(&NoPin);
            unwrap!(this.driver().info.sio3_input).connect_to(&NoPin);
            unwrap!(this.driver().info.sio3_output).connect_to(&NoPin);
        }

        Ok(this)
    }

    /// Converts the SPI instance into async mode.
    pub fn into_async(mut self) -> Spi<'d, Async> {
        self.set_interrupt_handler(self.spi.handler());
        Spi {
            spi: self.spi,
            _mode: PhantomData,
            guard: self.guard,
            pins: self.pins,
        }
    }

    /// Configures the SPI instance to use DMA with the specified channel.
    ///
    /// This method prepares the SPI instance for DMA transfers using SPI
    /// and returns an instance of `SpiDma` that supports DMA
    /// operations.
    /// ```rust, no_run
    #[doc = crate::before_snippet!()]
    /// # use esp_hal::spi::Mode;
    /// # use esp_hal::spi::master::{Config, Spi};
    /// # use esp_hal::dma::{DmaRxBuf, DmaTxBuf};
    /// # use esp_hal::dma_buffers;
    #[cfg_attr(any(esp32, esp32s2), doc = "let dma_channel = peripherals.DMA_SPI2;")]
    #[cfg_attr(
        not(any(esp32, esp32s2)),
        doc = "let dma_channel = peripherals.DMA_CH0;"
    )]
    /// let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) =
    ///     dma_buffers!(32000);
    ///
    /// let dma_rx_buf = DmaRxBuf::new(
    ///     rx_descriptors,
    ///     rx_buffer
    /// )?;
    ///
    /// let dma_tx_buf = DmaTxBuf::new(
    ///     tx_descriptors,
    ///     tx_buffer
    /// )?;
    ///
    /// let mut spi = Spi::new(
    ///     peripherals.SPI2,
    ///     Config::default()
    ///         .with_frequency(Rate::from_khz(100))
    ///         .with_mode(Mode::_0),
    /// )?
    /// .with_dma(dma_channel)
    /// .with_buffers(dma_rx_buf, dma_tx_buf);
    /// # Ok(())
    /// # }
    /// ```
    #[instability::unstable]
    pub fn with_dma(self, channel: impl DmaChannelFor<AnySpi<'d>>) -> SpiDma<'d, Blocking> {
        SpiDma::new(self.spi, self.pins, channel.degrade())
    }

    #[cfg_attr(
        not(multi_core),
        doc = "Registers an interrupt handler for the peripheral."
    )]
    #[cfg_attr(
        multi_core,
        doc = "Registers an interrupt handler for the peripheral on the current core."
    )]
    #[doc = ""]
    /// Note that this will replace any previously registered interrupt
    /// handlers.
    ///
    /// You can restore the default/unhandled interrupt handler by using
    /// [crate::interrupt::DEFAULT_INTERRUPT_HANDLER]
    ///
    /// # Panics
    ///
    /// Panics if passed interrupt handler is invalid (e.g. has priority
    /// `None`)
    #[instability::unstable]
    pub fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        let interrupt = self.driver().info.interrupt;
        for core in Cpu::other() {
            crate::interrupt::disable(core, interrupt);
        }
        unsafe { crate::interrupt::bind_interrupt(interrupt, handler.handler()) };
        unwrap!(crate::interrupt::enable(interrupt, handler.priority()));
    }
}

#[instability::unstable]
impl crate::interrupt::InterruptConfigurable for Spi<'_, Blocking> {
    /// Sets the interrupt handler
    ///
    /// Interrupts are not enabled at the peripheral level here.
    fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        Spi::set_interrupt_handler(self, handler);
    }
}

impl<'d> Spi<'d, Async> {
    /// Converts the SPI instance into blocking mode.
    pub fn into_blocking(self) -> Spi<'d, Blocking> {
        crate::interrupt::disable(Cpu::current(), self.driver().info.interrupt);
        Spi {
            spi: self.spi,
            _mode: PhantomData,
            guard: self.guard,
            pins: self.pins,
        }
    }

    /// Waits for the completion of previous operations.
    pub async fn flush_async(&mut self) -> Result<(), Error> {
        let driver = self.driver();

        if !driver.busy() {
            return Ok(());
        }

        let future = SpiFuture::setup(&driver).await;
        future.await;

        Ok(())
    }

    /// Sends `words` to the slave. Returns the `words` received from the slave.
    ///
    /// This function aborts the transfer when its Future is dropped. Some
    /// amount of data may have been transferred before the Future is
    /// dropped. Dropping the future may block for a short while to ensure
    /// the transfer is aborted.
    pub async fn transfer_in_place_async(&mut self, words: &mut [u8]) -> Result<(), Error> {
        // We need to flush because the blocking transfer functions may return while a
        // transfer is still in progress.
        self.flush_async().await?;
        self.driver().setup_full_duplex()?;
        self.driver().transfer_in_place_async(words).await
    }
}

impl<'d, Dm> Spi<'d, Dm>
where
    Dm: DriverMode,
{
    /// Assign the SCK (Serial Clock) pin for the SPI instance.
    ///
    /// Configures the specified pin to push-pull output and connects it to the
    /// SPI clock signal.
    ///
    /// Disconnects the previous pin that was assigned with `with_sck`.
    pub fn with_sck(mut self, sclk: impl PeripheralOutput<'d>) -> Self {
        let sclk = sclk.into();
        sclk.set_to_push_pull_output();
        self.pins.sclk_pin = sclk.connect_with_guard(self.driver().info.sclk);

        self
    }

    /// Assign the MOSI (Master Out Slave In) pin for the SPI instance.
    ///
    /// Enables output functionality for the pin, and connects it as the MOSI
    /// signal. You want to use this for full-duplex SPI or
    /// if you intend to use [DataMode::SingleTwoDataLines].
    ///
    /// Disconnects the previous pin that was assigned with `with_mosi` or
    /// `with_sio0`.
    pub fn with_mosi(mut self, mosi: impl PeripheralOutput<'d>) -> Self {
        let mosi = mosi.into();
        mosi.enable_output(false);

        self.pins.mosi_pin = mosi.connect_with_guard(self.driver().info.mosi);

        self
    }

    /// Assign the MISO (Master In Slave Out) pin for the SPI instance.
    ///
    /// Enables input functionality for the pin, and connects it to the MISO
    /// signal.
    ///
    /// You want to use this for full-duplex SPI or
    /// [DataMode::SingleTwoDataLines]
    pub fn with_miso(self, miso: impl PeripheralInput<'d>) -> Self {
        let miso = miso.into();
        miso.enable_input(true);

        self.driver().info.miso.connect_to(&miso);

        self
    }

    /// Assign the SIO0 pin for the SPI instance.
    ///
    /// Enables both input and output functionality for the pin, and connects it
    /// to the MOSI signal and SIO0 input signal.
    ///
    /// Disconnects the previous pin that was assigned with `with_sio0` or
    /// `with_mosi`.
    ///
    /// Use this if any of the devices on the bus use half-duplex SPI.
    ///
    /// The pin is configured to open-drain mode.
    ///
    /// Note: You do not need to call [Self::with_mosi] when this is used.
    #[instability::unstable]
    pub fn with_sio0(mut self, mosi: impl PeripheralOutput<'d>) -> Self {
        let mosi = mosi.into();
        mosi.enable_output(true);
        mosi.enable_input(true);

        self.driver().info.sio0_input.connect_to(&mosi);
        self.pins.mosi_pin = mosi.connect_with_guard(self.driver().info.mosi);

        self
    }

    /// Assign the SIO1/MISO pin for the SPI instance.
    ///
    /// Enables both input and output functionality for the pin, and connects it
    /// to the MISO signal and SIO1 input signal.
    ///
    /// Disconnects the previous pin that was assigned with `with_sio1`.
    ///
    /// Use this if any of the devices on the bus use half-duplex SPI.
    ///
    /// The pin is configured to open-drain mode.
    ///
    /// Note: You do not need to call [Self::with_miso] when this is used.
    #[instability::unstable]
    pub fn with_sio1(mut self, sio1: impl PeripheralOutput<'d>) -> Self {
        let sio1 = sio1.into();
        sio1.enable_input(true);
        sio1.enable_output(true);

        self.driver().info.miso.connect_to(&sio1);
        self.pins.sio1_pin = sio1.connect_with_guard(self.driver().info.sio1_output);

        self
    }

    /// Assign the SIO2 pin for the SPI instance.
    ///
    /// Enables both input and output functionality for the pin, and connects it
    /// to the SIO2 output and input signals.
    ///
    /// # Current Stability Limitations
    /// QSPI operations are unstable, associated pins configuration is
    /// inefficient.
    #[instability::unstable]
    pub fn with_sio2(mut self, sio2: impl PeripheralOutput<'d>) -> Self {
        // TODO: panic if not QSPI?
        let sio2 = sio2.into();
        sio2.enable_input(true);
        sio2.enable_output(true);

        unwrap!(self.driver().info.sio2_input).connect_to(&sio2);
        self.pins.sio2_pin = self
            .driver()
            .info
            .sio2_output
            .map(|signal| sio2.connect_with_guard(signal));

        self
    }

    /// Assign the SIO3 pin for the SPI instance.
    ///
    /// Enables both input and output functionality for the pin, and connects it
    /// to the SIO3 output and input signals.
    ///
    /// # Current Stability Limitations
    /// QSPI operations are unstable, associated pins configuration is
    /// inefficient.
    #[instability::unstable]
    pub fn with_sio3(mut self, sio3: impl PeripheralOutput<'d>) -> Self {
        // TODO: panic if not QSPI?
        let sio3 = sio3.into();
        sio3.enable_input(true);
        sio3.enable_output(true);

        unwrap!(self.driver().info.sio3_input).connect_to(&sio3);
        self.pins.sio3_pin = self
            .driver()
            .info
            .sio3_output
            .map(|signal| sio3.connect_with_guard(signal));

        self
    }

    /// Assign the CS (Chip Select) pin for the SPI instance.
    ///
    /// Configures the specified pin to push-pull output and connects it to the
    /// SPI CS signal.
    ///
    /// Disconnects the previous pin that was assigned with `with_cs`.
    ///
    /// # Current Stability Limitations
    /// The hardware chip select functionality is limited; only one CS line can
    /// be set, regardless of the total number available. There is no
    /// mechanism to select which CS line to use.
    #[instability::unstable]
    pub fn with_cs(mut self, cs: impl PeripheralOutput<'d>) -> Self {
        let cs = cs.into();
        cs.set_to_push_pull_output();
        self.pins.cs_pin = cs.connect_with_guard(self.driver().info.cs[0]);

        self
    }

    /// Change the bus configuration.
    ///
    /// # Errors
    ///
    /// If frequency passed in config exceeds
    #[cfg_attr(not(esp32h2), doc = " 80MHz")]
    #[cfg_attr(esp32h2, doc = " 48MHz")]
    /// or is below 70kHz,
    /// [`ConfigError::UnsupportedFrequency`] error will be returned.
    pub fn apply_config(&mut self, config: &Config) -> Result<(), ConfigError> {
        self.driver().apply_config(config)
    }

    /// Write bytes to SPI. After writing, flush is called to ensure all data
    /// has been transmitted.
    pub fn write(&mut self, words: &[u8]) -> Result<(), Error> {
        self.driver().setup_full_duplex()?;
        self.driver().write(words)?;
        self.driver().flush()?;

        Ok(())
    }

    /// Read bytes from SPI. The provided slice is filled with data received
    /// from the slave.
    pub fn read(&mut self, words: &mut [u8]) -> Result<(), Error> {
        self.driver().setup_full_duplex()?;
        self.driver().read(words)
    }

    /// Sends `words` to the slave. Returns the `words` received from the slave.
    pub fn transfer<'w>(&mut self, words: &'w mut [u8]) -> Result<&'w [u8], Error> {
        self.driver().setup_full_duplex()?;
        self.driver().transfer(words)
    }

    /// Half-duplex read.
    ///
    /// # Errors
    ///
    /// [`Error::FifoSizeExeeded`] or [`Error::Unsupported`] will be returned if
    /// passed buffer is bigger than FIFO size or if buffer is empty (currently
    /// unsupported). `DataMode::Single` cannot be combined with any other
    /// [`DataMode`], otherwise [`Error::Unsupported`] will be returned.
    #[instability::unstable]
    pub fn half_duplex_read(
        &mut self,
        data_mode: DataMode,
        cmd: Command,
        address: Address,
        dummy: u8,
        buffer: &mut [u8],
    ) -> Result<(), Error> {
        if buffer.len() > FIFO_SIZE {
            return Err(Error::FifoSizeExeeded);
        }

        if buffer.is_empty() {
            error!("Half-duplex mode does not support empty buffer");
            return Err(Error::Unsupported);
        }

        self.driver().setup_half_duplex(
            false,
            cmd,
            address,
            false,
            dummy,
            buffer.is_empty(),
            data_mode,
        )?;

        self.driver().configure_datalen(buffer.len(), 0);
        self.driver().start_operation();
        self.driver().flush()?;
        self.driver().read_from_fifo(buffer)
    }

    /// Half-duplex write.
    ///
    /// # Errors
    ///
    /// [`Error::FifoSizeExeeded`] will be returned if
    /// passed buffer is bigger than FIFO size.
    #[cfg_attr(
        esp32,
        doc = "Dummy phase configuration is currently not supported, only value `0` is valid (see issue [#2240](https://github.com/esp-rs/esp-hal/issues/2240))."
    )]
    #[instability::unstable]
    pub fn half_duplex_write(
        &mut self,
        data_mode: DataMode,
        cmd: Command,
        address: Address,
        dummy: u8,
        buffer: &[u8],
    ) -> Result<(), Error> {
        if buffer.len() > FIFO_SIZE {
            return Err(Error::FifoSizeExeeded);
        }

        cfg_if::cfg_if! {
            if #[cfg(all(esp32, spi_address_workaround))] {
                let mut buffer = buffer;
                let mut data_mode = data_mode;
                let mut address = address;
                let addr_bytes;
                if buffer.is_empty() && !address.is_none() {
                    // If the buffer is empty, we need to send a dummy byte
                    // to trigger the address phase.
                    let bytes_to_write = address.width().div_ceil(8);
                    // The address register is read in big-endian order,
                    // we have to prepare the emulated write in the same way.
                    addr_bytes = address.value().to_be_bytes();
                    buffer = &addr_bytes[4 - bytes_to_write..][..bytes_to_write];
                    data_mode = address.mode();
                    address = Address::None;
                }

                if dummy > 0 {
                    // FIXME: https://github.com/esp-rs/esp-hal/issues/2240
                    error!("Dummy bits are not supported without data");
                    return Err(Error::Unsupported);
                }
            }
        }

        self.driver().setup_half_duplex(
            true,
            cmd,
            address,
            false,
            dummy,
            buffer.is_empty(),
            data_mode,
        )?;

        if !buffer.is_empty() {
            // re-using the full-duplex write here
            self.driver().write(buffer)?;
        } else {
            self.driver().start_operation();
        }

        self.driver().flush()
    }

    fn driver(&self) -> Driver {
        Driver {
            info: self.spi.info(),
            state: self.spi.state(),
        }
    }
}

#[instability::unstable]
impl<Dm> embassy_embedded_hal::SetConfig for Spi<'_, Dm>
where
    Dm: DriverMode,
{
    type Config = Config;
    type ConfigError = ConfigError;

    fn set_config(&mut self, config: &Self::Config) -> Result<(), Self::ConfigError> {
        self.apply_config(config)
    }
}

mod dma {
    use core::{
        cmp::min,
        mem::ManuallyDrop,
        sync::atomic::{Ordering, fence},
    };

    use super::*;
    use crate::{
        dma::{Channel, DmaRxBuf, DmaTxBuf, EmptyBuf, PeripheralDmaChannel, asynch::DmaRxFuture},
        spi::master::dma::asynch::DropGuard,
    };

    /// A DMA capable SPI instance.
    ///
    /// Using `SpiDma` is not recommended unless you wish
    /// to manage buffers yourself. It's recommended to use
    /// [`SpiDmaBus`] via `with_buffers` to get access
    /// to a DMA capable SPI bus that implements the
    /// embedded-hal traits.
    /// ```rust, no_run
    #[doc = crate::before_snippet!()]
    /// # use esp_hal::spi::Mode;
    /// # use esp_hal::spi::master::{Config, Spi};
    /// # use esp_hal::dma::{DmaRxBuf, DmaTxBuf};
    /// # use esp_hal::dma_buffers;
    #[cfg_attr(any(esp32, esp32s2), doc = "let dma_channel = peripherals.DMA_SPI2;")]
    #[cfg_attr(
        not(any(esp32, esp32s2)),
        doc = "let dma_channel = peripherals.DMA_CH0;"
    )]
    /// let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) =
    ///     dma_buffers!(32000);
    ///
    /// let dma_rx_buf = DmaRxBuf::new(
    ///     rx_descriptors,
    ///     rx_buffer
    /// )?;
    ///
    /// let dma_tx_buf = DmaTxBuf::new(
    ///     tx_descriptors,
    ///     tx_buffer
    /// )?;
    ///
    /// let mut spi = Spi::new(
    ///     peripherals.SPI2,
    ///     Config::default()
    ///         .with_frequency(Rate::from_khz(100))
    ///         .with_mode(Mode::_0),
    /// )?
    /// .with_dma(dma_channel)
    /// .with_buffers(dma_rx_buf, dma_tx_buf);
    /// # Ok(())
    /// # }
    /// ```
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub struct SpiDma<'d, Dm>
    where
        Dm: DriverMode,
    {
        pub(crate) spi: AnySpi<'d>,
        pub(crate) channel: Channel<Dm, PeripheralDmaChannel<AnySpi<'d>>>,
        tx_transfer_in_progress: bool,
        rx_transfer_in_progress: bool,
        #[cfg(all(esp32, spi_address_workaround))]
        address_buffer: DmaTxBuf,
        guard: PeripheralGuard,
        pins: SpiPinGuard,
    }

    impl<Dm> crate::private::Sealed for SpiDma<'_, Dm> where Dm: DriverMode {}

    impl<'d> SpiDma<'d, Blocking> {
        /// Converts the SPI instance into async mode.
        #[instability::unstable]
        pub fn into_async(mut self) -> SpiDma<'d, Async> {
            self.set_interrupt_handler(self.spi.handler());
            SpiDma {
                spi: self.spi,
                channel: self.channel.into_async(),
                tx_transfer_in_progress: self.tx_transfer_in_progress,
                rx_transfer_in_progress: self.rx_transfer_in_progress,
                #[cfg(all(esp32, spi_address_workaround))]
                address_buffer: self.address_buffer,
                guard: self.guard,
                pins: self.pins,
            }
        }

        pub(super) fn new(
            spi: AnySpi<'d>,
            pins: SpiPinGuard,
            channel: PeripheralDmaChannel<AnySpi<'d>>,
        ) -> Self {
            let channel = Channel::new(channel);
            channel.runtime_ensure_compatible(&spi);
            #[cfg(all(esp32, spi_address_workaround))]
            let address_buffer = {
                use crate::dma::DmaDescriptor;
                const SPI_NUM: usize = 2;
                static mut DESCRIPTORS: [[DmaDescriptor; 1]; SPI_NUM] =
                    [[DmaDescriptor::EMPTY]; SPI_NUM];
                static mut BUFFERS: [[u32; 1]; SPI_NUM] = [[0; 1]; SPI_NUM];

                let id = if spi.info() == unsafe { crate::peripherals::SPI2::steal().info() } {
                    0
                } else {
                    1
                };

                unwrap!(DmaTxBuf::new(
                    unsafe { &mut DESCRIPTORS[id] },
                    crate::dma::as_mut_byte_array!(BUFFERS[id], 4)
                ))
            };

            let guard = PeripheralGuard::new(spi.info().peripheral);

            Self {
                spi,
                channel,
                #[cfg(all(esp32, spi_address_workaround))]
                address_buffer,
                tx_transfer_in_progress: false,
                rx_transfer_in_progress: false,
                guard,
                pins,
            }
        }

        /// Listen for the given interrupts
        #[instability::unstable]
        pub fn listen(&mut self, interrupts: impl Into<EnumSet<SpiInterrupt>>) {
            self.driver().enable_listen(interrupts.into(), true);
        }

        /// Unlisten the given interrupts
        #[instability::unstable]
        pub fn unlisten(&mut self, interrupts: impl Into<EnumSet<SpiInterrupt>>) {
            self.driver().enable_listen(interrupts.into(), false);
        }

        /// Gets asserted interrupts
        #[instability::unstable]
        pub fn interrupts(&mut self) -> EnumSet<SpiInterrupt> {
            self.driver().interrupts()
        }

        /// Resets asserted interrupts
        #[instability::unstable]
        pub fn clear_interrupts(&mut self, interrupts: impl Into<EnumSet<SpiInterrupt>>) {
            self.driver().clear_interrupts(interrupts.into());
        }

        #[cfg_attr(
            not(multi_core),
            doc = "Registers an interrupt handler for the peripheral."
        )]
        #[cfg_attr(
            multi_core,
            doc = "Registers an interrupt handler for the peripheral on the current core."
        )]
        #[doc = ""]
        /// Note that this will replace any previously registered interrupt
        /// handlers.
        ///
        /// You can restore the default/unhandled interrupt handler by using
        /// [crate::interrupt::DEFAULT_INTERRUPT_HANDLER]
        ///
        /// # Panics
        ///
        /// Panics if passed interrupt handler is invalid (e.g. has priority
        /// `None`)
        #[instability::unstable]
        pub fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
            let interrupt = self.driver().info.interrupt;
            for core in Cpu::other() {
                crate::interrupt::disable(core, interrupt);
            }
            unsafe { crate::interrupt::bind_interrupt(interrupt, handler.handler()) };
            unwrap!(crate::interrupt::enable(interrupt, handler.priority()));
        }
    }

    impl<'d> SpiDma<'d, Async> {
        /// Converts the SPI instance into async mode.
        #[instability::unstable]
        pub fn into_blocking(self) -> SpiDma<'d, Blocking> {
            crate::interrupt::disable(Cpu::current(), self.driver().info.interrupt);
            SpiDma {
                spi: self.spi,
                channel: self.channel.into_blocking(),
                tx_transfer_in_progress: self.tx_transfer_in_progress,
                rx_transfer_in_progress: self.rx_transfer_in_progress,
                #[cfg(all(esp32, spi_address_workaround))]
                address_buffer: self.address_buffer,
                guard: self.guard,
                pins: self.pins,
            }
        }

        async fn wait_for_idle_async(&mut self) {
            if self.rx_transfer_in_progress {
                _ = DmaRxFuture::new(&mut self.channel.rx).await;
                self.rx_transfer_in_progress = false;
            }

            struct Fut(Driver);
            impl Future for Fut {
                type Output = ();

                fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
                    if self.0.interrupts().contains(SpiInterrupt::TransferDone) {
                        #[cfg(any(esp32, esp32s2))]
                        // Need to poll for done-ness even after interrupt fires.
                        if self.0.busy() {
                            cx.waker().wake_by_ref();
                            return Poll::Pending;
                        }

                        self.0.clear_interrupts(SpiInterrupt::TransferDone.into());
                        return Poll::Ready(());
                    }

                    self.0.state.waker.register(cx.waker());
                    self.0
                        .enable_listen(SpiInterrupt::TransferDone.into(), true);
                    Poll::Pending
                }
            }
            impl Drop for Fut {
                fn drop(&mut self) {
                    self.0
                        .enable_listen(SpiInterrupt::TransferDone.into(), false);
                }
            }

            if !self.is_done() {
                Fut(self.driver()).await;
            }

            if self.tx_transfer_in_progress {
                // In case DMA TX buffer is bigger than what the SPI consumes, stop the DMA.
                if !self.channel.tx.is_done() {
                    self.channel.tx.stop_transfer();
                }
                self.tx_transfer_in_progress = false;
            }
        }
    }

    impl<Dm> core::fmt::Debug for SpiDma<'_, Dm>
    where
        Dm: DriverMode,
    {
        /// Formats the `SpiDma` instance for debugging purposes.
        ///
        /// This method returns a debug struct with the name "SpiDma" without
        /// exposing internal details.
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
            f.debug_struct("SpiDma").field("spi", &self.spi).finish()
        }
    }

    #[instability::unstable]
    impl crate::interrupt::InterruptConfigurable for SpiDma<'_, Blocking> {
        /// Sets the interrupt handler
        ///
        /// Interrupts are not enabled at the peripheral level here.
        fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
            let interrupt = self.driver().info.interrupt;
            for core in crate::system::Cpu::other() {
                crate::interrupt::disable(core, interrupt);
            }
            unsafe { crate::interrupt::bind_interrupt(interrupt, handler.handler()) };
            unwrap!(crate::interrupt::enable(interrupt, handler.priority()));
        }
    }

    impl<Dm> SpiDma<'_, Dm>
    where
        Dm: DriverMode,
    {
        fn driver(&self) -> Driver {
            Driver {
                info: self.spi.info(),
                state: self.spi.state(),
            }
        }

        fn dma_driver(&self) -> DmaDriver {
            DmaDriver {
                driver: self.driver(),
                dma_peripheral: self.spi.dma_peripheral(),
            }
        }

        fn is_done(&self) -> bool {
            if self.driver().busy() {
                return false;
            }
            if self.rx_transfer_in_progress {
                // If this is an asymmetric transfer and the RX side is smaller, the RX channel
                // will never be "done" as it won't have enough descriptors/buffer to receive
                // the EOF bit from the SPI. So instead the RX channel will hit
                // a "descriptor empty" which means the DMA is written as much
                // of the received data as possible into the buffer and
                // discarded the rest. The user doesn't care about this discarded data.

                if !self.channel.rx.is_done() && !self.channel.rx.has_dscr_empty_error() {
                    return false;
                }
            }
            true
        }

        fn wait_for_idle(&mut self) {
            while !self.is_done() {
                // Wait for the SPI to become idle
            }
            self.rx_transfer_in_progress = false;
            self.tx_transfer_in_progress = false;
            fence(Ordering::Acquire);
        }

        /// # Safety:
        ///
        /// The caller must ensure to not access the buffer contents while the
        /// transfer is in progress. Moving the buffer itself is allowed.
        #[cfg_attr(place_spi_driver_in_ram, ram)]
        unsafe fn start_transfer_dma<RX: DmaRxBuffer, TX: DmaTxBuffer>(
            &mut self,
            full_duplex: bool,
            bytes_to_read: usize,
            bytes_to_write: usize,
            rx_buffer: &mut RX,
            tx_buffer: &mut TX,
        ) -> Result<(), Error> {
            if bytes_to_read > MAX_DMA_SIZE || bytes_to_write > MAX_DMA_SIZE {
                return Err(Error::MaxDmaTransferSizeExceeded);
            }

            self.rx_transfer_in_progress = bytes_to_read > 0;
            self.tx_transfer_in_progress = bytes_to_write > 0;
            unsafe {
                self.dma_driver().start_transfer_dma(
                    full_duplex,
                    bytes_to_read,
                    bytes_to_write,
                    rx_buffer,
                    tx_buffer,
                    &mut self.channel,
                )
            }
        }

        #[cfg(all(esp32, spi_address_workaround))]
        fn set_up_address_workaround(
            &mut self,
            cmd: Command,
            address: Address,
            dummy: u8,
        ) -> Result<(), Error> {
            if dummy > 0 {
                // FIXME: https://github.com/esp-rs/esp-hal/issues/2240
                error!("Dummy bits are not supported when there is no data to write");
                return Err(Error::Unsupported);
            }

            let bytes_to_write = address.width().div_ceil(8);
            // The address register is read in big-endian order,
            // we have to prepare the emulated write in the same way.
            let addr_bytes = address.value().to_be_bytes();
            let addr_bytes = &addr_bytes[4 - bytes_to_write..][..bytes_to_write];
            self.address_buffer.fill(addr_bytes);

            self.driver().setup_half_duplex(
                true,
                cmd,
                Address::None,
                false,
                dummy,
                bytes_to_write == 0,
                address.mode(),
            )?;

            // FIXME: we could use self.start_transfer_dma if the address buffer was part of
            // the (yet-to-be-created) State struct.
            self.tx_transfer_in_progress = true;
            unsafe {
                self.dma_driver().start_transfer_dma(
                    false,
                    0,
                    bytes_to_write,
                    &mut EmptyBuf,
                    &mut self.address_buffer,
                    &mut self.channel,
                )
            }
        }

        fn cancel_transfer(&mut self) {
            // The SPI peripheral is controlling how much data we transfer, so let's
            // update its counter.
            // 0 doesn't take effect on ESP32 and cuts the currently transmitted byte
            // immediately.
            // 1 seems to stop after transmitting the current byte which is somewhat less
            // impolite.
            if self.tx_transfer_in_progress || self.rx_transfer_in_progress {
                self.dma_driver().abort_transfer();

                // We need to stop the DMA transfer, too.
                if self.tx_transfer_in_progress {
                    self.channel.tx.stop_transfer();
                    self.tx_transfer_in_progress = false;
                }
                if self.rx_transfer_in_progress {
                    self.channel.rx.stop_transfer();
                    self.rx_transfer_in_progress = false;
                }
            }
        }
    }

    #[instability::unstable]
    impl<Dm> embassy_embedded_hal::SetConfig for SpiDma<'_, Dm>
    where
        Dm: DriverMode,
    {
        type Config = Config;
        type ConfigError = ConfigError;

        fn set_config(&mut self, config: &Self::Config) -> Result<(), Self::ConfigError> {
            self.apply_config(config)
        }
    }

    /// A structure representing a DMA transfer for SPI.
    ///
    /// This structure holds references to the SPI instance, DMA buffers, and
    /// transfer status.
    #[instability::unstable]
    pub struct SpiDmaTransfer<'d, Dm, Buf>
    where
        Dm: DriverMode,
    {
        spi_dma: ManuallyDrop<SpiDma<'d, Dm>>,
        dma_buf: ManuallyDrop<Buf>,
    }

    impl<Buf> SpiDmaTransfer<'_, Async, Buf> {
        /// Waits for the DMA transfer to complete asynchronously.
        ///
        /// This method awaits the completion of both RX and TX operations.
        #[instability::unstable]
        pub async fn wait_for_done(&mut self) {
            self.spi_dma.wait_for_idle_async().await;
        }
    }

    impl<'d, Dm, Buf> SpiDmaTransfer<'d, Dm, Buf>
    where
        Dm: DriverMode,
    {
        fn new(spi_dma: SpiDma<'d, Dm>, dma_buf: Buf) -> Self {
            Self {
                spi_dma: ManuallyDrop::new(spi_dma),
                dma_buf: ManuallyDrop::new(dma_buf),
            }
        }

        /// Checks if the transfer is complete.
        ///
        /// This method returns `true` if both RX and TX operations are done,
        /// and the SPI instance is no longer busy.
        pub fn is_done(&self) -> bool {
            self.spi_dma.is_done()
        }

        /// Waits for the DMA transfer to complete.
        ///
        /// This method blocks until the transfer is finished and returns the
        /// `SpiDma` instance and the associated buffer.
        #[instability::unstable]
        pub fn wait(mut self) -> (SpiDma<'d, Dm>, Buf) {
            self.spi_dma.wait_for_idle();
            let retval = unsafe {
                (
                    ManuallyDrop::take(&mut self.spi_dma),
                    ManuallyDrop::take(&mut self.dma_buf),
                )
            };
            core::mem::forget(self);
            retval
        }

        /// Cancels the DMA transfer.
        #[instability::unstable]
        pub fn cancel(&mut self) {
            if !self.spi_dma.is_done() {
                self.spi_dma.cancel_transfer();
            }
        }
    }

    impl<Dm, Buf> Drop for SpiDmaTransfer<'_, Dm, Buf>
    where
        Dm: DriverMode,
    {
        fn drop(&mut self) {
            if !self.is_done() {
                self.spi_dma.cancel_transfer();
                self.spi_dma.wait_for_idle();

                unsafe {
                    ManuallyDrop::drop(&mut self.spi_dma);
                    ManuallyDrop::drop(&mut self.dma_buf);
                }
            }
        }
    }

    impl<'d, Dm> SpiDma<'d, Dm>
    where
        Dm: DriverMode,
    {
        /// # Safety:
        ///
        /// The caller must ensure that the buffers are not accessed while the
        /// transfer is in progress. Moving the buffers is allowed.
        #[cfg_attr(place_spi_driver_in_ram, ram)]
        unsafe fn start_dma_write(
            &mut self,
            bytes_to_write: usize,
            buffer: &mut impl DmaTxBuffer,
        ) -> Result<(), Error> {
            unsafe { self.start_dma_transfer(0, bytes_to_write, &mut EmptyBuf, buffer) }
        }

        /// Configures the DMA buffers for the SPI instance.
        ///
        /// This method sets up both RX and TX buffers for DMA transfers.
        /// It returns an instance of `SpiDmaBus` that can be used for SPI
        /// communication.
        #[instability::unstable]
        pub fn with_buffers(self, dma_rx_buf: DmaRxBuf, dma_tx_buf: DmaTxBuf) -> SpiDmaBus<'d, Dm> {
            SpiDmaBus::new(self, dma_rx_buf, dma_tx_buf)
        }

        /// Perform a DMA write.
        ///
        /// This will return a [SpiDmaTransfer] owning the buffer and the
        /// SPI instance. The maximum amount of data to be sent is 32736
        /// bytes.
        #[allow(clippy::type_complexity)]
        #[cfg_attr(place_spi_driver_in_ram, ram)]
        #[instability::unstable]
        pub fn write<TX: DmaTxBuffer>(
            mut self,
            bytes_to_write: usize,
            mut buffer: TX,
        ) -> Result<SpiDmaTransfer<'d, Dm, TX>, (Error, Self, TX)> {
            self.wait_for_idle();
            if let Err(e) = self.driver().setup_full_duplex() {
                return Err((e, self, buffer));
            };
            match unsafe { self.start_dma_write(bytes_to_write, &mut buffer) } {
                Ok(_) => Ok(SpiDmaTransfer::new(self, buffer)),
                Err(e) => Err((e, self, buffer)),
            }
        }

        /// # Safety:
        ///
        /// The caller must ensure that the buffers are not accessed while the
        /// transfer is in progress. Moving the buffers is allowed.
        #[cfg_attr(place_spi_driver_in_ram, ram)]
        unsafe fn start_dma_read(
            &mut self,
            bytes_to_read: usize,
            buffer: &mut impl DmaRxBuffer,
        ) -> Result<(), Error> {
            unsafe { self.start_dma_transfer(bytes_to_read, 0, buffer, &mut EmptyBuf) }
        }

        /// Perform a DMA read.
        ///
        /// This will return a [SpiDmaTransfer] owning the buffer and
        /// the SPI instance. The maximum amount of data to be
        /// received is 32736 bytes.
        #[allow(clippy::type_complexity)]
        #[cfg_attr(place_spi_driver_in_ram, ram)]
        #[instability::unstable]
        pub fn read<RX: DmaRxBuffer>(
            mut self,
            bytes_to_read: usize,
            mut buffer: RX,
        ) -> Result<SpiDmaTransfer<'d, Dm, RX>, (Error, Self, RX)> {
            self.wait_for_idle();
            if let Err(e) = self.driver().setup_full_duplex() {
                return Err((e, self, buffer));
            };
            match unsafe { self.start_dma_read(bytes_to_read, &mut buffer) } {
                Ok(_) => Ok(SpiDmaTransfer::new(self, buffer)),
                Err(e) => Err((e, self, buffer)),
            }
        }

        /// # Safety:
        ///
        /// The caller must ensure that the buffers are not accessed while the
        /// transfer is in progress. Moving the buffers is allowed.
        #[cfg_attr(place_spi_driver_in_ram, ram)]
        unsafe fn start_dma_transfer(
            &mut self,
            bytes_to_read: usize,
            bytes_to_write: usize,
            rx_buffer: &mut impl DmaRxBuffer,
            tx_buffer: &mut impl DmaTxBuffer,
        ) -> Result<(), Error> {
            unsafe {
                self.start_transfer_dma(true, bytes_to_read, bytes_to_write, rx_buffer, tx_buffer)
            }
        }

        /// Perform a DMA transfer
        ///
        /// This will return a [SpiDmaTransfer] owning the buffers and
        /// the SPI instance. The maximum amount of data to be
        /// sent/received is 32736 bytes.
        #[allow(clippy::type_complexity)]
        #[cfg_attr(place_spi_driver_in_ram, ram)]
        #[instability::unstable]
        pub fn transfer<RX: DmaRxBuffer, TX: DmaTxBuffer>(
            mut self,
            bytes_to_read: usize,
            mut rx_buffer: RX,
            bytes_to_write: usize,
            mut tx_buffer: TX,
        ) -> Result<SpiDmaTransfer<'d, Dm, (RX, TX)>, (Error, Self, RX, TX)> {
            self.wait_for_idle();
            if let Err(e) = self.driver().setup_full_duplex() {
                return Err((e, self, rx_buffer, tx_buffer));
            };
            match unsafe {
                self.start_dma_transfer(
                    bytes_to_read,
                    bytes_to_write,
                    &mut rx_buffer,
                    &mut tx_buffer,
                )
            } {
                Ok(_) => Ok(SpiDmaTransfer::new(self, (rx_buffer, tx_buffer))),
                Err(e) => Err((e, self, rx_buffer, tx_buffer)),
            }
        }

        /// # Safety:
        ///
        /// The caller must ensure that the buffers are not accessed while the
        /// transfer is in progress. Moving the buffers is allowed.
        #[cfg_attr(place_spi_driver_in_ram, ram)]
        unsafe fn start_half_duplex_read(
            &mut self,
            data_mode: DataMode,
            cmd: Command,
            address: Address,
            dummy: u8,
            bytes_to_read: usize,
            buffer: &mut impl DmaRxBuffer,
        ) -> Result<(), Error> {
            self.driver().setup_half_duplex(
                false,
                cmd,
                address,
                false,
                dummy,
                bytes_to_read == 0,
                data_mode,
            )?;

            unsafe { self.start_transfer_dma(false, bytes_to_read, 0, buffer, &mut EmptyBuf) }
        }

        /// Perform a half-duplex read operation using DMA.
        #[allow(clippy::type_complexity)]
        #[cfg_attr(place_spi_driver_in_ram, ram)]
        #[instability::unstable]
        pub fn half_duplex_read<RX: DmaRxBuffer>(
            mut self,
            data_mode: DataMode,
            cmd: Command,
            address: Address,
            dummy: u8,
            bytes_to_read: usize,
            mut buffer: RX,
        ) -> Result<SpiDmaTransfer<'d, Dm, RX>, (Error, Self, RX)> {
            self.wait_for_idle();

            match unsafe {
                self.start_half_duplex_read(
                    data_mode,
                    cmd,
                    address,
                    dummy,
                    bytes_to_read,
                    &mut buffer,
                )
            } {
                Ok(_) => Ok(SpiDmaTransfer::new(self, buffer)),
                Err(e) => Err((e, self, buffer)),
            }
        }

        /// # Safety:
        ///
        /// The caller must ensure that the buffers are not accessed while the
        /// transfer is in progress. Moving the buffers is allowed.
        #[cfg_attr(place_spi_driver_in_ram, ram)]
        unsafe fn start_half_duplex_write(
            &mut self,
            data_mode: DataMode,
            cmd: Command,
            address: Address,
            dummy: u8,
            bytes_to_write: usize,
            buffer: &mut impl DmaTxBuffer,
        ) -> Result<(), Error> {
            #[cfg(all(esp32, spi_address_workaround))]
            {
                // On the ESP32, if we don't have data, the address is always sent
                // on a single line, regardless of its data mode.
                if bytes_to_write == 0 && address.mode() != DataMode::SingleTwoDataLines {
                    return self.set_up_address_workaround(cmd, address, dummy);
                }
            }

            self.driver().setup_half_duplex(
                true,
                cmd,
                address,
                false,
                dummy,
                bytes_to_write == 0,
                data_mode,
            )?;

            unsafe { self.start_transfer_dma(false, 0, bytes_to_write, &mut EmptyBuf, buffer) }
        }

        /// Perform a half-duplex write operation using DMA.
        #[allow(clippy::type_complexity)]
        #[cfg_attr(place_spi_driver_in_ram, ram)]
        #[instability::unstable]
        pub fn half_duplex_write<TX: DmaTxBuffer>(
            mut self,
            data_mode: DataMode,
            cmd: Command,
            address: Address,
            dummy: u8,
            bytes_to_write: usize,
            mut buffer: TX,
        ) -> Result<SpiDmaTransfer<'d, Dm, TX>, (Error, Self, TX)> {
            self.wait_for_idle();

            match unsafe {
                self.start_half_duplex_write(
                    data_mode,
                    cmd,
                    address,
                    dummy,
                    bytes_to_write,
                    &mut buffer,
                )
            } {
                Ok(_) => Ok(SpiDmaTransfer::new(self, buffer)),
                Err(e) => Err((e, self, buffer)),
            }
        }

        /// Change the bus configuration.
        ///
        /// # Errors
        ///
        /// If frequency passed in config exceeds
        #[cfg_attr(not(esp32h2), doc = " 80MHz")]
        #[cfg_attr(esp32h2, doc = " 48MHz")]
        /// or is below 70kHz,
        /// [`ConfigError::UnsupportedFrequency`] error will be returned.
        #[instability::unstable]
        pub fn apply_config(&mut self, config: &Config) -> Result<(), ConfigError> {
            self.driver().apply_config(config)
        }
    }

    /// A DMA-capable SPI bus.
    ///
    /// This structure is responsible for managing SPI transfers using DMA
    /// buffers.
    #[derive(Debug)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    #[instability::unstable]
    pub struct SpiDmaBus<'d, Dm>
    where
        Dm: DriverMode,
    {
        spi_dma: SpiDma<'d, Dm>,
        rx_buf: DmaRxBuf,
        tx_buf: DmaTxBuf,
    }

    impl<Dm> crate::private::Sealed for SpiDmaBus<'_, Dm> where Dm: DriverMode {}

    impl<'d> SpiDmaBus<'d, Blocking> {
        /// Converts the SPI instance into async mode.
        #[instability::unstable]
        pub fn into_async(self) -> SpiDmaBus<'d, Async> {
            SpiDmaBus {
                spi_dma: self.spi_dma.into_async(),
                rx_buf: self.rx_buf,
                tx_buf: self.tx_buf,
            }
        }

        /// Listen for the given interrupts
        #[instability::unstable]
        pub fn listen(&mut self, interrupts: impl Into<EnumSet<SpiInterrupt>>) {
            self.spi_dma.listen(interrupts.into());
        }

        /// Unlisten the given interrupts
        #[instability::unstable]
        pub fn unlisten(&mut self, interrupts: impl Into<EnumSet<SpiInterrupt>>) {
            self.spi_dma.unlisten(interrupts.into());
        }

        /// Gets asserted interrupts
        #[instability::unstable]
        pub fn interrupts(&mut self) -> EnumSet<SpiInterrupt> {
            self.spi_dma.interrupts()
        }

        /// Resets asserted interrupts
        #[instability::unstable]
        pub fn clear_interrupts(&mut self, interrupts: impl Into<EnumSet<SpiInterrupt>>) {
            self.spi_dma.clear_interrupts(interrupts.into());
        }
    }

    impl<'d> SpiDmaBus<'d, Async> {
        /// Converts the SPI instance into async mode.
        #[instability::unstable]
        pub fn into_blocking(self) -> SpiDmaBus<'d, Blocking> {
            SpiDmaBus {
                spi_dma: self.spi_dma.into_blocking(),
                rx_buf: self.rx_buf,
                tx_buf: self.tx_buf,
            }
        }

        /// Fill the given buffer with data from the bus.
        #[instability::unstable]
        pub async fn read_async(&mut self, words: &mut [u8]) -> Result<(), Error> {
            self.spi_dma.wait_for_idle_async().await;
            self.spi_dma.driver().setup_full_duplex()?;
            let chunk_size = self.rx_buf.capacity();

            for chunk in words.chunks_mut(chunk_size) {
                let mut spi = DropGuard::new(&mut self.spi_dma, |spi| spi.cancel_transfer());

                unsafe { spi.start_dma_transfer(chunk.len(), 0, &mut self.rx_buf, &mut EmptyBuf)? };

                spi.wait_for_idle_async().await;

                chunk.copy_from_slice(&self.rx_buf.as_slice()[..chunk.len()]);

                spi.defuse();
            }

            Ok(())
        }

        /// Transmit the given buffer to the bus.
        #[instability::unstable]
        pub async fn write_async(&mut self, words: &[u8]) -> Result<(), Error> {
            self.spi_dma.wait_for_idle_async().await;
            self.spi_dma.driver().setup_full_duplex()?;

            let mut spi = DropGuard::new(&mut self.spi_dma, |spi| spi.cancel_transfer());
            let chunk_size = self.tx_buf.capacity();

            for chunk in words.chunks(chunk_size) {
                self.tx_buf.as_mut_slice()[..chunk.len()].copy_from_slice(chunk);

                unsafe { spi.start_dma_transfer(0, chunk.len(), &mut EmptyBuf, &mut self.tx_buf)? };

                spi.wait_for_idle_async().await;
            }
            spi.defuse();

            Ok(())
        }

        /// Transfer by writing out a buffer and reading the response from
        /// the bus into another buffer.
        #[instability::unstable]
        pub async fn transfer_async(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Error> {
            self.spi_dma.wait_for_idle_async().await;
            self.spi_dma.driver().setup_full_duplex()?;

            let mut spi = DropGuard::new(&mut self.spi_dma, |spi| spi.cancel_transfer());
            let chunk_size = min(self.tx_buf.capacity(), self.rx_buf.capacity());

            let common_length = min(read.len(), write.len());
            let (read_common, read_remainder) = read.split_at_mut(common_length);
            let (write_common, write_remainder) = write.split_at(common_length);

            for (read_chunk, write_chunk) in read_common
                .chunks_mut(chunk_size)
                .zip(write_common.chunks(chunk_size))
            {
                self.tx_buf.as_mut_slice()[..write_chunk.len()].copy_from_slice(write_chunk);

                unsafe {
                    spi.start_dma_transfer(
                        read_chunk.len(),
                        write_chunk.len(),
                        &mut self.rx_buf,
                        &mut self.tx_buf,
                    )?;
                }
                spi.wait_for_idle_async().await;

                read_chunk.copy_from_slice(&self.rx_buf.as_slice()[..read_chunk.len()]);
            }

            spi.defuse();

            if !read_remainder.is_empty() {
                self.read_async(read_remainder).await
            } else if !write_remainder.is_empty() {
                self.write_async(write_remainder).await
            } else {
                Ok(())
            }
        }

        /// Transfer by writing out a buffer and reading the response from
        /// the bus into the same buffer.
        #[instability::unstable]
        pub async fn transfer_in_place_async(&mut self, words: &mut [u8]) -> Result<(), Error> {
            self.spi_dma.wait_for_idle_async().await;
            self.spi_dma.driver().setup_full_duplex()?;

            let mut spi = DropGuard::new(&mut self.spi_dma, |spi| spi.cancel_transfer());
            for chunk in words.chunks_mut(self.tx_buf.capacity()) {
                self.tx_buf.as_mut_slice()[..chunk.len()].copy_from_slice(chunk);

                unsafe {
                    spi.start_dma_transfer(
                        chunk.len(),
                        chunk.len(),
                        &mut self.rx_buf,
                        &mut self.tx_buf,
                    )?;
                }
                spi.wait_for_idle_async().await;
                chunk.copy_from_slice(&self.rx_buf.as_slice()[..chunk.len()]);
            }

            spi.defuse();

            Ok(())
        }
    }

    impl<'d, Dm> SpiDmaBus<'d, Dm>
    where
        Dm: DriverMode,
    {
        /// Creates a new `SpiDmaBus` with the specified SPI instance and DMA
        /// buffers.
        pub fn new(spi_dma: SpiDma<'d, Dm>, rx_buf: DmaRxBuf, tx_buf: DmaTxBuf) -> Self {
            Self {
                spi_dma,
                rx_buf,
                tx_buf,
            }
        }

        /// Splits [SpiDmaBus] back into [SpiDma], [DmaRxBuf] and [DmaTxBuf].
        #[instability::unstable]
        pub fn split(mut self) -> (SpiDma<'d, Dm>, DmaRxBuf, DmaTxBuf) {
            self.wait_for_idle();
            (self.spi_dma, self.rx_buf, self.tx_buf)
        }

        fn wait_for_idle(&mut self) {
            self.spi_dma.wait_for_idle();
        }

        /// Change the bus configuration.
        ///
        /// # Errors
        ///
        /// If frequency passed in config exceeds
        #[cfg_attr(not(esp32h2), doc = " 80MHz")]
        #[cfg_attr(esp32h2, doc = " 48MHz")]
        /// or is below 70kHz,
        /// [`ConfigError::UnsupportedFrequency`] error will be returned.
        #[instability::unstable]
        pub fn apply_config(&mut self, config: &Config) -> Result<(), ConfigError> {
            self.spi_dma.apply_config(config)
        }

        /// Reads data from the SPI bus using DMA.
        #[instability::unstable]
        pub fn read(&mut self, words: &mut [u8]) -> Result<(), Error> {
            self.wait_for_idle();
            self.spi_dma.driver().setup_full_duplex()?;
            for chunk in words.chunks_mut(self.rx_buf.capacity()) {
                unsafe {
                    self.spi_dma.start_dma_transfer(
                        chunk.len(),
                        0,
                        &mut self.rx_buf,
                        &mut EmptyBuf,
                    )?;
                }

                self.wait_for_idle();
                chunk.copy_from_slice(&self.rx_buf.as_slice()[..chunk.len()]);
            }

            Ok(())
        }

        /// Writes data to the SPI bus using DMA.
        #[instability::unstable]
        pub fn write(&mut self, words: &[u8]) -> Result<(), Error> {
            self.wait_for_idle();
            self.spi_dma.driver().setup_full_duplex()?;
            for chunk in words.chunks(self.tx_buf.capacity()) {
                self.tx_buf.as_mut_slice()[..chunk.len()].copy_from_slice(chunk);

                unsafe {
                    self.spi_dma.start_dma_transfer(
                        0,
                        chunk.len(),
                        &mut EmptyBuf,
                        &mut self.tx_buf,
                    )?;
                }

                self.wait_for_idle();
            }

            Ok(())
        }

        /// Transfers data to and from the SPI bus simultaneously using DMA.
        #[instability::unstable]
        pub fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Error> {
            self.wait_for_idle();
            self.spi_dma.driver().setup_full_duplex()?;
            let chunk_size = min(self.tx_buf.capacity(), self.rx_buf.capacity());

            let common_length = min(read.len(), write.len());
            let (read_common, read_remainder) = read.split_at_mut(common_length);
            let (write_common, write_remainder) = write.split_at(common_length);

            for (read_chunk, write_chunk) in read_common
                .chunks_mut(chunk_size)
                .zip(write_common.chunks(chunk_size))
            {
                self.tx_buf.as_mut_slice()[..write_chunk.len()].copy_from_slice(write_chunk);

                unsafe {
                    self.spi_dma.start_dma_transfer(
                        read_chunk.len(),
                        write_chunk.len(),
                        &mut self.rx_buf,
                        &mut self.tx_buf,
                    )?;
                }
                self.wait_for_idle();

                read_chunk.copy_from_slice(&self.rx_buf.as_slice()[..read_chunk.len()]);
            }

            if !read_remainder.is_empty() {
                self.read(read_remainder)
            } else if !write_remainder.is_empty() {
                self.write(write_remainder)
            } else {
                Ok(())
            }
        }

        /// Transfers data in place on the SPI bus using DMA.
        #[instability::unstable]
        pub fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Error> {
            self.wait_for_idle();
            self.spi_dma.driver().setup_full_duplex()?;
            let chunk_size = min(self.tx_buf.capacity(), self.rx_buf.capacity());

            for chunk in words.chunks_mut(chunk_size) {
                self.tx_buf.as_mut_slice()[..chunk.len()].copy_from_slice(chunk);

                unsafe {
                    self.spi_dma.start_dma_transfer(
                        chunk.len(),
                        chunk.len(),
                        &mut self.rx_buf,
                        &mut self.tx_buf,
                    )?;
                }
                self.wait_for_idle();
                chunk.copy_from_slice(&self.rx_buf.as_slice()[..chunk.len()]);
            }

            Ok(())
        }

        /// Half-duplex read.
        #[instability::unstable]
        pub fn half_duplex_read(
            &mut self,
            data_mode: DataMode,
            cmd: Command,
            address: Address,
            dummy: u8,
            buffer: &mut [u8],
        ) -> Result<(), Error> {
            if buffer.len() > self.rx_buf.capacity() {
                return Err(Error::from(DmaError::Overflow));
            }
            self.wait_for_idle();

            unsafe {
                self.spi_dma.start_half_duplex_read(
                    data_mode,
                    cmd,
                    address,
                    dummy,
                    buffer.len(),
                    &mut self.rx_buf,
                )?;
            }

            self.wait_for_idle();

            buffer.copy_from_slice(&self.rx_buf.as_slice()[..buffer.len()]);

            Ok(())
        }

        /// Half-duplex write.
        #[instability::unstable]
        pub fn half_duplex_write(
            &mut self,
            data_mode: DataMode,
            cmd: Command,
            address: Address,
            dummy: u8,
            buffer: &[u8],
        ) -> Result<(), Error> {
            if buffer.len() > self.tx_buf.capacity() {
                return Err(Error::from(DmaError::Overflow));
            }
            self.wait_for_idle();
            self.tx_buf.as_mut_slice()[..buffer.len()].copy_from_slice(buffer);

            unsafe {
                self.spi_dma.start_half_duplex_write(
                    data_mode,
                    cmd,
                    address,
                    dummy,
                    buffer.len(),
                    &mut self.tx_buf,
                )?;
            }

            self.wait_for_idle();

            Ok(())
        }
    }

    #[instability::unstable]
    impl crate::interrupt::InterruptConfigurable for SpiDmaBus<'_, Blocking> {
        /// Sets the interrupt handler
        ///
        /// Interrupts are not enabled at the peripheral level here.
        fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
            self.spi_dma.set_interrupt_handler(handler);
        }
    }

    #[instability::unstable]
    impl<Dm> embassy_embedded_hal::SetConfig for SpiDmaBus<'_, Dm>
    where
        Dm: DriverMode,
    {
        type Config = Config;
        type ConfigError = ConfigError;

        fn set_config(&mut self, config: &Self::Config) -> Result<(), Self::ConfigError> {
            self.apply_config(config)
        }
    }

    /// Async functionality
    mod asynch {
        use core::ops::{Deref, DerefMut};

        use super::*;

        #[cfg_attr(not(feature = "unstable"), allow(dead_code))]
        pub(crate) struct DropGuard<I, F: FnOnce(I)> {
            inner: ManuallyDrop<I>,
            on_drop: ManuallyDrop<F>,
        }

        #[cfg_attr(not(feature = "unstable"), allow(dead_code))]
        impl<I, F: FnOnce(I)> DropGuard<I, F> {
            pub(crate) fn new(inner: I, on_drop: F) -> Self {
                Self {
                    inner: ManuallyDrop::new(inner),
                    on_drop: ManuallyDrop::new(on_drop),
                }
            }

            pub(crate) fn defuse(self) {}
        }

        impl<I, F: FnOnce(I)> Drop for DropGuard<I, F> {
            fn drop(&mut self) {
                let inner = unsafe { ManuallyDrop::take(&mut self.inner) };
                let on_drop = unsafe { ManuallyDrop::take(&mut self.on_drop) };
                (on_drop)(inner)
            }
        }

        impl<I, F: FnOnce(I)> Deref for DropGuard<I, F> {
            type Target = I;

            fn deref(&self) -> &I {
                &self.inner
            }
        }

        impl<I, F: FnOnce(I)> DerefMut for DropGuard<I, F> {
            fn deref_mut(&mut self) -> &mut I {
                &mut self.inner
            }
        }

        #[instability::unstable]
        impl embedded_hal_async::spi::SpiBus for SpiDmaBus<'_, Async> {
            async fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
                self.read_async(words).await
            }

            async fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
                self.write_async(words).await
            }

            async fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
                self.transfer_async(read, write).await
            }

            async fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
                self.transfer_in_place_async(words).await
            }

            async fn flush(&mut self) -> Result<(), Self::Error> {
                // All operations currently flush so this is no-op.
                Ok(())
            }
        }
    }

    mod ehal1 {
        #[cfg(any(doc, feature = "unstable"))]
        use embedded_hal::spi::{ErrorType, SpiBus};

        #[cfg(any(doc, feature = "unstable"))]
        use super::*;

        #[instability::unstable]
        impl<Dm> ErrorType for SpiDmaBus<'_, Dm>
        where
            Dm: DriverMode,
        {
            type Error = Error;
        }

        #[instability::unstable]
        impl<Dm> SpiBus for SpiDmaBus<'_, Dm>
        where
            Dm: DriverMode,
        {
            fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
                self.read(words)
            }

            fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
                self.write(words)
            }

            fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
                self.transfer(read, write)
            }

            fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
                self.transfer_in_place(words)
            }

            fn flush(&mut self) -> Result<(), Self::Error> {
                // All operations currently flush so this is no-op.
                Ok(())
            }
        }
    }
}

mod ehal1 {
    use embedded_hal::spi::SpiBus;
    use embedded_hal_async::spi::SpiBus as SpiBusAsync;

    use super::*;

    impl<Dm> embedded_hal::spi::ErrorType for Spi<'_, Dm>
    where
        Dm: DriverMode,
    {
        type Error = Error;
    }

    impl<Dm> SpiBus for Spi<'_, Dm>
    where
        Dm: DriverMode,
    {
        fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
            self.driver().read(words)
        }

        fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
            self.driver().write(words)
        }

        fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
            // Optimizations
            if read.is_empty() {
                return SpiBus::write(self, write);
            } else if write.is_empty() {
                return SpiBus::read(self, read);
            }

            let mut write_from = 0;
            let mut read_from = 0;

            loop {
                // How many bytes we write in this chunk
                let write_inc = core::cmp::min(FIFO_SIZE, write.len() - write_from);
                let write_to = write_from + write_inc;
                // How many bytes we read in this chunk
                let read_inc = core::cmp::min(FIFO_SIZE, read.len() - read_from);
                let read_to = read_from + read_inc;

                if (write_inc == 0) && (read_inc == 0) {
                    break;
                }

                // No need to flush here, `SpiBus::write` will do it for us

                if write_to < read_to {
                    // Read more than we write, must pad writing part with zeros
                    let mut empty = [EMPTY_WRITE_PAD; FIFO_SIZE];
                    empty[0..write_inc].copy_from_slice(&write[write_from..write_to]);
                    SpiBus::write(self, &empty)?;
                } else {
                    SpiBus::write(self, &write[write_from..write_to])?;
                }

                if read_inc > 0 {
                    SpiBus::flush(self)?;
                    self.driver()
                        .read_from_fifo(&mut read[read_from..read_to])?;
                }

                write_from = write_to;
                read_from = read_to;
            }
            Ok(())
        }

        fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
            self.driver().transfer(words).map(|_| ())
        }

        fn flush(&mut self) -> Result<(), Self::Error> {
            self.driver().flush()
        }
    }

    impl SpiBusAsync for Spi<'_, Async> {
        async fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
            // We need to flush because the blocking transfer functions may return while a
            // transfer is still in progress.
            self.flush_async().await?;
            self.driver().setup_full_duplex()?;
            self.driver().read_async(words).await
        }

        async fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
            // We need to flush because the blocking transfer functions may return while a
            // transfer is still in progress.
            self.flush_async().await?;
            self.driver().setup_full_duplex()?;
            self.driver().write_async(words).await
        }

        async fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
            // Optimizations
            if read.is_empty() {
                return SpiBusAsync::write(self, write).await;
            } else if write.is_empty() {
                return SpiBusAsync::read(self, read).await;
            }

            let mut write_from = 0;
            let mut read_from = 0;

            loop {
                // How many bytes we write in this chunk
                let write_inc = core::cmp::min(FIFO_SIZE, write.len() - write_from);
                let write_to = write_from + write_inc;
                // How many bytes we read in this chunk
                let read_inc = core::cmp::min(FIFO_SIZE, read.len() - read_from);
                let read_to = read_from + read_inc;

                if (write_inc == 0) && (read_inc == 0) {
                    break;
                }

                // No need to flush here, `SpiBusAsync::write` will do it for us

                if write_to < read_to {
                    // Read more than we write, must pad writing part with zeros
                    let mut empty = [EMPTY_WRITE_PAD; FIFO_SIZE];
                    empty[0..write_inc].copy_from_slice(&write[write_from..write_to]);
                    SpiBusAsync::write(self, &empty).await?;
                } else {
                    SpiBusAsync::write(self, &write[write_from..write_to]).await?;
                }

                if read_inc > 0 {
                    self.driver()
                        .read_from_fifo(&mut read[read_from..read_to])?;
                }

                write_from = write_to;
                read_from = read_to;
            }
            Ok(())
        }

        async fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
            self.transfer_in_place_async(words).await
        }

        async fn flush(&mut self) -> Result<(), Self::Error> {
            self.flush_async().await
        }
    }
}

/// SPI peripheral instance.
#[doc(hidden)]
#[allow(private_bounds)]
pub trait PeripheralInstance: private::Sealed + DmaEligible {
    /// Returns the peripheral data describing this SPI instance.
    fn info(&self) -> &'static Info;
}

/// Marker trait for QSPI-capable SPI peripherals.
#[doc(hidden)]
pub trait QspiInstance: PeripheralInstance {}

/// Peripheral data describing a particular SPI instance.
#[doc(hidden)]
#[non_exhaustive]
pub struct Info {
    /// Pointer to the register block for this SPI instance.
    ///
    /// Use [Self::register_block] to access the register block.
    pub register_block: *const RegisterBlock,

    /// The system peripheral marker.
    pub peripheral: crate::system::Peripheral,

    /// Interrupt for this SPI instance.
    pub interrupt: crate::peripherals::Interrupt,

    /// SCLK signal.
    pub sclk: OutputSignal,

    /// MOSI signal.
    pub mosi: OutputSignal,

    /// MISO signal.
    pub miso: InputSignal,

    /// Chip select signals.
    pub cs: &'static [OutputSignal],

    /// SIO0 (MOSI) input signal for half-duplex mode.
    pub sio0_input: InputSignal,

    /// SIO1 (MISO) output signal for half-duplex mode.
    pub sio1_output: OutputSignal,

    /// SIO2 output signal for QSPI mode.
    pub sio2_output: Option<OutputSignal>,

    /// SIO2 input signal for QSPI mode.
    pub sio2_input: Option<InputSignal>,

    /// SIO3 output signal for QSPI mode.
    pub sio3_output: Option<OutputSignal>,

    /// SIO3 input signal for QSPI mode.
    pub sio3_input: Option<InputSignal>,

    /// SIO4 output signal for OPI mode.
    #[cfg(spi_octal)]
    pub sio4_output: Option<OutputSignal>,

    /// SIO4 input signal for OPI mode.
    #[cfg(spi_octal)]
    pub sio4_input: Option<InputSignal>,

    /// SIO5 output signal for OPI mode.
    #[cfg(spi_octal)]
    pub sio5_output: Option<OutputSignal>,

    /// SIO5 input signal for OPI mode.
    #[cfg(spi_octal)]
    pub sio5_input: Option<InputSignal>,

    /// SIO6 output signal for OPI mode.
    #[cfg(spi_octal)]
    pub sio6_output: Option<OutputSignal>,

    /// SIO6 input signal for OPI mode.
    #[cfg(spi_octal)]
    pub sio6_input: Option<InputSignal>,

    /// SIO7 output signal for OPI mode.
    #[cfg(spi_octal)]
    pub sio7_output: Option<OutputSignal>,

    /// SIO7 input signal for OPI mode.
    #[cfg(spi_octal)]
    pub sio7_input: Option<InputSignal>,
}

struct DmaDriver {
    driver: Driver,
    dma_peripheral: crate::dma::DmaPeripheral,
}

impl DmaDriver {
    fn abort_transfer(&self) {
        self.driver.configure_datalen(1, 1);
        self.driver.update();
    }

    fn regs(&self) -> &RegisterBlock {
        self.driver.regs()
    }

    #[allow(clippy::too_many_arguments)]
    #[cfg_attr(place_spi_driver_in_ram, ram)]
    unsafe fn start_transfer_dma<Dm: DriverMode>(
        &self,
        _full_duplex: bool,
        rx_len: usize,
        tx_len: usize,
        rx_buffer: &mut impl DmaRxBuffer,
        tx_buffer: &mut impl DmaTxBuffer,
        channel: &mut Channel<Dm, PeripheralDmaChannel<AnySpi<'_>>>,
    ) -> Result<(), Error> {
        #[cfg(esp32s2)]
        {
            // without this a transfer after a write will fail
            self.regs().dma_out_link().write(|w| unsafe { w.bits(0) });
            self.regs().dma_in_link().write(|w| unsafe { w.bits(0) });
        }

        self.driver.configure_datalen(rx_len, tx_len);

        // enable the MISO and MOSI if needed
        self.regs()
            .user()
            .modify(|_, w| w.usr_miso().bit(rx_len > 0).usr_mosi().bit(tx_len > 0));

        self.enable_dma();

        if rx_len > 0 {
            unsafe {
                channel
                    .rx
                    .prepare_transfer(self.dma_peripheral, rx_buffer)
                    .and_then(|_| channel.rx.start_transfer())?;
            }
        } else {
            #[cfg(esp32)]
            {
                // see https://github.com/espressif/esp-idf/commit/366e4397e9dae9d93fe69ea9d389b5743295886f
                // see https://github.com/espressif/esp-idf/commit/0c3653b1fd7151001143451d4aa95dbf15ee8506
                if _full_duplex {
                    self.regs()
                        .dma_in_link()
                        .modify(|_, w| unsafe { w.inlink_addr().bits(0) });
                    self.regs()
                        .dma_in_link()
                        .modify(|_, w| w.inlink_start().set_bit());
                }
            }
        }
        if tx_len > 0 {
            unsafe {
                channel
                    .tx
                    .prepare_transfer(self.dma_peripheral, tx_buffer)
                    .and_then(|_| channel.tx.start_transfer())?;
            }
        }

        #[cfg(gdma)]
        self.reset_dma();

        self.driver.start_operation();

        Ok(())
    }

    fn enable_dma(&self) {
        #[cfg(gdma)]
        // for non GDMA this is done in `assign_tx_device` / `assign_rx_device`
        self.regs().dma_conf().modify(|_, w| {
            w.dma_tx_ena().set_bit();
            w.dma_rx_ena().set_bit()
        });

        #[cfg(pdma)]
        self.reset_dma();
    }

    fn reset_dma(&self) {
        fn set_reset_bit(reg_block: &RegisterBlock, bit: bool) {
            #[cfg(pdma)]
            reg_block.dma_conf().modify(|_, w| {
                w.out_rst().bit(bit);
                w.in_rst().bit(bit);
                w.ahbm_fifo_rst().bit(bit);
                w.ahbm_rst().bit(bit)
            });
            #[cfg(gdma)]
            reg_block.dma_conf().modify(|_, w| {
                w.rx_afifo_rst().bit(bit);
                w.buf_afifo_rst().bit(bit);
                w.dma_afifo_rst().bit(bit)
            });
        }

        set_reset_bit(self.regs(), true);
        set_reset_bit(self.regs(), false);
        self.clear_dma_interrupts();
    }

    #[cfg(gdma)]
    fn clear_dma_interrupts(&self) {
        self.regs().dma_int_clr().write(|w| {
            w.dma_infifo_full_err().clear_bit_by_one();
            w.dma_outfifo_empty_err().clear_bit_by_one();
            w.trans_done().clear_bit_by_one();
            w.mst_rx_afifo_wfull_err().clear_bit_by_one();
            w.mst_tx_afifo_rempty_err().clear_bit_by_one()
        });
    }

    #[cfg(pdma)]
    fn clear_dma_interrupts(&self) {
        self.regs().dma_int_clr().write(|w| {
            w.inlink_dscr_empty().clear_bit_by_one();
            w.outlink_dscr_error().clear_bit_by_one();
            w.inlink_dscr_error().clear_bit_by_one();
            w.in_done().clear_bit_by_one();
            w.in_err_eof().clear_bit_by_one();
            w.in_suc_eof().clear_bit_by_one();
            w.out_done().clear_bit_by_one();
            w.out_eof().clear_bit_by_one();
            w.out_total_eof().clear_bit_by_one()
        });
    }
}

struct Driver {
    info: &'static Info,
    state: &'static State,
}

// private implementation bits
// FIXME: split this up into peripheral versions
impl Driver {
    /// Returns the register block for this SPI instance.
    pub fn regs(&self) -> &RegisterBlock {
        unsafe { &*self.info.register_block }
    }

    fn abort_transfer(&self) {
        // Note(danielb): This method came later than DmaDriver::abort_transfer. That
        // function works for DMA so I have left it unchanged, but does not work
        // for CPU-controlled transfers on the ESP32. Toggling slave mode works on
        // ESP32, but not on later chips.
        cfg_if::cfg_if! {
            if #[cfg(esp32)] {
                self.regs().slave().modify(|_, w| w.mode().set_bit());
                self.regs().slave().modify(|_, w| w.mode().clear_bit());
            } else {
                self.configure_datalen(1, 1);
            }
        }
        self.update();
    }

    /// Initialize for full-duplex 1 bit mode
    fn init(&self) {
        self.regs().user().modify(|_, w| {
            w.usr_miso_highpart().clear_bit();
            w.usr_mosi_highpart().clear_bit();
            w.doutdin().set_bit();
            w.usr_miso().set_bit();
            w.usr_mosi().set_bit();
            w.cs_hold().set_bit();
            w.usr_dummy_idle().set_bit();
            w.usr_addr().clear_bit();
            w.usr_command().clear_bit()
        });

        #[cfg(gdma)]
        self.regs().clk_gate().modify(|_, w| {
            w.clk_en().set_bit();
            w.mst_clk_active().set_bit();
            w.mst_clk_sel().set_bit()
        });

        #[cfg(any(esp32c6, esp32h2))]
        unsafe {
            // use default clock source PLL_F80M_CLK (ESP32-C6) and
            // PLL_F48M_CLK (ESP32-H2)
            crate::peripherals::PCR::regs()
                .spi2_clkm_conf()
                .modify(|_, w| w.spi2_clkm_sel().bits(1));
        }

        #[cfg(gdma)]
        self.regs().ctrl().modify(|_, w| {
            w.q_pol().clear_bit();
            w.d_pol().clear_bit();
            w.hold_pol().clear_bit()
        });

        #[cfg(esp32s2)]
        self.regs().ctrl().modify(|_, w| {
            w.q_pol().clear_bit();
            w.d_pol().clear_bit();
            w.wp().clear_bit()
        });

        #[cfg(esp32)]
        self.regs().ctrl().modify(|_, w| w.wp().clear_bit());

        #[cfg(not(esp32))]
        self.regs().misc().write(|w| unsafe { w.bits(0) });

        self.regs().slave().write(|w| unsafe { w.bits(0) });
    }

    #[cfg(not(esp32))]
    fn init_spi_data_mode(
        &self,
        cmd_mode: DataMode,
        address_mode: DataMode,
        data_mode: DataMode,
    ) -> Result<(), Error> {
        self.regs().ctrl().modify(|_, w| {
            w.fcmd_dual().bit(cmd_mode == DataMode::Dual);
            w.fcmd_quad().bit(cmd_mode == DataMode::Quad);
            w.faddr_dual().bit(address_mode == DataMode::Dual);
            w.faddr_quad().bit(address_mode == DataMode::Quad);
            w.fread_dual().bit(data_mode == DataMode::Dual);
            w.fread_quad().bit(data_mode == DataMode::Quad)
        });
        self.regs().user().modify(|_, w| {
            w.fwrite_dual().bit(data_mode == DataMode::Dual);
            w.fwrite_quad().bit(data_mode == DataMode::Quad)
        });
        Ok(())
    }

    #[cfg(esp32)]
    fn init_spi_data_mode(
        &self,
        cmd_mode: DataMode,
        address_mode: DataMode,
        data_mode: DataMode,
    ) -> Result<(), Error> {
        match cmd_mode {
            DataMode::Single => (),
            DataMode::SingleTwoDataLines => (),
            // FIXME: more detailed error - Only 1-bit commands are supported.
            _ => {
                error!("Commands must be single bit wide");
                return Err(Error::Unsupported);
            }
        }

        match address_mode {
            DataMode::Single | DataMode::SingleTwoDataLines => {
                self.regs().ctrl().modify(|_, w| {
                    w.fastrd_mode()
                        .bit(matches!(data_mode, DataMode::Dual | DataMode::Quad));
                    w.fread_dio().clear_bit();
                    w.fread_qio().clear_bit();
                    w.fread_dual().bit(data_mode == DataMode::Dual);
                    w.fread_quad().bit(data_mode == DataMode::Quad)
                });

                self.regs().user().modify(|_, w| {
                    w.fwrite_dio().clear_bit();
                    w.fwrite_qio().clear_bit();
                    w.fwrite_dual().bit(data_mode == DataMode::Dual);
                    w.fwrite_quad().bit(data_mode == DataMode::Quad)
                });
            }
            address_mode if address_mode == data_mode => {
                self.regs().ctrl().modify(|_, w| {
                    w.fastrd_mode()
                        .bit(matches!(data_mode, DataMode::Dual | DataMode::Quad));
                    w.fread_dio().bit(address_mode == DataMode::Dual);
                    w.fread_qio().bit(address_mode == DataMode::Quad);
                    w.fread_dual().clear_bit();
                    w.fread_quad().clear_bit()
                });

                self.regs().user().modify(|_, w| {
                    w.fwrite_dio().bit(address_mode == DataMode::Dual);
                    w.fwrite_qio().bit(address_mode == DataMode::Quad);
                    w.fwrite_dual().clear_bit();
                    w.fwrite_quad().clear_bit()
                });
            }
            _ => {
                // FIXME: more detailed error - Unsupported combination of data-modes
                error!("Address mode must be single bit wide or equal to the data mode");
                return Err(Error::Unsupported);
            }
        }

        Ok(())
    }

    /// Enable or disable listening for the given interrupts.
    #[cfg_attr(not(feature = "unstable"), allow(dead_code))]
    fn enable_listen(&self, interrupts: EnumSet<SpiInterrupt>, enable: bool) {
        cfg_if::cfg_if! {
            if #[cfg(esp32)] {
                self.regs().slave().modify(|_, w| {
                    for interrupt in interrupts {
                        match interrupt {
                            SpiInterrupt::TransferDone => w.trans_inten().bit(enable),
                        };
                    }
                    w
                });
            } else if #[cfg(esp32s2)] {
                self.regs().slave().modify(|_, w| {
                    for interrupt in interrupts {
                        match interrupt {
                            SpiInterrupt::TransferDone => w.int_trans_done_en().bit(enable),
                            SpiInterrupt::DmaSegmentedTransferDone => w.int_dma_seg_trans_en().bit(enable),
                        };
                    }
                    w
                });
            } else {
                self.regs().dma_int_ena().modify(|_, w| {
                    for interrupt in interrupts {
                        match interrupt {
                            SpiInterrupt::TransferDone => w.trans_done().bit(enable),
                            SpiInterrupt::DmaSegmentedTransferDone => w.dma_seg_trans_done().bit(enable),
                            SpiInterrupt::App2 => w.app2().bit(enable),
                            SpiInterrupt::App1 => w.app1().bit(enable),
                        };
                    }
                    w
                });
            }
        }
    }

    /// Gets asserted interrupts
    #[cfg_attr(not(feature = "unstable"), allow(dead_code))]
    fn interrupts(&self) -> EnumSet<SpiInterrupt> {
        let mut res = EnumSet::new();

        cfg_if::cfg_if! {
            if #[cfg(esp32)] {
                if self.regs().slave().read().trans_done().bit() {
                    res.insert(SpiInterrupt::TransferDone);
                }
            } else if #[cfg(esp32s2)] {
                if self.regs().slave().read().trans_done().bit() {
                    res.insert(SpiInterrupt::TransferDone);
                }
                if self.regs().hold().read().dma_seg_trans_done().bit() {
                    res.insert(SpiInterrupt::DmaSegmentedTransferDone);
                }
            } else {
                let ints = self.regs().dma_int_raw().read();

                if ints.trans_done().bit() {
                    res.insert(SpiInterrupt::TransferDone);
                }
                if ints.dma_seg_trans_done().bit() {
                    res.insert(SpiInterrupt::DmaSegmentedTransferDone);
                }
                if ints.app2().bit() {
                    res.insert(SpiInterrupt::App2);
                }
                if ints.app1().bit() {
                    res.insert(SpiInterrupt::App1);
                }
            }
        }

        res
    }

    /// Resets asserted interrupts
    #[cfg_attr(not(feature = "unstable"), allow(dead_code))]
    fn clear_interrupts(&self, interrupts: EnumSet<SpiInterrupt>) {
        cfg_if::cfg_if! {
            if #[cfg(esp32)] {
                for interrupt in interrupts {
                    match interrupt {
                        SpiInterrupt::TransferDone => {
                            self.regs().slave().modify(|_, w| w.trans_done().clear_bit());
                        }
                    }
                }
            } else if #[cfg(esp32s2)] {
                for interrupt in interrupts {
                    match interrupt {
                        SpiInterrupt::TransferDone => {
                            self.regs().slave().modify(|_, w| w.trans_done().clear_bit());
                        }

                        SpiInterrupt::DmaSegmentedTransferDone => {
                            self.regs()
                                .hold()
                                .modify(|_, w| w.dma_seg_trans_done().clear_bit());
                        }
                    }
                }
            } else {
                self.regs().dma_int_clr().write(|w| {
                    for interrupt in interrupts {
                        match interrupt {
                            SpiInterrupt::TransferDone => w.trans_done().clear_bit_by_one(),
                            SpiInterrupt::DmaSegmentedTransferDone => w.dma_seg_trans_done().clear_bit_by_one(),
                            SpiInterrupt::App2 => w.app2().clear_bit_by_one(),
                            SpiInterrupt::App1 => w.app1().clear_bit_by_one(),
                        };
                    }
                    w
                });
            }
        }
    }

    fn apply_config(&self, config: &Config) -> Result<(), ConfigError> {
        config.validate()?;
        self.ch_bus_freq(config)?;
        self.set_bit_order(config.read_bit_order, config.write_bit_order);
        self.set_data_mode(config.mode);

        #[cfg(esp32)]
        self.calculate_half_duplex_values(config);

        Ok(())
    }

    #[cfg(esp32)]
    fn calculate_half_duplex_values(&self, config: &Config) {
        let f_apb = 80_000_000;
        let source_freq_hz = match config.clock_source {
            ClockSource::Apb => f_apb,
        };

        let clock_reg = self.regs().clock().read();
        let eff_clk = if clock_reg.clk_equ_sysclk().bit_is_set() {
            f_apb
        } else {
            let pre = clock_reg.clkdiv_pre().bits() as i32 + 1;
            let n = clock_reg.clkcnt_n().bits() as i32 + 1;
            f_apb / (pre * n)
        };

        let apbclk_khz = source_freq_hz / 1000;
        // how many apb clocks a period has
        let spiclk_apb_n = source_freq_hz / eff_clk;

        // How many apb clocks the delay is, the 1 is to compensate in case
        // ``input_delay_ns`` is rounded off. Change from esp-idf: we use
        // `input_delay_ns` to also represent the GPIO matrix delay.
        let input_delay_ns = 25; // TODO: allow configuring input delay.
        let delay_apb_n = (1 + input_delay_ns) * apbclk_khz / 1000 / 1000;

        let dummy_required = delay_apb_n / spiclk_apb_n;
        let timing_miso_delay = if dummy_required > 0 {
            // due to the clock delay between master and slave, there's a range in which
            // data is random give MISO a delay if needed to make sure we
            // sample at the time MISO is stable
            Some(((dummy_required + 1) * spiclk_apb_n - delay_apb_n - 1) as u8)
        } else if delay_apb_n * 4 <= spiclk_apb_n {
            // if the dummy is not required, maybe we should also delay half a SPI clock if
            // the data comes too early
            None
        } else {
            Some(0)
        };

        self.state.esp32_hack.extra_dummy.set(dummy_required as u8);
        self.state
            .esp32_hack
            .timing_miso_delay
            .set(timing_miso_delay);
    }

    fn set_data_mode(&self, data_mode: Mode) {
        cfg_if::cfg_if! {
            if #[cfg(esp32)] {
                let pin_reg = self.regs().pin();
            } else {
                let pin_reg = self.regs().misc();
            }
        };

        pin_reg.modify(|_, w| {
            w.ck_idle_edge()
                .bit(matches!(data_mode, Mode::_2 | Mode::_3))
        });
        self.regs().user().modify(|_, w| {
            w.ck_out_edge()
                .bit(matches!(data_mode, Mode::_1 | Mode::_2))
        });
    }

    fn ch_bus_freq(&self, bus_clock_config: &Config) -> Result<(), ConfigError> {
        fn enable_clocks(_reg_block: &RegisterBlock, _enable: bool) {
            #[cfg(gdma)]
            _reg_block.clk_gate().modify(|_, w| {
                w.clk_en().bit(_enable);
                w.mst_clk_active().bit(_enable);
                w.mst_clk_sel().bit(true) // TODO: support XTAL clock source
            });
        }

        // Change clock frequency
        let raw = bus_clock_config.raw_clock_reg_value()?;

        enable_clocks(self.regs(), false);
        self.regs().clock().write(|w| unsafe { w.bits(raw) });
        enable_clocks(self.regs(), true);

        Ok(())
    }

    #[cfg(not(any(esp32, esp32c3, esp32s2)))]
    fn set_bit_order(&self, read_order: BitOrder, write_order: BitOrder) {
        let read_value = match read_order {
            BitOrder::MsbFirst => 0,
            BitOrder::LsbFirst => 1,
        };
        let write_value = match write_order {
            BitOrder::MsbFirst => 0,
            BitOrder::LsbFirst => 1,
        };
        self.regs().ctrl().modify(|_, w| unsafe {
            w.rd_bit_order().bits(read_value);
            w.wr_bit_order().bits(write_value);
            w
        });
    }

    #[cfg(any(esp32, esp32c3, esp32s2))]
    fn set_bit_order(&self, read_order: BitOrder, write_order: BitOrder) {
        let read_value = match read_order {
            BitOrder::MsbFirst => false,
            BitOrder::LsbFirst => true,
        };
        let write_value = match write_order {
            BitOrder::MsbFirst => false,
            BitOrder::LsbFirst => true,
        };
        self.regs().ctrl().modify(|_, w| {
            w.rd_bit_order().bit(read_value);
            w.wr_bit_order().bit(write_value);
            w
        });
    }

    #[cfg_attr(place_spi_driver_in_ram, ram)]
    fn fill_fifo(&self, chunk: &[u8]) {
        // TODO: replace with `array_chunks` and `from_le_bytes`
        let mut c_iter = chunk.chunks_exact(4);
        let mut w_iter = self.regs().w_iter();
        for c in c_iter.by_ref() {
            if let Some(w_reg) = w_iter.next() {
                let word = (c[0] as u32)
                    | ((c[1] as u32) << 8)
                    | ((c[2] as u32) << 16)
                    | ((c[3] as u32) << 24);
                w_reg.write(|w| w.buf().set(word));
            }
        }
        let rem = c_iter.remainder();
        if !rem.is_empty() {
            if let Some(w_reg) = w_iter.next() {
                let word = match rem.len() {
                    3 => (rem[0] as u32) | ((rem[1] as u32) << 8) | ((rem[2] as u32) << 16),
                    2 => (rem[0] as u32) | ((rem[1] as u32) << 8),
                    1 => rem[0] as u32,
                    _ => unreachable!(),
                };
                w_reg.write(|w| w.buf().set(word));
            }
        }
    }

    /// Write bytes to SPI.
    ///
    /// This function will return before all bytes of the last chunk to transmit
    /// have been sent to the wire. If you must ensure that the whole
    /// messages was written correctly, use [`Self::flush`].
    #[cfg_attr(place_spi_driver_in_ram, ram)]
    fn write(&self, words: &[u8]) -> Result<(), Error> {
        let num_chunks = words.len() / FIFO_SIZE;

        // Flush in case previous writes have not completed yet, required as per
        // embedded-hal documentation (#1369).
        self.flush()?;

        // The fifo has a limited fixed size, so the data must be chunked and then
        // transmitted
        for (i, chunk) in words.chunks(FIFO_SIZE).enumerate() {
            self.configure_datalen(0, chunk.len());
            self.fill_fifo(chunk);

            self.start_operation();

            // Wait for all chunks to complete except the last one.
            // The function is allowed to return before the bus is idle.
            // see [embedded-hal flushing](https://docs.rs/embedded-hal/1.0.0/embedded_hal/spi/index.html#flushing)
            if i < num_chunks {
                self.flush()?;
            }
        }
        Ok(())
    }

    /// Write bytes to SPI.
    #[cfg_attr(place_spi_driver_in_ram, ram)]
    async fn write_async(&self, words: &[u8]) -> Result<(), Error> {
        // The fifo has a limited fixed size, so the data must be chunked and then
        // transmitted
        for chunk in words.chunks(FIFO_SIZE) {
            self.configure_datalen(0, chunk.len());
            self.fill_fifo(chunk);
            self.execute_operation_async().await;
        }
        Ok(())
    }

    /// Read bytes from SPI.
    ///
    /// Sends out a stuffing byte for every byte to read. This function doesn't
    /// perform flushing. If you want to read the response to something you
    /// have written before, consider using [`Self::transfer`] instead.
    #[cfg_attr(place_spi_driver_in_ram, ram)]
    fn read(&self, words: &mut [u8]) -> Result<(), Error> {
        let empty_array = [EMPTY_WRITE_PAD; FIFO_SIZE];

        for chunk in words.chunks_mut(FIFO_SIZE) {
            self.write(&empty_array[0..chunk.len()])?;
            self.flush()?;
            self.read_from_fifo(chunk)?;
        }
        Ok(())
    }

    /// Read bytes from SPI.
    ///
    /// Sends out a stuffing byte for every byte to read. This function doesn't
    /// perform flushing. If you want to read the response to something you
    /// have written before, consider using [`Self::transfer`] instead.
    #[cfg_attr(place_spi_driver_in_ram, ram)]
    async fn read_async(&self, words: &mut [u8]) -> Result<(), Error> {
        let empty_array = [EMPTY_WRITE_PAD; FIFO_SIZE];

        for chunk in words.chunks_mut(FIFO_SIZE) {
            self.write_async(&empty_array[0..chunk.len()]).await?;
            self.read_from_fifo(chunk)?;
        }
        Ok(())
    }

    /// Read received bytes from SPI FIFO.
    ///
    /// Copies the contents of the SPI receive FIFO into `words`. This function
    /// doesn't perform flushing. If you want to read the response to
    /// something you have written before, consider using [`Self::transfer`]
    /// instead.
    #[cfg_attr(place_spi_driver_in_ram, ram)]
    fn read_from_fifo(&self, words: &mut [u8]) -> Result<(), Error> {
        let reg_block = self.regs();

        for chunk in words.chunks_mut(FIFO_SIZE) {
            self.configure_datalen(chunk.len(), 0);

            for (index, w_reg) in (0..chunk.len()).step_by(4).zip(reg_block.w_iter()) {
                let reg_val = w_reg.read().bits();
                let bytes = reg_val.to_le_bytes();

                let len = usize::min(chunk.len(), index + 4) - index;
                chunk[index..(index + len)].clone_from_slice(&bytes[0..len]);
            }
        }

        Ok(())
    }

    fn busy(&self) -> bool {
        self.regs().cmd().read().usr().bit_is_set()
    }

    // Check if the bus is busy and if it is wait for it to be idle
    fn flush(&self) -> Result<(), Error> {
        while self.busy() {
            // wait for bus to be clear
        }
        Ok(())
    }

    #[cfg_attr(place_spi_driver_in_ram, ram)]
    fn transfer<'w>(&self, words: &'w mut [u8]) -> Result<&'w [u8], Error> {
        for chunk in words.chunks_mut(FIFO_SIZE) {
            self.write(chunk)?;
            self.flush()?;
            self.read_from_fifo(chunk)?;
        }

        Ok(words)
    }

    #[cfg_attr(place_spi_driver_in_ram, ram)]
    async fn transfer_in_place_async(&self, words: &mut [u8]) -> Result<(), Error> {
        for chunk in words.chunks_mut(FIFO_SIZE) {
            // Cut the transfer short if the future is dropped. We'll block for a short
            // while to ensure the peripheral is idle.
            let cancel_on_drop = OnDrop::new(|| {
                self.abort_transfer();
                while self.busy() {}
            });
            let res = self.write_async(chunk).await;
            cancel_on_drop.defuse();
            res?;

            self.read_from_fifo(chunk)?;
        }

        Ok(())
    }

    fn start_operation(&self) {
        self.update();
        self.regs().cmd().modify(|_, w| w.usr().set_bit());
    }

    /// Starts the operation and waits for it to complete.
    async fn execute_operation_async(&self) {
        // On ESP32, the interrupt seems to not fire in specific circumstances, when
        // `listen` is called after `start_operation`. Let's call it before, to be sure.
        let future = SpiFuture::setup(self).await;
        self.start_operation();
        future.await;
    }

    fn setup_full_duplex(&self) -> Result<(), Error> {
        self.regs().user().modify(|_, w| {
            w.usr_miso().set_bit();
            w.usr_mosi().set_bit();
            w.doutdin().set_bit();
            w.usr_dummy().clear_bit();
            w.sio().clear_bit()
        });

        self.init_spi_data_mode(
            DataMode::SingleTwoDataLines,
            DataMode::SingleTwoDataLines,
            DataMode::SingleTwoDataLines,
        )?;

        // For full-duplex, we don't need compensation according to esp-idf.
        #[cfg(esp32)]
        self.regs().ctrl2().modify(|_, w| unsafe {
            w.miso_delay_mode().bits(0);
            w.miso_delay_num().bits(0)
        });

        Ok(())
    }

    #[allow(clippy::too_many_arguments)]
    fn setup_half_duplex(
        &self,
        is_write: bool,
        cmd: Command,
        address: Address,
        dummy_idle: bool,
        dummy: u8,
        no_mosi_miso: bool,
        data_mode: DataMode,
    ) -> Result<(), Error> {
        self.init_spi_data_mode(cmd.mode(), address.mode(), data_mode)?;

        #[cfg(esp32)]
        let mut dummy = dummy;

        #[cfg(esp32)]
        self.regs().ctrl2().modify(|_, w| {
            let mut delay_mode = 0;
            let mut delay_num = 0;

            if !is_write {
                // Values are set up in apply_config
                let timing_miso_delay = self.state.esp32_hack.timing_miso_delay.get();
                let extra_dummy = self.state.esp32_hack.extra_dummy.get();
                dummy += extra_dummy;

                if let Some(delay) = timing_miso_delay {
                    delay_num = if extra_dummy > 0 { delay } else { 0 };
                } else {
                    let out_edge = self.regs().user().read().ck_out_edge().bit_is_set();
                    // SPI modes 1 and 2 need delay mode 1 according to esp-idf.
                    delay_mode = if out_edge { 1 } else { 2 };
                }
            }

            unsafe {
                w.miso_delay_mode().bits(delay_mode);
                w.miso_delay_num().bits(delay_num)
            }
        });

        let reg_block = self.regs();
        reg_block.user().modify(|_, w| {
            w.usr_miso_highpart().clear_bit();
            w.usr_mosi_highpart().clear_bit();
            // This bit tells the hardware whether we use Single or SingleTwoDataLines
            w.sio().bit(data_mode == DataMode::Single);
            w.doutdin().clear_bit();
            w.usr_miso().bit(!is_write && !no_mosi_miso);
            w.usr_mosi().bit(is_write && !no_mosi_miso);
            w.cs_hold().set_bit();
            w.usr_dummy_idle().bit(dummy_idle);
            w.usr_dummy().bit(dummy != 0);
            w.usr_addr().bit(!address.is_none());
            w.usr_command().bit(!cmd.is_none())
        });

        #[cfg(gdma)]
        reg_block.clk_gate().modify(|_, w| {
            w.clk_en().set_bit();
            w.mst_clk_active().set_bit();
            w.mst_clk_sel().set_bit()
        });

        #[cfg(any(esp32c6, esp32h2))]
        // use default clock source PLL_F80M_CLK
        crate::peripherals::PCR::regs()
            .spi2_clkm_conf()
            .modify(|_, w| unsafe { w.spi2_clkm_sel().bits(1) });

        #[cfg(not(esp32))]
        reg_block.misc().write(|w| unsafe { w.bits(0) });

        reg_block.slave().write(|w| unsafe { w.bits(0) });

        self.update();

        // set cmd, address, dummy cycles
        self.set_up_common_phases(cmd, address, dummy);

        Ok(())
    }

    fn set_up_common_phases(&self, cmd: Command, address: Address, dummy: u8) {
        let reg_block = self.regs();
        if !cmd.is_none() {
            reg_block.user2().modify(|_, w| unsafe {
                w.usr_command_bitlen().bits((cmd.width() - 1) as u8);
                w.usr_command_value().bits(cmd.value())
            });
        }

        if !address.is_none() {
            reg_block
                .user1()
                .modify(|_, w| unsafe { w.usr_addr_bitlen().bits((address.width() - 1) as u8) });

            let addr = address.value() << (32 - address.width());
            #[cfg(not(esp32))]
            reg_block
                .addr()
                .write(|w| unsafe { w.usr_addr_value().bits(addr) });
            #[cfg(esp32)]
            reg_block.addr().write(|w| unsafe { w.bits(addr) });
        }

        if dummy > 0 {
            reg_block
                .user1()
                .modify(|_, w| unsafe { w.usr_dummy_cyclelen().bits(dummy - 1) });
        }
    }

    fn update(&self) {
        cfg_if::cfg_if! {
            if #[cfg(gdma)] {
                let reg_block = self.regs();

                reg_block.cmd().modify(|_, w| w.update().set_bit());

                while reg_block.cmd().read().update().bit_is_set() {
                    // wait
                }
            } else {
                // Doesn't seem to be needed for ESP32 and ESP32-S2
            }
        }
    }

    fn configure_datalen(&self, rx_len_bytes: usize, tx_len_bytes: usize) {
        let reg_block = self.regs();

        let rx_len = rx_len_bytes as u32 * 8;
        let tx_len = tx_len_bytes as u32 * 8;

        let rx_len = rx_len.saturating_sub(1);
        let tx_len = tx_len.saturating_sub(1);

        cfg_if::cfg_if! {
            if #[cfg(esp32)] {
                let len = rx_len.max(tx_len);
                reg_block
                    .mosi_dlen()
                    .write(|w| unsafe { w.usr_mosi_dbitlen().bits(len) });

                reg_block
                    .miso_dlen()
                    .write(|w| unsafe { w.usr_miso_dbitlen().bits(len) });
            } else if #[cfg(esp32s2)] {
                reg_block
                    .mosi_dlen()
                    .write(|w| unsafe { w.usr_mosi_dbitlen().bits(tx_len) });

                reg_block
                    .miso_dlen()
                    .write(|w| unsafe { w.usr_miso_dbitlen().bits(rx_len) });
            } else {
                reg_block
                    .ms_dlen()
                    .write(|w| unsafe { w.ms_data_bitlen().bits(rx_len.max(tx_len)) });
            }

        }
    }
}

impl PartialEq for Info {
    fn eq(&self, other: &Self) -> bool {
        core::ptr::eq(self.register_block, other.register_block)
    }
}

unsafe impl Sync for Info {}

// TODO: this macro needs to move to one level up, and it needs to describe the
// hardware fully. The master module should extend it with the master specific
// details.
macro_rules! spi_instance {
    ($num:literal, $sclk:ident, $mosi:ident, $miso:ident, [$($cs:ident),+] $(, $sio2:ident, $sio3:ident $(, $sio4:ident, $sio5:ident, $sio6:ident, $sio7:ident)?)?) => {
        paste::paste! {
            impl PeripheralInstance for crate::peripherals::[<SPI $num>]<'_> {
                #[inline(always)]
                fn info(&self) -> &'static Info {
                    static INFO: Info = Info {
                        register_block: crate::peripherals::[<SPI $num>]::regs(),
                        peripheral: crate::system::Peripheral::[<Spi $num>],
                        interrupt: crate::peripherals::Interrupt::[<SPI $num>],
                        sclk: OutputSignal::$sclk,
                        mosi: OutputSignal::$mosi,
                        miso: InputSignal::$miso,
                        cs: &[$(OutputSignal::$cs),+],
                        sio0_input: InputSignal::$mosi,
                        sio1_output: OutputSignal::$miso,
                        sio2_output: $crate::if_set!($(Some(OutputSignal::$sio2))?, None),
                        sio2_input: $crate::if_set!($(Some(InputSignal::$sio2))?, None),
                        sio3_output: $crate::if_set!($(Some(OutputSignal::$sio3))?, None),
                        sio3_input: $crate::if_set!($(Some(InputSignal::$sio3))?, None),
                        #[cfg(spi_octal)]
                        sio4_output: $crate::if_set!($($(Some(OutputSignal::$sio4))?)?, None),
                        #[cfg(spi_octal)]
                        sio4_input: $crate::if_set!($($(Some(InputSignal::$sio4))?)?, None),
                        #[cfg(spi_octal)]
                        sio5_output: $crate::if_set!($($(Some(OutputSignal::$sio5))?)?, None),
                        #[cfg(spi_octal)]
                        sio5_input: $crate::if_set!($($(Some(InputSignal::$sio5))?)?, None),
                        #[cfg(spi_octal)]
                        sio6_output: $crate::if_set!($($(Some(OutputSignal::$sio6))?)?, None),
                        #[cfg(spi_octal)]
                        sio6_input: $crate::if_set!($($(Some(InputSignal::$sio6))?)?, None),
                        #[cfg(spi_octal)]
                        sio7_output: $crate::if_set!($($(Some(OutputSignal::$sio7))?)?, None),
                        #[cfg(spi_octal)]
                        sio7_input: $crate::if_set!($($(Some(InputSignal::$sio7))?)?, None),
                    };

                    &INFO
                }
            }

            $(
                // If the extra pins are set, implement QspiInstance
                $crate::ignore!($sio2);
                impl QspiInstance for crate::peripherals::[<SPI $num>]<'_> {}
            )?
        }
    }
}

#[cfg(spi2)]
cfg_if::cfg_if! {
    if #[cfg(esp32)] {
        spi_instance!(2, HSPICLK, HSPID, HSPIQ, [HSPICS0, HSPICS1, HSPICS2], HSPIWP, HSPIHD);
    } else if #[cfg(any(esp32s2, esp32s3))] {
        spi_instance!(2, FSPICLK, FSPID, FSPIQ, [FSPICS0, FSPICS1, FSPICS2, FSPICS3, FSPICS4, FSPICS5], FSPIWP, FSPIHD, FSPIIO4, FSPIIO5, FSPIIO6, FSPIIO7);
    } else {
        spi_instance!(2, FSPICLK_MUX, FSPID, FSPIQ, [FSPICS0, FSPICS1, FSPICS2, FSPICS3, FSPICS4, FSPICS5], FSPIWP, FSPIHD);
    }
}

#[cfg(spi3)]
cfg_if::cfg_if! {
    if #[cfg(esp32)] {
        spi_instance!(3, VSPICLK, VSPID, VSPIQ, [VSPICS0, VSPICS1, VSPICS2], VSPIWP, VSPIHD);
    } else if #[cfg(esp32s3)] {
        spi_instance!(3, SPI3_CLK, SPI3_D, SPI3_Q, [SPI3_CS0, SPI3_CS1, SPI3_CS2], SPI3_WP, SPI3_HD);
    } else {
        spi_instance!(3, SPI3_CLK, SPI3_D, SPI3_Q, [SPI3_CS0, SPI3_CS1, SPI3_CS2]);
    }
}

impl PeripheralInstance for super::AnySpi<'_> {
    delegate::delegate! {
        to match &self.0 {
            super::AnySpiInner::Spi2(spi) => spi,
            #[cfg(spi3)]
            super::AnySpiInner::Spi3(spi) => spi,
        } {
            fn info(&self) -> &'static Info;
        }
    }
}

impl QspiInstance for super::AnySpi<'_> {}

#[doc(hidden)]
pub struct State {
    waker: AtomicWaker,

    #[cfg(esp32)]
    esp32_hack: Esp32Hack,
}

#[cfg(esp32)]
struct Esp32Hack {
    timing_miso_delay: Cell<Option<u8>>,
    extra_dummy: Cell<u8>,
}

#[cfg(esp32)]
unsafe impl Sync for Esp32Hack {}

#[cfg_attr(place_spi_driver_in_ram, ram)]
fn handle_async(instance: impl Instance) {
    let state = instance.state();
    let info = instance.info();

    let driver = Driver { info, state };
    if driver.interrupts().contains(SpiInterrupt::TransferDone) {
        driver.enable_listen(SpiInterrupt::TransferDone.into(), false);
        state.waker.wake();
    }
}

/// A peripheral singleton compatible with the SPI master driver.
pub trait Instance: PeripheralInstance + super::IntoAnySpi {
    #[doc(hidden)]
    fn state(&self) -> &'static State;
    #[doc(hidden)]
    fn handler(&self) -> InterruptHandler;
}

macro_rules! master_instance {
    ($peri:ident) => {
        impl Instance for $crate::peripherals::$peri<'_> {
            fn state(&self) -> &'static State {
                static STATE: State = State {
                    waker: AtomicWaker::new(),
                    #[cfg(esp32)]
                    esp32_hack: Esp32Hack {
                        timing_miso_delay: Cell::new(None),
                        extra_dummy: Cell::new(0),
                    },
                };

                &STATE
            }

            fn handler(&self) -> InterruptHandler {
                #[$crate::handler]
                #[cfg_attr(place_spi_driver_in_ram, ram)]
                fn handle() {
                    handle_async(unsafe { $crate::peripherals::$peri::steal() })
                }

                handle
            }
        }
    };
}

master_instance!(SPI2);
#[cfg(spi3)]
master_instance!(SPI3);

impl Instance for super::AnySpi<'_> {
    delegate::delegate! {
        to match &self.0 {
            super::AnySpiInner::Spi2(spi) => spi,
            #[cfg(spi3)]
            super::AnySpiInner::Spi3(spi) => spi,
        } {
            fn state(&self) -> &'static State;
            fn handler(&self) -> InterruptHandler;
        }
    }
}

struct SpiFuture<'a> {
    driver: &'a Driver,
}

impl<'a> SpiFuture<'a> {
    fn setup(driver: &'a Driver) -> impl Future<Output = Self> {
        // Make sure this is called before starting an async operation. On the ESP32,
        // calling after may cause the interrupt to not fire.
        core::future::poll_fn(move |cx| {
            driver.state.waker.register(cx.waker());
            driver.clear_interrupts(SpiInterrupt::TransferDone.into());
            driver.enable_listen(SpiInterrupt::TransferDone.into(), true);
            Poll::Ready(Self { driver })
        })
    }
}

use core::{
    future::Future,
    pin::Pin,
    task::{Context, Poll},
};

use crate::dma::{Channel, PeripheralDmaChannel};

impl Future for SpiFuture<'_> {
    type Output = ();

    fn poll(self: Pin<&mut Self>, _cx: &mut Context<'_>) -> Poll<Self::Output> {
        if self
            .driver
            .interrupts()
            .contains(SpiInterrupt::TransferDone)
        {
            self.driver
                .clear_interrupts(SpiInterrupt::TransferDone.into());
            return Poll::Ready(());
        }

        Poll::Pending
    }
}

impl Drop for SpiFuture<'_> {
    fn drop(&mut self) {
        self.driver
            .enable_listen(SpiInterrupt::TransferDone.into(), false);
    }
}
