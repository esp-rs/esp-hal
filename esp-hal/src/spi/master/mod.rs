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
//! - Use the [`SpiBus`] or [`SpiBusAsync`] trait and its associated functions to initiate
//!   transactions with simultaneous reads and writes, or
//! - Use the `ExclusiveDevice` struct from [`embedded-hal-bus`] or `SpiDevice` from
//!   [`embassy-embedded-hal`].
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

use core::{marker::PhantomData, sync::atomic::Ordering};

#[cfg(spi_master_supports_dma)]
mod dma;
mod low_level;

#[instability::unstable]
#[cfg(spi_master_supports_dma)]
pub use dma::*;
use embedded_hal::spi::SpiBus;
use embedded_hal_async::spi::SpiBus as SpiBusAsync;
use enumset::EnumSetType;
use low_level::{Driver, SpiWrapper};
pub use low_level::{Info, Instance, QspiInstance, State};
use procmacros::doc_replace;

use super::{BitOrder, Error, Mode};
use crate::{
    Async,
    Blocking,
    DriverMode,
    gpio::{
        InputConfig,
        NoPin,
        OutputConfig,
        OutputSignal,
        PinGuard,
        interconnect::{self, PeripheralInput, PeripheralOutput},
    },
    interrupt::InterruptHandler,
    private::Sealed,
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
    #[cfg(spi_master_has_dma_segmented_transfer)]
    DmaSegmentedTransferDone,

    /// Used and triggered by software. Only used for user defined function.
    #[cfg(spi_master_has_app_interrupts)]
    App2,

    /// Used and triggered by software. Only used for user defined function.
    #[cfg(spi_master_has_app_interrupts)]
    App1,
}

/// The size of the FIFO buffer for SPI
const FIFO_SIZE: usize = property!("spi_master.fifo_size");

/// Padding byte for empty write transfers
const EMPTY_WRITE_PAD: u8 = 0x00;

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

    /// Minimum transfer size in bytes below which CPU-driven (blocking) I/O
    /// is used instead of async or DMA transfers.
    ///
    /// This can reduce overhead for small transfers where DMA setup or
    /// async context-switch cost exceeds the benefit. For
    /// [`SpiDma`][crate::spi::master::dma::SpiDma], the threshold applies in
    /// both blocking and async DMA modes: when met, DMA is disabled and the
    /// transfer is performed by the CPU.
    ///
    /// A value of `0` (the default) disables the threshold — all transfers use
    /// the driver's default method.
    #[builder_lite(unstable)]
    min_async_transfer_size: usize,
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
            min_async_transfer_size: 0,
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
        // FIXME: take clock source into account
        Rate::from_hz(crate::soc::spi_master_clock_source_frequency())
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
            return Err(ConfigError::FrequencyOutOfRange);
        }

        Ok(())
    }
}

const SIO_PIN_COUNT: usize = 4 + cfg!(spi_master_has_octal) as usize * 4;

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
struct SpiPinGuard {
    sclk_pin: PinGuard,
    cs_pin: PinGuard,
    sio_pins: [PinGuard; SIO_PIN_COUNT],
}

impl SpiPinGuard {
    const fn new_unconnected() -> Self {
        Self {
            sclk_pin: PinGuard::new_unconnected(),
            cs_pin: PinGuard::new_unconnected(),
            sio_pins: [const { PinGuard::new_unconnected() }; SIO_PIN_COUNT],
        }
    }
}

/// Configuration errors.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ConfigError {
    /// The requested frequency is not in the supported range.
    FrequencyOutOfRange,
}

impl core::error::Error for ConfigError {}

impl core::fmt::Display for ConfigError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            ConfigError::FrequencyOutOfRange => {
                write!(f, "The requested frequency is not in the supported range")
            }
        }
    }
}

#[procmacros::doc_replace]
/// SPI peripheral driver
///
/// ## Example
///
/// ```rust, no_run
/// # {before_snippet}
/// use esp_hal::spi::{
///     Mode,
///     master::{Config, Spi},
/// };
/// let mut spi = Spi::new(
///     peripherals.SPI2,
///     Config::default()
///         .with_frequency(Rate::from_khz(100))
///         .with_mode(Mode::_0),
/// )?
/// .with_sck(peripherals.GPIO0)
/// .with_mosi(peripherals.GPIO1)
/// .with_miso(peripherals.GPIO2);
/// # {after_snippet}
/// ```
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Spi<'d, Dm: DriverMode> {
    spi: SpiWrapper<'d>,
    _mode: PhantomData<Dm>,
}

impl<Dm: DriverMode> Sealed for Spi<'_, Dm> {}

impl<'d> Spi<'d, Blocking> {
    #[procmacros::doc_replace]
    /// Constructs an SPI instance in 8bit dataframe mode.
    ///
    /// ## Errors
    ///
    /// See [`Spi::apply_config`].
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::spi::{
    ///     Mode,
    ///     master::{Config, Spi},
    /// };
    /// let mut spi = Spi::new(peripherals.SPI2, Config::default())?
    ///     .with_sck(peripherals.GPIO0)
    ///     .with_mosi(peripherals.GPIO1)
    ///     .with_miso(peripherals.GPIO2);
    /// # {after_snippet}
    /// ```
    pub fn new(spi: impl Instance + 'd, config: Config) -> Result<Self, ConfigError> {
        let mut this = Spi {
            _mode: PhantomData,
            spi: SpiWrapper::new(spi),
        };

        this.driver().init();
        this.apply_config(&config)?;

        let this = this.with_sck(NoPin).with_cs(NoPin);

        for sio in 0..8 {
            if let Some(signal) = this.driver().info.opt_sio_input(sio) {
                signal.connect_to(&NoPin);
            }
            if let Some(signal) = this.driver().info.opt_sio_output(sio) {
                signal.connect_to(&NoPin);
            }
        }

        Ok(this)
    }

    /// Reconfigures the driver to operate in [`Async`] mode.
    ///
    /// See the [`Async`] documentation for an example on how to use this
    /// method.
    pub fn into_async(mut self) -> Spi<'d, Async> {
        self.set_interrupt_handler(self.spi.info().async_handler);
        Spi {
            spi: self.spi,
            _mode: PhantomData,
        }
    }

    #[doc_replace(
        "peripheral_on" => {
            cfg(multi_core) => "peripheral on the current core",
            _ => "peripheral",
        }
    )]
    /// # Registers an interrupt handler for the __peripheral_on__.
    ///
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
        self.spi.set_interrupt_handler(handler);
    }
}

#[instability::unstable]
impl crate::interrupt::InterruptConfigurable for Spi<'_, Blocking> {
    /// Sets the interrupt handler
    ///
    /// Interrupts are not enabled at the peripheral level here.
    fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        self.set_interrupt_handler(handler);
    }
}

impl<'d> Spi<'d, Async> {
    /// Reconfigures the driver to operate in [`Blocking`] mode.
    ///
    /// See the [`Blocking`] documentation for an example on how to use this
    /// method.
    pub fn into_blocking(self) -> Spi<'d, Blocking> {
        self.spi.disable_peri_interrupt_on_all_cores();
        Spi {
            spi: self.spi,
            _mode: PhantomData,
        }
    }

    #[procmacros::doc_replace]
    /// Waits for the completion of previous operations.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::spi::{
    ///     Mode,
    ///     master::{Config, Spi},
    /// };
    /// let mut spi = Spi::new(peripherals.SPI2, Config::default())?
    ///     .with_sck(peripherals.GPIO0)
    ///     .with_mosi(peripherals.GPIO1)
    ///     .with_miso(peripherals.GPIO2)
    ///     .into_async();
    ///
    /// let mut buffer = [0; 10];
    /// spi.transfer_in_place_async(&mut buffer).await?;
    /// spi.flush_async().await?;
    ///
    /// # {after_snippet}
    /// ```
    pub async fn flush_async(&mut self) -> Result<(), Error> {
        self.driver().flush_async().await;
        Ok(())
    }

    #[procmacros::doc_replace]
    /// Sends `words` to the slave. Returns the `words` received from the slave.
    ///
    /// This function aborts the transfer when its Future is dropped. Some
    /// amount of data may have been transferred before the Future is
    /// dropped. Dropping the future may block for a short while to ensure
    /// the transfer is aborted.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::spi::{
    ///     Mode,
    ///     master::{Config, Spi},
    /// };
    /// let mut spi = Spi::new(peripherals.SPI2, Config::default())?
    ///     .with_sck(peripherals.GPIO0)
    ///     .with_mosi(peripherals.GPIO1)
    ///     .with_miso(peripherals.GPIO2)
    ///     .into_async();
    ///
    /// let mut buffer = [0; 10];
    /// spi.transfer_in_place_async(&mut buffer).await?;
    ///
    /// # {after_snippet}
    /// ```
    pub async fn transfer_in_place_async(&mut self, words: &mut [u8]) -> Result<(), Error> {
        // We need to flush because the blocking transfer functions may return while a
        // transfer is still in progress.
        self.driver().flush_async().await;
        self.driver().setup_full_duplex()?;

        if self.use_blocking_transfer(words.len()) {
            return self.driver().transfer_in_place(words);
        }

        self.driver().transfer_in_place_async(words).await
    }

    // TODO: These inherent methods should be public

    async fn read_async(&mut self, words: &mut [u8]) -> Result<(), Error> {
        // We need to flush because the blocking transfer functions may return while a
        // transfer is still in progress.
        self.driver().flush_async().await;
        self.driver().setup_full_duplex()?;

        if self.use_blocking_transfer(words.len()) {
            return self.driver().read(words);
        }

        self.driver().read_async(words).await
    }

    async fn write_async(&mut self, words: &[u8]) -> Result<(), Error> {
        // We need to flush because the blocking transfer functions may return while a
        // transfer is still in progress.
        self.driver().flush_async().await;
        self.driver().setup_full_duplex()?;

        if self.use_blocking_transfer(words.len()) {
            self.driver().write(words)?;
            return self.driver().flush();
        }

        self.driver().write_async(words).await
    }
}

macro_rules! def_with_sio_pin {
    ($fn:ident, $n:literal) => {
        #[doc = concat!(" Assign the SIO", stringify!($n), " pin for the SPI instance.")]
        #[doc = " "]
        #[doc = " Enables both input and output functionality for the pin, and connects it"]
        #[doc = concat!(" to the SIO", stringify!($n), " output and input signals.")]
        #[instability::unstable]
        pub fn $fn(mut self, sio: impl PeripheralInput<'d> + PeripheralOutput<'d>) -> Self {
            self.spi.pins().sio_pins[$n] = self.connect_sio_pin(sio.into(), $n);

            self
        }
    };
}

impl<'d, Dm> Spi<'d, Dm>
where
    Dm: DriverMode,
{
    fn connect_sio_pin(&self, pin: interconnect::OutputSignal<'d>, n: usize) -> PinGuard {
        let in_signal = self.spi.info().sio_input(n);
        let out_signal = self.spi.info().sio_output(n);

        pin.apply_input_config(&InputConfig::default());
        pin.apply_output_config(&OutputConfig::default());

        pin.set_input_enable(true);
        pin.set_output_enable(false);

        in_signal.connect_to(&pin);
        pin.connect_with_guard(out_signal)
    }

    fn connect_sio_output_pin(&self, pin: interconnect::OutputSignal<'d>, n: usize) -> PinGuard {
        let out_signal = self.spi.info().sio_output(n);

        self.connect_output_pin(pin, out_signal)
    }

    fn connect_output_pin(
        &self,
        pin: interconnect::OutputSignal<'d>,
        signal: OutputSignal,
    ) -> PinGuard {
        pin.apply_output_config(&OutputConfig::default());
        pin.set_output_enable(true); // TODO turn this bool into a Yes/No/PeripheralControl trio

        pin.connect_with_guard(signal)
    }

    #[procmacros::doc_replace]
    /// Assign the SCK (Serial Clock) pin for the SPI instance.
    ///
    /// Configures the specified pin to push-pull output and connects it to the
    /// SPI clock signal.
    ///
    /// Disconnects the previous pin that was assigned with `with_sck`.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::spi::{
    ///     Mode,
    ///     master::{Config, Spi},
    /// };
    /// let mut spi = Spi::new(peripherals.SPI2, Config::default())?.with_sck(peripherals.GPIO0);
    ///
    /// # {after_snippet}
    /// ```
    pub fn with_sck(mut self, sclk: impl PeripheralOutput<'d>) -> Self {
        let info = self.spi.info();
        self.spi.pins().sclk_pin = self.connect_output_pin(sclk.into(), info.sclk);

        self
    }

    #[procmacros::doc_replace]
    /// Assign the MOSI (Master Out Slave In) pin for the SPI instance.
    ///
    /// Enables output functionality for the pin, and connects it as the MOSI
    /// signal. You want to use this for full-duplex SPI or
    /// if you intend to use [DataMode::SingleTwoDataLines].
    ///
    /// Disconnects the previous pin that was assigned with `with_mosi` or
    /// `with_sio0`.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::spi::{
    ///     Mode,
    ///     master::{Config, Spi},
    /// };
    /// let mut spi = Spi::new(peripherals.SPI2, Config::default())?.with_mosi(peripherals.GPIO1);
    ///
    /// # {after_snippet}
    /// ```
    pub fn with_mosi(mut self, mosi: impl PeripheralOutput<'d>) -> Self {
        self.spi.pins().sio_pins[0] = self.connect_sio_output_pin(mosi.into(), 0);
        self
    }

    #[procmacros::doc_replace]
    /// Assign the MISO (Master In Slave Out) pin for the SPI instance.
    ///
    /// Enables input functionality for the pin, and connects it to the MISO
    /// signal.
    ///
    /// You want to use this for full-duplex SPI or
    /// [DataMode::SingleTwoDataLines]
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::spi::{
    ///     Mode,
    ///     master::{Config, Spi},
    /// };
    /// let mut spi = Spi::new(peripherals.SPI2, Config::default())?.with_miso(peripherals.GPIO2);
    ///
    /// # {after_snippet}
    /// ```
    pub fn with_miso(self, miso: impl PeripheralInput<'d>) -> Self {
        let miso = miso.into();

        miso.apply_input_config(&InputConfig::default());
        miso.set_input_enable(true);

        self.driver().info.sio_input(1).connect_to(&miso);

        self
    }

    /// Assign the SIO0 pin for the SPI instance.
    ///
    /// Enables both input and output functionality for the pin, and connects it
    /// to the MOSI output signal and SIO0 input signal.
    ///
    /// Disconnects the previous pin that was assigned with `with_sio0` or
    /// `with_mosi`.
    ///
    /// Use this if any of the devices on the bus use half-duplex SPI.
    ///
    /// See also [Self::with_mosi] when you only need a one-directional MOSI
    /// signal.
    #[instability::unstable]
    pub fn with_sio0(mut self, mosi: impl PeripheralInput<'d> + PeripheralOutput<'d>) -> Self {
        self.spi.pins().sio_pins[0] = self.connect_sio_pin(mosi.into(), 0);

        self
    }

    /// Assign the SIO1/MISO pin for the SPI instance.
    ///
    /// Enables both input and output functionality for the pin, and connects it
    /// to the MISO input signal and SIO1 output signal.
    ///
    /// Disconnects the previous pin that was assigned with `with_sio1`.
    ///
    /// Use this if any of the devices on the bus use half-duplex SPI.
    ///
    /// See also [Self::with_miso] when you only need a one-directional MISO
    /// signal.
    #[instability::unstable]
    pub fn with_sio1(mut self, sio1: impl PeripheralInput<'d> + PeripheralOutput<'d>) -> Self {
        self.spi.pins().sio_pins[1] = self.connect_sio_pin(sio1.into(), 1);

        self
    }

    def_with_sio_pin!(with_sio2, 2);
    def_with_sio_pin!(with_sio3, 3);

    #[cfg(spi_master_has_octal)]
    def_with_sio_pin!(with_sio4, 4);

    #[cfg(spi_master_has_octal)]
    def_with_sio_pin!(with_sio5, 5);

    #[cfg(spi_master_has_octal)]
    def_with_sio_pin!(with_sio6, 6);

    #[cfg(spi_master_has_octal)]
    def_with_sio_pin!(with_sio7, 7);

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
        let info = self.spi.info();
        self.spi.pins().cs_pin = self.connect_output_pin(cs.into(), info.cs(0));

        self
    }

    #[doc_replace(
        "max_frequency" => {
            cfg(esp32h2) => "48MHz",
            _ => "80MHz",
        }
    )]
    /// Change the bus configuration.
    ///
    /// # Errors
    ///
    /// If frequency passed in config exceeds __max_frequency__ or is below 70kHz,
    /// [`ConfigError::FrequencyOutOfRange`] error will be returned.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::spi::{
    ///     Mode,
    ///     master::{Config, Spi},
    /// };
    /// let mut spi = Spi::new(peripherals.SPI2, Config::default())?;
    ///
    /// spi.apply_config(&Config::default().with_frequency(Rate::from_khz(100)));
    /// #
    /// # {after_snippet}
    /// ```
    pub fn apply_config(&mut self, config: &Config) -> Result<(), ConfigError> {
        self.driver().apply_config(config)
    }

    #[procmacros::doc_replace]
    /// Write bytes to SPI. After writing, flush is called to ensure all data
    /// has been transmitted.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::spi::{
    ///     Mode,
    ///     master::{Config, Spi},
    /// };
    /// let mut spi = Spi::new(peripherals.SPI2, Config::default())?
    ///     .with_sck(peripherals.GPIO0)
    ///     .with_mosi(peripherals.GPIO1)
    ///     .with_miso(peripherals.GPIO2)
    ///     .into_async();
    ///
    /// let buffer = [0; 10];
    /// spi.write(&buffer)?;
    ///
    /// # {after_snippet}
    /// ```
    pub fn write(&mut self, words: &[u8]) -> Result<(), Error> {
        self.driver().flush()?;
        self.driver().setup_full_duplex()?;

        for chunk in words.chunks(FIFO_SIZE) {
            self.driver().write_one(chunk)?;
            // NOTE: While we don't need to flush after the last chunk, changing
            // that would change the behavior of the function.
            // https://github.com/esp-rs/esp-hal/issues/5257
            self.driver().flush()?;
        }

        Ok(())
    }

    #[procmacros::doc_replace]
    /// Read bytes from SPI. The provided slice is filled with data received
    /// from the slave.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::spi::{
    ///     Mode,
    ///     master::{Config, Spi},
    /// };
    /// let mut spi = Spi::new(peripherals.SPI2, Config::default())?
    ///     .with_sck(peripherals.GPIO0)
    ///     .with_mosi(peripherals.GPIO1)
    ///     .with_miso(peripherals.GPIO2)
    ///     .into_async();
    ///
    /// let mut buffer = [0; 10];
    /// spi.read(&mut buffer)?;
    ///
    /// # {after_snippet}
    /// ```
    pub fn read(&mut self, words: &mut [u8]) -> Result<(), Error> {
        self.driver().flush()?;
        self.driver().setup_full_duplex()?;
        self.driver().read(words)
    }

    #[procmacros::doc_replace]
    /// Sends `words` to the slave. The received data will be written to
    /// `words`, overwriting its contents.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::spi::{
    ///     Mode,
    ///     master::{Config, Spi},
    /// };
    /// let mut spi = Spi::new(peripherals.SPI2, Config::default())?
    ///     .with_sck(peripherals.GPIO0)
    ///     .with_mosi(peripherals.GPIO1)
    ///     .with_miso(peripherals.GPIO2)
    ///     .into_async();
    ///
    /// let mut buffer = [0; 10];
    /// spi.transfer(&mut buffer)?;
    ///
    /// # {after_snippet}
    /// ```
    pub fn transfer(&mut self, words: &mut [u8]) -> Result<(), Error> {
        self.driver().flush()?;
        self.driver().setup_full_duplex()?;
        self.driver().transfer_in_place(words)
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

        self.flush()?;
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

        self.flush()?;

        cfg_if::cfg_if! {
            if #[cfg(all(spi_master_version = "1", spi_address_workaround))] {
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
            self.driver().configure_datalen(0, buffer.len());
            self.driver().fill_fifo(buffer);
        }

        self.driver().start_operation();

        self.driver().flush()
    }

    fn use_blocking_transfer(&self, transfer_size: usize) -> bool {
        let threshold = self
            .spi
            .state()
            .min_async_transfer_size
            .load(Ordering::Relaxed);
        threshold > 0 && transfer_size < threshold
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
        self.read(words)
    }

    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        // Do not call the inherent `write` method. The trait impl does not flush after.
        // Flush before starting to ensure the bus is clear before we reconfigure to full duplex.
        self.driver().flush()?;
        self.driver().setup_full_duplex()?;
        for chunk in words.chunks(FIFO_SIZE) {
            self.driver().flush()?;
            self.driver().write_one(chunk)?;
        }
        Ok(())
    }

    fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
        self.driver().flush()?;
        self.driver().setup_full_duplex()?;

        if read.is_empty() {
            self.driver().write(write)
        } else if write.is_empty() {
            self.driver().read(read)
        } else {
            self.driver().transfer(read, write)
        }
    }

    fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        self.driver().flush()?;
        self.driver().setup_full_duplex()?;
        self.driver().transfer_in_place(words)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        self.driver().flush()
    }
}

impl SpiBusAsync for Spi<'_, Async> {
    async fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        self.read_async(words).await
    }

    async fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        self.write_async(words).await
    }

    async fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
        self.driver().flush_async().await;
        self.driver().setup_full_duplex()?;

        if self.use_blocking_transfer(read.len().max(write.len())) {
            if read.is_empty() {
                self.driver().write(write)?;
                return self.driver().flush();
            } else if write.is_empty() {
                return self.driver().read(read);
            } else {
                return self.driver().transfer(read, write);
            }
        }

        if read.is_empty() {
            self.driver().write_async(write).await
        } else if write.is_empty() {
            self.driver().read_async(read).await
        } else {
            self.driver().transfer_async(read, write).await
        }
    }

    async fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        self.transfer_in_place_async(words).await
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        self.flush_async().await
    }
}

/// SPI data mode
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub enum DataMode {
    /// 1 bit, two data lines. (MOSI, MISO)
    SingleTwoDataLines,
    /// 1 bit, 1 data line (SIO0)
    Single,
    /// 2 bits, two data lines. (SIO0, SIO1)
    Dual,
    /// 4 bit, 4 data lines. (SIO0 .. SIO3)
    Quad,
    #[cfg(spi_master_has_octal)]
    /// 8 bit, 8 data lines. (SIO0 .. SIO7)
    Octal,
}

crate::any_peripheral! {
    /// Any SPI peripheral.
    pub peripheral AnySpi<'d> {
        #[cfg(spi_master_spi2)]
        Spi2(crate::peripherals::SPI2<'d>),
        #[cfg(spi_master_spi3)]
        Spi3(crate::peripherals::SPI3<'d>),
    }
}

impl QspiInstance for AnySpi<'_> {}

impl Instance for AnySpi<'_> {
    #[inline]
    fn parts(&self) -> (&'static Info, &'static State) {
        any::delegate!(self, spi => { spi.parts() })
    }
}

impl AnySpi<'_> {
    fn bind_peri_interrupt(&self, handler: InterruptHandler) {
        any::delegate!(self, spi => { spi.bind_peri_interrupt(handler) })
    }

    fn disable_peri_interrupt_on_all_cores(&self) {
        any::delegate!(self, spi => { spi.disable_peri_interrupt_on_all_cores() })
    }

    fn set_interrupt_handler(&self, handler: InterruptHandler) {
        self.disable_peri_interrupt_on_all_cores();
        self.bind_peri_interrupt(handler);
    }
}
