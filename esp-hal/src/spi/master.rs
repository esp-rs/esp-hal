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
//! and embassy-embedded-hal.
//!
//! ## Examples
//!
//! ### SPI Initialization
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! # use esp_hal::spi::SpiMode;
//! # use esp_hal::spi::master::{Config, Spi};
//! let sclk = peripherals.GPIO0;
//! let miso = peripherals.GPIO2;
//! let mosi = peripherals.GPIO1;
//! let cs = peripherals.GPIO5;
//!
//! let mut spi = Spi::new(
//!     peripherals.SPI2,
//!     Config::default().with_frequency(100.kHz()).with_mode(SpiMode::Mode0)
//! )
//! .unwrap()
//! .with_sck(sclk)
//! .with_mosi(mosi)
//! .with_miso(miso)
//! .with_cs(cs);
//! # }
//! ```
//! 
//! [`embedded-hal-bus`]: https://docs.rs/embedded-hal-bus/latest/embedded_hal_bus/spi/index.html
//! [`embassy-embedded-hal`]: https://docs.embassy.dev/embassy-embedded-hal/git/default/shared_bus/index.html

use core::marker::PhantomData;

pub use dma::*;
#[cfg(any(doc, feature = "unstable"))]
use embassy_embedded_hal::SetConfig;
#[cfg(gdma)]
use enumset::EnumSet;
#[cfg(gdma)]
use enumset::EnumSetType;
use fugit::HertzU32;
#[cfg(place_spi_driver_in_ram)]
use procmacros::ram;

use super::{DmaError, Error, SpiBitOrder, SpiDataMode, SpiMode};
use crate::{
    clock::Clocks,
    dma::{DmaChannelFor, DmaEligible, DmaRxBuffer, DmaTxBuffer, Rx, Tx},
    gpio::{interconnect::PeripheralOutput, InputSignal, NoPin, OutputSignal},
    interrupt::InterruptHandler,
    peripheral::{Peripheral, PeripheralRef},
    peripherals::spi2::RegisterBlock,
    private,
    spi::AnySpi,
    system::PeripheralGuard,
    Async,
    Blocking,
    Mode,
};

/// Enumeration of possible SPI interrupt events.
#[cfg(gdma)]
#[derive(Debug, Hash, EnumSetType)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum SpiInterrupt {
    /// Indicates that the SPI transaction has completed successfully.
    ///
    /// This interrupt is triggered when an SPI transaction has finished
    /// transmitting and receiving data.
    TransDone,
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
pub enum Command {
    /// No command is sent.
    None,
    /// Command1.
    Command1(u16, SpiDataMode),
    /// Command2.
    Command2(u16, SpiDataMode),
    /// Command3.
    Command3(u16, SpiDataMode),
    /// Command4.
    Command4(u16, SpiDataMode),
    /// Command5.
    Command5(u16, SpiDataMode),
    /// Command6.
    Command6(u16, SpiDataMode),
    /// Command7.
    Command7(u16, SpiDataMode),
    /// Command8.
    Command8(u16, SpiDataMode),
    /// Command9.
    Command9(u16, SpiDataMode),
    /// Command10.
    Command10(u16, SpiDataMode),
    /// Command11.
    Command11(u16, SpiDataMode),
    /// Command12.
    Command12(u16, SpiDataMode),
    /// Command13.
    Command13(u16, SpiDataMode),
    /// Command14.
    Command14(u16, SpiDataMode),
    /// Command15.
    Command15(u16, SpiDataMode),
    /// Command16.
    Command16(u16, SpiDataMode),
}

impl Command {
    fn width(&self) -> usize {
        match self {
            Command::None => 0,
            Command::Command1(_, _) => 1,
            Command::Command2(_, _) => 2,
            Command::Command3(_, _) => 3,
            Command::Command4(_, _) => 4,
            Command::Command5(_, _) => 5,
            Command::Command6(_, _) => 6,
            Command::Command7(_, _) => 7,
            Command::Command8(_, _) => 8,
            Command::Command9(_, _) => 9,
            Command::Command10(_, _) => 10,
            Command::Command11(_, _) => 11,
            Command::Command12(_, _) => 12,
            Command::Command13(_, _) => 13,
            Command::Command14(_, _) => 14,
            Command::Command15(_, _) => 15,
            Command::Command16(_, _) => 16,
        }
    }

    fn value(&self) -> u16 {
        match self {
            Command::None => 0,
            Command::Command1(value, _)
            | Command::Command2(value, _)
            | Command::Command3(value, _)
            | Command::Command4(value, _)
            | Command::Command5(value, _)
            | Command::Command6(value, _)
            | Command::Command7(value, _)
            | Command::Command8(value, _)
            | Command::Command9(value, _)
            | Command::Command10(value, _)
            | Command::Command11(value, _)
            | Command::Command12(value, _)
            | Command::Command13(value, _)
            | Command::Command14(value, _)
            | Command::Command15(value, _)
            | Command::Command16(value, _) => *value,
        }
    }

    fn mode(&self) -> SpiDataMode {
        match self {
            Command::None => SpiDataMode::Single,
            Command::Command1(_, mode)
            | Command::Command2(_, mode)
            | Command::Command3(_, mode)
            | Command::Command4(_, mode)
            | Command::Command5(_, mode)
            | Command::Command6(_, mode)
            | Command::Command7(_, mode)
            | Command::Command8(_, mode)
            | Command::Command9(_, mode)
            | Command::Command10(_, mode)
            | Command::Command11(_, mode)
            | Command::Command12(_, mode)
            | Command::Command13(_, mode)
            | Command::Command14(_, mode)
            | Command::Command15(_, mode)
            | Command::Command16(_, mode) => *mode,
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
pub enum Address {
    /// No address phase.
    None,
    /// Address with 1-bit.
    Address1(u32, SpiDataMode),
    /// Address with 2-bit.
    Address2(u32, SpiDataMode),
    /// Address with 3-bit.
    Address3(u32, SpiDataMode),
    /// Address with 4-bit.
    Address4(u32, SpiDataMode),
    /// Address with 5-bit.
    Address5(u32, SpiDataMode),
    /// Address with 6-bit.
    Address6(u32, SpiDataMode),
    /// Address with 7-bit.
    Address7(u32, SpiDataMode),
    /// Address with 8-bit.
    Address8(u32, SpiDataMode),
    /// Address with 9-bit.
    Address9(u32, SpiDataMode),
    /// Address with 10-bit.
    Address10(u32, SpiDataMode),
    /// Address with 11-bit.
    Address11(u32, SpiDataMode),
    /// Address with 12-bit.
    Address12(u32, SpiDataMode),
    /// Address with 13-bit.
    Address13(u32, SpiDataMode),
    /// Address with 14-bit.
    Address14(u32, SpiDataMode),
    /// Address with 15-bit.
    Address15(u32, SpiDataMode),
    /// Address with 16-bit.
    Address16(u32, SpiDataMode),
    /// Address with 17-bit.
    Address17(u32, SpiDataMode),
    /// Address with 18-bit.
    Address18(u32, SpiDataMode),
    /// Address with 19-bit.
    Address19(u32, SpiDataMode),
    /// Address with 20-bit.
    Address20(u32, SpiDataMode),
    /// Address with 21-bit.
    Address21(u32, SpiDataMode),
    /// Address with 22-bit.
    Address22(u32, SpiDataMode),
    /// Address with 23-bit.
    Address23(u32, SpiDataMode),
    /// Address with 24-bit.
    Address24(u32, SpiDataMode),
    /// Address with 25-bit.
    Address25(u32, SpiDataMode),
    /// Address with 26-bit.
    Address26(u32, SpiDataMode),
    /// Address with 27-bit.
    Address27(u32, SpiDataMode),
    /// Address with 28-bit.
    Address28(u32, SpiDataMode),
    /// Address with 29-bit.
    Address29(u32, SpiDataMode),
    /// Address with 30-bit.
    Address30(u32, SpiDataMode),
    /// Address with 31-bit.
    Address31(u32, SpiDataMode),
    /// Address with 32-bit.
    Address32(u32, SpiDataMode),
}

impl Address {
    fn width(&self) -> usize {
        match self {
            Address::None => 0,
            Address::Address1(_, _) => 1,
            Address::Address2(_, _) => 2,
            Address::Address3(_, _) => 3,
            Address::Address4(_, _) => 4,
            Address::Address5(_, _) => 5,
            Address::Address6(_, _) => 6,
            Address::Address7(_, _) => 7,
            Address::Address8(_, _) => 8,
            Address::Address9(_, _) => 9,
            Address::Address10(_, _) => 10,
            Address::Address11(_, _) => 11,
            Address::Address12(_, _) => 12,
            Address::Address13(_, _) => 13,
            Address::Address14(_, _) => 14,
            Address::Address15(_, _) => 15,
            Address::Address16(_, _) => 16,
            Address::Address17(_, _) => 17,
            Address::Address18(_, _) => 18,
            Address::Address19(_, _) => 19,
            Address::Address20(_, _) => 20,
            Address::Address21(_, _) => 21,
            Address::Address22(_, _) => 22,
            Address::Address23(_, _) => 23,
            Address::Address24(_, _) => 24,
            Address::Address25(_, _) => 25,
            Address::Address26(_, _) => 26,
            Address::Address27(_, _) => 27,
            Address::Address28(_, _) => 28,
            Address::Address29(_, _) => 29,
            Address::Address30(_, _) => 30,
            Address::Address31(_, _) => 31,
            Address::Address32(_, _) => 32,
        }
    }

    fn value(&self) -> u32 {
        match self {
            Address::None => 0,
            Address::Address1(value, _)
            | Address::Address2(value, _)
            | Address::Address3(value, _)
            | Address::Address4(value, _)
            | Address::Address5(value, _)
            | Address::Address6(value, _)
            | Address::Address7(value, _)
            | Address::Address8(value, _)
            | Address::Address9(value, _)
            | Address::Address10(value, _)
            | Address::Address11(value, _)
            | Address::Address12(value, _)
            | Address::Address13(value, _)
            | Address::Address14(value, _)
            | Address::Address15(value, _)
            | Address::Address16(value, _)
            | Address::Address17(value, _)
            | Address::Address18(value, _)
            | Address::Address19(value, _)
            | Address::Address20(value, _)
            | Address::Address21(value, _)
            | Address::Address22(value, _)
            | Address::Address23(value, _)
            | Address::Address24(value, _)
            | Address::Address25(value, _)
            | Address::Address26(value, _)
            | Address::Address27(value, _)
            | Address::Address28(value, _)
            | Address::Address29(value, _)
            | Address::Address30(value, _)
            | Address::Address31(value, _)
            | Address::Address32(value, _) => *value,
        }
    }

    fn is_none(&self) -> bool {
        matches!(self, Address::None)
    }

    fn mode(&self) -> SpiDataMode {
        match self {
            Address::None => SpiDataMode::Single,
            Address::Address1(_, mode)
            | Address::Address2(_, mode)
            | Address::Address3(_, mode)
            | Address::Address4(_, mode)
            | Address::Address5(_, mode)
            | Address::Address6(_, mode)
            | Address::Address7(_, mode)
            | Address::Address8(_, mode)
            | Address::Address9(_, mode)
            | Address::Address10(_, mode)
            | Address::Address11(_, mode)
            | Address::Address12(_, mode)
            | Address::Address13(_, mode)
            | Address::Address14(_, mode)
            | Address::Address15(_, mode)
            | Address::Address16(_, mode)
            | Address::Address17(_, mode)
            | Address::Address18(_, mode)
            | Address::Address19(_, mode)
            | Address::Address20(_, mode)
            | Address::Address21(_, mode)
            | Address::Address22(_, mode)
            | Address::Address23(_, mode)
            | Address::Address24(_, mode)
            | Address::Address25(_, mode)
            | Address::Address26(_, mode)
            | Address::Address27(_, mode)
            | Address::Address28(_, mode)
            | Address::Address29(_, mode)
            | Address::Address30(_, mode)
            | Address::Address31(_, mode)
            | Address::Address32(_, mode) => *mode,
        }
    }
}

/// SPI peripheral configuration
#[derive(Clone, Copy, Debug, PartialEq, Eq, procmacros::BuilderLite)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct Config {
    /// SPI clock frequency
    pub frequency: HertzU32,

    /// SPI mode
    pub mode: SpiMode,

    /// Bit order of the read data.
    pub read_bit_order: SpiBitOrder,

    /// Bit order of the written data.
    pub write_bit_order: SpiBitOrder,
}

impl Default for Config {
    fn default() -> Self {
        use fugit::RateExtU32;
        Config {
            frequency: 1_u32.MHz(),
            mode: SpiMode::Mode0,
            read_bit_order: SpiBitOrder::MsbFirst,
            write_bit_order: SpiBitOrder::MsbFirst,
        }
    }
}

/// Configuration errors.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ConfigError {}

/// SPI peripheral driver
#[derive(Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Spi<'d, M, T = AnySpi> {
    spi: PeripheralRef<'d, T>,
    _mode: PhantomData<M>,
    guard: PeripheralGuard,
}

impl<M, T> Spi<'_, M, T>
where
    T: Instance,
{
    fn driver(&self) -> &'static Info {
        self.spi.info()
    }

    /// Read a byte from SPI.
    ///
    /// Sends out a stuffing byte for every byte to read. This function doesn't
    /// perform flushing. If you want to read the response to something you
    /// have written before, consider using [`Self::transfer`] instead.
    pub fn read_byte(&mut self) -> nb::Result<u8, Error> {
        self.driver().read_byte()
    }

    /// Write a byte to SPI.
    pub fn write_byte(&mut self, word: u8) -> nb::Result<(), Error> {
        self.driver().write_byte(word)
    }

    /// Write bytes to SPI.
    ///
    /// Copies the content of `words` in chunks of 64 bytes into the SPI
    /// transmission FIFO. If `words` is longer than 64 bytes, multiple
    /// sequential transfers are performed.
    pub fn write_bytes(&mut self, words: &[u8]) -> Result<(), Error> {
        self.driver().write_bytes(words)?;
        self.driver().flush()?;

        Ok(())
    }

    /// Sends `words` to the slave. Returns the `words` received from the slave
    pub fn transfer<'w>(&mut self, words: &'w mut [u8]) -> Result<&'w [u8], Error> {
        self.driver().transfer(words)
    }
}

impl<'d> Spi<'d, Blocking> {
    /// Constructs an SPI instance in 8bit dataframe mode.
    pub fn new(
        spi: impl Peripheral<P = impl Instance> + 'd,
        config: Config,
    ) -> Result<Self, ConfigError> {
        Self::new_typed(spi.map_into(), config)
    }
}

impl<'d, T> Spi<'d, Blocking, T>
where
    T: Instance,
{
    /// Converts the SPI instance into async mode.
    pub fn into_async(self) -> Spi<'d, Async, T> {
        Spi {
            spi: self.spi,
            _mode: PhantomData,
            guard: self.guard,
        }
    }

    /// Configures the SPI instance to use DMA with the specified channel.
    ///
    /// This method prepares the SPI instance for DMA transfers using SPI
    /// and returns an instance of `SpiDma` that supports DMA
    /// operations.
    pub fn with_dma<CH>(self, channel: impl Peripheral<P = CH> + 'd) -> SpiDma<'d, Blocking, T>
    where
        CH: DmaChannelFor<T>,
    {
        SpiDma::new(self.spi, channel.map(|ch| ch.degrade()).into_ref())
    }
}

impl<'d, T> Spi<'d, Async, T>
where
    T: Instance,
{
    /// Converts the SPI instance into blocking mode.
    pub fn into_blocking(self) -> Spi<'d, Blocking, T> {
        Spi {
            spi: self.spi,
            _mode: PhantomData,
            guard: self.guard,
        }
    }
}

impl<'d, M, T> Spi<'d, M, T>
where
    T: Instance,
{
    /// Constructs an SPI instance in 8bit dataframe mode.
    pub fn new_typed(
        spi: impl Peripheral<P = T> + 'd,
        config: Config,
    ) -> Result<Self, ConfigError> {
        crate::into_ref!(spi);

        let guard = PeripheralGuard::new(spi.info().peripheral);

        let mut this = Spi {
            spi,
            _mode: PhantomData,
            guard,
        };

        this.driver().init();
        this.apply_config(&config)?;

        let this = this
            .with_mosi(NoPin)
            .with_miso(NoPin)
            .with_sck(NoPin)
            .with_cs(NoPin);

        let is_qspi = this.driver().sio2_input.is_some();
        if is_qspi {
            unwrap!(this.driver().sio2_input).connect_to(NoPin);
            unwrap!(this.driver().sio2_output).connect_to(NoPin);
            unwrap!(this.driver().sio3_input).connect_to(NoPin);
            unwrap!(this.driver().sio3_output).connect_to(NoPin);
        }

        Ok(this)
    }

    /// Assign the MOSI (Master Out Slave In) pin for the SPI instance.
    ///
    /// Enables both input and output functionality for the pin, and connects it
    /// to the MOSI signal and SIO0 input signal.
    pub fn with_mosi<MOSI: PeripheralOutput>(self, mosi: impl Peripheral<P = MOSI> + 'd) -> Self {
        crate::into_mapped_ref!(mosi);
        mosi.enable_output(true, private::Internal);
        mosi.enable_input(true, private::Internal);

        self.driver().mosi.connect_to(&mut mosi);
        self.driver().sio0_input.connect_to(&mut mosi);

        self
    }

    /// Assign the MISO (Master In Slave Out) pin for the SPI instance.
    ///
    /// Enables both input and output functionality for the pin, and connects it
    /// to the MISO signal and SIO1 input signal.
    pub fn with_miso<MISO: PeripheralOutput>(self, miso: impl Peripheral<P = MISO> + 'd) -> Self {
        crate::into_mapped_ref!(miso);
        miso.enable_input(true, private::Internal);
        miso.enable_output(true, private::Internal);

        self.driver().miso.connect_to(&mut miso);
        self.driver().sio1_output.connect_to(&mut miso);

        self
    }

    /// Assign the SCK (Serial Clock) pin for the SPI instance.
    ///
    /// Sets the specified pin to push-pull output and connects it to the SPI
    /// clock signal.
    pub fn with_sck<SCK: PeripheralOutput>(self, sclk: impl Peripheral<P = SCK> + 'd) -> Self {
        crate::into_mapped_ref!(sclk);
        sclk.set_to_push_pull_output(private::Internal);
        self.driver().sclk.connect_to(sclk);

        self
    }

    /// Assign the CS (Chip Select) pin for the SPI instance.
    ///
    /// Sets the specified pin to push-pull output and connects it to the SPI CS
    /// signal.
    pub fn with_cs<CS: PeripheralOutput>(self, cs: impl Peripheral<P = CS> + 'd) -> Self {
        crate::into_mapped_ref!(cs);
        cs.set_to_push_pull_output(private::Internal);
        self.driver().cs.connect_to(cs);

        self
    }

    /// Change the bus configuration.
    pub fn apply_config(&mut self, config: &Config) -> Result<(), ConfigError> {
        self.driver().apply_config(config)
    }
}

#[cfg(any(doc, feature = "unstable"))]
#[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
impl<M, T> SetConfig for Spi<'_, M, T>
where
    T: Instance,
    M: Mode,
{
    type Config = Config;
    type ConfigError = ConfigError;

    fn set_config(&mut self, config: &Self::Config) -> Result<(), Self::ConfigError> {
        self.apply_config(config)
    }
}

impl<'d, M, T> Spi<'d, M, T>
where
    T: QspiInstance,
{
    /// Assign the SIO2 pin for the SPI instance.
    ///
    /// Enables both input and output functionality for the pin, and connects it
    /// to the SIO2 output and input signals.
    pub fn with_sio2<SIO2: PeripheralOutput>(self, sio2: impl Peripheral<P = SIO2> + 'd) -> Self
    where
        T: QspiInstance,
    {
        crate::into_mapped_ref!(sio2);
        sio2.enable_input(true, private::Internal);
        sio2.enable_output(true, private::Internal);

        unwrap!(self.driver().sio2_input).connect_to(&mut sio2);
        unwrap!(self.driver().sio2_output).connect_to(&mut sio2);

        self
    }

    /// Assign the SIO3 pin for the SPI instance.
    ///
    /// Enables both input and output functionality for the pin, and connects it
    /// to the SIO3 output and input signals.
    pub fn with_sio3<SIO3: PeripheralOutput>(self, sio3: impl Peripheral<P = SIO3> + 'd) -> Self
    where
        T: QspiInstance,
    {
        crate::into_mapped_ref!(sio3);
        sio3.enable_input(true, private::Internal);
        sio3.enable_output(true, private::Internal);

        unwrap!(self.driver().sio3_input).connect_to(&mut sio3);
        unwrap!(self.driver().sio3_output).connect_to(&mut sio3);

        self
    }
}

impl<M, T> Spi<'_, M, T>
where
    T: Instance,
{
    /// Half-duplex read.
    pub fn half_duplex_read(
        &mut self,
        data_mode: SpiDataMode,
        cmd: Command,
        address: Address,
        dummy: u8,
        buffer: &mut [u8],
    ) -> Result<(), Error> {
        if buffer.len() > FIFO_SIZE {
            return Err(Error::FifoSizeExeeded);
        }

        if buffer.is_empty() {
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
        );

        self.driver().configure_datalen(buffer.len(), 0);
        self.driver().start_operation();
        self.driver().flush()?;
        self.driver().read_bytes_from_fifo(buffer)
    }

    /// Half-duplex write.
    pub fn half_duplex_write(
        &mut self,
        data_mode: SpiDataMode,
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
        );

        if !buffer.is_empty() {
            // re-using the full-duplex write here
            self.driver().write_bytes(buffer)?;
        } else {
            self.driver().start_operation();
        }

        self.driver().flush()
    }
}

mod dma {
    use core::{
        cmp::min,
        mem::ManuallyDrop,
        sync::atomic::{fence, Ordering},
    };

    use super::*;
    use crate::{
        dma::{
            asynch::{DmaRxFuture, DmaTxFuture},
            Channel,
            DmaRxBuf,
            DmaRxBuffer,
            DmaTxBuf,
            DmaTxBuffer,
            EmptyBuf,
            PeripheralDmaChannel,
            Rx,
            Tx,
        },
        interrupt::InterruptConfigurable,
        Async,
        Blocking,
    };

    /// A DMA capable SPI instance.
    ///
    /// Using `SpiDma` is not recommended unless you wish
    /// to manage buffers yourself. It's recommended to use
    /// [`SpiDmaBus`] via `with_buffers` to get access
    /// to a DMA capable SPI bus that implements the
    /// embedded-hal traits.
    pub struct SpiDma<'d, M, T = AnySpi>
    where
        T: Instance,
        M: Mode,
    {
        pub(crate) spi: PeripheralRef<'d, T>,
        pub(crate) channel: Channel<'d, M, PeripheralDmaChannel<T>>,
        tx_transfer_in_progress: bool,
        rx_transfer_in_progress: bool,
        #[cfg(all(esp32, spi_address_workaround))]
        address_buffer: DmaTxBuf,
        guard: PeripheralGuard,
    }

    impl<M, T> crate::private::Sealed for SpiDma<'_, M, T>
    where
        T: Instance,
        M: Mode,
    {
    }

    impl<'d, T> SpiDma<'d, Blocking, T>
    where
        T: Instance,
    {
        /// Converts the SPI instance into async mode.
        pub fn into_async(self) -> SpiDma<'d, Async, T> {
            SpiDma {
                spi: self.spi,
                channel: self.channel.into_async(),
                tx_transfer_in_progress: self.tx_transfer_in_progress,
                rx_transfer_in_progress: self.rx_transfer_in_progress,
                #[cfg(all(esp32, spi_address_workaround))]
                address_buffer: self.address_buffer,
                guard: self.guard,
            }
        }
    }

    impl<'d, T> SpiDma<'d, Async, T>
    where
        T: Instance,
    {
        /// Converts the SPI instance into async mode.
        pub fn into_blocking(self) -> SpiDma<'d, Blocking, T> {
            SpiDma {
                spi: self.spi,
                channel: self.channel.into_blocking(),
                tx_transfer_in_progress: self.tx_transfer_in_progress,
                rx_transfer_in_progress: self.rx_transfer_in_progress,
                #[cfg(all(esp32, spi_address_workaround))]
                address_buffer: self.address_buffer,
                guard: self.guard,
            }
        }
    }

    impl<M, T> core::fmt::Debug for SpiDma<'_, M, T>
    where
        T: Instance,
        M: Mode,
    {
        /// Formats the `SpiDma` instance for debugging purposes.
        ///
        /// This method returns a debug struct with the name "SpiDma" without
        /// exposing internal details.
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
            f.debug_struct("SpiDma").finish()
        }
    }

    impl<T> InterruptConfigurable for SpiDma<'_, Blocking, T>
    where
        T: Instance,
    {
        /// Sets the interrupt handler
        ///
        /// Interrupts are not enabled at the peripheral level here.
        fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
            let interrupt = self.driver().interrupt;
            for core in crate::Cpu::other() {
                crate::interrupt::disable(core, interrupt);
            }
            unsafe { crate::interrupt::bind_interrupt(interrupt, handler.handler()) };
            unwrap!(crate::interrupt::enable(interrupt, handler.priority()));
        }
    }

    #[cfg(gdma)]
    impl<T> SpiDma<'_, Blocking, T>
    where
        T: Instance,
    {
        /// Listen for the given interrupts
        pub fn listen(&mut self, interrupts: impl Into<EnumSet<SpiInterrupt>>) {
            self.driver().enable_listen(interrupts.into(), true);
        }

        /// Unlisten the given interrupts
        pub fn unlisten(&mut self, interrupts: impl Into<EnumSet<SpiInterrupt>>) {
            self.driver().enable_listen(interrupts.into(), false);
        }

        /// Gets asserted interrupts
        pub fn interrupts(&mut self) -> EnumSet<SpiInterrupt> {
            self.driver().interrupts()
        }

        /// Resets asserted interrupts
        pub fn clear_interrupts(&mut self, interrupts: impl Into<EnumSet<SpiInterrupt>>) {
            self.driver().clear_interrupts(interrupts.into());
        }
    }

    impl<'d, T> SpiDma<'d, Blocking, T>
    where
        T: Instance,
    {
        pub(super) fn new(
            spi: PeripheralRef<'d, T>,
            channel: PeripheralRef<'d, PeripheralDmaChannel<T>>,
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
                    crate::as_mut_byte_array!(BUFFERS[id], 4)
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
            }
        }
    }

    impl<'d, M, T> SpiDma<'d, M, T>
    where
        M: Mode,
        T: Instance,
    {
        fn driver(&self) -> &'static Info {
            self.spi.info()
        }

        fn dma_driver(&self) -> DmaDriver {
            DmaDriver {
                info: self.driver(),
                dma_peripheral: self.spi.dma_peripheral(),
            }
        }

        fn is_done(&self) -> bool {
            if self.tx_transfer_in_progress && !self.channel.tx.is_done() {
                return false;
            }
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

        async fn wait_for_idle_async(&mut self) {
            // As a future enhancement, setup Spi Future in here as well.

            if self.rx_transfer_in_progress {
                _ = DmaRxFuture::new(&mut self.channel.rx).await;
                self.rx_transfer_in_progress = false;
            }
            if self.tx_transfer_in_progress {
                _ = DmaTxFuture::new(&mut self.channel.tx).await;
                self.tx_transfer_in_progress = false;
            }

            core::future::poll_fn(|cx| {
                use core::task::Poll;
                if self.is_done() {
                    Poll::Ready(())
                } else {
                    cx.waker().wake_by_ref();
                    Poll::Pending
                }
            })
            .await;
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
                    &mut self.channel.rx,
                    &mut self.channel.tx,
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
            );

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
                    &mut self.channel.rx,
                    &mut self.channel.tx,
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

        /// Change the bus configuration.
        pub fn apply_config(&mut self, config: &Config) -> Result<(), ConfigError> {
            self.driver().apply_config(config)
        }

        /// Configures the DMA buffers for the SPI instance.
        ///
        /// This method sets up both RX and TX buffers for DMA transfers.
        /// It returns an instance of `SpiDmaBus` that can be used for SPI
        /// communication.
        pub fn with_buffers(
            self,
            dma_rx_buf: DmaRxBuf,
            dma_tx_buf: DmaTxBuf,
        ) -> SpiDmaBus<'d, M, T> {
            SpiDmaBus::new(self, dma_rx_buf, dma_tx_buf)
        }
    }

    #[cfg(any(doc, feature = "unstable"))]
    #[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
    impl<M, T> SetConfig for SpiDma<'_, M, T>
    where
        T: Instance,
        M: Mode,
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
    pub struct SpiDmaTransfer<'d, M, Buf, T = AnySpi>
    where
        T: Instance,
        M: Mode,
    {
        spi_dma: ManuallyDrop<SpiDma<'d, M, T>>,
        dma_buf: ManuallyDrop<Buf>,
    }

    impl<'d, M, T, Buf> SpiDmaTransfer<'d, M, Buf, T>
    where
        T: Instance,
        M: Mode,
    {
        fn new(spi_dma: SpiDma<'d, M, T>, dma_buf: Buf) -> Self {
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
        pub fn wait(mut self) -> (SpiDma<'d, M, T>, Buf) {
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
        pub fn cancel(&mut self) {
            if !self.spi_dma.is_done() {
                self.spi_dma.cancel_transfer();
            }
        }
    }

    impl<M, T, Buf> Drop for SpiDmaTransfer<'_, M, Buf, T>
    where
        T: Instance,
        M: Mode,
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

    impl<T, Buf> SpiDmaTransfer<'_, Async, Buf, T>
    where
        T: Instance,
    {
        /// Waits for the DMA transfer to complete asynchronously.
        ///
        /// This method awaits the completion of both RX and TX operations.
        pub async fn wait_for_done(&mut self) {
            self.spi_dma.wait_for_idle_async().await;
        }
    }

    impl<'d, M, T> SpiDma<'d, M, T>
    where
        T: Instance,
        M: Mode,
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
            self.start_dma_transfer(0, bytes_to_write, &mut EmptyBuf, buffer)
        }

        /// Perform a DMA write.
        ///
        /// This will return a [SpiDmaTransfer] owning the buffer and the
        /// SPI instance. The maximum amount of data to be sent is 32736
        /// bytes.
        #[allow(clippy::type_complexity)]
        #[cfg_attr(place_spi_driver_in_ram, ram)]
        pub fn write<TX: DmaTxBuffer>(
            mut self,
            bytes_to_write: usize,
            mut buffer: TX,
        ) -> Result<SpiDmaTransfer<'d, M, TX, T>, (Error, Self, TX)> {
            self.wait_for_idle();

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
            self.start_dma_transfer(bytes_to_read, 0, buffer, &mut EmptyBuf)
        }

        /// Perform a DMA read.
        ///
        /// This will return a [SpiDmaTransfer] owning the buffer and
        /// the SPI instance. The maximum amount of data to be
        /// received is 32736 bytes.
        #[allow(clippy::type_complexity)]
        #[cfg_attr(place_spi_driver_in_ram, ram)]
        pub fn read<RX: DmaRxBuffer>(
            mut self,
            bytes_to_read: usize,
            mut buffer: RX,
        ) -> Result<SpiDmaTransfer<'d, M, RX, T>, (Error, Self, RX)> {
            self.wait_for_idle();
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
            self.start_transfer_dma(true, bytes_to_read, bytes_to_write, rx_buffer, tx_buffer)
        }

        /// Perform a DMA transfer
        ///
        /// This will return a [SpiDmaTransfer] owning the buffers and
        /// the SPI instance. The maximum amount of data to be
        /// sent/received is 32736 bytes.
        #[allow(clippy::type_complexity)]
        #[cfg_attr(place_spi_driver_in_ram, ram)]
        pub fn transfer<RX: DmaRxBuffer, TX: DmaTxBuffer>(
            mut self,
            bytes_to_read: usize,
            mut rx_buffer: RX,
            bytes_to_write: usize,
            mut tx_buffer: TX,
        ) -> Result<SpiDmaTransfer<'d, M, (RX, TX), T>, (Error, Self, RX, TX)> {
            self.wait_for_idle();
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
            data_mode: SpiDataMode,
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
            );

            self.start_transfer_dma(false, bytes_to_read, 0, buffer, &mut EmptyBuf)
        }

        /// Perform a half-duplex read operation using DMA.
        #[allow(clippy::type_complexity)]
        #[cfg_attr(place_spi_driver_in_ram, ram)]
        pub fn half_duplex_read<RX: DmaRxBuffer>(
            mut self,
            data_mode: SpiDataMode,
            cmd: Command,
            address: Address,
            dummy: u8,
            bytes_to_read: usize,
            mut buffer: RX,
        ) -> Result<SpiDmaTransfer<'d, M, RX, T>, (Error, Self, RX)> {
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
            data_mode: SpiDataMode,
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
                if bytes_to_write == 0 && address.mode() != SpiDataMode::Single {
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
            );

            self.start_transfer_dma(false, 0, bytes_to_write, &mut EmptyBuf, buffer)
        }

        /// Perform a half-duplex write operation using DMA.
        #[allow(clippy::type_complexity)]
        #[cfg_attr(place_spi_driver_in_ram, ram)]
        pub fn half_duplex_write<TX: DmaTxBuffer>(
            mut self,
            data_mode: SpiDataMode,
            cmd: Command,
            address: Address,
            dummy: u8,
            bytes_to_write: usize,
            mut buffer: TX,
        ) -> Result<SpiDmaTransfer<'d, M, TX, T>, (Error, Self, TX)> {
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
    }

    /// A DMA-capable SPI bus.
    ///
    /// This structure is responsible for managing SPI transfers using DMA
    /// buffers.
    pub struct SpiDmaBus<'d, M, T = AnySpi>
    where
        T: Instance,

        M: Mode,
    {
        spi_dma: SpiDma<'d, M, T>,
        rx_buf: DmaRxBuf,
        tx_buf: DmaTxBuf,
    }

    impl<M, T> crate::private::Sealed for SpiDmaBus<'_, M, T>
    where
        T: Instance,
        M: Mode,
    {
    }

    impl<'d, T> SpiDmaBus<'d, Blocking, T>
    where
        T: Instance,
    {
        /// Converts the SPI instance into async mode.
        pub fn into_async(self) -> SpiDmaBus<'d, Async, T> {
            SpiDmaBus {
                spi_dma: self.spi_dma.into_async(),
                rx_buf: self.rx_buf,
                tx_buf: self.tx_buf,
            }
        }
    }

    impl<'d, T> SpiDmaBus<'d, Async, T>
    where
        T: Instance,
    {
        /// Converts the SPI instance into async mode.
        pub fn into_blocking(self) -> SpiDmaBus<'d, Blocking, T> {
            SpiDmaBus {
                spi_dma: self.spi_dma.into_blocking(),
                rx_buf: self.rx_buf,
                tx_buf: self.tx_buf,
            }
        }
    }

    impl<'d, M, T> SpiDmaBus<'d, M, T>
    where
        T: Instance,
        M: Mode,
    {
        /// Creates a new `SpiDmaBus` with the specified SPI instance and DMA
        /// buffers.
        pub fn new(spi_dma: SpiDma<'d, M, T>, rx_buf: DmaRxBuf, tx_buf: DmaTxBuf) -> Self {
            Self {
                spi_dma,
                rx_buf,
                tx_buf,
            }
        }
    }

    impl<T> InterruptConfigurable for SpiDmaBus<'_, Blocking, T>
    where
        T: Instance,
    {
        /// Sets the interrupt handler
        ///
        /// Interrupts are not enabled at the peripheral level here.
        fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
            self.spi_dma.set_interrupt_handler(handler);
        }
    }

    #[cfg(gdma)]
    impl<T> SpiDmaBus<'_, Blocking, T>
    where
        T: Instance,
    {
        /// Listen for the given interrupts
        pub fn listen(&mut self, interrupts: impl Into<EnumSet<SpiInterrupt>>) {
            self.spi_dma.listen(interrupts.into());
        }

        /// Unlisten the given interrupts
        pub fn unlisten(&mut self, interrupts: impl Into<EnumSet<SpiInterrupt>>) {
            self.spi_dma.unlisten(interrupts.into());
        }

        /// Gets asserted interrupts
        pub fn interrupts(&mut self) -> EnumSet<SpiInterrupt> {
            self.spi_dma.interrupts()
        }

        /// Resets asserted interrupts
        pub fn clear_interrupts(&mut self, interrupts: impl Into<EnumSet<SpiInterrupt>>) {
            self.spi_dma.clear_interrupts(interrupts.into());
        }
    }

    impl<M, T> SpiDmaBus<'_, M, T>
    where
        T: Instance,
        M: Mode,
    {
        fn wait_for_idle(&mut self) {
            self.spi_dma.wait_for_idle();
        }

        /// Change the bus configuration.
        pub fn apply_config(&mut self, config: &Config) -> Result<(), ConfigError> {
            self.spi_dma.apply_config(config)
        }

        /// Reads data from the SPI bus using DMA.
        pub fn read(&mut self, words: &mut [u8]) -> Result<(), Error> {
            self.wait_for_idle();
            for chunk in words.chunks_mut(self.rx_buf.capacity()) {
                self.rx_buf.set_length(chunk.len());

                unsafe {
                    self.spi_dma.start_dma_transfer(
                        chunk.len(),
                        0,
                        &mut self.rx_buf,
                        &mut EmptyBuf,
                    )?;
                }

                self.wait_for_idle();

                let bytes_read = self.rx_buf.read_received_data(chunk);
                debug_assert_eq!(bytes_read, chunk.len());
            }

            Ok(())
        }

        /// Writes data to the SPI bus using DMA.
        pub fn write(&mut self, words: &[u8]) -> Result<(), Error> {
            self.wait_for_idle();
            for chunk in words.chunks(self.tx_buf.capacity()) {
                self.tx_buf.fill(chunk);

                unsafe {
                    self.spi_dma.start_dma_transfer(
                        chunk.len(),
                        0,
                        &mut EmptyBuf,
                        &mut self.tx_buf,
                    )?;
                }

                self.wait_for_idle();
            }

            Ok(())
        }

        /// Transfers data to and from the SPI bus simultaneously using DMA.
        pub fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Error> {
            self.wait_for_idle();
            let chunk_size = min(self.tx_buf.capacity(), self.rx_buf.capacity());

            let common_length = min(read.len(), write.len());
            let (read_common, read_remainder) = read.split_at_mut(common_length);
            let (write_common, write_remainder) = write.split_at(common_length);

            for (read_chunk, write_chunk) in read_common
                .chunks_mut(chunk_size)
                .zip(write_common.chunks(chunk_size))
            {
                self.tx_buf.fill(write_chunk);
                self.rx_buf.set_length(read_chunk.len());

                unsafe {
                    self.spi_dma.start_dma_transfer(
                        read_chunk.len(),
                        write_chunk.len(),
                        &mut self.rx_buf,
                        &mut self.tx_buf,
                    )?;
                }
                self.wait_for_idle();

                let bytes_read = self.rx_buf.read_received_data(read_chunk);
                debug_assert_eq!(bytes_read, read_chunk.len());
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
        pub fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Error> {
            self.wait_for_idle();
            let chunk_size = min(self.tx_buf.capacity(), self.rx_buf.capacity());

            for chunk in words.chunks_mut(chunk_size) {
                self.tx_buf.fill(chunk);
                self.rx_buf.set_length(chunk.len());

                unsafe {
                    self.spi_dma.start_dma_transfer(
                        chunk.len(),
                        chunk.len(),
                        &mut self.rx_buf,
                        &mut self.tx_buf,
                    )?;
                }
                self.wait_for_idle();

                let bytes_read = self.rx_buf.read_received_data(chunk);
                debug_assert_eq!(bytes_read, chunk.len());
            }

            Ok(())
        }

        /// Half-duplex read.
        pub fn half_duplex_read(
            &mut self,
            data_mode: SpiDataMode,
            cmd: Command,
            address: Address,
            dummy: u8,
            buffer: &mut [u8],
        ) -> Result<(), Error> {
            if buffer.len() > self.rx_buf.capacity() {
                return Err(Error::from(DmaError::Overflow));
            }
            self.wait_for_idle();
            self.rx_buf.set_length(buffer.len());

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

            let bytes_read = self.rx_buf.read_received_data(buffer);
            debug_assert_eq!(bytes_read, buffer.len());

            Ok(())
        }

        /// Half-duplex write.
        pub fn half_duplex_write(
            &mut self,
            data_mode: SpiDataMode,
            cmd: Command,
            address: Address,
            dummy: u8,
            buffer: &[u8],
        ) -> Result<(), Error> {
            if buffer.len() > self.tx_buf.capacity() {
                return Err(Error::from(DmaError::Overflow));
            }
            self.wait_for_idle();
            self.tx_buf.fill(buffer);

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

    #[cfg(any(doc, feature = "unstable"))]
    #[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
    impl<M, T> SetConfig for SpiDmaBus<'_, M, T>
    where
        T: Instance,
        M: Mode,
    {
        type Config = Config;
        type ConfigError = ConfigError;

        fn set_config(&mut self, config: &Self::Config) -> Result<(), Self::ConfigError> {
            self.apply_config(config)
        }
    }

    /// Async functionality
    mod asynch {
        use core::{
            cmp::min,
            ops::{Deref, DerefMut},
        };

        use super::*;
        use crate::Async;

        struct DropGuard<I, F: FnOnce(I)> {
            inner: ManuallyDrop<I>,
            on_drop: ManuallyDrop<F>,
        }

        impl<I, F: FnOnce(I)> DropGuard<I, F> {
            fn new(inner: I, on_drop: F) -> Self {
                Self {
                    inner: ManuallyDrop::new(inner),
                    on_drop: ManuallyDrop::new(on_drop),
                }
            }

            fn defuse(self) {}
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

        impl<T> SpiDmaBus<'_, Async, T>
        where
            T: Instance,
        {
            /// Fill the given buffer with data from the bus.
            pub async fn read_async(&mut self, words: &mut [u8]) -> Result<(), Error> {
                self.spi_dma.wait_for_idle_async().await;
                let chunk_size = self.rx_buf.capacity();

                for chunk in words.chunks_mut(chunk_size) {
                    self.rx_buf.set_length(chunk.len());

                    let mut spi = DropGuard::new(&mut self.spi_dma, |spi| spi.cancel_transfer());

                    unsafe {
                        spi.start_dma_transfer(chunk.len(), 0, &mut self.rx_buf, &mut EmptyBuf)?
                    };

                    spi.wait_for_idle_async().await;

                    let bytes_read = self.rx_buf.read_received_data(chunk);
                    debug_assert_eq!(bytes_read, chunk.len());

                    spi.defuse();
                }

                Ok(())
            }

            /// Transmit the given buffer to the bus.
            pub async fn write_async(&mut self, words: &[u8]) -> Result<(), Error> {
                self.spi_dma.wait_for_idle_async().await;

                let mut spi = DropGuard::new(&mut self.spi_dma, |spi| spi.cancel_transfer());
                let chunk_size = self.tx_buf.capacity();

                for chunk in words.chunks(chunk_size) {
                    self.tx_buf.fill(chunk);

                    unsafe {
                        spi.start_dma_transfer(0, chunk.len(), &mut EmptyBuf, &mut self.tx_buf)?
                    };

                    spi.wait_for_idle_async().await;
                }
                spi.defuse();

                Ok(())
            }

            /// Transfer by writing out a buffer and reading the response from
            /// the bus into another buffer.
            pub async fn transfer_async(
                &mut self,
                read: &mut [u8],
                write: &[u8],
            ) -> Result<(), Error> {
                self.spi_dma.wait_for_idle_async().await;

                let mut spi = DropGuard::new(&mut self.spi_dma, |spi| spi.cancel_transfer());
                let chunk_size = min(self.tx_buf.capacity(), self.rx_buf.capacity());

                let common_length = min(read.len(), write.len());
                let (read_common, read_remainder) = read.split_at_mut(common_length);
                let (write_common, write_remainder) = write.split_at(common_length);

                for (read_chunk, write_chunk) in read_common
                    .chunks_mut(chunk_size)
                    .zip(write_common.chunks(chunk_size))
                {
                    self.tx_buf.fill(write_chunk);
                    self.rx_buf.set_length(read_chunk.len());

                    unsafe {
                        spi.start_dma_transfer(
                            read_chunk.len(),
                            write_chunk.len(),
                            &mut self.rx_buf,
                            &mut self.tx_buf,
                        )?;
                    }
                    spi.wait_for_idle_async().await;

                    let bytes_read = self.rx_buf.read_received_data(read_chunk);
                    assert_eq!(bytes_read, read_chunk.len());
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
            pub async fn transfer_in_place_async(&mut self, words: &mut [u8]) -> Result<(), Error> {
                self.spi_dma.wait_for_idle_async().await;

                let mut spi = DropGuard::new(&mut self.spi_dma, |spi| spi.cancel_transfer());
                for chunk in words.chunks_mut(self.tx_buf.capacity()) {
                    self.tx_buf.fill(chunk);
                    self.rx_buf.set_length(chunk.len());

                    unsafe {
                        spi.start_dma_transfer(
                            chunk.len(),
                            chunk.len(),
                            &mut self.rx_buf,
                            &mut self.tx_buf,
                        )?;
                    }
                    spi.wait_for_idle_async().await;

                    let bytes_read = self.rx_buf.read_received_data(chunk);
                    assert_eq!(bytes_read, chunk.len());
                }

                spi.defuse();

                Ok(())
            }
        }

        impl<T> embedded_hal_async::spi::SpiBus for SpiDmaBus<'_, Async, T>
        where
            T: Instance,
        {
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
        use embedded_hal::spi::{ErrorType, SpiBus};

        use super::*;

        impl<M, T> ErrorType for SpiDmaBus<'_, M, T>
        where
            T: Instance,
            M: Mode,
        {
            type Error = Error;
        }

        impl<M, T> SpiBus for SpiDmaBus<'_, M, T>
        where
            T: Instance,
            M: Mode,
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
    use embedded_hal_nb::spi::FullDuplex;

    use super::*;

    impl<M, T> embedded_hal::spi::ErrorType for Spi<'_, M, T> {
        type Error = Error;
    }

    impl<M, T> FullDuplex for Spi<'_, M, T>
    where
        T: Instance,
    {
        fn read(&mut self) -> nb::Result<u8, Self::Error> {
            self.driver().read_byte()
        }

        fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
            self.driver().write_byte(word)
        }
    }

    impl<M, T> SpiBus for Spi<'_, M, T>
    where
        T: Instance,
    {
        fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
            self.driver().read_bytes(words)
        }

        fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
            self.driver().write_bytes(words)
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
                        .read_bytes_from_fifo(&mut read[read_from..read_to])?;
                }

                write_from = write_to;
                read_from = read_to;
            }
            Ok(())
        }

        fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
            // Since we have the traits so neatly implemented above, use them!
            for chunk in words.chunks_mut(FIFO_SIZE) {
                SpiBus::write(self, chunk)?;
                SpiBus::flush(self)?;
                self.driver().read_bytes_from_fifo(chunk)?;
            }
            Ok(())
        }

        fn flush(&mut self) -> Result<(), Self::Error> {
            self.driver().flush()
        }
    }
}

/// SPI peripheral instance.
pub trait Instance: private::Sealed + Into<AnySpi> + DmaEligible + 'static {
    /// Returns the peripheral data describing this SPI instance.
    fn info(&self) -> &'static Info;
}

/// Marker trait for QSPI-capable SPI peripherals.
pub trait QspiInstance: Instance {}

/// Peripheral data describing a particular SPI instance.
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

    /// Chip select signal.
    pub cs: OutputSignal,

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
}

struct DmaDriver {
    info: &'static Info,
    dma_peripheral: crate::dma::DmaPeripheral,
}

impl DmaDriver {
    fn abort_transfer(&self) {
        self.info.configure_datalen(1, 1);
        self.info.update();
    }

    #[allow(clippy::too_many_arguments)]
    #[cfg_attr(place_spi_driver_in_ram, ram)]
    unsafe fn start_transfer_dma<RX: Rx, TX: Tx>(
        &self,
        _full_duplex: bool,
        rx_len: usize,
        tx_len: usize,
        rx_buffer: &mut impl DmaRxBuffer,
        tx_buffer: &mut impl DmaTxBuffer,
        rx: &mut RX,
        tx: &mut TX,
    ) -> Result<(), Error> {
        let reg_block = self.info.register_block();

        #[cfg(esp32s2)]
        {
            // without this a transfer after a write will fail
            reg_block.dma_out_link().write(|w| w.bits(0));
            reg_block.dma_in_link().write(|w| w.bits(0));
        }

        self.info.configure_datalen(rx_len, tx_len);

        // enable the MISO and MOSI if needed
        reg_block
            .user()
            .modify(|_, w| w.usr_miso().bit(rx_len > 0).usr_mosi().bit(tx_len > 0));

        self.enable_dma();

        if rx_len > 0 {
            rx.prepare_transfer(self.dma_peripheral, rx_buffer)
                .and_then(|_| rx.start_transfer())?;
        } else {
            #[cfg(esp32)]
            {
                // see https://github.com/espressif/esp-idf/commit/366e4397e9dae9d93fe69ea9d389b5743295886f
                // see https://github.com/espressif/esp-idf/commit/0c3653b1fd7151001143451d4aa95dbf15ee8506
                if _full_duplex {
                    reg_block
                        .dma_in_link()
                        .modify(|_, w| unsafe { w.inlink_addr().bits(0) });
                    reg_block
                        .dma_in_link()
                        .modify(|_, w| w.inlink_start().set_bit());
                }
            }
        }
        if tx_len > 0 {
            tx.prepare_transfer(self.dma_peripheral, tx_buffer)
                .and_then(|_| tx.start_transfer())?;
        }

        #[cfg(gdma)]
        self.reset_dma();

        self.info.start_operation();

        Ok(())
    }

    fn enable_dma(&self) {
        #[cfg(gdma)]
        {
            // for non GDMA this is done in `assign_tx_device` / `assign_rx_device`
            let reg_block = self.info.register_block();
            reg_block.dma_conf().modify(|_, w| {
                w.dma_tx_ena().set_bit();
                w.dma_rx_ena().set_bit()
            });
        }
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
        let reg_block = self.info.register_block();
        set_reset_bit(reg_block, true);
        set_reset_bit(reg_block, false);
        self.clear_dma_interrupts();
    }

    #[cfg(gdma)]
    fn clear_dma_interrupts(&self) {
        let reg_block = self.info.register_block();
        reg_block.dma_int_clr().write(|w| {
            w.dma_infifo_full_err().clear_bit_by_one();
            w.dma_outfifo_empty_err().clear_bit_by_one();
            w.trans_done().clear_bit_by_one();
            w.mst_rx_afifo_wfull_err().clear_bit_by_one();
            w.mst_tx_afifo_rempty_err().clear_bit_by_one()
        });
    }

    #[cfg(pdma)]
    fn clear_dma_interrupts(&self) {
        let reg_block = self.info.register_block();
        reg_block.dma_int_clr().write(|w| {
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

// private implementation bits
// FIXME: split this up into peripheral versions
impl Info {
    /// Returns the register block for this SPI instance.
    pub fn register_block(&self) -> &RegisterBlock {
        unsafe { &*self.register_block }
    }

    /// Initialize for full-duplex 1 bit mode
    fn init(&self) {
        let reg_block = self.register_block();
        reg_block.user().modify(|_, w| {
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
        reg_block.clk_gate().modify(|_, w| {
            w.clk_en().set_bit();
            w.mst_clk_active().set_bit();
            w.mst_clk_sel().set_bit()
        });

        #[cfg(any(esp32c6, esp32h2))]
        unsafe {
            // use default clock source PLL_F80M_CLK (ESP32-C6) and
            // PLL_F48M_CLK (ESP32-H2)
            crate::peripherals::PCR::steal()
                .spi2_clkm_conf()
                .modify(|_, w| w.spi2_clkm_sel().bits(1));
        }

        #[cfg(gdma)]
        reg_block.ctrl().modify(|_, w| {
            w.q_pol().clear_bit();
            w.d_pol().clear_bit();
            w.hold_pol().clear_bit()
        });

        #[cfg(esp32s2)]
        reg_block.ctrl().modify(|_, w| {
            w.q_pol().clear_bit();
            w.d_pol().clear_bit();
            w.wp().clear_bit()
        });

        #[cfg(esp32)]
        reg_block.ctrl().modify(|_, w| w.wp().clear_bit());

        #[cfg(not(esp32))]
        reg_block.misc().write(|w| unsafe { w.bits(0) });

        reg_block.slave().write(|w| unsafe { w.bits(0) });
    }

    #[cfg(not(esp32))]
    fn init_spi_data_mode(
        &self,
        cmd_mode: SpiDataMode,
        address_mode: SpiDataMode,
        data_mode: SpiDataMode,
    ) {
        let reg_block = self.register_block();
        reg_block.ctrl().modify(|_, w| {
            w.fcmd_dual().bit(cmd_mode == SpiDataMode::Dual);
            w.fcmd_quad().bit(cmd_mode == SpiDataMode::Quad);
            w.faddr_dual().bit(address_mode == SpiDataMode::Dual);
            w.faddr_quad().bit(address_mode == SpiDataMode::Quad);
            w.fread_dual().bit(data_mode == SpiDataMode::Dual);
            w.fread_quad().bit(data_mode == SpiDataMode::Quad)
        });
        reg_block.user().modify(|_, w| {
            w.fwrite_dual().bit(data_mode == SpiDataMode::Dual);
            w.fwrite_quad().bit(data_mode == SpiDataMode::Quad)
        });
    }

    #[cfg(esp32)]
    fn init_spi_data_mode(
        &self,
        cmd_mode: SpiDataMode,
        address_mode: SpiDataMode,
        data_mode: SpiDataMode,
    ) {
        let reg_block = self.register_block();
        match cmd_mode {
            SpiDataMode::Single => (),
            _ => panic!("Only 1-bit command supported"),
        }

        match address_mode {
            SpiDataMode::Single => {
                reg_block.ctrl().modify(|_, w| {
                    w.fread_dio().clear_bit();
                    w.fread_qio().clear_bit();
                    w.fread_dual().bit(data_mode == SpiDataMode::Dual);
                    w.fread_quad().bit(data_mode == SpiDataMode::Quad)
                });

                reg_block.user().modify(|_, w| {
                    w.fwrite_dio().clear_bit();
                    w.fwrite_qio().clear_bit();
                    w.fwrite_dual().bit(data_mode == SpiDataMode::Dual);
                    w.fwrite_quad().bit(data_mode == SpiDataMode::Quad)
                });
            }
            address_mode if address_mode == data_mode => {
                reg_block.ctrl().modify(|_, w| {
                    w.fastrd_mode().set_bit();
                    w.fread_dio().bit(address_mode == SpiDataMode::Dual);
                    w.fread_qio().bit(address_mode == SpiDataMode::Quad);
                    w.fread_dual().clear_bit();
                    w.fread_quad().clear_bit()
                });

                reg_block.user().modify(|_, w| {
                    w.fwrite_dio().bit(address_mode == SpiDataMode::Dual);
                    w.fwrite_qio().bit(address_mode == SpiDataMode::Quad);
                    w.fwrite_dual().clear_bit();
                    w.fwrite_quad().clear_bit()
                });
            }
            _ => panic!("Unsupported combination of data-modes"),
        }
    }

    // taken from https://github.com/apache/incubator-nuttx/blob/8267a7618629838231256edfa666e44b5313348e/arch/risc-v/src/esp32c3/esp32c3_spi.c#L496
    fn setup(&self, frequency: HertzU32) {
        let clocks = Clocks::get();
        cfg_if::cfg_if! {
            if #[cfg(esp32h2)] {
                // ESP32-H2 is using PLL_48M_CLK source instead of APB_CLK
                let apb_clk_freq = HertzU32::Hz(clocks.pll_48m_clock.to_Hz());
            } else {
                let apb_clk_freq = HertzU32::Hz(clocks.apb_clock.to_Hz());
            }
        }

        let reg_val: u32;
        let duty_cycle = 128;

        // In HW, n, h and l fields range from 1 to 64, pre ranges from 1 to 8K.
        // The value written to register is one lower than the used value.

        if frequency > ((apb_clk_freq / 4) * 3) {
            // Using APB frequency directly will give us the best result here.
            reg_val = 1 << 31;
        } else {
            /* For best duty cycle resolution, we want n to be as close to 32 as
             * possible, but we also need a pre/n combo that gets us as close as
             * possible to the intended frequency. To do this, we bruteforce n and
             * calculate the best pre to go along with that. If there's a choice
             * between pre/n combos that give the same result, use the one with the
             * higher n.
             */

            let mut pre: i32;
            let mut bestn: i32 = -1;
            let mut bestpre: i32 = -1;
            let mut besterr: i32 = 0;
            let mut errval: i32;

            /* Start at n = 2. We need to be able to set h/l so we have at least
             * one high and one low pulse.
             */

            for n in 2..64 {
                /* Effectively, this does:
                 *   pre = round((APB_CLK_FREQ / n) / frequency)
                 */

                pre = ((apb_clk_freq.raw() as i32 / n) + (frequency.raw() as i32 / 2))
                    / frequency.raw() as i32;

                if pre <= 0 {
                    pre = 1;
                }

                if pre > 16 {
                    pre = 16;
                }

                errval = (apb_clk_freq.raw() as i32 / (pre * n) - frequency.raw() as i32).abs();
                if bestn == -1 || errval <= besterr {
                    besterr = errval;
                    bestn = n;
                    bestpre = pre;
                }
            }

            let n: i32 = bestn;
            pre = bestpre;
            let l: i32 = n;

            /* Effectively, this does:
             *   h = round((duty_cycle * n) / 256)
             */

            let mut h: i32 = (duty_cycle * n + 127) / 256;
            if h <= 0 {
                h = 1;
            }

            reg_val = (l as u32 - 1)
                | ((h as u32 - 1) << 6)
                | ((n as u32 - 1) << 12)
                | ((pre as u32 - 1) << 18);
        }

        self.register_block()
            .clock()
            .write(|w| unsafe { w.bits(reg_val) });
    }

    /// Enable or disable listening for the given interrupts.
    #[cfg(gdma)]
    fn enable_listen(&self, interrupts: EnumSet<SpiInterrupt>, enable: bool) {
        let reg_block = self.register_block();

        reg_block.dma_int_ena().modify(|_, w| {
            for interrupt in interrupts {
                match interrupt {
                    SpiInterrupt::TransDone => w.trans_done().bit(enable),
                };
            }
            w
        });
    }

    /// Gets asserted interrupts
    #[cfg(gdma)]
    fn interrupts(&self) -> EnumSet<SpiInterrupt> {
        let mut res = EnumSet::new();
        let reg_block = self.register_block();

        let ints = reg_block.dma_int_st().read();

        if ints.trans_done().bit() {
            res.insert(SpiInterrupt::TransDone);
        }

        res
    }

    /// Resets asserted interrupts
    #[cfg(gdma)]
    fn clear_interrupts(&self, interrupts: EnumSet<SpiInterrupt>) {
        let reg_block = self.register_block();

        reg_block.dma_int_clr().write(|w| {
            for interrupt in interrupts {
                match interrupt {
                    SpiInterrupt::TransDone => w.trans_done().clear_bit_by_one(),
                };
            }
            w
        });
    }

    fn apply_config(&self, config: &Config) -> Result<(), ConfigError> {
        self.ch_bus_freq(config.frequency);
        self.set_bit_order(config.read_bit_order, config.write_bit_order);
        self.set_data_mode(config.mode);
        Ok(())
    }

    fn set_data_mode(&self, data_mode: SpiMode) {
        let reg_block = self.register_block();

        cfg_if::cfg_if! {
            if #[cfg(esp32)] {
                let pin_reg = reg_block.pin();
            } else {
                let pin_reg = reg_block.misc();
            }
        };

        pin_reg.modify(|_, w| {
            w.ck_idle_edge()
                .bit(matches!(data_mode, SpiMode::Mode2 | SpiMode::Mode3))
        });
        reg_block.user().modify(|_, w| {
            w.ck_out_edge()
                .bit(matches!(data_mode, SpiMode::Mode1 | SpiMode::Mode2))
        });
    }

    fn ch_bus_freq(&self, frequency: HertzU32) {
        fn enable_clocks(_reg_block: &RegisterBlock, _enable: bool) {
            #[cfg(gdma)]
            _reg_block.clk_gate().modify(|_, w| {
                w.clk_en().bit(_enable);
                w.mst_clk_active().bit(_enable);
                w.mst_clk_sel().bit(_enable)
            });
        }

        enable_clocks(self.register_block(), false);

        // Change clock frequency
        self.setup(frequency);

        enable_clocks(self.register_block(), true);
    }

    #[cfg(not(any(esp32, esp32c3, esp32s2)))]
    fn set_bit_order(&self, read_order: SpiBitOrder, write_order: SpiBitOrder) {
        let reg_block = self.register_block();

        let read_value = match read_order {
            SpiBitOrder::MsbFirst => 0,
            SpiBitOrder::LsbFirst => 1,
        };
        let write_value = match write_order {
            SpiBitOrder::MsbFirst => 0,
            SpiBitOrder::LsbFirst => 1,
        };
        reg_block.ctrl().modify(|_, w| unsafe {
            w.rd_bit_order().bits(read_value);
            w.wr_bit_order().bits(write_value);
            w
        });
    }

    #[cfg(any(esp32, esp32c3, esp32s2))]
    fn set_bit_order(&self, read_order: SpiBitOrder, write_order: SpiBitOrder) {
        let reg_block = self.register_block();

        let read_value = match read_order {
            SpiBitOrder::MsbFirst => false,
            SpiBitOrder::LsbFirst => true,
        };
        let write_value = match write_order {
            SpiBitOrder::MsbFirst => false,
            SpiBitOrder::LsbFirst => true,
        };
        reg_block.ctrl().modify(|_, w| {
            w.rd_bit_order().bit(read_value);
            w.wr_bit_order().bit(write_value);
            w
        });
    }

    fn read_byte(&self) -> nb::Result<u8, Error> {
        if self.busy() {
            return Err(nb::Error::WouldBlock);
        }

        let reg_block = self.register_block();
        Ok(u32::try_into(reg_block.w(0).read().bits()).unwrap_or_default())
    }

    fn write_byte(&self, word: u8) -> nb::Result<(), Error> {
        if self.busy() {
            return Err(nb::Error::WouldBlock);
        }

        self.configure_datalen(0, 1);

        let reg_block = self.register_block();
        reg_block.w(0).write(|w| w.buf().set(word.into()));

        self.start_operation();

        Ok(())
    }

    /// Write bytes to SPI.
    ///
    /// Copies the content of `words` in chunks of 64 bytes into the SPI
    /// transmission FIFO. If `words` is longer than 64 bytes, multiple
    /// sequential transfers are performed. This function will return before
    /// all bytes of the last chunk to transmit have been sent to the wire. If
    /// you must ensure that the whole messages was written correctly, use
    /// [`Self::flush`].
    #[cfg_attr(place_spi_driver_in_ram, ram)]
    fn write_bytes(&self, words: &[u8]) -> Result<(), Error> {
        let num_chunks = words.len() / FIFO_SIZE;

        // Flush in case previous writes have not completed yet, required as per
        // embedded-hal documentation (#1369).
        self.flush()?;

        // The fifo has a limited fixed size, so the data must be chunked and then
        // transmitted
        for (i, chunk) in words.chunks(FIFO_SIZE).enumerate() {
            self.configure_datalen(0, chunk.len());

            {
                // TODO: replace with `array_chunks` and `from_le_bytes`
                let mut c_iter = chunk.chunks_exact(4);
                let mut w_iter = self.register_block().w_iter();
                for c in c_iter.by_ref() {
                    if let Some(w_reg) = w_iter.next() {
                        let word = (c[0] as u32)
                            | (c[1] as u32) << 8
                            | (c[2] as u32) << 16
                            | (c[3] as u32) << 24;
                        w_reg.write(|w| w.buf().set(word));
                    }
                }
                let rem = c_iter.remainder();
                if !rem.is_empty() {
                    if let Some(w_reg) = w_iter.next() {
                        let word = match rem.len() {
                            3 => (rem[0] as u32) | (rem[1] as u32) << 8 | (rem[2] as u32) << 16,
                            2 => (rem[0] as u32) | (rem[1] as u32) << 8,
                            1 => rem[0] as u32,
                            _ => unreachable!(),
                        };
                        w_reg.write(|w| w.buf().set(word));
                    }
                }
            }

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

    /// Read bytes from SPI.
    ///
    /// Sends out a stuffing byte for every byte to read. This function doesn't
    /// perform flushing. If you want to read the response to something you
    /// have written before, consider using [`Self::transfer`] instead.
    #[cfg_attr(place_spi_driver_in_ram, ram)]
    fn read_bytes(&self, words: &mut [u8]) -> Result<(), Error> {
        let empty_array = [EMPTY_WRITE_PAD; FIFO_SIZE];

        for chunk in words.chunks_mut(FIFO_SIZE) {
            self.write_bytes(&empty_array[0..chunk.len()])?;
            self.flush()?;
            self.read_bytes_from_fifo(chunk)?;
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
    fn read_bytes_from_fifo(&self, words: &mut [u8]) -> Result<(), Error> {
        let reg_block = self.register_block();

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
        let reg_block = self.register_block();
        reg_block.cmd().read().usr().bit_is_set()
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
            self.write_bytes(chunk)?;
            self.flush()?;
            self.read_bytes_from_fifo(chunk)?;
        }

        Ok(words)
    }

    fn start_operation(&self) {
        let reg_block = self.register_block();
        self.update();
        reg_block.cmd().modify(|_, w| w.usr().set_bit());
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
        data_mode: SpiDataMode,
    ) {
        self.init_spi_data_mode(cmd.mode(), address.mode(), data_mode);

        let reg_block = self.register_block();
        reg_block.user().modify(|_, w| {
            w.usr_miso_highpart().clear_bit();
            w.usr_mosi_highpart().clear_bit();
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
        unsafe {
            let pcr = crate::peripherals::PCR::steal();

            // use default clock source PLL_F80M_CLK
            pcr.spi2_clkm_conf()
                .modify(|_, w| w.spi2_clkm_sel().bits(1));
        }

        #[cfg(not(esp32))]
        reg_block.misc().write(|w| unsafe { w.bits(0) });

        reg_block.slave().write(|w| unsafe { w.bits(0) });

        self.update();

        // set cmd, address, dummy cycles
        self.set_up_common_phases(cmd, address, dummy);
    }

    fn set_up_common_phases(&self, cmd: Command, address: Address, dummy: u8) {
        let reg_block = self.register_block();
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
                let reg_block = self.register_block();

                reg_block.cmd().modify(|_, w| w.update().set_bit());

                while reg_block.cmd().read().update().bit_is_set() {
                    // wait
                }
            } else if #[cfg(esp32)] {
                xtensa_lx::timer::delay(1);
            } else {
                // Doesn't seem to be needed for ESP32-S2
            }
        }
    }

    fn configure_datalen(&self, rx_len_bytes: usize, tx_len_bytes: usize) {
        let reg_block = self.register_block();

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
        self.register_block == other.register_block
    }
}

unsafe impl Sync for Info {}

macro_rules! spi_instance {
    ($num:literal, $sclk:ident, $mosi:ident, $miso:ident, $cs:ident $(, $sio2:ident, $sio3:ident)?) => {
        paste::paste! {
            impl Instance for crate::peripherals::[<SPI $num>] {
                #[inline(always)]
                fn info(&self) -> &'static Info {
                    static INFO: Info = Info {
                        register_block: crate::peripherals::[<SPI $num>]::PTR,
                        peripheral: crate::system::Peripheral::[<Spi $num>],
                        interrupt: crate::peripherals::Interrupt::[<SPI $num>],
                        sclk: OutputSignal::$sclk,
                        mosi: OutputSignal::$mosi,
                        miso: InputSignal::$miso,
                        cs: OutputSignal::$cs,
                        sio0_input: InputSignal::$mosi,
                        sio1_output: OutputSignal::$miso,
                        sio2_output: $crate::if_set!($(Some(OutputSignal::$sio2))?, None),
                        sio2_input: $crate::if_set!($(Some(InputSignal::$sio2))?, None),
                        sio3_output: $crate::if_set!($(Some(OutputSignal::$sio3))?, None),
                        sio3_input: $crate::if_set!($(Some(InputSignal::$sio3))?, None),
                    };

                    &INFO
                }
            }

            $(
                // If the extra pins are set, implement QspiInstance
                $crate::ignore!($sio2);
                impl QspiInstance for crate::peripherals::[<SPI $num>] {}
            )?
        }
    }
}

#[cfg(spi2)]
cfg_if::cfg_if! {
    if #[cfg(esp32)] {
        spi_instance!(2, HSPICLK, HSPID, HSPIQ, HSPICS0, HSPIWP, HSPIHD);
    } else if #[cfg(any(esp32s2, esp32s3))] {
        spi_instance!(2, FSPICLK, FSPID, FSPIQ, FSPICS0, FSPIWP, FSPIHD);
    } else {
        spi_instance!(2, FSPICLK_MUX, FSPID, FSPIQ, FSPICS0, FSPIWP, FSPIHD);
    }
}

#[cfg(spi3)]
cfg_if::cfg_if! {
    if #[cfg(esp32)] {
        spi_instance!(3, VSPICLK, VSPID, VSPIQ, VSPICS0, HSPIWP, HSPIHD);
    } else if #[cfg(esp32s3)] {
        spi_instance!(3, SPI3_CLK, SPI3_D, SPI3_Q, SPI3_CS0, SPI3_WP, SPI3_HD);
    } else {
        spi_instance!(3, SPI3_CLK, SPI3_D, SPI3_Q, SPI3_CS0);
    }
}

impl Instance for super::AnySpi {
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

impl QspiInstance for super::AnySpi {}
