//! # Serial Peripheral Interface - Master Mode
//!
//! ## Overview
//! In this mode, the SPI acts as master and initiates the SPI transactions.
//!
//! ## Configuration
//! The peripheral can be used in full-duplex and half-duplex mode and can
//! leverage DMA for data transfers. It can also be used in blocking or async.
//!
//! ### Exclusive access to the SPI bus
//! If all you want to do is to communicate to a single device, and you initiate
//! transactions yourself, there are a number of ways to achieve this:
//!
//! - Use the [`FullDuplex`](embedded_hal_02::spi::FullDuplex) trait to
//!   read/write single bytes at a time,
//! - Use the [`SpiBus`](embedded_hal::spi::SpiBus) trait (requires the
//!   "embedded-hal" feature) and its associated functions to initiate
//!   transactions with simultaneous reads and writes, or
//! - Use the `ExclusiveDevice` struct from [`embedded-hal-bus`] or `SpiDevice`
//!   from [`embassy-embedded-hal`].
//!
//!
//! ### Shared SPI access
//! If you have multiple devices on the same SPI bus that each have their own CS
//! line, you may want to have a look at the implementations provided by
//! [`embedded-hal-bus`] and [`embassy-embedded-hal`].
//!
//! ## Usage
//! The module implements several third-party traits from embedded-hal@0.2.x,
//! embedded-hal@1.x.x and embassy-embedded-hal
//!
//! ## Example
//! ### SPI Initialization
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! # use crate::esp_hal::prelude::_fugit_RateExtU32;
//! # use esp_hal::spi::SpiMode;
//! # use esp_hal::spi::master::Spi;
//! # use esp_hal::gpio::Io;
//! # let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
//! let sclk = io.pins.gpio0;
//! let miso = io.pins.gpio2;
//! let mosi = io.pins.gpio1;
//! let cs = io.pins.gpio5;
//!
//! let mut spi = Spi::new(
//!     peripherals.SPI2,
//!     100.kHz(),
//!     SpiMode::Mode0,
//!     &mut clocks,
//! )
//! .with_pins(Some(sclk), Some(mosi), Some(miso), Some(cs));
//! # }
//! ```
//! 
//! [`embedded-hal-bus`]: https://docs.rs/embedded-hal-bus/latest/embedded_hal_bus/spi/index.html
//! [`embassy-embedded-hal`]: https://docs.embassy.dev/embassy-embedded-hal/git/default/shared_bus/index.html

use core::marker::PhantomData;

#[cfg(not(any(esp32, esp32s2)))]
use enumset::EnumSet;
#[cfg(not(any(esp32, esp32s2)))]
use enumset::EnumSetType;
use fugit::HertzU32;
#[cfg(feature = "place-spi-driver-in-ram")]
use procmacros::ram;

use super::{
    DuplexMode,
    Error,
    FullDuplexMode,
    HalfDuplexMode,
    IsFullDuplex,
    IsHalfDuplex,
    SpiBitOrder,
    SpiDataMode,
    SpiMode,
};
use crate::{
    clock::Clocks,
    dma::{DmaDescriptor, DmaPeripheral, Rx, Tx},
    gpio::{InputPin, InputSignal, OutputPin, OutputSignal},
    interrupt::InterruptHandler,
    peripheral::{Peripheral, PeripheralRef},
    peripherals::spi2::RegisterBlock,
    private,
    system::PeripheralClockControl,
};

/// Prelude for the SPI (Master) driver
pub mod prelude {
    pub use super::{
        Instance as _esp_hal_spi_master_Instance,
        InstanceDma as _esp_hal_spi_master_InstanceDma,
    };
}

/// Enumeration of possible SPI interrupt events.
#[cfg(not(any(esp32, esp32s2)))]
#[derive(EnumSetType)]
pub enum SpiInterrupt {
    /// Indicates that the SPI transaction has completed successfully.
    ///
    /// This interrupt is triggered when an SPI transaction has finished
    /// transmitting and receiving data.
    TransDone,
}

/// The size of the FIFO buffer for SPI
#[cfg(not(esp32s2))]
const FIFO_SIZE: usize = 64;
#[cfg(esp32s2)]
const FIFO_SIZE: usize = 72;

/// Padding byte for empty write transfers
const EMPTY_WRITE_PAD: u8 = 0x00u8;

#[allow(unused)]
const MAX_DMA_SIZE: usize = 32736;

/// SPI commands, each consisting of a 16-bit command value and a data mode.
///
/// Used to define specific commands sent over the SPI bus.
/// Can be [Command::None] if command phase should be suppressed.
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

/// Read and Write in half duplex mode.
pub trait HalfDuplexReadWrite {
    /// The associated error type that will be returned in the event of a
    /// failure.
    type Error;

    /// Half-duplex read.
    fn read(
        &mut self,
        data_mode: SpiDataMode,
        cmd: Command,
        address: Address,
        dummy: u8,
        buffer: &mut [u8],
    ) -> Result<(), Self::Error>;

    /// Half-duplex write.
    fn write(
        &mut self,
        data_mode: SpiDataMode,
        cmd: Command,
        address: Address,
        dummy: u8,
        buffer: &[u8],
    ) -> Result<(), Self::Error>;
}

/// SPI peripheral driver
pub struct Spi<'d, T, M> {
    spi: PeripheralRef<'d, T>,
    _mode: PhantomData<M>,
}

impl<'d, T, M> Spi<'d, T, M>
where
    T: Instance,
    M: IsFullDuplex,
{
    /// Read bytes from SPI.
    ///
    /// Sends out a stuffing byte for every byte to read. This function doesn't
    /// perform flushing. If you want to read the response to something you
    /// have written before, consider using [`Self::transfer`] instead.
    pub fn read_byte(&mut self) -> nb::Result<u8, Error> {
        self.spi.read_byte()
    }

    /// Write a byte to SPI.
    pub fn write_byte(&mut self, word: u8) -> nb::Result<(), Error> {
        self.spi.write_byte(word)
    }

    /// Write bytes to SPI.
    ///
    /// Copies the content of `words` in chunks of 64 bytes into the SPI
    /// transmission FIFO. If `words` is longer than 64 bytes, multiple
    /// sequential transfers are performed. This function will return before
    /// all bytes of the last chunk to transmit have been sent to the wire. If
    /// you must ensure that the whole messages was written correctly, use
    /// `flush`.
    pub fn write_bytes(&mut self, words: &[u8]) -> Result<(), Error> {
        self.spi.write_bytes(words)?;
        self.spi.flush()?;

        Ok(())
    }

    /// Sends `words` to the slave. Returns the `words` received from the slave
    pub fn transfer<'w>(&mut self, words: &'w mut [u8]) -> Result<&'w [u8], Error> {
        self.spi.transfer(words)
    }
}

impl<'d, T> Spi<'d, T, FullDuplexMode>
where
    T: Instance,
{
    /// Constructs an SPI instance in 8bit dataframe mode.
    ///
    /// All pins are optional. Setup these pins using
    /// [with_pins](Self::with_pins) or individual methods for each pin.
    pub fn new(
        spi: impl Peripheral<P = T> + 'd,
        frequency: HertzU32,
        mode: SpiMode,
        clocks: &Clocks<'d>,
    ) -> Spi<'d, T, FullDuplexMode> {
        crate::into_ref!(spi);
        Self::new_internal(spi, frequency, mode, clocks)
    }

    /// Assign the SCK (Serial Clock) pin for the SPI instance.
    ///
    /// Sets the specified pin to push-pull output and connects it to the SPI
    /// clock signal.
    pub fn with_sck<SCK: OutputPin>(self, sck: impl Peripheral<P = SCK> + 'd) -> Self {
        crate::into_ref!(sck);
        sck.set_to_push_pull_output(private::Internal);
        sck.connect_peripheral_to_output(self.spi.sclk_signal(), private::Internal);

        self
    }

    /// Assign the MOSI (Master Out Slave In) pin for the SPI instance.
    ///
    /// Sets the specified pin to push-pull output and connects it to the SPI
    /// MOSI signal.
    pub fn with_mosi<MOSI: OutputPin>(self, mosi: impl Peripheral<P = MOSI> + 'd) -> Self {
        crate::into_ref!(mosi);
        mosi.set_to_push_pull_output(private::Internal);
        mosi.connect_peripheral_to_output(self.spi.mosi_signal(), private::Internal);

        self
    }

    /// Assign the MISO (Master In Slave Out) pin for the SPI instance.
    ///
    /// Sets the specified pin to input and connects it to the SPI MISO signal.
    pub fn with_miso<MISO: InputPin>(self, miso: impl Peripheral<P = MISO> + 'd) -> Self {
        crate::into_ref!(miso);
        miso.set_to_input(private::Internal);
        miso.connect_input_to_peripheral(self.spi.miso_signal(), private::Internal);

        self
    }

    /// Assign the CS (Chip Select) pin for the SPI instance.
    ///
    /// Sets the specified pin to push-pull output and connects it to the SPI CS
    /// signal.
    pub fn with_cs<CS: OutputPin>(self, cs: impl Peripheral<P = CS> + 'd) -> Self {
        crate::into_ref!(cs);
        cs.set_to_push_pull_output(private::Internal);
        cs.connect_peripheral_to_output(self.spi.cs_signal(), private::Internal);

        self
    }

    /// Set the bit order for the SPI instance.
    ///
    /// The default is MSB first for both read and write.
    pub fn with_bit_order(mut self, read_order: SpiBitOrder, write_order: SpiBitOrder) -> Self {
        self.spi.set_bit_order(read_order, write_order);
        self
    }

    /// Setup pins for this SPI instance.
    ///
    /// All pins are optional. Pass [crate::gpio::NO_PIN] if you don't need the
    /// given pin.
    pub fn with_pins<SCK: OutputPin, MOSI: OutputPin, MISO: InputPin, CS: OutputPin>(
        self,
        sck: Option<impl Peripheral<P = SCK> + 'd>,
        mosi: Option<impl Peripheral<P = MOSI> + 'd>,
        miso: Option<impl Peripheral<P = MISO> + 'd>,
        cs: Option<impl Peripheral<P = CS> + 'd>,
    ) -> Self {
        if let Some(sck) = sck {
            crate::into_ref!(sck);
            sck.set_to_push_pull_output(private::Internal);
            sck.connect_peripheral_to_output(self.spi.sclk_signal(), private::Internal);
        }

        if let Some(mosi) = mosi {
            crate::into_ref!(mosi);
            mosi.set_to_push_pull_output(private::Internal);
            mosi.connect_peripheral_to_output(self.spi.mosi_signal(), private::Internal);
        }

        if let Some(miso) = miso {
            crate::into_ref!(miso);
            miso.set_to_input(private::Internal);
            miso.connect_input_to_peripheral(self.spi.miso_signal(), private::Internal);
        }

        if let Some(cs) = cs {
            crate::into_ref!(cs);
            cs.set_to_push_pull_output(private::Internal);
            cs.connect_peripheral_to_output(self.spi.cs_signal(), private::Internal);
        }

        self
    }

    pub(crate) fn new_internal(
        spi: PeripheralRef<'d, T>,
        frequency: HertzU32,
        mode: SpiMode,
        clocks: &Clocks<'d>,
    ) -> Spi<'d, T, FullDuplexMode> {
        spi.reset_peripheral();
        spi.enable_peripheral();

        let mut spi = Spi {
            spi,
            _mode: PhantomData,
        };
        spi.spi.setup(frequency, clocks);
        spi.spi.init();
        spi.spi.set_data_mode(mode);

        spi
    }

    /// Change the bus frequency of the SPI instance.
    ///
    /// This method allows user to update the bus frequency for the SPI
    /// communication after the instance has been created.
    pub fn change_bus_frequency(&mut self, frequency: HertzU32, clocks: &Clocks<'d>) {
        self.spi.ch_bus_freq(frequency, clocks);
    }
}

impl<'d, T> Spi<'d, T, HalfDuplexMode>
where
    T: ExtendedInstance,
{
    /// Constructs an SPI instance in half-duplex mode.
    ///
    /// All pins are optional. Setup these pins using
    /// [with_pins](Self::with_pins) or individual methods for each pin.
    pub fn new_half_duplex(
        spi: impl Peripheral<P = T> + 'd,
        frequency: HertzU32,
        mode: SpiMode,
        clocks: &Clocks<'d>,
    ) -> Spi<'d, T, HalfDuplexMode> {
        crate::into_ref!(spi);
        Self::new_internal(spi, frequency, mode, clocks)
    }

    /// Assign the SCK (Serial Clock) pin for the SPI instance.
    ///
    /// Sets the specified pin to push-pull output and connects it to the SPI
    /// clock signal.
    pub fn with_sck<SCK: OutputPin>(self, sck: impl Peripheral<P = SCK> + 'd) -> Self {
        crate::into_ref!(sck);
        sck.set_to_push_pull_output(private::Internal);
        sck.connect_peripheral_to_output(self.spi.sclk_signal(), private::Internal);

        self
    }

    /// Assign the MOSI (Master Out Slave In) pin for the SPI instance in
    /// half-duplex mode.
    ///
    /// Enables both input and output functionality for the pin, and connects it
    /// to the MOSI signal and SIO0 input signal.
    pub fn with_mosi<MOSI: OutputPin + InputPin>(
        self,
        mosi: impl Peripheral<P = MOSI> + 'd,
    ) -> Self {
        crate::into_ref!(mosi);
        mosi.enable_output(true, private::Internal);
        mosi.connect_peripheral_to_output(self.spi.mosi_signal(), private::Internal);
        mosi.enable_input(true, private::Internal);
        mosi.connect_input_to_peripheral(self.spi.sio0_input_signal(), private::Internal);

        self
    }

    /// Assign the MISO (Master In Slave Out) pin for the SPI instance in
    /// half-duplex mode.
    ///
    /// Enables both input and output functionality for the pin, and connects it
    /// to the MISO signal and SIO1 input signal.
    pub fn with_miso<MISO: OutputPin + InputPin>(
        self,
        miso: impl Peripheral<P = MISO> + 'd,
    ) -> Self {
        crate::into_ref!(miso);
        miso.enable_output(true, private::Internal);
        miso.connect_peripheral_to_output(self.spi.sio1_output_signal(), private::Internal);
        miso.enable_input(true, private::Internal);
        miso.connect_input_to_peripheral(self.spi.miso_signal(), private::Internal);

        self
    }

    /// Assign the SIO2 pin for the SPI instance.
    ///
    /// Enables both input and output functionality for the pin, and connects it
    /// to the SIO2 output and input signals.
    pub fn with_sio2<SIO2: OutputPin + InputPin>(
        self,
        sio2: impl Peripheral<P = SIO2> + 'd,
    ) -> Self {
        crate::into_ref!(sio2);
        sio2.enable_output(true, private::Internal);
        sio2.connect_peripheral_to_output(self.spi.sio2_output_signal(), private::Internal);
        sio2.enable_input(true, private::Internal);
        sio2.connect_input_to_peripheral(self.spi.sio2_input_signal(), private::Internal);

        self
    }

    /// Assign the SIO3 pin for the SPI instance.
    ///
    /// Enables both input and output functionality for the pin, and connects it
    /// to the SIO3 output and input signals.
    pub fn with_sio3<SIO3: OutputPin + InputPin>(
        self,
        sio3: impl Peripheral<P = SIO3> + 'd,
    ) -> Self {
        crate::into_ref!(sio3);
        sio3.enable_output(true, private::Internal);
        sio3.connect_peripheral_to_output(self.spi.sio3_output_signal(), private::Internal);
        sio3.enable_input(true, private::Internal);
        sio3.connect_input_to_peripheral(self.spi.sio3_input_signal(), private::Internal);

        self
    }

    /// Assign the CS (Chip Select) pin for the SPI instance.
    ///
    /// Sets the specified pin to push-pull output and connects it to the SPI CS
    /// signal.
    pub fn with_cs<CS: OutputPin>(self, cs: impl Peripheral<P = CS> + 'd) -> Self {
        crate::into_ref!(cs);
        cs.set_to_push_pull_output(private::Internal);
        cs.connect_peripheral_to_output(self.spi.cs_signal(), private::Internal);

        self
    }

    /// Setup pins for this SPI instance.
    ///
    /// All pins are optional. Pass [crate::gpio::NO_PIN] if you don't need the
    /// given pin.
    pub fn with_pins<
        SCK: OutputPin,
        MOSI: OutputPin + InputPin,
        MISO: OutputPin + InputPin,
        SIO2: OutputPin + InputPin,
        SIO3: OutputPin + InputPin,
        CS: OutputPin,
    >(
        self,
        sck: Option<impl Peripheral<P = SCK> + 'd>,
        mosi: Option<impl Peripheral<P = MOSI> + 'd>,
        miso: Option<impl Peripheral<P = MISO> + 'd>,
        sio2: Option<impl Peripheral<P = SIO2> + 'd>,
        sio3: Option<impl Peripheral<P = SIO3> + 'd>,
        cs: Option<impl Peripheral<P = CS> + 'd>,
    ) -> Self {
        if let Some(sck) = sck {
            crate::into_ref!(sck);
            sck.set_to_push_pull_output(private::Internal);
            sck.connect_peripheral_to_output(self.spi.sclk_signal(), private::Internal);
        }

        if let Some(mosi) = mosi {
            crate::into_ref!(mosi);
            mosi.enable_output(true, private::Internal);
            mosi.connect_peripheral_to_output(self.spi.mosi_signal(), private::Internal);
            mosi.enable_input(true, private::Internal);
            mosi.connect_input_to_peripheral(self.spi.sio0_input_signal(), private::Internal);
        }

        if let Some(miso) = miso {
            crate::into_ref!(miso);
            miso.enable_output(true, private::Internal);
            miso.connect_peripheral_to_output(self.spi.sio1_output_signal(), private::Internal);
            miso.enable_input(true, private::Internal);
            miso.connect_input_to_peripheral(self.spi.miso_signal(), private::Internal);
        }

        if let Some(sio2) = sio2 {
            crate::into_ref!(sio2);
            sio2.enable_output(true, private::Internal);
            sio2.connect_peripheral_to_output(self.spi.sio2_output_signal(), private::Internal);
            sio2.enable_input(true, private::Internal);
            sio2.connect_input_to_peripheral(self.spi.sio2_input_signal(), private::Internal);
        }

        if let Some(sio3) = sio3 {
            crate::into_ref!(sio3);
            sio3.enable_output(true, private::Internal);
            sio3.connect_peripheral_to_output(self.spi.sio3_output_signal(), private::Internal);
            sio3.enable_input(true, private::Internal);
            sio3.connect_input_to_peripheral(self.spi.sio3_input_signal(), private::Internal);
        }

        if let Some(cs) = cs {
            crate::into_ref!(cs);
            cs.set_to_push_pull_output(private::Internal);
            cs.connect_peripheral_to_output(self.spi.cs_signal(), private::Internal);
        }

        self
    }

    pub(crate) fn new_internal(
        spi: PeripheralRef<'d, T>,
        frequency: HertzU32,
        mode: SpiMode,
        clocks: &Clocks<'d>,
    ) -> Spi<'d, T, HalfDuplexMode> {
        spi.reset_peripheral();
        spi.enable_peripheral();

        let mut spi = Spi {
            spi,
            _mode: PhantomData,
        };
        spi.spi.setup(frequency, clocks);
        spi.spi.init();
        spi.spi.set_data_mode(mode);

        spi
    }

    /// Change the bus frequency of the SPI instance in half-duplex mode.
    ///
    /// This method allows you to update the bus frequency for the SPI
    /// communication after the instance has been created.
    pub fn change_bus_frequency(&mut self, frequency: HertzU32, clocks: &Clocks<'d>) {
        self.spi.ch_bus_freq(frequency, clocks);
    }

    /// Set the bit order for the SPI instance.
    ///
    /// The default is MSB first for both read and write.
    pub fn with_bit_order(mut self, read_order: SpiBitOrder, write_order: SpiBitOrder) -> Self {
        self.spi.set_bit_order(read_order, write_order);
        self
    }
}

impl<T, M> HalfDuplexReadWrite for Spi<'_, T, M>
where
    T: Instance,
    M: IsHalfDuplex,
{
    type Error = Error;

    fn read(
        &mut self,
        data_mode: SpiDataMode,
        cmd: Command,
        address: Address,
        dummy: u8,
        buffer: &mut [u8],
    ) -> Result<(), Self::Error> {
        if buffer.len() > FIFO_SIZE {
            return Err(Error::FifoSizeExeeded);
        }

        if buffer.is_empty() {
            return Err(Error::Unsupported);
        }

        self.spi
            .init_spi_data_mode(cmd.mode(), address.mode(), data_mode);
        self.spi.read_bytes_half_duplex(cmd, address, dummy, buffer)
    }

    fn write(
        &mut self,
        data_mode: SpiDataMode,
        cmd: Command,
        address: Address,
        dummy: u8,
        buffer: &[u8],
    ) -> Result<(), Self::Error> {
        if buffer.len() > FIFO_SIZE {
            return Err(Error::FifoSizeExeeded);
        }

        self.spi
            .init_spi_data_mode(cmd.mode(), address.mode(), data_mode);
        self.spi
            .write_bytes_half_duplex(cmd, address, dummy, buffer)
    }
}

#[cfg(feature = "embedded-hal-02")]
impl<T, M> embedded_hal_02::spi::FullDuplex<u8> for Spi<'_, T, M>
where
    T: Instance,
    M: IsFullDuplex,
{
    type Error = Error;

    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        self.read_byte()
    }

    fn send(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        self.write_byte(word)
    }
}

#[cfg(feature = "embedded-hal-02")]
impl<T, M> embedded_hal_02::blocking::spi::Transfer<u8> for Spi<'_, T, M>
where
    T: Instance,
    M: IsFullDuplex,
{
    type Error = Error;

    fn transfer<'w>(&mut self, words: &'w mut [u8]) -> Result<&'w [u8], Self::Error> {
        self.transfer(words)
    }
}

#[cfg(feature = "embedded-hal-02")]
impl<T, M> embedded_hal_02::blocking::spi::Write<u8> for Spi<'_, T, M>
where
    T: Instance,
    M: IsFullDuplex,
{
    type Error = Error;

    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        self.write_bytes(words)?;
        self.spi.flush()
    }
}

/// DMA (Direct Memory Access) funtionality (Master).
pub mod dma {
    use core::{
        cmp::min,
        sync::atomic::{fence, Ordering},
    };

    use super::*;
    #[cfg(feature = "async")]
    use crate::dma::asynch::{DmaRxFuture, DmaTxFuture};
    #[cfg(spi3)]
    use crate::dma::Spi3Peripheral;
    use crate::{
        dma::{
            Channel,
            DmaChannel,
            DmaRxBuf,
            DmaTxBuf,
            RxPrivate,
            Spi2Peripheral,
            SpiPeripheral,
            TxPrivate,
        },
        Blocking,
        InterruptConfigurable,
        Mode,
    };

    impl<'d, M> Spi<'d, crate::peripherals::SPI2, M>
    where
        M: DuplexMode,
    {
        /// Configures the SPI instance to use DMA with the specified channel.
        ///
        /// This method prepares the SPI instance for DMA transfers. It
        /// initializes the DMA channel for transmission and returns an
        /// instance of `SpiDma` that supports DMA operations.
        pub fn with_dma<C, DmaMode>(
            self,
            mut channel: Channel<'d, C, DmaMode>,
        ) -> SpiDma<'d, crate::peripherals::SPI2, C, M, DmaMode>
        where
            C: DmaChannel,
            C::P: SpiPeripheral + Spi2Peripheral,
            DmaMode: Mode,
        {
            channel.tx.init_channel(); // no need to call this for both, TX and RX

            SpiDma {
                spi: self.spi,
                channel,
                _mode: PhantomData,
            }
        }
    }

    #[cfg(spi3)]
    impl<'d, M> Spi<'d, crate::peripherals::SPI3, M>
    where
        M: DuplexMode,
    {
        /// Configures the SPI3 instance to use DMA with the specified channel.
        ///
        /// This method prepares the SPI instance for DMA transfers using SPI3
        /// and returns an instance of `SpiDma` that supports DMA
        /// operations.
        pub fn with_dma<C, DmaMode>(
            self,
            mut channel: Channel<'d, C, DmaMode>,
        ) -> SpiDma<'d, crate::peripherals::SPI3, C, M, DmaMode>
        where
            C: DmaChannel,
            C::P: SpiPeripheral + Spi3Peripheral,
            DmaMode: Mode,
        {
            channel.tx.init_channel(); // no need to call this for both, TX and RX

            SpiDma {
                spi: self.spi,
                channel,
                _mode: PhantomData,
            }
        }
    }

    /// A DMA capable SPI instance.
    pub struct SpiDma<'d, T, C, M, DmaMode>
    where
        C: DmaChannel,
        C::P: SpiPeripheral,
        M: DuplexMode,
        DmaMode: Mode,
    {
        pub(crate) spi: PeripheralRef<'d, T>,
        pub(crate) channel: Channel<'d, C, DmaMode>,
        _mode: PhantomData<M>,
    }

    impl<'d, T, C, M, DmaMode> core::fmt::Debug for SpiDma<'d, T, C, M, DmaMode>
    where
        C: DmaChannel,
        C::P: SpiPeripheral,
        M: DuplexMode,
        DmaMode: Mode,
    {
        /// Formats the `SpiDma` instance for debugging purposes.
        ///
        /// This method returns a debug struct with the name "SpiDma" without
        /// exposing internal details.
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
            f.debug_struct("SpiDma").finish()
        }
    }

    impl<'d, T, C, M, DmaMode> SpiDma<'d, T, C, M, DmaMode>
    where
        T: InstanceDma,
        C: DmaChannel,
        C::P: SpiPeripheral,
        M: DuplexMode,
        DmaMode: Mode,
    {
        /// Sets the interrupt handler
        ///
        /// Interrupts are not enabled at the peripheral level here.
        pub fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
            self.spi.set_interrupt_handler(handler);
        }

        /// Listen for the given interrupts
        #[cfg(not(any(esp32, esp32s2)))]
        pub fn listen(&mut self, interrupts: EnumSet<SpiInterrupt>) {
            self.spi.listen(interrupts);
        }

        /// Unlisten the given interrupts
        #[cfg(not(any(esp32, esp32s2)))]
        pub fn unlisten(&mut self, interrupts: EnumSet<SpiInterrupt>) {
            self.spi.unlisten(interrupts);
        }

        /// Gets asserted interrupts
        #[cfg(not(any(esp32, esp32s2)))]
        pub fn interrupts(&mut self) -> EnumSet<SpiInterrupt> {
            self.spi.interrupts()
        }

        /// Resets asserted interrupts
        #[cfg(not(any(esp32, esp32s2)))]
        pub fn clear_interrupts(&mut self, interrupts: EnumSet<SpiInterrupt>) {
            self.spi.clear_interrupts(interrupts);
        }
    }

    impl<'d, T, C, M, DmaMode> crate::private::Sealed for SpiDma<'d, T, C, M, DmaMode>
    where
        T: InstanceDma,
        C: DmaChannel,
        C::P: SpiPeripheral,
        M: DuplexMode,
        DmaMode: Mode,
    {
    }

    impl<'d, T, C, M, DmaMode> InterruptConfigurable for SpiDma<'d, T, C, M, DmaMode>
    where
        T: InstanceDma,
        C: DmaChannel,
        C::P: SpiPeripheral,
        M: DuplexMode,
        DmaMode: Mode,
    {
        /// Configures the interrupt handler for the DMA-enabled SPI instance.
        fn set_interrupt_handler(&mut self, handler: crate::interrupt::InterruptHandler) {
            SpiDma::set_interrupt_handler(self, handler);
        }
    }

    impl<'d, T, C, M, DmaMode> SpiDma<'d, T, C, M, DmaMode>
    where
        T: InstanceDma,
        C: DmaChannel,
        C::P: SpiPeripheral,
        M: DuplexMode,
        DmaMode: Mode,
    {
        /// Changes the SPI bus frequency for the DMA-enabled SPI instance.
        pub fn change_bus_frequency(&mut self, frequency: HertzU32, clocks: &Clocks<'d>) {
            self.spi.ch_bus_freq(frequency, clocks);
        }
    }

    impl<'d, T, C> SpiDma<'d, T, C, FullDuplexMode, Blocking>
    where
        T: InstanceDma,
        C: DmaChannel,
        C::P: SpiPeripheral,
    {
        /// Configures the DMA buffers for the SPI instance.
        ///
        /// This method sets up both TX and RX buffers for DMA transfers.
        /// It returns an instance of `SpiDmaBus` that can be used for SPI
        /// communication.
        pub fn with_buffers(
            self,
            dma_tx_buf: DmaTxBuf,
            dma_rx_buf: DmaRxBuf,
        ) -> SpiDmaBus<'d, T, C> {
            SpiDmaBus::new(self, dma_tx_buf, dma_rx_buf)
        }
    }

    #[cfg(feature = "async")]
    impl<'d, T, C> SpiDma<'d, T, C, FullDuplexMode, crate::Async>
    where
        T: InstanceDma,
        C: DmaChannel,
        C::P: SpiPeripheral,
    {
        /// Configures the DMA buffers for asynchronous SPI communication.
        ///
        /// This method sets up both TX and RX buffers for DMA transfers.
        /// It eturns an instance of `SpiDmaAsyncBus` to be used for
        /// asynchronous SPI operations.
        pub fn with_buffers(
            self,
            dma_tx_buf: DmaTxBuf,
            dma_rx_buf: DmaRxBuf,
        ) -> asynch::SpiDmaAsyncBus<'d, T, C> {
            asynch::SpiDmaAsyncBus::new(self, dma_tx_buf, dma_rx_buf)
        }
    }

    /// A structure representing a DMA transfer for SPI.
    ///
    /// This structure holds references to the SPI instance, DMA buffers, and
    /// transfer status.
    pub struct SpiDmaTransfer<'d, T, C, M, DmaMode, Buf>
    where
        C: DmaChannel,
        C::P: SpiPeripheral,
        M: DuplexMode,
        DmaMode: Mode,
    {
        spi_dma: SpiDma<'d, T, C, M, DmaMode>,
        dma_buf: Buf,
        is_rx: bool,
        is_tx: bool,

        rx_future_awaited: bool,
        tx_future_awaited: bool,
    }

    impl<'d, T, C, M, DmaMode, Buf> SpiDmaTransfer<'d, T, C, M, DmaMode, Buf>
    where
        T: Instance,
        C: DmaChannel,
        C::P: SpiPeripheral,
        M: DuplexMode,
        DmaMode: Mode,
    {
        fn new(
            spi_dma: SpiDma<'d, T, C, M, DmaMode>,
            dma_buf: Buf,
            is_rx: bool,
            is_tx: bool,
        ) -> Self {
            Self {
                spi_dma,
                dma_buf,
                is_rx,
                is_tx,
                rx_future_awaited: false,
                tx_future_awaited: false,
            }
        }

        /// Checks if the DMA transfer is complete.
        ///
        /// This method returns `true` if both TX and RX operations are done,
        /// and the SPI instance is no longer busy.
        pub fn is_done(&self) -> bool {
            if self.is_tx && !self.tx_future_awaited && !self.spi_dma.channel.tx.is_done() {
                return false;
            }
            if self.spi_dma.spi.busy() {
                return false;
            }
            if self.is_rx && !self.rx_future_awaited {
                // If this is an asymmetric transfer and the RX side is smaller, the RX channel
                // will never be "done" as it won't have enough descriptors/buffer to receive
                // the EOF bit from the SPI. So instead the RX channel will hit
                // a "descriptor empty" which means the DMA is written as much
                // of the received data as possible into the buffer and
                // discarded the rest. The user doesn't care about this discarded data.

                if !self.spi_dma.channel.rx.is_done()
                    && !self.spi_dma.channel.rx.has_dscr_empty_error()
                {
                    return false;
                }
            }
            true
        }

        /// Waits for the DMA transfer to complete.
        ///
        /// This method blocks until the transfer is finished and returns the
        /// `SpiDma` instance and the associated buffer.
        pub fn wait(mut self) -> (SpiDma<'d, T, C, M, DmaMode>, Buf) {
            self.spi_dma.spi.flush().ok();
            fence(Ordering::Acquire);
            (self.spi_dma, self.dma_buf)
        }
    }

    #[cfg(feature = "async")]
    impl<'d, T, C, M, Buf> SpiDmaTransfer<'d, T, C, M, crate::Async, Buf>
    where
        T: Instance,
        C: DmaChannel,
        C::P: SpiPeripheral,
        M: DuplexMode,
    {
        /// Waits for the DMA transfer to complete asynchronously.
        ///
        /// This method awaits the completion of both TX and RX operations.
        pub async fn wait_for_done(&mut self) {
            if self.is_tx && !self.tx_future_awaited {
                let _ = DmaTxFuture::new(&mut self.spi_dma.channel.tx).await;
                self.tx_future_awaited = true;
            }

            // As a future enhancement, setup Spi Future in here as well.

            if self.is_rx && !self.rx_future_awaited {
                let _ = DmaRxFuture::new(&mut self.spi_dma.channel.rx).await;
                self.rx_future_awaited = true;
            }
        }
    }

    impl<'d, T, C, M, DmaMode> SpiDma<'d, T, C, M, DmaMode>
    where
        T: InstanceDma,
        C: DmaChannel,
        C::P: SpiPeripheral,
        M: IsFullDuplex,
        DmaMode: Mode,
    {
        /// Perform a DMA write.
        ///
        /// This will return a [SpiDmaTransfer] owning the buffer and the
        /// SPI instance. The maximum amount of data to be sent is 32736
        /// bytes.
        #[allow(clippy::type_complexity)]
        #[cfg_attr(feature = "place-spi-driver-in-ram", ram)]
        pub fn dma_write(
            mut self,
            buffer: DmaTxBuf,
        ) -> Result<SpiDmaTransfer<'d, T, C, M, DmaMode, DmaTxBuf>, (Error, Self, DmaTxBuf)>
        {
            let bytes_to_write = buffer.len();
            if bytes_to_write > MAX_DMA_SIZE {
                return Err((Error::MaxDmaTransferSizeExceeded, self, buffer));
            }

            let result = unsafe {
                self.spi
                    .start_write_bytes_dma(buffer.first(), bytes_to_write, &mut self.channel.tx)
            };
            if let Err(e) = result {
                return Err((e, self, buffer));
            }

            Ok(SpiDmaTransfer::new(self, buffer, false, true))
        }

        /// Perform a DMA read.
        ///
        /// This will return a [SpiDmaTransfer] owning the buffer and
        /// the SPI instance. The maximum amount of data to be
        /// received is 32736 bytes.
        #[allow(clippy::type_complexity)]
        #[cfg_attr(feature = "place-spi-driver-in-ram", ram)]
        pub fn dma_read(
            mut self,
            buffer: DmaRxBuf,
        ) -> Result<SpiDmaTransfer<'d, T, C, M, DmaMode, DmaRxBuf>, (Error, Self, DmaRxBuf)>
        {
            let bytes_to_read = buffer.len();
            if bytes_to_read > MAX_DMA_SIZE {
                return Err((Error::MaxDmaTransferSizeExceeded, self, buffer));
            }

            let result = unsafe {
                self.spi
                    .start_read_bytes_dma(buffer.first(), bytes_to_read, &mut self.channel.rx)
            };
            if let Err(e) = result {
                return Err((e, self, buffer));
            }

            Ok(SpiDmaTransfer::new(self, buffer, true, false))
        }

        /// Perform a DMA transfer
        ///
        /// This will return a [SpiDmaTransfer] owning the buffers and
        /// the SPI instance. The maximum amount of data to be
        /// sent/received is 32736 bytes.
        #[allow(clippy::type_complexity)]
        pub fn dma_transfer(
            mut self,
            tx_buffer: DmaTxBuf,
            rx_buffer: DmaRxBuf,
        ) -> Result<
            SpiDmaTransfer<'d, T, C, M, DmaMode, (DmaTxBuf, DmaRxBuf)>,
            (Error, Self, DmaTxBuf, DmaRxBuf),
        > {
            let bytes_to_write = tx_buffer.len();
            let bytes_to_read = rx_buffer.len();

            if bytes_to_write > MAX_DMA_SIZE || bytes_to_read > MAX_DMA_SIZE {
                return Err((
                    Error::MaxDmaTransferSizeExceeded,
                    self,
                    tx_buffer,
                    rx_buffer,
                ));
            }

            let result = unsafe {
                self.spi.start_transfer_dma(
                    tx_buffer.first(),
                    rx_buffer.first(),
                    bytes_to_write,
                    bytes_to_read,
                    &mut self.channel.tx,
                    &mut self.channel.rx,
                )
            };
            if let Err(e) = result {
                return Err((e, self, tx_buffer, rx_buffer));
            }

            Ok(SpiDmaTransfer::new(
                self,
                (tx_buffer, rx_buffer),
                true,
                true,
            ))
        }
    }

    impl<'d, T, C, M, DmaMode> SpiDma<'d, T, C, M, DmaMode>
    where
        T: InstanceDma,
        C: DmaChannel,
        C::P: SpiPeripheral,
        M: IsHalfDuplex,
        DmaMode: Mode,
    {
        /// Perform a half-duplex read operation using DMA.
        #[allow(clippy::type_complexity)]
        #[cfg_attr(feature = "place-spi-driver-in-ram", ram)]
        pub fn read(
            mut self,
            data_mode: SpiDataMode,
            cmd: Command,
            address: Address,
            dummy: u8,
            buffer: DmaRxBuf,
        ) -> Result<SpiDmaTransfer<'d, T, C, M, DmaMode, DmaRxBuf>, (Error, Self, DmaRxBuf)>
        {
            let bytes_to_read = buffer.len();
            if bytes_to_read > MAX_DMA_SIZE {
                return Err((Error::MaxDmaTransferSizeExceeded, self, buffer));
            }

            self.spi.init_half_duplex(
                false,
                !cmd.is_none(),
                !address.is_none(),
                false,
                dummy != 0,
                bytes_to_read == 0,
            );
            self.spi
                .init_spi_data_mode(cmd.mode(), address.mode(), data_mode);

            // set cmd, address, dummy cycles
            let reg_block = self.spi.register_block();
            if !cmd.is_none() {
                reg_block.user2().modify(|_, w| unsafe {
                    w.usr_command_bitlen()
                        .bits((cmd.width() - 1) as u8)
                        .usr_command_value()
                        .bits(cmd.value())
                });
            }

            #[cfg(not(esp32))]
            if !address.is_none() {
                reg_block.user1().modify(|_, w| unsafe {
                    w.usr_addr_bitlen().bits((address.width() - 1) as u8)
                });

                let addr = address.value() << (32 - address.width());
                reg_block
                    .addr()
                    .write(|w| unsafe { w.usr_addr_value().bits(addr) });
            }

            #[cfg(esp32)]
            if !address.is_none() {
                reg_block.user1().modify(|r, w| unsafe {
                    w.bits(r.bits() & !(0x3f << 26) | (((address.width() - 1) as u32) & 0x3f) << 26)
                });

                let addr = address.value() << (32 - address.width());
                reg_block.addr().write(|w| unsafe { w.bits(addr) });
            }

            if dummy > 0 {
                reg_block
                    .user1()
                    .modify(|_, w| unsafe { w.usr_dummy_cyclelen().bits(dummy - 1) });
            }

            let result = unsafe {
                self.spi
                    .start_read_bytes_dma(buffer.first(), bytes_to_read, &mut self.channel.rx)
            };
            if let Err(e) = result {
                return Err((e, self, buffer));
            }

            Ok(SpiDmaTransfer::new(self, buffer, bytes_to_read > 0, false))
        }

        /// Perform a half-duplex write operation using DMA.
        #[allow(clippy::type_complexity)]
        #[cfg_attr(feature = "place-spi-driver-in-ram", ram)]
        pub fn write(
            mut self,
            data_mode: SpiDataMode,
            cmd: Command,
            address: Address,
            dummy: u8,
            buffer: DmaTxBuf,
        ) -> Result<SpiDmaTransfer<'d, T, C, M, DmaMode, DmaTxBuf>, (Error, Self, DmaTxBuf)>
        {
            let bytes_to_write = buffer.len();
            if bytes_to_write > MAX_DMA_SIZE {
                return Err((Error::MaxDmaTransferSizeExceeded, self, buffer));
            }

            self.spi.init_half_duplex(
                true,
                !cmd.is_none(),
                !address.is_none(),
                false,
                dummy != 0,
                bytes_to_write == 0,
            );
            self.spi
                .init_spi_data_mode(cmd.mode(), address.mode(), data_mode);

            // set cmd, address, dummy cycles
            let reg_block = self.spi.register_block();
            if !cmd.is_none() {
                reg_block.user2().modify(|_, w| unsafe {
                    w.usr_command_bitlen()
                        .bits((cmd.width() - 1) as u8)
                        .usr_command_value()
                        .bits(cmd.value())
                });
            }

            #[cfg(not(esp32))]
            if !address.is_none() {
                reg_block.user1().modify(|_, w| unsafe {
                    w.usr_addr_bitlen().bits((address.width() - 1) as u8)
                });

                let addr = address.value() << (32 - address.width());
                reg_block
                    .addr()
                    .write(|w| unsafe { w.usr_addr_value().bits(addr) });
            }

            #[cfg(esp32)]
            if !address.is_none() {
                reg_block.user1().modify(|r, w| unsafe {
                    w.bits(r.bits() & !(0x3f << 26) | (((address.width() - 1) as u32) & 0x3f) << 26)
                });

                let addr = address.value() << (32 - address.width());
                reg_block.addr().write(|w| unsafe { w.bits(addr) });
            }

            if dummy > 0 {
                reg_block
                    .user1()
                    .modify(|_, w| unsafe { w.usr_dummy_cyclelen().bits(dummy - 1) });
            }

            let result = unsafe {
                self.spi
                    .start_write_bytes_dma(buffer.first(), bytes_to_write, &mut self.channel.tx)
            };
            if let Err(e) = result {
                return Err((e, self, buffer));
            }

            Ok(SpiDmaTransfer::new(self, buffer, false, bytes_to_write > 0))
        }
    }

    /// A DMA-capable SPI bus that handles full-duplex transfers.
    ///
    /// This structure is responsible for managing SPI transfers using DMA
    /// buffers.
    pub struct SpiDmaBus<'d, T, C>
    where
        T: InstanceDma,
        C: DmaChannel,
        C::P: SpiPeripheral,
    {
        spi_dma: Option<SpiDma<'d, T, C, FullDuplexMode, crate::Blocking>>,
        buffers: Option<(DmaTxBuf, DmaRxBuf)>,
    }

    impl<'d, T, C> SpiDmaBus<'d, T, C>
    where
        T: InstanceDma,
        C: DmaChannel,
        C::P: SpiPeripheral,
    {
        /// Creates a new `SpiDmaBus` with the specified SPI instance and DMA
        /// buffers.
        pub fn new(
            spi_dma: SpiDma<'d, T, C, FullDuplexMode, crate::Blocking>,
            tx_buffer: DmaTxBuf,
            rx_buffer: DmaRxBuf,
        ) -> Self {
            Self {
                spi_dma: Some(spi_dma),
                buffers: Some((tx_buffer, rx_buffer)),
            }
        }

        /// Reads data from the SPI bus using DMA.
        pub fn read(&mut self, words: &mut [u8]) -> Result<(), Error> {
            let mut spi_dma = self.spi_dma.take().unwrap();
            let (tx_buf, mut rx_buf) = self.buffers.take().unwrap();

            for chunk in words.chunks_mut(rx_buf.capacity()) {
                rx_buf.set_length(chunk.len());

                let transfer = match spi_dma.dma_read(rx_buf) {
                    Ok(transfer) => transfer,
                    Err((e, spi, rx)) => {
                        self.spi_dma = Some(spi);
                        self.buffers = Some((tx_buf, rx));
                        return Err(e);
                    }
                };
                (spi_dma, rx_buf) = transfer.wait();

                let bytes_read = rx_buf.read_received_data(chunk);
                debug_assert_eq!(bytes_read, chunk.len());
            }

            self.spi_dma = Some(spi_dma);
            self.buffers = Some((tx_buf, rx_buf));

            Ok(())
        }

        /// Writes data to the SPI bus using DMA.
        pub fn write(&mut self, words: &[u8]) -> Result<(), Error> {
            let mut spi_dma = self.spi_dma.take().unwrap();
            let (mut tx_buf, rx_buf) = self.buffers.take().unwrap();

            for chunk in words.chunks(tx_buf.capacity()) {
                tx_buf.fill(chunk);

                let transfer = match spi_dma.dma_write(tx_buf) {
                    Ok(transfer) => transfer,
                    Err((e, spi, tx)) => {
                        self.spi_dma = Some(spi);
                        self.buffers = Some((tx, rx_buf));
                        return Err(e);
                    }
                };
                (spi_dma, tx_buf) = transfer.wait();
            }

            self.spi_dma = Some(spi_dma);
            self.buffers = Some((tx_buf, rx_buf));

            Ok(())
        }

        /// Transfers data to and from the SPI bus simultaneously using DMA.
        pub fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Error> {
            let mut spi_dma = self.spi_dma.take().unwrap();
            let (mut tx_buf, mut rx_buf) = self.buffers.take().unwrap();

            let chunk_size = min(tx_buf.capacity(), rx_buf.capacity());

            let common_length = min(read.len(), write.len());
            let (read_common, read_remainder) = read.split_at_mut(common_length);
            let (write_common, write_remainder) = write.split_at(common_length);

            for (read_chunk, write_chunk) in read_common
                .chunks_mut(chunk_size)
                .zip(write_common.chunks(chunk_size))
            {
                tx_buf.fill(write_chunk);
                rx_buf.set_length(read_chunk.len());

                let transfer = match spi_dma.dma_transfer(tx_buf, rx_buf) {
                    Ok(transfer) => transfer,
                    Err((e, spi, tx, rx)) => {
                        self.spi_dma = Some(spi);
                        self.buffers = Some((tx, rx));
                        return Err(e);
                    }
                };
                (spi_dma, (tx_buf, rx_buf)) = transfer.wait();

                let bytes_read = rx_buf.read_received_data(read_chunk);
                debug_assert_eq!(bytes_read, read_chunk.len());
            }

            self.spi_dma = Some(spi_dma);
            self.buffers = Some((tx_buf, rx_buf));

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
            let mut spi_dma = self.spi_dma.take().unwrap();
            let (mut tx_buf, mut rx_buf) = self.buffers.take().unwrap();

            let chunk_size = min(tx_buf.capacity(), rx_buf.capacity());

            for chunk in words.chunks_mut(chunk_size) {
                tx_buf.fill(chunk);
                rx_buf.set_length(chunk.len());

                let transfer = match spi_dma.dma_transfer(tx_buf, rx_buf) {
                    Ok(transfer) => transfer,
                    Err((e, spi, tx, rx)) => {
                        self.spi_dma = Some(spi);
                        self.buffers = Some((tx, rx));
                        return Err(e);
                    }
                };
                (spi_dma, (tx_buf, rx_buf)) = transfer.wait();

                let bytes_read = rx_buf.read_received_data(chunk);
                debug_assert_eq!(bytes_read, chunk.len());
            }

            self.spi_dma = Some(spi_dma);
            self.buffers = Some((tx_buf, rx_buf));

            Ok(())
        }
    }

    #[cfg(feature = "embedded-hal-02")]
    impl<'d, T, C> embedded_hal_02::blocking::spi::Transfer<u8> for SpiDmaBus<'d, T, C>
    where
        T: InstanceDma,
        C: DmaChannel,
        C::P: SpiPeripheral,
    {
        type Error = super::Error;

        fn transfer<'w>(&mut self, words: &'w mut [u8]) -> Result<&'w [u8], Self::Error> {
            self.transfer_in_place(words)?;
            Ok(words)
        }
    }

    #[cfg(feature = "embedded-hal-02")]
    impl<'d, T, C> embedded_hal_02::blocking::spi::Write<u8> for SpiDmaBus<'d, T, C>
    where
        T: InstanceDma,
        C: DmaChannel,
        C::P: SpiPeripheral,
    {
        type Error = super::Error;

        fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
            self.write(words)?;
            Ok(())
        }
    }

    /// Async functionality
    #[cfg(feature = "async")]
    pub mod asynch {
        use core::{cmp::min, mem::take};

        use embedded_hal::spi::ErrorType;

        use super::*;

        #[derive(Default)]
        enum State<'d, T, C>
        where
            T: InstanceDma,
            C: DmaChannel,
            C::P: SpiPeripheral,
        {
            Idle(
                SpiDma<'d, T, C, FullDuplexMode, crate::Async>,
                DmaTxBuf,
                DmaRxBuf,
            ),
            Reading(
                SpiDmaTransfer<'d, T, C, FullDuplexMode, crate::Async, DmaRxBuf>,
                DmaTxBuf,
            ),
            Writing(
                SpiDmaTransfer<'d, T, C, FullDuplexMode, crate::Async, DmaTxBuf>,
                DmaRxBuf,
            ),
            Transferring(
                SpiDmaTransfer<'d, T, C, FullDuplexMode, crate::Async, (DmaTxBuf, DmaRxBuf)>,
            ),
            #[default]
            InUse,
        }

        /// An asynchronous DMA-capable SPI bus for full-duplex operations.
        ///
        /// This struct provides an interface for SPI operations using DMA in an
        /// asynchronous way.
        pub struct SpiDmaAsyncBus<'d, T, C>
        where
            T: InstanceDma,
            C: DmaChannel,
            C::P: SpiPeripheral,
        {
            state: State<'d, T, C>,
        }

        impl<'d, T, C> SpiDmaAsyncBus<'d, T, C>
        where
            T: InstanceDma,
            C: DmaChannel,
            C::P: SpiPeripheral,
        {
            /// Creates a new asynchronous DMA SPI bus instance.
            ///
            /// Initializes the bus with the provided SPI instance and DMA
            /// buffers for transmit and receive operations.
            pub fn new(
                spi: SpiDma<'d, T, C, FullDuplexMode, crate::Async>,
                dma_tx_buf: DmaTxBuf,
                dma_rx_buf: DmaRxBuf,
            ) -> Self {
                Self {
                    state: State::Idle(spi, dma_tx_buf, dma_rx_buf),
                }
            }

            /// Waits for the current SPI DMA transfer to complete, ensuring the
            /// bus is idle.
            async fn wait_for_idle(
                &mut self,
            ) -> (
                SpiDma<'d, T, C, FullDuplexMode, crate::Async>,
                DmaTxBuf,
                DmaRxBuf,
            ) {
                match &mut self.state {
                    State::Idle(_, _, _) => (),
                    State::Reading(transfer, _) => transfer.wait_for_done().await,
                    State::Writing(transfer, _) => transfer.wait_for_done().await,
                    State::Transferring(transfer) => transfer.wait_for_done().await,
                    State::InUse => unreachable!(),
                }
                match take(&mut self.state) {
                    State::Idle(spi, tx_buf, rx_buf) => (spi, tx_buf, rx_buf),
                    State::Reading(transfer, tx_buf) => {
                        let (spi, rx_buf) = transfer.wait();
                        (spi, tx_buf, rx_buf)
                    }
                    State::Writing(transfer, rx_buf) => {
                        let (spi, tx_buf) = transfer.wait();
                        (spi, tx_buf, rx_buf)
                    }
                    State::Transferring(transfer) => {
                        let (spi, (tx_buf, rx_buf)) = transfer.wait();
                        (spi, tx_buf, rx_buf)
                    }
                    State::InUse => unreachable!(),
                }
            }
        }

        impl<'d, T, C> ErrorType for SpiDmaAsyncBus<'d, T, C>
        where
            T: InstanceDma,
            C: DmaChannel,
            C::P: SpiPeripheral,
        {
            type Error = Error;
        }

        impl<'d, T, C> embedded_hal_async::spi::SpiBus for SpiDmaAsyncBus<'d, T, C>
        where
            T: InstanceDma,
            C: DmaChannel,
            C::P: SpiPeripheral,
        {
            /// Asynchronously reads data from the SPI bus into the provided
            /// buffer.
            async fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
                // Get previous transfer.
                let (mut spi_dma, mut tx_buf, mut rx_buf) = self.wait_for_idle().await;

                for chunk in words.chunks_mut(rx_buf.capacity()) {
                    rx_buf.set_length(chunk.len());

                    match spi_dma.dma_read(rx_buf) {
                        Ok(transfer) => {
                            self.state = State::Reading(transfer, tx_buf);
                        }
                        Err((e, spi, rx)) => {
                            self.state = State::Idle(spi, tx_buf, rx);
                            return Err(e);
                        }
                    };

                    match &mut self.state {
                        State::Reading(transfer, _) => transfer.wait_for_done().await,
                        _ => unreachable!(),
                    };
                    (spi_dma, tx_buf, rx_buf) = match take(&mut self.state) {
                        State::Reading(transfer, tx_buf) => {
                            let (spi, rx_buf) = transfer.wait();
                            (spi, tx_buf, rx_buf)
                        }
                        _ => unreachable!(),
                    };

                    let bytes_read = rx_buf.read_received_data(chunk);
                    debug_assert_eq!(bytes_read, chunk.len());
                }

                self.state = State::Idle(spi_dma, tx_buf, rx_buf);

                Ok(())
            }

            /// Asynchronously writes data to the SPI bus from the provided
            /// buffer.
            async fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
                // Get previous transfer.
                let (mut spi_dma, mut tx_buf, mut rx_buf) = self.wait_for_idle().await;

                for chunk in words.chunks(tx_buf.capacity()) {
                    tx_buf.fill(chunk);

                    match spi_dma.dma_write(tx_buf) {
                        Ok(transfer) => {
                            self.state = State::Writing(transfer, rx_buf);
                        }
                        Err((e, spi, tx)) => {
                            self.state = State::Idle(spi, tx, rx_buf);
                            return Err(e);
                        }
                    };

                    match &mut self.state {
                        State::Writing(transfer, _) => transfer.wait_for_done().await,
                        _ => unreachable!(),
                    };

                    (spi_dma, tx_buf, rx_buf) = match take(&mut self.state) {
                        State::Writing(transfer, rx_buf) => {
                            let (spi, tx_buf) = transfer.wait();
                            (spi, tx_buf, rx_buf)
                        }
                        _ => unreachable!(),
                    };
                }

                self.state = State::Idle(spi_dma, tx_buf, rx_buf);

                Ok(())
            }

            /// Asynchronously performs a full-duplex transfer over the SPI bus.
            ///
            /// This method splits the transfer operation into chunks and
            /// processes it asynchronously. It simultaneously
            /// writes data from the `write` buffer and reads data into the
            /// `read` buffer.
            async fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
                // Get previous transfer.
                let (mut spi_dma, mut tx_buf, mut rx_buf) = self.wait_for_idle().await;

                let chunk_size = min(tx_buf.capacity(), rx_buf.capacity());

                let common_length = min(read.len(), write.len());
                let (read_common, read_remainder) = read.split_at_mut(common_length);
                let (write_common, write_remainder) = write.split_at(common_length);

                for (read_chunk, write_chunk) in read_common
                    .chunks_mut(chunk_size)
                    .zip(write_common.chunks(chunk_size))
                {
                    tx_buf.fill(write_chunk);
                    rx_buf.set_length(read_chunk.len());

                    match spi_dma.dma_transfer(tx_buf, rx_buf) {
                        Ok(transfer) => {
                            self.state = State::Transferring(transfer);
                        }
                        Err((e, spi, tx, rx)) => {
                            self.state = State::Idle(spi, tx, rx);
                            return Err(e);
                        }
                    };

                    match &mut self.state {
                        State::Transferring(transfer) => transfer.wait_for_done().await,
                        _ => unreachable!(),
                    };

                    (spi_dma, tx_buf, rx_buf) = match take(&mut self.state) {
                        State::Transferring(transfer) => {
                            let (spi, (tx_buf, rx_buf)) = transfer.wait();
                            (spi, tx_buf, rx_buf)
                        }
                        _ => unreachable!(),
                    };

                    let bytes_read = rx_buf.read_received_data(read_chunk);
                    assert_eq!(bytes_read, read_chunk.len());
                }

                self.state = State::Idle(spi_dma, tx_buf, rx_buf);

                if !read_remainder.is_empty() {
                    self.read(read_remainder).await
                } else if !write_remainder.is_empty() {
                    self.write(write_remainder).await
                } else {
                    Ok(())
                }
            }

            /// Asynchronously performs an in-place full-duplex transfer over
            /// the SPI bus.
            async fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
                // Get previous transfer.
                let (mut spi_dma, mut tx_buf, mut rx_buf) = self.wait_for_idle().await;

                for chunk in words.chunks_mut(tx_buf.capacity()) {
                    tx_buf.fill(chunk);
                    rx_buf.set_length(chunk.len());

                    match spi_dma.dma_transfer(tx_buf, rx_buf) {
                        Ok(transfer) => {
                            self.state = State::Transferring(transfer);
                        }
                        Err((e, spi, tx, rx)) => {
                            self.state = State::Idle(spi, tx, rx);
                            return Err(e);
                        }
                    };

                    match &mut self.state {
                        State::Transferring(transfer) => transfer.wait_for_done().await,
                        _ => unreachable!(),
                    };

                    (spi_dma, tx_buf, rx_buf) = match take(&mut self.state) {
                        State::Transferring(transfer) => {
                            let (spi, (tx_buf, rx_buf)) = transfer.wait();
                            (spi, tx_buf, rx_buf)
                        }
                        _ => unreachable!(),
                    };

                    let bytes_read = rx_buf.read_received_data(chunk);
                    debug_assert_eq!(bytes_read, chunk.len());
                }

                self.state = State::Idle(spi_dma, tx_buf, rx_buf);

                Ok(())
            }

            async fn flush(&mut self) -> Result<(), Self::Error> {
                // Get previous transfer.
                let (spi_dma, tx_buf, rx_buf) = self.wait_for_idle().await;
                self.state = State::Idle(spi_dma, tx_buf, rx_buf);
                Ok(())
            }
        }
    }

    #[cfg(feature = "embedded-hal")]
    mod ehal1 {
        use embedded_hal::spi::{ErrorType, SpiBus};

        use super::*;

        impl<'d, T, C> ErrorType for SpiDmaBus<'d, T, C>
        where
            T: InstanceDma,
            C: DmaChannel,
            C::P: SpiPeripheral,
        {
            type Error = Error;
        }

        impl<'d, T, C> SpiBus for SpiDmaBus<'d, T, C>
        where
            T: InstanceDma,
            C: DmaChannel,
            C::P: SpiPeripheral,
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

#[cfg(feature = "embedded-hal")]
mod ehal1 {
    use embedded_hal::spi::SpiBus;
    use embedded_hal_nb::spi::FullDuplex;

    use super::*;

    impl<T, M> embedded_hal::spi::ErrorType for Spi<'_, T, M> {
        type Error = super::Error;
    }

    impl<T, M> FullDuplex for Spi<'_, T, M>
    where
        T: Instance,
        M: IsFullDuplex,
    {
        fn read(&mut self) -> nb::Result<u8, Self::Error> {
            self.spi.read_byte()
        }

        fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
            self.spi.write_byte(word)
        }
    }

    impl<T, M> SpiBus for Spi<'_, T, M>
    where
        T: Instance,
        M: IsFullDuplex,
    {
        fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
            self.spi.read_bytes(words)
        }

        fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
            self.spi.write_bytes(words)
        }

        fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
            // Optimizations
            if read.is_empty() {
                SpiBus::write(self, write)?;
            } else if write.is_empty() {
                SpiBus::read(self, read)?;
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

                if write_to < read_to {
                    // Read more than we write, must pad writing part with zeros
                    let mut empty = [EMPTY_WRITE_PAD; FIFO_SIZE];
                    empty[0..write_inc].copy_from_slice(&write[write_from..write_to]);
                    SpiBus::write(self, &empty)?;
                } else {
                    SpiBus::write(self, &write[write_from..write_to])?;
                }

                SpiBus::flush(self)?;

                if read_inc > 0 {
                    self.spi
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
                self.spi.read_bytes_from_fifo(chunk)?;
            }
            Ok(())
        }

        fn flush(&mut self) -> Result<(), Self::Error> {
            self.spi.flush()
        }
    }
}

#[doc(hidden)]
pub trait InstanceDma: Instance {
    #[allow(clippy::too_many_arguments)]
    unsafe fn start_transfer_dma<TX: Tx, RX: Rx>(
        &mut self,
        tx_desc: *mut DmaDescriptor,
        rx_desc: *mut DmaDescriptor,
        write_buffer_len: usize,
        read_buffer_len: usize,
        tx: &mut TX,
        rx: &mut RX,
    ) -> Result<(), Error> {
        let reg_block = self.register_block();
        self.configure_datalen(usize::max(read_buffer_len, write_buffer_len) as u32 * 8);

        tx.is_done();
        rx.is_done();

        // re-enable the MISO and MOSI
        reg_block
            .user()
            .modify(|_, w| w.usr_miso().bit(true).usr_mosi().bit(true));

        self.enable_dma();
        self.update();

        self.clear_dma_interrupts();
        reset_dma_before_load_dma_dscr(reg_block);
        tx.prepare_transfer(self.dma_peripheral(), tx_desc)
            .and_then(|_| tx.start_transfer())?;
        rx.prepare_transfer(self.dma_peripheral(), rx_desc)
            .and_then(|_| rx.start_transfer())?;

        reset_dma_before_usr_cmd(reg_block);

        reg_block.cmd().modify(|_, w| w.usr().set_bit());

        Ok(())
    }

    #[cfg_attr(feature = "place-spi-driver-in-ram", ram)]
    unsafe fn start_write_bytes_dma<TX: Tx>(
        &mut self,
        first_desc: *mut DmaDescriptor,
        len: usize,
        tx: &mut TX,
    ) -> Result<(), Error> {
        let reg_block = self.register_block();
        self.configure_datalen(len as u32 * 8);

        tx.is_done();

        // disable MISO and re-enable MOSI
        reg_block
            .user()
            .modify(|_, w| w.usr_miso().bit(false).usr_mosi().bit(true));

        self.enable_dma();
        self.update();

        reset_dma_before_load_dma_dscr(reg_block);
        self.clear_dma_interrupts();

        tx.prepare_transfer(self.dma_peripheral(), first_desc)?;
        tx.start_transfer()?;
        reset_dma_before_usr_cmd(reg_block);

        // Wait for at least one clock cycle for the DMA to fill the SPI async FIFO,
        // before starting the SPI
        #[cfg(riscv)]
        riscv::asm::delay(1);
        #[cfg(xtensa)]
        xtensa_lx::timer::delay(1);

        reg_block.cmd().modify(|_, w| w.usr().set_bit());

        Ok(())
    }

    #[cfg_attr(feature = "place-spi-driver-in-ram", ram)]
    unsafe fn start_read_bytes_dma<RX: Rx>(
        &mut self,
        desc: *mut DmaDescriptor,
        data_length: usize,
        rx: &mut RX,
    ) -> Result<(), Error> {
        let reg_block = self.register_block();
        self.configure_datalen(data_length as u32 * 8);

        rx.is_done();

        // re-enable MISO and disable MOSI
        reg_block
            .user()
            .modify(|_, w| w.usr_miso().bit(true).usr_mosi().bit(false));

        self.enable_dma();
        self.update();

        reset_dma_before_load_dma_dscr(reg_block);

        self.clear_dma_interrupts();
        reset_dma_before_usr_cmd(reg_block);

        rx.prepare_transfer(self.dma_peripheral(), desc)?;
        rx.start_transfer()?;

        reg_block.cmd().modify(|_, w| w.usr().set_bit());

        Ok(())
    }

    fn dma_peripheral(&self) -> DmaPeripheral {
        match self.spi_num() {
            2 => DmaPeripheral::Spi2,
            #[cfg(spi3)]
            3 => DmaPeripheral::Spi3,
            _ => panic!("Illegal SPI instance"),
        }
    }

    #[cfg(any(esp32c2, esp32c3, esp32c6, esp32h2, esp32s3))]
    fn enable_dma(&self) {
        let reg_block = self.register_block();
        reg_block.dma_conf().modify(|_, w| w.dma_tx_ena().set_bit());
        reg_block.dma_conf().modify(|_, w| w.dma_rx_ena().set_bit());
    }

    #[cfg(any(esp32, esp32s2))]
    fn enable_dma(&self) {
        // for non GDMA this is done in `assign_tx_device` / `assign_rx_device`
    }

    #[cfg(any(esp32c2, esp32c3, esp32c6, esp32h2, esp32s3))]
    fn clear_dma_interrupts(&self) {
        let reg_block = self.register_block();
        reg_block.dma_int_clr().write(|w| {
            w.dma_infifo_full_err()
                .clear_bit_by_one()
                .dma_outfifo_empty_err()
                .clear_bit_by_one()
                .trans_done()
                .clear_bit_by_one()
                .mst_rx_afifo_wfull_err()
                .clear_bit_by_one()
                .mst_tx_afifo_rempty_err()
                .clear_bit_by_one()
        });
    }

    #[cfg(any(esp32, esp32s2))]
    fn clear_dma_interrupts(&self) {
        let reg_block = self.register_block();
        reg_block.dma_int_clr().write(|w| {
            w.inlink_dscr_empty()
                .clear_bit_by_one()
                .outlink_dscr_error()
                .clear_bit_by_one()
                .inlink_dscr_error()
                .clear_bit_by_one()
                .in_done()
                .clear_bit_by_one()
                .in_err_eof()
                .clear_bit_by_one()
                .in_suc_eof()
                .clear_bit_by_one()
                .out_done()
                .clear_bit_by_one()
                .out_eof()
                .clear_bit_by_one()
                .out_total_eof()
                .clear_bit_by_one()
        });
    }
}

#[cfg(not(any(esp32, esp32s2)))]
fn reset_dma_before_usr_cmd(reg_block: &RegisterBlock) {
    reg_block.dma_conf().modify(|_, w| {
        w.rx_afifo_rst()
            .set_bit()
            .buf_afifo_rst()
            .set_bit()
            .dma_afifo_rst()
            .set_bit()
    });
}

#[cfg(any(esp32, esp32s2))]
fn reset_dma_before_usr_cmd(_reg_block: &RegisterBlock) {}

#[cfg(not(any(esp32, esp32s2)))]
fn reset_dma_before_load_dma_dscr(_reg_block: &RegisterBlock) {}

#[cfg(any(esp32, esp32s2))]
fn reset_dma_before_load_dma_dscr(reg_block: &RegisterBlock) {
    reg_block.dma_conf().modify(|_, w| {
        w.out_rst()
            .set_bit()
            .in_rst()
            .set_bit()
            .ahbm_fifo_rst()
            .set_bit()
            .ahbm_rst()
            .set_bit()
    });

    reg_block.dma_conf().modify(|_, w| {
        w.out_rst()
            .clear_bit()
            .in_rst()
            .clear_bit()
            .ahbm_fifo_rst()
            .clear_bit()
            .ahbm_rst()
            .clear_bit()
    });
}

impl InstanceDma for crate::peripherals::SPI2 {}

#[cfg(spi3)]
impl InstanceDma for crate::peripherals::SPI3 {}

#[doc(hidden)]
pub trait ExtendedInstance: Instance {
    fn sio0_input_signal(&self) -> InputSignal;

    fn sio1_output_signal(&self) -> OutputSignal;

    fn sio2_output_signal(&self) -> OutputSignal;

    fn sio2_input_signal(&self) -> InputSignal;

    fn sio3_output_signal(&self) -> OutputSignal;

    fn sio3_input_signal(&self) -> InputSignal;
}

#[doc(hidden)]
pub trait Instance: private::Sealed {
    fn register_block(&self) -> &RegisterBlock;

    fn sclk_signal(&self) -> OutputSignal;

    fn mosi_signal(&self) -> OutputSignal;

    fn miso_signal(&self) -> InputSignal;

    fn cs_signal(&self) -> OutputSignal;

    fn enable_peripheral(&self);

    fn reset_peripheral(&self);

    fn spi_num(&self) -> u8;

    /// Initialize for full-duplex 1 bit mode
    fn init(&mut self) {
        let reg_block = self.register_block();
        reg_block.user().modify(|_, w| {
            w.usr_miso_highpart()
                .clear_bit()
                .usr_miso_highpart()
                .clear_bit()
                .doutdin()
                .set_bit()
                .usr_miso()
                .set_bit()
                .usr_mosi()
                .set_bit()
                .cs_hold()
                .set_bit()
                .usr_dummy_idle()
                .set_bit()
                .usr_addr()
                .clear_bit()
                .usr_command()
                .clear_bit()
        });

        #[cfg(not(any(esp32, esp32s2)))]
        reg_block.clk_gate().modify(|_, w| {
            w.clk_en()
                .set_bit()
                .mst_clk_active()
                .set_bit()
                .mst_clk_sel()
                .set_bit()
        });

        #[cfg(any(esp32c6, esp32h2))]
        unsafe {
            // use default clock source PLL_F80M_CLK (ESP32-C6) and
            // PLL_F48M_CLK (ESP32-H2)
            (*crate::peripherals::PCR::PTR)
                .spi2_clkm_conf()
                .modify(|_, w| w.spi2_clkm_sel().bits(1));
        }

        #[cfg(not(any(esp32, esp32s2)))]
        reg_block.ctrl().modify(|_, w| {
            w.q_pol()
                .clear_bit()
                .d_pol()
                .clear_bit()
                .hold_pol()
                .clear_bit()
        });

        #[cfg(esp32s2)]
        reg_block
            .ctrl()
            .modify(|_, w| w.q_pol().clear_bit().d_pol().clear_bit().wp().clear_bit());

        #[cfg(esp32)]
        reg_block.ctrl().modify(|_, w| w.wp().clear_bit());

        #[cfg(not(esp32))]
        reg_block.misc().write(|w| unsafe { w.bits(0) });

        reg_block.slave().write(|w| unsafe { w.bits(0) });
    }

    #[cfg(not(esp32))]
    fn init_spi_data_mode(
        &mut self,
        cmd_mode: SpiDataMode,
        address_mode: SpiDataMode,
        data_mode: SpiDataMode,
    ) {
        let reg_block = self.register_block();
        match cmd_mode {
            SpiDataMode::Single => reg_block
                .ctrl()
                .modify(|_, w| w.fcmd_dual().clear_bit().fcmd_quad().clear_bit()),
            SpiDataMode::Dual => reg_block
                .ctrl()
                .modify(|_, w| w.fcmd_dual().set_bit().fcmd_quad().clear_bit()),
            SpiDataMode::Quad => reg_block
                .ctrl()
                .modify(|_, w| w.fcmd_dual().clear_bit().fcmd_quad().set_bit()),
        }

        match address_mode {
            SpiDataMode::Single => reg_block
                .ctrl()
                .modify(|_, w| w.faddr_dual().clear_bit().faddr_quad().clear_bit()),
            SpiDataMode::Dual => reg_block
                .ctrl()
                .modify(|_, w| w.faddr_dual().set_bit().faddr_quad().clear_bit()),
            SpiDataMode::Quad => reg_block
                .ctrl()
                .modify(|_, w| w.faddr_dual().clear_bit().faddr_quad().set_bit()),
        }

        match data_mode {
            SpiDataMode::Single => {
                reg_block
                    .ctrl()
                    .modify(|_, w| w.fread_dual().clear_bit().fread_quad().clear_bit());
                reg_block
                    .user()
                    .modify(|_, w| w.fwrite_dual().clear_bit().fwrite_quad().clear_bit());
            }
            SpiDataMode::Dual => {
                reg_block
                    .ctrl()
                    .modify(|_, w| w.fread_dual().set_bit().fread_quad().clear_bit());
                reg_block
                    .user()
                    .modify(|_, w| w.fwrite_dual().set_bit().fwrite_quad().clear_bit());
            }
            SpiDataMode::Quad => {
                reg_block
                    .ctrl()
                    .modify(|_, w| w.fread_quad().set_bit().fread_dual().clear_bit());
                reg_block
                    .user()
                    .modify(|_, w| w.fwrite_quad().set_bit().fwrite_dual().clear_bit());
            }
        }
    }

    #[cfg(esp32)]
    fn init_spi_data_mode(
        &mut self,
        cmd_mode: SpiDataMode,
        address_mode: SpiDataMode,
        data_mode: SpiDataMode,
    ) {
        let reg_block = self.register_block();
        match cmd_mode {
            SpiDataMode::Single => (),
            _ => panic!("Only 1-bit command supported"),
        }

        match (address_mode, data_mode) {
            (SpiDataMode::Single, SpiDataMode::Single) => {
                reg_block.ctrl().modify(|_, w| {
                    w.fread_dio()
                        .clear_bit()
                        .fread_qio()
                        .clear_bit()
                        .fread_dual()
                        .clear_bit()
                        .fread_quad()
                        .clear_bit()
                });

                reg_block.user().modify(|_, w| {
                    w.fwrite_dio()
                        .clear_bit()
                        .fwrite_qio()
                        .clear_bit()
                        .fwrite_dual()
                        .clear_bit()
                        .fwrite_quad()
                        .clear_bit()
                });
            }
            (SpiDataMode::Single, SpiDataMode::Dual) => {
                reg_block.ctrl().modify(|_, w| {
                    w.fread_dio()
                        .clear_bit()
                        .fread_qio()
                        .clear_bit()
                        .fread_dual()
                        .set_bit()
                        .fread_quad()
                        .clear_bit()
                });

                reg_block.user().modify(|_, w| {
                    w.fwrite_dio()
                        .clear_bit()
                        .fwrite_qio()
                        .clear_bit()
                        .fwrite_dual()
                        .set_bit()
                        .fwrite_quad()
                        .clear_bit()
                });
            }
            (SpiDataMode::Single, SpiDataMode::Quad) => {
                reg_block.ctrl().modify(|_, w| {
                    w.fread_dio()
                        .clear_bit()
                        .fread_qio()
                        .clear_bit()
                        .fread_dual()
                        .clear_bit()
                        .fread_quad()
                        .set_bit()
                });

                reg_block.user().modify(|_, w| {
                    w.fwrite_dio()
                        .clear_bit()
                        .fwrite_qio()
                        .clear_bit()
                        .fwrite_dual()
                        .clear_bit()
                        .fwrite_quad()
                        .set_bit()
                });
            }
            (SpiDataMode::Dual, SpiDataMode::Single) => {
                panic!("Unsupported combination of data-modes")
            }
            (SpiDataMode::Dual, SpiDataMode::Dual) => {
                reg_block.ctrl().modify(|_, w| {
                    w.fread_dio()
                        .set_bit()
                        .fread_qio()
                        .clear_bit()
                        .fread_dual()
                        .clear_bit()
                        .fread_quad()
                        .clear_bit()
                });

                reg_block.user().modify(|_, w| {
                    w.fwrite_dio()
                        .set_bit()
                        .fwrite_qio()
                        .clear_bit()
                        .fwrite_dual()
                        .clear_bit()
                        .fwrite_quad()
                        .clear_bit()
                });
            }
            (SpiDataMode::Dual, SpiDataMode::Quad) => {
                panic!("Unsupported combination of data-modes")
            }
            (SpiDataMode::Quad, SpiDataMode::Single) => {
                panic!("Unsupported combination of data-modes")
            }
            (SpiDataMode::Quad, SpiDataMode::Dual) => {
                panic!("Unsupported combination of data-modes")
            }
            (SpiDataMode::Quad, SpiDataMode::Quad) => {
                reg_block.ctrl().modify(|_, w| {
                    w.fread_dio()
                        .clear_bit()
                        .fread_qio()
                        .set_bit()
                        .fread_dual()
                        .clear_bit()
                        .fread_quad()
                        .clear_bit()
                });

                reg_block.user().modify(|_, w| {
                    w.fwrite_dio()
                        .clear_bit()
                        .fwrite_qio()
                        .set_bit()
                        .fwrite_dual()
                        .clear_bit()
                        .fwrite_quad()
                        .clear_bit()
                });
            }
        }
    }

    // taken from https://github.com/apache/incubator-nuttx/blob/8267a7618629838231256edfa666e44b5313348e/arch/risc-v/src/esp32c3/esp32c3_spi.c#L496
    fn setup(&mut self, frequency: HertzU32, clocks: &Clocks<'_>) {
        #[cfg(not(esp32h2))]
        let apb_clk_freq: HertzU32 = HertzU32::Hz(clocks.apb_clock.to_Hz());
        // ESP32-H2 is using PLL_48M_CLK source instead of APB_CLK
        #[cfg(esp32h2)]
        let apb_clk_freq: HertzU32 = HertzU32::Hz(clocks.pll_48m_clock.to_Hz());

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

    /// Set the interrupt handler
    fn set_interrupt_handler(&mut self, handler: InterruptHandler);

    /// Listen for the given interrupts
    #[cfg(not(any(esp32, esp32s2)))]
    fn listen(&mut self, interrupts: EnumSet<SpiInterrupt>) {
        let reg_block = self.register_block();

        for interrupt in interrupts {
            match interrupt {
                SpiInterrupt::TransDone => {
                    reg_block
                        .dma_int_ena()
                        .modify(|_, w| w.trans_done().set_bit());
                }
            }
        }
    }

    /// Unlisten the given interrupts
    #[cfg(not(any(esp32, esp32s2)))]
    fn unlisten(&mut self, interrupts: EnumSet<SpiInterrupt>) {
        let reg_block = self.register_block();

        for interrupt in interrupts {
            match interrupt {
                SpiInterrupt::TransDone => {
                    reg_block
                        .dma_int_ena()
                        .modify(|_, w| w.trans_done().clear_bit());
                }
            }
        }
    }

    /// Gets asserted interrupts
    #[cfg(not(any(esp32, esp32s2)))]
    fn interrupts(&mut self) -> EnumSet<SpiInterrupt> {
        let mut res = EnumSet::new();
        let reg_block = self.register_block();

        let ints = reg_block.dma_int_st().read();

        if ints.trans_done().bit() {
            res.insert(SpiInterrupt::TransDone);
        }

        res
    }

    /// Resets asserted interrupts
    #[cfg(not(any(esp32, esp32s2)))]
    fn clear_interrupts(&mut self, interrupts: EnumSet<SpiInterrupt>) {
        let reg_block = self.register_block();

        for interrupt in interrupts {
            match interrupt {
                SpiInterrupt::TransDone => {
                    reg_block
                        .dma_int_clr()
                        .write(|w| w.trans_done().clear_bit_by_one());
                }
            }
        }
    }

    #[cfg(not(esp32))]
    fn set_data_mode(&mut self, data_mode: SpiMode) -> &mut Self {
        let reg_block = self.register_block();

        match data_mode {
            SpiMode::Mode0 => {
                reg_block.misc().modify(|_, w| w.ck_idle_edge().clear_bit());
                reg_block.user().modify(|_, w| w.ck_out_edge().clear_bit());
            }
            SpiMode::Mode1 => {
                reg_block.misc().modify(|_, w| w.ck_idle_edge().clear_bit());
                reg_block.user().modify(|_, w| w.ck_out_edge().set_bit());
            }
            SpiMode::Mode2 => {
                reg_block.misc().modify(|_, w| w.ck_idle_edge().set_bit());
                reg_block.user().modify(|_, w| w.ck_out_edge().set_bit());
            }
            SpiMode::Mode3 => {
                reg_block.misc().modify(|_, w| w.ck_idle_edge().set_bit());
                reg_block.user().modify(|_, w| w.ck_out_edge().clear_bit());
            }
        }
        self
    }

    #[cfg(esp32)]
    fn set_data_mode(&mut self, data_mode: SpiMode) -> &mut Self {
        let reg_block = self.register_block();

        match data_mode {
            SpiMode::Mode0 => {
                reg_block.pin().modify(|_, w| w.ck_idle_edge().clear_bit());
                reg_block.user().modify(|_, w| w.ck_out_edge().clear_bit());
            }
            SpiMode::Mode1 => {
                reg_block.pin().modify(|_, w| w.ck_idle_edge().clear_bit());
                reg_block.user().modify(|_, w| w.ck_out_edge().set_bit());
            }
            SpiMode::Mode2 => {
                reg_block.pin().modify(|_, w| w.ck_idle_edge().set_bit());
                reg_block.user().modify(|_, w| w.ck_out_edge().set_bit());
            }
            SpiMode::Mode3 => {
                reg_block.pin().modify(|_, w| w.ck_idle_edge().set_bit());
                reg_block.user().modify(|_, w| w.ck_out_edge().clear_bit());
            }
        }
        self
    }

    fn ch_bus_freq(&mut self, frequency: HertzU32, clocks: &Clocks<'_>) {
        // Disable clock source
        #[cfg(not(any(esp32, esp32s2)))]
        self.register_block().clk_gate().modify(|_, w| {
            w.clk_en()
                .clear_bit()
                .mst_clk_active()
                .clear_bit()
                .mst_clk_sel()
                .clear_bit()
        });

        // Change clock frequency
        self.setup(frequency, clocks);

        // Enable clock source
        #[cfg(not(any(esp32, esp32s2)))]
        self.register_block().clk_gate().modify(|_, w| {
            w.clk_en()
                .set_bit()
                .mst_clk_active()
                .set_bit()
                .mst_clk_sel()
                .set_bit()
        });
    }

    #[cfg(not(any(esp32, esp32c3, esp32s2)))]
    fn set_bit_order(&mut self, read_order: SpiBitOrder, write_order: SpiBitOrder) {
        let reg_block = self.register_block();

        let read_value = match read_order {
            SpiBitOrder::MSBFirst => 0,
            SpiBitOrder::LSBFirst => 1,
        };
        let write_value = match write_order {
            SpiBitOrder::MSBFirst => 0,
            SpiBitOrder::LSBFirst => 1,
        };
        reg_block.ctrl().modify(|_, w| unsafe {
            w.rd_bit_order().bits(read_value);
            w.wr_bit_order().bits(write_value);
            w
        });
    }
    #[cfg(any(esp32, esp32c3, esp32s2))]
    fn set_bit_order(&mut self, read_order: SpiBitOrder, write_order: SpiBitOrder) {
        let reg_block = self.register_block();

        let read_value = match read_order {
            SpiBitOrder::MSBFirst => false,
            SpiBitOrder::LSBFirst => true,
        };
        let write_value = match write_order {
            SpiBitOrder::MSBFirst => false,
            SpiBitOrder::LSBFirst => true,
        };
        reg_block.ctrl().modify(|_, w| {
            w.rd_bit_order().bit(read_value);
            w.wr_bit_order().bit(write_value);
            w
        });
    }

    fn read_byte(&mut self) -> nb::Result<u8, Error> {
        if self.busy() {
            return Err(nb::Error::WouldBlock);
        }

        let reg_block = self.register_block();
        Ok(u32::try_into(reg_block.w(0).read().bits()).unwrap_or_default())
    }

    fn write_byte(&mut self, word: u8) -> nb::Result<(), Error> {
        if self.busy() {
            return Err(nb::Error::WouldBlock);
        }

        self.configure_datalen(8);

        let reg_block = self.register_block();
        reg_block.w(0).write(|w| w.buf().set(word.into()));

        self.update();

        reg_block.cmd().modify(|_, w| w.usr().set_bit());

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
    #[cfg_attr(feature = "place-spi-driver-in-ram", ram)]
    fn write_bytes(&mut self, words: &[u8]) -> Result<(), Error> {
        let num_chunks = words.len() / FIFO_SIZE;

        // Flush in case previous writes have not completed yet, required as per
        // embedded-hal documentation (#1369).
        self.flush()?;

        // The fifo has a limited fixed size, so the data must be chunked and then
        // transmitted
        for (i, chunk) in words.chunks(FIFO_SIZE).enumerate() {
            self.configure_datalen(chunk.len() as u32 * 8);

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

            self.update();

            self.register_block().cmd().modify(|_, w| w.usr().set_bit());

            // Wait for all chunks to complete except the last one.
            // The function is allowed to return before the bus is idle.
            // see [embedded-hal flushing](https://docs.rs/embedded-hal/1.0.0-alpha.8/embedded_hal/spi/blocking/index.html#flushing)
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
    #[cfg_attr(feature = "place-spi-driver-in-ram", ram)]
    fn read_bytes(&mut self, words: &mut [u8]) -> Result<(), Error> {
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
    #[cfg_attr(feature = "place-spi-driver-in-ram", ram)]
    fn read_bytes_from_fifo(&mut self, words: &mut [u8]) -> Result<(), Error> {
        let reg_block = self.register_block();

        for chunk in words.chunks_mut(FIFO_SIZE) {
            self.configure_datalen(chunk.len() as u32 * 8);

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
    fn flush(&mut self) -> Result<(), Error> {
        while self.busy() {
            // wait for bus to be clear
        }
        Ok(())
    }

    #[cfg_attr(feature = "place-spi-driver-in-ram", ram)]
    fn transfer<'w>(&mut self, words: &'w mut [u8]) -> Result<&'w [u8], Error> {
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

    fn init_half_duplex(
        &mut self,
        is_write: bool,
        command_state: bool,
        address_state: bool,
        dummy_idle: bool,
        dummy_state: bool,
        no_mosi_miso: bool,
    ) {
        let reg_block = self.register_block();
        reg_block.user().modify(|_, w| {
            w.usr_miso_highpart()
                .clear_bit()
                .usr_miso_highpart()
                .clear_bit()
                .doutdin()
                .clear_bit()
                .usr_miso()
                .bit(!is_write && !no_mosi_miso)
                .usr_mosi()
                .bit(is_write && !no_mosi_miso)
                .cs_hold()
                .set_bit()
                .usr_dummy_idle()
                .bit(dummy_idle)
                .usr_dummy()
                .bit(dummy_state)
                .usr_addr()
                .bit(address_state)
                .usr_command()
                .bit(command_state)
        });

        #[cfg(not(any(esp32, esp32s2)))]
        reg_block.clk_gate().modify(|_, w| {
            w.clk_en()
                .set_bit()
                .mst_clk_active()
                .set_bit()
                .mst_clk_sel()
                .set_bit()
        });

        #[cfg(any(esp32c6, esp32h2))]
        unsafe {
            let pcr = &*crate::peripherals::PCR::PTR;

            // use default clock source PLL_F80M_CLK
            pcr.spi2_clkm_conf()
                .modify(|_, w| w.spi2_clkm_sel().bits(1));
        }

        #[cfg(not(esp32))]
        reg_block.misc().write(|w| unsafe { w.bits(0) });

        reg_block.slave().write(|w| unsafe { w.bits(0) });

        self.update();
    }

    fn write_bytes_half_duplex(
        &mut self,
        cmd: Command,
        address: Address,
        dummy: u8,
        buffer: &[u8],
    ) -> Result<(), Error> {
        self.init_half_duplex(
            true,
            !cmd.is_none(),
            !address.is_none(),
            false,
            dummy != 0,
            buffer.is_empty(),
        );

        // set cmd, address, dummy cycles
        let reg_block = self.register_block();
        if !cmd.is_none() {
            reg_block.user2().modify(|_, w| unsafe {
                w.usr_command_bitlen()
                    .bits((cmd.width() - 1) as u8)
                    .usr_command_value()
                    .bits(cmd.value())
            });
        }

        #[cfg(not(esp32))]
        if !address.is_none() {
            reg_block
                .user1()
                .modify(|_, w| unsafe { w.usr_addr_bitlen().bits((address.width() - 1) as u8) });

            let addr = address.value() << (32 - address.width());
            reg_block
                .addr()
                .write(|w| unsafe { w.usr_addr_value().bits(addr) });
        }

        #[cfg(esp32)]
        if !address.is_none() {
            reg_block.user1().modify(|r, w| unsafe {
                w.bits(r.bits() & !(0x3f << 26) | (((address.width() - 1) as u32) & 0x3f) << 26)
            });

            let addr = address.value() << (32 - address.width());
            reg_block.addr().write(|w| unsafe { w.bits(addr) });
        }

        if dummy > 0 {
            reg_block
                .user1()
                .modify(|_, w| unsafe { w.usr_dummy_cyclelen().bits(dummy - 1) });
        }

        if !buffer.is_empty() {
            // re-using the full-duplex write here
            self.write_bytes(buffer)?;
        } else {
            self.start_operation();
        }

        self.flush()
    }

    fn read_bytes_half_duplex(
        &mut self,
        cmd: Command,
        address: Address,
        dummy: u8,
        buffer: &mut [u8],
    ) -> Result<(), Error> {
        self.init_half_duplex(
            false,
            !cmd.is_none(),
            !address.is_none(),
            false,
            dummy != 0,
            buffer.is_empty(),
        );

        // set cmd, address, dummy cycles
        let reg_block = self.register_block();
        if !cmd.is_none() {
            reg_block.user2().modify(|_, w| unsafe {
                w.usr_command_bitlen()
                    .bits((cmd.width() - 1) as u8)
                    .usr_command_value()
                    .bits(cmd.value())
            });
        }

        #[cfg(not(esp32))]
        if !address.is_none() {
            reg_block
                .user1()
                .modify(|_, w| unsafe { w.usr_addr_bitlen().bits((address.width() - 1) as u8) });

            let addr = address.value() << (32 - address.width());
            reg_block
                .addr()
                .write(|w| unsafe { w.usr_addr_value().bits(addr) });
        }

        #[cfg(esp32)]
        if !address.is_none() {
            reg_block.user1().modify(|r, w| unsafe {
                w.bits(r.bits() & !(0x3f << 26) | (((address.width() - 1) as u32) & 0x3f) << 26)
            });

            let addr = address.value() << (32 - address.width());
            reg_block.addr().write(|w| unsafe { w.bits(addr) });
        }

        if dummy > 0 {
            reg_block
                .user1()
                .modify(|_, w| unsafe { w.usr_dummy_cyclelen().bits(dummy - 1) });
        }

        self.configure_datalen(buffer.len() as u32 * 8);
        self.update();
        reg_block.cmd().modify(|_, w| w.usr().set_bit());
        self.flush()?;
        self.read_bytes_from_fifo(buffer)
    }

    #[cfg(not(any(esp32, esp32s2)))]
    fn update(&self) {
        let reg_block = self.register_block();

        reg_block.cmd().modify(|_, w| w.update().set_bit());

        while reg_block.cmd().read().update().bit_is_set() {
            // wait
        }
    }

    #[cfg(any(esp32, esp32s2))]
    fn update(&self) {
        // not need/available on ESP32/ESP32S2
    }

    fn configure_datalen(&self, len: u32) {
        let reg_block = self.register_block();
        let len = if len > 0 { len - 1 } else { 0 };

        #[cfg(any(esp32c2, esp32c3, esp32c6, esp32h2, esp32s3))]
        reg_block
            .ms_dlen()
            .write(|w| unsafe { w.ms_data_bitlen().bits(len) });

        #[cfg(not(any(esp32c2, esp32c3, esp32c6, esp32h2, esp32s3)))]
        {
            reg_block
                .mosi_dlen()
                .write(|w| unsafe { w.usr_mosi_dbitlen().bits(len) });

            reg_block
                .miso_dlen()
                .write(|w| unsafe { w.usr_miso_dbitlen().bits(len) });
        }
    }
}

#[cfg(any(esp32c2, esp32c3, esp32c6, esp32h2))]
impl Instance for crate::peripherals::SPI2 {
    #[inline(always)]
    fn register_block(&self) -> &RegisterBlock {
        self
    }

    #[inline(always)]
    fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        self.bind_spi2_interrupt(handler.handler());
        crate::interrupt::enable(crate::peripherals::Interrupt::SPI2, handler.priority()).unwrap();
    }

    #[inline(always)]
    fn sclk_signal(&self) -> OutputSignal {
        OutputSignal::FSPICLK_MUX
    }

    #[inline(always)]
    fn mosi_signal(&self) -> OutputSignal {
        OutputSignal::FSPID
    }

    #[inline(always)]
    fn miso_signal(&self) -> InputSignal {
        InputSignal::FSPIQ
    }

    #[inline(always)]
    fn cs_signal(&self) -> OutputSignal {
        OutputSignal::FSPICS0
    }

    #[inline(always)]
    fn enable_peripheral(&self) {
        PeripheralClockControl::enable(crate::system::Peripheral::Spi2);
    }

    #[inline(always)]
    fn reset_peripheral(&self) {
        PeripheralClockControl::reset(crate::system::Peripheral::Spi2);
    }

    #[inline(always)]
    fn spi_num(&self) -> u8 {
        2
    }
}

#[cfg(any(esp32c2, esp32c3, esp32c6, esp32h2))]
impl ExtendedInstance for crate::peripherals::SPI2 {
    #[inline(always)]
    fn sio0_input_signal(&self) -> InputSignal {
        InputSignal::FSPID
    }

    #[inline(always)]
    fn sio1_output_signal(&self) -> OutputSignal {
        OutputSignal::FSPIQ
    }

    #[inline(always)]
    fn sio2_output_signal(&self) -> OutputSignal {
        OutputSignal::FSPIWP
    }

    #[inline(always)]
    fn sio2_input_signal(&self) -> InputSignal {
        InputSignal::FSPIWP
    }

    #[inline(always)]
    fn sio3_output_signal(&self) -> OutputSignal {
        OutputSignal::FSPIHD
    }

    #[inline(always)]
    fn sio3_input_signal(&self) -> InputSignal {
        InputSignal::FSPIHD
    }
}

#[cfg(esp32)]
impl Instance for crate::peripherals::SPI2 {
    #[inline(always)]
    fn register_block(&self) -> &RegisterBlock {
        self
    }

    #[inline(always)]
    fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        self.bind_spi2_interrupt(handler.handler());
        crate::interrupt::enable(crate::peripherals::Interrupt::SPI2, handler.priority()).unwrap();
    }

    #[inline(always)]
    fn sclk_signal(&self) -> OutputSignal {
        OutputSignal::HSPICLK
    }

    #[inline(always)]
    fn mosi_signal(&self) -> OutputSignal {
        OutputSignal::HSPID
    }

    #[inline(always)]
    fn miso_signal(&self) -> InputSignal {
        InputSignal::HSPIQ
    }

    #[inline(always)]
    fn cs_signal(&self) -> OutputSignal {
        OutputSignal::HSPICS0
    }

    #[inline(always)]
    fn enable_peripheral(&self) {
        PeripheralClockControl::enable(crate::system::Peripheral::Spi2);
    }

    #[inline(always)]
    fn reset_peripheral(&self) {
        PeripheralClockControl::reset(crate::system::Peripheral::Spi2);
    }

    #[inline(always)]
    fn spi_num(&self) -> u8 {
        2
    }
}

#[cfg(esp32)]
impl ExtendedInstance for crate::peripherals::SPI2 {
    fn sio0_input_signal(&self) -> InputSignal {
        InputSignal::HSPID
    }

    fn sio1_output_signal(&self) -> OutputSignal {
        OutputSignal::HSPIQ
    }

    fn sio2_output_signal(&self) -> OutputSignal {
        OutputSignal::HSPIWP
    }

    fn sio2_input_signal(&self) -> InputSignal {
        InputSignal::HSPIWP
    }

    fn sio3_output_signal(&self) -> OutputSignal {
        OutputSignal::HSPIHD
    }

    fn sio3_input_signal(&self) -> InputSignal {
        InputSignal::HSPIHD
    }
}

#[cfg(esp32)]
impl Instance for crate::peripherals::SPI3 {
    #[inline(always)]
    fn register_block(&self) -> &RegisterBlock {
        self
    }

    #[inline(always)]
    fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        self.bind_spi3_interrupt(handler.handler());
        crate::interrupt::enable(crate::peripherals::Interrupt::SPI2, handler.priority()).unwrap();
    }

    #[inline(always)]
    fn sclk_signal(&self) -> OutputSignal {
        OutputSignal::VSPICLK
    }

    #[inline(always)]
    fn mosi_signal(&self) -> OutputSignal {
        OutputSignal::VSPID
    }

    #[inline(always)]
    fn miso_signal(&self) -> InputSignal {
        InputSignal::VSPIQ
    }

    #[inline(always)]
    fn cs_signal(&self) -> OutputSignal {
        OutputSignal::VSPICS0
    }

    #[inline(always)]
    fn enable_peripheral(&self) {
        PeripheralClockControl::enable(crate::system::Peripheral::Spi3)
    }

    #[inline(always)]
    fn reset_peripheral(&self) {
        PeripheralClockControl::reset(crate::system::Peripheral::Spi3)
    }

    #[inline(always)]
    fn spi_num(&self) -> u8 {
        3
    }
}

#[cfg(any(esp32s2, esp32s3))]
impl Instance for crate::peripherals::SPI2 {
    #[inline(always)]
    fn register_block(&self) -> &RegisterBlock {
        self
    }

    #[inline(always)]
    fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        self.bind_spi2_interrupt(handler.handler());
        crate::interrupt::enable(crate::peripherals::Interrupt::SPI2, handler.priority()).unwrap();
    }

    #[inline(always)]
    fn sclk_signal(&self) -> OutputSignal {
        OutputSignal::FSPICLK
    }

    #[inline(always)]
    fn mosi_signal(&self) -> OutputSignal {
        OutputSignal::FSPID
    }

    #[inline(always)]
    fn miso_signal(&self) -> InputSignal {
        InputSignal::FSPIQ
    }

    #[inline(always)]
    fn cs_signal(&self) -> OutputSignal {
        OutputSignal::FSPICS0
    }

    #[inline(always)]
    fn enable_peripheral(&self) {
        PeripheralClockControl::enable(crate::system::Peripheral::Spi2)
    }

    #[inline(always)]
    fn reset_peripheral(&self) {
        PeripheralClockControl::reset(crate::system::Peripheral::Spi2)
    }

    #[inline(always)]
    fn spi_num(&self) -> u8 {
        2
    }
}

#[cfg(any(esp32s2, esp32s3))]
impl ExtendedInstance for crate::peripherals::SPI2 {
    #[inline(always)]
    fn sio0_input_signal(&self) -> InputSignal {
        InputSignal::FSPID
    }

    #[inline(always)]
    fn sio1_output_signal(&self) -> OutputSignal {
        OutputSignal::FSPIQ
    }

    #[inline(always)]
    fn sio2_output_signal(&self) -> OutputSignal {
        OutputSignal::FSPIWP
    }

    #[inline(always)]
    fn sio2_input_signal(&self) -> InputSignal {
        InputSignal::FSPIWP
    }

    #[inline(always)]
    fn sio3_output_signal(&self) -> OutputSignal {
        OutputSignal::FSPIHD
    }

    #[inline(always)]
    fn sio3_input_signal(&self) -> InputSignal {
        InputSignal::FSPIHD
    }
}

#[cfg(any(esp32s2, esp32s3))]
impl Instance for crate::peripherals::SPI3 {
    #[inline(always)]
    fn register_block(&self) -> &RegisterBlock {
        self
    }

    #[inline(always)]
    fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        self.bind_spi3_interrupt(handler.handler());
        crate::interrupt::enable(crate::peripherals::Interrupt::SPI2, handler.priority()).unwrap();
    }

    #[inline(always)]
    fn sclk_signal(&self) -> OutputSignal {
        OutputSignal::SPI3_CLK
    }

    #[inline(always)]
    fn mosi_signal(&self) -> OutputSignal {
        OutputSignal::SPI3_D
    }

    #[inline(always)]
    fn miso_signal(&self) -> InputSignal {
        InputSignal::SPI3_Q
    }

    #[inline(always)]
    fn cs_signal(&self) -> OutputSignal {
        OutputSignal::SPI3_CS0
    }

    #[inline(always)]
    fn enable_peripheral(&self) {
        PeripheralClockControl::enable(crate::system::Peripheral::Spi3)
    }

    #[inline(always)]
    fn reset_peripheral(&self) {
        PeripheralClockControl::reset(crate::system::Peripheral::Spi3)
    }

    #[inline(always)]
    fn spi_num(&self) -> u8 {
        3
    }
}

#[cfg(esp32s3)]
impl ExtendedInstance for crate::peripherals::SPI3 {
    #[inline(always)]
    fn sio0_input_signal(&self) -> InputSignal {
        InputSignal::SPI3_D
    }

    #[inline(always)]
    fn sio1_output_signal(&self) -> OutputSignal {
        OutputSignal::SPI3_Q
    }

    #[inline(always)]
    fn sio2_output_signal(&self) -> OutputSignal {
        OutputSignal::SPI3_WP
    }

    #[inline(always)]
    fn sio2_input_signal(&self) -> InputSignal {
        InputSignal::SPI3_WP
    }

    #[inline(always)]
    fn sio3_output_signal(&self) -> OutputSignal {
        OutputSignal::SPI3_HD
    }

    #[inline(always)]
    fn sio3_input_signal(&self) -> InputSignal {
        InputSignal::SPI3_HD
    }
}
