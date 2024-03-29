//! # I2S Master
//!
//! ## Overview
//!
//! The I2S Master peripheral driver provides support for the I2S (Inter-IC
//! Sound) Master functionality on ESP chips. It enables audio data transmission
//! and reception with external audio devices, such as DACs (Digital-to-Analog
//! Converters) and ADCs (Analog-to-Digital Converters) through the I2S
//! interface. Also this module supports different data formats, including
//! varying data and channel widths, different standards, such as the Philips
//! standard and configurable pin mappings for I2S clock (BCLK), word select
//! (WS), and data input/output (DOUT/DIN).
//!
//! The driver uses DMA (Direct Memory Access) for efficient data transfer and
//! supports various configurations, such as different data formats, standards
//! (e.g., Philips) and pin configurations. It relies on other peripheral
//! modules, such as
//!     - `GPIO`
//!     - `DMA`
//!     - `system` (to configure and enable the I2S peripheral)
//!
//! ## Examples
//!
//! ### initialization
//! ```no_run
//! let i2s = I2s::new(
//!     peripherals.I2S0,
//!     Standard::Philips,
//!     DataFormat::Data16Channel16,
//!     44100.Hz(),
//!     dma_channel.configure(
//!         false,
//!         &mut tx_descriptors,
//!         &mut rx_descriptors,
//!         DmaPriority::Priority0,
//!     ),
//!     &clocks,
//! )
//! .with_mclk(io.pins.gpio4);
//! ```
//!
//! ### Reading
//! ```no_run
//! let i2s_rx = i2s.i2s_rx.
//!     .with_bclk(io.pins.gpio1)
//!     .with_ws(io.pins.gpio2)
//!     .with_dout(io.pins.gpio5)
//!     .build();
//!
//! // Creating DMA buffer
//! static mut BUFFER: [u8; 4092 * 4] = [0u8; 4092 * 4];
//! let buffer: &'static mut [u8; 4092 * 4] = unsafe { &mut BUFFER };
//!
//! let mut transfer = i2s_rx.read_dma_circular(buffer).unwrap();
//! println!("Started transfer");
//!
//! loop {
//!     let avail = transfer.available();
//!
//!     if avail > 0 {
//!         let mut rcv = [0u8; 5000];
//!         transfer.pop(&mut rcv[..avail]).unwrap();
//!         println!("Received {:x?}...", &rcv[..30]);
//!     }
//! }
//! ```

use core::marker::PhantomData;

use embedded_dma::{ReadBuffer, WriteBuffer};
use enumset::{EnumSet, EnumSetType};
use private::*;

#[cfg(any(esp32, esp32s3))]
use crate::dma::I2s1Peripheral;
use crate::{
    clock::Clocks,
    dma::{
        Channel,
        ChannelTypes,
        DmaError,
        DmaTransfer,
        I2s0Peripheral,
        I2sPeripheral,
        RxPrivate,
        TxPrivate,
    },
    gpio::OutputPin,
    interrupt::InterruptHandler,
    into_ref,
    peripheral::Peripheral,
    system::PeripheralClockControl,
    Mode,
};

#[derive(EnumSetType)]
pub enum I2sInterrupt {
    TxHung,
    RxHung,
    #[cfg(not(any(esp32, esp32s2)))]
    TxDone,
    #[cfg(not(any(esp32, esp32s2)))]
    RxDone,
}

#[cfg(any(esp32, esp32s2, esp32s3))]
const I2S_LL_MCLK_DIVIDER_BIT_WIDTH: usize = 6;

#[cfg(any(esp32c3, esp32c6, esp32h2))]
const I2S_LL_MCLK_DIVIDER_BIT_WIDTH: usize = 9;

const I2S_LL_MCLK_DIVIDER_MAX: usize = (1 << I2S_LL_MCLK_DIVIDER_BIT_WIDTH) - 1;

trait AcceptedWord {}
impl AcceptedWord for u8 {}
impl AcceptedWord for u16 {}
impl AcceptedWord for u32 {}
impl AcceptedWord for i8 {}
impl AcceptedWord for i16 {}
impl AcceptedWord for i32 {}

/// I2S Error
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    Unknown,
    DmaError(DmaError),
    IllegalArgument,
}

impl From<DmaError> for Error {
    fn from(value: DmaError) -> Self {
        Error::DmaError(value)
    }
}

/// Supported standards.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Standard {
    Philips,
    // Tdm,
    // Pdm,
}

/// Supported data formats
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg(not(any(esp32, esp32s2)))]
pub enum DataFormat {
    Data32Channel32,
    Data32Channel24,
    Data32Channel16,
    Data32Channel8,
    Data16Channel16,
    Data16Channel8,
    Data8Channel8,
}

/// Supported data formats
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg(any(esp32, esp32s2))]
pub enum DataFormat {
    Data32Channel32,
    Data16Channel16,
}

#[cfg(not(any(esp32, esp32s2)))]
impl DataFormat {
    pub fn data_bits(&self) -> u8 {
        match self {
            DataFormat::Data32Channel32 => 32,
            DataFormat::Data32Channel24 => 32,
            DataFormat::Data32Channel16 => 32,
            DataFormat::Data32Channel8 => 32,
            DataFormat::Data16Channel16 => 16,
            DataFormat::Data16Channel8 => 16,
            DataFormat::Data8Channel8 => 8,
        }
    }

    pub fn channel_bits(&self) -> u8 {
        match self {
            DataFormat::Data32Channel32 => 32,
            DataFormat::Data32Channel24 => 24,
            DataFormat::Data32Channel16 => 16,
            DataFormat::Data32Channel8 => 8,
            DataFormat::Data16Channel16 => 16,
            DataFormat::Data16Channel8 => 8,
            DataFormat::Data8Channel8 => 8,
        }
    }
}

#[cfg(any(esp32, esp32s2))]
impl DataFormat {
    pub fn data_bits(&self) -> u8 {
        match self {
            DataFormat::Data32Channel32 => 32,
            DataFormat::Data16Channel16 => 16,
        }
    }

    pub fn channel_bits(&self) -> u8 {
        match self {
            DataFormat::Data32Channel32 => 32,
            DataFormat::Data16Channel16 => 16,
        }
    }
}

/// An in-progress DMA write transfer.
#[must_use]
pub struct I2sWriteDmaTransfer<'t, 'd, T, CH, DmaMode>
where
    T: RegisterAccess,
    CH: ChannelTypes,
    DmaMode: Mode,
{
    i2s_tx: &'t mut I2sTx<'d, T, CH, DmaMode>,
}

impl<'t, 'd, T, CH, DmaMode> I2sWriteDmaTransfer<'t, 'd, T, CH, DmaMode>
where
    T: RegisterAccess,
    CH: ChannelTypes,
    DmaMode: Mode,
{
    /// Amount of bytes which can be pushed.
    /// Only useful for circular DMA transfers
    pub fn available(&mut self) -> usize {
        self.i2s_tx.tx_channel.available()
    }

    /// Push bytes into the DMA buffer.
    /// Only useful for circular DMA transfers
    pub fn push(&mut self, data: &[u8]) -> Result<usize, Error> {
        Ok(self.i2s_tx.tx_channel.push(data)?)
    }

    /// Push bytes into the DMA buffer via the given closure.
    /// The closure *must* return the actual number of bytes written.
    /// The closure *might* get called with a slice which is smaller than the
    /// total available buffer. Only useful for circular DMA transfers
    pub fn push_with(&mut self, f: impl FnOnce(&mut [u8]) -> usize) -> Result<usize, Error> {
        Ok(self.i2s_tx.tx_channel.push_with(f)?)
    }

    /// Stop for the DMA transfer and return the buffer and the
    /// I2sTx instance.
    #[allow(clippy::type_complexity)]
    pub fn stop(self) -> Result<(), DmaError> {
        T::tx_stop();

        if self.i2s_tx.tx_channel.has_error() {
            Err(DmaError::DescriptorError)
        } else {
            Ok(())
        }
    }
}

impl<'t, 'd, T, CH, DmaMode> DmaTransfer for I2sWriteDmaTransfer<'t, 'd, T, CH, DmaMode>
where
    T: RegisterAccess,
    CH: ChannelTypes,
    DmaMode: Mode,
{
    /// Wait for the DMA transfer to complete
    fn wait(self) -> Result<(), DmaError> {
        // Waiting for the DMA transfer is not enough. We need to wait for the
        // peripheral to finish flushing its buffers, too.
        self.i2s_tx.wait_tx_dma_done().ok();

        if self.i2s_tx.tx_channel.has_error() {
            Err(DmaError::DescriptorError)
        } else {
            Ok(())
        }
    }

    /// Check if the DMA transfer is complete
    fn is_done(&self) -> bool {
        self.i2s_tx.tx_channel.is_done()
    }
}

impl<'t, 'd, T, CH, DmaMode> Drop for I2sWriteDmaTransfer<'t, 'd, T, CH, DmaMode>
where
    T: RegisterAccess,
    CH: ChannelTypes,
    DmaMode: Mode,
{
    fn drop(&mut self) {
        self.i2s_tx.wait_tx_dma_done().ok();
    }
}

/// Blocking I2s Write
pub trait I2sWrite<W> {
    fn write(&mut self, words: &[W]) -> Result<(), Error>;
}

/// Initiate a DMA tx transfer
pub trait I2sWriteDma<'d, T, CH, TXBUF, DmaMode>
where
    T: RegisterAccess,
    CH: ChannelTypes,
    DmaMode: Mode,
{
    /// Write I2S.
    /// Returns [I2sWriteDmaTransfer] which represents the in-progress DMA
    /// transfer
    fn write_dma<'t>(
        &'t mut self,
        words: &'t TXBUF,
    ) -> Result<I2sWriteDmaTransfer<'t, 'd, T, CH, DmaMode>, Error>
    where
        TXBUF: ReadBuffer<Word = u8>;

    /// Continuously write to I2S. Returns [I2sWriteDmaTransfer] which
    /// represents the in-progress DMA transfer
    fn write_dma_circular<'t>(
        &'t mut self,
        words: &'t TXBUF,
    ) -> Result<I2sWriteDmaTransfer<'t, 'd, T, CH, DmaMode>, Error>
    where
        TXBUF: ReadBuffer<Word = u8>;
}

/// An in-progress DMA read transfer.
#[must_use]
pub struct I2sReadDmaTransfer<'t, 'd, T, CH, DmaMode>
where
    T: RegisterAccess,
    CH: ChannelTypes,
    DmaMode: Mode,
{
    i2s_rx: &'t mut I2sRx<'d, T, CH, DmaMode>,
}

impl<'t, 'd, T, CH, DmaMode> I2sReadDmaTransfer<'t, 'd, T, CH, DmaMode>
where
    T: RegisterAccess,
    CH: ChannelTypes,
    DmaMode: Mode,
{
    /// Amount of bytes which can be popped
    pub fn available(&mut self) -> usize {
        self.i2s_rx.rx_channel.available()
    }

    pub fn pop(&mut self, data: &mut [u8]) -> Result<usize, Error> {
        Ok(self.i2s_rx.rx_channel.pop(data)?)
    }

    /// Wait for the DMA transfer to complete.
    /// Length of the received data is returned
    #[allow(clippy::type_complexity)]
    pub fn wait_receive(self, dst: &mut [u8]) -> Result<usize, (DmaError, usize)> {
        // Waiting for the DMA transfer is not enough. We need to wait for the
        // peripheral to finish flushing its buffers, too.
        self.i2s_rx.wait_rx_dma_done().ok();
        let len = self.i2s_rx.rx_channel.drain_buffer(dst).unwrap();

        if self.i2s_rx.rx_channel.has_error() {
            Err((DmaError::DescriptorError, len))
        } else {
            Ok(len)
        }
    }
}

impl<'t, 'd, T, CH, DmaMode> DmaTransfer for I2sReadDmaTransfer<'t, 'd, T, CH, DmaMode>
where
    T: RegisterAccess,
    CH: ChannelTypes,
    DmaMode: Mode,
{
    /// Wait for the DMA transfer to complete
    fn wait(self) -> Result<(), DmaError> {
        // Waiting for the DMA transfer is not enough. We need to wait for the
        // peripheral to finish flushing its buffers, too.
        self.i2s_rx.wait_rx_dma_done().ok();

        if self.i2s_rx.rx_channel.has_error() {
            Err(DmaError::DescriptorError)
        } else {
            Ok(())
        }
    }

    /// Check if the DMA transfer is complete
    fn is_done(&self) -> bool {
        self.i2s_rx.rx_channel.is_done()
    }
}

impl<'t, T, CH, DmaMode> Drop for I2sReadDmaTransfer<'t, '_, T, CH, DmaMode>
where
    T: RegisterAccess,
    CH: ChannelTypes,
    DmaMode: Mode,
{
    fn drop(&mut self) {
        self.i2s_rx.wait_rx_dma_done().ok();
    }
}

/// Blocking I2S Read
pub trait I2sRead<W> {
    fn read(&mut self, words: &mut [W]) -> Result<(), Error>;
}

/// Initiate a DMA rx transfer
pub trait I2sReadDma<'d, T, CH, RXBUF, DmaMode>
where
    T: RegisterAccess,
    CH: ChannelTypes,
    DmaMode: Mode,
{
    /// Read I2S.
    /// Returns [I2sReadDmaTransfer] which represents the in-progress DMA
    /// transfer
    fn read_dma<'t>(
        &'t mut self,
        words: &'t mut RXBUF,
    ) -> Result<I2sReadDmaTransfer<'t, 'd, T, CH, DmaMode>, Error>
    where
        RXBUF: WriteBuffer<Word = u8>;

    /// Continuously read from I2S.
    /// Returns [I2sReadDmaTransfer] which represents the in-progress DMA
    /// transfer
    fn read_dma_circular<'t>(
        &'t mut self,
        words: &'t mut RXBUF,
    ) -> Result<I2sReadDmaTransfer<'t, 'd, T, CH, DmaMode>, Error>
    where
        RXBUF: WriteBuffer<Word = u8>;
}

/// Instance of the I2S peripheral driver
pub struct I2s<'d, I, CH, DmaMode>
where
    I: RegisterAccess,
    CH: ChannelTypes,
    DmaMode: Mode,
{
    pub i2s_tx: TxCreator<'d, I, CH, DmaMode>,
    pub i2s_rx: RxCreator<'d, I, CH, DmaMode>,
    phantom: PhantomData<DmaMode>,
}

impl<'d, I, CH, DmaMode> I2s<'d, I, CH, DmaMode>
where
    I: RegisterAccess,
    CH: ChannelTypes,
    DmaMode: Mode,
{
    fn new_internal(
        _i2s: impl Peripheral<P = I> + 'd,
        standard: Standard,
        data_format: DataFormat,
        sample_rate: impl Into<fugit::HertzU32>,
        mut channel: Channel<'d, CH, DmaMode>,
        clocks: &Clocks,
    ) -> Self {
        // on ESP32-C3 / ESP32-S3 and later RX and TX are independent and
        // could be configured totally independently but for now handle all
        // the targets the same and force same configuration for both, TX and RX

        channel.tx.init_channel();
        PeripheralClockControl::enable(I::get_peripheral());
        I::set_clock(calculate_clock(
            sample_rate,
            2,
            data_format.channel_bits(),
            clocks,
        ));
        I::configure(&standard, &data_format);
        I::set_master();
        I::update();

        Self {
            i2s_tx: TxCreator {
                register_access: PhantomData,
                tx_channel: channel.tx,
                phantom: PhantomData,
            },
            i2s_rx: RxCreator {
                register_access: PhantomData,
                rx_channel: channel.rx,
                phantom: PhantomData,
            },
            phantom: PhantomData,
        }
    }
}

impl<'d, I, CH, DmaMode> I2s<'d, I, CH, DmaMode>
where
    I: RegisterAccess,
    CH: ChannelTypes,
    DmaMode: Mode,
{
    /// Sets the interrupt handler, enables it with
    /// [crate::interrupt::Priority::min()]
    ///
    /// Interrupts are not enabled at the peripheral level here.
    pub fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        I::set_interrupt_handler(handler);
    }

    /// Listen for the given interrupts
    pub fn listen(&mut self, interrupts: EnumSet<I2sInterrupt>) {
        I::listen(interrupts);
    }

    /// Unlisten the given interrupts
    pub fn unlisten(&mut self, interrupts: EnumSet<I2sInterrupt>) {
        I::unlisten(interrupts);
    }

    /// Gets asserted interrupts
    pub fn interrupts(&mut self) -> EnumSet<I2sInterrupt> {
        I::interrupts()
    }

    /// Resets asserted interrupts
    pub fn clear_interrupts(&mut self, interrupts: EnumSet<I2sInterrupt>) {
        I::clear_interrupts(interrupts);
    }
}

impl<'d, I, CH, DmaMode> I2s<'d, I, CH, DmaMode>
where
    I: RegisterAccess,
    CH: ChannelTypes,
    DmaMode: Mode,
{
    /// Construct a new I2S peripheral driver instance for the first I2S
    /// peripheral
    pub fn new(
        i2s: impl Peripheral<P = I> + 'd,
        standard: Standard,
        data_format: DataFormat,
        sample_rate: impl Into<fugit::HertzU32>,
        channel: Channel<'d, CH, DmaMode>,
        clocks: &Clocks,
    ) -> Self
    where
        I: I2s0Instance,
        CH::P: I2sPeripheral + I2s0Peripheral,
        DmaMode: Mode,
    {
        Self::new_internal(i2s, standard, data_format, sample_rate, channel, clocks)
    }

    /// Construct a new I2S peripheral driver instance for the second I2S
    /// peripheral
    #[cfg(any(esp32s3, esp32))]
    pub fn new_i2s1(
        i2s: impl Peripheral<P = I> + 'd,
        standard: Standard,
        data_format: DataFormat,
        sample_rate: impl Into<fugit::HertzU32>,
        channel: Channel<'d, CH, DmaMode>,
        clocks: &Clocks,
    ) -> Self
    where
        I: I2s1Instance,
        CH::P: I2sPeripheral + I2s1Peripheral,
    {
        Self::new_internal(i2s, standard, data_format, sample_rate, channel, clocks)
    }

    pub fn with_mclk<P: OutputPin>(self, pin: impl Peripheral<P = P> + 'd) -> Self {
        into_ref!(pin);
        pin.set_to_push_pull_output()
            .connect_peripheral_to_output(I::mclk_signal());
        self
    }
}

/// I2S TX channel
pub struct I2sTx<'d, T, CH, DmaMode>
where
    T: RegisterAccess,
    CH: ChannelTypes,
{
    register_access: PhantomData<T>,
    tx_channel: CH::Tx<'d>,
    phantom: PhantomData<DmaMode>,
}

impl<'d, T, CH, DmaMode> core::fmt::Debug for I2sTx<'d, T, CH, DmaMode>
where
    T: RegisterAccess,
    CH: ChannelTypes,
    DmaMode: Mode,
{
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("I2sTx").finish()
    }
}

impl<'d, T, CH, DmaMode> I2sTx<'d, T, CH, DmaMode>
where
    T: RegisterAccess,
    CH: ChannelTypes,
    DmaMode: Mode,
{
    fn new(tx_channel: CH::Tx<'d>) -> Self {
        Self {
            register_access: PhantomData,
            tx_channel,
            phantom: PhantomData,
        }
    }

    fn write_bytes(&mut self, data: &[u8]) -> Result<(), Error> {
        let ptr = data as *const _ as *const u8;

        // Reset TX unit and TX FIFO
        T::reset_tx();

        // Enable corresponding interrupts if needed

        // configure DMA outlink
        self.tx_channel
            .prepare_transfer_without_start(T::get_dma_peripheral(), false, ptr, data.len())
            .and_then(|_| self.tx_channel.start_transfer())?;

        // set I2S_TX_STOP_EN if needed

        // start: set I2S_TX_START
        T::tx_start();

        // wait until I2S_TX_IDLE is 1
        T::wait_for_tx_done();

        Ok(())
    }

    fn start_tx_transfer<'t, TXBUF>(
        &'t mut self,
        words: &'t TXBUF,
        circular: bool,
    ) -> Result<I2sWriteDmaTransfer<'t, 'd, T, CH, DmaMode>, Error>
    where
        TXBUF: ReadBuffer<Word = u8>,
        DmaMode: Mode,
    {
        let (ptr, len) = unsafe { words.read_buffer() };

        // Reset TX unit and TX FIFO
        T::reset_tx();

        // Enable corresponding interrupts if needed

        // configure DMA outlink
        self.tx_channel
            .prepare_transfer_without_start(T::get_dma_peripheral(), circular, ptr, len)
            .and_then(|_| self.tx_channel.start_transfer())?;

        // set I2S_TX_STOP_EN if needed

        // start: set I2S_TX_START
        T::tx_start();

        Ok(I2sWriteDmaTransfer { i2s_tx: self })
    }

    fn wait_tx_dma_done(&self) -> Result<(), Error> {
        // wait until I2S_TX_IDLE is 1
        T::wait_for_tx_done();

        Ok(())
    }

    #[cfg(feature = "async")]
    fn start_tx_transfer_async(
        &mut self,
        ptr: *const u8,
        len: usize,
        circular: bool,
    ) -> Result<(), Error> {
        // Reset TX unit and TX FIFO
        T::reset_tx();

        // Enable corresponding interrupts if needed

        // configure DMA outlink
        self.tx_channel
            .prepare_transfer_without_start(T::get_dma_peripheral(), circular, ptr, len)
            .and_then(|_| self.tx_channel.start_transfer())?;

        // set I2S_TX_STOP_EN if needed

        // start: set I2S_TX_START
        T::tx_start();

        Ok(())
    }
}

impl<'d, T, W, CH, DmaMode> I2sWrite<W> for I2sTx<'d, T, CH, DmaMode>
where
    T: RegisterAccess,
    CH: ChannelTypes,
    W: AcceptedWord,
    DmaMode: Mode,
{
    fn write(&mut self, words: &[W]) -> Result<(), Error> {
        self.write_bytes(unsafe {
            core::slice::from_raw_parts(
                words as *const _ as *const u8,
                core::mem::size_of_val(words),
            )
        })
    }
}

impl<'d, T, CH, TXBUF, DmaMode> I2sWriteDma<'d, T, CH, TXBUF, DmaMode> for I2sTx<'d, T, CH, DmaMode>
where
    T: RegisterAccess,
    CH: ChannelTypes,
    DmaMode: Mode,
{
    fn write_dma<'t>(
        &'t mut self,
        words: &'t TXBUF,
    ) -> Result<I2sWriteDmaTransfer<'t, 'd, T, CH, DmaMode>, Error>
    where
        TXBUF: ReadBuffer<Word = u8>,
    {
        self.start_tx_transfer(words, false)
    }

    fn write_dma_circular<'t>(
        &'t mut self,
        words: &'t TXBUF,
    ) -> Result<I2sWriteDmaTransfer<'t, 'd, T, CH, DmaMode>, Error>
    where
        TXBUF: ReadBuffer<Word = u8>,
    {
        self.start_tx_transfer(words, true)
    }
}

/// I2S RX channel
pub struct I2sRx<'d, T, CH, DmaMode>
where
    T: RegisterAccess,
    CH: ChannelTypes,
    DmaMode: Mode,
{
    register_access: PhantomData<T>,
    rx_channel: CH::Rx<'d>,
    phantom: PhantomData<DmaMode>,
}

impl<'d, T, CH, DmaMode> core::fmt::Debug for I2sRx<'d, T, CH, DmaMode>
where
    T: RegisterAccess,
    CH: ChannelTypes,
    DmaMode: Mode,
{
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("I2sRx").finish()
    }
}

impl<'d, T, CH, DmaMode> I2sRx<'d, T, CH, DmaMode>
where
    T: RegisterAccess,
    CH: ChannelTypes,
    DmaMode: Mode,
{
    fn new(rx_channel: CH::Rx<'d>) -> Self {
        Self {
            register_access: PhantomData,
            rx_channel,
            phantom: PhantomData,
        }
    }

    fn read_bytes(&mut self, data: &mut [u8]) -> Result<(), Error> {
        let ptr = data as *mut _ as *mut u8;

        // Reset RX unit and RX FIFO
        T::reset_rx();

        // Enable corresponding interrupts if needed

        // configure DMA outlink
        self.rx_channel
            .prepare_transfer_without_start(false, T::get_dma_peripheral(), ptr, data.len())
            .and_then(|_| self.rx_channel.start_transfer())?;

        // set I2S_TX_STOP_EN if needed

        // start: set I2S_TX_START
        T::rx_start(data.len() - 1);

        // wait until I2S_TX_IDLE is 1
        T::wait_for_rx_done();

        Ok(())
    }

    fn start_rx_transfer<'t, RXBUF>(
        &'t mut self,
        words: &'t mut RXBUF,
        circular: bool,
    ) -> Result<I2sReadDmaTransfer<'t, 'd, T, CH, DmaMode>, Error>
    where
        RXBUF: WriteBuffer<Word = u8>,
    {
        let (ptr, len) = unsafe { words.write_buffer() };

        if len % 4 != 0 {
            return Err(Error::IllegalArgument);
        }

        // Reset TX unit and TX FIFO
        T::reset_rx();

        // Enable corresponding interrupts if needed

        // configure DMA outlink
        self.rx_channel
            .prepare_transfer_without_start(circular, T::get_dma_peripheral(), ptr, len)
            .and_then(|_| self.rx_channel.start_transfer())?;

        // set I2S_TX_STOP_EN if needed

        // start: set I2S_RX_START
        #[cfg(not(esp32))]
        T::rx_start(len - 1);

        #[cfg(esp32)]
        T::rx_start(len);

        Ok(I2sReadDmaTransfer { i2s_rx: self })
    }

    fn wait_rx_dma_done(&self) -> Result<(), Error> {
        T::wait_for_rx_done();

        Ok(())
    }

    #[cfg(feature = "async")]
    fn start_rx_transfer_async(
        &mut self,
        ptr: *mut u8,
        len: usize,
        circular: bool,
    ) -> Result<(), Error> {
        if len % 4 != 0 {
            return Err(Error::IllegalArgument);
        }

        // Reset TX unit and TX FIFO
        T::reset_rx();

        // Enable corresponding interrupts if needed

        // configure DMA outlink
        self.rx_channel
            .prepare_transfer_without_start(circular, T::get_dma_peripheral(), ptr, len)
            .and_then(|_| self.rx_channel.start_transfer())?;

        // set I2S_TX_STOP_EN if needed

        // start: set I2S_RX_START
        #[cfg(not(esp32))]
        T::rx_start(len - 1);

        #[cfg(esp32)]
        T::rx_start(len);

        Ok(())
    }
}

impl<'d, W, T, CH, DmaMode> I2sRead<W> for I2sRx<'d, T, CH, DmaMode>
where
    T: RegisterAccess,
    CH: ChannelTypes,
    W: AcceptedWord,
    DmaMode: Mode,
{
    fn read(&mut self, words: &mut [W]) -> Result<(), Error> {
        if core::mem::size_of_val(words) > 4096 || words.is_empty() {
            return Err(Error::IllegalArgument);
        }

        self.read_bytes(unsafe {
            core::slice::from_raw_parts_mut(
                words as *mut _ as *mut u8,
                core::mem::size_of_val(words),
            )
        })
    }
}

impl<'d, T, CH, RXBUF, DmaMode> I2sReadDma<'d, T, CH, RXBUF, DmaMode> for I2sRx<'d, T, CH, DmaMode>
where
    T: RegisterAccess,
    CH: ChannelTypes,
    DmaMode: Mode,
{
    fn read_dma<'t>(
        &'t mut self,
        words: &'t mut RXBUF,
    ) -> Result<I2sReadDmaTransfer<'t, 'd, T, CH, DmaMode>, Error>
    where
        RXBUF: WriteBuffer<Word = u8>,
    {
        self.start_rx_transfer(words, false)
    }

    fn read_dma_circular<'t>(
        &'t mut self,
        words: &'t mut RXBUF,
    ) -> Result<I2sReadDmaTransfer<'t, 'd, T, CH, DmaMode>, Error>
    where
        RXBUF: WriteBuffer<Word = u8>,
    {
        self.start_rx_transfer(words, true)
    }
}

pub trait RegisterAccess: RegisterAccessPrivate {}

mod private {
    use core::marker::PhantomData;

    use enumset::EnumSet;
    use fugit::HertzU32;

    use super::{
        DataFormat,
        I2sInterrupt,
        I2sRx,
        I2sTx,
        RegisterAccess,
        Standard,
        I2S_LL_MCLK_DIVIDER_MAX,
    };
    #[cfg(not(any(esp32, esp32s3)))]
    use crate::peripherals::i2s0::RegisterBlock;
    // on ESP32-S3 I2S1 doesn't support all features - use that to avoid using those features
    // by accident
    #[cfg(any(esp32, esp32s3))]
    use crate::peripherals::{i2s1::RegisterBlock, I2S1};
    use crate::{
        clock::Clocks,
        dma::{ChannelTypes, DmaPeripheral},
        gpio::{InputPin, InputSignal, OutputPin, OutputSignal},
        interrupt::InterruptHandler,
        into_ref,
        peripherals::I2S0,
        system::Peripheral,
        Mode,
    };

    pub struct TxCreator<'d, T, CH, DmaMode>
    where
        T: RegisterAccess,
        CH: ChannelTypes,
        DmaMode: Mode,
    {
        pub register_access: PhantomData<T>,
        pub tx_channel: CH::Tx<'d>,
        pub(crate) phantom: PhantomData<DmaMode>,
    }

    impl<'d, T, CH, DmaMode> TxCreator<'d, T, CH, DmaMode>
    where
        T: RegisterAccess,
        CH: ChannelTypes,
        DmaMode: Mode,
    {
        pub fn build(self) -> I2sTx<'d, T, CH, DmaMode> {
            I2sTx::new(self.tx_channel)
        }

        pub fn with_bclk<P>(self, pin: impl crate::peripheral::Peripheral<P = P> + 'd) -> Self
        where
            P: OutputPin,
        {
            into_ref!(pin);
            pin.set_to_push_pull_output()
                .connect_peripheral_to_output(T::bclk_signal());
            self
        }

        pub fn with_ws<P>(self, pin: impl crate::peripheral::Peripheral<P = P> + 'd) -> Self
        where
            P: OutputPin,
        {
            into_ref!(pin);
            pin.set_to_push_pull_output()
                .connect_peripheral_to_output(T::ws_signal());
            self
        }

        pub fn with_dout<P>(self, pin: impl crate::peripheral::Peripheral<P = P> + 'd) -> Self
        where
            P: OutputPin,
        {
            into_ref!(pin);
            pin.set_to_push_pull_output()
                .connect_peripheral_to_output(T::dout_signal());
            self
        }
    }

    pub struct RxCreator<'d, T, CH, DmaMode>
    where
        T: RegisterAccess,
        CH: ChannelTypes,
        DmaMode: Mode,
    {
        pub register_access: PhantomData<T>,
        pub rx_channel: CH::Rx<'d>,
        pub(crate) phantom: PhantomData<DmaMode>,
    }

    impl<'d, T, CH, DmaMode> RxCreator<'d, T, CH, DmaMode>
    where
        T: RegisterAccess,
        CH: ChannelTypes,
        DmaMode: Mode,
    {
        pub fn build(self) -> I2sRx<'d, T, CH, DmaMode> {
            I2sRx::new(self.rx_channel)
        }

        pub fn with_bclk<P>(self, pin: impl crate::peripheral::Peripheral<P = P> + 'd) -> Self
        where
            P: OutputPin,
        {
            into_ref!(pin);
            pin.set_to_push_pull_output()
                .connect_peripheral_to_output(T::bclk_rx_signal());
            self
        }

        pub fn with_ws<P>(self, pin: impl crate::peripheral::Peripheral<P = P> + 'd) -> Self
        where
            P: OutputPin,
        {
            into_ref!(pin);
            pin.set_to_push_pull_output()
                .connect_peripheral_to_output(T::ws_rx_signal());
            self
        }

        pub fn with_din<P>(self, pin: impl crate::peripheral::Peripheral<P = P> + 'd) -> Self
        where
            P: InputPin,
        {
            into_ref!(pin);
            pin.set_to_input()
                .connect_input_to_peripheral(T::din_signal());
            self
        }
    }

    pub trait Signals {
        fn get_peripheral() -> Peripheral;

        fn get_dma_peripheral() -> DmaPeripheral;

        fn mclk_signal() -> OutputSignal;

        fn bclk_signal() -> OutputSignal;

        fn ws_signal() -> OutputSignal;

        fn dout_signal() -> OutputSignal;

        fn bclk_rx_signal() -> OutputSignal;

        fn ws_rx_signal() -> OutputSignal;

        fn din_signal() -> InputSignal;
    }

    pub trait RegBlock {
        fn register_block() -> &'static RegisterBlock;
    }

    #[cfg(any(esp32, esp32s2))]
    pub trait RegisterAccessPrivate: Signals + RegBlock {
        fn set_interrupt_handler(handler: InterruptHandler);

        fn listen(interrupts: EnumSet<I2sInterrupt>) {
            let reg_block = Self::register_block();

            for interrupt in interrupts {
                match interrupt {
                    I2sInterrupt::TxHung => {
                        reg_block.int_ena().modify(|_, w| w.tx_hung().set_bit())
                    }
                    I2sInterrupt::RxHung => {
                        reg_block.int_ena().modify(|_, w| w.rx_hung().set_bit())
                    }
                }
            }
        }

        fn unlisten(interrupts: EnumSet<I2sInterrupt>) {
            let reg_block = Self::register_block();

            for interrupt in interrupts {
                match interrupt {
                    I2sInterrupt::TxHung => {
                        reg_block.int_ena().modify(|_, w| w.tx_hung().clear_bit())
                    }
                    I2sInterrupt::RxHung => {
                        reg_block.int_ena().modify(|_, w| w.rx_hung().clear_bit())
                    }
                }
            }
        }

        fn interrupts() -> EnumSet<I2sInterrupt> {
            let mut res = EnumSet::new();
            let reg_block = Self::register_block();
            let ints = reg_block.int_st().read();

            if ints.tx_hung().bit() {
                res.insert(I2sInterrupt::TxHung);
            }
            if ints.rx_hung().bit() {
                res.insert(I2sInterrupt::RxHung);
            }

            res
        }

        fn clear_interrupts(interrupts: EnumSet<I2sInterrupt>) {
            let reg_block = Self::register_block();

            for interrupt in interrupts {
                match interrupt {
                    I2sInterrupt::TxHung => reg_block
                        .int_clr()
                        .write(|w| w.tx_hung().clear_bit_by_one()),
                    I2sInterrupt::RxHung => reg_block
                        .int_clr()
                        .write(|w| w.rx_hung().clear_bit_by_one()),
                }
            }
        }

        fn set_clock(clock_settings: I2sClockDividers) {
            let i2s = Self::register_block();

            i2s.clkm_conf().modify(|r, w| unsafe {
                w.bits(r.bits() | (crate::soc::constants::I2S_DEFAULT_CLK_SRC << 21))
                // select PLL_160M
            });

            #[cfg(esp32)]
            i2s.clkm_conf().modify(|_, w| w.clka_ena().clear_bit());

            i2s.clkm_conf().modify(|_, w| {
                w.clk_en()
                    .set_bit()
                    .clkm_div_num()
                    .variant(clock_settings.mclk_divider as u8)
            });

            i2s.clkm_conf().modify(|_, w| {
                w.clkm_div_a()
                    .variant(clock_settings.denominator as u8)
                    .clkm_div_b()
                    .variant(clock_settings.numerator as u8)
            });

            i2s.sample_rate_conf().modify(|_, w| {
                w.tx_bck_div_num()
                    .variant(clock_settings.bclk_divider as u8)
                    .rx_bck_div_num()
                    .variant(clock_settings.bclk_divider as u8)
            });
        }

        fn configure(_standard: &Standard, data_format: &DataFormat) {
            let i2s = Self::register_block();

            let fifo_mod = match data_format {
                DataFormat::Data32Channel32 => 2,
                DataFormat::Data16Channel16 => 0,
            };

            i2s.sample_rate_conf()
                .modify(|_, w| w.tx_bits_mod().variant(data_format.channel_bits()));
            i2s.sample_rate_conf()
                .modify(|_, w| w.rx_bits_mod().variant(data_format.channel_bits()));

            i2s.conf().modify(|_, w| {
                w.tx_slave_mod()
                    .clear_bit()
                    .rx_slave_mod()
                    .clear_bit()
                    .tx_msb_shift()
                    .set_bit() // ?
                    .rx_msb_shift()
                    .set_bit() // ?
                    .tx_short_sync()
                    .variant(false) //??
                    .rx_short_sync()
                    .variant(false) //??
                    .tx_msb_right()
                    .clear_bit()
                    .rx_msb_right()
                    .clear_bit()
                    .tx_right_first()
                    .clear_bit()
                    .rx_right_first()
                    .clear_bit()
                    .tx_mono()
                    .clear_bit()
                    .rx_mono()
                    .clear_bit()
                    .sig_loopback()
                    .clear_bit()
            });

            i2s.fifo_conf().modify(|_, w| {
                w.tx_fifo_mod()
                    .variant(fifo_mod)
                    .tx_fifo_mod_force_en()
                    .set_bit()
                    .dscr_en()
                    .set_bit()
                    .rx_fifo_mod()
                    .variant(fifo_mod)
                    .rx_fifo_mod_force_en()
                    .set_bit()
            });

            i2s.conf_chan()
                .modify(|_, w| w.tx_chan_mod().variant(0).rx_chan_mod().variant(0)); // for now only stereo

            i2s.conf1()
                .modify(|_, w| w.tx_pcm_bypass().set_bit().rx_pcm_bypass().set_bit());

            i2s.pd_conf()
                .modify(|_, w| w.fifo_force_pu().set_bit().fifo_force_pd().clear_bit());

            i2s.conf2()
                .modify(|_, w| w.camera_en().clear_bit().lcd_en().clear_bit());
        }

        fn set_master() {
            let i2s = Self::register_block();
            i2s.conf()
                .modify(|_, w| w.rx_slave_mod().clear_bit().tx_slave_mod().clear_bit());
        }

        fn update() {
            // nothing to do
        }

        fn reset_tx() {
            let i2s = Self::register_block();
            i2s.conf()
                .modify(|_, w| w.tx_reset().set_bit().tx_fifo_reset().set_bit());
            i2s.conf()
                .modify(|_, w| w.tx_reset().clear_bit().tx_fifo_reset().clear_bit());

            i2s.lc_conf().modify(|_, w| w.out_rst().set_bit());
            i2s.lc_conf().modify(|_, w| w.out_rst().clear_bit());

            i2s.int_clr().write(|w| {
                w.out_done()
                    .clear_bit_by_one()
                    .out_total_eof()
                    .clear_bit_by_one()
            });
        }

        fn tx_start() {
            let i2s = Self::register_block();
            i2s.conf().modify(|_, w| w.tx_start().set_bit());
        }

        fn tx_stop() {
            let i2s = Self::register_block();
            i2s.conf().modify(|_, w| w.tx_start().clear_bit());
        }

        fn wait_for_tx_done() {
            let i2s = Self::register_block();
            while i2s.state().read().tx_idle().bit_is_clear() {
                // wait
            }

            i2s.conf().modify(|_, w| w.tx_start().clear_bit());
        }

        fn reset_rx() {
            let i2s = Self::register_block();
            i2s.conf()
                .modify(|_, w| w.rx_reset().set_bit().rx_fifo_reset().set_bit());
            i2s.conf()
                .modify(|_, w| w.rx_reset().clear_bit().rx_fifo_reset().clear_bit());

            i2s.lc_conf().modify(|_, w| w.in_rst().set_bit());
            i2s.lc_conf().modify(|_, w| w.in_rst().clear_bit());

            i2s.int_clr().write(|w| {
                w.in_done()
                    .clear_bit_by_one()
                    .in_suc_eof()
                    .clear_bit_by_one()
            });
        }

        fn rx_start(len: usize) {
            let i2s = Self::register_block();

            i2s.int_clr().write(|w| w.in_suc_eof().clear_bit_by_one());

            #[cfg(not(esp32))]
            i2s.rxeof_num()
                .modify(|_, w| w.rx_eof_num().variant(len as u32));

            // On ESP32, the eof_num count in words.
            #[cfg(esp32)]
            i2s.rxeof_num()
                .modify(|_, w| w.rx_eof_num().variant((len / 4) as u32));

            i2s.conf().modify(|_, w| w.rx_start().set_bit());
        }

        fn wait_for_rx_done() {
            let i2s = Self::register_block();
            while i2s.int_raw().read().in_suc_eof().bit_is_clear() {
                // wait
            }

            i2s.int_clr().write(|w| w.in_suc_eof().clear_bit_by_one());
        }
    }

    #[cfg(any(esp32c3, esp32c6, esp32h2, esp32s3))]
    pub trait RegisterAccessPrivate: Signals + RegBlock {
        fn set_interrupt_handler(handler: InterruptHandler);

        fn listen(interrupts: EnumSet<I2sInterrupt>) {
            let reg_block = Self::register_block();

            for interrupt in interrupts {
                match interrupt {
                    I2sInterrupt::TxHung => {
                        reg_block.int_ena().modify(|_, w| w.tx_hung().set_bit())
                    }
                    I2sInterrupt::RxHung => {
                        reg_block.int_ena().modify(|_, w| w.rx_hung().set_bit())
                    }
                    I2sInterrupt::TxDone => {
                        reg_block.int_ena().modify(|_, w| w.tx_done().set_bit())
                    }
                    I2sInterrupt::RxDone => {
                        reg_block.int_ena().modify(|_, w| w.rx_done().set_bit())
                    }
                }
            }
        }

        fn unlisten(interrupts: EnumSet<I2sInterrupt>) {
            let reg_block = Self::register_block();

            for interrupt in interrupts {
                match interrupt {
                    I2sInterrupt::TxHung => {
                        reg_block.int_ena().modify(|_, w| w.tx_hung().clear_bit())
                    }
                    I2sInterrupt::RxHung => {
                        reg_block.int_ena().modify(|_, w| w.rx_hung().clear_bit())
                    }
                    I2sInterrupt::TxDone => {
                        reg_block.int_ena().modify(|_, w| w.tx_done().clear_bit())
                    }
                    I2sInterrupt::RxDone => {
                        reg_block.int_ena().modify(|_, w| w.rx_done().clear_bit())
                    }
                }
            }
        }

        fn interrupts() -> EnumSet<I2sInterrupt> {
            let mut res = EnumSet::new();
            let reg_block = Self::register_block();
            let ints = reg_block.int_st().read();

            if ints.tx_hung().bit() {
                res.insert(I2sInterrupt::TxHung);
            }
            if ints.rx_hung().bit() {
                res.insert(I2sInterrupt::RxHung);
            }
            if ints.tx_done().bit() {
                res.insert(I2sInterrupt::TxDone);
            }
            if ints.rx_done().bit() {
                res.insert(I2sInterrupt::RxDone);
            }

            res
        }

        fn clear_interrupts(interrupts: EnumSet<I2sInterrupt>) {
            let reg_block = Self::register_block();

            for interrupt in interrupts {
                match interrupt {
                    I2sInterrupt::TxHung => reg_block
                        .int_clr()
                        .write(|w| w.tx_hung().clear_bit_by_one()),
                    I2sInterrupt::RxHung => reg_block
                        .int_clr()
                        .write(|w| w.rx_hung().clear_bit_by_one()),
                    I2sInterrupt::TxDone => reg_block
                        .int_clr()
                        .write(|w| w.tx_done().clear_bit_by_one()),
                    I2sInterrupt::RxDone => reg_block
                        .int_clr()
                        .write(|w| w.rx_done().clear_bit_by_one()),
                }
            }
        }

        #[cfg(any(esp32c3, esp32s3))]
        fn set_clock(clock_settings: I2sClockDividers) {
            let i2s = Self::register_block();

            let clkm_div_x: u32;
            let clkm_div_y: u32;
            let clkm_div_z: u32;
            let clkm_div_yn1: u32;

            if clock_settings.denominator == 0 || clock_settings.numerator == 0 {
                clkm_div_x = 0;
                clkm_div_y = 0;
                clkm_div_z = 0;
                clkm_div_yn1 = 1;
            } else if clock_settings.numerator > clock_settings.denominator / 2 {
                clkm_div_x = clock_settings
                    .denominator
                    .overflowing_div(
                        clock_settings
                            .denominator
                            .overflowing_sub(clock_settings.numerator)
                            .0,
                    )
                    .0
                    .overflowing_sub(1)
                    .0;
                clkm_div_y = clock_settings.denominator
                    % (clock_settings
                        .denominator
                        .overflowing_sub(clock_settings.numerator)
                        .0);
                clkm_div_z = clock_settings
                    .denominator
                    .overflowing_sub(clock_settings.numerator)
                    .0;
                clkm_div_yn1 = 1;
            } else {
                clkm_div_x = clock_settings.denominator / clock_settings.numerator - 1;
                clkm_div_y = clock_settings.denominator % clock_settings.numerator;
                clkm_div_z = clock_settings.numerator;
                clkm_div_yn1 = 0;
            }

            i2s.tx_clkm_div_conf().modify(|_, w| {
                w.tx_clkm_div_x()
                    .variant(clkm_div_x as u16)
                    .tx_clkm_div_y()
                    .variant(clkm_div_y as u16)
                    .tx_clkm_div_yn1()
                    .variant(clkm_div_yn1 != 0)
                    .tx_clkm_div_z()
                    .variant(clkm_div_z as u16)
            });

            i2s.tx_clkm_conf().modify(|_, w| {
                w.clk_en()
                    .set_bit()
                    .tx_clk_active()
                    .set_bit()
                    .tx_clk_sel()
                    .variant(crate::soc::constants::I2S_DEFAULT_CLK_SRC) // for now fixed at 160MHz
                    .tx_clkm_div_num()
                    .variant(clock_settings.mclk_divider as u8)
            });

            i2s.tx_conf1().modify(|_, w| {
                w.tx_bck_div_num()
                    .variant((clock_settings.bclk_divider - 1) as u8)
            });

            i2s.rx_clkm_div_conf().modify(|_, w| {
                w.rx_clkm_div_x()
                    .variant(clkm_div_x as u16)
                    .rx_clkm_div_y()
                    .variant(clkm_div_y as u16)
                    .rx_clkm_div_yn1()
                    .variant(clkm_div_yn1 != 0)
                    .rx_clkm_div_z()
                    .variant(clkm_div_z as u16)
            });

            i2s.rx_clkm_conf().modify(|_, w| {
                w.rx_clk_active()
                    .set_bit()
                    .rx_clk_sel()
                    .variant(crate::soc::constants::I2S_DEFAULT_CLK_SRC) // for now fixed at 160MHz
                    .rx_clkm_div_num()
                    .variant(clock_settings.mclk_divider as u8)
                    .mclk_sel()
                    .variant(true)
            });

            i2s.rx_conf1().modify(|_, w| {
                w.rx_bck_div_num()
                    .variant((clock_settings.bclk_divider - 1) as u8)
            });
        }

        #[cfg(any(esp32c6, esp32h2))]
        fn set_clock(clock_settings: I2sClockDividers) {
            let i2s = Self::register_block();
            let pcr = unsafe { &*crate::peripherals::PCR::PTR }; // I2S clocks are configured via PCR

            let clkm_div_x: u32;
            let clkm_div_y: u32;
            let clkm_div_z: u32;
            let clkm_div_yn1: u32;

            if clock_settings.denominator == 0 || clock_settings.numerator == 0 {
                clkm_div_x = 0;
                clkm_div_y = 0;
                clkm_div_z = 0;
                clkm_div_yn1 = 1;
            } else if clock_settings.numerator > clock_settings.denominator / 2 {
                clkm_div_x = clock_settings
                    .denominator
                    .overflowing_div(
                        clock_settings
                            .denominator
                            .overflowing_sub(clock_settings.numerator)
                            .0,
                    )
                    .0
                    .overflowing_sub(1)
                    .0;
                clkm_div_y = clock_settings.denominator
                    % (clock_settings
                        .denominator
                        .overflowing_sub(clock_settings.numerator)
                        .0);
                clkm_div_z = clock_settings
                    .denominator
                    .overflowing_sub(clock_settings.numerator)
                    .0;
                clkm_div_yn1 = 1;
            } else {
                clkm_div_x = clock_settings.denominator / clock_settings.numerator - 1;
                clkm_div_y = clock_settings.denominator % clock_settings.numerator;
                clkm_div_z = clock_settings.numerator;
                clkm_div_yn1 = 0;
            }

            pcr.i2s_tx_clkm_div_conf().modify(|_, w| {
                w.i2s_tx_clkm_div_x()
                    .variant(clkm_div_x as u16)
                    .i2s_tx_clkm_div_y()
                    .variant(clkm_div_y as u16)
                    .i2s_tx_clkm_div_yn1()
                    .variant(clkm_div_yn1 != 0)
                    .i2s_tx_clkm_div_z()
                    .variant(clkm_div_z as u16)
            });

            pcr.i2s_tx_clkm_conf().modify(|_, w| {
                w.i2s_tx_clkm_en()
                    .set_bit()
                    .i2s_tx_clkm_sel()
                    .variant(crate::soc::constants::I2S_DEFAULT_CLK_SRC) // for now fixed at 160MHz for C6 and 96MHz for H2
                    .i2s_tx_clkm_div_num()
                    .variant(clock_settings.mclk_divider as u8)
            });

            #[cfg(not(esp32h2))]
            i2s.tx_conf1().modify(|_, w| {
                w.tx_bck_div_num()
                    .variant((clock_settings.bclk_divider - 1) as u8)
            });
            #[cfg(esp32h2)]
            i2s.tx_conf().modify(|_, w| {
                w.tx_bck_div_num()
                    .variant((clock_settings.bclk_divider - 1) as u8)
            });

            pcr.i2s_rx_clkm_div_conf().modify(|_, w| {
                w.i2s_rx_clkm_div_x()
                    .variant(clkm_div_x as u16)
                    .i2s_rx_clkm_div_y()
                    .variant(clkm_div_y as u16)
                    .i2s_rx_clkm_div_yn1()
                    .variant(clkm_div_yn1 != 0)
                    .i2s_rx_clkm_div_z()
                    .variant(clkm_div_z as u16)
            });

            pcr.i2s_rx_clkm_conf().modify(|_, w| {
                w.i2s_rx_clkm_en()
                    .set_bit()
                    .i2s_rx_clkm_sel()
                    .variant(crate::soc::constants::I2S_DEFAULT_CLK_SRC) // for now fixed at 160MHz for C6 and 96MHz for H2
                    .i2s_rx_clkm_div_num()
                    .variant(clock_settings.mclk_divider as u8)
                    .i2s_mclk_sel()
                    .variant(true)
            });
            #[cfg(not(esp32h2))]
            i2s.rx_conf1().modify(|_, w| {
                w.rx_bck_div_num()
                    .variant((clock_settings.bclk_divider - 1) as u8)
            });
            #[cfg(esp32h2)]
            i2s.rx_conf().modify(|_, w| {
                w.rx_bck_div_num()
                    .variant((clock_settings.bclk_divider - 1) as u8)
            });
        }

        fn configure(_standard: &Standard, data_format: &DataFormat) {
            let i2s = Self::register_block();

            #[allow(clippy::useless_conversion)]
            i2s.tx_conf1().modify(|_, w| {
                w.tx_tdm_ws_width()
                    .variant((data_format.channel_bits() - 1).into())
                    .tx_bits_mod()
                    .variant(data_format.data_bits() - 1)
                    .tx_tdm_chan_bits()
                    .variant(data_format.channel_bits() - 1)
                    .tx_half_sample_bits()
                    .variant(data_format.channel_bits() - 1)
            });
            #[cfg(not(esp32h2))]
            i2s.tx_conf1().modify(|_, w| w.tx_msb_shift().set_bit());
            #[cfg(esp32h2)]
            i2s.tx_conf().modify(|_, w| w.tx_msb_shift().set_bit());
            i2s.tx_conf().modify(|_, w| {
                w.tx_mono()
                    .clear_bit()
                    .tx_mono_fst_vld()
                    .set_bit()
                    .tx_stop_en()
                    .set_bit()
                    .tx_chan_equal()
                    .clear_bit()
                    .tx_tdm_en()
                    .set_bit()
                    .tx_pdm_en()
                    .clear_bit()
                    .tx_pcm_bypass()
                    .set_bit()
                    .tx_big_endian()
                    .clear_bit()
                    .tx_bit_order()
                    .clear_bit()
                    .tx_chan_mod()
                    .variant(0)
            });

            i2s.tx_tdm_ctrl().modify(|_, w| {
                w.tx_tdm_tot_chan_num()
                    .variant(1)
                    .tx_tdm_chan0_en()
                    .set_bit()
                    .tx_tdm_chan1_en()
                    .set_bit()
                    .tx_tdm_chan2_en()
                    .clear_bit()
                    .tx_tdm_chan3_en()
                    .clear_bit()
                    .tx_tdm_chan4_en()
                    .clear_bit()
                    .tx_tdm_chan5_en()
                    .clear_bit()
                    .tx_tdm_chan6_en()
                    .clear_bit()
                    .tx_tdm_chan7_en()
                    .clear_bit()
                    .tx_tdm_chan8_en()
                    .clear_bit()
                    .tx_tdm_chan9_en()
                    .clear_bit()
                    .tx_tdm_chan10_en()
                    .clear_bit()
                    .tx_tdm_chan11_en()
                    .clear_bit()
                    .tx_tdm_chan12_en()
                    .clear_bit()
                    .tx_tdm_chan13_en()
                    .clear_bit()
                    .tx_tdm_chan14_en()
                    .clear_bit()
                    .tx_tdm_chan15_en()
                    .clear_bit()
            });

            #[allow(clippy::useless_conversion)]
            i2s.rx_conf1().modify(|_, w| {
                w.rx_tdm_ws_width()
                    .variant((data_format.channel_bits() - 1).into())
                    .rx_bits_mod()
                    .variant(data_format.data_bits() - 1)
                    .rx_tdm_chan_bits()
                    .variant(data_format.channel_bits() - 1)
                    .rx_half_sample_bits()
                    .variant(data_format.channel_bits() - 1)
            });
            #[cfg(not(esp32h2))]
            i2s.rx_conf1().modify(|_, w| w.rx_msb_shift().set_bit());
            #[cfg(esp32h2)]
            i2s.rx_conf().modify(|_, w| w.rx_msb_shift().set_bit());

            i2s.rx_conf().modify(|_, w| {
                w.rx_mono()
                    .clear_bit()
                    .rx_mono_fst_vld()
                    .set_bit()
                    .rx_stop_mode()
                    .variant(2)
                    .rx_tdm_en()
                    .set_bit()
                    .rx_pdm_en()
                    .clear_bit()
                    .rx_pcm_bypass()
                    .set_bit()
                    .rx_big_endian()
                    .clear_bit()
                    .rx_bit_order()
                    .clear_bit()
            });

            i2s.rx_tdm_ctrl().modify(|_, w| {
                w.rx_tdm_tot_chan_num()
                    .variant(1)
                    .rx_tdm_pdm_chan0_en()
                    .set_bit()
                    .rx_tdm_pdm_chan1_en()
                    .set_bit()
                    .rx_tdm_pdm_chan2_en()
                    .clear_bit()
                    .rx_tdm_pdm_chan3_en()
                    .clear_bit()
                    .rx_tdm_pdm_chan4_en()
                    .clear_bit()
                    .rx_tdm_pdm_chan5_en()
                    .clear_bit()
                    .rx_tdm_pdm_chan6_en()
                    .clear_bit()
                    .rx_tdm_pdm_chan7_en()
                    .clear_bit()
                    .rx_tdm_chan8_en()
                    .clear_bit()
                    .rx_tdm_chan9_en()
                    .clear_bit()
                    .rx_tdm_chan10_en()
                    .clear_bit()
                    .rx_tdm_chan11_en()
                    .clear_bit()
                    .rx_tdm_chan12_en()
                    .clear_bit()
                    .rx_tdm_chan13_en()
                    .clear_bit()
                    .rx_tdm_chan14_en()
                    .clear_bit()
                    .rx_tdm_chan15_en()
                    .clear_bit()
            });
        }

        fn set_master() {
            let i2s = Self::register_block();
            i2s.tx_conf().modify(|_, w| w.tx_slave_mod().clear_bit());
            i2s.rx_conf().modify(|_, w| w.rx_slave_mod().clear_bit());
        }

        fn update() {
            let i2s = Self::register_block();
            i2s.tx_conf().modify(|_, w| w.tx_update().clear_bit());
            i2s.tx_conf().modify(|_, w| w.tx_update().set_bit());

            i2s.rx_conf().modify(|_, w| w.rx_update().clear_bit());
            i2s.rx_conf().modify(|_, w| w.rx_update().set_bit());
        }

        fn reset_tx() {
            let i2s = Self::register_block();
            i2s.tx_conf()
                .modify(|_, w| w.tx_reset().set_bit().tx_fifo_reset().set_bit());
            i2s.tx_conf()
                .modify(|_, w| w.tx_reset().clear_bit().tx_fifo_reset().clear_bit());

            i2s.int_clr()
                .write(|w| w.tx_done().clear_bit_by_one().tx_hung().clear_bit_by_one());
        }

        fn tx_start() {
            let i2s = Self::register_block();
            i2s.tx_conf().modify(|_, w| w.tx_start().set_bit());
        }

        fn tx_stop() {
            let i2s = Self::register_block();
            i2s.tx_conf().modify(|_, w| w.tx_start().clear_bit());
        }

        fn wait_for_tx_done() {
            let i2s = Self::register_block();
            while i2s.state().read().tx_idle().bit_is_clear() {
                // wait
            }

            i2s.tx_conf().modify(|_, w| w.tx_start().clear_bit());
        }

        fn reset_rx() {
            let i2s = Self::register_block();
            i2s.rx_conf()
                .modify(|_, w| w.rx_reset().set_bit().rx_fifo_reset().set_bit());
            i2s.rx_conf()
                .modify(|_, w| w.rx_reset().clear_bit().rx_fifo_reset().clear_bit());

            i2s.int_clr()
                .write(|w| w.rx_done().clear_bit_by_one().rx_hung().clear_bit_by_one());
        }

        fn rx_start(len: usize) {
            let i2s = Self::register_block();
            i2s.rxeof_num()
                .write(|w| w.rx_eof_num().variant(len as u16));
            i2s.rx_conf().modify(|_, w| w.rx_start().set_bit());
        }

        fn wait_for_rx_done() {
            let i2s = Self::register_block();
            while i2s.int_raw().read().rx_done().bit_is_clear() {
                // wait
            }

            i2s.int_clr().write(|w| w.rx_done().clear_bit_by_one());
        }
    }

    #[cfg(any(esp32c3, esp32c6, esp32h2))]
    impl Signals for crate::peripherals::I2S0 {
        fn get_peripheral() -> Peripheral {
            Peripheral::I2s0
        }

        fn get_dma_peripheral() -> DmaPeripheral {
            DmaPeripheral::I2s0
        }

        fn mclk_signal() -> OutputSignal {
            OutputSignal::I2S_MCLK
        }

        fn bclk_signal() -> OutputSignal {
            OutputSignal::I2SO_BCK
        }

        fn ws_signal() -> OutputSignal {
            OutputSignal::I2SO_WS
        }

        fn dout_signal() -> OutputSignal {
            OutputSignal::I2SO_SD
        }

        fn bclk_rx_signal() -> OutputSignal {
            OutputSignal::I2SI_BCK
        }

        fn ws_rx_signal() -> OutputSignal {
            OutputSignal::I2SI_WS
        }

        fn din_signal() -> InputSignal {
            InputSignal::I2SI_SD
        }
    }

    #[cfg(esp32s3)]
    impl Signals for crate::peripherals::I2S0 {
        fn get_peripheral() -> Peripheral {
            Peripheral::I2s0
        }

        fn get_dma_peripheral() -> DmaPeripheral {
            DmaPeripheral::I2s0
        }

        fn mclk_signal() -> OutputSignal {
            OutputSignal::I2S0_MCLK
        }

        fn bclk_signal() -> OutputSignal {
            OutputSignal::I2S0O_BCK
        }

        fn ws_signal() -> OutputSignal {
            OutputSignal::I2S0O_WS
        }

        fn dout_signal() -> OutputSignal {
            OutputSignal::I2S0O_SD
        }

        fn bclk_rx_signal() -> OutputSignal {
            OutputSignal::I2S0I_BCK
        }

        fn ws_rx_signal() -> OutputSignal {
            OutputSignal::I2S0I_WS
        }

        fn din_signal() -> InputSignal {
            InputSignal::I2S0I_SD
        }
    }

    #[cfg(esp32s3)]
    impl Signals for crate::peripherals::I2S1 {
        fn get_peripheral() -> Peripheral {
            Peripheral::I2s1
        }

        fn get_dma_peripheral() -> DmaPeripheral {
            DmaPeripheral::I2s1
        }

        fn mclk_signal() -> OutputSignal {
            OutputSignal::I2S1_MCLK
        }

        fn bclk_signal() -> OutputSignal {
            OutputSignal::I2S1O_BCK
        }

        fn ws_signal() -> OutputSignal {
            OutputSignal::I2S1O_WS
        }

        fn dout_signal() -> OutputSignal {
            OutputSignal::I2S1O_SD
        }

        fn bclk_rx_signal() -> OutputSignal {
            OutputSignal::I2S1I_BCK
        }

        fn ws_rx_signal() -> OutputSignal {
            OutputSignal::I2S1I_WS
        }

        fn din_signal() -> InputSignal {
            InputSignal::I2S1I_SD
        }
    }

    #[cfg(esp32)]
    impl Signals for crate::peripherals::I2S0 {
        fn get_peripheral() -> Peripheral {
            Peripheral::I2s0
        }

        fn get_dma_peripheral() -> DmaPeripheral {
            DmaPeripheral::I2s0
        }

        fn mclk_signal() -> OutputSignal {
            panic!("MCLK currently not supported on ESP32");
        }

        fn bclk_signal() -> OutputSignal {
            OutputSignal::I2S0O_BCK
        }

        fn ws_signal() -> OutputSignal {
            OutputSignal::I2S0O_WS
        }

        fn dout_signal() -> OutputSignal {
            OutputSignal::I2S0O_DATA_23
        }

        fn bclk_rx_signal() -> OutputSignal {
            OutputSignal::I2S0I_BCK
        }

        fn ws_rx_signal() -> OutputSignal {
            OutputSignal::I2S0I_WS
        }

        fn din_signal() -> InputSignal {
            InputSignal::I2S0I_DATA_15
        }
    }

    #[cfg(esp32)]
    impl Signals for crate::peripherals::I2S1 {
        fn get_peripheral() -> Peripheral {
            Peripheral::I2s1
        }

        fn get_dma_peripheral() -> DmaPeripheral {
            DmaPeripheral::I2s1
        }

        fn mclk_signal() -> OutputSignal {
            panic!("MCLK currently not supported on ESP32");
        }

        fn bclk_signal() -> OutputSignal {
            OutputSignal::I2S1O_BCK
        }

        fn ws_signal() -> OutputSignal {
            OutputSignal::I2S1O_WS
        }

        fn dout_signal() -> OutputSignal {
            OutputSignal::I2S1O_DATA_23
        }

        fn bclk_rx_signal() -> OutputSignal {
            OutputSignal::I2S1I_BCK
        }

        fn ws_rx_signal() -> OutputSignal {
            OutputSignal::I2S1I_WS
        }

        fn din_signal() -> InputSignal {
            InputSignal::I2S1I_DATA_15
        }
    }

    #[cfg(esp32s2)]
    impl Signals for crate::peripherals::I2S0 {
        fn get_peripheral() -> Peripheral {
            Peripheral::I2s0
        }

        fn get_dma_peripheral() -> DmaPeripheral {
            DmaPeripheral::I2s0
        }

        fn mclk_signal() -> OutputSignal {
            OutputSignal::CLK_I2S
        }

        fn bclk_signal() -> OutputSignal {
            OutputSignal::I2S0O_BCK
        }

        fn ws_signal() -> OutputSignal {
            OutputSignal::I2S0O_WS
        }

        fn dout_signal() -> OutputSignal {
            OutputSignal::I2S0O_DATA_OUT23
        }

        fn bclk_rx_signal() -> OutputSignal {
            OutputSignal::I2S0I_BCK
        }

        fn ws_rx_signal() -> OutputSignal {
            OutputSignal::I2S0I_WS
        }

        fn din_signal() -> InputSignal {
            InputSignal::I2S0I_DATA_IN15
        }
    }

    impl RegBlock for I2S0 {
        fn register_block() -> &'static RegisterBlock {
            unsafe { &*I2S0::PTR.cast::<RegisterBlock>() }
        }
    }

    #[cfg(any(esp32s3, esp32))]
    impl RegBlock for I2S1 {
        fn register_block() -> &'static RegisterBlock {
            unsafe { &*I2S1::PTR.cast::<RegisterBlock>() }
        }
    }

    impl RegisterAccessPrivate for I2S0 {
        fn set_interrupt_handler(handler: InterruptHandler) {
            unsafe { crate::peripherals::I2S0::steal() }.bind_i2s0_interrupt(handler.handler());
            crate::interrupt::enable(crate::peripherals::Interrupt::I2S0, handler.priority())
                .unwrap();
        }
    }
    impl super::RegisterAccess for I2S0 {}

    #[cfg(any(esp32s3, esp32))]
    impl RegisterAccessPrivate for I2S1 {
        fn set_interrupt_handler(handler: InterruptHandler) {
            unsafe { crate::peripherals::I2S1::steal() }.bind_i2s1_interrupt(handler.handler());
            crate::interrupt::enable(crate::peripherals::Interrupt::I2S1, handler.priority())
                .unwrap();
        }
    }
    #[cfg(any(esp32s3, esp32))]
    impl super::RegisterAccess for I2S1 {}

    pub trait I2s0Instance {}

    #[cfg(any(esp32s3, esp32))]
    pub trait I2s1Instance {}

    impl I2s0Instance for I2S0 {}

    #[cfg(any(esp32s3, esp32))]
    impl I2s1Instance for I2S1 {}

    pub struct I2sClockDividers {
        mclk_divider: u32,
        bclk_divider: u32,
        denominator: u32,
        numerator: u32,
    }

    pub fn calculate_clock(
        sample_rate: impl Into<fugit::HertzU32>,
        channels: u8,
        data_bits: u8,
        _clocks: &Clocks,
    ) -> I2sClockDividers {
        // this loosely corresponds to `i2s_std_calculate_clock` and
        // `i2s_ll_tx_set_mclk` in esp-idf
        //
        // main difference is we are using fixed-point arithmetic here

        // If data_bits is a power of two, use 256 as the mclk_multiple
        // If data_bits is 24, use 192 (24 * 8) as the mclk_multiple
        let mclk_multiple = if data_bits == 24 { 192 } else { 256 };
        let sclk = crate::soc::constants::I2S_SCLK; // for now it's fixed 160MHz and 96MHz (just H2)

        let rate_hz: HertzU32 = sample_rate.into();
        let rate = rate_hz.raw();

        let bclk = rate * channels as u32 * data_bits as u32;
        let mclk = rate * mclk_multiple;
        let bclk_divider = mclk / bclk;
        let mut mclk_divider = sclk / mclk;

        let mut ma: u32;
        let mut mb: u32;
        let mut denominator: u32 = 0;
        let mut numerator: u32 = 0;

        let freq_diff = sclk.abs_diff(mclk * mclk_divider);

        if freq_diff != 0 {
            let decimal = freq_diff as u64 * 10000 / mclk as u64;

            // Carry bit if the decimal is greater than 1.0 - 1.0 / (63.0 * 2) = 125.0 /
            // 126.0
            if decimal > 1250000 / 126 {
                mclk_divider += 1;
            } else {
                let mut min: u32 = !0;

                for a in 2..=I2S_LL_MCLK_DIVIDER_MAX {
                    let b = (a as u64) * (freq_diff as u64 * 10000u64 / mclk as u64) + 5000;
                    ma = ((freq_diff as u64 * 10000u64 * a as u64) / 10000) as u32;
                    mb = (mclk as u64 * (b / 10000)) as u32;

                    if ma == mb {
                        denominator = a as u32;
                        numerator = (b / 10000) as u32;
                        break;
                    }

                    if mb.abs_diff(ma) < min {
                        denominator = a as u32;
                        numerator = b as u32;
                        min = mb.abs_diff(ma);
                    }
                }
            }
        }

        I2sClockDividers {
            mclk_divider,
            bclk_divider,
            denominator,
            numerator,
        }
    }
}

#[cfg(feature = "async")]
pub mod asynch {
    use embedded_dma::{ReadBuffer, WriteBuffer};

    use super::{Error, I2sRx, I2sTx, RegisterAccess};
    use crate::{
        dma::{
            asynch::{DmaRxDoneChFuture, DmaRxFuture, DmaTxDoneChFuture, DmaTxFuture},
            ChannelTypes,
            RxPrivate,
            TxPrivate,
        },
        Async,
    };

    /// Initiate an async DMA tx transfer
    pub trait I2sWriteDmaAsync<'d, T, CH>
    where
        T: RegisterAccess,
        CH: ChannelTypes,
    {
        /// One-shot write I2S.
        async fn write_dma_async(&mut self, words: &mut [u8]) -> Result<(), Error>;

        /// Continuously write to I2S. Returns [I2sWriteDmaTransferAsync]
        fn write_dma_circular_async<TXBUF>(
            self,
            words: TXBUF,
        ) -> Result<I2sWriteDmaTransferAsync<'d, T, CH, TXBUF>, Error>
        where
            TXBUF: ReadBuffer<Word = u8>;
    }

    impl<'d, T, CH> I2sWriteDmaAsync<'d, T, CH> for super::I2sTx<'d, T, CH, Async>
    where
        T: RegisterAccess,
        CH: ChannelTypes,
    {
        async fn write_dma_async(&mut self, words: &mut [u8]) -> Result<(), Error> {
            let (ptr, len) = (words.as_ptr(), words.len());

            self.tx_channel.listen_eof();

            self.start_tx_transfer_async(ptr, len, false)?;

            DmaTxFuture::new(&mut self.tx_channel).await;

            T::reset_tx();

            Ok(())
        }

        fn write_dma_circular_async<TXBUF>(
            mut self,
            words: TXBUF,
        ) -> Result<I2sWriteDmaTransferAsync<'d, T, CH, TXBUF>, Error>
        where
            TXBUF: ReadBuffer<Word = u8>,
        {
            let (ptr, len) = unsafe { words.read_buffer() };
            self.start_tx_transfer_async(ptr, len, true)?;

            Ok(I2sWriteDmaTransferAsync {
                i2s_tx: self,
                _buffer: words,
            })
        }
    }

    /// An in-progress async circular DMA write transfer.
    #[non_exhaustive]

    pub struct I2sWriteDmaTransferAsync<'d, T, CH, BUFFER>
    where
        T: RegisterAccess,
        CH: ChannelTypes,
    {
        i2s_tx: I2sTx<'d, T, CH, Async>,
        _buffer: BUFFER,
    }

    impl<'d, T, CH, BUFFER> I2sWriteDmaTransferAsync<'d, T, CH, BUFFER>
    where
        T: RegisterAccess,
        CH: ChannelTypes,
    {
        /// How many bytes can be pushed into the DMA transaction.
        /// Will wait for more than 0 bytes available.
        pub async fn available(&mut self) -> usize {
            loop {
                let res = self.i2s_tx.tx_channel.available();

                if res != 0 {
                    break res;
                }

                let future = DmaTxDoneChFuture::new(&mut self.i2s_tx.tx_channel);
                future.await;
            }
        }

        /// Push bytes into the DMA transaction.
        pub async fn push(&mut self, data: &[u8]) -> Result<usize, Error> {
            let avail = self.available().await;
            let to_send = usize::min(avail, data.len());
            Ok(self.i2s_tx.tx_channel.push(&data[..to_send])?)
        }

        /// Push bytes into the DMA buffer via the given closure.
        /// The closure *must* return the actual number of bytes written.
        /// The closure *might* get called with a slice which is smaller than
        /// the total available buffer. Only useful for circular DMA
        /// transfers
        pub async fn push_with(
            &mut self,
            f: impl FnOnce(&mut [u8]) -> usize,
        ) -> Result<usize, Error> {
            let _avail = self.available().await;
            Ok(self.i2s_tx.tx_channel.push_with(f)?)
        }
    }

    /// Initiate an async DMA rx transfer
    pub trait I2sReadDmaAsync<'d, T, CH>
    where
        T: RegisterAccess,
        CH: ChannelTypes,
    {
        /// One-shot read I2S.
        async fn read_dma_async(&mut self, words: &mut [u8]) -> Result<(), Error>;

        /// Continuously read frm I2S. Returns [I2sReadDmaTransferAsync]
        fn read_dma_circular_async<RXBUF>(
            self,
            words: RXBUF,
        ) -> Result<I2sReadDmaTransferAsync<'d, T, CH, RXBUF>, Error>
        where
            RXBUF: WriteBuffer<Word = u8>;
    }

    impl<'d, T, CH> I2sReadDmaAsync<'d, T, CH> for super::I2sRx<'d, T, CH, Async>
    where
        T: RegisterAccess,
        CH: ChannelTypes,
    {
        async fn read_dma_async(&mut self, words: &mut [u8]) -> Result<(), Error> {
            let (ptr, len) = (words.as_mut_ptr(), words.len());

            self.rx_channel.listen_eof();

            self.start_rx_transfer_async(ptr, len, false)?;

            DmaRxFuture::new(&mut self.rx_channel).await;

            // ??? T::reset_tx();

            Ok(())
        }

        fn read_dma_circular_async<RXBUF>(
            mut self,
            mut words: RXBUF,
        ) -> Result<I2sReadDmaTransferAsync<'d, T, CH, RXBUF>, Error>
        where
            RXBUF: WriteBuffer<Word = u8>,
        {
            let (ptr, len) = unsafe { words.write_buffer() };
            self.start_rx_transfer_async(ptr, len, true)?;

            Ok(I2sReadDmaTransferAsync {
                i2s_rx: self,
                _buffer: words,
            })
        }
    }

    /// An in-progress async circular DMA read transfer.
    #[non_exhaustive]

    pub struct I2sReadDmaTransferAsync<'d, T, CH, BUFFER>
    where
        T: RegisterAccess,
        CH: ChannelTypes,
    {
        i2s_rx: I2sRx<'d, T, CH, Async>,
        _buffer: BUFFER,
    }

    impl<'d, T, CH, BUFFER> I2sReadDmaTransferAsync<'d, T, CH, BUFFER>
    where
        T: RegisterAccess,
        CH: ChannelTypes,
    {
        /// How many bytes can be popped from the DMA transaction.
        /// Will wait for more than 0 bytes available.
        pub async fn available(&mut self) -> usize {
            loop {
                let res = self.i2s_rx.rx_channel.available();

                if res != 0 {
                    break res;
                }

                let future = DmaRxDoneChFuture::new(&mut self.i2s_rx.rx_channel);
                future.await;
            }
        }

        /// Pop bytes from the DMA transaction.
        pub async fn pop(&mut self, data: &mut [u8]) -> Result<usize, Error> {
            let avail = self.available().await;
            let to_rcv = usize::min(avail, data.len());
            Ok(self.i2s_rx.rx_channel.pop(&mut data[..to_rcv])?)
        }
    }
}
