//! # Inter-IC Sound (I2S)
//!
//! ## Overview
//!
//! I2S (Inter-IC Sound) is a synchronous serial communication protocol usually
//! used for transmitting audio data between two digital audio devices.
//! Espressif devices may contain more than one I2S peripheral(s). These
//! peripherals can be configured to input and output sample data via the I2S
//! driver.
//!
//! ## Configuration
//!
//! I2S supports different data formats, including varying data and channel
//! widths, different standards, such as the Philips standard and configurable
//! pin mappings for I2S clock (BCLK), word select (WS), and data input/output
//! (DOUT/DIN).
//!
//! The driver uses DMA (Direct Memory Access) for efficient data transfer and
//! supports various configurations, such as different data formats, standards
//! (e.g., Philips) and pin configurations. It relies on other peripheral
//! modules, such as
//!   - `GPIO`
//!   - `DMA`
//!   - `system` (to configure and enable the I2S peripheral)
//!
//! ## Example
//!
//! ### Initialization
//!
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! # use esp_hal::i2s::I2s;
//! # use esp_hal::i2s::Standard;
//! # use esp_hal::i2s::DataFormat;
//! # use esp_hal::gpio::Io;
//! # use esp_hal::dma_buffers;
//! # use esp_hal::dma::{Dma, DmaPriority};
//! # let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
//! let dma = Dma::new(peripherals.DMA);
#![cfg_attr(any(esp32, esp32s2), doc = "let dma_channel = dma.i2s0channel;")]
#![cfg_attr(not(any(esp32, esp32s2)), doc = "let dma_channel = dma.channel0;")]
//! let (mut rx_buffer, rx_descriptors, _, tx_descriptors) =
//! dma_buffers!(0, 4 * 4092);
//!
//! let i2s = I2s::new(
//!     peripherals.I2S0,
//!     Standard::Philips,
//!     DataFormat::Data16Channel16,
//!     44100.Hz(),
//!     dma_channel.configure(
//!         false,
//!         DmaPriority::Priority0,
//!     ),
//!     rx_descriptors,
//!     tx_descriptors,
//! );
#![cfg_attr(not(esp32), doc = "let i2s = i2s.with_mclk(io.pins.gpio0);")]
//! let mut i2s_rx = i2s.i2s_rx
//!     .with_bclk(io.pins.gpio1)
//!     .with_ws(io.pins.gpio2)
//!     .with_din(io.pins.gpio5)
//!     .build();
//!
//! let mut transfer = i2s_rx.read_dma_circular(&mut rx_buffer).unwrap();
//!
//! loop {
//!     let avail = transfer.available();
//!
//!     if avail > 0 {
//!         let mut rcv = [0u8; 5000];
//!         transfer.pop(&mut rcv[..avail]).unwrap();
//!     }
//! }
//! # }
//! ```
//! 
//! ## Implementation State
//! - Only master mode is supported.
//! - Only TDM Philips standard is supported.

use core::marker::PhantomData;

use enumset::{EnumSet, EnumSetType};
use private::*;

use crate::{
    dma::{
        dma_private::{DmaSupport, DmaSupportRx, DmaSupportTx},
        Channel,
        ChannelRx,
        ChannelTx,
        DescriptorChain,
        DmaChannelConvert,
        DmaDescriptor,
        DmaEligible,
        DmaError,
        DmaTransferRx,
        DmaTransferRxCircular,
        DmaTransferTx,
        DmaTransferTxCircular,
        ReadBuffer,
        Rx,
        Tx,
        WriteBuffer,
    },
    gpio::interconnect::PeripheralOutput,
    interrupt::InterruptHandler,
    peripheral::{Peripheral, PeripheralRef},
    system::PeripheralClockControl,
    InterruptConfigurable,
    Mode,
};

#[derive(EnumSetType)]
/// Represents the various interrupt types for the I2S peripheral.
pub enum I2sInterrupt {
    /// Receive buffer hung, indicating a stall in data reception.
    RxHung,
    /// Transmit buffer hung, indicating a stall in data transmission.
    TxHung,
    #[cfg(not(any(esp32, esp32s2)))]
    /// Reception of data is complete.
    RxDone,
    #[cfg(not(any(esp32, esp32s2)))]
    /// Transmission of data is complete.
    TxDone,
}

#[cfg(any(esp32, esp32s2, esp32s3))]
const I2S_LL_MCLK_DIVIDER_BIT_WIDTH: usize = 6;

#[cfg(any(esp32c3, esp32c6, esp32h2))]
const I2S_LL_MCLK_DIVIDER_BIT_WIDTH: usize = 9;

const I2S_LL_MCLK_DIVIDER_MAX: usize = (1 << I2S_LL_MCLK_DIVIDER_BIT_WIDTH) - 1;

/// Data types that the I2S peripheral can work with.
pub trait AcceptedWord: crate::private::Sealed {}
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
    /// An unspecified or unknown error occurred during an I2S operation.
    Unknown,
    /// A DMA-related error occurred during I2S operations.
    DmaError(DmaError),
    /// An illegal or invalid argument was passed to an I2S function or method.
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
    /// The Philips I2S standard.
    Philips,
    // Tdm,
    // Pdm,
}

/// Supported data formats
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg(not(any(esp32, esp32s2)))]
pub enum DataFormat {
    /// 32-bit data width and 32-bit channel width.
    Data32Channel32,
    /// 32-bit data width and 24-bit channel width.
    Data32Channel24,
    /// 32-bit data width and 16-bit channel width.
    Data32Channel16,
    /// 32-bit data width and 8-bit channel width.
    Data32Channel8,
    /// 16-bit data width and 16-bit channel width.
    Data16Channel16,
    /// 16-bit data width and 8-bit channel width.
    Data16Channel8,
    /// 8-bit data width and 8-bit channel width.
    Data8Channel8,
}

/// Supported data formats
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg(any(esp32, esp32s2))]
pub enum DataFormat {
    /// 32-bit data width and 32-bit channel width.
    Data32Channel32,
    /// 16-bit data width and 16-bit channel width.
    Data16Channel16,
}

#[cfg(not(any(esp32, esp32s2)))]
impl DataFormat {
    /// Returns the number of data bits for the selected data format.
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

    /// Returns the number of channel bits for the selected data format.
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
    /// Returns the number of data bits for the selected data format.
    pub fn data_bits(&self) -> u8 {
        match self {
            DataFormat::Data32Channel32 => 32,
            DataFormat::Data16Channel16 => 16,
        }
    }

    /// Returns the number of channel bits for the selected data format.
    pub fn channel_bits(&self) -> u8 {
        match self {
            DataFormat::Data32Channel32 => 32,
            DataFormat::Data16Channel16 => 16,
        }
    }
}

/// Instance of the I2S peripheral driver
pub struct I2s<'d, DmaMode, T = AnyI2s>
where
    T: RegisterAccess,
    DmaMode: Mode,
{
    /// Handles the reception (RX) side of the I2S peripheral.
    pub i2s_rx: RxCreator<'d, DmaMode, T>,
    /// Handles the transmission (TX) side of the I2S peripheral.
    pub i2s_tx: TxCreator<'d, DmaMode, T>,
    phantom: PhantomData<DmaMode>,
}

impl<'d, DmaMode, T> I2s<'d, DmaMode, T>
where
    T: RegisterAccess,
    DmaMode: Mode,
{
    #[allow(clippy::too_many_arguments)]
    fn new_internal<CH>(
        i2s: PeripheralRef<'d, T>,
        standard: Standard,
        data_format: DataFormat,
        sample_rate: impl Into<fugit::HertzU32>,
        channel: Channel<'d, CH, DmaMode>,
        rx_descriptors: &'static mut [DmaDescriptor],
        tx_descriptors: &'static mut [DmaDescriptor],
    ) -> Self
    where
        CH: DmaChannelConvert<T::Dma>,
    {
        channel.runtime_ensure_compatible(&i2s);
        // on ESP32-C3 / ESP32-S3 and later RX and TX are independent and
        // could be configured totally independently but for now handle all
        // the targets the same and force same configuration for both, TX and RX

        PeripheralClockControl::reset(i2s.peripheral());
        PeripheralClockControl::enable(i2s.peripheral());
        i2s.set_clock(calculate_clock(sample_rate, 2, data_format.channel_bits()));
        i2s.configure(&standard, &data_format);
        i2s.set_master();
        i2s.update();

        let channel = channel.degrade();
        Self {
            i2s_rx: RxCreator {
                i2s: unsafe { i2s.clone_unchecked() },
                rx_channel: channel.rx,
                descriptors: rx_descriptors,
                phantom: PhantomData,
            },
            i2s_tx: TxCreator {
                i2s,
                tx_channel: channel.tx,
                descriptors: tx_descriptors,
                phantom: PhantomData,
            },
            phantom: PhantomData,
        }
    }
}

impl<'d, DmaMode, T> I2s<'d, DmaMode, T>
where
    T: RegisterAccess,
    DmaMode: Mode,
{
    /// Sets the interrupt handler
    ///
    /// Interrupts are not enabled at the peripheral level here.
    pub fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        // tx.i2s and rx.i2s is the same, we could use either one
        self.i2s_tx.i2s.set_interrupt_handler(handler);
    }

    /// Listen for the given interrupts
    pub fn listen(&mut self, interrupts: EnumSet<I2sInterrupt>) {
        // tx.i2s and rx.i2s is the same, we could use either one
        self.i2s_tx.i2s.listen(interrupts);
    }

    /// Unlisten the given interrupts
    pub fn unlisten(&mut self, interrupts: EnumSet<I2sInterrupt>) {
        // tx.i2s and rx.i2s is the same, we could use either one
        self.i2s_tx.i2s.unlisten(interrupts);
    }

    /// Gets asserted interrupts
    pub fn interrupts(&mut self) -> EnumSet<I2sInterrupt> {
        // tx.i2s and rx.i2s is the same, we could use either one
        self.i2s_tx.i2s.interrupts()
    }

    /// Resets asserted interrupts
    pub fn clear_interrupts(&mut self, interrupts: EnumSet<I2sInterrupt>) {
        // tx.i2s and rx.i2s is the same, we could use either one
        self.i2s_tx.i2s.clear_interrupts(interrupts);
    }
}

impl<'d, DmaMode, I> crate::private::Sealed for I2s<'d, DmaMode, I>
where
    I: RegisterAccess,
    DmaMode: Mode,
{
}

impl<'d, DmaMode, I> InterruptConfigurable for I2s<'d, DmaMode, I>
where
    I: RegisterAccess,
    DmaMode: Mode,
{
    fn set_interrupt_handler(&mut self, handler: crate::interrupt::InterruptHandler) {
        I2s::set_interrupt_handler(self, handler);
    }
}

impl<'d, DmaMode> I2s<'d, DmaMode>
where
    DmaMode: Mode,
{
    /// Construct a new I2S peripheral driver instance for the first I2S
    /// peripheral
    #[allow(clippy::too_many_arguments)]
    pub fn new<CH>(
        i2s: impl Peripheral<P = impl RegisterAccess> + 'd,
        standard: Standard,
        data_format: DataFormat,
        sample_rate: impl Into<fugit::HertzU32>,
        channel: Channel<'d, CH, DmaMode>,
        rx_descriptors: &'static mut [DmaDescriptor],
        tx_descriptors: &'static mut [DmaDescriptor],
    ) -> Self
    where
        CH: DmaChannelConvert<<AnyI2s as DmaEligible>::Dma>,
    {
        Self::new_typed(
            i2s.map_into(),
            standard,
            data_format,
            sample_rate,
            channel,
            rx_descriptors,
            tx_descriptors,
        )
    }
}

impl<'d, DmaMode, T> I2s<'d, DmaMode, T>
where
    T: RegisterAccess,
    DmaMode: Mode,
{
    /// Construct a new I2S peripheral driver instance for the first I2S
    /// peripheral
    #[allow(clippy::too_many_arguments)]
    pub fn new_typed<CH>(
        i2s: impl Peripheral<P = T> + 'd,
        standard: Standard,
        data_format: DataFormat,
        sample_rate: impl Into<fugit::HertzU32>,
        channel: Channel<'d, CH, DmaMode>,
        rx_descriptors: &'static mut [DmaDescriptor],
        tx_descriptors: &'static mut [DmaDescriptor],
    ) -> Self
    where
        CH: DmaChannelConvert<T::Dma>,
    {
        crate::into_ref!(i2s);
        Self::new_internal(
            i2s,
            standard,
            data_format,
            sample_rate,
            channel,
            rx_descriptors,
            tx_descriptors,
        )
    }

    /// Configures the I2S peripheral to use a master clock (MCLK) output pin.
    pub fn with_mclk<P: PeripheralOutput>(self, pin: impl Peripheral<P = P> + 'd) -> Self {
        crate::into_mapped_ref!(pin);
        pin.set_to_push_pull_output(crate::private::Internal);
        pin.connect_peripheral_to_output(self.i2s_tx.i2s.mclk_signal(), crate::private::Internal);

        self
    }
}

/// I2S TX channel
pub struct I2sTx<'d, DmaMode, T = AnyI2s>
where
    T: RegisterAccess,
{
    i2s: PeripheralRef<'d, T>,
    tx_channel: ChannelTx<'d, T::Dma>,
    tx_chain: DescriptorChain,
    phantom: PhantomData<DmaMode>,
}

impl<'d, DmaMode, T> core::fmt::Debug for I2sTx<'d, DmaMode, T>
where
    T: RegisterAccess,
    DmaMode: Mode,
{
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("I2sTx").finish()
    }
}

impl<'d, DmaMode, T> DmaSupport for I2sTx<'d, DmaMode, T>
where
    T: RegisterAccess,
    DmaMode: Mode,
{
    fn peripheral_wait_dma(&mut self, _is_rx: bool, _is_tx: bool) {
        self.i2s.wait_for_tx_done();
    }

    fn peripheral_dma_stop(&mut self) {
        self.i2s.tx_stop();
    }
}

impl<'d, DmaMode, T> DmaSupportTx for I2sTx<'d, DmaMode, T>
where
    T: RegisterAccess,
    DmaMode: Mode,
{
    type TX = ChannelTx<'d, T::Dma>;

    fn tx(&mut self) -> &mut Self::TX {
        &mut self.tx_channel
    }

    fn chain(&mut self) -> &mut DescriptorChain {
        &mut self.tx_chain
    }
}

impl<'d, DmaMode, T> I2sTx<'d, DmaMode, T>
where
    T: RegisterAccess,
    DmaMode: Mode,
{
    fn write_bytes(&mut self, data: &[u8]) -> Result<(), Error> {
        self.start_tx_transfer(&data, false)?;

        // wait until I2S_TX_IDLE is 1
        self.i2s.wait_for_tx_done();

        Ok(())
    }

    fn start_tx_transfer<'t, TXBUF>(
        &'t mut self,
        words: &'t TXBUF,
        circular: bool,
    ) -> Result<(), Error>
    where
        TXBUF: ReadBuffer,
        DmaMode: Mode,
    {
        let (ptr, len) = unsafe { words.read_buffer() };

        // Reset TX unit and TX FIFO
        self.i2s.reset_tx();

        // Enable corresponding interrupts if needed

        // configure DMA outlink
        unsafe {
            self.tx_chain.fill_for_tx(circular, ptr, len)?;
            self.tx_channel
                .prepare_transfer_without_start(self.i2s.dma_peripheral(), &self.tx_chain)
                .and_then(|_| self.tx_channel.start_transfer())?;
        }

        // set I2S_TX_STOP_EN if needed

        // start: set I2S_TX_START
        self.i2s.tx_start();

        Ok(())
    }

    /// Writes a slice of data to the I2S peripheral.
    pub fn write(&mut self, words: &[impl AcceptedWord]) -> Result<(), Error> {
        self.write_bytes(unsafe {
            core::slice::from_raw_parts(words.as_ptr().cast::<u8>(), core::mem::size_of_val(words))
        })
    }

    /// Write I2S.
    /// Returns [DmaTransferTx] which represents the in-progress DMA
    /// transfer
    pub fn write_dma<'t>(
        &'t mut self,
        words: &'t impl ReadBuffer,
    ) -> Result<DmaTransferTx<'t, Self>, Error>
    where
        Self: DmaSupportTx,
    {
        self.start_tx_transfer(words, false)?;
        Ok(DmaTransferTx::new(self))
    }

    /// Continuously write to I2S. Returns [DmaTransferTxCircular] which
    /// represents the in-progress DMA transfer
    pub fn write_dma_circular<'t>(
        &'t mut self,
        words: &'t impl ReadBuffer,
    ) -> Result<DmaTransferTxCircular<'t, Self>, Error>
    where
        Self: DmaSupportTx,
    {
        self.start_tx_transfer(words, true)?;
        Ok(DmaTransferTxCircular::new(self))
    }
}

/// I2S RX channel
pub struct I2sRx<'d, DmaMode, T = AnyI2s>
where
    T: RegisterAccess,
    DmaMode: Mode,
{
    i2s: PeripheralRef<'d, T>,
    rx_channel: ChannelRx<'d, T::Dma>,
    rx_chain: DescriptorChain,
    phantom: PhantomData<DmaMode>,
}

impl<'d, DmaMode, T> core::fmt::Debug for I2sRx<'d, DmaMode, T>
where
    T: RegisterAccess,
    DmaMode: Mode,
{
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("I2sRx").finish()
    }
}

impl<'d, DmaMode, T> DmaSupport for I2sRx<'d, DmaMode, T>
where
    T: RegisterAccess,
    DmaMode: Mode,
{
    fn peripheral_wait_dma(&mut self, _is_rx: bool, _is_tx: bool) {
        self.i2s.wait_for_rx_done();
    }

    fn peripheral_dma_stop(&mut self) {
        self.i2s.reset_rx();
    }
}

impl<'d, DmaMode, T> DmaSupportRx for I2sRx<'d, DmaMode, T>
where
    T: RegisterAccess,
    DmaMode: Mode,
{
    type RX = ChannelRx<'d, T::Dma>;

    fn rx(&mut self) -> &mut Self::RX {
        &mut self.rx_channel
    }

    fn chain(&mut self) -> &mut DescriptorChain {
        &mut self.rx_chain
    }
}

impl<'d, DmaMode, T> I2sRx<'d, DmaMode, T>
where
    T: RegisterAccess,
    DmaMode: Mode,
{
    fn read_bytes(&mut self, mut data: &mut [u8]) -> Result<(), Error> {
        self.start_rx_transfer(&mut data, false)?;

        // wait until I2S_RX_IDLE is 1
        self.i2s.wait_for_rx_done();

        Ok(())
    }

    fn start_rx_transfer<'t, RXBUF>(
        &'t mut self,
        words: &'t mut RXBUF,
        circular: bool,
    ) -> Result<(), Error>
    where
        RXBUF: WriteBuffer,
    {
        let (ptr, len) = unsafe { words.write_buffer() };

        if len % 4 != 0 {
            return Err(Error::IllegalArgument);
        }

        // Reset RX unit and RX FIFO
        self.i2s.reset_rx();

        // Enable corresponding interrupts if needed

        // configure DMA inlink
        unsafe {
            self.rx_chain.fill_for_rx(circular, ptr, len)?;
            self.rx_channel
                .prepare_transfer_without_start(self.i2s.dma_peripheral(), &self.rx_chain)
                .and_then(|_| self.rx_channel.start_transfer())?;
        }

        // start: set I2S_RX_START
        self.i2s.rx_start(len);
        Ok(())
    }

    /// Reads a slice of data from the I2S peripheral and stores it in the
    /// provided buffer.
    pub fn read(&mut self, words: &mut [impl AcceptedWord]) -> Result<(), Error> {
        if core::mem::size_of_val(words) > 4096 || words.is_empty() {
            return Err(Error::IllegalArgument);
        }

        self.read_bytes(unsafe {
            core::slice::from_raw_parts_mut(
                words.as_mut_ptr().cast::<u8>(),
                core::mem::size_of_val(words),
            )
        })
    }

    /// Read I2S.
    /// Returns [DmaTransferRx] which represents the in-progress DMA
    /// transfer
    pub fn read_dma<'t>(
        &'t mut self,
        words: &'t mut impl WriteBuffer,
    ) -> Result<DmaTransferRx<'t, Self>, Error>
    where
        Self: DmaSupportRx,
    {
        self.start_rx_transfer(words, false)?;
        Ok(DmaTransferRx::new(self))
    }

    /// Continuously read from I2S.
    /// Returns [DmaTransferRxCircular] which represents the in-progress DMA
    /// transfer
    pub fn read_dma_circular<'t>(
        &'t mut self,
        words: &'t mut impl WriteBuffer,
    ) -> Result<DmaTransferRxCircular<'t, Self>, Error>
    where
        Self: DmaSupportRx,
    {
        self.start_rx_transfer(words, true)?;
        Ok(DmaTransferRxCircular::new(self))
    }
}

/// Provides an abstraction for accessing the I2S peripheral registers.
pub trait RegisterAccess: RegisterAccessPrivate {}
impl<T> RegisterAccess for T where T: RegisterAccessPrivate {}

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
    #[cfg(not(i2s1))]
    use crate::peripherals::i2s0::RegisterBlock;
    // on ESP32-S3 I2S1 doesn't support all features - use that to avoid using those features
    // by accident
    #[cfg(i2s1)]
    use crate::peripherals::{i2s1::RegisterBlock, I2S1};
    use crate::{
        dma::{
            ChannelRx,
            ChannelTx,
            DescriptorChain,
            DmaDescriptor,
            DmaEligible,
            PeripheralMarker,
        },
        gpio::{
            interconnect::{PeripheralInput, PeripheralOutput},
            InputSignal,
            OutputSignal,
        },
        interrupt::InterruptHandler,
        peripheral::{Peripheral, PeripheralRef},
        peripherals::I2S0,
        private,
        Mode,
    };

    pub struct TxCreator<'d, DmaMode, T>
    where
        T: RegisterAccess,
        DmaMode: Mode,
    {
        pub i2s: PeripheralRef<'d, T>,
        pub tx_channel: ChannelTx<'d, T::Dma>,
        pub descriptors: &'static mut [DmaDescriptor],
        pub(crate) phantom: PhantomData<DmaMode>,
    }

    impl<'d, DmaMode, T> TxCreator<'d, DmaMode, T>
    where
        T: RegisterAccess,
        DmaMode: Mode,
    {
        pub fn build(self) -> I2sTx<'d, DmaMode, T> {
            I2sTx {
                i2s: self.i2s,
                tx_channel: self.tx_channel,
                tx_chain: DescriptorChain::new(self.descriptors),
                phantom: PhantomData,
            }
        }

        pub fn with_bclk<P>(self, pin: impl Peripheral<P = P> + 'd) -> Self
        where
            P: PeripheralOutput,
        {
            crate::into_mapped_ref!(pin);
            pin.set_to_push_pull_output(private::Internal);
            pin.connect_peripheral_to_output(self.i2s.bclk_signal(), private::Internal);

            self
        }

        pub fn with_ws<P>(self, pin: impl Peripheral<P = P> + 'd) -> Self
        where
            P: PeripheralOutput,
        {
            crate::into_mapped_ref!(pin);
            pin.set_to_push_pull_output(private::Internal);
            pin.connect_peripheral_to_output(self.i2s.ws_signal(), private::Internal);

            self
        }

        pub fn with_dout<P>(self, pin: impl Peripheral<P = P> + 'd) -> Self
        where
            P: PeripheralOutput,
        {
            crate::into_mapped_ref!(pin);
            pin.set_to_push_pull_output(private::Internal);
            pin.connect_peripheral_to_output(self.i2s.dout_signal(), private::Internal);

            self
        }
    }

    pub struct RxCreator<'d, DmaMode, T>
    where
        T: RegisterAccess,
        DmaMode: Mode,
    {
        pub i2s: PeripheralRef<'d, T>,
        pub rx_channel: ChannelRx<'d, T::Dma>,
        pub descriptors: &'static mut [DmaDescriptor],
        pub(crate) phantom: PhantomData<DmaMode>,
    }

    impl<'d, DmaMode, T> RxCreator<'d, DmaMode, T>
    where
        T: RegisterAccess,
        DmaMode: Mode,
    {
        pub fn build(self) -> I2sRx<'d, DmaMode, T> {
            I2sRx {
                i2s: self.i2s,
                rx_channel: self.rx_channel,
                rx_chain: DescriptorChain::new(self.descriptors),
                phantom: PhantomData,
            }
        }

        pub fn with_bclk<P>(self, pin: impl Peripheral<P = P> + 'd) -> Self
        where
            P: PeripheralOutput,
        {
            crate::into_mapped_ref!(pin);
            pin.set_to_push_pull_output(private::Internal);
            pin.connect_peripheral_to_output(self.i2s.bclk_rx_signal(), private::Internal);

            self
        }

        pub fn with_ws<P>(self, pin: impl Peripheral<P = P> + 'd) -> Self
        where
            P: PeripheralOutput,
        {
            crate::into_mapped_ref!(pin);
            pin.set_to_push_pull_output(private::Internal);
            pin.connect_peripheral_to_output(self.i2s.ws_rx_signal(), private::Internal);

            self
        }

        pub fn with_din<P>(self, pin: impl Peripheral<P = P> + 'd) -> Self
        where
            P: PeripheralInput,
        {
            crate::into_mapped_ref!(pin);
            pin.init_input(crate::gpio::Pull::None, private::Internal);
            pin.connect_input_to_peripheral(self.i2s.din_signal(), private::Internal);

            self
        }
    }

    pub trait RegBlock:
        Peripheral<P = Self> + PeripheralMarker + DmaEligible + Into<super::AnyI2s> + 'static
    {
        fn register_block(&self) -> &RegisterBlock;
    }

    pub trait Signals: RegBlock {
        fn mclk_signal(&self) -> OutputSignal;
        fn bclk_signal(&self) -> OutputSignal;
        fn ws_signal(&self) -> OutputSignal;
        fn dout_signal(&self) -> OutputSignal;
        fn bclk_rx_signal(&self) -> OutputSignal;
        fn ws_rx_signal(&self) -> OutputSignal;
        fn din_signal(&self) -> InputSignal;
    }

    #[cfg(any(esp32, esp32s2))]
    pub trait RegisterAccessPrivate: Signals + RegBlock {
        fn set_interrupt_handler(&self, handler: InterruptHandler);

        fn enable_listen(&self, interrupts: EnumSet<I2sInterrupt>, enable: bool) {
            let reg_block = self.register_block();

            reg_block.int_ena().modify(|_, w| {
                for interrupt in interrupts {
                    match interrupt {
                        I2sInterrupt::RxHung => w.rx_hung().bit(enable),
                        I2sInterrupt::TxHung => w.tx_hung().bit(enable),
                    };
                }
                w
            });
        }

        fn listen(&self, interrupts: impl Into<EnumSet<I2sInterrupt>>) {
            self.enable_listen(interrupts.into(), true);
        }

        fn unlisten(&self, interrupts: impl Into<EnumSet<I2sInterrupt>>) {
            self.enable_listen(interrupts.into(), false);
        }

        fn interrupts(&self) -> EnumSet<I2sInterrupt> {
            let mut res = EnumSet::new();
            let reg_block = self.register_block();
            let ints = reg_block.int_st().read();

            if ints.rx_hung().bit() {
                res.insert(I2sInterrupt::RxHung);
            }
            if ints.tx_hung().bit() {
                res.insert(I2sInterrupt::TxHung);
            }

            res
        }

        fn clear_interrupts(&self, interrupts: EnumSet<I2sInterrupt>) {
            let reg_block = self.register_block();

            reg_block.int_clr().write(|w| {
                for interrupt in interrupts {
                    match interrupt {
                        I2sInterrupt::RxHung => w.rx_hung().clear_bit_by_one(),
                        I2sInterrupt::TxHung => w.tx_hung().clear_bit_by_one(),
                    };
                }
                w
            });
        }

        fn set_clock(&self, clock_settings: I2sClockDividers) {
            let i2s = self.register_block();

            i2s.clkm_conf().modify(|r, w| unsafe {
                w.bits(r.bits() | (crate::soc::constants::I2S_DEFAULT_CLK_SRC << 21))
                // select PLL_160M
            });

            #[cfg(esp32)]
            i2s.clkm_conf().modify(|_, w| w.clka_ena().clear_bit());

            i2s.clkm_conf().modify(|_, w| unsafe {
                w.clk_en().set_bit();
                w.clkm_div_num().bits(clock_settings.mclk_divider as u8)
            });

            i2s.clkm_conf().modify(|_, w| unsafe {
                w.clkm_div_a().bits(clock_settings.denominator as u8);
                w.clkm_div_b().bits(clock_settings.numerator as u8)
            });

            i2s.sample_rate_conf().modify(|_, w| unsafe {
                w.tx_bck_div_num().bits(clock_settings.bclk_divider as u8);
                w.rx_bck_div_num().bits(clock_settings.bclk_divider as u8)
            });
        }

        fn configure(&self, _standard: &Standard, data_format: &DataFormat) {
            let i2s = self.register_block();

            let fifo_mod = match data_format {
                DataFormat::Data32Channel32 => 2,
                DataFormat::Data16Channel16 => 0,
            };

            i2s.sample_rate_conf()
                .modify(|_, w| unsafe { w.tx_bits_mod().bits(data_format.channel_bits()) });
            i2s.sample_rate_conf()
                .modify(|_, w| unsafe { w.rx_bits_mod().bits(data_format.channel_bits()) });

            i2s.conf().modify(|_, w| {
                w.tx_slave_mod().clear_bit();
                w.rx_slave_mod().clear_bit();
                // If the I2S_RX_MSB_SHIFT bit and the I2S_TX_MSB_SHIFT bit of register
                // I2S_CONF_REG are set to 1, respectively, the I2S module will use the Philips
                // standard when receiving and transmitting data.
                w.tx_msb_shift().set_bit();
                w.rx_msb_shift().set_bit();
                // Short frame synchronization
                w.tx_short_sync().bit(false);
                w.rx_short_sync().bit(false);
                // Send MSB to the right channel to be consistent with ESP32-S3 et al.
                w.tx_msb_right().set_bit();
                w.rx_msb_right().set_bit();
                // ESP32 generates two clock pulses first. If the WS is low, those first clock
                // pulses are indistinguishable from real data, which corrupts the first few
                // samples. So we send the right channel first (which means WS is high during
                // the first sample) to prevent this issue.
                w.tx_right_first().set_bit();
                w.rx_right_first().set_bit();
                w.tx_mono().clear_bit();
                w.rx_mono().clear_bit();
                w.sig_loopback().clear_bit()
            });

            i2s.fifo_conf().modify(|_, w| unsafe {
                w.tx_fifo_mod().bits(fifo_mod);
                w.tx_fifo_mod_force_en().set_bit();
                w.dscr_en().set_bit();
                w.rx_fifo_mod().bits(fifo_mod);
                w.rx_fifo_mod_force_en().set_bit()
            });

            i2s.conf_chan().modify(|_, w| unsafe {
                // for now only stereo
                w.tx_chan_mod().bits(0);
                w.rx_chan_mod().bits(0)
            });

            i2s.conf1().modify(|_, w| {
                w.tx_pcm_bypass().set_bit();
                w.rx_pcm_bypass().set_bit()
            });

            i2s.pd_conf().modify(|_, w| {
                w.fifo_force_pu().set_bit();
                w.fifo_force_pd().clear_bit()
            });

            i2s.conf2().modify(|_, w| {
                w.camera_en().clear_bit();
                w.lcd_en().clear_bit()
            });
        }

        fn set_master(&self) {
            let i2s = self.register_block();
            i2s.conf()
                .modify(|_, w| w.rx_slave_mod().clear_bit().tx_slave_mod().clear_bit());
        }

        fn update(&self) {
            // nothing to do
        }

        fn reset_tx(&self) {
            let i2s = self.register_block();
            i2s.conf().modify(|_, w| {
                w.tx_reset().set_bit();
                w.tx_fifo_reset().set_bit()
            });
            i2s.conf().modify(|_, w| {
                w.tx_reset().clear_bit();
                w.tx_fifo_reset().clear_bit()
            });

            i2s.lc_conf().modify(|_, w| w.out_rst().set_bit());
            i2s.lc_conf().modify(|_, w| w.out_rst().clear_bit());

            i2s.int_clr().write(|w| {
                w.out_done().clear_bit_by_one();
                w.out_total_eof().clear_bit_by_one()
            });
        }

        fn tx_start(&self) {
            let i2s = self.register_block();
            i2s.conf().modify(|_, w| w.tx_start().set_bit());

            while i2s.state().read().tx_idle().bit_is_set() {
                // wait
            }
        }

        fn tx_stop(&self) {
            let i2s = self.register_block();
            i2s.conf().modify(|_, w| w.tx_start().clear_bit());
        }

        fn wait_for_tx_done(&self) {
            let i2s = self.register_block();
            while i2s.state().read().tx_idle().bit_is_clear() {
                // wait
            }

            i2s.conf().modify(|_, w| w.tx_start().clear_bit());
        }

        fn reset_rx(&self) {
            let i2s = self.register_block();
            i2s.conf().modify(|_, w| {
                w.rx_reset().set_bit();
                w.rx_fifo_reset().set_bit()
            });
            i2s.conf().modify(|_, w| {
                w.rx_reset().clear_bit();
                w.rx_fifo_reset().clear_bit()
            });

            i2s.lc_conf().modify(|_, w| w.in_rst().set_bit());
            i2s.lc_conf().modify(|_, w| w.in_rst().clear_bit());

            i2s.int_clr().write(|w| {
                w.in_done().clear_bit_by_one();
                w.in_suc_eof().clear_bit_by_one()
            });
        }

        fn rx_start(&self, len: usize) {
            let i2s = self.register_block();

            i2s.int_clr().write(|w| w.in_suc_eof().clear_bit_by_one());

            cfg_if::cfg_if! {
                if #[cfg(esp32)] {
                    // On ESP32, the eof_num count in words.
                    let eof_num = len / 4;
                } else {
                    let eof_num = len - 1;
                }
            }

            i2s.rxeof_num()
                .modify(|_, w| unsafe { w.rx_eof_num().bits(eof_num as u32) });

            i2s.conf().modify(|_, w| w.rx_start().set_bit());
        }

        fn wait_for_rx_done(&self) {
            let i2s = self.register_block();
            while i2s.int_raw().read().in_suc_eof().bit_is_clear() {
                // wait
            }

            i2s.int_clr().write(|w| w.in_suc_eof().clear_bit_by_one());
        }
    }

    #[cfg(any(esp32c3, esp32c6, esp32h2, esp32s3))]
    pub trait RegisterAccessPrivate: Signals + RegBlock {
        fn set_interrupt_handler(&self, handler: InterruptHandler);

        fn enable_listen(&self, interrupts: EnumSet<I2sInterrupt>, enable: bool) {
            let reg_block = self.register_block();

            reg_block.int_ena().modify(|_, w| {
                for interrupt in interrupts {
                    match interrupt {
                        I2sInterrupt::RxHung => w.rx_hung().bit(enable),
                        I2sInterrupt::TxHung => w.tx_hung().bit(enable),
                        I2sInterrupt::RxDone => w.rx_done().bit(enable),
                        I2sInterrupt::TxDone => w.tx_done().bit(enable),
                    };
                }
                w
            });
        }

        fn listen(&self, interrupts: impl Into<EnumSet<I2sInterrupt>>) {
            self.enable_listen(interrupts.into(), true);
        }

        fn unlisten(&self, interrupts: impl Into<EnumSet<I2sInterrupt>>) {
            self.enable_listen(interrupts.into(), false);
        }

        fn interrupts(&self) -> EnumSet<I2sInterrupt> {
            let mut res = EnumSet::new();
            let reg_block = self.register_block();
            let ints = reg_block.int_st().read();

            if ints.rx_hung().bit() {
                res.insert(I2sInterrupt::RxHung);
            }
            if ints.tx_hung().bit() {
                res.insert(I2sInterrupt::TxHung);
            }
            if ints.rx_done().bit() {
                res.insert(I2sInterrupt::RxDone);
            }
            if ints.tx_done().bit() {
                res.insert(I2sInterrupt::TxDone);
            }

            res
        }

        fn clear_interrupts(&self, interrupts: EnumSet<I2sInterrupt>) {
            let reg_block = self.register_block();

            reg_block.int_clr().write(|w| {
                for interrupt in interrupts {
                    match interrupt {
                        I2sInterrupt::RxHung => w.rx_hung().clear_bit_by_one(),
                        I2sInterrupt::TxHung => w.tx_hung().clear_bit_by_one(),
                        I2sInterrupt::RxDone => w.rx_done().clear_bit_by_one(),
                        I2sInterrupt::TxDone => w.tx_done().clear_bit_by_one(),
                    };
                }
                w
            });
        }

        #[cfg(any(esp32c3, esp32s3))]
        fn set_clock(&self, clock_settings: I2sClockDividers) {
            let i2s = self.register_block();

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

            i2s.tx_clkm_div_conf().modify(|_, w| unsafe {
                w.tx_clkm_div_x().bits(clkm_div_x as u16);
                w.tx_clkm_div_y().bits(clkm_div_y as u16);
                w.tx_clkm_div_yn1().bit(clkm_div_yn1 != 0);
                w.tx_clkm_div_z().bits(clkm_div_z as u16)
            });

            i2s.tx_clkm_conf().modify(|_, w| unsafe {
                w.clk_en().set_bit();
                w.tx_clk_active().set_bit();
                w.tx_clk_sel()
                    .bits(crate::soc::constants::I2S_DEFAULT_CLK_SRC) // for now fixed at 160MHz
                    ;
                w.tx_clkm_div_num().bits(clock_settings.mclk_divider as u8)
            });

            i2s.tx_conf1().modify(|_, w| unsafe {
                w.tx_bck_div_num()
                    .bits((clock_settings.bclk_divider - 1) as u8)
            });

            i2s.rx_clkm_div_conf().modify(|_, w| unsafe {
                w.rx_clkm_div_x().bits(clkm_div_x as u16);
                w.rx_clkm_div_y().bits(clkm_div_y as u16);
                w.rx_clkm_div_yn1().bit(clkm_div_yn1 != 0);
                w.rx_clkm_div_z().bits(clkm_div_z as u16)
            });

            i2s.rx_clkm_conf().modify(|_, w| unsafe {
                w.rx_clk_active().set_bit();
                // for now fixed at 160MHz
                w.rx_clk_sel()
                    .bits(crate::soc::constants::I2S_DEFAULT_CLK_SRC);
                w.rx_clkm_div_num().bits(clock_settings.mclk_divider as u8);
                w.mclk_sel().bit(true)
            });

            i2s.rx_conf1().modify(|_, w| unsafe {
                w.rx_bck_div_num()
                    .bits((clock_settings.bclk_divider - 1) as u8)
            });
        }

        #[cfg(any(esp32c6, esp32h2))]
        fn set_clock(&self, clock_settings: I2sClockDividers) {
            let i2s = self.register_block();
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

            pcr.i2s_tx_clkm_div_conf().modify(|_, w| unsafe {
                w.i2s_tx_clkm_div_x().bits(clkm_div_x as u16);
                w.i2s_tx_clkm_div_y().bits(clkm_div_y as u16);
                w.i2s_tx_clkm_div_yn1().bit(clkm_div_yn1 != 0);
                w.i2s_tx_clkm_div_z().bits(clkm_div_z as u16)
            });

            pcr.i2s_tx_clkm_conf().modify(|_, w| unsafe {
                w.i2s_tx_clkm_en().set_bit();
                // for now fixed at 160MHz for C6 and 96MHz for H2
                w.i2s_tx_clkm_sel()
                    .bits(crate::soc::constants::I2S_DEFAULT_CLK_SRC);
                w.i2s_tx_clkm_div_num()
                    .bits(clock_settings.mclk_divider as u8)
            });

            #[cfg(not(esp32h2))]
            i2s.tx_conf1().modify(|_, w| unsafe {
                w.tx_bck_div_num()
                    .bits((clock_settings.bclk_divider - 1) as u8)
            });
            #[cfg(esp32h2)]
            i2s.tx_conf().modify(|_, w| unsafe {
                w.tx_bck_div_num()
                    .bits((clock_settings.bclk_divider - 1) as u8)
            });

            pcr.i2s_rx_clkm_div_conf().modify(|_, w| unsafe {
                w.i2s_rx_clkm_div_x().bits(clkm_div_x as u16);
                w.i2s_rx_clkm_div_y().bits(clkm_div_y as u16);
                w.i2s_rx_clkm_div_yn1().bit(clkm_div_yn1 != 0);
                w.i2s_rx_clkm_div_z().bits(clkm_div_z as u16)
            });

            pcr.i2s_rx_clkm_conf().modify(|_, w| unsafe {
                w.i2s_rx_clkm_en().set_bit();
                // for now fixed at 160MHz for C6 and 96MHz for H2
                w.i2s_rx_clkm_sel()
                    .bits(crate::soc::constants::I2S_DEFAULT_CLK_SRC);
                w.i2s_rx_clkm_div_num()
                    .bits(clock_settings.mclk_divider as u8);
                w.i2s_mclk_sel().bit(true)
            });
            #[cfg(not(esp32h2))]
            i2s.rx_conf1().modify(|_, w| unsafe {
                w.rx_bck_div_num()
                    .bits((clock_settings.bclk_divider - 1) as u8)
            });
            #[cfg(esp32h2)]
            i2s.rx_conf().modify(|_, w| unsafe {
                w.rx_bck_div_num()
                    .bits((clock_settings.bclk_divider - 1) as u8)
            });
        }

        fn configure(&self, _standard: &Standard, data_format: &DataFormat) {
            let i2s = self.register_block();

            #[allow(clippy::useless_conversion)]
            i2s.tx_conf1().modify(|_, w| unsafe {
                w.tx_tdm_ws_width()
                    .bits((data_format.channel_bits() - 1).into());
                w.tx_bits_mod().bits(data_format.data_bits() - 1);
                w.tx_tdm_chan_bits().bits(data_format.channel_bits() - 1);
                w.tx_half_sample_bits().bits(data_format.channel_bits() - 1)
            });
            #[cfg(not(esp32h2))]
            i2s.tx_conf1().modify(|_, w| w.tx_msb_shift().set_bit());
            #[cfg(esp32h2)]
            i2s.tx_conf().modify(|_, w| w.tx_msb_shift().set_bit());
            i2s.tx_conf().modify(|_, w| unsafe {
                w.tx_mono().clear_bit();
                w.tx_mono_fst_vld().set_bit();
                w.tx_stop_en().set_bit();
                w.tx_chan_equal().clear_bit();
                w.tx_tdm_en().set_bit();
                w.tx_pdm_en().clear_bit();
                w.tx_pcm_bypass().set_bit();
                w.tx_big_endian().clear_bit();
                w.tx_bit_order().clear_bit();
                w.tx_chan_mod().bits(0)
            });

            i2s.tx_tdm_ctrl().modify(|_, w| unsafe {
                w.tx_tdm_tot_chan_num().bits(1);
                w.tx_tdm_chan0_en().set_bit();
                w.tx_tdm_chan1_en().set_bit();
                w.tx_tdm_chan2_en().clear_bit();
                w.tx_tdm_chan3_en().clear_bit();
                w.tx_tdm_chan4_en().clear_bit();
                w.tx_tdm_chan5_en().clear_bit();
                w.tx_tdm_chan6_en().clear_bit();
                w.tx_tdm_chan7_en().clear_bit();
                w.tx_tdm_chan8_en().clear_bit();
                w.tx_tdm_chan9_en().clear_bit();
                w.tx_tdm_chan10_en().clear_bit();
                w.tx_tdm_chan11_en().clear_bit();
                w.tx_tdm_chan12_en().clear_bit();
                w.tx_tdm_chan13_en().clear_bit();
                w.tx_tdm_chan14_en().clear_bit();
                w.tx_tdm_chan15_en().clear_bit()
            });

            #[allow(clippy::useless_conversion)]
            i2s.rx_conf1().modify(|_, w| unsafe {
                w.rx_tdm_ws_width()
                    .bits((data_format.channel_bits() - 1).into());
                w.rx_bits_mod().bits(data_format.data_bits() - 1);
                w.rx_tdm_chan_bits().bits(data_format.channel_bits() - 1);
                w.rx_half_sample_bits().bits(data_format.channel_bits() - 1)
            });
            #[cfg(not(esp32h2))]
            i2s.rx_conf1().modify(|_, w| w.rx_msb_shift().set_bit());
            #[cfg(esp32h2)]
            i2s.rx_conf().modify(|_, w| w.rx_msb_shift().set_bit());

            i2s.rx_conf().modify(|_, w| unsafe {
                w.rx_mono().clear_bit();
                w.rx_mono_fst_vld().set_bit();
                w.rx_stop_mode().bits(2);
                w.rx_tdm_en().set_bit();
                w.rx_pdm_en().clear_bit();
                w.rx_pcm_bypass().set_bit();
                w.rx_big_endian().clear_bit();
                w.rx_bit_order().clear_bit()
            });

            i2s.rx_tdm_ctrl().modify(|_, w| unsafe {
                w.rx_tdm_tot_chan_num().bits(1);
                w.rx_tdm_pdm_chan0_en().set_bit();
                w.rx_tdm_pdm_chan1_en().set_bit();
                w.rx_tdm_pdm_chan2_en().clear_bit();
                w.rx_tdm_pdm_chan3_en().clear_bit();
                w.rx_tdm_pdm_chan4_en().clear_bit();
                w.rx_tdm_pdm_chan5_en().clear_bit();
                w.rx_tdm_pdm_chan6_en().clear_bit();
                w.rx_tdm_pdm_chan7_en().clear_bit();
                w.rx_tdm_chan8_en().clear_bit();
                w.rx_tdm_chan9_en().clear_bit();
                w.rx_tdm_chan10_en().clear_bit();
                w.rx_tdm_chan11_en().clear_bit();
                w.rx_tdm_chan12_en().clear_bit();
                w.rx_tdm_chan13_en().clear_bit();
                w.rx_tdm_chan14_en().clear_bit();
                w.rx_tdm_chan15_en().clear_bit()
            });
        }

        fn set_master(&self) {
            let i2s = self.register_block();
            i2s.tx_conf().modify(|_, w| w.tx_slave_mod().clear_bit());
            i2s.rx_conf().modify(|_, w| w.rx_slave_mod().clear_bit());
        }

        fn update(&self) {
            let i2s = self.register_block();
            i2s.tx_conf().modify(|_, w| w.tx_update().clear_bit());
            i2s.tx_conf().modify(|_, w| w.tx_update().set_bit());

            i2s.rx_conf().modify(|_, w| w.rx_update().clear_bit());
            i2s.rx_conf().modify(|_, w| w.rx_update().set_bit());
        }

        fn reset_tx(&self) {
            let i2s = self.register_block();
            i2s.tx_conf().modify(|_, w| {
                w.tx_reset().set_bit();
                w.tx_fifo_reset().set_bit()
            });
            i2s.tx_conf().modify(|_, w| {
                w.tx_reset().clear_bit();
                w.tx_fifo_reset().clear_bit()
            });

            i2s.int_clr().write(|w| {
                w.tx_done().clear_bit_by_one();
                w.tx_hung().clear_bit_by_one()
            });
        }

        fn tx_start(&self) {
            let i2s = self.register_block();
            i2s.tx_conf().modify(|_, w| w.tx_start().set_bit());
        }

        fn tx_stop(&self) {
            let i2s = self.register_block();
            i2s.tx_conf().modify(|_, w| w.tx_start().clear_bit());
        }

        fn wait_for_tx_done(&self) {
            let i2s = self.register_block();
            while i2s.state().read().tx_idle().bit_is_clear() {
                // wait
            }

            i2s.tx_conf().modify(|_, w| w.tx_start().clear_bit());
        }

        fn reset_rx(&self) {
            let i2s = self.register_block();
            i2s.rx_conf().modify(|_, w| {
                w.rx_reset().set_bit();
                w.rx_fifo_reset().set_bit()
            });
            i2s.rx_conf().modify(|_, w| {
                w.rx_reset().clear_bit();
                w.rx_fifo_reset().clear_bit()
            });

            i2s.int_clr().write(|w| {
                w.rx_done().clear_bit_by_one();
                w.rx_hung().clear_bit_by_one()
            });
        }

        fn rx_start(&self, len: usize) {
            let len = len - 1;

            let i2s = self.register_block();
            i2s.rxeof_num()
                .write(|w| unsafe { w.rx_eof_num().bits(len as u16) });
            i2s.rx_conf().modify(|_, w| w.rx_start().set_bit());
        }

        fn wait_for_rx_done(&self) {
            let i2s = self.register_block();
            while i2s.int_raw().read().rx_done().bit_is_clear() {
                // wait
            }

            i2s.int_clr().write(|w| w.rx_done().clear_bit_by_one());
        }
    }

    impl RegBlock for I2S0 {
        fn register_block(&self) -> &RegisterBlock {
            unsafe { &*I2S0::PTR.cast::<RegisterBlock>() }
        }
    }

    impl RegisterAccessPrivate for I2S0 {
        fn set_interrupt_handler(&self, handler: InterruptHandler) {
            unsafe { crate::peripherals::I2S0::steal() }.bind_i2s0_interrupt(handler.handler());
            crate::interrupt::enable(crate::peripherals::Interrupt::I2S0, handler.priority())
                .unwrap();
        }
    }

    impl Signals for crate::peripherals::I2S0 {
        fn mclk_signal(&self) -> OutputSignal {
            cfg_if::cfg_if! {
                if #[cfg(esp32)] {
                    panic!("MCLK currently not supported on ESP32");
                } else if #[cfg(esp32s2)] {
                    OutputSignal::CLK_I2S
                } else if #[cfg(esp32s3)] {
                    OutputSignal::I2S0_MCLK
                } else {
                    OutputSignal::I2S_MCLK
                }
            }
        }

        fn bclk_signal(&self) -> OutputSignal {
            cfg_if::cfg_if! {
                if #[cfg(any(esp32, esp32s2, esp32s3))] {
                    OutputSignal::I2S0O_BCK
                } else {
                    OutputSignal::I2SO_BCK
                }
            }
        }

        fn ws_signal(&self) -> OutputSignal {
            cfg_if::cfg_if! {
                if #[cfg(any(esp32, esp32s2, esp32s3))] {
                    OutputSignal::I2S0O_WS
                } else {
                    OutputSignal::I2SO_WS
                }
            }
        }

        fn dout_signal(&self) -> OutputSignal {
            cfg_if::cfg_if! {
                if #[cfg(esp32)] {
                    OutputSignal::I2S0O_DATA_23
                } else if #[cfg(esp32s2)] {
                    OutputSignal::I2S0O_DATA_OUT23
                } else if #[cfg(esp32s3)] {
                    OutputSignal::I2S0O_SD
                } else {
                    OutputSignal::I2SO_SD
                }
            }
        }

        fn bclk_rx_signal(&self) -> OutputSignal {
            cfg_if::cfg_if! {
                if #[cfg(any(esp32, esp32s2, esp32s3))] {
                    OutputSignal::I2S0I_BCK
                } else {
                    OutputSignal::I2SI_BCK
                }
            }
        }

        fn ws_rx_signal(&self) -> OutputSignal {
            cfg_if::cfg_if! {
                if #[cfg(any(esp32, esp32s2, esp32s3))] {
                    OutputSignal::I2S0I_WS
                } else {
                    OutputSignal::I2SI_WS
                }
            }
        }

        fn din_signal(&self) -> InputSignal {
            cfg_if::cfg_if! {
                if #[cfg(esp32)] {
                    InputSignal::I2S0I_DATA_15
                } else if #[cfg(esp32s2)] {
                    InputSignal::I2S0I_DATA_IN15
                } else if #[cfg(esp32s3)] {
                    InputSignal::I2S0I_SD
                } else {
                    InputSignal::I2SI_SD
                }
            }
        }
    }

    #[cfg(i2s1)]
    impl RegBlock for I2S1 {
        fn register_block(&self) -> &RegisterBlock {
            unsafe { &*I2S1::PTR.cast::<RegisterBlock>() }
        }
    }

    #[cfg(i2s1)]
    impl RegisterAccessPrivate for I2S1 {
        fn set_interrupt_handler(&self, handler: InterruptHandler) {
            unsafe { crate::peripherals::I2S1::steal() }.bind_i2s1_interrupt(handler.handler());
            crate::interrupt::enable(crate::peripherals::Interrupt::I2S1, handler.priority())
                .unwrap();
        }
    }

    #[cfg(i2s1)]
    impl Signals for crate::peripherals::I2S1 {
        fn mclk_signal(&self) -> OutputSignal {
            cfg_if::cfg_if! {
                if #[cfg(esp32)] {
                    panic!("MCLK currently not supported on ESP32");
                } else {
                    OutputSignal::I2S1_MCLK
                }
            }
        }

        fn bclk_signal(&self) -> OutputSignal {
            OutputSignal::I2S1O_BCK
        }

        fn ws_signal(&self) -> OutputSignal {
            OutputSignal::I2S1O_WS
        }

        fn dout_signal(&self) -> OutputSignal {
            cfg_if::cfg_if! {
                if #[cfg(esp32)] {
                    OutputSignal::I2S1O_DATA_23
                } else {
                    OutputSignal::I2S1O_SD
                }
            }
        }

        fn bclk_rx_signal(&self) -> OutputSignal {
            OutputSignal::I2S1I_BCK
        }

        fn ws_rx_signal(&self) -> OutputSignal {
            OutputSignal::I2S1I_WS
        }

        fn din_signal(&self) -> InputSignal {
            cfg_if::cfg_if! {
                if #[cfg(esp32)] {
                    InputSignal::I2S1I_DATA_15
                } else {
                    InputSignal::I2S1I_SD
                }
            }
        }
    }

    impl RegBlock for super::AnyI2s {
        delegate::delegate! {
            to match &self.0 {
                super::AnyI2sInner::I2s0(i2s) => i2s,
                #[cfg(i2s1)]
                super::AnyI2sInner::I2s1(i2s) => i2s,
            } {
                fn register_block(&self) -> &RegisterBlock;
            }
        }
    }

    impl RegisterAccessPrivate for super::AnyI2s {
        delegate::delegate! {
            to match &self.0 {
                super::AnyI2sInner::I2s0(i2s) => i2s,
                #[cfg(i2s1)]
                super::AnyI2sInner::I2s1(i2s) => i2s,
            } {
                fn set_interrupt_handler(&self, handler: InterruptHandler);
            }
        }
    }

    impl Signals for super::AnyI2s {
        delegate::delegate! {
            to match &self.0 {
                super::AnyI2sInner::I2s0(i2s) => i2s,
                #[cfg(i2s1)]
                super::AnyI2sInner::I2s1(i2s) => i2s,
            } {
                fn mclk_signal(&self) -> OutputSignal;
                fn bclk_signal(&self) -> OutputSignal;
                fn ws_signal(&self) -> OutputSignal;
                fn dout_signal(&self) -> OutputSignal;
                fn bclk_rx_signal(&self) -> OutputSignal;
                fn ws_rx_signal(&self) -> OutputSignal;
                fn din_signal(&self) -> InputSignal;
            }
        }
    }

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

/// Async functionality
pub mod asynch {
    use super::{Error, I2sRx, I2sTx, RegisterAccess};
    use crate::{
        dma::{
            asynch::{DmaRxDoneChFuture, DmaRxFuture, DmaTxDoneChFuture, DmaTxFuture},
            ReadBuffer,
            Rx,
            RxCircularState,
            Tx,
            TxCircularState,
            WriteBuffer,
        },
        Async,
    };

    impl<'d, T> I2sTx<'d, Async, T>
    where
        T: RegisterAccess,
    {
        /// One-shot write I2S.
        pub async fn write_dma_async(&mut self, words: &mut [u8]) -> Result<(), Error> {
            let (ptr, len) = (words.as_ptr(), words.len());

            self.i2s.reset_tx();

            let future = DmaTxFuture::new(&mut self.tx_channel);

            unsafe {
                self.tx_chain.fill_for_tx(false, ptr, len)?;
                future
                    .tx
                    .prepare_transfer_without_start(self.i2s.dma_peripheral(), &self.tx_chain)
                    .and_then(|_| future.tx.start_transfer())?;
            }

            self.i2s.tx_start();
            future.await?;

            Ok(())
        }

        /// Continuously write to I2S. Returns [I2sWriteDmaTransferAsync]
        pub fn write_dma_circular_async<TXBUF: ReadBuffer>(
            mut self,
            words: TXBUF,
        ) -> Result<I2sWriteDmaTransferAsync<'d, TXBUF, T>, Error> {
            let (ptr, len) = unsafe { words.read_buffer() };

            // Reset TX unit and TX FIFO
            self.i2s.reset_tx();

            // Enable corresponding interrupts if needed

            // configure DMA outlink
            unsafe {
                self.tx_chain.fill_for_tx(true, ptr, len)?;
                self.tx_channel
                    .prepare_transfer_without_start(self.i2s.dma_peripheral(), &self.tx_chain)
                    .and_then(|_| self.tx_channel.start_transfer())?;
            }

            // set I2S_TX_STOP_EN if needed

            // start: set I2S_TX_START
            self.i2s.tx_start();

            let state = TxCircularState::new(&mut self.tx_chain);
            Ok(I2sWriteDmaTransferAsync {
                i2s_tx: self,
                state,
                _buffer: words,
            })
        }
    }

    /// An in-progress async circular DMA write transfer.
    pub struct I2sWriteDmaTransferAsync<'d, BUFFER, T = super::AnyI2s>
    where
        T: RegisterAccess,
    {
        i2s_tx: I2sTx<'d, Async, T>,
        state: TxCircularState,
        _buffer: BUFFER,
    }

    impl<'d, T, BUFFER> I2sWriteDmaTransferAsync<'d, BUFFER, T>
    where
        T: RegisterAccess,
    {
        /// How many bytes can be pushed into the DMA transaction.
        /// Will wait for more than 0 bytes available.
        pub async fn available(&mut self) -> Result<usize, Error> {
            loop {
                self.state.update(&self.i2s_tx.tx_channel);
                let res = self.state.available;

                if res != 0 {
                    break Ok(res);
                }

                DmaTxDoneChFuture::new(&mut self.i2s_tx.tx_channel).await?
            }
        }

        /// Push bytes into the DMA transaction.
        pub async fn push(&mut self, data: &[u8]) -> Result<usize, Error> {
            let avail = self.available().await?;
            let to_send = usize::min(avail, data.len());
            Ok(self.state.push(&data[..to_send])?)
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
            Ok(self.state.push_with(f)?)
        }
    }

    impl<'d, T> I2sRx<'d, Async, T>
    where
        T: RegisterAccess,
    {
        /// One-shot read I2S.
        pub async fn read_dma_async(&mut self, words: &mut [u8]) -> Result<(), Error> {
            let (ptr, len) = (words.as_mut_ptr(), words.len());

            if len % 4 != 0 {
                return Err(Error::IllegalArgument);
            }

            // Reset RX unit and RX FIFO
            self.i2s.reset_rx();

            let future = DmaRxFuture::new(&mut self.rx_channel);

            // configure DMA inlink
            unsafe {
                self.rx_chain.fill_for_rx(false, ptr, len)?;
                future
                    .rx
                    .prepare_transfer_without_start(self.i2s.dma_peripheral(), &self.rx_chain)
                    .and_then(|_| future.rx.start_transfer())?;
            }

            // start: set I2S_RX_START
            self.i2s.rx_start(len);

            future.await?;

            Ok(())
        }

        /// Continuously read from I2S. Returns [I2sReadDmaTransferAsync]
        pub fn read_dma_circular_async<RXBUF>(
            mut self,
            mut words: RXBUF,
        ) -> Result<I2sReadDmaTransferAsync<'d, RXBUF, T>, Error>
        where
            RXBUF: WriteBuffer,
        {
            let (ptr, len) = unsafe { words.write_buffer() };

            if len % 4 != 0 {
                return Err(Error::IllegalArgument);
            }

            // Reset RX unit and RX FIFO
            self.i2s.reset_rx();

            // Enable corresponding interrupts if needed

            // configure DMA inlink
            unsafe {
                self.rx_chain.fill_for_rx(true, ptr, len)?;
                self.rx_channel
                    .prepare_transfer_without_start(self.i2s.dma_peripheral(), &self.rx_chain)
                    .and_then(|_| self.rx_channel.start_transfer())?;
            }

            // start: set I2S_RX_START
            self.i2s.rx_start(len);

            let state = RxCircularState::new(&mut self.rx_chain);
            Ok(I2sReadDmaTransferAsync {
                i2s_rx: self,
                state,
                _buffer: words,
            })
        }
    }

    /// An in-progress async circular DMA read transfer.
    pub struct I2sReadDmaTransferAsync<'d, BUFFER, T = super::AnyI2s>
    where
        T: RegisterAccess,
    {
        i2s_rx: I2sRx<'d, Async, T>,
        state: RxCircularState,
        _buffer: BUFFER,
    }

    impl<'d, T, BUFFER> I2sReadDmaTransferAsync<'d, BUFFER, T>
    where
        T: RegisterAccess,
    {
        /// How many bytes can be popped from the DMA transaction.
        /// Will wait for more than 0 bytes available.
        pub async fn available(&mut self) -> Result<usize, Error> {
            loop {
                self.state.update();

                let res = self.state.available;

                if res != 0 {
                    break Ok(res);
                }

                DmaRxDoneChFuture::new(&mut self.i2s_rx.rx_channel).await?;
            }
        }

        /// Pop bytes from the DMA transaction.
        pub async fn pop(&mut self, data: &mut [u8]) -> Result<usize, Error> {
            let avail = self.available().await?;
            let to_rcv = usize::min(avail, data.len());
            Ok(self.state.pop(&mut data[..to_rcv])?)
        }
    }
}

crate::any_peripheral! {
    /// Any SPI peripheral.
    pub peripheral AnyI2s {
        #[cfg(i2s0)]
        I2s0(crate::peripherals::I2S0),
        #[cfg(i2s1)]
        I2s1(crate::peripherals::I2S1),
    }
}

impl DmaEligible for AnyI2s {
    #[cfg(gdma)]
    type Dma = crate::dma::AnyGdmaChannel;
    #[cfg(pdma)]
    type Dma = crate::dma::AnyI2sDmaChannel;

    fn dma_peripheral(&self) -> crate::dma::DmaPeripheral {
        match &self.0 {
            AnyI2sInner::I2s0(_) => crate::dma::DmaPeripheral::I2s0,
            #[cfg(i2s1)]
            AnyI2sInner::I2s1(_) => crate::dma::DmaPeripheral::I2s1,
        }
    }
}
