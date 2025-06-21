#![cfg_attr(docsrs, procmacros::doc_replace(
    "dma_channel" => {
        cfg(any(esp32, esp32s2)) => "let dma_channel = peripherals.DMA_I2S0;",
        cfg(not(any(esp32, esp32s2))) => "let dma_channel = peripherals.DMA_CH0;"
    },
    "mclk" => {
        cfg(not(esp32)) => "let i2s = i2s.with_mclk(peripherals.GPIO0);",
        _ => ""
    }

))]
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
//! ## Examples
//!
//! ### I2S Read
//!
//! ```rust, no_run
//! # {before_snippet}
//! # use esp_hal::i2s::master::{I2s, Standard, DataFormat};
//! # use esp_hal::dma_buffers;
//! # {dma_channel}
//! let (mut rx_buffer, rx_descriptors, _, _) = dma_buffers!(4 * 4092, 0);
//!
//! let i2s = I2s::new(
//!     peripherals.I2S0,
//!     Standard::Philips,
//!     DataFormat::Data16Channel16,
//!     Rate::from_hz(44100),
//!     dma_channel,
//! );
//! # {mclk}
//! let mut i2s_rx = i2s
//!     .i2s_rx
//!     .with_bclk(peripherals.GPIO1)
//!     .with_ws(peripherals.GPIO2)
//!     .with_din(peripherals.GPIO5)
//!     .build(rx_descriptors);
//!
//! let mut transfer = i2s_rx.read_dma_circular(&mut rx_buffer)?;
//!
//! loop {
//!     let avail = transfer.available()?;
//!
//!     if avail > 0 {
//!         let mut rcv = [0u8; 5000];
//!         transfer.pop(&mut rcv[..avail])?;
//!     }
//! }
//! # }
//! ```
//!
//! ## Implementation State
//!
//! - Only TDM Philips standard is supported.

use core::{
    mem::ManuallyDrop,
    ops::{Deref, DerefMut},
};

use enumset::{EnumSet, EnumSetType};
use private::*;

use crate::{
    Async,
    Blocking,
    DriverMode,
    dma::{
        Channel,
        ChannelRx,
        ChannelTx,
        DmaChannelFor,
        DmaEligible,
        DmaError,
        DmaRxBuffer,
        DmaTxBuffer,
        PeripheralRxChannel,
        PeripheralTxChannel,
    },
    gpio::{OutputConfig, interconnect::PeripheralOutput},
    i2s::AnyI2s,
    interrupt::{InterruptConfigurable, InterruptHandler},
    system::PeripheralGuard,
    time::Rate,
};

#[derive(Debug, EnumSetType)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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
pub(crate) const I2S_LL_MCLK_DIVIDER_BIT_WIDTH: usize = 6;

#[cfg(any(esp32c3, esp32c6, esp32h2))]
pub(crate) const I2S_LL_MCLK_DIVIDER_BIT_WIDTH: usize = 9;

pub(crate) const I2S_LL_MCLK_DIVIDER_MAX: usize = (1 << I2S_LL_MCLK_DIVIDER_BIT_WIDTH) - 1;

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
#[allow(clippy::enum_variant_names, reason = "peripheral is unstable")]
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
#[non_exhaustive]
pub struct I2s<'d, Dm>
where
    Dm: DriverMode,
{
    /// Handles the reception (RX) side of the I2S peripheral.
    pub i2s_rx: RxCreator<'d, Dm>,
    /// Handles the transmission (TX) side of the I2S peripheral.
    pub i2s_tx: TxCreator<'d, Dm>,
}

impl<Dm> I2s<'_, Dm>
where
    Dm: DriverMode,
{
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
    #[instability::unstable]
    pub fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        // tx.i2s and rx.i2s is the same, we could use either one
        self.i2s_tx.i2s.set_interrupt_handler(handler);
    }

    /// Listen for the given interrupts
    #[instability::unstable]
    pub fn listen(&mut self, interrupts: impl Into<EnumSet<I2sInterrupt>>) {
        // tx.i2s and rx.i2s is the same, we could use either one
        self.i2s_tx.i2s.enable_listen(interrupts.into(), true);
    }

    /// Unlisten the given interrupts
    #[instability::unstable]
    pub fn unlisten(&mut self, interrupts: impl Into<EnumSet<I2sInterrupt>>) {
        // tx.i2s and rx.i2s is the same, we could use either one
        self.i2s_tx.i2s.enable_listen(interrupts.into(), false);
    }

    /// Gets asserted interrupts
    #[instability::unstable]
    pub fn interrupts(&mut self) -> EnumSet<I2sInterrupt> {
        // tx.i2s and rx.i2s is the same, we could use either one
        self.i2s_tx.i2s.interrupts()
    }

    /// Resets asserted interrupts
    #[instability::unstable]
    pub fn clear_interrupts(&mut self, interrupts: impl Into<EnumSet<I2sInterrupt>>) {
        // tx.i2s and rx.i2s is the same, we could use either one
        self.i2s_tx.i2s.clear_interrupts(interrupts.into());
    }
}

impl<Dm> crate::private::Sealed for I2s<'_, Dm> where Dm: DriverMode {}

impl<Dm> InterruptConfigurable for I2s<'_, Dm>
where
    Dm: DriverMode,
{
    fn set_interrupt_handler(&mut self, handler: crate::interrupt::InterruptHandler) {
        I2s::set_interrupt_handler(self, handler);
    }
}

impl<'d> I2s<'d, Blocking> {
    /// Construct a new I2S peripheral driver instance for the first I2S
    /// peripheral
    pub fn new(
        i2s: impl Instance + 'd,
        standard: Standard,
        data_format: DataFormat,
        sample_rate: Rate,
        channel: impl DmaChannelFor<AnyI2s<'d>>,
    ) -> Self {
        let channel = Channel::new(channel.degrade());
        channel.runtime_ensure_compatible(&i2s);

        let i2s = i2s.degrade();

        // on ESP32-C3 / ESP32-S3 and later RX and TX are independent and
        // could be configured totally independently but for now handle all
        // the targets the same and force same configuration for both, TX and RX

        // make sure the peripheral is enabled before configuring it
        let peripheral = i2s.peripheral();
        let rx_guard = PeripheralGuard::new(peripheral);
        let tx_guard = PeripheralGuard::new(peripheral);

        i2s.set_clock(calculate_clock(sample_rate, 2, data_format.channel_bits()));
        i2s.configure(&standard, &data_format);
        i2s.set_master();
        i2s.update();

        Self {
            i2s_rx: RxCreator {
                i2s: unsafe { i2s.clone_unchecked() },
                rx_channel: channel.rx,
                guard: rx_guard,
            },
            i2s_tx: TxCreator {
                i2s,
                tx_channel: channel.tx,
                guard: tx_guard,
            },
        }
    }

    /// Converts the I2S instance into async mode.
    pub fn into_async(self) -> I2s<'d, Async> {
        I2s {
            i2s_rx: RxCreator {
                i2s: self.i2s_rx.i2s,
                rx_channel: self.i2s_rx.rx_channel.into_async(),
                guard: self.i2s_rx.guard,
            },
            i2s_tx: TxCreator {
                i2s: self.i2s_tx.i2s,
                tx_channel: self.i2s_tx.tx_channel.into_async(),
                guard: self.i2s_tx.guard,
            },
        }
    }
}

impl<'d, Dm> I2s<'d, Dm>
where
    Dm: DriverMode,
{
    /// Configures the I2S peripheral to use a master clock (MCLK) output pin.
    pub fn with_mclk(self, mclk: impl PeripheralOutput<'d>) -> Self {
        let mclk = mclk.into();

        mclk.apply_output_config(&OutputConfig::default());
        mclk.set_output_enable(true);

        self.i2s_tx.i2s.mclk_signal().connect_to(&mclk);

        self
    }
}

/// I2S TX channel
pub struct I2sTx<'d, Dm>
where
    Dm: DriverMode,
{
    i2s: AnyI2s<'d>,
    tx_channel: ChannelTx<Dm, PeripheralTxChannel<AnyI2s<'d>>>,
    _guard: PeripheralGuard,
}

impl<Dm> core::fmt::Debug for I2sTx<'_, Dm>
where
    Dm: DriverMode,
{
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("I2sTx").finish()
    }
}

impl<'d, Dm> I2sTx<'d, Dm>
where
    Dm: DriverMode,
{
    /// Starts a DMA transfer to write data to the I2S transmitter.
    pub fn write<TXBUF: DmaTxBuffer>(
        mut self,
        mut buf: TXBUF,
    ) -> Result<I2sWriteDmaTransfer<'d, Dm, TXBUF>, Error> {
        // Reset TX unit and TX FIFO
        self.i2s.reset_tx();

        // Enable corresponding interrupts if needed

        // configure DMA outlink
        unsafe {
            self.tx_channel
                .prepare_transfer(self.i2s.dma_peripheral(), &mut buf)
                .and_then(|_| self.tx_channel.start_transfer())?;
        }

        // set I2S_TX_STOP_EN if needed

        // start: set I2S_TX_START
        self.i2s.tx_start();

        Ok(I2sWriteDmaTransfer {
            i2s_tx: ManuallyDrop::new(self),
            buffer_view: ManuallyDrop::new(buf.into_view()),
        })
    }
}

/// I2S RX channel
pub struct I2sRx<'d, Dm>
where
    Dm: DriverMode,
{
    i2s: AnyI2s<'d>,
    rx_channel: ChannelRx<Dm, PeripheralRxChannel<AnyI2s<'d>>>,
    _guard: PeripheralGuard,
}

impl<Dm> core::fmt::Debug for I2sRx<'_, Dm>
where
    Dm: DriverMode,
{
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("I2sRx").finish()
    }
}

impl<'d, Dm> I2sRx<'d, Dm>
where
    Dm: DriverMode,
{
    /// Start reading from the I2S RX unit. Notice because of the DMA transfer,
    /// the buffer must be aligned to 32-bit.
    pub fn read<'t, RXBUF>(
        mut self,
        mut buf: RXBUF,
    ) -> Result<I2sReadDmaTransfer<'d, Dm, RXBUF>, Error>
    where
        RXBUF: DmaRxBuffer,
    {
        // FIXME: not checking if buf is aligned to 32bit anymore as we did before here.
        // Given current implementation, it is the responsibility of the creater
        // of `buf` to check alignment.

        // Reset RX unit and RX FIFO
        self.i2s.reset_rx();

        // Enable corresponding interrupts if needed

        // configure DMA inlink
        unsafe {
            // self.rx_chain.fill_for_rx(circular, ptr, len)?;
            self.rx_channel
                .prepare_transfer(self.i2s.dma_peripheral(), &mut buf)?;
        }
        self.rx_channel.start_transfer()?;

        // start: set I2S_RX_START
        self.i2s.rx_start();
        Ok(I2sReadDmaTransfer {
            i2s_rx: ManuallyDrop::new(self),
            buffer_view: ManuallyDrop::new(buf.into_view()),
        })
    }
}

/// An in-progress DMA write transfer.
pub struct I2sWriteDmaTransfer<'d, Dm: DriverMode, BUFFER: DmaTxBuffer> {
    i2s_tx: ManuallyDrop<I2sTx<'d, Dm>>,
    buffer_view: ManuallyDrop<BUFFER::View>,
}

impl<'d, Dm: DriverMode, BUFFER: DmaTxBuffer> Deref for I2sWriteDmaTransfer<'d, Dm, BUFFER> {
    type Target = BUFFER::View;

    fn deref(&self) -> &Self::Target {
        &self.buffer_view
    }
}

impl<'d, Dm: DriverMode, BUFFER: DmaTxBuffer> DerefMut for I2sWriteDmaTransfer<'d, Dm, BUFFER> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.buffer_view
    }
}

impl<'d, Dm: DriverMode, BUFFER: DmaTxBuffer> Drop for I2sWriteDmaTransfer<'d, Dm, BUFFER> {
    fn drop(&mut self) {
        self.stop_peripheral();
        // SAFETY: This is Drop, we know that self.i2s_tx and self.buffer_view
        // won't be touched again.
        unsafe {
            ManuallyDrop::drop(&mut self.i2s_tx);
            ManuallyDrop::drop(&mut self.buffer_view);
        }
    }
}

impl<'d, Dm: DriverMode, BUFFER: DmaTxBuffer> I2sWriteDmaTransfer<'d, Dm, BUFFER> {
    /// Stops the DMA transfer and returns the I2S transmitter and buffer.
    pub fn stop(mut self) -> (I2sTx<'d, Dm>, BUFFER) {
        self.stop_peripheral();
        let (i2s_tx, view) = self.release();
        (i2s_tx, BUFFER::from_view(view))
    }

    /// Checks if the DMA transfer is done.
    pub fn is_done(&self) -> bool {
        self.i2s_tx.tx_channel.is_done()
    }

    /// Stops and restarts the DMA transfer.
    pub fn restart(self) -> Result<Self, Error> {
        let (i2s, buf) = self.stop();
        i2s.write(buf)
    }

    /// Checks if the transfer has an error.
    pub fn has_error(&self) -> bool {
        self.i2s_tx.tx_channel.has_error()
    }

    fn release(mut self) -> (I2sTx<'d, Dm>, BUFFER::View) {
        // SAFETY: Since forget is called on self, we know that self.i2s_tx and
        // self.buffer_view won't be touched again.
        let result = unsafe {
            (
                ManuallyDrop::take(&mut self.i2s_tx),
                ManuallyDrop::take(&mut self.buffer_view),
            )
        };
        core::mem::forget(self);
        result
    }

    fn stop_peripheral(&mut self) {
        self.i2s_tx.i2s.tx_stop();
    }
}

/// An in-progress async circular DMA read transfer.
pub struct I2sReadDmaTransfer<'d, Dm: DriverMode, BUFFER: DmaRxBuffer> {
    i2s_rx: ManuallyDrop<I2sRx<'d, Dm>>,
    buffer_view: ManuallyDrop<BUFFER::View>,
}

impl<'d, Dm: DriverMode, BUFFER: DmaRxBuffer> Deref for I2sReadDmaTransfer<'d, Dm, BUFFER> {
    type Target = BUFFER::View;

    fn deref(&self) -> &Self::Target {
        &self.buffer_view
    }
}

impl<'d, Dm: DriverMode, BUFFER: DmaRxBuffer> DerefMut for I2sReadDmaTransfer<'d, Dm, BUFFER> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.buffer_view
    }
}

impl<'d, Dm: DriverMode, BUFFER: DmaRxBuffer> Drop for I2sReadDmaTransfer<'d, Dm, BUFFER> {
    fn drop(&mut self) {
        self.stop_peripheral();
        // SAFETY: This is Drop, we know that self.i2s_rx and self.buffer_view
        // won't be touched again.
        unsafe {
            ManuallyDrop::drop(&mut self.i2s_rx);
            ManuallyDrop::drop(&mut self.buffer_view);
        }
    }
}

impl<'d, Dm: DriverMode, BUFFER: DmaRxBuffer> I2sReadDmaTransfer<'d, Dm, BUFFER> {
    /// Stops the transfer and returns the I2S receiver and the buffer.
    pub fn stop(mut self) -> (I2sRx<'d, Dm>, BUFFER) {
        self.stop_peripheral();
        let (i2s_rx, view) = self.release();
        (i2s_rx, BUFFER::from_view(view))
    }

    /// Returns true if the transfer is done.
    pub fn is_done(&self) -> bool {
        self.i2s_rx.rx_channel.is_done()
    }

    /// Stops and restarts the DMA transfer.
    pub fn restart(self) -> Result<Self, Error> {
        let (i2s, buf) = self.stop();
        i2s.read(buf)
    }

    /// Checks if the transfer has an error.
    pub fn has_error(&self) -> bool {
        self.i2s_rx.rx_channel.has_error()
    }

    fn release(mut self) -> (I2sRx<'d, Dm>, BUFFER::View) {
        // SAFETY: Since forget is called on self, we know that self.i2s_tx and
        // self.buffer_view won't be touched again.
        let result = unsafe {
            (
                ManuallyDrop::take(&mut self.i2s_rx),
                ManuallyDrop::take(&mut self.buffer_view),
            )
        };
        core::mem::forget(self);
        result
    }

    fn stop_peripheral(&mut self) {
        self.i2s_rx.i2s.rx_stop();
    }
}

/// A peripheral singleton compatible with the I2S master driver.
pub trait Instance: RegisterAccessPrivate + super::any::Degrade {}
#[cfg(soc_has_i2s0)]
impl Instance for crate::peripherals::I2S0<'_> {}
#[cfg(soc_has_i2s1)]
impl Instance for crate::peripherals::I2S1<'_> {}
impl Instance for AnyI2s<'_> {}

mod private {
    use enumset::EnumSet;

    use super::*;
    #[cfg(not(soc_has_i2s1))]
    use crate::pac::i2s0::RegisterBlock;
    use crate::{
        DriverMode,
        dma::{ChannelRx, ChannelTx, DmaEligible},
        gpio::{
            InputConfig,
            InputSignal,
            OutputConfig,
            OutputSignal,
            interconnect::{PeripheralInput, PeripheralOutput},
        },
        i2s::any::Inner as AnyI2sInner,
        interrupt::InterruptHandler,
        peripherals::I2S0,
    };
    // on ESP32-S3 I2S1 doesn't support all features - use that to avoid using those features
    // by accident
    #[cfg(soc_has_i2s1)]
    use crate::{pac::i2s1::RegisterBlock, peripherals::I2S1};

    pub struct TxCreator<'d, Dm>
    where
        Dm: DriverMode,
    {
        pub i2s: AnyI2s<'d>,
        pub tx_channel: ChannelTx<Dm, PeripheralTxChannel<AnyI2s<'d>>>,
        pub(crate) guard: PeripheralGuard,
    }

    impl<'d, Dm> TxCreator<'d, Dm>
    where
        Dm: DriverMode,
    {
        pub fn build(self) -> I2sTx<'d, Dm> {
            let peripheral = self.i2s.peripheral();
            I2sTx {
                i2s: self.i2s,
                tx_channel: self.tx_channel,
                _guard: PeripheralGuard::new(peripheral),
            }
        }

        pub fn with_bclk(self, bclk: impl PeripheralOutput<'d>) -> Self {
            let bclk = bclk.into();

            bclk.apply_output_config(&OutputConfig::default());
            bclk.set_output_enable(true);

            self.i2s.bclk_signal().connect_to(&bclk);

            self
        }

        pub fn with_ws(self, ws: impl PeripheralOutput<'d>) -> Self {
            let ws = ws.into();

            ws.apply_output_config(&OutputConfig::default());
            ws.set_output_enable(true);

            self.i2s.ws_signal().connect_to(&ws);

            self
        }

        pub fn with_dout(self, dout: impl PeripheralOutput<'d>) -> Self {
            let dout = dout.into();

            dout.apply_output_config(&OutputConfig::default());
            dout.set_output_enable(true);

            self.i2s.dout_signal().connect_to(&dout);

            self
        }
    }

    pub struct RxCreator<'d, Dm>
    where
        Dm: DriverMode,
    {
        pub i2s: AnyI2s<'d>,
        pub rx_channel: ChannelRx<Dm, PeripheralRxChannel<AnyI2s<'d>>>,
        pub(crate) guard: PeripheralGuard,
    }

    impl<'d, Dm> RxCreator<'d, Dm>
    where
        Dm: DriverMode,
    {
        pub fn build(self) -> I2sRx<'d, Dm> {
            let peripheral = self.i2s.peripheral();
            I2sRx {
                i2s: self.i2s,
                rx_channel: self.rx_channel,
                _guard: PeripheralGuard::new(peripheral),
            }
        }

        pub fn with_bclk(self, bclk: impl PeripheralOutput<'d>) -> Self {
            let bclk = bclk.into();

            bclk.apply_output_config(&OutputConfig::default());
            bclk.set_output_enable(true);

            self.i2s.bclk_rx_signal().connect_to(&bclk);

            self
        }

        pub fn with_ws(self, ws: impl PeripheralOutput<'d>) -> Self {
            let ws = ws.into();

            ws.apply_output_config(&OutputConfig::default());
            ws.set_output_enable(true);

            self.i2s.ws_rx_signal().connect_to(&ws);

            self
        }

        pub fn with_din(self, din: impl PeripheralInput<'d>) -> Self {
            let din = din.into();

            din.apply_input_config(&InputConfig::default());
            din.set_input_enable(true);

            self.i2s.din_signal().connect_to(&din);

            self
        }
    }

    #[allow(private_bounds)]
    pub trait RegBlock: DmaEligible {
        fn regs(&self) -> &RegisterBlock;
        fn peripheral(&self) -> crate::system::Peripheral;
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
        fn enable_listen(&self, interrupts: EnumSet<I2sInterrupt>, enable: bool) {
            self.regs().int_ena().modify(|_, w| {
                for interrupt in interrupts {
                    match interrupt {
                        I2sInterrupt::RxHung => w.rx_hung().bit(enable),
                        I2sInterrupt::TxHung => w.tx_hung().bit(enable),
                    };
                }
                w
            });
        }

        fn interrupts(&self) -> EnumSet<I2sInterrupt> {
            let mut res = EnumSet::new();
            let ints = self.regs().int_st().read();

            if ints.rx_hung().bit() {
                res.insert(I2sInterrupt::RxHung);
            }
            if ints.tx_hung().bit() {
                res.insert(I2sInterrupt::TxHung);
            }

            res
        }

        fn clear_interrupts(&self, interrupts: EnumSet<I2sInterrupt>) {
            self.regs().int_clr().write(|w| {
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
            self.regs().clkm_conf().modify(|r, w| unsafe {
                // select PLL_160M
                w.bits(r.bits() | (crate::soc::constants::I2S_DEFAULT_CLK_SRC << 21))
            });

            #[cfg(esp32)]
            self.regs()
                .clkm_conf()
                .modify(|_, w| w.clka_ena().clear_bit());

            self.regs().clkm_conf().modify(|_, w| unsafe {
                w.clk_en().set_bit();
                w.clkm_div_num().bits(clock_settings.mclk_divider as u8);
                w.clkm_div_a().bits(clock_settings.denominator as u8);
                w.clkm_div_b().bits(clock_settings.numerator as u8)
            });

            self.regs().sample_rate_conf().modify(|_, w| unsafe {
                w.tx_bck_div_num().bits(clock_settings.bclk_divider as u8);
                w.rx_bck_div_num().bits(clock_settings.bclk_divider as u8)
            });
        }

        fn configure(&self, _standard: &Standard, data_format: &DataFormat) {
            let fifo_mod = match data_format {
                DataFormat::Data32Channel32 => 2,
                DataFormat::Data16Channel16 => 0,
            };

            self.regs().sample_rate_conf().modify(|_, w| unsafe {
                w.tx_bits_mod().bits(data_format.channel_bits());
                w.rx_bits_mod().bits(data_format.channel_bits())
            });

            self.regs().conf().modify(|_, w| {
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

            self.regs().fifo_conf().modify(|_, w| unsafe {
                w.tx_fifo_mod().bits(fifo_mod);
                w.tx_fifo_mod_force_en().set_bit();
                w.dscr_en().set_bit();
                w.rx_fifo_mod().bits(fifo_mod);
                w.rx_fifo_mod_force_en().set_bit()
            });

            self.regs().conf_chan().modify(|_, w| unsafe {
                // for now only stereo
                w.tx_chan_mod().bits(0);
                w.rx_chan_mod().bits(0)
            });

            self.regs().conf1().modify(|_, w| {
                w.tx_pcm_bypass().set_bit();
                w.rx_pcm_bypass().set_bit()
            });

            self.regs().pd_conf().modify(|_, w| {
                w.fifo_force_pu().set_bit();
                w.fifo_force_pd().clear_bit()
            });

            self.regs().conf2().modify(|_, w| {
                w.camera_en().clear_bit();
                w.lcd_en().clear_bit()
            });
        }

        fn set_master(&self) {
            self.regs().conf().modify(|_, w| {
                w.rx_slave_mod().clear_bit();
                w.tx_slave_mod().clear_bit()
            });
        }

        fn update(&self) {
            // nothing to do
        }

        fn reset_tx(&self) {
            self.regs().conf().modify(|_, w| {
                w.tx_reset().set_bit();
                w.tx_fifo_reset().set_bit()
            });
            self.regs().conf().modify(|_, w| {
                w.tx_reset().clear_bit();
                w.tx_fifo_reset().clear_bit()
            });

            self.regs().lc_conf().modify(|_, w| w.out_rst().set_bit());
            self.regs().lc_conf().modify(|_, w| w.out_rst().clear_bit());

            self.regs().int_clr().write(|w| {
                w.out_done().clear_bit_by_one();
                w.out_total_eof().clear_bit_by_one()
            });
        }

        fn tx_start(&self) {
            self.regs().conf().modify(|_, w| w.tx_start().set_bit());

            while self.regs().state().read().tx_idle().bit_is_set() {
                // wait
            }
        }

        fn tx_stop(&self) {
            self.regs().conf().modify(|_, w| w.tx_start().clear_bit());
        }

        fn wait_for_tx_done(&self) {
            while self.regs().state().read().tx_idle().bit_is_clear() {
                // wait
            }

            self.regs().conf().modify(|_, w| w.tx_start().clear_bit());
        }

        fn reset_rx(&self) {
            self.regs().conf().modify(|_, w| {
                w.rx_reset().set_bit();
                w.rx_fifo_reset().set_bit()
            });
            self.regs().conf().modify(|_, w| {
                w.rx_reset().clear_bit();
                w.rx_fifo_reset().clear_bit()
            });

            self.regs().lc_conf().modify(|_, w| w.in_rst().set_bit());
            self.regs().lc_conf().modify(|_, w| w.in_rst().clear_bit());

            self.regs().int_clr().write(|w| {
                w.in_done().clear_bit_by_one();
                w.in_suc_eof().clear_bit_by_one()
            });
        }

        fn rx_start(&self, len: usize) {
            self.regs()
                .int_clr()
                .write(|w| w.in_suc_eof().clear_bit_by_one());

            cfg_if::cfg_if! {
                if #[cfg(esp32)] {
                    // On ESP32, the eof_num count in words.
                    let eof_num = len / 4;
                } else {
                    let eof_num = len - 1;
                }
            }

            self.regs()
                .rxeof_num()
                .modify(|_, w| unsafe { w.rx_eof_num().bits(eof_num as u32) });

            self.regs().conf().modify(|_, w| w.rx_start().set_bit());
        }

        fn wait_for_rx_done(&self) {
            while self.regs().int_raw().read().in_suc_eof().bit_is_clear() {
                // wait
            }

            self.regs()
                .int_clr()
                .write(|w| w.in_suc_eof().clear_bit_by_one());
        }
    }

    #[cfg(any(esp32c3, esp32c6, esp32h2, esp32s3))]
    pub trait RegisterAccessPrivate: Signals + RegBlock {
        fn enable_listen(&self, interrupts: EnumSet<I2sInterrupt>, enable: bool) {
            self.regs().int_ena().modify(|_, w| {
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
            let ints = self.regs().int_st().read();

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
            self.regs().int_clr().write(|w| {
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

            self.regs().tx_clkm_div_conf().modify(|_, w| unsafe {
                w.tx_clkm_div_x().bits(clkm_div_x as u16);
                w.tx_clkm_div_y().bits(clkm_div_y as u16);
                w.tx_clkm_div_yn1().bit(clkm_div_yn1 != 0);
                w.tx_clkm_div_z().bits(clkm_div_z as u16)
            });

            self.regs().tx_clkm_conf().modify(|_, w| unsafe {
                w.clk_en().set_bit();
                w.tx_clk_active().set_bit();
                w.tx_clk_sel()
                    .bits(crate::soc::constants::I2S_DEFAULT_CLK_SRC) // for now fixed at 160MHz
                    ;
                w.tx_clkm_div_num().bits(clock_settings.mclk_divider as u8)
            });

            self.regs().tx_conf1().modify(|_, w| unsafe {
                w.tx_bck_div_num()
                    .bits((clock_settings.bclk_divider - 1) as u8)
            });

            self.regs().rx_clkm_div_conf().modify(|_, w| unsafe {
                w.rx_clkm_div_x().bits(clkm_div_x as u16);
                w.rx_clkm_div_y().bits(clkm_div_y as u16);
                w.rx_clkm_div_yn1().bit(clkm_div_yn1 != 0);
                w.rx_clkm_div_z().bits(clkm_div_z as u16)
            });

            self.regs().rx_clkm_conf().modify(|_, w| unsafe {
                w.rx_clk_active().set_bit();
                // for now fixed at 160MHz
                w.rx_clk_sel()
                    .bits(crate::soc::constants::I2S_DEFAULT_CLK_SRC);
                w.rx_clkm_div_num().bits(clock_settings.mclk_divider as u8);
                w.mclk_sel().bit(true)
            });

            self.regs().rx_conf1().modify(|_, w| unsafe {
                w.rx_bck_div_num()
                    .bits((clock_settings.bclk_divider - 1) as u8)
            });
        }

        #[cfg(any(esp32c6, esp32h2))]
        fn set_clock(&self, clock_settings: I2sClockDividers) {
            // I2S clocks are configured via PCR
            use crate::peripherals::PCR;

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

            PCR::regs().i2s_tx_clkm_div_conf().modify(|_, w| unsafe {
                w.i2s_tx_clkm_div_x().bits(clkm_div_x as u16);
                w.i2s_tx_clkm_div_y().bits(clkm_div_y as u16);
                w.i2s_tx_clkm_div_yn1().bit(clkm_div_yn1 != 0);
                w.i2s_tx_clkm_div_z().bits(clkm_div_z as u16)
            });

            PCR::regs().i2s_tx_clkm_conf().modify(|_, w| unsafe {
                w.i2s_tx_clkm_en().set_bit();
                // for now fixed at 160MHz for C6 and 96MHz for H2
                w.i2s_tx_clkm_sel()
                    .bits(crate::soc::constants::I2S_DEFAULT_CLK_SRC);
                w.i2s_tx_clkm_div_num()
                    .bits(clock_settings.mclk_divider as u8)
            });

            #[cfg(not(esp32h2))]
            self.regs().tx_conf1().modify(|_, w| unsafe {
                w.tx_bck_div_num()
                    .bits((clock_settings.bclk_divider - 1) as u8)
            });
            #[cfg(esp32h2)]
            self.regs().tx_conf().modify(|_, w| unsafe {
                w.tx_bck_div_num()
                    .bits((clock_settings.bclk_divider - 1) as u8)
            });

            PCR::regs().i2s_rx_clkm_div_conf().modify(|_, w| unsafe {
                w.i2s_rx_clkm_div_x().bits(clkm_div_x as u16);
                w.i2s_rx_clkm_div_y().bits(clkm_div_y as u16);
                w.i2s_rx_clkm_div_yn1().bit(clkm_div_yn1 != 0);
                w.i2s_rx_clkm_div_z().bits(clkm_div_z as u16)
            });

            PCR::regs().i2s_rx_clkm_conf().modify(|_, w| unsafe {
                w.i2s_rx_clkm_en().set_bit();
                // for now fixed at 160MHz for C6 and 96MHz for H2
                w.i2s_rx_clkm_sel()
                    .bits(crate::soc::constants::I2S_DEFAULT_CLK_SRC);
                w.i2s_rx_clkm_div_num()
                    .bits(clock_settings.mclk_divider as u8);
                w.i2s_mclk_sel().bit(true)
            });
            #[cfg(not(esp32h2))]
            self.regs().rx_conf1().modify(|_, w| unsafe {
                w.rx_bck_div_num()
                    .bits((clock_settings.bclk_divider - 1) as u8)
            });
            #[cfg(esp32h2)]
            self.regs().rx_conf().modify(|_, w| unsafe {
                w.rx_bck_div_num()
                    .bits((clock_settings.bclk_divider - 1) as u8)
            });
        }

        fn configure(&self, _standard: &Standard, data_format: &DataFormat) {
            #[allow(clippy::useless_conversion)]
            self.regs().tx_conf1().modify(|_, w| unsafe {
                w.tx_tdm_ws_width()
                    .bits((data_format.channel_bits() - 1).into());
                w.tx_bits_mod().bits(data_format.data_bits() - 1);
                w.tx_tdm_chan_bits().bits(data_format.channel_bits() - 1);
                w.tx_half_sample_bits().bits(data_format.channel_bits() - 1)
            });
            #[cfg(not(esp32h2))]
            self.regs()
                .tx_conf1()
                .modify(|_, w| w.tx_msb_shift().set_bit());
            #[cfg(esp32h2)]
            self.regs()
                .tx_conf()
                .modify(|_, w| w.tx_msb_shift().set_bit());
            self.regs().tx_conf().modify(|_, w| unsafe {
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

            self.regs().tx_tdm_ctrl().modify(|_, w| unsafe {
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
            self.regs().rx_conf1().modify(|_, w| unsafe {
                w.rx_tdm_ws_width()
                    .bits((data_format.channel_bits() - 1).into());
                w.rx_bits_mod().bits(data_format.data_bits() - 1);
                w.rx_tdm_chan_bits().bits(data_format.channel_bits() - 1);
                w.rx_half_sample_bits().bits(data_format.channel_bits() - 1)
            });
            #[cfg(not(esp32h2))]
            self.regs()
                .rx_conf1()
                .modify(|_, w| w.rx_msb_shift().set_bit());
            #[cfg(esp32h2)]
            self.regs()
                .rx_conf()
                .modify(|_, w| w.rx_msb_shift().set_bit());

            self.regs().rx_conf().modify(|_, w| unsafe {
                w.rx_mono().clear_bit();
                w.rx_mono_fst_vld().set_bit();
                w.rx_stop_mode().bits(2);
                w.rx_tdm_en().set_bit();
                w.rx_pdm_en().clear_bit();
                w.rx_pcm_bypass().set_bit();
                w.rx_big_endian().clear_bit();
                w.rx_bit_order().clear_bit()
            });

            self.regs().rx_tdm_ctrl().modify(|_, w| unsafe {
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
            self.regs()
                .tx_conf()
                .modify(|_, w| w.tx_slave_mod().clear_bit());
            self.regs()
                .rx_conf()
                .modify(|_, w| w.rx_slave_mod().clear_bit());
        }

        fn update(&self) {
            self.regs()
                .tx_conf()
                .modify(|_, w| w.tx_update().clear_bit());
            self.regs().tx_conf().modify(|_, w| w.tx_update().set_bit());

            self.regs()
                .rx_conf()
                .modify(|_, w| w.rx_update().clear_bit());
            self.regs().rx_conf().modify(|_, w| w.rx_update().set_bit());
        }

        fn reset_tx(&self) {
            self.regs().tx_conf().modify(|_, w| {
                w.tx_reset().set_bit();
                w.tx_fifo_reset().set_bit()
            });
            self.regs().tx_conf().modify(|_, w| {
                w.tx_reset().clear_bit();
                w.tx_fifo_reset().clear_bit()
            });

            self.regs().int_clr().write(|w| {
                w.tx_done().clear_bit_by_one();
                w.tx_hung().clear_bit_by_one()
            });
        }

        fn tx_start(&self) {
            self.regs().tx_conf().modify(|_, w| w.tx_start().set_bit());
        }

        fn tx_stop(&self) {
            self.regs()
                .tx_conf()
                .modify(|_, w| w.tx_start().clear_bit());
        }

        fn wait_for_tx_done(&self) {
            while self.regs().state().read().tx_idle().bit_is_clear() {
                // wait
            }

            self.regs()
                .tx_conf()
                .modify(|_, w| w.tx_start().clear_bit());
        }

        fn reset_rx(&self) {
            self.regs()
                .rx_conf()
                .modify(|_, w| w.rx_start().clear_bit());

            self.regs().rx_conf().modify(|_, w| {
                w.rx_reset().set_bit();
                w.rx_fifo_reset().set_bit()
            });
            self.regs().rx_conf().modify(|_, w| {
                w.rx_reset().clear_bit();
                w.rx_fifo_reset().clear_bit()
            });

            self.regs().int_clr().write(|w| {
                w.rx_done().clear_bit_by_one();
                w.rx_hung().clear_bit_by_one()
            });
        }

        fn rx_start(&self) {
            self.regs().rx_conf().modify(|_, w| w.rx_start().set_bit());
        }

        fn rx_stop(&self) {
            self.regs()
                .rx_conf()
                .modify(|_, w| w.rx_start().clear_bit());
        }

        fn wait_for_rx_done(&self) {
            while self.regs().int_raw().read().rx_done().bit_is_clear() {
                // wait
            }

            self.regs()
                .int_clr()
                .write(|w| w.rx_done().clear_bit_by_one());
        }
    }

    impl RegBlock for I2S0<'_> {
        fn regs(&self) -> &RegisterBlock {
            unsafe { &*I2S0::PTR.cast::<RegisterBlock>() }
        }

        fn peripheral(&self) -> crate::system::Peripheral {
            crate::system::Peripheral::I2s0
        }
    }

    impl RegisterAccessPrivate for I2S0<'_> {}

    impl Signals for crate::peripherals::I2S0<'_> {
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

    #[cfg(soc_has_i2s1)]
    impl RegBlock for I2S1<'_> {
        fn regs(&self) -> &RegisterBlock {
            unsafe { &*I2S1::PTR.cast::<RegisterBlock>() }
        }

        fn peripheral(&self) -> crate::system::Peripheral {
            crate::system::Peripheral::I2s1
        }
    }

    #[cfg(soc_has_i2s1)]
    impl RegisterAccessPrivate for I2S1<'_> {}

    #[cfg(soc_has_i2s1)]
    impl Signals for crate::peripherals::I2S1<'_> {
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

    impl RegBlock for super::AnyI2s<'_> {
        fn regs(&self) -> &RegisterBlock {
            match &self.0 {
                #[cfg(soc_has_i2s0)]
                AnyI2sInner::I2s0(i2s) => RegBlock::regs(i2s),
                #[cfg(soc_has_i2s1)]
                AnyI2sInner::I2s1(i2s) => RegBlock::regs(i2s),
            }
        }

        delegate::delegate! {
            to match &self.0 {
                #[cfg(soc_has_i2s0)]
                AnyI2sInner::I2s0(i2s) => i2s,
                #[cfg(soc_has_i2s1)]
                AnyI2sInner::I2s1(i2s) => i2s,
            } {
                fn peripheral(&self) -> crate::system::Peripheral;
            }
        }
    }

    impl RegisterAccessPrivate for super::AnyI2s<'_> {}

    impl super::AnyI2s<'_> {
        delegate::delegate! {
            to match &self.0 {
                #[cfg(soc_has_i2s0)]
                AnyI2sInner::I2s0(i2s) => i2s,
                #[cfg(soc_has_i2s1)]
                AnyI2sInner::I2s1(i2s) => i2s,
            } {
                fn bind_peri_interrupt(&self, handler: unsafe extern "C" fn() -> ());
                fn disable_peri_interrupt(&self);
                fn enable_peri_interrupt(&self, priority: crate::interrupt::Priority);
            }
        }

        pub(super) fn set_interrupt_handler(&self, handler: InterruptHandler) {
            self.disable_peri_interrupt();
            self.bind_peri_interrupt(handler.handler());
            self.enable_peri_interrupt(handler.priority());
        }
    }

    impl Signals for super::AnyI2s<'_> {
        delegate::delegate! {
            to match &self.0 {
                #[cfg(soc_has_i2s0)]
                AnyI2sInner::I2s0(i2s) => i2s,
                #[cfg(soc_has_i2s1)]
                AnyI2sInner::I2s1(i2s) => i2s,
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

    pub fn calculate_clock(sample_rate: Rate, channels: u8, data_bits: u8) -> I2sClockDividers {
        // this loosely corresponds to `i2s_std_calculate_clock` and
        // `i2s_ll_tx_set_mclk` in esp-idf
        //
        // main difference is we are using fixed-point arithmetic here

        // If data_bits is a power of two, use 256 as the mclk_multiple
        // If data_bits is 24, use 192 (24 * 8) as the mclk_multiple
        let mclk_multiple = if data_bits == 24 { 192 } else { 256 };
        let sclk = crate::soc::constants::I2S_SCLK; // for now it's fixed 160MHz and 96MHz (just H2)

        let rate = sample_rate.as_hz();

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
    use super::{Error, I2sWriteDmaTransfer};
    use crate::{
        Async,
        dma::{
            DmaRxBuffer,
            DmaTxBuffer,
            asynch::{DmaRxDoneChFuture, DmaTxDoneChFuture},
        },
        i2s::master::I2sReadDmaTransfer,
    };

    impl<BUFFER: DmaTxBuffer> I2sWriteDmaTransfer<'_, Async, BUFFER> {
        /// Waits for DMA process to be made and additional room to be
        /// available.
        ///
        /// Returns [crate::dma::DmaError::Late] if DMA is already completed.
        pub async fn wait_for_available(&mut self) -> Result<(), Error> {
            DmaTxDoneChFuture::new(&mut self.i2s_tx.tx_channel).await?;
            Ok(())
        }
    }

    impl<BUFFER: DmaRxBuffer> I2sReadDmaTransfer<'_, Async, BUFFER> {
        /// Waits for DMA process to be made and additional room to be
        /// available.
        ///
        /// Returns [crate::dma::DmaError::Late] if DMA is already completed.
        pub async fn wait_for_available(&mut self) -> Result<(), Error> {
            DmaRxDoneChFuture::new(&mut self.i2s_rx.rx_channel).await?;
            Ok(())
        }
    }
}
