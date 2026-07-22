#![cfg_attr(docsrs, procmacros::doc_replace(
    "dma_channel" => {
        cfg(i2s_dma_engine = "I2S_DMA")  => "DMA_I2S0",
        _ => "DMA_CH0",
    },
    "mclk" => {
        cfg(not(esp32)) => "let i2s = i2s.with_mclk(peripherals.GPIO0);",
        _ => ""
    },
    "tdm_slot_philips" => include_str!("../tdm_slot_philips.svg"),
    "tdm_slot_msb" => include_str!("../tdm_slot_msb.svg"),
    "tdm_slot_pcm_short" => include_str!("../tdm_slot_pcm_short.svg"),
    "tdm_slot_pcm_long" => include_str!("../tdm_slot_pcm_long.svg"),
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
//! ### Standards
//!
//! I2S supports different standards, which you can access using [`TdmConfig::new_tdm_philips`]
//! and related helpers. You can also configure custom data formats using methods such as
//! [`TdmConfig::with_msb_shift`], [`TdmConfig::with_ws_width`], [`TdmConfig::with_ws_polarity`],
//! etc.
//!
//! In TDM mode, WS (word select, sometimes called LRCLK or left/right clock) becomes a frame
//! synchronization signal that signals the first slot of a frame. The two sides of the TDM link
//! must agree on the number of channels, data bit width, and frame synchronization pattern; this
//! cannot be determined by examining the signal itself.
//!
//! #### TDM Philips Standard
//!
//! TDM Philips mode pulls the WS line low one BCK period before the first data bit of the first
//! slot is sent and holds it low for 50% of the frame.
//!
//! # {tdm_slot_philips}
//!
//! #### TDM MSB Standard
//!
//! MSB (most-significant bit) mode is similar to Philips mode, except the WS line is pulled low at
//! the same time the first data bit of the first slot is sent. It is held low for 50% of the frame.
//!
//! # {tdm_slot_msb}
//!
//! #### TDM PCM Short Standard
//!
//! PCM (pulse-code modulation) short mode pulls the WS line *high* one BCK period before the first
//! data bit of the first slot is sent, keeps it high for one BCK, then pulls it low for the
//! remainder of the frame.
//!
//! # {tdm_slot_pcm_short}
//!
//! #### TDM PCM Long Standard
//!
//! PCM long mode pulls the WS line *high* one BCK period before the first data bit of the first
//! slot is sent, keeps it high until just before the last data bit of the first slot is sent, then
//! pulls it low for the remainder of the frame.
//!
//! # {tdm_slot_pcm_long}
//!
//! Diagrams from _ESP-IDF Programming Guide_; rendered by Wavedrom.
//!
//! ## Examples
//!
//! ### I2S Read
//!
//! ```rust, no_run
//! # {before_snippet}
//! # use esp_hal::i2s::master::{I2s, Channels, DataFormat, TdmConfig};
//! # use esp_hal::dma_rx_stream_buffer;
//! let rx_buffer = dma_rx_stream_buffer!(4 * 4092, 1024);
//!
//! let i2s = I2s::new(
//!     peripherals.I2S0,
//!     peripherals.__dma_channel__,
//!     TdmConfig::new_tdm_philips()
//!         .with_sample_rate(Rate::from_hz(44100))
//!         .with_data_format(DataFormat::Data16Channel16)
//!         .with_channels(Channels::STEREO),
//! )?;
//! # {mclk}
//! let mut i2s_rx = i2s
//!     .i2s_rx
//!     .with_bclk(peripherals.GPIO1)
//!     .with_ws(peripherals.GPIO2)
//!     .with_din(peripherals.GPIO5)
//!     .build();
//!
//! let mut transfer = i2s_rx.read(rx_buffer)?;
//!
//! loop {
//!     let avail = transfer.available_bytes();
//!
//!     if avail > 0 {
//!         let mut rcv = [0u8; 5000];
//!         transfer.pop(&mut rcv[..avail]);
//!     }
//! }
//! # }
//! ```
#![cfg_attr(
    any(i2s_supports_pdm_tx, i2s_supports_pdm_rx),
    doc = r"## PDM mode

PDM (pulse-density modulation) is supported on **I2S0 only** on chips where the
hardware provides PDM filters. Use [`I2s::new_pdm`] with [`PdmConfig`]. PDM uses a
single clock pin (`with_clk` on [`I2s::i2s_tx`] / [`I2s::i2s_rx`]) instead of
separate BCLK and WS lines. Only simplex operation (TX *or* RX) is supported.

```rust, no_run
# {before_snippet}
use esp_hal::i2s::master::{I2s, PdmSlotMode, PdmTxConfig, PdmConfig};
use esp_hal::time::Rate;

let pdm = PdmConfig::tx_only(PdmTxConfig::new_codec_default(
    Rate::from_hz(16_000),
    PdmSlotMode::Mono,
));
let i2s = I2s::new_pdm(peripherals.I2S0, peripherals.__dma_channel__, pdm)?;
# {after_snippet}
```
"
)]

use core::mem::ManuallyDrop;

use enumset::{EnumSet, EnumSetType, enum_set};
use private::*;

mod low_level;

#[doc(hidden)]
pub use low_level::Info;

#[cfg(any(i2s_supports_pdm_tx, i2s_supports_pdm_rx))]
pub use super::pdm::{PdmConfig, PdmError, PdmInstance, PdmRxConfig, PdmSlotMode, PdmTxConfig};
use crate::{
    Async,
    Blocking,
    DriverMode,
    dma::{
        Channel,
        ChannelRx,
        ChannelTx,
        DmaChannel,
        DmaEligiblePeripheral,
        DmaError,
        DmaRxBuffer,
        DmaRxInterrupt,
        DmaTxBuffer,
        DmaTxInterrupt,
        asynch::{DmaRxFuture, DmaTxFuture},
    },
    gpio::{
        InputConfig,
        InputSignal,
        OutputConfig,
        OutputSignal,
        interconnect::{PeripheralInput, PeripheralOutput},
    },
    i2s::{AnyI2s, any::Inner as AnyI2sInner},
    interrupt::{InterruptConfigurable, InterruptHandler},
    pac::i2s0::RegisterBlock,
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

    #[cfg(not(i2s_version = "1"))]
    /// Reception of data is complete.
    RxDone,

    #[cfg(not(i2s_version = "1"))]
    /// Transmission of data is complete.
    TxDone,
}

with_i2s_dma_engine! {
    ($engine:tt, $any_channel:ident) => {
        /// DMA channel trait for I2S master peripherals.
        ///
        /// Implemented for each channel type that can serve a particular I2S instance `S`.
        #[instability::unstable]
        #[diagnostic::on_unimplemented(
            message = "The DMA channel cannot be used with this I2S peripheral",
            label = "This DMA channel",
            note = "Use a channel that matches the I2S instance."
        )]
        pub trait I2sMasterDmaChannel<'d, S>: crate::private::Sealed + Into<crate::dma::$any_channel<'d>> {}

        crate::macros::impl_dma_channel_trait! {
            $engine,
            any_peri = AnyI2s<'d>,
            peris = for_each_i2s,
            ($peri:path, $ch:path) => {
                impl<'d> I2sMasterDmaChannel<'d, $peri> for $ch {}
            }
        }

        // Proxy type so that the type-erased DMA channel can be named in the driver, regardless of the DMA engine.
        type I2sMasterErased<'d> = crate::dma::$any_channel<'d>;
    };
}

impl<'d> I2s<'d, crate::Blocking> {
    /// Construct a new I2S instance in TDM mode.
    pub fn new<I: Instance + 'd>(
        i2s: I,
        channel: impl I2sMasterDmaChannel<'d, I>,
        config: TdmConfig,
    ) -> Result<Self, ConfigError> {
        Self::new_internal(i2s, channel.into(), Config::Tdm(config))
    }

    /// Construct a new I2S instance in PDM mode (I2S0 only).
    #[cfg(any(i2s_supports_pdm_tx, i2s_supports_pdm_rx))]
    pub fn new_pdm<I: Instance + PdmInstance + 'd>(
        i2s: I,
        channel: impl I2sMasterDmaChannel<'d, I>,
        config: PdmConfig,
    ) -> Result<Self, ConfigError> {
        Self::new_internal(i2s, channel.into(), Config::Pdm(config))
    }
}

pub(crate) const I2S_LL_MCLK_DIVIDER_BIT_WIDTH: usize = property!("i2s.mclk_divider_bit_width");

pub(crate) const I2S_LL_MCLK_DIVIDER_MAX: usize = (1 << I2S_LL_MCLK_DIVIDER_BIT_WIDTH) - 1;

/// A structure representing a DMA transfer.
///
/// This structure holds references to the driver instance, DMA buffers, and
/// transfer status.
#[instability::unstable]
pub struct I2sTxDmaTransfer<'d, Dm, Buf>
where
    Dm: DriverMode,
    Buf: DmaTxBuffer,
{
    i2s_tx: ManuallyDrop<I2sTx<'d, Dm>>,
    buffer_view: ManuallyDrop<Buf::View>,
    completed: bool,
}

impl<'d, Dm, Buf> I2sTxDmaTransfer<'d, Dm, Buf>
where
    Dm: DriverMode,
    Buf: DmaTxBuffer,
{
    /// Returns true when [Self::wait] will not block.
    pub fn is_done(&self) -> bool {
        self.completed || self.i2s_tx.i2s.info().is_tx_done()
    }

    /// Waits for the transfer to finish and returns the peripheral and buffer.
    pub fn wait(self) -> (Result<(), DmaError>, I2sTx<'d, Dm>, Buf::Final) {
        while !self.is_done() {}
        let (i2s_tx, buf) = self.stop();

        if i2s_tx.tx_channel.has_error() {
            (Err(DmaError::DescriptorError), i2s_tx, buf)
        } else {
            (Ok(()), i2s_tx, buf)
        }
    }

    /// Immediately stop the transfer and return the peripheral and buffer.
    pub fn stop(mut self) -> (I2sTx<'d, Dm>, Buf::Final) {
        self.i2s_tx.tx_channel.stop_transfer();
        self.i2s_tx.i2s.info().tx_stop();

        let (i2s_tx, buf) = self.release();

        (i2s_tx, Buf::from_view(buf))
    }

    fn release(mut self) -> (I2sTx<'d, Dm>, Buf::View) {
        let (i2s_tx, buffer_view) = unsafe {
            (
                ManuallyDrop::take(&mut self.i2s_tx),
                ManuallyDrop::take(&mut self.buffer_view),
            )
        };

        core::mem::forget(self);
        (i2s_tx, buffer_view)
    }
}

impl<'d, Buf> I2sTxDmaTransfer<'d, Async, Buf>
where
    Buf: DmaTxBuffer,
{
    /// Waits for the transfer to finish and returns the peripheral and buffer.
    pub async fn wait_async(mut self) -> (Result<(), DmaError>, I2sTx<'d, Async>, Buf::Final) {
        if !self.completed {
            if let Err(err) = DmaTxFuture::new(&mut self.i2s_tx.tx_channel).await {
                self.i2s_tx.tx_channel.stop_transfer();
                self.i2s_tx.i2s.info().tx_stop();
                let (i2s_tx, buf) = self.release();
                return (Err(err), i2s_tx, Buf::from_view(buf));
            }
            while !self.is_done() {}
            self.completed = true;
        }

        let (i2s_tx, buf) = self.stop();

        if i2s_tx.tx_channel.has_error() {
            (Err(DmaError::DescriptorError), i2s_tx, buf)
        } else {
            (Ok(()), i2s_tx, buf)
        }
    }

    /// Waits for the transfer to finish.
    pub async fn wait_for_done_async(&mut self) -> Result<(), DmaError> {
        if !self.completed {
            DmaTxFuture::new(&mut self.i2s_tx.tx_channel).await?;
            while !self.is_done() {}
            self.completed = true;
        }
        Ok(())
    }

    /// Waits for a condition that might indicate more data is available.
    pub async fn wait_for_available_async(&mut self) -> Result<(), DmaError> {
        DmaTxFuture::new_with_config(
            &mut self.i2s_tx.tx_channel,
            enum_set!(DmaTxInterrupt::Eof),
            enum_set!(DmaTxInterrupt::DescriptorError | DmaTxInterrupt::TotalEof),
        )
        .await
    }
}

impl<Dm: DriverMode, BUF: DmaTxBuffer> core::ops::Deref for I2sTxDmaTransfer<'_, Dm, BUF> {
    type Target = BUF::View;

    fn deref(&self) -> &Self::Target {
        &self.buffer_view
    }
}

impl<Dm: DriverMode, BUF: DmaTxBuffer> core::ops::DerefMut for I2sTxDmaTransfer<'_, Dm, BUF> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.buffer_view
    }
}

impl<Dm: DriverMode, BUF: DmaTxBuffer> Drop for I2sTxDmaTransfer<'_, Dm, BUF> {
    fn drop(&mut self) {
        self.i2s_tx.tx_channel.stop_transfer();
        self.i2s_tx.i2s.info().tx_stop();

        // SAFETY: This is Drop, we know that the parts are no longer used
        let view = unsafe {
            ManuallyDrop::drop(&mut self.i2s_tx);
            ManuallyDrop::take(&mut self.buffer_view)
        };
        let _ = BUF::from_view(view);
    }
}

/// A structure representing a DMA transfer.
///
/// This structure holds references to the driver instance, DMA buffers, and
/// transfer status.
#[instability::unstable]
pub struct I2sRxDmaTransfer<'d, Dm, Buf>
where
    Dm: DriverMode,
    Buf: DmaRxBuffer,
{
    i2s_rx: ManuallyDrop<I2sRx<'d, Dm>>,
    buffer_view: ManuallyDrop<Buf::View>,
    completed: bool,
}

impl<'d, Dm, Buf> I2sRxDmaTransfer<'d, Dm, Buf>
where
    Dm: DriverMode,
    Buf: DmaRxBuffer,
{
    /// Returns true when [Self::wait] will not block.
    pub fn is_done(&self) -> bool {
        self.completed || self.i2s_rx.i2s.info().is_rx_done()
    }

    /// Waits for the transfer to finish and returns the peripheral and buffer.
    pub fn wait(self) -> (Result<(), DmaError>, I2sRx<'d, Dm>, Buf::Final) {
        while !self.is_done() {}

        let (i2s_rx, buf) = self.stop();

        if i2s_rx.rx_channel.has_error() {
            (Err(DmaError::DescriptorError), i2s_rx, buf)
        } else {
            (Ok(()), i2s_rx, buf)
        }
    }

    /// Immediately stop the transfer and return the peripheral and buffer.
    pub fn stop(mut self) -> (I2sRx<'d, Dm>, Buf::Final) {
        self.i2s_rx.i2s.info().rx_stop();
        self.i2s_rx.rx_channel.stop_transfer();

        let (i2s_rx, buf) = self.release();
        (i2s_rx, Buf::from_view(buf))
    }

    fn release(mut self) -> (I2sRx<'d, Dm>, Buf::View) {
        let (i2s_rx, buffer_view) = unsafe {
            (
                ManuallyDrop::take(&mut self.i2s_rx),
                ManuallyDrop::take(&mut self.buffer_view),
            )
        };

        core::mem::forget(self);
        (i2s_rx, buffer_view)
    }
}

impl<'d, Buf> I2sRxDmaTransfer<'d, Async, Buf>
where
    Buf: DmaRxBuffer,
{
    /// Waits for the transfer to finish and returns the peripheral and buffer.
    pub async fn wait_async(mut self) -> (Result<(), DmaError>, I2sRx<'d, Async>, Buf::Final) {
        if !self.completed {
            // we treat DescriptorEmpty as rx transfer is done
            if let Err(err) = DmaRxFuture::new_with_config(
                &mut self.i2s_rx.rx_channel,
                enum_set!(DmaRxInterrupt::DescriptorEmpty),
                enum_set!(DmaRxInterrupt::ErrorEof | DmaRxInterrupt::DescriptorError),
            )
            .await
            {
                self.completed = true;
                self.i2s_rx.i2s.info().rx_stop();
                self.i2s_rx.rx_channel.stop_transfer();
                let (i2s_rx, buf) = self.release();
                return (Err(err), i2s_rx, Buf::from_view(buf));
            }
            self.completed = true;
        }

        let (i2s_rx, buf) = self.stop();

        if i2s_rx.rx_channel.has_error() {
            (Err(DmaError::DescriptorError), i2s_rx, buf)
        } else {
            (Ok(()), i2s_rx, buf)
        }
    }

    /// Waits for the transfer to finish and returns the peripheral and buffer.
    pub async fn wait_for_done_async(&mut self) -> Result<(), DmaError> {
        if !self.completed {
            // we treat DescriptorEmpty as rx transfer is done
            DmaRxFuture::new_with_config(
                &mut self.i2s_rx.rx_channel,
                enum_set!(DmaRxInterrupt::DescriptorEmpty),
                enum_set!(DmaRxInterrupt::ErrorEof | DmaRxInterrupt::DescriptorError),
            )
            .await?;
            self.completed = true;
        }
        Ok(())
    }

    /// Waits for a condition that might indicate more data is available.
    pub async fn wait_for_available_async(&mut self) -> Result<(), DmaError> {
        DmaRxFuture::new_with_config(
            &mut self.i2s_rx.rx_channel,
            enum_set!(DmaRxInterrupt::SuccessfulEof | DmaRxInterrupt::Done),
            enum_set!(
                DmaRxInterrupt::ErrorEof
                    | DmaRxInterrupt::DescriptorError
                    | DmaRxInterrupt::DescriptorEmpty
            ),
        )
        .await
    }
}

impl<Dm: DriverMode, BUF: DmaRxBuffer> core::ops::Deref for I2sRxDmaTransfer<'_, Dm, BUF> {
    type Target = BUF::View;

    fn deref(&self) -> &Self::Target {
        &self.buffer_view
    }
}

impl<Dm: DriverMode, BUF: DmaRxBuffer> core::ops::DerefMut for I2sRxDmaTransfer<'_, Dm, BUF> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.buffer_view
    }
}

impl<Dm: DriverMode, BUF: DmaRxBuffer> Drop for I2sRxDmaTransfer<'_, Dm, BUF> {
    fn drop(&mut self) {
        self.i2s_rx.rx_channel.stop_transfer();
        self.i2s_rx.i2s.info().rx_stop();

        // SAFETY: This is Drop, we know that the parts are no longer used
        let view = unsafe {
            ManuallyDrop::drop(&mut self.i2s_rx);
            ManuallyDrop::take(&mut self.buffer_view)
        };
        let _ = BUF::from_view(view);
    }
}

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

/// Supported data formats
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg(not(i2s_version = "1"))]
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
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg(esp32s2)]
pub enum DataFormat {
    /// 32-bit data width and 32-bit channel width.
    Data32Channel32,
    /// 24-bit data width and 24-bit channel width.
    Data24Channel24,
    /// 16-bit data width and 16-bit channel width.
    Data16Channel16,
    /// 8-bit data width and 8-bit channel width.
    Data8Channel8,
}

/// Supported data formats
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg(esp32)]
pub enum DataFormat {
    /// 32-bit data width and 32-bit channel width.
    Data32Channel32,
    /// 16-bit data width and 16-bit channel width.
    Data16Channel16,
}

#[cfg(not(i2s_version = "1"))]
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

#[cfg(esp32s2)]
impl DataFormat {
    /// Returns the number of data bits for the selected data format.
    pub fn data_bits(&self) -> u8 {
        match self {
            DataFormat::Data32Channel32 => 32,
            DataFormat::Data24Channel24 => 24,
            DataFormat::Data16Channel16 => 16,
            DataFormat::Data8Channel8 => 8,
        }
    }

    /// Returns the number of channel bits for the selected data format.
    pub fn channel_bits(&self) -> u8 {
        match self {
            DataFormat::Data32Channel32 => 32,
            DataFormat::Data24Channel24 => 24,
            DataFormat::Data16Channel16 => 16,
            DataFormat::Data8Channel8 => 8,
        }
    }
}

#[cfg(esp32)]
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

/// I2S bit order
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum BitOrder {
    /// Most Significant Bit (MSB) is transmitted first.
    #[default]
    MsbFirst,
    /// Least Significant Bit (LSB) is transmitted first.
    LsbFirst,
}

/// I2S endianness
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Endianness {
    /// Most Significant Byte (MSB) is transmitted first.
    #[default]
    LittleEndian,
    /// Least Significant Byte (LSB) is transmitted first.
    BigEndian,
}

/// I2S word select signal width
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum WsWidth {
    /// Word select signal will be kept active for half of the frame
    #[default]
    HalfFrame,
    /// Word select signal will be kept active for the length of the first channel (PCM long frame
    /// standard)
    #[cfg(not(i2s_version = "1"))]
    OneChannel,
    /// Word select signal will be kept active for a single BCLK cycle (PCM short frame standard)
    Bit,
    /// Word select signal will be kept active for the specified amount of bits(BCLK cycles)
    #[cfg(not(i2s_version = "1"))]
    Bits(u16),
}

/// Represents the polarity of a signal
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Polarity {
    /// The signal is high when active
    #[default]
    ActiveHigh,
    /// The signal is low when active
    ActiveLow,
}

/// I2S channels configuration
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Channels {
    count: u8,
    mask: u16,
    fill: Option<u32>,
}

impl Channels {
    /// Two channels will use different data
    pub const STEREO: Channels = Channels::new_impl(2, 0b11, None);
    /// Two channels will use the same data
    pub const MONO: Channels = Channels::new_impl(2, 0b01, None);
    /// Two channels. Left(first) channel will contain data. Right(second) channel will contain
    /// zeros.
    pub const LEFT: Channels = Channels::new_impl(2, 0b01, Some(0));
    /// Two channels. Right(second) channel will contain data. Left(first) channel will contain
    /// zeros.
    pub const RIGHT: Channels = Channels::new_impl(2, 0b10, Some(0));

    #[procmacros::doc_replace]
    /// Creates arbitrary configuration for I2S channels.
    ///
    /// - `count` the total number of channels. Must be at least 1 and no more than 16.
    /// - `mask` determines which channels will be active, with the least significant bit
    ///   representing first channel. Setting the bit at the nth position means nth channel is
    ///   active. Inactive channels do not consume or write data in the DMA buffer.
    /// - `fill` determines the behavior of inactive channels. `Some(n)` will make all inactive
    ///   channel send out specified value, truncated to the channel width. `None` will make
    ///   disabled channels repeat the data from the last active channel. This field is ignored in
    ///   the receiver unit.
    ///
    /// ## Example
    ///
    /// The following example prepares configuration for 6 channels. Only 1st and 4th channels
    /// are active. Channels 2-3 will use the same data as the 1st, and channels 5-6 will use the
    /// data from the 4th.
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::i2s::master::Channels;
    ///
    /// let channels = Channels::new(6, 0b_001_001, None);
    /// # {after_snippet}
    /// ```
    #[cfg(not(i2s_version = "1"))]
    pub const fn new(count: u8, mask: u16, fill: Option<u32>) -> Self {
        Self::new_impl(count, mask, fill)
    }

    const fn new_impl(count: u8, mut mask: u16, fill: Option<u32>) -> Self {
        mask &= (1 << count) - 1;

        Self { count, mask, fill }
    }

    #[cfg(not(i2s_version = "1"))]
    fn active_count(&self) -> u8 {
        self.mask.count_ones() as u8
    }
}

/// Internal I2S peripheral configuration (TDM or PDM mode).
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(not(any(i2s_supports_pdm_tx, i2s_supports_pdm_rx)), derive(Eq, Hash))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub(crate) enum Config {
    /// Time-division multiplexed (TDM) configuration.
    Tdm(TdmConfig),
    /// Pulse-density modulation (PDM) configuration (I2S0 only).
    #[cfg(any(i2s_supports_pdm_tx, i2s_supports_pdm_rx))]
    Pdm(PdmConfig),
}

/// TDM mode peripheral configuration.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, procmacros::BuilderLite)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct TdmConfig {
    /// Receiver unit config
    rx_config: TdmUnitConfig,

    /// Transmitter unit config
    tx_config: TdmUnitConfig,

    /// Sets `I2S_SIG_LOOPBACK`: TX and RX share the same WS and BCK.
    signal_loopback: bool,

    /// The target sample rate
    #[cfg(i2s_version = "1")]
    sample_rate: Rate,

    /// Format of the data
    #[cfg(i2s_version = "1")]
    data_format: DataFormat,
}

impl Config {
    fn validate(&self, _info: &Info) -> Result<(), ConfigError> {
        match self {
            Self::Tdm(c) => c.validate(),
            #[cfg(any(i2s_supports_pdm_tx, i2s_supports_pdm_rx))]
            Self::Pdm(c) => c.validate(_info).map_err(ConfigError::Pdm),
        }
    }

    #[cfg(i2s_version = "1")]
    fn calculate_clock(&self) -> I2sClockDividers {
        match self {
            Self::Tdm(c) => c.calculate_clock(),
            #[cfg(any(i2s_supports_pdm_tx, i2s_supports_pdm_rx))]
            Self::Pdm(_) => unreachable!(),
        }
    }
}

impl TdmConfig {
    /// TDM Philips standard configuration with two 16-bit active channels.
    pub fn new_tdm_philips() -> Self {
        Self {
            rx_config: TdmUnitConfig::new_tdm_philips(),
            tx_config: TdmUnitConfig::new_tdm_philips(),
            ..Default::default()
        }
    }

    /// TDM MSB standard configuration with two 16-bit active channels.
    pub fn new_tdm_msb() -> Self {
        Self {
            rx_config: TdmUnitConfig::new_tdm_msb(),
            tx_config: TdmUnitConfig::new_tdm_msb(),
            ..Default::default()
        }
    }

    /// TDM PCM short frame standard configuration with two 16-bit active channels.
    pub fn new_tdm_pcm_short() -> Self {
        Self {
            rx_config: TdmUnitConfig::new_tdm_pcm_short(),
            tx_config: TdmUnitConfig::new_tdm_pcm_short(),
            ..Default::default()
        }
    }

    /// TDM PCM long frame standard configuration with two 16-bit active channels.
    #[cfg(not(i2s_version = "1"))]
    pub fn new_tdm_pcm_long() -> Self {
        Self {
            rx_config: TdmUnitConfig::new_tdm_pcm_long(),
            tx_config: TdmUnitConfig::new_tdm_pcm_long(),
            ..Default::default()
        }
    }

    fn validate(&self) -> Result<(), ConfigError> {
        self.rx_config.validate()?;
        self.tx_config.validate()?;
        Ok(())
    }

    #[cfg(i2s_version = "1")]
    fn calculate_clock(&self) -> I2sClockDividers {
        I2sClockDividers::new(self.sample_rate, 2, self.data_format.data_bits())
    }

    /// Assign the given value to the `sample_rate` field in both units.
    #[must_use]
    #[cfg(not(i2s_version = "1"))]
    pub fn with_sample_rate(self, sample_rate: Rate) -> Self {
        Self {
            rx_config: self.rx_config.with_sample_rate(sample_rate),
            tx_config: self.tx_config.with_sample_rate(sample_rate),
            ..self
        }
    }

    /// Assign the given value to the `channels` field in both units.
    #[must_use]
    pub fn with_channels(self, channels: Channels) -> Self {
        Self {
            rx_config: self.rx_config.with_channels(channels),
            tx_config: self.tx_config.with_channels(channels),
            ..self
        }
    }

    /// Assign the given value to the `data_format` field in both units.
    #[must_use]
    #[cfg(not(i2s_version = "1"))]
    pub fn with_data_format(self, data_format: DataFormat) -> Self {
        Self {
            rx_config: self.rx_config.with_data_format(data_format),
            tx_config: self.tx_config.with_data_format(data_format),
            ..self
        }
    }

    /// Assign the given value to the `ws_width` field in both units.
    #[must_use]
    pub fn with_ws_width(self, ws_width: WsWidth) -> Self {
        Self {
            rx_config: self.rx_config.with_ws_width(ws_width),
            tx_config: self.tx_config.with_ws_width(ws_width),
            ..self
        }
    }

    /// Assign the given value to the `ws_polarity` field in both units.
    #[must_use]
    pub fn with_ws_polarity(self, ws_polarity: Polarity) -> Self {
        Self {
            rx_config: self.rx_config.with_ws_polarity(ws_polarity),
            tx_config: self.tx_config.with_ws_polarity(ws_polarity),
            ..self
        }
    }

    /// Assign the given value to the `msb_shift` field in both units.
    #[must_use]
    pub fn with_msb_shift(self, msb_shift: bool) -> Self {
        Self {
            rx_config: self.rx_config.with_msb_shift(msb_shift),
            tx_config: self.tx_config.with_msb_shift(msb_shift),
            ..self
        }
    }

    /// Assign the given value to the `endianness` field in both units.
    #[cfg(not(esp32))]
    #[must_use]
    pub fn with_endianness(self, endianness: Endianness) -> Self {
        Self {
            rx_config: self.rx_config.with_endianness(endianness),
            tx_config: self.tx_config.with_endianness(endianness),
            ..self
        }
    }

    /// Assign the given value to the `bit_order` field in both units.
    #[cfg(not(i2s_version = "1"))]
    #[must_use]
    pub fn with_bit_order(self, bit_order: BitOrder) -> Self {
        Self {
            rx_config: self.rx_config.with_bit_order(bit_order),
            tx_config: self.tx_config.with_bit_order(bit_order),
            ..self
        }
    }
}

#[allow(clippy::derivable_impls)]
impl Default for TdmConfig {
    fn default() -> Self {
        Self {
            rx_config: TdmUnitConfig::new_tdm_philips(),
            tx_config: TdmUnitConfig::new_tdm_philips(),
            signal_loopback: false,
            #[cfg(i2s_version = "1")]
            sample_rate: Rate::from_hz(44100),
            #[cfg(i2s_version = "1")]
            data_format: DataFormat::Data16Channel16,
        }
    }
}

/// I2S receiver/transmitter unit configuration (TDM mode).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, procmacros::BuilderLite)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct TdmUnitConfig {
    /// The target sample rate
    #[cfg(not(i2s_version = "1"))]
    sample_rate: Rate,

    /// I2S channels configuration
    channels: Channels,

    /// Format of the data
    #[cfg(not(i2s_version = "1"))]
    data_format: DataFormat,

    /// Duration for which WS signal is kept active
    ws_width: WsWidth,

    /// Polarity of WS signal
    ws_polarity: Polarity,

    /// Data signal will lag by one bit relative to the WS signal
    msb_shift: bool,

    /// Byte order of the data
    #[cfg(not(esp32))]
    endianness: Endianness,

    /// Bit order of the data
    #[cfg(not(i2s_version = "1"))]
    bit_order: BitOrder,
}

/// Alias for [`TdmUnitConfig`] (TDM mode unit configuration).
pub type UnitConfig = TdmUnitConfig;

impl TdmUnitConfig {
    /// TDM Philips standard configuration with two 16-bit active channels
    pub fn new_tdm_philips() -> Self {
        Self {
            #[cfg(not(i2s_version = "1"))]
            sample_rate: Rate::from_hz(44100),
            channels: Channels::STEREO,
            #[cfg(not(i2s_version = "1"))]
            data_format: DataFormat::Data16Channel16,
            ws_width: WsWidth::HalfFrame,
            ws_polarity: Polarity::ActiveLow,
            msb_shift: true,
            #[cfg(not(esp32))]
            endianness: Endianness::LittleEndian,
            #[cfg(not(i2s_version = "1"))]
            bit_order: BitOrder::MsbFirst,
        }
    }

    /// TDM MSB standard configuration with two 16-bit active channels
    pub fn new_tdm_msb() -> Self {
        Self::new_tdm_philips().with_msb_shift(false)
    }

    /// TDM PCM short frame standard configuration with two 16-bit active channels
    pub fn new_tdm_pcm_short() -> Self {
        Self::new_tdm_philips()
            .with_ws_width(WsWidth::Bit)
            .with_ws_polarity(Polarity::ActiveHigh)
    }

    /// TDM PCM long frame standard configuration with two 16-bit active channels
    #[cfg(not(i2s_version = "1"))]
    pub fn new_tdm_pcm_long() -> Self {
        Self::new_tdm_philips()
            .with_ws_width(WsWidth::OneChannel)
            .with_ws_polarity(Polarity::ActiveHigh)
    }

    fn validate(&self) -> Result<(), ConfigError> {
        #[cfg(not(i2s_version = "1"))]
        if self.channels.active_count() == 0 || self.channels.count > 16 {
            return Err(ConfigError::ChannelsOutOfRange);
        }

        Ok(())
    }

    #[cfg(not(i2s_version = "1"))]
    fn calculate_ws_width(&self) -> Result<u16, ConfigError> {
        let ws_width = match self.ws_width {
            WsWidth::HalfFrame => {
                self.data_format.data_bits() as u16 * self.channels.count as u16 / 2
            }
            WsWidth::Bit => 1,
            WsWidth::OneChannel => self.data_format.data_bits() as u16,
            WsWidth::Bits(bits) => bits,
        };

        const MAX_WS_WIDTH: u16 = property!("i2s.max_ws_width");

        if !(1..=MAX_WS_WIDTH).contains(&ws_width)
            || ws_width > self.data_format.data_bits() as u16 * self.channels.count as u16
        {
            return Err(ConfigError::WsWidthOutOfRange);
        }

        Ok(ws_width)
    }

    #[cfg(not(i2s_version = "1"))]
    fn calculate_clock(&self) -> I2sClockDividers {
        I2sClockDividers::new(
            self.sample_rate,
            self.channels.count,
            self.data_format.data_bits(),
        )
    }
}

impl Default for TdmUnitConfig {
    fn default() -> Self {
        Self::new_tdm_philips()
    }
}

/// Configuration errors.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ConfigError {
    /// Provided [Channels] configuration has no active channels or has over 16 total channels
    #[cfg(not(i2s_version = "1"))]
    ChannelsOutOfRange,
    /// Requested WS signal width is out of range
    #[cfg(not(i2s_version = "1"))]
    WsWidthOutOfRange,
    /// PDM configuration error
    #[cfg(any(i2s_supports_pdm_tx, i2s_supports_pdm_rx))]
    Pdm(PdmError),
}

#[cfg(any(i2s_supports_pdm_tx, i2s_supports_pdm_rx))]
impl From<PdmError> for ConfigError {
    fn from(err: PdmError) -> Self {
        Self::Pdm(err)
    }
}

impl core::error::Error for ConfigError {}

impl core::fmt::Display for ConfigError {
    #[allow(unused_variables)]
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match *self {
            #[cfg(not(i2s_version = "1"))]
            ConfigError::ChannelsOutOfRange => {
                write!(
                    f,
                    "Provided channels configuration has no active channels or has over 16 total channels"
                )
            }
            #[cfg(not(i2s_version = "1"))]
            ConfigError::WsWidthOutOfRange => {
                write!(
                    f,
                    "The requested WS signal width is out of supported range (1..=128)"
                )
            }
            #[cfg(any(i2s_supports_pdm_tx, i2s_supports_pdm_rx))]
            ConfigError::Pdm(err) => write!(f, "{err}"),
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
        self.i2s_tx
            .i2s
            .info()
            .enable_listen(interrupts.into(), true);
    }

    /// Unlisten the given interrupts
    #[instability::unstable]
    pub fn unlisten(&mut self, interrupts: impl Into<EnumSet<I2sInterrupt>>) {
        // tx.i2s and rx.i2s is the same, we could use either one
        self.i2s_tx
            .i2s
            .info()
            .enable_listen(interrupts.into(), false);
    }

    /// Gets asserted interrupts
    #[instability::unstable]
    pub fn interrupts(&mut self) -> EnumSet<I2sInterrupt> {
        // tx.i2s and rx.i2s is the same, we could use either one
        self.i2s_tx.i2s.info().interrupts()
    }

    /// Resets asserted interrupts
    #[instability::unstable]
    pub fn clear_interrupts(&mut self, interrupts: impl Into<EnumSet<I2sInterrupt>>) {
        // tx.i2s and rx.i2s is the same, we could use either one
        self.i2s_tx.i2s.info().clear_interrupts(interrupts.into());
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
    #[cfg(i2s_driver_supported)]
    fn new_internal(
        i2s: impl Instance + 'd,
        channel: I2sMasterErased<'d>,
        config: Config,
    ) -> Result<Self, ConfigError> {
        let channel = Channel::new(channel);
        let i2s = i2s.degrade();
        channel.runtime_ensure_compatible(i2s.dma_peripheral());

        // on ESP32-C3 / ESP32-S3 and later RX and TX are independent and
        // could be configured totally independently but for now handle all
        // the targets the same and force same configuration for both, TX and RX

        // make sure the peripheral is enabled before configuring it
        let peripheral = i2s.info().peripheral;
        let rx_guard = PeripheralGuard::new(peripheral);
        let tx_guard = PeripheralGuard::new(peripheral);

        i2s.info().set_master();
        i2s.info().configure(&config)?;
        match &config {
            Config::Tdm(_) => {
                i2s.info().update_tx();
                i2s.info().update_rx();
            }
            #[cfg(any(i2s_supports_pdm_tx, i2s_supports_pdm_rx))]
            Config::Pdm(c) => {
                if c.tx.is_some() {
                    i2s.info().update_tx();
                }
                if c.rx.is_some() {
                    i2s.info().update_rx();
                }
            }
        }

        Ok(Self {
            i2s_rx: RxCreator {
                i2s: unsafe { i2s.clone_unchecked() },
                rx_channel: channel.rx,
                guard: rx_guard,
                #[cfg(i2s_version = "1")]
                data_format: match config {
                    Config::Tdm(c) => c.data_format,
                    #[cfg(any(i2s_supports_pdm_tx, i2s_supports_pdm_rx))]
                    Config::Pdm(_) => DataFormat::Data16Channel16,
                },
            },
            i2s_tx: TxCreator {
                i2s,
                tx_channel: channel.tx,
                guard: tx_guard,
                #[cfg(i2s_version = "1")]
                data_format: match config {
                    Config::Tdm(c) => c.data_format,
                    #[cfg(any(i2s_supports_pdm_tx, i2s_supports_pdm_rx))]
                    Config::Pdm(_) => DataFormat::Data16Channel16,
                },
            },
        })
    }

    /// Converts the I2S instance into async mode.
    pub fn into_async(self) -> I2s<'d, Async> {
        I2s {
            i2s_rx: RxCreator {
                i2s: self.i2s_rx.i2s,
                rx_channel: self.i2s_rx.rx_channel.into_async(),
                guard: self.i2s_rx.guard,
                #[cfg(i2s_version = "1")]
                data_format: self.i2s_rx.data_format,
            },
            i2s_tx: TxCreator {
                i2s: self.i2s_tx.i2s,
                tx_channel: self.i2s_tx.tx_channel.into_async(),
                guard: self.i2s_tx.guard,
                #[cfg(i2s_version = "1")]
                data_format: self.i2s_tx.data_format,
            },
        }
    }
}

#[cfg(esp32)]
mod esp32 {
    use super::*;
    use crate::gpio::OutputSignal;

    /// Pins that can be used as clock outputs.
    pub trait ClkPin<'d>: PeripheralOutput<'d> {
        #[doc(hidden)]
        fn signal(&self) -> OutputSignal;
    }

    // TODO: implementations should be generated. This is the same problem as SDIO pins -
    // alternate function without GPIO matrix option. S2 and S3 also have the CLK_OUTn functions,
    // although they don't seem to need the same mechanism.
    impl<'d> ClkPin<'d> for crate::peripherals::GPIO0<'d> {
        fn signal(&self) -> OutputSignal {
            OutputSignal::CLK_OUT1
        }
    }
    impl<'d> ClkPin<'d> for crate::peripherals::GPIO1<'d> {
        fn signal(&self) -> OutputSignal {
            OutputSignal::CLK_OUT3
        }
    }
    impl<'d> ClkPin<'d> for crate::peripherals::GPIO3<'d> {
        fn signal(&self) -> OutputSignal {
            OutputSignal::CLK_OUT2
        }
    }
}

#[cfg(esp32)]
pub use esp32::ClkPin;

impl<'d, Dm> I2s<'d, Dm>
where
    Dm: DriverMode,
{
    /// Configures the I2S peripheral to use a master clock (MCLK) output pin.
    #[cfg(not(esp32))]
    pub fn with_mclk(self, mclk: impl PeripheralOutput<'d>) -> Self {
        let mclk = mclk.into();

        mclk.apply_output_config(&OutputConfig::default());
        mclk.set_output_enable(true);

        self.i2s_tx.i2s.info().mclk.connect_to(&mclk);

        self
    }

    /// Configures the I2S peripheral to output its clock to a pin.
    #[cfg(esp32)]
    pub fn with_mclk(self, mclk: impl ClkPin<'d>) -> Self {
        use crate::{gpio::OutputSignal, peripherals::IO_MUX};

        let clk_signal = mclk.signal();

        let mclk = mclk.into();

        mclk.apply_output_config(&OutputConfig::default());
        mclk.set_output_enable(true);

        // We need to do two things:
        // - Configure the IO_MUX_PIN_CTRL register
        // - Select the correct pin function

        let selector = match self.i2s_rx.i2s.0 {
            super::any::Inner::I2s0(_) => 0x0,
            super::any::Inner::I2s1(_) => 0xF,
        };

        // Route the appropriate I2S clock output to the selected signal.
        IO_MUX::regs().pin_ctrl().modify(|_, w| unsafe {
            match clk_signal {
                OutputSignal::CLK_OUT1 => w.clk1().bits(selector),
                OutputSignal::CLK_OUT2 => w.clk2().bits(selector),
                OutputSignal::CLK_OUT3 => w.clk3().bits(selector),
                _ => unreachable!(),
            }
        });

        // Connect the clock signal to the selected pin.
        clk_signal.connect_to(&mclk);

        // I think it's okay to leave the configuration untouched when dropping the driver. We'll
        // gate the clock source, which should also stop the clock output. Reusing the pins will
        // remove the output signal assignment.

        self
    }
}

/// I2S TX channel
pub struct I2sTx<'d, Dm>
where
    Dm: DriverMode,
{
    i2s: AnyI2s<'d>,
    tx_channel: ChannelTx<Dm, <I2sMasterErased<'d> as DmaChannel>::Tx>,
    _guard: PeripheralGuard,
    #[cfg(i2s_version = "1")]
    data_format: DataFormat,
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
    /// Perform a DMA write.
    #[allow(clippy::type_complexity)]
    #[instability::unstable]
    pub fn write<TX: DmaTxBuffer>(
        mut self,
        mut buffer: TX,
    ) -> Result<I2sTxDmaTransfer<'d, Dm, TX>, (Error, Self, TX)> {
        self.i2s.info().reset_tx();
        let res = unsafe {
            self.tx_channel
                .prepare_transfer(self.i2s.dma_peripheral(), &mut buffer)
                .and_then(|_| self.tx_channel.start_transfer())
        };
        if let Err(err) = res {
            return Err((Error::DmaError(err), self, buffer));
        }

        self.i2s.info().tx_start();

        Ok(I2sTxDmaTransfer {
            i2s_tx: ManuallyDrop::new(self),
            buffer_view: ManuallyDrop::new(buffer.into_view()),
            completed: false,
        })
    }

    /// Change the I2S Tx unit configuration.
    pub fn apply_config(&mut self, tx_config: &UnitConfig) -> Result<(), ConfigError> {
        tx_config.validate()?;
        self.i2s.info().configure_tx(
            tx_config,
            #[cfg(i2s_version = "1")]
            self.data_format,
        )
    }
}

/// I2S RX channel
pub struct I2sRx<'d, Dm>
where
    Dm: DriverMode,
{
    i2s: AnyI2s<'d>,
    rx_channel: ChannelRx<Dm, <I2sMasterErased<'d> as DmaChannel>::Rx>,
    _guard: PeripheralGuard,
    #[cfg(i2s_version = "1")]
    data_format: DataFormat,
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
    /// Perform a DMA read.
    ///
    /// The number of read bytes might be less than the capacity of the provided buffer since the
    /// peripheral might not completely fill each descriptor's buffer.
    ///
    /// This will return a [I2sRxDmaTransfer]
    pub fn read<BUF>(
        mut self,
        mut buffer: BUF,
    ) -> Result<I2sRxDmaTransfer<'d, Dm, BUF>, (Error, Self, BUF)>
    where
        BUF: DmaRxBuffer,
    {
        // Reset RX unit and RX FIFO
        self.i2s.info().reset_rx();

        let res = unsafe {
            self.rx_channel
                .prepare_transfer(self.i2s.dma_peripheral(), &mut buffer)
                .and_then(|_| self.rx_channel.start_transfer())
        };
        if let Err(err) = res {
            return Err((Error::DmaError(err), self, buffer));
        }

        // start: set I2S_RX_START
        self.i2s.info().rx_start(usize::MAX); // really limited by exhausting DMA rx buffer

        Ok(I2sRxDmaTransfer {
            i2s_rx: ManuallyDrop::new(self),
            buffer_view: ManuallyDrop::new(buffer.into_view()),
            completed: false,
        })
    }

    /// Change the I2S Rx unit configuration.
    pub fn apply_config(&mut self, rx_config: &UnitConfig) -> Result<(), ConfigError> {
        rx_config.validate()?;
        self.i2s.info().configure_rx(
            rx_config,
            #[cfg(i2s_version = "1")]
            self.data_format,
        )
    }
}

/// A peripheral singleton compatible with the I2S master driver.
pub trait Instance: crate::private::Sealed + super::any::Degrade {
    /// Returns the peripheral data describing this instance.
    #[doc(hidden)]
    fn info(&self) -> &'static Info;
}

for_each_i2s! {
    (
        $instance:ident, $sys:ident, $mclk:ident,
        $bclk:ident, $ws:ident, $bclk_rx:ident, $ws_rx:ident,
        [$($dout:ident),+], [$($din:ident),+], $pdm_tx:literal, $pdm_rx:literal,
        $pcm2pdm:literal, $pdm2pcm:literal
    ) => {
        impl Instance for crate::peripherals::$instance<'_> {
            fn info(&self) -> &'static Info {
                static INFO: Info = Info {
                    register_block: crate::peripherals::$instance::PTR.cast::<RegisterBlock>(),
                    peripheral: crate::system::Peripheral::$sys,
                    // MCLK on ESP32 requires special handling, so it has no signal here.
                    #[cfg(not(esp32))]
                    mclk: OutputSignal::$mclk,
                    bclk: OutputSignal::$bclk,
                    ws: OutputSignal::$ws,
                    bclk_rx: OutputSignal::$bclk_rx,
                    ws_rx: OutputSignal::$ws_rx,
                    dout_lines: &[$(OutputSignal::$dout),+],
                    din_lines: &[$(InputSignal::$din),+],
                    pdm_tx: $pdm_tx,
                    pdm_rx: $pdm_rx,
                    pcm2pdm: $pcm2pdm,
                    pdm2pcm: $pdm2pcm,
                };
                &INFO
            }
        }
    };
}

impl Instance for AnyI2s<'_> {
    fn info(&self) -> &'static Info {
        match &self.0 {
            #[cfg(soc_has_i2s0)]
            AnyI2sInner::I2s0(i2s) => i2s.info(),
            #[cfg(soc_has_i2s1)]
            AnyI2sInner::I2s1(i2s) => i2s.info(),
            #[cfg(soc_has_i2s2)]
            AnyI2sInner::I2s2(i2s) => i2s.info(),
        }
    }
}

impl AnyI2s<'_> {
    delegate::delegate! {
        to match &self.0 {
            #[cfg(soc_has_i2s0)]
            AnyI2sInner::I2s0(i2s) => i2s,
            #[cfg(soc_has_i2s1)]
            AnyI2sInner::I2s1(i2s) => i2s,
            #[cfg(soc_has_i2s2)]
            AnyI2sInner::I2s2(i2s) => i2s,
        } {
            fn bind_peri_interrupt(&self, handler: InterruptHandler);
            fn disable_peri_interrupt_on_all_cores(&self);
        }
    }

    pub(super) fn set_interrupt_handler(&self, handler: InterruptHandler) {
        self.disable_peri_interrupt_on_all_cores();
        self.bind_peri_interrupt(handler);
    }
}

pub(crate) mod private {
    use super::*;

    pub struct TxCreator<'d, Dm>
    where
        Dm: DriverMode,
    {
        pub i2s: AnyI2s<'d>,
        pub tx_channel: ChannelTx<Dm, <I2sMasterErased<'d> as DmaChannel>::Tx>,
        pub(crate) guard: PeripheralGuard,
        #[cfg(i2s_version = "1")]
        pub(crate) data_format: DataFormat,
    }

    impl<'d, Dm> TxCreator<'d, Dm>
    where
        Dm: DriverMode,
    {
        pub fn build(self) -> I2sTx<'d, Dm> {
            let peripheral = self.i2s.info().peripheral;
            I2sTx {
                i2s: self.i2s,
                tx_channel: self.tx_channel,
                _guard: PeripheralGuard::new(peripheral),
                #[cfg(i2s_version = "1")]
                data_format: self.data_format,
            }
        }

        pub fn with_bclk(self, bclk: impl PeripheralOutput<'d>) -> Self {
            let bclk = bclk.into();

            bclk.apply_output_config(&OutputConfig::default());
            bclk.set_output_enable(true);

            self.i2s.info().bclk.connect_to(&bclk);

            self
        }

        pub fn with_ws(self, ws: impl PeripheralOutput<'d>) -> Self {
            let ws = ws.into();

            ws.apply_output_config(&OutputConfig::default());
            ws.set_output_enable(true);

            self.i2s.info().ws.connect_to(&ws);

            self
        }

        pub fn with_dout(self, dout: impl PeripheralOutput<'d>) -> Self {
            let dout = dout.into();

            dout.apply_output_config(&OutputConfig::default());
            dout.set_output_enable(true);

            unwrap!(self.i2s.info().dout(0)).connect_to(&dout);

            self
        }

        /// Connect the PDM clock pin (maps to the WS output signal).
        pub fn with_clk(self, clk: impl PeripheralOutput<'d>) -> Self {
            self.with_ws(clk)
        }

        /// Connect a second PDM TX data line (line 1, two-line DAC mode, HW v2+).
        #[cfg(all(i2s_supports_pdm_tx, not(i2s_version = "1")))]
        pub fn with_dout2(self, dout: impl PeripheralOutput<'d>) -> Result<Self, ConfigError> {
            let dout = dout.into();

            dout.apply_output_config(&OutputConfig::default());
            dout.set_output_enable(true);

            let signal = self.i2s.info().dout(1).ok_or(PdmError::InvalidLine)?;
            signal.connect_to(&dout);

            Ok(self)
        }
    }

    pub struct RxCreator<'d, Dm>
    where
        Dm: DriverMode,
    {
        pub i2s: AnyI2s<'d>,
        pub rx_channel: ChannelRx<Dm, <I2sMasterErased<'d> as DmaChannel>::Rx>,
        pub(crate) guard: PeripheralGuard,
        #[cfg(i2s_version = "1")]
        pub(crate) data_format: DataFormat,
    }

    impl<'d, Dm> RxCreator<'d, Dm>
    where
        Dm: DriverMode,
    {
        pub fn build(self) -> I2sRx<'d, Dm> {
            let peripheral = self.i2s.info().peripheral;
            I2sRx {
                i2s: self.i2s,
                rx_channel: self.rx_channel,
                _guard: PeripheralGuard::new(peripheral),
                #[cfg(i2s_version = "1")]
                data_format: self.data_format,
            }
        }

        pub fn with_bclk(self, bclk: impl PeripheralOutput<'d>) -> Self {
            let bclk = bclk.into();

            bclk.apply_output_config(&OutputConfig::default());
            bclk.set_output_enable(true);

            self.i2s.info().bclk_rx.connect_to(&bclk);

            self
        }

        pub fn with_ws(self, ws: impl PeripheralOutput<'d>) -> Self {
            let ws = ws.into();

            ws.apply_output_config(&OutputConfig::default());
            ws.set_output_enable(true);

            self.i2s.info().ws_rx.connect_to(&ws);

            self
        }

        pub fn with_din(self, din: impl PeripheralInput<'d>) -> Self {
            let din = din.into();

            din.apply_input_config(&InputConfig::default());
            din.set_input_enable(true);

            unwrap!(self.i2s.info().din(0)).connect_to(&din);

            self
        }

        /// Connect the PDM clock pin (maps to the WS output signal).
        pub fn with_clk(self, clk: impl PeripheralOutput<'d>) -> Self {
            self.with_ws(clk)
        }

        /// Connect a PDM RX data line (`line` 0..=`pdm_max_rx_lines`-1).
        #[cfg(i2s_supports_pdm_rx)]
        pub fn with_din_line(
            self,
            line: u8,
            din: impl PeripheralInput<'d>,
        ) -> Result<Self, ConfigError> {
            let din = din.into();

            din.apply_input_config(&InputConfig::default());
            din.set_input_enable(true);

            let signal = self.i2s.info().din(line).ok_or(PdmError::InvalidLine)?;
            signal.connect_to(&din);

            Ok(self)
        }
    }

    pub struct I2sClockDividers {
        pub(crate) mclk_divider: u32,
        pub(crate) bclk_divider: u32,
        pub(crate) denominator: u32,
        pub(crate) numerator: u32,
    }

    impl I2sClockDividers {
        pub fn new(sample_rate: Rate, channels: u8, data_bits: u8) -> I2sClockDividers {
            // this loosely corresponds to `i2s_std_calculate_clock` and
            // `i2s_ll_tx_set_mclk` in esp-idf
            //
            // main difference is we are using fixed-point arithmetic here

            // If data_bits is a power of two, use 256 as the mclk_multiple
            // If data_bits is 24, use 192 (24 * 8) as the mclk_multiple
            let mclk_multiple = if data_bits == 24 { 192 } else { 256 };
            let sclk = crate::soc::i2s_sclk_frequency();

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
                            numerator = (b / 10000) as u32;
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
}
