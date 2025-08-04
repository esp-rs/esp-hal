use core::mem::ManuallyDrop;

use crate::{
    Async,
    Blocking,
    DriverMode,
    dma::{
        Channel,
        DmaChannelFor,
        DmaEligible,
        DmaRxBuffer,
        DmaTxBuffer,
        asynch::{DmaRxFuture, DmaTxFuture},
    },
    into_internal,
    peripherals,
    uart,
    uart::{
        Uart,
        uhci::{AnyUhci, UhciInternal, normal::ConfigError::*},
    },
};

/// A configuration error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum ConfigError {
    /// chunk_limit is above 4095
    AboveReadLimit,
}

/// UHCI Configuration
#[derive(Debug, Clone, Copy, procmacros::BuilderLite)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct Config {
    /// If this is set to true UHCI will end the payload receiving process when UART has been in
    /// idle state.
    idle_eof: bool,
    /// If this is set to true UHCI decoder receiving payload data ends when the receiving
    /// byte count has reached the specified value (in len_eof).
    /// If this is set to false UHCI decoder receiving payload data is end when 0xc0 is received.
    len_eof: bool,
    /// The limit of how much to read in a single read call. It cannot be higher than the dma
    /// buffer size, otherwise uart/dma/uhci will freeze. It cannot exceed 4095 (12 bits), above
    /// this value it will simply also split the readings
    chunk_limit: u16,
}

impl Default for Config {
    fn default() -> Config {
        Config {
            idle_eof: true,
            len_eof: true,
            chunk_limit: 128, // This is the default in the register at boot
        }
    }
}

impl core::error::Error for ConfigError {}

impl core::fmt::Display for ConfigError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            ConfigError::AboveReadLimit => {
                write!(
                    f,
                    "The requested read limit is not possible. The max is 4095 (12 bits)"
                )
            }
        }
    }
}

#[instability::unstable]
impl<Dm> embassy_embedded_hal::SetConfig for Uhci<'_, Dm>
where
    Dm: DriverMode,
{
    type Config = Config;
    type ConfigError = ConfigError;

    fn set_config(&mut self, config: &Self::Config) -> Result<(), Self::ConfigError> {
        self.apply_config(config)
    }
}

/// todo
pub struct Uhci<'d, Dm>
where
    Dm: DriverMode,
{
    /// todo
    pub(crate) internal: UhciInternal<'d, Dm>,
}

impl<'d> Uhci<'d, Blocking> {
    /// todo
    pub fn new(
        uart: Uart<'d, Blocking>,
        uhci: peripherals::UHCI0<'static>,
        channel: impl DmaChannelFor<AnyUhci<'d>>,
    ) -> Self {
        let channel = Channel::new(channel.degrade());
        channel.runtime_ensure_compatible(&uhci);

        let self_uhci = UhciInternal {
            uart,
            uhci: uhci.into(),
            channel,
        };

        self_uhci.init();

        Self {
            internal: self_uhci,
        }
    }

    /// todo
    pub fn into_async(self) -> Uhci<'d, Async> {
        let internal = self.internal.into_async();
        Uhci { internal }
    }
}

impl<'d> Uhci<'d, Async> {
    /// todo
    pub async fn write<Buf: DmaTxBuffer>(
        mut self,
        mut tx_buffer: Buf,
    ) -> UhciDmaTxTransfer<'d, Async, Buf> {
        {
            unsafe {
                self.internal
                    .channel
                    .tx
                    .prepare_transfer(self.internal.uhci.dma_peripheral(), &mut tx_buffer)
                    .unwrap()
            };

            self.internal.channel.tx.start_transfer().unwrap();

            return UhciDmaTxTransfer::new(self, tx_buffer);
        }
    }

    /// todo
    pub async fn read<Buf: DmaRxBuffer>(
        mut self,
        mut rx_buffer: Buf,
    ) -> UhciDmaRxTransfer<'d, Async, Buf> {
        {
            unsafe {
                self.internal
                    .channel
                    .rx
                    .prepare_transfer(self.internal.uhci.dma_peripheral(), &mut rx_buffer)
                    .unwrap()
            };

            self.internal.channel.rx.start_transfer().unwrap();

            return UhciDmaRxTransfer::new(self, rx_buffer);
        }
    }

    /// todo
    pub fn into_blocking(self) -> Uhci<'d, Blocking> {
        Uhci {
            internal: self.internal.into_blocking(),
        }
    }
}

impl<'d, Dm: DriverMode> Uhci<'d, Dm> {
    into_internal!();

    /// todo
    pub fn apply_config(&mut self, config: &Config) -> Result<(), ConfigError> {
        let reg: &esp32c6::uhci0::RegisterBlock = self.internal.uhci.give_uhci().register_block();

        reg.conf0()
            .modify(|_, w| w.uart_idle_eof_en().bit(config.idle_eof));

        reg.conf0()
            .modify(|_, w| w.len_eof_en().bit(config.len_eof));

        if self.internal.set_chunk_limit(config.chunk_limit).is_err() {
            return Err(AboveReadLimit);
        }

        Ok(())
    }
}

// Based on SpiDmaTransfer
/// A structure representing a DMA transfer for UHCI/UART.
///
/// This structure holds references to the UHCI instance, DMA buffers, and
/// transfer status.
#[instability::unstable]
pub struct UhciDmaTxTransfer<'d, Dm, Buf>
where
    Dm: DriverMode,
    Buf: DmaTxBuffer,
{
    uhci: ManuallyDrop<Uhci<'d, Dm>>,
    dma_buf: ManuallyDrop<Buf::View>,
}

impl<'d, Buf: DmaTxBuffer> UhciDmaTxTransfer<'d, Async, Buf> {
    fn new(uhci: Uhci<'d, Async>, dma_buf: Buf) -> Self {
        Self {
            uhci: ManuallyDrop::new(uhci),
            dma_buf: ManuallyDrop::new(dma_buf.into_view()),
        }
    }

    /// todo
    pub fn is_done(&self) -> bool {
        self.uhci.internal.channel.tx.is_done()
    }

    async fn wait_for_idle(&mut self) {
        DmaTxFuture::new(&mut self.uhci.internal.channel.tx)
            .await
            .unwrap();
    }

    /// Waits for the DMA transfer to complete.
    ///
    /// This method blocks until the transfer is finished and returns the
    /// `Uhci` instance and the associated buffer.
    #[instability::unstable]
    pub async fn wait(mut self) -> (Uhci<'d, Async>, Buf::View) {
        self.wait_for_idle().await;

        let retval = unsafe {
            (
                ManuallyDrop::take(&mut self.uhci),
                ManuallyDrop::take(&mut self.dma_buf),
            )
        };
        core::mem::forget(self);
        retval
    }

    /// Cancels the DMA transfer.
    #[instability::unstable]
    pub fn cancel(mut self) -> (Uhci<'d, Async>, Buf::View) {
        if !self.uhci.internal.channel.rx.is_done() {
            self.uhci.internal.channel.rx.stop_transfer();
        }

        let retval = unsafe {
            (
                ManuallyDrop::take(&mut self.uhci),
                ManuallyDrop::take(&mut self.dma_buf),
            )
        };
        core::mem::forget(self);
        retval
    }
}

impl<Dm, Buf> Drop for UhciDmaTxTransfer<'_, Dm, Buf>
where
    Dm: DriverMode,
    Buf: DmaTxBuffer,
{
    fn drop(&mut self) {
        if !self.uhci.internal.channel.tx.is_done() {
            self.uhci.internal.channel.tx.stop_transfer();
            // TODO: Implement uhci drop

            unsafe {
                ManuallyDrop::drop(&mut self.uhci);
                ManuallyDrop::drop(&mut self.dma_buf);
            }
        }
    }
}

// Based on SpiDmaTransfer
/// A structure representing a DMA transfer for UHCI/UART.
///
/// This structure holds references to the UHCI instance, DMA buffers, and
/// transfer status.
#[instability::unstable]
pub struct UhciDmaRxTransfer<'d, Dm, Buf>
where
    Dm: DriverMode,
    Buf: DmaRxBuffer,
{
    uhci: ManuallyDrop<Uhci<'d, Dm>>,
    dma_buf: ManuallyDrop<Buf::View>,
}

impl<'d, Buf: DmaRxBuffer> UhciDmaRxTransfer<'d, Async, Buf> {
    fn new(uhci: Uhci<'d, Async>, dma_buf: Buf) -> Self {
        Self {
            uhci: ManuallyDrop::new(uhci),
            dma_buf: ManuallyDrop::new(dma_buf.into_view()),
        }
    }

    /// todo
    pub fn is_done(&self) -> bool {
        self.uhci.internal.channel.tx.is_done()
    }

    async fn wait_for_idle(&mut self) {
        DmaRxFuture::new(&mut self.uhci.internal.channel.rx)
            .await
            .unwrap();
    }

    /// Waits for the DMA transfer to complete.
    ///
    /// This method blocks until the transfer is finished and returns the
    /// `Uhci` instance and the associated buffer.
    #[instability::unstable]
    pub async fn wait(mut self) -> (Uhci<'d, Async>, Buf::View) {
        self.wait_for_idle().await;

        let retval = unsafe {
            (
                ManuallyDrop::take(&mut self.uhci),
                ManuallyDrop::take(&mut self.dma_buf),
            )
        };
        core::mem::forget(self);
        retval
    }

    /// Cancels the DMA transfer.
    #[instability::unstable]
    pub fn cancel(mut self) -> (Uhci<'d, Async>, Buf::View) {
        if !self.uhci.internal.channel.tx.is_done() {
            self.uhci.internal.channel.tx.stop_transfer();
        }

        let retval = unsafe {
            (
                ManuallyDrop::take(&mut self.uhci),
                ManuallyDrop::take(&mut self.dma_buf),
            )
        };
        core::mem::forget(self);
        retval
    }
}

impl<Dm, Buf> Drop for UhciDmaRxTransfer<'_, Dm, Buf>
where
    Dm: DriverMode,
    Buf: DmaRxBuffer,
{
    fn drop(&mut self) {
        if !self.uhci.internal.channel.rx.is_done() {
            self.uhci.internal.channel.rx.stop_transfer();
            // TODO: Implement uhci drop

            unsafe {
                ManuallyDrop::drop(&mut self.uhci);
                ManuallyDrop::drop(&mut self.dma_buf);
            }
        }
    }
}
