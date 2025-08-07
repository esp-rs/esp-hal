use core::{
    mem::ManuallyDrop,
    ops::{Deref, DerefMut}, u8,
};

use esp32c6::uhci0;

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
    peripherals,
    uart::{
        self,
        Uart,
        uhci::{AnyUhci, UhciInternal, normal::ConfigError::*},
    },
};

/// A configuration error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum ConfigError {
    /// chunk_limit is above 4095, this is not allowed (hardware limit)
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
            // This is the default in the register at boot, still should be changed!
            chunk_limit: 128,
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

/// "Normal" Uhci implementation, which implements regular dma transfers, can be expanded upon in
/// the future with Uhci specific features
pub struct Uhci<'d, Dm>
where
    Dm: DriverMode,
{
    pub(crate) internal: ManuallyDrop<Option<UhciInternal<'d, Dm>>>,
}

impl<'d> Uhci<'d, Blocking> {
    /// Creates a new instance of Uhci
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
            internal: ManuallyDrop::new(Some(self_uhci)),
        }
    }

    /// Create a new instance in [crate::Async] mode.
    pub fn into_async(mut self) -> Uhci<'d, Async> {
        let internal = self.internal.take().unwrap().into_async();
        Uhci {
            internal: ManuallyDrop::new(Some(internal)),
        }
    }
}

impl<Dm> Drop for Uhci<'_, Dm>
where
    Dm: DriverMode,
{
    fn drop(&mut self) {
        if let Some(mut internal) = self.internal.take() {
            internal.turn_off();
            // Drop uart too to be sure?
            drop(internal.uart);
        }
    }
}

impl<'d> Uhci<'d, Async> {
    /// Starts the write DMA transfer and returns the instance of UhciDmaTxTransfer
    pub async fn write<Buf: DmaTxBuffer>(
        mut self,
        mut tx_buffer: Buf,
    ) -> UhciDmaTxTransfer<'d, Async, Buf> {
        {
            let mut just_self = self.internal.take().unwrap();

            unsafe {
                just_self
                    .channel
                    .tx
                    .prepare_transfer(just_self.uhci.dma_peripheral(), &mut tx_buffer)
                    .unwrap()
            };

            just_self.channel.tx.start_transfer().unwrap();

            self.internal = ManuallyDrop::new(Some(just_self));

            return UhciDmaTxTransfer::new(self, tx_buffer);
        }
    }

    /// Starts the read DMA transfer and returns the instance of UhciDmaRxTransfer
    pub async fn read<Buf: DmaRxBuffer>(
        mut self,
        mut rx_buffer: Buf,
    ) -> UhciDmaRxTransfer<'d, Async, Buf> {
        {
            let mut just_self = self.internal.take().unwrap();

            unsafe {
                just_self
                    .channel
                    .rx
                    .prepare_transfer(just_self.uhci.dma_peripheral(), &mut rx_buffer)
                    .unwrap()
            };

            just_self.channel.rx.start_transfer().unwrap();

            self.internal = ManuallyDrop::new(Some(just_self));

            return UhciDmaRxTransfer::new(self, rx_buffer);
        }
    }

    /// Create a new instance in [crate::Blocking] mode.
    pub fn into_blocking(mut self) -> Uhci<'d, Blocking> {
        let internal = self.internal.take().unwrap().into_blocking();
        Uhci {
            internal: ManuallyDrop::new(Some(internal)),
        }
    }
}

impl<'d, Dm: DriverMode> Uhci<'d, Dm> {
    pub(crate) fn new_from_internal(uhci_internal: UhciInternal<'d, Dm>) -> Self {
        Self {
            internal: ManuallyDrop::new(Some(uhci_internal)),
        }
    }

    /// Sets the UART config for the consumer earlier uart
    pub fn set_uart_config(&mut self, uart_config: &uart::Config) -> Result<(), uart::ConfigError> {
        let mut just_self = self.internal.take().unwrap();

        let res = just_self.set_uart_config(uart_config);

        self.internal = ManuallyDrop::new(Some(just_self));
        res
    }

    /// Change the UHCI peripheral configuration
    pub fn apply_config(&mut self, config: &Config) -> Result<(), ConfigError> {
        let just_self = self.internal.take().unwrap();
        let reg: &uhci0::RegisterBlock = just_self.uhci.give_uhci().register_block();

        reg.conf0()
            .modify(|_, w| w.uart_idle_eof_en().bit(config.idle_eof));

        reg.conf0()
            .modify(|_, w| w.len_eof_en().bit(config.len_eof));

        if just_self.set_chunk_limit(config.chunk_limit).is_err() {
            return Err(AboveReadLimit);
        }

        self.internal = ManuallyDrop::new(Some(just_self));

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
    uhci: ManuallyDrop<UhciInternal<'d, Dm>>,
    dma_buf: ManuallyDrop<Buf::View>,
}

impl<'d, Buf: DmaTxBuffer> UhciDmaTxTransfer<'d, Async, Buf> {
    fn new(mut uhci: Uhci<'d, Async>, dma_buf: Buf) -> Self {
        let just_self = uhci.internal.take().unwrap();
        Self {
            uhci: ManuallyDrop::new(just_self),
            dma_buf: ManuallyDrop::new(dma_buf.into_view()),
        }
    }

    /// Returns true when [Self::wait] will not block.
    pub fn is_done(&self) -> bool {
        self.uhci.channel.tx.is_done()
    }

    /// to
    async fn wait_for_idle(&mut self) {
        self.uhci.uart.flush_async().await.unwrap();
        DmaTxFuture::new(&mut self.uhci.channel.tx).await.unwrap();
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
                Uhci::new_from_internal(ManuallyDrop::take(&mut self.uhci)),
                ManuallyDrop::take(&mut self.dma_buf),
            )
        };
        core::mem::forget(self);
        retval
    }

    /// Cancels the DMA transfer.
    #[instability::unstable]
    pub fn cancel(mut self) -> (Uhci<'d, Async>, Buf::View) {
        if !self.uhci.channel.tx.is_done() {
            self.uhci.channel.tx.stop_transfer();
        }

        let retval = unsafe {
            (
                Uhci::new_from_internal(ManuallyDrop::take(&mut self.uhci)),
                ManuallyDrop::take(&mut self.dma_buf),
            )
        };
        core::mem::forget(self);
        retval
    }
}

impl<Dm: DriverMode, Buf: DmaTxBuffer> Deref for UhciDmaTxTransfer<'_, Dm, Buf> {
    type Target = Buf::View;

    fn deref(&self) -> &Self::Target {
        &self.dma_buf
    }
}

impl<Dm: DriverMode, Buf: DmaTxBuffer> DerefMut for UhciDmaTxTransfer<'_, Dm, Buf> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.dma_buf
    }
}

impl<Dm, Buf> Drop for UhciDmaTxTransfer<'_, Dm, Buf>
where
    Dm: DriverMode,
    Buf: DmaTxBuffer,
{
    fn drop(&mut self) {
        if !self.uhci.channel.tx.is_done() {
            self.uhci.channel.tx.stop_transfer();
        }
        // TODO: Implement uhci drop

        unsafe {
            ManuallyDrop::drop(&mut self.uhci);
            ManuallyDrop::drop(&mut self.dma_buf);
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
    uhci: ManuallyDrop<UhciInternal<'d, Dm>>,
    dma_buf: ManuallyDrop<Buf::View>,
}

impl<'d, Buf: DmaRxBuffer> UhciDmaRxTransfer<'d, Async, Buf> {
    fn new(mut uhci: Uhci<'d, Async>, dma_buf: Buf) -> Self {
        Self {
            uhci: ManuallyDrop::new(uhci.internal.take().unwrap()),
            dma_buf: ManuallyDrop::new(dma_buf.into_view()),
        }
    }

    /// Returns true when [Self::wait] will not block.
    pub fn is_done(&self) -> bool {
        self.uhci.channel.rx.is_done()
    }

    async fn wait_for_idle(&mut self) {
        DmaRxFuture::new(&mut self.uhci.channel.rx).await.unwrap();
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
                Uhci::new_from_internal(ManuallyDrop::take(&mut self.uhci)),
                ManuallyDrop::take(&mut self.dma_buf),
            )
        };
        core::mem::forget(self);
        retval
    }

    /// Cancels the DMA transfer.
    #[instability::unstable]
    pub fn cancel(mut self) -> (Uhci<'d, Async>, Buf::View) {
        if !self.uhci.channel.rx.is_done() {
            self.uhci.channel.rx.stop_transfer();
        }

        let retval = unsafe {
            (
                Uhci::new_from_internal(ManuallyDrop::take(&mut self.uhci)),
                ManuallyDrop::take(&mut self.dma_buf),
            )
        };
        core::mem::forget(self);
        retval
    }
}

impl<Dm: DriverMode, Buf: DmaRxBuffer> Deref for UhciDmaRxTransfer<'_, Dm, Buf> {
    type Target = Buf::View;

    fn deref(&self) -> &Self::Target {
        &self.dma_buf
    }
}

impl<Dm: DriverMode, Buf: DmaRxBuffer> DerefMut for UhciDmaRxTransfer<'_, Dm, Buf> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.dma_buf
    }
}

impl<Dm, Buf> Drop for UhciDmaRxTransfer<'_, Dm, Buf>
where
    Dm: DriverMode,
    Buf: DmaRxBuffer,
{
    fn drop(&mut self) {
        if !self.uhci.channel.rx.is_done() {
            self.uhci.channel.rx.stop_transfer();
            // TODO: Implement uhci drop
        }
        unsafe {
            ManuallyDrop::drop(&mut self.uhci);
            ManuallyDrop::drop(&mut self.dma_buf);
        }
    }
}
