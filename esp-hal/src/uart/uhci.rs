use core::{
    mem::ManuallyDrop,
    ops::{Deref, DerefMut},
};

use embassy_embedded_hal::SetConfig;
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
        PeripheralDmaChannel,
        asynch::{DmaRxFuture, DmaTxFuture},
    },
    peripherals,
    uart::{self, Uart, uhci::Error::AboveReadLimit},
};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
/// Uhci specific errors
pub enum Error {
    /// set_chunk_limit() argument is above what's possible by the hardware. It cannot exceed 4095
    /// (12 bits), above this value it will simply also split the readings
    AboveReadLimit,
}

crate::any_peripheral! {
    pub peripheral AnyUhci<'d> {
        Uhci0(crate::peripherals::UHCI0<'d>),
    }
}

impl<'d> DmaEligible for AnyUhci<'d> {
    #[cfg(gdma)]
    type Dma = crate::dma::AnyGdmaChannel<'d>;

    fn dma_peripheral(&self) -> crate::dma::DmaPeripheral {
        match &self.0 {
            any::Inner::Uhci0(_) => crate::dma::DmaPeripheral::Uhci0,
        }
    }
}

impl AnyUhci<'_> {
    /// Unwraps the enum into the peripheral below
    pub fn give_uhci(&self) -> &peripherals::UHCI0<'_> {
        match &self.0 {
            any::Inner::Uhci0(x) => x,
        }
    }
}

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

/// todo
pub struct Uhci<'d, Dm>
where
    Dm: DriverMode,
{
    uart: Uart<'d, Dm>,
    uhci: AnyUhci<'static>,
    channel: Channel<Dm, PeripheralDmaChannel<AnyUhci<'d>>>,
}

impl<'d, Dm> Uhci<'d, Dm>
where
    Dm: DriverMode,
{
    fn init(&self) {
        self.clean_turn_on();
        self.reset();
        self.conf_uart();
    }

    fn clean_turn_on(&self) {
        // General conf registers
        let reg: &uhci0::RegisterBlock = &self.uhci.give_uhci().register_block();
        reg.conf0().modify(|_, w| w.clk_en().set_bit());
        reg.conf0().modify(|_, w| {
            unsafe { w.bits(0) };
            w.clk_en().set_bit()
        });
        reg.conf1().modify(|_, w| unsafe { w.bits(0) });

        // For TX
        reg.escape_conf().modify(|_, w| unsafe { w.bits(0) });
    }

    fn reset(&self) {
        let reg: &uhci0::RegisterBlock = self.uhci.give_uhci().register_block();
        reg.conf0().modify(|_, w| w.rx_rst().set_bit());
        reg.conf0().modify(|_, w| w.rx_rst().clear_bit());

        reg.conf0().modify(|_, w| w.tx_rst().set_bit());
        reg.conf0().modify(|_, w| w.tx_rst().clear_bit());
    }

    fn conf_uart(&self) {
        let reg: &uhci0::RegisterBlock = self.uhci.give_uhci().register_block();

        // Idk if there is a better way to check it, but it works
        match &self.uart.tx.uart.0 {
            super::any::Inner::Uart0(_) => {
                info!("Uhci will use uart0");
                reg.conf0().modify(|_, w| w.uart0_ce().set_bit());
            }
            super::any::Inner::Uart1(_) => {
                info!("Uhci will use uart1");
                reg.conf0().modify(|_, w| w.uart1_ce().set_bit());
            }
        }
    }

    #[allow(dead_code)]
    fn set_chunk_limit(&self, limit: u16) -> Result<(), Error> {
        let reg: &uhci0::RegisterBlock = self.uhci.give_uhci().register_block();
        // let val = reg.pkt_thres().read().pkt_thrs().bits();
        // info!("Read limit value: {} to set: {}", val, limit);

        // limit is 12 bits
        // Above this value, it will probably split the messages, anyway, the point is below (the
        // dma buffer length) it it will not freeze itself
        if limit > 4095 {
            return Err(AboveReadLimit);
        }

        reg.pkt_thres().write(|w| unsafe { w.bits(limit as u32) });
        Ok(())
    }

    /// todo
    pub fn set_uart_config(&mut self, uart_config: &uart::Config) -> Result<(), uart::ConfigError> {
        self.uart.set_config(uart_config)
    }

    /// todo
    pub fn apply_config(&mut self, config: &Config) -> Result<(), ConfigError> {
        let reg: &uhci0::RegisterBlock = self.uhci.give_uhci().register_block();

        reg.conf0()
            .modify(|_, w| w.uart_idle_eof_en().bit(config.idle_eof));

        reg.conf0()
            .modify(|_, w| w.len_eof_en().bit(config.len_eof));

        if self.set_chunk_limit(config.chunk_limit).is_err() {
            return Err(ConfigError::AboveReadLimit);
        }

        Ok(())
    }
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

        let uhci = Uhci {
            uart,
            uhci: uhci.into(),
            channel,
        };

        uhci.init();
        uhci
    }

    /// Create a new instance in [crate::Async] mode.
    pub fn into_async(self) -> Uhci<'d, Async> {
        Uhci {
            uart: self.uart.into_async(),
            uhci: self.uhci,
            channel: self.channel.into_async(),
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
            unsafe {
                self.channel
                    .tx
                    .prepare_transfer(self.uhci.dma_peripheral(), &mut tx_buffer)
                    .unwrap()
            };

            self.channel.tx.start_transfer().unwrap();

            return UhciDmaTxTransfer::new(self, tx_buffer);
        }
    }

    /// Starts the read DMA transfer and returns the instance of UhciDmaRxTransfer
    pub async fn read<Buf: DmaRxBuffer>(
        mut self,
        mut rx_buffer: Buf,
    ) -> UhciDmaRxTransfer<'d, Async, Buf> {
        {
            unsafe {
                self.channel
                    .rx
                    .prepare_transfer(self.uhci.dma_peripheral(), &mut rx_buffer)
                    .unwrap()
            };

            self.channel.rx.start_transfer().unwrap();

            return UhciDmaRxTransfer::new(self, rx_buffer);
        }
    }

    /// Create a new instance in [crate::Blocking] mode.
    pub fn into_blocking(self) -> Uhci<'d, Blocking> {
        Uhci {
            uart: self.uart.into_blocking(),
            uhci: self.uhci,
            channel: self.channel.into_blocking(),
        }
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

    /// Returns true when [Self::wait] will not block.
    pub fn is_done(&self) -> bool {
        self.uhci.channel.tx.is_done()
    }

    /// todo
    pub async fn wait_for_idle(&mut self) {
        // Workaround for an issue when it doesn't actually wait for the transfer to complete. I'm
        // lost at this point, this is the only thing that worked
        self.uhci.uart.tx.flush_async().await.unwrap();
        DmaTxFuture::new(&mut self.uhci.channel.tx).await.unwrap();
    }

    /// Waits for the DMA transfer to complete.
    ///
    /// This method blocks until the transfer is finished and returns the
    /// `Uhci` instance and the associated buffer.
    #[instability::unstable]
    pub async fn wait(mut self) -> (Uhci<'d, Async>, Buf::View) {
        self.wait_for_idle().await;
        self.uhci.channel.tx.stop_transfer();
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
    pub fn cancel(mut self) -> (Uhci<'d, Async>, Buf) {
        self.uhci.channel.tx.stop_transfer();

        let retval = unsafe {
            (
                ManuallyDrop::take(&mut self.uhci),
                DmaTxBuffer::from_view(ManuallyDrop::take(&mut self.dma_buf)),
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
        self.uhci.channel.tx.stop_transfer();

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

    /// Returns true when [Self::wait] will not block.
    pub fn is_done(&self) -> bool {
        self.uhci.channel.rx.is_done()
    }

    /// todo
    pub async fn wait_for_idle(&mut self) {
        DmaRxFuture::new(&mut self.uhci.channel.rx).await.unwrap();
    }

    /// Waits for the DMA transfer to complete.
    ///
    /// This method blocks until the transfer is finished and returns the
    /// `Uhci` instance and the associated buffer.
    #[instability::unstable]
    pub async fn wait(mut self) -> (Uhci<'d, Async>, Buf::View) {
        self.wait_for_idle().await;
        self.uhci.channel.rx.stop_transfer();

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
    pub fn cancel(mut self) -> (Uhci<'d, Async>, Buf) {
        self.uhci.channel.rx.stop_transfer();

        let retval = unsafe {
            (
                ManuallyDrop::take(&mut self.uhci),
                DmaRxBuffer::from_view(ManuallyDrop::take(&mut self.dma_buf)),
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
        self.uhci.channel.rx.stop_transfer();

        unsafe {
            ManuallyDrop::drop(&mut self.uhci);
            ManuallyDrop::drop(&mut self.dma_buf);
        }
    }
}
