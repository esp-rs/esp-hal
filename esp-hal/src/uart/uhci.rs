#![cfg_attr(docsrs, procmacros::doc_replace)]
//! ## Usage
//! ```rust, no_run
//! #![no_std]
//! #![no_main]
//!
//! #[panic_handler]
//! fn panic(_: &core::panic::PanicInfo) -> ! {
//!     loop {}
//! }
//!
//! use esp_hal::{
//!     clock::CpuClock,
//!     dma::{DmaRxBuf, DmaTxBuf},
//!     dma_buffers,
//!     main,
//!     rom::software_reset,
//!     uart,
//!     uart::{RxConfig, Uart, uhci, uhci::Uhci},
//! };
//!
//! #[main]
//! fn main() -> ! {
//!     let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
//!     let peripherals = esp_hal::init(config);
//!     let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
//!     let peripherals = esp_hal::init(config);
//!
//!     let config = uart::Config::default()
//!         .with_rx(RxConfig::default().with_fifo_full_threshold(64))
//!         .with_baudrate(115200);
//!
//!     let uart = Uart::new(peripherals.UART1, config)
//!         .unwrap()
//!         .with_tx(peripherals.GPIO2)
//!         .with_rx(peripherals.GPIO3);
//!
//!     let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(4092);
//!     let dma_rx = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
//!     let mut dma_tx = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();
//!
//!     let mut uhci = Uhci::new(uart, peripherals.UHCI0, peripherals.DMA_CH0);
//!     uhci.apply_config(&uhci::Config::default().with_chunk_limit(dma_rx.len() as u16))
//!         .unwrap();
//!
//!     let config = uart::Config::default()
//!         .with_rx(RxConfig::default().with_fifo_full_threshold(64))
//!         .with_baudrate(9600);
//!     uhci.set_uart_config(&config).unwrap();
//!
//!     let (uhci_rx, uhci_tx) = uhci.split();
//!     // Waiting for message
//!     let transfer = uhci_rx
//!         .read(dma_rx)
//!         .unwrap_or_else(|x| panic!("Something went horribly wrong: {:?}", x.0));
//!     let (err, _uhci_rx, dma_rx) = transfer.wait();
//!     err.unwrap();
//!
//!     let received = dma_rx.number_of_received_bytes();
//!     // println!("Received dma bytes: {}", received);
//!
//!     let rec_slice = &dma_rx.as_slice()[0..received];
//!     if received > 0 {
//!         match core::str::from_utf8(&rec_slice) {
//!             Ok(x) => {
//!                 // println!("Received DMA message: \"{}\"", x);
//!                 dma_tx.as_mut_slice()[0..received].copy_from_slice(&rec_slice);
//!                 dma_tx.set_length(received);
//!                 let transfer = uhci_tx
//!                     .write(dma_tx)
//!                     .unwrap_or_else(|x| panic!("Something went horribly wrong: {:?}", x.0));
//!                 let (err, _uhci, _dma_tx) = transfer.wait();
//!                 err.unwrap();
//!             }
//!             Err(x) => panic!("Error string: {}", x),
//!         }
//!     }
//!     software_reset()
//! }
//! ```

use core::{
    mem::ManuallyDrop,
    ops::{Deref, DerefMut},
};

use embassy_embedded_hal::SetConfig;

use crate::{
    Async,
    Blocking,
    DriverMode,
    dma::{
        AnyGdmaRxChannel,
        AnyGdmaTxChannel,
        Channel,
        ChannelRx,
        ChannelTx,
        DmaChannelFor,
        DmaEligible,
        DmaError,
        DmaRxBuffer,
        DmaTxBuffer,
        PeripheralDmaChannel,
        asynch::{DmaRxFuture, DmaTxFuture},
    },
    pac::uhci0,
    peripherals,
    uart::{self, TxError, Uart, UartRx, UartTx, uhci::Error::AboveReadLimit},
};

#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
/// Uhci specific errors
pub enum Error {
    /// set_chunk_limit() argument is above what's possible by the hardware. It cannot exceed 4095
    /// (12 bits), above this value it will simply also split the readings
    AboveReadLimit,
    /// DMA originating error
    Dma(DmaError),
    /// UART Tx originating error
    Tx(TxError),
}

impl From<DmaError> for Error {
    fn from(value: DmaError) -> Self {
        Error::Dma(value)
    }
}

impl From<TxError> for Error {
    fn from(value: TxError) -> Self {
        Error::Tx(value)
    }
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
    /// Opens the enum into the peripheral below
    fn give_uhci(&self) -> &peripherals::UHCI0<'_> {
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

/// UHCI (To use with UART over DMA)
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
        let reg: &uhci0::RegisterBlock = self.uhci.give_uhci().register_block();
        reg.conf0().modify(|_, w| w.clk_en().set_bit());
        reg.conf0().write(|w| {
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

    /// Sets the config the the consumed UART
    pub fn set_uart_config(&mut self, uart_config: &uart::Config) -> Result<(), uart::ConfigError> {
        self.uart.set_config(uart_config)
    }

    /// Sets the config to the UHCI peripheral
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

    /// Split the Uhci into UhciRx and UhciTx
    pub fn split(self) -> (UhciRx<'d, Dm>, UhciTx<'d, Dm>) {
        let (uart_rx, uart_tx) = self.uart.split();
        (
            UhciRx {
                uhci: unsafe { self.uhci.clone_unchecked() },
                uart_rx,
                channel_rx: self.channel.rx,
            },
            UhciTx {
                uhci: self.uhci,
                uart_tx,
                channel_tx: self.channel.tx,
            },
        )
    }
}

impl<'d> Uhci<'d, Blocking> {
    /// Creates a new instance of UHCI
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
    /// Create a new instance in [crate::Blocking] mode.
    pub fn into_blocking(self) -> Uhci<'d, Blocking> {
        Uhci {
            uart: self.uart.into_blocking(),
            uhci: self.uhci,
            channel: self.channel.into_blocking(),
        }
    }
}

/// Splitted Uhci structs, Tx part for sending data
pub struct UhciTx<'d, Dm>
where
    Dm: DriverMode,
{
    uhci: AnyUhci<'static>,
    uart_tx: UartTx<'d, Dm>,
    channel_tx: ChannelTx<Dm, AnyGdmaTxChannel<'d>>,
}

impl<'d, Dm> UhciTx<'d, Dm>
where
    Dm: DriverMode,
{
    /// Starts the write DMA transfer and returns the instance of UhciDmaTxTransfer
    pub fn write<Buf: DmaTxBuffer>(
        mut self,
        mut tx_buffer: Buf,
    ) -> Result<UhciDmaTxTransfer<'d, Dm, Buf>, (Error, Self, Buf)> {
        let res = unsafe {
            self.channel_tx
                .prepare_transfer(self.uhci.dma_peripheral(), &mut tx_buffer)
        };
        if let Err(err) = res {
            return Err((err.into(), self, tx_buffer));
        }

        let res = self.channel_tx.start_transfer();
        if let Err(err) = res {
            return Err((err.into(), self, tx_buffer));
        }

        Ok(UhciDmaTxTransfer::new(self, tx_buffer))
    }
}

/// Splitted Uhci structs, Rx part for receiving data
pub struct UhciRx<'d, Dm>
where
    Dm: DriverMode,
{
    uhci: AnyUhci<'static>,
    #[allow(dead_code)]
    uart_rx: UartRx<'d, Dm>,
    channel_rx: ChannelRx<Dm, AnyGdmaRxChannel<'d>>,
}

impl<'d, Dm> UhciRx<'d, Dm>
where
    Dm: DriverMode,
{
    /// Starts the read DMA transfer and returns the instance of UhciDmaRxTransfer
    pub fn read<Buf: DmaRxBuffer>(
        mut self,
        mut rx_buffer: Buf,
    ) -> Result<UhciDmaRxTransfer<'d, Dm, Buf>, (Error, Self, Buf)> {
        {
            let res = unsafe {
                self.channel_rx
                    .prepare_transfer(self.uhci.dma_peripheral(), &mut rx_buffer)
            };
            if let Err(err) = res {
                return Err((err.into(), self, rx_buffer));
            }

            let res = self.channel_rx.start_transfer();
            if let Err(err) = res {
                return Err((err.into(), self, rx_buffer));
            }

            Ok(UhciDmaRxTransfer::new(self, rx_buffer))
        }
    }
}

/// A structure representing a DMA transfer for UHCI/UART.
///
/// This structure holds references to the UHCI instance, DMA buffers, and
/// transfer status.
pub struct UhciDmaTxTransfer<'d, Dm, Buf>
where
    Dm: DriverMode,
    Buf: DmaTxBuffer,
{
    uhci: ManuallyDrop<UhciTx<'d, Dm>>,
    dma_buf: ManuallyDrop<Buf::View>,
    done: bool,
    saved_err: Result<(), Error>,
}

impl<'d, Buf: DmaTxBuffer, Dm: DriverMode> UhciDmaTxTransfer<'d, Dm, Buf> {
    fn new(uhci: UhciTx<'d, Dm>, dma_buf: Buf) -> Self {
        Self {
            uhci: ManuallyDrop::new(uhci),
            dma_buf: ManuallyDrop::new(dma_buf.into_view()),
            done: false,
            saved_err: Ok(()),
        }
    }

    /// Returns true when [Self::wait] will not block.
    pub fn is_done(&self) -> bool {
        self.uhci.channel_tx.is_done()
    }

    /// Cancels the DMA transfer.
    pub fn cancel(mut self) -> (UhciTx<'d, Dm>, Buf::Final) {
        self.uhci.channel_tx.stop_transfer();

        let retval = unsafe {
            (
                ManuallyDrop::take(&mut self.uhci),
                Buf::from_view(ManuallyDrop::take(&mut self.dma_buf)),
            )
        };
        core::mem::forget(self);
        retval
    }

    /// Waits for the DMA transfer to complete.
    ///
    /// This method blocks until the transfer is finished and returns the
    /// `Uhci` instance and the associated buffer.
    pub fn wait(mut self) -> (Result<(), Error>, UhciTx<'d, Dm>, Buf::Final) {
        if let Err(err) = self.saved_err {
            return (
                Err(err),
                unsafe { ManuallyDrop::take(&mut self.uhci) },
                unsafe { Buf::from_view(ManuallyDrop::take(&mut self.dma_buf)) },
            );
        }

        if !self.done {
            let res = self.uhci.uart_tx.flush();
            if let Err(err) = res {
                return (
                    Err(err.into()),
                    unsafe { ManuallyDrop::take(&mut self.uhci) },
                    unsafe { Buf::from_view(ManuallyDrop::take(&mut self.dma_buf)) },
                );
            }

            while !self.is_done() {}
        }

        self.uhci.channel_tx.stop_transfer();
        let retval = unsafe {
            (
                Result::<(), Error>::Ok(()),
                ManuallyDrop::take(&mut self.uhci),
                Buf::from_view(ManuallyDrop::take(&mut self.dma_buf)),
            )
        };
        core::mem::forget(self);
        retval
    }
}

impl<'d, Buf: DmaTxBuffer> UhciDmaTxTransfer<'d, Async, Buf> {
    /// Waits for the DMA transfer to complete, but async. After that, you still need to wait()
    pub async fn wait_for_done(&mut self) {
        // Workaround for an issue when it doesn't actually wait for the transfer to complete. I'm
        // lost at this point, this is the only thing that worked
        let res = self.uhci.uart_tx.flush_async().await;
        if let Err(err) = res {
            self.saved_err = Err(err.into());
            return;
        }

        let res = DmaTxFuture::new(&mut self.uhci.channel_tx).await;
        if let Err(err) = res {
            self.saved_err = Err(err.into());
            return;
        }

        self.done = true;
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
        self.uhci.channel_tx.stop_transfer();

        unsafe {
            ManuallyDrop::drop(&mut self.uhci);
            drop(Buf::from_view(ManuallyDrop::take(&mut self.dma_buf)));
        }
    }
}

/// A structure representing a DMA transfer for UHCI/UART.
///
/// This structure holds references to the UHCI instance, DMA buffers, and
/// transfer status.
pub struct UhciDmaRxTransfer<'d, Dm, Buf>
where
    Dm: DriverMode,
    Buf: DmaRxBuffer,
{
    uhci: ManuallyDrop<UhciRx<'d, Dm>>,
    dma_buf: ManuallyDrop<Buf::View>,
    done: bool,
    saved_err: Result<(), Error>,
}

impl<'d, Buf: DmaRxBuffer, Dm: DriverMode> UhciDmaRxTransfer<'d, Dm, Buf> {
    fn new(uhci: UhciRx<'d, Dm>, dma_buf: Buf) -> Self {
        Self {
            uhci: ManuallyDrop::new(uhci),
            dma_buf: ManuallyDrop::new(dma_buf.into_view()),
            done: false,
            saved_err: Ok(()),
        }
    }

    /// Returns true when [Self::wait] will not block.
    pub fn is_done(&self) -> bool {
        self.uhci.channel_rx.is_done()
    }

    /// Cancels the DMA transfer.
    pub fn cancel(mut self) -> (UhciRx<'d, Dm>, Buf::Final) {
        self.uhci.channel_rx.stop_transfer();

        let retval = unsafe {
            (
                ManuallyDrop::take(&mut self.uhci),
                Buf::from_view(ManuallyDrop::take(&mut self.dma_buf)),
            )
        };
        core::mem::forget(self);
        retval
    }

    /// Waits for the DMA transfer to complete.
    ///
    /// This method blocks until the transfer is finished and returns the
    /// `Uhci` instance and the associated buffer.
    pub fn wait(mut self) -> (Result<(), Error>, UhciRx<'d, Dm>, Buf::Final) {
        if let Err(err) = self.saved_err {
            return (
                Err(err),
                unsafe { ManuallyDrop::take(&mut self.uhci) },
                unsafe { Buf::from_view(ManuallyDrop::take(&mut self.dma_buf)) },
            );
        }

        if !self.done {
            while !self.is_done() {}
        }
        self.uhci.channel_rx.stop_transfer();

        let retval = unsafe {
            (
                Result::<(), Error>::Ok(()),
                ManuallyDrop::take(&mut self.uhci),
                Buf::from_view(ManuallyDrop::take(&mut self.dma_buf)),
            )
        };
        core::mem::forget(self);
        retval
    }
}

impl<'d, Buf: DmaRxBuffer> UhciDmaRxTransfer<'d, Async, Buf> {
    /// Waits for the DMA transfer to complete, but async. After that, you still need to wait()
    pub async fn wait_for_done(&mut self) {
        let res = DmaRxFuture::new(&mut self.uhci.channel_rx).await;
        if let Err(err) = res {
            self.saved_err = Err(err.into());
            return;
        }

        self.done = true;
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
        self.uhci.channel_rx.stop_transfer();

        unsafe {
            ManuallyDrop::drop(&mut self.uhci);
            drop(Buf::from_view(ManuallyDrop::take(&mut self.dma_buf)));
        }
    }
}
