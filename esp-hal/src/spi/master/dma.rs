use core::{
    cell::Cell,
    cmp::min,
    mem::ManuallyDrop,
    sync::atomic::{Ordering, fence},
};

#[cfg(feature = "unstable")]
use embedded_hal::spi::{ErrorType, SpiBus};

use super::*;
use crate::{
    dma::{
        Channel,
        DmaChannelFor,
        DmaEligible,
        DmaRxBuf,
        DmaRxBuffer,
        DmaTxBuf,
        DmaTxBuffer,
        EmptyBuf,
        PeripheralDmaChannel,
        asynch::DmaRxFuture,
    },
    private::DropGuard,
    spi::DmaError,
};

const MAX_DMA_SIZE: usize = 32736;

impl<'d> Spi<'d, Blocking> {
    #[doc_replace(
        "dma_channel" => {
            cfg(any(esp32, esp32s2)) => "DMA_SPI2",
            _ => "DMA_CH0",
        }
    )]
    /// Configures the SPI instance to use DMA with the specified channel.
    ///
    /// This method prepares the SPI instance for DMA transfers using SPI
    /// and returns an instance of `SpiDma` that supports DMA
    /// operations.
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::{
    ///     dma::{DmaRxBuf, DmaTxBuf},
    ///     dma_buffers,
    ///     spi::{
    ///         Mode,
    ///         master::{Config, Spi},
    ///     },
    /// };
    /// let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(32000);
    ///
    /// let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer)?;
    /// let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer)?;
    ///
    /// let mut spi = Spi::new(
    ///     peripherals.SPI2,
    ///     Config::default()
    ///         .with_frequency(Rate::from_khz(100))
    ///         .with_mode(Mode::_0),
    /// )?
    /// .with_dma(peripherals.__dma_channel__)
    /// .with_buffers(dma_rx_buf, dma_tx_buf);
    /// # {after_snippet}
    /// ```
    #[instability::unstable]
    pub fn with_dma(self, channel: impl DmaChannelFor<AnySpi<'d>>) -> SpiDma<'d, Blocking> {
        SpiDma::new(self, channel.degrade())
    }
}

#[doc_replace(
    "dma_channel" => {
        cfg(any(esp32, esp32s2)) => "DMA_SPI2",
        _ => "DMA_CH0",
    }
)]
/// A DMA capable SPI instance.
///
/// Using `SpiDma` is not recommended unless you wish
/// to manage buffers yourself. It's recommended to use
/// [`SpiDmaBus`] via `with_buffers` to get access
/// to a DMA capable SPI bus that implements the
/// embedded-hal traits.
/// ```rust, no_run
/// # {before_snippet}
/// use esp_hal::{
///     dma::{DmaRxBuf, DmaTxBuf},
///     dma_buffers,
///     spi::{
///         Mode,
///         master::{Config, Spi},
///     },
/// };
/// let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(32000);
///
/// let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer)?;
/// let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer)?;
///
/// let mut spi = Spi::new(
///     peripherals.SPI2,
///     Config::default()
///         .with_frequency(Rate::from_khz(100))
///         .with_mode(Mode::_0),
/// )?
/// .with_dma(peripherals.__dma_channel__)
/// .with_buffers(dma_rx_buf, dma_tx_buf);
/// #
/// # {after_snippet}
/// ```
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SpiDma<'d, Dm>
where
    Dm: DriverMode,
{
    spi: SpiWrapper<'d>,
    pub(crate) channel: Channel<Dm, PeripheralDmaChannel<AnySpi<'d>>>,
    #[cfg(all(esp32, spi_address_workaround))]
    address_buffer: DmaTxBuf,
}

impl<Dm> crate::private::Sealed for SpiDma<'_, Dm> where Dm: DriverMode {}

impl<'d> SpiDma<'d, Blocking> {
    /// Converts the SPI instance into async mode.
    #[instability::unstable]
    pub fn into_async(self) -> SpiDma<'d, Async> {
        self.spi
            .set_interrupt_handler(self.spi.info().async_handler);
        SpiDma {
            spi: self.spi,
            channel: self.channel.into_async(),
            #[cfg(all(esp32, spi_address_workaround))]
            address_buffer: self.address_buffer,
        }
    }

    pub(super) fn new(
        spi_driver: Spi<'d, Blocking>,
        channel: PeripheralDmaChannel<AnySpi<'d>>,
    ) -> Self {
        let spi = spi_driver.spi;

        let channel = Channel::new(channel);
        channel.runtime_ensure_compatible(&spi.spi);
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
                crate::dma::as_mut_byte_array!(BUFFERS[id], 4)
            ))
        };

        let (_info, state) = spi.spi.dma_parts();

        state.tx_transfer_in_progress.set(false);
        state.rx_transfer_in_progress.set(false);

        Self {
            spi,
            channel,
            #[cfg(all(esp32, spi_address_workaround))]
            address_buffer,
        }
    }

    /// Listen for the given interrupts
    #[instability::unstable]
    pub fn listen(&mut self, interrupts: impl Into<EnumSet<SpiInterrupt>>) {
        self.driver().enable_listen(interrupts.into(), true);
    }

    /// Unlisten the given interrupts
    #[instability::unstable]
    pub fn unlisten(&mut self, interrupts: impl Into<EnumSet<SpiInterrupt>>) {
        self.driver().enable_listen(interrupts.into(), false);
    }

    /// Gets asserted interrupts
    #[instability::unstable]
    pub fn interrupts(&mut self) -> EnumSet<SpiInterrupt> {
        self.driver().interrupts()
    }

    /// Resets asserted interrupts
    #[instability::unstable]
    pub fn clear_interrupts(&mut self, interrupts: impl Into<EnumSet<SpiInterrupt>>) {
        self.driver().clear_interrupts(interrupts.into());
    }

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
    ///
    /// # Panics
    ///
    /// Panics if passed interrupt handler is invalid (e.g. has priority
    /// `None`)
    #[instability::unstable]
    pub fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        self.spi.set_interrupt_handler(handler);
    }
}

impl<'d> SpiDma<'d, Async> {
    /// Converts the SPI instance into async mode.
    #[instability::unstable]
    pub fn into_blocking(self) -> SpiDma<'d, Blocking> {
        self.spi.disable_peri_interrupt_on_all_cores();
        SpiDma {
            spi: self.spi,
            channel: self.channel.into_blocking(),
            #[cfg(all(esp32, spi_address_workaround))]
            address_buffer: self.address_buffer,
        }
    }

    async fn wait_for_idle_async(&mut self) {
        if self.dma_driver().state.rx_transfer_in_progress.get() {
            _ = DmaRxFuture::new(&mut self.channel.rx).await;
            self.dma_driver().state.rx_transfer_in_progress.set(false);
        }

        struct Fut(Driver);
        impl Fut {
            const DONE_EVENTS: EnumSet<SpiInterrupt> =
                enumset::enum_set!(SpiInterrupt::TransferDone);
        }
        impl Future for Fut {
            type Output = ();

            fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
                if !self.0.interrupts().is_disjoint(Self::DONE_EVENTS) {
                    #[cfg(any(esp32, esp32s2))]
                    // Need to poll for done-ness even after interrupt fires.
                    if self.0.busy() {
                        cx.waker().wake_by_ref();
                        return Poll::Pending;
                    }

                    self.0.clear_interrupts(Self::DONE_EVENTS);
                    return Poll::Ready(());
                }

                self.0.state.waker.register(cx.waker());
                self.0.enable_listen(Self::DONE_EVENTS, true);
                Poll::Pending
            }
        }
        impl Drop for Fut {
            fn drop(&mut self) {
                self.0.enable_listen(Self::DONE_EVENTS, false);
            }
        }

        if !self.is_done() {
            Fut(self.driver()).await;
        }

        if self.dma_driver().state.tx_transfer_in_progress.get() {
            // In case DMA TX buffer is bigger than what the SPI consumes, stop the DMA.
            if !self.channel.tx.is_done() {
                self.channel.tx.stop_transfer();
            }
            self.dma_driver().state.tx_transfer_in_progress.set(false);
        }
    }
}

impl<Dm> core::fmt::Debug for SpiDma<'_, Dm>
where
    Dm: DriverMode + core::fmt::Debug,
{
    /// Formats the `SpiDma` instance for debugging purposes.
    ///
    /// This method returns a debug struct with the name "SpiDma" without
    /// exposing internal details.
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("SpiDma").field("spi", &self.spi).finish()
    }
}

#[instability::unstable]
impl crate::interrupt::InterruptConfigurable for SpiDma<'_, Blocking> {
    /// Sets the interrupt handler
    ///
    /// Interrupts are not enabled at the peripheral level here.
    fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        self.set_interrupt_handler(handler);
    }
}

impl<Dm> SpiDma<'_, Dm>
where
    Dm: DriverMode,
{
    fn spi(&self) -> &SpiWrapper<'_> {
        &self.spi
    }

    fn driver(&self) -> Driver {
        Driver {
            info: self.spi.info(),
            state: self.spi.state(),
        }
    }

    fn dma_driver(&self) -> DmaDriver {
        DmaDriver {
            driver: self.driver(),
            dma_peripheral: self.spi().dma_peripheral(),
            state: self.spi().dma_state(),
        }
    }

    fn is_done(&self) -> bool {
        if self.driver().busy() {
            return false;
        }
        if self.dma_driver().state.rx_transfer_in_progress.get() {
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
        self.dma_driver().state.rx_transfer_in_progress.set(false);
        self.dma_driver().state.tx_transfer_in_progress.set(false);
        fence(Ordering::Acquire);
    }

    /// # Safety:
    ///
    /// The caller must ensure to not access the buffer contents while the
    /// transfer is in progress. Moving the buffer itself is allowed.
    #[cfg_attr(place_spi_master_driver_in_ram, ram)]
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

        self.dma_driver()
            .state
            .rx_transfer_in_progress
            .set(bytes_to_read > 0);
        self.dma_driver()
            .state
            .tx_transfer_in_progress
            .set(bytes_to_write > 0);
        unsafe {
            self.dma_driver().start_transfer_dma(
                full_duplex,
                bytes_to_read,
                bytes_to_write,
                rx_buffer,
                tx_buffer,
                &mut self.channel,
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
        if dummy > 0 {
            // FIXME: https://github.com/esp-rs/esp-hal/issues/2240
            error!("Dummy bits are not supported when there is no data to write");
            return Err(Error::Unsupported);
        }

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
        )?;

        // FIXME: we could use self.start_transfer_dma
        self.dma_driver().state.tx_transfer_in_progress.set(true);
        unsafe {
            self.dma_driver().start_transfer_dma(
                false,
                0,
                bytes_to_write,
                &mut EmptyBuf,
                &mut self.address_buffer,
                &mut self.channel,
            )
        }
    }

    fn cancel_transfer(&mut self) {
        let state = self.dma_driver().state;
        if state.tx_transfer_in_progress.get() || state.rx_transfer_in_progress.get() {
            self.dma_driver().abort_transfer();

            // We need to stop the DMA transfer, too.
            if state.tx_transfer_in_progress.get() {
                self.channel.tx.stop_transfer();
                state.tx_transfer_in_progress.set(false);
            }
            if state.rx_transfer_in_progress.get() {
                self.channel.rx.stop_transfer();
                state.rx_transfer_in_progress.set(false);
            }
        }
    }
}

#[instability::unstable]
impl<Dm> embassy_embedded_hal::SetConfig for SpiDma<'_, Dm>
where
    Dm: DriverMode,
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
#[instability::unstable]
pub struct SpiDmaTransfer<'d, Dm, Buf>
where
    Dm: DriverMode,
{
    spi_dma: ManuallyDrop<SpiDma<'d, Dm>>,
    dma_buf: ManuallyDrop<Buf>,
}

impl<Buf> SpiDmaTransfer<'_, Async, Buf> {
    /// Waits for the DMA transfer to complete asynchronously.
    ///
    /// This method awaits the completion of both RX and TX operations.
    #[instability::unstable]
    pub async fn wait_for_done(&mut self) {
        self.spi_dma.wait_for_idle_async().await;
    }
}

impl<'d, Dm, Buf> SpiDmaTransfer<'d, Dm, Buf>
where
    Dm: DriverMode,
{
    fn new(spi_dma: SpiDma<'d, Dm>, dma_buf: Buf) -> Self {
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
    #[instability::unstable]
    pub fn wait(mut self) -> (SpiDma<'d, Dm>, Buf) {
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
    #[instability::unstable]
    pub fn cancel(&mut self) {
        if !self.spi_dma.is_done() {
            self.spi_dma.cancel_transfer();
        }
    }
}

impl<Dm, Buf> Drop for SpiDmaTransfer<'_, Dm, Buf>
where
    Dm: DriverMode,
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

impl<'d, Dm> SpiDma<'d, Dm>
where
    Dm: DriverMode,
{
    /// # Safety:
    ///
    /// The caller must ensure that the buffers are not accessed while the
    /// transfer is in progress. Moving the buffers is allowed.
    #[cfg_attr(place_spi_master_driver_in_ram, ram)]
    unsafe fn start_dma_write(
        &mut self,
        bytes_to_write: usize,
        buffer: &mut impl DmaTxBuffer,
    ) -> Result<(), Error> {
        unsafe { self.start_dma_transfer(0, bytes_to_write, &mut EmptyBuf, buffer) }
    }

    /// Configures the DMA buffers for the SPI instance.
    ///
    /// This method sets up both RX and TX buffers for DMA transfers.
    /// It returns an instance of `SpiDmaBus` that can be used for SPI
    /// communication.
    #[instability::unstable]
    pub fn with_buffers(self, dma_rx_buf: DmaRxBuf, dma_tx_buf: DmaTxBuf) -> SpiDmaBus<'d, Dm> {
        SpiDmaBus::new(self, dma_rx_buf, dma_tx_buf)
    }

    /// Perform a DMA write.
    ///
    /// This will return a [SpiDmaTransfer] owning the buffer and the
    /// SPI instance. The maximum amount of data to be sent is 32736
    /// bytes.
    #[allow(clippy::type_complexity)]
    #[cfg_attr(place_spi_master_driver_in_ram, ram)]
    #[instability::unstable]
    pub fn write<TX: DmaTxBuffer>(
        mut self,
        bytes_to_write: usize,
        mut buffer: TX,
    ) -> Result<SpiDmaTransfer<'d, Dm, TX>, (Error, Self, TX)> {
        self.wait_for_idle();
        if let Err(e) = self.driver().setup_full_duplex() {
            return Err((e, self, buffer));
        };
        match unsafe { self.start_dma_write(bytes_to_write, &mut buffer) } {
            Ok(_) => Ok(SpiDmaTransfer::new(self, buffer)),
            Err(e) => Err((e, self, buffer)),
        }
    }

    /// # Safety:
    ///
    /// The caller must ensure that the buffers are not accessed while the
    /// transfer is in progress. Moving the buffers is allowed.
    #[cfg_attr(place_spi_master_driver_in_ram, ram)]
    unsafe fn start_dma_read(
        &mut self,
        bytes_to_read: usize,
        buffer: &mut impl DmaRxBuffer,
    ) -> Result<(), Error> {
        unsafe { self.start_dma_transfer(bytes_to_read, 0, buffer, &mut EmptyBuf) }
    }

    /// Perform a DMA read.
    ///
    /// This will return a [SpiDmaTransfer] owning the buffer and
    /// the SPI instance. The maximum amount of data to be
    /// received is 32736 bytes.
    #[allow(clippy::type_complexity)]
    #[cfg_attr(place_spi_master_driver_in_ram, ram)]
    #[instability::unstable]
    pub fn read<RX: DmaRxBuffer>(
        mut self,
        bytes_to_read: usize,
        mut buffer: RX,
    ) -> Result<SpiDmaTransfer<'d, Dm, RX>, (Error, Self, RX)> {
        self.wait_for_idle();
        if let Err(e) = self.driver().setup_full_duplex() {
            return Err((e, self, buffer));
        };
        match unsafe { self.start_dma_read(bytes_to_read, &mut buffer) } {
            Ok(_) => Ok(SpiDmaTransfer::new(self, buffer)),
            Err(e) => Err((e, self, buffer)),
        }
    }

    /// # Safety:
    ///
    /// The caller must ensure that the buffers are not accessed while the
    /// transfer is in progress. Moving the buffers is allowed.
    #[cfg_attr(place_spi_master_driver_in_ram, ram)]
    unsafe fn start_dma_transfer(
        &mut self,
        bytes_to_read: usize,
        bytes_to_write: usize,
        rx_buffer: &mut impl DmaRxBuffer,
        tx_buffer: &mut impl DmaTxBuffer,
    ) -> Result<(), Error> {
        unsafe {
            self.start_transfer_dma(true, bytes_to_read, bytes_to_write, rx_buffer, tx_buffer)
        }
    }

    /// Perform a DMA transfer
    ///
    /// This will return a [SpiDmaTransfer] owning the buffers and
    /// the SPI instance. The maximum amount of data to be
    /// sent/received is 32736 bytes.
    #[allow(clippy::type_complexity)]
    #[cfg_attr(place_spi_master_driver_in_ram, ram)]
    #[instability::unstable]
    pub fn transfer<RX: DmaRxBuffer, TX: DmaTxBuffer>(
        mut self,
        bytes_to_read: usize,
        mut rx_buffer: RX,
        bytes_to_write: usize,
        mut tx_buffer: TX,
    ) -> Result<SpiDmaTransfer<'d, Dm, (RX, TX)>, (Error, Self, RX, TX)> {
        self.wait_for_idle();
        if let Err(e) = self.driver().setup_full_duplex() {
            return Err((e, self, rx_buffer, tx_buffer));
        };
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
    #[cfg_attr(place_spi_master_driver_in_ram, ram)]
    unsafe fn start_half_duplex_read(
        &mut self,
        data_mode: DataMode,
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
        )?;

        unsafe { self.start_transfer_dma(false, bytes_to_read, 0, buffer, &mut EmptyBuf) }
    }

    /// Perform a half-duplex read operation using DMA.
    #[allow(clippy::type_complexity)]
    #[cfg_attr(place_spi_master_driver_in_ram, ram)]
    #[instability::unstable]
    pub fn half_duplex_read<RX: DmaRxBuffer>(
        mut self,
        data_mode: DataMode,
        cmd: Command,
        address: Address,
        dummy: u8,
        bytes_to_read: usize,
        mut buffer: RX,
    ) -> Result<SpiDmaTransfer<'d, Dm, RX>, (Error, Self, RX)> {
        self.wait_for_idle();

        match unsafe {
            self.start_half_duplex_read(data_mode, cmd, address, dummy, bytes_to_read, &mut buffer)
        } {
            Ok(_) => Ok(SpiDmaTransfer::new(self, buffer)),
            Err(e) => Err((e, self, buffer)),
        }
    }

    /// # Safety:
    ///
    /// The caller must ensure that the buffers are not accessed while the
    /// transfer is in progress. Moving the buffers is allowed.
    #[cfg_attr(place_spi_master_driver_in_ram, ram)]
    unsafe fn start_half_duplex_write(
        &mut self,
        data_mode: DataMode,
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
            if bytes_to_write == 0 && address.mode() != DataMode::SingleTwoDataLines {
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
        )?;

        unsafe { self.start_transfer_dma(false, 0, bytes_to_write, &mut EmptyBuf, buffer) }
    }

    /// Perform a half-duplex write operation using DMA.
    #[allow(clippy::type_complexity)]
    #[cfg_attr(place_spi_master_driver_in_ram, ram)]
    #[instability::unstable]
    pub fn half_duplex_write<TX: DmaTxBuffer>(
        mut self,
        data_mode: DataMode,
        cmd: Command,
        address: Address,
        dummy: u8,
        bytes_to_write: usize,
        mut buffer: TX,
    ) -> Result<SpiDmaTransfer<'d, Dm, TX>, (Error, Self, TX)> {
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

    /// Change the bus configuration.
    ///
    /// # Errors
    ///
    /// If frequency passed in config exceeds
    #[cfg_attr(not(esp32h2), doc = " 80MHz")]
    #[cfg_attr(esp32h2, doc = " 48MHz")]
    /// or is below 70kHz,
    /// [`ConfigError::UnsupportedFrequency`] error will be returned.
    #[instability::unstable]
    pub fn apply_config(&mut self, config: &Config) -> Result<(), ConfigError> {
        self.driver().apply_config(config)
    }
}

/// A DMA-capable SPI bus.
///
/// This structure is responsible for managing SPI transfers using DMA
/// buffers.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub struct SpiDmaBus<'d, Dm>
where
    Dm: DriverMode,
{
    spi_dma: SpiDma<'d, Dm>,
    rx_buf: DmaRxBuf,
    tx_buf: DmaTxBuf,
}

impl<Dm> crate::private::Sealed for SpiDmaBus<'_, Dm> where Dm: DriverMode {}

impl<'d> SpiDmaBus<'d, Blocking> {
    /// Converts the SPI instance into async mode.
    #[instability::unstable]
    pub fn into_async(self) -> SpiDmaBus<'d, Async> {
        SpiDmaBus {
            spi_dma: self.spi_dma.into_async(),
            rx_buf: self.rx_buf,
            tx_buf: self.tx_buf,
        }
    }

    /// Listen for the given interrupts
    #[instability::unstable]
    pub fn listen(&mut self, interrupts: impl Into<EnumSet<SpiInterrupt>>) {
        self.spi_dma.listen(interrupts.into());
    }

    /// Unlisten the given interrupts
    #[instability::unstable]
    pub fn unlisten(&mut self, interrupts: impl Into<EnumSet<SpiInterrupt>>) {
        self.spi_dma.unlisten(interrupts.into());
    }

    /// Gets asserted interrupts
    #[instability::unstable]
    pub fn interrupts(&mut self) -> EnumSet<SpiInterrupt> {
        self.spi_dma.interrupts()
    }

    /// Resets asserted interrupts
    #[instability::unstable]
    pub fn clear_interrupts(&mut self, interrupts: impl Into<EnumSet<SpiInterrupt>>) {
        self.spi_dma.clear_interrupts(interrupts.into());
    }
}

impl<'d> SpiDmaBus<'d, Async> {
    /// Converts the SPI instance into async mode.
    #[instability::unstable]
    pub fn into_blocking(self) -> SpiDmaBus<'d, Blocking> {
        SpiDmaBus {
            spi_dma: self.spi_dma.into_blocking(),
            rx_buf: self.rx_buf,
            tx_buf: self.tx_buf,
        }
    }

    /// Fill the given buffer with data from the bus.
    #[instability::unstable]
    pub async fn read_async(&mut self, words: &mut [u8]) -> Result<(), Error> {
        self.spi_dma.wait_for_idle_async().await;
        self.spi_dma.driver().setup_full_duplex()?;
        let chunk_size = self.rx_buf.capacity();

        for chunk in words.chunks_mut(chunk_size) {
            let mut spi = DropGuard::new(&mut self.spi_dma, |spi| spi.cancel_transfer());

            unsafe { spi.start_dma_transfer(chunk.len(), 0, &mut self.rx_buf, &mut EmptyBuf)? };

            spi.wait_for_idle_async().await;

            chunk.copy_from_slice(&self.rx_buf.as_slice()[..chunk.len()]);

            spi.defuse();
        }

        Ok(())
    }

    /// Transmit the given buffer to the bus.
    #[instability::unstable]
    pub async fn write_async(&mut self, words: &[u8]) -> Result<(), Error> {
        self.spi_dma.wait_for_idle_async().await;
        self.spi_dma.driver().setup_full_duplex()?;

        let mut spi = DropGuard::new(&mut self.spi_dma, |spi| spi.cancel_transfer());
        let chunk_size = self.tx_buf.capacity();

        for chunk in words.chunks(chunk_size) {
            self.tx_buf.as_mut_slice()[..chunk.len()].copy_from_slice(chunk);

            unsafe { spi.start_dma_transfer(0, chunk.len(), &mut EmptyBuf, &mut self.tx_buf)? };

            spi.wait_for_idle_async().await;
        }
        spi.defuse();

        Ok(())
    }

    /// Transfer by writing out a buffer and reading the response from
    /// the bus into another buffer.
    #[instability::unstable]
    pub async fn transfer_async(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Error> {
        self.spi_dma.wait_for_idle_async().await;
        self.spi_dma.driver().setup_full_duplex()?;

        let mut spi = DropGuard::new(&mut self.spi_dma, |spi| spi.cancel_transfer());
        let chunk_size = min(self.tx_buf.capacity(), self.rx_buf.capacity());

        let common_length = min(read.len(), write.len());
        let (read_common, read_remainder) = read.split_at_mut(common_length);
        let (write_common, write_remainder) = write.split_at(common_length);

        for (read_chunk, write_chunk) in read_common
            .chunks_mut(chunk_size)
            .zip(write_common.chunks(chunk_size))
        {
            self.tx_buf.as_mut_slice()[..write_chunk.len()].copy_from_slice(write_chunk);

            unsafe {
                spi.start_dma_transfer(
                    read_chunk.len(),
                    write_chunk.len(),
                    &mut self.rx_buf,
                    &mut self.tx_buf,
                )?;
            }
            spi.wait_for_idle_async().await;

            read_chunk.copy_from_slice(&self.rx_buf.as_slice()[..read_chunk.len()]);
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
    #[instability::unstable]
    pub async fn transfer_in_place_async(&mut self, words: &mut [u8]) -> Result<(), Error> {
        self.spi_dma.wait_for_idle_async().await;
        self.spi_dma.driver().setup_full_duplex()?;

        let mut spi = DropGuard::new(&mut self.spi_dma, |spi| spi.cancel_transfer());
        for chunk in words.chunks_mut(self.tx_buf.capacity()) {
            self.tx_buf.as_mut_slice()[..chunk.len()].copy_from_slice(chunk);

            unsafe {
                spi.start_dma_transfer(
                    chunk.len(),
                    chunk.len(),
                    &mut self.rx_buf,
                    &mut self.tx_buf,
                )?;
            }
            spi.wait_for_idle_async().await;
            chunk.copy_from_slice(&self.rx_buf.as_slice()[..chunk.len()]);
        }

        spi.defuse();

        Ok(())
    }
}

impl<'d, Dm> SpiDmaBus<'d, Dm>
where
    Dm: DriverMode,
{
    /// Creates a new `SpiDmaBus` with the specified SPI instance and DMA
    /// buffers.
    pub fn new(spi_dma: SpiDma<'d, Dm>, rx_buf: DmaRxBuf, tx_buf: DmaTxBuf) -> Self {
        Self {
            spi_dma,
            rx_buf,
            tx_buf,
        }
    }

    /// Splits [SpiDmaBus] back into [SpiDma], [DmaRxBuf] and [DmaTxBuf].
    #[instability::unstable]
    pub fn split(mut self) -> (SpiDma<'d, Dm>, DmaRxBuf, DmaTxBuf) {
        self.wait_for_idle();
        (self.spi_dma, self.rx_buf, self.tx_buf)
    }

    fn wait_for_idle(&mut self) {
        self.spi_dma.wait_for_idle();
    }

    /// Change the bus configuration.
    ///
    /// # Errors
    ///
    /// If frequency passed in config exceeds
    #[cfg_attr(not(esp32h2), doc = " 80MHz")]
    #[cfg_attr(esp32h2, doc = " 48MHz")]
    /// or is below 70kHz,
    /// [`ConfigError::UnsupportedFrequency`] error will be returned.
    #[instability::unstable]
    pub fn apply_config(&mut self, config: &Config) -> Result<(), ConfigError> {
        self.spi_dma.apply_config(config)
    }

    /// Reads data from the SPI bus using DMA.
    #[instability::unstable]
    pub fn read(&mut self, words: &mut [u8]) -> Result<(), Error> {
        self.wait_for_idle();
        self.spi_dma.driver().setup_full_duplex()?;
        for chunk in words.chunks_mut(self.rx_buf.capacity()) {
            unsafe {
                self.spi_dma
                    .start_dma_transfer(chunk.len(), 0, &mut self.rx_buf, &mut EmptyBuf)?;
            }

            self.wait_for_idle();
            chunk.copy_from_slice(&self.rx_buf.as_slice()[..chunk.len()]);
        }

        Ok(())
    }

    /// Writes data to the SPI bus using DMA.
    #[instability::unstable]
    pub fn write(&mut self, words: &[u8]) -> Result<(), Error> {
        self.wait_for_idle();
        self.spi_dma.driver().setup_full_duplex()?;
        for chunk in words.chunks(self.tx_buf.capacity()) {
            self.tx_buf.as_mut_slice()[..chunk.len()].copy_from_slice(chunk);

            unsafe {
                self.spi_dma
                    .start_dma_transfer(0, chunk.len(), &mut EmptyBuf, &mut self.tx_buf)?;
            }

            self.wait_for_idle();
        }

        Ok(())
    }

    /// Transfers data to and from the SPI bus simultaneously using DMA.
    #[instability::unstable]
    pub fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Error> {
        self.wait_for_idle();
        self.spi_dma.driver().setup_full_duplex()?;
        let chunk_size = min(self.tx_buf.capacity(), self.rx_buf.capacity());

        let common_length = min(read.len(), write.len());
        let (read_common, read_remainder) = read.split_at_mut(common_length);
        let (write_common, write_remainder) = write.split_at(common_length);

        for (read_chunk, write_chunk) in read_common
            .chunks_mut(chunk_size)
            .zip(write_common.chunks(chunk_size))
        {
            self.tx_buf.as_mut_slice()[..write_chunk.len()].copy_from_slice(write_chunk);

            unsafe {
                self.spi_dma.start_dma_transfer(
                    read_chunk.len(),
                    write_chunk.len(),
                    &mut self.rx_buf,
                    &mut self.tx_buf,
                )?;
            }
            self.wait_for_idle();

            read_chunk.copy_from_slice(&self.rx_buf.as_slice()[..read_chunk.len()]);
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
    #[instability::unstable]
    pub fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Error> {
        self.wait_for_idle();
        self.spi_dma.driver().setup_full_duplex()?;
        let chunk_size = min(self.tx_buf.capacity(), self.rx_buf.capacity());

        for chunk in words.chunks_mut(chunk_size) {
            self.tx_buf.as_mut_slice()[..chunk.len()].copy_from_slice(chunk);

            unsafe {
                self.spi_dma.start_dma_transfer(
                    chunk.len(),
                    chunk.len(),
                    &mut self.rx_buf,
                    &mut self.tx_buf,
                )?;
            }
            self.wait_for_idle();
            chunk.copy_from_slice(&self.rx_buf.as_slice()[..chunk.len()]);
        }

        Ok(())
    }

    /// Half-duplex read.
    #[instability::unstable]
    pub fn half_duplex_read(
        &mut self,
        data_mode: DataMode,
        cmd: Command,
        address: Address,
        dummy: u8,
        buffer: &mut [u8],
    ) -> Result<(), Error> {
        if buffer.len() > self.rx_buf.capacity() {
            return Err(Error::from(DmaError::Overflow));
        }
        self.wait_for_idle();

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

        buffer.copy_from_slice(&self.rx_buf.as_slice()[..buffer.len()]);

        Ok(())
    }

    /// Half-duplex write.
    #[instability::unstable]
    pub fn half_duplex_write(
        &mut self,
        data_mode: DataMode,
        cmd: Command,
        address: Address,
        dummy: u8,
        buffer: &[u8],
    ) -> Result<(), Error> {
        if buffer.len() > self.tx_buf.capacity() {
            return Err(Error::from(DmaError::Overflow));
        }
        self.wait_for_idle();
        self.tx_buf.as_mut_slice()[..buffer.len()].copy_from_slice(buffer);

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

#[instability::unstable]
impl crate::interrupt::InterruptConfigurable for SpiDmaBus<'_, Blocking> {
    /// Sets the interrupt handler
    ///
    /// Interrupts are not enabled at the peripheral level here.
    fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        self.spi_dma.set_interrupt_handler(handler);
    }
}

#[instability::unstable]
impl<Dm> embassy_embedded_hal::SetConfig for SpiDmaBus<'_, Dm>
where
    Dm: DriverMode,
{
    type Config = Config;
    type ConfigError = ConfigError;

    fn set_config(&mut self, config: &Self::Config) -> Result<(), Self::ConfigError> {
        self.apply_config(config)
    }
}

pub(super) struct DmaDriver {
    driver: Driver,
    dma_peripheral: crate::dma::DmaPeripheral,
    state: &'static DmaState,
}

impl DmaDriver {
    fn abort_transfer(&self) {
        // The SPI peripheral is controlling how much data we transfer, so let's
        // update its counter.
        // 0 doesn't take effect on ESP32 and cuts the currently transmitted byte
        // immediately.
        // 1 seems to stop after transmitting the current byte which is somewhat less
        // impolite.
        self.driver.configure_datalen(1, 1);
        self.driver.update();
    }

    fn regs(&self) -> &RegisterBlock {
        self.driver.regs()
    }

    #[cfg_attr(place_spi_master_driver_in_ram, ram)]
    unsafe fn start_transfer_dma<Dm: DriverMode>(
        &self,
        _full_duplex: bool,
        rx_len: usize,
        tx_len: usize,
        rx_buffer: &mut impl DmaRxBuffer,
        tx_buffer: &mut impl DmaTxBuffer,
        channel: &mut Channel<Dm, PeripheralDmaChannel<AnySpi<'_>>>,
    ) -> Result<(), Error> {
        #[cfg(esp32s2)]
        {
            // without this a transfer after a write will fail
            self.regs().dma_out_link().write(|w| unsafe { w.bits(0) });
            self.regs().dma_in_link().write(|w| unsafe { w.bits(0) });
        }

        self.driver.configure_datalen(rx_len, tx_len);

        // enable the MISO and MOSI if needed
        self.regs()
            .user()
            .modify(|_, w| w.usr_miso().bit(rx_len > 0).usr_mosi().bit(tx_len > 0));

        self.enable_dma();

        if rx_len > 0 {
            unsafe {
                channel
                    .rx
                    .prepare_transfer(self.dma_peripheral, rx_buffer)
                    .and_then(|_| channel.rx.start_transfer())?;
            }
        } else {
            #[cfg(esp32)]
            {
                // see https://github.com/espressif/esp-idf/commit/366e4397e9dae9d93fe69ea9d389b5743295886f
                // see https://github.com/espressif/esp-idf/commit/0c3653b1fd7151001143451d4aa95dbf15ee8506
                if _full_duplex {
                    self.regs()
                        .dma_in_link()
                        .modify(|_, w| unsafe { w.inlink_addr().bits(0) });
                    self.regs()
                        .dma_in_link()
                        .modify(|_, w| w.inlink_start().set_bit());
                }
            }
        }
        if tx_len > 0 {
            unsafe {
                channel
                    .tx
                    .prepare_transfer(self.dma_peripheral, tx_buffer)
                    .and_then(|_| channel.tx.start_transfer())?;
            }
        }

        #[cfg(dma_kind = "gdma")]
        self.reset_dma();

        self.driver.start_operation();

        Ok(())
    }

    fn enable_dma(&self) {
        #[cfg(dma_kind = "gdma")]
        // for non GDMA this is done in `assign_tx_device` / `assign_rx_device`
        self.regs().dma_conf().modify(|_, w| {
            w.dma_tx_ena().set_bit();
            w.dma_rx_ena().set_bit()
        });

        #[cfg(dma_kind = "pdma")]
        self.reset_dma();
    }

    fn reset_dma(&self) {
        #[cfg(dma_kind = "pdma")]
        self.regs().dma_conf().toggle(|w, bit| {
            w.out_rst().bit(bit);
            w.in_rst().bit(bit);
            w.ahbm_fifo_rst().bit(bit);
            w.ahbm_rst().bit(bit)
        });

        #[cfg(dma_kind = "gdma")]
        self.regs().dma_conf().toggle(|w, bit| {
            w.rx_afifo_rst().bit(bit);
            w.buf_afifo_rst().bit(bit);
            w.dma_afifo_rst().bit(bit)
        });

        self.clear_dma_interrupts();
    }

    #[cfg(dma_kind = "gdma")]
    fn clear_dma_interrupts(&self) {
        self.regs().dma_int_clr().write(|w| {
            w.dma_infifo_full_err().clear_bit_by_one();
            w.dma_outfifo_empty_err().clear_bit_by_one();
            w.trans_done().clear_bit_by_one();
            w.mst_rx_afifo_wfull_err().clear_bit_by_one();
            w.mst_tx_afifo_rempty_err().clear_bit_by_one()
        });
    }

    #[cfg(dma_kind = "pdma")]
    fn clear_dma_interrupts(&self) {
        self.regs().dma_int_clr().write(|w| {
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

impl<'d> DmaEligible for AnySpi<'d> {
    #[cfg(dma_kind = "gdma")]
    type Dma = crate::dma::AnyGdmaChannel<'d>;
    #[cfg(dma_kind = "pdma")]
    type Dma = crate::dma::AnySpiDmaChannel<'d>;

    fn dma_peripheral(&self) -> crate::dma::DmaPeripheral {
        let (info, _state) = self.dma_parts();
        info.dma_peripheral
    }
}

#[instability::unstable]
impl embedded_hal_async::spi::SpiBus for SpiDmaBus<'_, Async> {
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

#[instability::unstable]
impl<Dm> ErrorType for SpiDmaBus<'_, Dm>
where
    Dm: DriverMode,
{
    type Error = Error;
}

#[instability::unstable]
impl<Dm> SpiBus for SpiDmaBus<'_, Dm>
where
    Dm: DriverMode,
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

struct DmaInfo {
    dma_peripheral: crate::dma::DmaPeripheral,
}
struct DmaState {
    tx_transfer_in_progress: Cell<bool>,
    rx_transfer_in_progress: Cell<bool>,
}

// SAFETY: State belongs to the currently constructed driver instance. As such, it'll not be
// accessed concurrently in multiple threads.
unsafe impl Sync for DmaState {}

for_each_spi_master!(
    (all $( ($peri:ident, $sys:ident, $sclk:ident $_cs:tt $_sio:tt $(, $is_qspi:tt)?)),* ) => {
        impl AnySpi<'_> {
            #[inline(always)]
            fn dma_parts(&self) -> (&'static DmaInfo, &'static DmaState) {
                match &self.0 {
                    $(
                        super::any::Inner::$sys(_spi) => {
                            static DMA_INFO: DmaInfo = DmaInfo {
                                dma_peripheral: crate::dma::DmaPeripheral::$sys,
                            };

                            static DMA_STATE: DmaState = DmaState {
                                tx_transfer_in_progress: Cell::new(false),
                                rx_transfer_in_progress: Cell::new(false),
                            };

                            (&DMA_INFO, &DMA_STATE)
                        }
                    )*
                }
            }

            #[inline(always)]
            fn dma_state(&self) -> &'static DmaState {
                let (_, state) = self.dma_parts();
                state
            }

            #[inline(always)]
            fn dma_info(&self) -> &'static DmaInfo {
                let (info, _) = self.dma_parts();
                info
            }
        }
    };
);

impl SpiWrapper<'_> {
    fn dma_state(&self) -> &'static DmaState {
        self.spi.dma_state()
    }

    #[inline(always)]
    fn dma_peripheral(&self) -> crate::dma::DmaPeripheral {
        self.spi.dma_peripheral()
    }
}
