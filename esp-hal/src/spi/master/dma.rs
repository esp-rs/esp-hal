use core::{
    cell::{Cell, UnsafeCell},
    cmp::min,
    mem::{ManuallyDrop, MaybeUninit},
    pin::Pin,
    ptr::NonNull,
    sync::atomic::{Ordering, fence},
    task::{Context, Poll},
};

#[cfg(feature = "unstable")]
use embedded_hal::spi::{ErrorType, SpiBus};
use enumset::EnumSet;

use super::*;
use crate::{
    RegisterToggle,
    dma::{
        CHUNK_SIZE,
        Channel,
        DmaChannelFor,
        DmaDescriptor,
        DmaEligible,
        DmaRxBuf,
        DmaRxBuffer,
        DmaTxBuf,
        DmaTxBuffer,
        NoBuffer,
        PeripheralDmaChannel,
        ScopedDmaRxBuf,
        ScopedDmaTxBuf,
        TransferDirection,
        asynch::DmaRxFuture,
        prepare_for_rx,
        prepare_for_tx,
    },
    pac::spi2::RegisterBlock,
    private::DropGuard,
    soc::is_slice_in_dram,
    spi::DmaError,
};
#[cfg(dma_can_access_psram)]
use crate::{dma::ManualWritebackBuffer, soc::is_slice_in_psram};

const MAX_DMA_SIZE: usize = 32736;

impl<'d> Spi<'d, Blocking> {
    #[doc_replace(
        "dma_channel" => {
            cfg(dma_kind = "pdma") => "DMA_SPI2",
            cfg(dma_kind = "gdma") => "DMA_CH0",
        }
    )]
    /// Converts the driver into an [`SpiDma`] driver that uses the specified DMA channel.
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::spi::{
    ///     Mode,
    ///     master::{Config, Spi},
    /// };
    ///
    /// let mut spi_dma = Spi::new(
    ///     peripherals.SPI2,
    ///     Config::default()
    ///         .with_frequency(Rate::from_khz(100))
    ///         .with_mode(Mode::_0),
    /// )?
    /// .with_dma(peripherals.__dma_channel__);
    /// # {after_snippet}
    /// ```
    #[instability::unstable]
    pub fn with_dma(self, channel: impl DmaChannelFor<AnySpi<'d>>) -> SpiDma<'d, Blocking> {
        SpiDma::new_from_spi(self, channel.degrade())
    }
}

#[doc_replace(
    "dma_channel" => {
        cfg(dma_kind = "pdma") => "DMA_SPI2",
        cfg(dma_kind = "gdma") => "DMA_CH0",
    }
)]
/// DMA-controlled SPI driver.
///
/// This driver uses DMA to transfer data, allowing the CPU to continue working while the SPI
/// transfer is in progress.
///
/// The driver provides two separate approaches to transferring data:
///
/// - The slice-based API allows transferring data from/to slices of memory. The data may be copied
///   into an internal buffer before the transfer begins. A pair of copy buffers can be set up by
///   passing them to [`with_buffers`](SpiDma::with_buffers) before the first transfer begins. For
///   more details on when copying is necessary, see the documentation of the
///   [`with_buffers`](SpiDma::with_buffers) method.
/// - The buffer API allows transferring externally managed buffers. In this mode, you provide the
///   buffers to be transferred. The buffer objects ensure that data is located in appropriate
///   memory regions. The buffers and the driver object are moved into transfer objects for the
///   duration of the transfer. These functions take [`DmaRxBuf`] and [`DmaTxBuf`] objects as
///   arguments as well as the number of bytes to transfer, and their names end with `_buffer`.
///
/// These approaches provide different trade-offs between memory usage / CPU overhead and ease of
/// use. `embedded-hal` traits are implemented by the slice-based API's functions.
///
/// ## Examples
///
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
///
/// // Optional: create and set up copy buffers.
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
}

impl<Dm> crate::private::Sealed for SpiDma<'_, Dm> where Dm: DriverMode {}

impl<Dm> core::fmt::Debug for SpiDma<'_, Dm>
where
    Dm: DriverMode + core::fmt::Debug,
{
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("SpiDma").field("spi", &self.spi).finish()
    }
}

#[instability::unstable]
impl crate::interrupt::InterruptConfigurable for SpiDma<'_, Blocking> {
    fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        self.set_interrupt_handler(handler);
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

#[instability::unstable]
impl<Dm> ErrorType for SpiDma<'_, Dm>
where
    Dm: DriverMode,
{
    type Error = Error;
}

#[instability::unstable]
impl<Dm> SpiBus for SpiDma<'_, Dm>
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
        // DMA limitation - we must ensure the transfers complete before returning
        // to user code, otherwise the user might access the buffers while the transfer
        // is still in progress. Therefore, there is no such thing as "flushing".
        Ok(())
    }
}

#[instability::unstable]
impl embedded_hal_async::spi::SpiBus for SpiDma<'_, Async> {
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
        // DMA limitation - we must ensure the transfers complete before returning
        // to user code, otherwise the user might access the buffers while the transfer
        // is still in progress. Therefore, there is no such thing as "flushing".
        Ok(())
    }
}

impl<'d> SpiDma<'d, Blocking> {
    /// Converts the SPI driver into async mode.
    #[instability::unstable]
    pub fn into_async(self) -> SpiDma<'d, Async> {
        self.spi
            .set_interrupt_handler(self.spi.info().async_handler);
        SpiDma {
            spi: self.spi,
            channel: self.channel.into_async(),
        }
    }

    fn new_inner(spi: SpiWrapper<'d>, channel: PeripheralDmaChannel<AnySpi<'d>>) -> Self {
        let channel = Channel::new(channel);
        channel.runtime_ensure_compatible(&spi.spi);

        for_each_spi_master!((all $($inst:tt),*) => {
            const SPI_NUM: usize = 0 $(+ { stringify!($inst); 1 })*;
        };);
        let id = if spi.info() == unsafe { crate::peripherals::SPI2::steal().info() } {
            0
        } else {
            1
        };

        let state = spi.spi.dma_state();

        state.tx_transfer_in_progress.set(false);
        state.rx_transfer_in_progress.set(false);

        static mut TX_DESCRIPTORS: [[DmaDescriptor; 1]; SPI_NUM] =
            [[DmaDescriptor::EMPTY]; SPI_NUM];
        static mut RX_DESCRIPTORS: [[DmaDescriptor; 1]; SPI_NUM] =
            [[DmaDescriptor::EMPTY]; SPI_NUM];

        let rx_buffer = unwrap!(DmaRxBuf::new(unsafe { &mut RX_DESCRIPTORS[id] }, &mut []));

        cfg_if::cfg_if! {
            if #[cfg(all(spi_master_version = "1", spi_address_workaround))] {
                static mut BUFFERS: [[u32; 1]; SPI_NUM] = [[0]; SPI_NUM];
                let buffer = crate::dma::as_mut_byte_array!(BUFFERS[id], 4);
                let tx_buffer = unwrap!(DmaTxBuf::new(unsafe { &mut TX_DESCRIPTORS[id] }, buffer));
            } else {
                let tx_buffer = unwrap!(DmaTxBuf::new(unsafe { &mut TX_DESCRIPTORS[id] }, &mut []));
            }
        }

        // The buffers must be set up when creating the driver.
        unsafe { (&mut *state.tx_buffer.get()).write(tx_buffer.into_scoped()) };
        unsafe { (&mut *state.rx_buffer.get()).write(rx_buffer.into_scoped()) };

        Self { spi, channel }
    }

    pub(super) fn new_from_spi(
        spi_driver: Spi<'d, Blocking>,
        channel: PeripheralDmaChannel<AnySpi<'d>>,
    ) -> Self {
        let spi = spi_driver.spi;

        Self::new_inner(spi, channel)
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
    /// Converts the SPI instance into blocking mode.
    #[instability::unstable]
    pub fn into_blocking(self) -> SpiDma<'d, Blocking> {
        self.spi.disable_peri_interrupt_on_all_cores();
        SpiDma {
            spi: self.spi,
            channel: self.channel.into_blocking(),
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
                    #[cfg(any(spi_master_version = "1", spi_master_version = "2"))]
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

    /// Fill the given buffer with data from the bus.
    #[instability::unstable]
    pub async fn read_async(&mut self, words: &mut [u8]) -> Result<(), Error> {
        if words.is_empty() {
            return Ok(());
        }

        self.wait_for_idle_async().await;
        self.driver().setup_full_duplex()?;

        if self.use_blocking_transfer(words.len()) {
            self.dma_driver().disable_dma();
            return self.driver().read(words);
        }

        let mut descriptors = [DmaDescriptor::EMPTY; LINK_DESCRIPTOR_COUNT];
        let mut maybe_copy_buffer = match DmaOperationKind::for_read(words) {
            DmaOperationKind::Copied => {
                MaybeCopyRxBuf::Copy(unsafe { self.spi.dma_state().rx_buffer() })
            }
            DmaOperationKind::InPlace => MaybeCopyRxBuf::Direct {
                descriptors: &mut descriptors,
                #[cfg(dma_can_access_psram)]
                align_buffer: [const { None }; 2],
            },
        };

        if maybe_copy_buffer.chunk_size() == 0 {
            return Err(Error::from(DmaError::BufferTooSmall));
        }

        for chunk in words.chunks_mut(maybe_copy_buffer.chunk_size()) {
            let read_bytes = chunk.len();
            let rx_buffer = unsafe { maybe_copy_buffer.setup(NonNull::from(&mut *chunk)) };
            let tx_buffer = unsafe { NoBuffer(self.spi.dma_state().tx_buffer().prepare()) };

            self.transfer_buffers_dma_async(read_bytes, 0, rx_buffer, tx_buffer)
                .await?;

            maybe_copy_buffer.finish(chunk);
        }

        Ok(())
    }

    /// Transmit the given buffer to the bus.
    #[instability::unstable]
    pub async fn write_async(&mut self, words: &[u8]) -> Result<(), Error> {
        if words.is_empty() {
            return Ok(());
        }

        self.wait_for_idle_async().await;
        self.driver().setup_full_duplex()?;

        if self.use_blocking_transfer(words.len()) {
            self.dma_driver().disable_dma();
            self.driver().write(words)?;
            return self.driver().flush();
        }

        let mut descriptors = [DmaDescriptor::EMPTY; LINK_DESCRIPTOR_COUNT];
        let mut maybe_copy_buffer = match DmaOperationKind::for_write(words) {
            DmaOperationKind::Copied => {
                MaybeCopyTxBuf::Copy(unsafe { self.spi.dma_state().tx_buffer() })
            }
            DmaOperationKind::InPlace => MaybeCopyTxBuf::Direct(&mut descriptors),
        };

        if maybe_copy_buffer.chunk_size() == 0 {
            return Err(Error::from(DmaError::BufferTooSmall));
        }

        for chunk in words.chunks(maybe_copy_buffer.chunk_size()) {
            let write_bytes = chunk.len();
            let rx_buffer = unsafe { NoBuffer(self.spi.dma_state().rx_buffer().prepare()) };
            let tx_buffer = unsafe { maybe_copy_buffer.setup(NonNull::from(chunk)) };

            self.transfer_buffers_dma_async(0, write_bytes, rx_buffer, tx_buffer)
                .await?;
        }

        Ok(())
    }

    /// Transfer by writing out a buffer and reading the response from
    /// the bus into another buffer.
    #[instability::unstable]
    pub async fn transfer_async(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Error> {
        if read.is_empty() && write.is_empty() {
            return Ok(());
        }

        self.wait_for_idle_async().await;
        self.driver().setup_full_duplex()?;

        if self.use_blocking_transfer(read.len().max(write.len())) {
            self.dma_driver().disable_dma();
            if read.is_empty() {
                self.driver().write(write)?;
                return self.driver().flush();
            } else if write.is_empty() {
                return self.driver().read(read);
            } else {
                return self.driver().transfer(read, write);
            }
        }

        let mut rx_descriptors = [DmaDescriptor::EMPTY; LINK_DESCRIPTOR_COUNT];
        let mut maybe_copy_rx_buffer = match DmaOperationKind::for_read(read) {
            DmaOperationKind::Copied => {
                MaybeCopyRxBuf::Copy(unsafe { self.spi.dma_state().rx_buffer() })
            }
            DmaOperationKind::InPlace => MaybeCopyRxBuf::Direct {
                descriptors: &mut rx_descriptors,
                #[cfg(dma_can_access_psram)]
                align_buffer: [const { None }; 2],
            },
        };

        let mut tx_descriptors = [DmaDescriptor::EMPTY; LINK_DESCRIPTOR_COUNT];
        let mut maybe_copy_tx_buffer = match DmaOperationKind::for_write(write) {
            DmaOperationKind::Copied => {
                MaybeCopyTxBuf::Copy(unsafe { self.spi.dma_state().tx_buffer() })
            }
            DmaOperationKind::InPlace => MaybeCopyTxBuf::Direct(&mut tx_descriptors),
        };

        let chunk_size = min(
            maybe_copy_rx_buffer.chunk_size(),
            maybe_copy_tx_buffer.chunk_size(),
        );

        if chunk_size == 0 {
            return Err(Error::from(DmaError::BufferTooSmall));
        }

        let common_length = min(read.len(), write.len());
        let (read_common, read_remainder) = read.split_at_mut(common_length);
        let (write_common, write_remainder) = write.split_at(common_length);

        for (read_chunk, write_chunk) in read_common
            .chunks_mut(chunk_size)
            .zip(write_common.chunks(chunk_size))
        {
            let read_bytes = read_chunk.len();
            let write_bytes = write_chunk.len();
            let tx_buffer = unsafe { maybe_copy_tx_buffer.setup(NonNull::from(write_chunk)) };
            let rx_buffer = unsafe { maybe_copy_rx_buffer.setup(NonNull::from(&mut *read_chunk)) };

            self.transfer_buffers_dma_async(read_bytes, write_bytes, rx_buffer, tx_buffer)
                .await?;

            maybe_copy_rx_buffer.finish(read_chunk);
        }

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
        if words.is_empty() {
            return Ok(());
        }

        self.wait_for_idle_async().await;
        self.driver().setup_full_duplex()?;

        if self.use_blocking_transfer(words.len()) {
            self.dma_driver().disable_dma();
            return self.driver().transfer_in_place(words);
        }

        let mut rx_descriptors = [DmaDescriptor::EMPTY; LINK_DESCRIPTOR_COUNT];
        let mut tx_descriptors = [DmaDescriptor::EMPTY; LINK_DESCRIPTOR_COUNT];
        let (mut maybe_copy_rx_buffer, mut maybe_copy_tx_buffer) =
            match DmaOperationKind::for_write(words) {
                DmaOperationKind::Copied => (
                    MaybeCopyRxBuf::Copy(unsafe { self.spi.dma_state().rx_buffer() }),
                    MaybeCopyTxBuf::Copy(unsafe { self.spi.dma_state().tx_buffer() }),
                ),
                DmaOperationKind::InPlace => (
                    MaybeCopyRxBuf::Direct {
                        descriptors: &mut rx_descriptors,
                        #[cfg(dma_can_access_psram)]
                        align_buffer: [const { None }; 2],
                    },
                    MaybeCopyTxBuf::Direct(&mut tx_descriptors),
                ),
            };

        let chunk_size = min(
            maybe_copy_rx_buffer.chunk_size(),
            maybe_copy_tx_buffer.chunk_size(),
        );

        if chunk_size == 0 {
            return Err(Error::from(DmaError::BufferTooSmall));
        }

        for chunk in words.chunks_mut(chunk_size) {
            let bytes = chunk.len();
            let ptr = NonNull::from(&mut *chunk);
            let tx_buffer = unsafe { maybe_copy_tx_buffer.setup(ptr) };
            let rx_buffer = unsafe { maybe_copy_rx_buffer.setup(ptr) };

            self.transfer_buffers_dma_async(bytes, bytes, rx_buffer, tx_buffer)
                .await?;

            maybe_copy_rx_buffer.finish(chunk);
        }

        Ok(())
    }

    async fn transfer_buffers_dma_async(
        &mut self,
        read_bytes: usize,
        write_bytes: usize,
        mut rx_buffer: impl DmaRxBuffer,
        mut tx_buffer: impl DmaTxBuffer,
    ) -> Result<(), Error> {
        let mut spi = DropGuard::new(&mut *self, |spi| spi.cancel_transfer());
        unsafe {
            spi.start_dma_transfer(read_bytes, write_bytes, &mut rx_buffer, &mut tx_buffer)?;
        }
        spi.wait_for_idle_async().await;
        spi.defuse();
        Ok(())
    }
}

// +1 to make sure we have enough descriptors to satisfy strict alignment requirements
const LINK_DESCRIPTOR_COUNT: usize = MAX_DMA_SIZE.div_ceil(CHUNK_SIZE) + 2 + 1;

enum MaybeCopyTxBuf<'a> {
    Copy(&'a mut ScopedDmaTxBuf<'static>),
    Direct(&'a mut [DmaDescriptor; LINK_DESCRIPTOR_COUNT]),
}

impl<'a> MaybeCopyTxBuf<'a> {
    unsafe fn setup(&mut self, data: NonNull<[u8]>) -> NoBuffer {
        match self {
            MaybeCopyTxBuf::Copy(tx_buffer) => {
                tx_buffer.as_mut_slice()[..data.len()].copy_from_slice(unsafe { data.as_ref() });
                NoBuffer(tx_buffer.prepare())
            }
            MaybeCopyTxBuf::Direct(descriptors) => {
                let (buffer, _) = unsafe { unwrap!(prepare_for_tx(&mut **descriptors, data, 1)) };
                buffer
            }
        }
    }

    fn chunk_size(&self) -> usize {
        match self {
            MaybeCopyTxBuf::Copy(buffer) => buffer.capacity().min(MAX_DMA_SIZE),
            MaybeCopyTxBuf::Direct(_) => MAX_DMA_SIZE,
        }
    }
}

enum MaybeCopyRxBuf<'a> {
    Copy(&'a mut ScopedDmaRxBuf<'static>),
    Direct {
        descriptors: &'a mut [DmaDescriptor; LINK_DESCRIPTOR_COUNT],
        #[cfg(dma_can_access_psram)]
        align_buffer: [Option<ManualWritebackBuffer>; 2],
    },
}

impl<'a> MaybeCopyRxBuf<'a> {
    unsafe fn setup(&mut self, data: NonNull<[u8]>) -> NoBuffer {
        match self {
            MaybeCopyRxBuf::Copy(rx_buffer) => NoBuffer(rx_buffer.prepare()),
            MaybeCopyRxBuf::Direct {
                descriptors,
                #[cfg(dma_can_access_psram)]
                align_buffer,
            } => {
                let (buffer, _) = unsafe {
                    prepare_for_rx(
                        &mut **descriptors,
                        #[cfg(dma_can_access_psram)]
                        align_buffer,
                        data,
                    )
                };
                buffer
            }
        }
    }

    fn chunk_size(&self) -> usize {
        match self {
            MaybeCopyRxBuf::Copy(buffer) => buffer.capacity().min(MAX_DMA_SIZE),
            MaybeCopyRxBuf::Direct { .. } => MAX_DMA_SIZE,
        }
    }

    fn finish(&mut self, chunk: &mut [u8]) {
        match self {
            MaybeCopyRxBuf::Copy(buffer) => {
                chunk.copy_from_slice(&buffer.as_slice()[..chunk.len()]);
            }
            MaybeCopyRxBuf::Direct {
                #[cfg(dma_can_access_psram)]
                align_buffer,
                ..
            } =>
            {
                #[cfg(dma_can_access_psram)]
                for buffer in align_buffer.iter_mut() {
                    if let Some(buffer) = buffer.as_ref() {
                        buffer.write_back();
                    }
                    *buffer = None;
                }
            }
        }
    }
}

enum DmaOperationKind {
    /// The entire slice must be copied into the internal buffer first
    Copied,

    /// The slice can be transferred directly, with minimal copying done for alignment
    InPlace,
}

impl DmaOperationKind {
    fn compute(buffer: &[u8], direction: TransferDirection) -> Self {
        fn is_dma_compatible(buffer: &[u8], _direction: TransferDirection) -> bool {
            // FIXME: lazy workaround for ESP32 TX DMA alignment requirements.
            // `prepare_for_tx` and `prepare_for_rx` should be updated to handle ESP32.
            #[cfg(spi_master_version = "1")]
            if !((buffer.as_ptr() as usize).is_multiple_of(4) && buffer.len().is_multiple_of(4)) {
                return false;
            }

            if is_slice_in_dram(buffer) {
                return true;
            }
            #[cfg(dma_can_access_psram)]
            if is_slice_in_psram(buffer) {
                #[cfg(spi_master_version = "2")]
                if _direction == TransferDirection::In {
                    // For some reason, having tail bytes in internal RAM causes issues, so we
                    // force copying if the end of the PSRAM buffer is not aligned.
                    let tail_bytes = (buffer.as_ptr() as usize + buffer.len()).wrapping_neg() & 15;
                    if tail_bytes > 0 {
                        return false;
                    }
                }

                return true;
            }

            // TODO: C5+ DMA can read from flash

            false
        }

        if is_dma_compatible(buffer, direction) {
            Self::InPlace
        } else {
            Self::Copied
        }
    }

    fn for_read(buffer: &mut [u8]) -> Self {
        Self::compute(buffer, TransferDirection::In)
    }

    fn for_write(buffer: &[u8]) -> Self {
        Self::compute(buffer, TransferDirection::Out)
    }
}

impl<'d, Dm> SpiDma<'d, Dm>
where
    Dm: DriverMode,
{
    fn use_blocking_transfer(&self, transfer_size: usize) -> bool {
        let threshold = self
            .spi
            .state()
            .min_async_transfer_size
            .load(Ordering::Relaxed);
        threshold > 0 && transfer_size < threshold
    }

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

    /// # Safety:
    ///
    /// The caller must ensure that the buffers are not accessed while the
    /// transfer is in progress. Moving the buffers is allowed.
    #[cfg(all(spi_master_version = "1", spi_address_workaround))]
    unsafe fn set_up_address_workaround(
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

        let buffer = unsafe { self.dma_driver().tx_buffer() };

        let bytes_to_write = address.width().div_ceil(8);
        // The address register is read in big-endian order,
        // we have to prepare the emulated write in the same way.
        let addr_bytes = address.value().to_be_bytes();
        let addr_bytes = &addr_bytes[4 - bytes_to_write..][..bytes_to_write];
        buffer.fill(addr_bytes);

        self.driver().setup_half_duplex(
            true,
            cmd,
            Address::None,
            false,
            dummy,
            bytes_to_write == 0,
            address.mode(),
        )?;

        let rx_buffer = unsafe { self.dma_driver().rx_buffer() };

        unsafe { self.start_transfer_dma(false, 0, bytes_to_write, rx_buffer, buffer) }
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
        let rx_buffer = unsafe { self.dma_driver().rx_buffer() };

        unsafe { self.start_dma_transfer(0, bytes_to_write, rx_buffer, buffer) }
    }

    /// Assigns copy buffers to the SPI driver.
    ///
    /// These buffers will be used to copy data when using the slice-based transfer functions.
    ///
    /// Data is copied in two cases:
    ///   - When the buffer is not located in a memory region that can be accessed by the DMA.
    #[cfg_attr(
        not(spi_master_dma_can_access_flash),
        doc = "The DMA cannot read flash memory."
    )]
    ///   - When the alignment of the buffer does not meet the DMA's requirements, the unaligned
    ///     parts of the buffer are copied.
    #[cfg_attr(
        spi_master_version = "1",
        doc = "On ESP32, transferring from internal SRAM requires copying the entire buffer if it is
not 4-byte aligned. This is a limitation of the current implementation."
    )]
    #[cfg_attr(
        spi_master_version = "2",
        doc = "On ESP32-S2, receiving into PSRAM requires the buffer's _end_ to be 16-byte
aligned, otherwise the driver requires copying the entire buffer."
    )]
    #[doc = ""]
    /// The maximum useful size for these buffers is 32736 bytes, any additional memory will
    /// be wasted.
    ///
    /// For an example of how to create these buffers, see the [`SpiDma`] documentation.
    #[instability::unstable]
    pub fn with_buffers(self, dma_rx_buf: DmaRxBuf, dma_tx_buf: DmaTxBuf) -> SpiDma<'d, Dm> {
        unsafe {
            (&mut *self.spi.dma_state().rx_buffer.get()).write(dma_rx_buf.into_scoped());
            (&mut *self.spi.dma_state().tx_buffer.get()).write(dma_tx_buf.into_scoped());
        }
        self
    }

    /// Perform a DMA write.
    ///
    /// This will return a [SpiDmaTransfer] owning the buffer and the
    /// SPI instance. The maximum amount of data to be sent is 32736
    /// bytes.
    #[allow(clippy::type_complexity)]
    #[cfg_attr(place_spi_master_driver_in_ram, ram)]
    #[instability::unstable]
    pub fn write_buffer<TX: DmaTxBuffer>(
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
        let tx_buffer = unsafe { self.dma_driver().tx_buffer() };

        unsafe { self.start_dma_transfer(bytes_to_read, 0, buffer, tx_buffer) }
    }

    /// Perform a DMA read.
    ///
    /// This will return a [SpiDmaTransfer] owning the buffer and
    /// the SPI instance. The maximum amount of data to be
    /// received is 32736 bytes.
    #[allow(clippy::type_complexity)]
    #[cfg_attr(place_spi_master_driver_in_ram, ram)]
    #[instability::unstable]
    pub fn read_buffer<RX: DmaRxBuffer>(
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
    pub fn transfer_buffers<RX: DmaRxBuffer, TX: DmaTxBuffer>(
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

        let tx_buffer = unsafe { self.dma_driver().tx_buffer() };

        unsafe { self.start_transfer_dma(false, bytes_to_read, 0, buffer, tx_buffer) }
    }

    /// Perform a half-duplex read operation using DMA.
    #[allow(clippy::type_complexity)]
    #[cfg_attr(place_spi_master_driver_in_ram, ram)]
    #[instability::unstable]
    pub fn half_duplex_read_buffer<RX: DmaRxBuffer>(
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
        #[cfg(all(spi_master_version = "1", spi_address_workaround))]
        {
            // On the ESP32, if we don't have data, the address is always sent
            // on a single line, regardless of its data mode.
            if bytes_to_write == 0 && address.mode() != DataMode::SingleTwoDataLines {
                return unsafe { self.set_up_address_workaround(cmd, address, dummy) };
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

        let rx_buffer = unsafe { self.dma_driver().rx_buffer() };

        unsafe { self.start_transfer_dma(false, 0, bytes_to_write, rx_buffer, buffer) }
    }

    /// Perform a half-duplex write operation using DMA.
    #[allow(clippy::type_complexity)]
    #[cfg_attr(place_spi_master_driver_in_ram, ram)]
    #[instability::unstable]
    pub fn half_duplex_write_buffer<TX: DmaTxBuffer>(
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

    #[doc_replace(
        "max_frequency" => {
            cfg(esp32h2) => "48MHz",
            _ => "80MHz",
        }
    )]
    /// Change the bus configuration.
    ///
    /// # Errors
    ///
    /// If frequency passed in config exceeds __max_frequency__ or is below 70kHz,
    /// [`ConfigError::UnsupportedFrequency`] error will be returned.
    #[instability::unstable]
    pub fn apply_config(&mut self, config: &Config) -> Result<(), ConfigError> {
        self.driver().apply_config(config)
    }

    fn transfer_buffers_dma(
        &mut self,
        read_bytes: usize,
        write_bytes: usize,
        mut rx_buffer: impl DmaRxBuffer,
        mut tx_buffer: impl DmaTxBuffer,
    ) -> Result<(), Error> {
        unsafe {
            self.start_dma_transfer(read_bytes, write_bytes, &mut rx_buffer, &mut tx_buffer)?;
        }
        self.wait_for_idle();
        Ok(())
    }

    /// Reads data from the SPI bus using DMA.
    #[instability::unstable]
    pub fn read(&mut self, words: &mut [u8]) -> Result<(), Error> {
        self.wait_for_idle();
        self.driver().setup_full_duplex()?;

        if self.use_blocking_transfer(words.len()) {
            self.dma_driver().disable_dma();
            return self.driver().read(words);
        }

        let mut descriptors = [DmaDescriptor::EMPTY; LINK_DESCRIPTOR_COUNT];
        let mut maybe_copy_buffer = match DmaOperationKind::for_read(words) {
            DmaOperationKind::Copied => {
                MaybeCopyRxBuf::Copy(unsafe { self.spi.dma_state().rx_buffer() })
            }
            DmaOperationKind::InPlace => MaybeCopyRxBuf::Direct {
                descriptors: &mut descriptors,
                #[cfg(dma_can_access_psram)]
                align_buffer: [const { None }; 2],
            },
        };

        if maybe_copy_buffer.chunk_size() == 0 {
            return Err(Error::from(DmaError::BufferTooSmall));
        }

        for chunk in words.chunks_mut(maybe_copy_buffer.chunk_size()) {
            let read_bytes = chunk.len();
            let rx_buffer = unsafe { maybe_copy_buffer.setup(NonNull::from(&mut *chunk)) };
            let tx_buffer = unsafe { NoBuffer(self.spi.dma_state().tx_buffer().prepare()) };

            self.transfer_buffers_dma(read_bytes, 0, rx_buffer, tx_buffer)?;

            maybe_copy_buffer.finish(chunk);
        }

        Ok(())
    }

    /// Writes data to the SPI bus using DMA.
    #[instability::unstable]
    pub fn write(&mut self, words: &[u8]) -> Result<(), Error> {
        self.wait_for_idle();
        self.driver().setup_full_duplex()?;

        if self.use_blocking_transfer(words.len()) {
            self.dma_driver().disable_dma();
            self.driver().write(words)?;
            return self.driver().flush();
        }

        let mut descriptors = [DmaDescriptor::EMPTY; LINK_DESCRIPTOR_COUNT];
        let mut maybe_copy_buffer = match DmaOperationKind::for_write(words) {
            DmaOperationKind::Copied => {
                MaybeCopyTxBuf::Copy(unsafe { self.spi.dma_state().tx_buffer() })
            }
            DmaOperationKind::InPlace => MaybeCopyTxBuf::Direct(&mut descriptors),
        };

        if maybe_copy_buffer.chunk_size() == 0 {
            return Err(Error::from(DmaError::BufferTooSmall));
        }

        for chunk in words.chunks(maybe_copy_buffer.chunk_size()) {
            let write_bytes = chunk.len();
            let rx_buffer = unsafe { NoBuffer(self.spi.dma_state().rx_buffer().prepare()) };
            let tx_buffer = unsafe { maybe_copy_buffer.setup(NonNull::from(chunk)) };

            self.transfer_buffers_dma(0, write_bytes, rx_buffer, tx_buffer)?;
        }

        Ok(())
    }

    /// Transfers data to and from the SPI bus simultaneously using DMA.
    #[instability::unstable]
    pub fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Error> {
        self.wait_for_idle();
        self.driver().setup_full_duplex()?;

        if self.use_blocking_transfer(read.len().max(write.len())) {
            self.dma_driver().disable_dma();
            if read.is_empty() {
                self.driver().write(write)?;
                return self.driver().flush();
            } else if write.is_empty() {
                return self.driver().read(read);
            } else {
                return self.driver().transfer(read, write);
            }
        }

        let mut rx_descriptors = [DmaDescriptor::EMPTY; LINK_DESCRIPTOR_COUNT];
        let mut maybe_copy_rx_buffer = match DmaOperationKind::for_read(read) {
            DmaOperationKind::Copied => {
                MaybeCopyRxBuf::Copy(unsafe { self.spi.dma_state().rx_buffer() })
            }
            DmaOperationKind::InPlace => MaybeCopyRxBuf::Direct {
                descriptors: &mut rx_descriptors,
                #[cfg(dma_can_access_psram)]
                align_buffer: [const { None }; 2],
            },
        };

        let mut tx_descriptors = [DmaDescriptor::EMPTY; LINK_DESCRIPTOR_COUNT];
        let mut maybe_copy_tx_buffer = match DmaOperationKind::for_write(write) {
            DmaOperationKind::Copied => {
                MaybeCopyTxBuf::Copy(unsafe { self.spi.dma_state().tx_buffer() })
            }
            DmaOperationKind::InPlace => MaybeCopyTxBuf::Direct(&mut tx_descriptors),
        };

        let chunk_size = min(
            maybe_copy_rx_buffer.chunk_size(),
            maybe_copy_tx_buffer.chunk_size(),
        );

        if chunk_size == 0 {
            return Err(Error::from(DmaError::BufferTooSmall));
        }

        let common_length = min(read.len(), write.len());
        let (read_common, read_remainder) = read.split_at_mut(common_length);
        let (write_common, write_remainder) = write.split_at(common_length);

        for (read_chunk, write_chunk) in read_common
            .chunks_mut(chunk_size)
            .zip(write_common.chunks(chunk_size))
        {
            let read_bytes = read_chunk.len();
            let write_bytes = write_chunk.len();
            let tx_buffer = unsafe { maybe_copy_tx_buffer.setup(NonNull::from(write_chunk)) };
            let rx_buffer = unsafe { maybe_copy_rx_buffer.setup(NonNull::from(&mut *read_chunk)) };

            self.transfer_buffers_dma(read_bytes, write_bytes, rx_buffer, tx_buffer)?;

            maybe_copy_rx_buffer.finish(read_chunk);
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
        self.driver().setup_full_duplex()?;

        if self.use_blocking_transfer(words.len()) {
            self.dma_driver().disable_dma();
            return self.driver().transfer_in_place(words);
        }

        let mut rx_descriptors = [DmaDescriptor::EMPTY; LINK_DESCRIPTOR_COUNT];
        let mut tx_descriptors = [DmaDescriptor::EMPTY; LINK_DESCRIPTOR_COUNT];
        let (mut maybe_copy_rx_buffer, mut maybe_copy_tx_buffer) =
            match DmaOperationKind::for_write(words) {
                DmaOperationKind::Copied => (
                    MaybeCopyRxBuf::Copy(unsafe { self.spi.dma_state().rx_buffer() }),
                    MaybeCopyTxBuf::Copy(unsafe { self.spi.dma_state().tx_buffer() }),
                ),
                DmaOperationKind::InPlace => (
                    MaybeCopyRxBuf::Direct {
                        descriptors: &mut rx_descriptors,
                        #[cfg(dma_can_access_psram)]
                        align_buffer: [const { None }; 2],
                    },
                    MaybeCopyTxBuf::Direct(&mut tx_descriptors),
                ),
            };

        let chunk_size = min(
            maybe_copy_rx_buffer.chunk_size(),
            maybe_copy_tx_buffer.chunk_size(),
        );

        if chunk_size == 0 {
            return Err(Error::from(DmaError::BufferTooSmall));
        }

        for chunk in words.chunks_mut(chunk_size) {
            let bytes = chunk.len();
            let ptr = NonNull::from(&mut *chunk);
            let tx_buffer = unsafe { maybe_copy_tx_buffer.setup(ptr) };
            let rx_buffer = unsafe { maybe_copy_rx_buffer.setup(ptr) };

            self.transfer_buffers_dma(bytes, bytes, rx_buffer, tx_buffer)?;

            maybe_copy_rx_buffer.finish(chunk);
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
        self.wait_for_idle();

        let rx_buffer = unsafe { self.dma_driver().rx_buffer() };
        if rx_buffer.capacity() == 0 {
            return Err(Error::from(DmaError::BufferTooSmall));
        }
        if buffer.len() > rx_buffer.capacity() {
            return Err(Error::from(DmaError::Overflow));
        }

        unsafe {
            self.start_half_duplex_read(data_mode, cmd, address, dummy, buffer.len(), rx_buffer)?;
        }

        self.wait_for_idle();

        buffer.copy_from_slice(&rx_buffer.as_slice()[..buffer.len()]);

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
        self.wait_for_idle();

        let tx_buffer = unsafe { self.dma_driver().tx_buffer() };
        if tx_buffer.capacity() == 0 {
            return Err(Error::from(DmaError::BufferTooSmall));
        }
        if buffer.len() > tx_buffer.capacity() {
            return Err(Error::from(DmaError::Overflow));
        }

        tx_buffer.as_mut_slice()[..buffer.len()].copy_from_slice(buffer);

        unsafe {
            self.start_half_duplex_write(data_mode, cmd, address, dummy, buffer.len(), tx_buffer)?;
        }

        self.wait_for_idle();

        Ok(())
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
        }

        unsafe {
            ManuallyDrop::drop(&mut self.spi_dma);
            ManuallyDrop::drop(&mut self.dma_buf);
        }
    }
}

pub(super) struct DmaDriver {
    driver: Driver,
    dma_peripheral: crate::dma::DmaPeripheral,
    state: &'static DmaState,
}

impl DmaDriver {
    unsafe fn rx_buffer(&self) -> &'static mut ScopedDmaRxBuf<'static> {
        unsafe { self.state.rx_buffer() }
    }

    unsafe fn tx_buffer(&self) -> &'static mut ScopedDmaTxBuf<'static> {
        unsafe { self.state.tx_buffer() }
    }

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

    fn disable_dma(&self) {
        #[cfg(dma_kind = "gdma")]
        self.regs().dma_conf().modify(|_, w| {
            w.dma_tx_ena().clear_bit();
            w.dma_rx_ena().clear_bit()
        });

        // PDMA: nothing to do
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
        #[cfg(spi_master_version = "2")]
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
            #[cfg(spi_master_version = "1")]
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

struct DmaInfo {
    dma_peripheral: crate::dma::DmaPeripheral,
}
struct DmaState {
    tx_transfer_in_progress: Cell<bool>,
    rx_transfer_in_progress: Cell<bool>,

    rx_buffer: UnsafeCell<MaybeUninit<ScopedDmaRxBuf<'static>>>,
    tx_buffer: UnsafeCell<MaybeUninit<ScopedDmaTxBuf<'static>>>,
}

impl DmaState {
    // Syntactic helper to get a mutable reference to the "empty" RX DMA buffer.
    //
    // # Safety
    //
    // The caller must ensure that Rust's aliasing rules are upheld.
    #[allow(
        clippy::mut_from_ref,
        reason = "Safety requirements ensure this is okay"
    )]
    unsafe fn rx_buffer(&self) -> &mut ScopedDmaRxBuf<'static> {
        unsafe { (&mut *self.rx_buffer.get()).assume_init_mut() }
    }

    // Syntactic helper to get a mutable reference to the "empty" TX DMA buffer.
    //
    // # Safety
    //
    // The caller must ensure that Rust's aliasing rules are upheld.
    #[allow(
        clippy::mut_from_ref,
        reason = "Safety requirements ensure this is okay"
    )]
    unsafe fn tx_buffer(&self) -> &mut ScopedDmaTxBuf<'static> {
        unsafe { (&mut *self.tx_buffer.get()).assume_init_mut() }
    }
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

                                rx_buffer: UnsafeCell::new(MaybeUninit::uninit()),
                                tx_buffer: UnsafeCell::new(MaybeUninit::uninit()),
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
