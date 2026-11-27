#[cfg(dma_can_access_psram)]
use core::{mem::MaybeUninit, ops::Range};
use core::{
    ops::{Deref, DerefMut},
    ptr::{NonNull, null_mut},
};

use super::*;
#[cfg(dma_can_access_psram)]
use crate::soc::{is_slice_in_psram, is_valid_psram_address, is_valid_ram_address};
use crate::{
    dma::aligned::{DmaAlignedMut, InternalMemory},
    soc::is_slice_in_dram,
};

pub(crate) mod scoped;
pub(crate) use scoped::*;

/// Error returned from Dma[Rx|Tx|RxTx]Buf operations.
#[derive(Debug, PartialEq, Eq, Hash, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DmaBufError {
    /// The buffer is smaller than the requested size.
    BufferTooSmall,

    /// More descriptors are needed for the buffer size.
    InsufficientDescriptors,

    /// Descriptors or buffers are not located in a supported memory region.
    UnsupportedMemoryRegion,

    /// Buffer address or size is not properly aligned.
    InvalidAlignment(DmaAlignmentError),

    /// Invalid chunk size: must be > 0 and <= 4095.
    InvalidChunkSize,
}

impl core::fmt::Display for DmaBufError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            DmaBufError::BufferTooSmall => {
                write!(f, "The buffer is smaller than the requested size")
            }
            DmaBufError::InsufficientDescriptors => {
                write!(f, "More descriptors are needed for the buffer size")
            }
            DmaBufError::UnsupportedMemoryRegion => write!(
                f,
                "Descriptors or buffers are not located in a supported memory region"
            ),
            DmaBufError::InvalidAlignment(x) => write!(f, "{x}"),
            DmaBufError::InvalidChunkSize => {
                write!(f, "Invalid chunk size: must be > 0 and <= 4095")
            }
        }
    }
}

impl core::error::Error for DmaBufError {}

/// DMA buffer alignment errors.
#[derive(Debug, PartialEq, Eq, Hash, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DmaAlignmentError {
    /// Buffer address is not properly aligned.
    Address,

    /// Buffer size is not properly aligned.
    Size,
}

impl From<DmaAlignmentError> for DmaBufError {
    fn from(err: DmaAlignmentError) -> Self {
        DmaBufError::InvalidAlignment(err)
    }
}

impl core::fmt::Display for DmaAlignmentError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            DmaAlignmentError::Address => write!(f, "Buffer address is not properly aligned"),
            DmaAlignmentError::Size => write!(f, "Buffer size is not properly aligned"),
        }
    }
}

impl core::error::Error for DmaAlignmentError {}

/// A burst alignment that a buffer guarantees.
///
/// Some DMA engines need an aligned buffer for a data burst. These engines use a smaller burst
/// than the configured one when the buffer guarantees less alignment. The other engines need no
/// aligned buffer and ignore it.
///
/// A buffer created with a minimum burst, for example with [`DmaRxBuf::new_with_min_burst`],
/// guarantees that burst for its whole life, so it does not limit a burst up to that size. A
/// buffer created without one guarantees the natural alignment of its address and length, up to
/// the largest burst that needs alignment on the chip.
///
/// The burst types of the engine configurations convert into this type.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
#[repr(u8)]
pub enum BurstSize {
    /// 4 bytes.
    Size4   = 2,

    /// 8 bytes.
    Size8   = 3,

    /// 16 bytes.
    Size16  = 4,

    /// 32 bytes.
    Size32  = 5,

    /// 64 bytes.
    Size64  = 6,

    /// 128 bytes.
    Size128 = 7,
}

impl BurstSize {
    /// Returns the size in bytes.
    pub const fn bytes(self) -> usize {
        1 << self as u8
    }

    /// Returns the size whose log2 is `log2`. Values below 2 give `Size4`, values above 7 give
    /// `Size128`.
    pub(crate) const fn from_log2(log2: u8) -> Self {
        match log2 {
            0..=2 => Self::Size4,
            3 => Self::Size8,
            4 => Self::Size16,
            5 => Self::Size32,
            6 => Self::Size64,
            _ => Self::Size128,
        }
    }
}

pub(crate) const fn max(a: usize, b: usize) -> usize {
    if a > b { a } else { b }
}

const fn chunk_size_for_alignment(alignment: usize) -> usize {
    // DMA descriptors have a 12-bit field for the size/length of the buffer they
    // point at. As there is no such thing as 0-byte alignment, this means the
    // maximum size is 4095 bytes.
    4096 - alignment
}

/// Returns a chunk size that keeps the alignment of a buffer in internal memory that guarantees
/// `min`, in any direction.
#[doc(hidden)]
pub const fn max_compatible_chunk_size(min: Option<BurstSize>) -> usize {
    let alignment = max(MAX_ALIGNED_BURST, BurstAlignment(min).min_dram_alignment());
    chunk_size_for_alignment(alignment)
}

/// The alignment rules of a buffer: the minimum burst that it guarantees, if any.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(crate) struct BurstAlignment(pub(crate) Option<BurstSize>);

impl BurstAlignment {
    /// A buffer that guarantees only its natural alignment.
    pub(crate) const NATURAL: Self = Self(None);

    // Size and address alignment as those come in pairs on current hardware.
    const fn min_dram_alignment(self) -> usize {
        // ESP32 SPI DMA: Burst transmission is supported. The data size for
        // a single transfer must be four bytes aligned.
        // ESP32 I2S DMA: Burst transfer is supported. However, unlike the
        // SPI DMA channels, the data size for a single transfer is
        // one word, or four bytes.
        // NOTE: The buffer address must be word-aligned in both directions.
        let alignment = if cfg!(esp32) { 4 } else { 1 };
        match self.0 {
            Some(min) => max(alignment, min.bytes()),
            None => alignment,
        }
    }

    /// The alignment that the address of `buffer`, and the length of a receive buffer, must
    /// have.
    fn min_alignment(self, _buffer: &[u8], _direction: TransferDirection) -> usize {
        let alignment = self.min_dram_alignment();

        #[cfg(dma_can_access_psram)]
        let alignment = max(alignment, min_psram_alignment(_buffer, _direction));

        alignment
    }

    // Note: this function ignores address alignment as we assume the buffers are
    // aligned.
    fn max_chunk_size_for(self, buffer: &[u8], direction: TransferDirection) -> usize {
        let alignment = self.min_alignment(buffer, direction);
        // Every descriptor of a receive buffer keeps the alignment that the buffer guarantees.
        let alignment = if direction == TransferDirection::In {
            max(alignment, MAX_ALIGNED_BURST)
        } else {
            alignment
        };
        chunk_size_for_alignment(alignment)
    }

    /// Returns the burst alignment that a buffer at `addr` guarantees: its natural alignment up
    /// to [`MAX_ALIGNED_BURST`], and at least the minimum burst. `len` is the length of a receive
    /// buffer, or the bitwise OR of its descriptor sizes.
    fn guarantee(self, addr: usize, len: usize, direction: TransferDirection) -> Option<BurstSize> {
        let mut log2 = addr.trailing_zeros();
        if direction == TransferDirection::In {
            log2 = log2.min(len.trailing_zeros());
        }
        let log2 = log2.min(MAX_ALIGNED_BURST.trailing_zeros());
        let log2 = log2.max(self.0.map_or(0, |min| min as u32));

        (log2 >= BurstSize::Size4 as u32).then(|| BurstSize::from_log2(log2 as u8))
    }

    fn ensure_buffer_aligned(
        self,
        buffer: &[u8],
        direction: TransferDirection,
    ) -> Result<(), DmaAlignmentError> {
        let alignment = self.min_alignment(buffer, direction);
        if !(buffer.as_ptr() as usize).is_multiple_of(alignment) {
            return Err(DmaAlignmentError::Address);
        }

        // NB: the TRMs suggest that buffer length don't need to be aligned, but
        // for IN transfers, we configure the DMA descriptors' size field, which needs
        // to be aligned.
        let size_alignment = cfg_select! {
            dma_can_access_psram => match direction {
                TransferDirection::In => alignment,
                TransferDirection::Out => min_psram_alignment(buffer, direction),
            },
            _ => match direction {
                TransferDirection::In => alignment,
                TransferDirection::Out => 1,
            },
        };
        if !buffer.len().is_multiple_of(size_alignment) {
            return Err(DmaAlignmentError::Size);
        }

        Ok(())
    }

    fn ensure_buffer_compatible(
        self,
        buffer: &[u8],
        direction: TransferDirection,
    ) -> Result<(), DmaBufError> {
        if buffer.is_empty() {
            return Ok(());
        }
        // buffer can be either DRAM or PSRAM (if supported)
        let is_in_dram = is_slice_in_dram(buffer);
        cfg_select! {
            dma_can_access_psram => {
                let is_in_psram = is_slice_in_psram(buffer);
            }
            _ => {
                let is_in_psram = false;
            }
        }

        if !(is_in_dram || is_in_psram) {
            return Err(DmaBufError::UnsupportedMemoryRegion);
        }

        self.ensure_buffer_aligned(buffer, direction)?;

        Ok(())
    }
}

/// The alignment that a buffer in PSRAM needs, in address and size. 1 for other buffers.
#[cfg(dma_can_access_psram)]
fn min_psram_alignment(buffer: &[u8], direction: TransferDirection) -> usize {
    if !is_valid_psram_address(buffer.as_ptr() as usize) {
        return 1;
    }

    // S2 TRM: Specifically, size and buffer address pointer in receive descriptors
    // should be 16-byte, 32-byte or 64-byte aligned. For data frame whose
    // length is not a multiple of 16 bytes, 32 bytes, or 64 bytes, EDMA adds
    // padding bytes to the end.

    // S3 TRM: Size and Address for IN transfers must be block aligned. For receive
    // descriptors, if the data length received are not aligned with block size,
    // GDMA will pad the data received with 0 until they are aligned to
    // initiate burst transfer. You can read the length field in receive descriptors
    // to obtain the length of valid data received

    // S2 TRM: Size, length and buffer address pointer in transmit descriptors are
    // not necessarily aligned with block size.

    // S3 TRM: Size, length, and buffer address pointer in transmit descriptors do
    // not need to be aligned.

    // The smallest block is 16 bytes. The transfer uses a larger block only when the buffer
    // guarantees it.
    if cfg!(dma_ext_mem_configurable_block_size) && direction == TransferDirection::In {
        return BurstSize::Size16.bytes();
    }

    // With flash encryption, the MSPI needs PSRAM accesses that are aligned to 16 bytes.
    if crate::efuse::flash_encryption() {
        return BurstSize::Size16.bytes();
    }

    1
}

/// The direction of the DMA transfer.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TransferDirection {
    /// DMA transfer from peripheral or external memory to memory.
    In,
    /// DMA transfer from memory to peripheral or external memory.
    Out,
}

/// Holds all the information needed to configure a DMA channel for a transfer.
#[derive(PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Preparation {
    /// The descriptor the DMA will start from.
    pub start: *mut DmaDescriptor,

    /// Must be `true` if any of the DMA descriptors contain data in PSRAM.
    #[cfg(dma_can_access_psram)]
    pub accesses_psram: bool,

    /// The burst alignment that the buffer guarantees, or `None` for less than 4 bytes.
    ///
    /// The address and size of every receive descriptor must be a multiple of this size, and
    /// the address of every transmit descriptor. DMA engines that need aligned buffers for a
    /// burst use a burst no larger than this. See [`BurstSize`].
    ///
    /// For details on alignment requirements, refer to the chip's
    #[doc = crate::trm_markdown_link!()]
    pub burst_align: Option<BurstSize>,

    /// Configures the "check owner" feature of the DMA channel.
    ///
    /// Most DMA channels allow software to configure whether the hardware
    /// checks that [DmaDescriptor::owner] is set to [Owner::Dma] before
    /// consuming the descriptor. If this check fails, the channel stops
    /// operating and fires
    /// [DmaRxInterrupt::DescriptorError]/[DmaTxInterrupt::DescriptorError].
    ///
    /// This field allows buffer implementation to configure this behavior.
    /// - `Some(true)`: DMA channel must check the owner bit.
    /// - `Some(false)`: DMA channel must NOT check the owner bit.
    /// - `None`: DMA channel should check the owner bit if it is supported.
    ///
    /// Some buffer implementations may require that the DMA channel performs
    /// this check before consuming the descriptor to ensure correct
    /// behavior. e.g. To prevent wrap-around in a circular transfer.
    ///
    /// Some buffer implementations may require that the DMA channel does NOT
    /// perform this check as the ownership bit will not be set before the
    /// channel tries to consume the descriptor.
    ///
    /// Most implementations do not have any such requirements and work
    /// correctly regardless of whether the DMA channel checks or not.
    ///
    /// If the DMA channel does not support the provided option, preparation
    /// fails.
    pub check_owner: Option<bool>,

    /// Configures whether the DMA channel automatically clears the
    /// [DmaDescriptor::owner] bit after it is done with the buffer pointed
    /// to by a descriptor.
    ///
    /// For RX transfers, this is always true and the value specified here is
    /// ignored.
    ///
    /// SPI_DMA on the ESP32 does not support this and panics if set
    /// to true.
    pub auto_write_back: bool,
}

/// [`DmaTxBuffer`] is a DMA descriptor + memory combo that can be used for
/// transmitting data from a DMA channel to a peripheral's FIFO.
///
/// # Safety
///
/// The implementing type must keep all its descriptors and the buffers they
/// point to valid while the buffer is being transferred.
pub unsafe trait DmaTxBuffer {
    /// A type providing operations that are safe to perform on the buffer
    /// while the DMA is actively using it.
    type View;

    /// The type returned when a transfer finishes.
    ///
    /// Some buffers do not need to be reconstructed.
    type Final;

    /// Prepares the buffer for an imminent transfer and returns
    /// information required to use this buffer.
    ///
    /// This operation is idempotent.
    fn prepare(&mut self) -> Preparation;

    /// This is called before the DMA starts using the buffer.
    fn into_view(self) -> Self::View;

    /// This is called after the DMA is done using the buffer.
    fn from_view(view: Self::View) -> Self::Final;
}

/// [`DmaRxBuffer`] is a DMA descriptor + memory combo that can be used for
/// receiving data from a peripheral's FIFO to a DMA channel.
///
/// Implementations of this trait may only support having a single EOF bit
/// which resides in the last descriptor. There will be a separate trait in
/// future to support multiple EOFs.
///
/// # Safety
///
/// The implementing type must keep all its descriptors and the buffers they
/// point to valid while the buffer is being transferred.
pub unsafe trait DmaRxBuffer {
    /// A type providing operations that are safe to perform on the buffer
    /// while the DMA is actively using it.
    type View;

    /// The type returned when a transfer finishes.
    ///
    /// Some buffers do not need to be reconstructed.
    type Final;

    /// Prepares the buffer for an imminent transfer and returns
    /// information required to use this buffer.
    ///
    /// This operation is idempotent.
    fn prepare(&mut self) -> Preparation;

    /// This is called before the DMA starts using the buffer.
    fn into_view(self) -> Self::View;

    /// This is called after the DMA is done using the buffer.
    fn from_view(view: Self::View) -> Self::Final;
}

/// An in-progress view into [`DmaRxBuf`]/[`DmaTxBuf`].
///
/// In the future, this could support peeking into state of the
/// descriptors/buffers.
pub struct BufView<T>(T);

/// DMA transmit buffer
///
/// This is a contiguous buffer linked together by DMA descriptors of length
/// 4095 at most. It can only be used for transmitting data to a peripheral's
/// FIFO. See [`DmaRxBuf`] for receiving data.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DmaTxBuf(ScopedDmaTxBuf<'static>);

impl DmaTxBuf {
    /// Creates a new [`DmaTxBuf`] from some descriptors and a buffer.
    pub fn new(
        descriptors: DmaAlignedMut<'static, [DmaDescriptor]>,
        buffer: DmaAlignedMut<'static, [u8]>,
    ) -> Result<Self, DmaBufError> {
        ScopedDmaTxBuf::new(descriptors, buffer).map(Self)
    }

    /// Creates a new [`DmaTxBuf`] that guarantees an alignment of at least `min`.
    ///
    /// There must be enough descriptors for the provided buffer.
    /// Depending on alignment requirements, each descriptor can handle at most
    /// 4095 bytes worth of buffer.
    ///
    /// Both the descriptors and buffer must be in DMA-capable memory.
    /// Only DRAM is supported for descriptors.
    ///
    /// # Errors
    ///
    /// [`DmaBufError::InvalidAlignment`] if the address of the buffer is not a multiple of
    /// `min`.
    pub fn new_with_min_burst(
        descriptors: DmaAlignedMut<'static, [DmaDescriptor]>,
        buffer: DmaAlignedMut<'static, [u8]>,
        min: BurstSize,
    ) -> Result<Self, DmaBufError> {
        ScopedDmaTxBuf::new_with_min_burst(descriptors, buffer, min).map(Self)
    }

    /// Consumes the buf, returning the descriptors and buffer.
    pub fn split(
        self,
    ) -> (
        DmaAlignedMut<'static, [DmaDescriptor]>,
        DmaAlignedMut<'static, [u8]>,
    ) {
        self.0.split()
    }

    /// Returns the size of the underlying buffer.
    pub fn capacity(&self) -> usize {
        self.0.capacity()
    }

    /// Returns the number of bytes that would be transmitted by this buf.
    #[allow(clippy::len_without_is_empty)]
    pub fn len(&self) -> usize {
        self.0.len()
    }

    /// Resets the descriptors to only transmit `len` amount of bytes from this
    /// buf.
    ///
    /// The number of bytes in data must be less than or equal to the buffer
    /// size.
    pub fn set_length(&mut self, len: usize) {
        self.0.set_length(len);
    }

    /// Fills the TX buffer with the bytes provided in `data` and reset the
    /// descriptors to only cover the filled section.
    ///
    /// The number of bytes in data must be less than or equal to the buffer
    /// size.
    pub fn fill(&mut self, data: &[u8]) {
        self.0.fill(data);
    }

    /// Returns the buf as a mutable slice that can be written.
    pub fn as_mut_slice(&mut self) -> &mut [u8] {
        self.0.as_mut_slice()
    }

    /// Returns the buf as a slice that can be read.
    pub fn as_slice(&self) -> &[u8] {
        self.0.as_slice()
    }

    /// Consumes the buffer and returns the scoped version.
    pub(crate) fn into_scoped(self) -> ScopedDmaTxBuf<'static> {
        self.0
    }
}

unsafe impl DmaTxBuffer for DmaTxBuf {
    type View = BufView<DmaTxBuf>;
    type Final = DmaTxBuf;

    fn prepare(&mut self) -> Preparation {
        self.0.prepare()
    }

    fn into_view(self) -> BufView<DmaTxBuf> {
        BufView(self)
    }

    fn from_view(view: Self::View) -> Self {
        view.0
    }
}

/// DMA receive buffer
///
/// This is a contiguous buffer linked together by DMA descriptors of length
/// 4092. It can only be used for receiving data from a peripheral's FIFO.
/// See [`DmaTxBuf`] for transmitting data.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DmaRxBuf(ScopedDmaRxBuf<'static>);

impl DmaRxBuf {
    /// Creates a new [`DmaRxBuf`] from some descriptors and a buffer.
    pub fn new(
        descriptors: DmaAlignedMut<'static, [DmaDescriptor]>,
        buffer: DmaAlignedMut<'static, [u8]>,
    ) -> Result<Self, DmaBufError> {
        ScopedDmaRxBuf::new(descriptors, buffer).map(Self)
    }

    /// Creates a new [`DmaRxBuf`] that guarantees an alignment of at least `min`.
    ///
    /// There must be enough descriptors for the provided buffer. Each descriptor
    /// handles at most `4096 - min.bytes()` bytes of the buffer, or less,
    /// depending on alignment requirements.
    ///
    /// Both the descriptors and buffer must be in DMA-capable memory.
    /// Only DRAM is supported for descriptors.
    ///
    /// [`Self::set_length`] panics if the length is not a multiple of `min`.
    ///
    /// # Errors
    ///
    /// [`DmaBufError::InvalidAlignment`] if the address or the length of the buffer
    /// is not a multiple of `min`.
    pub fn new_with_min_burst(
        descriptors: DmaAlignedMut<'static, [DmaDescriptor]>,
        buffer: DmaAlignedMut<'static, [u8]>,
        min: BurstSize,
    ) -> Result<Self, DmaBufError> {
        ScopedDmaRxBuf::new_with_min_burst(descriptors, buffer, min).map(Self)
    }

    /// Consumes the buf, returning the descriptors and buffer.
    pub fn split(
        self,
    ) -> (
        DmaAlignedMut<'static, [DmaDescriptor]>,
        DmaAlignedMut<'static, [u8]>,
    ) {
        self.0.split()
    }

    /// Returns the size of the underlying buffer.
    pub fn capacity(&self) -> usize {
        self.0.capacity()
    }

    /// Returns the maximum number of bytes that this buf has been configured to
    /// receive.
    #[allow(clippy::len_without_is_empty)]
    pub fn len(&self) -> usize {
        self.0.len()
    }

    /// Resets the descriptors to only receive `len` amount of bytes into this
    /// buf.
    ///
    /// The number of bytes in data must be less than or equal to the buffer
    /// size.
    pub fn set_length(&mut self, len: usize) {
        self.0.set_length(len)
    }

    /// Returns the entire underlying buffer as a slice that can be read.
    pub fn as_slice(&self) -> &[u8] {
        self.0.as_slice()
    }

    /// Returns the entire underlying buffer as a slice that can be written.
    pub fn as_mut_slice(&mut self) -> &mut [u8] {
        self.0.as_mut_slice()
    }

    /// Returns the number of bytes that was received by this buf.
    pub fn number_of_received_bytes(&self) -> usize {
        self.0.number_of_received_bytes()
    }

    /// Reads the received data into the provided `buf`.
    ///
    /// If `buf.len()` is less than the amount of received data then only the
    /// first `buf.len()` bytes of received data is written into `buf`.
    ///
    /// Returns the number of bytes written to `buf`.
    pub fn read_received_data(&self, buf: &mut [u8]) -> usize {
        self.0.read_received_data(buf)
    }

    /// Returns the received data as an iterator of slices.
    pub fn received_data(&self) -> impl Iterator<Item = &[u8]> {
        self.0.received_data()
    }

    /// Consumes the buffer and returns the scoped version.
    pub(crate) fn into_scoped(self) -> ScopedDmaRxBuf<'static> {
        self.0
    }
}

unsafe impl DmaRxBuffer for DmaRxBuf {
    type View = BufView<DmaRxBuf>;
    type Final = DmaRxBuf;

    fn prepare(&mut self) -> Preparation {
        self.0.prepare()
    }

    fn into_view(self) -> BufView<DmaRxBuf> {
        BufView(self)
    }

    fn from_view(view: Self::View) -> Self {
        view.0
    }
}

/// DMA transmit and receive buffer.
///
/// This is a (single) contiguous buffer linked together by two sets of DMA
/// descriptors of length 4092 each.
/// It can be used for simultaneously transmitting to and receiving from a
/// peripheral's FIFO. These are typically full-duplex transfers.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DmaRxTxBuf {
    rx_descriptors: DescriptorSet<'static>,
    tx_descriptors: DescriptorSet<'static>,
    buffer: DmaAlignedMut<'static, [u8]>,
    alignment: BurstAlignment,
    burst_align: Option<BurstSize>,
}

impl DmaRxTxBuf {
    /// Creates a new [DmaRxTxBuf] from some descriptors and a buffer.
    pub fn new(
        rx_descriptors: DmaAlignedMut<'static, [DmaDescriptor]>,
        tx_descriptors: DmaAlignedMut<'static, [DmaDescriptor]>,
        buffer: DmaAlignedMut<'static, [u8]>,
    ) -> Result<Self, DmaBufError> {
        Self::new_with_alignment(
            rx_descriptors,
            tx_descriptors,
            buffer,
            BurstAlignment::NATURAL,
        )
    }

    /// Creates a new [DmaRxTxBuf] that guarantees an alignment of at least `min`.
    ///
    /// [`Self::set_length`] panics if the length is not a multiple of `min`.
    ///
    /// # Errors
    ///
    /// [`DmaBufError::InvalidAlignment`] if the address or the length of the buffer
    /// is not a multiple of `min`.
    pub fn new_with_min_burst(
        rx_descriptors: DmaAlignedMut<'static, [DmaDescriptor]>,
        tx_descriptors: DmaAlignedMut<'static, [DmaDescriptor]>,
        buffer: DmaAlignedMut<'static, [u8]>,
        min: BurstSize,
    ) -> Result<Self, DmaBufError> {
        Self::new_with_alignment(
            rx_descriptors,
            tx_descriptors,
            buffer,
            BurstAlignment(Some(min)),
        )
    }

    fn new_with_alignment(
        rx_descriptors: DmaAlignedMut<'static, [DmaDescriptor]>,
        tx_descriptors: DmaAlignedMut<'static, [DmaDescriptor]>,
        buffer: DmaAlignedMut<'static, [u8]>,
        alignment: BurstAlignment,
    ) -> Result<Self, DmaBufError> {
        let mut buf = Self {
            rx_descriptors: DescriptorSet::new(rx_descriptors)?,
            tx_descriptors: DescriptorSet::new(tx_descriptors)?,
            buffer,
            alignment,
            burst_align: None,
        };

        let capacity = buf.capacity();
        buf.set_length_fallible(capacity)?;

        let max_chunk_size_in = alignment.max_chunk_size_for(&buf.buffer, TransferDirection::In);
        let max_chunk_size_out = alignment.max_chunk_size_for(&buf.buffer, TransferDirection::Out);
        buf.rx_descriptors
            .link_with_buffer(&mut buf.buffer, max_chunk_size_in)?;
        buf.tx_descriptors
            .link_with_buffer(&mut buf.buffer, max_chunk_size_out)?;

        Ok(buf)
    }

    /// Consumes the buf, returning the rx descriptors, tx descriptors and
    /// buffer.
    #[allow(clippy::type_complexity)]
    pub fn split(
        self,
    ) -> (
        DmaAlignedMut<'static, [DmaDescriptor]>,
        DmaAlignedMut<'static, [DmaDescriptor]>,
        DmaAlignedMut<'static, [u8]>,
    ) {
        (
            self.rx_descriptors.into_inner(),
            self.tx_descriptors.into_inner(),
            self.buffer,
        )
    }

    /// Returns the size of the underlying buffer.
    pub fn capacity(&self) -> usize {
        self.buffer.len()
    }

    /// Returns the number of bytes that would be transmitted by this buf.
    #[allow(clippy::len_without_is_empty)]
    pub fn len(&self) -> usize {
        self.tx_descriptors
            .linked_iter()
            .map(|d| d.len())
            .sum::<usize>()
    }

    /// Returns the entire buf as a slice that can be read.
    pub fn as_slice(&self) -> &[u8] {
        &self.buffer
    }

    /// Returns the entire buf as a slice that can be written.
    pub fn as_mut_slice(&mut self) -> &mut [u8] {
        &mut self.buffer
    }

    fn set_length_fallible(&mut self, len: usize) -> Result<(), DmaBufError> {
        if len > self.capacity() {
            return Err(DmaBufError::BufferTooSmall);
        }
        let alignment = self.alignment;
        let data = &self.buffer[..len];
        alignment.ensure_buffer_compatible(data, TransferDirection::In)?;
        alignment.ensure_buffer_compatible(data, TransferDirection::Out)?;
        let burst_align =
            alignment.guarantee(data.as_ptr() as usize, data.len(), TransferDirection::In);

        let max_chunk_size_in = alignment.max_chunk_size_for(&self.buffer, TransferDirection::In);
        let max_chunk_size_out = alignment.max_chunk_size_for(&self.buffer, TransferDirection::Out);
        self.rx_descriptors.set_rx_length(len, max_chunk_size_in)?;
        self.tx_descriptors.set_tx_length(len, max_chunk_size_out)?;
        self.burst_align = burst_align;

        Ok(())
    }

    /// Resets the descriptors to only transmit/receive `len` amount of bytes
    /// with this buf.
    ///
    /// `len` must be less than or equal to the buffer size.
    pub fn set_length(&mut self, len: usize) {
        unwrap!(self.set_length_fallible(len));
    }
}

unsafe impl DmaTxBuffer for DmaRxTxBuf {
    type View = BufView<DmaRxTxBuf>;
    type Final = DmaRxTxBuf;

    fn prepare(&mut self) -> Preparation {
        for desc in self.tx_descriptors.linked_iter_mut() {
            // In non-circular mode, we only set `suc_eof` for the last descriptor to signal
            // the end of the transfer.
            desc.reset_for_tx(desc.next.is_null());
        }

        #[cfg(dma_can_access_psram)]
        let is_data_in_psram = !is_valid_ram_address(self.buffer.as_ptr() as usize);

        #[cfg(any(soc_internal_memory_cached, dma_can_access_psram))]
        self.buffer.writeback();

        Preparation {
            start: self.tx_descriptors.head(),
            #[cfg(dma_can_access_psram)]
            accesses_psram: is_data_in_psram,
            burst_align: self.burst_align,
            check_owner: None,
            auto_write_back: false,
        }
    }

    fn into_view(self) -> BufView<DmaRxTxBuf> {
        BufView(self)
    }

    fn from_view(view: Self::View) -> Self {
        view.0
    }
}

unsafe impl DmaRxBuffer for DmaRxTxBuf {
    type View = BufView<DmaRxTxBuf>;
    type Final = DmaRxTxBuf;

    fn prepare(&mut self) -> Preparation {
        for desc in self.rx_descriptors.linked_iter_mut() {
            desc.reset_for_rx();
        }

        cfg_select! {
            dma_can_access_psram => {
                // Optimization: avoid locking for PSRAM range.
                let is_data_in_psram = !is_valid_ram_address(self.buffer.as_ptr() as usize);
                if is_data_in_psram || cfg!(soc_internal_memory_cached) {
                    unsafe {
                        crate::soc::cache_invalidate_addr(
                            self.buffer.as_ptr() as u32,
                            self.buffer.len() as u32,
                        )
                    };
                }
            }
            _ => {}
        }

        Preparation {
            start: self.rx_descriptors.head(),
            #[cfg(dma_can_access_psram)]
            accesses_psram: is_data_in_psram,
            burst_align: self.burst_align,
            check_owner: None,
            auto_write_back: true,
        }
    }

    fn into_view(self) -> BufView<DmaRxTxBuf> {
        BufView(self)
    }

    fn from_view(view: Self::View) -> Self {
        view.0
    }
}

/// Distributes `buffer` evenly between `descriptors`, and returns the burst alignment that the
/// stream buffer guarantees.
fn distribute_stream_buffer(
    descriptors: &mut [DmaDescriptor],
    buffer: &mut [u8],
    alignment: BurstAlignment,
    direction: TransferDirection,
) -> Result<Option<BurstSize>, DmaBufError> {
    // see https://github.com/esp-rs/esp-hal/issues/2269#issuecomment-4397953660
    // we can lift that requirement once we sort out this issue
    if descriptors.len() < 4 {
        return Err(DmaBufError::InsufficientDescriptors);
    }

    // Evenly distribute the buffer between the descriptors.
    let chunk_size = Some(buffer.len() / descriptors.len())
        .filter(|x| *x <= 4095)
        .ok_or(DmaBufError::InsufficientDescriptors)?;

    let mut chunks = buffer.chunks_exact_mut(chunk_size);
    for (desc, chunk) in descriptors.iter_mut().zip(chunks.by_ref()) {
        desc.buffer = chunk.as_mut_ptr();
        desc.set_size(chunk.len());
        if direction == TransferDirection::Out {
            desc.set_length(chunk.len());
        }
    }

    let remainder = chunks.into_remainder();

    // Append any excess to the last descriptor.
    let last_descriptor = descriptors.last_mut().unwrap();
    let last_size = last_descriptor.size() + remainder.len();
    if last_size > 4095 {
        return Err(DmaBufError::InsufficientDescriptors);
    }
    last_descriptor.set_size(last_size);

    let addr = buffer.as_ptr() as usize;
    let sizes = chunk_size | last_size;
    if let Some(min) = alignment.0 {
        if !addr.is_multiple_of(min.bytes()) {
            return Err(DmaAlignmentError::Address.into());
        }
        if direction == TransferDirection::In && !sizes.is_multiple_of(min.bytes()) {
            return Err(DmaAlignmentError::Size.into());
        }
    }

    Ok(alignment.guarantee(addr, sizes, direction))
}

/// DMA Streaming Receive Buffer.
///
/// This is a contiguous buffer linked together by DMA descriptors.
///
/// It is used for continuously streaming data from a peripheral's FIFO.
///
/// It maintains a sliding window of descriptors that progresses when
/// [DmaRxStreamBufView::consume] is called.
///
/// The list starts out like so `A (empty) -> B (empty) -> C (empty) -> D
/// (empty) -> NULL`.
///
/// As the DMA writes to the buffers the list progresses like so:
/// - `A (empty) -> B (empty) -> C (empty) -> D (empty) -> NULL`
/// - `A (full)  -> B (empty) -> C (empty) -> D (empty) -> NULL`
/// - `A (full)  -> B (full)  -> C (empty) -> D (empty) -> NULL`
/// - `A (full)  -> B (full)  -> C (full)  -> D (empty) -> NULL`
///
/// As [DmaRxStreamBufView::consume] is called, the list (approximately)
/// progresses like so:
/// - `A (full)  -> B (full)  -> C (full)  -> D (empty) -> NULL`
/// - `B (full)  -> C (full)  -> D (empty) -> A (empty) -> NULL`
/// - `C (full)  -> D (empty) -> A (empty) -> B (empty) -> NULL`
/// - `D (empty) -> A (empty) -> B (empty) -> C (empty) -> NULL`
///
/// If all the descriptors fill up, the [DmaRxInterrupt::DescriptorEmpty]
/// interrupt fires and the DMA stops writing. The transfer must then be resumed
/// or restarted.
///
/// This buffer does not indicate when this condition occurs. Check with the
/// driver to see if the DMA has stopped.
///
/// When constructing this buffer, tune the ratio between the chunk size and
/// buffer size appropriately. Smaller chunk sizes mean data is received more
/// frequently, but the DMA interrupts ([DmaRxInterrupt::Done]) also fire more
/// frequently when they are used.
///
/// See [DmaRxStreamBufView] for APIs available while a transfer is in
/// progress.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DmaRxStreamBuf {
    descriptors: DmaAlignedMut<'static, [DmaDescriptor]>,
    buffer: DmaAlignedMut<'static, [u8]>,
    burst_align: Option<BurstSize>,
}

impl DmaRxStreamBuf {
    /// Creates a new [`DmaRxStreamBuf`] and splits `buffer` across `descriptors`.
    ///
    /// Each descriptor's size is a multiple of the alignment that a receive transfer needs.
    /// In PSRAM, that alignment is the default 16-byte block. The sizes differ by at most
    /// one alignment unit.
    ///
    /// # Errors
    ///
    /// [`DmaBufError::InvalidAlignment`] when the address or the length of `buffer` is not a
    /// multiple of the alignment.
    ///
    /// [`DmaBufError::InsufficientDescriptors`] when any of the following is true:
    ///
    /// - `descriptors` has fewer than four entries
    /// - `buffer` has fewer alignment blocks than descriptors
    /// - a descriptor would hold more than 4095 bytes
    pub fn new(
        descriptors: DmaAlignedMut<'static, [DmaDescriptor]>,
        buffer: DmaAlignedMut<'static, [u8]>,
    ) -> Result<Self, DmaBufError> {
        Self::new_with_alignment(descriptors, buffer, BurstAlignment::NATURAL)
    }

    /// Creates a new [`DmaRxStreamBuf`] that guarantees an alignment of at least `min`.
    ///
    /// # Errors
    ///
    /// [`DmaBufError::InvalidAlignment`] if the address of the buffer, or the size of a
    /// descriptor's part of it, is not a multiple of `min`.
    pub fn new_with_min_burst(
        descriptors: DmaAlignedMut<'static, [DmaDescriptor]>,
        buffer: DmaAlignedMut<'static, [u8]>,
        min: BurstSize,
    ) -> Result<Self, DmaBufError> {
        Self::new_with_alignment(descriptors, buffer, BurstAlignment(Some(min)))
    }

    fn new_with_alignment(
        mut descriptors: DmaAlignedMut<'static, [DmaDescriptor]>,
        mut buffer: DmaAlignedMut<'static, [u8]>,
        alignment: BurstAlignment,
    ) -> Result<Self, DmaBufError> {
        // see https://github.com/esp-rs/esp-hal/issues/2269#issuecomment-4397953660
        // we can lift that requirement once we sort out this issue
        if descriptors.len() < 4 {
            return Err(DmaBufError::InsufficientDescriptors);
        }

        alignment.ensure_buffer_aligned(&buffer, TransferDirection::In)?;
        // PSRAM receive descriptors stay a multiple of the default 16-byte block. A requested
        // minimum burst can only make that block larger.
        let block = alignment.min_alignment(&buffer, TransferDirection::In);
        #[cfg(dma_can_access_psram)]
        let block = if is_valid_psram_address(buffer.as_ptr() as usize) {
            max(block, BurstSize::Size16.bytes())
        } else {
            block
        };
        if !(buffer.as_ptr() as usize).is_multiple_of(block) {
            return Err(DmaAlignmentError::Address.into());
        }
        if !buffer.len().is_multiple_of(block) {
            return Err(DmaAlignmentError::Size.into());
        }

        let blocks = buffer.len() / block;
        let blocks_per_descriptor = blocks / descriptors.len();
        let extra_blocks = blocks % descriptors.len();
        let max_size = (blocks_per_descriptor + usize::from(extra_blocks != 0)) * block;
        if blocks_per_descriptor == 0 || max_size > 4095 {
            return Err(DmaBufError::InsufficientDescriptors);
        }

        let mut rest = &mut buffer[..];
        for (i, desc) in descriptors.iter_mut().enumerate() {
            let size = (blocks_per_descriptor + usize::from(i < extra_blocks)) * block;
            let (chunk, tail) = core::mem::take(&mut rest).split_at_mut(size);
            desc.buffer = chunk.as_mut_ptr();
            desc.set_size(size);
            rest = tail;
        }

        let small = blocks_per_descriptor * block;
        let sizes = if extra_blocks == 0 {
            small
        } else {
            small | (small + block)
        };
        let burst_align =
            alignment.guarantee(buffer.as_ptr() as usize, sizes, TransferDirection::In);

        Ok(Self {
            descriptors,
            buffer,
            burst_align,
        })
    }

    /// Consumes the buf, returning the descriptors and buffer.
    pub fn split(
        self,
    ) -> (
        DmaAlignedMut<'static, [DmaDescriptor]>,
        DmaAlignedMut<'static, [u8]>,
    ) {
        (self.descriptors, self.buffer)
    }
}

unsafe impl DmaRxBuffer for DmaRxStreamBuf {
    type View = DmaRxStreamBufView;
    type Final = DmaRxStreamBuf;

    fn prepare(&mut self) -> Preparation {
        // Link up all the descriptors (but not in a circle).
        let mut next = null_mut();
        for desc in self.descriptors.iter_mut().rev() {
            desc.next = next;
            next = desc;

            desc.reset_for_rx();
        }

        #[cfg(any(soc_internal_memory_cached, dma_can_access_psram))]
        {
            self.descriptors.writeback();
            self.buffer.invalidate();
        }

        Preparation {
            start: self.descriptors.as_mut_ptr(),
            #[cfg(dma_can_access_psram)]
            accesses_psram: is_valid_psram_address(self.buffer.as_ptr() as usize),
            burst_align: self.burst_align,

            // Whilst we give ownership of the descriptors the DMA, the correctness of this buffer
            // implementation doesn't rely on the DMA checking for descriptor ownership.
            // No descriptor is added back to the end of the stream before it's ready for the DMA
            // to consume it.
            check_owner: None,
            auto_write_back: true,
        }
    }

    fn into_view(self) -> DmaRxStreamBufView {
        DmaRxStreamBufView {
            buf: self,
            descriptor_idx: 0,
            descriptor_offset: 0,
        }
    }

    fn from_view(view: Self::View) -> Self {
        view.buf
    }
}

fn offset_in_buffer(buffer: *const u8, ptr: *mut u8, offset: usize) -> usize {
    ptr.addr().wrapping_sub(buffer.addr()) + offset
}

/// A view into a [DmaRxStreamBuf].
pub struct DmaRxStreamBufView {
    buf: DmaRxStreamBuf,
    descriptor_idx: usize,
    descriptor_offset: usize,
}

impl DmaRxStreamBufView {
    /// Returns the number of bytes that are available to read from the buf.
    pub fn available_bytes(&mut self) -> usize {
        #[cfg(any(soc_internal_memory_cached, dma_can_access_psram))]
        self.buf.descriptors.invalidate();

        let (tail, head) = self.buf.descriptors.split_at(self.descriptor_idx);
        let mut result = 0;
        for desc in head.iter().chain(tail) {
            if desc.owner() == Owner::Dma {
                break;
            }
            result += desc.len();
        }
        result - self.descriptor_offset
    }

    /// Reads as much as possible into the buf from the available data.
    pub fn pop(&mut self, buf: &mut [u8]) -> usize {
        if buf.is_empty() {
            return 0;
        }
        let total_bytes = buf.len();

        let mut remaining = buf;
        loop {
            let available = self.peek();
            if available.is_empty() {
                break;
            }
            if available.len() >= remaining.len() {
                remaining.copy_from_slice(&available[0..remaining.len()]);
                self.consume(remaining.len());
                let consumed = remaining.len();
                remaining = &mut remaining[consumed..];
                break;
            } else {
                let to_consume = available.len();
                remaining[0..to_consume].copy_from_slice(available);
                self.consume(to_consume);
                remaining = &mut remaining[to_consume..];
            }
        }

        total_bytes - remaining.len()
    }

    /// Returns a slice into the buffer containing available data.
    /// This will be the longest possible contiguous slice into the buffer that
    /// contains data that is available to read.
    ///
    /// Ignores EOFs. See [Self::peek_until_eof] for EOF support.
    pub fn peek(&mut self) -> &[u8] {
        let (slice, _) = self.peek_internal(false);
        slice
    }

    /// Same as [Self::peek] but will not skip over any EOFs.
    ///
    /// It also returns a boolean indicating whether this slice ends with an EOF
    /// or not.
    pub fn peek_until_eof(&mut self) -> (&[u8], bool) {
        self.peek_internal(true)
    }

    /// Consumes the first `n` bytes from the available data, returning any
    /// fully consumed descriptors back to the DMA.
    /// This is typically called after [Self::peek]/[Self::peek_until_eof].
    ///
    /// Returns the number of bytes that were actually consumed.
    pub fn consume(&mut self, n: usize) -> usize {
        let mut remaining_bytes_to_consume = n;
        let mut descriptors_modified = false;

        #[cfg(any(soc_internal_memory_cached, dma_can_access_psram))]
        self.buf.descriptors.invalidate();

        loop {
            let desc = &mut self.buf.descriptors[self.descriptor_idx];

            if desc.owner() == Owner::Dma {
                // Descriptor is still owned by DMA so it can't be read yet.
                // This should only happen when there is no more data available to read.
                break;
            }

            let remaining_bytes_in_descriptor = desc.len() - self.descriptor_offset;
            if remaining_bytes_to_consume < remaining_bytes_in_descriptor {
                self.descriptor_offset += remaining_bytes_to_consume;
                remaining_bytes_to_consume = 0;
                break;
            }

            // Reset the descriptor for reuse.
            desc.set_owner(Owner::Dma);
            desc.set_suc_eof(false);
            desc.set_length(0);

            // Before connecting this descriptor to the end of the list, the next descriptor
            // must be disconnected from this one to prevent the DMA from
            // overtaking.
            desc.next = null_mut();

            let desc_ptr: *mut _ = desc;

            let prev_descriptor_index = self
                .descriptor_idx
                .checked_sub(1)
                .unwrap_or(self.buf.descriptors.len() - 1);

            // Connect this consumed descriptor to the end of the chain.
            self.buf.descriptors[prev_descriptor_index].next = desc_ptr;
            descriptors_modified = true;

            self.descriptor_idx += 1;
            if self.descriptor_idx >= self.buf.descriptors.len() {
                self.descriptor_idx = 0;
            }
            self.descriptor_offset = 0;

            remaining_bytes_to_consume -= remaining_bytes_in_descriptor;
        }

        if descriptors_modified {
            #[cfg(any(soc_internal_memory_cached, dma_can_access_psram))]
            self.buf.descriptors.writeback();
        }

        n - remaining_bytes_to_consume
    }

    fn peek_internal(&mut self, stop_at_eof: bool) -> (&[u8], bool) {
        #[cfg(any(soc_internal_memory_cached, dma_can_access_psram))]
        self.buf.descriptors.invalidate();

        let descriptors = &self.buf.descriptors[self.descriptor_idx..];

        // There must be at least one descriptor.
        debug_assert!(!descriptors.is_empty());

        // Descriptor lengths can differ.
        let buffer_start = offset_in_buffer(
            self.buf.buffer.as_ptr(),
            descriptors[0].buffer,
            self.descriptor_offset,
        );

        if descriptors.len() == 1 {
            let last_descriptor = &descriptors[0];
            if last_descriptor.owner() == Owner::Dma {
                // No data available.
                (&[], false)
            } else {
                let length = last_descriptor.len() - self.descriptor_offset;
                #[cfg(any(soc_internal_memory_cached, dma_can_access_psram))]
                cache_invalidate(&self.buf.buffer[buffer_start..][..length]);
                (
                    &self.buf.buffer[buffer_start..][..length],
                    last_descriptor.flags.suc_eof(),
                )
            }
        } else {
            let mut found_eof = false;

            let mut number_of_contiguous_bytes = 0;
            for desc in descriptors {
                if desc.owner() == Owner::Dma {
                    break;
                }
                number_of_contiguous_bytes += desc.len();

                if stop_at_eof && desc.flags.suc_eof() {
                    found_eof = true;
                    break;
                }
                // If the length is smaller than the size, the contiguous-ness ends here.
                if desc.len() < desc.size() {
                    break;
                }
            }

            let length = number_of_contiguous_bytes - self.descriptor_offset;
            #[cfg(any(soc_internal_memory_cached, dma_can_access_psram))]
            cache_invalidate(&self.buf.buffer[buffer_start..][..length]);

            (&self.buf.buffer[buffer_start..][..length], found_eof)
        }
    }
}

/// DMA Streaming Transmit Buffer.
///
/// This is symmetric implementation to [DmaRxStreamBuf], used for continuously
/// streaming data to a peripheral's FIFO.
///
/// The list starts out like so `A(full) -> B(full) -> C(full) -> D(full) -> NULL`.
///
/// As the DMA writes to FIFO, the list progresses like so:
/// - `A(full)  -> B(full)  -> C(full)  -> D(full) -> NULL`
/// - `A(empty) -> B(full)  -> C(full)  -> D(full) -> NULL`
/// - `A(empty) -> B(empty) -> C(full)  -> D(full) -> NULL`
/// - `A(empty) -> B(empty) -> C(empty) -> D(full) -> NULL`
///
/// As [DmaTxStreamBufView::push] is called, the list (approximately) progresses like so:
/// - `A(empty) -> B(empty) -> C(empty) -> D(full) -> NULL`
/// - `B(empty) -> C(empty) -> D(full)  -> A(full) -> NULL`
/// - `C(empty) -> D(full)  -> A(full)  -> B(full) -> NULL`
/// - `D(full)  -> A(full)  -> B(full)  -> C(full) -> NULL`
///
/// If all the descriptors run out, the [DmaTxInterrupt::TotalEof] interrupt fires and the DMA
/// stops writing. The transfer must then be resumed or restarted.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DmaTxStreamBuf {
    descriptors: DmaAlignedMut<'static, [DmaDescriptor]>,
    buffer: DmaAlignedMut<'static, [u8]>,
    burst_align: Option<BurstSize>,
    pre_filled: Option<usize>,
    view_descriptor_idx: usize,
    view_descriptor_offset: usize,
}

impl DmaTxStreamBuf {
    /// Creates a new [`DmaTxStreamBuf`] evenly distributing the buffer between
    /// the provided descriptors.
    pub fn new(
        descriptors: DmaAlignedMut<'static, [DmaDescriptor]>,
        buffer: DmaAlignedMut<'static, [u8]>,
    ) -> Result<Self, DmaBufError> {
        Self::new_with_alignment(descriptors, buffer, BurstAlignment::NATURAL)
    }

    /// Creates a new [`DmaTxStreamBuf`] that guarantees an alignment of at least `min`.
    ///
    /// # Errors
    ///
    /// [`DmaBufError::InvalidAlignment`] if the address of the buffer is not a multiple of
    /// `min`.
    pub fn new_with_min_burst(
        descriptors: DmaAlignedMut<'static, [DmaDescriptor]>,
        buffer: DmaAlignedMut<'static, [u8]>,
        min: BurstSize,
    ) -> Result<Self, DmaBufError> {
        Self::new_with_alignment(descriptors, buffer, BurstAlignment(Some(min)))
    }

    fn new_with_alignment(
        mut descriptors: DmaAlignedMut<'static, [DmaDescriptor]>,
        mut buffer: DmaAlignedMut<'static, [u8]>,
        alignment: BurstAlignment,
    ) -> Result<Self, DmaBufError> {
        let burst_align = distribute_stream_buffer(
            &mut descriptors,
            &mut buffer,
            alignment,
            TransferDirection::Out,
        )?;

        Ok(Self {
            descriptors,
            buffer,
            burst_align,
            pre_filled: None,
            view_descriptor_idx: 0,
            view_descriptor_offset: 0,
        })
    }

    /// Consumes the buf, returning the descriptors and buffer.
    pub fn split(
        self,
    ) -> (
        DmaAlignedMut<'static, [DmaDescriptor]>,
        DmaAlignedMut<'static, [u8]>,
    ) {
        (self.descriptors, self.buffer)
    }

    /// Pushes the buffer with the given data before DMA transfer starts.
    ///
    /// It is expected to pre-fill at least enough data to fill the first two descriptors' buffers.
    /// The more data is pre-filled, the more head-room is left to push more data.
    ///
    /// Pre-filled data only applies to the next transfer: a buffer returned from a
    /// transfer starts with nothing pre-filled. If nothing is pushed before a
    /// transfer starts, the entire buffer is transmitted with its current contents.
    pub fn push(&mut self, data: &[u8]) -> usize {
        self.push_with(|buf| {
            let len = buf.len().min(data.len());
            buf[..len].copy_from_slice(&data[..len]);
            len
        })
    }

    /// Pushes the buffer with the given data before DMA transfer starts.
    ///
    /// It is expected to pre-fill at least enough data to fill the first two descriptors' buffers.
    /// The more data is pre-filled, the more head-room is left to push more data.
    ///
    /// Pre-filled data only applies to the next transfer: a buffer returned from a
    /// transfer starts with nothing pre-filled. If nothing is pushed before a
    /// transfer starts, the entire buffer is transmitted with its current contents.
    ///
    /// Returns the number of bytes filled.
    pub fn push_with(&mut self, f: impl FnOnce(&mut [u8]) -> usize) -> usize {
        let start = self.pre_filled.unwrap_or(0);
        let bytes_pushed = f(&mut self.buffer[start..]);
        self.pre_filled = Some(start + bytes_pushed);
        bytes_pushed
    }

    fn setup_view_state(&mut self) {
        let pre_filled = self.pre_filled.unwrap_or(self.buffer.len());
        let (idx, offset) = mark_tx_stream_descriptors_ready(&mut self.descriptors, pre_filled);
        self.view_descriptor_idx = idx;
        self.view_descriptor_offset = offset;
        #[cfg(any(soc_internal_memory_cached, dma_can_access_psram))]
        cache_writeback(&self.buffer[..pre_filled]);
    }
}

/// Marks descriptors containing data that should be transmitted when the DMA
/// channel starts.
fn mark_tx_stream_descriptors_ready(
    descriptors: &mut DmaAlignedMut<'_, [DmaDescriptor]>,
    bytes_pushed: usize,
) -> (usize, usize) {
    if bytes_pushed == 0 {
        return (0, 0);
    }

    #[cfg(any(soc_internal_memory_cached, dma_can_access_psram))]
    descriptors.invalidate();

    let num = descriptors.len();
    let mut bytes_filled = 0;
    let mut cursor = (0, 0);

    for d in 0..num {
        let remaining = bytes_pushed - bytes_filled;
        let size = descriptors[d].size();

        if remaining == 0 {
            terminate_tx_stream_at(descriptors, d);
            cursor = (d, 0);
            break;
        }

        if remaining < size {
            if d == 0 {
                // The transfer needs at least one descriptor; send the partial chunk and
                // continue filling from the next one.
                descriptors[d].set_owner(Owner::Dma);
                descriptors[d].set_length(remaining);
                descriptors[d].set_suc_eof(true);
                if num > 1 {
                    terminate_tx_stream_at(descriptors, 1);
                    cursor = (1, 0);
                } else {
                    descriptors[d].next = null_mut();
                }
            } else {
                terminate_tx_stream_at(descriptors, d);
                cursor = (d, remaining);
            }
            break;
        }

        bytes_filled += size;
        descriptors[d].set_owner(Owner::Dma);
        descriptors[d].set_length(size);
        descriptors[d].set_suc_eof(true);
    }

    #[cfg(any(soc_internal_memory_cached, dma_can_access_psram))]
    descriptors.writeback();

    cursor
}

fn terminate_tx_stream_at(descriptors: &mut DmaAlignedMut<'_, [DmaDescriptor]>, start: usize) {
    if start > 0 {
        descriptors[start - 1].next = null_mut();
    }
    for desc in descriptors.iter_mut().skip(start) {
        desc.set_owner(Owner::Cpu);
    }
}

fn advance_tx_stream_descriptors(
    descriptors: &mut DmaAlignedMut<'_, [DmaDescriptor]>,
    descriptor_idx: &mut usize,
    descriptor_offset: &mut usize,
    bytes_pushed: usize,
) {
    if bytes_pushed == 0 {
        return;
    }

    let mut bytes_filled = 0;
    let num_descriptors = descriptors.len();

    #[cfg(any(soc_internal_memory_cached, dma_can_access_psram))]
    descriptors.invalidate();

    for i in 0..num_descriptors {
        let d = (*descriptor_idx + i) % num_descriptors;
        let desc = &mut descriptors[d];
        let bytes_in_d = desc.size() - *descriptor_offset;
        if bytes_in_d + bytes_filled > bytes_pushed {
            *descriptor_idx = d;
            *descriptor_offset = *descriptor_offset + bytes_pushed - bytes_filled;
            #[cfg(any(soc_internal_memory_cached, dma_can_access_psram))]
            descriptors.writeback();
            return;
        }
        bytes_filled += bytes_in_d;
        *descriptor_offset = 0;

        // Put the current descriptor at the end of the list
        desc.set_owner(Owner::Dma);
        desc.set_length(desc.size());
        desc.set_suc_eof(true);
        let p = d.checked_sub(1).unwrap_or(num_descriptors - 1);
        if p != d {
            let [prev, desc] = descriptors.get_disjoint_mut([p, d]).unwrap();
            desc.next = null_mut();
            prev.next = desc;
        }
    }

    #[cfg(any(soc_internal_memory_cached, dma_can_access_psram))]
    descriptors.writeback();
}

unsafe impl DmaTxBuffer for DmaTxStreamBuf {
    type View = DmaTxStreamBufView;
    type Final = Self;

    fn prepare(&mut self) -> Preparation {
        // Link up all the descriptors (but not in a circle).
        let mut next = null_mut();
        for desc in self.descriptors.iter_mut().rev() {
            desc.next = next;
            desc.set_owner(Owner::Dma);
            next = desc;
        }
        #[cfg(any(soc_internal_memory_cached, dma_can_access_psram))]
        self.descriptors.writeback();

        self.setup_view_state();

        Preparation {
            start: self.descriptors.as_mut_ptr(),
            #[cfg(dma_can_access_psram)]
            accesses_psram: is_valid_psram_address(self.buffer.as_ptr() as usize),
            burst_align: self.burst_align,

            // Whilst we give ownership of the descriptors the DMA, the correctness of this buffer
            // implementation doesn't rely on the DMA checking for descriptor ownership.
            // No descriptor is added back to the end of the stream before it's ready for the DMA
            // to consume it.
            check_owner: None,
            auto_write_back: true,
        }
    }

    fn into_view(self) -> Self::View {
        DmaTxStreamBufView {
            descriptor_idx: self.view_descriptor_idx,
            descriptor_offset: self.view_descriptor_offset,
            buf: self,
        }
    }

    fn from_view(view: Self::View) -> Self {
        let DmaTxStreamBufView {
            mut buf,
            descriptor_idx,
            descriptor_offset,
        } = view;
        buf.view_descriptor_idx = descriptor_idx;
        buf.view_descriptor_offset = descriptor_offset;
        buf.pre_filled = None;
        buf
    }
}

/// A view into a [DmaTxStreamBuf].
pub struct DmaTxStreamBufView {
    buf: DmaTxStreamBuf,
    descriptor_idx: usize,
    descriptor_offset: usize,
}

impl DmaTxStreamBufView {
    /// Returns the number of bytes available for writing.
    pub fn available_bytes(&mut self) -> usize {
        #[cfg(any(soc_internal_memory_cached, dma_can_access_psram))]
        self.buf.descriptors.invalidate();

        let (tail, head) = self.buf.descriptors.split_at(self.descriptor_idx);
        head.iter()
            .chain(tail)
            .take_while(|d| d.owner() == Owner::Cpu)
            .map(|d| d.size())
            .sum::<usize>()
            .saturating_sub(self.descriptor_offset)
    }

    fn write_position(&self) -> usize {
        let desc = &self.buf.descriptors[self.descriptor_idx];
        offset_in_buffer(
            self.buf.buffer.as_ptr(),
            desc.buffer,
            self.descriptor_offset,
        )
    }

    /// Pushes a buffer into the stream buffer.
    /// Returns the number of bytes pushed.
    pub fn push_with(&mut self, f: impl FnOnce(&mut [u8]) -> usize) -> usize {
        let dma_start = self.write_position();
        let dma_end = dma_start
            .saturating_add(self.available_bytes())
            .min(self.buf.buffer.len())
            .max(dma_start);
        let bytes_pushed = f(&mut self.buf.buffer[dma_start..dma_end]).min(dma_end - dma_start);
        #[cfg(any(soc_internal_memory_cached, dma_can_access_psram))]
        cache_writeback(&self.buf.buffer[dma_start..][..bytes_pushed]);

        self.advance(bytes_pushed);
        bytes_pushed
    }

    /// Advances the first `n` bytes from the available data.
    pub fn advance(&mut self, bytes_pushed: usize) {
        advance_tx_stream_descriptors(
            &mut self.buf.descriptors,
            &mut self.descriptor_idx,
            &mut self.descriptor_offset,
            bytes_pushed,
        );
    }

    /// Pushes a buffer into the stream buffer.
    /// Returns the number of bytes pushed.
    pub fn push(&mut self, data: &[u8]) -> usize {
        let total_len = data.len();
        let mut remaining = data;

        while !remaining.is_empty() && self.available_bytes() > 0 {
            let written = self.push_with(|buffer| {
                let len = usize::min(buffer.len(), remaining.len());
                buffer[..len].copy_from_slice(&remaining[..len]);
                len
            });
            if written == 0 {
                break;
            }
            remaining = &remaining[written..];
        }

        total_len - remaining.len()
    }
}

static mut EMPTY: InternalMemory<[DmaDescriptor; 1]> = InternalMemory::new([DmaDescriptor::EMPTY]);

/// An empty buffer for transfers that carry no data.
pub struct EmptyBuf;

unsafe impl DmaTxBuffer for EmptyBuf {
    type View = EmptyBuf;
    type Final = EmptyBuf;

    fn prepare(&mut self) -> Preparation {
        #[cfg(soc_internal_memory_cached)]
        #[allow(static_mut_refs)]
        unsafe {
            EMPTY.get_mut().writeback();
        }

        Preparation {
            start: (&raw mut EMPTY).cast(),
            #[cfg(dma_can_access_psram)]
            accesses_psram: false,
            burst_align: None,

            // As we don't give ownership of the descriptor to the DMA, it's important that the DMA
            // channel does *NOT* check for ownership, otherwise the channel will return an error.
            check_owner: Some(false),

            // The DMA should not write back to the descriptor as it is shared.
            auto_write_back: false,
        }
    }

    fn into_view(self) -> EmptyBuf {
        self
    }

    fn from_view(view: Self::View) -> Self {
        view
    }
}

unsafe impl DmaRxBuffer for EmptyBuf {
    type View = EmptyBuf;
    type Final = EmptyBuf;

    fn prepare(&mut self) -> Preparation {
        #[cfg(soc_internal_memory_cached)]
        #[allow(static_mut_refs)]
        unsafe {
            EMPTY.get_mut().writeback();
        }

        Preparation {
            start: (&raw mut EMPTY).cast(),
            #[cfg(dma_can_access_psram)]
            accesses_psram: false,
            burst_align: None,

            // As we don't give ownership of the descriptor to the DMA, it's important that the DMA
            // channel does *NOT* check for ownership, otherwise the channel will return an error.
            check_owner: Some(false),
            auto_write_back: true,
        }
    }

    fn into_view(self) -> EmptyBuf {
        self
    }

    fn from_view(view: Self::View) -> Self {
        view
    }
}

/// DMA Loop Buffer
///
/// This consists of a single descriptor that points to itself and points to a
/// single buffer, resulting in the buffer being transmitted over and over
/// again, indefinitely.
///
/// A DMA descriptor is 12 bytes. If the buffer is significantly shorter
/// than this, the DMA channel spends more time reading the descriptor than
/// it does reading the buffer, which may leave it unable to keep up with the
/// bandwidth requirements of some peripherals at high frequencies.
pub struct DmaLoopBuf {
    descriptor: DmaAlignedMut<'static, [DmaDescriptor]>,
    buffer: DmaAlignedMut<'static, [u8]>,
}

impl DmaLoopBuf {
    /// Creates a new [DmaLoopBuf].
    pub fn new(
        mut descriptors: DmaAlignedMut<'static, [DmaDescriptor]>,
        mut buffer: DmaAlignedMut<'static, [u8]>,
    ) -> Result<DmaLoopBuf, DmaBufError> {
        if buffer.len()
            > BurstAlignment::NATURAL.max_chunk_size_for(&buffer, TransferDirection::Out)
        {
            return Err(DmaBufError::InsufficientDescriptors);
        }

        descriptors[0].set_owner(Owner::Dma); // Doesn't matter
        descriptors[0].set_suc_eof(false);
        descriptors[0].set_length(buffer.len());
        descriptors[0].set_size(buffer.len());
        descriptors[0].buffer = buffer.as_mut_ptr();
        descriptors[0].next = descriptors.as_mut_ptr();

        Ok(Self {
            descriptor: descriptors,
            buffer,
        })
    }

    /// Consumes the buf, returning the descriptor and buffer.
    pub fn split(
        self,
    ) -> (
        DmaAlignedMut<'static, [DmaDescriptor]>,
        DmaAlignedMut<'static, [u8]>,
    ) {
        (self.descriptor, self.buffer)
    }
}

unsafe impl DmaTxBuffer for DmaLoopBuf {
    type View = DmaLoopBuf;
    type Final = DmaLoopBuf;

    fn prepare(&mut self) -> Preparation {
        #[cfg(any(soc_internal_memory_cached, dma_can_access_psram))]
        {
            self.descriptor.writeback();
            self.buffer.writeback();
        }

        Preparation {
            start: self.descriptor.as_mut_ptr(),
            #[cfg(dma_can_access_psram)]
            accesses_psram: is_valid_psram_address(self.buffer.as_ptr() as usize),
            burst_align: None,
            // The DMA must not check the owner bit, as it is never set.
            check_owner: Some(false),

            // Doesn't matter either way but it is set to true for ESP32 SPI_DMA compatibility.
            auto_write_back: false,
        }
    }

    fn into_view(self) -> Self::View {
        self
    }

    fn from_view(view: Self::View) -> Self {
        view
    }
}

impl Deref for DmaLoopBuf {
    type Target = [u8];

    fn deref(&self) -> &Self::Target {
        &self.buffer
    }
}

impl DerefMut for DmaLoopBuf {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.buffer
    }
}

/// A Preparation that masks itself as a DMA buffer.
///
/// For low-level use, where none of the pre-made buffers really fit.
///
/// Intended for low-level use inside esp-hal only.
pub(crate) struct NoBuffer(pub(crate) Preparation);
impl NoBuffer {
    fn prep(&self) -> Preparation {
        Preparation {
            start: self.0.start,
            #[cfg(dma_can_access_psram)]
            accesses_psram: self.0.accesses_psram,
            burst_align: self.0.burst_align,
            check_owner: self.0.check_owner,
            auto_write_back: self.0.auto_write_back,
        }
    }
}
unsafe impl DmaTxBuffer for NoBuffer {
    type View = ();
    type Final = ();

    fn prepare(&mut self) -> Preparation {
        self.prep()
    }

    fn into_view(self) -> Self::View {}
    fn from_view(_view: Self::View) {}
}
unsafe impl DmaRxBuffer for NoBuffer {
    type View = ();
    type Final = ();

    fn prepare(&mut self) -> Preparation {
        self.prep()
    }

    fn into_view(self) -> Self::View {}
    fn from_view(_view: Self::View) {}
}

/// Prepares data unsafely to be transmitted via DMA.
///
/// `block_size` is the requirement imposed by the peripheral that receives the data. It
/// ensures that the DMA will not try to copy a partial block, which would cause the RX DMA (that
/// moves results back into RAM) to never complete.
///
/// The function returns the DMA buffer, and the number of bytes that will be transferred.
///
/// # Safety
///
/// The caller must keep all its descriptors and the buffers they
/// point to valid while the buffer is being transferred.
#[cfg_attr(not(any(aes_supports_dma, spi_master_supports_dma)), expect(unused))]
pub(crate) unsafe fn prepare_for_tx(
    descriptors: &mut [DmaDescriptor],
    data: NonNull<[u8]>,
    block_size: usize,
) -> Result<(NoBuffer, usize), DmaError> {
    let alignment =
        BurstAlignment::NATURAL.min_alignment(unsafe { data.as_ref() }, TransferDirection::Out);

    if !data.addr().get().is_multiple_of(alignment) {
        // ESP32 has word alignment requirement on the TX descriptors, too.
        return Err(DmaError::InvalidAlignment(DmaAlignmentError::Address));
    }

    // Whichever is stricter, data location or peripheral requirements.
    //
    // This ensures that the RX DMA, if used, can transfer the returned number of bytes using at
    // most N+2 descriptors. While the hardware doesn't require this on the TX DMA side, (the TX DMA
    // can, except on the ESP32, transfer any amount of data), it makes usage MUCH simpler.
    let alignment = alignment.max(block_size);
    let chunk_size = 4096 - alignment;

    // Whole blocks only, and only as many bytes as the descriptors hold. The caller transfers the
    // rest some other way.
    let data_len = data.len().min(chunk_size * descriptors.len());
    let data_len = data_len - data_len % block_size;
    let mut data = NonNull::slice_from_raw_parts(data.cast::<u8>(), data_len);

    cfg_select! {
        dma_can_access_psram => {
            let data_addr = data.addr().get();
            let data_in_psram = crate::psram::psram_range().contains(&data_addr);

            // Make sure input data is in PSRAM instead of cache
            if data_in_psram || cfg!(soc_internal_memory_cached) {
                unsafe { crate::soc::cache_writeback_addr(data_addr as u32, data_len as u32) };
            }
        }
        soc_internal_memory_cached => {
            unsafe { crate::soc::cache_writeback_addr(data.addr().get() as u32, data_len as u32) };
        }
        _ => {}
    }

    let descriptors = unsafe { DmaAlignedMut::new_unchecked(descriptors) };
    let mut descriptors = unwrap!(DescriptorSet::new(descriptors));
    unwrap!(descriptors.link_with_buffer(unsafe { data.as_mut() }, chunk_size));
    unwrap!(descriptors.set_tx_length(data_len, chunk_size));

    for desc in descriptors.linked_iter_mut() {
        desc.reset_for_tx(desc.next.is_null());
    }

    #[cfg(soc_internal_memory_cached)]
    descriptors.descriptors.writeback();

    Ok((
        NoBuffer(Preparation {
            start: descriptors.head(),
            burst_align: None,
            check_owner: None,
            auto_write_back: false,
            #[cfg(dma_can_access_psram)]
            accesses_psram: data_in_psram,
        }),
        data_len,
    ))
}

/// Prepares buffers to receive data from DMA.
///
/// The function returns the DMA buffer, and the number of bytes that will be transferred.
///
/// # Safety
///
/// The caller must keep all its descriptors and the buffers they
/// point to valid while the buffer is being transferred.
#[cfg_attr(not(any(aes_supports_dma, spi_master_supports_dma)), expect(unused))]
pub(crate) unsafe fn prepare_for_rx(
    descriptors: &mut [DmaDescriptor],
    #[cfg(dma_can_access_psram)] align_buffers: &mut [Option<ManualWritebackBuffer>; 2],
    mut data: NonNull<[u8]>,
) -> (NoBuffer, usize) {
    let chunk_size = chunk_size_for_alignment(
        BurstAlignment::NATURAL.min_alignment(unsafe { data.as_ref() }, TransferDirection::In),
    );

    // The data we have to process may not be appropriate for the DMA:
    // - it may be improperly aligned for PSRAM
    // - it may not have a length that is a multiple of the external memory block size

    cfg_select! {
        dma_can_access_psram => {
            let data_addr = data.addr().get();
            let data_in_psram = crate::psram::psram_range().contains(&data_addr);
        }
        _ => {
            let data_in_psram = false;
        }
    }

    let descriptors = unsafe { DmaAlignedMut::new_unchecked(descriptors) };
    let mut descriptors = unwrap!(DescriptorSet::new(descriptors));
    // The DMA can write a receive buffer in cached memory directly only for the cache lines that
    // lie wholly inside the buffer. A partial line at either end also holds other data, which the
    // CPU can load or dirty during the transfer. So the partial first and last lines must go
    // through the bounce buffers.
    let bounce = data_in_psram || cfg!(soc_internal_memory_cached);
    let data_len = if bounce {
        cfg_select! {
            dma_can_access_psram => {
                let alignment = bounce_alignment(data_addr, data_in_psram);
                build_bounced_descriptor_list(&mut descriptors, align_buffers, data, alignment)
            }
            _ => {
                unreachable!()
            }
        }
    } else {
        // Just set up descriptors as usual
        let data_len = data.len();
        unwrap!(descriptors.link_with_buffer(unsafe { data.as_mut() }, chunk_size));
        unwrap!(descriptors.set_tx_length(data_len, chunk_size));

        data_len
    };

    for desc in descriptors.linked_iter_mut() {
        desc.reset_for_rx();
    }

    #[cfg(soc_internal_memory_cached)]
    descriptors.descriptors.writeback();

    (
        NoBuffer(Preparation {
            start: descriptors.head(),
            burst_align: None,
            check_owner: None,
            auto_write_back: true,
            #[cfg(dma_can_access_psram)]
            accesses_psram: data_in_psram,
        }),
        data_len,
    )
}

/// Returns the alignment of the part of a receive buffer that the DMA writes directly.
///
/// The alignment is the DMA's own requirement for the memory, or the cache line where the
/// memory is cached, whichever is larger.
#[cfg(dma_can_access_psram)]
fn bounce_alignment(addr: usize, in_psram: bool) -> usize {
    // The smallest PSRAM block.
    let dma = if in_psram {
        BurstSize::Size16.bytes()
    } else {
        1
    };
    dma.max(unwrap!(crate::dma::aligned::region_dma_alignment(addr)))
}

/// Invalidates the cache lines that lie wholly inside `data`, after a receive transfer.
///
/// The DMA does not write the partial lines at either end. The CPU copies their received bytes
/// from the bounce buffers, and their other bytes can hold the CPU's own writes. The invalidate
/// must not touch those partial lines.
#[cfg(any(soc_internal_memory_cached, dma_can_access_psram))]
pub(crate) fn invalidate_received(data: NonNull<[u8]>) {
    let addr = data.addr().get();
    let len = data.len();
    if !crate::dma::aligned::region_needs_cache_op(addr, len) {
        return;
    }
    let Some(alignment) = crate::dma::aligned::region_dma_alignment(addr) else {
        return;
    };
    let start = addr.next_multiple_of(alignment);
    let end = (addr + len) & !(alignment - 1);
    if end > start {
        unsafe { crate::soc::cache_invalidate_addr(start as u32, (end - start) as u32) };
    }
}

#[cfg(any(soc_internal_memory_cached, dma_can_access_psram))]
fn cache_writeback(data: &[u8]) {
    let addr = data.as_ptr() as usize;
    if !crate::dma::aligned::region_needs_cache_op(addr, data.len()) {
        return;
    }
    unsafe { crate::soc::cache_writeback_addr(addr as u32, data.len() as u32) };
}

#[cfg(any(soc_internal_memory_cached, dma_can_access_psram))]
fn cache_invalidate(data: &[u8]) {
    let addr = data.as_ptr() as usize;
    if !crate::dma::aligned::region_needs_cache_op(addr, data.len()) {
        return;
    }
    unsafe { crate::soc::cache_invalidate_addr(addr as u32, data.len() as u32) };
}

#[cfg(dma_can_access_psram)]
fn build_bounced_descriptor_list(
    descriptors: &mut DescriptorSet<'_>,
    copy_buffers: &mut [Option<ManualWritebackBuffer>; 2],
    data: NonNull<[u8]>,
    min_alignment: usize,
) -> usize {
    let data_len = data.len();
    let data_addr = data.addr().get();

    let chunk_size = 4096 - min_alignment;

    let mut descriptor_iter = DescriptorChainingIter::new(&mut descriptors.descriptors);
    let mut copy_buffer_iter = copy_buffers.iter_mut();

    // MIN_LAST_DMA_LEN could make this really annoying, so we're just allocating a bit larger
    // buffer and shove edge cases into a single one. If we have >24 bytes on the S2, the 2-buffer
    // alignment algo works fine as one of them can steal 16 bytes, the other will have
    // MIN_LAST_DMA_LEN data to work with.
    let has_aligned_data = data_len > BUF_LEN;

    // Calculate byte offset to the start of the buffer. An already-aligned
    // buffer needs no head at all.
    let offset = data_addr % min_alignment;
    let head_to_copy = (min_alignment - offset) % min_alignment;
    let head_to_copy = if !has_aligned_data {
        BUF_LEN
    } else if head_to_copy > 0 && head_to_copy < MIN_LAST_DMA_LEN {
        head_to_copy + min_alignment
    } else {
        head_to_copy
    };
    let head_to_copy = head_to_copy.min(data_len);

    // Calculate last unaligned part
    let tail_to_copy = (data_len - head_to_copy) % min_alignment;
    let tail_to_copy = if tail_to_copy > 0 && tail_to_copy < MIN_LAST_DMA_LEN {
        tail_to_copy + min_alignment
    } else {
        tail_to_copy
    };

    let mut consumed = 0;

    // Align beginning
    if head_to_copy > 0 {
        let copy_buffer = unwrap!(copy_buffer_iter.next());
        let buffer =
            copy_buffer.insert(ManualWritebackBuffer::new(get_range(data, 0..head_to_copy)));
        buffer.prepare_for_dma();

        let Some(descriptor) = descriptor_iter.next() else {
            return consumed;
        };
        descriptor.set_size(head_to_copy);
        descriptor.buffer = buffer.mut_buffer_ptr();
        consumed += head_to_copy;
    };

    // Chain up descriptors for the main aligned data part.
    let mut aligned_data = get_range(data, head_to_copy..data.len() - tail_to_copy);

    // The DMA writes these lines directly. They must not hold dirty data when the DMA writes
    // them, and must not be in the cache when the CPU reads them afterwards.
    if crate::dma::aligned::region_needs_cache_op(aligned_data.addr().get(), aligned_data.len()) {
        unsafe {
            crate::soc::cache_writeback_addr(
                aligned_data.addr().get() as u32,
                aligned_data.len() as u32,
            );
            crate::soc::cache_invalidate_addr(
                aligned_data.addr().get() as u32,
                aligned_data.len() as u32,
            );
        }
    }

    while !aligned_data.is_empty() {
        let Some(descriptor) = descriptor_iter.next() else {
            return consumed;
        };
        let chunk = aligned_data.len().min(chunk_size);

        descriptor.set_size(chunk);
        descriptor.buffer = aligned_data.cast::<u8>().as_ptr();
        consumed += chunk;
        aligned_data = get_range(aligned_data, chunk..aligned_data.len());
    }

    // Align end
    if tail_to_copy > 0 {
        let copy_buffer = unwrap!(copy_buffer_iter.next());
        let buffer = copy_buffer.insert(ManualWritebackBuffer::new(get_range(
            data,
            data.len() - tail_to_copy..data.len(),
        )));
        buffer.prepare_for_dma();

        let Some(descriptor) = descriptor_iter.next() else {
            return consumed;
        };
        descriptor.set_size(tail_to_copy);
        descriptor.buffer = buffer.mut_buffer_ptr();
        consumed += tail_to_copy;
    }

    consumed
}

#[cfg(dma_can_access_psram)]
fn get_range(ptr: NonNull<[u8]>, range: Range<usize>) -> NonNull<[u8]> {
    let len = range.end - range.start;
    NonNull::slice_from_raw_parts(unsafe { ptr.cast().byte_add(range.start) }, len)
}

#[cfg(dma_can_access_psram)]
struct DescriptorChainingIter<'a> {
    /// index of the next element to emit
    index: usize,
    descriptors: &'a mut [DmaDescriptor],
}
#[cfg(dma_can_access_psram)]
impl<'a> DescriptorChainingIter<'a> {
    fn new(descriptors: &'a mut [DmaDescriptor]) -> Self {
        Self {
            descriptors,
            index: 0,
        }
    }

    fn next(&mut self) -> Option<&'_ mut DmaDescriptor> {
        if self.index == 0 {
            self.index += 1;
            self.descriptors.get_mut(0)
        } else if self.index < self.descriptors.len() {
            let index = self.index;
            self.index += 1;

            // Grab a pointer to the current descriptor.
            let ptr = &raw mut self.descriptors[index];

            // Link the descriptor to the previous one.
            self.descriptors[index - 1].next = ptr;

            // Reborrow the pointer so that it doesn't get invalidated by our continued use of the
            // descriptor reference.
            Some(unsafe { &mut *ptr })
        } else {
            None
        }
    }
}

#[cfg(dma_can_access_psram)]
const MIN_LAST_DMA_LEN: usize = if cfg!(esp32s2) { 5 } else { 1 };
// A bounce buffer holds the longest partial line the DMA writes, plus the bytes that
// `MIN_LAST_DMA_LEN` can add to a head and to a tail. The longest line is the PSRAM cache line.
// A buffer that fits goes into one bounce buffer whole, so every longer buffer leaves a head
// and a tail that each fit in one.
#[cfg(dma_can_access_psram)]
const BUF_LEN: usize = crate::dma::aligned::PSRAM_CACHE_LINE + 2 * (MIN_LAST_DMA_LEN - 1);

/// Bounce buffer for a partial cache line at either end of a receive buffer. The DMA writes to
/// it, and then the CPU copies it to the caller's buffer.
#[cfg(dma_can_access_psram)]
pub(crate) struct ManualWritebackBuffer {
    buffer: InternalMemory<MaybeUninit<[u8; BUF_LEN]>>,
    dst_address: NonNull<u8>,
    n_bytes: u8,
}

#[cfg(dma_can_access_psram)]
impl ManualWritebackBuffer {
    pub fn new(ptr: NonNull<[u8]>) -> Self {
        assert!(ptr.len() <= BUF_LEN);
        Self {
            buffer: InternalMemory::new(MaybeUninit::uninit()),
            dst_address: ptr.cast(),
            n_bytes: ptr.len() as u8,
        }
    }

    pub fn prepare_for_dma(&mut self) {
        // Ensure our cache line is not dirty. A dirty cacheline
        // evicted during DMA operation can clobber received data.
        #[cfg(soc_internal_memory_cached)]
        self.buffer.get_mut().invalidate();
    }

    pub fn write_back(&mut self) {
        // The DMA wrote its data directly to memory, bypassing the CPU cache.
        // Invalidate the cache lines covering the alignment buffer so the CPU
        // reads the fresh DMA data rather than the stale zeros from new().
        #[cfg(soc_internal_memory_cached)]
        self.buffer.get_mut().invalidate();

        let src = self.mut_buffer_ptr().cast_const();
        unsafe {
            self.dst_address
                .as_ptr()
                .copy_from(src, self.n_bytes as usize);
        }
    }

    pub fn mut_buffer_ptr(&mut self) -> *mut u8 {
        self.buffer.get_mut().as_mut_ptr().cast::<u8>()
    }

    /// Zeroes the buffer. It can hold plaintext.
    // Only the GCM path zeroes a buffer, so the method is dead code without it.
    #[cfg(aes_dma_mode_gcm)]
    pub fn clear(&mut self) {
        let mut buffer = self.buffer.get_mut();
        unsafe { core::ptr::write_volatile(buffer.as_mut_ptr(), [0; BUF_LEN]) };
        #[cfg(soc_internal_memory_cached)]
        buffer.writeback();
    }
}
