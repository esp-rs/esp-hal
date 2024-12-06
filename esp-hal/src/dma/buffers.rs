use core::{
    ops::{Deref, DerefMut},
    ptr::null_mut,
};

use super::*;
use crate::soc::{is_slice_in_dram, is_slice_in_psram};
#[cfg(psram_dma)]
use crate::soc::{is_valid_psram_address, is_valid_ram_address};

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

cfg_if::cfg_if! {
    if #[cfg(psram_dma)] {
        /// Burst size used when transferring to and from external memory.
        #[derive(Clone, Copy, PartialEq, Eq, Debug)]
        #[cfg_attr(feature = "defmt", derive(defmt::Format))]
        pub enum ExternalBurstConfig {
            /// 16 bytes
            Size16 = 16,

            /// 32 bytes
            Size32 = 32,

            /// 64 bytes
            Size64 = 64,
        }

        impl ExternalBurstConfig {
            /// The default external memory burst length.
            pub const DEFAULT: Self = Self::Size16;
        }

        impl Default for ExternalBurstConfig {
            fn default() -> Self {
                Self::DEFAULT
            }
        }

        /// Internal memory access burst mode.
        #[derive(Clone, Copy, PartialEq, Eq, Debug)]
        #[cfg_attr(feature = "defmt", derive(defmt::Format))]
        pub enum InternalBurstConfig {
            /// Burst mode is disabled.
            Disabled,

            /// Burst mode is enabled.
            Enabled,
        }

        impl InternalBurstConfig {
            /// The default internal burst mode configuration.
            pub const DEFAULT: Self = Self::Disabled;
        }

        impl Default for InternalBurstConfig {
            fn default() -> Self {
                Self::DEFAULT
            }
        }

        /// Burst transfer configuration.
        #[derive(Clone, Copy, PartialEq, Eq, Debug)]
        #[cfg_attr(feature = "defmt", derive(defmt::Format))]
        pub struct BurstConfig {
            /// Configures the burst size for PSRAM transfers.
            ///
            /// Burst mode is always enabled for PSRAM transfers.
            pub external_memory: ExternalBurstConfig,

            /// Enables or disables the burst mode for internal memory transfers.
            ///
            /// The burst size is not configurable.
            pub internal_memory: InternalBurstConfig,
        }

        impl BurstConfig {
            /// The default burst mode configuration.
            pub const DEFAULT: Self = Self {
                external_memory: ExternalBurstConfig::DEFAULT,
                internal_memory: InternalBurstConfig::DEFAULT,
            };
        }

        impl Default for BurstConfig {
            fn default() -> Self {
                Self::DEFAULT
            }
        }

        impl From<InternalBurstConfig> for BurstConfig {
            fn from(internal_memory: InternalBurstConfig) -> Self {
                Self {
                    external_memory: ExternalBurstConfig::DEFAULT,
                    internal_memory,
                }
            }
        }

        impl From<ExternalBurstConfig> for BurstConfig {
            fn from(external_memory: ExternalBurstConfig) -> Self {
                Self {
                    external_memory,
                    internal_memory: InternalBurstConfig::DEFAULT,
                }
            }
        }
    } else {
        /// Burst transfer configuration.
        #[derive(Clone, Copy, PartialEq, Eq, Debug)]
        #[cfg_attr(feature = "defmt", derive(defmt::Format))]
        pub enum BurstConfig {
            /// Burst mode is disabled.
            Disabled,

            /// Burst mode is enabled.
            Enabled,
        }

        impl BurstConfig {
            /// The default burst mode configuration.
            pub const DEFAULT: Self = Self::Disabled;
        }

        impl Default for BurstConfig {
            fn default() -> Self {
                Self::DEFAULT
            }
        }

        type InternalBurstConfig = BurstConfig;
    }
}

#[cfg(psram_dma)]
impl ExternalBurstConfig {
    const fn min_psram_alignment(self, direction: TransferDirection) -> usize {
        // S2 TRM: Specifically, size and buffer address pointer in receive descriptors
        // should be 16-byte, 32-byte or 64-byte aligned. For data frame whose
        // length is not a multiple of 16 bytes, 32 bytes, or 64 bytes, EDMA adds
        // padding bytes to the end.

        // S3 TRM: Size and Address for IN transfers must be block aligned. For receive
        // descriptors, if the data length received are not aligned with block size,
        // GDMA will pad the data received with 0 until they are aligned to
        // initiate burst transfer. You can read the length field in receive descriptors
        // to obtain the length of valid data received
        if matches!(direction, TransferDirection::In) {
            self as usize
        } else {
            // S2 TRM: Size, length and buffer address pointer in transmit descriptors are
            // not necessarily aligned with block size.

            // S3 TRM: Size, length, and buffer address pointer in transmit descriptors do
            // not need to be aligned.
            1
        }
    }
}

impl InternalBurstConfig {
    pub(super) const fn is_burst_enabled(self) -> bool {
        !matches!(self, Self::Disabled)
    }

    // Size and address alignment as those come in pairs on current hardware.
    const fn min_dram_alignment(self, direction: TransferDirection) -> usize {
        if matches!(direction, TransferDirection::In) {
            // NOTE(danielb): commenting this check is incorrect as per TRM, but works.
            //                we'll need to restore this once peripherals can read a
            //                different amount of data than what is configured in the
            //                buffer.
            // if cfg!(esp32) {
            //     // NOTE: The size must be word-aligned.
            //     // NOTE: The buffer address must be word-aligned
            //     4
            // }
            if self.is_burst_enabled() {
                // As described in "Accessing Internal Memory" paragraphs in the various TRMs.
                4
            } else {
                1
            }
        } else {
            // OUT transfers have no alignment requirements, except for ESP32, which is
            // described below.
            if cfg!(esp32) {
                // SPI DMA: Burst transmission is supported. The data size for
                // a single transfer must be four bytes aligned.
                // I2S DMA: Burst transfer is supported. However, unlike the
                // SPI DMA channels, the data size for a single transfer is
                // one word, or four bytes.
                4
            } else {
                1
            }
        }
    }
}

const fn max(a: usize, b: usize) -> usize {
    if a > b {
        a
    } else {
        b
    }
}

impl BurstConfig {
    delegate::delegate! {
        #[cfg(psram_dma)]
        to self.internal_memory {
            pub(super) const fn min_dram_alignment(self, direction: TransferDirection) -> usize;
            pub(super) fn is_burst_enabled(self) -> bool;
        }
    }

    /// Calculates an alignment that is compatible with the current burst
    /// configuration.
    ///
    /// This is an over-estimation so that Descriptors can be safely used with
    /// any DMA channel in any direction.
    pub const fn min_compatible_alignment(self) -> usize {
        let in_alignment = self.min_dram_alignment(TransferDirection::In);
        let out_alignment = self.min_dram_alignment(TransferDirection::Out);
        let alignment = max(in_alignment, out_alignment);

        #[cfg(psram_dma)]
        let alignment = max(alignment, self.external_memory as usize);

        alignment
    }

    const fn chunk_size_for_alignment(alignment: usize) -> usize {
        // DMA descriptors have a 12-bit field for the size/length of the buffer they
        // point at. As there is no such thing as 0-byte alignment, this means the
        // maximum size is 4095 bytes.
        4096 - alignment
    }

    /// Calculates a chunk size that is compatible with the current burst
    /// configuration's alignment requirements.
    ///
    /// This is an over-estimation so that Descriptors can be safely used with
    /// any DMA channel in any direction.
    pub const fn max_compatible_chunk_size(self) -> usize {
        Self::chunk_size_for_alignment(self.min_compatible_alignment())
    }

    fn min_alignment(self, _buffer: &[u8], direction: TransferDirection) -> usize {
        let alignment = self.min_dram_alignment(direction);

        cfg_if::cfg_if! {
            if #[cfg(psram_dma)] {
                let mut alignment = alignment;
                if is_valid_psram_address(_buffer.as_ptr() as usize) {
                    alignment = max(alignment, self.external_memory.min_psram_alignment(direction));
                }
            }
        }

        alignment
    }

    // Note: this function ignores address alignment as we assume the buffers are
    // aligned.
    fn max_chunk_size_for(self, buffer: &[u8], direction: TransferDirection) -> usize {
        Self::chunk_size_for_alignment(self.min_alignment(buffer, direction))
    }

    fn ensure_buffer_aligned(
        self,
        buffer: &[u8],
        direction: TransferDirection,
    ) -> Result<(), DmaAlignmentError> {
        let alignment = self.min_alignment(buffer, direction);
        if buffer.as_ptr() as usize % alignment != 0 {
            return Err(DmaAlignmentError::Address);
        }

        // NB: the TRMs suggest that buffer length don't need to be aligned, but
        // for IN transfers, we configure the DMA descriptors' size field, which needs
        // to be aligned.
        if direction == TransferDirection::In && buffer.len() % alignment != 0 {
            return Err(DmaAlignmentError::Size);
        }

        Ok(())
    }

    fn ensure_buffer_compatible(
        self,
        buffer: &[u8],
        direction: TransferDirection,
    ) -> Result<(), DmaBufError> {
        // buffer can be either DRAM or PSRAM (if supported)
        let is_in_dram = is_slice_in_dram(buffer);
        let is_in_psram = cfg!(psram_dma) && is_slice_in_psram(buffer);
        if !(is_in_dram || is_in_psram) {
            return Err(DmaBufError::UnsupportedMemoryRegion);
        }

        self.ensure_buffer_aligned(buffer, direction)?;

        Ok(())
    }
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

    /// The direction of the DMA transfer.
    pub direction: TransferDirection,

    /// Must be `true` if any of the DMA descriptors contain data in PSRAM.
    #[cfg(psram_dma)]
    pub accesses_psram: bool,

    /// Configures the DMA to transfer data in bursts.
    ///
    /// The implementation of the buffer must ensure that buffer size
    /// and alignment in each descriptor is compatible with the burst
    /// transfer configuration.
    ///
    /// For details on alignment requirements, refer to your chip's
    #[doc = crate::trm_markdown_link!()]
    pub burst_transfer: BurstConfig,

    /// Configures the "check owner" feature of the DMA channel.
    ///
    /// Most DMA channels allow software to configure whether the hardware
    /// checks that [DmaDescriptor::owner] is set to [Owner::Dma] before
    /// consuming the descriptor. If this check fails, the channel stops
    /// operating and fires
    /// [DmaRxInterrupt::DescriptorError]/[DmaTxInterrupt::DescriptorError].
    ///
    /// This field allows buffer implementation to configure this behaviour.
    /// - `Some(true)`: DMA channel must check the owner bit.
    /// - `Some(false)`: DMA channel must NOT check the owner bit.
    /// - `None`: DMA channel should check the owner bit if it is supported.
    ///
    /// Some buffer implementations may require that the DMA channel performs
    /// this check before consuming the descriptor to ensure correct
    /// behaviour. e.g. To prevent wrap-around in a circular transfer.
    ///
    /// Some buffer implementations may require that the DMA channel does NOT
    /// perform this check as the ownership bit will not be set before the
    /// channel tries to consume the descriptor.
    ///
    /// Most implementations won't have any such requirements and will work
    /// correctly regardless of whether the DMA channel checks or not.
    ///
    /// Note: If the DMA channel doesn't support the provided option,
    /// preparation will fail.
    pub check_owner: Option<bool>,
}

/// [DmaTxBuffer] is a DMA descriptor + memory combo that can be used for
/// transmitting data from a DMA channel to a peripheral's FIFO.
///
/// # Safety
///
/// The implementing type must keep all its descriptors and the buffers they
/// point to valid while the buffer is being transferred.
pub unsafe trait DmaTxBuffer {
    /// A type providing operations that are safe to perform on the buffer
    /// whilst the DMA is actively using it.
    type View;

    /// Prepares the buffer for an imminent transfer and returns
    /// information required to use this buffer.
    ///
    /// Note: This operation is idempotent.
    fn prepare(&mut self) -> Preparation;

    /// This is called before the DMA starts using the buffer.
    fn into_view(self) -> Self::View;

    /// This is called after the DMA is done using the buffer.
    fn from_view(view: Self::View) -> Self;
}

/// [DmaRxBuffer] is a DMA descriptor + memory combo that can be used for
/// receiving data from a peripheral's FIFO to a DMA channel.
///
/// Note: Implementations of this trait may only support having a single EOF bit
/// which resides in the last descriptor. There will be a separate trait in
/// future to support multiple EOFs.
///
/// # Safety
///
/// The implementing type must keep all its descriptors and the buffers they
/// point to valid while the buffer is being transferred.
pub unsafe trait DmaRxBuffer {
    /// A type providing operations that are safe to perform on the buffer
    /// whilst the DMA is actively using it.
    type View;

    /// Prepares the buffer for an imminent transfer and returns
    /// information required to use this buffer.
    ///
    /// Note: This operation is idempotent.
    fn prepare(&mut self) -> Preparation;

    /// This is called before the DMA starts using the buffer.
    fn into_view(self) -> Self::View;

    /// This is called after the DMA is done using the buffer.
    fn from_view(view: Self::View) -> Self;
}

/// An in-progress view into [DmaRxBuf]/[DmaTxBuf].
///
/// In the future, this could support peeking into state of the
/// descriptors/buffers.
pub struct BufView<T>(T);

/// DMA transmit buffer
///
/// This is a contiguous buffer linked together by DMA descriptors of length
/// 4095 at most. It can only be used for transmitting data to a peripheral's
/// FIFO. See [DmaRxBuf] for receiving data.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DmaTxBuf {
    descriptors: DescriptorSet<'static>,
    buffer: &'static mut [u8],
    burst: BurstConfig,
}

impl DmaTxBuf {
    /// Creates a new [DmaTxBuf] from some descriptors and a buffer.
    ///
    /// There must be enough descriptors for the provided buffer.
    /// Depending on alignment requirements, each descriptor can handle at most
    /// 4095 bytes worth of buffer.
    ///
    /// Both the descriptors and buffer must be in DMA-capable memory.
    /// Only DRAM is supported for descriptors.
    pub fn new(
        descriptors: &'static mut [DmaDescriptor],
        buffer: &'static mut [u8],
    ) -> Result<Self, DmaBufError> {
        Self::new_with_config(descriptors, buffer, BurstConfig::default())
    }

    /// Creates a new [DmaTxBuf] from some descriptors and a buffer.
    ///
    /// There must be enough descriptors for the provided buffer.
    /// Depending on alignment requirements, each descriptor can handle at most
    /// 4095 bytes worth of buffer.
    ///
    /// Both the descriptors and buffer must be in DMA-capable memory.
    /// Only DRAM is supported for descriptors.
    pub fn new_with_config(
        descriptors: &'static mut [DmaDescriptor],
        buffer: &'static mut [u8],
        config: impl Into<BurstConfig>,
    ) -> Result<Self, DmaBufError> {
        let mut buf = Self {
            descriptors: DescriptorSet::new(descriptors)?,
            buffer,
            burst: BurstConfig::default(),
        };

        let capacity = buf.capacity();
        buf.configure(config, capacity)?;

        Ok(buf)
    }

    fn configure(
        &mut self,
        burst: impl Into<BurstConfig>,
        length: usize,
    ) -> Result<(), DmaBufError> {
        let burst = burst.into();
        self.set_length_fallible(length, burst)?;

        self.descriptors.link_with_buffer(
            self.buffer,
            burst.max_chunk_size_for(self.buffer, TransferDirection::Out),
        )?;

        self.burst = burst;
        Ok(())
    }

    /// Configures the DMA to use burst transfers to access this buffer.
    pub fn set_burst_config(&mut self, burst: BurstConfig) -> Result<(), DmaBufError> {
        let len = self.len();
        self.configure(burst, len)
    }

    /// Consume the buf, returning the descriptors and buffer.
    pub fn split(self) -> (&'static mut [DmaDescriptor], &'static mut [u8]) {
        (self.descriptors.into_inner(), self.buffer)
    }

    /// Returns the size of the underlying buffer
    pub fn capacity(&self) -> usize {
        self.buffer.len()
    }

    /// Return the number of bytes that would be transmitted by this buf.
    #[allow(clippy::len_without_is_empty)]
    pub fn len(&self) -> usize {
        self.descriptors
            .linked_iter()
            .map(|d| d.len())
            .sum::<usize>()
    }

    fn set_length_fallible(&mut self, len: usize, burst: BurstConfig) -> Result<(), DmaBufError> {
        if len > self.capacity() {
            return Err(DmaBufError::BufferTooSmall);
        }
        burst.ensure_buffer_compatible(&self.buffer[..len], TransferDirection::Out)?;

        self.descriptors.set_tx_length(
            len,
            burst.max_chunk_size_for(self.buffer, TransferDirection::Out),
        )
    }

    /// Reset the descriptors to only transmit `len` amount of bytes from this
    /// buf.
    ///
    /// The number of bytes in data must be less than or equal to the buffer
    /// size.
    pub fn set_length(&mut self, len: usize) {
        unwrap!(self.set_length_fallible(len, self.burst))
    }

    /// Fills the TX buffer with the bytes provided in `data` and reset the
    /// descriptors to only cover the filled section.
    ///
    /// The number of bytes in data must be less than or equal to the buffer
    /// size.
    pub fn fill(&mut self, data: &[u8]) {
        self.set_length(data.len());
        self.as_mut_slice()[..data.len()].copy_from_slice(data);
    }

    /// Returns the buf as a mutable slice than can be written.
    pub fn as_mut_slice(&mut self) -> &mut [u8] {
        self.buffer
    }

    /// Returns the buf as a slice than can be read.
    pub fn as_slice(&self) -> &[u8] {
        self.buffer
    }
}

unsafe impl DmaTxBuffer for DmaTxBuf {
    type View = BufView<DmaTxBuf>;

    fn prepare(&mut self) -> Preparation {
        for desc in self.descriptors.linked_iter_mut() {
            // In non-circular mode, we only set `suc_eof` for the last descriptor to signal
            // the end of the transfer.
            desc.reset_for_tx(desc.next.is_null());
        }

        cfg_if::cfg_if! {
            if #[cfg(psram_dma)] {
                let is_data_in_psram = !is_valid_ram_address(self.buffer.as_ptr() as usize);
                if is_data_in_psram {
                    unsafe {
                        crate::soc::cache_writeback_addr(
                            self.buffer.as_ptr() as u32,
                            self.buffer.len() as u32,
                        )
                    };
                }
            }
        }

        Preparation {
            start: self.descriptors.head(),
            direction: TransferDirection::Out,
            #[cfg(psram_dma)]
            accesses_psram: is_data_in_psram,
            burst_transfer: self.burst,
            check_owner: None,
        }
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
/// See [DmaTxBuf] for transmitting data.
pub struct DmaRxBuf {
    descriptors: DescriptorSet<'static>,
    buffer: &'static mut [u8],
    burst: BurstConfig,
}

impl DmaRxBuf {
    /// Creates a new [DmaRxBuf] from some descriptors and a buffer.
    ///
    /// There must be enough descriptors for the provided buffer.
    /// Each descriptor can handle 4092 bytes worth of buffer.
    ///
    /// Both the descriptors and buffer must be in DMA-capable memory.
    /// Only DRAM is supported.
    pub fn new(
        descriptors: &'static mut [DmaDescriptor],
        buffer: &'static mut [u8],
    ) -> Result<Self, DmaBufError> {
        let mut buf = Self {
            descriptors: DescriptorSet::new(descriptors)?,
            buffer,
            burst: BurstConfig::default(),
        };

        buf.configure(buf.burst, buf.capacity())?;

        Ok(buf)
    }

    fn configure(
        &mut self,
        burst: impl Into<BurstConfig>,
        length: usize,
    ) -> Result<(), DmaBufError> {
        let burst = burst.into();
        self.set_length_fallible(length, burst)?;

        self.descriptors.link_with_buffer(
            self.buffer,
            burst.max_chunk_size_for(self.buffer, TransferDirection::In),
        )?;

        self.burst = burst;
        Ok(())
    }

    /// Configures the DMA to use burst transfers to access this buffer.
    pub fn set_burst_config(&mut self, burst: BurstConfig) -> Result<(), DmaBufError> {
        let len = self.len();
        self.configure(burst, len)
    }

    /// Consume the buf, returning the descriptors and buffer.
    pub fn split(self) -> (&'static mut [DmaDescriptor], &'static mut [u8]) {
        (self.descriptors.into_inner(), self.buffer)
    }

    /// Returns the size of the underlying buffer
    pub fn capacity(&self) -> usize {
        self.buffer.len()
    }

    /// Returns the maximum number of bytes that this buf has been configured to
    /// receive.
    #[allow(clippy::len_without_is_empty)]
    pub fn len(&self) -> usize {
        self.descriptors
            .linked_iter()
            .map(|d| d.size())
            .sum::<usize>()
    }

    fn set_length_fallible(&mut self, len: usize, burst: BurstConfig) -> Result<(), DmaBufError> {
        if len > self.capacity() {
            return Err(DmaBufError::BufferTooSmall);
        }
        burst.ensure_buffer_compatible(&self.buffer[..len], TransferDirection::In)?;

        self.descriptors.set_rx_length(
            len,
            burst.max_chunk_size_for(&self.buffer[..len], TransferDirection::In),
        )
    }

    /// Reset the descriptors to only receive `len` amount of bytes into this
    /// buf.
    ///
    /// The number of bytes in data must be less than or equal to the buffer
    /// size.
    pub fn set_length(&mut self, len: usize) {
        unwrap!(self.set_length_fallible(len, self.burst));
    }

    /// Returns the entire underlying buffer as a slice than can be read.
    pub fn as_slice(&self) -> &[u8] {
        self.buffer
    }

    /// Returns the entire underlying buffer as a slice than can be written.
    pub fn as_mut_slice(&mut self) -> &mut [u8] {
        self.buffer
    }

    /// Return the number of bytes that was received by this buf.
    pub fn number_of_received_bytes(&self) -> usize {
        self.descriptors
            .linked_iter()
            .map(|d| d.len())
            .sum::<usize>()
    }

    /// Reads the received data into the provided `buf`.
    ///
    /// If `buf.len()` is less than the amount of received data then only the
    /// first `buf.len()` bytes of received data is written into `buf`.
    ///
    /// Returns the number of bytes in written to `buf`.
    pub fn read_received_data(&self, mut buf: &mut [u8]) -> usize {
        let capacity = buf.len();
        for chunk in self.received_data() {
            if buf.is_empty() {
                break;
            }
            let to_fill;
            (to_fill, buf) = buf.split_at_mut(chunk.len());
            to_fill.copy_from_slice(chunk);
        }

        capacity - buf.len()
    }

    /// Returns the received data as an iterator of slices.
    pub fn received_data(&self) -> impl Iterator<Item = &[u8]> {
        self.descriptors.linked_iter().map(|desc| {
            // SAFETY: We set up the descriptor to point to a subslice of the buffer, and
            // here we are only recreating that slice with a perhaps shorter length.
            // We are also not accessing `self.buffer` while this slice is alive, so we
            // are not violating any aliasing rules.
            unsafe { core::slice::from_raw_parts(desc.buffer.cast_const(), desc.len()) }
        })
    }
}

unsafe impl DmaRxBuffer for DmaRxBuf {
    type View = BufView<DmaRxBuf>;

    fn prepare(&mut self) -> Preparation {
        for desc in self.descriptors.linked_iter_mut() {
            desc.reset_for_rx();
        }

        cfg_if::cfg_if! {
            if #[cfg(psram_dma)] {
                // Optimization: avoid locking for PSRAM range.
                let is_data_in_psram = !is_valid_ram_address(self.buffer.as_ptr() as usize);
                if is_data_in_psram {
                    unsafe {
                        crate::soc::cache_invalidate_addr(
                            self.buffer.as_ptr() as u32,
                            self.buffer.len() as u32,
                        )
                    };
                }
            }
        }

        Preparation {
            start: self.descriptors.head(),
            direction: TransferDirection::In,
            #[cfg(psram_dma)]
            accesses_psram: is_data_in_psram,
            burst_transfer: self.burst,
            check_owner: None,
        }
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
pub struct DmaRxTxBuf {
    rx_descriptors: DescriptorSet<'static>,
    tx_descriptors: DescriptorSet<'static>,
    buffer: &'static mut [u8],
    burst: BurstConfig,
}

impl DmaRxTxBuf {
    /// Creates a new [DmaRxTxBuf] from some descriptors and a buffer.
    ///
    /// There must be enough descriptors for the provided buffer.
    /// Each descriptor can handle 4092 bytes worth of buffer.
    ///
    /// Both the descriptors and buffer must be in DMA-capable memory.
    /// Only DRAM is supported.
    pub fn new(
        rx_descriptors: &'static mut [DmaDescriptor],
        tx_descriptors: &'static mut [DmaDescriptor],
        buffer: &'static mut [u8],
    ) -> Result<Self, DmaBufError> {
        let mut buf = Self {
            rx_descriptors: DescriptorSet::new(rx_descriptors)?,
            tx_descriptors: DescriptorSet::new(tx_descriptors)?,
            buffer,
            burst: BurstConfig::default(),
        };

        let capacity = buf.capacity();
        buf.configure(buf.burst, capacity)?;

        Ok(buf)
    }

    fn configure(
        &mut self,
        burst: impl Into<BurstConfig>,
        length: usize,
    ) -> Result<(), DmaBufError> {
        let burst = burst.into();
        self.set_length_fallible(length, burst)?;

        self.rx_descriptors.link_with_buffer(
            self.buffer,
            burst.max_chunk_size_for(self.buffer, TransferDirection::In),
        )?;
        self.tx_descriptors.link_with_buffer(
            self.buffer,
            burst.max_chunk_size_for(self.buffer, TransferDirection::Out),
        )?;

        self.burst = burst;

        Ok(())
    }

    /// Configures the DMA to use burst transfers to access this buffer.
    pub fn set_burst_config(&mut self, burst: BurstConfig) -> Result<(), DmaBufError> {
        let len = self.len();
        self.configure(burst, len)
    }

    /// Consume the buf, returning the rx descriptors, tx descriptors and
    /// buffer.
    pub fn split(
        self,
    ) -> (
        &'static mut [DmaDescriptor],
        &'static mut [DmaDescriptor],
        &'static mut [u8],
    ) {
        (
            self.rx_descriptors.into_inner(),
            self.tx_descriptors.into_inner(),
            self.buffer,
        )
    }

    /// Return the size of the underlying buffer.
    pub fn capacity(&self) -> usize {
        self.buffer.len()
    }

    /// Return the number of bytes that would be transmitted by this buf.
    #[allow(clippy::len_without_is_empty)]
    pub fn len(&self) -> usize {
        self.tx_descriptors
            .linked_iter()
            .map(|d| d.len())
            .sum::<usize>()
    }

    /// Returns the entire buf as a slice than can be read.
    pub fn as_slice(&self) -> &[u8] {
        self.buffer
    }

    /// Returns the entire buf as a slice than can be written.
    pub fn as_mut_slice(&mut self) -> &mut [u8] {
        self.buffer
    }

    fn set_length_fallible(&mut self, len: usize, burst: BurstConfig) -> Result<(), DmaBufError> {
        if len > self.capacity() {
            return Err(DmaBufError::BufferTooSmall);
        }
        burst.ensure_buffer_compatible(&self.buffer[..len], TransferDirection::In)?;
        burst.ensure_buffer_compatible(&self.buffer[..len], TransferDirection::Out)?;

        self.rx_descriptors.set_rx_length(
            len,
            burst.max_chunk_size_for(self.buffer, TransferDirection::In),
        )?;
        self.tx_descriptors.set_tx_length(
            len,
            burst.max_chunk_size_for(self.buffer, TransferDirection::Out),
        )?;

        Ok(())
    }

    /// Reset the descriptors to only transmit/receive `len` amount of bytes
    /// with this buf.
    ///
    /// `len` must be less than or equal to the buffer size.
    pub fn set_length(&mut self, len: usize) {
        unwrap!(self.set_length_fallible(len, self.burst));
    }
}

unsafe impl DmaTxBuffer for DmaRxTxBuf {
    type View = BufView<DmaRxTxBuf>;

    fn prepare(&mut self) -> Preparation {
        for desc in self.tx_descriptors.linked_iter_mut() {
            // In non-circular mode, we only set `suc_eof` for the last descriptor to signal
            // the end of the transfer.
            desc.reset_for_tx(desc.next.is_null());
        }

        cfg_if::cfg_if! {
            if #[cfg(psram_dma)] {
                // Optimization: avoid locking for PSRAM range.
                let is_data_in_psram = !is_valid_ram_address(self.buffer.as_ptr() as usize);
                if is_data_in_psram {
                    unsafe {
                        crate::soc::cache_writeback_addr(
                            self.buffer.as_ptr() as u32,
                            self.buffer.len() as u32,
                        )
                    };
                }
            }
        }

        Preparation {
            start: self.tx_descriptors.head(),
            direction: TransferDirection::Out,
            #[cfg(psram_dma)]
            accesses_psram: is_data_in_psram,
            burst_transfer: self.burst,
            check_owner: None,
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

    fn prepare(&mut self) -> Preparation {
        for desc in self.rx_descriptors.linked_iter_mut() {
            desc.reset_for_rx();
        }

        cfg_if::cfg_if! {
            if #[cfg(psram_dma)] {
                // Optimization: avoid locking for PSRAM range.
                let is_data_in_psram = !is_valid_ram_address(self.buffer.as_ptr() as usize);
                if is_data_in_psram {
                    unsafe {
                        crate::soc::cache_invalidate_addr(
                            self.buffer.as_ptr() as u32,
                            self.buffer.len() as u32,
                        )
                    };
                }
            }
        }

        Preparation {
            start: self.rx_descriptors.head(),
            direction: TransferDirection::In,
            #[cfg(psram_dma)]
            accesses_psram: is_data_in_psram,
            burst_transfer: self.burst,
            check_owner: None,
        }
    }

    fn into_view(self) -> BufView<DmaRxTxBuf> {
        BufView(self)
    }

    fn from_view(view: Self::View) -> Self {
        view.0
    }
}

/// DMA Streaming Receive Buffer.
///
/// This is a contiguous buffer linked together by DMA descriptors, and the
/// buffer is evenly distributed between each descriptor provided.
///
/// It is used for continuously streaming data from a peripheral's FIFO.
///
/// It does so by maintaining sliding window of descriptors that progresses when
/// you call [DmaRxStreamBufView::consume].
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
/// As you call [DmaRxStreamBufView::consume] the list (approximately)
/// progresses like so:
/// - `A (full)  -> B (full)  -> C (full)  -> D (empty) -> NULL`
/// - `B (full)  -> C (full)  -> D (empty) -> A (empty) -> NULL`
/// - `C (full)  -> D (empty) -> A (empty) -> B (empty) -> NULL`
/// - `D (empty) -> A (empty) -> B (empty) -> C (empty) -> NULL`
///
/// If all the descriptors fill up, the [DmaRxInterrupt::DescriptorEmpty]
/// interrupt will fire and the DMA will stop writing, at which point it is up
/// to you to resume/restart the transfer.
///
/// Note: This buffer will not tell you when this condition occurs, you should
/// check with the driver to see if the DMA has stopped.
///
/// When constructing this buffer, it is important to tune the ratio between the
/// chunk size and buffer size appropriately. Smaller chunk sizes means you
/// receive data more frequently but this means the DMA interrupts
/// ([DmaRxInterrupt::Done]) also fire more frequently (if you use them).
///
/// See [DmaRxStreamBufView] for APIs available whilst a transfer is in
/// progress.
pub struct DmaRxStreamBuf {
    descriptors: &'static mut [DmaDescriptor],
    buffer: &'static mut [u8],
    burst: BurstConfig,
}

impl DmaRxStreamBuf {
    /// Creates a new [DmaRxStreamBuf] evenly distributing the buffer between
    /// the provided descriptors.
    pub fn new(
        descriptors: &'static mut [DmaDescriptor],
        buffer: &'static mut [u8],
    ) -> Result<Self, DmaBufError> {
        if !is_slice_in_dram(descriptors) {
            return Err(DmaBufError::UnsupportedMemoryRegion);
        }
        if !is_slice_in_dram(buffer) {
            return Err(DmaBufError::UnsupportedMemoryRegion);
        }

        if descriptors.is_empty() {
            return Err(DmaBufError::InsufficientDescriptors);
        }

        // Evenly distribute the buffer between the descriptors.
        let chunk_size = buffer.len() / descriptors.len();

        if chunk_size > 4095 {
            return Err(DmaBufError::InsufficientDescriptors);
        }

        // Check that the last descriptor can hold the excess
        let excess = buffer.len() % descriptors.len();
        if chunk_size + excess > 4095 {
            return Err(DmaBufError::InsufficientDescriptors);
        }

        // Link up all the descriptors (but not in a circle).
        let mut next = null_mut();
        for desc in descriptors.iter_mut().rev() {
            desc.next = next;
            next = desc;
        }

        let mut chunks = buffer.chunks_exact_mut(chunk_size);
        for (desc, chunk) in descriptors.iter_mut().zip(chunks.by_ref()) {
            desc.buffer = chunk.as_mut_ptr();
            desc.set_size(chunk.len());
        }

        let remainder = chunks.into_remainder();
        debug_assert_eq!(remainder.len(), excess);

        if !remainder.is_empty() {
            // Append any excess to the last descriptor.
            let last_descriptor = descriptors.last_mut().unwrap();
            last_descriptor.set_size(last_descriptor.size() + remainder.len());
        }

        Ok(Self {
            descriptors,
            buffer,
            burst: BurstConfig::default(),
        })
    }

    /// Consume the buf, returning the descriptors and buffer.
    pub fn split(self) -> (&'static mut [DmaDescriptor], &'static mut [u8]) {
        (self.descriptors, self.buffer)
    }
}

unsafe impl DmaRxBuffer for DmaRxStreamBuf {
    type View = DmaRxStreamBufView;

    fn prepare(&mut self) -> Preparation {
        for desc in self.descriptors.iter_mut() {
            desc.reset_for_rx();
        }
        Preparation {
            start: self.descriptors.as_mut_ptr(),
            direction: TransferDirection::In,
            #[cfg(psram_dma)]
            accesses_psram: false,
            burst_transfer: self.burst,

            // Whilst we give ownership of the descriptors the DMA, the correctness of this buffer
            // implementation doesn't rely on the DMA checking for descriptor ownership.
            // No descriptor is added back to the end of the stream before it's ready for the DMA
            // to consume it.
            check_owner: None,
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

/// A view into a [DmaRxStreamBuf]
pub struct DmaRxStreamBufView {
    buf: DmaRxStreamBuf,
    descriptor_idx: usize,
    descriptor_offset: usize,
}

impl DmaRxStreamBufView {
    /// Returns the number of bytes that are available to read from the buf.
    pub fn available_bytes(&self) -> usize {
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
    /// Note: This function ignores EOFs, see [Self::peek_until_eof] if you need
    /// EOF support.
    pub fn peek(&self) -> &[u8] {
        let (slice, _) = self.peek_internal(false);
        slice
    }

    /// Same as [Self::peek] but will not skip over any EOFs.
    ///
    /// It also returns a boolean indicating whether this slice ends with an EOF
    /// or not.
    pub fn peek_until_eof(&self) -> (&[u8], bool) {
        self.peek_internal(true)
    }

    /// Consumes the first `n` bytes from the available data, returning any
    /// fully consumed descriptors back to the DMA.
    /// This is typically called after [Self::peek]/[Self::peek_until_eof].
    ///
    /// Returns the number of bytes that were actually consumed.
    pub fn consume(&mut self, n: usize) -> usize {
        let mut remaining_bytes_to_consume = n;

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

            self.descriptor_idx += 1;
            if self.descriptor_idx >= self.buf.descriptors.len() {
                self.descriptor_idx = 0;
            }
            self.descriptor_offset = 0;

            remaining_bytes_to_consume -= remaining_bytes_in_descriptor;
        }

        n - remaining_bytes_to_consume
    }

    fn peek_internal(&self, stop_at_eof: bool) -> (&[u8], bool) {
        let descriptors = &self.buf.descriptors[self.descriptor_idx..];

        // There must be at least one descriptor.
        debug_assert!(!descriptors.is_empty());

        if descriptors.len() == 1 {
            let last_descriptor = &descriptors[0];
            if last_descriptor.owner() == Owner::Dma {
                // No data available.
                (&[], false)
            } else {
                let length = last_descriptor.len() - self.descriptor_offset;
                (
                    &self.buf.buffer[self.buf.buffer.len() - length..],
                    last_descriptor.flags.suc_eof(),
                )
            }
        } else {
            let chunk_size = descriptors[0].size();
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

            (
                &self.buf.buffer[chunk_size * self.descriptor_idx..][..number_of_contiguous_bytes]
                    [self.descriptor_offset..],
                found_eof,
            )
        }
    }
}

static mut EMPTY: [DmaDescriptor; 1] = [DmaDescriptor::EMPTY];

/// An empty buffer that can be used when you don't need to transfer any data.
pub struct EmptyBuf;

unsafe impl DmaTxBuffer for EmptyBuf {
    type View = EmptyBuf;

    fn prepare(&mut self) -> Preparation {
        #[allow(unused_unsafe)] // stable requires unsafe, nightly complains about it
        Preparation {
            start: unsafe { core::ptr::addr_of_mut!(EMPTY).cast() },
            direction: TransferDirection::Out,
            #[cfg(psram_dma)]
            accesses_psram: false,
            burst_transfer: BurstConfig::default(),

            // As we don't give ownership of the descriptor to the DMA, it's important that the DMA
            // channel does *NOT* check for ownership, otherwise the channel will return an error.
            check_owner: Some(false),
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

    fn prepare(&mut self) -> Preparation {
        #[allow(unused_unsafe)] // stable requires unsafe, nightly complains about it
        Preparation {
            start: unsafe { core::ptr::addr_of_mut!(EMPTY).cast() },
            direction: TransferDirection::In,
            #[cfg(psram_dma)]
            accesses_psram: false,
            burst_transfer: BurstConfig::default(),

            // As we don't give ownership of the descriptor to the DMA, it's important that the DMA
            // channel does *NOT* check for ownership, otherwise the channel will return an error.
            check_owner: Some(false),
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
/// Note: A DMA descriptor is 12 bytes. If your buffer is significantly shorter
/// than this, the DMA channel will spend more time reading the descriptor than
/// it does reading the buffer, which may leave it unable to keep up with the
/// bandwidth requirements of some peripherals at high frequencies.
pub struct DmaLoopBuf {
    descriptor: &'static mut DmaDescriptor,
    buffer: &'static mut [u8],
}

impl DmaLoopBuf {
    /// Create a new [DmaLoopBuf].
    pub fn new(
        descriptor: &'static mut DmaDescriptor,
        buffer: &'static mut [u8],
    ) -> Result<DmaLoopBuf, DmaBufError> {
        if !is_slice_in_dram(buffer) {
            return Err(DmaBufError::UnsupportedMemoryRegion);
        }
        if !is_slice_in_dram(core::slice::from_ref(descriptor)) {
            return Err(DmaBufError::UnsupportedMemoryRegion);
        }

        if buffer.len() > BurstConfig::default().max_chunk_size_for(buffer, TransferDirection::Out)
        {
            return Err(DmaBufError::InsufficientDescriptors);
        }

        descriptor.set_owner(Owner::Dma); // Doesn't matter
        descriptor.set_suc_eof(false);
        descriptor.set_length(buffer.len());
        descriptor.set_size(buffer.len());
        descriptor.buffer = buffer.as_mut_ptr();
        descriptor.next = descriptor;

        Ok(Self { descriptor, buffer })
    }

    /// Consume the buf, returning the descriptor and buffer.
    pub fn split(self) -> (&'static mut DmaDescriptor, &'static mut [u8]) {
        (self.descriptor, self.buffer)
    }
}

unsafe impl DmaTxBuffer for DmaLoopBuf {
    type View = Self;

    fn prepare(&mut self) -> Preparation {
        Preparation {
            start: self.descriptor,
            #[cfg(psram_dma)]
            accesses_psram: false,
            direction: TransferDirection::Out,
            burst_transfer: BurstConfig::default(),
            // The DMA must not check the owner bit, as it is never set.
            check_owner: Some(false),
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
        self.buffer
    }
}

impl DerefMut for DmaLoopBuf {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.buffer
    }
}
