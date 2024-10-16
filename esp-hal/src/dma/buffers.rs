use core::ptr::null_mut;

use super::*;
use crate::soc::is_slice_in_dram;
#[cfg(esp32s3)]
use crate::soc::is_slice_in_psram;

/// Holds all the information needed to configure a DMA channel for a transfer.
pub struct Preparation {
    pub(super) start: *mut DmaDescriptor,

    /// block size for PSRAM transfers
    #[cfg_attr(not(esp32s3), allow(dead_code))]
    pub(super) block_size: Option<DmaBufBlkSize>,

    /// Specifies whether descriptor linked list specified in `start` conforms
    /// to the alignment requirements required to enable burst transfers.
    ///
    /// Note: This only applies to burst transfer of the buffer data, not the
    /// descriptors themselves.
    ///
    /// There are no additional alignment requirements for TX burst transfers,
    /// but RX transfers require all descriptors to have buffer pointers and
    /// sizes that are a multiple of 4 (word aligned).
    pub(super) is_burstable: bool,

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
    /// - `None`: DMA channel can operate in any mode it supports.
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
    pub(super) check_owner: Option<bool>,
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

    /// Returns the maximum number of bytes that would be transmitted by this
    /// buffer.
    ///
    /// This is a convenience hint for SPI. Most peripherals don't care how long
    /// the transfer is.
    fn length(&self) -> usize;
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

    /// Returns the maximum number of bytes that can be received by this buffer.
    ///
    /// This is a convenience hint for SPI. Most peripherals don't care how long
    /// the transfer is.
    fn length(&self) -> usize;
}

/// An in-progress view into [DmaRxBuf]/[DmaTxBuf].
///
/// In the future, this could support peeking into state of the
/// descriptors/buffers.
pub struct BufView<T>(T);

/// Error returned from Dma[Rx|Tx|RxTx]Buf operations.
#[derive(Debug, PartialEq, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DmaBufError {
    /// More descriptors are needed for the buffer size
    InsufficientDescriptors,
    /// Descriptors or buffers are not located in a supported memory region
    UnsupportedMemoryRegion,
    /// Buffer is not aligned to the required size
    InvalidAlignment,
    /// Invalid chunk size: must be > 0 and <= 4095
    InvalidChunkSize,
}

/// DMA buffer alignments
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DmaBufBlkSize {
    /// 16 bytes
    Size16 = 16,
    /// 32 bytes
    Size32 = 32,
    /// 64 bytes
    Size64 = 64,
}

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
    block_size: Option<DmaBufBlkSize>,
}

impl DmaTxBuf {
    /// Creates a new [DmaTxBuf] from some descriptors and a buffer.
    ///
    /// There must be enough descriptors for the provided buffer.
    /// Each descriptor can handle 4092 bytes worth of buffer.
    ///
    /// Both the descriptors and buffer must be in DMA-capable memory.
    /// Only DRAM is supported for descriptors.
    pub fn new(
        descriptors: &'static mut [DmaDescriptor],
        buffer: &'static mut [u8],
    ) -> Result<Self, DmaBufError> {
        Self::new_with_block_size(descriptors, buffer, None)
    }

    /// Compute max chunk size based on block size
    pub const fn compute_chunk_size(block_size: Option<DmaBufBlkSize>) -> usize {
        max_chunk_size(block_size)
    }

    /// Compute the number of descriptors required for a given block size and
    /// buffer size
    pub const fn compute_descriptor_count(
        buffer_size: usize,
        block_size: Option<DmaBufBlkSize>,
    ) -> usize {
        descriptor_count(buffer_size, Self::compute_chunk_size(block_size), false)
    }

    /// Creates a new [DmaTxBuf] from some descriptors and a buffer.
    ///
    /// There must be enough descriptors for the provided buffer.
    /// Each descriptor can handle at most 4095 bytes worth of buffer.
    /// Optionally, a block size can be provided for PSRAM & Burst transfers.
    ///
    /// Both the descriptors and buffer must be in DMA-capable memory.
    /// Only DRAM is supported for descriptors.
    pub fn new_with_block_size(
        descriptors: &'static mut [DmaDescriptor],
        buffer: &'static mut [u8],
        block_size: Option<DmaBufBlkSize>,
    ) -> Result<Self, DmaBufError> {
        cfg_if::cfg_if! {
            if #[cfg(esp32s3)] {
                // buffer can be either DRAM or PSRAM (if supported)
                if !is_slice_in_dram(buffer) && !is_slice_in_psram(buffer) {
                    return Err(DmaBufError::UnsupportedMemoryRegion);
                }
                // if its PSRAM, the block_size/alignment must be specified
                if is_slice_in_psram(buffer) && block_size.is_none() {
                    return Err(DmaBufError::InvalidAlignment);
                }
            } else {
                #[cfg(any(esp32,esp32s2))]
                if buffer.len() % 4 != 0 && buffer.as_ptr() as usize % 4 != 0 {
                    // ESP32 requires word alignment for DMA buffers.
                    // ESP32-S2 technically supports byte-aligned DMA buffers, but the
                    // transfer ends up writing out of bounds if the buffer's length
                    // is 2 or 3 (mod 4).
                    return Err(DmaBufError::InvalidAlignment);
                }
                // buffer can only be DRAM
                if !is_slice_in_dram(buffer) {
                    return Err(DmaBufError::UnsupportedMemoryRegion);
                }
            }
        }

        let block_size = if is_slice_in_dram(buffer) {
            // no need for block size if the buffer is in DRAM
            None
        } else {
            block_size
        };
        let mut buf = Self {
            descriptors: DescriptorSet::new(descriptors)?,
            buffer,
            block_size,
        };

        buf.descriptors
            .link_with_buffer(buf.buffer, max_chunk_size(block_size))?;
        buf.set_length(buf.capacity());

        Ok(buf)
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

    /// Reset the descriptors to only transmit `len` amount of bytes from this
    /// buf.
    ///
    /// The number of bytes in data must be less than or equal to the buffer
    /// size.
    pub fn set_length(&mut self, len: usize) {
        assert!(len <= self.buffer.len());

        unwrap!(self
            .descriptors
            .set_tx_length(len, max_chunk_size(self.block_size)));
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

        #[cfg(esp32s3)]
        if crate::soc::is_valid_psram_address(self.buffer.as_ptr() as usize) {
            unsafe {
                crate::soc::cache_writeback_addr(
                    self.buffer.as_ptr() as u32,
                    self.buffer.len() as u32,
                )
            };
        }

        Preparation {
            start: self.descriptors.head(),
            block_size: self.block_size,
            // This is TX, the DMA channel is free to do a burst transfer.
            is_burstable: true,
            check_owner: None,
        }
    }

    fn into_view(self) -> BufView<DmaTxBuf> {
        BufView(self)
    }

    fn from_view(view: Self::View) -> Self {
        view.0
    }

    fn length(&self) -> usize {
        self.len()
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
        if !is_slice_in_dram(buffer) {
            return Err(DmaBufError::UnsupportedMemoryRegion);
        }

        let mut buf = Self {
            descriptors: DescriptorSet::new(descriptors)?,
            buffer,
        };

        buf.descriptors
            .link_with_buffer(buf.buffer, max_chunk_size(None))?;
        buf.set_length(buf.capacity());

        Ok(buf)
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

    /// Reset the descriptors to only receive `len` amount of bytes into this
    /// buf.
    ///
    /// The number of bytes in data must be less than or equal to the buffer
    /// size.
    pub fn set_length(&mut self, len: usize) {
        assert!(len <= self.buffer.len());

        unwrap!(self.descriptors.set_rx_length(len, max_chunk_size(None)));
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

        Preparation {
            start: self.descriptors.head(),
            block_size: None,
            // DmaRxBuf doesn't currently enforce the alignment requirements required for bursting.
            // In the future, it could either enforce the alignment or calculate if the alignment
            // requirements happen to be met.
            is_burstable: false,
            check_owner: None,
        }
    }

    fn into_view(self) -> BufView<DmaRxBuf> {
        BufView(self)
    }

    fn from_view(view: Self::View) -> Self {
        view.0
    }

    fn length(&self) -> usize {
        self.len()
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
        if !is_slice_in_dram(buffer) {
            return Err(DmaBufError::UnsupportedMemoryRegion);
        }

        let mut buf = Self {
            rx_descriptors: DescriptorSet::new(rx_descriptors)?,
            tx_descriptors: DescriptorSet::new(tx_descriptors)?,
            buffer,
        };
        buf.rx_descriptors
            .link_with_buffer(buf.buffer, max_chunk_size(None))?;
        buf.tx_descriptors
            .link_with_buffer(buf.buffer, max_chunk_size(None))?;
        buf.set_length(buf.capacity());

        Ok(buf)
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

    /// Reset the descriptors to only transmit/receive `len` amount of bytes
    /// with this buf.
    ///
    /// `len` must be less than or equal to the buffer size.
    pub fn set_length(&mut self, len: usize) {
        assert!(len <= self.buffer.len());

        unwrap!(self.rx_descriptors.set_rx_length(len, max_chunk_size(None)));
        unwrap!(self.tx_descriptors.set_tx_length(len, max_chunk_size(None)));
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

        Preparation {
            start: self.tx_descriptors.head(),
            block_size: None, // TODO: support block size!

            // This is TX, the DMA channel is free to do a burst transfer.
            is_burstable: true,
            check_owner: None,
        }
    }

    fn into_view(self) -> BufView<DmaRxTxBuf> {
        BufView(self)
    }

    fn from_view(view: Self::View) -> Self {
        view.0
    }

    fn length(&self) -> usize {
        self.len()
    }
}

unsafe impl DmaRxBuffer for DmaRxTxBuf {
    type View = BufView<DmaRxTxBuf>;

    fn prepare(&mut self) -> Preparation {
        for desc in self.rx_descriptors.linked_iter_mut() {
            desc.reset_for_rx();
        }

        Preparation {
            start: self.rx_descriptors.head(),
            block_size: None, // TODO: support block size!

            // DmaRxTxBuf doesn't currently enforce the alignment requirements required for
            // bursting.
            is_burstable: false,
            check_owner: None,
        }
    }

    fn into_view(self) -> BufView<DmaRxTxBuf> {
        BufView(self)
    }

    fn from_view(view: Self::View) -> Self {
        view.0
    }

    fn length(&self) -> usize {
        self.len()
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
            block_size: None,

            // DmaRxStreamBuf doesn't currently enforce the alignment requirements required for
            // bursting.
            is_burstable: false,

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

    fn length(&self) -> usize {
        panic!("DmaCircularBuf doesn't have a length")
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
            block_size: None,

            // This is TX, the DMA channel is free to do a burst transfer.
            is_burstable: true,

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

    fn length(&self) -> usize {
        0
    }
}

unsafe impl DmaRxBuffer for EmptyBuf {
    type View = EmptyBuf;

    fn prepare(&mut self) -> Preparation {
        #[allow(unused_unsafe)] // stable requires unsafe, nightly complains about it
        Preparation {
            start: unsafe { core::ptr::addr_of_mut!(EMPTY).cast() },
            block_size: None,

            // As much as bursting is meaningless here, the descriptor does meet the requirements.
            is_burstable: true,

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

    fn length(&self) -> usize {
        0
    }
}
