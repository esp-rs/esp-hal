use super::*;

/// DMA transmit buffer
///
/// This is a contiguous buffer linked together by DMA descriptors of length
/// 4095 at most. It can only be used for transmitting data to a peripheral's
/// FIFO. See [DmaRxBuf] for receiving data.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(crate) struct ScopedDmaTxBuf<'a> {
    descriptors: DescriptorSet<'a>,
    buffer: &'a mut [u8],
    alignment: usize,
}

impl<'a> ScopedDmaTxBuf<'a> {
    /// Creates a new [ScopedDmaTxBuf] aligned to the requirements of the memory
    /// region the buffer lives in (internal or external, inferred from its
    /// address). Use [ScopedDmaTxBuf::new_aligned] to request a stronger
    /// alignment.
    pub fn new(
        descriptors: &'a mut [DmaDescriptor],
        buffer: &'a mut [u8],
    ) -> Result<Self, DmaBufError> {
        let alignment = region_alignment(buffer, TransferDirection::Out);
        Self::new_aligned(descriptors, buffer, alignment)
    }

    /// Creates a new [ScopedDmaTxBuf] with a caller-chosen minimum alignment.
    ///
    /// There must be enough descriptors for the provided buffer.
    /// Depending on the alignment, each descriptor can handle at most
    /// 4095 bytes worth of buffer.
    ///
    /// Both the descriptors and buffer must be in DMA-capable memory.
    /// Only DRAM is supported for descriptors.
    pub fn new_aligned(
        descriptors: &'a mut [DmaDescriptor],
        buffer: &'a mut [u8],
        alignment: usize,
    ) -> Result<Self, DmaBufError> {
        let mut buf = Self {
            descriptors: DescriptorSet::new(descriptors)?,
            buffer,
            alignment: 1,
        };

        let capacity = buf.capacity();
        buf.configure(alignment, capacity)?;

        Ok(buf)
    }

    fn configure(&mut self, alignment: usize, length: usize) -> Result<(), DmaBufError> {
        let alignment = effective_alignment(self.buffer, TransferDirection::Out, alignment);
        self.set_length_fallible(length, alignment)?;

        self.descriptors
            .link_with_buffer(self.buffer, chunk_size_for_alignment(alignment))?;

        self.alignment = alignment;
        Ok(())
    }

    /// Consume the buf, returning the descriptors and buffer.
    pub fn split(self) -> (&'a mut [DmaDescriptor], &'a mut [u8]) {
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

    fn set_length_fallible(&mut self, len: usize, alignment: usize) -> Result<(), DmaBufError> {
        if len > self.capacity() {
            return Err(DmaBufError::BufferTooSmall);
        }
        ensure_buffer_compatible(&self.buffer[..len], TransferDirection::Out, alignment)?;

        self.descriptors
            .set_tx_length(len, chunk_size_for_alignment(alignment))?;

        // This only needs to be done once (after every significant length change) as
        // Self::prepare sets Preparation::auto_write_back to false.
        for desc in self.descriptors.linked_iter_mut() {
            // In non-circular mode, we only set `suc_eof` for the last descriptor to signal
            // the end of the transfer.
            desc.reset_for_tx(desc.next.is_null());
        }

        Ok(())
    }

    /// Reset the descriptors to only transmit `len` amount of bytes from this
    /// buf.
    ///
    /// The number of bytes in data must be less than or equal to the buffer
    /// size.
    pub fn set_length(&mut self, len: usize) {
        unwrap!(self.set_length_fallible(len, self.alignment))
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

unsafe impl<'a> DmaTxBuffer for ScopedDmaTxBuf<'a> {
    type View = BufView<ScopedDmaTxBuf<'a>>;
    type Final = ScopedDmaTxBuf<'a>;

    fn prepare(&mut self) -> Preparation {
        #[cfg(soc_internal_memory_cached)]
        unsafe {
            crate::soc::cache_writeback_addr(
                self.descriptors.head() as u32,
                core::mem::size_of_val(self.descriptors.descriptors) as u32,
            );
        }

        cfg_if::cfg_if! {
            if #[cfg(dma_can_access_psram)] {
                let is_data_in_psram = !is_valid_ram_address(self.buffer.as_ptr() as usize);
                if is_data_in_psram || cfg!(soc_internal_memory_cached) {
                    unsafe {
                        crate::soc::cache_writeback_addr(
                            self.buffer.as_ptr() as u32,
                            self.buffer.len() as u32,
                        )
                    };
                }
            } else if #[cfg(soc_internal_memory_cached)] {
                unsafe {
                    crate::soc::cache_writeback_addr(
                        self.buffer.as_ptr() as u32,
                        self.buffer.len() as u32,
                    )
                };
            }
        }

        Preparation {
            start: self.descriptors.head(),
            #[cfg(dma_can_access_psram)]
            accesses_psram: is_data_in_psram,
            max_alignment: self.alignment,
            check_owner: None,
            auto_write_back: false,
        }
    }

    fn into_view(self) -> BufView<ScopedDmaTxBuf<'a>> {
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
/// See [ScopedDmaTxBuf] for transmitting data.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(crate) struct ScopedDmaRxBuf<'a> {
    descriptors: DescriptorSet<'a>,
    buffer: &'a mut [u8],
    alignment: usize,
}

impl<'a> ScopedDmaRxBuf<'a> {
    /// Creates a new [ScopedDmaRxBuf] aligned to the requirements of the memory
    /// region the buffer lives in (internal or external, inferred from its
    /// address). Use [ScopedDmaRxBuf::new_aligned] to request a stronger
    /// alignment.
    pub fn new(
        descriptors: &'a mut [DmaDescriptor],
        buffer: &'a mut [u8],
    ) -> Result<Self, DmaBufError> {
        let alignment = region_alignment(buffer, TransferDirection::In);
        Self::new_aligned(descriptors, buffer, alignment)
    }

    /// Creates a new [ScopedDmaRxBuf] with a caller-chosen minimum alignment.
    ///
    /// There must be enough descriptors for the provided buffer.
    /// Depending on the alignment, each descriptor can handle at most
    /// 4092 bytes worth of buffer.
    ///
    /// Both the descriptors and buffer must be in DMA-capable memory.
    /// Only DRAM is supported for descriptors.
    pub fn new_aligned(
        descriptors: &'a mut [DmaDescriptor],
        buffer: &'a mut [u8],
        alignment: usize,
    ) -> Result<Self, DmaBufError> {
        let mut buf = Self {
            descriptors: DescriptorSet::new(descriptors)?,
            buffer,
            alignment: 1,
        };

        buf.configure(alignment, buf.capacity())?;

        Ok(buf)
    }

    fn configure(&mut self, alignment: usize, length: usize) -> Result<(), DmaBufError> {
        let alignment = effective_alignment(self.buffer, TransferDirection::In, alignment);
        self.set_length_fallible(length, alignment)?;

        self.descriptors
            .link_with_buffer(self.buffer, chunk_size_for_alignment(alignment))?;

        self.alignment = alignment;
        Ok(())
    }

    /// Consume the buf, returning the descriptors and buffer.
    pub fn split(self) -> (&'a mut [DmaDescriptor], &'a mut [u8]) {
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

    fn set_length_fallible(&mut self, len: usize, alignment: usize) -> Result<(), DmaBufError> {
        if len > self.capacity() {
            return Err(DmaBufError::BufferTooSmall);
        }
        ensure_buffer_compatible(&self.buffer[..len], TransferDirection::In, alignment)?;

        self.descriptors
            .set_rx_length(len, chunk_size_for_alignment(alignment))
    }

    /// Reset the descriptors to only receive `len` amount of bytes into this
    /// buf.
    ///
    /// The number of bytes in data must be less than or equal to the buffer
    /// size.
    pub fn set_length(&mut self, len: usize) {
        unwrap!(self.set_length_fallible(len, self.alignment));
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
        // Note that due to an ESP32 quirk, the last received descriptor may not get
        // updated.
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

unsafe impl<'a> DmaRxBuffer for ScopedDmaRxBuf<'a> {
    type View = BufView<ScopedDmaRxBuf<'a>>;
    type Final = ScopedDmaRxBuf<'a>;

    fn prepare(&mut self) -> Preparation {
        for desc in self.descriptors.linked_iter_mut() {
            desc.reset_for_rx();
        }

        #[cfg(soc_internal_memory_cached)]
        unsafe {
            crate::soc::cache_writeback_addr(
                self.descriptors.head() as u32,
                core::mem::size_of_val(self.descriptors.descriptors) as u32,
            );
        }

        cfg_if::cfg_if! {
            if #[cfg(dma_can_access_psram)] {
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
            } else if #[cfg(soc_internal_memory_cached)] {
                unsafe {
                    crate::soc::cache_invalidate_addr(
                        self.buffer.as_ptr() as u32,
                        self.buffer.len() as u32,
                    )
                };
            }
        }

        Preparation {
            start: self.descriptors.head(),
            #[cfg(dma_can_access_psram)]
            accesses_psram: is_data_in_psram,
            max_alignment: self.alignment,
            check_owner: None,
            auto_write_back: true,
        }
    }

    fn into_view(self) -> BufView<Self> {
        BufView(self)
    }

    fn from_view(view: Self::View) -> Self {
        view.0
    }
}
