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
    burst: BurstConfig,
}

impl<'a> ScopedDmaTxBuf<'a> {
    /// Creates a new [ScopedDmaTxBuf] from some descriptors and a buffer.
    ///
    /// There must be enough descriptors for the provided buffer.
    /// Depending on alignment requirements, each descriptor can handle at most
    /// 4095 bytes worth of buffer.
    ///
    /// Both the descriptors and buffer must be in DMA-capable memory.
    /// Only DRAM is supported for descriptors.
    pub fn new_with_config(
        descriptors: &'a mut [DmaDescriptor],
        buffer: &'a mut [u8],
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

    fn set_length_fallible(&mut self, len: usize, burst: BurstConfig) -> Result<(), DmaBufError> {
        if len > self.capacity() {
            return Err(DmaBufError::BufferTooSmall);
        }
        burst.ensure_buffer_compatible(&self.buffer[..len], TransferDirection::Out)?;

        self.descriptors.set_tx_length(
            len,
            burst.max_chunk_size_for(self.buffer, TransferDirection::Out),
        )?;

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

unsafe impl<'a> DmaTxBuffer for ScopedDmaTxBuf<'a> {
    type View = BufView<ScopedDmaTxBuf<'a>>;
    type Final = ScopedDmaTxBuf<'a>;

    fn prepare(&mut self) -> Preparation {
        cfg_if::cfg_if! {
            if #[cfg(dma_can_access_psram)] {
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
            #[cfg(dma_can_access_psram)]
            accesses_psram: is_data_in_psram,
            burst_transfer: self.burst,
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

/// DMA transmit buffer
///
/// This is a contiguous buffer linked together by DMA descriptors of length
/// 4095 at most. It can only be used for transmitting data to a peripheral's
/// FIFO. See [DmaRxBuf] for receiving data.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(crate) struct ScopedDmaRxBuf<'a> {
    descriptors: DescriptorSet<'a>,
    buffer: &'a mut [u8],
    burst: BurstConfig,
}

impl<'a> ScopedDmaRxBuf<'a> {
    /// Creates a new [ScopedDmaRxBuf] from some descriptors and a buffer.
    ///
    /// There must be enough descriptors for the provided buffer.
    /// Depending on alignment requirements, each descriptor can handle at most
    /// 4092 bytes worth of buffer.
    ///
    /// Both the descriptors and buffer must be in DMA-capable memory.
    /// Only DRAM is supported for descriptors.
    pub fn new_with_config(
        descriptors: &'a mut [DmaDescriptor],
        buffer: &'a mut [u8],
        config: impl Into<BurstConfig>,
    ) -> Result<Self, DmaBufError> {
        let mut buf = Self {
            descriptors: DescriptorSet::new(descriptors)?,
            buffer,
            burst: BurstConfig::default(),
        };

        buf.configure(config, buf.capacity())?;

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

        cfg_if::cfg_if! {
            if #[cfg(dma_can_access_psram)] {
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
            #[cfg(dma_can_access_psram)]
            accesses_psram: is_data_in_psram,
            burst_transfer: self.burst,
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
