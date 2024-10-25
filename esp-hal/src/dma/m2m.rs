#[cfg(esp32s3)]
use crate::dma::DmaExtMemBKSize;
use crate::{
    dma::{
        dma_private::{DmaSupport, DmaSupportRx},
        AnyGdmaChannel,
        Channel,
        ChannelRx,
        DescriptorChain,
        DmaChannelConvert,
        DmaDescriptor,
        DmaEligible,
        DmaError,
        DmaPeripheral,
        DmaTransferRx,
        ReadBuffer,
        Rx,
        Tx,
        WriteBuffer,
    },
    Mode,
};

/// DMA Memory to Memory pseudo-Peripheral
///
/// This is a pseudo-peripheral that allows for memory to memory transfers.
/// It is not a real peripheral, but a way to use the DMA engine for memory
/// to memory transfers.
pub struct Mem2Mem<'d, M>
where
    M: Mode,
{
    channel: Channel<'d, AnyGdmaChannel, M>,
    rx_chain: DescriptorChain,
    tx_chain: DescriptorChain,
    peripheral: DmaPeripheral,
}

impl<'d, M> Mem2Mem<'d, M>
where
    M: Mode,
{
    /// Create a new Mem2Mem instance.
    pub fn new<CH>(
        channel: Channel<'d, CH, M>,
        peripheral: impl DmaEligible,
        rx_descriptors: &'static mut [DmaDescriptor],
        tx_descriptors: &'static mut [DmaDescriptor],
    ) -> Result<Self, DmaError>
    where
        CH: DmaChannelConvert<AnyGdmaChannel>,
    {
        unsafe {
            Self::new_unsafe(
                channel,
                peripheral.dma_peripheral(),
                rx_descriptors,
                tx_descriptors,
                crate::dma::CHUNK_SIZE,
            )
        }
    }

    /// Create a new Mem2Mem instance with specific chunk size.
    pub fn new_with_chunk_size<CH>(
        channel: Channel<'d, CH, M>,
        peripheral: impl DmaEligible,
        rx_descriptors: &'static mut [DmaDescriptor],
        tx_descriptors: &'static mut [DmaDescriptor],
        chunk_size: usize,
    ) -> Result<Self, DmaError>
    where
        CH: DmaChannelConvert<AnyGdmaChannel>,
    {
        unsafe {
            Self::new_unsafe(
                channel,
                peripheral.dma_peripheral(),
                rx_descriptors,
                tx_descriptors,
                chunk_size,
            )
        }
    }

    /// Create a new Mem2Mem instance.
    ///
    /// # Safety
    ///
    /// You must ensure that your not using DMA for the same peripheral and
    /// that your the only one using the DmaPeripheral.
    pub unsafe fn new_unsafe<CH>(
        channel: Channel<'d, CH, M>,
        peripheral: DmaPeripheral,
        rx_descriptors: &'static mut [DmaDescriptor],
        tx_descriptors: &'static mut [DmaDescriptor],
        chunk_size: usize,
    ) -> Result<Self, DmaError>
    where
        CH: DmaChannelConvert<AnyGdmaChannel>,
    {
        if !(1..=4092).contains(&chunk_size) {
            return Err(DmaError::InvalidChunkSize);
        }
        if tx_descriptors.is_empty() || rx_descriptors.is_empty() {
            return Err(DmaError::OutOfDescriptors);
        }
        Ok(Mem2Mem {
            channel: channel.degrade(),
            peripheral,
            rx_chain: DescriptorChain::new_with_chunk_size(rx_descriptors, chunk_size),
            tx_chain: DescriptorChain::new_with_chunk_size(tx_descriptors, chunk_size),
        })
    }

    /// Start a memory to memory transfer.
    pub fn start_transfer<'t, TXBUF, RXBUF>(
        &mut self,
        rx_buffer: &'t mut RXBUF,
        tx_buffer: &'t TXBUF,
    ) -> Result<DmaTransferRx<'_, Self>, DmaError>
    where
        TXBUF: ReadBuffer,
        RXBUF: WriteBuffer,
    {
        let (tx_ptr, tx_len) = unsafe { tx_buffer.read_buffer() };
        let (rx_ptr, rx_len) = unsafe { rx_buffer.write_buffer() };
        self.tx_chain.fill_for_tx(false, tx_ptr, tx_len)?;
        self.rx_chain.fill_for_rx(false, rx_ptr, rx_len)?;
        unsafe {
            self.channel.tx.prepare_transfer_without_start(
                self.peripheral,
                &self.tx_chain,
                false,
            )?;
            self.channel
                .rx
                .prepare_transfer_without_start(self.peripheral, &self.rx_chain)?;
            self.channel.rx.set_mem2mem_mode(true);
        }
        #[cfg(esp32s3)]
        {
            let align = match unsafe { crate::soc::cache_get_dcache_line_size() } {
                16 => DmaExtMemBKSize::Size16,
                32 => DmaExtMemBKSize::Size32,
                64 => DmaExtMemBKSize::Size64,
                _ => panic!("unsupported cache line size"),
            };
            if crate::soc::is_valid_psram_address(tx_ptr as usize) {
                self.channel.tx.set_ext_mem_block_size(align);
            }
            if crate::soc::is_valid_psram_address(rx_ptr as usize) {
                self.channel.rx.set_ext_mem_block_size(align);
            }
        }
        self.channel.tx.start_transfer()?;
        self.channel.rx.start_transfer()?;
        Ok(DmaTransferRx::new(self))
    }
}

impl<'d, MODE> DmaSupport for Mem2Mem<'d, MODE>
where
    MODE: Mode,
{
    fn peripheral_wait_dma(&mut self, _is_rx: bool, _is_tx: bool) {
        while !self.channel.rx.is_done() {}
    }

    fn peripheral_dma_stop(&mut self) {
        unreachable!("unsupported")
    }
}

impl<'d, MODE> DmaSupportRx for Mem2Mem<'d, MODE>
where
    MODE: Mode,
{
    type RX = ChannelRx<'d, AnyGdmaChannel>;

    fn rx(&mut self) -> &mut Self::RX {
        &mut self.channel.rx
    }

    fn chain(&mut self) -> &mut DescriptorChain {
        &mut self.tx_chain
    }
}
