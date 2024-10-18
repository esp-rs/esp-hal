use core::{
    marker::PhantomData,
    mem::ManuallyDrop,
    ops::{Deref, DerefMut},
};

use crate::{
    dma::{
        AnyGdmaChannel,
        Channel,
        ChannelRx,
        ChannelTx,
        DmaChannelConvert,
        DmaEligible,
        DmaError,
        DmaPeripheral,
        DmaRxBuffer,
        DmaRxInterrupt,
        DmaTxBuffer,
        DmaTxInterrupt,
        Rx,
        Tx,
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
    /// RX Half
    pub rx: Mem2MemRx<'d, M>,
    /// TX Half
    pub tx: Mem2MemTx<'d, M>,
}

impl<'d, M> Mem2Mem<'d, M>
where
    M: Mode,
{
    /// Create a new Mem2Mem instance.
    pub fn new<CH>(channel: Channel<'d, CH, M>, peripheral: impl DmaEligible) -> Self
    where
        CH: DmaChannelConvert<AnyGdmaChannel>,
    {
        unsafe { Self::new_unsafe(channel, peripheral.dma_peripheral()) }
    }

    /// Create a new Mem2Mem instance.
    ///
    /// # Safety
    ///
    /// You must ensure that you're not using DMA for the same peripheral and
    /// that you're the only one using the DmaPeripheral.
    pub unsafe fn new_unsafe<CH>(channel: Channel<'d, CH, M>, peripheral: DmaPeripheral) -> Self
    where
        CH: DmaChannelConvert<AnyGdmaChannel>,
    {
        let mut channel = channel.degrade();

        channel.rx.set_mem2mem_mode(true);

        Mem2Mem {
            rx: Mem2MemRx {
                channel: channel.rx,
                peripheral,
                _mode: PhantomData,
            },
            tx: Mem2MemTx {
                channel: channel.tx,
                peripheral,
                _mode: PhantomData,
            },
        }
    }
}

/// The RX half of [Mem2Mem].
pub struct Mem2MemRx<'d, M: Mode> {
    channel: ChannelRx<'d, AnyGdmaChannel>,
    peripheral: DmaPeripheral,
    _mode: PhantomData<M>,
}

impl<'d, M: Mode> Mem2MemRx<'d, M> {
    /// Start the RX half of a memory to memory transfer.
    ///
    /// If `reset` is true, the DMA channel's state machine is reset before
    /// starting the transfer. (Which is what all other drivers do)
    /// Keeping it false allows you to gradually receive data in smaller chunks,
    /// rather than all at once.
    ///
    /// Note: You must set `reset` to true if a
    /// [DmaRxInterrupt::DescriptorError] occurred.
    pub fn receive<BUF>(
        mut self,
        mut buf: BUF,
        reset: bool,
    ) -> Result<Mem2MemRxTransfer<'d, M, BUF>, (DmaError, Self, BUF)>
    where
        BUF: DmaRxBuffer,
    {
        let result = unsafe {
            self.channel
                .prepare_transfer(self.peripheral, &mut buf, !reset)
                .and_then(|_| self.channel.start_transfer())
        };

        if let Err(e) = result {
            return Err((e, self, buf));
        }

        Ok(Mem2MemRxTransfer {
            m2m: ManuallyDrop::new(self),
            buf_view: ManuallyDrop::new(buf.into_view()),
        })
    }
}

/// Represents an ongoing (or potentially finished) DMA Memory-to-Memory RX
/// transfer.
pub struct Mem2MemRxTransfer<'d, M: Mode, BUF: DmaRxBuffer> {
    m2m: ManuallyDrop<Mem2MemRx<'d, M>>,
    buf_view: ManuallyDrop<BUF::View>,
}

impl<'d, M: Mode, BUF: DmaRxBuffer> Mem2MemRxTransfer<'d, M, BUF> {
    /// Returns true when [Self::wait] will not block.
    pub fn is_done(&self) -> bool {
        let done_interrupts = DmaRxInterrupt::DescriptorError | DmaRxInterrupt::DescriptorEmpty;
        !self
            .m2m
            .channel
            .pending_in_interrupts()
            .is_disjoint(done_interrupts)
    }

    /// Waits for the transfer to stop and returns the peripheral and buffer.
    pub fn wait(self) -> (Result<(), DmaError>, Mem2MemRx<'d, M>, BUF) {
        while !self.is_done() {}

        let (m2m, view) = self.release();

        let result = if m2m.channel.has_error() {
            Err(DmaError::DescriptorError)
        } else {
            Ok(())
        };

        (result, m2m, BUF::from_view(view))
    }

    /// Stops this transfer on the spot and returns the peripheral and buffer.
    pub fn stop(self) -> (Mem2MemRx<'d, M>, BUF) {
        let (mut m2m, view) = self.release();

        m2m.channel.stop_transfer();

        (m2m, BUF::from_view(view))
    }

    fn release(mut self) -> (Mem2MemRx<'d, M>, BUF::View) {
        // SAFETY: Since forget is called on self, we know that self.m2m and
        // self.buf_view won't be touched again.
        let result = unsafe {
            let m2m = ManuallyDrop::take(&mut self.m2m);
            let view = ManuallyDrop::take(&mut self.buf_view);
            (m2m, view)
        };
        core::mem::forget(self);
        result
    }
}

impl<'d, M: Mode, BUF: DmaRxBuffer> Deref for Mem2MemRxTransfer<'d, M, BUF> {
    type Target = BUF::View;

    fn deref(&self) -> &Self::Target {
        &self.buf_view
    }
}

impl<'d, M: Mode, BUF: DmaRxBuffer> DerefMut for Mem2MemRxTransfer<'d, M, BUF> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.buf_view
    }
}

impl<'d, M: Mode, BUF: DmaRxBuffer> Drop for Mem2MemRxTransfer<'d, M, BUF> {
    fn drop(&mut self) {
        self.m2m.channel.stop_transfer();

        // SAFETY: This is Drop, we know that self.m2m and self.buf_view
        // won't be touched again.
        let view = unsafe {
            ManuallyDrop::drop(&mut self.m2m);
            ManuallyDrop::take(&mut self.buf_view)
        };
        let _ = BUF::from_view(view);
    }
}

/// The TX half of [Mem2Mem].
pub struct Mem2MemTx<'d, M: Mode> {
    channel: ChannelTx<'d, AnyGdmaChannel>,
    peripheral: DmaPeripheral,
    _mode: PhantomData<M>,
}

impl<'d, M: Mode> Mem2MemTx<'d, M> {
    /// Start the TX half of a memory to memory transfer.
    ///
    /// If `reset` is true, the DMA channel's state machine is reset before
    /// starting the transfer. (Which is what all other drivers do)
    /// Keeping it false allows you to gradually send data in smaller chunks,
    /// rather than all at once.
    ///
    /// Note: You must set `reset` to true if a
    /// [DmaTxInterrupt::DescriptorError] occurred.
    pub fn send<BUF>(
        mut self,
        mut buf: BUF,
        reset: bool,
    ) -> Result<Mem2MemTxTransfer<'d, M, BUF>, (DmaError, Self, BUF)>
    where
        BUF: DmaTxBuffer,
    {
        let result = unsafe {
            self.channel
                .prepare_transfer(self.peripheral, &mut buf, !reset)
                .and_then(|_| self.channel.start_transfer())
        };

        if let Err(e) = result {
            return Err((e, self, buf));
        }

        Ok(Mem2MemTxTransfer {
            m2m: ManuallyDrop::new(self),
            buf_view: ManuallyDrop::new(buf.into_view()),
        })
    }
}

/// Represents an ongoing (or potentially finished) DMA Memory-to-Memory TX
/// transfer.
pub struct Mem2MemTxTransfer<'d, M: Mode, BUF: DmaTxBuffer> {
    m2m: ManuallyDrop<Mem2MemTx<'d, M>>,
    buf_view: ManuallyDrop<BUF::View>,
}

impl<'d, M: Mode, BUF: DmaTxBuffer> Mem2MemTxTransfer<'d, M, BUF> {
    /// Returns true when [Self::wait] will not block.
    pub fn is_done(&self) -> bool {
        let done_interrupts = DmaTxInterrupt::DescriptorError | DmaTxInterrupt::TotalEof;
        !self
            .m2m
            .channel
            .pending_out_interrupts()
            .is_disjoint(done_interrupts)
    }

    /// Waits for the transfer to stop and returns the peripheral and buffer.
    pub fn wait(self) -> (Result<(), DmaError>, Mem2MemTx<'d, M>, BUF) {
        while !self.is_done() {}

        let (m2m, view) = self.release();

        let result = if m2m.channel.has_error() {
            Err(DmaError::DescriptorError)
        } else {
            Ok(())
        };

        (result, m2m, BUF::from_view(view))
    }

    /// Stops this transfer on the spot and returns the peripheral and buffer.
    pub fn stop(self) -> (Mem2MemTx<'d, M>, BUF) {
        let (mut m2m, view) = self.release();

        m2m.channel.stop_transfer();

        (m2m, BUF::from_view(view))
    }

    fn release(mut self) -> (Mem2MemTx<'d, M>, BUF::View) {
        // SAFETY: Since forget is called on self, we know that self.m2m and
        // self.buf_view won't be touched again.
        let result = unsafe {
            let m2m = ManuallyDrop::take(&mut self.m2m);
            let view = ManuallyDrop::take(&mut self.buf_view);
            (m2m, view)
        };
        core::mem::forget(self);
        result
    }
}

impl<'d, M: Mode, BUF: DmaTxBuffer> Deref for Mem2MemTxTransfer<'d, M, BUF> {
    type Target = BUF::View;

    fn deref(&self) -> &Self::Target {
        &self.buf_view
    }
}

impl<'d, M: Mode, BUF: DmaTxBuffer> DerefMut for Mem2MemTxTransfer<'d, M, BUF> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.buf_view
    }
}

impl<'d, M: Mode, BUF: DmaTxBuffer> Drop for Mem2MemTxTransfer<'d, M, BUF> {
    fn drop(&mut self) {
        self.m2m.channel.stop_transfer();

        // SAFETY: This is Drop, we know that self.m2m and self.buf_view
        // won't be touched again.
        let view = unsafe {
            ManuallyDrop::drop(&mut self.m2m);
            ManuallyDrop::take(&mut self.buf_view)
        };
        let _ = BUF::from_view(view);
    }
}
