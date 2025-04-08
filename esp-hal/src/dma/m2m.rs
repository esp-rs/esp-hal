use core::{
    mem::ManuallyDrop,
    ops::{Deref, DerefMut},
};

#[cfg(not(esp32s2))]
use crate::dma::{
    AnyGdmaChannel,
    AnyGdmaRxChannel,
    AnyGdmaTxChannel,
    DmaChannelConvert,
    DmaEligible,
};
#[cfg(esp32s2)]
use crate::dma::{CopyDmaChannel, CopyDmaRxChannel, CopyDmaTxChannel};
use crate::{
    dma::{
        BurstConfig,
        Channel,
        ChannelRx,
        ChannelTx,
        DmaDescriptor,
        DmaError,
        DmaPeripheral,
        DmaRxBuf,
        DmaRxBuffer,
        DmaRxInterrupt,
        DmaTxBuf,
        DmaTxBuffer,
        DmaTxInterrupt,
    },
    Async,
    Blocking,
    DriverMode,
};

/// DMA Memory to Memory pseudo-Peripheral
///
/// This is a pseudo-peripheral that allows for memory to memory transfers.
/// It is not a real peripheral, but a way to use the DMA engine for memory
/// to memory transfers.
pub struct Mem2Mem<'d, Dm>
where
    Dm: DriverMode,
{
    /// RX Half
    pub rx: Mem2MemRx<'d, Dm>,
    /// TX Half
    pub tx: Mem2MemTx<'d, Dm>,
}

impl<'d> Mem2Mem<'d, Blocking> {
    /// Create a new Mem2Mem instance.
    #[cfg(not(esp32s2))]
    pub fn new(
        channel: impl DmaChannelConvert<AnyGdmaChannel<'d>>,
        peripheral: impl DmaEligible,
    ) -> Self {
        unsafe { Self::new_unsafe(channel, peripheral.dma_peripheral()) }
    }

    /// Create a new Mem2Mem instance.
    ///
    /// # Safety
    ///
    /// You must ensure that you're not using DMA for the same peripheral and
    /// that you're the only one using the DmaPeripheral.
    #[cfg(not(esp32s2))]
    pub unsafe fn new_unsafe(
        channel: impl DmaChannelConvert<AnyGdmaChannel<'d>>,
        peripheral: DmaPeripheral,
    ) -> Self {
        let mut channel = Channel::new(channel.degrade());

        channel.rx.set_mem2mem_mode(true);

        Mem2Mem {
            rx: Mem2MemRx {
                channel: channel.rx,
                peripheral,
            },
            tx: Mem2MemTx {
                channel: channel.tx,
                peripheral,
            },
        }
    }

    /// Create a new Mem2Mem instance.
    #[cfg(esp32s2)]
    pub fn new(channel: CopyDmaChannel<'d>) -> Self {
        let channel = Channel::new(channel);

        // The S2's COPY DMA channel doesn't care about this. Once support for other
        // channels are added, this will need updating.
        let peripheral = DmaPeripheral::Spi2;

        Mem2Mem {
            rx: Mem2MemRx {
                channel: channel.rx,
                peripheral,
            },
            tx: Mem2MemTx {
                channel: channel.tx,
                peripheral,
            },
        }
    }

    /// Shortcut to create a [SimpleMem2Mem]
    pub fn with_descriptors(
        self,
        rx_descriptors: &'static mut [DmaDescriptor],
        tx_descriptors: &'static mut [DmaDescriptor],
        config: BurstConfig,
    ) -> Result<SimpleMem2Mem<'d, Blocking>, DmaError> {
        SimpleMem2Mem::new(self, rx_descriptors, tx_descriptors, config)
    }

    /// Convert Mem2Mem to an async Mem2Mem.
    pub fn into_async(self) -> Mem2Mem<'d, Async> {
        Mem2Mem {
            rx: self.rx.into_async(),
            tx: self.tx.into_async(),
        }
    }
}

/// The RX half of [Mem2Mem].
pub struct Mem2MemRx<'d, Dm: DriverMode> {
    #[cfg(not(esp32s2))]
    channel: ChannelRx<Dm, AnyGdmaRxChannel<'d>>,
    #[cfg(esp32s2)]
    channel: ChannelRx<Dm, CopyDmaRxChannel<'d>>,
    peripheral: DmaPeripheral,
}

impl<'d> Mem2MemRx<'d, Blocking> {
    /// Convert Mem2MemRx to an async Mem2MemRx.
    pub fn into_async(self) -> Mem2MemRx<'d, Async> {
        Mem2MemRx {
            channel: self.channel.into_async(),
            peripheral: self.peripheral,
        }
    }
}

impl<'d, Dm> Mem2MemRx<'d, Dm>
where
    Dm: DriverMode,
{
    /// Start the RX half of a memory to memory transfer.
    pub fn receive<BUF>(
        mut self,
        mut buf: BUF,
    ) -> Result<Mem2MemRxTransfer<'d, Dm, BUF>, (DmaError, Self, BUF)>
    where
        BUF: DmaRxBuffer,
    {
        let result = unsafe {
            self.channel
                .prepare_transfer(self.peripheral, &mut buf)
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
pub struct Mem2MemRxTransfer<'d, M: DriverMode, BUF: DmaRxBuffer> {
    m2m: ManuallyDrop<Mem2MemRx<'d, M>>,
    buf_view: ManuallyDrop<BUF::View>,
}

impl<'d, M: DriverMode, BUF: DmaRxBuffer> Mem2MemRxTransfer<'d, M, BUF> {
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

impl<M: DriverMode, BUF: DmaRxBuffer> Deref for Mem2MemRxTransfer<'_, M, BUF> {
    type Target = BUF::View;

    fn deref(&self) -> &Self::Target {
        &self.buf_view
    }
}

impl<M: DriverMode, BUF: DmaRxBuffer> DerefMut for Mem2MemRxTransfer<'_, M, BUF> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.buf_view
    }
}

impl<M: DriverMode, BUF: DmaRxBuffer> Drop for Mem2MemRxTransfer<'_, M, BUF> {
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
pub struct Mem2MemTx<'d, Dm: DriverMode> {
    #[cfg(not(esp32s2))]
    channel: ChannelTx<Dm, AnyGdmaTxChannel<'d>>,
    #[cfg(esp32s2)]
    channel: ChannelTx<Dm, CopyDmaTxChannel<'d>>,
    peripheral: DmaPeripheral,
}

impl<'d> Mem2MemTx<'d, Blocking> {
    /// Convert Mem2MemTx to an async Mem2MemTx.
    pub fn into_async(self) -> Mem2MemTx<'d, Async> {
        Mem2MemTx {
            channel: self.channel.into_async(),
            peripheral: self.peripheral,
        }
    }
}

impl<'d, Dm: DriverMode> Mem2MemTx<'d, Dm> {
    /// Start the TX half of a memory to memory transfer.
    pub fn send<BUF>(
        mut self,
        mut buf: BUF,
    ) -> Result<Mem2MemTxTransfer<'d, Dm, BUF>, (DmaError, Self, BUF)>
    where
        BUF: DmaTxBuffer,
    {
        let result = unsafe {
            self.channel
                .prepare_transfer(self.peripheral, &mut buf)
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
pub struct Mem2MemTxTransfer<'d, Dm: DriverMode, BUF: DmaTxBuffer> {
    m2m: ManuallyDrop<Mem2MemTx<'d, Dm>>,
    buf_view: ManuallyDrop<BUF::View>,
}

impl<'d, Dm: DriverMode, BUF: DmaTxBuffer> Mem2MemTxTransfer<'d, Dm, BUF> {
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
    pub fn wait(self) -> (Result<(), DmaError>, Mem2MemTx<'d, Dm>, BUF) {
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
    pub fn stop(self) -> (Mem2MemTx<'d, Dm>, BUF) {
        let (mut m2m, view) = self.release();

        m2m.channel.stop_transfer();

        (m2m, BUF::from_view(view))
    }

    fn release(mut self) -> (Mem2MemTx<'d, Dm>, BUF::View) {
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

impl<Dm: DriverMode, BUF: DmaTxBuffer> Deref for Mem2MemTxTransfer<'_, Dm, BUF> {
    type Target = BUF::View;

    fn deref(&self) -> &Self::Target {
        &self.buf_view
    }
}

impl<Dm: DriverMode, BUF: DmaTxBuffer> DerefMut for Mem2MemTxTransfer<'_, Dm, BUF> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.buf_view
    }
}

impl<Dm: DriverMode, BUF: DmaTxBuffer> Drop for Mem2MemTxTransfer<'_, Dm, BUF> {
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

/// A simple and easy to use wrapper around [SimpleMem2Mem].
/// More complex memory to memory transfers should use [Mem2Mem] directly.
pub struct SimpleMem2Mem<'d, Dm: DriverMode> {
    state: State<'d, Dm>,
    config: BurstConfig,
}

enum State<'d, Dm: DriverMode> {
    Idle(
        Mem2Mem<'d, Dm>,
        &'d mut [DmaDescriptor],
        &'d mut [DmaDescriptor],
    ),
    Active(
        Mem2MemRxTransfer<'d, Dm, DmaRxBuf>,
        Mem2MemTxTransfer<'d, Dm, DmaTxBuf>,
    ),
    InUse,
}

impl<'d, Dm: DriverMode> SimpleMem2Mem<'d, Dm> {
    /// Creates a new [SimpleMem2Mem].
    pub fn new(
        mem2mem: Mem2Mem<'d, Dm>,
        rx_descriptors: &'d mut [DmaDescriptor],
        tx_descriptors: &'d mut [DmaDescriptor],
        config: BurstConfig,
    ) -> Result<Self, DmaError> {
        if rx_descriptors.is_empty() || tx_descriptors.is_empty() {
            return Err(DmaError::OutOfDescriptors);
        }
        Ok(Self {
            state: State::Idle(mem2mem, rx_descriptors, tx_descriptors),
            config,
        })
    }
}

impl<'d, Dm: DriverMode> SimpleMem2Mem<'d, Dm> {
    /// Starts a memory to memory transfer.
    pub fn start_transfer(
        &mut self,
        rx_buffer: &mut [u8],
        tx_buffer: &[u8],
    ) -> Result<SimpleMem2MemTransfer<'_, 'd, Dm>, DmaError> {
        let State::Idle(mem2mem, rx_descriptors, tx_descriptors) =
            core::mem::replace(&mut self.state, State::InUse)
        else {
            panic!("SimpleMem2MemTransfer was forgotten with core::mem::forget or similar");
        };

        // Raise these buffers to 'static. This is not safe, bad things will happen if
        // the user calls core::mem::forget on SimpleMem2MemTransfer. This is
        // just the unfortunate consequence of doing DMA without enforcing
        // 'static.
        let rx_buffer =
            unsafe { core::slice::from_raw_parts_mut(rx_buffer.as_mut_ptr(), rx_buffer.len()) };
        let tx_buffer =
            unsafe { core::slice::from_raw_parts_mut(tx_buffer.as_ptr() as _, tx_buffer.len()) };
        let rx_descriptors = unsafe {
            core::slice::from_raw_parts_mut(rx_descriptors.as_mut_ptr(), rx_descriptors.len())
        };
        let tx_descriptors = unsafe {
            core::slice::from_raw_parts_mut(tx_descriptors.as_mut_ptr(), tx_descriptors.len())
        };

        let dma_tx_buf = unwrap!(
            DmaTxBuf::new_with_config(tx_descriptors, tx_buffer, self.config),
            "There's no way to get the descriptors back yet"
        );

        let tx = match mem2mem.tx.send(dma_tx_buf) {
            Ok(tx) => tx,
            Err((err, tx, buf)) => {
                let (tx_descriptors, _tx_buffer) = buf.split();
                self.state = State::Idle(
                    Mem2Mem { rx: mem2mem.rx, tx },
                    rx_descriptors,
                    tx_descriptors,
                );
                return Err(err);
            }
        };

        let dma_rx_buf = unwrap!(
            DmaRxBuf::new_with_config(rx_descriptors, rx_buffer, self.config),
            "There's no way to get the descriptors back yet"
        );

        let rx = match mem2mem.rx.receive(dma_rx_buf) {
            Ok(rx) => rx,
            Err((err, rx, buf)) => {
                let (rx_descriptors, _rx_buffer) = buf.split();
                let (tx, buf) = tx.stop();
                let (tx_descriptors, _tx_buffer) = buf.split();
                self.state = State::Idle(Mem2Mem { rx, tx }, rx_descriptors, tx_descriptors);
                return Err(err);
            }
        };

        self.state = State::Active(rx, tx);

        Ok(SimpleMem2MemTransfer(self))
    }
}

impl<Dm: DriverMode> Drop for SimpleMem2Mem<'_, Dm> {
    fn drop(&mut self) {
        if !matches!(&mut self.state, State::Idle(_, _, _)) {
            panic!("SimpleMem2MemTransfer was forgotten with core::mem::forget or similar");
        }
    }
}

/// Represents an ongoing (or potentially finished) DMA Memory-to-Memory
/// transfer.
pub struct SimpleMem2MemTransfer<'a, 'd, Dm: DriverMode>(&'a mut SimpleMem2Mem<'d, Dm>);

impl<Dm: DriverMode> SimpleMem2MemTransfer<'_, '_, Dm> {
    /// Returns true when [Self::wait] will not block.
    pub fn is_done(&self) -> bool {
        let State::Active(rx, tx) = &self.0.state else {
            unreachable!()
        };

        // Wait for transmission to finish, and wait for the RX channel to receive the
        // one and only EOF that DmaTxBuf will send.
        tx.is_done()
            && rx
                .m2m
                .channel
                .pending_in_interrupts()
                .contains(DmaRxInterrupt::SuccessfulEof)
    }

    /// Wait for the transfer to finish.
    pub fn wait(self) -> Result<(), DmaError> {
        while !self.is_done() {}
        Ok(())
    }
}

impl<Dm: DriverMode> Drop for SimpleMem2MemTransfer<'_, '_, Dm> {
    fn drop(&mut self) {
        let State::Active(rx, tx) = core::mem::replace(&mut self.0.state, State::InUse) else {
            unreachable!()
        };

        let (tx, dma_tx_buf) = tx.stop();
        let (rx, dma_rx_buf) = rx.stop();

        let (tx_descriptors, _tx_buffer) = dma_tx_buf.split();
        let (rx_descriptors, _rx_buffer) = dma_rx_buf.split();

        self.0.state = State::Idle(Mem2Mem { rx, tx }, rx_descriptors, tx_descriptors);
    }
}
