use core::{
    mem::ManuallyDrop,
    ops::{Deref, DerefMut},
};

#[cfg(dma_mem2mem_requires_peripheral)]
use crate::dma::DmaEligiblePeripheral;
use crate::{
    Async,
    Blocking,
    DriverMode,
    dma::{
        BurstConfig,
        Channel,
        ChannelRx,
        ChannelTx,
        DmaChannel,
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
};

/// A DMA channel singleton that supports memory-to-memory transfers on this chip.
///
/// Only channels listed in device metadata (`mem2mem = true`) implement this trait.
/// Use [`Mem2Mem::new`] to construct a transfer engine from such a channel.
#[diagnostic::on_unimplemented(
    message = "this DMA channel does not support memory-to-memory transfers",
    note = "Use a channel with `mem2mem = true` in device metadata. See `Mem2Mem::new`."
)]
pub trait Mem2MemCapableChannel: crate::private::Sealed {
    /// Peripheral selector programmed for memory-to-memory on this channel.
    #[cfg(not(dma_mem2mem_requires_peripheral))]
    fn mem2mem_id(&self) -> DmaPeripheral;
}

for_each_mem2mem_channel! {
    ($engine:literal, $ch:ident, $id:literal) => {
        impl Mem2MemCapableChannel for crate::peripherals::$ch<'_> {
            #[cfg(not(dma_mem2mem_requires_peripheral))]
            fn mem2mem_id(&self) -> DmaPeripheral {
                DmaPeripheral($id)
            }
        }
    };
    ($engine:literal, $any_ch:ident, $($hw:literal, $id:literal),+) => {
        impl Mem2MemCapableChannel for crate::dma::$any_ch<'_> {
            #[cfg(not(dma_mem2mem_requires_peripheral))]
            fn mem2mem_id(&self) -> DmaPeripheral {
                match self.channel_index() {
                    $( $hw => DmaPeripheral($id), )+
                    ch => panic!("Channel {} does not support memory-to-memory transfers", ch),
                }
            }
        }
    };
}

/// DMA Memory to Memory pseudo-Peripheral
///
/// This is a pseudo-peripheral that allows for memory to memory transfers.
/// It is not a real peripheral, but a way to use the DMA engine for memory
/// to memory transfers.
pub struct Mem2Mem<E, Dm>
where
    E: DmaChannel,
    Dm: DriverMode,
{
    /// RX Half
    pub rx: Mem2MemRx<E::Rx, Dm>,
    /// TX Half
    pub tx: Mem2MemTx<E::Tx, Dm>,
}

impl<E> Mem2Mem<E, Blocking>
where
    E: DmaChannel,
{
    /// Create a new [`Mem2Mem`] instance.
    #[cfg(not(dma_mem2mem_requires_peripheral))]
    pub fn new(channel: E) -> Self
    where
        E: Mem2MemCapableChannel,
    {
        let dma_peri = channel.mem2mem_id();
        Self::new_inner(channel, dma_peri)
    }

    /// Create a new [`Mem2Mem`] instance from a capable channel and a DMA peripheral.
    #[cfg(dma_mem2mem_requires_peripheral)]
    pub fn new<'d, CH, P>(channel: CH, peripheral: P) -> Self
    where
        CH: Mem2MemCapableChannel + Into<E>,
        P: DmaEligiblePeripheral<ErasedChannel<'d> = E>,
    {
        Self::new_inner(channel.into(), peripheral.dma_peripheral())
    }

    /// Create a new [`Mem2Mem`] instance.
    ///
    /// # Safety
    ///
    /// You must ensure that you're not using DMA for the same peripheral and
    /// that you're the only one using the peripheral. You must also ensure that
    /// the peripheral is compatible with the channel.
    #[cfg(dma_mem2mem_requires_peripheral)]
    pub unsafe fn new_unsafe<CH>(channel: CH, peripheral: DmaPeripheral) -> Self
    where
        CH: Mem2MemCapableChannel + Into<E>,
    {
        Self::new_inner(channel.into(), peripheral)
    }

    /// Convert Mem2Mem to an async Mem2Mem.
    pub fn into_async(self) -> Mem2Mem<E, Async> {
        Mem2Mem {
            rx: self.rx.into_async(),
            tx: self.tx.into_async(),
        }
    }
}

impl<E> Mem2Mem<E, Blocking>
where
    E: DmaChannel,
{
    fn new_inner(channel: E, peripheral: DmaPeripheral) -> Self {
        let mut channel = Channel::new(channel);

        #[cfg(dma_supports_mem2mem)]
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

    /// Shortcut to create a [SimpleMem2Mem]
    pub fn with_descriptors<'d>(
        self,
        rx_descriptors: &'d mut [DmaDescriptor],
        tx_descriptors: &'d mut [DmaDescriptor],
        config: BurstConfig,
    ) -> Result<SimpleMem2Mem<'d, E, Blocking>, DmaError>
    where
        Self: 'd,
    {
        SimpleMem2Mem::new(self, rx_descriptors, tx_descriptors, config)
    }
}

/// The RX half of [Mem2Mem].
pub struct Mem2MemRx<Rx, Dm>
where
    Rx: crate::dma::DmaRxChannel,
    Dm: DriverMode,
{
    channel: ChannelRx<Dm, Rx>,
    peripheral: DmaPeripheral,
}

impl<Rx> Mem2MemRx<Rx, Blocking>
where
    Rx: crate::dma::DmaRxChannel,
{
    /// Convert Mem2MemRx to an async Mem2MemRx.
    pub fn into_async(self) -> Mem2MemRx<Rx, Async> {
        Mem2MemRx {
            channel: self.channel.into_async(),
            peripheral: self.peripheral,
        }
    }
}

impl<Rx, Dm> Mem2MemRx<Rx, Dm>
where
    Rx: crate::dma::DmaRxChannel,
    Dm: DriverMode,
{
    /// Start the RX half of a memory to memory transfer.
    pub fn receive<BUF>(
        mut self,
        mut buf: BUF,
    ) -> Result<Mem2MemRxTransfer<Rx, BUF, Dm>, (DmaError, Self, BUF)>
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
pub struct Mem2MemRxTransfer<Rx, BUF, Dm>
where
    Rx: crate::dma::DmaRxChannel,
    BUF: DmaRxBuffer,
    Dm: DriverMode,
{
    m2m: ManuallyDrop<Mem2MemRx<Rx, Dm>>,
    buf_view: ManuallyDrop<BUF::View>,
}

impl<Rx, BUF, Dm> Mem2MemRxTransfer<Rx, BUF, Dm>
where
    Rx: crate::dma::DmaRxChannel,
    BUF: DmaRxBuffer,
    Dm: DriverMode,
{
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
    pub fn wait(self) -> (Result<(), DmaError>, Mem2MemRx<Rx, Dm>, BUF::Final) {
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
    pub fn stop(self) -> (Mem2MemRx<Rx, Dm>, BUF::Final) {
        let (mut m2m, view) = self.release();

        m2m.channel.stop_transfer();

        (m2m, BUF::from_view(view))
    }

    fn release(mut self) -> (Mem2MemRx<Rx, Dm>, BUF::View) {
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

impl<Rx, BUF, Dm> Deref for Mem2MemRxTransfer<Rx, BUF, Dm>
where
    Rx: crate::dma::DmaRxChannel,
    BUF: DmaRxBuffer,
    Dm: DriverMode,
{
    type Target = BUF::View;

    fn deref(&self) -> &Self::Target {
        &self.buf_view
    }
}

impl<Rx, BUF, Dm> DerefMut for Mem2MemRxTransfer<Rx, BUF, Dm>
where
    Rx: crate::dma::DmaRxChannel,
    BUF: DmaRxBuffer,
    Dm: DriverMode,
{
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.buf_view
    }
}

impl<Rx, BUF, Dm> Drop for Mem2MemRxTransfer<Rx, BUF, Dm>
where
    Rx: crate::dma::DmaRxChannel,
    BUF: DmaRxBuffer,
    Dm: DriverMode,
{
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
pub struct Mem2MemTx<Tx, Dm>
where
    Tx: crate::dma::DmaTxChannel,
    Dm: DriverMode,
{
    channel: ChannelTx<Dm, Tx>,
    peripheral: DmaPeripheral,
}

impl<Tx> Mem2MemTx<Tx, Blocking>
where
    Tx: crate::dma::DmaTxChannel,
{
    /// Convert Mem2MemTx to an async Mem2MemTx.
    pub fn into_async(self) -> Mem2MemTx<Tx, Async> {
        Mem2MemTx {
            channel: self.channel.into_async(),
            peripheral: self.peripheral,
        }
    }
}

impl<Tx, Dm> Mem2MemTx<Tx, Dm>
where
    Tx: crate::dma::DmaTxChannel,
    Dm: DriverMode,
{
    /// Start the TX half of a memory to memory transfer.
    pub fn send<BUF>(
        mut self,
        mut buf: BUF,
    ) -> Result<Mem2MemTxTransfer<Tx, BUF, Dm>, (DmaError, Self, BUF)>
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
pub struct Mem2MemTxTransfer<Tx, BUF, Dm>
where
    Tx: crate::dma::DmaTxChannel,
    BUF: DmaTxBuffer,
    Dm: DriverMode,
{
    m2m: ManuallyDrop<Mem2MemTx<Tx, Dm>>,
    buf_view: ManuallyDrop<BUF::View>,
}

impl<Tx, BUF, Dm> Mem2MemTxTransfer<Tx, BUF, Dm>
where
    Tx: crate::dma::DmaTxChannel,
    BUF: DmaTxBuffer,
    Dm: DriverMode,
{
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
    pub fn wait(self) -> (Result<(), DmaError>, Mem2MemTx<Tx, Dm>, BUF::Final) {
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
    pub fn stop(self) -> (Mem2MemTx<Tx, Dm>, BUF::Final) {
        let (mut m2m, view) = self.release();

        m2m.channel.stop_transfer();

        (m2m, BUF::from_view(view))
    }

    fn release(mut self) -> (Mem2MemTx<Tx, Dm>, BUF::View) {
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

impl<Tx, BUF, Dm> Deref for Mem2MemTxTransfer<Tx, BUF, Dm>
where
    Tx: crate::dma::DmaTxChannel,
    BUF: DmaTxBuffer,
    Dm: DriverMode,
{
    type Target = BUF::View;

    fn deref(&self) -> &Self::Target {
        &self.buf_view
    }
}

impl<Tx, BUF, Dm> DerefMut for Mem2MemTxTransfer<Tx, BUF, Dm>
where
    Tx: crate::dma::DmaTxChannel,
    BUF: DmaTxBuffer,
    Dm: DriverMode,
{
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.buf_view
    }
}

impl<Tx, BUF, Dm> Drop for Mem2MemTxTransfer<Tx, BUF, Dm>
where
    Tx: crate::dma::DmaTxChannel,
    BUF: DmaTxBuffer,
    Dm: DriverMode,
{
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
pub struct SimpleMem2Mem<'d, E, Dm>
where
    E: DmaChannel,
    Dm: DriverMode,
{
    state: State<'d, E, Dm>,
    config: BurstConfig,
}

enum State<'d, E: DmaChannel, Dm: DriverMode> {
    Idle(
        Mem2Mem<E, Dm>,
        &'d mut [DmaDescriptor],
        &'d mut [DmaDescriptor],
    ),
    Active(
        Mem2MemRxTransfer<E::Rx, DmaRxBuf, Dm>,
        Mem2MemTxTransfer<E::Tx, DmaTxBuf, Dm>,
    ),
    InUse,
}

impl<'d, E, Dm> SimpleMem2Mem<'d, E, Dm>
where
    E: DmaChannel,
    Dm: DriverMode,
{
    /// Creates a new [SimpleMem2Mem].
    pub fn new(
        mem2mem: Mem2Mem<E, Dm>,
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

impl<'d, E, Dm> SimpleMem2Mem<'d, E, Dm>
where
    E: DmaChannel,
    Dm: DriverMode,
{
    /// Starts a memory to memory transfer.
    pub fn start_transfer(
        &mut self,
        rx_buffer: &mut [u8],
        tx_buffer: &[u8],
    ) -> Result<SimpleMem2MemTransfer<'_, 'd, E, Dm>, DmaError> {
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

        // Note: The ESP32-S2 insists that RX is started before TX. Contrary to the TRM
        // and every other chip.

        let dma_rx_buf = unwrap!(
            DmaRxBuf::new_with_config(rx_descriptors, rx_buffer, self.config),
            "There's no way to get the descriptors back yet"
        );

        let rx = match mem2mem.rx.receive(dma_rx_buf) {
            Ok(rx) => rx,
            Err((err, rx, buf)) => {
                let (rx_descriptors, _rx_buffer) = buf.split();
                self.state = State::Idle(
                    Mem2Mem { rx, tx: mem2mem.tx },
                    rx_descriptors,
                    tx_descriptors,
                );
                return Err(err);
            }
        };

        let dma_tx_buf = unwrap!(
            DmaTxBuf::new_with_config(tx_descriptors, tx_buffer, self.config),
            "There's no way to get the descriptors back yet"
        );

        let tx = match mem2mem.tx.send(dma_tx_buf) {
            Ok(tx) => tx,
            Err((err, tx, buf)) => {
                let (tx_descriptors, _tx_buffer) = buf.split();
                let (rx, buf) = rx.stop();
                let (rx_descriptors, _rx_buffer) = buf.split();
                self.state = State::Idle(Mem2Mem { rx, tx }, rx_descriptors, tx_descriptors);
                return Err(err);
            }
        };

        self.state = State::Active(rx, tx);

        Ok(SimpleMem2MemTransfer(self))
    }
}

impl<E, Dm> Drop for SimpleMem2Mem<'_, E, Dm>
where
    E: DmaChannel,
    Dm: DriverMode,
{
    fn drop(&mut self) {
        if !matches!(&mut self.state, State::Idle(_, _, _)) {
            panic!("SimpleMem2MemTransfer was forgotten with core::mem::forget or similar");
        }
    }
}

/// Represents an ongoing (or potentially finished) DMA Memory-to-Memory
/// transfer.
pub struct SimpleMem2MemTransfer<'a, 'd, E, Dm>(&'a mut SimpleMem2Mem<'d, E, Dm>)
where
    E: DmaChannel,
    Dm: DriverMode;

impl<E, Dm> SimpleMem2MemTransfer<'_, '_, E, Dm>
where
    E: DmaChannel,
    Dm: DriverMode,
{
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

impl<E, Dm> Drop for SimpleMem2MemTransfer<'_, '_, E, Dm>
where
    E: DmaChannel,
    Dm: DriverMode,
{
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
