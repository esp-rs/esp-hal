use core::{
    mem::ManuallyDrop,
    ops::{Deref, DerefMut},
};

use enumset::EnumSet;

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
pub trait Mem2MemCapableChannel<'d>: DmaChannel {
    #[doc(hidden)]
    type Erased: DmaChannel + From<Self>;

    /// Peripheral selector programmed for memory-to-memory on this channel.
    #[cfg(not(dma_mem2mem_requires_peripheral))]
    fn mem2mem_id(&self) -> DmaPeripheral;

    #[doc(hidden)]
    #[allow(private_interfaces)]
    fn into_channel(self) -> ErasedChannel<'d, Blocking>;
}

// Type-erased version of Channel/ChannelRx/ChannelTx
for_each_mem2mem_channel! {
    (engines $( ($engine:literal, $variant:ident, $any_ch:ident) ),* ) => {
        struct ErasedChannel<'d, Dm: DriverMode> {
            rx: ErasedChannelRx<'d, Dm>,
            tx: ErasedChannelTx<'d, Dm>,
        }

        enum ErasedChannelRx<'d, Dm: DriverMode> {
            $(
                $variant(ChannelRx<Dm, <crate::dma::$any_ch<'d> as DmaChannel>::Rx>),
            )*
        }

        impl<Dm: DriverMode> ErasedChannelRx<'_, Dm> {
            delegate::delegate! {
                to match self {
                    $( Self::$variant(channel) => channel, )*
                } {
                    fn has_error(&self) -> bool;
                    fn pending_in_interrupts(&self) -> EnumSet<DmaRxInterrupt>;
                    #[cfg(dma_mem2mem_requires_peripheral)]
                    fn runtime_ensure_compatible(&self, peripheral: DmaPeripheral);
                }

                to match self {
                    $( Self::$variant(channel) => channel, )*
                } {
                    fn set_mem2mem_mode(&mut self, value: bool);
                    fn start_transfer(&mut self) -> Result<(), DmaError>;
                    fn stop_transfer(&mut self);
                    unsafe fn prepare_transfer<BUF: DmaRxBuffer>(
                        &mut self,
                        peri: DmaPeripheral,
                        buffer: &mut BUF,
                    ) -> Result<(), DmaError>;
                }
            }
        }

        impl<'d> ErasedChannelRx<'d, Blocking> {
            fn into_async(self) -> ErasedChannelRx<'d, Async> {
                match self {
                    $( Self::$variant(channel) => ErasedChannelRx::$variant(channel.into_async()), )*
                }
            }
        }

        impl<'d> ErasedChannelRx<'d, Async> {
            fn into_blocking(self) -> ErasedChannelRx<'d, Blocking> {
                match self {
                    $( Self::$variant(channel) => ErasedChannelRx::$variant(channel.into_blocking()), )*
                }
            }
        }

        enum ErasedChannelTx<'d, Dm: DriverMode> {
            $(
                $variant(ChannelTx<Dm, <crate::dma::$any_ch<'d> as DmaChannel>::Tx>),
            )*
        }

        impl<Dm: DriverMode> ErasedChannelTx<'_, Dm> {
            delegate::delegate! {
                to match self {
                    $( Self::$variant(channel) => channel, )*
                } {
                    fn has_error(&self) -> bool;
                    fn pending_out_interrupts(&self) -> EnumSet<DmaTxInterrupt>;
                }

                to match self {
                    $( Self::$variant(channel) => channel, )*
                } {
                    fn start_transfer(&mut self) -> Result<(), DmaError>;
                    fn stop_transfer(&mut self);
                    unsafe fn prepare_transfer<BUF: DmaTxBuffer>(
                        &mut self,
                        peri: DmaPeripheral,
                        buffer: &mut BUF,
                    ) -> Result<(), DmaError>;
                }
            }
        }

        impl<'d> ErasedChannelTx<'d, Blocking> {
            fn into_async(self) -> ErasedChannelTx<'d, Async> {
                match self {
                    $( Self::$variant(channel) => ErasedChannelTx::$variant(channel.into_async()), )*
                }
            }
        }

        impl<'d> ErasedChannelTx<'d, Async> {
            fn into_blocking(self) -> ErasedChannelTx<'d, Blocking> {
                match self {
                    $( Self::$variant(channel) => ErasedChannelTx::$variant(channel.into_blocking()), )*
                }
            }
        }
    };
}

for_each_mem2mem_channel! {
    ($engine:literal, $variant:ident, $any_ch:ident, $($hw:literal, $id:literal),+) => {
        impl<'d> Mem2MemCapableChannel<'d> for crate::dma::$any_ch<'d> {
            type Erased = Self;

            #[cfg(not(dma_mem2mem_requires_peripheral))]
            fn mem2mem_id(&self) -> DmaPeripheral {
                match self.channel_index() {
                    $( $hw => DmaPeripheral($id), )+
                    ch => panic!(
                        "Channel {} does not support memory-to-memory transfers",
                        ch
                    ),
                }
            }

            fn into_channel(self) -> ErasedChannel<'d, Blocking> {
                let channel = Channel::new(self);
                ErasedChannel {
                    rx: ErasedChannelRx::$variant(channel.rx),
                    tx: ErasedChannelTx::$variant(channel.tx),
                }
            }
        }
    };
    ($engine:literal, $variant:ident, $any_ch:ident, $ch:ident, $id:literal) => {
        impl<'d> Mem2MemCapableChannel<'d> for crate::peripherals::$ch<'d> {
            type Erased = crate::dma::$any_ch<'d>;

            #[cfg(not(dma_mem2mem_requires_peripheral))]
            fn mem2mem_id(&self) -> DmaPeripheral {
                DmaPeripheral($id)
            }

            fn into_channel(self) -> ErasedChannel<'d, Blocking> {
                let channel = Channel::new(crate::dma::$any_ch::from(self));
                ErasedChannel {
                    rx: ErasedChannelRx::$variant(channel.rx),
                    tx: ErasedChannelTx::$variant(channel.tx),
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
    /// Create a new [`Mem2Mem`] instance.
    pub fn new<CH>(
        channel: CH,
        #[cfg(dma_mem2mem_requires_peripheral)] peripheral: impl DmaEligiblePeripheral<CH::Erased>,
    ) -> Self
    where
        CH: Mem2MemCapableChannel<'d>,
    {
        let dma_peri = cfg_select! {
            dma_mem2mem_requires_peripheral => peripheral.dma_peripheral(),
            _ => channel.mem2mem_id(),
        };
        Self::new_inner(channel, dma_peri)
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
        CH: Mem2MemCapableChannel<'d>,
    {
        Self::new_inner(channel, peripheral)
    }

    /// Convert Mem2Mem to an async Mem2Mem.
    pub fn into_async(self) -> Mem2Mem<'d, Async> {
        Mem2Mem {
            rx: self.rx.into_async(),
            tx: self.tx.into_async(),
        }
    }
}

impl<'d> Mem2Mem<'d, Blocking> {
    fn new_inner(channel: impl Mem2MemCapableChannel<'d>, peripheral: DmaPeripheral) -> Self {
        let mut channel = channel.into_channel();

        #[cfg(dma_mem2mem_requires_peripheral)]
        channel.rx.runtime_ensure_compatible(peripheral);

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
    pub fn with_descriptors(
        self,
        rx_descriptors: &'d mut [DmaDescriptor],
        tx_descriptors: &'d mut [DmaDescriptor],
        config: BurstConfig,
    ) -> Result<SimpleMem2Mem<'d, Blocking>, DmaError> {
        SimpleMem2Mem::new(self, rx_descriptors, tx_descriptors, config)
    }
}

/// The RX half of [Mem2Mem].
pub struct Mem2MemRx<'d, Dm>
where
    Dm: DriverMode,
{
    channel: ErasedChannelRx<'d, Dm>,
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
    ) -> Result<Mem2MemRxTransfer<'d, BUF, Dm>, (DmaError, Self, BUF)>
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
pub struct Mem2MemRxTransfer<'d, BUF, Dm>
where
    BUF: DmaRxBuffer,
    Dm: DriverMode,
{
    m2m: ManuallyDrop<Mem2MemRx<'d, Dm>>,
    buf_view: ManuallyDrop<BUF::View>,
}

impl<'d, BUF, Dm> Mem2MemRxTransfer<'d, BUF, Dm>
where
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
    pub fn wait(self) -> (Result<(), DmaError>, Mem2MemRx<'d, Dm>, BUF::Final) {
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
    pub fn stop(self) -> (Mem2MemRx<'d, Dm>, BUF::Final) {
        let (mut m2m, view) = self.release();

        m2m.channel.stop_transfer();

        (m2m, BUF::from_view(view))
    }

    fn release(mut self) -> (Mem2MemRx<'d, Dm>, BUF::View) {
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

impl<'d, BUF, Dm> Deref for Mem2MemRxTransfer<'d, BUF, Dm>
where
    BUF: DmaRxBuffer,
    Dm: DriverMode,
{
    type Target = BUF::View;

    fn deref(&self) -> &Self::Target {
        &self.buf_view
    }
}

impl<'d, BUF, Dm> DerefMut for Mem2MemRxTransfer<'d, BUF, Dm>
where
    BUF: DmaRxBuffer,
    Dm: DriverMode,
{
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.buf_view
    }
}

impl<'d, BUF, Dm> Drop for Mem2MemRxTransfer<'d, BUF, Dm>
where
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
pub struct Mem2MemTx<'d, Dm>
where
    Dm: DriverMode,
{
    channel: ErasedChannelTx<'d, Dm>,
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

impl<'d, Dm> Mem2MemTx<'d, Dm>
where
    Dm: DriverMode,
{
    /// Start the TX half of a memory to memory transfer.
    pub fn send<BUF>(
        mut self,
        mut buf: BUF,
    ) -> Result<Mem2MemTxTransfer<'d, BUF, Dm>, (DmaError, Self, BUF)>
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
pub struct Mem2MemTxTransfer<'d, BUF, Dm>
where
    BUF: DmaTxBuffer,
    Dm: DriverMode,
{
    m2m: ManuallyDrop<Mem2MemTx<'d, Dm>>,
    buf_view: ManuallyDrop<BUF::View>,
}

impl<'d, BUF, Dm> Mem2MemTxTransfer<'d, BUF, Dm>
where
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
    pub fn wait(self) -> (Result<(), DmaError>, Mem2MemTx<'d, Dm>, BUF::Final) {
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
    pub fn stop(self) -> (Mem2MemTx<'d, Dm>, BUF::Final) {
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

impl<'d, BUF, Dm> Deref for Mem2MemTxTransfer<'d, BUF, Dm>
where
    BUF: DmaTxBuffer,
    Dm: DriverMode,
{
    type Target = BUF::View;

    fn deref(&self) -> &Self::Target {
        &self.buf_view
    }
}

impl<'d, BUF, Dm> DerefMut for Mem2MemTxTransfer<'d, BUF, Dm>
where
    BUF: DmaTxBuffer,
    Dm: DriverMode,
{
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.buf_view
    }
}

impl<'d, BUF, Dm> Drop for Mem2MemTxTransfer<'d, BUF, Dm>
where
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
pub struct SimpleMem2Mem<'d, Dm>
where
    Dm: DriverMode,
{
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
        Mem2MemRxTransfer<'d, DmaRxBuf, Dm>,
        Mem2MemTxTransfer<'d, DmaTxBuf, Dm>,
    ),
    InUse,
}

impl<'d, Dm> SimpleMem2Mem<'d, Dm>
where
    Dm: DriverMode,
{
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

impl<'d, Dm> SimpleMem2Mem<'d, Dm>
where
    Dm: DriverMode,
{
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

impl<Dm> Drop for SimpleMem2Mem<'_, Dm>
where
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
pub struct SimpleMem2MemTransfer<'a, 'd, Dm>(&'a mut SimpleMem2Mem<'d, Dm>)
where
    Dm: DriverMode;

impl<Dm> SimpleMem2MemTransfer<'_, '_, Dm>
where
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

impl<Dm> Drop for SimpleMem2MemTransfer<'_, '_, Dm>
where
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
