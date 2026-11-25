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
        DmaConfigError,
        DmaDescriptor,
        DmaError,
        DmaPeripheral,
        DmaRxBuf,
        DmaRxBuffer,
        DmaRxInterrupt,
        DmaTxBuf,
        DmaTxBuffer,
        DmaTxInterrupt,
        aligned::DmaAlignedMut,
    },
};

/// A DMA engine that supports memory-to-memory transfers.
///
/// [`Mem2Mem`] and the other memory-to-memory types take this engine as a type parameter.
#[instability::unstable]
pub trait Mem2MemEngine: crate::private::Sealed {
    /// The configuration type of this engine.
    type Config: Copy + Default;

    #[doc(hidden)]
    type Channel<'d>: DmaChannel<Config = Self::Config>;
}

/// A DMA channel singleton that supports memory-to-memory transfers on this chip.
///
/// Only channels listed in device metadata (`mem2mem = true`) implement this trait.
/// Use [`Mem2Mem::new`] to construct a transfer engine from such a channel.
#[diagnostic::on_unimplemented(
    message = "this DMA channel does not support memory-to-memory transfers",
    note = "Use a channel with `mem2mem = true` in device metadata. See `Mem2Mem::new`."
)]
pub trait Mem2MemCapableChannel<'d>: DmaChannel {
    /// The [`Mem2MemEngine`] of this channel.
    type Engine: Mem2MemEngine;

    /// Peripheral selector programmed for memory-to-memory on this channel.
    #[cfg(not(dma_mem2mem_requires_peripheral))]
    fn mem2mem_id(&self) -> DmaPeripheral;

    #[doc(hidden)]
    fn degrade(self) -> <Self::Engine as Mem2MemEngine>::Channel<'d>;
}

for_each_mem2mem_channel! {
    (engines $( ($engine:tt, $variant:ident, $any_ch:ident) ),* ) => {
        $(
            #[doc = concat!("Marker type for the `", $engine, "` [`Mem2MemEngine`].")]
            #[derive(Debug)]
            #[cfg_attr(feature = "defmt", derive(defmt::Format))]
            pub enum $variant {}

            impl crate::private::Sealed for $variant {}

            impl Mem2MemEngine for $variant {
                type Config = crate::dma::engine_config!($engine);
                type Channel<'d> = crate::dma::$any_ch<'d>;
            }
        )*
    };
}

for_each_mem2mem_channel! {
    ($engine:literal, $variant:ident, $any_ch:ident, $($hw:literal, $id:literal),+) => {
        impl<'d> Mem2MemCapableChannel<'d> for crate::dma::$any_ch<'d> {
            type Engine = $variant;

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

            fn degrade(self) -> crate::dma::$any_ch<'d> {
                self
            }
        }
    };
    ($engine:literal, $variant:ident, $any_ch:ident, $ch:ident, $id:literal) => {
        impl<'d> Mem2MemCapableChannel<'d> for crate::peripherals::$ch<'d> {
            type Engine = $variant;

            #[cfg(not(dma_mem2mem_requires_peripheral))]
            fn mem2mem_id(&self) -> DmaPeripheral {
                DmaPeripheral($id)
            }

            fn degrade(self) -> crate::dma::$any_ch<'d> {
                crate::dma::$any_ch::from(self)
            }
        }
    };
}

type EngineChannel<'d, E> = <E as Mem2MemEngine>::Channel<'d>;
type EngineRx<'d, Dm, E> = ChannelRx<Dm, <EngineChannel<'d, E> as DmaChannel>::Rx>;
type EngineTx<'d, Dm, E> = ChannelTx<Dm, <EngineChannel<'d, E> as DmaChannel>::Tx>;

/// DMA Memory to Memory pseudo-Peripheral
///
/// This is a pseudo-peripheral that allows for memory to memory transfers.
/// It is not a real peripheral, but a way to use the DMA engine for memory
/// to memory transfers.
pub struct Mem2Mem<'d, Dm, E>
where
    Dm: DriverMode,
    E: Mem2MemEngine,
{
    /// RX Half
    pub rx: Mem2MemRx<'d, Dm, E>,
    /// TX Half
    pub tx: Mem2MemTx<'d, Dm, E>,
}

impl<'d, E> Mem2Mem<'d, Blocking, E>
where
    E: Mem2MemEngine,
{
    /// Creates a new [`Mem2Mem`] instance.
    pub fn new<CH>(
        channel: CH,
        #[cfg(dma_mem2mem_requires_peripheral)] peripheral: impl DmaEligiblePeripheral<
            EngineChannel<'d, E>,
        >,
    ) -> Self
    where
        CH: Mem2MemCapableChannel<'d, Engine = E>,
    {
        let dma_peri = cfg_select! {
            dma_mem2mem_requires_peripheral => peripheral.dma_peripheral(),
            _ => channel.mem2mem_id(),
        };
        Self::new_inner(channel, dma_peri)
    }

    /// Creates a new [`Mem2Mem`] instance.
    ///
    /// # Safety
    ///
    /// The caller must ensure that DMA is not used for the same peripheral,
    /// that this is the only user of the peripheral, and that the peripheral is
    /// compatible with the channel.
    #[cfg(dma_mem2mem_requires_peripheral)]
    pub unsafe fn new_unsafe<CH>(channel: CH, peripheral: DmaPeripheral) -> Self
    where
        CH: Mem2MemCapableChannel<'d, Engine = E>,
    {
        Self::new_inner(channel, peripheral)
    }

    /// Converts Mem2Mem to an async Mem2Mem.
    pub fn into_async(self) -> Mem2Mem<'d, Async, E> {
        Mem2Mem {
            rx: self.rx.into_async(),
            tx: self.tx.into_async(),
        }
    }

    fn new_inner<CH>(channel: CH, peripheral: DmaPeripheral) -> Self
    where
        CH: Mem2MemCapableChannel<'d, Engine = E>,
    {
        let mut channel = Channel::new(channel.degrade());

        #[cfg(dma_mem2mem_requires_peripheral)]
        channel.rx.runtime_ensure_compatible(peripheral);

        channel.rx.set_peripheral(peripheral);
        channel.tx.set_peripheral(peripheral);

        #[cfg(dma_supports_mem2mem)]
        channel.rx.set_mem2mem_mode(true);

        Mem2Mem {
            rx: Mem2MemRx {
                channel: channel.rx,
            },
            tx: Mem2MemTx {
                channel: channel.tx,
            },
        }
    }

    /// Shortcut to create a [SimpleMem2Mem]
    pub fn with_descriptors(
        self,
        rx_descriptors: &'d mut [DmaDescriptor],
        tx_descriptors: &'d mut [DmaDescriptor],
        config: BurstConfig,
    ) -> Result<SimpleMem2Mem<'d, Blocking, E>, DmaError> {
        SimpleMem2Mem::new(self, rx_descriptors, tx_descriptors, config)
    }
}

impl<Dm, E> Mem2Mem<'_, Dm, E>
where
    Dm: DriverMode,
    E: Mem2MemEngine,
{
    /// Applies a DMA configuration to both halves of the DMA channel.
    ///
    /// # Errors
    ///
    /// [`DmaConfigError`] when the channel cannot apply the configuration.
    #[instability::unstable]
    pub fn apply_dma_config(&mut self, config: &E::Config) -> Result<(), DmaConfigError> {
        self.rx.channel.apply_config(config)?;
        self.tx.channel.apply_config(config)
    }
}

/// The RX half of [Mem2Mem].
pub struct Mem2MemRx<'d, Dm, E>
where
    Dm: DriverMode,
    E: Mem2MemEngine,
{
    channel: EngineRx<'d, Dm, E>,
}

impl<'d, E> Mem2MemRx<'d, Blocking, E>
where
    E: Mem2MemEngine,
{
    /// Converts Mem2MemRx to an async Mem2MemRx.
    pub fn into_async(self) -> Mem2MemRx<'d, Async, E> {
        Mem2MemRx {
            channel: self.channel.into_async(),
        }
    }
}

impl<'d, Dm, E> Mem2MemRx<'d, Dm, E>
where
    Dm: DriverMode,
    E: Mem2MemEngine,
{
    /// Starts the RX half of a memory to memory transfer.
    pub fn receive<BUF>(
        mut self,
        mut buf: BUF,
    ) -> Result<Mem2MemRxTransfer<'d, BUF, Dm, E>, (DmaError, Self, BUF)>
    where
        BUF: DmaRxBuffer,
    {
        let result = unsafe {
            self.channel
                .prepare_transfer(&mut buf)
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
pub struct Mem2MemRxTransfer<'d, BUF, Dm, E>
where
    BUF: DmaRxBuffer,
    Dm: DriverMode,
    E: Mem2MemEngine,
{
    m2m: ManuallyDrop<Mem2MemRx<'d, Dm, E>>,
    buf_view: ManuallyDrop<BUF::View>,
}

impl<'d, BUF, Dm, E> Mem2MemRxTransfer<'d, BUF, Dm, E>
where
    BUF: DmaRxBuffer,
    Dm: DriverMode,
    E: Mem2MemEngine,
{
    /// Returns whether [`Self::wait`] will not block.
    pub fn is_done(&self) -> bool {
        let done_interrupts = DmaRxInterrupt::DescriptorError | DmaRxInterrupt::DescriptorEmpty;
        !self
            .m2m
            .channel
            .pending_in_interrupts()
            .is_disjoint(done_interrupts)
    }

    /// Waits for the transfer to stop and returns the peripheral and buffer.
    pub fn wait(self) -> (Result<(), DmaError>, Mem2MemRx<'d, Dm, E>, BUF::Final) {
        while !self.is_done() {}

        let (mut m2m, view) = self.release();
        m2m.channel.end_transfer();

        let result = if m2m.channel.has_error() {
            Err(DmaError::DescriptorError)
        } else {
            Ok(())
        };

        (result, m2m, BUF::from_view(view))
    }

    /// Stops this transfer on the spot and returns the peripheral and buffer.
    pub fn stop(self) -> (Mem2MemRx<'d, Dm, E>, BUF::Final) {
        let (mut m2m, view) = self.release();

        m2m.channel.stop_transfer();

        (m2m, BUF::from_view(view))
    }

    fn release(mut self) -> (Mem2MemRx<'d, Dm, E>, BUF::View) {
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

impl<'d, BUF, Dm, E> Deref for Mem2MemRxTransfer<'d, BUF, Dm, E>
where
    BUF: DmaRxBuffer,
    Dm: DriverMode,
    E: Mem2MemEngine,
{
    type Target = BUF::View;

    fn deref(&self) -> &Self::Target {
        &self.buf_view
    }
}

impl<'d, BUF, Dm, E> DerefMut for Mem2MemRxTransfer<'d, BUF, Dm, E>
where
    BUF: DmaRxBuffer,
    Dm: DriverMode,
    E: Mem2MemEngine,
{
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.buf_view
    }
}

impl<'d, BUF, Dm, E> Drop for Mem2MemRxTransfer<'d, BUF, Dm, E>
where
    BUF: DmaRxBuffer,
    Dm: DriverMode,
    E: Mem2MemEngine,
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
pub struct Mem2MemTx<'d, Dm, E>
where
    Dm: DriverMode,
    E: Mem2MemEngine,
{
    channel: EngineTx<'d, Dm, E>,
}

impl<'d, E> Mem2MemTx<'d, Blocking, E>
where
    E: Mem2MemEngine,
{
    /// Converts Mem2MemTx to an async Mem2MemTx.
    pub fn into_async(self) -> Mem2MemTx<'d, Async, E> {
        Mem2MemTx {
            channel: self.channel.into_async(),
        }
    }
}

impl<'d, Dm, E> Mem2MemTx<'d, Dm, E>
where
    Dm: DriverMode,
    E: Mem2MemEngine,
{
    /// Starts the TX half of a memory to memory transfer.
    pub fn send<BUF>(
        mut self,
        mut buf: BUF,
    ) -> Result<Mem2MemTxTransfer<'d, BUF, Dm, E>, (DmaError, Self, BUF)>
    where
        BUF: DmaTxBuffer,
    {
        let result = unsafe {
            self.channel
                .prepare_transfer(&mut buf)
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
pub struct Mem2MemTxTransfer<'d, BUF, Dm, E>
where
    BUF: DmaTxBuffer,
    Dm: DriverMode,
    E: Mem2MemEngine,
{
    m2m: ManuallyDrop<Mem2MemTx<'d, Dm, E>>,
    buf_view: ManuallyDrop<BUF::View>,
}

impl<'d, BUF, Dm, E> Mem2MemTxTransfer<'d, BUF, Dm, E>
where
    BUF: DmaTxBuffer,
    Dm: DriverMode,
    E: Mem2MemEngine,
{
    /// Returns whether [`Self::wait`] will not block.
    pub fn is_done(&self) -> bool {
        let done_interrupts = DmaTxInterrupt::DescriptorError | DmaTxInterrupt::TotalEof;
        !self
            .m2m
            .channel
            .pending_out_interrupts()
            .is_disjoint(done_interrupts)
    }

    /// Waits for the transfer to stop and returns the peripheral and buffer.
    pub fn wait(self) -> (Result<(), DmaError>, Mem2MemTx<'d, Dm, E>, BUF::Final) {
        while !self.is_done() {}

        let (mut m2m, view) = self.release();
        m2m.channel.end_transfer();

        let result = if m2m.channel.has_error() {
            Err(DmaError::DescriptorError)
        } else {
            Ok(())
        };

        (result, m2m, BUF::from_view(view))
    }

    /// Stops this transfer on the spot and returns the peripheral and buffer.
    pub fn stop(self) -> (Mem2MemTx<'d, Dm, E>, BUF::Final) {
        let (mut m2m, view) = self.release();

        m2m.channel.stop_transfer();

        (m2m, BUF::from_view(view))
    }

    fn release(mut self) -> (Mem2MemTx<'d, Dm, E>, BUF::View) {
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

impl<'d, BUF, Dm, E> Deref for Mem2MemTxTransfer<'d, BUF, Dm, E>
where
    BUF: DmaTxBuffer,
    Dm: DriverMode,
    E: Mem2MemEngine,
{
    type Target = BUF::View;

    fn deref(&self) -> &Self::Target {
        &self.buf_view
    }
}

impl<'d, BUF, Dm, E> DerefMut for Mem2MemTxTransfer<'d, BUF, Dm, E>
where
    BUF: DmaTxBuffer,
    Dm: DriverMode,
    E: Mem2MemEngine,
{
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.buf_view
    }
}

impl<'d, BUF, Dm, E> Drop for Mem2MemTxTransfer<'d, BUF, Dm, E>
where
    BUF: DmaTxBuffer,
    Dm: DriverMode,
    E: Mem2MemEngine,
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

/// A simple and easy to use wrapper around [`Mem2Mem`].
/// More complex memory to memory transfers should use [`Mem2Mem`] directly.
pub struct SimpleMem2Mem<'d, Dm, E>
where
    Dm: DriverMode,
    E: Mem2MemEngine,
{
    state: State<'d, Dm, E>,
    config: BurstConfig,
}

enum State<'d, Dm: DriverMode, E: Mem2MemEngine> {
    Idle(
        Mem2Mem<'d, Dm, E>,
        DmaAlignedMut<'d, [DmaDescriptor]>,
        DmaAlignedMut<'d, [DmaDescriptor]>,
    ),
    Active(
        Mem2MemRxTransfer<'d, DmaRxBuf, Dm, E>,
        Mem2MemTxTransfer<'d, DmaTxBuf, Dm, E>,
    ),
    InUse,
}

impl<'d, Dm, E> SimpleMem2Mem<'d, Dm, E>
where
    Dm: DriverMode,
    E: Mem2MemEngine,
{
    /// Creates a new [SimpleMem2Mem].
    pub fn new(
        mem2mem: Mem2Mem<'d, Dm, E>,
        rx_descriptors: &'d mut [DmaDescriptor],
        tx_descriptors: &'d mut [DmaDescriptor],
        config: BurstConfig,
    ) -> Result<Self, DmaError> {
        if rx_descriptors.is_empty() || tx_descriptors.is_empty() {
            return Err(DmaError::OutOfDescriptors);
        }

        // Safety: descriptors are aligned to what the DMA requires, and we don't call invalidate on
        // these slices.
        let rx_descriptors = unsafe { DmaAlignedMut::new_unchecked(rx_descriptors) };
        let tx_descriptors = unsafe { DmaAlignedMut::new_unchecked(tx_descriptors) };

        Ok(Self {
            state: State::Idle(mem2mem, rx_descriptors, tx_descriptors),
            config,
        })
    }
}

impl<'d, Dm, E> SimpleMem2Mem<'d, Dm, E>
where
    Dm: DriverMode,
    E: Mem2MemEngine,
{
    /// Starts a memory to memory transfer.
    pub fn start_transfer(
        &mut self,
        rx_buffer: &mut [u8],
        tx_buffer: &[u8],
    ) -> Result<SimpleMem2MemTransfer<'_, 'd, Dm, E>, DmaError> {
        if !matches!(self.state, State::Idle(..)) {
            panic!("SimpleMem2MemTransfer was forgotten with core::mem::forget or similar");
        }

        // Raise these buffers to 'static. This is not safe, bad things will happen if
        // the user calls core::mem::forget on SimpleMem2MemTransfer. This is
        // just the unfortunate consequence of doing DMA without enforcing
        // 'static.
        let rx_buffer = DmaAlignedMut::new(unsafe {
            core::slice::from_raw_parts_mut(rx_buffer.as_mut_ptr(), rx_buffer.len())
        })?;

        let State::Idle(mem2mem, mut rx_descriptors, mut tx_descriptors) =
            core::mem::replace(&mut self.state, State::InUse)
        else {
            unreachable!("the state is Idle");
        };

        let tx_buffer = unsafe {
            DmaAlignedMut::new_unchecked(core::slice::from_raw_parts_mut(
                tx_buffer.as_ptr() as _,
                tx_buffer.len(),
            ))
        };
        let rx_descriptors = unsafe {
            DmaAlignedMut::new_unchecked(core::slice::from_raw_parts_mut(
                rx_descriptors.as_mut_ptr(),
                rx_descriptors.len(),
            ))
        };
        let tx_descriptors = unsafe {
            DmaAlignedMut::new_unchecked(core::slice::from_raw_parts_mut(
                tx_descriptors.as_mut_ptr(),
                tx_descriptors.len(),
            ))
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

impl<Dm, E> Drop for SimpleMem2Mem<'_, Dm, E>
where
    Dm: DriverMode,
    E: Mem2MemEngine,
{
    fn drop(&mut self) {
        if !matches!(&mut self.state, State::Idle(_, _, _)) {
            panic!("SimpleMem2MemTransfer was forgotten with core::mem::forget or similar");
        }
    }
}

/// Represents an ongoing (or potentially finished) DMA Memory-to-Memory
/// transfer.
pub struct SimpleMem2MemTransfer<'a, 'd, Dm, E>(&'a mut SimpleMem2Mem<'d, Dm, E>)
where
    Dm: DriverMode,
    E: Mem2MemEngine;

impl<Dm, E> SimpleMem2MemTransfer<'_, '_, Dm, E>
where
    Dm: DriverMode,
    E: Mem2MemEngine,
{
    /// Returns whether [`Self::wait`] will not block.
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

    /// Waits for the transfer to finish.
    pub fn wait(self) -> Result<(), DmaError> {
        while !self.is_done() {}
        Ok(())
    }
}

impl<Dm, E> Drop for SimpleMem2MemTransfer<'_, '_, Dm, E>
where
    Dm: DriverMode,
    E: Mem2MemEngine,
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
