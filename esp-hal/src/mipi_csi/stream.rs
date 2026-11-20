//! Continuous capture into two frame buffers that the DMA cycles through.

use core::{future::poll_fn, mem::ManuallyDrop, ops::Deref, ptr, task::Poll};

use esp_sync::NonReentrantMutex;

#[cfg(any(soc_internal_memory_cached, dma_can_access_psram))]
use crate::dma::aligned::DmaAlignedMut;
use crate::{
    Async,
    Blocking,
    DriverMode,
    asynch::AtomicWaker,
    dma::DmaBufError,
    interrupt,
    mipi_csi::{
        CsiCapture,
        DMA_COMPLETION_INTERRUPT_MASK,
        DMA_STATUS_BLOCK_TRANSFER_DONE,
        DMA_STATUS_DMA_TRANSFER_DONE,
        DMA_STATUS_ERROR_MASK,
        bridge_event_status,
        flush_bridge_fifo,
        host_error_status,
        stop_dma_channel,
    },
    mipi_dw_gdma::{
        BlockTransferConfig,
        DwGdmaChannel,
        channel_interrupt_available,
        register_channel_handler,
        unregister_channel_handler,
    },
    peripherals::{Interrupt, MIPI_CSI_BRIDGE},
    system::Cpu,
    time::{Duration, Instant},
};

/// Number of frame buffers a [`CsiStream`] cycles through.
pub const STREAM_BUFFER_COUNT: usize = 2;

/// Ownership of one stream buffer.
///
/// `Free -> Dma` when the interrupt handler, or a resync, arms the channel on
/// it. `Dma -> Ready` when its block completed as one whole, error-free frame,
/// `Dma -> Free` otherwise. `Ready -> App` when [`CsiStream::next_frame`]
/// hands it out, `Ready -> Dma` when a newer frame completed first and the
/// buffer is reused. `App -> Free` when the [`StreamFrame`] is dropped.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Slot {
    Free,
    Dma,
    Ready(u32),
    App,
}

/// State shared by the stream handle and the DMA and bridge interrupt handlers.
struct Shared {
    active: bool,
    channel: Option<DwGdmaChannel>,
    slots: [Slot; STREAM_BUFFER_COUNT],
    addrs: [u32; STREAM_BUFFER_COUNT],
    lens: [u32; STREAM_BUFFER_COUNT],
    /// The block of the slot in `Dma`, with its destination address unset.
    block: Option<BlockTransferConfig>,
    frame_bytes: u32,
    /// The bridge is off and no slot is in `Dma`. Set after a frame failed,
    /// because the next block boundary would no longer be a frame boundary.
    paused: bool,
    /// A DMA error left the channel enabled. Only the bounded shutdown in
    /// [`CsiStream::stop`] may touch the channel.
    fault: bool,
    next_sequence: u32,
    stats: StreamStats,
}

impl Shared {
    const IDLE: Self = Self {
        active: false,
        channel: None,
        slots: [Slot::Free; STREAM_BUFFER_COUNT],
        addrs: [0; STREAM_BUFFER_COUNT],
        lens: [0; STREAM_BUFFER_COUNT],
        block: None,
        frame_bytes: 0,
        paused: false,
        fault: false,
        next_sequence: 0,
        stats: StreamStats::ZERO,
    };

    fn dma_slot(&self) -> Option<usize> {
        self.slots.iter().position(|slot| *slot == Slot::Dma)
    }

    /// Points the channel at `slot` and enables it.
    fn arm(&mut self, channel: DwGdmaChannel, slot: usize) {
        let Some(mut block) = self.block else {
            return;
        };
        block.dst.addr = self.addrs[slot];
        self.slots[slot] = Slot::Dma;
        channel.configure_block_transfer(block);
        channel.set_enabled(true);
    }
}

static SHARED: NonReentrantMutex<Shared> = NonReentrantMutex::new(Shared::IDLE);
/// Woken by the interrupt handler after every change of [`SHARED`].
static STREAM_WAKER: AtomicWaker = AtomicWaker::new();

/// Counters of a [`CsiStream`] since it started.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct StreamStats {
    /// Frames the DMA completed with the configured size and no DMA, bridge or
    /// host error.
    pub completed: u32,
    /// Completed frames handed out by [`CsiStream::next_frame`].
    pub delivered: u32,
    /// Completed frames that were overwritten before they were handed out,
    /// because a newer frame completed or the application held the other
    /// buffer.
    pub overwritten: u32,
    /// Frames dropped because of an error or a size other than the configured
    /// frame size.
    pub errored: u32,
    /// Bridge restarts after a failed frame.
    pub resyncs: u32,
    /// Runs of the stream's DMA interrupt handler.
    pub interrupts: u32,
}

impl StreamStats {
    const ZERO: Self = Self {
        completed: 0,
        delivered: 0,
        overwritten: 0,
        errored: 0,
        resyncs: 0,
        interrupts: 0,
    };
}

/// Why [`CsiCapture::into_stream`] did not start.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum StreamStartError {
    /// A buffer is smaller than the configured frame size.
    BufferTooSmall {
        /// Configured frame size in bytes.
        required: usize,
        /// Size of the smallest buffer.
        provided: usize,
    },
    /// A buffer is not DMA-capable or not aligned for it.
    InvalidBuffer(DmaBufError),
    /// The buffers overlap.
    BuffersOverlap,
    /// The capture channel already has a handler in the shared DW-GDMA
    /// interrupt dispatcher, or another stream is running.
    InterruptUnavailable,
}

impl core::fmt::Display for StreamStartError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::BufferTooSmall { required, provided } => write!(
                f,
                "stream buffer too small (required {required} bytes, provided {provided} bytes)"
            ),
            Self::InvalidBuffer(err) => write!(f, "invalid DMA buffer: {err}"),
            Self::BuffersOverlap => write!(f, "stream buffers overlap"),
            Self::InterruptUnavailable => write!(f, "capture channel interrupt already in use"),
        }
    }
}

impl core::error::Error for StreamStartError {}

/// Why [`CsiStream::next_frame`] returned no frame.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum StreamError {
    /// No frame completed before the timeout.
    Timeout,
    /// A DMA error left the channel running. The stream delivers no more
    /// frames; [`CsiStream::stop`] shuts the channel down.
    DmaFault,
}

impl core::fmt::Display for StreamError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Timeout => write!(f, "timed out waiting for a stream frame"),
            Self::DmaFault => write!(f, "DMA error stopped the stream"),
        }
    }
}

impl core::error::Error for StreamError {}

/// A [`CsiCapture`] that captures continuously into two buffers.
///
/// The DMA interrupt handler starts the next frame on the other buffer as soon
/// as one completes, during the sensor's vertical blanking, so the source can
/// stream without being stopped between frames. A buffer is either written by
/// the DMA or readable by the application, never both: [`Self::next_frame`]
/// only hands out a buffer whose block completed and that the DMA was not
/// re-armed on.
///
/// When the application does not take frames as fast as the source sends
/// them, only the newest completed frame is kept. While the application holds
/// a frame, the DMA keeps capturing into the other buffer and overwrites each
/// frame with the next one. [`StreamStats::overwritten`] counts those frames.
///
/// A frame with a DMA, bridge or host error, or with a size other than the
/// configured frame size, is dropped. The bridge is then switched off and
/// switched on again by the next [`Self::next_frame`] call, so that the next
/// frame starts on a frame boundary again.
///
/// In [`Async`] mode, [`Self::next_frame_async`] waits for the DMA interrupt
/// instead of polling.
pub struct CsiStream<'d, Dm: DriverMode = Blocking> {
    capture: CsiCapture<'d, Dm>,
    buffers: [&'static mut [u8]; STREAM_BUFFER_COUNT],
}

/// A completed frame, borrowed from a [`CsiStream`].
///
/// While it is alive the DMA does not write its buffer, and the stream stays
/// borrowed. Dropping it gives the buffer back to the stream.
pub struct StreamFrame<'s> {
    data: &'s [u8],
    slot: usize,
    sequence: u32,
}

impl StreamFrame<'_> {
    /// Position of this frame among the completed frames of the stream,
    /// starting at 0. Gaps are frames that were overwritten.
    pub fn sequence(&self) -> u32 {
        self.sequence
    }
}

impl Deref for StreamFrame<'_> {
    type Target = [u8];

    fn deref(&self) -> &[u8] {
        self.data
    }
}

impl Drop for StreamFrame<'_> {
    fn drop(&mut self) {
        SHARED.with(|shared| {
            if shared.slots[self.slot] == Slot::App {
                shared.slots[self.slot] = Slot::Free;
            }
        });
    }
}

/// The capture, the buffers and the reason, returned when
/// [`CsiCapture::into_stream`] fails.
pub type IntoStreamError<'d, Dm> = (
    CsiCapture<'d, Dm>,
    [&'static mut [u8]; STREAM_BUFFER_COUNT],
    StreamStartError,
);

impl<'d, Dm: DriverMode> CsiStream<'d, Dm> {
    pub(super) fn new(
        capture: CsiCapture<'d, Dm>,
        buffers: [&'static mut [u8]; STREAM_BUFFER_COUNT],
    ) -> Result<Self, IntoStreamError<'d, Dm>> {
        let mut stream = Self { capture, buffers };
        match stream.start() {
            Ok(()) => Ok(stream),
            Err(error) => {
                let (capture, buffers) = stream.into_parts();
                Err((capture, buffers, error))
            }
        }
    }

    fn start(&mut self) -> Result<(), StreamStartError> {
        let frame_bytes = self.capture.config.frame_bytes;
        let smallest = self.buffers.iter().map(|b| b.len()).min().unwrap_or(0);
        if smallest < frame_bytes {
            return Err(StreamStartError::BufferTooSmall {
                required: frame_bytes,
                provided: smallest,
            });
        }
        let [a, b] = &self.buffers;
        let (a_start, b_start) = (a.as_ptr() as usize, b.as_ptr() as usize);
        if a_start < b_start + b.len() && b_start < a_start + a.len() {
            return Err(StreamStartError::BuffersOverlap);
        }
        let mut addrs = [0; STREAM_BUFFER_COUNT];
        for (buffer, addr) in self.buffers.iter_mut().zip(addrs.iter_mut()) {
            #[allow(unused_mut, reason = "only written back on cached targets")]
            let mut aligned = crate::dma::aligned::DmaAlignedMut::new(&mut buffer[..frame_bytes])
                .map_err(StreamStartError::InvalidBuffer)?;
            // No dirty line may be evicted over the frames the DMA writes.
            #[cfg(any(soc_internal_memory_cached, dma_can_access_psram))]
            aligned.writeback();
            *addr = aligned.as_mut_ptr() as u32;
        }

        let lens = [self.buffers[0].len() as u32, self.buffers[1].len() as u32];
        let channel = self.capture.dma_channel;
        let block = self.capture.block_transfer(0);
        let started = SHARED.with(|shared| {
            if shared.active || !channel_interrupt_available(channel) {
                return false;
            }
            *shared = Shared {
                active: true,
                channel: Some(channel),
                addrs,
                lens,
                block: Some(block),
                frame_bytes: frame_bytes as u32,
                ..Shared::IDLE
            };
            flush_bridge_fifo();
            self.capture.clear_transfer_status();
            register_channel_handler(channel, stream_interrupt_handler);
            channel.set_interrupt_signal_enable(DMA_COMPLETION_INTERRUPT_MASK);
            interrupt::bind_handler(Interrupt::CSI_BRIDGE, stream_bridge_interrupt_handler);
            shared.arm(channel, 0);
            enable_bridge(true);
            true
        });
        if started {
            Ok(())
        } else {
            Err(StreamStartError::InterruptUnavailable)
        }
    }

    /// Returns the newest completed frame, waiting up to `timeout` for one.
    ///
    /// Restarts the bridge first if the previous frame failed.
    ///
    /// # Errors
    ///
    /// [`StreamError::Timeout`] if no frame completed in time,
    /// [`StreamError::DmaFault`] if the stream stopped on a DMA error.
    pub fn next_frame(&mut self, timeout: Duration) -> Result<StreamFrame<'_>, StreamError> {
        let channel = self.capture.dma_channel;
        let deadline = Instant::now() + timeout;
        loop {
            if let Some((slot, sequence)) = SHARED.with(|shared| take_newest(shared, channel))? {
                return Ok(self.frame(slot, sequence));
            }
            if Instant::now() > deadline {
                return Err(StreamError::Timeout);
            }
        }
    }

    /// Hands out `slot`, which [`take_newest`] moved to [`Slot::App`].
    fn frame(&mut self, slot: usize, sequence: u32) -> StreamFrame<'_> {
        let frame_bytes = self.capture.config.frame_bytes;
        // The DMA finished this buffer and is not armed on it.
        #[cfg(any(soc_internal_memory_cached, dma_can_access_psram))]
        if let Ok(aligned) = DmaAlignedMut::new(&mut self.buffers[slot][..frame_bytes]) {
            aligned.invalidate();
        }
        StreamFrame {
            data: &self.buffers[slot][..frame_bytes],
            slot,
            sequence,
        }
    }

    /// Returns the stream counters.
    pub fn stats(&self) -> StreamStats {
        SHARED.with(|shared| shared.stats)
    }

    /// Stops capturing and returns the capture and the buffers.
    ///
    /// # Panics
    ///
    /// Panics if the DMA channel cannot be shown to have stopped, as
    /// [`CsiCapture::receive`] does, so that the buffers are never handed back
    /// while the DMA may still write them.
    pub fn stop(self) -> (CsiCapture<'d, Dm>, [&'static mut [u8]; STREAM_BUFFER_COUNT]) {
        let mut stream = ManuallyDrop::new(self);
        stream.halt();
        // SAFETY: `stream` is never used or dropped again.
        unsafe { (ptr::read(&stream.capture), ptr::read(&stream.buffers)) }
    }

    fn into_parts(self) -> (CsiCapture<'d, Dm>, [&'static mut [u8]; STREAM_BUFFER_COUNT]) {
        let stream = ManuallyDrop::new(self);
        // SAFETY: `stream` is never used or dropped again, and was not started.
        unsafe { (ptr::read(&stream.capture), ptr::read(&stream.buffers)) }
    }

    fn halt(&mut self) {
        let channel = self.capture.dma_channel;
        // No interrupt handler run arms the channel after this.
        SHARED.with(|shared| shared.active = false);
        if !self.capture.stop_transfer() && (channel.is_enabled() || channel.abort_in_progress()) {
            panic!("mipi-csi stream DMA shutdown failed");
        }
        MIPI_CSI_BRIDGE::regs().int_ena().reset();
        for core in Cpu::all() {
            interrupt::disable(core, Interrupt::CSI_BRIDGE);
        }
        channel.set_interrupt_signal_enable(0);
        unregister_channel_handler(channel);
        self.capture.clear_transfer_status();
        SHARED.with(|shared| *shared = Shared::IDLE);
    }
}

impl<'d> CsiStream<'d, Async> {
    /// Returns the newest completed frame, waiting for the DMA interrupt until
    /// one completes.
    ///
    /// Restarts the bridge first if the previous frame failed. Dropping the
    /// future before it completes leaves the stream unchanged. Use a timer
    /// future to bound the wait.
    ///
    /// # Errors
    ///
    /// [`StreamError::DmaFault`] if the stream stopped on a DMA error.
    pub async fn next_frame_async(&mut self) -> Result<StreamFrame<'_>, StreamError> {
        let channel = self.capture.dma_channel;
        let (slot, sequence) = poll_fn(|cx| {
            // Registered before the check, so a frame that completes after
            // the check wakes this task.
            STREAM_WAKER.register(cx.waker());
            match SHARED.with(|shared| take_newest(shared, channel)) {
                Ok(Some(taken)) => Poll::Ready(Ok(taken)),
                Ok(None) => Poll::Pending,
                Err(error) => Poll::Ready(Err(error)),
            }
        })
        .await?;
        Ok(self.frame(slot, sequence))
    }
}

impl<Dm: DriverMode> Drop for CsiStream<'_, Dm> {
    fn drop(&mut self) {
        self.halt();
    }
}

/// Moves the newest completed frame to [`Slot::App`] and returns its slot and
/// sequence number, after restarting the bridge if the previous frame failed.
fn take_newest(
    shared: &mut Shared,
    channel: DwGdmaChannel,
) -> Result<Option<(usize, u32)>, StreamError> {
    if shared.fault {
        return Err(StreamError::DmaFault);
    }
    if shared.paused {
        resync(shared, channel);
    }
    let ready = shared
        .slots
        .iter()
        .enumerate()
        .filter_map(|(slot, state)| match state {
            Slot::Ready(sequence) => Some((slot, *sequence)),
            _ => None,
        })
        .max_by_key(|(_, sequence)| *sequence);
    if let Some((slot, _)) = ready {
        shared.slots[slot] = Slot::App;
        shared.stats.delivered += 1;
    }
    Ok(ready)
}

/// Switches the bridge on or off, with its discard interrupt.
fn enable_bridge(enable: bool) {
    let bridge = MIPI_CSI_BRIDGE::regs();
    bridge.int_ena().write(|w| w.discard().bit(enable));
    bridge.csi_en().modify(|_, w| w.csi_brig_en().bit(enable));
}

/// Rearms the bridge and the channel after a failed frame. The bridge is off,
/// so its FIFO can be flushed and its event bits cleared, and it waits for the
/// next frame start.
fn resync(shared: &mut Shared, channel: DwGdmaChannel) {
    let Some(slot) = shared.slots.iter().position(|slot| *slot == Slot::Free) else {
        return;
    };
    flush_bridge_fifo();
    super::clear_transfer_status(channel);
    shared.arm(channel, slot);
    enable_bridge(true);
    shared.paused = false;
    shared.stats.resyncs += 1;
}

/// Ends the frame of the slot in `Dma` and arms the channel on the next one.
///
/// Called by the shared DW-GDMA dispatcher only for the capture channel.
#[crate::ram]
fn stream_interrupt_handler(channel: DwGdmaChannel, pending: u32) {
    update_on_interrupt(channel, pending);
    STREAM_WAKER.wake();
}

/// Drops the frame of the slot in `Dma` after the bridge discarded data.
///
/// If the source stops in the middle of a frame, the bridge can stop
/// requesting DMA bursts when the next frame starts. It then fills its FIFO and
/// discards, and the channel never ends its block, so no DMA interrupt comes.
#[crate::ram]
#[crate::handler]
fn stream_bridge_interrupt_handler() {
    SHARED.with(|shared| {
        // Releases the interrupt line. The next arm unmasks it again.
        MIPI_CSI_BRIDGE::regs().int_ena().reset();
        if !shared.active || shared.fault {
            return;
        }
        let (Some(channel), Some(done)) = (shared.channel, shared.dma_slot()) else {
            return;
        };
        // The bridge must stay enabled until the channel stopped.
        if !stop_dma_channel(channel) {
            shared.fault = true;
            return;
        }
        enable_bridge(false);
        shared.slots[done] = Slot::Free;
        shared.stats.errored += 1;
        shared.paused = true;
    });
    STREAM_WAKER.wake();
}

fn update_on_interrupt(channel: DwGdmaChannel, _pending: u32) {
    SHARED.with(|shared| {
        shared.stats.interrupts += 1;
        let status = channel.interrupt_status();
        if !shared.active {
            // A stop in progress may be waiting for `CH_SUSPENDED`, which
            // does not raise the line.
            channel.clear_interrupts(status & DMA_COMPLETION_INTERRUPT_MASK);
            return;
        }
        channel.clear_interrupts(status);
        // The channel clears `CH_EN` itself when the block ends. One that is
        // still enabled is either still writing after an error or did not
        // stop, and is left to the bounded shutdown.
        if channel.is_enabled() {
            shared.fault = true;
            return;
        }
        let Some(done) = shared.dma_slot() else {
            return;
        };

        let written = channel
            .destination_address()
            .saturating_sub(shared.addrs[done]);
        // The bridge, not `BLOCK_TS`, ends the block, so a longer source frame
        // runs past the buffer. Memory after it is no longer known to be intact.
        if written > shared.lens[done] {
            panic!(
                "mipi-csi stream DMA wrote {} bytes from 0x{:08x} into a {}-byte buffer",
                written, shared.addrs[done], shared.lens[done]
            );
        }
        let csi_error = host_error_status().any_error() || bridge_event_status().any_error();
        let ok = status & (DMA_STATUS_BLOCK_TRANSFER_DONE | DMA_STATUS_DMA_TRANSFER_DONE) != 0
            && status & DMA_STATUS_ERROR_MASK == 0
            && written == shared.frame_bytes
            && !csi_error;
        if !ok {
            shared.slots[done] = Slot::Free;
            shared.stats.errored += 1;
            enable_bridge(false);
            shared.paused = true;
            return;
        }
        shared.slots[done] = Slot::Ready(shared.next_sequence);
        shared.next_sequence = shared.next_sequence.wrapping_add(1);
        shared.stats.completed += 1;

        let other = (done + 1) % STREAM_BUFFER_COUNT;
        let next = match shared.slots[other] {
            Slot::Free => other,
            Slot::Ready(_) => {
                shared.stats.overwritten += 1;
                other
            }
            Slot::App | Slot::Dma => {
                shared.stats.overwritten += 1;
                done
            }
        };
        shared.arm(channel, next);
    });
}
