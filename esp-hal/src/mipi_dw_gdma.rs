//! Shared DW-GDMA helpers for MIPI peripherals on ESP32-P4.
//!
//! This module intentionally stays small and register-focused so MIPI-DSI and
//! MIPI-CSI can share controller/channel programming without introducing
//! public abstractions.

use core::{
    ptr,
    sync::atomic::{AtomicBool, AtomicPtr, AtomicU32, Ordering},
};

use crate::{
    interrupt,
    peripherals::{Interrupt, VDMA},
};

const CHANNEL_ENABLE_WRITE: u32 = 0x0100;
const CHANNEL_ENABLE_SET: u32 = 0x0101;
const CHANNEL_SUSPEND_WRITE: u32 = 0x0100_0000;
const CHANNEL_SUSPEND_SET: u32 = 0x0101_0000;
const CHANNEL_ABORT_SET: u32 = 0x0101;

/// Link-list master select for LLP pointers.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[allow(
    dead_code,
    reason = "Variants are used by DSI now and CSI in follow-up work"
)]
pub(crate) enum LlPointerMasterSelect {
    Peripheral = 0,
    Memory     = 1,
}

impl LlPointerMasterSelect {
    #[inline]
    pub(crate) const fn bit(self) -> bool {
        matches!(self, Self::Memory)
    }

    #[inline]
    pub(crate) const fn raw(self) -> u32 {
        self as u32
    }
}

/// Handshake source selector.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[allow(
    dead_code,
    reason = "Variants are used by DSI now and CSI in follow-up work"
)]
pub(crate) enum HandshakeSelect {
    /// Use the hardware handshake line.
    Hardware,
    /// Use software handshake.
    Software,
}

impl HandshakeSelect {
    #[inline]
    const fn bit(self) -> bool {
        matches!(self, Self::Software)
    }
}

/// Address increment mode for block transfers.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum AddressIncrementMode {
    Increment,
    Fixed,
}

impl AddressIncrementMode {
    #[inline]
    const fn bit(self) -> bool {
        matches!(self, Self::Fixed)
    }
}

/// Maximum block transfer size in transfer-width units.
///
/// `BLOCK_TS0` is 22-bit and stores `(transfer_size - 1)`.
pub(crate) const MAX_BLOCK_TRANSFER_SIZE_UNITS: u32 = 0x0040_0000;

/// One endpoint of a DW-GDMA block transfer.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct BlockTransferEndpointConfig {
    pub(crate) addr: u32,
    pub(crate) ll_pointer_master: LlPointerMasterSelect,
    pub(crate) addr_inc_mode: AddressIncrementMode,
    pub(crate) burst_size: u8,
    pub(crate) axi_burst_len: u8,
    pub(crate) width: u8,
}

/// Block transfer configuration in shadow-register mode.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct BlockTransferConfig {
    pub(crate) src: BlockTransferEndpointConfig,
    pub(crate) dst: BlockTransferEndpointConfig,
    /// Transfer size in transfer-width units.
    pub(crate) transfer_size_in_units: u32,
    pub(crate) interrupt_on_block_done: bool,
}

/// Per-channel DW-GDMA configuration.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct ChannelConfig {
    pub(crate) src_multblk_type: u8,
    pub(crate) dst_multblk_type: u8,
    pub(crate) transfer_type_flow_control: u8,
    pub(crate) src_handshake: HandshakeSelect,
    pub(crate) dst_handshake: HandshakeSelect,
    pub(crate) src_peripheral: u8,
    pub(crate) dst_peripheral: u8,
    pub(crate) src_status_fetch_addr: u32,
    pub(crate) dst_status_fetch_addr: u32,
    pub(crate) priority: u8,
    pub(crate) src_outstanding_limit: u8,
    pub(crate) dst_outstanding_limit: u8,
}

/// Enables the DW-GDMA controller without resetting it.
///
/// Peripheral-wide reset/clock ownership is handled by
/// `GenericPeripheralGuard<{ Peripheral::Vdma as u8 }>` at the driver level.
#[inline]
pub(crate) fn enable_controller() {
    VDMA::regs().cfg0().modify(|_, w| {
        w.dmac_en().set_bit();
        w.int_en().set_bit()
    });
}

const CHANNEL_COUNT: usize = 4;
/// Channel interrupt handler, called from the shared DW-GDMA interrupt with the
/// channel's pending sources (`INTSTATUS & INTSIGNAL_ENABLE`, never 0).
///
/// The handler must clear or mask its pending sources before returning, and
/// must only touch its own channel. It runs in interrupt context on the core
/// that installed the dispatcher, so it must be in RAM and must not block.
pub(crate) type ChannelInterruptHandler = fn(channel: DwGdmaChannel, pending: u32);

static CHANNEL_HANDLERS: [AtomicPtr<()>; CHANNEL_COUNT] =
    [const { AtomicPtr::new(ptr::null_mut()) }; CHANNEL_COUNT];
/// Number of dispatcher runs currently between reading a channel's pending
/// sources and finishing with them. A count rather than a flag, so that it
/// stays correct if [`Interrupt::DMA`] is ever serviced on both cores.
static CHANNEL_DISPATCHES_IN_FLIGHT: [AtomicU32; CHANNEL_COUNT] =
    [const { AtomicU32::new(0) }; CHANNEL_COUNT];
static DISPATCHER_INSTALLED: AtomicBool = AtomicBool::new(false);

/// Routes the pending interrupt sources of `channel` to `handler`.
///
/// The caller must own the channel, and must call this before it unmasks any
/// of the channel's sources in `INTSIGNAL_ENABLE`. On first use this binds the
/// dispatcher to [`Interrupt::DMA`] on the current core. The dispatcher then
/// stays bound and mapped there, later registrations never remap it.
pub(crate) fn register_channel_handler(channel: DwGdmaChannel, handler: ChannelInterruptHandler) {
    let previous =
        CHANNEL_HANDLERS[channel.channel_id as usize].swap(handler as *mut (), Ordering::SeqCst);
    debug_assert!(
        previous.is_null(),
        "DW-GDMA channel handler registered twice"
    );

    if DISPATCHER_INSTALLED
        .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
        .is_ok()
    {
        interrupt::bind_handler(Interrupt::DMA, channel_interrupt_dispatcher);
    }
}

/// Removes the handler of `channel` and waits until the dispatcher no longer
/// runs it, on any core.
///
/// The caller must mask the channel's sources in `INTSIGNAL_ENABLE` first.
/// Must not be called from an interrupt handler that can preempt the
/// dispatcher on the same core.
pub(crate) fn unregister_channel_handler(channel: DwGdmaChannel) {
    let index = channel.channel_id as usize;
    CHANNEL_HANDLERS[index].store(ptr::null_mut(), Ordering::SeqCst);
    while CHANNEL_DISPATCHES_IN_FLIGHT[index].load(Ordering::SeqCst) != 0 {
        core::hint::spin_loop();
    }
}

/// Returns whether `channel` can take interrupts through the dispatcher, that
/// is, no handler is registered for it.
///
/// Whether something else rebound [`Interrupt::DMA`] is not checked:
/// `interrupt::bound_handler` reads the vector table non-volatilely and was
/// seen on hardware to return the link-time default after `bind_handler`.
pub(crate) fn channel_interrupt_available(channel: DwGdmaChannel) -> bool {
    CHANNEL_HANDLERS[channel.channel_id as usize]
        .load(Ordering::SeqCst)
        .is_null()
}

/// The only handler bound to [`Interrupt::DMA`].
///
/// Hands each channel's pending sources to that channel's handler. Pending
/// sources of a channel without a handler are masked, so that they cannot keep
/// the shared line asserted, and pending common-register sources (slave
/// interface errors) are cleared. Nothing else of another channel is touched.
#[crate::ram]
#[crate::handler]
fn channel_interrupt_dispatcher() {
    let regs = VDMA::regs();
    for (channel_id, (slot, in_flight)) in CHANNEL_HANDLERS
        .iter()
        .zip(CHANNEL_DISPATCHES_IN_FLIGHT.iter())
        .enumerate()
    {
        // Counted before the registers are read: once `unregister` has seen 0,
        // a later run reads the sources only after the owner masked them.
        in_flight.fetch_add(1, Ordering::SeqCst);
        let ch = regs.ch(channel_id);
        let pending = ch.intstatus0().read().bits() & ch.intsignal_enable0().read().bits();
        if pending != 0 {
            let handler = slot.load(Ordering::SeqCst);
            if handler.is_null() {
                ch.intsignal_enable0()
                    .modify(|r, w| unsafe { w.bits(r.bits() & !pending) });
            } else {
                // SAFETY: non-null slots only ever hold a `ChannelInterruptHandler`.
                let handler =
                    unsafe { core::mem::transmute::<*mut (), ChannelInterruptHandler>(handler) };
                handler(DwGdmaChannel::new(channel_id as u8), pending);
            }
        }
        in_flight.fetch_sub(1, Ordering::SeqCst);
    }

    let common = regs.commonreg_intstatus0().read().bits()
        & regs.commonreg_intsignal_enable0().read().bits();
    if common != 0 {
        unsafe { regs.commonreg_intclear0().write(|w| w.bits(common)) };
    }
}

/// Register-level handle for one DW-GDMA channel.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct DwGdmaChannel {
    channel_id: u8,
}

impl DwGdmaChannel {
    #[inline]
    pub(crate) const fn new(channel_id: u8) -> Self {
        Self { channel_id }
    }

    #[inline]
    pub(crate) const fn channel_id(&self) -> u8 {
        self.channel_id
    }

    /// Programs channel mode/handshake/direction settings.
    #[inline]
    pub(crate) fn configure(&self, config: ChannelConfig) {
        let ch = VDMA::regs().ch(self.channel_id as usize);

        ch.cfg0().write(|w| unsafe {
            w.ch1_src_multblk_type().bits(config.src_multblk_type);
            w.ch1_dst_multblk_type().bits(config.dst_multblk_type)
        });

        ch.cfg1().write(|w| unsafe {
            w.ch1_tt_fc().bits(config.transfer_type_flow_control);
            w.ch1_hs_sel_src().bit(config.src_handshake.bit());
            w.ch1_hs_sel_dst().bit(config.dst_handshake.bit());
            w.ch1_src_per().bits(config.src_peripheral);
            w.ch1_dst_per().bits(config.dst_peripheral);
            w.ch1_ch_prior().bits(config.priority);
            w.ch1_src_osr_lmt().bits(config.src_outstanding_limit);
            w.ch1_dst_osr_lmt().bits(config.dst_outstanding_limit)
        });

        // For source-controlled P2M transfers (e.g. CSI capture), DW-GDMA
        // fetches peripheral status from this aperture.
        unsafe {
            ch.sstatar0()
                .write(|w| w.bits(config.src_status_fetch_addr))
        };
        unsafe {
            ch.dstatar0()
                .write(|w| w.bits(config.dst_status_fetch_addr))
        };
    }

    /// Programs LLP registers for linked-list mode.
    #[inline]
    pub(crate) fn set_link_list_pointer(&self, llp_addr: u32, lms: LlPointerMasterSelect) {
        debug_assert_eq!(llp_addr & 0x3F, 0, "LLI must be 64-byte aligned");

        let ch = VDMA::regs().ch(self.channel_id as usize);
        ch.llp0()
            .write(|w| unsafe { w.ch1_lms().bit(lms.bit()).ch1_loc0().bits(llp_addr >> 6) });
        unsafe { ch.llp1().write_with_zero(|w| w) };
    }

    /// Enables/disables this channel.
    #[inline]
    pub(crate) fn set_enabled(&self, enabled: bool) {
        let shift = self.channel_id as u32;
        let value = if enabled {
            CHANNEL_ENABLE_SET << shift
        } else {
            CHANNEL_ENABLE_WRITE << shift
        };
        unsafe { VDMA::regs().chen0().write(|w| w.bits(value)) };
    }

    /// Suspends or resumes this channel.
    #[inline]
    pub(crate) fn set_suspended(&self, suspended: bool) {
        let shift = self.channel_id as u32;
        let value = if suspended {
            CHANNEL_SUSPEND_SET << shift
        } else {
            CHANNEL_SUSPEND_WRITE << shift
        };
        unsafe { VDMA::regs().chen0().write(|w| w.bits(value)) };
    }

    /// Requests channel abort for hung-transfer recovery.
    #[inline]
    pub(crate) fn trigger_abort(&self) {
        let shift = self.channel_id as u32;
        let value = CHANNEL_ABORT_SET << shift;
        unsafe { VDMA::regs().chen1().write(|w| w.bits(value)) };
    }

    /// Starts linked-list transfer from the given LLP address.
    #[inline]
    pub(crate) fn start_link_list_transfer(&self, llp_addr: u32, lms: LlPointerMasterSelect) {
        self.set_link_list_pointer(llp_addr, lms);
        self.set_enabled(true);
    }

    /// Programs a one-shot block transfer via channel shadow registers.
    #[inline]
    pub(crate) fn configure_block_transfer(&self, config: BlockTransferConfig) {
        let ch = VDMA::regs().ch(self.channel_id as usize);
        let block_ts = config.transfer_size_in_units - 1;

        debug_assert!(config.transfer_size_in_units > 0);
        debug_assert!(config.transfer_size_in_units <= MAX_BLOCK_TRANSFER_SIZE_UNITS);

        ch.sar0()
            .write(|w| unsafe { w.ch1_sar0().bits(config.src.addr) });
        unsafe { ch.sar1().write_with_zero(|w| w) };
        ch.dar0()
            .write(|w| unsafe { w.ch1_dar0().bits(config.dst.addr) });
        unsafe { ch.dar1().write_with_zero(|w| w) };
        ch.block_ts0()
            .write(|w| unsafe { w.ch1_block_ts().bits(block_ts) });

        ch.ctl0().write(|w| unsafe {
            w.ch1_sms().bit(config.src.ll_pointer_master.bit());
            w.ch1_dms().bit(config.dst.ll_pointer_master.bit());
            w.ch1_sinc().bit(config.src.addr_inc_mode.bit());
            w.ch1_dinc().bit(config.dst.addr_inc_mode.bit());
            w.ch1_src_tr_width().bits(config.src.width);
            w.ch1_dst_tr_width().bits(config.dst.width);
            w.ch1_src_msize().bits(config.src.burst_size);
            w.ch1_dst_msize().bits(config.dst.burst_size)
        });

        ch.ctl1().write(|w| unsafe {
            w.ch1_arlen_en().set_bit();
            w.ch1_arlen().bits(config.src.axi_burst_len);
            w.ch1_awlen_en().set_bit();
            w.ch1_awlen().bits(config.dst.axi_burst_len);
            w.ch1_ioc_blktfr().bit(config.interrupt_on_block_done);
            // Contiguous transfer: no shadow-register or link-list block
            // markers. Link-list users set them in the item itself.
            w.ch1_shadowreg_or_lli_last().clear_bit();
            w.ch1_shadowreg_or_lli_valid().clear_bit()
        });
    }

    /// Enables/disables block-transfer-done interrupt generation for this channel.
    #[inline]
    pub(crate) fn set_block_done_interrupt(&self, enabled: bool) {
        let ch = VDMA::regs().ch(self.channel_id as usize);
        ch.intstatus_enable0()
            .modify(|_, w| w.ch1_enable_block_tfr_done_intstat().bit(enabled));
        ch.intsignal_enable0()
            .modify(|_, w| w.ch1_enable_block_tfr_done_intsignal().bit(enabled));
    }

    /// Sets the channel interrupt-status enable bitmap.
    #[inline]
    pub(crate) fn set_interrupt_status_enable(&self, mask: u32) {
        let ch = VDMA::regs().ch(self.channel_id as usize);
        unsafe { ch.intstatus_enable0().write(|w| w.bits(mask)) };
    }

    /// Sets the channel interrupt-signal enable bitmap.
    #[inline]
    pub(crate) fn set_interrupt_signal_enable(&self, mask: u32) {
        let ch = VDMA::regs().ch(self.channel_id as usize);
        unsafe { ch.intsignal_enable0().write(|w| w.bits(mask)) };
    }

    /// Returns the raw channel interrupt status bitmap.
    #[inline]
    pub(crate) fn interrupt_status(&self) -> u32 {
        VDMA::regs()
            .ch(self.channel_id as usize)
            .intstatus0()
            .read()
            .bits()
    }

    /// Returns the current source address (`SARx`).
    #[inline]
    pub(crate) fn source_address(&self) -> u32 {
        VDMA::regs()
            .ch(self.channel_id as usize)
            .sar0()
            .read()
            .bits()
    }

    /// Returns the current destination address (`DARx`).
    #[inline]
    pub(crate) fn destination_address(&self) -> u32 {
        VDMA::regs()
            .ch(self.channel_id as usize)
            .dar0()
            .read()
            .bits()
    }

    /// Returns whether this DMA channel is currently enabled.
    #[inline]
    pub(crate) fn is_enabled(&self) -> bool {
        ((VDMA::regs().chen0().read().bits() >> self.channel_id) & 1) != 0
    }

    /// Returns whether abort is still in progress for this channel.
    #[inline]
    pub(crate) fn abort_in_progress(&self) -> bool {
        let shift = self.channel_id as u32;
        (VDMA::regs().chen1().read().bits() & (CHANNEL_ABORT_SET << shift)) != 0
    }

    /// Clears raw interrupt status bits for this channel.
    #[inline]
    pub(crate) fn clear_interrupts(&self, mask: u32) {
        let ch = VDMA::regs().ch(self.channel_id as usize);
        unsafe { ch.intclear0().write(|w| w.bits(mask)) };
    }

    /// Clears all interrupt status bits for this channel.
    #[inline]
    pub(crate) fn clear_all_interrupts(&self) {
        self.clear_interrupts(u32::MAX);
    }
}
