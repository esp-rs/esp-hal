//! Register helpers for the DW-GDMA controller (`VDMA`).
//!
//! The VDMA uses DW-GDMA link-list mode, which does not fit the GDMA channel
//! traits. The MIPI drivers share the controller, its channels, and the single
//! [`Interrupt::DMA`] line through this module.

use core::{
    ptr,
    sync::atomic::{AtomicBool, AtomicPtr, AtomicU32, Ordering},
};

use crate::{
    interrupt,
    peripherals::{Interrupt, VDMA},
    private::Sealed,
};

const CHANNEL_COUNT: usize = 4;

// `CHEN0`: `CH_EN` bits [3:0], `CH_EN_WE` bits [11:8].
const CHANNEL_ENABLE_WRITE: u32 = 0x0100;
const CHANNEL_ENABLE_SET: u32 = 0x0101;
// `CHEN0`: `CH_SUSP` bits [19:16], `CH_SUSP_WE` bits [27:24].
const CHANNEL_SUSPEND_WRITE: u32 = 0x0100_0000;
const CHANNEL_SUSPEND_SET: u32 = 0x0101_0000;
// `CHEN1`: `CH_ABORT` bits [3:0], `CH_ABORT_WE` bits [11:8].
const CHANNEL_ABORT_SET: u32 = 0x0101;

/// Implemented by VDMA channel singletons (`VDMA_CH0`–`VDMA_CH3`).
///
/// Provides the hardware channel index without exposing the full DMA trait
/// surface, since VDMA uses DW-GDMA linked-list mode which is incompatible
/// with the standard GDMA channel traits.
pub trait VdmaDmaChannel: Sealed {
    /// Zero-based hardware channel index (0–3).
    fn channel_id(&self) -> u8;
}

for_each_dma_channel! {
    ("VDMA", $ch:ident, $num:literal, compatible = [$($compatible:ident),*]) => {
        impl VdmaDmaChannel for crate::peripherals::$ch<'_> {
            #[inline]
            fn channel_id(&self) -> u8 { $num }
        }
    };
}

/// AXI master of a transfer endpoint or of a link-list item (`SMS`, `DMS`, `LMS`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum MasterSelect {
    /// AXI master 1, connected to the peripherals.
    Peripheral = 0,
    /// AXI master 2, connected to memory.
    Memory     = 1,
}

impl MasterSelect {
    #[inline]
    pub(crate) const fn raw(self) -> u32 {
        self as u32
    }
}

/// Address increment mode of a block transfer endpoint.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum AddressIncrementMode {
    Increment,
    Fixed,
}

/// Maximum block transfer size in transfer-width units.
///
/// `BLOCK_TS0` is 22-bit and stores `(transfer_size - 1)`.
pub(crate) const MAX_BLOCK_TRANSFER_SIZE_UNITS: u32 = 0x0040_0000;

/// One endpoint of a block transfer.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct BlockTransferEndpointConfig {
    pub(crate) addr: u32,
    pub(crate) master: MasterSelect,
    pub(crate) addr_inc_mode: AddressIncrementMode,
    /// `CTL0.*_MSIZE`.
    pub(crate) burst_size: u8,
    /// `CTL1.ARLEN` or `CTL1.AWLEN`.
    pub(crate) axi_burst_len: u8,
    /// `CTL0.*_TR_WIDTH`.
    pub(crate) width: u8,
}

/// A single block transfer, programmed through the channel registers.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct BlockTransferConfig {
    pub(crate) src: BlockTransferEndpointConfig,
    pub(crate) dst: BlockTransferEndpointConfig,
    /// Transfer size in transfer-width units.
    pub(crate) transfer_size_in_units: u32,
    pub(crate) interrupt_on_block_done: bool,
}

/// Channel configuration. Both ends use the hardware handshake.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct ChannelConfig {
    /// `CFG0.SRC_MULTBLK_TYPE`.
    pub(crate) src_multblk_type: u8,
    /// `CFG0.DST_MULTBLK_TYPE`.
    pub(crate) dst_multblk_type: u8,
    /// `CFG1.TT_FC`: transfer type and flow controller.
    pub(crate) transfer_type_flow_control: u8,
    /// `CFG1.SRC_PER`: hardware handshake interface of the source.
    pub(crate) src_peripheral: u8,
    /// `CFG1.DST_PER`: hardware handshake interface of the destination.
    pub(crate) dst_peripheral: u8,
    /// `SSTATAR`: address that the channel reads the source status from, or 0.
    pub(crate) src_status_fetch_addr: u32,
    /// `DSTATAR`: address that the channel reads the destination status from, or 0.
    pub(crate) dst_status_fetch_addr: u32,
    /// `CFG1.CH_PRIOR`.
    pub(crate) priority: u8,
    /// `CFG1.SRC_OSR_LMT`: outstanding source requests, minus one.
    pub(crate) src_outstanding_limit: u8,
    /// `CFG1.DST_OSR_LMT`: outstanding destination requests, minus one.
    pub(crate) dst_outstanding_limit: u8,
}

/// Enables the DW-GDMA controller without resetting it.
///
/// Other channels can be in use. The `Vdma` peripheral guard resets the
/// controller when its first user enables it.
#[inline]
pub(crate) fn enable_controller() {
    VDMA::regs().cfg0().modify(|_, w| {
        w.dmac_en().set_bit();
        w.int_en().set_bit()
    });
}

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

    /// Programs the transfer type, handshake, and request limits of the channel.
    #[inline]
    pub(crate) fn configure(&self, config: ChannelConfig) {
        let ch = VDMA::regs().ch(self.channel_id as usize);

        ch.cfg0().write(|w| unsafe {
            w.ch1_src_multblk_type().bits(config.src_multblk_type);
            w.ch1_dst_multblk_type().bits(config.dst_multblk_type)
        });

        ch.cfg1().write(|w| unsafe {
            w.ch1_tt_fc().bits(config.transfer_type_flow_control);
            w.ch1_hs_sel_src().clear_bit();
            w.ch1_hs_sel_dst().clear_bit();
            w.ch1_src_per().bits(config.src_peripheral);
            w.ch1_dst_per().bits(config.dst_peripheral);
            w.ch1_ch_prior().bits(config.priority);
            w.ch1_src_osr_lmt().bits(config.src_outstanding_limit);
            w.ch1_dst_osr_lmt().bits(config.dst_outstanding_limit)
        });

        unsafe {
            ch.sstatar0()
                .write(|w| w.bits(config.src_status_fetch_addr));
            ch.dstatar0()
                .write(|w| w.bits(config.dst_status_fetch_addr));
        }
    }

    /// Programs a single block transfer through the channel registers.
    #[inline]
    pub(crate) fn configure_block_transfer(&self, config: BlockTransferConfig) {
        debug_assert!(config.transfer_size_in_units > 0);
        debug_assert!(config.transfer_size_in_units <= MAX_BLOCK_TRANSFER_SIZE_UNITS);

        let ch = VDMA::regs().ch(self.channel_id as usize);
        ch.sar0()
            .write(|w| unsafe { w.ch1_sar0().bits(config.src.addr) });
        unsafe { ch.sar1().write_with_zero(|w| w) };
        ch.dar0()
            .write(|w| unsafe { w.ch1_dar0().bits(config.dst.addr) });
        unsafe { ch.dar1().write_with_zero(|w| w) };
        ch.block_ts0()
            .write(|w| unsafe { w.ch1_block_ts().bits(config.transfer_size_in_units - 1) });

        ch.ctl0().write(|w| unsafe {
            w.ch1_sms().bit(config.src.master == MasterSelect::Memory);
            w.ch1_dms().bit(config.dst.master == MasterSelect::Memory);
            w.ch1_sinc()
                .bit(config.src.addr_inc_mode == AddressIncrementMode::Fixed);
            w.ch1_dinc()
                .bit(config.dst.addr_inc_mode == AddressIncrementMode::Fixed);
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
            // A single block: no shadow-register or link-list block markers.
            w.ch1_shadowreg_or_lli_last().clear_bit();
            w.ch1_shadowreg_or_lli_valid().clear_bit()
        });
    }

    /// Points the channel at the link-list item at `llp_addr` and enables it.
    #[inline]
    pub(crate) fn start_link_list_transfer(&self, llp_addr: u32, lms: MasterSelect) {
        debug_assert_eq!(llp_addr & 0x3F, 0, "LLI must be 64-byte aligned");

        let ch = VDMA::regs().ch(self.channel_id as usize);
        ch.llp0().write(|w| unsafe {
            w.ch1_lms()
                .bit(lms == MasterSelect::Memory)
                .ch1_loc0()
                .bits(llp_addr >> 6)
        });
        unsafe { ch.llp1().write_with_zero(|w| w) };

        self.set_enabled(true);
    }

    /// Enables or disables the channel.
    #[inline]
    pub(crate) fn set_enabled(&self, enabled: bool) {
        let value = if enabled {
            CHANNEL_ENABLE_SET
        } else {
            CHANNEL_ENABLE_WRITE
        };
        unsafe {
            VDMA::regs()
                .chen0()
                .write(|w| w.bits(value << self.channel_id))
        };
    }

    /// Returns whether the channel is enabled.
    #[inline]
    pub(crate) fn is_enabled(&self) -> bool {
        VDMA::regs().chen0().read().bits() & (1 << self.channel_id) != 0
    }

    /// Suspends or resumes the channel.
    #[inline]
    pub(crate) fn set_suspended(&self, suspended: bool) {
        let value = if suspended {
            CHANNEL_SUSPEND_SET
        } else {
            CHANNEL_SUSPEND_WRITE
        };
        unsafe {
            VDMA::regs()
                .chen0()
                .write(|w| w.bits(value << self.channel_id))
        };
    }

    /// Requests a channel abort, to recover from a transfer that does not complete.
    #[inline]
    pub(crate) fn trigger_abort(&self) {
        unsafe {
            VDMA::regs()
                .chen1()
                .write(|w| w.bits(CHANNEL_ABORT_SET << self.channel_id))
        };
    }

    /// Returns whether an abort of the channel is still in progress.
    #[inline]
    pub(crate) fn abort_in_progress(&self) -> bool {
        VDMA::regs().chen1().read().bits() & (1 << self.channel_id) != 0
    }

    /// Sets the channel interrupt-status enable bitmap.
    ///
    /// The register ignores writes while the channel is enabled.
    #[inline]
    pub(crate) fn set_interrupt_status_enable(&self, mask: u32) {
        let ch = VDMA::regs().ch(self.channel_id as usize);
        unsafe { ch.intstatus_enable0().write(|w| w.bits(mask)) };
    }

    /// Sets the channel interrupt-signal enable bitmap.
    ///
    /// The register ignores writes while the channel is enabled.
    #[inline]
    pub(crate) fn set_interrupt_signal_enable(&self, mask: u32) {
        let ch = VDMA::regs().ch(self.channel_id as usize);
        unsafe { ch.intsignal_enable0().write(|w| w.bits(mask)) };
    }

    /// Returns the raw interrupt status bitmap of the channel.
    #[inline]
    pub(crate) fn interrupt_status(&self) -> u32 {
        VDMA::regs()
            .ch(self.channel_id as usize)
            .intstatus0()
            .read()
            .bits()
    }

    /// Clears the interrupt status bits in `mask`.
    #[inline]
    pub(crate) fn clear_interrupts(&self, mask: u32) {
        let ch = VDMA::regs().ch(self.channel_id as usize);
        unsafe { ch.intclear0().write(|w| w.bits(mask)) };
    }

    /// Clears all interrupt status bits of the channel.
    #[inline]
    pub(crate) fn clear_all_interrupts(&self) {
        self.clear_interrupts(u32::MAX);
    }

    /// Returns the address that the channel writes next (`DAR`).
    #[inline]
    pub(crate) fn destination_address(&self) -> u32 {
        VDMA::regs()
            .ch(self.channel_id as usize)
            .dar0()
            .read()
            .bits()
    }

    /// Returns the address that the channel reads next (`SAR`).
    #[inline]
    pub(crate) fn source_address(&self) -> u32 {
        VDMA::regs()
            .ch(self.channel_id as usize)
            .sar0()
            .read()
            .bits()
    }
}
