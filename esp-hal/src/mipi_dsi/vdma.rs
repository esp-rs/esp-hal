//! Minimal VDMA abstraction for MIPI-DSI video streaming.

use crate::{
    mipi_dw_gdma::{
        ChannelConfig,
        DwGdmaChannel,
        HandshakeSelect,
        LlPointerMasterSelect,
        enable_controller,
    },
    private::Sealed,
    reg_access::VolatileCell,
};

/// Implemented by VDMA channel singletons (`VDMA_CH0`–`VDMA_CH3`).
///
/// Provides the hardware channel index without exposing the full DMA trait
/// surface, since VDMA uses DW-GDMA linked-list mode which is incompatible
/// with the standard GDMA channel traits.
pub trait VdmaDmaChannel: Sealed {
    /// Zero-based hardware channel index (0–3).
    fn channel_id(&self) -> u8;
}

#[allow(non_camel_case_types)]
#[allow(
    dead_code,
    reason = "Singleton token is intentionally held to preserve exclusive ownership"
)]
pub(crate) enum OwnedVdmaChannel<'d> {
    VDMA_CH0(crate::peripherals::VDMA_CH0<'d>),
    VDMA_CH1(crate::peripherals::VDMA_CH1<'d>),
    VDMA_CH2(crate::peripherals::VDMA_CH2<'d>),
    VDMA_CH3(crate::peripherals::VDMA_CH3<'d>),
}

for_each_dma_channel! {
    ("VDMA", $ch:ident, $num:literal, compatible = [$($compatible:ident),*]) => {
        impl VdmaDmaChannel for crate::peripherals::$ch<'_> {
            #[inline]
            fn channel_id(&self) -> u8 {
                $num
            }
        }
    };
}

pub(crate) fn take_owned_vdma_channel<'d>(
    channel: impl VdmaDmaChannel + 'd,
) -> (OwnedVdmaChannel<'d>, u8) {
    let channel_id = channel.channel_id();
    drop(channel);

    // SAFETY: `channel` is consumed by value, so in safe code there is no
    // remaining alias after `drop(channel)`. `VdmaDmaChannel` is sealed and is
    // only implemented for `VDMA_CH0..VDMA_CH3`, so this reconstructs the same
    // singleton token in type-erased form.
    let owned = unsafe {
        match channel_id {
            0 => OwnedVdmaChannel::VDMA_CH0(crate::peripherals::VDMA_CH0::steal()),
            1 => OwnedVdmaChannel::VDMA_CH1(crate::peripherals::VDMA_CH1::steal()),
            2 => OwnedVdmaChannel::VDMA_CH2(crate::peripherals::VDMA_CH2::steal()),
            3 => OwnedVdmaChannel::VDMA_CH3(crate::peripherals::VDMA_CH3::steal()),
            _ => unreachable!("invalid VDMA channel id: {channel_id}"),
        }
    };

    (owned, channel_id)
}

/// Fixed write destination for all DSI DMA transfers (DSI bridge pixel FIFO).
pub(super) const DSI_BRG_MEM_BASE: u32 = 0x5010_5000;

const PORT_MEMORY: u32 = LlPointerMasterSelect::Memory.raw();
const PORT_DSI: u32 = 0;

/// CTL0 value for mem → DSI transfers (constant across all frame buffers):
///   sms=1 (MEMORY), dms=0 (DSI), sinc=0 (increment), dinc=1 (fixed),
///   src/dst transfer width = 64-bit (3),
///   src_msize = 512 items (8), dst_msize = 256 items (7).
const CTRL_LO: u32 = PORT_MEMORY        // bit  0: sms
    | (PORT_DSI  << 2)                  // bit  2: dms
    // | (0      << 4)                  // bit  4: sinc = INCREMENT
    | (1         << 6)                  // bit  6: dinc = FIXED
    | (3         << 8)                  // bits 10:8  src_tr_width  (64-bit)
    | (3         << 11)                 // bits 13:11 dst_tr_width  (64-bit)
    | (8         << 14)                 // bits 17:14 src_msize     (512)
    | (7         << 18); // bits 21:18 dst_msize     (256)

/// CTL1 base: arlen_en=1, arlen=16, awlen_en=1, awlen=16.
const CTRL_HI_BASE: u32 = (1 << 6) | (16 << 7) | (1 << 15) | (16 << 16);

const LLI_VALID: u32 = 1 << 31;

const DSI_CHANNEL_CONFIG: ChannelConfig = ChannelConfig {
    src_multblk_type: 3,
    dst_multblk_type: 3,
    // M→P, DMA as flow controller
    transfer_type_flow_control: 1,
    src_handshake: HandshakeSelect::Hardware,
    dst_handshake: HandshakeSelect::Hardware,
    src_peripheral: 0,
    // DSI bridge handshake peripheral = 0
    dst_peripheral: 0,
    priority: 1,
    // Outstanding: src = 5 (4+1), dst = 2 (1+1)
    src_outstanding_limit: 4,
    dst_outstanding_limit: 1,
};

/// One VDMA link-list item (LLI).
#[repr(C)]
pub(super) struct VdmaLinkItem {
    sar_lo: VolatileCell<u32>,    // 0x00 – source address (low 32 bits)
    sar_hi: VolatileCell<u32>,    // 0x04 – source address (high 32 bits, always 0)
    dar_lo: VolatileCell<u32>,    // 0x08 – destination address (DSI_BRG_MEM_BASE)
    dar_hi: VolatileCell<u32>,    // 0x0C
    block_ts: VolatileCell<u32>,  // 0x10 – transfer size in 64-bit units, minus 1
    _res1: VolatileCell<u32>,     // 0x14
    llp_lo: VolatileCell<u32>,    // 0x18 – next LLI pointer low | LMS
    llp_hi: VolatileCell<u32>,    // 0x1C
    ctrl_lo: VolatileCell<u32>,   // 0x20 – CTL0
    ctrl_hi: VolatileCell<u32>,   // 0x24 – CTL1
    sstat: VolatileCell<u32>,     // 0x28
    dstat: VolatileCell<u32>,     // 0x2C
    status_lo: VolatileCell<u32>, // 0x30
    status_hi: VolatileCell<u32>, // 0x34
    _res2: VolatileCell<u32>,     // 0x38
    _res3: VolatileCell<u32>,     // 0x3C
}

impl VdmaLinkItem {
    pub(super) const fn zeroed() -> Self {
        Self {
            sar_lo: VolatileCell::new(0),
            sar_hi: VolatileCell::new(0),
            dar_lo: VolatileCell::new(0),
            dar_hi: VolatileCell::new(0),
            block_ts: VolatileCell::new(0),
            _res1: VolatileCell::new(0),
            llp_lo: VolatileCell::new(0),
            llp_hi: VolatileCell::new(0),
            ctrl_lo: VolatileCell::new(0),
            ctrl_hi: VolatileCell::new(0),
            sstat: VolatileCell::new(0),
            dstat: VolatileCell::new(0),
            status_lo: VolatileCell::new(0),
            status_hi: VolatileCell::new(0),
            _res2: VolatileCell::new(0),
            _res3: VolatileCell::new(0),
        }
    }

    /// Populate this LLI for a circular frame-buffer transfer.
    ///
    /// `next` must point to the next LLI in the chain.
    /// `LLI_LAST` is deliberately **not** set; the DMA always follows the LLP
    /// pointer to continue the linked-list ring.
    pub(super) fn configure(&self, src_addr: u32, fb_size: usize, next: *const VdmaLinkItem) {
        debug_assert!(fb_size.is_multiple_of(8), "fb_size must be a multiple of 8");
        let block_ts = (fb_size / 8) as u32 - 1;
        let ctrl_hi = CTRL_HI_BASE | LLI_VALID; // no LLI_LAST → loop forever
        self.sar_lo.set(src_addr);
        self.sar_hi.set(0);
        self.dar_lo.set(DSI_BRG_MEM_BASE);
        self.dar_hi.set(0);
        self.block_ts.set(block_ts);
        self._res1.set(0);
        // llp_lo: bits[31:6] = next_addr >> 6, bit[0] = lms (PORT_MEMORY).
        // next is 64-byte aligned so bottom 6 bits are zero.
        self.llp_lo.set(next as u32 | PORT_MEMORY);
        self.llp_hi.set(0);
        self.ctrl_lo.set(CTRL_LO);
        self.ctrl_hi.set(ctrl_hi);
        self.sstat.set(0);
        self.dstat.set(0);
        self.status_lo.set(0);
        self.status_hi.set(0);
        self._res2.set(0);
        self._res3.set(0);
    }

    /// Updates the source address in this LLI.
    ///
    /// Safe to call while the DMA is running: the controller latches `sar_lo`
    /// at the **start** of each block, so an in-flight block is unaffected and
    /// the new address takes effect for the next block.
    pub(super) fn set_source(&self, src_addr: u32) {
        self.sar_lo.set(src_addr);
    }

    /// Re-arm this LLI so the DMA can use it again.
    ///
    /// The DW-GDMA clears `LLI_VALID` in the LLI memory after consuming a
    /// block.  Writing the complete `ctrl_hi` value (including `LLI_VALID`)
    /// lets the DMA resume when it next fetches this LLI.  Call this on the
    /// **just-consumed** LLI from the ping-pong pair while the DMA is busy
    /// with the other one.
    pub(super) fn rearm(&self) {
        self.ctrl_hi.set(CTRL_HI_BASE | LLI_VALID);
    }
}

/// Handles for a single VDMA channel dedicated to the DSI bridge.
pub(super) struct VdmaChannel {
    channel: DwGdmaChannel,
}

impl VdmaChannel {
    /// Initializes the VDMA controller and configures channel `channel_id`
    /// (0-indexed, 0–3) for mem→DSI linked-list transfers.
    ///
    /// The caller must hold the `Vdma` peripheral guard before calling this.
    pub(super) fn new(channel_id: u8) -> Self {
        enable_controller();

        let channel = DwGdmaChannel::new(channel_id);
        channel.configure(DSI_CHANNEL_CONFIG);
        Self { channel }
    }

    /// Creates an unconfigured handle for channel-local operations such as
    /// interrupt acknowledge/disable and stop.
    pub(super) const fn from_channel_id(channel_id: u8) -> Self {
        Self {
            channel: DwGdmaChannel::new(channel_id),
        }
    }

    /// Points the channel's LLP at `item` and enables the channel.
    pub(super) fn start(&self, item: &VdmaLinkItem) {
        let addr = item as *const VdmaLinkItem as u32;
        self.channel
            .start_link_list_transfer(addr, LlPointerMasterSelect::Memory);
    }

    pub(super) fn set_block_done_interrupt(&self, enabled: bool) {
        self.channel.set_block_done_interrupt(enabled);
    }

    pub(super) fn clear_all_interrupts(&self) {
        self.channel.clear_all_interrupts();
    }

    pub(super) fn stop(&self) {
        self.channel.set_enabled(false);
    }
}
