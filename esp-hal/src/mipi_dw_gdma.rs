//! Shared DW-GDMA helpers for MIPI peripherals on ESP32-P4.
//!
//! This module intentionally stays small and register-focused so MIPI-DSI and
//! future MIPI-CSI code can share controller/channel programming without
//! introducing public abstractions.

use crate::peripherals::VDMA;

const CHANNEL_ENABLE_WRITE: u32 = 0x0100;
const CHANNEL_ENABLE_SET: u32 = 0x0101;

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

    /// Starts linked-list transfer from the given LLP address.
    #[inline]
    pub(crate) fn start_link_list_transfer(&self, llp_addr: u32, lms: LlPointerMasterSelect) {
        self.set_link_list_pointer(llp_addr, lms);
        self.set_enabled(true);
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
