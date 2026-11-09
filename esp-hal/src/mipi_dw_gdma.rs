//! Shared DW-GDMA helpers for MIPI peripherals on ESP32-P4.
//!
//! This module intentionally stays small and register-focused so MIPI-DSI and
//! future MIPI-CSI code can share controller/channel programming without
//! introducing public abstractions.

use crate::peripherals::VDMA;

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

/// Register-level handle for one DW-GDMA channel.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct DwGdmaChannel {
    channel_id: u8,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct DwGdmaChannelDebugRegisters {
    pub(crate) controller_cfg0: u32,
    pub(crate) controller_reset0: u32,
    pub(crate) controller_lowpower_cfg0: u32,
    pub(crate) controller_lowpower_cfg1: u32,
    pub(crate) chen0: u32,
    pub(crate) chen1: u32,
    pub(crate) cfg0: u32,
    pub(crate) cfg1: u32,
    pub(crate) ctl0: u32,
    pub(crate) ctl1: u32,
    pub(crate) sar0: u32,
    pub(crate) dar0: u32,
    pub(crate) block_ts0: u32,
    pub(crate) sstatar0: u32,
    pub(crate) dstatar0: u32,
    pub(crate) sstat0: u32,
    pub(crate) dstat0: u32,
    pub(crate) status0: u32,
    pub(crate) status1: u32,
    pub(crate) intstatus_enable0: u32,
    pub(crate) intstatus0: u32,
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
            // Keep contiguous transfer programming aligned with IDF:
            // block markers are configured separately, and are not asserted
            // for this one-shot CSI transfer path.
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

    /// Returns completed transfer size in transfer-width units.
    #[inline]
    pub(crate) fn completed_transfer_units(&self) -> u32 {
        VDMA::regs()
            .ch(self.channel_id as usize)
            .status0()
            .read()
            .ch1_cmpltd_blk_tfr_size()
            .bits()
    }

    /// Returns number of transfer-width units still left in the channel FIFO.
    #[inline]
    pub(crate) fn fifo_remaining_units(&self) -> u16 {
        VDMA::regs()
            .ch(self.channel_id as usize)
            .status1()
            .read()
            .ch1_data_left_in_fifo()
            .bits()
    }

    #[inline]
    pub(crate) fn debug_registers(&self) -> DwGdmaChannelDebugRegisters {
        let regs = VDMA::regs();
        let ch = regs.ch(self.channel_id as usize);
        DwGdmaChannelDebugRegisters {
            controller_cfg0: regs.cfg0().read().bits(),
            controller_reset0: regs.reset0().read().bits(),
            controller_lowpower_cfg0: regs.lowpower_cfg0().read().bits(),
            controller_lowpower_cfg1: regs.lowpower_cfg1().read().bits(),
            chen0: regs.chen0().read().bits(),
            chen1: regs.chen1().read().bits(),
            cfg0: ch.cfg0().read().bits(),
            cfg1: ch.cfg1().read().bits(),
            ctl0: ch.ctl0().read().bits(),
            ctl1: ch.ctl1().read().bits(),
            sar0: ch.sar0().read().bits(),
            dar0: ch.dar0().read().bits(),
            block_ts0: ch.block_ts0().read().bits(),
            sstatar0: ch.sstatar0().read().bits(),
            dstatar0: ch.dstatar0().read().bits(),
            sstat0: ch.sstat0().read().bits(),
            dstat0: ch.dstat0().read().bits(),
            status0: ch.status0().read().bits(),
            status1: ch.status1().read().bits(),
            intstatus_enable0: ch.intstatus_enable0().read().bits(),
            intstatus0: ch.intstatus0().read().bits(),
        }
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
