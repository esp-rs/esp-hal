//! # General Direct Memory Access (AXI-GDMA)
//!
//! AXI-GDMA provides peripheral-to-memory, memory-to-peripheral, and
//! memory-to-memory transfers over the AXI bus. All three channels have
//! separate IN and OUT interrupt lines.

use core::marker::PhantomData;

use crate::{
    RegisterToggle,
    asynch::AtomicWaker,
    dma::*,
    handler,
    interrupt::Priority,
    peripherals::{AXI_GDMA, Interrupt, pac},
    system::{Peripheral, PeripheralGuard},
};

/// Immutable per-channel metadata owned by each `AXI_DMA_CH*` singleton.
pub(crate) struct ChannelInfo {
    pub(crate) channel: u8,
    pub(crate) handler_in: Option<InterruptHandler>,
    pub(crate) handler_out: Option<InterruptHandler>,
    pub(crate) isr_in: Option<Interrupt>,
    pub(crate) isr_out: Option<Interrupt>,
    pub(crate) compatible_peripherals: &'static [u8],
}

/// Mutable per-channel runtime state (wakers).
pub(crate) struct ChannelState {
    pub(crate) tx_waker: AtomicWaker,
    pub(crate) rx_waker: AtomicWaker,
}

/// An arbitrary AXI-GDMA channel.
pub struct AxiGdmaChannel<'d> {
    info: &'static ChannelInfo,
    state: &'static ChannelState,
    _lifetime: PhantomData<&'d mut ()>,
}

impl core::fmt::Debug for AxiGdmaChannel<'_> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("AxiGdmaChannel")
            .field("channel", &self.info.channel)
            .finish()
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for AxiGdmaChannel<'_> {
    fn format(&self, fmt: defmt::Formatter<'_>) {
        defmt::write!(fmt, "AxiGdmaChannel {{ channel: {} }}", self.info.channel)
    }
}

impl AxiGdmaChannel<'_> {
    pub(crate) fn channel_index(&self) -> u8 {
        self.info.channel
    }

    pub(crate) unsafe fn clone_unchecked(&self) -> Self {
        Self {
            info: self.info,
            state: self.state,
            _lifetime: PhantomData,
        }
    }
}

impl crate::private::Sealed for AxiGdmaChannel<'_> {}
impl<'d> DmaChannel for AxiGdmaChannel<'d> {
    type Rx = AxiGdmaRxChannel<'d>;
    type Tx = AxiGdmaTxChannel<'d>;

    unsafe fn split_internal(self, _: crate::private::Internal) -> (Self::Rx, Self::Tx) {
        (
            AxiGdmaRxChannel(unsafe { self.clone_unchecked() }),
            AxiGdmaTxChannel(self),
        )
    }
}

/// An arbitrary AXI-GDMA RX channel.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AxiGdmaRxChannel<'d>(AxiGdmaChannel<'d>);

/// An arbitrary AXI-GDMA TX channel.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AxiGdmaTxChannel<'d>(AxiGdmaChannel<'d>);

impl crate::private::Sealed for AxiGdmaTxChannel<'_> {}
impl DmaTxChannel for AxiGdmaTxChannel<'_> {}

impl crate::private::Sealed for AxiGdmaRxChannel<'_> {}
impl DmaRxChannel for AxiGdmaRxChannel<'_> {}

impl<'d> From<AxiGdmaChannel<'d>> for AxiGdmaRxChannel<'d> {
    fn from(this: AxiGdmaChannel<'d>) -> AxiGdmaRxChannel<'d> {
        AxiGdmaRxChannel(this)
    }
}

impl<'d> From<AxiGdmaChannel<'d>> for AxiGdmaTxChannel<'d> {
    fn from(this: AxiGdmaChannel<'d>) -> AxiGdmaTxChannel<'d> {
        AxiGdmaTxChannel(this)
    }
}

impl AxiGdmaTxChannel<'_> {
    #[inline(always)]
    fn ch(&self) -> &pac::axi_dma::OUT_CH {
        AXI_GDMA::regs().out_ch(self.0.info.channel as usize)
    }
}

impl RegisterAccess for AxiGdmaTxChannel<'_> {
    #[allow(private_interfaces)]
    fn enable(&self) -> Option<PeripheralGuard> {
        Some(PeripheralGuard::new_with(
            Peripheral::AxiGdma,
            init_axi_dma_racey,
        ))
    }

    fn reset(&self) {
        self.ch().out_conf0().toggle(|w, en| w.out_rst().bit(en));
    }

    // AXI-DMA data burst is always enabled; nothing to configure.
    fn set_burst_mode(&self, _burst_mode: BurstConfig) {}

    fn set_descr_burst_mode(&self, burst_mode: bool) {
        self.ch()
            .out_conf0()
            .modify(|_, w| w.outdscr_burst_en().bit(burst_mode));
    }

    fn set_priority(&self, priority: DmaPriority) {
        self.ch()
            .out_pri()
            .write(|w| unsafe { w.tx_pri().bits(priority as u8) });
    }

    fn set_peripheral(&self, peripheral: u8) {
        self.ch()
            .out_peri_sel()
            .write(|w| unsafe { w.peri_out_sel().bits(peripheral) });
    }

    fn set_link_addr(&self, address: u32) {
        trace!("Setting out-link address to 0x{:08X}", address);
        self.ch()
            .out_link2()
            .write(|w| unsafe { w.outlink_addr().bits(address) });
    }

    fn start(&self) {
        self.ch()
            .out_link1()
            .modify(|_, w| w.outlink_start().set_bit());
    }

    fn stop(&self) {
        self.ch()
            .out_link1()
            .modify(|_, w| w.outlink_stop().set_bit());
    }

    fn restart(&self) {
        self.ch()
            .out_link1()
            .modify(|_, w| w.outlink_restart().set_bit());
    }

    fn set_check_owner(&self, check_owner: Option<bool>) {
        self.ch()
            .out_conf1()
            .modify(|_, w| w.out_check_owner().bit(check_owner.unwrap_or(true)));
    }

    #[cfg(dma_can_access_psram)]
    fn can_access_psram(&self) -> bool {
        true
    }

    fn compatible_peripherals(&self) -> &[u8] {
        self.0.info.compatible_peripherals
    }
}

impl TxRegisterAccess for AxiGdmaTxChannel<'_> {
    fn is_fifo_empty(&self) -> bool {
        self.ch()
            .outfifo_status()
            .read()
            .outfifo_l3_empty()
            .bit_is_set()
    }

    fn set_auto_write_back(&self, enable: bool) {
        self.ch()
            .out_conf0()
            .modify(|_, w| w.out_auto_wrback().bit(enable));
    }

    fn last_dscr_address(&self) -> usize {
        self.ch()
            .out_eof_des_addr()
            .read()
            .out_eof_des_addr()
            .bits() as _
    }

    fn async_handler(&self) -> Option<InterruptHandler> {
        self.0.info.handler_out
    }

    fn peripheral_interrupt(&self) -> Option<Interrupt> {
        self.0.info.isr_out
    }
}

impl InterruptAccess<DmaTxInterrupt> for AxiGdmaTxChannel<'_> {
    fn enable_listen(&self, interrupts: EnumSet<DmaTxInterrupt>, enable: bool) {
        self.ch().out_int().ena().modify(|_, w| {
            for interrupt in interrupts {
                match interrupt {
                    DmaTxInterrupt::TotalEof => w.out_total_eof().bit(enable),
                    DmaTxInterrupt::DescriptorError => w.out_dscr_err().bit(enable),
                    DmaTxInterrupt::Eof => w.out_eof().bit(enable),
                    DmaTxInterrupt::Done => w.out_done().bit(enable),
                };
            }
            w
        });
    }

    fn is_listening(&self) -> EnumSet<DmaTxInterrupt> {
        let mut result = EnumSet::new();
        let ena = self.ch().out_int().ena().read();
        if ena.out_total_eof().bit_is_set() {
            result |= DmaTxInterrupt::TotalEof;
        }
        if ena.out_dscr_err().bit_is_set() {
            result |= DmaTxInterrupt::DescriptorError;
        }
        if ena.out_eof().bit_is_set() {
            result |= DmaTxInterrupt::Eof;
        }
        if ena.out_done().bit_is_set() {
            result |= DmaTxInterrupt::Done;
        }
        result
    }

    fn clear(&self, interrupts: impl Into<EnumSet<DmaTxInterrupt>>) {
        self.ch().out_int().clr().write(|w| {
            for interrupt in interrupts.into() {
                match interrupt {
                    DmaTxInterrupt::TotalEof => w.out_total_eof().clear_bit_by_one(),
                    DmaTxInterrupt::DescriptorError => w.out_dscr_err().clear_bit_by_one(),
                    DmaTxInterrupt::Eof => w.out_eof().clear_bit_by_one(),
                    DmaTxInterrupt::Done => w.out_done().clear_bit_by_one(),
                };
            }
            w
        });
    }

    fn pending_interrupts(&self) -> EnumSet<DmaTxInterrupt> {
        let mut result = EnumSet::new();
        let raw = self.ch().out_int().raw().read();
        if raw.out_total_eof().bit_is_set() {
            result |= DmaTxInterrupt::TotalEof;
        }
        if raw.out_dscr_err().bit_is_set() {
            result |= DmaTxInterrupt::DescriptorError;
        }
        if raw.out_eof().bit_is_set() {
            result |= DmaTxInterrupt::Eof;
        }
        if raw.out_done().bit_is_set() {
            result |= DmaTxInterrupt::Done;
        }
        result
    }

    fn waker(&self) -> &'static AtomicWaker {
        &self.0.state.tx_waker
    }

    fn is_async(&self) -> bool {
        true
    }

    fn set_async(&self, _is_async: bool) {}
}

impl AxiGdmaRxChannel<'_> {
    #[inline(always)]
    fn ch(&self) -> &pac::axi_dma::IN_CH {
        AXI_GDMA::regs().in_ch(self.0.info.channel as usize)
    }
}

impl RegisterAccess for AxiGdmaRxChannel<'_> {
    #[allow(private_interfaces)]
    fn enable(&self) -> Option<PeripheralGuard> {
        Some(PeripheralGuard::new_with(
            Peripheral::AxiGdma,
            init_axi_dma_racey,
        ))
    }

    fn reset(&self) {
        self.ch().in_conf0().toggle(|w, en| w.in_rst().bit(en));
    }

    // AXI-DMA data burst is always enabled; nothing to configure.
    fn set_burst_mode(&self, _burst_mode: BurstConfig) {}

    fn set_descr_burst_mode(&self, burst_mode: bool) {
        self.ch()
            .in_conf0()
            .modify(|_, w| w.indscr_burst_en().bit(burst_mode));
    }

    fn set_priority(&self, priority: DmaPriority) {
        self.ch()
            .in_pri()
            .write(|w| unsafe { w.rx_pri().bits(priority as u8) });
    }

    fn set_peripheral(&self, peripheral: u8) {
        self.ch()
            .in_peri_sel()
            .write(|w| unsafe { w.peri_in_sel().bits(peripheral) });
    }

    fn set_link_addr(&self, address: u32) {
        trace!("Setting in-link address to 0x{:08X}", address);
        self.ch()
            .in_link2()
            .write(|w| unsafe { w.inlink_addr().bits(address) });
    }

    fn start(&self) {
        self.ch()
            .in_link1()
            .modify(|_, w| w.inlink_start().set_bit());
    }

    fn stop(&self) {
        self.ch()
            .in_link1()
            .modify(|_, w| w.inlink_stop().set_bit());
    }

    fn restart(&self) {
        self.ch()
            .in_link1()
            .modify(|_, w| w.inlink_restart().set_bit());
    }

    fn set_check_owner(&self, check_owner: Option<bool>) {
        self.ch()
            .in_conf1()
            .modify(|_, w| w.in_check_owner().bit(check_owner.unwrap_or(true)));
    }

    #[cfg(dma_can_access_psram)]
    fn can_access_psram(&self) -> bool {
        true
    }

    fn compatible_peripherals(&self) -> &[u8] {
        self.0.info.compatible_peripherals
    }
}

impl RxRegisterAccess for AxiGdmaRxChannel<'_> {
    #[cfg(dma_supports_mem2mem)]
    fn set_mem2mem_mode(&self, value: bool) {
        self.ch()
            .in_conf0()
            .modify(|_, w| w.mem_trans_en().bit(value));
    }

    fn async_handler(&self) -> Option<InterruptHandler> {
        self.0.info.handler_in
    }

    fn peripheral_interrupt(&self) -> Option<Interrupt> {
        self.0.info.isr_in
    }
}

impl InterruptAccess<DmaRxInterrupt> for AxiGdmaRxChannel<'_> {
    fn enable_listen(&self, interrupts: EnumSet<DmaRxInterrupt>, enable: bool) {
        self.ch().in_int().ena().modify(|_, w| {
            for interrupt in interrupts {
                match interrupt {
                    DmaRxInterrupt::SuccessfulEof => w.in_suc_eof().bit(enable),
                    DmaRxInterrupt::ErrorEof => w.in_err_eof().bit(enable),
                    DmaRxInterrupt::DescriptorError => w.in_dscr_err().bit(enable),
                    DmaRxInterrupt::DescriptorEmpty => w.in_dscr_empty().bit(enable),
                    DmaRxInterrupt::Done => w.in_done().bit(enable),
                };
            }
            w
        });
    }

    fn is_listening(&self) -> EnumSet<DmaRxInterrupt> {
        let mut result = EnumSet::new();
        let ena = self.ch().in_int().ena().read();
        if ena.in_suc_eof().bit_is_set() {
            result |= DmaRxInterrupt::SuccessfulEof;
        }
        if ena.in_err_eof().bit_is_set() {
            result |= DmaRxInterrupt::ErrorEof;
        }
        if ena.in_dscr_err().bit_is_set() {
            result |= DmaRxInterrupt::DescriptorError;
        }
        if ena.in_dscr_empty().bit_is_set() {
            result |= DmaRxInterrupt::DescriptorEmpty;
        }
        if ena.in_done().bit_is_set() {
            result |= DmaRxInterrupt::Done;
        }
        result
    }

    fn clear(&self, interrupts: impl Into<EnumSet<DmaRxInterrupt>>) {
        self.ch().in_int().clr().write(|w| {
            for interrupt in interrupts.into() {
                match interrupt {
                    DmaRxInterrupt::SuccessfulEof => w.in_suc_eof().clear_bit_by_one(),
                    DmaRxInterrupt::ErrorEof => w.in_err_eof().clear_bit_by_one(),
                    DmaRxInterrupt::DescriptorError => w.in_dscr_err().clear_bit_by_one(),
                    DmaRxInterrupt::DescriptorEmpty => w.in_dscr_empty().clear_bit_by_one(),
                    DmaRxInterrupt::Done => w.in_done().clear_bit_by_one(),
                };
            }
            w
        });
    }

    fn pending_interrupts(&self) -> EnumSet<DmaRxInterrupt> {
        let mut result = EnumSet::new();
        let raw = self.ch().in_int().raw().read();
        if raw.in_suc_eof().bit_is_set() {
            result |= DmaRxInterrupt::SuccessfulEof;
        }
        if raw.in_err_eof().bit_is_set() {
            result |= DmaRxInterrupt::ErrorEof;
        }
        if raw.in_dscr_err().bit_is_set() {
            result |= DmaRxInterrupt::DescriptorError;
        }
        if raw.in_dscr_empty().bit_is_set() {
            result |= DmaRxInterrupt::DescriptorEmpty;
        }
        if raw.in_done().bit_is_set() {
            result |= DmaRxInterrupt::Done;
        }
        result
    }

    fn waker(&self) -> &'static AtomicWaker {
        &self.0.state.rx_waker
    }

    fn is_async(&self) -> bool {
        true
    }

    fn set_async(&self, _is_async: bool) {}
}

macro_rules! impl_channel {
    ($ch:ident, $num:literal, $interrupt_in:ident, $interrupt_out:ident, compatible = [$($compatible:ident),*]) => {
        use $crate::peripherals::$ch;
        impl $ch<'_> {
            pub(super) fn info() -> &'static ChannelInfo {
                #[handler(priority = Priority::max())]
                fn interrupt_handler_in() {
                    asynch::handle_in_interrupt::<$ch<'static>>();
                }

                #[handler(priority = Priority::max())]
                fn interrupt_handler_out() {
                    asynch::handle_out_interrupt::<$ch<'static>>();
                }

                static INFO: ChannelInfo = ChannelInfo {
                    channel: $num,
                    handler_in: Some(interrupt_handler_in),
                    handler_out: Some(interrupt_handler_out),
                    isr_in: Some(Interrupt::$interrupt_in),
                    isr_out: Some(Interrupt::$interrupt_out),
                    compatible_peripherals: &[$(crate::dma::DmaPeripheral::$compatible.0),*],
                };
                &INFO
            }

            pub(super) fn state() -> &'static ChannelState {
                static STATE: ChannelState = ChannelState {
                    tx_waker: AtomicWaker::new(),
                    rx_waker: AtomicWaker::new(),
                };
                &STATE
            }
        }

        impl<'d> From<$ch<'d>> for AxiGdmaChannel<'d> {
            fn from(_ch: $ch<'d>) -> AxiGdmaChannel<'d> {
                AxiGdmaChannel {
                    info: $ch::info(),
                    state: $ch::state(),
                    _lifetime: core::marker::PhantomData,
                }
            }
        }
        crate::dma::impl_channel_common!(AxiGdma, $ch);
    };
}

for_each_dma_channel! {
    ("AXI_GDMA", $ch:ident, $num:literal, interrupt_in = $interrupt_in:ident, interrupt_out = $interrupt_out:ident, compatible = [$($compatible:ident),*]) => {
        impl_channel!($ch, $num, $interrupt_in, $interrupt_out, compatible = [$($compatible),*]);
    };
}

fn init_axi_dma_racey() {
    let regs = AXI_GDMA::regs();

    // Reset the AXI master read and write FSMs.
    regs.misc_conf().toggle(|w, en| {
        w.axim_rst_rd_inter().bit(en);
        w.axim_rst_wr_inter().bit(en)
    });

    regs.misc_conf().modify(|_, w| w.clk_en().set_bit());

    // AXI-DMA can access L2MEM, L2ROM, MSPI Flash, MSPI PSRAM.
    regs.intr_mem_start_addr()
        .write(|w| unsafe { w.access_intr_mem_start_addr().bits(0x4FC0_0000) });
    regs.intr_mem_end_addr()
        .write(|w| unsafe { w.access_intr_mem_end_addr().bits(0x4FFC_0000) });
    regs.extr_mem_start_addr()
        .write(|w| unsafe { w.access_extr_mem_start_addr().bits(0x4000_0000) });
    regs.extr_mem_end_addr()
        .write(|w| unsafe { w.access_extr_mem_end_addr().bits(0x4C00_0000) });
}
