use enumset::EnumSet;
use portable_atomic::{AtomicBool, Ordering};

use crate::{
    RegisterToggle,
    asynch::AtomicWaker,
    dma::{
        BurstConfig,
        DmaChannel,
        DmaExtMemBKSize,
        DmaRxChannel,
        DmaRxInterrupt,
        DmaTxChannel,
        DmaTxInterrupt,
        InterruptAccess,
        RegisterAccess,
        RxRegisterAccess,
        TxRegisterAccess,
        asynch,
    },
    interrupt::InterruptHandler,
    peripherals::{DMA_COPY, Interrupt},
    system::{Peripheral, PeripheralGuard},
};

/// Immutable per-channel metadata.
#[doc(hidden)]
pub struct ChannelInfo {
    #[expect(dead_code)]
    pub(crate) peripheral_interrupt: Interrupt,

    #[expect(dead_code)]
    pub(crate) async_handler: InterruptHandler,

    /// Peripheral IDs this channel can serve. An empty slice means no runtime check is needed.
    pub(crate) compatible_peripherals: &'static [u8],
}

/// Mutable per-channel runtime state (wakers and async-mode flags).
pub(crate) struct ChannelState {
    /// Async waker for the TX (out) half of this channel.
    pub(crate) tx_waker: AtomicWaker,

    /// Async waker for the RX (in) half of this channel.
    pub(crate) rx_waker: AtomicWaker,

    /// Whether the TX half is currently in async mode.
    pub(crate) tx_async_flag: portable_atomic::AtomicBool,

    /// Whether the RX half is currently in async mode.
    pub(crate) rx_async_flag: portable_atomic::AtomicBool,
}

pub(super) type CopyRegisterBlock = crate::pac::copy_dma::RegisterBlock;

/// The RX half of a Copy DMA channel.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CopyDmaRxChannel<'d>(CopyDmaChannel<'d>);

impl CopyDmaRxChannel<'_> {
    fn regs(&self) -> &CopyRegisterBlock {
        self.0.register_block()
    }
}

impl crate::private::Sealed for CopyDmaRxChannel<'_> {}
impl DmaRxChannel for CopyDmaRxChannel<'_> {}

/// The TX half of a Copy DMA channel.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CopyDmaTxChannel<'d>(CopyDmaChannel<'d>);

impl CopyDmaTxChannel<'_> {
    fn regs(&self) -> &CopyRegisterBlock {
        self.0.register_block()
    }
}

impl crate::private::Sealed for CopyDmaTxChannel<'_> {}
impl DmaTxChannel for CopyDmaTxChannel<'_> {}

impl RegisterAccess for CopyDmaTxChannel<'_> {
    #[allow(private_interfaces)]
    fn enable(&self) -> Option<PeripheralGuard> {
        Some(PeripheralGuard::new(Peripheral::CopyDma))
    }

    fn reset(&self) {
        self.regs().conf().toggle(|w, bit| w.out_rst().bit(bit));
    }

    fn set_burst_mode(&self, _burst_mode: BurstConfig) {}

    fn set_descr_burst_mode(&self, _burst_mode: bool) {}

    fn set_link_addr(&self, address: u32) {
        self.regs()
            .out_link()
            .modify(|_, w| unsafe { w.outlink_addr().bits(address) });
    }

    fn start(&self) {
        self.regs()
            .out_link()
            .modify(|_, w| w.outlink_start().set_bit());
    }

    fn stop(&self) {
        self.regs()
            .out_link()
            .modify(|_, w| w.outlink_stop().set_bit());
    }

    fn restart(&self) {
        self.regs()
            .out_link()
            .modify(|_, w| w.outlink_restart().set_bit());
    }

    fn set_check_owner(&self, check_owner: Option<bool>) {
        if check_owner == Some(true) {
            panic!("Copy DMA does not support checking descriptor ownership");
        }
    }

    #[cfg(dma_ext_mem_configurable_block_size)]
    fn set_ext_mem_block_size(&self, _size: DmaExtMemBKSize) {
        // not supported
    }

    #[cfg(dma_can_access_psram)]
    fn can_access_psram(&self) -> bool {
        false
    }

    fn compatible_peripherals(&self) -> &[u8] {
        self.0.info().compatible_peripherals
    }
}

impl TxRegisterAccess for CopyDmaTxChannel<'_> {
    fn is_fifo_empty(&self) -> bool {
        self.regs().in_st().read().fifo_empty().bit()
    }

    fn set_auto_write_back(&self, enable: bool) {
        self.regs()
            .conf()
            .modify(|_, w| w.out_auto_wrback().bit(enable));
    }

    fn last_dscr_address(&self) -> usize {
        self.regs()
            .out_eof_des_addr()
            .read()
            .out_eof_des_addr()
            .bits() as usize
    }

    fn peripheral_interrupt(&self) -> Option<Interrupt> {
        None
    }

    fn async_handler(&self) -> Option<InterruptHandler> {
        None
    }
}

impl InterruptAccess<DmaTxInterrupt> for CopyDmaTxChannel<'_> {
    fn enable_listen(&self, interrupts: EnumSet<DmaTxInterrupt>, enable: bool) {
        self.regs().int_ena().modify(|_, w| {
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

        let int_ena = self.regs().int_ena().read();
        if int_ena.out_total_eof().bit_is_set() {
            result |= DmaTxInterrupt::TotalEof;
        }
        if int_ena.out_dscr_err().bit_is_set() {
            result |= DmaTxInterrupt::DescriptorError;
        }
        if int_ena.out_eof().bit_is_set() {
            result |= DmaTxInterrupt::Eof;
        }
        if int_ena.out_done().bit_is_set() {
            result |= DmaTxInterrupt::Done;
        }

        result
    }

    fn clear(&self, interrupts: impl Into<EnumSet<DmaTxInterrupt>>) {
        self.regs().int_clr().write(|w| {
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

        let int_raw = self.regs().int_raw().read();
        if int_raw.out_total_eof().bit_is_set() {
            result |= DmaTxInterrupt::TotalEof;
        }
        if int_raw.out_dscr_err().bit_is_set() {
            result |= DmaTxInterrupt::DescriptorError;
        }
        if int_raw.out_eof().bit_is_set() {
            result |= DmaTxInterrupt::Eof;
        }
        if int_raw.out_done().bit_is_set() {
            result |= DmaTxInterrupt::Done;
        }

        result
    }

    fn waker(&self) -> &'static AtomicWaker {
        &self.0.state().tx_waker
    }

    fn is_async(&self) -> bool {
        self.0.state().tx_async_flag.load(Ordering::Acquire)
    }

    fn set_async(&self, is_async: bool) {
        self.0
            .state()
            .tx_async_flag
            .store(is_async, Ordering::Release);
    }
}

impl RegisterAccess for CopyDmaRxChannel<'_> {
    #[allow(private_interfaces)]
    fn enable(&self) -> Option<PeripheralGuard> {
        Some(PeripheralGuard::new(Peripheral::CopyDma))
    }

    fn reset(&self) {
        self.regs().conf().toggle(|w, bit| w.in_rst().bit(bit));
    }

    fn set_burst_mode(&self, _burst_mode: BurstConfig) {}

    fn set_descr_burst_mode(&self, _burst_mode: bool) {}

    fn set_link_addr(&self, address: u32) {
        self.regs()
            .in_link()
            .modify(|_, w| unsafe { w.inlink_addr().bits(address) });
    }

    fn start(&self) {
        self.regs()
            .in_link()
            .modify(|_, w| w.inlink_start().set_bit());
    }

    fn stop(&self) {
        self.regs()
            .in_link()
            .modify(|_, w| w.inlink_stop().set_bit());
    }

    fn restart(&self) {
        self.regs()
            .in_link()
            .modify(|_, w| w.inlink_restart().set_bit());
    }

    fn set_check_owner(&self, check_owner: Option<bool>) {
        if check_owner == Some(true) {
            panic!("Copy DMA does not support checking descriptor ownership");
        }
    }

    #[cfg(dma_ext_mem_configurable_block_size)]
    fn set_ext_mem_block_size(&self, _size: DmaExtMemBKSize) {
        // not supported
    }

    #[cfg(dma_can_access_psram)]
    fn can_access_psram(&self) -> bool {
        false
    }

    fn compatible_peripherals(&self) -> &[u8] {
        self.0.info().compatible_peripherals
    }
}

impl RxRegisterAccess for CopyDmaRxChannel<'_> {
    fn peripheral_interrupt(&self) -> Option<Interrupt> {
        None
    }

    fn async_handler(&self) -> Option<InterruptHandler> {
        None
    }
}

impl InterruptAccess<DmaRxInterrupt> for CopyDmaRxChannel<'_> {
    fn enable_listen(&self, interrupts: EnumSet<DmaRxInterrupt>, enable: bool) {
        self.regs().int_ena().modify(|_, w| {
            for interrupt in interrupts {
                match interrupt {
                    DmaRxInterrupt::SuccessfulEof => w.in_suc_eof().bit(enable),
                    DmaRxInterrupt::ErrorEof => unimplemented!(),
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

        let int_ena = self.regs().int_ena().read();
        if int_ena.in_dscr_err().bit_is_set() {
            result |= DmaRxInterrupt::DescriptorError;
        }
        if int_ena.in_dscr_empty().bit_is_set() {
            result |= DmaRxInterrupt::DescriptorEmpty;
        }
        if int_ena.in_suc_eof().bit_is_set() {
            result |= DmaRxInterrupt::SuccessfulEof;
        }
        if int_ena.in_done().bit_is_set() {
            result |= DmaRxInterrupt::Done;
        }

        result
    }

    fn clear(&self, interrupts: impl Into<EnumSet<DmaRxInterrupt>>) {
        self.regs().int_clr().write(|w| {
            for interrupt in interrupts.into() {
                match interrupt {
                    DmaRxInterrupt::SuccessfulEof => w.in_suc_eof().clear_bit_by_one(),
                    DmaRxInterrupt::ErrorEof => continue,
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

        let int_raw = self.regs().int_raw().read();
        if int_raw.in_dscr_err().bit_is_set() {
            result |= DmaRxInterrupt::DescriptorError;
        }
        if int_raw.in_dscr_empty().bit_is_set() {
            result |= DmaRxInterrupt::DescriptorEmpty;
        }
        if int_raw.in_suc_eof().bit_is_set() {
            result |= DmaRxInterrupt::SuccessfulEof;
        }
        if int_raw.in_done().bit_is_set() {
            result |= DmaRxInterrupt::Done;
        }

        result
    }

    fn waker(&self) -> &'static AtomicWaker {
        &self.0.state().rx_waker
    }

    fn is_async(&self) -> bool {
        self.0.state().rx_async_flag.load(Ordering::Relaxed)
    }

    fn set_async(&self, is_async: bool) {
        self.0
            .state()
            .rx_async_flag
            .store(is_async, Ordering::Relaxed);
    }
}

/// A type-erased Copy DMA channel.
pub type CopyDmaChannel<'d> = DMA_COPY<'d>;

impl DMA_COPY<'_> {
    pub(super) fn info(&self) -> &'static ChannelInfo {
        #[crate::handler(priority = crate::interrupt::Priority::max())]
        fn interrupt_handler() {
            asynch::handle_in_interrupt::<DMA_COPY<'static>>();
            asynch::handle_out_interrupt::<DMA_COPY<'static>>();
        }
        static INFO: ChannelInfo = ChannelInfo {
            peripheral_interrupt: Interrupt::DMA_COPY,
            async_handler: interrupt_handler,
            compatible_peripherals: &[],
        };
        &INFO
    }
    pub(super) fn state(&self) -> &'static ChannelState {
        static STATE: ChannelState = ChannelState {
            tx_waker: AtomicWaker::new(),
            rx_waker: AtomicWaker::new(),
            tx_async_flag: AtomicBool::new(false),
            rx_async_flag: AtomicBool::new(false),
        };
        &STATE
    }
}

crate::dma::impl_channel_common!(CopyDma, DMA_COPY);
