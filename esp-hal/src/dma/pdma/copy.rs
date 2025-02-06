use portable_atomic::{AtomicBool, Ordering};

use crate::{
    asynch::AtomicWaker,
    dma::*,
    interrupt::{InterruptHandler, Priority},
    peripherals::{Interrupt, COPY_DMA},
};

pub(super) type CopyRegisterBlock = crate::pac::copy_dma::RegisterBlock;

/// The RX half of a Copy DMA channel.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CopyDmaRxChannel<'d>(pub(crate) CopyDmaChannel<'d>);

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
pub struct CopyDmaTxChannel<'d>(pub(crate) CopyDmaChannel<'d>);

impl CopyDmaTxChannel<'_> {
    fn regs(&self) -> &CopyRegisterBlock {
        self.0.register_block()
    }
}

impl crate::private::Sealed for CopyDmaTxChannel<'_> {}
impl DmaTxChannel for CopyDmaTxChannel<'_> {}

impl RegisterAccess for CopyDmaTxChannel<'_> {
    fn reset(&self) {
        self.regs().conf().modify(|_, w| w.out_rst().set_bit());
        self.regs().conf().modify(|_, w| w.out_rst().clear_bit());
    }

    fn set_burst_mode(&self, _burst_mode: BurstConfig) {}

    fn set_descr_burst_mode(&self, _burst_mode: bool) {}

    fn set_peripheral(&self, _peripheral: u8) {
        // no-op
    }

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

    fn is_compatible_with(&self, peripheral: DmaPeripheral) -> bool {
        self.0.is_compatible_with(peripheral)
    }

    #[cfg(psram_dma)]
    fn set_ext_mem_block_size(&self, _size: DmaExtMemBKSize) {
        // not supported
    }

    #[cfg(psram_dma)]
    fn can_access_psram(&self) -> bool {
        false
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
        self.0.tx_waker()
    }

    fn is_async(&self) -> bool {
        self.0.tx_async_flag().load(Ordering::Acquire)
    }

    fn set_async(&self, is_async: bool) {
        self.0.tx_async_flag().store(is_async, Ordering::Release);
    }
}

impl RegisterAccess for CopyDmaRxChannel<'_> {
    fn reset(&self) {
        self.regs().conf().modify(|_, w| w.in_rst().set_bit());
        self.regs().conf().modify(|_, w| w.in_rst().clear_bit());
    }

    fn set_burst_mode(&self, _burst_mode: BurstConfig) {}

    fn set_descr_burst_mode(&self, _burst_mode: bool) {}

    fn set_peripheral(&self, _peripheral: u8) {
        // no-op
    }

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

    fn is_compatible_with(&self, peripheral: DmaPeripheral) -> bool {
        self.0.is_compatible_with(peripheral)
    }

    #[cfg(psram_dma)]
    fn set_ext_mem_block_size(&self, _size: DmaExtMemBKSize) {
        // not supported
    }

    #[cfg(psram_dma)]
    fn can_access_psram(&self) -> bool {
        false
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
        if int_ena.in_dscr_err().bit_is_set() {
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
                    DmaRxInterrupt::ErrorEof => unimplemented!(),
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
        self.0.rx_waker()
    }

    fn is_async(&self) -> bool {
        self.0.rx_async_flag().load(Ordering::Relaxed)
    }

    fn set_async(&self, _is_async: bool) {
        self.0.rx_async_flag().store(_is_async, Ordering::Relaxed);
    }
}

#[doc = "DMA channel suitable for COPY"]
#[non_exhaustive]
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CopyDmaChannel<'d> {
    _lifetime: core::marker::PhantomData<&'d mut ()>,
}

impl crate::private::Sealed for CopyDmaChannel<'_> {}

impl CopyDmaChannel<'_> {
    #[doc = r" Unsafely constructs a new DMA channel."]
    #[doc = r""]
    #[doc = r" # Safety"]
    #[doc = r""]
    #[doc = r" The caller must ensure that only a single instance is used."]
    pub unsafe fn steal() -> Self {
        Self {
            _lifetime: core::marker::PhantomData,
        }
    }
}
impl<'d> DmaChannel for CopyDmaChannel<'d> {
    type Rx = CopyDmaRxChannel<'d>;
    type Tx = CopyDmaTxChannel<'d>;
    unsafe fn split_internal(self, _: crate::private::Internal) -> (Self::Rx, Self::Tx) {
        (
            CopyDmaRxChannel(unsafe { Self::steal() }),
            CopyDmaTxChannel(unsafe { Self::steal() }),
        )
    }
}
impl DmaChannelExt for CopyDmaChannel<'_> {
    fn rx_interrupts() -> impl InterruptAccess<DmaRxInterrupt> {
        CopyDmaRxChannel(unsafe { Self::steal() })
    }
    fn tx_interrupts() -> impl InterruptAccess<DmaTxInterrupt> {
        CopyDmaTxChannel(unsafe { Self::steal() })
    }
}
impl PdmaChannel for CopyDmaChannel<'_> {
    type RegisterBlock = CopyRegisterBlock;
    fn register_block(&self) -> &Self::RegisterBlock {
        COPY_DMA::regs()
    }
    fn tx_waker(&self) -> &'static AtomicWaker {
        static WAKER: AtomicWaker = AtomicWaker::new();
        &WAKER
    }
    fn rx_waker(&self) -> &'static AtomicWaker {
        static WAKER: AtomicWaker = AtomicWaker::new();
        &WAKER
    }
    fn is_compatible_with(&self, peripheral: DmaPeripheral) -> bool {
        let compatible_peripherals = [DmaPeripheral::Aes, DmaPeripheral::Sha];
        compatible_peripherals.contains(&peripheral)
    }
    fn peripheral_interrupt(&self) -> Interrupt {
        Interrupt::DMA_COPY
    }
    fn async_handler(&self) -> InterruptHandler {
        pub(crate) extern "C" fn __esp_hal_internal_interrupt_handler() {
            super::asynch::handle_in_interrupt::<CopyDmaChannel<'static>>();
            super::asynch::handle_out_interrupt::<CopyDmaChannel<'static>>();
        }
        #[allow(non_upper_case_globals)]
        pub(crate) static interrupt_handler: InterruptHandler =
            InterruptHandler::new(__esp_hal_internal_interrupt_handler, Priority::max());
        interrupt_handler
    }
    fn rx_async_flag(&self) -> &'static AtomicBool {
        static FLAG: AtomicBool = AtomicBool::new(false);
        &FLAG
    }
    fn tx_async_flag(&self) -> &'static AtomicBool {
        static FLAG: AtomicBool = AtomicBool::new(false);
        &FLAG
    }
}
impl<'d> DmaChannelConvert<CopyDmaRxChannel<'d>> for CopyDmaChannel<'d> {
    fn degrade(self) -> CopyDmaRxChannel<'d> {
        CopyDmaRxChannel(self)
    }
}
impl<'d> DmaChannelConvert<CopyDmaTxChannel<'d>> for CopyDmaChannel<'d> {
    fn degrade(self) -> CopyDmaTxChannel<'d> {
        CopyDmaTxChannel(self)
    }
}
