use portable_atomic::{AtomicBool, Ordering};

use crate::{
    asynch::AtomicWaker,
    dma::*,
    interrupt::Priority,
    peripherals::{DMA_CRYPTO, Interrupt},
};

pub(super) type CryptoRegisterBlock = crate::pac::crypto_dma::RegisterBlock;

/// The RX half of a Crypto DMA channel.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CryptoDmaRxChannel<'d>(pub(crate) DMA_CRYPTO<'d>);

impl CryptoDmaRxChannel<'_> {
    fn regs(&self) -> &CryptoRegisterBlock {
        self.0.register_block()
    }
}

impl crate::private::Sealed for CryptoDmaRxChannel<'_> {}
impl DmaRxChannel for CryptoDmaRxChannel<'_> {}

/// The TX half of a Crypto DMA channel.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CryptoDmaTxChannel<'d>(pub(crate) DMA_CRYPTO<'d>);

impl CryptoDmaTxChannel<'_> {
    fn regs(&self) -> &CryptoRegisterBlock {
        self.0.register_block()
    }
}

impl crate::private::Sealed for CryptoDmaTxChannel<'_> {}
impl DmaTxChannel for CryptoDmaTxChannel<'_> {}

impl RegisterAccess for CryptoDmaTxChannel<'_> {
    fn reset(&self) {
        self.regs().conf().modify(|_, w| {
            w.out_rst().set_bit();
            w.ahbm_rst().set_bit();
            w.ahbm_fifo_rst().set_bit()
        });
        self.regs().conf().modify(|_, w| {
            w.out_rst().clear_bit();
            w.ahbm_rst().clear_bit();
            w.ahbm_fifo_rst().clear_bit()
        });
    }

    fn set_burst_mode(&self, burst_mode: BurstConfig) {
        self.regs()
            .conf()
            .modify(|_, w| w.out_data_burst_en().bit(burst_mode.is_burst_enabled()));
    }

    fn set_descr_burst_mode(&self, burst_mode: bool) {
        self.regs()
            .conf()
            .modify(|_, w| w.outdscr_burst_en().bit(burst_mode));
    }

    fn set_peripheral(&self, peripheral: u8) {
        use esp32s2::crypto_dma::aes_sha_select::SELECT;
        let peripheral = match peripheral {
            p if p == DmaPeripheral::Aes as u8 => SELECT::Aes,
            p if p == DmaPeripheral::Sha as u8 => SELECT::Sha,
            _ => unreachable!(),
        };
        self.regs()
            .aes_sha_select()
            .modify(|_, w| w.select().variant(peripheral));
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
            panic!("Crypto DMA does not support checking descriptor ownership");
        }
    }

    fn is_compatible_with(&self, peripheral: DmaPeripheral) -> bool {
        self.0.is_compatible_with(peripheral)
    }

    #[cfg(psram_dma)]
    fn set_ext_mem_block_size(&self, size: DmaExtMemBKSize) {
        self.regs()
            .conf1()
            .modify(|_, w| unsafe { w.ext_mem_bk_size().bits(size as u8) });
    }

    #[cfg(psram_dma)]
    fn can_access_psram(&self) -> bool {
        true
    }
}

impl TxRegisterAccess for CryptoDmaTxChannel<'_> {
    fn is_fifo_empty(&self) -> bool {
        self.regs().state1().read().outfifo_cnt_debug().bits() == 0
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

impl InterruptAccess<DmaTxInterrupt> for CryptoDmaTxChannel<'_> {
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

impl RegisterAccess for CryptoDmaRxChannel<'_> {
    fn reset(&self) {
        self.regs().conf().modify(|_, w| {
            w.in_rst().set_bit();
            w.ahbm_rst().set_bit();
            w.ahbm_fifo_rst().set_bit()
        });
        self.regs().conf().modify(|_, w| {
            w.in_rst().clear_bit();
            w.ahbm_rst().clear_bit();
            w.ahbm_fifo_rst().clear_bit()
        });
    }

    fn set_burst_mode(&self, _burst_mode: BurstConfig) {}

    fn set_descr_burst_mode(&self, burst_mode: bool) {
        self.regs()
            .conf()
            .modify(|_, w| w.indscr_burst_en().bit(burst_mode));
    }

    fn set_peripheral(&self, peripheral: u8) {
        use esp32s2::crypto_dma::aes_sha_select::SELECT;
        let peripheral = match peripheral {
            p if p == DmaPeripheral::Aes as u8 => SELECT::Aes,
            p if p == DmaPeripheral::Sha as u8 => SELECT::Sha,
            _ => unreachable!(),
        };
        self.regs()
            .aes_sha_select()
            .modify(|_, w| w.select().variant(peripheral));
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
            panic!("Crypto DMA does not support checking descriptor ownership");
        }
    }

    fn is_compatible_with(&self, peripheral: DmaPeripheral) -> bool {
        self.0.is_compatible_with(peripheral)
    }

    #[cfg(psram_dma)]
    fn set_ext_mem_block_size(&self, size: DmaExtMemBKSize) {
        self.regs()
            .conf1()
            .modify(|_, w| unsafe { w.ext_mem_bk_size().bits(size as u8) });
    }

    #[cfg(psram_dma)]
    fn can_access_psram(&self) -> bool {
        true
    }
}

impl RxRegisterAccess for CryptoDmaRxChannel<'_> {
    fn peripheral_interrupt(&self) -> Option<Interrupt> {
        // We don't know if the channel is used by AES or SHA, so interrupt handler
        // setup is the responsibility of the peripheral driver.
        None
    }

    fn async_handler(&self) -> Option<InterruptHandler> {
        None
    }
}

impl InterruptAccess<DmaRxInterrupt> for CryptoDmaRxChannel<'_> {
    fn enable_listen(&self, interrupts: EnumSet<DmaRxInterrupt>, enable: bool) {
        self.regs().int_ena().modify(|_, w| {
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
        if int_ena.in_err_eof().bit_is_set() {
            result |= DmaRxInterrupt::ErrorEof;
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
        if int_raw.in_err_eof().bit_is_set() {
            result |= DmaRxInterrupt::ErrorEof;
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

impl<'d> DmaChannel for DMA_CRYPTO<'d> {
    type Rx = CryptoDmaRxChannel<'d>;
    type Tx = CryptoDmaTxChannel<'d>;
    unsafe fn split_internal(self, _: crate::private::Internal) -> (Self::Rx, Self::Tx) {
        (
            CryptoDmaRxChannel(unsafe { Self::steal() }),
            CryptoDmaTxChannel(unsafe { Self::steal() }),
        )
    }
}
impl DmaChannelExt for DMA_CRYPTO<'_> {
    fn rx_interrupts() -> impl InterruptAccess<DmaRxInterrupt> {
        CryptoDmaRxChannel(unsafe { Self::steal() })
    }
    fn tx_interrupts() -> impl InterruptAccess<DmaTxInterrupt> {
        CryptoDmaTxChannel(unsafe { Self::steal() })
    }
}
impl PdmaChannel for DMA_CRYPTO<'_> {
    type RegisterBlock = CryptoRegisterBlock;
    fn register_block(&self) -> &Self::RegisterBlock {
        DMA_CRYPTO::regs()
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
        Interrupt::CRYPTO_DMA
    }
    fn async_handler(&self) -> InterruptHandler {
        pub(crate) extern "C" fn __esp_hal_internal_interrupt_handler() {
            super::asynch::handle_in_interrupt::<DMA_CRYPTO<'static>>();
            super::asynch::handle_out_interrupt::<DMA_CRYPTO<'static>>();
        }
        #[allow(non_upper_case_globals)]
        pub(crate) static interrupt_handler: crate::interrupt::InterruptHandler =
            crate::interrupt::InterruptHandler::new(
                __esp_hal_internal_interrupt_handler,
                Priority::max(),
            );
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
impl<'d> DmaChannelConvert<CryptoDmaRxChannel<'d>> for DMA_CRYPTO<'d> {
    fn degrade(self) -> CryptoDmaRxChannel<'d> {
        CryptoDmaRxChannel(self)
    }
}
impl<'d> DmaChannelConvert<CryptoDmaTxChannel<'d>> for DMA_CRYPTO<'d> {
    fn degrade(self) -> CryptoDmaTxChannel<'d> {
        CryptoDmaTxChannel(self)
    }
}
