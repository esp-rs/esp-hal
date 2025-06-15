use portable_atomic::{AtomicBool, Ordering};

use crate::{asynch::AtomicWaker, dma::*, peripherals::Interrupt};

pub(super) type I2sRegisterBlock = crate::pac::i2s0::RegisterBlock;

/// The RX half of an arbitrary I2S DMA channel.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AnyI2sDmaRxChannel<'d>(pub(crate) AnyI2sDmaChannel<'d>);

impl AnyI2sDmaRxChannel<'_> {
    fn regs(&self) -> &I2sRegisterBlock {
        self.0.register_block()
    }
}

impl crate::private::Sealed for AnyI2sDmaRxChannel<'_> {}
impl DmaRxChannel for AnyI2sDmaRxChannel<'_> {}

/// The TX half of an arbitrary I2S DMA channel.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AnyI2sDmaTxChannel<'d>(pub(crate) AnyI2sDmaChannel<'d>);

impl AnyI2sDmaTxChannel<'_> {
    fn regs(&self) -> &I2sRegisterBlock {
        self.0.register_block()
    }
}

impl crate::private::Sealed for AnyI2sDmaTxChannel<'_> {}
impl DmaTxChannel for AnyI2sDmaTxChannel<'_> {}

impl RegisterAccess for AnyI2sDmaTxChannel<'_> {
    fn reset(&self) {
        self.regs().lc_conf().modify(|_, w| w.out_rst().set_bit());
        self.regs().lc_conf().modify(|_, w| w.out_rst().clear_bit());
    }

    fn set_burst_mode(&self, burst_mode: BurstConfig) {
        self.regs()
            .lc_conf()
            .modify(|_, w| w.out_data_burst_en().bit(burst_mode.is_burst_enabled()));
    }

    fn set_descr_burst_mode(&self, burst_mode: bool) {
        self.regs()
            .lc_conf()
            .modify(|_, w| w.outdscr_burst_en().bit(burst_mode));
    }

    fn set_link_addr(&self, address: u32) {
        self.regs()
            .out_link()
            .modify(|_, w| unsafe { w.outlink_addr().bits(address) });
    }

    fn set_peripheral(&self, _peripheral: u8) {
        // no-op
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
        self.regs()
            .lc_conf()
            .modify(|_, w| w.check_owner().bit(check_owner.unwrap_or(true)));
    }

    fn is_compatible_with(&self, peripheral: DmaPeripheral) -> bool {
        self.0.is_compatible_with(peripheral)
    }

    #[cfg(psram_dma)]
    fn set_ext_mem_block_size(&self, size: DmaExtMemBKSize) {
        self.regs()
            .lc_conf()
            .modify(|_, w| unsafe { w.ext_mem_bk_size().bits(size as u8) });
    }

    #[cfg(psram_dma)]
    fn can_access_psram(&self) -> bool {
        matches!(self.0, AnyI2sDmaChannel(AnyI2sDmaChannelInner::I2s0(_)))
    }
}

impl TxRegisterAccess for AnyI2sDmaTxChannel<'_> {
    fn is_fifo_empty(&self) -> bool {
        cfg_if::cfg_if! {
            if #[cfg(esp32)] {
                self.regs().lc_state0().read().bits() & 0x80000000 != 0
            } else {
                self.regs().lc_state0().read().out_empty().bit_is_set()
            }
        }
    }

    fn set_auto_write_back(&self, enable: bool) {
        self.regs()
            .lc_conf()
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
        Some(self.0.peripheral_interrupt())
    }

    fn async_handler(&self) -> Option<InterruptHandler> {
        Some(self.0.async_handler())
    }
}

impl InterruptAccess<DmaTxInterrupt> for AnyI2sDmaTxChannel<'_> {
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

    fn waker(&self) -> &'static AtomicWaker {
        self.0.tx_waker()
    }

    fn is_async(&self) -> bool {
        self.0.tx_async_flag().load(Ordering::Relaxed)
    }

    fn set_async(&self, _is_async: bool) {
        self.0.tx_async_flag().store(_is_async, Ordering::Relaxed);
    }
}

impl RegisterAccess for AnyI2sDmaRxChannel<'_> {
    fn reset(&self) {
        self.regs().lc_conf().modify(|_, w| w.in_rst().set_bit());
        self.regs().lc_conf().modify(|_, w| w.in_rst().clear_bit());
    }

    fn set_burst_mode(&self, _burst_mode: BurstConfig) {}

    fn set_descr_burst_mode(&self, burst_mode: bool) {
        self.regs()
            .lc_conf()
            .modify(|_, w| w.indscr_burst_en().bit(burst_mode));
    }

    fn set_link_addr(&self, address: u32) {
        self.regs()
            .in_link()
            .modify(|_, w| unsafe { w.inlink_addr().bits(address) });
    }

    fn set_peripheral(&self, _peripheral: u8) {
        // no-op
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
        self.regs()
            .lc_conf()
            .modify(|_, w| w.check_owner().bit(check_owner.unwrap_or(true)));
    }

    fn is_compatible_with(&self, peripheral: DmaPeripheral) -> bool {
        self.0.is_compatible_with(peripheral)
    }

    #[cfg(psram_dma)]
    fn set_ext_mem_block_size(&self, size: DmaExtMemBKSize) {
        self.regs()
            .lc_conf()
            .modify(|_, w| unsafe { w.ext_mem_bk_size().bits(size as u8) });
    }

    #[cfg(psram_dma)]
    fn can_access_psram(&self) -> bool {
        matches!(self.0, AnyI2sDmaChannel(AnyI2sDmaChannelInner::I2s0(_)))
    }
}

impl RxRegisterAccess for AnyI2sDmaRxChannel<'_> {
    fn peripheral_interrupt(&self) -> Option<Interrupt> {
        Some(self.0.peripheral_interrupt())
    }

    fn async_handler(&self) -> Option<InterruptHandler> {
        Some(self.0.async_handler())
    }
}

impl InterruptAccess<DmaRxInterrupt> for AnyI2sDmaRxChannel<'_> {
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
        if int_ena.in_dscr_empty().bit_is_set() {
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

crate::any_peripheral! {
    /// An I2S-compatible type-erased DMA channel.
    pub peripheral AnyI2sDmaChannel<'d> {
        I2s0(super::DMA_I2S0<'d>),
        #[cfg(i2s1)]
        I2s1(super::DMA_I2S1<'d>),
    }
}

impl<'d> DmaChannel for AnyI2sDmaChannel<'d> {
    type Rx = AnyI2sDmaRxChannel<'d>;
    type Tx = AnyI2sDmaTxChannel<'d>;

    unsafe fn split_internal(self, _: crate::private::Internal) -> (Self::Rx, Self::Tx) {
        (
            AnyI2sDmaRxChannel(unsafe { self.clone_unchecked() }),
            AnyI2sDmaTxChannel(unsafe { self.clone_unchecked() }),
        )
    }
}

impl PdmaChannel for AnyI2sDmaChannel<'_> {
    type RegisterBlock = I2sRegisterBlock;

    delegate::delegate! {
        to match &self.0 {
            AnyI2sDmaChannelInner::I2s0(channel) => channel,
            #[cfg(i2s1)]
            AnyI2sDmaChannelInner::I2s1(channel) => channel,
        } {
            fn register_block(&self) -> &I2sRegisterBlock;
            fn tx_waker(&self) -> &'static AtomicWaker;
            fn rx_waker(&self) -> &'static AtomicWaker;
            fn is_compatible_with(&self, peripheral: DmaPeripheral) -> bool;
            fn peripheral_interrupt(&self) -> Interrupt;
            fn async_handler(&self) -> InterruptHandler;
            fn rx_async_flag(&self) -> &'static AtomicBool;
            fn tx_async_flag(&self) -> &'static AtomicBool;
        }
    }
}
