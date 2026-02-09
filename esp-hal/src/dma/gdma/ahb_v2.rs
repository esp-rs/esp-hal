use super::*;

impl AnyGdmaTxChannel<'_> {
    #[inline(always)]
    pub(super) fn ch(&self) -> &pac::dma::ch::CH {
        DMA::regs().ch(self.channel as usize)
    }

    #[inline(always)]
    pub(super) fn int(&self) -> &pac::dma::out_int_ch::OUT_INT_CH {
        DMA::regs().out_int_ch(self.channel as usize)
    }
}

impl RegisterAccess for AnyGdmaTxChannel<'_> {
    fn reset(&self) {
        let conf0 = self.ch().out_conf0();
        conf0.modify(|_, w| w.out_rst().set_bit());
        conf0.modify(|_, w| w.out_rst().clear_bit());
    }

    fn set_burst_mode(&self, burst_mode: BurstConfig) {
        self.ch()
            .out_conf0()
            .modify(|_, w| w.out_data_burst_en().bit(burst_mode.is_burst_enabled()));
    }

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
        DMA::regs()
            .out_link_addr_ch(self.channel as usize)
            .write(|w| unsafe { w.outlink_addr().bits(address) });
    }

    fn start(&self) {
        self.ch()
            .out_link()
            .modify(|_, w| w.outlink_start().set_bit());
    }

    fn stop(&self) {
        self.ch()
            .out_link()
            .modify(|_, w| w.outlink_stop().set_bit());
    }

    fn restart(&self) {
        self.ch()
            .out_link()
            .modify(|_, w| w.outlink_restart().set_bit());
    }

    fn set_check_owner(&self, check_owner: Option<bool>) {
        self.ch()
            .out_conf1()
            .modify(|_, w| w.out_check_owner().bit(check_owner.unwrap_or(true)));
    }

    // fn can_access_psram(&self) -> bool {
    //    true
    //}
}

impl TxRegisterAccess for AnyGdmaTxChannel<'_> {
    fn is_fifo_empty(&self) -> bool {
        self.ch()
            .outfifo_status()
            .read()
            .outfifo_empty()
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
        match self.channel {
            #[cfg(soc_has_dma_ch0)]
            0 => DMA_CH0::handler_out(),
            #[cfg(soc_has_dma_ch1)]
            1 => DMA_CH1::handler_out(),
            #[cfg(soc_has_dma_ch2)]
            2 => DMA_CH2::handler_out(),
            #[cfg(soc_has_dma_ch3)]
            3 => DMA_CH3::handler_out(),
            #[cfg(soc_has_dma_ch4)]
            4 => DMA_CH4::handler_out(),
            _ => unreachable!(),
        }
    }

    fn peripheral_interrupt(&self) -> Option<Interrupt> {
        match self.channel {
            #[cfg(soc_has_dma_ch0)]
            0 => DMA_CH0::isr_out(),
            #[cfg(soc_has_dma_ch1)]
            1 => DMA_CH1::isr_out(),
            #[cfg(soc_has_dma_ch2)]
            2 => DMA_CH2::isr_out(),
            #[cfg(soc_has_dma_ch3)]
            3 => DMA_CH3::isr_out(),
            #[cfg(soc_has_dma_ch4)]
            4 => DMA_CH4::isr_out(),
            _ => unreachable!(),
        }
    }
}

impl InterruptAccess<DmaTxInterrupt> for AnyGdmaTxChannel<'_> {
    fn enable_listen(&self, interrupts: EnumSet<DmaTxInterrupt>, enable: bool) {
        self.int().ena().modify(|_, w| {
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

        let int_ena = self.int().ena().read();
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
        self.int().clr().write(|w| {
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

        let int_raw = self.int().raw().read();
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
        &TX_WAKERS[self.channel as usize]
    }

    fn is_async(&self) -> bool {
        true
    }

    fn set_async(&self, _is_async: bool) {}
}

impl AnyGdmaRxChannel<'_> {
    #[inline(always)]
    fn ch(&self) -> &pac::dma::ch::CH {
        DMA::regs().ch(self.channel as usize)
    }

    #[inline(always)]
    fn int(&self) -> &pac::dma::in_int_ch::IN_INT_CH {
        DMA::regs().in_int_ch(self.channel as usize)
    }
}

impl RegisterAccess for AnyGdmaRxChannel<'_> {
    fn reset(&self) {
        let conf0 = self.ch().in_conf0();
        conf0.modify(|_, w| w.in_rst().set_bit());
        conf0.modify(|_, w| w.in_rst().clear_bit());
    }

    fn set_burst_mode(&self, burst_mode: BurstConfig) {
        self.ch()
            .in_conf0()
            .modify(|_, w| w.in_data_burst_en().bit(burst_mode.is_burst_enabled()));
    }

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
        DMA::regs()
            .in_link_addr_ch(self.channel as usize)
            .write(|w| unsafe { w.inlink_addr().bits(address) });
    }

    fn start(&self) {
        self.ch()
            .in_link()
            .modify(|_, w| w.inlink_start().set_bit());
    }

    fn stop(&self) {
        self.ch().in_link().modify(|_, w| w.inlink_stop().set_bit());
    }

    fn restart(&self) {
        self.ch()
            .in_link()
            .modify(|_, w| w.inlink_restart().set_bit());
    }

    fn set_check_owner(&self, check_owner: Option<bool>) {
        self.ch()
            .in_conf1()
            .modify(|_, w| w.in_check_owner().bit(check_owner.unwrap_or(true)));
    }

    // fn can_access_psram(&self) -> bool {
    //    true
    //}
}

impl RxRegisterAccess for AnyGdmaRxChannel<'_> {
    fn set_mem2mem_mode(&self, value: bool) {
        self.ch()
            .in_conf0()
            .modify(|_, w| w.mem_trans_en().bit(value));
    }

    fn async_handler(&self) -> Option<InterruptHandler> {
        match self.channel {
            #[cfg(soc_has_dma_ch0)]
            0 => DMA_CH0::handler_in(),
            #[cfg(soc_has_dma_ch1)]
            1 => DMA_CH1::handler_in(),
            #[cfg(soc_has_dma_ch2)]
            2 => DMA_CH2::handler_in(),
            #[cfg(soc_has_dma_ch3)]
            3 => DMA_CH3::handler_in(),
            #[cfg(soc_has_dma_ch4)]
            4 => DMA_CH4::handler_in(),
            _ => unreachable!(),
        }
    }

    fn peripheral_interrupt(&self) -> Option<Interrupt> {
        match self.channel {
            #[cfg(soc_has_dma_ch0)]
            0 => DMA_CH0::isr_in(),
            #[cfg(soc_has_dma_ch1)]
            1 => DMA_CH1::isr_in(),
            #[cfg(soc_has_dma_ch2)]
            2 => DMA_CH2::isr_in(),
            #[cfg(soc_has_dma_ch3)]
            3 => DMA_CH3::isr_in(),
            #[cfg(soc_has_dma_ch4)]
            4 => DMA_CH4::isr_in(),
            _ => unreachable!(),
        }
    }
}

impl InterruptAccess<DmaRxInterrupt> for AnyGdmaRxChannel<'_> {
    fn enable_listen(&self, interrupts: EnumSet<DmaRxInterrupt>, enable: bool) {
        self.int().ena().modify(|_, w| {
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

        let int_ena = self.int().ena().read();
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

    fn clear(&self, interrupts: impl Into<EnumSet<DmaRxInterrupt>>) {
        self.int().clr().write(|w| {
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

        let int_raw = self.int().raw().read();
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
        &RX_WAKERS[self.channel as usize]
    }

    fn is_async(&self) -> bool {
        true
    }

    fn set_async(&self, _is_async: bool) {}
}

pub(crate) fn setup() {
    // TODO: the range is device-specific
    let range = 0x4080_0000..0x4400_0000;
    trace!(
        "Configuring accessible memory range: 0x{:x}..0x{:x}",
        range.start, range.end
    );
    DMA::regs()
        .intr_mem_start_addr()
        .write(|w| unsafe { w.bits(range.start) });
    DMA::regs()
        .intr_mem_end_addr()
        .write(|w| unsafe { w.bits(range.end) });
}
