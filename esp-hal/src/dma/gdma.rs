//! # General Direct Memory Access (GMDA)
//!
//! ## Overview
//! GDMA is a feature that allows peripheral-to-memory, memory-to-peripheral,
//! and memory-to-memory data transfer at high speed. The CPU is not involved in
//! the GDMA transfer and therefore is more efficient with less workload.
//!
//! The `GDMA` module provides multiple DMA channels, each capable of managing
//! data transfer for various peripherals.
//!
//! ## Configuration
//! GDMA peripheral can be initializes using the `new` function, which requires
//! a DMA peripheral instance and a clock control reference.
//!
//! <em>PS: Note that the number of DMA channels is chip-specific.</em>

use critical_section::CriticalSection;

use crate::{
    dma::*,
    interrupt::Priority,
    handler,
    peripheral::{Peripheral, PeripheralRef},
    peripherals::Interrupt,
};

/// An arbitrary GDMA channel
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AnyGdmaChannel(u8);

impl Peripheral for AnyGdmaChannel {
    type P = Self;

    unsafe fn clone_unchecked(&self) -> Self::P {
        Self(self.0)
    }
}

impl crate::private::Sealed for AnyGdmaChannel {}
impl DmaChannel for AnyGdmaChannel {
    type Rx = AnyGdmaRxChannel;
    type Tx = AnyGdmaTxChannel;

    fn set_priority(&self, priority: DmaPriority) {
        AnyGdmaRxChannel(self.0).set_priority(priority);
        AnyGdmaTxChannel(self.0).set_priority(priority);
    }

    unsafe fn split_internal(self, _: crate::private::Internal) -> (Self::Rx, Self::Tx) {
        (AnyGdmaRxChannel(self.0), AnyGdmaTxChannel(self.0))
    }
}

/// An arbitrary GDMA RX channel
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AnyGdmaRxChannel(u8);

impl Peripheral for AnyGdmaRxChannel {
    type P = Self;

    unsafe fn clone_unchecked(&self) -> Self::P {
        Self(self.0)
    }
}

impl DmaChannelConvert<AnyGdmaRxChannel> for AnyGdmaRxChannel {
    fn degrade(self) -> AnyGdmaRxChannel {
        self
    }
}

/// An arbitrary GDMA TX channel
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AnyGdmaTxChannel(u8);

impl Peripheral for AnyGdmaTxChannel {
    type P = Self;

    unsafe fn clone_unchecked(&self) -> Self::P {
        Self(self.0)
    }
}

impl DmaChannelConvert<AnyGdmaTxChannel> for AnyGdmaTxChannel {
    fn degrade(self) -> AnyGdmaTxChannel {
        self
    }
}

use crate::asynch::AtomicWaker;

static TX_WAKERS: [AtomicWaker; CHANNEL_COUNT] = [const { AtomicWaker::new() }; CHANNEL_COUNT];
static RX_WAKERS: [AtomicWaker; CHANNEL_COUNT] = [const { AtomicWaker::new() }; CHANNEL_COUNT];

cfg_if::cfg_if! {
    if #[cfg(any(esp32c2, esp32c3))] {
        use portable_atomic::AtomicBool;
        static TX_IS_ASYNC: [AtomicBool; CHANNEL_COUNT] = [const { AtomicBool::new(false) }; CHANNEL_COUNT];
        static RX_IS_ASYNC: [AtomicBool; CHANNEL_COUNT] = [const { AtomicBool::new(false) }; CHANNEL_COUNT];
    }
}

impl crate::private::Sealed for AnyGdmaTxChannel {}
impl DmaTxChannel for AnyGdmaTxChannel {}

impl AnyGdmaTxChannel {
    #[inline(always)]
    fn ch(&self) -> &crate::peripherals::dma::ch::CH {
        let dma = unsafe { &*crate::peripherals::DMA::PTR };
        dma.ch(self.0 as usize)
    }

    #[cfg(any(esp32c2, esp32c3))]
    #[inline(always)]
    fn int(&self) -> &crate::peripherals::dma::int_ch::INT_CH {
        let dma = unsafe { &*crate::peripherals::DMA::PTR };
        dma.int_ch(self.0 as usize)
    }
    #[inline(always)]
    #[cfg(any(esp32c6, esp32h2))]
    fn int(&self) -> &crate::peripherals::dma::out_int_ch::OUT_INT_CH {
        let dma = unsafe { &*crate::peripherals::DMA::PTR };
        dma.out_int_ch(self.0 as usize)
    }
    #[cfg(esp32s3)]
    #[inline(always)]
    fn int(&self) -> &crate::peripherals::dma::ch::out_int::OUT_INT {
        let dma = unsafe { &*crate::peripherals::DMA::PTR };
        dma.ch(self.0 as usize).out_int()
    }
}

impl RegisterAccess for AnyGdmaTxChannel {
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
            .modify(|_, w| unsafe { w.peri_out_sel().bits(peripheral) });
    }

    fn set_link_addr(&self, address: u32) {
        self.ch()
            .out_link()
            .modify(|_, w| unsafe { w.outlink_addr().bits(address) });
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

    #[cfg(esp32s3)]
    fn set_ext_mem_block_size(&self, size: DmaExtMemBKSize) {
        self.ch()
            .out_conf1()
            .modify(|_, w| unsafe { w.out_ext_mem_bk_size().bits(size as u8) });
    }

    #[cfg(psram_dma)]
    fn can_access_psram(&self) -> bool {
        true
    }
}

impl TxRegisterAccess for AnyGdmaTxChannel {
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
        match self.0 {
            0 => DmaChannel0::handler_out(),
            #[cfg(not(esp32c2))]
            1 => DmaChannel1::handler_out(),
            #[cfg(not(esp32c2))]
            2 => DmaChannel2::handler_out(),
            #[cfg(esp32s3)]
            3 => DmaChannel3::handler_out(),
            #[cfg(esp32s3)]
            4 => DmaChannel4::handler_out(),
            _ => unreachable!(),
        }
    }

    fn peripheral_interrupt(&self) -> Option<Interrupt> {
        match self.0 {
            0 => DmaChannel0::isr_out(),
            #[cfg(not(esp32c2))]
            1 => DmaChannel1::isr_out(),
            #[cfg(not(esp32c2))]
            2 => DmaChannel2::isr_out(),
            #[cfg(esp32s3)]
            3 => DmaChannel3::isr_out(),
            #[cfg(esp32s3)]
            4 => DmaChannel4::isr_out(),
            _ => unreachable!(),
        }
    }
}

impl InterruptAccess<DmaTxInterrupt> for AnyGdmaTxChannel {
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
        &TX_WAKERS[self.0 as usize]
    }

    fn is_async(&self) -> bool {
        cfg_if::cfg_if! {
            if #[cfg(any(esp32c2, esp32c3))] {
                TX_IS_ASYNC[self.0 as usize].load(portable_atomic::Ordering::Acquire)
            } else {
                true
            }
        }
    }

    fn set_async(&self, _is_async: bool) {
        cfg_if::cfg_if! {
            if #[cfg(any(esp32c2, esp32c3))] {
                TX_IS_ASYNC[self.0 as usize].store(_is_async, portable_atomic::Ordering::Release);
            }
        }
    }
}

impl crate::private::Sealed for AnyGdmaRxChannel {}
impl DmaRxChannel for AnyGdmaRxChannel {}

impl AnyGdmaRxChannel {
    #[inline(always)]
    fn ch(&self) -> &crate::peripherals::dma::ch::CH {
        let dma = unsafe { &*crate::peripherals::DMA::PTR };
        dma.ch(self.0 as usize)
    }

    #[cfg(any(esp32c2, esp32c3))]
    #[inline(always)]
    fn int(&self) -> &crate::peripherals::dma::int_ch::INT_CH {
        let dma = unsafe { &*crate::peripherals::DMA::PTR };
        dma.int_ch(self.0 as usize)
    }

    #[inline(always)]
    #[cfg(any(esp32c6, esp32h2))]
    fn int(&self) -> &crate::peripherals::dma::in_int_ch::IN_INT_CH {
        let dma = unsafe { &*crate::peripherals::DMA::PTR };
        dma.in_int_ch(self.0 as usize)
    }

    #[cfg(esp32s3)]
    #[inline(always)]
    fn int(&self) -> &crate::peripherals::dma::ch::in_int::IN_INT {
        let dma = unsafe { &*crate::peripherals::DMA::PTR };
        dma.ch(self.0 as usize).in_int()
    }
}

impl RegisterAccess for AnyGdmaRxChannel {
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
            .modify(|_, w| unsafe { w.peri_in_sel().bits(peripheral) });
    }

    fn set_link_addr(&self, address: u32) {
        self.ch()
            .in_link()
            .modify(|_, w| unsafe { w.inlink_addr().bits(address) });
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

    #[cfg(esp32s3)]
    fn set_ext_mem_block_size(&self, size: DmaExtMemBKSize) {
        self.ch()
            .in_conf1()
            .modify(|_, w| unsafe { w.in_ext_mem_bk_size().bits(size as u8) });
    }

    #[cfg(psram_dma)]
    fn can_access_psram(&self) -> bool {
        true
    }
}

impl RxRegisterAccess for AnyGdmaRxChannel {
    fn set_mem2mem_mode(&self, value: bool) {
        self.ch()
            .in_conf0()
            .modify(|_, w| w.mem_trans_en().bit(value));
    }

    fn async_handler(&self) -> Option<InterruptHandler> {
        match self.0 {
            0 => DmaChannel0::handler_in(),
            #[cfg(not(esp32c2))]
            1 => DmaChannel1::handler_in(),
            #[cfg(not(esp32c2))]
            2 => DmaChannel2::handler_in(),
            #[cfg(esp32s3)]
            3 => DmaChannel3::handler_in(),
            #[cfg(esp32s3)]
            4 => DmaChannel4::handler_in(),
            _ => unreachable!(),
        }
    }

    fn peripheral_interrupt(&self) -> Option<Interrupt> {
        match self.0 {
            0 => DmaChannel0::isr_in(),
            #[cfg(not(esp32c2))]
            1 => DmaChannel1::isr_in(),
            #[cfg(not(esp32c2))]
            2 => DmaChannel2::isr_in(),
            #[cfg(esp32s3)]
            3 => DmaChannel3::isr_in(),
            #[cfg(esp32s3)]
            4 => DmaChannel4::isr_in(),
            _ => unreachable!(),
        }
    }
}

impl InterruptAccess<DmaRxInterrupt> for AnyGdmaRxChannel {
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
        &RX_WAKERS[self.0 as usize]
    }

    fn is_async(&self) -> bool {
        cfg_if::cfg_if! {
            if #[cfg(any(esp32c2, esp32c3))] {
                RX_IS_ASYNC[self.0 as usize].load(portable_atomic::Ordering::Acquire)
            } else {
                true
            }
        }
    }

    fn set_async(&self, _is_async: bool) {
        cfg_if::cfg_if! {
            if #[cfg(any(esp32c2, esp32c3))] {
                RX_IS_ASYNC[self.0 as usize].store(_is_async, portable_atomic::Ordering::Release);
            }
        }
    }
}

impl<CH: DmaChannel, Dm: DriverMode> Channel<'_, Dm, CH> {
    /// Asserts that the channel is compatible with the given peripheral.
    pub fn runtime_ensure_compatible<P: DmaEligible>(&self, _peripheral: &PeripheralRef<'_, P>) {
        // No runtime checks; GDMA channels are compatible with any peripheral
    }
}

macro_rules! impl_channel {
    ($num:literal, $interrupt_in:ident $(, $interrupt_out:ident)? ) => {
        paste::paste! {
            /// A description of a specific GDMA channel
            #[non_exhaustive]
            #[derive(Debug, PartialEq, Eq)]
            #[cfg_attr(feature = "defmt", derive(defmt::Format))]
            pub struct [<DmaChannel $num>] {}

            impl $crate::private::Sealed for [<DmaChannel $num>] {}

            impl Peripheral for [<DmaChannel $num>] {
                type P = Self;

                unsafe fn clone_unchecked(&self) -> Self::P {
                    Self::steal()
                }
            }

            impl [<DmaChannel $num>] {
                /// Unsafely constructs a new DMA channel.
                ///
                /// # Safety
                ///
                /// The caller must ensure that only a single instance is used.
                pub unsafe fn steal() -> Self {
                    Self {}
                }
            }

            impl [<DmaChannel $num>] {
                fn handler_in() -> Option<InterruptHandler> {
                    $crate::if_set! {
                        $({
                            // $interrupt_out is present, meaning we have split handlers
                            #[handler(priority = Priority::max())]
                            fn interrupt_handler_in() {
                                $crate::ignore!($interrupt_out);
                                super::asynch::handle_in_interrupt::<[< DmaChannel $num >]>();
                            }
                            Some(interrupt_handler_in)
                        })?,
                        {
                            #[handler(priority = Priority::max())]
                            fn interrupt_handler() {
                                super::asynch::handle_in_interrupt::<[< DmaChannel $num >]>();
                                super::asynch::handle_out_interrupt::<[< DmaChannel $num >]>();
                            }
                            Some(interrupt_handler)
                        }
                    }
                }

                fn isr_in() -> Option<Interrupt> {
                    Some(Interrupt::$interrupt_in)
                }

                fn handler_out() -> Option<InterruptHandler> {
                    $crate::if_set! {
                        $({
                            #[handler(priority = Priority::max())]
                            fn interrupt_handler_out() {
                                $crate::ignore!($interrupt_out);
                                super::asynch::handle_out_interrupt::<[< DmaChannel $num >]>();
                            }
                            Some(interrupt_handler_out)
                        })?,
                        None
                    }
                }

                fn isr_out() -> Option<Interrupt> {
                    $crate::if_set! { $(Some(Interrupt::$interrupt_out))?, None }
                }
            }

            impl DmaChannel for [<DmaChannel $num>] {
                type Rx = AnyGdmaRxChannel;
                type Tx = AnyGdmaTxChannel;

                fn set_priority(&self, priority: DmaPriority) {
                    AnyGdmaChannel($num).set_priority(priority);
                }

                unsafe fn split_internal(self, _: $crate::private::Internal) -> (Self::Rx, Self::Tx) {
                    (AnyGdmaRxChannel($num), AnyGdmaTxChannel($num))
                }
            }

            impl DmaChannelConvert<AnyGdmaChannel> for [<DmaChannel $num>] {
                fn degrade(self) -> AnyGdmaChannel {
                    AnyGdmaChannel($num)
                }
            }

            impl DmaChannelConvert<AnyGdmaRxChannel> for [<DmaChannel $num>] {
                fn degrade(self) -> AnyGdmaRxChannel {
                    AnyGdmaRxChannel($num)
                }
            }

            impl DmaChannelConvert<AnyGdmaTxChannel> for [<DmaChannel $num>] {
                fn degrade(self) -> AnyGdmaTxChannel {
                    AnyGdmaTxChannel($num)
                }
            }

            impl DmaChannelExt for [<DmaChannel $num>] {
                fn rx_interrupts() -> impl InterruptAccess<DmaRxInterrupt> {
                    AnyGdmaRxChannel($num)
                }

                fn tx_interrupts() -> impl InterruptAccess<DmaTxInterrupt> {
                    AnyGdmaTxChannel($num)
                }
            }
        }
    };
}

cfg_if::cfg_if! {
    if #[cfg(esp32c2)] {
        const CHANNEL_COUNT: usize = 1;
        impl_channel!(0, DMA_CH0);
    } else if #[cfg(esp32c3)] {
        const CHANNEL_COUNT: usize = 3;
        impl_channel!(0, DMA_CH0);
        impl_channel!(1, DMA_CH1);
        impl_channel!(2, DMA_CH2);
    } else if #[cfg(any(esp32c6, esp32h2))] {
        const CHANNEL_COUNT: usize = 3;
        impl_channel!(0, DMA_IN_CH0, DMA_OUT_CH0);
        impl_channel!(1, DMA_IN_CH1, DMA_OUT_CH1);
        impl_channel!(2, DMA_IN_CH2, DMA_OUT_CH2);
    } else if #[cfg(esp32s3)] {
        const CHANNEL_COUNT: usize = 5;
        impl_channel!(0, DMA_IN_CH0, DMA_OUT_CH0);
        impl_channel!(1, DMA_IN_CH1, DMA_OUT_CH1);
        impl_channel!(2, DMA_IN_CH2, DMA_OUT_CH2);
        impl_channel!(3, DMA_IN_CH3, DMA_OUT_CH3);
        impl_channel!(4, DMA_IN_CH4, DMA_OUT_CH4);
    }
}

crate::dma::impl_dma_eligible! {
    AnyGdmaChannel {
        #[cfg(spi2)]
        SPI2 => Spi2,

        #[cfg(spi3)]
        SPI3 => Spi3,

        #[cfg(uhci0)]
        UHCI0 => Uhci0,

        #[cfg(i2s0)]
        I2S0 => I2s0,

        #[cfg(i2s1)]
        I2S1 => I2s1,

        #[cfg(esp32s3)]
        LCD_CAM => LcdCam,

        #[cfg(all(gdma, aes))]
        AES => Aes,

        #[cfg(all(gdma, sha))]
        SHA => Sha,

        #[cfg(any(esp32c3, esp32c6, esp32h2, esp32s3))]
        ADC1 => Adc,

        #[cfg(any(esp32c3, esp32s3))]
        ADC2 => Adc,

        #[cfg(esp32s3)]
        RMT => Rmt,

        #[cfg(parl_io)]
        PARL_IO => ParlIo,

        #[cfg(any(esp32c2, esp32c6, esp32h2))]
        MEM2MEM1 => Mem2Mem1,
    }
}

#[cfg(any(esp32c6, esp32h2))]
crate::dma::impl_dma_eligible! {
    AnyGdmaChannel {
        MEM2MEM4 => Mem2Mem4,
        MEM2MEM5 => Mem2Mem5,
        MEM2MEM10 => Mem2Mem10,
        MEM2MEM11 => Mem2Mem11,
        MEM2MEM12 => Mem2Mem12,
        MEM2MEM13 => Mem2Mem13,
        MEM2MEM14 => Mem2Mem14,
        MEM2MEM15 => Mem2Mem15,
    }
}

pub(super) fn init_dma(_cs: CriticalSection<'_>) {
    let dma = unsafe { crate::soc::peripherals::DMA::steal() };
    dma.misc_conf().modify(|_, w| w.ahbm_rst_inter().set_bit());
    dma.misc_conf()
        .modify(|_, w| w.ahbm_rst_inter().clear_bit());
    dma.misc_conf().modify(|_, w| w.clk_en().set_bit());
}
