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

use crate::{
    dma::*,
    peripheral::PeripheralRef,
    system::{Peripheral, PeripheralClockControl},
};

#[doc(hidden)]
pub trait GdmaChannel {
    fn number(&self) -> u8;
}

/// An arbitrary GDMA channel
#[non_exhaustive]
pub struct AnyGdmaChannel(u8);

impl crate::private::Sealed for AnyGdmaChannel {}
impl DmaChannel for AnyGdmaChannel {
    type Rx = ChannelRxImpl<Self>;
    type Tx = ChannelTxImpl<Self>;
}

#[non_exhaustive]
#[doc(hidden)]
pub struct SpecificGdmaChannel<const N: u8> {}

impl GdmaChannel for AnyGdmaChannel {
    fn number(&self) -> u8 {
        self.0
    }
}
impl<const N: u8> GdmaChannel for SpecificGdmaChannel<N> {
    fn number(&self) -> u8 {
        N
    }
}

#[non_exhaustive]
#[doc(hidden)]
pub struct ChannelTxImpl<C: GdmaChannel>(C);

use embassy_sync::waitqueue::AtomicWaker;

static TX_WAKERS: [AtomicWaker; CHANNEL_COUNT] = [const { AtomicWaker::new() }; CHANNEL_COUNT];
static RX_WAKERS: [AtomicWaker; CHANNEL_COUNT] = [const { AtomicWaker::new() }; CHANNEL_COUNT];

impl<C: GdmaChannel> crate::private::Sealed for ChannelTxImpl<C> {}

impl<C: GdmaChannel> ChannelTxImpl<C> {
    #[inline(always)]
    fn ch(&self) -> &crate::peripherals::dma::ch::CH {
        let dma = unsafe { &*crate::peripherals::DMA::PTR };
        dma.ch(self.0.number() as usize)
    }

    #[cfg(any(esp32c2, esp32c3))]
    #[inline(always)]
    fn int(&self) -> &crate::peripherals::dma::int_ch::INT_CH {
        let dma = unsafe { &*crate::peripherals::DMA::PTR };
        dma.int_ch(self.0.number() as usize)
    }
    #[inline(always)]
    #[cfg(any(esp32c6, esp32h2))]
    fn int(&self) -> &crate::peripherals::dma::out_int_ch::OUT_INT_CH {
        let dma = unsafe { &*crate::peripherals::DMA::PTR };
        dma.out_int_ch(self.0.number() as usize)
    }
    #[cfg(esp32s3)]
    #[inline(always)]
    fn int(&self) -> &crate::peripherals::dma::ch::out_int::OUT_INT {
        let dma = unsafe { &*crate::peripherals::DMA::PTR };
        dma.ch(self.0.number() as usize).out_int()
    }

    fn degrade(self) -> ChannelTxImpl<AnyGdmaChannel> {
        ChannelTxImpl(AnyGdmaChannel(self.0.number()))
    }
}

impl<C: GdmaChannel> RegisterAccess for ChannelTxImpl<C> {
    fn reset(&self) {
        let conf0 = self.ch().out_conf0();
        conf0.modify(|_, w| w.out_rst().set_bit());
        conf0.modify(|_, w| w.out_rst().clear_bit());
    }

    fn set_burst_mode(&self, burst_mode: bool) {
        self.ch().out_conf0().modify(|_, w| {
            w.out_data_burst_en().bit(burst_mode);
            w.outdscr_burst_en().bit(burst_mode)
        });
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
}

impl<C: GdmaChannel> TxRegisterAccess for ChannelTxImpl<C> {
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
}

impl<C: GdmaChannel> InterruptAccess<DmaTxInterrupt> for ChannelTxImpl<C> {
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
        })
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
        })
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
        &TX_WAKERS[self.0.number() as usize]
    }
}

#[non_exhaustive]
#[doc(hidden)]
pub struct ChannelRxImpl<C: GdmaChannel>(C);

impl<C: GdmaChannel> crate::private::Sealed for ChannelRxImpl<C> {}

impl<C: GdmaChannel> ChannelRxImpl<C> {
    #[inline(always)]
    fn ch(&self) -> &crate::peripherals::dma::ch::CH {
        let dma = unsafe { &*crate::peripherals::DMA::PTR };
        dma.ch(self.0.number() as usize)
    }

    #[cfg(any(esp32c2, esp32c3))]
    #[inline(always)]
    fn int(&self) -> &crate::peripherals::dma::int_ch::INT_CH {
        let dma = unsafe { &*crate::peripherals::DMA::PTR };
        dma.int_ch(self.0.number() as usize)
    }

    #[inline(always)]
    #[cfg(any(esp32c6, esp32h2))]
    fn int(&self) -> &crate::peripherals::dma::in_int_ch::IN_INT_CH {
        let dma = unsafe { &*crate::peripherals::DMA::PTR };
        dma.in_int_ch(self.0.number() as usize)
    }

    #[cfg(esp32s3)]
    #[inline(always)]
    fn int(&self) -> &crate::peripherals::dma::ch::in_int::IN_INT {
        let dma = unsafe { &*crate::peripherals::DMA::PTR };
        dma.ch(self.0.number() as usize).in_int()
    }

    fn degrade(self) -> ChannelRxImpl<AnyGdmaChannel> {
        ChannelRxImpl(AnyGdmaChannel(self.0.number()))
    }
}

impl<C: GdmaChannel> RegisterAccess for ChannelRxImpl<C> {
    fn reset(&self) {
        let conf0 = self.ch().in_conf0();
        conf0.modify(|_, w| w.in_rst().set_bit());
        conf0.modify(|_, w| w.in_rst().clear_bit());
    }

    fn set_burst_mode(&self, burst_mode: bool) {
        self.ch().in_conf0().modify(|_, w| {
            w.in_data_burst_en().bit(burst_mode);
            w.indscr_burst_en().bit(burst_mode)
        });
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
}

impl<C: GdmaChannel> RxRegisterAccess for ChannelRxImpl<C> {
    fn set_mem2mem_mode(&self, value: bool) {
        self.ch()
            .in_conf0()
            .modify(|_, w| w.mem_trans_en().bit(value));
    }
}

impl<C: GdmaChannel> InterruptAccess<DmaRxInterrupt> for ChannelRxImpl<C> {
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
        })
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
        &RX_WAKERS[self.0.number() as usize]
    }
}

/// A Channel can be created from this
#[non_exhaustive]
pub struct ChannelCreator<const N: u8> {}

impl<CH: DmaChannel, M: Mode> Channel<'_, CH, M> {
    /// Asserts that the channel is compatible with the given peripheral.
    pub fn runtime_ensure_compatible<P: PeripheralMarker + DmaEligible>(
        &self,
        _peripheral: &PeripheralRef<'_, P>,
    ) {
        // No runtime checks; GDMA channels are compatible with any peripheral
    }
}

macro_rules! impl_channel {
    ($num: literal, $async_handler: path, $($interrupt: ident),* ) => {
        paste::paste! {
            /// A description of a specific GDMA channel
            #[non_exhaustive]
            pub struct [<DmaChannel $num>] {}

            impl crate::private::Sealed for [<DmaChannel $num>] {}

            impl DmaChannel for [<DmaChannel $num>] {
                type Rx = ChannelRxImpl<SpecificGdmaChannel<$num>>;
                type Tx = ChannelTxImpl<SpecificGdmaChannel<$num>>;
            }

            impl DmaChannelConvert<AnyGdmaChannel> for [<DmaChannel $num>] {
                fn degrade_rx(rx: Self::Rx) -> ChannelRxImpl<AnyGdmaChannel> {
                    rx.degrade()
                }
                fn degrade_tx(tx: Self::Tx) -> ChannelTxImpl<AnyGdmaChannel> {
                    tx.degrade()
                }
            }

            impl DmaChannelExt for [<DmaChannel $num>] {
                fn get_rx_interrupts() -> impl InterruptAccess<DmaRxInterrupt> {
                    ChannelRxImpl(SpecificGdmaChannel::<$num> {})
                }

                fn get_tx_interrupts() -> impl InterruptAccess<DmaTxInterrupt> {
                    ChannelTxImpl(SpecificGdmaChannel::<$num> {})
                }

                fn set_isr(handler: $crate::interrupt::InterruptHandler) {
                    let mut dma = unsafe { crate::peripherals::DMA::steal() };
                    $(
                        dma.[< bind_ $interrupt:lower _interrupt >](handler.handler());
                        $crate::interrupt::enable($crate::peripherals::Interrupt::$interrupt, handler.priority()).unwrap();
                    )*
                }
            }

            impl ChannelCreator<$num> {
                fn do_configure<'a, M: crate::Mode>(
                    self,
                    burst_mode: bool,
                    priority: DmaPriority,
                ) -> crate::dma::Channel<'a, [<DmaChannel $num>], M> {
                    let tx_impl = ChannelTxImpl(SpecificGdmaChannel::<$num> {});
                    tx_impl.set_burst_mode(burst_mode);
                    tx_impl.set_priority(priority);

                    let rx_impl = ChannelRxImpl(SpecificGdmaChannel::<$num> {});
                    rx_impl.set_burst_mode(burst_mode);
                    rx_impl.set_priority(priority);
                    // clear the mem2mem mode to avoid failed DMA if this
                    // channel was previously used for a mem2mem transfer.
                    rx_impl.set_mem2mem_mode(false);

                    crate::dma::Channel {
                        tx: ChannelTx::new(tx_impl, burst_mode),
                        rx: ChannelRx::new(rx_impl, burst_mode),
                        phantom: PhantomData,
                    }
                }

                /// Configure the channel for use with blocking APIs
                ///
                /// Descriptors should be sized as `(CHUNK_SIZE + 4091) / 4092`. I.e., to
                /// transfer buffers of size `1..=4092`, you need 1 descriptor.
                pub fn configure<'a>(
                    self,
                    burst_mode: bool,
                    priority: DmaPriority,
                ) -> crate::dma::Channel<'a, [<DmaChannel $num>], crate::Blocking> {
                    self.do_configure(burst_mode, priority)
                }

                /// Configure the channel for use with async APIs
                ///
                /// Descriptors should be sized as `(CHUNK_SIZE + 4091) / 4092`. I.e., to
                /// transfer buffers of size `1..=4092`, you need 1 descriptor.
                pub fn configure_for_async<'a>(
                    self,
                    burst_mode: bool,
                    priority: DmaPriority,
                ) -> crate::dma::Channel<'a, [<DmaChannel $num>], $crate::Async> {
                    let this = self.do_configure(burst_mode, priority);

                    [<DmaChannel $num>]::set_isr($async_handler);

                    this
                }
            }
        }
    };
}

cfg_if::cfg_if! {
    if #[cfg(esp32c2)] {
        const CHANNEL_COUNT: usize = 1;
        impl_channel!(0, super::asynch::interrupt::interrupt_handler_ch0, DMA_CH0);
    } else if #[cfg(esp32c3)] {
        const CHANNEL_COUNT: usize = 3;
        impl_channel!(0, super::asynch::interrupt::interrupt_handler_ch0, DMA_CH0);
        impl_channel!(1, super::asynch::interrupt::interrupt_handler_ch1, DMA_CH1);
        impl_channel!(2, super::asynch::interrupt::interrupt_handler_ch2, DMA_CH2);
    } else if #[cfg(any(esp32c6, esp32h2))] {
        const CHANNEL_COUNT: usize = 3;
        impl_channel!(0, super::asynch::interrupt::interrupt_handler_ch0, DMA_IN_CH0, DMA_OUT_CH0);
        impl_channel!(1, super::asynch::interrupt::interrupt_handler_ch1, DMA_IN_CH1, DMA_OUT_CH1);
        impl_channel!(2, super::asynch::interrupt::interrupt_handler_ch2, DMA_IN_CH2, DMA_OUT_CH2);
    } else if #[cfg(esp32s3)] {
        const CHANNEL_COUNT: usize = 5;
        impl_channel!(0, super::asynch::interrupt::interrupt_handler_ch0, DMA_IN_CH0, DMA_OUT_CH0);
        impl_channel!(1, super::asynch::interrupt::interrupt_handler_ch1, DMA_IN_CH1, DMA_OUT_CH1);
        impl_channel!(2, super::asynch::interrupt::interrupt_handler_ch2, DMA_IN_CH2, DMA_OUT_CH2);
        impl_channel!(3, super::asynch::interrupt::interrupt_handler_ch3, DMA_IN_CH3, DMA_OUT_CH3);
        impl_channel!(4, super::asynch::interrupt::interrupt_handler_ch4, DMA_IN_CH4, DMA_OUT_CH4);
    }
}

crate::impl_dma_eligible! {
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
crate::impl_dma_eligible! {
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

/// GDMA Peripheral
///
/// This offers the available DMA channels.
pub struct Dma<'d> {
    _inner: PeripheralRef<'d, crate::peripherals::DMA>,
    /// Channel 0
    pub channel0: ChannelCreator<0>,
    /// Channel 1
    #[cfg(not(esp32c2))]
    pub channel1: ChannelCreator<1>,
    /// Channel 2
    #[cfg(not(esp32c2))]
    pub channel2: ChannelCreator<2>,
    /// Channel 3
    #[cfg(esp32s3)]
    pub channel3: ChannelCreator<3>,
    /// Channel 4
    #[cfg(esp32s3)]
    pub channel4: ChannelCreator<4>,
}

impl<'d> Dma<'d> {
    /// Create a DMA instance.
    pub fn new(
        dma: impl crate::peripheral::Peripheral<P = crate::peripherals::DMA> + 'd,
    ) -> Dma<'d> {
        crate::into_ref!(dma);

        PeripheralClockControl::enable(Peripheral::Gdma);
        dma.misc_conf().modify(|_, w| w.ahbm_rst_inter().set_bit());
        dma.misc_conf()
            .modify(|_, w| w.ahbm_rst_inter().clear_bit());
        dma.misc_conf().modify(|_, w| w.clk_en().set_bit());

        Dma {
            _inner: dma,
            channel0: ChannelCreator {},
            #[cfg(not(esp32c2))]
            channel1: ChannelCreator {},
            #[cfg(not(esp32c2))]
            channel2: ChannelCreator {},
            #[cfg(esp32s3)]
            channel3: ChannelCreator {},
            #[cfg(esp32s3)]
            channel4: ChannelCreator {},
        }
    }
}
