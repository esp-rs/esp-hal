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

#[non_exhaustive]
#[doc(hidden)]
pub struct ChannelTxImpl<const N: u8> {}

use embassy_sync::waitqueue::AtomicWaker;

static TX_WAKERS: [AtomicWaker; CHANNEL_COUNT] = [const { AtomicWaker::new() }; CHANNEL_COUNT];
static RX_WAKERS: [AtomicWaker; CHANNEL_COUNT] = [const { AtomicWaker::new() }; CHANNEL_COUNT];

impl<const N: u8> crate::private::Sealed for ChannelTxImpl<N> {}

impl<const N: u8> ChannelTxImpl<N> {
    #[inline(always)]
    fn ch() -> &'static crate::peripherals::dma::ch::CH {
        let dma = unsafe { &*crate::peripherals::DMA::PTR };
        dma.ch(N as usize)
    }

    #[cfg(any(esp32c2, esp32c3))]
    #[inline(always)]
    fn int() -> &'static crate::peripherals::dma::int_ch::INT_CH {
        let dma = unsafe { &*crate::peripherals::DMA::PTR };
        dma.int_ch(N as usize)
    }
    #[inline(always)]
    #[cfg(any(esp32c6, esp32h2))]
    fn int() -> &'static crate::peripherals::dma::out_int_ch::OUT_INT_CH {
        let dma = unsafe { &*crate::peripherals::DMA::PTR };
        dma.out_int_ch(N as usize)
    }
    #[cfg(esp32s3)]
    #[inline(always)]
    fn int() -> &'static crate::peripherals::dma::ch::out_int::OUT_INT {
        let dma = unsafe { &*crate::peripherals::DMA::PTR };
        dma.ch(N as usize).out_int()
    }
}

impl<const N: u8> RegisterAccess for ChannelTxImpl<N> {
    fn reset(&self) {
        let conf0 = Self::ch().out_conf0();
        conf0.modify(|_, w| w.out_rst().set_bit());
        conf0.modify(|_, w| w.out_rst().clear_bit());
    }

    fn set_burst_mode(&self, burst_mode: bool) {
        Self::ch().out_conf0().modify(|_, w| {
            w.out_data_burst_en().bit(burst_mode);
            w.outdscr_burst_en().bit(burst_mode)
        });
    }

    fn set_priority(&self, priority: DmaPriority) {
        Self::ch()
            .out_pri()
            .write(|w| unsafe { w.tx_pri().bits(priority as u8) });
    }

    fn set_peripheral(&self, peripheral: u8) {
        Self::ch()
            .out_peri_sel()
            .modify(|_, w| unsafe { w.peri_out_sel().bits(peripheral) });
    }

    fn set_link_addr(&self, address: u32) {
        Self::ch()
            .out_link()
            .modify(|_, w| unsafe { w.outlink_addr().bits(address) });
    }

    fn start(&self) {
        Self::ch()
            .out_link()
            .modify(|_, w| w.outlink_start().set_bit());
    }

    fn stop(&self) {
        Self::ch()
            .out_link()
            .modify(|_, w| w.outlink_stop().set_bit());
    }

    fn restart(&self) {
        Self::ch()
            .out_link()
            .modify(|_, w| w.outlink_restart().set_bit());
    }

    fn clear_interrupts(&self) {
        #[cfg(not(esp32s3))]
        Self::int().clr().write(|w| {
            w.out_eof().clear_bit_by_one();
            w.out_dscr_err().clear_bit_by_one();
            w.out_done().clear_bit_by_one();
            w.out_total_eof().clear_bit_by_one();
            w.outfifo_ovf().clear_bit_by_one();
            w.outfifo_udf().clear_bit_by_one()
        });

        #[cfg(esp32s3)]
        Self::int().clr().write(|w| {
            w.out_eof().clear_bit_by_one();
            w.out_dscr_err().clear_bit_by_one();
            w.out_done().clear_bit_by_one();
            w.out_total_eof().clear_bit_by_one();
            w.outfifo_ovf_l1().clear_bit_by_one();
            w.outfifo_ovf_l3().clear_bit_by_one();
            w.outfifo_udf_l1().clear_bit_by_one();
            w.outfifo_udf_l3().clear_bit_by_one()
        });
    }

    #[cfg(esp32s3)]
    fn set_ext_mem_block_size(&self, size: DmaExtMemBKSize) {
        Self::ch()
            .out_conf1()
            .modify(|_, w| unsafe { w.out_ext_mem_bk_size().bits(size as u8) });
    }
}

impl<const N: u8> TxRegisterAccess for ChannelTxImpl<N> {
    fn last_dscr_address(&self) -> usize {
        Self::ch()
            .out_eof_des_addr()
            .read()
            .out_eof_des_addr()
            .bits() as _
    }
}

impl<const N: u8> InterruptAccess<DmaTxInterrupt> for ChannelTxImpl<N> {
    fn listen(&self, interrupts: impl Into<EnumSet<DmaTxInterrupt>>) {
        Self::int().ena().modify(|_, w| {
            for interrupt in interrupts.into() {
                match interrupt {
                    DmaTxInterrupt::TotalEof => w.out_total_eof().set_bit(),
                    DmaTxInterrupt::DescriptorError => w.out_dscr_err().set_bit(),
                    DmaTxInterrupt::Eof => w.out_eof().set_bit(),
                    DmaTxInterrupt::Done => w.out_done().set_bit(),
                };
            }
            w
        })
    }

    fn unlisten(&self, interrupts: impl Into<EnumSet<DmaTxInterrupt>>) {
        Self::int().ena().modify(|_, w| {
            for interrupt in interrupts.into() {
                match interrupt {
                    DmaTxInterrupt::TotalEof => w.out_total_eof().clear_bit(),
                    DmaTxInterrupt::DescriptorError => w.out_dscr_err().clear_bit(),
                    DmaTxInterrupt::Eof => w.out_eof().clear_bit(),
                    DmaTxInterrupt::Done => w.out_done().clear_bit(),
                };
            }
            w
        })
    }

    fn is_listening(&self) -> EnumSet<DmaTxInterrupt> {
        let mut result = EnumSet::new();

        let int_ena = Self::int().ena().read();
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
        Self::int().clr().write(|w| {
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

        let int_raw = Self::int().raw().read();
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
        &TX_WAKERS[N as usize]
    }
}

#[non_exhaustive]
#[doc(hidden)]
pub struct ChannelRxImpl<const N: u8> {}

impl<const N: u8> crate::private::Sealed for ChannelRxImpl<N> {}

impl<const N: u8> ChannelRxImpl<N> {
    #[inline(always)]
    fn ch() -> &'static crate::peripherals::dma::ch::CH {
        let dma = unsafe { &*crate::peripherals::DMA::PTR };
        dma.ch(N as usize)
    }

    #[cfg(any(esp32c2, esp32c3))]
    #[inline(always)]
    fn int() -> &'static crate::peripherals::dma::int_ch::INT_CH {
        let dma = unsafe { &*crate::peripherals::DMA::PTR };
        dma.int_ch(N as usize)
    }

    #[inline(always)]
    #[cfg(any(esp32c6, esp32h2))]
    fn int() -> &'static crate::peripherals::dma::in_int_ch::IN_INT_CH {
        let dma = unsafe { &*crate::peripherals::DMA::PTR };
        dma.in_int_ch(N as usize)
    }

    #[cfg(esp32s3)]
    #[inline(always)]
    fn int() -> &'static crate::peripherals::dma::ch::in_int::IN_INT {
        let dma = unsafe { &*crate::peripherals::DMA::PTR };
        dma.ch(N as usize).in_int()
    }
}

impl<const N: u8> RegisterAccess for ChannelRxImpl<N> {
    fn reset(&self) {
        let conf0 = Self::ch().in_conf0();
        conf0.modify(|_, w| w.in_rst().set_bit());
        conf0.modify(|_, w| w.in_rst().clear_bit());
    }

    fn set_burst_mode(&self, burst_mode: bool) {
        Self::ch().in_conf0().modify(|_, w| {
            w.in_data_burst_en().bit(burst_mode);
            w.indscr_burst_en().bit(burst_mode)
        });
    }

    fn set_priority(&self, priority: DmaPriority) {
        Self::ch()
            .in_pri()
            .write(|w| unsafe { w.rx_pri().bits(priority as u8) });
    }

    fn set_peripheral(&self, peripheral: u8) {
        Self::ch()
            .in_peri_sel()
            .modify(|_, w| unsafe { w.peri_in_sel().bits(peripheral) });
    }

    fn set_link_addr(&self, address: u32) {
        Self::ch()
            .in_link()
            .modify(|_, w| unsafe { w.inlink_addr().bits(address) });
    }

    fn start(&self) {
        Self::ch()
            .in_link()
            .modify(|_, w| w.inlink_start().set_bit());
    }

    fn stop(&self) {
        Self::ch()
            .in_link()
            .modify(|_, w| w.inlink_stop().set_bit());
    }

    fn restart(&self) {
        Self::ch()
            .in_link()
            .modify(|_, w| w.inlink_restart().set_bit());
    }

    fn clear_interrupts(&self) {
        #[cfg(not(esp32s3))]
        Self::int().clr().write(|w| {
            w.in_suc_eof().clear_bit_by_one();
            w.in_err_eof().clear_bit_by_one();
            w.in_dscr_err().clear_bit_by_one();
            w.in_dscr_empty().clear_bit_by_one();
            w.in_done().clear_bit_by_one();
            w.infifo_ovf().clear_bit_by_one();
            w.infifo_udf().clear_bit_by_one()
        });

        #[cfg(esp32s3)]
        Self::int().clr().write(|w| {
            w.in_suc_eof().clear_bit_by_one();
            w.in_err_eof().clear_bit_by_one();
            w.in_dscr_err().clear_bit_by_one();
            w.in_dscr_empty().clear_bit_by_one();
            w.in_done().clear_bit_by_one();
            w.infifo_ovf_l1().clear_bit_by_one();
            w.infifo_ovf_l3().clear_bit_by_one();
            w.infifo_udf_l1().clear_bit_by_one();
            w.infifo_udf_l3().clear_bit_by_one()
        });
    }

    #[cfg(esp32s3)]
    fn set_ext_mem_block_size(&self, size: DmaExtMemBKSize) {
        Self::ch()
            .in_conf1()
            .modify(|_, w| unsafe { w.in_ext_mem_bk_size().bits(size as u8) });
    }
}

impl<const N: u8> RxRegisterAccess for ChannelRxImpl<N> {
    fn set_mem2mem_mode(&self, value: bool) {
        Self::ch()
            .in_conf0()
            .modify(|_, w| w.mem_trans_en().bit(value));
    }
}

impl<const N: u8> InterruptAccess<DmaRxInterrupt> for ChannelRxImpl<N> {
    fn listen(&self, interrupts: impl Into<EnumSet<DmaRxInterrupt>>) {
        Self::int().ena().modify(|_, w| {
            for interrupt in interrupts.into() {
                match interrupt {
                    DmaRxInterrupt::SuccessfulEof => w.in_suc_eof().set_bit(),
                    DmaRxInterrupt::ErrorEof => w.in_err_eof().set_bit(),
                    DmaRxInterrupt::DescriptorError => w.in_dscr_err().set_bit(),
                    DmaRxInterrupt::DescriptorEmpty => w.in_dscr_empty().set_bit(),
                    DmaRxInterrupt::Done => w.in_done().set_bit(),
                };
            }
            w
        });
    }

    fn unlisten(&self, interrupts: impl Into<EnumSet<DmaRxInterrupt>>) {
        Self::int().ena().modify(|_, w| {
            for interrupt in interrupts.into() {
                match interrupt {
                    DmaRxInterrupt::SuccessfulEof => w.in_suc_eof().clear_bit(),
                    DmaRxInterrupt::ErrorEof => w.in_err_eof().clear_bit(),
                    DmaRxInterrupt::DescriptorError => w.in_dscr_err().clear_bit(),
                    DmaRxInterrupt::DescriptorEmpty => w.in_dscr_empty().clear_bit(),
                    DmaRxInterrupt::Done => w.in_done().clear_bit(),
                };
            }
            w
        })
    }

    fn is_listening(&self) -> EnumSet<DmaRxInterrupt> {
        let mut result = EnumSet::new();

        let int_ena = Self::int().ena().read();
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
        Self::int().clr().write(|w| {
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

        let int_raw = Self::int().raw().read();
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
        &RX_WAKERS[N as usize]
    }
}

/// A Channel can be created from this
#[non_exhaustive]
pub struct ChannelCreator<const N: u8> {}

#[non_exhaustive]
#[doc(hidden)]
pub struct SuitablePeripheral<const N: u8> {}
impl<const N: u8> PeripheralMarker for SuitablePeripheral<N> {}

// with GDMA every channel can be used for any peripheral
impl<const N: u8> SpiPeripheral for SuitablePeripheral<N> {}
impl<const N: u8> Spi2Peripheral for SuitablePeripheral<N> {}
#[cfg(spi3)]
impl<const N: u8> Spi3Peripheral for SuitablePeripheral<N> {}
#[cfg(any(i2s0, i2s1))]
impl<const N: u8> I2sPeripheral for SuitablePeripheral<N> {}
#[cfg(i2s0)]
impl<const N: u8> I2s0Peripheral for SuitablePeripheral<N> {}
#[cfg(i2s1)]
impl<const N: u8> I2s1Peripheral for SuitablePeripheral<N> {}
#[cfg(parl_io)]
impl<const N: u8> ParlIoPeripheral for SuitablePeripheral<N> {}
#[cfg(aes)]
impl<const N: u8> AesPeripheral for SuitablePeripheral<N> {}
#[cfg(lcd_cam)]
impl<const N: u8> LcdCamPeripheral for SuitablePeripheral<N> {}

macro_rules! impl_channel {
    ($num: literal, $async_handler: path, $($interrupt: ident),* ) => {
        paste::paste! {
            /// A description of a GDMA channel
            #[non_exhaustive]
            pub struct [<DmaChannel $num>] {}

            impl crate::private::Sealed for [<DmaChannel $num>] {}

            impl DmaChannel for [<DmaChannel $num>] {
                type Rx = ChannelRxImpl<$num>;
                type Tx = ChannelTxImpl<$num>;
                type P = SuitablePeripheral<$num>;

                fn set_isr(handler: $crate::interrupt::InterruptHandler) {
                    let mut dma = unsafe { crate::peripherals::DMA::steal() };
                    $(
                        dma.[< bind_ $interrupt:lower _interrupt >](handler.handler());
                        $crate::interrupt::enable($crate::peripherals::Interrupt::$interrupt, handler.priority()).unwrap();
                    )*
                }
            }

            impl DmaChannelExt for [<DmaChannel $num>] {
                fn get_rx_interrupts() -> impl InterruptAccess<DmaRxInterrupt> {
                    ChannelRxImpl::<$num> {}
                }

                fn get_tx_interrupts() -> impl InterruptAccess<DmaTxInterrupt> {
                    ChannelTxImpl::<$num> {}
                }
            }

            impl ChannelCreator<$num> {
                fn do_configure<'a, M: crate::Mode>(
                    self,
                    burst_mode: bool,
                    priority: DmaPriority,
                ) -> crate::dma::Channel<'a, [<DmaChannel $num>], M> {
                    let tx_impl = ChannelTxImpl {};
                    tx_impl.set_burst_mode(burst_mode);
                    tx_impl.set_priority(priority);

                    let rx_impl = ChannelRxImpl {};
                    rx_impl.set_burst_mode(burst_mode);
                    rx_impl.set_priority(priority);
                    // clear the mem2mem mode to avoid failed DMA if this
                    // channel was previously used for a mem2mem transfer.
                    #[cfg(gdma)]
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

pub use m2m::*;
mod m2m {
    #[cfg(esp32s3)]
    use crate::dma::DmaExtMemBKSize;
    use crate::dma::{
        dma_private::{DmaSupport, DmaSupportRx},
        Channel,
        ChannelRx,
        DescriptorChain,
        DmaChannel,
        DmaDescriptor,
        DmaEligible,
        DmaError,
        DmaPeripheral,
        DmaTransferRx,
        ReadBuffer,
        Rx,
        Tx,
        WriteBuffer,
    };

    /// DMA Memory to Memory pseudo-Peripheral
    ///
    /// This is a pseudo-peripheral that allows for memory to memory transfers.
    /// It is not a real peripheral, but a way to use the DMA engine for memory
    /// to memory transfers.
    pub struct Mem2Mem<'d, C, MODE>
    where
        C: DmaChannel,
        MODE: crate::Mode,
    {
        channel: Channel<'d, C, MODE>,
        rx_chain: DescriptorChain,
        tx_chain: DescriptorChain,
        peripheral: DmaPeripheral,
    }

    impl<'d, C, MODE> Mem2Mem<'d, C, MODE>
    where
        C: DmaChannel,
        MODE: crate::Mode,
    {
        /// Create a new Mem2Mem instance.
        pub fn new(
            channel: Channel<'d, C, MODE>,
            peripheral: impl DmaEligible,
            rx_descriptors: &'static mut [DmaDescriptor],
            tx_descriptors: &'static mut [DmaDescriptor],
        ) -> Result<Self, DmaError> {
            unsafe {
                Self::new_unsafe(
                    channel,
                    peripheral.dma_peripheral(),
                    rx_descriptors,
                    tx_descriptors,
                    crate::dma::CHUNK_SIZE,
                )
            }
        }

        /// Create a new Mem2Mem instance with specific chunk size.
        pub fn new_with_chunk_size(
            channel: Channel<'d, C, MODE>,
            peripheral: impl DmaEligible,
            rx_descriptors: &'static mut [DmaDescriptor],
            tx_descriptors: &'static mut [DmaDescriptor],
            chunk_size: usize,
        ) -> Result<Self, DmaError> {
            unsafe {
                Self::new_unsafe(
                    channel,
                    peripheral.dma_peripheral(),
                    rx_descriptors,
                    tx_descriptors,
                    chunk_size,
                )
            }
        }

        /// Create a new Mem2Mem instance.
        ///
        /// # Safety
        ///
        /// You must ensure that your not using DMA for the same peripheral and
        /// that your the only one using the DmaPeripheral.
        pub unsafe fn new_unsafe(
            channel: Channel<'d, C, MODE>,
            peripheral: DmaPeripheral,
            rx_descriptors: &'static mut [DmaDescriptor],
            tx_descriptors: &'static mut [DmaDescriptor],
            chunk_size: usize,
        ) -> Result<Self, DmaError> {
            if !(1..=4092).contains(&chunk_size) {
                return Err(DmaError::InvalidChunkSize);
            }
            if tx_descriptors.is_empty() || rx_descriptors.is_empty() {
                return Err(DmaError::OutOfDescriptors);
            }
            Ok(Mem2Mem {
                channel,
                peripheral,
                rx_chain: DescriptorChain::new_with_chunk_size(rx_descriptors, chunk_size),
                tx_chain: DescriptorChain::new_with_chunk_size(tx_descriptors, chunk_size),
            })
        }

        /// Start a memory to memory transfer.
        pub fn start_transfer<'t, TXBUF, RXBUF>(
            &mut self,
            rx_buffer: &'t mut RXBUF,
            tx_buffer: &'t TXBUF,
        ) -> Result<DmaTransferRx<'_, Self>, DmaError>
        where
            TXBUF: ReadBuffer,
            RXBUF: WriteBuffer,
        {
            let (tx_ptr, tx_len) = unsafe { tx_buffer.read_buffer() };
            let (rx_ptr, rx_len) = unsafe { rx_buffer.write_buffer() };
            self.tx_chain.fill_for_tx(false, tx_ptr, tx_len)?;
            self.rx_chain.fill_for_rx(false, rx_ptr, rx_len)?;
            unsafe {
                self.channel
                    .tx
                    .prepare_transfer_without_start(self.peripheral, &self.tx_chain)?;
                self.channel
                    .rx
                    .prepare_transfer_without_start(self.peripheral, &self.rx_chain)?;
                self.channel.rx.set_mem2mem_mode(true);
            }
            #[cfg(esp32s3)]
            {
                let align = match unsafe { crate::soc::cache_get_dcache_line_size() } {
                    16 => DmaExtMemBKSize::Size16,
                    32 => DmaExtMemBKSize::Size32,
                    64 => DmaExtMemBKSize::Size64,
                    _ => panic!("unsupported cache line size"),
                };
                if crate::soc::is_valid_psram_address(tx_ptr as usize) {
                    self.channel.tx.set_ext_mem_block_size(align);
                }
                if crate::soc::is_valid_psram_address(rx_ptr as usize) {
                    self.channel.rx.set_ext_mem_block_size(align);
                }
            }
            self.channel.tx.start_transfer()?;
            self.channel.rx.start_transfer()?;
            Ok(DmaTransferRx::new(self))
        }
    }

    impl<'d, C, MODE> DmaSupport for Mem2Mem<'d, C, MODE>
    where
        C: DmaChannel,
        MODE: crate::Mode,
    {
        fn peripheral_wait_dma(&mut self, _is_rx: bool, _is_tx: bool) {
            while !self.channel.rx.is_done() {}
        }

        fn peripheral_dma_stop(&mut self) {
            unreachable!("unsupported")
        }
    }

    impl<'d, C, MODE> DmaSupportRx for Mem2Mem<'d, C, MODE>
    where
        C: DmaChannel,
        MODE: crate::Mode,
    {
        type RX = ChannelRx<'d, C>;

        fn rx(&mut self) -> &mut Self::RX {
            &mut self.channel.rx
        }

        fn chain(&mut self) -> &mut DescriptorChain {
            &mut self.tx_chain
        }
    }
}
