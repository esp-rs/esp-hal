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
//! ## Usage
//! This module implements DMA channels, such as `channel0`, `channel1` and so
//! on. Each channel struct implements the `ChannelTypes` trait, which provides
//! associated types for peripheral configuration.
//! <em>PS: Note that the number of DMA channels is chip-specific.</em>

use crate::{
    dma::*,
    peripheral::PeripheralRef,
    system::{Peripheral, PeripheralClockControl},
};

#[non_exhaustive]
pub struct Channel<const N: u8> {}

impl<const N: u8> crate::private::Sealed for Channel<N> {}

#[doc(hidden)]
#[non_exhaustive]
pub struct ChannelInterruptBinder<const N: u8> {}

impl<const N: u8> crate::private::Sealed for ChannelInterruptBinder<N> {}

impl<const N: u8> Channel<N> {
    #[inline(always)]
    fn ch() -> &'static crate::peripherals::dma::ch::CH {
        let dma = unsafe { &*crate::peripherals::DMA::PTR };
        dma.ch(N as usize)
    }

    #[cfg(any(esp32c2, esp32c3))]
    #[inline(always)]
    fn in_int() -> &'static crate::peripherals::dma::int_ch::INT_CH {
        let dma = unsafe { &*crate::peripherals::DMA::PTR };
        dma.int_ch(N as usize)
    }
    #[inline(always)]
    #[cfg(any(esp32c6, esp32h2))]
    fn in_int() -> &'static crate::peripherals::dma::in_int_ch::IN_INT_CH {
        let dma = unsafe { &*crate::peripherals::DMA::PTR };
        dma.in_int_ch(N as usize)
    }
    #[cfg(esp32s3)]
    #[inline(always)]
    fn in_int() -> &'static crate::peripherals::dma::ch::in_int::IN_INT {
        let dma = unsafe { &*crate::peripherals::DMA::PTR };
        dma.ch(N as usize).in_int()
    }

    #[cfg(any(esp32c2, esp32c3))]
    #[inline(always)]
    fn out_int() -> &'static crate::peripherals::dma::int_ch::INT_CH {
        let dma = unsafe { &*crate::peripherals::DMA::PTR };
        dma.int_ch(N as usize)
    }
    #[inline(always)]
    #[cfg(any(esp32c6, esp32h2))]
    fn out_int() -> &'static crate::peripherals::dma::out_int_ch::OUT_INT_CH {
        let dma = unsafe { &*crate::peripherals::DMA::PTR };
        dma.out_int_ch(N as usize)
    }
    #[cfg(esp32s3)]
    #[inline(always)]
    fn out_int() -> &'static crate::peripherals::dma::ch::out_int::OUT_INT {
        let dma = unsafe { &*crate::peripherals::DMA::PTR };
        dma.ch(N as usize).out_int()
    }
}

impl<const N: u8> RegisterAccess for Channel<N> {
    fn init_channel() {
        // nothing special to be done here
    }

    #[cfg(gdma)]
    fn set_mem2mem_mode(value: bool) {
        Self::ch()
            .in_conf0()
            .modify(|_, w| w.mem_trans_en().bit(value));
    }

    #[cfg(esp32s3)]
    fn set_out_ext_mem_block_size(size: DmaExtMemBKSize) {
        Self::ch()
            .out_conf1()
            .modify(|_, w| unsafe { w.out_ext_mem_bk_size().bits(size as u8) });
    }

    fn set_out_burstmode(burst_mode: bool) {
        Self::ch().out_conf0().modify(|_, w| {
            w.out_data_burst_en()
                .bit(burst_mode)
                .outdscr_burst_en()
                .bit(burst_mode)
        });
    }

    fn set_out_priority(priority: DmaPriority) {
        Self::ch()
            .out_pri()
            .write(|w| unsafe { w.tx_pri().bits(priority as u8) });
    }

    fn clear_out_interrupts() {
        #[cfg(not(esp32s3))]
        Self::out_int().clr().write(|w| {
            w.out_eof().clear_bit_by_one();
            w.out_dscr_err().clear_bit_by_one();
            w.out_done().clear_bit_by_one();
            w.out_total_eof().clear_bit_by_one();
            w.outfifo_ovf().clear_bit_by_one();
            w.outfifo_udf().clear_bit_by_one()
        });

        #[cfg(esp32s3)]
        Self::out_int().clr().write(|w| {
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

    fn reset_out() {
        let conf0 = Self::ch().out_conf0();
        conf0.modify(|_, w| w.out_rst().set_bit());
        conf0.modify(|_, w| w.out_rst().clear_bit());
    }

    fn set_out_descriptors(address: u32) {
        Self::ch()
            .out_link()
            .modify(|_, w| unsafe { w.outlink_addr().bits(address) });
    }

    fn has_out_descriptor_error() -> bool {
        Self::out_int().raw().read().out_dscr_err().bit()
    }

    fn set_out_peripheral(peripheral: u8) {
        Self::ch()
            .out_peri_sel()
            .modify(|_, w| unsafe { w.peri_out_sel().bits(peripheral) });
    }

    fn start_out() {
        Self::ch()
            .out_link()
            .modify(|_, w| w.outlink_start().set_bit());
    }

    fn clear_ch_out_done() {
        Self::out_int()
            .clr()
            .write(|w| w.out_done().clear_bit_by_one());
    }

    fn is_ch_out_done_set() -> bool {
        Self::out_int().raw().read().out_done().bit()
    }

    fn listen_ch_out_done() {
        Self::out_int().ena().modify(|_, w| w.out_done().set_bit())
    }

    fn unlisten_ch_out_done() {
        Self::out_int()
            .ena()
            .modify(|_, w| w.out_done().clear_bit())
    }

    fn is_listening_ch_out_done() -> bool {
        Self::out_int().ena().read().out_done().bit()
    }

    fn is_out_done() -> bool {
        Self::out_int().raw().read().out_total_eof().bit()
    }

    fn last_out_dscr_address() -> usize {
        Self::ch()
            .out_eof_des_addr()
            .read()
            .out_eof_des_addr()
            .bits() as _
    }

    fn is_out_eof_interrupt_set() -> bool {
        Self::out_int().raw().read().out_eof().bit()
    }

    fn reset_out_eof_interrupt() {
        Self::out_int()
            .clr()
            .write(|w| w.out_eof().clear_bit_by_one());
    }

    #[cfg(esp32s3)]
    fn set_in_ext_mem_block_size(size: DmaExtMemBKSize) {
        Self::ch()
            .in_conf1()
            .modify(|_, w| unsafe { w.in_ext_mem_bk_size().bits(size as u8) });
    }

    fn set_in_burstmode(burst_mode: bool) {
        Self::ch().in_conf0().modify(|_, w| {
            w.in_data_burst_en()
                .bit(burst_mode)
                .indscr_burst_en()
                .bit(burst_mode)
        });
    }

    fn set_in_priority(priority: DmaPriority) {
        Self::ch()
            .in_pri()
            .write(|w| unsafe { w.rx_pri().bits(priority as u8) });
    }

    fn clear_in_interrupts() {
        #[cfg(not(esp32s3))]
        Self::in_int().clr().write(|w| {
            w.in_suc_eof().clear_bit_by_one();
            w.in_err_eof().clear_bit_by_one();
            w.in_dscr_err().clear_bit_by_one();
            w.in_dscr_empty().clear_bit_by_one();
            w.in_done().clear_bit_by_one();
            w.infifo_ovf().clear_bit_by_one();
            w.infifo_udf().clear_bit_by_one()
        });

        #[cfg(esp32s3)]
        Self::in_int().clr().write(|w| {
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

    fn reset_in() {
        let conf0 = Self::ch().in_conf0();
        conf0.modify(|_, w| w.in_rst().set_bit());
        conf0.modify(|_, w| w.in_rst().clear_bit());
    }

    fn set_in_descriptors(address: u32) {
        Self::ch()
            .in_link()
            .modify(|_, w| unsafe { w.inlink_addr().bits(address) });
    }

    fn has_in_descriptor_error() -> bool {
        Self::in_int().raw().read().in_dscr_err().bit()
    }

    fn has_in_descriptor_error_dscr_empty() -> bool {
        Self::in_int().raw().read().in_dscr_empty().bit()
    }

    fn has_in_descriptor_error_err_eof() -> bool {
        Self::in_int().raw().read().in_err_eof().bit()
    }

    fn set_in_peripheral(peripheral: u8) {
        Self::ch()
            .in_peri_sel()
            .modify(|_, w| unsafe { w.peri_in_sel().bits(peripheral) });
    }

    fn start_in() {
        Self::ch()
            .in_link()
            .modify(|_, w| w.inlink_start().set_bit());
    }

    fn is_in_done() -> bool {
        Self::in_int().raw().read().in_suc_eof().bit()
    }

    fn is_listening_in_eof() -> bool {
        Self::in_int().ena().read().in_suc_eof().bit_is_set()
    }

    fn is_listening_out_eof() -> bool {
        Self::out_int().ena().read().out_total_eof().bit_is_set()
    }

    fn listen_in_eof() {
        Self::in_int().ena().modify(|_, w| w.in_suc_eof().set_bit());
    }

    fn listen_out_eof() {
        Self::out_int()
            .ena()
            .modify(|_, w| w.out_total_eof().set_bit());
    }

    fn unlisten_in_eof() {
        Self::in_int()
            .ena()
            .modify(|_, w| w.in_suc_eof().clear_bit());
    }

    fn unlisten_out_eof() {
        Self::out_int()
            .ena()
            .modify(|_, w| w.out_total_eof().clear_bit());
    }

    fn listen_ch_in_done() {
        Self::in_int().ena().modify(|_, w| w.in_done().set_bit())
    }

    fn clear_ch_in_done() {
        Self::in_int()
            .clr()
            .write(|w| w.in_done().clear_bit_by_one());
    }

    fn is_ch_in_done_set() -> bool {
        Self::in_int().raw().read().in_done().bit()
    }

    fn unlisten_ch_in_done() {
        Self::in_int().ena().modify(|_, w| w.in_done().clear_bit());
    }

    fn is_listening_ch_in_done() -> bool {
        Self::in_int().ena().read().in_done().bit()
    }

    fn listen_in_descriptor_error() {
        Self::in_int()
            .ena()
            .modify(|_, w| w.in_dscr_err().set_bit())
    }

    fn unlisten_in_descriptor_error() {
        Self::in_int()
            .ena()
            .modify(|_, w| w.in_dscr_err().clear_bit())
    }

    fn is_listening_in_descriptor_error() -> bool {
        Self::in_int().ena().read().in_dscr_err().bit()
    }

    fn listen_in_descriptor_error_dscr_empty() {
        Self::in_int()
            .ena()
            .modify(|_, w| w.in_dscr_empty().set_bit())
    }

    fn unlisten_in_descriptor_error_dscr_empty() {
        Self::in_int()
            .ena()
            .modify(|_, w| w.in_dscr_empty().clear_bit())
    }

    fn is_listening_in_descriptor_error_dscr_empty() -> bool {
        Self::in_int().ena().read().in_dscr_empty().bit()
    }

    fn listen_in_descriptor_error_err_eof() {
        Self::in_int().ena().modify(|_, w| w.in_err_eof().set_bit())
    }

    fn unlisten_in_descriptor_error_err_eof() {
        Self::in_int()
            .ena()
            .modify(|_, w| w.in_err_eof().clear_bit())
    }

    fn is_listening_in_descriptor_error_err_eof() -> bool {
        Self::in_int().ena().read().in_err_eof().bit()
    }

    fn listen_out_descriptor_error() {
        Self::out_int()
            .ena()
            .modify(|_, w| w.out_dscr_err().set_bit())
    }

    fn unlisten_out_descriptor_error() {
        Self::out_int()
            .ena()
            .modify(|_, w| w.out_dscr_err().clear_bit())
    }

    fn is_listening_out_descriptor_error() -> bool {
        Self::out_int().ena().read().out_dscr_err().bit()
    }
}

#[non_exhaustive]
#[doc(hidden)]
pub struct ChannelTxImpl<const N: u8> {}

#[cfg(feature = "async")]
use embassy_sync::waitqueue::AtomicWaker;

#[cfg(feature = "async")]
const NEW_WAKER: AtomicWaker = AtomicWaker::new();

#[cfg(feature = "async")]
static TX_WAKER: [AtomicWaker; CHANNEL_COUNT] = [NEW_WAKER; CHANNEL_COUNT];

#[cfg(feature = "async")]
static RX_WAKER: [AtomicWaker; CHANNEL_COUNT] = [NEW_WAKER; CHANNEL_COUNT];

impl<const N: u8> crate::private::Sealed for ChannelTxImpl<N> {}

impl<const N: u8> TxChannel<Channel<N>> for ChannelTxImpl<N> {
    #[cfg(feature = "async")]
    fn waker() -> &'static AtomicWaker {
        &TX_WAKER[N as usize]
    }
}

#[non_exhaustive]
#[doc(hidden)]
pub struct ChannelRxImpl<const N: u8> {}

impl<const N: u8> crate::private::Sealed for ChannelRxImpl<N> {}

impl<const N: u8> RxChannel<Channel<N>> for ChannelRxImpl<N> {
    #[cfg(feature = "async")]
    fn waker() -> &'static AtomicWaker {
        &RX_WAKER[N as usize]
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
#[cfg(esp32s3)]
impl<const N: u8> Spi3Peripheral for SuitablePeripheral<N> {}
impl<const N: u8> I2sPeripheral for SuitablePeripheral<N> {}
impl<const N: u8> I2s0Peripheral for SuitablePeripheral<N> {}
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
            #[doc(hidden)]
            pub type [<Channel $num>] = Channel<$num>;

            #[doc(hidden)]
            pub type [<Channel $num TxImpl>] = ChannelTxImpl<$num>;

            #[doc(hidden)]
            pub type [<Channel $num RxImpl>] = ChannelRxImpl<$num>;

            #[doc(hidden)]
            pub type [<ChannelCreator $num>] = ChannelCreator<$num>;

            #[doc(hidden)]
            pub type [<Channel $num InterruptBinder>] = ChannelInterruptBinder<$num>;

            impl InterruptBinder for ChannelInterruptBinder<$num> {
                fn set_isr(handler: $crate::interrupt::InterruptHandler) {
                    let mut dma = unsafe { crate::peripherals::DMA::steal() };
                    $(
                        dma.[< bind_ $interrupt:lower _interrupt >](handler.handler());
                        $crate::interrupt::enable($crate::peripherals::Interrupt::$interrupt, handler.priority()).unwrap();
                    )*
                }
            }

            impl ChannelTypes for Channel<$num> {
                type Binder = ChannelInterruptBinder<$num>;
            }

            /// A description of a GDMA channel
            #[non_exhaustive]
            pub struct [<DmaChannel $num>] {}

            impl crate::private::Sealed for [<DmaChannel $num>] {}

            impl DmaChannel for [<DmaChannel $num>] {
                type Channel = Channel<$num>;
                type Rx = ChannelRxImpl<$num>;
                type Tx = ChannelTxImpl<$num>;
                type P = SuitablePeripheral<$num>;
            }

            impl ChannelCreator<$num> {
                /// Configure the channel for use with blocking APIs
                ///
                /// Descriptors should be sized as `(CHUNK_SIZE + 4091) / 4092`. I.e., to
                /// transfer buffers of size `1..=4092`, you need 1 descriptor.
                pub fn configure<'a>(
                    self,
                    burst_mode: bool,
                    priority: DmaPriority,
                ) -> crate::dma::Channel<'a, [<DmaChannel $num>], crate::Blocking> {
                    let mut tx_impl = ChannelTxImpl {};
                    tx_impl.init(burst_mode, priority);

                    let mut rx_impl = ChannelRxImpl {};
                    rx_impl.init(burst_mode, priority);

                    crate::dma::Channel {
                        tx: ChannelTx::new(tx_impl, burst_mode),
                        rx: ChannelRx::new(rx_impl, burst_mode),
                        phantom: PhantomData,
                    }
                }

                /// Configure the channel for use with async APIs
                ///
                /// Descriptors should be sized as `(CHUNK_SIZE + 4091) / 4092`. I.e., to
                /// transfer buffers of size `1..=4092`, you need 1 descriptor.
                #[cfg(feature = "async")]
                pub fn configure_for_async<'a>(
                    self,
                    burst_mode: bool,
                    priority: DmaPriority,
                ) -> crate::dma::Channel<'a, [<DmaChannel $num>], $crate::Async> {
                    let mut tx_impl = ChannelTxImpl {};
                    tx_impl.init(burst_mode, priority);

                    let mut rx_impl = ChannelRxImpl {};
                    rx_impl.init(burst_mode, priority);

                    <Channel<$num> as ChannelTypes>::Binder::set_isr($async_handler);

                    crate::dma::Channel {
                        tx: ChannelTx::new(tx_impl, burst_mode),
                        rx: ChannelRx::new(rx_impl, burst_mode),
                        phantom: PhantomData,
                    }
                }
            }
        }
    };
}

cfg_if::cfg_if! {
    if #[cfg(esp32c2)] {
        #[cfg(feature = "async")]
        const CHANNEL_COUNT: usize = 1;
        impl_channel!(0, super::asynch::interrupt::interrupt_handler_ch0, DMA_CH0);
    } else if #[cfg(esp32c3)] {
        #[cfg(feature = "async")]
        const CHANNEL_COUNT: usize = 3;
        impl_channel!(0, super::asynch::interrupt::interrupt_handler_ch0, DMA_CH0);
        impl_channel!(1, super::asynch::interrupt::interrupt_handler_ch1, DMA_CH1);
        impl_channel!(2, super::asynch::interrupt::interrupt_handler_ch2, DMA_CH2);
    } else if #[cfg(any(esp32c6, esp32h2))] {
        #[cfg(feature = "async")]
        const CHANNEL_COUNT: usize = 3;
        impl_channel!(0, super::asynch::interrupt::interrupt_handler_ch0, DMA_IN_CH0, DMA_OUT_CH0);
        impl_channel!(1, super::asynch::interrupt::interrupt_handler_ch1, DMA_IN_CH1, DMA_OUT_CH1);
        impl_channel!(2, super::asynch::interrupt::interrupt_handler_ch2, DMA_IN_CH2, DMA_OUT_CH2);
    } else if #[cfg(esp32s3)] {
        #[cfg(feature = "async")]
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
        RxPrivate,
        TxPrivate,
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
        tx_chain: DescriptorChain,
        rx_chain: DescriptorChain,
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
            tx_descriptors: &'static mut [DmaDescriptor],
            rx_descriptors: &'static mut [DmaDescriptor],
        ) -> Result<Self, DmaError> {
            unsafe {
                Self::new_unsafe(
                    channel,
                    peripheral.dma_peripheral(),
                    tx_descriptors,
                    rx_descriptors,
                    crate::dma::CHUNK_SIZE,
                )
            }
        }

        /// Create a new Mem2Mem instance with specific chunk size.
        pub fn new_with_chunk_size(
            channel: Channel<'d, C, MODE>,
            peripheral: impl DmaEligible,
            tx_descriptors: &'static mut [DmaDescriptor],
            rx_descriptors: &'static mut [DmaDescriptor],
            chunk_size: usize,
        ) -> Result<Self, DmaError> {
            unsafe {
                Self::new_unsafe(
                    channel,
                    peripheral.dma_peripheral(),
                    tx_descriptors,
                    rx_descriptors,
                    chunk_size,
                )
            }
        }

        /// Create a new Mem2Mem instance.
        ///
        /// # Safety
        ///
        /// You must insure that your not using DMA for the same peripheral and
        /// that your the only one using the DmaPeripheral.
        pub unsafe fn new_unsafe(
            mut channel: Channel<'d, C, MODE>,
            peripheral: DmaPeripheral,
            tx_descriptors: &'static mut [DmaDescriptor],
            rx_descriptors: &'static mut [DmaDescriptor],
            chunk_size: usize,
        ) -> Result<Self, DmaError> {
            if !(1..=4092).contains(&chunk_size) {
                return Err(DmaError::InvalidChunkSize);
            }
            if tx_descriptors.is_empty() || rx_descriptors.is_empty() {
                return Err(DmaError::OutOfDescriptors);
            }
            channel.tx.init_channel();
            channel.rx.init_channel();
            Ok(Mem2Mem {
                channel,
                peripheral,
                tx_chain: DescriptorChain::new_with_chunk_size(tx_descriptors, chunk_size),
                rx_chain: DescriptorChain::new_with_chunk_size(rx_descriptors, chunk_size),
            })
        }

        /// Start a memory to memory transfer.
        pub fn start_transfer<'t, TXBUF, RXBUF>(
            &mut self,
            tx_buffer: &'t TXBUF,
            rx_buffer: &'t mut RXBUF,
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
                if crate::soc::is_valid_psram_address(tx_ptr as u32) {
                    self.channel.tx.set_ext_mem_block_size(align);
                }
                if crate::soc::is_valid_psram_address(rx_ptr as u32) {
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
        fn peripheral_wait_dma(&mut self, _is_tx: bool, _is_rx: bool) {
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
