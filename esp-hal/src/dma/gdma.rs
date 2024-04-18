//! # Direct Memory Access
//!
//! ## Overview
//!
//! The GDMA (General DMA) module is a part of the DMA (Direct Memory Access)
//! driver for ESP chips. Of the Espressif chip range, every chip except of
//! `ESP32` and `ESP32-S2` uses the `GDMA` type of direct memory access.
//!
//! DMA is a hardware feature that allows data transfer between memory and
//! peripherals without involving the CPU, resulting in efficient data movement
//! and reduced CPU overhead. The `GDMA` module provides multiple DMA channels,
//! each capable of managing data transfer for various peripherals.
//!
//! This module implements DMA channels, such as `channel0`, `channel1` and so
//! on. Each channel struct implements the `ChannelTypes` trait, which provides
//! associated types for peripheral configuration.
//!
//! GDMA peripheral can be initializes using the `new` function, which requires
//! a DMA peripheral instance and a clock control reference.
//!
//! ```no_run
//! let dma = Gdma::new(peripherals.DMA);
//! ```
//!
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

    fn last_in_dscr_address() -> usize {
        Self::ch().in_dscr_bf0().read().inlink_dscr_bf0().bits() as _
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

impl<const N: u8> crate::private::Sealed for ChannelTxImpl<N> {}

impl<const N: u8> TxChannel<Channel<N>> for ChannelTxImpl<N> {
    #[cfg(feature = "async")]
    fn waker() -> &'static embassy_sync::waitqueue::AtomicWaker {
        static WAKER: embassy_sync::waitqueue::AtomicWaker =
            embassy_sync::waitqueue::AtomicWaker::new();
        &WAKER
    }
}

#[non_exhaustive]
#[doc(hidden)]
pub struct ChannelRxImpl<const N: u8> {}

impl<const N: u8> crate::private::Sealed for ChannelRxImpl<N> {}

impl<const N: u8> RxChannel<Channel<N>> for ChannelRxImpl<N> {
    #[cfg(feature = "async")]
    fn waker() -> &'static embassy_sync::waitqueue::AtomicWaker {
        static WAKER: embassy_sync::waitqueue::AtomicWaker =
            embassy_sync::waitqueue::AtomicWaker::new();
        &WAKER
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
                type P = SuitablePeripheral<$num>;
                type Tx<'a> = ChannelTx<'a, ChannelTxImpl<$num>, Channel<$num>>;
                type Rx<'a> = ChannelRx<'a, ChannelRxImpl<$num>, Channel<$num>>;
                type Binder = ChannelInterruptBinder<$num>;
            }

            impl ChannelCreator<$num> {
                /// Configure the channel for use with blocking APIs
                ///
                /// Descriptors should be sized as `(CHUNK_SIZE + 4091) / 4092`. I.e., to
                /// transfer buffers of size `1..=4092`, you need 1 descriptor.
                pub fn configure<'a>(
                    self,
                    burst_mode: bool,
                    tx_descriptors: &'a mut [DmaDescriptor],
                    rx_descriptors: &'a mut [DmaDescriptor],
                    priority: DmaPriority,
                ) -> crate::dma::Channel<'a, Channel<$num>, crate::Blocking> {
                    let mut tx_impl = ChannelTxImpl {};
                    tx_impl.init(burst_mode, priority);

                    let mut rx_impl = ChannelRxImpl {};
                    rx_impl.init(burst_mode, priority);

                    crate::dma::Channel {
                        tx: ChannelTx::new(tx_descriptors, tx_impl, burst_mode),
                        rx: ChannelRx::new(rx_descriptors, rx_impl, burst_mode),
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
                    tx_descriptors: &'a mut [DmaDescriptor],
                    rx_descriptors: &'a mut [DmaDescriptor],
                    priority: DmaPriority,
                ) -> crate::dma::Channel<'a, Channel<$num>, $crate::Async> {
                    let mut tx_impl = ChannelTxImpl {};
                    tx_impl.init(burst_mode, priority);

                    let mut rx_impl = ChannelRxImpl {};
                    rx_impl.init(burst_mode, priority);

                    <Channel<$num> as ChannelTypes>::Binder::set_isr($async_handler);

                    crate::dma::Channel {
                        tx: ChannelTx::new(tx_descriptors, tx_impl, burst_mode),
                        rx: ChannelRx::new(rx_descriptors, rx_impl, burst_mode),
                        phantom: PhantomData,
                    }
                }
            }
        }
    };
}

cfg_if::cfg_if! {
    if #[cfg(esp32c2)] {
        impl_channel!(0, super::asynch::interrupt::interrupt_handler_ch0, DMA_CH0);
    } else if #[cfg(esp32c3)] {
        impl_channel!(0, super::asynch::interrupt::interrupt_handler_ch0, DMA_CH0);
        impl_channel!(1, super::asynch::interrupt::interrupt_handler_ch1, DMA_CH1);
        impl_channel!(2, super::asynch::interrupt::interrupt_handler_ch2, DMA_CH2);
    } else if #[cfg(any(esp32c6, esp32h2))] {
        impl_channel!(0, super::asynch::interrupt::interrupt_handler_ch0, DMA_IN_CH0, DMA_OUT_CH0);
        impl_channel!(1, super::asynch::interrupt::interrupt_handler_ch1, DMA_IN_CH1, DMA_OUT_CH1);
        impl_channel!(2, super::asynch::interrupt::interrupt_handler_ch2, DMA_IN_CH2, DMA_OUT_CH2);
   } else if #[cfg(esp32s3)] {
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
