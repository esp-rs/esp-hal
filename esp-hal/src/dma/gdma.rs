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

macro_rules! impl_channel {
    ($num: literal, $async_handler: path, $($interrupt: ident),* ) => {
        paste::paste! {
            #[non_exhaustive]
            pub struct [<Channel $num InterruptBinder>] {}

            impl InterruptBinder for [<Channel $num InterruptBinder>] {
                #[cfg(feature = "vectored")]
                fn set_isr(handler: $crate::interrupt::InterruptHandler) {
                    let mut dma = unsafe { crate::peripherals::DMA::steal() };
                    $(
                        dma.[< bind_ $interrupt:lower _interrupt >](handler.handler());
                        $crate::interrupt::enable($crate::peripherals::Interrupt::$interrupt, handler.priority()).unwrap();
                    )*
                }
            }

            #[non_exhaustive]
            pub struct [<Channel $num>] {}

            impl ChannelTypes for [<Channel $num>] {
                type P = [<SuitablePeripheral $num>];
                type Tx<'a> = ChannelTx<'a, [<Channel $num TxImpl>], [<Channel $num>]>;
                type Rx<'a> = ChannelRx<'a, [<Channel $num RxImpl>], [<Channel $num>]>;
                type Binder = [<Channel $num InterruptBinder>];
            }

            impl RegisterAccess for [<Channel $num>] {
                fn init_channel() {
                    // nothing special to be done here
                }

                fn set_out_burstmode(burst_mode: bool) {
                    let dma = unsafe { &*crate::peripherals::DMA::PTR };

                    dma.out_conf0_ch($num).modify(|_,w| {
                        w.out_data_burst_en().bit(burst_mode)
                            .outdscr_burst_en().bit(burst_mode)
                    });
                }

                fn set_out_priority(priority: DmaPriority) {
                    let dma = unsafe { &*crate::peripherals::DMA::PTR };

                    dma.out_pri_ch($num).write(|w| {
                        w.tx_pri().variant(priority as u8)
                    });
                }

                fn clear_out_interrupts() {
                    let dma = unsafe { &*crate::peripherals::DMA::PTR };

                    #[cfg(not(any(esp32c6, esp32h2, esp32s3)))]
                    dma.int_clr_ch($num).write(|w| {
                        w.out_eof()
                            .set_bit()
                            .out_dscr_err()
                            .set_bit()
                            .out_done()
                            .set_bit()
                            .out_total_eof()
                            .set_bit()
                            .outfifo_ovf()
                            .set_bit()
                            .outfifo_udf()
                            .set_bit()
                    });

                    #[cfg(any(esp32c6, esp32h2))]
                    dma.out_int_clr_ch($num).write(|w| {
                        w.out_eof()
                            .set_bit()
                            .out_dscr_err()
                            .set_bit()
                            .out_done()
                            .set_bit()
                            .out_total_eof()
                            .set_bit()
                            .outfifo_ovf()
                            .set_bit()
                            .outfifo_udf()
                            .set_bit()
                    });

                    #[cfg(esp32s3)]
                    dma.out_int_clr_ch($num).write(|w| {
                        w.out_eof()
                            .set_bit()
                            .out_dscr_err()
                            .set_bit()
                            .out_done()
                            .set_bit()
                            .out_total_eof()
                            .set_bit()
                            .outfifo_ovf_l1()
                            .set_bit()
                            .outfifo_ovf_l3()
                            .set_bit()
                            .outfifo_udf_l1()
                            .set_bit()
                            .outfifo_udf_l3()
                            .set_bit()
                    });
                }

                fn reset_out() {
                    let dma = unsafe { &*crate::peripherals::DMA::PTR };

                    dma.out_conf0_ch($num).modify(|_, w| w.out_rst().set_bit());
                    dma.out_conf0_ch($num).modify(|_, w| w.out_rst().clear_bit());
                }

                fn set_out_descriptors(address: u32) {
                    let dma = unsafe { &*crate::peripherals::DMA::PTR };

                    dma.out_link_ch($num).modify(|_, w| unsafe { w.outlink_addr().bits(address) });
                }

                fn has_out_descriptor_error() -> bool {
                    let dma = unsafe { &*crate::peripherals::DMA::PTR };

                    #[cfg(not(any(esp32c6, esp32h2, esp32s3)))]
                    let ret = dma.int_raw_ch($num).read().out_dscr_err().bit();
                    #[cfg(any(esp32c6, esp32h2, esp32s3))]
                    let ret = dma.out_int_raw_ch($num).read().out_dscr_err().bit();

                    ret
                }

                fn set_out_peripheral(peripheral: u8) {
                    let dma = unsafe { &*crate::peripherals::DMA::PTR };

                    dma.out_peri_sel_ch($num).modify(|_, w| w.peri_out_sel().variant(peripheral));
                }

                fn start_out() {
                    let dma = unsafe { &*crate::peripherals::DMA::PTR };

                    dma.out_link_ch($num).modify(|_, w| w.outlink_start().set_bit());
                }

                fn clear_ch_out_done() {
                    let dma = unsafe { &*crate::peripherals::DMA::PTR };

                    #[cfg(not(any(esp32c6, esp32h2, esp32s3)))]
                    dma.int_clr_ch($num).write(|w| w.out_done().set_bit());
                    #[cfg(any(esp32c6, esp32h2, esp32s3))]
                    dma.out_int_clr_ch($num).write(|w| w.out_done().set_bit());
                }

                fn is_ch_out_done_set() -> bool {
                    let dma = unsafe { &*crate::peripherals::DMA::PTR };

                    #[cfg(not(any(esp32c6, esp32h2, esp32s3)))]
                    let ret = dma.int_raw_ch($num).read().out_done().bit();
                    #[cfg(any(esp32c6, esp32h2, esp32s3))]
                    let ret = dma.out_int_raw_ch($num).read().out_done().bit();

                    ret
                }

                fn listen_ch_out_done() {
                    let dma = unsafe { &*crate::peripherals::DMA::PTR };
                    cfg_if::cfg_if! {
                        if #[cfg(any(esp32c6, esp32h2, esp32s3))] {
                            dma.out_int_ena_ch($num).modify(|_, w| w.out_done().set_bit())
                        } else {
                            dma.int_ena_ch($num).modify(|_, w| w.out_done().set_bit())
                        }
                    }
                }

                fn unlisten_ch_out_done() {
                    let dma = unsafe { &*crate::peripherals::DMA::PTR };
                    cfg_if::cfg_if! {
                        if #[cfg(any(esp32c6, esp32h2, esp32s3))] {
                            dma.out_int_ena_ch($num).modify(|_, w| w.out_done().clear_bit())
                        } else {
                            dma.int_ena_ch($num).modify(|_, w| w.out_done().clear_bit())
                        }
                    }
                }

                fn is_listening_ch_out_done() -> bool {
                    let dma = unsafe { &*crate::peripherals::DMA::PTR };
                    cfg_if::cfg_if! {
                        if #[cfg(any(esp32c6, esp32h2, esp32s3))] {
                            dma.out_int_ena_ch($num).read().out_done().bit()
                        } else {
                            dma.int_ena_ch($num).read().out_done().bit()
                        }
                    }
                }

                fn is_out_done() -> bool {
                    let dma = unsafe { &*crate::peripherals::DMA::PTR };

                    #[cfg(not(any(esp32c6, esp32h2, esp32s3)))]
                    let ret = dma.int_raw_ch($num).read().out_total_eof().bit();
                    #[cfg(any(esp32c6, esp32h2, esp32s3))]
                    let ret = dma.out_int_raw_ch($num).read().out_total_eof().bit();

                    ret
                }

                fn last_out_dscr_address() -> usize {
                    let dma = unsafe { &*crate::peripherals::DMA::PTR };
                    dma.out_eof_des_addr_ch($num).read().out_eof_des_addr().bits() as usize
                }

                fn is_out_eof_interrupt_set() -> bool {
                    let dma = unsafe { &*crate::peripherals::DMA::PTR };

                    #[cfg(not(any(esp32c6, esp32h2, esp32s3)))]
                    let ret = dma.int_raw_ch($num).read().out_eof().bit();
                    #[cfg(any(esp32c6, esp32h2, esp32s3))]
                    let ret = dma.out_int_raw_ch($num).read().out_eof().bit();

                    ret
                }

                fn reset_out_eof_interrupt() {
                    let dma = unsafe { &*crate::peripherals::DMA::PTR };

                    #[cfg(not(any(esp32c6, esp32h2, esp32s3)))]
                    dma.int_clr_ch($num).write(|w| {
                        w.out_eof()
                            .set_bit()
                    });

                    #[cfg(any(esp32c6, esp32h2, esp32s3))]
                    dma.out_int_clr_ch($num).write(|w| {
                        w.out_eof()
                            .set_bit()
                    });
                }

                fn set_in_burstmode(burst_mode: bool) {
                    let dma = unsafe { &*crate::peripherals::DMA::PTR };

                    dma.in_conf0_ch($num).modify(|_,w| {
                        w.in_data_burst_en().bit(burst_mode).indscr_burst_en().bit(burst_mode)
                    });
                }

                fn set_in_priority(priority: DmaPriority) {
                    let dma = unsafe { &*crate::peripherals::DMA::PTR };

                    dma.in_pri_ch($num).write(|w| {
                        w.rx_pri().variant(priority as u8)
                    });
                }

                fn clear_in_interrupts() {
                    let dma = unsafe { &*crate::peripherals::DMA::PTR };

                    #[cfg(not(any(esp32c6, esp32h2, esp32s3)))]
                    dma.int_clr_ch($num).write(|w| {
                        w.in_suc_eof()
                            .set_bit()
                            .in_err_eof()
                            .set_bit()
                            .in_dscr_err()
                            .set_bit()
                            .in_dscr_empty()
                            .set_bit()
                            .in_done()
                            .set_bit()
                            .infifo_ovf()
                            .set_bit()
                            .infifo_udf()
                            .set_bit()
                    });

                    #[cfg(any(esp32c6, esp32h2))]
                    dma.in_int_clr_ch($num).write(|w| {
                        w.in_suc_eof()
                            .set_bit()
                            .in_err_eof()
                            .set_bit()
                            .in_dscr_err()
                            .set_bit()
                            .in_dscr_empty()
                            .set_bit()
                            .in_done()
                            .set_bit()
                            .infifo_ovf()
                            .set_bit()
                            .infifo_udf()
                            .set_bit()
                    });

                    #[cfg(esp32s3)]
                    dma.in_int_clr_ch($num).write(|w| {
                        w.in_suc_eof()
                            .set_bit()
                            .in_err_eof()
                            .set_bit()
                            .in_dscr_err()
                            .set_bit()
                            .in_dscr_empty()
                            .set_bit()
                            .in_done()
                            .set_bit()
                            .infifo_ovf_l1()
                            .set_bit()
                            .infifo_ovf_l3()
                            .set_bit()
                            .infifo_udf_l1()
                            .set_bit()
                            .infifo_udf_l3()
                            .set_bit()
                    });
                }

                fn reset_in() {
                    let dma = unsafe { &*crate::peripherals::DMA::PTR };

                    dma.in_conf0_ch($num).modify(|_, w| w.in_rst().set_bit());
                    dma.in_conf0_ch($num).modify(|_, w| w.in_rst().clear_bit());
                }

                fn set_in_descriptors(address: u32) {
                    let dma = unsafe { &*crate::peripherals::DMA::PTR };

                    dma.in_link_ch($num).modify(|_, w| unsafe { w.inlink_addr().bits(address) });
                }

                fn has_in_descriptor_error() -> bool {
                    let dma = unsafe { &*crate::peripherals::DMA::PTR };

                    #[cfg(not(any(esp32c6, esp32h2, esp32s3)))]
                    let ret = dma.int_raw_ch($num).read().in_dscr_err().bit();
                    #[cfg(any(esp32c6, esp32h2, esp32s3))]
                    let ret = dma.in_int_raw_ch($num).read().in_dscr_err().bit();

                    ret
                }

                fn has_in_descriptor_error_dscr_empty() -> bool {
                    let dma = unsafe { &*crate::peripherals::DMA::PTR };

                    #[cfg(not(any(esp32c6, esp32h2, esp32s3)))]
                    let ret = dma.int_raw_ch($num).read().in_dscr_empty().bit();
                    #[cfg(any(esp32c6, esp32h2, esp32s3))]
                    let ret = dma.in_int_raw_ch($num).read().in_dscr_empty().bit();

                    ret
                }

                fn has_in_descriptor_error_err_eof() -> bool {
                    let dma = unsafe { &*crate::peripherals::DMA::PTR };

                    #[cfg(not(any(esp32c6, esp32h2, esp32s3)))]
                    let ret = dma.int_raw_ch($num).read().in_err_eof().bit();
                    #[cfg(any(esp32c6, esp32h2, esp32s3))]
                    let ret = dma.in_int_raw_ch($num).read().in_err_eof().bit();

                    ret
                }

                fn set_in_peripheral(peripheral: u8) {
                    let dma = unsafe { &*crate::peripherals::DMA::PTR };

                    dma.in_peri_sel_ch($num).modify(|_, w| w.peri_in_sel().variant(peripheral));
                }

                fn start_in() {
                    let dma = unsafe { &*crate::peripherals::DMA::PTR };

                    dma.in_link_ch($num).modify(|_, w| w.inlink_start().set_bit());
                }

                fn is_in_done() -> bool {
                    let dma = unsafe { &*crate::peripherals::DMA::PTR };

                    #[cfg(not(any(esp32c6, esp32h2, esp32s3)))]
                    let ret = dma.int_raw_ch($num).read().in_suc_eof().bit();
                    #[cfg(any(esp32c6, esp32h2, esp32s3))]
                    let ret = dma.in_int_raw_ch($num).read().in_suc_eof().bit();

                    ret
                }

                fn last_in_dscr_address() -> usize {
                    let dma = unsafe { &*crate::peripherals::DMA::PTR };
                    dma.in_dscr_bf0_ch($num).read().inlink_dscr_bf0().bits() as usize
                }

                fn is_listening_in_eof() -> bool {
                    let dma = unsafe { &*crate::peripherals::DMA::PTR };
                    cfg_if::cfg_if! {
                        if #[cfg(any(esp32c6, esp32h2, esp32s3))] {
                            dma.in_int_ena_ch($num).read().in_suc_eof().bit_is_set()
                        } else {
                            dma.int_ena_ch($num).read().in_suc_eof().bit_is_set()
                        }
                    }
                }

                fn is_listening_out_eof() -> bool {
                    let dma = unsafe { &*crate::peripherals::DMA::PTR };
                    cfg_if::cfg_if! {
                        if #[cfg(any(esp32c6, esp32h2, esp32s3))] {
                            dma.out_int_ena_ch($num).read().out_total_eof().bit_is_set()
                        } else {
                            dma.int_ena_ch($num).read().out_total_eof().bit_is_set()
                        }
                    }
                }

                fn listen_in_eof() {
                    let dma = unsafe { &*crate::peripherals::DMA::PTR };
                    cfg_if::cfg_if! {
                        if #[cfg(any(esp32c6, esp32h2, esp32s3))] {
                            dma.in_int_ena_ch($num).modify(|_, w| w.in_suc_eof().set_bit())
                        } else {
                            dma.int_ena_ch($num).modify(|_, w| w.in_suc_eof().set_bit())
                        }
                    }
                }

                fn listen_out_eof() {
                    let dma = unsafe { &*crate::peripherals::DMA::PTR };
                    cfg_if::cfg_if! {
                        if #[cfg(any(esp32c6, esp32h2, esp32s3))] {
                            dma.out_int_ena_ch($num).modify(|_, w| w.out_total_eof().set_bit())
                        } else {
                            dma.int_ena_ch($num).modify(|_, w| w.out_total_eof().set_bit())
                        }
                    }
                }

                fn unlisten_in_eof() {
                    let dma = unsafe { &*crate::peripherals::DMA::PTR };
                    cfg_if::cfg_if! {
                        if #[cfg(any(esp32c6, esp32h2, esp32s3))] {
                            dma.in_int_ena_ch($num).modify(|_, w| w.in_suc_eof().clear_bit())
                        } else {
                            dma.int_ena_ch($num).modify(|_, w| w.in_suc_eof().clear_bit())
                        }
                    }
                }

                fn unlisten_out_eof() {
                    let dma = unsafe { &*crate::peripherals::DMA::PTR };
                    cfg_if::cfg_if! {
                        if #[cfg(any(esp32c6, esp32h2, esp32s3))] {
                            dma.out_int_ena_ch($num).modify(|_, w| w.out_total_eof().clear_bit())
                        } else {
                            dma.int_ena_ch($num).modify(|_, w| w.out_total_eof().clear_bit())
                        }
                    }
                }

                fn listen_ch_in_done(){
                    let dma = unsafe { &*crate::peripherals::DMA::PTR };
                    cfg_if::cfg_if! {
                        if #[cfg(any(esp32c6, esp32h2, esp32s3))] {
                            dma.in_int_ena_ch($num).modify(|_, w| w.in_done().set_bit())
                        } else {
                            dma.int_ena_ch($num).modify(|_, w| w.in_done().set_bit())
                        }
                    }
                }

                fn clear_ch_in_done(){
                    let dma = unsafe { &*crate::peripherals::DMA::PTR };

                    #[cfg(not(any(esp32c6, esp32h2, esp32s3)))]
                    dma.int_clr_ch($num).write(|w| w.in_done().set_bit());
                    #[cfg(any(esp32c6, esp32h2, esp32s3))]
                    dma.in_int_clr_ch($num).write(|w| w.in_done().set_bit());
                }

                fn is_ch_in_done_set() -> bool {
                    let dma = unsafe { &*crate::peripherals::DMA::PTR };

                    #[cfg(not(any(esp32c6, esp32h2, esp32s3)))]
                    let ret = dma.int_raw_ch($num).read().in_done().bit();
                    #[cfg(any(esp32c6, esp32h2, esp32s3))]
                    let ret = dma.in_int_raw_ch($num).read().in_done().bit();

                    ret
                }

                fn unlisten_ch_in_done() {
                    let dma = unsafe { &*crate::peripherals::DMA::PTR };
                    cfg_if::cfg_if! {
                        if #[cfg(any(esp32c6, esp32h2, esp32s3))] {
                            dma.in_int_ena_ch($num).modify(|_, w| w.in_done().clear_bit())
                        } else {
                            dma.int_ena_ch($num).modify(|_, w| w.in_done().clear_bit())
                        }
                    }
                }

                fn is_listening_ch_in_done() -> bool {
                    let dma = unsafe { &*crate::peripherals::DMA::PTR };
                    cfg_if::cfg_if! {
                        if #[cfg(any(esp32c6, esp32h2, esp32s3))] {
                            dma.in_int_ena_ch($num).read().in_done().bit()
                        } else {
                            dma.int_ena_ch($num).read().in_done().bit()
                        }
                    }
                }
            }

            #[non_exhaustive]
            pub struct [<Channel $num TxImpl>] {}

            impl<'a> TxChannel<[<Channel $num>]> for [<Channel $num TxImpl>] {
                #[cfg(feature = "async")]
                fn waker() -> &'static embassy_sync::waitqueue::AtomicWaker {
                    static WAKER: embassy_sync::waitqueue::AtomicWaker = embassy_sync::waitqueue::AtomicWaker::new();
                    &WAKER
                }
            }

            #[non_exhaustive]
            pub struct [<Channel $num RxImpl>] {}

            impl<'a> RxChannel<[<Channel $num>]> for [<Channel $num RxImpl>] {
                #[cfg(feature = "async")]
                fn waker() -> &'static embassy_sync::waitqueue::AtomicWaker {
                    static WAKER: embassy_sync::waitqueue::AtomicWaker = embassy_sync::waitqueue::AtomicWaker::new();
                    &WAKER
                }
            }

            #[non_exhaustive]
            pub struct [<ChannelCreator $num>] {}

            impl [<ChannelCreator $num>] {
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
                ) -> Channel<'a, [<Channel $num>], $crate::Blocking> {
                    let mut tx_impl = [<Channel $num TxImpl>] {};
                    tx_impl.init(burst_mode, priority);

                    let mut rx_impl = [<Channel $num RxImpl>] {};
                    rx_impl.init(burst_mode, priority);

                    Channel {
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
                ) -> Channel<'a, [<Channel $num>], $crate::Async> {
                    let mut tx_impl = [<Channel $num TxImpl>] {};
                    tx_impl.init(burst_mode, priority);

                    let mut rx_impl = [<Channel $num RxImpl>] {};
                    rx_impl.init(burst_mode, priority);

                    <[<Channel $num>] as ChannelTypes>::Binder::set_isr($async_handler);

                    Channel {
                        tx: ChannelTx::new(tx_descriptors, tx_impl, burst_mode),
                        rx: ChannelRx::new(rx_descriptors, rx_impl, burst_mode),
                        phantom: PhantomData,
                    }
                }
            }

            #[non_exhaustive]
            pub struct [<SuitablePeripheral $num>] {}
            impl PeripheralMarker for [<SuitablePeripheral $num>] {}

            // with GDMA every channel can be used for any peripheral
            impl SpiPeripheral for [<SuitablePeripheral $num>] {}
            impl Spi2Peripheral for [<SuitablePeripheral $num>] {}
            #[cfg(esp32s3)]
            impl Spi3Peripheral for [<SuitablePeripheral $num>] {}
            impl I2sPeripheral for [<SuitablePeripheral $num>] {}
            impl I2s0Peripheral for [<SuitablePeripheral $num>] {}
            impl I2s1Peripheral for [<SuitablePeripheral $num>] {}
            #[cfg(parl_io)]
            impl ParlIoPeripheral for [<SuitablePeripheral $num>] {}
            #[cfg(aes)]
            impl AesPeripheral for [<SuitablePeripheral $num>] {}
            #[cfg(lcd_cam)]
            impl LcdCamPeripheral for [<SuitablePeripheral $num>] {}
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
    pub channel0: ChannelCreator0,
    #[cfg(not(esp32c2))]
    pub channel1: ChannelCreator1,
    #[cfg(not(esp32c2))]
    pub channel2: ChannelCreator2,
    #[cfg(esp32s3)]
    pub channel3: ChannelCreator3,
    #[cfg(esp32s3)]
    pub channel4: ChannelCreator4,
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
            channel0: ChannelCreator0 {},
            #[cfg(not(esp32c2))]
            channel1: ChannelCreator1 {},
            #[cfg(not(esp32c2))]
            channel2: ChannelCreator2 {},
            #[cfg(esp32s3)]
            channel3: ChannelCreator3 {},
            #[cfg(esp32s3)]
            channel4: ChannelCreator4 {},
        }
    }
}
