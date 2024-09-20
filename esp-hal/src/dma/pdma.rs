//! # Direct Memory Access
//!
//! ## Overview
//! The `pdma` module is part of the DMA driver of `ESP32` and `ESP32-S2`.
//!
//! This module provides efficient direct data transfer capabilities between
//! peripherals and memory without involving the CPU. It enables bidirectional
//! data transfers through DMA channels, making it particularly useful for
//! high-speed data transfers, such as [SPI] and [I2S] communication.
//!
//! [SPI]: ../spi/index.html
//! [I2S]: ../i2s/index.html

use crate::{
    dma::*,
    peripheral::PeripheralRef,
    system::{Peripheral, PeripheralClockControl},
};

macro_rules! ImplSpiChannel {
    ($num: literal) => {
        paste::paste! {
            #[non_exhaustive]
            #[doc(hidden)]
            pub struct [<Channel $num InterruptBinder>] {}

            impl $crate::private::Sealed for [<Channel $num InterruptBinder>] {}

            impl InterruptBinder for [<Channel $num InterruptBinder>] {
                fn set_isr(handler: $crate::interrupt::InterruptHandler) {
                    let mut spi = unsafe { $crate::peripherals::[< SPI $num >]::steal() };
                    spi.[< bind_spi $num _dma_interrupt>](handler.handler());
                    $crate::interrupt::enable($crate::peripherals::Interrupt::[< SPI $num _DMA >], handler.priority()).unwrap();
                }
            }

            #[doc = concat!("DMA channel suitable for SPI", $num)]
            #[non_exhaustive]
            pub struct [<Spi $num DmaChannel>] {}

            impl DmaChannel for [<Spi $num DmaChannel>] {
                type Channel = [<Spi $num DmaChannel>];
                type Rx = [<Spi $num DmaChannelRxImpl>];
                type Tx = [<Spi $num DmaChannelTxImpl>];
                type P = [<Spi $num DmaSuitablePeripheral>];
            }

            impl $crate::private::Sealed for [<Spi $num DmaChannel>] {}

            impl ChannelTypes for [<Spi $num DmaChannel>] {
                type Binder = [<Channel $num InterruptBinder>];
            }

            impl RegisterAccess for [<Spi $num DmaChannel>] {
                fn init_channel() {
                    // (only) on ESP32 we need to configure DPORT for the SPI DMA channels
                    #[cfg(esp32)]
                    {
                        let dport = unsafe { &*crate::peripherals::DPORT::PTR };

                        match $num {
                            2 => {
                                dport
                                .spi_dma_chan_sel()
                                .modify(|_, w| unsafe { w.spi2_dma_chan_sel().bits(1) });
                            },
                            3 => {
                                dport
                                .spi_dma_chan_sel()
                                .modify(|_, w| unsafe { w.spi3_dma_chan_sel().bits(2) });
                            },
                            _ => panic!("Only SPI2 and SPI3 supported"),
                        }
                    }
                }

                fn set_out_burstmode(burst_mode: bool) {
                    let spi = unsafe { &*crate::peripherals::[<SPI $num>]::PTR };
                    spi.dma_conf()
                        .modify(|_, w| w.outdscr_burst_en().bit(burst_mode));
                }

                fn set_out_priority(_priority: DmaPriority) {}

                fn clear_out_interrupts() {
                    let spi = unsafe { &*crate::peripherals::[<SPI $num>]::PTR };
                    spi.dma_int_clr().write(|w| {
                        w.out_done()
                            .clear_bit_by_one()
                            .out_eof()
                            .clear_bit_by_one()
                            .out_total_eof()
                            .clear_bit_by_one()
                            .outlink_dscr_error()
                            .clear_bit_by_one()
                    });
                }

                fn reset_out() {
                    let spi = unsafe { &*crate::peripherals::[<SPI $num>]::PTR };
                    spi.dma_conf().modify(|_, w| w.out_rst().set_bit());
                    spi.dma_conf().modify(|_, w| w.out_rst().clear_bit());
                }

                fn set_out_descriptors(address: u32) {
                    let spi = unsafe { &*crate::peripherals::[<SPI $num>]::PTR };
                    spi.dma_out_link()
                        .modify(|_, w| unsafe { w.outlink_addr().bits(address) });
                }

                fn set_out_peripheral(_peripheral: u8) {
                    // no-op
                }

                fn start_out() {
                    let spi = unsafe { &*crate::peripherals::[<SPI $num>]::PTR };
                    spi.dma_out_link().modify(|_, w| w.outlink_start().set_bit());
                }

                fn last_out_dscr_address() -> usize {
                    let spi = unsafe { &*crate::peripherals::[<SPI $num>]::PTR };
                    spi.out_eof_des_addr().read().dma_out_eof_des_addr().bits() as usize
                }

                fn set_in_burstmode(burst_mode: bool) {
                    let spi = unsafe { &*crate::peripherals::[<SPI $num>]::PTR };
                    spi.dma_conf()
                        .modify(|_, w| w.indscr_burst_en().bit(burst_mode));
                }

                fn set_in_priority(_priority: DmaPriority) {}

                fn clear_in_interrupts() {
                    let spi = unsafe { &*crate::peripherals::[<SPI $num>]::PTR };
                    spi.dma_int_clr().write(|w| {
                        w.in_done()
                            .clear_bit_by_one()
                            .in_err_eof()
                            .clear_bit_by_one()
                            .in_suc_eof()
                            .clear_bit_by_one()
                            .inlink_dscr_error()
                            .clear_bit_by_one()
                    });
                }

                fn reset_in() {
                    let spi = unsafe { &*crate::peripherals::[<SPI $num>]::PTR };
                    spi.dma_conf().modify(|_, w| w.in_rst().set_bit());
                    spi.dma_conf().modify(|_, w| w.in_rst().clear_bit());
                }

                fn set_in_descriptors(address: u32) {
                    let spi = unsafe { &*crate::peripherals::[<SPI $num>]::PTR };
                    spi.dma_in_link()
                        .modify(|_, w| unsafe { w.inlink_addr().bits(address) });
                }

                fn set_in_peripheral(_peripheral: u8) {
                    // no-op
                }

                fn start_in() {
                    let spi = unsafe { &*crate::peripherals::[<SPI $num>]::PTR };
                    spi.dma_in_link().modify(|_, w| w.inlink_start().set_bit());
                }

                fn listen_out(interrupts: impl Into<EnumSet<DmaTxInterrupt>>) {
                    let spi = unsafe { &*crate::peripherals::[<SPI $num>]::PTR };
                    spi.dma_int_ena().modify(|_, w| {
                        for interrupt in interrupts.into() {
                            match interrupt {
                                DmaTxInterrupt::TotalEof => w.out_total_eof().set_bit(),
                                DmaTxInterrupt::DescriptorError => w.outlink_dscr_error().set_bit(),
                                DmaTxInterrupt::Eof => w.out_eof().set_bit(),
                                DmaTxInterrupt::Done => w.out_done().set_bit(),
                            };
                        }
                        w
                    })
                }

                fn unlisten_out(interrupts: impl Into<EnumSet<DmaTxInterrupt>>) {
                    let spi = unsafe { &*crate::peripherals::[<SPI $num>]::PTR };
                    spi.dma_int_ena().modify(|_, w| {
                        for interrupt in interrupts.into() {
                            match interrupt {
                                DmaTxInterrupt::TotalEof => w.out_total_eof().clear_bit(),
                                DmaTxInterrupt::DescriptorError => w.outlink_dscr_error().clear_bit(),
                                DmaTxInterrupt::Eof => w.out_eof().clear_bit(),
                                DmaTxInterrupt::Done => w.out_done().clear_bit(),
                            };
                        }
                        w
                    })
                }

                fn is_listening_out() -> EnumSet<DmaTxInterrupt> {
                    let mut result = EnumSet::new();

                    let spi = unsafe { &*crate::peripherals::[<SPI $num>]::PTR };
                    let int_ena = spi.dma_int_ena().read();
                    if int_ena.out_total_eof().bit_is_set() {
                        result |= DmaTxInterrupt::TotalEof;
                    }
                    if int_ena.outlink_dscr_error().bit_is_set() {
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

                fn out_interrupts() -> EnumSet<DmaTxInterrupt> {
                    let mut result = EnumSet::new();

                    let spi = unsafe { &*crate::peripherals::[<SPI $num>]::PTR };
                    let int_raw = spi.dma_int_raw().read();
                    if int_raw.out_total_eof().bit_is_set() {
                        result |= DmaTxInterrupt::TotalEof;
                    }
                    if int_raw.outlink_dscr_error().bit_is_set() {
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

                fn clear_out(interrupts: impl Into<EnumSet<DmaTxInterrupt>>) {
                    let spi = unsafe { &*crate::peripherals::[<SPI $num>]::PTR };
                    spi.dma_int_clr().write(|w| {
                        for interrupt in interrupts.into() {
                            match interrupt {
                                DmaTxInterrupt::TotalEof => w.out_total_eof().clear_bit_by_one(),
                                DmaTxInterrupt::DescriptorError => w.outlink_dscr_error().clear_bit_by_one(),
                                DmaTxInterrupt::Eof => w.out_eof().clear_bit_by_one(),
                                DmaTxInterrupt::Done => w.out_done().clear_bit_by_one(),
                            };
                        }
                        w
                    })
                }

                fn listen_in(interrupts: impl Into<EnumSet<DmaRxInterrupt>>) {
                    let spi = unsafe { &*crate::peripherals::[<SPI $num>]::PTR };
                    spi.dma_int_ena().modify(|_, w| {
                        for interrupt in interrupts.into() {
                            match interrupt {
                                DmaRxInterrupt::SuccessfulEof => w.in_suc_eof().set_bit(),
                                DmaRxInterrupt::ErrorEof => w.in_err_eof().set_bit(),
                                DmaRxInterrupt::DescriptorError => w.inlink_dscr_error().set_bit(),
                                DmaRxInterrupt::DescriptorEmpty => w.inlink_dscr_empty().set_bit(),
                                DmaRxInterrupt::Done => w.in_done().set_bit(),
                            };
                        }
                        w
                    })
                }

                fn unlisten_in(interrupts: impl Into<EnumSet<DmaRxInterrupt>>) {
                    let spi = unsafe { &*crate::peripherals::[<SPI $num>]::PTR };
                    spi.dma_int_ena().modify(|_, w| {
                        for interrupt in interrupts.into() {
                            match interrupt {
                                DmaRxInterrupt::SuccessfulEof => w.in_suc_eof().clear_bit(),
                                DmaRxInterrupt::ErrorEof => w.in_err_eof().clear_bit(),
                                DmaRxInterrupt::DescriptorError => w.inlink_dscr_error().clear_bit(),
                                DmaRxInterrupt::DescriptorEmpty => w.inlink_dscr_empty().clear_bit(),
                                DmaRxInterrupt::Done => w.in_done().clear_bit(),
                            };
                        }
                        w
                    })
                }

                fn is_listening_in() -> EnumSet<DmaRxInterrupt> {
                    let mut result = EnumSet::new();

                    let spi = unsafe { &*crate::peripherals::[<SPI $num>]::PTR };
                    let int_ena = spi.dma_int_ena().read();
                    if int_ena.inlink_dscr_error().bit_is_set() {
                        result |= DmaRxInterrupt::DescriptorError;
                    }
                    if int_ena.inlink_dscr_empty().bit_is_set() {
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

                fn in_interrupts() -> EnumSet<DmaRxInterrupt> {
                    let mut result = EnumSet::new();

                    let spi = unsafe { &*crate::peripherals::[<SPI $num>]::PTR };
                    let int_raw = spi.dma_int_raw().read();
                    if int_raw.inlink_dscr_error().bit_is_set() {
                        result |= DmaRxInterrupt::DescriptorError;
                    }
                    if int_raw.inlink_dscr_empty().bit_is_set() {
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

                fn clear_in(interrupts: impl Into<EnumSet<DmaRxInterrupt>>) {
                    let spi = unsafe { &*crate::peripherals::[<SPI $num>]::PTR };
                    spi.dma_int_clr().modify(|_, w| {
                        for interrupt in interrupts.into() {
                            match interrupt {
                                DmaRxInterrupt::SuccessfulEof => w.in_suc_eof().clear_bit_by_one(),
                                DmaRxInterrupt::ErrorEof => w.in_err_eof().clear_bit_by_one(),
                                DmaRxInterrupt::DescriptorError => w.inlink_dscr_error().clear_bit_by_one(),
                                DmaRxInterrupt::DescriptorEmpty => w.inlink_dscr_empty().clear_bit_by_one(),
                                DmaRxInterrupt::Done => w.in_done().clear_bit_by_one(),
                            };
                        }
                        w
                    })
                }
            }

            #[non_exhaustive]
            #[doc(hidden)]
            pub struct [<Spi $num DmaChannelTxImpl>] {}

            impl $crate::private::Sealed for [<Spi $num DmaChannelTxImpl>] {}

            impl<'a> TxChannel<[<Spi $num DmaChannel>]> for [<Spi $num DmaChannelTxImpl>] {
                fn waker() -> &'static embassy_sync::waitqueue::AtomicWaker {
                    static WAKER: embassy_sync::waitqueue::AtomicWaker = embassy_sync::waitqueue::AtomicWaker::new();
                    &WAKER
                }
            }

            #[non_exhaustive]
            #[doc(hidden)]
            pub struct [<Spi $num DmaChannelRxImpl>] {}

            impl $crate::private::Sealed for [<Spi $num DmaChannelRxImpl>] {}

            impl<'a> RxChannel<[<Spi $num DmaChannel>]> for [<Spi $num DmaChannelRxImpl>] {
                fn waker() -> &'static embassy_sync::waitqueue::AtomicWaker {
                    static WAKER: embassy_sync::waitqueue::AtomicWaker = embassy_sync::waitqueue::AtomicWaker::new();
                    &WAKER
                }
            }

            #[doc = concat!("Creates a channel for SPI", $num)]
            #[non_exhaustive]
            pub struct [<Spi $num DmaChannelCreator>] {}

            impl [<Spi $num DmaChannelCreator>] {
                /// Configure the channel for use with blocking APIs
                ///
                /// Descriptors should be sized as `(CHUNK_SIZE + 4091) / 4092`. I.e., to
                /// transfer buffers of size `1..=4092`, you need 1 descriptor.
                pub fn configure<'a>(
                    self,
                    burst_mode: bool,
                    priority: DmaPriority,
                ) -> Channel<'a, [<Spi $num DmaChannel>], $crate::Blocking> {
                    let mut tx_impl = [<Spi $num DmaChannelTxImpl>] {};
                    tx_impl.init(burst_mode, priority);

                    let mut rx_impl = [<Spi $num DmaChannelRxImpl>] {};
                    rx_impl.init(burst_mode, priority);

                    Channel {
                        tx: ChannelTx::new(tx_impl, burst_mode),
                        rx: ChannelRx::new(rx_impl, burst_mode),
                        phantom: PhantomData,
                    }
                }

                /// Configure the channel for use with async APIs
                ///
                /// Descriptors should be sized as `(CHUNK_SIZE + 4091) / 4092`. I.e., to
                /// transfer buffers of size `1..=4092`, you need 1 descriptor.
                pub fn configure_for_async<'a>(
                    self,
                    burst_mode: bool,
                    priority: DmaPriority,
                ) -> Channel<'a, [<Spi $num DmaChannel>], $crate::Async> {
                    let mut tx_impl = [<Spi $num DmaChannelTxImpl>] {};
                    tx_impl.init(burst_mode, priority);

                    let mut rx_impl = [<Spi $num DmaChannelRxImpl>] {};
                    rx_impl.init(burst_mode, priority);

                    <[<Spi $num DmaChannel>] as ChannelTypes>::Binder::set_isr(super::asynch::interrupt::[< interrupt_handler_spi $num _dma >]);

                    Channel {
                        tx: ChannelTx::new(tx_impl, burst_mode),
                        rx: ChannelRx::new(rx_impl, burst_mode),
                        phantom: PhantomData,
                    }
                }
            }
        }
    };
}

macro_rules! ImplI2sChannel {
    ($num: literal, $peripheral: literal) => {
        paste::paste! {
            #[non_exhaustive]
            #[doc(hidden)]
            pub struct [<Channel $num InterruptBinder>] {}

            impl $crate::private::Sealed for [<Channel $num InterruptBinder>] {}

            impl InterruptBinder for [<Channel $num InterruptBinder>] {
                fn set_isr(handler:  $crate::interrupt::InterruptHandler) {
                    let mut i2s = unsafe { $crate::peripherals::[< I2S $num >]::steal() };
                    i2s.[< bind_i2s $num _interrupt>](handler.handler());
                    $crate::interrupt::enable($crate::peripherals::Interrupt::[< I2S $num  >], handler.priority()).unwrap();
                }
            }

            #[doc = concat!("DMA channel suitable for I2S", $num)]
            pub struct [<I2s $num DmaChannel>] {}

            impl $crate::private::Sealed for [<I2s $num DmaChannel>] {}

            impl DmaChannel for [<I2s $num DmaChannel>] {
                type Channel = [<I2s $num DmaChannel>];
                type Rx = [<I2s $num DmaChannelRxImpl>];
                type Tx = [<I2s $num DmaChannelTxImpl>];
                type P = [<I2s $num DmaSuitablePeripheral>];
            }

            impl ChannelTypes for [<I2s $num DmaChannel>] {
                type Binder = [<Channel $num InterruptBinder>];
            }

            impl RegisterAccess for [<I2s $num DmaChannel>] {
                fn init_channel() {
                    // nothing to do
                }

                fn set_out_burstmode(burst_mode: bool) {
                    let reg_block = unsafe { &*crate::peripherals::[<$peripheral>]::PTR };
                    reg_block.lc_conf()
                        .modify(|_, w| w.outdscr_burst_en().bit(burst_mode));
                }

                fn set_out_priority(_priority: DmaPriority) {}

                fn clear_out_interrupts() {
                    let reg_block = unsafe { &*crate::peripherals::[<$peripheral>]::PTR };
                    reg_block.int_clr().write(|w| {
                        w.out_done()
                            .clear_bit_by_one()
                            .out_eof()
                            .clear_bit_by_one()
                            .out_total_eof()
                            .clear_bit_by_one()
                            .out_dscr_err()
                            .clear_bit_by_one()
                    });
                }

                fn reset_out() {
                    let reg_block = unsafe { &*crate::peripherals::[<$peripheral>]::PTR };
                    reg_block.lc_conf().modify(|_, w| w.out_rst().set_bit());
                    reg_block.lc_conf().modify(|_, w| w.out_rst().clear_bit());
                }

                fn set_out_descriptors(address: u32) {
                    let reg_block = unsafe { &*crate::peripherals::[<$peripheral>]::PTR };
                    reg_block.out_link()
                        .modify(|_, w| unsafe { w.outlink_addr().bits(address) });
                }

                fn set_out_peripheral(_peripheral: u8) {
                    // no-op
                }

                fn start_out() {
                    let reg_block = unsafe { &*crate::peripherals::[<$peripheral>]::PTR };
                    reg_block.out_link().modify(|_, w| w.outlink_start().set_bit());
                }

                fn last_out_dscr_address() -> usize {
                    let reg_block = unsafe { &*crate::peripherals::[<$peripheral>]::PTR };
                    reg_block.out_eof_des_addr().read().out_eof_des_addr().bits() as usize
                }

                fn set_in_burstmode(burst_mode: bool) {
                    let reg_block = unsafe { &*crate::peripherals::[<$peripheral>]::PTR };
                    reg_block.lc_conf()
                        .modify(|_, w| w.indscr_burst_en().bit(burst_mode));
                }

                fn set_in_priority(_priority: DmaPriority) {}

                fn clear_in_interrupts() {
                    let reg_block = unsafe { &*crate::peripherals::[<$peripheral>]::PTR };
                    reg_block.int_clr().write(|w| {
                        w.in_done()
                            .clear_bit_by_one()
                            .in_err_eof()
                            .clear_bit_by_one()
                            .in_suc_eof()
                            .clear_bit_by_one()
                            .in_dscr_err()
                            .clear_bit_by_one()
                    });
                }

                fn reset_in() {
                    let reg_block = unsafe { &*crate::peripherals::[<$peripheral>]::PTR };
                    reg_block.lc_conf().modify(|_, w| w.in_rst().set_bit());
                    reg_block.lc_conf().modify(|_, w| w.in_rst().clear_bit());
                }

                fn set_in_descriptors(address: u32) {
                    let reg_block = unsafe { &*crate::peripherals::[<$peripheral>]::PTR };
                    reg_block.in_link()
                        .modify(|_, w| unsafe { w.inlink_addr().bits(address) });
                }

                fn set_in_peripheral(_peripheral: u8) {
                    // no-op
                }

                fn start_in() {
                    let reg_block = unsafe { &*crate::peripherals::[<$peripheral>]::PTR };
                    reg_block.in_link().modify(|_, w| w.inlink_start().set_bit());
                }

                fn listen_out(interrupts: impl Into<EnumSet<DmaTxInterrupt>>) {
                    let reg_block = unsafe { &*crate::peripherals::[<$peripheral>]::PTR };
                    reg_block.int_ena().modify(|_, w| {
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

                fn unlisten_out(interrupts: impl Into<EnumSet<DmaTxInterrupt>>) {
                    let reg_block = unsafe { &*crate::peripherals::[<$peripheral>]::PTR };
                    reg_block.int_ena().modify(|_, w| {
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

                fn is_listening_out() -> EnumSet<DmaTxInterrupt> {
                    let mut result = EnumSet::new();

                    let reg_block = unsafe { &*crate::peripherals::[<$peripheral>]::PTR };
                    let int_ena = reg_block.int_ena().read();
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

                fn out_interrupts() -> EnumSet<DmaTxInterrupt> {
                    let mut result = EnumSet::new();

                    let reg_block = unsafe { &*crate::peripherals::[<$peripheral>]::PTR };
                    let int_raw = reg_block.int_raw().read();
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

                fn clear_out(interrupts: impl Into<EnumSet<DmaTxInterrupt>>) {
                    let reg_block = unsafe { &*crate::peripherals::[<$peripheral>]::PTR };
                    reg_block.int_clr().write(|w| {
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

                fn listen_in(interrupts: impl Into<EnumSet<DmaRxInterrupt>>) {
                    let reg_block = unsafe { &*crate::peripherals::[<$peripheral>]::PTR };
                    reg_block.int_ena().modify(|_, w| {
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
                    })
                }

                fn unlisten_in(interrupts: impl Into<EnumSet<DmaRxInterrupt>>) {
                    let reg_block = unsafe { &*crate::peripherals::[<$peripheral>]::PTR };
                    reg_block.int_ena().modify(|_, w| {
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

                fn is_listening_in() -> EnumSet<DmaRxInterrupt> {
                    let mut result = EnumSet::new();

                    let reg_block = unsafe { &*crate::peripherals::[<$peripheral>]::PTR };
                    let int_ena = reg_block.int_ena().read();
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

                fn in_interrupts() -> EnumSet<DmaRxInterrupt> {
                    let mut result = EnumSet::new();

                    let reg_block = unsafe { &*crate::peripherals::[<$peripheral>]::PTR };
                    let int_raw = reg_block.int_raw().read();
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

                fn clear_in(interrupts: impl Into<EnumSet<DmaRxInterrupt>>) {
                    let reg_block = unsafe { &*crate::peripherals::[<$peripheral>]::PTR };
                    reg_block.int_clr().write(|w| {
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
            }

            #[doc(hidden)]
            pub struct [<I2s $num DmaChannelTxImpl>] {}

            impl $crate::private::Sealed for [<I2s $num DmaChannelTxImpl>] {}

            impl<'a> TxChannel<[<I2s $num DmaChannel>]> for [<I2s $num DmaChannelTxImpl>] {
                fn waker() -> &'static embassy_sync::waitqueue::AtomicWaker {
                    static WAKER: embassy_sync::waitqueue::AtomicWaker = embassy_sync::waitqueue::AtomicWaker::new();
                    &WAKER
                }
            }

            #[doc(hidden)]
            pub struct [<I2s $num DmaChannelRxImpl>] {}

            impl $crate::private::Sealed for [<I2s $num DmaChannelRxImpl>] {}

            impl<'a> RxChannel<[<I2s $num DmaChannel>]> for [<I2s $num DmaChannelRxImpl>] {
                fn waker() -> &'static embassy_sync::waitqueue::AtomicWaker {
                    static WAKER: embassy_sync::waitqueue::AtomicWaker = embassy_sync::waitqueue::AtomicWaker::new();
                    &WAKER
                }
            }

            #[doc = concat!("Creates a channel for I2S", $num)]
            pub struct [<I2s $num DmaChannelCreator>] {}

            impl [<I2s $num DmaChannelCreator>] {
                /// Configure the channel for use with blocking APIs
                ///
                /// Descriptors should be sized as `(CHUNK_SIZE + 4091) / 4092`. I.e., to
                /// transfer buffers of size `1..=4092`, you need 1 descriptor.
                pub fn configure<'a>(
                    self,
                    burst_mode: bool,
                    priority: DmaPriority,
                ) -> Channel<'a, [<I2s $num DmaChannel>], $crate::Blocking> {
                    let mut tx_impl = [<I2s $num DmaChannelTxImpl>] {};
                    tx_impl.init(burst_mode, priority);

                    let mut rx_impl = [<I2s $num DmaChannelRxImpl>] {};
                    rx_impl.init(burst_mode, priority);

                    Channel {
                        tx: ChannelTx::new(tx_impl, burst_mode),
                        rx: ChannelRx::new(rx_impl, burst_mode),
                        phantom: PhantomData,
                    }
                }

                /// Configure the channel for use with async APIs
                ///
                /// Descriptors should be sized as `(CHUNK_SIZE + 4091) / 4092`. I.e., to
                /// transfer buffers of size `1..=4092`, you need 1 descriptor.
                pub fn configure_for_async<'a>(
                    self,
                    burst_mode: bool,
                    priority: DmaPriority,
                ) -> Channel<'a, [<I2s $num DmaChannel>], $crate::Async> {
                    let mut tx_impl = [<I2s $num DmaChannelTxImpl>] {};
                    tx_impl.init(burst_mode, priority);

                    let mut rx_impl = [<I2s $num DmaChannelRxImpl>] {};
                    rx_impl.init(burst_mode, priority);

                    <[<I2s $num DmaChannel>] as ChannelTypes>::Binder::set_isr(super::asynch::interrupt::[< interrupt_handler_i2s $num >]);

                    Channel {
                        tx: ChannelTx::new(tx_impl, burst_mode),
                        rx: ChannelRx::new(rx_impl, burst_mode),
                        phantom: PhantomData,
                    }
                }
            }
        }
    };
}

#[doc(hidden)]
#[non_exhaustive]
pub struct Spi2DmaSuitablePeripheral {}
impl PeripheralMarker for Spi2DmaSuitablePeripheral {}
impl SpiPeripheral for Spi2DmaSuitablePeripheral {}
impl Spi2Peripheral for Spi2DmaSuitablePeripheral {}

#[doc(hidden)]
#[non_exhaustive]
pub struct Spi3DmaSuitablePeripheral {}
impl PeripheralMarker for Spi3DmaSuitablePeripheral {}
impl SpiPeripheral for Spi3DmaSuitablePeripheral {}
impl Spi3Peripheral for Spi3DmaSuitablePeripheral {}

ImplSpiChannel!(2);
ImplSpiChannel!(3);

#[doc(hidden)]
#[non_exhaustive]
pub struct I2s0DmaSuitablePeripheral {}
impl PeripheralMarker for I2s0DmaSuitablePeripheral {}
impl I2sPeripheral for I2s0DmaSuitablePeripheral {}
impl I2s0Peripheral for I2s0DmaSuitablePeripheral {}

ImplI2sChannel!(0, "I2S0");

#[doc(hidden)]
#[non_exhaustive]
pub struct I2s1DmaSuitablePeripheral {}
impl PeripheralMarker for I2s1DmaSuitablePeripheral {}
impl I2sPeripheral for I2s1DmaSuitablePeripheral {}
impl I2s1Peripheral for I2s1DmaSuitablePeripheral {}

#[cfg(esp32)]
ImplI2sChannel!(1, "I2S1");

/// DMA Peripheral
///
/// This offers the available DMA channels.
pub struct Dma<'d> {
    _inner: PeripheralRef<'d, crate::peripherals::DMA>,
    /// DMA channel for SPI2
    pub spi2channel: Spi2DmaChannelCreator,
    /// DMA channel for SPI3
    pub spi3channel: Spi3DmaChannelCreator,
    /// DMA channel for I2S0
    pub i2s0channel: I2s0DmaChannelCreator,
    /// DMA channel for I2S1
    #[cfg(esp32)]
    pub i2s1channel: I2s1DmaChannelCreator,
}

impl<'d> Dma<'d> {
    /// Create a DMA instance.
    pub fn new(
        dma: impl crate::peripheral::Peripheral<P = crate::peripherals::DMA> + 'd,
    ) -> Dma<'d> {
        PeripheralClockControl::enable(Peripheral::Dma);

        Dma {
            _inner: dma.into_ref(),
            spi2channel: Spi2DmaChannelCreator {},
            spi3channel: Spi3DmaChannelCreator {},
            i2s0channel: I2s0DmaChannelCreator {},
            #[cfg(esp32)]
            i2s1channel: I2s1DmaChannelCreator {},
        }
    }
}
