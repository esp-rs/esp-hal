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

use embassy_sync::waitqueue::AtomicWaker;

use crate::{
    dma::*,
    peripheral::PeripheralRef,
    system::{Peripheral, PeripheralClockControl},
};

type SpiRegisterBlock = crate::peripherals::spi2::RegisterBlock;
type I2sRegisterBlock = crate::peripherals::i2s0::RegisterBlock;

#[doc(hidden)]
pub trait SpiPdmaChannel: crate::private::Sealed {
    fn register_block(&self) -> &SpiRegisterBlock;
    fn tx_waker(&self) -> &'static AtomicWaker;
    fn rx_waker(&self) -> &'static AtomicWaker;
}

#[doc(hidden)]
pub trait I2sPdmaChannel: crate::private::Sealed {
    fn register_block(&self) -> &I2sRegisterBlock;
    fn tx_waker(&self) -> &'static AtomicWaker;
    fn rx_waker(&self) -> &'static AtomicWaker;
}

#[doc(hidden)]
pub struct SpiDmaRxChannelImpl<C>(C);

impl<C> crate::private::Sealed for SpiDmaRxChannelImpl<C> {}

impl<C: SpiPdmaChannel> SpiDmaRxChannelImpl<C>
where
    AnyPdmaChannel: From<C>,
{
    fn degrade(self) -> AnyPdmaRxChannelImpl {
        AnyPdmaRxChannelImpl(AnyPdmaChannel::from(self.0))
    }
}

#[doc(hidden)]
pub struct SpiDmaTxChannelImpl<C>(C);

impl<C> crate::private::Sealed for SpiDmaTxChannelImpl<C> {}

impl<C: SpiPdmaChannel> SpiDmaTxChannelImpl<C>
where
    AnyPdmaChannel: From<C>,
{
    fn degrade(self) -> AnyPdmaTxChannelImpl {
        AnyPdmaTxChannelImpl(AnyPdmaChannel::from(self.0))
    }
}

impl<C: SpiPdmaChannel> RegisterAccess for SpiDmaTxChannelImpl<C> {
    fn reset(&self) {
        let spi = self.0.register_block();
        spi.dma_conf().modify(|_, w| w.out_rst().set_bit());
        spi.dma_conf().modify(|_, w| w.out_rst().clear_bit());
    }

    fn set_burst_mode(&self, burst_mode: bool) {
        let spi = self.0.register_block();
        spi.dma_conf()
            .modify(|_, w| w.outdscr_burst_en().bit(burst_mode));
    }

    fn set_priority(&self, _priority: DmaPriority) {}

    fn set_peripheral(&self, _peripheral: u8) {
        // no-op
    }

    fn set_link_addr(&self, address: u32) {
        let spi = self.0.register_block();
        spi.dma_out_link()
            .modify(|_, w| unsafe { w.outlink_addr().bits(address) });
    }

    fn start(&self) {
        let spi = self.0.register_block();
        spi.dma_out_link()
            .modify(|_, w| w.outlink_start().set_bit());
    }

    fn stop(&self) {
        let spi = self.0.register_block();
        spi.dma_out_link().modify(|_, w| w.outlink_stop().set_bit());
    }

    fn restart(&self) {
        let spi = self.0.register_block();
        spi.dma_out_link()
            .modify(|_, w| w.outlink_restart().set_bit());
    }
}

impl<C: SpiPdmaChannel> TxRegisterAccess for SpiDmaTxChannelImpl<C> {
    fn last_dscr_address(&self) -> usize {
        let spi = self.0.register_block();
        spi.out_eof_des_addr().read().dma_out_eof_des_addr().bits() as usize
    }
}

impl<C: SpiPdmaChannel> InterruptAccess<DmaTxInterrupt> for SpiDmaTxChannelImpl<C> {
    fn enable_listen(&self, interrupts: EnumSet<DmaTxInterrupt>, enable: bool) {
        let spi = self.0.register_block();
        spi.dma_int_ena().modify(|_, w| {
            for interrupt in interrupts {
                match interrupt {
                    DmaTxInterrupt::TotalEof => w.out_total_eof().bit(enable),
                    DmaTxInterrupt::DescriptorError => w.outlink_dscr_error().bit(enable),
                    DmaTxInterrupt::Eof => w.out_eof().bit(enable),
                    DmaTxInterrupt::Done => w.out_done().bit(enable),
                };
            }
            w
        })
    }

    fn is_listening(&self) -> EnumSet<DmaTxInterrupt> {
        let mut result = EnumSet::new();

        let spi = self.0.register_block();
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

    fn clear(&self, interrupts: impl Into<EnumSet<DmaTxInterrupt>>) {
        let spi = self.0.register_block();
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

    fn pending_interrupts(&self) -> EnumSet<DmaTxInterrupt> {
        let mut result = EnumSet::new();

        let spi = self.0.register_block();
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

    fn waker(&self) -> &'static AtomicWaker {
        self.0.tx_waker()
    }
}

impl<C: SpiPdmaChannel> RegisterAccess for SpiDmaRxChannelImpl<C> {
    fn reset(&self) {
        let spi = self.0.register_block();
        spi.dma_conf().modify(|_, w| w.in_rst().set_bit());
        spi.dma_conf().modify(|_, w| w.in_rst().clear_bit());
    }

    fn set_burst_mode(&self, burst_mode: bool) {
        let spi = self.0.register_block();
        spi.dma_conf()
            .modify(|_, w| w.indscr_burst_en().bit(burst_mode));
    }

    fn set_priority(&self, _priority: DmaPriority) {}

    fn set_peripheral(&self, _peripheral: u8) {
        // no-op
    }

    fn set_link_addr(&self, address: u32) {
        let spi = self.0.register_block();
        spi.dma_in_link()
            .modify(|_, w| unsafe { w.inlink_addr().bits(address) });
    }

    fn start(&self) {
        let spi = self.0.register_block();
        spi.dma_in_link().modify(|_, w| w.inlink_start().set_bit());
    }

    fn stop(&self) {
        let spi = self.0.register_block();
        spi.dma_in_link().modify(|_, w| w.inlink_stop().set_bit());
    }

    fn restart(&self) {
        let spi = self.0.register_block();
        spi.dma_in_link()
            .modify(|_, w| w.inlink_restart().set_bit());
    }
}

impl<C: SpiPdmaChannel> RxRegisterAccess for SpiDmaRxChannelImpl<C> {}

impl<C: SpiPdmaChannel> InterruptAccess<DmaRxInterrupt> for SpiDmaRxChannelImpl<C> {
    fn enable_listen(&self, interrupts: EnumSet<DmaRxInterrupt>, enable: bool) {
        let spi = self.0.register_block();
        spi.dma_int_ena().modify(|_, w| {
            for interrupt in interrupts {
                match interrupt {
                    DmaRxInterrupt::SuccessfulEof => w.in_suc_eof().bit(enable),
                    DmaRxInterrupt::ErrorEof => w.in_err_eof().bit(enable),
                    DmaRxInterrupt::DescriptorError => w.inlink_dscr_error().bit(enable),
                    DmaRxInterrupt::DescriptorEmpty => w.inlink_dscr_empty().bit(enable),
                    DmaRxInterrupt::Done => w.in_done().bit(enable),
                };
            }
            w
        })
    }

    fn is_listening(&self) -> EnumSet<DmaRxInterrupt> {
        let mut result = EnumSet::new();

        let spi = self.0.register_block();
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

    fn clear(&self, interrupts: impl Into<EnumSet<DmaRxInterrupt>>) {
        let spi = self.0.register_block();
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

    fn pending_interrupts(&self) -> EnumSet<DmaRxInterrupt> {
        let mut result = EnumSet::new();

        let spi = self.0.register_block();
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

    fn waker(&self) -> &'static AtomicWaker {
        self.0.rx_waker()
    }
}

#[doc(hidden)]
pub struct SpiDmaChannel<C>(PhantomData<C>);

impl<C> crate::private::Sealed for SpiDmaChannel<C> {}

macro_rules! ImplSpiChannel {
    ($num: literal) => {
        paste::paste! {
            #[doc = concat!("DMA channel suitable for SPI", $num)]
            #[non_exhaustive]
            pub struct [<Spi $num DmaChannel>] {}

            impl [<Spi $num DmaChannel>] {
                unsafe fn clone_unchecked(&self) -> Self {
                    Self {}
                }
            }

            impl From<[<Spi $num DmaChannel>]> for AnyPdmaChannel {
                fn from(channel: [<Spi $num DmaChannel>]) -> Self {
                    AnyPdmaChannel::[<Spi $num>](channel)
                }
            }

            impl DmaChannel for [<Spi $num DmaChannel>] {
                type Rx = SpiDmaRxChannelImpl<Self>;
                type Tx = SpiDmaTxChannelImpl<Self>;

                fn degrade_rx(rx: Self::Rx) -> AnyPdmaRxChannelImpl {
                    rx.degrade()
                }
                fn degrade_tx(tx: Self::Tx) -> AnyPdmaTxChannelImpl {
                    tx.degrade()
                }
            }
            impl PeripheralDmaChannel for [<Spi $num DmaChannel>] {
                type P = [<Spi $num DmaSuitablePeripheral>];
            }

            impl DmaChannelExt for [<Spi $num DmaChannel>] {
                fn get_rx_interrupts() -> impl InterruptAccess<DmaRxInterrupt> {
                    SpiDmaRxChannelImpl(Self {})
                }
                fn get_tx_interrupts() -> impl InterruptAccess<DmaTxInterrupt> {
                    SpiDmaTxChannelImpl(Self {})
                }

                fn set_isr(handler: InterruptHandler) {
                    let interrupt = $crate::peripherals::Interrupt::[< SPI $num _DMA >];
                    unsafe {
                        crate::interrupt::bind_interrupt(interrupt, handler.handler());
                    }
                    crate::interrupt::enable(interrupt, handler.priority()).unwrap();
                }
            }

            impl SpiPdmaChannel for [<Spi $num DmaChannel>] {
                fn register_block(&self) -> &SpiRegisterBlock {
                    unsafe { &*crate::peripherals::[<SPI $num>]::PTR }
                }
                fn tx_waker(&self) -> &'static embassy_sync::waitqueue::AtomicWaker {
                    static WAKER: embassy_sync::waitqueue::AtomicWaker = embassy_sync::waitqueue::AtomicWaker::new();
                    &WAKER
                }
                fn rx_waker(&self) -> &'static embassy_sync::waitqueue::AtomicWaker {
                    static WAKER: embassy_sync::waitqueue::AtomicWaker = embassy_sync::waitqueue::AtomicWaker::new();
                    &WAKER
                }
            }

            impl $crate::private::Sealed for [<Spi $num DmaChannel>] {}

            #[doc = concat!("Creates a channel for SPI", $num)]
            #[non_exhaustive]
            pub struct [<Spi $num DmaChannelCreator>] {}

            impl [<Spi $num DmaChannelCreator>] {
                fn do_configure<'a, M: $crate::Mode>(
                    self,
                    burst_mode: bool,
                    priority: DmaPriority,
                ) -> Channel<'a, [<Spi $num DmaChannel>], M> {
                    #[cfg(esp32)]
                    {
                        // (only) on ESP32 we need to configure DPORT for the SPI DMA channels
                        let dport = unsafe { &*crate::peripherals::DPORT::PTR };
                        dport
                            .spi_dma_chan_sel()
                            .modify(|_, w| unsafe { w.[< spi $num _dma_chan_sel>]().bits($num - 1) });
                    }

                    let tx_impl = SpiDmaTxChannelImpl([<Spi $num DmaChannel>] {});
                    tx_impl.set_burst_mode(burst_mode);
                    tx_impl.set_priority(priority);

                    let rx_impl = SpiDmaRxChannelImpl([<Spi $num DmaChannel>] {});
                    rx_impl.set_burst_mode(burst_mode);
                    rx_impl.set_priority(priority);

                    Channel {
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
                ) -> Channel<'a, [<Spi $num DmaChannel>], $crate::Blocking> {
                    Self::do_configure(self, burst_mode, priority)
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
                    let this = Self::do_configure(self, burst_mode, priority);

                    [<Spi $num DmaChannel>]::set_isr(super::asynch::interrupt::[< interrupt_handler_spi $num _dma >]);

                    this
                }
            }
        }
    };
}

#[doc(hidden)]
pub struct I2sDmaRxChannelImpl<C>(C);

impl<C> crate::private::Sealed for I2sDmaRxChannelImpl<C> {}

impl<C: I2sPdmaChannel> I2sDmaRxChannelImpl<C>
where
    AnyPdmaChannel: From<C>,
{
    fn degrade(self) -> AnyPdmaRxChannelImpl {
        AnyPdmaRxChannelImpl(AnyPdmaChannel::from(self.0))
    }
}

#[doc(hidden)]
pub struct I2sDmaTxChannelImpl<C>(C);

impl<C> crate::private::Sealed for I2sDmaTxChannelImpl<C> {}

impl<C: I2sPdmaChannel> I2sDmaTxChannelImpl<C>
where
    AnyPdmaChannel: From<C>,
{
    fn degrade(self) -> AnyPdmaTxChannelImpl {
        AnyPdmaTxChannelImpl(AnyPdmaChannel::from(self.0))
    }
}

impl<C: I2sPdmaChannel> RegisterAccess for I2sDmaTxChannelImpl<C> {
    fn set_burst_mode(&self, burst_mode: bool) {
        let reg_block = self.0.register_block();
        reg_block
            .lc_conf()
            .modify(|_, w| w.outdscr_burst_en().bit(burst_mode));
    }

    fn set_priority(&self, _priority: DmaPriority) {}

    fn reset(&self) {
        let reg_block = self.0.register_block();
        reg_block.lc_conf().modify(|_, w| w.out_rst().set_bit());
        reg_block.lc_conf().modify(|_, w| w.out_rst().clear_bit());
    }

    fn set_link_addr(&self, address: u32) {
        let reg_block = self.0.register_block();
        reg_block
            .out_link()
            .modify(|_, w| unsafe { w.outlink_addr().bits(address) });
    }

    fn set_peripheral(&self, _peripheral: u8) {
        // no-op
    }

    fn start(&self) {
        let reg_block = self.0.register_block();
        reg_block
            .out_link()
            .modify(|_, w| w.outlink_start().set_bit());
    }

    fn stop(&self) {
        let reg_block = self.0.register_block();
        reg_block
            .out_link()
            .modify(|_, w| w.outlink_stop().set_bit());
    }

    fn restart(&self) {
        let reg_block = self.0.register_block();
        reg_block
            .out_link()
            .modify(|_, w| w.outlink_restart().set_bit());
    }
}

impl<C: I2sPdmaChannel> TxRegisterAccess for I2sDmaTxChannelImpl<C> {
    fn last_dscr_address(&self) -> usize {
        let reg_block = self.0.register_block();
        reg_block
            .out_eof_des_addr()
            .read()
            .out_eof_des_addr()
            .bits() as usize
    }
}

impl<C: I2sPdmaChannel> InterruptAccess<DmaTxInterrupt> for I2sDmaTxChannelImpl<C> {
    fn enable_listen(&self, interrupts: EnumSet<DmaTxInterrupt>, enable: bool) {
        let reg_block = self.0.register_block();
        reg_block.int_ena().modify(|_, w| {
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

        let reg_block = self.0.register_block();
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

    fn pending_interrupts(&self) -> EnumSet<DmaTxInterrupt> {
        let mut result = EnumSet::new();

        let reg_block = self.0.register_block();
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

    fn clear(&self, interrupts: impl Into<EnumSet<DmaTxInterrupt>>) {
        let reg_block = self.0.register_block();
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

    fn waker(&self) -> &'static AtomicWaker {
        self.0.tx_waker()
    }
}

impl<C: I2sPdmaChannel> RegisterAccess for I2sDmaRxChannelImpl<C> {
    fn set_burst_mode(&self, burst_mode: bool) {
        let reg_block = self.0.register_block();
        reg_block
            .lc_conf()
            .modify(|_, w| w.indscr_burst_en().bit(burst_mode));
    }

    fn set_priority(&self, _priority: DmaPriority) {}

    fn reset(&self) {
        let reg_block = self.0.register_block();
        reg_block.lc_conf().modify(|_, w| w.in_rst().set_bit());
        reg_block.lc_conf().modify(|_, w| w.in_rst().clear_bit());
    }

    fn set_link_addr(&self, address: u32) {
        let reg_block = self.0.register_block();
        reg_block
            .in_link()
            .modify(|_, w| unsafe { w.inlink_addr().bits(address) });
    }

    fn set_peripheral(&self, _peripheral: u8) {
        // no-op
    }

    fn start(&self) {
        let reg_block = self.0.register_block();
        reg_block
            .in_link()
            .modify(|_, w| w.inlink_start().set_bit());
    }

    fn stop(&self) {
        let reg_block = self.0.register_block();
        reg_block.in_link().modify(|_, w| w.inlink_stop().set_bit());
    }

    fn restart(&self) {
        let reg_block = self.0.register_block();
        reg_block
            .in_link()
            .modify(|_, w| w.inlink_restart().set_bit());
    }
}

impl<C: I2sPdmaChannel> RxRegisterAccess for I2sDmaRxChannelImpl<C> {}

impl<C: I2sPdmaChannel> InterruptAccess<DmaRxInterrupt> for I2sDmaRxChannelImpl<C> {
    fn enable_listen(&self, interrupts: EnumSet<DmaRxInterrupt>, enable: bool) {
        let spi = self.0.register_block();
        reg_block.int_ena().modify(|_, w| {
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
        })
    }

    fn is_listening(&self) -> EnumSet<DmaRxInterrupt> {
        let mut result = EnumSet::new();

        let reg_block = self.0.register_block();
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

    fn pending_interrupts(&self) -> EnumSet<DmaRxInterrupt> {
        let mut result = EnumSet::new();

        let reg_block = self.0.register_block();
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

    fn clear(&self, interrupts: impl Into<EnumSet<DmaRxInterrupt>>) {
        let reg_block = self.0.register_block();
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

    fn waker(&self) -> &'static AtomicWaker {
        self.0.rx_waker()
    }
}

macro_rules! ImplI2sChannel {
    ($num: literal) => {
        paste::paste! {
            #[doc = concat!("DMA channel suitable for I2S", $num)]
            pub struct [<I2s $num DmaChannel>] {}

            impl [<I2s $num DmaChannel>] {
                unsafe fn clone_unchecked(&self) -> Self {
                    Self {}
                }
            }

            impl From<[<I2s $num DmaChannel>]> for AnyPdmaChannel {
                fn from(channel: [<I2s $num DmaChannel>]) -> Self {
                    AnyPdmaChannel::[<I2s $num>](channel)
                }
            }

            impl $crate::private::Sealed for [<I2s $num DmaChannel>] {}

            impl DmaChannel for [<I2s $num DmaChannel>] {
                type Rx = I2sDmaRxChannelImpl<Self>;
                type Tx = I2sDmaTxChannelImpl<Self>;

                fn degrade_rx(rx: Self::Rx) -> AnyPdmaRxChannelImpl {
                    rx.degrade()
                }
                fn degrade_tx(tx: Self::Tx) -> AnyPdmaTxChannelImpl {
                    tx.degrade()
                }
            }
            impl PeripheralDmaChannel for [<I2s $num DmaChannel>] {
                type P = [<I2s $num DmaSuitablePeripheral>];
            }

            impl DmaChannelExt for [<I2s $num DmaChannel>] {
                fn get_rx_interrupts() -> impl InterruptAccess<DmaRxInterrupt> {
                    I2sDmaRxChannelImpl(Self {})
                }
                fn get_tx_interrupts() -> impl InterruptAccess<DmaTxInterrupt> {
                    I2sDmaTxChannelImpl(Self {})
                }

                fn set_isr(handler: InterruptHandler) {
                    let interrupt = $crate::peripherals::Interrupt::[< I2S $num  >];
                    unsafe {
                        crate::interrupt::bind_interrupt(interrupt, handler.handler());
                    }
                    crate::interrupt::enable(interrupt, handler.priority()).unwrap();
                }
            }

            impl I2sPdmaChannel for [<I2s $num DmaChannel>] {
                fn register_block(&self) -> &I2sRegisterBlock {
                    unsafe { &*crate::peripherals::[< I2S $num >]::PTR }
                }
                fn tx_waker(&self) -> &'static embassy_sync::waitqueue::AtomicWaker {
                    static WAKER: embassy_sync::waitqueue::AtomicWaker = embassy_sync::waitqueue::AtomicWaker::new();
                    &WAKER
                }
                fn rx_waker(&self) -> &'static embassy_sync::waitqueue::AtomicWaker {
                    static WAKER: embassy_sync::waitqueue::AtomicWaker = embassy_sync::waitqueue::AtomicWaker::new();
                    &WAKER
                }
            }

            #[doc = concat!("Creates a channel for I2S", $num)]
            pub struct [<I2s $num DmaChannelCreator>] {}

            impl [<I2s $num DmaChannelCreator>] {
                fn do_configure<'a, M: $crate::Mode>(
                    self,
                    burst_mode: bool,
                    priority: DmaPriority,
                ) -> Channel<'a, [<I2s $num DmaChannel>], M> {
                    let tx_impl = I2sDmaTxChannelImpl([<I2s $num DmaChannel>] {});
                    tx_impl.set_burst_mode(burst_mode);
                    tx_impl.set_priority(priority);

                    let rx_impl = I2sDmaRxChannelImpl([<I2s $num DmaChannel>] {});
                    rx_impl.set_burst_mode(burst_mode);
                    rx_impl.set_priority(priority);

                    Channel {
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
                ) -> Channel<'a, [<I2s $num DmaChannel>], $crate::Blocking> {
                    Self::do_configure(self, burst_mode, priority)
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
                    let this = Self::do_configure(self, burst_mode, priority);

                    [<I2s $num DmaChannel>]::set_isr(super::asynch::interrupt::[< interrupt_handler_i2s $num >]);

                    this
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

ImplI2sChannel!(0);

#[doc(hidden)]
#[non_exhaustive]
pub struct I2s1DmaSuitablePeripheral {}
impl PeripheralMarker for I2s1DmaSuitablePeripheral {}
impl I2sPeripheral for I2s1DmaSuitablePeripheral {}
impl I2s1Peripheral for I2s1DmaSuitablePeripheral {}

#[cfg(i2s1)]
ImplI2sChannel!(1);

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
            #[cfg(i2s1)]
            i2s1channel: I2s1DmaChannelCreator {},
        }
    }
}

#[doc(hidden)]
pub enum AnyPdmaChannel {
    Spi2(Spi2DmaChannel),
    Spi3(Spi3DmaChannel),
    I2s0(I2s0DmaChannel),
    #[cfg(i2s1)]
    I2s1(I2s1DmaChannel),
}

impl DmaChannel for AnyDmaChannel {
    type Rx = AnyPdmaRxChannelImpl;
    type Tx = AnyPdmaTxChannelImpl;

    fn degrade_rx(rx: Self::Rx) -> AnyPdmaRxChannelImpl {
        rx
    }

    fn degrade_tx(tx: Self::Tx) -> AnyPdmaTxChannelImpl {
        tx
    }
}

#[doc(hidden)]
pub struct AnyPdmaRxChannelImpl(AnyPdmaChannel);

impl crate::private::Sealed for AnyPdmaRxChannelImpl {}
impl InterruptAccess<DmaRxInterrupt> for AnyPdmaRxChannelImpl {
    delegate::delegate! {
        to match &self.0 {
            AnyPdmaChannel::Spi2(channel) => unsafe { SpiDmaRxChannelImpl(channel.clone_unchecked()) },
            AnyPdmaChannel::Spi3(channel) => unsafe { SpiDmaRxChannelImpl(channel.clone_unchecked()) },
            AnyPdmaChannel::I2s0(channel) => unsafe { I2sDmaRxChannelImpl(channel.clone_unchecked()) },
            #[cfg(i2s1)]
            AnyPdmaChannel::I2s1(channel) => unsafe { I2sDmaRxChannelImpl(channel.clone_unchecked()) },
        } {
            fn listen(&self, interrupts: impl Into<EnumSet<DmaRxInterrupt>>);
            fn unlisten(&self, interrupts: impl Into<EnumSet<DmaRxInterrupt>>);
            fn is_listening(&self) -> EnumSet<DmaRxInterrupt>;
            fn clear(&self, interrupts: impl Into<EnumSet<DmaRxInterrupt>>);
            fn pending_interrupts(&self) -> EnumSet<DmaRxInterrupt>;
            fn waker(&self) -> &'static AtomicWaker;
        }
    }
}
impl RegisterAccess for AnyPdmaRxChannelImpl {
    delegate::delegate! {
        to match &self.0 {
            AnyPdmaChannel::Spi2(channel) => unsafe { SpiDmaRxChannelImpl(channel.clone_unchecked()) },
            AnyPdmaChannel::Spi3(channel) => unsafe { SpiDmaRxChannelImpl(channel.clone_unchecked()) },
            AnyPdmaChannel::I2s0(channel) => unsafe { I2sDmaRxChannelImpl(channel.clone_unchecked()) },
            #[cfg(i2s1)]
            AnyPdmaChannel::I2s1(channel) => unsafe { I2sDmaRxChannelImpl(channel.clone_unchecked()) },
        } {
            fn set_burst_mode(&self, burst_mode: bool);
            fn set_priority(&self, priority: DmaPriority);
            fn clear_interrupts(&self);
            fn reset(&self);
            fn set_link_addr(&self, address: u32);
            fn set_peripheral(&self, peripheral: u8);
            fn start(&self);
            fn stop(&self);
            fn restart(&self);
        }
    }
}
impl RxRegisterAccess for AnyPdmaRxChannelImpl {}

#[doc(hidden)]
pub struct AnyPdmaTxChannelImpl(AnyPdmaChannel);

impl crate::private::Sealed for AnyPdmaTxChannelImpl {}
impl InterruptAccess<DmaTxInterrupt> for AnyPdmaTxChannelImpl {
    delegate::delegate! {
        to match &self.0 {
            AnyPdmaChannel::Spi2(channel) => unsafe { SpiDmaTxChannelImpl(channel.clone_unchecked()) },
            AnyPdmaChannel::Spi3(channel) => unsafe { SpiDmaTxChannelImpl(channel.clone_unchecked()) },
            AnyPdmaChannel::I2s0(channel) => unsafe { I2sDmaTxChannelImpl(channel.clone_unchecked()) },
            #[cfg(i2s1)]
            AnyPdmaChannel::I2s1(channel) => unsafe { I2sDmaTxChannelImpl(channel.clone_unchecked()) },
        } {
            fn listen(&self, interrupts: impl Into<EnumSet<DmaTxInterrupt>>);
            fn unlisten(&self, interrupts: impl Into<EnumSet<DmaTxInterrupt>>);
            fn is_listening(&self) -> EnumSet<DmaTxInterrupt>;
            fn clear(&self, interrupts: impl Into<EnumSet<DmaTxInterrupt>>);
            fn pending_interrupts(&self) -> EnumSet<DmaTxInterrupt>;
            fn waker(&self) -> &'static AtomicWaker;
        }
    }
}
impl RegisterAccess for AnyPdmaTxChannelImpl {
    delegate::delegate! {
        to match &self.0 {
            AnyPdmaChannel::Spi2(channel) => unsafe { SpiDmaTxChannelImpl(channel.clone_unchecked()) },
            AnyPdmaChannel::Spi3(channel) => unsafe { SpiDmaTxChannelImpl(channel.clone_unchecked()) },
            AnyPdmaChannel::I2s0(channel) => unsafe { I2sDmaTxChannelImpl(channel.clone_unchecked()) },
            #[cfg(i2s1)]
            AnyPdmaChannel::I2s1(channel) => unsafe { I2sDmaTxChannelImpl(channel.clone_unchecked()) },
        } {
            fn set_burst_mode(&self, burst_mode: bool);
            fn set_priority(&self, priority: DmaPriority);
            fn clear_interrupts(&self);
            fn reset(&self);
            fn set_link_addr(&self, address: u32);
            fn set_peripheral(&self, peripheral: u8);
            fn start(&self);
            fn stop(&self);
            fn restart(&self);
        }
    }
}
impl TxRegisterAccess for AnyPdmaTxChannelImpl {
    delegate::delegate! {
        to match &self.0 {
            AnyPdmaChannel::Spi2(channel) => unsafe { SpiDmaTxChannelImpl(channel.clone_unchecked()) },
            AnyPdmaChannel::Spi3(channel) => unsafe { SpiDmaTxChannelImpl(channel.clone_unchecked()) },
            AnyPdmaChannel::I2s0(channel) => unsafe { I2sDmaTxChannelImpl(channel.clone_unchecked()) },
            #[cfg(i2s1)]
            AnyPdmaChannel::I2s1(channel) => unsafe { I2sDmaTxChannelImpl(channel.clone_unchecked()) },
        } {
            fn last_dscr_address(&self) -> usize;
        }
    }
}
