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
    peripherals::Interrupt,
    system::{Peripheral, PeripheralClockControl},
    Blocking,
};

type SpiRegisterBlock = crate::peripherals::spi2::RegisterBlock;
type I2sRegisterBlock = crate::peripherals::i2s0::RegisterBlock;

#[doc(hidden)]
pub trait PdmaChannel: crate::private::Sealed {
    type RegisterBlock;

    fn register_block(&self) -> &Self::RegisterBlock;
    fn tx_waker(&self) -> &'static AtomicWaker;
    fn rx_waker(&self) -> &'static AtomicWaker;
    fn is_compatible_with(&self, peripheral: &impl PeripheralMarker) -> bool;
}

#[doc(hidden)]
pub struct SpiDmaRxChannelImpl<C>(C);

impl<C> crate::private::Sealed for SpiDmaRxChannelImpl<C> {}

#[doc(hidden)]
pub struct SpiDmaTxChannelImpl<C>(C);

impl<C> crate::private::Sealed for SpiDmaTxChannelImpl<C> {}

impl<C: PdmaChannel<RegisterBlock = SpiRegisterBlock>> RegisterAccess for SpiDmaTxChannelImpl<C> {
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

    fn set_check_owner(&self, check_owner: Option<bool>) {
        if check_owner == Some(true) {
            panic!("SPI DMA does not support checking descriptor ownership");
        }
    }

    fn is_compatible_with(&self, peripheral: &impl PeripheralMarker) -> bool {
        self.0.is_compatible_with(peripheral)
    }
}

impl<C: PdmaChannel<RegisterBlock = SpiRegisterBlock>> TxRegisterAccess for SpiDmaTxChannelImpl<C> {
    fn set_auto_write_back(&self, enable: bool) {
        // there is no `auto_wrback` for SPI
        assert!(!enable);
    }

    fn last_dscr_address(&self) -> usize {
        let spi = self.0.register_block();
        spi.out_eof_des_addr().read().dma_out_eof_des_addr().bits() as usize
    }
}

impl<C: PdmaChannel<RegisterBlock = SpiRegisterBlock>> InterruptAccess<DmaTxInterrupt>
    for SpiDmaTxChannelImpl<C>
{
    fn enable_listen(&self, interrupts: EnumSet<DmaTxInterrupt>, enable: bool) {
        let reg_block = self.0.register_block();
        reg_block.dma_int_ena().modify(|_, w| {
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

impl<C: PdmaChannel<RegisterBlock = SpiRegisterBlock>> RegisterAccess for SpiDmaRxChannelImpl<C> {
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

    fn set_check_owner(&self, check_owner: Option<bool>) {
        if check_owner == Some(true) {
            panic!("SPI DMA does not support checking descriptor ownership");
        }
    }

    fn is_compatible_with(&self, peripheral: &impl PeripheralMarker) -> bool {
        self.0.is_compatible_with(peripheral)
    }
}

impl<C: PdmaChannel<RegisterBlock = SpiRegisterBlock>> RxRegisterAccess for SpiDmaRxChannelImpl<C> {}

impl<C: PdmaChannel<RegisterBlock = SpiRegisterBlock>> InterruptAccess<DmaRxInterrupt>
    for SpiDmaRxChannelImpl<C>
{
    fn enable_listen(&self, interrupts: EnumSet<DmaRxInterrupt>, enable: bool) {
        let reg_block = self.0.register_block();
        reg_block.dma_int_ena().modify(|_, w| {
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
                fn handler() -> InterruptHandler {
                    super::asynch::interrupt::[< interrupt_handler_spi $num _dma >]
                }

                fn isrs() -> &'static [Interrupt] {
                    &[Interrupt::[< SPI $num _DMA >]]
                }
            }

            impl DmaChannel for [<Spi $num DmaChannel>] {
                type Rx = SpiDmaRxChannelImpl<Self>;
                type Tx = SpiDmaTxChannelImpl<Self>;

                fn async_handler<M: Mode>(_ch: &Channel<'_, Self, M>) -> InterruptHandler {
                    Self::handler()
                }

                fn interrupts<M: Mode>(_ch: &Channel<'_, Self, M>) -> &'static [Interrupt] {
                    Self::isrs()
                }
            }

            impl DmaChannelExt for [<Spi $num DmaChannel>] {
                fn get_rx_interrupts() -> impl InterruptAccess<DmaRxInterrupt> {
                    SpiDmaRxChannelImpl(Self {})
                }
                fn get_tx_interrupts() -> impl InterruptAccess<DmaTxInterrupt> {
                    SpiDmaTxChannelImpl(Self {})
                }
            }

            impl PdmaChannel for [<Spi $num DmaChannel>] {
                type RegisterBlock = SpiRegisterBlock;

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

                fn is_compatible_with(&self, peripheral: &impl PeripheralMarker) -> bool {
                    peripheral.peripheral() == crate::system::Peripheral::[<Spi $num>]
                }
            }

            impl DmaChannelConvert<AnySpiDmaChannel> for [<Spi $num DmaChannel>] {
                fn degrade_rx(rx: SpiDmaRxChannelImpl<Self>) -> SpiDmaRxChannelImpl<AnySpiDmaChannelInner> {
                    SpiDmaRxChannelImpl(rx.0.into())
                }
                fn degrade_tx(tx: SpiDmaTxChannelImpl<Self>) -> SpiDmaTxChannelImpl<AnySpiDmaChannelInner> {
                    SpiDmaTxChannelImpl(tx.0.into())
                }
            }

            impl $crate::private::Sealed for [<Spi $num DmaChannel>] {}

            #[doc = concat!("Creates a channel for SPI", $num)]
            #[non_exhaustive]
            pub struct [<Spi $num DmaChannelCreator>] {}

            impl [<Spi $num DmaChannelCreator>] {
                /// Configure the channel for use with blocking APIs
                pub fn configure<'a>(
                    self,
                    burst_mode: bool,
                    priority: DmaPriority,
                ) -> Channel<'a, [<Spi $num DmaChannel>], Blocking> {
                    let mut this = Channel {
                        tx: ChannelTx::new(SpiDmaTxChannelImpl([<Spi $num DmaChannel>] {})),
                        rx: ChannelRx::new(SpiDmaRxChannelImpl([<Spi $num DmaChannel>] {})),
                        phantom: PhantomData,
                    };

                    this.configure(burst_mode, priority);

                    this
                }
            }
        }
    };
}

#[doc(hidden)]
pub struct I2sDmaRxChannelImpl<C>(C);

impl<C> crate::private::Sealed for I2sDmaRxChannelImpl<C> {}

#[doc(hidden)]
pub struct I2sDmaTxChannelImpl<C>(C);

impl<C> crate::private::Sealed for I2sDmaTxChannelImpl<C> {}

impl<C: PdmaChannel<RegisterBlock = I2sRegisterBlock>> RegisterAccess for I2sDmaTxChannelImpl<C> {
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

    fn set_check_owner(&self, check_owner: Option<bool>) {
        let reg_block = self.0.register_block();
        reg_block
            .lc_conf()
            .modify(|_, w| w.check_owner().bit(check_owner.unwrap_or(true)));
    }

    fn is_compatible_with(&self, peripheral: &impl PeripheralMarker) -> bool {
        self.0.is_compatible_with(peripheral)
    }
}

impl<C: PdmaChannel<RegisterBlock = I2sRegisterBlock>> TxRegisterAccess for I2sDmaTxChannelImpl<C> {
    fn set_auto_write_back(&self, enable: bool) {
        let reg_block = self.0.register_block();
        reg_block
            .lc_conf()
            .modify(|_, w| w.out_auto_wrback().bit(enable));
    }

    fn last_dscr_address(&self) -> usize {
        let reg_block = self.0.register_block();
        reg_block
            .out_eof_des_addr()
            .read()
            .out_eof_des_addr()
            .bits() as usize
    }
}

impl<C: PdmaChannel<RegisterBlock = I2sRegisterBlock>> InterruptAccess<DmaTxInterrupt>
    for I2sDmaTxChannelImpl<C>
{
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

impl<C: PdmaChannel<RegisterBlock = I2sRegisterBlock>> RegisterAccess for I2sDmaRxChannelImpl<C> {
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

    fn set_check_owner(&self, check_owner: Option<bool>) {
        let reg_block = self.0.register_block();
        reg_block
            .lc_conf()
            .modify(|_, w| w.check_owner().bit(check_owner.unwrap_or(true)));
    }

    fn is_compatible_with(&self, peripheral: &impl PeripheralMarker) -> bool {
        self.0.is_compatible_with(peripheral)
    }
}

impl<C: PdmaChannel<RegisterBlock = I2sRegisterBlock>> RxRegisterAccess for I2sDmaRxChannelImpl<C> {}

impl<C: PdmaChannel<RegisterBlock = I2sRegisterBlock>> InterruptAccess<DmaRxInterrupt>
    for I2sDmaRxChannelImpl<C>
{
    fn enable_listen(&self, interrupts: EnumSet<DmaRxInterrupt>, enable: bool) {
        let reg_block = self.0.register_block();
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

            impl $crate::private::Sealed for [<I2s $num DmaChannel>] {}

            impl [<I2s $num DmaChannel>] {
                fn handler() -> InterruptHandler {
                    super::asynch::interrupt::[< interrupt_handler_i2s $num _dma >]
                }

                fn isrs() -> &'static [Interrupt] {
                    &[Interrupt::[< I2S $num >]]
                }
            }

            impl DmaChannel for [<I2s $num DmaChannel>] {
                type Rx = I2sDmaRxChannelImpl<Self>;
                type Tx = I2sDmaTxChannelImpl<Self>;

                fn async_handler<M: Mode>(_ch: &Channel<'_, Self, M>) -> InterruptHandler {
                    Self::handler()
                }

                fn interrupts<M: Mode>(_ch: &Channel<'_, Self, M>) -> &'static [Interrupt] {
                    Self::isrs()
                }
            }

            impl DmaChannelExt for [<I2s $num DmaChannel>] {
                fn get_rx_interrupts() -> impl InterruptAccess<DmaRxInterrupt> {
                    I2sDmaRxChannelImpl(Self {})
                }
                fn get_tx_interrupts() -> impl InterruptAccess<DmaTxInterrupt> {
                    I2sDmaTxChannelImpl(Self {})
                }
            }

            impl PdmaChannel for [<I2s $num DmaChannel>] {
                type RegisterBlock = I2sRegisterBlock;

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
                fn is_compatible_with(&self, peripheral: &impl PeripheralMarker) -> bool {
                    peripheral.peripheral() == crate::system::Peripheral::[<I2s $num>]
                }
            }

            impl DmaChannelConvert<AnyI2sDmaChannel> for [<I2s $num DmaChannel>] {
                fn degrade_rx(rx: I2sDmaRxChannelImpl<Self>) -> I2sDmaRxChannelImpl<AnyI2sDmaChannelInner> {
                    I2sDmaRxChannelImpl(rx.0.into())
                }
                fn degrade_tx(tx: I2sDmaTxChannelImpl<Self>) -> I2sDmaTxChannelImpl<AnyI2sDmaChannelInner> {
                    I2sDmaTxChannelImpl(tx.0.into())
                }
            }

            #[doc = concat!("Creates a channel for I2S", $num)]
            pub struct [<I2s $num DmaChannelCreator>] {}

            impl [<I2s $num DmaChannelCreator>] {
                /// Configure the channel for use with blocking APIs
                pub fn configure<'a>(
                    self,
                    burst_mode: bool,
                    priority: DmaPriority,
                ) -> Channel<'a, [<I2s $num DmaChannel>], Blocking> {
                    let mut this = Channel {
                        tx: ChannelTx::new(I2sDmaTxChannelImpl([<I2s $num DmaChannel>] {})),
                        rx: ChannelRx::new(I2sDmaRxChannelImpl([<I2s $num DmaChannel>] {})),
                        phantom: PhantomData,
                    };

                    this.configure(burst_mode, priority);

                    this
                }
            }
        }
    };
}

ImplSpiChannel!(2);
ImplSpiChannel!(3);

ImplI2sChannel!(0);
#[cfg(i2s1)]
ImplI2sChannel!(1);

// Specific peripherals use specific channels. Note that this may be overly
// restrictive (ESP32 allows configuring 2 SPI DMA channels between 3 different
// peripherals), but for the current set of restrictions this is sufficient.
crate::impl_dma_eligible!([Spi2DmaChannel] SPI2 => Spi2);
crate::impl_dma_eligible!([Spi3DmaChannel] SPI3 => Spi3);
crate::impl_dma_eligible!([I2s0DmaChannel] I2S0 => I2s0);
#[cfg(i2s1)]
crate::impl_dma_eligible!([I2s1DmaChannel] I2S1 => I2s1);

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
    #[cfg(i2s1)]
    pub i2s1channel: I2s1DmaChannelCreator,
}

impl<'d> Dma<'d> {
    /// Create a DMA instance.
    pub fn new(
        dma: impl crate::peripheral::Peripheral<P = crate::peripherals::DMA> + 'd,
    ) -> Dma<'d> {
        PeripheralClockControl::enable(Peripheral::Dma);

        #[cfg(esp32)]
        {
            // (only) on ESP32 we need to configure DPORT for the SPI DMA channels
            // This assignes the DMA channels to the SPI peripherals, which is more
            // restrictive than necessary but we currently support the same
            // number of SPI peripherals as SPI DMA channels so it's not a big
            // deal.
            let dport = unsafe { &*crate::peripherals::DPORT::PTR };
            dport.spi_dma_chan_sel().modify(|_, w| unsafe {
                w.spi2_dma_chan_sel().bits(1).spi3_dma_chan_sel().bits(2)
            });
        }

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

impl<'d, C, M: Mode> Channel<'d, C, M>
where
    C: DmaChannel,
{
    /// Asserts that the channel is compatible with the given peripheral.
    pub fn runtime_ensure_compatible(&self, peripheral: &PeripheralRef<'_, impl PeripheralMarker>) {
        assert!(
            self.tx.tx_impl.is_compatible_with(&**peripheral),
            "This DMA channel is not compatible with {:?}",
            peripheral.peripheral()
        );
    }
}

/// A marker for SPI-compatible type-erased DMA channels.
pub struct AnySpiDmaChannel;

impl crate::private::Sealed for AnySpiDmaChannel {}

impl DmaChannel for AnySpiDmaChannel {
    type Rx = SpiDmaRxChannelImpl<AnySpiDmaChannelInner>;
    type Tx = SpiDmaTxChannelImpl<AnySpiDmaChannelInner>;

    fn async_handler<M: Mode>(ch: &Channel<'_, Self, M>) -> InterruptHandler {
        match &ch.tx.tx_impl.0 {
            AnySpiDmaChannelInner::Spi2(_) => Spi2DmaChannel::handler(),
            AnySpiDmaChannelInner::Spi3(_) => Spi3DmaChannel::handler(),
        }
    }

    fn interrupts<M: Mode>(ch: &Channel<'_, Self, M>) -> &'static [Interrupt] {
        match &ch.tx.tx_impl.0 {
            AnySpiDmaChannelInner::Spi2(_) => Spi2DmaChannel::isrs(),
            AnySpiDmaChannelInner::Spi3(_) => Spi3DmaChannel::isrs(),
        }
    }
}

crate::any_enum! {
    #[doc(hidden)]
    pub enum AnySpiDmaChannelInner {
        Spi2(Spi2DmaChannel),
        Spi3(Spi3DmaChannel),
    }
}

impl crate::private::Sealed for AnySpiDmaChannelInner {}

impl PdmaChannel for AnySpiDmaChannelInner {
    type RegisterBlock = SpiRegisterBlock;

    delegate::delegate! {
        to match self {
            AnySpiDmaChannelInner::Spi2(channel) => channel,
            AnySpiDmaChannelInner::Spi3(channel) => channel,
        } {
            fn register_block(&self) -> &SpiRegisterBlock;
            fn tx_waker(&self) -> &'static AtomicWaker;
            fn rx_waker(&self) -> &'static AtomicWaker;
            fn is_compatible_with(&self, peripheral: &impl PeripheralMarker) -> bool;
        }
    }
}

/// A marker for I2S-compatible type-erased DMA channels.
pub struct AnyI2sDmaChannel;

impl crate::private::Sealed for AnyI2sDmaChannel {}

impl DmaChannel for AnyI2sDmaChannel {
    type Rx = I2sDmaRxChannelImpl<AnyI2sDmaChannelInner>;
    type Tx = I2sDmaTxChannelImpl<AnyI2sDmaChannelInner>;

    fn async_handler<M: Mode>(ch: &Channel<'_, Self, M>) -> InterruptHandler {
        match &ch.tx.tx_impl.0 {
            AnyI2sDmaChannelInner::I2s0(_) => I2s0DmaChannel::handler(),
            #[cfg(i2s1)]
            AnyI2sDmaChannelInner::I2s1(_) => I2s1DmaChannel::handler(),
        }
    }

    fn interrupts<M: Mode>(ch: &Channel<'_, Self, M>) -> &'static [Interrupt] {
        match &ch.tx.tx_impl.0 {
            AnyI2sDmaChannelInner::I2s0(_) => I2s0DmaChannel::isrs(),
            #[cfg(i2s1)]
            AnyI2sDmaChannelInner::I2s1(_) => I2s1DmaChannel::isrs(),
        }
    }
}

crate::any_enum! {
    #[doc(hidden)]
    pub enum AnyI2sDmaChannelInner {
        I2s0(I2s0DmaChannel),
        #[cfg(i2s1)]
        I2s1(I2s1DmaChannel),
    }
}

impl crate::private::Sealed for AnyI2sDmaChannelInner {}

impl PdmaChannel for AnyI2sDmaChannelInner {
    type RegisterBlock = I2sRegisterBlock;

    delegate::delegate! {
        to match self {
            AnyI2sDmaChannelInner::I2s0(channel) => channel,
            #[cfg(i2s1)]
            AnyI2sDmaChannelInner::I2s1(channel) => channel,
        } {
            fn register_block(&self) -> &I2sRegisterBlock;
            fn tx_waker(&self) -> &'static AtomicWaker;
            fn rx_waker(&self) -> &'static AtomicWaker;
            fn is_compatible_with(&self, peripheral: &impl PeripheralMarker) -> bool;
        }
    }
}
