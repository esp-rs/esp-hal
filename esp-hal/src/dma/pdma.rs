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

use portable_atomic::{AtomicBool, Ordering};

use crate::{
    asynch::AtomicWaker,
    dma::*,
    interrupt::Priority,
    macros::handler,
    peripheral::{Peripheral, PeripheralRef},
    peripherals::Interrupt,
    system::{self, PeripheralClockControl},
};

type SpiRegisterBlock = crate::peripherals::spi2::RegisterBlock;
type I2sRegisterBlock = crate::peripherals::i2s0::RegisterBlock;

#[doc(hidden)]
pub trait PdmaChannel: crate::private::Sealed {
    type RegisterBlock;

    fn register_block(&self) -> &Self::RegisterBlock;
    fn tx_waker(&self) -> &'static AtomicWaker;
    fn rx_waker(&self) -> &'static AtomicWaker;
    fn is_compatible_with(&self, peripheral: DmaPeripheral) -> bool;

    fn peripheral_interrupt(&self) -> Interrupt;
    fn async_handler(&self) -> InterruptHandler;
    fn rx_async_flag(&self) -> &'static AtomicBool;
    fn tx_async_flag(&self) -> &'static AtomicBool;
}

/// The RX half of an arbitrary SPI DMA channel.
pub struct AnySpiDmaRxChannel(AnySpiDmaChannel);

impl crate::private::Sealed for AnySpiDmaRxChannel {}
impl DmaRxChannel for AnySpiDmaRxChannel {}
impl Peripheral for AnySpiDmaRxChannel {
    type P = Self;

    unsafe fn clone_unchecked(&self) -> Self::P {
        Self(self.0.clone_unchecked())
    }
}

/// The TX half of an arbitrary SPI DMA channel.
pub struct AnySpiDmaTxChannel(AnySpiDmaChannel);

impl crate::private::Sealed for AnySpiDmaTxChannel {}
impl DmaTxChannel for AnySpiDmaTxChannel {}
impl Peripheral for AnySpiDmaTxChannel {
    type P = Self;

    unsafe fn clone_unchecked(&self) -> Self::P {
        Self(self.0.clone_unchecked())
    }
}

impl RegisterAccess for AnySpiDmaTxChannel {
    fn reset(&self) {
        let spi = self.0.register_block();
        spi.dma_conf().modify(|_, w| w.out_rst().set_bit());
        spi.dma_conf().modify(|_, w| w.out_rst().clear_bit());
    }

    fn set_burst_mode(&self, burst_mode: BurstConfig) {
        let spi = self.0.register_block();
        spi.dma_conf()
            .modify(|_, w| w.out_data_burst_en().bit(burst_mode.is_burst_enabled()));
    }

    fn set_descr_burst_mode(&self, burst_mode: bool) {
        let spi = self.0.register_block();
        spi.dma_conf()
            .modify(|_, w| w.outdscr_burst_en().bit(burst_mode));
    }

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

    fn is_compatible_with(&self, peripheral: DmaPeripheral) -> bool {
        self.0.is_compatible_with(peripheral)
    }
}

impl TxRegisterAccess for AnySpiDmaTxChannel {
    fn set_auto_write_back(&self, enable: bool) {
        // there is no `auto_wrback` for SPI
        assert!(!enable);
    }

    fn last_dscr_address(&self) -> usize {
        let spi = self.0.register_block();
        spi.out_eof_des_addr().read().dma_out_eof_des_addr().bits() as usize
    }

    fn peripheral_interrupt(&self) -> Option<Interrupt> {
        None
    }

    fn async_handler(&self) -> Option<InterruptHandler> {
        None
    }
}

impl InterruptAccess<DmaTxInterrupt> for AnySpiDmaTxChannel {
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
        });
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
        });
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

    fn is_async(&self) -> bool {
        self.0.tx_async_flag().load(Ordering::Acquire)
    }

    fn set_async(&self, is_async: bool) {
        self.0.tx_async_flag().store(is_async, Ordering::Release);
    }
}

impl RegisterAccess for AnySpiDmaRxChannel {
    fn reset(&self) {
        let spi = self.0.register_block();
        spi.dma_conf().modify(|_, w| w.in_rst().set_bit());
        spi.dma_conf().modify(|_, w| w.in_rst().clear_bit());
    }

    fn set_burst_mode(&self, _burst_mode: BurstConfig) {}

    fn set_descr_burst_mode(&self, burst_mode: bool) {
        let spi = self.0.register_block();
        spi.dma_conf()
            .modify(|_, w| w.indscr_burst_en().bit(burst_mode));
    }

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

    fn is_compatible_with(&self, peripheral: DmaPeripheral) -> bool {
        self.0.is_compatible_with(peripheral)
    }
}

impl RxRegisterAccess for AnySpiDmaRxChannel {
    fn peripheral_interrupt(&self) -> Option<Interrupt> {
        Some(self.0.peripheral_interrupt())
    }

    fn async_handler(&self) -> Option<InterruptHandler> {
        Some(self.0.async_handler())
    }
}

impl InterruptAccess<DmaRxInterrupt> for AnySpiDmaRxChannel {
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
        });
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
        spi.dma_int_clr().write(|w| {
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
        });
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

    fn is_async(&self) -> bool {
        self.0.rx_async_flag().load(Ordering::Relaxed)
    }

    fn set_async(&self, _is_async: bool) {
        self.0.rx_async_flag().store(_is_async, Ordering::Relaxed);
    }
}

/// The RX half of an arbitrary I2S DMA channel.
pub struct AnyI2sDmaRxChannel(AnyI2sDmaChannel);

impl crate::private::Sealed for AnyI2sDmaRxChannel {}
impl DmaRxChannel for AnyI2sDmaRxChannel {}
impl Peripheral for AnyI2sDmaRxChannel {
    type P = Self;

    unsafe fn clone_unchecked(&self) -> Self::P {
        Self(self.0.clone_unchecked())
    }
}

/// The TX half of an arbitrary I2S DMA channel.
pub struct AnyI2sDmaTxChannel(AnyI2sDmaChannel);

impl crate::private::Sealed for AnyI2sDmaTxChannel {}
impl DmaTxChannel for AnyI2sDmaTxChannel {}
impl Peripheral for AnyI2sDmaTxChannel {
    type P = Self;

    unsafe fn clone_unchecked(&self) -> Self::P {
        Self(self.0.clone_unchecked())
    }
}

impl RegisterAccess for AnyI2sDmaTxChannel {
    fn reset(&self) {
        let reg_block = self.0.register_block();
        reg_block.lc_conf().modify(|_, w| w.out_rst().set_bit());
        reg_block.lc_conf().modify(|_, w| w.out_rst().clear_bit());
    }

    fn set_burst_mode(&self, burst_mode: BurstConfig) {
        let reg_block = self.0.register_block();
        reg_block
            .lc_conf()
            .modify(|_, w| w.out_data_burst_en().bit(burst_mode.is_burst_enabled()));
    }

    fn set_descr_burst_mode(&self, burst_mode: bool) {
        let reg_block = self.0.register_block();
        reg_block
            .lc_conf()
            .modify(|_, w| w.outdscr_burst_en().bit(burst_mode));
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

    fn is_compatible_with(&self, peripheral: DmaPeripheral) -> bool {
        self.0.is_compatible_with(peripheral)
    }
}

impl TxRegisterAccess for AnyI2sDmaTxChannel {
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

    fn peripheral_interrupt(&self) -> Option<Interrupt> {
        None
    }

    fn async_handler(&self) -> Option<InterruptHandler> {
        None
    }
}

impl InterruptAccess<DmaTxInterrupt> for AnyI2sDmaTxChannel {
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
        });
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
        });
    }

    fn waker(&self) -> &'static AtomicWaker {
        self.0.tx_waker()
    }

    fn is_async(&self) -> bool {
        self.0.tx_async_flag().load(Ordering::Relaxed)
    }

    fn set_async(&self, _is_async: bool) {
        self.0.tx_async_flag().store(_is_async, Ordering::Relaxed);
    }
}

impl RegisterAccess for AnyI2sDmaRxChannel {
    fn reset(&self) {
        let reg_block = self.0.register_block();
        reg_block.lc_conf().modify(|_, w| w.in_rst().set_bit());
        reg_block.lc_conf().modify(|_, w| w.in_rst().clear_bit());
    }

    fn set_burst_mode(&self, _burst_mode: BurstConfig) {}

    fn set_descr_burst_mode(&self, burst_mode: bool) {
        let reg_block = self.0.register_block();
        reg_block
            .lc_conf()
            .modify(|_, w| w.indscr_burst_en().bit(burst_mode));
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

    fn is_compatible_with(&self, peripheral: DmaPeripheral) -> bool {
        self.0.is_compatible_with(peripheral)
    }
}

impl RxRegisterAccess for AnyI2sDmaRxChannel {
    fn peripheral_interrupt(&self) -> Option<Interrupt> {
        Some(self.0.peripheral_interrupt())
    }

    fn async_handler(&self) -> Option<InterruptHandler> {
        Some(self.0.async_handler())
    }
}

impl InterruptAccess<DmaRxInterrupt> for AnyI2sDmaRxChannel {
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
        });
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
        });
    }

    fn waker(&self) -> &'static AtomicWaker {
        self.0.rx_waker()
    }

    fn is_async(&self) -> bool {
        self.0.rx_async_flag().load(Ordering::Relaxed)
    }

    fn set_async(&self, _is_async: bool) {
        self.0.rx_async_flag().store(_is_async, Ordering::Relaxed);
    }
}

macro_rules! ImplPdmaChannel {
    ($peri:ident, $register_block:ident, $instance:ident, $int:ident, [$($compatible:ident),*]) => {
        paste::paste! {
            #[doc = concat!("DMA channel suitable for ", stringify!([< $instance:upper >]))]
            #[non_exhaustive]
            pub struct [<$instance DmaChannel>] {}

            impl $crate::private::Sealed for [<$instance DmaChannel>] {}

            impl Peripheral for [<$instance DmaChannel>] {
                type P = Self;

                unsafe fn clone_unchecked(&self) -> Self::P {
                    Self::steal()
                }
            }

            impl [<$instance DmaChannel>] {
                /// Unsafely constructs a new DMA channel.
                ///
                /// # Safety
                ///
                /// The caller must ensure that only a single instance is used.
                pub unsafe fn steal() -> Self {
                    Self {}
                }
            }

            impl DmaChannel for [<$instance DmaChannel>] {
                type Rx = [<$peri DmaRxChannel>];
                type Tx = [<$peri DmaTxChannel>];

                unsafe fn split_internal(self, _: $crate::private::Internal) -> (Self::Rx, Self::Tx) {
                    ([<$peri DmaRxChannel>](Self {}.into()), [<$peri DmaTxChannel>](Self {}.into()))
                }
            }

            impl DmaChannelExt for [<$instance DmaChannel>] {
                fn rx_interrupts() -> impl InterruptAccess<DmaRxInterrupt> {
                    [<$peri DmaRxChannel>](Self {}.into())
                }
                fn tx_interrupts() -> impl InterruptAccess<DmaTxInterrupt> {
                    [<$peri DmaTxChannel>](Self {}.into())
                }
            }

            impl PdmaChannel for [<$instance DmaChannel>] {
                type RegisterBlock = $register_block;

                fn register_block(&self) -> &Self::RegisterBlock {
                    unsafe { &*crate::peripherals::[< $instance:upper >]::PTR }
                }
                fn tx_waker(&self) -> &'static AtomicWaker {
                    static WAKER: AtomicWaker = AtomicWaker::new();
                    &WAKER
                }
                fn rx_waker(&self) -> &'static AtomicWaker {
                    static WAKER: AtomicWaker = AtomicWaker::new();
                    &WAKER
                }
                fn is_compatible_with(&self, peripheral: DmaPeripheral) -> bool {
                    let compatible_peripherals = [$(DmaPeripheral::$compatible),*];
                    compatible_peripherals.contains(&peripheral)
                }

                fn peripheral_interrupt(&self) -> Interrupt {
                    Interrupt::$int
                }

                fn async_handler(&self) -> InterruptHandler {
                    #[handler(priority = Priority::max())]
                    pub(crate) fn interrupt_handler() {
                        super::asynch::handle_in_interrupt::<[< $instance DmaChannel >]>();
                        super::asynch::handle_out_interrupt::<[< $instance DmaChannel >]>();
                    }

                    interrupt_handler
                }
                fn rx_async_flag(&self) -> &'static AtomicBool {
                    static FLAG: AtomicBool = AtomicBool::new(false);
                    &FLAG
                }
                fn tx_async_flag(&self) -> &'static AtomicBool {
                    static FLAG: AtomicBool = AtomicBool::new(false);
                    &FLAG
                }
            }

            impl DmaChannelConvert<[<$peri DmaChannel>]> for [<$instance DmaChannel>] {
                fn degrade(self) -> [<$peri DmaChannel>] {
                    self.into()
                }
            }

            impl DmaChannelConvert<[<$peri DmaRxChannel>]> for [<$instance DmaChannel>] {
                fn degrade(self) -> [<$peri DmaRxChannel>] {
                    [<$peri DmaRxChannel>](self.into())
                }
            }

            impl DmaChannelConvert<[<$peri DmaTxChannel>]> for [<$instance DmaChannel>] {
                fn degrade(self) -> [<$peri DmaTxChannel>] {
                    [<$peri DmaTxChannel>](self.into())
                }
            }
        }
    };
}

ImplPdmaChannel!(AnySpi, SpiRegisterBlock, Spi2, SPI2_DMA, [Spi2]);
ImplPdmaChannel!(AnySpi, SpiRegisterBlock, Spi3, SPI3_DMA, [Spi3]);

ImplPdmaChannel!(AnyI2s, I2sRegisterBlock, I2s0, I2S0, [I2s0]);
#[cfg(i2s1)]
ImplPdmaChannel!(AnyI2s, I2sRegisterBlock, I2s1, I2S1, [I2s1]);

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
    pub spi2channel: Spi2DmaChannel,
    /// DMA channel for SPI3
    pub spi3channel: Spi3DmaChannel,
    /// DMA channel for I2S0
    pub i2s0channel: I2s0DmaChannel,
    /// DMA channel for I2S1
    #[cfg(i2s1)]
    pub i2s1channel: I2s1DmaChannel,
}

impl<'d> Dma<'d> {
    /// Create a DMA instance.
    pub fn new(dma: impl Peripheral<P = crate::peripherals::DMA> + 'd) -> Dma<'d> {
        if PeripheralClockControl::enable(system::Peripheral::Dma) {
            PeripheralClockControl::reset(system::Peripheral::Dma);
        }

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

        unsafe {
            Dma {
                _inner: dma.into_ref(),
                spi2channel: Spi2DmaChannel::steal(),
                spi3channel: Spi3DmaChannel::steal(),
                i2s0channel: I2s0DmaChannel::steal(),
                #[cfg(i2s1)]
                i2s1channel: I2s1DmaChannel::steal(),
            }
        }
    }
}

impl<'d, CH, M> Channel<'d, M, CH>
where
    CH: DmaChannel,
    M: Mode,
{
    /// Asserts that the channel is compatible with the given peripheral.
    pub fn runtime_ensure_compatible(&self, peripheral: &PeripheralRef<'_, impl DmaEligible>) {
        assert!(
            self.tx
                .tx_impl
                .is_compatible_with(peripheral.dma_peripheral()),
            "This DMA channel is not compatible with {:?}",
            peripheral.dma_peripheral()
        );
    }
}

crate::any_peripheral! {
    /// An SPI-compatible type-erased DMA channel.
    pub peripheral AnySpiDmaChannel {
        Spi2(Spi2DmaChannel),
        Spi3(Spi3DmaChannel),
    }
}

impl DmaChannel for AnySpiDmaChannel {
    type Rx = AnySpiDmaRxChannel;
    type Tx = AnySpiDmaTxChannel;

    unsafe fn split_internal(self, _: crate::private::Internal) -> (Self::Rx, Self::Tx) {
        (
            AnySpiDmaRxChannel(unsafe { self.clone_unchecked() }),
            AnySpiDmaTxChannel(unsafe { self.clone_unchecked() }),
        )
    }
}

impl PdmaChannel for AnySpiDmaChannel {
    type RegisterBlock = SpiRegisterBlock;

    delegate::delegate! {
        to match &self.0 {
            AnySpiDmaChannelInner::Spi2(channel) => channel,
            AnySpiDmaChannelInner::Spi3(channel) => channel,
        } {
            fn register_block(&self) -> &SpiRegisterBlock;
            fn tx_waker(&self) -> &'static AtomicWaker;
            fn rx_waker(&self) -> &'static AtomicWaker;
            fn is_compatible_with(&self, peripheral: DmaPeripheral) -> bool;
            fn peripheral_interrupt(&self) -> Interrupt;
            fn async_handler(&self) -> InterruptHandler;
            fn rx_async_flag(&self) -> &'static AtomicBool;
            fn tx_async_flag(&self) -> &'static AtomicBool;
        }
    }
}

crate::any_peripheral! {
    /// An I2S-compatible type-erased DMA channel.
    pub peripheral AnyI2sDmaChannel {
        I2s0(I2s0DmaChannel),
        #[cfg(i2s1)]
        I2s1(I2s1DmaChannel),
    }
}

impl DmaChannel for AnyI2sDmaChannel {
    type Rx = AnyI2sDmaRxChannel;
    type Tx = AnyI2sDmaTxChannel;

    unsafe fn split_internal(self, _: crate::private::Internal) -> (Self::Rx, Self::Tx) {
        (
            AnyI2sDmaRxChannel(unsafe { self.clone_unchecked() }),
            AnyI2sDmaTxChannel(unsafe { self.clone_unchecked() }),
        )
    }
}

impl PdmaChannel for AnyI2sDmaChannel {
    type RegisterBlock = I2sRegisterBlock;

    delegate::delegate! {
        to match &self.0 {
            AnyI2sDmaChannelInner::I2s0(channel) => channel,
            #[cfg(i2s1)]
            AnyI2sDmaChannelInner::I2s1(channel) => channel,
        } {
            fn register_block(&self) -> &I2sRegisterBlock;
            fn tx_waker(&self) -> &'static AtomicWaker;
            fn rx_waker(&self) -> &'static AtomicWaker;
            fn is_compatible_with(&self, peripheral: DmaPeripheral) -> bool;
            fn peripheral_interrupt(&self) -> Interrupt;
            fn async_handler(&self) -> InterruptHandler;
            fn rx_async_flag(&self) -> &'static AtomicBool;
            fn tx_async_flag(&self) -> &'static AtomicBool;
        }
    }
}
