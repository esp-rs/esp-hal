//! DMA2D transmit and receive channels.
//!
//! Drivers take one transmit and one receive channel singleton. Disjoint
//! channel pairs can use the controller concurrently.

use core::sync::atomic::{Ordering, compiler_fence};

pub use self::{rx::AnyDma2dRxChannel, tx::AnyDma2dTxChannel};
use crate::{
    peripherals::{DMA2D, HP_SYS},
    soc::{cache_writeback_addr, is_slice_in_psram},
    system::{GenericPeripheralGuard, Peripheral},
};

pub(crate) const CACHE_LINE_SIZE: usize = 64;
pub(crate) const BURST_128_BYTES: u8 = 4;
pub(crate) const MACRO_BLOCK_NONE: u8 = 3;

macro_rules! define_channel {
    ($channel:ident, $doc:literal, $(($index:literal, $name:ident, $variant:ident)),*) => {
        crate::any_peripheral! {
            #[doc = $doc]
            #[instability::unstable]
            pub peripheral $channel<'d> {
                $($variant(crate::peripherals::$name<'d>),)*
            }
        }

        impl $channel<'_> {
            pub(super) fn index(&self) -> usize {
                match &self.0 {
                    $(any::Inner::$variant(_) => $index,)*
                }
            }
        }
    };
}

mod tx {
    for_each_dma2d_channel! {
        (tx $($channel:tt),*) => {
            define_channel!(AnyDma2dTxChannel, "A DMA2D transmit channel singleton.", $($channel),*);
        };
    }
}

mod rx {
    for_each_dma2d_channel! {
        (rx $($channel:tt),*) => {
            define_channel!(AnyDma2dRxChannel, "A DMA2D receive channel singleton.", $($channel),*);
        };
    }
}

fn init_dma2d() {
    // DMA2D transfers stall while SRAM low-power enable is set.
    HP_SYS::regs()
        .dma2d_mem_lp_ctrl()
        .modify(|_, w| w._2ddma_mem_lp_en().clear_bit());

    let dma = DMA2D::regs();
    dma.rst_conf().modify(|_, w| {
        w.clk_en().set_bit();
        w.axim_rd_rst().set_bit();
        w.axim_wr_rst().set_bit()
    });
    dma.rst_conf().modify(|_, w| {
        w.axim_rd_rst().clear_bit();
        w.axim_wr_rst().clear_bit()
    });
}

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(crate) struct Channels<'d> {
    tx: AnyDma2dTxChannel<'d>,
    rx: AnyDma2dRxChannel<'d>,
    _guard: GenericPeripheralGuard<{ Peripheral::Dma2d as u8 }>,
}

impl<'d> Channels<'d> {
    pub(crate) fn new(
        tx: impl Into<AnyDma2dTxChannel<'d>>,
        rx: impl Into<AnyDma2dRxChannel<'d>>,
    ) -> Self {
        Self {
            tx: tx.into(),
            rx: rx.into(),
            _guard: GenericPeripheralGuard::new_with(init_dma2d),
        }
    }

    pub(crate) fn indices(&self) -> (usize, usize) {
        (self.tx.index(), self.rx.index())
    }

    pub(crate) fn reset(&mut self) {
        let (tx, rx) = self.indices();
        let dma = DMA2D::regs();
        dma.out_link_conf_ch(tx)
            .modify(|_, w| w.outlink_stop_ch().set_bit());
        dma.in_link_conf_ch(rx)
            .modify(|_, w| w.inlink_stop_ch().set_bit());
        dma.out_conf0_ch(tx)
            .modify(|_, w| w.out_cmd_disable_ch().set_bit());
        dma.in_conf0_ch(rx)
            .modify(|_, w| w.in_cmd_disable_ch().set_bit());
        // Wait for outstanding AXI transactions before pulsing either channel reset.
        // See <https://github.com/espressif/esp-idf/blob/0e6b480f858c69862ffb6001d96e9eef74599a6e/components/esp_hal_dma/dma2d_hal.c#L17-L31>.
        while !dma.out_state_ch(tx).read().out_reset_avail_ch().bit()
            || !dma.in_state_ch(rx).read().in_reset_avail_ch().bit()
        {
            core::hint::spin_loop();
        }
        dma.out_conf0_ch(tx).modify(|_, w| w.out_rst_ch().set_bit());
        dma.in_conf0_ch(rx).modify(|_, w| w.in_rst_ch().set_bit());
        dma.out_conf0_ch(tx)
            .modify(|_, w| w.out_rst_ch().clear_bit());
        dma.in_conf0_ch(rx).modify(|_, w| w.in_rst_ch().clear_bit());
        dma.out_conf0_ch(tx).reset();
        dma.in_conf0_ch(rx).reset();
        // Disconnect the trigger before another pair can bind to the peripheral.
        dma.out_peri_sel_ch(tx).reset();
        dma.in_peri_sel_ch(rx).reset();
        dma.out_int_clr_ch(tx)
            .write(|w| unsafe { w.bits(u32::MAX) });
        dma.in_int_clr_ch(rx).write(|w| unsafe { w.bits(u32::MAX) });
    }

    /// # Safety
    ///
    /// Both channels must be idle. Their descriptors and buffers must be ready
    /// for DMA and remain valid until the channels stop accessing them.
    #[inline]
    pub(crate) unsafe fn start(&mut self, tx_address: u32, rx_address: u32) {
        let (tx, rx) = self.indices();
        let dma = DMA2D::regs();
        compiler_fence(Ordering::SeqCst);
        dma.out_link_addr_ch(tx)
            .write(|w| unsafe { w.outlink_addr_ch().bits(tx_address) });
        dma.in_link_addr_ch(rx)
            .write(|w| unsafe { w.inlink_addr_ch().bits(rx_address) });
        dma.out_link_conf_ch(tx)
            .modify(|_, w| w.outlink_start_ch().set_bit());
        dma.in_link_conf_ch(rx)
            .modify(|_, w| w.inlink_start_ch().set_bit());
    }

    pub(crate) fn connect(&mut self, trigger: u8) {
        let (tx, rx) = self.indices();
        let dma = DMA2D::regs();
        dma.out_peri_sel_ch(tx)
            .modify(|_, w| unsafe { w.out_peri_sel_ch().bits(trigger) });
        dma.in_peri_sel_ch(rx)
            .modify(|_, w| unsafe { w.in_peri_sel_ch().bits(trigger) });
    }
}

pub(crate) fn writeback<T>(buffer: &[T]) {
    if is_slice_in_psram(buffer) {
        let start = buffer.as_ptr() as usize & !(CACHE_LINE_SIZE - 1);
        let end = (buffer.as_ptr_range().end as usize).next_multiple_of(CACHE_LINE_SIZE);
        unsafe { cache_writeback_addr(start as u32, (end - start) as u32) };
    }
}
