use super::PAD_CLK_SEL;
use crate::peripherals::{HP_ALIVE_SYS, HP_SYS_CLKRST};

pub(super) const MAX_DIV_NUM: u32 = 0xFF;

pub(super) fn uses_pad_clock(tx: bool) -> bool {
    let clkrst = HP_SYS_CLKRST::regs();
    if tx {
        clkrst.parlio_tx_ctrl0().read().clk_src_sel().bits() == PAD_CLK_SEL
    } else {
        clkrst.parlio_rx_ctrl0().read().clk_src_sel().bits() == PAD_CLK_SEL
    }
}

pub(super) fn select_pad_clock(tx: bool) {
    let clkrst = HP_SYS_CLKRST::regs();
    let pad_clk = HP_ALIVE_SYS::regs().hp_pad_parlio_ctrl();
    if tx {
        pad_clk.modify(|_, w| w.hp_pad_parlio_tx_clk_en().set_bit());
        clkrst.parlio_tx_ctrl0().modify(|_, w| unsafe {
            w.clk_src_sel().bits(PAD_CLK_SEL);
            w.clk_div_num().bits(0)
        });
    } else {
        pad_clk.modify(|_, w| w.hp_pad_parlio_rx_clk_en().set_bit());
        clkrst.parlio_rx_ctrl0().modify(|_, w| unsafe {
            w.clk_src_sel().bits(PAD_CLK_SEL);
            w.clk_div_num().bits(0)
        });
    }
}

pub(super) fn set_div_num(tx: bool, div_num: u32) {
    let clkrst = HP_SYS_CLKRST::regs();
    if tx {
        clkrst
            .parlio_tx_ctrl0()
            .modify(|_, w| unsafe { w.clk_div_num().bits(div_num as u8) });
    } else {
        clkrst
            .parlio_rx_ctrl0()
            .modify(|_, w| unsafe { w.clk_div_num().bits(div_num as u8) });
    }
}

pub(super) fn set_reset(tx: bool, reset: bool) {
    let clkrst = HP_SYS_CLKRST::regs();
    if tx {
        clkrst
            .parlio_tx_ctrl0()
            .modify(|_, w| w.rst_en().bit(reset));
    } else {
        clkrst
            .parlio_rx_ctrl0()
            .modify(|_, w| w.rst_en().bit(reset));
    }
}
