use super::PAD_CLK_SEL;
use crate::{peripherals::PCR, time::Rate};

pub(super) const MAX_DIV_NUM: u32 = 0xFFFF;

pub(super) const DEFAULT_FREQUENCY: Rate = Rate::from_khz(100);

pub(super) fn uses_pad_clock(tx: bool) -> bool {
    let pcr = PCR::regs();
    if tx {
        pcr.parl_clk_tx_conf().read().parl_clk_tx_sel().bits() == PAD_CLK_SEL
    } else {
        pcr.parl_clk_rx_conf().read().parl_clk_rx_sel().bits() == PAD_CLK_SEL
    }
}

pub(super) fn select_pad_clock(tx: bool) {
    let pcr = PCR::regs();
    if tx {
        pcr.parl_clk_tx_conf().modify(|_, w| unsafe {
            w.parl_clk_tx_sel().bits(PAD_CLK_SEL);
            w.parl_clk_tx_div_num().bits(0)
        });
    } else {
        pcr.parl_clk_rx_conf().modify(|_, w| unsafe {
            w.parl_clk_rx_sel().bits(PAD_CLK_SEL);
            w.parl_clk_rx_div_num().bits(0)
        });
    }
}

pub(super) fn set_div_num(tx: bool, div_num: u32) {
    let pcr = PCR::regs();
    if tx {
        pcr.parl_clk_tx_conf()
            .modify(|_, w| unsafe { w.parl_clk_tx_div_num().bits(div_num as u16) });
    } else {
        pcr.parl_clk_rx_conf()
            .modify(|_, w| unsafe { w.parl_clk_rx_div_num().bits(div_num as u16) });
    }
}

pub(super) fn set_reset(tx: bool, reset: bool) {
    let pcr = PCR::regs();
    if tx {
        pcr.parl_clk_tx_conf()
            .modify(|_, w| w.parl_tx_rst_en().bit(reset));
    } else {
        pcr.parl_clk_rx_conf()
            .modify(|_, w| w.parl_rx_rst_en().bit(reset));
    }
}
