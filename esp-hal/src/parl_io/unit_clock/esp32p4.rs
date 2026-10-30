use super::PAD_CLK_SEL;
use crate::{
    peripherals::{HP_SYS_CLKRST, LP_AON_CLKRST},
    time::Rate,
};

pub(super) const MAX_DIV_NUM: u32 = 0xFF;

// The 8-bit divider cannot make 100 kHz from the default 160 MHz source.
pub(super) const DEFAULT_FREQUENCY: Rate = Rate::from_mhz(1);

pub(super) fn uses_pad_clock(tx: bool) -> bool {
    let clkrst = HP_SYS_CLKRST::regs();
    if tx {
        clkrst
            .peri_clk_ctrl118()
            .read()
            .parlio_tx_clk_src_sel()
            .bits()
            == PAD_CLK_SEL
    } else {
        clkrst
            .peri_clk_ctrl117()
            .read()
            .parlio_rx_clk_src_sel()
            .bits()
            == PAD_CLK_SEL
    }
}

pub(super) fn select_pad_clock(tx: bool) {
    let clkrst = HP_SYS_CLKRST::regs();
    let pad_clk = LP_AON_CLKRST::regs().lp_aonclkrst_hp_clk_ctrl();
    if tx {
        pad_clk.modify(|_, w| w.lp_aonclkrst_hp_pad_parlio_tx_clk_en().set_bit());
        clkrst.peri_clk_ctrl118().modify(|_, w| unsafe {
            w.parlio_tx_clk_src_sel().bits(PAD_CLK_SEL);
            w.parlio_tx_clk_div_num().bits(0)
        });
    } else {
        pad_clk.modify(|_, w| w.lp_aonclkrst_hp_pad_parlio_rx_clk_en().set_bit());
        clkrst.peri_clk_ctrl117().modify(|_, w| unsafe {
            w.parlio_rx_clk_src_sel().bits(PAD_CLK_SEL);
            w.parlio_rx_clk_div_num().bits(0)
        });
    }
}

pub(super) fn set_div_num(tx: bool, div_num: u32) {
    let clkrst = HP_SYS_CLKRST::regs();
    if tx {
        clkrst
            .peri_clk_ctrl118()
            .modify(|_, w| unsafe { w.parlio_tx_clk_div_num().bits(div_num as u8) });
    } else {
        clkrst
            .peri_clk_ctrl117()
            .modify(|_, w| unsafe { w.parlio_rx_clk_div_num().bits(div_num as u8) });
    }
}

pub(super) fn set_reset(tx: bool, reset: bool) {
    HP_SYS_CLKRST::regs().hp_rst_en2().modify(|_, w| {
        if tx {
            w.rst_en_parlio_tx().bit(reset)
        } else {
            w.rst_en_parlio_rx().bit(reset)
        }
    });
}
