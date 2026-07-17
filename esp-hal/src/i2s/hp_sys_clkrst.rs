//! I2S clock programming via `HP_SYS_CLKRST`.

use super::master::private::I2sClockDividers;
use crate::{peripherals::HP_SYS_CLKRST, system::Peripheral};

pub(crate) fn set_tx_clock(peripheral: Peripheral, clock_settings: &I2sClockDividers) {
    let clkm_div = clock_settings.mclk_dividers();
    let clkrst = HP_SYS_CLKRST::regs();
    let clock_source = property!("i2s.default_clock_source");

    match peripheral {
        Peripheral::I2s0 => {
            clkrst
                .peri_clk_ctrl14()
                .modify(|_, w| w.i2s0_mst_clk_sel().set_bit());

            // Workaround for the double-division issue documented in esp-idf i2s_ll.h.
            clkrst
                .peri_clk_ctrl13()
                .modify(|_, w| unsafe { w.i2s0_tx_div_n().bits(2) });
            clkrst.peri_clk_ctrl14().modify(|_, w| {
                w.i2s0_tx_div_yn1().clear_bit();
                unsafe {
                    w.i2s0_tx_div_y().bits(1);
                    w.i2s0_tx_div_z().bits(0)
                }
            });
            clkrst
                .peri_clk_ctrl13()
                .modify(|_, w| unsafe { w.i2s0_tx_div_x().bits(0) });

            clkrst.peri_clk_ctrl14().modify(|_, w| {
                w.i2s0_tx_div_yn1().bit(clkm_div.yn1);
                unsafe {
                    w.i2s0_tx_div_z().bits(clkm_div.z as u16);
                    w.i2s0_tx_div_y().bits(clkm_div.y as u16)
                }
            });
            clkrst.peri_clk_ctrl13().modify(|_, w| unsafe {
                w.i2s0_tx_div_x().bits(clkm_div.x as u16);
                w.i2s0_tx_div_n().bits(clock_settings.mclk_divider as u8);
                w.i2s0_tx_clk_en().set_bit();
                w.i2s0_tx_clk_src_sel().bits(clock_source)
            });
        }
        Peripheral::I2s1 => {
            clkrst
                .peri_clk_ctrl17()
                .modify(|_, w| w.i2s1_mst_clk_sel().set_bit());

            clkrst
                .peri_clk_ctrl16()
                .modify(|_, w| unsafe { w.i2s1_tx_div_n().bits(2) });
            // See I2S0 RX: clear `z` before `y`/`x` when they are in different registers.
            clkrst.peri_clk_ctrl17().modify(|_, w| {
                w.i2s1_tx_div_yn1().clear_bit();
                unsafe { w.i2s1_tx_div_z().bits(0) }
            });
            clkrst.peri_clk_ctrl16().modify(|_, w| unsafe {
                w.i2s1_tx_div_y().bits(1);
                w.i2s1_tx_div_x().bits(0)
            });

            clkrst.peri_clk_ctrl17().modify(|_, w| {
                w.i2s1_tx_div_yn1().bit(clkm_div.yn1);
                unsafe { w.i2s1_tx_div_z().bits(clkm_div.z as u16) }
            });
            clkrst.peri_clk_ctrl16().modify(|_, w| unsafe {
                w.i2s1_tx_div_y().bits(clkm_div.y as u16);
                w.i2s1_tx_div_x().bits(clkm_div.x as u16);
                w.i2s1_tx_div_n().bits(clock_settings.mclk_divider as u8)
            });
            clkrst.peri_clk_ctrl15().modify(|_, w| unsafe {
                w.i2s1_tx_clk_en().set_bit();
                w.i2s1_tx_clk_src_sel().bits(clock_source)
            });
        }
        Peripheral::I2s2 => {
            clkrst
                .peri_clk_ctrl19()
                .modify(|_, w| w.i2s2_mst_clk_sel().set_bit());

            clkrst
                .peri_clk_ctrl18()
                .modify(|_, w| unsafe { w.i2s2_tx_div_n().bits(2) });
            clkrst.peri_clk_ctrl19().modify(|_, w| {
                w.i2s2_tx_div_yn1().clear_bit();
                unsafe {
                    w.i2s2_tx_div_y().bits(1);
                    w.i2s2_tx_div_z().bits(0);
                    w.i2s2_tx_div_x().bits(0)
                }
            });

            clkrst.peri_clk_ctrl19().modify(|_, w| {
                w.i2s2_tx_div_yn1().bit(clkm_div.yn1);
                unsafe {
                    w.i2s2_tx_div_z().bits(clkm_div.z as u16);
                    w.i2s2_tx_div_y().bits(clkm_div.y as u16);
                    w.i2s2_tx_div_x().bits(clkm_div.x as u16)
                }
            });
            clkrst.peri_clk_ctrl18().modify(|_, w| unsafe {
                w.i2s2_tx_div_n().bits(clock_settings.mclk_divider as u8);
                w.i2s2_tx_clk_en().set_bit();
                w.i2s2_tx_clk_src_sel().bits(clock_source)
            });
        }
        _ => unreachable!(),
    }
}

pub(crate) fn set_rx_clock(peripheral: Peripheral, clock_settings: &I2sClockDividers) {
    let clkm_div = clock_settings.mclk_dividers();
    let clkrst = HP_SYS_CLKRST::regs();
    let clock_source = property!("i2s.default_clock_source");

    match peripheral {
        Peripheral::I2s0 => {
            clkrst
                .peri_clk_ctrl14()
                .modify(|_, w| w.i2s0_mst_clk_sel().clear_bit());

            // Workaround for the double-division issue documented in esp-idf i2s_ll.h:
            // bypass the divider (`div_n = 2`), reset the fractional terms, then apply
            // the target values. Clear `z` before programming `y`/`x` when they sit in
            // different registers, or the clock can glitch during reconfiguration.
            clkrst
                .peri_clk_ctrl12()
                .modify(|_, w| unsafe { w.i2s0_rx_div_n().bits(2) });
            clkrst.peri_clk_ctrl13().modify(|_, w| {
                w.i2s0_rx_div_yn1().clear_bit();
                unsafe { w.i2s0_rx_div_z().bits(0) }
            });
            clkrst.peri_clk_ctrl12().modify(|_, w| unsafe {
                w.i2s0_rx_div_y().bits(1);
                w.i2s0_rx_div_x().bits(0)
            });

            clkrst.peri_clk_ctrl13().modify(|_, w| {
                w.i2s0_rx_div_yn1().bit(clkm_div.yn1);
                unsafe { w.i2s0_rx_div_z().bits(clkm_div.z as u16) }
            });
            clkrst.peri_clk_ctrl12().modify(|_, w| unsafe {
                w.i2s0_rx_div_y().bits(clkm_div.y as u16);
                w.i2s0_rx_div_x().bits(clkm_div.x as u16);
                w.i2s0_rx_div_n().bits(clock_settings.mclk_divider as u8)
            });
            clkrst.peri_clk_ctrl11().modify(|_, w| unsafe {
                w.i2s0_rx_clk_en().set_bit();
                w.i2s0_rx_clk_src_sel().bits(clock_source)
            });
        }
        Peripheral::I2s1 => {
            clkrst
                .peri_clk_ctrl17()
                .modify(|_, w| w.i2s1_mst_clk_sel().clear_bit());

            // See the I2S0 RX path for the double-division workaround.
            clkrst
                .peri_clk_ctrl14()
                .modify(|_, w| unsafe { w.i2s1_rx_div_n().bits(2) });
            clkrst.peri_clk_ctrl15().modify(|_, w| {
                w.i2s1_rx_div_yn1().clear_bit();
                unsafe {
                    w.i2s1_rx_div_y().bits(1);
                    w.i2s1_rx_div_x().bits(0);
                    w.i2s1_rx_div_z().bits(0)
                }
            });

            clkrst.peri_clk_ctrl15().modify(|_, w| {
                w.i2s1_rx_div_yn1().bit(clkm_div.yn1);
                unsafe {
                    w.i2s1_rx_div_z().bits(clkm_div.z as u16);
                    w.i2s1_rx_div_y().bits(clkm_div.y as u16);
                    w.i2s1_rx_div_x().bits(clkm_div.x as u16)
                }
            });
            clkrst.peri_clk_ctrl14().modify(|_, w| unsafe {
                w.i2s1_rx_div_n().bits(clock_settings.mclk_divider as u8);
                w.i2s1_rx_clk_en().set_bit();
                w.i2s1_rx_clk_src_sel().bits(clock_source)
            });
        }
        Peripheral::I2s2 => {
            clkrst
                .peri_clk_ctrl19()
                .modify(|_, w| w.i2s2_mst_clk_sel().clear_bit());

            // See the I2S0 RX path for the double-division workaround.
            clkrst
                .peri_clk_ctrl17()
                .modify(|_, w| unsafe { w.i2s2_rx_div_n().bits(2) });
            clkrst.peri_clk_ctrl18().modify(|_, w| {
                w.i2s2_rx_div_yn1().clear_bit();
                unsafe {
                    w.i2s2_rx_div_y().bits(1);
                    w.i2s2_rx_div_z().bits(0)
                }
            });
            clkrst
                .peri_clk_ctrl17()
                .modify(|_, w| unsafe { w.i2s2_rx_div_x().bits(0) });

            clkrst.peri_clk_ctrl18().modify(|_, w| {
                w.i2s2_rx_div_yn1().bit(clkm_div.yn1);
                unsafe {
                    w.i2s2_rx_div_z().bits(clkm_div.z as u16);
                    w.i2s2_rx_div_y().bits(clkm_div.y as u16)
                }
            });
            clkrst.peri_clk_ctrl17().modify(|_, w| unsafe {
                w.i2s2_rx_div_x().bits(clkm_div.x as u16);
                w.i2s2_rx_div_n().bits(clock_settings.mclk_divider as u8);
                w.i2s2_rx_clk_en().set_bit();
                w.i2s2_rx_clk_src_sel().bits(clock_source)
            });
        }
        _ => unreachable!(),
    }
}
