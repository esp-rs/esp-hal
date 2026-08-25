//! I2S clock programming via `HP_SYS_CLKRST` on ESP32-S31.
//!
//! Register layout differs from ESP32-P4: each instance has dedicated
//! `i2sN_{tx,rx}_{ctrl0,div_ctrl0}` registers. Sequence follows ESP-IDF
//! `i2s_ll_{tx,rx}_set_raw_clk_div`.

use super::master::private::I2sClockDividers;
use crate::{peripherals::HP_SYS_CLKRST, system::Peripheral};

pub(crate) fn set_tx_clock(peripheral: Peripheral, clock_settings: &I2sClockDividers) {
    let clkm_div = clock_settings.mclk_dividers();
    let clkrst = HP_SYS_CLKRST::regs();
    let clock_source = property!("i2s.default_clock_source");

    match peripheral {
        Peripheral::I2s0 => {
            // Bind MCLK to the TX module (`mst_clk_sel` lives on the RX ctrl register).
            clkrst
                .i2s0_rx_ctrl0()
                .modify(|_, w| w.mst_clk_sel().set_bit());

            // Workaround for the double-division issue documented in esp-idf i2s_ll.h.
            clkrst
                .i2s0_tx_ctrl0()
                .modify(|_, w| unsafe { w.tx_div_n().bits(2) });
            clkrst.i2s0_tx_div_ctrl0().modify(|_, w| {
                w.tx_div_yn1().clear_bit();
                unsafe {
                    w.tx_div_y().bits(1);
                    w.tx_div_z().bits(0);
                    w.tx_div_x().bits(0)
                }
            });

            clkrst.i2s0_tx_div_ctrl0().modify(|_, w| {
                w.tx_div_yn1().bit(clkm_div.yn1);
                unsafe {
                    w.tx_div_z().bits(clkm_div.z as u16);
                    w.tx_div_y().bits(clkm_div.y as u16);
                    w.tx_div_x().bits(clkm_div.x as u16)
                }
            });
            clkrst.i2s0_tx_ctrl0().modify(|_, w| unsafe {
                w.tx_div_n().bits(clock_settings.mclk_divider as u8);
                w.tx_clk_en().set_bit();
                w.tx_clk_src_sel().bits(clock_source)
            });
        }
        Peripheral::I2s1 => {
            clkrst
                .i2s1_rx_ctrl0()
                .modify(|_, w| w.mst_clk_sel().set_bit());

            clkrst
                .i2s1_tx_ctrl0()
                .modify(|_, w| unsafe { w.tx_div_n().bits(2) });
            clkrst.i2s1_tx_div_ctrl0().modify(|_, w| {
                w.tx_div_yn1().clear_bit();
                unsafe {
                    w.tx_div_y().bits(1);
                    w.tx_div_z().bits(0);
                    w.tx_div_x().bits(0)
                }
            });

            clkrst.i2s1_tx_div_ctrl0().modify(|_, w| {
                w.tx_div_yn1().bit(clkm_div.yn1);
                unsafe {
                    w.tx_div_z().bits(clkm_div.z as u16);
                    w.tx_div_y().bits(clkm_div.y as u16);
                    w.tx_div_x().bits(clkm_div.x as u16)
                }
            });
            clkrst.i2s1_tx_ctrl0().modify(|_, w| unsafe {
                w.tx_div_n().bits(clock_settings.mclk_divider as u8);
                w.tx_clk_en().set_bit();
                w.tx_clk_src_sel().bits(clock_source)
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
            // Bind MCLK to the RX module, then apply the double-division workaround
            // documented in esp-idf i2s_ll.h (`mst_clk_sel` lives on this register).
            clkrst.i2s0_rx_ctrl0().modify(|_, w| {
                w.mst_clk_sel().clear_bit();
                unsafe { w.rx_div_n().bits(2) }
            });
            clkrst.i2s0_rx_div_ctrl0().modify(|_, w| {
                w.rx_div_yn1().clear_bit();
                unsafe {
                    w.rx_div_y().bits(1);
                    w.rx_div_z().bits(0);
                    w.rx_div_x().bits(0)
                }
            });

            clkrst.i2s0_rx_div_ctrl0().modify(|_, w| {
                w.rx_div_yn1().bit(clkm_div.yn1);
                unsafe {
                    w.rx_div_z().bits(clkm_div.z as u16);
                    w.rx_div_y().bits(clkm_div.y as u16);
                    w.rx_div_x().bits(clkm_div.x as u16)
                }
            });
            clkrst.i2s0_rx_ctrl0().modify(|_, w| unsafe {
                w.rx_div_n().bits(clock_settings.mclk_divider as u8);
                w.rx_clk_en().set_bit();
                w.rx_clk_src_sel().bits(clock_source)
            });
        }
        Peripheral::I2s1 => {
            clkrst.i2s1_rx_ctrl0().modify(|_, w| {
                w.mst_clk_sel().clear_bit();
                unsafe { w.rx_div_n().bits(2) }
            });
            clkrst.i2s1_rx_div_ctrl0().modify(|_, w| {
                w.rx_div_yn1().clear_bit();
                unsafe {
                    w.rx_div_y().bits(1);
                    w.rx_div_z().bits(0);
                    w.rx_div_x().bits(0)
                }
            });

            clkrst.i2s1_rx_div_ctrl0().modify(|_, w| {
                w.rx_div_yn1().bit(clkm_div.yn1);
                unsafe {
                    w.rx_div_z().bits(clkm_div.z as u16);
                    w.rx_div_y().bits(clkm_div.y as u16);
                    w.rx_div_x().bits(clkm_div.x as u16)
                }
            });
            clkrst.i2s1_rx_ctrl0().modify(|_, w| unsafe {
                w.rx_div_n().bits(clock_settings.mclk_divider as u8);
                w.rx_clk_en().set_bit();
                w.rx_clk_src_sel().bits(clock_source)
            });
        }
        _ => unreachable!(),
    }
}
