//! Module clock is engine-wide (`SdHostController::new`); per-slot card clock
//! is programmed by `EngineSession` on engine acquire.

use esp_rom_sys::rom::ets_delay_us;

use super::*;
use crate::{
    clock::ll::{ClockTree, request_mpll_clk},
    peripherals::{CNNT_IO_MUX, CNNT_SYS, HP_SYS_CLKRST},
};

pub fn chip_setup() {
    CNNT_SYS::regs()
        .sys_sdmmc_mem_lp_ctrl()
        .modify(|_, w| unsafe {
            // sdmmc_ll_mem_power_by_pmu
            w.sys_sdmmc_mem_lp_force_ctrl().clear_bit();
            w.sys_sdmmc_mem_lp_en().clear_bit();
            // sdmmc_ll_mem_set_low_power_mode
            w.sys_sdmmc_mem_lp_mode().bits(2) // SDMMC_LL_MEM_LP_MODE_SHUT_DOWN
        });
    // Use CNNT pads
    CNNT_IO_MUX::regs()
        .ctrl()
        .modify(|_, w| w.sdio_pad_pin_ctrl_ded_sel().set_bit());

    ClockTree::with(request_mpll_clk);
}

/// Programs the shared module clock register (divider, source, phases).
///
/// Field encodings differ per chip (see each chip's `sdmmc_ll`).
pub fn set_module_clock(_source: ClockSource, div: u8) {
    // sdmmc_ll_select_clk_source
    HP_SYS_CLKRST::regs().sdio_host_ctrl0().modify(|_, w| {
        w.ls_clk_src_sel().clear_bit(); // MPLL
        w.ls_clk_en().set_bit()
    });

    // sdmmc_ll_set_clock_div
    if div > 1 {
        let l = div - 1;
        let h = (div / 2).saturating_sub(1);
        let n = l;
        HP_SYS_CLKRST::regs()
            .sdio_host_func_ctrl0()
            .modify(|_, w| unsafe {
                w.clk_edge_h().bits(h);
                w.clk_edge_n().bits(n);
                w.clk_edge_l().bits(l);

                w
            });
    } else {
        HP_SYS_CLKRST::regs()
            .sdio_host_ctrl0()
            .modify(|_, w| w.hs_mode().set_bit());
        HP_SYS_CLKRST::regs()
            .sdio_host_func_ctrl0()
            .modify(|_, w| unsafe {
                w.clk_edge_h().bits(0);
                w.clk_edge_n().bits(0);
                w.clk_edge_l().bits(0);

                w
            });
    }

    // sdmmc_ll_init_phase_delay
    HP_SYS_CLKRST::regs()
        .sdio_host_func_ctrl0()
        .modify(|_, w| unsafe {
            w.drv_clk_en().set_bit();
            w.sam_clk_en().set_bit();
            w.slf_clk_en().set_bit();

            w.drv_clk_edge_sel().bits(1);
            w.sam_clk_edge_sel().bits(0);
            w.slf_clk_edge_sel().bits(0);

            w
        });
    reg_update();

    ets_delay_us(10);
}

fn reg_update() {
    HP_SYS_CLKRST::regs()
        .sdio_host_func_ctrl0()
        .modify(|_, w| w.clk_edge_cfg_update().set_bit());
    HP_SYS_CLKRST::regs()
        .sdio_host_func_ctrl0()
        .modify(|_, w| w.clk_edge_cfg_update().clear_bit());
}

pub fn set_input_delay_phase(_phase: DelayPhase, _hz: u32) {
    // TODO
    SDHOST::regs()
        .clk_edge_sel()
        .modify(|_, w| unsafe { w.cclkin_edge_sam_sel().bits(0) });
}
