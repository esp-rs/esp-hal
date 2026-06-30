//! Module clock is engine-wide (`SdHostController::new`); per-slot card clock
//! is programmed by `EngineSession` on engine acquire.
use super::*;

/// Powers the SD card / SD IO domain from on-chip LDO channel 4 at 3.3 V.
///
/// On the ESP32-P4 the dedicated SD IO pins (GPIO39-44) and the card VDD are
/// supplied by internal LDO channel 4 (analog unit `ext_ldo[4]`), which must
/// be brought up before any card communication or every command response
/// times out. esp-idf does this in the application via
/// `sd_pwr_ctrl_new_on_chip_ldo` (`CONFIG_EXAMPLE_SD_PWR_CTRL_LDO_IO_ID = 4`
/// on the reference board). This is board-specific and belongs in a future
/// regulator API; it lives here for now to bring P4 up.
///
/// Voltage selection (mirrors esp-idf `ldo_ll`): `force_tieh_sel = 1`
/// hands control to software (`tieh_sel`), `tieh_sel = 0` selects the `tieh`
/// bit, and `tieh = 1` ties the output to the 3.3 V rail directly, so the
/// `dref`/`mul` reference is irrelevant. `target0`/`target1` keep their
/// reset (power-on delay) defaults via `modify`.
pub fn enable_sd_io_ldo() {
    let pmu = crate::peripherals::PMU::regs();

    // Limit inrush current while the output cap charges; keep ripple
    // suppression (voltage detector) enabled.
    pmu.ext_ldo_p1_0p2a_ana().modify(|_, w| {
        w.ana_0p2a_en_cur_lim_1().set_bit();
        w.ana_0p2a_en_vdet_1().set_bit()
    });

    // Software-controlled, 3.3 V rail, then enable the regulator.
    pmu.ext_ldo_p1_0p2a().modify(|_, w| {
        w._0p2a_force_tieh_sel_1().set_bit();
        unsafe { w._0p2a_tieh_sel_1().bits(0) };
        w._0p2a_tieh_1().set_bit();
        w._0p2a_xpd_1().set_bit()
    });

    crate::rom::ets_delay_us(500);

    // Drop the inrush current limit once the output has settled.
    pmu.ext_ldo_p1_0p2a_ana()
        .modify(|_, w| w.ana_0p2a_en_cur_lim_1().clear_bit());
    crate::rom::ets_delay_us(100);
}

/// Programs the module clock in `HP_SYS_CLKRST` (P4 keeps the divider/source
/// outside the SDHOST block; see `sdmmc_ll`). Source fixed at PLL160M.
pub fn set_module_clock(_source: ClockSource, div: u8) {
    let c = crate::peripherals::HP_SYS_CLKRST::regs();

    // Enable the PLL_F160M reference clock that feeds the SDMMC LS clock
    // (esp-idf `esp_clk_tree_enable_src`). Without this gate `cclk_in` stays at
    // 0, the controller never clocks, and every command times out.
    c.ref_clk_ctrl2()
        .modify(|_, w| w.ref_160m_clk_en().set_bit());

    c.peri_clk_ctrl01().modify(|_, w| {
        w.sdio_ls_clk_src_sel().bit(false); // PLL_F160M
        w.sdio_ls_clk_en().set_bit()
    });

    // Low-speed clock divider edges (`h`/`l`/`n`); `div == 1` bypasses the
    // divider via `sdio_hs_mode`.
    if div > 1 {
        let h = div / 2 - 1;
        let l = div - 1;
        let n = div - 1;
        c.peri_clk_ctrl02().modify(|_, w| unsafe {
            w.sdio_ls_clk_edge_h().bits(h);
            w.sdio_ls_clk_edge_l().bits(l);
            w.sdio_ls_clk_edge_n().bits(n)
        });
    } else {
        c.peri_clk_ctrl01()
            .modify(|_, w| w.sdio_hs_mode().set_bit());
        c.peri_clk_ctrl02().modify(|_, w| unsafe {
            w.sdio_ls_clk_edge_h().bits(0);
            w.sdio_ls_clk_edge_l().bits(0);
            w.sdio_ls_clk_edge_n().bits(0)
        });
    }

    // Enable the drive (output), sample (input) and self (core) clocks with
    // their default edge phases (drive=1, sample=0, self=0). Without these the
    // controller never drives the card clock out, so commands never complete.
    // Mirrors esp-idf `sdmmc_ll_init_phase_delay`.
    c.peri_clk_ctrl02().modify(|_, w| unsafe {
        w.sdio_ls_drv_clk_en().set_bit();
        w.sdio_ls_sam_clk_en().set_bit();
        w.sdio_ls_slf_clk_en().set_bit();
        w.sdio_ls_drv_clk_edge_sel().bits(1);
        w.sdio_ls_sam_clk_edge_sel().bits(0);
        w.sdio_ls_slf_clk_edge_sel().bits(0)
    });

    // Commit the divider/phase edge configuration.
    c.peri_clk_ctrl02()
        .modify(|_, w| w.sdio_ls_clk_edge_cfg_update().set_bit());
    c.peri_clk_ctrl02()
        .modify(|_, w| w.sdio_ls_clk_edge_cfg_update().clear_bit());
}

/// Card-clock frequency at which the P4 DLL delay path is used (SDR104).
const SDR104_HZ: u32 = 200_000_000;

/// Programs the input sampling delay phase (LS edge select below SDR104, DLL above).
pub fn set_input_delay_phase(phase: DelayPhase, hz: u32) {
    let v = match phase {
        DelayPhase::_0 => 0u8,
        DelayPhase::_1 => 1,
        DelayPhase::_2 => 2,
        DelayPhase::_3 => 3,
    };

    if hz >= SDR104_HZ {
        let bits = v << 3;
        SDHOST::regs().dll_clk_conf().modify(|_, w| unsafe {
            w.dll_cclk_in_sam_phase().bits(bits);
            w.dll_cclk_in_drv_phase().bits(bits)
        });
        return;
    }

    let c = crate::peripherals::HP_SYS_CLKRST::regs();
    c.peri_clk_ctrl02()
        .modify(|_, w| unsafe { w.sdio_ls_sam_clk_edge_sel().bits(v) });
    c.peri_clk_ctrl02()
        .modify(|_, w| w.sdio_ls_clk_edge_cfg_update().set_bit());
    c.peri_clk_ctrl02()
        .modify(|_, w| w.sdio_ls_clk_edge_cfg_update().clear_bit());
}
