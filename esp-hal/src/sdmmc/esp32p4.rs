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
pub fn chip_setup() {
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
/// outside the SDHOST block; see `sdmmc_ll`).
///
/// Two sources are supported: [`ClockSource::Pll160m`] (the default, 160 MHz —
/// enough for Default/High Speed) and [`ClockSource::SdioPll`] (the dedicated
/// 200 MHz SDIO PLL, required for the UHS-I SDR50 mode). The module clock is
/// the upper bound for the card clock (`card = module / (2 * div)`, or `module`
/// when the card divider is bypassed).
pub fn set_module_clock(source: ClockSource, div: u8) {
    let c = crate::peripherals::HP_SYS_CLKRST::regs();

    // PLL_F160M is always available; the SDIO PLL must be powered up first.
    let use_sdio_pll = match source {
        ClockSource::SdioPll => {
            enable_sdio_pll();
            true
        }
        _ => {
            // Enable the PLL_F160M reference clock that feeds the SDMMC LS clock
            // (esp-idf `esp_clk_tree_enable_src`). Without this gate `cclk_in` stays
            // at 0, the controller never clocks, and every command times out.
            c.ref_clk_ctrl2()
                .modify(|_, w| w.ref_160m_clk_en().set_bit());
            false
        }
    };

    c.peri_clk_ctrl01().modify(|_, w| {
        w.sdio_ls_clk_src_sel().bit(use_sdio_pll);
        w.sdio_ls_clk_en().set_bit()
    });

    // Low-speed clock divider edges (`h`/`l`/`n`); `div == 1` bypasses the
    // divider via `sdio_hs_mode`.
    if div > 1 {
        // Make sure the divider is not bypassed: a previous `div == 1` session
        // leaves `sdio_hs_mode` set, which would otherwise make the card clock
        // run at the module rate regardless of the divisor programmed here.
        c.peri_clk_ctrl01()
            .modify(|_, w| w.sdio_hs_mode().clear_bit());
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

    // Enable the drive (output), sample (input) and self (core) clocks. Without
    // these the controller never drives the card clock out, so commands never
    // complete.
    //
    // The drive edge starts at 0 (correct for every divided card clock; see
    // [`set_drive_clk_edge`] for why esp-idf's 1 breaks writes at 50 MHz here
    // and why the *bypassed* SDR50 point needs 1). `EngineSession` re-programs
    // it on every clock change.
    c.peri_clk_ctrl02().modify(|_, w| unsafe {
        w.sdio_ls_drv_clk_en().set_bit();
        w.sdio_ls_sam_clk_en().set_bit();
        w.sdio_ls_slf_clk_en().set_bit();
        w.sdio_ls_drv_clk_edge_sel().bits(0);
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

/// Programs the **drive** (host→card) clock edge.
///
/// `sdio_ls_drv_clk_edge_sel` picks which phase of the module clock launches
/// command/data to the card. It is not output-tunable by `tune_bus` (that
/// sweeps only the sample edge), so the value must match the card-clock
/// divider: with the divider active (the card sees every other module edge)
/// edge 0 is correct, but with the divider **bypassed** (`card_div == 0`, the
/// SDR50 100 MHz point) the card samples the same module edge that edge 0
/// launches on. Edge 1 restores the setup margin there; edge 0 at 100 MHz loses
/// the worst-case command argument (`0xaaaa0000`, the RCA) while benign
/// arguments still pass, which shows up as `ResponseTimeout` on the first
/// addressed command after the SDR50 switch.
///
/// `EngineSession::program_slot_hw` calls this on every clock change with
/// `card_div == 0`, so one build serves both the bypassed SDR50 point and the
/// divided High Speed one.
pub fn set_drive_clk_edge(edge: u8) {
    let c = crate::peripherals::HP_SYS_CLKRST::regs();
    c.peri_clk_ctrl02()
        .modify(|_, w| unsafe { w.sdio_ls_drv_clk_edge_sel().bits(edge & 0b11) });
    c.peri_clk_ctrl02()
        .modify(|_, w| w.sdio_ls_clk_edge_cfg_update().set_bit());
    c.peri_clk_ctrl02()
        .modify(|_, w| w.sdio_ls_clk_edge_cfg_update().clear_bit());
}

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

/// Powers up the dedicated 200 MHz SDIO PLL and gates its clock tree.
///
/// The SDIO PLL is a normally-off PLL (unlike PLL_F160M); it must be enabled
/// before it can be selected as the SDMMC source. Mirrors esp-idf
/// `sdmmc_ll_enable_sdio_pll()`. Idempotent.
pub fn enable_sdio_pll() {
    let pmu = crate::peripherals::PMU::regs();

    // Power on the PLL.
    pmu.rf_pwc().modify(|_, w| w.sdio_pll_xpd().set_bit());

    // Tie the PLL power/clock gates high. `PMU_IMM_HP_CK_POWER` bits 20, 26 and
    // 30 (GLOBAL_SDIOPLL_ICG / XPD_SDIOPLL_I2C / XPD_SDIOPLL) are not modelled
    // by the PAC, so set them with a raw read-modify-write.
    let p = pmu.imm_hp_ck_power().as_ptr();
    unsafe {
        const GLOBAL_SDIOPLL_ICG: u32 = 1 << 20;
        const XPD_SDIOPLL_I2C: u32 = 1 << 26;
        const XPD_SDIOPLL: u32 = 1 << 30;
        p.write_volatile(p.read_volatile() | GLOBAL_SDIOPLL_ICG | XPD_SDIOPLL_I2C | XPD_SDIOPLL);
    }

    // Enable the three PLL output clock branches.
    crate::peripherals::LP_AON_CLKRST::regs()
        .lp_aonclkrst_hp_clk_ctrl()
        .modify(|_, w| {
            w.lp_aonclkrst_hp_sdio_pll0_clk_en().set_bit();
            w.lp_aonclkrst_hp_sdio_pll1_clk_en().set_bit();
            w.lp_aonclkrst_hp_sdio_pll2_clk_en().set_bit()
        });

    // Give the PLL time to power up and lock before the source is selected.
    //
    // 10 ms, not the 100 us esp-idf uses: on a *cold* start the PLL has not been
    // powered before, and if the module clock is not running yet the SDHOST
    // produces no events at all -- a command issued then never completes and the
    // driver waits for an interrupt that can never arrive (a hang that a warm
    // reset hides, because the PLL is already locked by then).
    crate::rom::ets_delay_us(10_000);
}

/// Switches the SD IO signalling rail (`LDO_VO4` → `VDDPST_5`/SD_VREF and the
/// bus pull-ups) to **1.8 V** for UHS-I operation.
///
/// With `tieh = 0` the LDO output is `Vref(dref) * (1 + 0.25*mul)`; `dref = 6`,
/// `mul = 5` gives `0.8 V * 2.25 = 1.8 V` (see esp-idf `ldo_ll.h`). Only the
/// `tieh`/`dref`/`mul` fields are touched, matching
/// `esp_ldo_channel_adjust_voltage`; the regulator stays enabled.
pub fn set_io_voltage_1v8() {
    let pmu = crate::peripherals::PMU::regs();

    pmu.ext_ldo_p1_0p2a().modify(|_, w| {
        w._0p2a_force_tieh_sel_1().set_bit(); // software owns the mux
        unsafe { w._0p2a_tieh_sel_1().bits(0) }; // follow `tieh`
        w._0p2a_tieh_1().clear_bit() // 0 = Vref * Mul
    });
    pmu.ext_ldo_p1_0p2a_ana().modify(|_, w| unsafe {
        w.ana_0p2a_dref_1().bits(6);
        w.ana_0p2a_mul_1().bits(5)
    });
}

/// Restores the SD IO signalling rail to the **3.3 V** rail.
///
/// `tieh = 1` ties the LDO output directly to the 3.3 V rail, so `dref`/`mul`
/// do not matter and are left untouched.
pub fn set_io_voltage_3v3() {
    let pmu = crate::peripherals::PMU::regs();

    pmu.ext_ldo_p1_0p2a().modify(|_, w| {
        w._0p2a_force_tieh_sel_1().set_bit();
        unsafe { w._0p2a_tieh_sel_1().bits(0) };
        w._0p2a_tieh_1().set_bit() // tie to the 3.3 V rail
    });
}

/// Enables or removes power from the SD IO rail (`LDO_VO4` -> `VDDPST_5`).
///
/// On boards that tie the SD DAT/CMD pull-ups to this rail, removing power also
/// removes the pull-ups. That matters for a software card power cycle: cutting
/// the card's own VDD switch while the rail stays up leaves the card back-fed
/// through its IO clamp diodes, so it stays partially alive and keeps its 1.8 V
/// signalling latch. With the rail down too the card loses VDD and returns to
/// 3.3 V.
///
/// `true` re-runs the `chip_setup` bring-up (inrush limit, 3.3 V tie, enable,
/// settle), so the rail comes back as at boot.
pub fn set_io_power(enabled: bool) {
    let pmu = crate::peripherals::PMU::regs();
    if enabled {
        chip_setup();
    } else {
        pmu.ext_ldo_p1_0p2a()
            .modify(|_, w| w._0p2a_xpd_1().clear_bit());
    }
}
