//! PSRAM driver for ESP32-P4.

use super::{EXTMEM_ORIGIN, PsramSize};
use crate::{
    clock::ll::{ClockTree, PsramFunctionClockConfig, PsramInstance},
    efuse,
    peripherals::{HP_SYS, HP_SYS_CLKRST, MEMSPI2, PMU},
};

mod oct_hex;

/// PSRAM interface mode (line count of the data bus).
#[derive(Copy, Clone, Debug, Default, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub enum PsramMode {
    /// 16-line DDR. Default.
    #[default]
    Hex,

    /// 8-line DDR.
    Oct,
}

/// PSRAM configuration.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub struct PsramConfig {
    /// PSRAM interface mode.
    pub mode: PsramMode,

    /// Size of PSRAM to map. Default: `AutoDetect` via MR2 density.
    pub size: PsramSize,

    /// PSRAM timing parameters. Default: 250 MHz.
    pub timing: PsramTimingParams,
    // TODO: ECC enable.
    // The MSPI0 controller has a ECC engine.
    // pub ecc: bool, // or any other enum.
}

/// PSRAM timing parameters.
///
/// These values should be tuned together with the source clock frequency. The source clock
/// frequency must be an integer multiple of the PSRAM frequency.
#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PsramTimingParams {
    /// PSRAM clock source.
    pub clock_source: PsramFunctionClockConfig,

    /// Bus clock frequency in MHz. Must divide the source clock evenly.
    pub clock: u32,

    /// MR0.read_latency field value (cycles = 2 * value + 6).
    pub mr0_rl: u8,

    /// MR4.wr_latency field value.
    pub mr4_wl: u8,

    /// Reads dummy length in bits for sync data reads (cache path).
    ///
    /// For `N` dummy bits, configure `reg_dummy_bits` to `N - 1`.
    pub rd_dummy_bits: u8,

    /// Writes dummy length in bits for sync data writes (cache path).
    ///
    /// For `N` dummy bits, configure `reg_dummy_bits` to `N - 1`.
    pub wr_dummy_bits: u8,

    /// Register-read dummy length for direct command path (MSPI3).
    pub reg_dummy_bits: u32,
}

impl Default for PsramTimingParams {
    fn default() -> Self {
        Self::MHZ_250
    }
}

impl PsramTimingParams {
    /// Preset for 20 MHz clock speed.
    pub const MHZ_20: Self = Self {
        clock_source: PsramFunctionClockConfig::Mpll,
        clock: 20,
        mr0_rl: 2,
        mr4_wl: 2,
        rd_dummy_bits: 17,
        wr_dummy_bits: 7,
        reg_dummy_bits: 8,
    };

    /// Preset for 80 MHz clock speed.
    pub const MHZ_80: Self = Self {
        clock_source: PsramFunctionClockConfig::Mpll,
        clock: 80,
        mr0_rl: 2,
        mr4_wl: 2,
        rd_dummy_bits: 17,
        wr_dummy_bits: 7,
        reg_dummy_bits: 8,
    };

    /// Preset for 125 MHz clock speed.
    pub const MHZ_125: Self = Self {
        clock_source: PsramFunctionClockConfig::Mpll,
        clock: 125,
        mr0_rl: 2,
        mr4_wl: 2,
        rd_dummy_bits: 17,
        wr_dummy_bits: 7,
        reg_dummy_bits: 8,
    };

    /// Preset for 200 MHz clock speed.
    pub const MHZ_200: Self = Self {
        clock_source: PsramFunctionClockConfig::Mpll,
        clock: 200,
        mr0_rl: 4,
        mr4_wl: 1,
        rd_dummy_bits: 25,
        wr_dummy_bits: 11,
        reg_dummy_bits: 12,
    };

    /// Preset for 250 MHz clock speed.
    pub const MHZ_250: Self = Self {
        clock_source: PsramFunctionClockConfig::Mpll,
        clock: 250,
        mr0_rl: 6,
        mr4_wl: 3,
        rd_dummy_bits: 33,
        wr_dummy_bits: 15,
        reg_dummy_bits: 16,
    };
}

/// Initializes PSRAM.
#[crate::ram]
pub(crate) fn init_psram(config: &mut PsramConfig) -> bool {
    psram_phy_ldo_init();

    // Module clock + clock source
    enable_psram_mspi();
    reset_psram_mspi();

    ClockTree::with(|clocks| {
        PsramInstance::Psram.configure_function_clock(clocks, config.timing.clock_source);
        PsramInstance::Psram.request_function_clock(clocks);
    });

    // Controller + PHY pad bring-up.
    if !oct_hex::set_bus_clock(config.timing.clock) {
        return false;
    }
    let is_hex = matches!(config.mode, PsramMode::Hex);
    oct_hex::enable_dll();
    oct_hex::psram_pad_init(is_hex); // required for DDR strobe latch
    oct_hex::set_cs_timing();

    // SoC MR init (via MSPI3 SPI direct)
    oct_hex::init_mr_registers(&config.timing, is_hex);

    if config.size.is_auto() {
        config.size = PsramSize::Size(oct_hex::psram_detect_size(&config.timing));
    }

    // basic AXI configuration here
    oct_hex::configure_psram_mspi(&config.timing, is_hex);

    // Silicon revision 3.0 (ECO5) requires two dummy PSRAM reads + a controller
    // reset before the real MMU mapping is committed.  Port of IDF's
    // `esp_psram_p4_rev3_workaround` in `esp_psram.c`.
    if efuse::chip_revision() == efuse::ChipRevision::from_combined(300) {
        debug!("Applying P4 v3.0 PSRAM workaround");
        p4_rev3_psram_workaround();
    }

    oct_hex::mmu_map_psram(config.size.get());

    true
}

pub(crate) fn map_psram(config: PsramConfig) -> core::ops::Range<usize> {
    let start = EXTMEM_ORIGIN;
    start..start + config.size.get()
}

/// Programs the PMU external LDO regulators for the MSPI PHY.
fn psram_phy_ldo_init() {
    PMU::regs()
        .ext_ldo_p0_0p1a()
        .write(|w| unsafe { w.bits(0x4020_0100) });
    PMU::regs()
        .ext_ldo_p0_0p1a_ana()
        .write(|w| unsafe { w.bits(0xB100_0000) });

    PMU::regs()
        .ext_ldo_p0_0p2a()
        .write(|w| unsafe { w.bits(0x4020_0000) });
    PMU::regs()
        .ext_ldo_p0_0p2a_ana()
        .write(|w| unsafe { w.bits(0xA000_0000) });

    PMU::regs()
        .ext_ldo_p0_0p3a()
        .write(|w| unsafe { w.bits(0x4020_0000) });
    PMU::regs()
        .ext_ldo_p0_0p3a_ana()
        .write(|w| unsafe { w.bits(0xA000_0000) });

    PMU::regs()
        .ext_ldo_p1_0p1a()
        .write(|w| unsafe { w.bits(0x4020_0180) });
    PMU::regs()
        .ext_ldo_p1_0p1a_ana()
        .write(|w| unsafe { w.bits(0x5700_0000) });

    PMU::regs()
        .ext_ldo_p1_0p2a()
        .write(|w| unsafe { w.bits(0x4020_0000) });
    PMU::regs()
        .ext_ldo_p1_0p2a_ana()
        .write(|w| unsafe { w.bits(0xA000_0000) });

    PMU::regs()
        .ext_ldo_p1_0p3a()
        .write(|w| unsafe { w.bits(0x4020_0000) });
    PMU::regs()
        .ext_ldo_p1_0p3a_ana()
        .write(|w| unsafe { w.bits(0xA000_0000) });

    // Allow LDO output to settle before the MSPI PHY is exercised.
    crate::rom::ets_delay_us(50);
}

fn enable_psram_mspi() {
    HP_SYS_CLKRST::regs()
        .soc_clk_ctrl0()
        .modify(|_, w| w.psram_sys_clk_en().set_bit());
}

fn reset_psram_mspi() {
    HP_SYS_CLKRST::regs().hp_rst_en0().modify(|_, w| {
        w.rst_en_dual_mspi_axi().set_bit();
        w.rst_en_dual_mspi_apb().set_bit()
    });
    HP_SYS_CLKRST::regs().hp_rst_en0().modify(|_, w| {
        w.rst_en_dual_mspi_axi().clear_bit();
        w.rst_en_dual_mspi_apb().clear_bit()
    });
}

/// ESP32-P4 silicon revision 3.0 workaround.
///
/// Port of IDF `esp_psram_p4_rev3_workaround` (`esp_psram.c`). Must be called
/// after `configure_psram_mspi` and before `mmu_map_psram`.
fn p4_rev3_psram_workaround() {
    // Snapshot the MSPI0 registers before the reset wipes them.
    let cache_fctrl = MEMSPI2::regs().cache_fctrl().read().bits();
    let cache_sctrl = MEMSPI2::regs().cache_sctrl().read().bits();
    let sram_cmd = MEMSPI2::regs().sram_cmd().read().bits();
    let sram_drd_cmd = MEMSPI2::regs().sram_drd_cmd().read().bits();
    let sram_dwr_cmd = MEMSPI2::regs().sram_dwr_cmd().read().bits();
    let sram_clk = MEMSPI2::regs().sram_clk().read().bits();
    let ctrl1 = MEMSPI2::regs().ctrl1().read().bits();
    let smem_ddr = MEMSPI2::regs().smem_ddr().read().bits();
    let timing_cali = MEMSPI2::regs().timing_cali().read().bits();
    let smem_timing_cali = MEMSPI2::regs().smem_timing_cali().read().bits();
    let smem_ac = MEMSPI2::regs().smem_ac().read().bits();

    // Suppress CPU bus-error response so the dummy reads below don't trap.
    HP_SYS::regs()
        .core_err_resp_dis()
        .write(|w| unsafe { w.bits(0x7) });

    unsafe {
        // Map physical PSRAM page 0 to MMU entry 0 so both 0x4800_0000 and
        // its uncached alias 0x8800_0000 reach the PSRAM hardware.
        oct_hex::write_psram_mmu_entry(0, 0);

        // Two dummy reads at the uncached PSRAM alias; result is discarded.
        // The reads are expected to fail (garbage data) but must hit the controller.
        let _ = core::ptr::read_volatile(0x8800_0000u32 as *const u32);
        let _ = core::ptr::read_volatile(0x8800_0080u32 as *const u32);

        crate::rom::ets_delay_us(1);
    }

    // Pulse the PSRAM controller reset (AXI then APB), mirroring IDF's
    // `_psram_ctrlr_ll_reset_module_clock`.
    reset_psram_mspi();

    // Re-enable bus-error responses.
    HP_SYS::regs()
        .core_err_resp_dis()
        .write(|w| unsafe { w.bits(0) });

    // Restore the MSPI0 registers cleared by the reset.
    unsafe {
        MEMSPI2::regs().cache_fctrl().write(|w| w.bits(cache_fctrl));
        MEMSPI2::regs().cache_sctrl().write(|w| w.bits(cache_sctrl));
        MEMSPI2::regs().sram_cmd().write(|w| w.bits(sram_cmd));
        MEMSPI2::regs()
            .sram_drd_cmd()
            .write(|w| w.bits(sram_drd_cmd));
        MEMSPI2::regs()
            .sram_dwr_cmd()
            .write(|w| w.bits(sram_dwr_cmd));
        MEMSPI2::regs().sram_clk().write(|w| w.bits(sram_clk));
        MEMSPI2::regs().ctrl1().write(|w| w.bits(ctrl1));
        MEMSPI2::regs().smem_ddr().write(|w| w.bits(smem_ddr));
        MEMSPI2::regs().timing_cali().write(|w| w.bits(timing_cali));
        MEMSPI2::regs()
            .smem_timing_cali()
            .write(|w| w.bits(smem_timing_cali));
        MEMSPI2::regs().smem_ac().write(|w| w.bits(smem_ac));
    }
}
