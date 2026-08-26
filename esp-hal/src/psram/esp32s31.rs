//! PSRAM driver for ESP32-S31.

use super::{EXTMEM_ORIGIN, PsramSize};
use crate::{
    clock::ll::{ClockTree, PsramFunctionClockConfig, PsramInstance},
    peripherals::HP_SYS_CLKRST,
};

mod oct_hex;

/// PSRAM configuration.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub struct PsramConfig {
    /// Size of PSRAM to map. Default: `AutoDetect` via MR2 density
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
    /// For `N` dummy bits, configure `reg_dummy_bits` to `N - 1`
    pub rd_dummy_bits: u8,

    /// Writes dummy length in bits for sync data writes (cache path).
    ///
    /// For `N` dummy bits, configure `reg_dummy_bits` to `N - 1`
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
    oct_hex::enable_dll();
    oct_hex::psram_pad_init(false); // required for DDR strobe latch
    oct_hex::set_cs_timing();

    // SoC MR init (via MSPI3 SPI direct)
    oct_hex::init_mr_registers(&config.timing, false);

    if config.size.is_auto() {
        config.size = PsramSize::Size(oct_hex::psram_detect_size(&config.timing));
    }

    // basic AXI configuration here
    oct_hex::configure_psram_mspi(&config.timing, false);
    oct_hex::mmu_map_psram(config.size.get());

    true
}

pub(crate) fn map_psram(config: PsramConfig) -> core::ops::Range<usize> {
    let start = EXTMEM_ORIGIN;
    start..start + config.size.get()
}

fn enable_psram_mspi() {
    HP_SYS_CLKRST::regs()
        .psram_ctrl0()
        .modify(|_, w| w.sys_clk_en().set_bit());
}

fn reset_psram_mspi() {
    HP_SYS_CLKRST::regs().psram_ctrl0().modify(|_, w| {
        w.axi_rst_en().set_bit();
        w.apb_rst_en().set_bit()
    });
    HP_SYS_CLKRST::regs().psram_ctrl0().modify(|_, w| {
        w.axi_rst_en().clear_bit();
        w.apb_rst_en().clear_bit()
    });
}
