//! PSRAM driver for ESP32-S31.

use esp_sync::NonReentrantMutex;

use super::{EXTMEM_ORIGIN, PsramSize};
use crate::{
    clock::ll::{ClockTree, MpllClkConfig, PsramFunctionClockConfig, PsramInstance},
    peripherals::{HP_SYS_CLKRST, MEMSPI2},
};

mod oct_hex;

pub(crate) use oct_hex::low_speed;

/// PSRAM configuration.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub struct PsramConfig {
    /// Size of PSRAM to map. Default: `AutoDetect` via MR2 density.
    pub size: PsramSize,

    /// PSRAM timing parameters. Default: 200 MHz. 250 MHz needs the MPLL at 500 MHz.
    ///
    /// A change of the MPLL rate moves PSRAM on the MPLL to the preset for the new rate:
    /// [`PsramTimingParams::MHZ_200`] at 400 MHz, [`PsramTimingParams::MHZ_250`] at 500 MHz.
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
        Self::MHZ_200
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

    let previous = ClockTree::with(|clocks| {
        PsramInstance::Psram.configure_function_clock(clocks, config.timing.clock_source);
        PsramInstance::Psram.request_function_clock(clocks);
        TIMING.with(|timing| core::mem::replace(timing, config.timing))
    });

    // Controller + PHY pad bring-up.
    if !oct_hex::set_bus_clock(config.timing.clock) {
        // PSRAM did not start, so a later change of the MPLL rate must not retime it.
        ClockTree::with(|clocks| {
            PsramInstance::Psram.release_function_clock(clocks);
            TIMING.with(|timing| *timing = previous);
        });
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

/// The timing that PSRAM runs at. `init_psram` sets it.
static TIMING: NonReentrantMutex<PsramTimingParams> =
    NonReentrantMutex::new(PsramTimingParams::MHZ_200);

/// Returns the timing of PSRAM on an MPLL at `mpll`: the preset at half the MPLL rate.
fn timing_for_mpll(mpll: MpllClkConfig) -> PsramTimingParams {
    match mpll {
        MpllClkConfig::_400 => PsramTimingParams::MHZ_200,
        MpllClkConfig::_500 => PsramTimingParams::MHZ_250,
    }
}

/// Moves the running MPLL to `mpll`, and PSRAM, which runs from the MPLL, to the timing for the
/// new rate.
///
/// The other core is parked, and this core masks its interrupts, until PSRAM runs at the new
/// timing, because no code can run from flash or PSRAM meanwhile. The other core must not wait
/// for the clock tree lock with its interrupts masked: it then never answers the park request.
///
/// A cache freeze does not replace the park: a PSRAM read of the other core that misses the data
/// cache during a retime under a freeze never ends.
pub(crate) fn follow_mpll_rate(mpll: MpllClkConfig) {
    TIMING.with(|timing| {
        let new = timing_for_mpll(mpll);
        let _stopped = OtherCoreStopped::new();
        retime(*timing, new, mpll.value() / 1_000_000);
        *timing = new;
    });
}

/// Parks the other core, if it runs code of this program, and masks the interrupts of this core
/// until the guard drops.
struct OtherCoreStopped {
    parked: Option<crate::system::Cpu>,
    mask: esp_sync::RestoreState,
}

impl OtherCoreStopped {
    fn new() -> Self {
        use esp_sync::raw::{RawLock, SingleCoreInterruptLock};
        match crate::system::multi_core::park_other_masked() {
            Some((core, mask)) => Self {
                parked: Some(core),
                mask,
            },
            None => Self {
                parked: None,
                mask: unsafe { SingleCoreInterruptLock.enter() },
            },
        }
    }
}

impl Drop for OtherCoreStopped {
    fn drop(&mut self) {
        use esp_sync::raw::{RawLock, SingleCoreInterruptLock};
        if let Some(core) = self.parked {
            crate::system::multi_core::unpark(core);
        }
        unsafe { SingleCoreInterruptLock.exit(self.mask) };
    }
}

/// Moves PSRAM from `old` to `new`, and the MPLL to `mpll_mhz`, with the caches suspended.
///
/// The latencies change while the bus runs at the lower of the two clocks, where the longer
/// latencies also work. The DLL gets no settle time after the clock returns, as in the light-sleep
/// wake ([`pmu_sleep_finish`](crate::rtc_cntl::sleep::pmu_sleep_finish)). The timings are passed
/// by value, because a constant in flash or PSRAM cannot be read while the caches are suspended.
#[crate::ram(unstable(checked))]
fn retime(old: PsramTimingParams, new: PsramTimingParams, mpll_mhz: u32) {
    let _caches = crate::soc::cache::CacheGuard::suspend();
    wait_for_idle();

    let faster = new.clock > old.clock;
    if faster {
        set_latencies(&old, &new);
    }
    // PSRAM moves to XTAL_CLK with its clock gated while the MPLL changes.
    HP_SYS_CLKRST::regs().psram_ctrl0().modify(|_, w| unsafe {
        w.clk_src_sel().bits(0);
        w.core_clk_en().clear_bit();
        w.pll_clk_en().clear_bit()
    });
    crate::soc::clocks::set_mpll_rate(mpll_mhz);
    // The presets divide their MPLL rates, see `timing_for_mpll`.
    let _ = oct_hex::set_bus_clock_from(mpll_mhz, new.clock);
    HP_SYS_CLKRST::regs().psram_ctrl0().modify(|_, w| unsafe {
        w.clk_src_sel().bits(1);
        w.pll_clk_en().set_bit();
        w.core_clk_en().set_bit()
    });
    if !faster {
        set_latencies(&old, &new);
    }
}

/// Waits until the PSRAM controller ends every transaction.
#[inline(always)]
pub(crate) fn wait_for_idle() {
    // Bits 26–31 are the six FIFO-empty bits.
    const ALL_TRANSACTION_DONE: u32 = 0xfc00_0000;
    while MEMSPI2::regs().smem_axi_addr_ctrl().read().bits() != ALL_TRANSACTION_DONE {}
}

/// Moves the chip and the controller from `old`'s latencies to `new`'s.
#[inline(always)]
fn set_latencies(old: &PsramTimingParams, new: &PsramTimingParams) {
    oct_hex::retime_mr_registers(old, new);
    oct_hex::configure_psram_mspi(new, false);
}
