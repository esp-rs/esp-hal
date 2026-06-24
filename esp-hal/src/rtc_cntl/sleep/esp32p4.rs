//! Light- and deep-sleep support for the ESP32-P4 (chip revision v3.x / eco5).
//!
//! v1 scope: timer wakeup only, single core. Ported from the ESP32-C6 PMU sleep
//! driver and adapted to the P4 PMU (DCDC, no wireless modem, no regdma
//! retention) using the esp-idf `pmu_sleep.c` / `pmu_param.h` references.

use core::ops::Not;

use crate::{
    clock::RtcClock,
    rtc_cntl::{
        Rtc,
        rtc::{HpAnalog, HpSysCntlReg, HpSysPower, LpAnalog, LpSysPower, SavedClockConfig},
        sleep::{TimerWakeupSource, WakeSource, WakeTriggers},
    },
    soc::clocks::{self, ClockTree, CpuRootClkConfig, LpSlowClkConfig, TimgCalibrationClockConfig},
};

unsafe fn pmu<'a>() -> &'a esp32p4::pmu::RegisterBlock {
    unsafe { &*esp32p4::PMU::ptr() }
}

// ----------------------------------------------------------------------------
// USB-Serial-JTAG pad handling across light sleep.
//
// In light sleep the HP peripheral domain stays powered, so the USJ PHY keeps
// driving its pads (including the D+ pull-up) while the peripheral is clock
// gated. The host therefore keeps the device enumerated but unresponsive, and
// the link does not recover after wake. Mirroring esp-idf `sleep_console.c`,
// disable the USJ pad (releasing the pull-up so the host sees a clean
// disconnect) on light-sleep entry and restore it on wake. ESP32-P4 does not
// support keeping USJ alive across light sleep, so this is unconditional.
// ----------------------------------------------------------------------------

use core::sync::atomic::{AtomicBool, Ordering};

static USJ_CLOCK_WAS_ENABLED: AtomicBool = AtomicBool::new(false);
static USJ_PAD_WAS_ENABLED: AtomicBool = AtomicBool::new(false);

fn usj_module_is_enabled() -> bool {
    let clkrst = crate::peripherals::HP_SYS_CLKRST::regs();
    let aon = crate::peripherals::LP_AON_CLKRST::regs();
    clkrst
        .soc_clk_ctrl2()
        .read()
        .usb_device_apb_clk_en()
        .bit_is_set()
        && aon
            .lp_aonclkrst_hp_usb_clkrst_ctrl1()
            .read()
            .lp_aonclkrst_rst_en_usb_device()
            .bit_is_clear()
}

fn usj_enable_bus_clock(enable: bool) {
    crate::peripherals::HP_SYS_CLKRST::regs()
        .soc_clk_ctrl2()
        .modify(|_, w| w.usb_device_apb_clk_en().bit(enable));
    // PHY 48 MHz clock for USB FSLS PHY 0.
    crate::peripherals::LP_AON_CLKRST::regs()
        .lp_aonclkrst_hp_usb_clkrst_ctrl0()
        .modify(|_, w| w.lp_aonclkrst_usb_device_48m_clk_en().bit(enable));
}

fn usj_reset_register() {
    let aon = crate::peripherals::LP_AON_CLKRST::regs();
    aon.lp_aonclkrst_hp_usb_clkrst_ctrl1()
        .modify(|_, w| w.lp_aonclkrst_rst_en_usb_device().set_bit());
    aon.lp_aonclkrst_hp_usb_clkrst_ctrl1()
        .modify(|_, w| w.lp_aonclkrst_rst_en_usb_device().clear_bit());
}

fn usj_set_pad_enable(enable: bool) {
    crate::peripherals::USB_DEVICE::regs()
        .conf0()
        .modify(|_, w| w.usb_pad_enable().bit(enable));
}

fn usj_pad_is_enabled() -> bool {
    crate::peripherals::USB_DEVICE::regs()
        .conf0()
        .read()
        .usb_pad_enable()
        .bit_is_set()
}

/// Backup and disable the USJ pad on light-sleep entry. esp-idf
/// `sleep_console_usj_pad_backup_and_disable`.
fn usj_pad_backup_and_disable() {
    let clock_enabled = usj_module_is_enabled();
    let pad_enabled = if clock_enabled {
        usj_pad_is_enabled()
    } else {
        // Bring the register block up so we can touch the pad enable.
        usj_enable_bus_clock(true);
        usj_reset_register();
        false
    };
    usj_set_pad_enable(false);
    usj_enable_bus_clock(false);

    USJ_CLOCK_WAS_ENABLED.store(clock_enabled, Ordering::Relaxed);
    USJ_PAD_WAS_ENABLED.store(pad_enabled, Ordering::Relaxed);
}

/// Restore the USJ pad on wake. esp-idf `sleep_console_usj_pad_restore`.
fn usj_pad_restore() {
    usj_enable_bus_clock(true);
    usj_set_pad_enable(USJ_PAD_WAS_ENABLED.load(Ordering::Relaxed));
    if !USJ_CLOCK_WAS_ENABLED.load(Ordering::Relaxed) {
        usj_enable_bus_clock(false);
    }
}

/// LP SPM RAM base. On rev-3.0 the deep-sleep wake reset vector can be
/// redirected here (see [`install_mspi_workaround_stub`]).
const P4_LP_RAM_BOOT_ADDR: usize = 0x5010_8000;

/// Returns whether this silicon is ESP32-P4 rev 3.0 (ECO5), the only revision
/// affected by the "MSPI crash after power up" deep-sleep erratum (fixed in
/// rev 3.1). Mirrors esp-idf's `efuse_hal_chip_revision() == 300` gate.
fn is_rev3_mspi_workaround_needed() -> bool {
    crate::efuse::chip_revision() == crate::efuse::ChipRevision::from_combined(300)
}

// ESP32-P4 rev-3.0 deep-sleep wake stub (esp-idf
// `p4_rev3_mspi_workaround.S`). On wake the chip powers up and would crash on
// the first flash fetch; this stub runs from LP RAM, stabilizes the MSPI/flash
// cache, resets the MSPI AXI/APB interfaces and then jumps to the HP ROM
// first-stage boot. It is position-independent (only absolute `li`/`jr` and a
// PC-relative delay loop), so it can be copied to `P4_LP_RAM_BOOT_ADDR` and run
// from there. Absolute register addresses are computed from the rev-3.x
// `reg_base.h` (HPPERIPH0 = 0x5000_0000, HPPERIPH1 = 0x500C_0000,
// LPAON = 0x5011_0000).
core::arch::global_asm!(
    ".pushsection .rodata.p4_rev3_mspi_wa, \"a\"",
    ".option push",
    ".option norelax",
    ".option norvc",
    ".balign 4",
    ".global _p4_rev3_mspi_wa_start",
    ".global _p4_rev3_mspi_wa_end",
    "_p4_rev3_mspi_wa_start:",
    // Recover the reset vector to HP ROM: LP_CLKRST_HPCPU_RESET_CTRL0 |= STAT_VECTOR_SEL
    "li   a0, 0x50111014",
    "li   a1, 0x8000",
    "lw   a2, 0(a0)",
    "or   a2, a2, a1",
    "sw   a2, 0(a0)",
    // SPI_MEM_C_CACHE_FCTRL &= ~CLOSE_AXI_INF_EN
    "li   a0, 0x5008C03C",
    "li   a1, 0x80000000",
    "not  a1, a1",
    "lw   a2, 0(a0)",
    "and  a2, a2, a1",
    "sw   a2, 0(a0)",
    // SPI_MEM_C_CACHE_FCTRL |= AXI_REQ_EN
    "li   a1, 0x1",
    "lw   a2, 0(a0)",
    "or   a2, a2, a1",
    "sw   a2, 0(a0)",
    // One MSPI MMU entry mapping AXI addr -> flash addr.
    "li   a0, 0x5008C380", // MMU_ITEM_INDEX = 0
    "sw   zero, 0(a0)",
    "li   a0, 0x5008C37C", // MMU_ITEM_CONTENT = 0x1000
    "li   a1, 0x1000",
    "sw   a1, 0(a0)",
    // Disable cpu error response: CORE_ERR_RESP_DIS = 0x7
    "li   a0, 0x500E51A4",
    "li   a1, 0x7",
    "sw   a1, 0(a0)",
    // Two dummy flash reads to stabilize MSPI.
    "li   a0, 0x80000000",
    "lw   a1, 0(a0)",
    "li   a0, 0x80000080",
    "lw   a1, 0(a0)",
    // Delay ~1us (CPU runs at 40 MHz right after reset).
    "li   t3, 40",
    "csrr t0, cycle",
    "add  t1, t0, t3",
    "100:",
    "csrr t2, cycle",
    "blt  t2, t1, 100b",
    // Re-enable cpu error response: CORE_ERR_RESP_DIS = 0
    "li   a0, 0x500E51A4",
    "sw   zero, 0(a0)",
    // Reset MSPI AXI + APB interfaces, then release.
    "li   a0, 0x500E60C0",
    "li   a1, 0x400000", // RST_EN_MSPI_AXI
    "lw   a2, 0(a0)",
    "or   a2, a2, a1",
    "sw   a2, 0(a0)",
    "li   a1, 0x1000000", // RST_EN_MSPI_APB
    "lw   a2, 0(a0)",
    "or   a2, a2, a1",
    "sw   a2, 0(a0)",
    "li   a1, 0x400000",
    "not  a1, a1",
    "lw   a2, 0(a0)",
    "and  a2, a2, a1",
    "sw   a2, 0(a0)",
    "li   a1, 0x1000000",
    "not  a1, a1",
    "lw   a2, 0(a0)",
    "and  a2, a2, a1",
    "sw   a2, 0(a0)",
    // Jump to HP ROM first-stage boot.
    "li   a5, 0x4FC00000",
    "jr   a5",
    "_p4_rev3_mspi_wa_end:",
    ".option pop",
    ".popsection",
);

/// Copies the rev-3.0 MSPI workaround wake stub into LP RAM at
/// [`P4_LP_RAM_BOOT_ADDR`]. The stub is position-independent, so a plain word
/// copy is sufficient. The first 0x100 bytes of `RTC_FAST` are reserved for it
/// in the linker script.
fn install_mspi_workaround_stub() {
    unsafe extern "C" {
        static _p4_rev3_mspi_wa_start: u8;
        static _p4_rev3_mspi_wa_end: u8;
    }

    let src = &raw const _p4_rev3_mspi_wa_start;
    let end = &raw const _p4_rev3_mspi_wa_end;
    let len = end as usize - src as usize;
    let words = len.div_ceil(4);

    let src = src as *const u32;
    let dst = P4_LP_RAM_BOOT_ADDR as *mut u32;
    for i in 0..words {
        unsafe { dst.add(i).write_volatile(src.add(i).read_volatile()) };
    }
}

/// Redirects (or restores) the HP-core wake reset vector.
///
/// `lp_clkrst_ll_boot_from_lp_ram`: `hpcore0_stat_vector_sel = !boot_from_lp_ram`
/// (0 -> boot from LP SPM RAM 0x50108000, 1 -> boot from HP ROM 0x4FC00000).
fn set_boot_from_lp_ram(boot_from_lp_ram: bool) {
    crate::peripherals::LP_AON_CLKRST::regs()
        .lp_aonclkrst_hpcpu_reset_ctrl0()
        .modify(|_, w| {
            w.lp_aonclkrst_hpcore0_stat_vector_sel()
                .bit(!boot_from_lp_ram)
        });
}

/// ESP32-P4 deep-sleep DCDC -> LDO supply handover.
///
/// The HP digital rail is normally supplied by the on-chip DCDC converter. On
/// deep-sleep entry the PMU FSM powers down the DCDC switch; if the DCDC is
/// still actively regulating at that point, the rail glitches when the LDO has
/// to take over on wake-up and the chip fails to reboot (it appears to "never
/// wake"). esp-idf avoids this by raising the HP LDO so it can take over
/// (`pmu_sleep_increase_ldo_volt`), pre-lowering the DCDC set-point to limit the
/// hand-over overshoot, waiting for the LDO to settle, then disabling the DCDC
/// (`pmu_sleep_shutdown_dcdc`). The DCDC is re-enabled by the bootloader on
/// wake. C-series parts have no DCDC and do not need this.
fn pmu_sleep_dcdc_to_ldo_handover() {
    // esp-idf: LDO_POWER_TAKEOVER_PREPARATION_TIME_US.
    const LDO_TAKEOVER_PREPARATION_TIME_US: u32 = 185;
    // esp-idf: HP_CALI_ACTIVE_DBIAS_DEFAULT.
    const HP_CALI_ACTIVE_DBIAS: u8 = 24;
    // esp-idf pmu_sleep_increase_ldo_volt() constants.
    const LDO_TAKEOVER_DBIAS: u8 = 30;
    const LDO_TAKEOVER_DCM_VSET: u8 = 24;

    let pmu = unsafe { pmu() };

    // pmu_sleep_increase_ldo_volt(): raise the HP LDO and pre-lower the DCDC
    // voltage so the LDO can take over without overshoot.
    pmu.hp_active_hp_regulator0()
        .modify(|_, w| unsafe { w.hp_active_hp_regulator_dbias().bits(LDO_TAKEOVER_DBIAS) });
    pmu.hp_active_hp_regulator0()
        .modify(|_, w| w.hp_active_hp_regulator_xpd().set_bit());
    pmu.hp_active_bias()
        .modify(|_, w| unsafe { w.hp_active_dcm_vset().bits(LDO_TAKEOVER_DCM_VSET) });

    crate::rom::ets_delay_us(LDO_TAKEOVER_PREPARATION_TIME_US);

    // pmu_sleep_shutdown_dcdc(): request the DCDC off (done_force latches it off,
    // the dcdc_switch stays on and is disabled by the PMU when sleep is entered)
    // and drop the HP LDO back to the active default voltage.
    pmu.dcm_ctrl().modify(|_, w| {
        w.dcdc_off_req().set_bit();
        w.dcdc_done_force().set_bit()
    });
    pmu.hp_active_hp_regulator0()
        .modify(|_, w| unsafe { w.hp_active_hp_regulator_dbias().bits(HP_CALI_ACTIVE_DBIAS) });
}

impl WakeSource for TimerWakeupSource {
    fn apply(
        &self,
        rtc: &Rtc<'_>,
        triggers: &mut WakeTriggers,
        _sleep_config: &mut RtcSleepConfig,
    ) {
        triggers.set_timer(true);

        let lp_timer = unsafe { &*esp32p4::LP_TIMER::ptr() };
        let ticks = crate::clock::us_to_rtc_ticks(self.duration.as_micros());
        let now = rtc.time_since_boot_raw();
        let time_in_ticks = now + ticks;
        unsafe {
            lp_timer.tar0_high().write(|w| {
                w.main_timer_tar_high0()
                    .bits(((time_in_ticks >> 32) & 0xffff) as u16)
            });
            lp_timer.tar0_low().write(|w| {
                w.main_timer_tar_low0()
                    .bits((time_in_ticks & 0xffffffff) as u32)
            });
            lp_timer
                .int_clr()
                .write(|w| w.soc_wakeup().clear_bit_by_one());
            lp_timer
                .tar0_high()
                .modify(|_, w| w.main_timer_tar_en0().set_bit());
        }
    }
}

/// Configuration controlling the analog behavior during sleep.
#[derive(Clone, Copy)]
// pmu_sleep_analog_config_t
pub struct AnalogSleepConfig {
    /// High-power system analog configuration.
    pub hp_sys: HpAnalog,
    /// Low-power system analog configuration (LP_SLEEP).
    pub lp_sys_sleep: LpAnalog,
}

impl AnalogSleepConfig {
    fn defaults_deep_sleep() -> Self {
        // PMU_SLEEP_ANALOG_DSLP_CONFIG_DEFAULT
        Self {
            hp_sys: {
                let mut cfg = HpAnalog::default();
                cfg.bias.set_dcm_mode(0);
                cfg.bias.set_pd_cur(true); // PMU_PD_CUR_SLEEP_DEFAULT
                cfg.bias.set_bias_sleep(true); // PMU_BIASSLP_SLEEP_DEFAULT
                cfg.regulator0.set_xpd(false); // PMU_HP_XPD_DEEPSLEEP
                cfg.bias.set_dbg_atten(0); // PMU_DBG_HP_DEEPSLEEP
                cfg
            },
            lp_sys_sleep: {
                let mut cfg = LpAnalog::default();
                cfg.regulator1.set_drv_b(0);
                cfg.bias.set_pd_cur(true);
                cfg.bias.set_bias_sleep(true);
                cfg.regulator0.set_slp_xpd(false);
                cfg.regulator0.set_slp_dbias(0);
                cfg.regulator0.set_xpd(true);
                cfg.bias.set_dbg_atten(12); // PMU_DBG_ATTEN_DEEPSLEEP_DEFAULT
                cfg.regulator0.set_dbias(23); // PMU_LP_DBIAS_DEEPSLEEP_0V7
                cfg
            },
        }
    }

    fn defaults_light_sleep(pd_flags: PowerDownFlags) -> Self {
        // PMU_SLEEP_ANALOG_LSLP_CONFIG_DEFAULT
        let mut this = Self {
            hp_sys: {
                let mut cfg = HpAnalog::default();
                cfg.bias.set_dcm_mode(1);
                cfg.bias.set_dcm_vset(DCM_VSET_IN_SLEEP);
                cfg.regulator1.set_drv_b(0); // PMU_HP_DRVB_LIGHTSLEEP
                cfg.bias.set_pd_cur(true); // PMU_PD_CUR_SLEEP_DEFAULT
                cfg.bias.set_bias_sleep(true); // PMU_BIASSLP_SLEEP_DEFAULT
                cfg.regulator0.set_xpd(false); // PMU_HP_XPD_LIGHTSLEEP (use DCDC)
                cfg.bias.set_dbg_atten(0); // PMU_DBG_ATTEN_LIGHTSLEEP_DEFAULT
                cfg.regulator0.set_dbias(1); // PMU_HP_DBIAS_LIGHTSLEEP_0V6
                cfg
            },
            lp_sys_sleep: {
                let mut cfg = LpAnalog::default();
                cfg.regulator1.set_drv_b(0);
                cfg.bias.set_pd_cur(true);
                cfg.bias.set_bias_sleep(true);
                cfg.regulator0.set_slp_xpd(false);
                cfg.regulator0.set_slp_dbias(0);
                cfg.regulator0.set_xpd(true);
                cfg.bias.set_dbg_atten(0);
                cfg.regulator0.set_dbias(12); // PMU_LP_DBIAS_LIGHTSLEEP_0V7
                cfg
            },
        };

        // When the main XTAL stays powered during sleep, the analog domain must
        // be kept in its active operating point (esp-idf pmu_sleep_config_default).
        if !pd_flags.pd_xtal() {
            this.hp_sys.bias.set_pd_cur(false);
            this.hp_sys.bias.set_bias_sleep(false);
            this.hp_sys.bias.set_dbg_atten(0);
            this.hp_sys.regulator0.set_dbias(HP_CALI_ACTIVE_DBIAS);

            this.lp_sys_sleep.bias.set_pd_cur(false);
            this.lp_sys_sleep.bias.set_bias_sleep(false);
            this.lp_sys_sleep.bias.set_dbg_atten(0);
        }

        this
    }

    fn apply(&self, dslp: bool) {
        // pmu_sleep_analog_init
        unsafe {
            // HP_ACTIVE dcm_mode (deep sleep forces 0, otherwise 1).
            pmu()
                .hp_active_bias()
                .modify(|_, w| w.hp_active_dcm_mode().bits(if dslp { 0 } else { 1 }));

            pmu().hp_sleep_bias().modify(|_, w| {
                w.hp_sleep_dcm_mode().bits(self.hp_sys.bias.dcm_mode());
                w.hp_sleep_dcm_vset().bits(self.hp_sys.bias.dcm_vset());
                w.hp_sleep_dbg_atten().bits(self.hp_sys.bias.dbg_atten());
                w.hp_sleep_pd_cur().bit(self.hp_sys.bias.pd_cur());
                w.sleep().bit(self.hp_sys.bias.bias_sleep())
            });
            pmu().hp_sleep_hp_regulator0().modify(|_, w| {
                w.hp_sleep_hp_regulator_slp_mem_xpd()
                    .bit(self.hp_sys.regulator0.slp_mem_xpd());
                w.hp_sleep_hp_regulator_slp_logic_xpd()
                    .bit(self.hp_sys.regulator0.slp_logic_xpd());
                w.hp_sleep_hp_regulator_xpd()
                    .bit(self.hp_sys.regulator0.xpd());
                w.hp_sleep_hp_regulator_slp_logic_dbias()
                    .bits(self.hp_sys.regulator0.slp_logic_dbias());
                w.hp_sleep_hp_regulator_dbias()
                    .bits(self.hp_sys.regulator0.dbias())
            });
            pmu().hp_sleep_hp_regulator1().modify(|_, w| {
                w.hp_sleep_hp_regulator_drv_b()
                    .bits(self.hp_sys.regulator1.drv_b())
            });

            // LP_SLEEP
            pmu().lp_sleep_bias().modify(|_, w| {
                w.lp_sleep_dbg_atten()
                    .bits(self.lp_sys_sleep.bias.dbg_atten());
                w.lp_sleep_pd_cur().bit(self.lp_sys_sleep.bias.pd_cur());
                w.sleep().bit(self.lp_sys_sleep.bias.bias_sleep())
            });
            pmu().lp_sleep_lp_regulator0().modify(|_, w| {
                w.lp_sleep_lp_regulator_slp_xpd()
                    .bit(self.lp_sys_sleep.regulator0.slp_xpd());
                w.lp_sleep_lp_regulator_xpd()
                    .bit(self.lp_sys_sleep.regulator0.xpd());
                w.lp_sleep_lp_regulator_slp_dbias()
                    .bits(self.lp_sys_sleep.regulator0.slp_dbias());
                w.lp_sleep_lp_regulator_dbias()
                    .bits(self.lp_sys_sleep.regulator0.dbias())
            });
            pmu().lp_sleep_lp_regulator1().modify(|_, w| {
                w.lp_sleep_lp_regulator_drv_b()
                    .bits(self.lp_sys_sleep.regulator1.drv_b())
            });
        }
    }
}

/// Configuration controlling digital peripherals during sleep.
#[derive(Clone, Copy)]
// pmu_sleep_digital_config_t
pub struct DigitalSleepConfig {
    /// High-power system control register configuration.
    pub syscntl: HpSysCntlReg,
}

impl DigitalSleepConfig {
    fn defaults_light_sleep(pd_flags: PowerDownFlags) -> Self {
        // PMU_SLEEP_DIGITAL_LSLP_CONFIG_DEFAULT
        Self {
            syscntl: {
                let mut cfg = HpSysCntlReg::default();
                cfg.set_dig_pad_slp_sel(false);
                // Hold the LP pads if the LP peripheral domain is powered down.
                cfg.set_lp_pad_hold_all(pd_flags.pd_lp_periph());
                cfg.set_dig_pause_wdt(true);
                cfg
            },
        }
    }

    fn apply(&self) {
        // pmu_sleep_digital_init
        unsafe {
            pmu().hp_sleep_hp_sys_cntl().modify(|_, w| {
                w.hp_sleep_dig_pad_slp_sel()
                    .bit(self.syscntl.dig_pad_slp_sel());
                w.hp_sleep_lp_pad_hold_all()
                    .bit(self.syscntl.lp_pad_hold_all());
                w.hp_sleep_dig_pause_wdt().bit(self.syscntl.dig_pause_wdt())
            });
        }
    }
}

/// Configuration controlling the power state of the HP and LP systems during
/// sleep.
#[derive(Clone, Copy)]
// pmu_sleep_power_config_t
pub struct PowerSleepConfig {
    /// Power configuration for the high-power system during sleep.
    pub hp_sys: HpSysPower,
    /// Power configuration for the low-power system when active.
    pub lp_sys_active: LpSysPower,
    /// Power configuration for the low-power system during sleep.
    pub lp_sys_sleep: LpSysPower,
}

impl PowerSleepConfig {
    fn defaults(pd_flags: PowerDownFlags) -> Self {
        let mut this = Self {
            hp_sys: HpSysPower::default(),
            lp_sys_active: LpSysPower::default(),
            lp_sys_sleep: LpSysPower::default(),
        };
        this.apply_flags(pd_flags);
        this
    }

    fn apply_flags(&mut self, pd_flags: PowerDownFlags) {
        // PMU_HP_SLEEP_POWER_CONFIG_DEFAULT + flag overrides.
        // `dcdc_switch_pd_en` is powered down only in deep sleep (which sets
        // `pd_vddsdio`); light sleep keeps the DCDC switch so the DCDC can supply
        // the HP domain at the light-sleep voltage.
        self.hp_sys
            .dig_power
            .set_dcdc_switch_pd_en(pd_flags.pd_vddsdio());
        self.hp_sys.dig_power.set_cnnt_pd_en(pd_flags.pd_modem());
        self.hp_sys.dig_power.set_cpu_pd_en(pd_flags.pd_cpu());
        self.hp_sys.dig_power.set_top_pd_en(pd_flags.pd_top());
        self.hp_sys.dig_power.set_mem_pd_en(pd_flags.pd_mem());

        self.hp_sys.clk.set_i2c_iso_en(true);
        self.hp_sys.clk.set_i2c_retention(true);
        self.hp_sys.clk.set_xpd_pll_i2c(0);
        self.hp_sys.clk.set_xpd_pll(0);

        self.hp_sys.xtal.set_xpd_xtal(pd_flags.pd_xtal().not());

        self.lp_sys_active.clk_power.set_xpd_lppll(true);
        self.lp_sys_active.clk_power.set_xpd_xtal32k(true);
        self.lp_sys_active.clk_power.set_xpd_rc32k(true);
        self.lp_sys_active.clk_power.set_xpd_fosc(true);

        self.lp_sys_sleep
            .dig_power
            .set_peri_pd_en(pd_flags.pd_lp_periph());

        self.lp_sys_sleep
            .clk_power
            .set_xpd_xtal32k(pd_flags.pd_xtal32k().not());
        self.lp_sys_sleep
            .clk_power
            .set_xpd_rc32k(pd_flags.pd_rc32k().not());
        self.lp_sys_sleep
            .clk_power
            .set_xpd_fosc(pd_flags.pd_rc_fast().not());

        self.lp_sys_sleep
            .xtal
            .set_xpd_xtal(pd_flags.pd_xtal().not());
    }

    fn apply(&self) {
        // pmu_sleep_power_init
        unsafe {
            // HP_SLEEP
            pmu()
                .hp_sleep_dig_power()
                .modify(|_, w| w.bits(self.hp_sys.dig_power.0));
            pmu()
                .hp_sleep_hp_ck_power()
                .modify(|_, w| w.bits(self.hp_sys.clk.0));
            pmu()
                .hp_sleep_xtal()
                .modify(|_, w| w.hp_sleep_xpd_xtal().bit(self.hp_sys.xtal.xpd_xtal()));

            // LP_ACTIVE (hp_sleep_lp_*)
            pmu()
                .hp_sleep_lp_dig_power()
                .modify(|_, w| w.bits(self.lp_sys_active.dig_power.0));
            pmu()
                .hp_sleep_lp_ck_power()
                .modify(|_, w| w.bits(self.lp_sys_active.clk_power.0));

            // LP_SLEEP
            pmu()
                .lp_sleep_lp_dig_power()
                .modify(|_, w| w.bits(self.lp_sys_sleep.dig_power.0));
            pmu()
                .lp_sleep_lp_ck_power()
                .modify(|_, w| w.bits(self.lp_sys_sleep.clk_power.0));
            pmu()
                .lp_sleep_xtal()
                .modify(|_, w| w.lp_sleep_xpd_xtal().bit(self.lp_sys_sleep.xtal.xpd_xtal()));
        }
    }
}

/// High-power system sleep timing parameters (pmu_hp_param_t subset).
#[derive(Clone, Copy, Default)]
pub struct HpParam {
    analog_wait_target_cycle: u16,
    digital_power_supply_wait_cycle: u16,
    digital_power_up_wait_cycle: u16,
    pll_stable_wait_cycle: u16,
    min_slp_slow_clk_cycle: u8,
}

/// Low-power system sleep timing parameters (pmu_lp_param_t subset).
#[derive(Clone, Copy, Default)]
pub struct LpParam {
    digital_power_supply_wait_cycle: u16,
    min_slp_slow_clk_cycle: u8,
    analog_wait_target_cycle: u8,
    digital_power_up_wait_cycle: u16,
}

/// Shared HP/LP sleep timing parameters.
#[derive(Clone, Copy, Default)]
pub struct HpLpParam {
    xtal_stable_wait_cycle: u16,
}

/// Sleep timing parameter configuration (pmu_sleep_param_config_t).
#[derive(Clone, Copy)]
pub struct ParamSleepConfig {
    hp_sys: HpParam,
    lp_sys: LpParam,
    hp_lp: HpLpParam,
}

impl ParamSleepConfig {
    fn apply(&self) {
        // pmu_sleep_param_init
        unsafe {
            pmu().slp_wakeup_cntl3().modify(|_, w| {
                w.hp_min_slp_val()
                    .bits(self.hp_sys.min_slp_slow_clk_cycle)
                    .lp_min_slp_val()
                    .bits(self.lp_sys.min_slp_slow_clk_cycle)
            });

            pmu().slp_wakeup_cntl7().modify(|_, w| {
                w.ana_wait_target()
                    .bits(self.hp_sys.analog_wait_target_cycle)
            });

            pmu().power_wait_timer0().modify(|_, w| {
                w.dg_hp_wait_timer()
                    .bits(self.hp_sys.digital_power_supply_wait_cycle)
                    .dg_hp_powerup_timer()
                    .bits(self.hp_sys.digital_power_up_wait_cycle)
            });

            pmu().power_wait_timer1().modify(|_, w| {
                w.dg_lp_wait_timer()
                    .bits(self.lp_sys.digital_power_supply_wait_cycle)
                    .dg_lp_powerup_timer()
                    .bits(self.lp_sys.digital_power_up_wait_cycle)
            });

            pmu().slp_wakeup_cntl5().modify(|_, w| {
                w.lp_ana_wait_target()
                    .bits(self.lp_sys.analog_wait_target_cycle)
            });

            pmu().power_ck_wait_cntl().modify(|_, w| {
                w.pmu_wait_xtl_stable()
                    .bits(self.hp_lp.xtal_stable_wait_cycle)
                    .pmu_wait_pll_stable()
                    .bits(self.hp_sys.pll_stable_wait_cycle)
            });
        }
    }

    fn defaults(config: SleepTimeConfig, pd_flags: PowerDownFlags, pd_xtal: bool) -> Self {
        // pmu_sleep_param_config_default
        let hp_analog_wait_time_us = if pd_flags.pd_top() {
            MachineConstants::HP_ANA_WAIT_TIME_PD_TOP_US
        } else {
            MachineConstants::HP_ANA_WAIT_TIME_PU_TOP_US
        };

        let hp_sys = HpParam {
            min_slp_slow_clk_cycle: config.us_to_slowclk(MachineConstants::HP_MIN_SLP_TIME_US)
                as u8,
            analog_wait_target_cycle: config.us_to_slowclk(hp_analog_wait_time_us) as u16,
            digital_power_supply_wait_cycle: config
                .us_to_fastclk(MachineConstants::HP_POWER_SUPPLY_WAIT_TIME_US)
                as u16,
            digital_power_up_wait_cycle: config
                .us_to_fastclk(MachineConstants::HP_POWER_UP_WAIT_TIME_US)
                as u16,
            pll_stable_wait_cycle: config
                .us_to_fastclk(MachineConstants::HP_PLL_WAIT_STABLE_TIME_US)
                as u16,
        };

        let lp_sys = LpParam {
            min_slp_slow_clk_cycle: config.us_to_slowclk(MachineConstants::LP_MIN_SLP_TIME_US)
                as u8,
            analog_wait_target_cycle: config.us_to_slowclk(MachineConstants::LP_ANALOG_WAIT_TIME_US)
                as u8,
            digital_power_supply_wait_cycle: config
                .us_to_fastclk(MachineConstants::LP_POWER_SUPPLY_WAIT_TIME_US)
                as u16,
            digital_power_up_wait_cycle: config
                .us_to_fastclk(MachineConstants::LP_POWER_UP_WAIT_TIME_US)
                as u16,
        };

        let xtal_stable_wait_cycle = if pd_xtal {
            config.us_to_slowclk(MachineConstants::LP_XTAL_WAIT_STABLE_TIME_US) as u16
        } else {
            config.us_to_fastclk(MachineConstants::HP_XTAL_WAIT_STABLE_TIME_US) as u16
        };

        Self {
            hp_sys,
            lp_sys,
            hp_lp: HpLpParam {
                xtal_stable_wait_cycle,
            },
        }
    }
}

#[derive(Clone, Copy)]
struct SleepTimeConfig {
    sleep_time_adjustment: u32,
    slowclk_period: u32,
    fastclk_period: u32,
}

const CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ: u32 = 360;

impl SleepTimeConfig {
    const RTC_CLK_CAL_FRACT: u32 = 19;

    fn rtc_clk_cal_fast(slowclk_cycles: u32) -> u32 {
        RtcClock::calibrate(TimgCalibrationClockConfig::RcFastClk, slowclk_cycles)
    }

    fn new(_deep: bool) -> Self {
        // rtc slow clock period (saved during clock init in the LP_AON store1).
        let slowclk_period = crate::peripherals::LP_AON::regs()
            .lp_store1()
            .read()
            .lp_scratch1()
            .bits();

        const FAST_CLK_SRC_CAL_CYCLES: u32 = 2048;
        let fastclk_period = Self::rtc_clk_cal_fast(FAST_CLK_SRC_CAL_CYCLES);

        Self {
            sleep_time_adjustment: 0,
            slowclk_period,
            fastclk_period,
        }
    }

    fn light_sleep(pd_flags: PowerDownFlags) -> Self {
        const LIGHT_SLEEP_TIME_OVERHEAD_US: u32 = 56;

        let mut this = Self::new(false);
        let sw = LIGHT_SLEEP_TIME_OVERHEAD_US;
        let hw = this.pmu_sleep_calculate_hw_wait_time(pd_flags);
        this.sleep_time_adjustment = sw + hw;
        this
    }

    fn deep_sleep() -> Self {
        let mut this = Self::new(true);
        this.sleep_time_adjustment = 250 + 100 * 240 / CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;
        this
    }

    fn us_to_slowclk(&self, us: u32) -> u32 {
        (us << Self::RTC_CLK_CAL_FRACT) / self.slowclk_period
    }

    fn slowclk_to_us(&self, rtc_cycles: u32) -> u32 {
        (rtc_cycles * self.slowclk_period) >> Self::RTC_CLK_CAL_FRACT
    }

    fn us_to_fastclk(&self, us: u32) -> u32 {
        (us << Self::RTC_CLK_CAL_FRACT) / self.fastclk_period
    }

    fn pmu_sleep_calculate_hw_wait_time(&self, pd_flags: PowerDownFlags) -> u32 {
        // LP core hardware wait time, microseconds.
        let lp_wakeup_wait_time_us = self.slowclk_to_us(MachineConstants::LP_WAKEUP_WAIT_CYCLE);
        let lp_clk_switch_time_us = self.slowclk_to_us(MachineConstants::LP_CLK_SWITCH_CYCLE);
        // XTAL is not used as the RTC_FAST source here, so the clock power-on
        // wait is measured in slow-clock cycles.
        let lp_clk_power_on_wait_time_us =
            self.slowclk_to_us(MachineConstants::LP_CLK_POWER_ON_WAIT_CYCLE);

        let lp_hw_wait_time_us = MachineConstants::LP_MIN_SLP_TIME_US
            + MachineConstants::LP_ANALOG_WAIT_TIME_US
            + lp_clk_power_on_wait_time_us
            + lp_wakeup_wait_time_us
            + lp_clk_switch_time_us
            + MachineConstants::LP_POWER_SUPPLY_WAIT_TIME_US
            + MachineConstants::LP_POWER_UP_WAIT_TIME_US;

        // HP core hardware wait time, microseconds.
        let hp_analog_wait_time_us = if pd_flags.pd_top() {
            MachineConstants::HP_ANA_WAIT_TIME_PD_TOP_US
        } else {
            MachineConstants::HP_ANA_WAIT_TIME_PU_TOP_US
        };
        let hp_digital_power_up_wait_time_us = MachineConstants::HP_POWER_SUPPLY_WAIT_TIME_US
            + MachineConstants::HP_POWER_UP_WAIT_TIME_US;
        // No regdma retention in v1, so the regdma wait time is 0.
        let hp_regdma_wait_time_us = 0;
        // XTAL is powered down but not used as RTC_FAST, so wait for it to
        // stabilize on wake along with the PLL.
        let hp_clock_wait_time_us = if pd_flags.pd_xtal() {
            MachineConstants::HP_XTAL_WAIT_STABLE_TIME_US
                + MachineConstants::HP_PLL_WAIT_STABLE_TIME_US
        } else {
            MachineConstants::HP_PLL_WAIT_STABLE_TIME_US
        };

        let hp_hw_wait_time_us = hp_analog_wait_time_us
            + hp_digital_power_up_wait_time_us
            + hp_regdma_wait_time_us
            + hp_clock_wait_time_us;

        lp_hw_wait_time_us + hp_hw_wait_time_us
    }
}

/// Configuration for the RTC sleep behavior.
#[derive(Clone, Copy)]
pub struct RtcSleepConfig {
    /// Deep sleep flag.
    pub deep: bool,
    /// Power-down flags.
    pub pd_flags: PowerDownFlags,
}

impl Default for RtcSleepConfig {
    fn default() -> Self {
        Self {
            deep: false,
            pd_flags: PowerDownFlags(0),
        }
    }
}

bitfield::bitfield! {
    #[derive(Clone, Copy)]
    /// Power domains to be powered down during sleep.
    pub struct PowerDownFlags(u32);

    /// Controls the power-down status of the top power domain.
    pub u32, pd_top      , set_pd_top      : 0;
    /// Controls the power-down status of the VDD_SDIO / DCDC switch.
    pub u32, pd_vddsdio  , set_pd_vddsdio  : 1;
    /// Controls the power-down status of the connectivity power domain.
    pub u32, pd_modem    , set_pd_modem    : 2;
    /// Controls the power-down status of the high-performance peripheral domain.
    pub u32, pd_hp_periph, set_pd_hp_periph: 3;
    /// Controls the power-down status of the CPU power domain.
    pub u32, pd_cpu      , set_pd_cpu      : 4;
    /// Controls the power-down status of the high-performance always-on domain.
    pub u32, pd_hp_aon   , set_pd_hp_aon   : 5;
    /// Controls the power-down status of memory group 0.
    pub u32, pd_mem_g0   , set_pd_mem_g0   : 6;
    /// Controls the power-down status of memory group 1.
    pub u32, pd_mem_g1   , set_pd_mem_g1   : 7;
    /// Controls the power-down status of memory group 2.
    pub u32, pd_mem_g2   , set_pd_mem_g2   : 8;
    /// Controls the power-down status of memory group 3.
    pub u32, pd_mem_g3   , set_pd_mem_g3   : 9;
    /// Controls the power-down status of the crystal oscillator.
    pub u32, pd_xtal     , set_pd_xtal     : 10;
    /// Controls the power-down status of the fast RC oscillator.
    pub u32, pd_rc_fast  , set_pd_rc_fast  : 11;
    /// Controls the power-down status of the 32kHz crystal oscillator.
    pub u32, pd_xtal32k  , set_pd_xtal32k  : 12;
    /// Controls the power-down status of the 32kHz RC oscillator.
    pub u32, pd_rc32k    , set_pd_rc32k    : 13;
    /// Controls the power-down status of the low-power peripheral domain.
    pub u32, pd_lp_periph, set_pd_lp_periph: 14;
}

impl PowerDownFlags {
    /// Checks whether all memory groups are powered down.
    pub fn pd_mem(self) -> bool {
        self.pd_mem_g0() && self.pd_mem_g1() && self.pd_mem_g2() && self.pd_mem_g3()
    }

    /// Sets the power-down status for all memory groups at once.
    pub fn set_pd_mem(&mut self, value: bool) {
        self.set_pd_mem_g0(value);
        self.set_pd_mem_g1(value);
        self.set_pd_mem_g2(value);
        self.set_pd_mem_g3(value);
    }
}

// Default DCDC voltage parameter during sleep (Kconfig
// CONFIG_ESP_SLEEP_DCM_VSET_VAL_IN_SLEEP default).
const DCM_VSET_IN_SLEEP: u8 = 14;
// HP active calibration dbias (esp-idf HP_CALI_ACTIVE_DBIAS_DEFAULT).
const HP_CALI_ACTIVE_DBIAS: u8 = 24;

// Constants from `PMU_SLEEP_MC_DEFAULT()` in esp-idf pmu_param.h.
struct MachineConstants;
impl MachineConstants {
    const LP_MIN_SLP_TIME_US: u32 = 450;
    const LP_WAKEUP_WAIT_CYCLE: u32 = 4;
    const LP_ANALOG_WAIT_TIME_US: u32 = 154;
    const LP_XTAL_WAIT_STABLE_TIME_US: u32 = 250;
    const LP_CLK_SWITCH_CYCLE: u32 = 1;
    const LP_CLK_POWER_ON_WAIT_CYCLE: u32 = 1;
    const LP_POWER_SUPPLY_WAIT_TIME_US: u32 = 2;
    const LP_POWER_UP_WAIT_TIME_US: u32 = 2;

    const HP_MIN_SLP_TIME_US: u32 = 450;
    // analog_wait_time depends on whether TOP is powered down.
    const HP_ANA_WAIT_TIME_PD_TOP_US: u32 = 260;
    const HP_REGDMA_S2A_WORK_TIME_US: u32 = 685;
    const HP_ANA_WAIT_TIME_PU_TOP_US: u32 =
        Self::HP_ANA_WAIT_TIME_PD_TOP_US + Self::HP_REGDMA_S2A_WORK_TIME_US;
    const HP_POWER_SUPPLY_WAIT_TIME_US: u32 = 2;
    const HP_POWER_UP_WAIT_TIME_US: u32 = 26;
    const HP_XTAL_WAIT_STABLE_TIME_US: u32 = 250;
    const HP_PLL_WAIT_STABLE_TIME_US: u32 = 50;
}

impl RtcSleepConfig {
    /// Returns whether the device is in deep sleep mode.
    pub fn deep_slp(&self) -> bool {
        self.deep
    }

    /// Configures the device for deep sleep mode.
    pub fn deep() -> Self {
        Self {
            deep: true,
            ..Self::default()
        }
    }

    pub(crate) fn is_deep_sleep(&self) -> bool {
        self.deep_slp()
    }

    pub(crate) fn base_settings(_rtc: &Rtc<'_>) {}

    /// Finalize power-down flags, apply configuration based on the flags.
    pub(crate) fn apply(&mut self) {
        let lp_slow_uses_xtal32k = ClockTree::with(|clocks| {
            matches!(
                clocks::lp_slow_clk_config(clocks),
                Some(LpSlowClkConfig::Xtal32k)
            )
        });

        if self.deep {
            self.pd_flags.set_pd_top(true);
            self.pd_flags.set_pd_vddsdio(true);
            self.pd_flags.set_pd_modem(true);
            self.pd_flags.set_pd_hp_periph(true);
            self.pd_flags.set_pd_cpu(true);
            self.pd_flags.set_pd_mem(true);
            self.pd_flags.set_pd_xtal(true);
            self.pd_flags.set_pd_hp_aon(true);
            self.pd_flags.set_pd_lp_periph(true);
            self.pd_flags.set_pd_xtal32k(!lp_slow_uses_xtal32k);
            self.pd_flags.set_pd_rc32k(true);
            self.pd_flags.set_pd_rc_fast(true);
        } else {
            // Light sleep: the digital domain stays powered (DCDC-supplied at the
            // light-sleep voltage) and only clock-gated, so execution resumes in
            // place. Power down the analog clock sources nothing needs while the
            // core is gated. Powering down XTAL also makes the analog config use
            // the 0.6 V light-sleep operating point.
            self.pd_flags.set_pd_xtal(true);
            self.pd_flags.set_pd_rc_fast(true);
            self.pd_flags.set_pd_xtal32k(!lp_slow_uses_xtal32k);
        }
    }

    /// Configures wakeup options and enters sleep.
    pub(crate) fn start_sleep(&self, wakeup_triggers: WakeTriggers) {
        // ESP32-P4 PMU wakeup-source bitmap (esp-idf `pmu_bit_defs.h`).
        const PMU_SDIO_WAKEUP_EN: u32 = 1 << 0;
        const PMU_GPIO_WAKEUP_EN: u32 = 1 << 2;
        const PMU_USB_WAKEUP_EN: u32 = 1 << 3;
        const PMU_UART1_WAKEUP_EN: u32 = 1 << 7;
        const PMU_UART0_WAKEUP_EN: u32 = 1 << 8;
        const PMU_EXT1_WAKEUP_EN: u32 = 1 << 12;
        const PMU_LP_TIMER_WAKEUP_EN: u32 = 1 << 13;

        const RTC_SLEEP_REJECT_MASK: u32 = PMU_EXT1_WAKEUP_EN
            | PMU_GPIO_WAKEUP_EN
            | PMU_LP_TIMER_WAKEUP_EN
            | PMU_UART0_WAKEUP_EN
            | PMU_UART1_WAKEUP_EN
            | PMU_SDIO_WAKEUP_EN
            | PMU_USB_WAKEUP_EN;

        let wakeup_mask = wakeup_triggers.0 as u32;
        let reject_mask = if self.deep {
            0
        } else {
            wakeup_mask & RTC_SLEEP_REJECT_MASK
        };

        // Switch the CPU root clock to XTAL for the duration of sleep.
        let cpu_freq_config = ClockTree::with(|clocks| {
            let cpu_freq_config = SavedClockConfig::save(clocks);
            clocks::configure_cpu_root_clk(clocks, CpuRootClkConfig::Xtal);
            cpu_freq_config
        });

        let power = PowerSleepConfig::defaults(self.pd_flags);
        power.apply();

        let config = if self.deep {
            SleepTimeConfig::deep_sleep()
        } else {
            SleepTimeConfig::light_sleep(self.pd_flags)
        };

        // `pd_xtal` here means "the main XTAL is powered down during sleep", which
        // selects the slow-clock xtal-stable wait on wake. That is exactly
        // `pd_flags.pd_xtal()`; passing `xpd_xtal` (its inverse) used the fast-clock
        // wait and produced a ~120x too-long wake-up xtal wait.
        let mut param = ParamSleepConfig::defaults(config, self.pd_flags, self.pd_flags.pd_xtal());

        if self.deep {
            const PMU_LP_ANALOG_WAIT_TARGET_TIME_DSLP_US: u32 = 500;
            param.lp_sys.analog_wait_target_cycle =
                config.us_to_slowclk(PMU_LP_ANALOG_WAIT_TARGET_TIME_DSLP_US) as u8;

            AnalogSleepConfig::defaults_deep_sleep().apply(true);
        } else {
            AnalogSleepConfig::defaults_light_sleep(self.pd_flags).apply(false);
            DigitalSleepConfig::defaults_light_sleep(self.pd_flags).apply();
        }

        param.apply();

        // ESP32-P4 rev 3.0 (ECO5) "MSPI crash after power up" deep-sleep
        // erratum: redirect the wake reset vector to a stub in LP RAM that
        // recovers MSPI before the first flash fetch (esp-idf pmu_sleep.c).
        let mspi_workaround = self.deep && is_rev3_mspi_workaround_needed();
        if mspi_workaround {
            install_mspi_workaround_stub();
            set_boot_from_lp_ram(true);
        }

        // like esp-idf pmu_sleep_start()
        unsafe {
            // lp_aon_hal_inform_wakeup_type: on P4 RTC_SLEEP_MODE_REG is
            // LP_SYSTEM_REG_LP_STORE8 (bit0 = run deep-sleep wake stub). The ROM
            // reads this on wake to pick the deep vs light wake path.
            crate::peripherals::LP_AON::regs()
                .lp_store8()
                .modify(|r, w| w.bits(r.bits() & !0x01 | self.deep as u32));

            pmu().slp_wakeup_cntl2().write(|w| w.bits(wakeup_mask));

            pmu().slp_wakeup_cntl1().modify(|_, w| {
                w.slp_reject_en().bit(true);
                w.sleep_reject_ena().bits(reject_mask)
            });

            pmu()
                .slp_wakeup_cntl4()
                .write(|w| w.slp_reject_cause_clr().bit(true));

            pmu().int_clr().write(|w| {
                w.sw().clear_bit_by_one();
                w.soc_sleep_reject().clear_bit_by_one();
                w.soc_wakeup().clear_bit_by_one()
            });

            // ESP32-P4 deep-sleep DCDC -> LDO supply handover. The HP digital rail
            // is normally fed by the on-chip DCDC; if it is left running while the
            // PMU powers down the DCDC switch on deep-sleep entry, the rail glitches
            // when the LDO takes over on wake-up and the chip fails to reboot (it
            // looks like it "never wakes"). esp-idf raises the HP LDO so it can take
            // over, waits for it to settle, then disables the DCDC before entering
            // deep sleep (pmu_sleep_increase_ldo_volt + pmu_sleep_shutdown_dcdc).
            // C-series parts have no DCDC and skip this.
            if self.deep {
                pmu_sleep_dcdc_to_ldo_handover();
            }

            // Light sleep keeps the HP domain (and thus the USJ PHY) powered, so
            // de-enumerate USB-Serial-JTAG cleanly before sleeping and restore it
            // in `finish_sleep`. Deep sleep powers the PHY off on its own.
            if !self.deep {
                usj_pad_backup_and_disable();
            }

            // The PMU FSM switches the pads to their sleep setting and holds IOs
            // at the same stage; trigger the pad sleep selection first so the IOs
            // do not get held in an indeterminate state.
            pmu()
                .imm_pad_hold_all()
                .write(|w| w.tie_high_pad_slp_sel().set_bit());

            // FIXME HERE

            // Start entry into sleep mode.
            pmu().slp_wakeup_cntl0().write(|w| w.sleep_req().bit(true));

            // In deep sleep we never get here.
            loop {
                let int_raw = pmu().int_raw().read();
                if int_raw.soc_wakeup().bit_is_set() || int_raw.soc_sleep_reject().bit_is_set() {
                    break;
                }
            }
        }

        // We only reach this point if the (deep) sleep was rejected: the wake
        // stub itself restores the vector on a real wake. Point the vector back
        // at the HP ROM so a later reset boots normally.
        if mspi_workaround {
            set_boot_from_lp_ram(false);
        }

        ClockTree::with(|clocks| {
            cpu_freq_config.restore(clocks);
        });
    }

    /// Cleans up after sleep.
    pub(crate) fn finish_sleep(&self) {
        // like esp-idf pmu_sleep_finish(): switch the pad configuration back from
        // the sleep state to the active state. In deep sleep we never get here.
        unsafe {
            pmu()
                .imm_pad_hold_all()
                .write(|w| w.tie_low_pad_slp_sel().set_bit());
        }

        // Re-enumerate USB-Serial-JTAG (only disabled for light sleep; in deep
        // sleep we never reach here).
        if !self.deep {
            usj_pad_restore();
        }
    }
}
