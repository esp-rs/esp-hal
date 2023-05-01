use fugit::HertzU32;
use strum::FromRepr;

use crate::clock::Clock;

use crate::{
    clock::{clocks_ll::regi2c_write_mask, Clock, XtalClock},
    peripherals::{LP_AON, LP_CLKRST, PCR, PMU, TIMG0},
};

const I2C_PMU: u8 = 0x6d;
const I2C_PMU_HOSTID: u8 = 0;

const I2C_PMU_EN_I2C_RTC_DREG: u8 = 8;
const I2C_PMU_EN_I2C_RTC_DREG_MSB: u8 = 0;
const I2C_PMU_EN_I2C_RTC_DREG_LSB: u8 = 0;

const I2C_PMU_EN_I2C_DIG_DREG: u8 = 8;
const I2C_PMU_EN_I2C_DIG_DREG_MSB: u8 = 1;
const I2C_PMU_EN_I2C_DIG_DREG_LSB: u8 = 1;

const I2C_PMU_EN_I2C_RTC_DREG_SLP: u8 = 8;
const I2C_PMU_EN_I2C_RTC_DREG_SLP_MSB: u8 = 2;
const I2C_PMU_EN_I2C_RTC_DREG_SLP_LSB: u8 = 2;

const I2C_PMU_EN_I2C_DIG_DREG_SLP: u8 = 8;
const I2C_PMU_EN_I2C_DIG_DREG_SLP_MSB: u8 = 3;
const I2C_PMU_EN_I2C_DIG_DREG_SLP_LSB: u8 = 3;

const I2C_PMU_OR_XPD_RTC_REG: u8 = 8;
const I2C_PMU_OR_XPD_RTC_REG_MSB: u8 = 4;
const I2C_PMU_OR_XPD_RTC_REG_LSB: u8 = 4;

const I2C_PMU_OR_XPD_DIG_REG: u8 = 8;
const I2C_PMU_OR_XPD_DIG_REG_MSB: u8 = 5;
const I2C_PMU_OR_XPD_DIG_REG_LSB: u8 = 5;

const I2C_PMU_OR_XPD_TRX: u8 = 15;
const I2C_PMU_OR_XPD_TRX_MSB: u8 = 2;
const I2C_PMU_OR_XPD_TRX_LSB: u8 = 2;

const DR_REG_PMU_BASE: u32 = 0x600B0000;

const PMU_POWER_PD_TOP_CNTL_REG: u32 = DR_REG_PMU_BASE + 0xf4;
const PMU_POWER_PD_HPAON_CNTL_REG: u32 = DR_REG_PMU_BASE + 0xf8;
const PMU_POWER_PD_HPCPU_CNTL_REG: u32 = DR_REG_PMU_BASE + 0xfc;
const PMU_POWER_PD_HPPERI_RESERVE_REG: u32 = DR_REG_PMU_BASE + 0x100;
const PMU_POWER_PD_HPWIFI_CNTL_REG: u32 = DR_REG_PMU_BASE + 0x104;
const PMU_POWER_PD_LPPERI_CNTL_REG: u32 = DR_REG_PMU_BASE + 0x108;

const PMU_SLP_WAKEUP_CNTL5_REG: u32 = DR_REG_PMU_BASE + 0x134;
const PMU_SLP_WAKEUP_CNTL7_REG: u32 = DR_REG_PMU_BASE + 0x13c;


pub(crate) fn init() {
    // * No peripheral reg i2c power up required on the target */

    unsafe {
        regi2c_write_mask(
            I2C_PMU,
            I2C_PMU_HOSTID,
            I2C_PMU_EN_I2C_RTC_DREG,
            I2C_PMU_EN_I2C_RTC_DREG_MSB,
            I2C_PMU_EN_I2C_RTC_DREG_LSB,
            0,
        );
        regi2c_write_mask(
            I2C_PMU,
            I2C_PMU_HOSTID,
            I2C_PMU_EN_I2C_DIG_DREG,
            I2C_PMU_EN_I2C_DIG_DREG_MSB,
            I2C_PMU_EN_I2C_DIG_DREG_LSB,
            0,
        );
        regi2c_write_mask(
            I2C_PMU,
            I2C_PMU_HOSTID,
            I2C_PMU_EN_I2C_RTC_DREG_SLP,
            I2C_PMU_EN_I2C_RTC_DREG_SLP_MSB,
            I2C_PMU_EN_I2C_RTC_DREG_SLP_LSB,
            0,
        );
        regi2c_write_mask(
            I2C_PMU,
            I2C_PMU_HOSTID,
            I2C_PMU_EN_I2C_DIG_DREG_SLP,
            I2C_PMU_EN_I2C_DIG_DREG_SLP_MSB,
            I2C_PMU_EN_I2C_DIG_DREG_SLP_LSB,
            0,
        );
        regi2c_write_mask(
            I2C_PMU,
            I2C_PMU_HOSTID,
            I2C_PMU_OR_XPD_RTC_REG,
            I2C_PMU_OR_XPD_RTC_REG_MSB,
            I2C_PMU_OR_XPD_RTC_REG_LSB,
            0,
        );
        regi2c_write_mask(
            I2C_PMU,
            I2C_PMU_HOSTID,
            I2C_PMU_OR_XPD_DIG_REG,
            I2C_PMU_OR_XPD_DIG_REG_MSB,
            I2C_PMU_OR_XPD_DIG_REG_LSB,
            0,
        );
        regi2c_write_mask(
            I2C_PMU,
            I2C_PMU_HOSTID,
            I2C_PMU_OR_XPD_TRX,
            I2C_PMU_OR_XPD_TRX_MSB,
            I2C_PMU_OR_XPD_TRX_LSB,
            0,
        );

        (PMU_POWER_PD_TOP_CNTL_REG as *mut u32).write_volatile(0);
        (PMU_POWER_PD_HPAON_CNTL_REG as *mut u32).write_volatile(0);
        (PMU_POWER_PD_HPCPU_CNTL_REG as *mut u32).write_volatile(0);
        (PMU_POWER_PD_HPPERI_RESERVE_REG as *mut u32).write_volatile(0);
        (PMU_POWER_PD_HPWIFI_CNTL_REG as *mut u32).write_volatile(0);
        (PMU_POWER_PD_LPPERI_CNTL_REG as *mut u32).write_volatile(0);

        let pmu = &*PMU::ptr();

        pmu.hp_active_hp_regulator0
        .modify(|_, w| w.hp_active_hp_regulator_dbias().bits(25));
        pmu.hp_sleep_lp_regulator0
        .modify(|_, w| w.hp_sleep_lp_regulator_dbias().bits(26));

        pmu.slp_wakeup_cntl5
        .modify(|_, w| w.lp_ana_wait_target().bits(15));
        pmu.slp_wakeup_cntl7
        .modify(|_, w| w.ana_wait_target().bits(1700));
    }
}

pub(crate) fn configure_clock() {
    assert!(matches!(
        RtcClock::get_xtal_freq(),
        XtalClock::RtcXtalFreq40M
    ));

    RtcClock::set_fast_freq(RtcFastClock::RtcFastClockRcFast);

    let cal_val = loop {
        RtcClock::set_slow_freq(RtcSlowClock::RtcSlowClockRcSlow);

        let res = RtcClock::calibrate(RtcCalSel::RtcCalRtcMux, 1024);
        if res != 0 {
            break res;
        }
    };

    unsafe {
        let lp_aon = &*LP_AON::ptr();
        lp_aon.store1.modify(|_, w| w.bits(cal_val));
    }

    modem_clk_domain_active_state_icg_map_preinit();
}

// Terminology:
//
// CPU Reset:    Reset CPU core only, once reset done, CPU will execute from
//               reset vector
// Core Reset:   Reset the whole digital system except RTC sub-system
// System Reset: Reset the whole digital system, including RTC sub-system
// Chip Reset:   Reset the whole chip, including the analog part

#[derive(Debug, Clone, Copy, PartialEq, Eq, FromRepr)]
pub enum SocResetReason {
    /// Power on reset
    ///
    /// In ESP-IDF this value (0x01) can *also* be `ChipBrownOut` or
    /// `ChipSuperWdt`, however that is not really compatible with Rust-style
    /// enums.
    ChipPowerOn   = 0x01,
    /// Software resets the digital core by RTC_CNTL_SW_SYS_RST
    CoreSw        = 0x03,
    /// Deep sleep reset the digital core
    CoreDeepSleep = 0x05,
    /// Main watch dog 0 resets digital core
    CoreMwdt0     = 0x07,
    /// Main watch dog 1 resets digital core
    CoreMwdt1     = 0x08,
    /// RTC watch dog resets digital core
    CoreRtcWdt    = 0x09,
    /// Main watch dog 0 resets CPU 0
    Cpu0Mwdt0     = 0x0B,
    /// Software resets CPU 0 by RTC_CNTL_SW_PROCPU_RST
    Cpu0Sw        = 0x0C,
    /// RTC watch dog resets CPU 0
    Cpu0RtcWdt    = 0x0D,
    /// VDD voltage is not stable and resets the digital core
    SysBrownOut   = 0x0F,
    /// RTC watch dog resets digital core and rtc module
    SysRtcWdt     = 0x10,
    /// Main watch dog 1 resets CPU 0
    Cpu0Mwdt1     = 0x11,
    /// Super watch dog resets the digital core and rtc module
    SysSuperWdt   = 0x12,
    /// Glitch on clock resets the digital core and rtc module
    SysClkGlitch  = 0x13,
    /// eFuse CRC error resets the digital core
    CoreEfuseCrc  = 0x14,
    /// USB UART resets the digital core
    CoreUsbUart   = 0x15,
    /// USB JTAG resets the digital core
    CoreUsbJtag   = 0x16,
    /// Glitch on power resets the digital core
    CorePwrGlitch = 0x17,
}

/// RTC SLOW_CLK frequency values
#[derive(Debug, Clone, Copy)]
pub(crate) enum RtcFastClock {
    /// Select RC_FAST_CLK as RTC_FAST_CLK source
    RtcFastClockRcFast = 0,
    /// Select XTAL_D2_CLK as RTC_FAST_CLK source
    RtcFastClockXtalD2 = 1,
    /// Select LP_PLL_CLK as RTC_FAST_CLK source
    //RtcFastClockLpPll  = 2, // TODO : Check if it's used
}

impl Clock for RtcFastClock {
    fn frequency(&self) -> HertzU32 {
        match self {
            RtcFastClock::RtcFastClockXtalD2 => HertzU32::Hz(16_000_000),
            RtcFastClock::RtcFastClockRcFast => HertzU32::Hz(8_000_000),
        }
    }
}

extern "C" {
    fn ets_delay_us(us: u32);
}

/// RTC SLOW_CLK frequency values
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) enum RtcSlowClock {
        /// Select RC_SLOW_CLK as RTC_SLOW_CLK source
        RtcSlowClockRcSlow  = 0,
        /// Select XTAL32K_CLK as RTC_SLOW_CLK source
        RtcSlowClock32kXtal = 1,
        /// Select RC32K_CLK as RTC_SLOW_CLK source
        RtcSlowClock32kRc   = 2,
        /// Select OSC_SLOW_CLK (external slow clock) as RTC_SLOW_CLK source
        RtcSlowOscSlow      = 3,
}

impl Clock for RtcSlowClock {
    fn frequency(&self) -> HertzU32 {
        match self {
            RtcSlowClock::RtcSlowClockRcSlow => HertzU32::Hz(150_000),
            RtcSlowClock::RtcSlowClock32kXtal => HertzU32::Hz(32_768),
            RtcSlowClock::RtcSlowClock32kRc => HertzU32::Hz(32_768),
            RtcSlowClock::RtcSlowOscSlow => HertzU32::Hz(32_768),
        }
    }
}

/// RTC Watchdog Timer
pub struct RtcClock;

/// RTC Watchdog Timer driver
impl RtcClock {
    /// Calculate the necessary RTC_SLOW_CLK cycles to complete 1 millisecond.
    pub(crate) fn cycles_to_1ms() -> u16 {
        todo!()
    }
}


fn set_fast_freq(fast_freq: RtcFastClock) {
    // components/hal/esp32s2/include/hal/clk_tree_ll.h
    unsafe {
        let lp_clkrst = &*LP_CLKRST::PTR;
        lp_clkrst.lp_clk_conf.modify(|_, w| {
            w.fast_clk_sel().bit(match fast_freq {
                RtcFastClock::RtcFastClockRcFast => false,
                RtcFastClock::RtcFastClockXtalD2 => true,
            })
        });
        ets_delay_us(3);
    }
}
