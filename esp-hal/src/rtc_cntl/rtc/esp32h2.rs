use strum::FromRepr;

use crate::{
    clock::{RtcClock, RtcFastClock, RtcSlowClock, clocks_ll::regi2c_write_mask},
    peripherals::{LP_AON, PMU},
    rtc_cntl::RtcCalSel,
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

pub(crate) fn init() {
    // * No peripheral reg i2c power up required on the target */
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

    let pmu = PMU::regs();
    unsafe {
        pmu.power_pd_top_cntl().write(|w| w.bits(0));
        pmu.power_pd_hpaon_cntl().write(|w| w.bits(0));
        pmu.power_pd_hpcpu_cntl().write(|w| w.bits(0));
        pmu.power_pd_hpperi_reserve().write(|w| w.bits(0));
        pmu.power_pd_hpwifi_cntl().write(|w| w.bits(0));
        pmu.power_pd_lpperi_cntl().write(|w| w.bits(0));

        pmu.hp_active_hp_regulator0()
            .modify(|_, w| w.hp_active_hp_regulator_dbias().bits(25));
        pmu.hp_sleep_lp_regulator0()
            .modify(|_, w| w.hp_sleep_lp_regulator_dbias().bits(26));

        pmu.slp_wakeup_cntl5()
            .modify(|_, w| w.lp_ana_wait_target().bits(15));
        pmu.slp_wakeup_cntl7()
            .modify(|_, w| w.ana_wait_target().bits(1700));
    }

    RtcClock::set_fast_freq(RtcFastClock::RcFast);
    RtcClock::set_slow_freq(RtcSlowClock::RcSlow);
}

pub(crate) fn configure_clock() {
    let cal_val = loop {
        let res = RtcClock::calibrate(RtcCalSel::RtcMux, 1024);
        if res != 0 {
            break res;
        }
    };

    LP_AON::regs()
        .store1()
        .modify(|_, w| unsafe { w.bits(cal_val) });
}

// Terminology:
//
// CPU Reset:    Reset CPU core only, once reset done, CPU will execute from
//               reset vector
// Core Reset:   Reset the whole digital system except RTC sub-system
// System Reset: Reset the whole digital system, including RTC sub-system
// Chip Reset:   Reset the whole chip, including the analog part

/// SOC Reset Reason.
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
