use strum::FromRepr;

use crate::{
    clock::{
        Clock,
        RtcClock,
        RtcFastClock,
        RtcSlowClock,
        clocks_ll::{
            clk_ll_ahb_set_divider,
            clk_ll_bus_update,
            clk_ll_cpu_set_divider,
            regi2c_write_mask,
        },
    },
    peripherals::{LP_AON, PCR, PMU},
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

const I2C_BIAS: u8 = 0x6a;
const I2C_BIAS_HOSTID: u8 = 0;

const I2C_BIAS_DREG_0P8: u8 = 0;
const I2C_BIAS_DREG_0P8_MSB: u8 = 7;
const I2C_BIAS_DREG_0P8_LSB: u8 = 4;

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
    regi2c_write_mask(
        I2C_BIAS,
        I2C_BIAS_HOSTID,
        I2C_BIAS_DREG_0P8,
        I2C_BIAS_DREG_0P8_MSB,
        I2C_BIAS_DREG_0P8_LSB,
        8,
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

        pmu.hp_sleep_dig_power().modify(|_, w| {
            w.hp_sleep_vdd_spi_pd_en()
                .set_bit()
                .hp_sleep_pd_hp_wifi_pd_en()
                .set_bit()
                .hp_sleep_pd_hp_cpu_pd_en()
                .set_bit()
                .hp_sleep_pd_top_pd_en()
                .set_bit()
        });

        pmu.hp_active_hp_ck_power().modify(|_, w| {
            w.hp_active_xpd_bbpll()
                .set_bit()
                .hp_active_xpd_bb_i2c()
                .set_bit()
                .hp_active_xpd_bbpll_i2c()
                .set_bit()
        });

        pmu.hp_active_sysclk().modify(|_, w| {
            w.hp_active_icg_sys_clock_en()
                .set_bit()
                .hp_active_sys_clk_slp_sel()
                .clear_bit()
                .hp_active_icg_slp_sel()
                .clear_bit()
        });
        pmu.hp_sleep_sysclk().modify(|_, w| {
            w.hp_sleep_icg_sys_clock_en()
                .clear_bit()
                .hp_sleep_sys_clk_slp_sel()
                .set_bit()
                .hp_sleep_icg_slp_sel()
                .set_bit()
        });

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

bitfield::bitfield! {
    /// Representation of `PMU_HP_{ACTIVE,SLEEP}_DIG_POWER_REG` registers.
    #[derive(Clone, Copy, Default)]
    pub struct HpDigPower(u32);

    pub bool, vdd_spi_pd_en, set_vdd_spi_pd_en: 21;
    pub bool, mem_dslp     , set_mem_dslp     : 22;
    pub bool, modem_pd_en  , set_modem_pd_en  : 27;
    pub bool, cpu_pd_en    , set_cpu_pd_en    : 29;
    pub bool, top_pd_en    , set_top_pd_en    : 31;
}

bitfield::bitfield! {
    /// Representation of `PMU_HP_{ACTIVE,SLEEP}_HP_CK_POWER_REG` registers.
    #[derive(Clone, Copy, Default)]
    pub struct HpClkPower(u32);

    pub bool, xpd_bbpll    , set_xpd_bbpll    : 30;
}

bitfield::bitfield! {
    /// Representation of `PMU_{HP_ACTIVE,HP_SLEEP,LP_SLEEP}_XTAL_REG` register.
    #[derive(Clone, Copy, Default)]
    pub struct XtalPower(u32);

    pub bool, xpd_xtal     , set_xpd_xtal     : 31;
}

/// Combined HP system power settings.
#[derive(Clone, Copy, Default)]
pub struct HpSysPower {
    pub dig_power: HpDigPower,
    pub clk: HpClkPower,
    pub xtal: XtalPower,
}

bitfield::bitfield! {
    /// Representation of `PMU_{HP,LP}_SLEEP_LP_DIG_POWER_REG`.
    #[derive(Clone, Copy, Default)]
    pub struct LpDigPower(u32);

    pub bool, bod_source_sel, set_bod_source_sel : 27;
    pub u32, vddbat_mode, set_vddbat_mode : 29, 28;
    pub u32, mem_dslp  , set_mem_dslp  : 30;

}

bitfield::bitfield! {
    /// Representation of `PMU_{HP,LP}_SLEEP_LP_CK_POWER_REG`.
    #[derive(Clone, Copy, Default)]
    pub struct LpClkPower(u32);

    pub u32, xpd_xtal32k, set_xpd_xtal32k: 28;
    pub u32, xpd_fosc   , set_xpd_fosc   : 30;
}

/// Combined LP system power settings.
#[derive(Clone, Copy, Default)]
pub struct LpSysPower {
    pub dig_power: LpDigPower,
    pub clk_power: LpClkPower,
    pub xtal: XtalPower,
}

bitfield::bitfield! {
    /// Representation of `PMU_HP_{ACTIVE,SLEEP}_HP_SYS_CNTL_REG` register.
    #[derive(Clone, Copy, Default)]
    pub struct HpSysCntlReg(u32);

    pub bool, uart_wakeup_en , set_uart_wakeup_en : 24;
    pub bool, lp_pad_hold_all, set_lp_pad_hold_all: 25;
    pub bool, hp_pad_hold_all, set_hp_pad_hold_all: 26;
    pub bool, dig_pad_slp_sel, set_dig_pad_slp_sel: 27;
    pub bool, dig_pause_wdt  , set_dig_pause_wdt  : 28;
    pub bool, dig_cpu_stall  , set_dig_cpu_stall  : 29;
}

/// Representation of `PCR_SOC_CLK_SEL` values in `PCR_SYSCLK_CONF_REG` register.
#[derive(Clone, Copy, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(crate) enum CpuClockSource {
    Xtal,
    PllF96M,
    RcFast,
    PllF64M,
}

impl CpuClockSource {
    fn current() -> Self {
        match PCR::regs().sysclk_conf().read().soc_clk_sel().bits() {
            0 => CpuClockSource::Xtal,
            1 => CpuClockSource::PllF96M,
            3 => CpuClockSource::PllF64M,
            2 => CpuClockSource::RcFast,
            _ => unreachable!("PCR_SOC_CLK_SEL is only 2 bits long"),
        }
    }

    fn select(self) {
        PCR::regs().sysclk_conf().modify(|_, w| unsafe {
            w.soc_clk_sel().bits(match self {
                CpuClockSource::Xtal => 0,
                CpuClockSource::PllF96M => 1,
                CpuClockSource::RcFast => 2,
                CpuClockSource::PllF64M => 3,
            })
        });
    }
}

fn rtc_clk_cpu_freq_to_xtal(freq_mhz: u32, div: u8) {
    clk_ll_cpu_set_divider(div as u32);
    clk_ll_ahb_set_divider(div as u32);

    CpuClockSource::Xtal.select();
    clk_ll_bus_update();

    crate::rom::ets_update_cpu_frequency_rom(freq_mhz);
}

pub(crate) fn rtc_clk_cpu_freq_set_xtal() {
    // rtc_clk_cpu_set_to_default_config
    let freq = RtcClock::xtal_freq().mhz();

    rtc_clk_cpu_freq_to_xtal(freq, 1);
}
