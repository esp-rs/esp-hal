use fugit::HertzU32;
use strum::FromRepr;

use crate::{
    clock::{clocks_ll::regi2c_write_mask, Clock, XtalClock},
    peripherals::{LP_AON, LP_CLKRST, PCR, PMU, TIMG0},
    soc::efuse::{Efuse, WAFER_VERSION_MAJOR, WAFER_VERSION_MINOR},
};

const I2C_DIG_REG: u8 = 0x6d;
const I2C_DIG_REG_HOSTID: u8 = 0;

const I2C_DIG_REG_XPD_RTC_REG: u8 = 13;
const I2C_DIG_REG_XPD_RTC_REG_MSB: u8 = 2;
const I2C_DIG_REG_XPD_RTC_REG_LSB: u8 = 2;

const I2C_DIG_REG_XPD_DIG_REG: u8 = 13;
const I2C_DIG_REG_XPD_DIG_REG_MSB: u8 = 3;
const I2C_DIG_REG_XPD_DIG_REG_LSB: u8 = 3;

const I2C_DIG_REG_ENIF_RTC_DREG: u8 = 5;
const I2C_DIG_REG_ENIF_RTC_DREG_MSB: u8 = 7;
const I2C_DIG_REG_ENIF_RTC_DREG_LSB: u8 = 7;

const I2C_DIG_REG_ENIF_DIG_DREG: u8 = 7;
const I2C_DIG_REG_ENIF_DIG_DREG_MSB: u8 = 7;
const I2C_DIG_REG_ENIF_DIG_DREG_LSB: u8 = 7;

const I2C_DIG_REG_SCK_DCAP: u8 = 14;
const I2C_DIG_REG_SCK_DCAP_MSB: u8 = 7;
const I2C_DIG_REG_SCK_DCAP_LSB: u8 = 0;

pub(crate) fn init() {
    let pmu = unsafe { &*PMU::ptr() };

    pmu.rf_pwc
        .modify(|_, w| w.perif_i2c_rstb().set_bit().xpd_perif_i2c().set_bit());

    unsafe {
        regi2c_write_mask(
            I2C_DIG_REG,
            I2C_DIG_REG_HOSTID,
            I2C_DIG_REG_ENIF_RTC_DREG,
            I2C_DIG_REG_ENIF_RTC_DREG_MSB,
            I2C_DIG_REG_ENIF_RTC_DREG_LSB,
            1,
        );
        regi2c_write_mask(
            I2C_DIG_REG,
            I2C_DIG_REG_HOSTID,
            I2C_DIG_REG_ENIF_DIG_DREG,
            I2C_DIG_REG_ENIF_DIG_DREG_MSB,
            I2C_DIG_REG_ENIF_DIG_DREG_LSB,
            1,
        );

        regi2c_write_mask(
            I2C_DIG_REG,
            I2C_DIG_REG_HOSTID,
            I2C_DIG_REG_XPD_RTC_REG,
            I2C_DIG_REG_XPD_RTC_REG_MSB,
            I2C_DIG_REG_XPD_RTC_REG_LSB,
            0,
        );
        regi2c_write_mask(
            I2C_DIG_REG,
            I2C_DIG_REG_HOSTID,
            I2C_DIG_REG_XPD_DIG_REG,
            I2C_DIG_REG_XPD_DIG_REG_MSB,
            I2C_DIG_REG_XPD_DIG_REG_LSB,
            0,
        );

        pmu.hp_active_hp_regulator0
            .modify(|_, w| w.hp_active_hp_regulator_dbias().bits(25));
        pmu.hp_sleep_lp_regulator0
            .modify(|_, w| w.hp_sleep_lp_regulator_dbias().bits(26));
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

fn modem_clk_domain_active_state_icg_map_preinit() {
    unsafe {
        let pmu = &*PMU::PTR;
        let lp_clkrst = &*LP_CLKRST::PTR;
        let pcr = &*PCR::PTR;

        pmu.hp_active_icg_modem
            .modify(|_, w| w.hp_active_dig_icg_modem_code().bits(2));

        const MODEM_SYSCON_CLK_CONF_POWER_ST: u32 = 0x600A9800 + 0xc;
        const MODEM_LPCON_CLK_CONF_POWER_ST: u32 = 0x600A9800 + 0x20;

        (MODEM_SYSCON_CLK_CONF_POWER_ST as *mut u32).write_volatile(
            (MODEM_SYSCON_CLK_CONF_POWER_ST as *mut u32).read_volatile() & !(3 << 28) | 2 << 28,
        );

        (MODEM_LPCON_CLK_CONF_POWER_ST as *mut u32).write_volatile(
            (MODEM_LPCON_CLK_CONF_POWER_ST as *mut u32).read_volatile() & !(3 << 28) | 2 << 28,
        );

        (MODEM_LPCON_CLK_CONF_POWER_ST as *mut u32).write_volatile(
            (MODEM_LPCON_CLK_CONF_POWER_ST as *mut u32).read_volatile() & !(3 << 28) | 2 << 28,
        );

        pmu.imm_modem_icg
            .write(|w| w.update_dig_icg_modem_en().set_bit());
        pmu.imm_sleep_sysclk
            .write(|w| w.update_dig_icg_switch().set_bit());

        lp_clkrst.fosc_cntl.modify(|_, w| w.fosc_dfreq().bits(100));
        regi2c_write_mask(
            I2C_DIG_REG,
            I2C_DIG_REG_HOSTID,
            I2C_DIG_REG_SCK_DCAP,
            I2C_DIG_REG_SCK_DCAP_MSB,
            I2C_DIG_REG_SCK_DCAP_LSB,
            128,
        );
        lp_clkrst
            .rc32k_cntl
            .modify(|_, w| w.rc32k_dfreq().bits(100));

        // https://github.com/espressif/esp-idf/commit/e3148369f32fdc6de62c35a67f7adb6f4faef4e3#diff-cc84d279f2f3d77fe252aa40a64d4813f271a52b5a4055e876efd012d888e135R810-R815
        pcr.ctrl_tick_conf
            .modify(|_, w| w.fosc_tick_num().bits(255 as u8));
    }
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
    /// SDIO Core reset
    CoreSDIO      = 0x06,
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
    /// eFuse CRC error resets the digital core
    CoreEfuseCrc  = 0x14,
    /// USB UART resets the digital core
    CoreUsbUart   = 0x15,
    /// USB JTAG resets the digital core
    CoreUsbJtag   = 0x16,
    /// JTAG resets CPU
    Cpu0JtagCpu   = 0x18,
}

extern "C" {
    fn ets_delay_us(us: u32);
}

#[allow(unused)]
#[derive(Debug, Clone, Copy)]
/// RTC SLOW_CLK frequency values
pub(crate) enum RtcFastClock {
    /// Select RC_FAST_CLK as RTC_FAST_CLK source
    RtcFastClockRcFast = 0,
    /// Select XTAL_D2_CLK as RTC_FAST_CLK source
    RtcFastClockXtalD2 = 1,
}

impl Clock for RtcFastClock {
    fn frequency(&self) -> HertzU32 {
        match self {
            RtcFastClock::RtcFastClockXtalD2 => HertzU32::Hz(40_000_000 / 2), /* TODO: Is the value correct? */
            RtcFastClock::RtcFastClockRcFast => HertzU32::Hz(17_500_000),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
/// RTC SLOW_CLK frequency values
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
            RtcSlowClock::RtcSlowClockRcSlow => HertzU32::Hz(136_000),
            RtcSlowClock::RtcSlowClock32kXtal => HertzU32::Hz(32_768),
            RtcSlowClock::RtcSlowClock32kRc => HertzU32::Hz(32_768),
            RtcSlowClock::RtcSlowOscSlow => HertzU32::Hz(32_768),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
/// Clock source to be calibrated using rtc_clk_cal function
pub(crate) enum RtcCalSel {
    /// Currently selected RTC SLOW_CLK
    RtcCalRtcMux     = -1,
    /// Internal 150kHz RC oscillator
    RtcCalRcSlow     = 0,
    /// External 32kHz XTAL, as one type of 32k clock
    RtcCal32kXtal    = 1,
    /// Internal 32kHz RC oscillator, as one type of 32k clock
    RtcCal32kRc      = 2,
    /// External slow clock signal input by lp_pad_gpio0, as one type of 32k
    /// clock
    RtcCal32kOscSlow = 3,
    /// Internal 20MHz RC oscillator
    RtcCalRcFast,
}

#[derive(Clone)]
pub(crate) enum RtcCaliClkSel {
    CaliClkRcSlow = 0,
    CaliClkRcFast = 1,
    CaliClk32k    = 2,
}
/// RTC Watchdog Timer
pub struct RtcClock;

/// RTC Watchdog Timer driver
impl RtcClock {
    const CAL_FRACT: u32 = 19;

    /// Get main XTAL frequency
    /// This is the value stored in RTC register RTC_XTAL_FREQ_REG by the
    /// bootloader, as passed to rtc_clk_init function.
    fn get_xtal_freq() -> XtalClock {
        let xtal_freq_reg = unsafe { &*LP_AON::PTR }.store4.read().bits();

        // Values of RTC_XTAL_FREQ_REG and RTC_APB_FREQ_REG are stored as two copies in
        // lower and upper 16-bit halves. These are the routines to work with such a
        // representation.
        let clk_val_is_valid = |val| {
            (val & 0xffffu32) == ((val >> 16u32) & 0xffffu32) && val != 0u32 && val != u32::MAX
        };
        let reg_val_to_clk_val = |val| val & u16::MAX as u32;

        if !clk_val_is_valid(xtal_freq_reg) {
            return XtalClock::RtcXtalFreq40M;
        }

        match reg_val_to_clk_val(xtal_freq_reg) {
            40 => XtalClock::RtcXtalFreq40M,
            other => XtalClock::RtcXtalFreqOther(other),
        }
    }

    /// Get the RTC_SLOW_CLK source
    pub(crate) fn get_slow_freq() -> RtcSlowClock {
        let lp_clrst = unsafe { &*LP_CLKRST::ptr() };

        let slow_freq = lp_clrst.lp_clk_conf.read().slow_clk_sel().bits();
        match slow_freq {
            0 => RtcSlowClock::RtcSlowClockRcSlow,
            1 => RtcSlowClock::RtcSlowClock32kXtal,
            2 => RtcSlowClock::RtcSlowClock32kRc,
            3 => RtcSlowClock::RtcSlowOscSlow,
            _ => unreachable!(),
        }
    }

    fn set_slow_freq(slow_freq: RtcSlowClock) {
        unsafe {
            let lp_clkrst = &*LP_CLKRST::PTR;

            lp_clkrst
                .lp_clk_conf
                .modify(|_, w| w.slow_clk_sel().bits(slow_freq as u8));
            lp_clkrst.clk_to_hp.modify(|_, w| {
                w.icg_hp_xtal32k()
                    .bit(match slow_freq {
                        RtcSlowClock::RtcSlowClock32kXtal => true,
                        _ => false,
                    })
                    .icg_hp_xtal32k()
                    .bit(match slow_freq {
                        RtcSlowClock::RtcSlowClock32kXtal => true,
                        _ => false,
                    })
            });
        }
    }

    // TODO: IDF-5781 Some of esp32c6 SOC_RTC_FAST_CLK_SRC_XTAL_D2 rtc_fast clock
    // has timing issue Force to use SOC_RTC_FAST_CLK_SRC_RC_FAST since 2nd
    // stage bootloader https://github.com/espressif/esp-idf/blob/master/components/bootloader_support/src/bootloader_clock_init.c#L65-L67
    fn set_fast_freq(fast_freq: RtcFastClock) {
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

    /// Calibration of RTC_SLOW_CLK is performed using a special feature of
    /// TIMG0. This feature counts the number of XTAL clock cycles within a
    /// given number of RTC_SLOW_CLK cycles.
    fn calibrate_internal(mut cal_clk: RtcCalSel, slowclk_cycles: u32) -> u32 {
        const SOC_CLK_RC_FAST_FREQ_APPROX: u32 = 17_500_000;
        const SOC_CLK_RC_SLOW_FREQ_APPROX: u32 = 136_000;
        const SOC_CLK_XTAL32K_FREQ_APPROX: u32 = 32768;

        if cal_clk == RtcCalSel::RtcCalRtcMux {
            cal_clk = match cal_clk {
                RtcCalSel::RtcCalRtcMux => match RtcClock::get_slow_freq() {
                    RtcSlowClock::RtcSlowClock32kXtal => RtcCalSel::RtcCal32kXtal,
                    RtcSlowClock::RtcSlowClock32kRc => RtcCalSel::RtcCal32kRc,
                    _ => cal_clk,
                },
                RtcCalSel::RtcCal32kOscSlow => RtcCalSel::RtcCalRtcMux,
                _ => cal_clk,
            };
        }

        let lp_clkrst = unsafe { &*LP_CLKRST::ptr() };
        let pcr = unsafe { &*PCR::ptr() };
        let pmu = unsafe { &*PMU::ptr() };

        let clk_src = RtcClock::get_slow_freq();

        if cal_clk == RtcCalSel::RtcCalRtcMux {
            cal_clk = match clk_src {
                RtcSlowClock::RtcSlowClockRcSlow => RtcCalSel::RtcCalRcSlow,
                RtcSlowClock::RtcSlowClock32kXtal => RtcCalSel::RtcCal32kXtal,
                RtcSlowClock::RtcSlowClock32kRc => RtcCalSel::RtcCal32kRc,
                RtcSlowClock::RtcSlowOscSlow => RtcCalSel::RtcCal32kOscSlow,
            };
        }

        let cali_clk_sel;
        if cal_clk == RtcCalSel::RtcCalRtcMux {
            cal_clk = match clk_src {
                RtcSlowClock::RtcSlowClockRcSlow => RtcCalSel::RtcCalRcSlow,
                RtcSlowClock::RtcSlowClock32kXtal => RtcCalSel::RtcCal32kXtal,
                RtcSlowClock::RtcSlowClock32kRc => RtcCalSel::RtcCal32kRc,
                RtcSlowClock::RtcSlowOscSlow => RtcCalSel::RtcCalRcSlow,
            }
        }

        if cal_clk == RtcCalSel::RtcCalRcFast {
            cali_clk_sel = RtcCaliClkSel::CaliClkRcFast;
        } else if cal_clk == RtcCalSel::RtcCalRcSlow {
            cali_clk_sel = RtcCaliClkSel::CaliClkRcSlow;
        } else {
            cali_clk_sel = RtcCaliClkSel::CaliClk32k;
            match cal_clk {
                RtcCalSel::RtcCalRtcMux | RtcCalSel::RtcCalRcSlow | RtcCalSel::RtcCalRcFast => (),
                RtcCalSel::RtcCal32kRc => pcr
                    .ctrl_32k_conf
                    .modify(|_, w| unsafe { w.clk_32k_sel().bits(0) }),
                RtcCalSel::RtcCal32kXtal => pcr
                    .ctrl_32k_conf
                    .modify(|_, w| unsafe { w.clk_32k_sel().bits(1) }),
                RtcCalSel::RtcCal32kOscSlow => pcr
                    .ctrl_32k_conf
                    .modify(|_, w| unsafe { w.clk_32k_sel().bits(2) }),
            }
        }

        // Enable requested clock (150k is always on)
        // Some delay is required before the time is stable
        // Only enable if originaly was disabled
        // If clock is already on, do nothing

        let dig_32k_xtal_enabled = lp_clkrst.clk_to_hp.read().icg_hp_xtal32k().bit_is_set();

        if cal_clk == RtcCalSel::RtcCal32kXtal && !dig_32k_xtal_enabled {
            lp_clkrst
                .clk_to_hp
                .modify(|_, w| w.icg_hp_xtal32k().set_bit());
        }

        // TODO: very hacky
        // in ESP-IDF these are not called in this function but the fields are set
        lp_clkrst
            .clk_to_hp
            .modify(|_, w| w.icg_hp_xtal32k().set_bit());
        pmu.hp_sleep_lp_ck_power
            .modify(|_, w| w.hp_sleep_xpd_xtal32k().set_bit());

        pmu.hp_sleep_lp_ck_power
            .modify(|_, w| w.hp_sleep_xpd_rc32k().set_bit());

        let rc_fast_enabled = pmu
            .hp_sleep_lp_ck_power
            .read()
            .hp_sleep_xpd_fosc_clk()
            .bit_is_set();
        let dig_rc_fast_enabled = lp_clkrst.clk_to_hp.read().icg_hp_fosc().bit_is_set();

        if cal_clk == RtcCalSel::RtcCalRcFast {
            if !rc_fast_enabled {
                pmu.hp_sleep_lp_ck_power
                    .modify(|_, w| w.hp_sleep_xpd_fosc_clk().set_bit());
                unsafe {
                    ets_delay_us(50);
                }
            }

            if !dig_rc_fast_enabled {
                lp_clkrst.clk_to_hp.modify(|_, w| w.icg_hp_fosc().set_bit());
                unsafe {
                    ets_delay_us(5);
                }
            }
        }

        let rc32k_enabled = pmu
            .hp_sleep_lp_ck_power
            .read()
            .hp_sleep_xpd_rc32k()
            .bit_is_set();
        let dig_rc32k_enabled = lp_clkrst.clk_to_hp.read().icg_hp_osc32k().bit_is_set();

        if cal_clk == RtcCalSel::RtcCal32kRc {
            if !rc32k_enabled {
                pmu.hp_sleep_lp_ck_power
                    .modify(|_, w| w.hp_sleep_xpd_rc32k().set_bit());
                unsafe {
                    ets_delay_us(300);
                }
            }

            if !dig_rc32k_enabled {
                lp_clkrst
                    .clk_to_hp
                    .modify(|_, w| w.icg_hp_osc32k().set_bit());
            }
        }

        // Check if there is already running calibration process
        // TODO: &mut TIMG0 for calibration
        let timg0 = unsafe { &*TIMG0::ptr() };

        if timg0
            .rtccalicfg
            .read()
            .rtc_cali_start_cycling()
            .bit_is_set()
        {
            timg0
                .rtccalicfg2
                .modify(|_, w| unsafe { w.rtc_cali_timeout_thres().bits(1) });

            // Set small timeout threshold to accelerate the generation of timeot
            // Internal circuit will be reset when timeout occurs and will not affect the
            // next calibration
            while !timg0.rtccalicfg.read().rtc_cali_rdy().bit_is_set()
                && !timg0.rtccalicfg2.read().rtc_cali_timeout().bit_is_set()
            {}
        }

        // Prepare calibration
        timg0
            .rtccalicfg
            .modify(|_, w| unsafe { w.rtc_cali_clk_sel().bits(cali_clk_sel.clone() as u8) });
        timg0
            .rtccalicfg
            .modify(|_, w| w.rtc_cali_start_cycling().clear_bit());
        timg0
            .rtccalicfg
            .modify(|_, w| unsafe { w.rtc_cali_max().bits(slowclk_cycles as u16) });

        let expected_freq = match cali_clk_sel {
            RtcCaliClkSel::CaliClk32k => {
                timg0.rtccalicfg2.modify(|_, w| unsafe {
                    w.rtc_cali_timeout_thres().bits(slowclk_cycles << 12)
                });
                SOC_CLK_XTAL32K_FREQ_APPROX
            }
            RtcCaliClkSel::CaliClkRcFast => {
                timg0
                    .rtccalicfg2
                    .modify(|_, w| unsafe { w.rtc_cali_timeout_thres().bits(0x01FFFFFF) });
                SOC_CLK_RC_FAST_FREQ_APPROX
            }
            _ => {
                timg0.rtccalicfg2.modify(|_, w| unsafe {
                    w.rtc_cali_timeout_thres().bits(slowclk_cycles << 10)
                });
                SOC_CLK_RC_SLOW_FREQ_APPROX
            }
        };

        let us_time_estimate = (HertzU32::MHz(slowclk_cycles) / expected_freq).to_Hz();

        // Start calibration
        timg0
            .rtccalicfg
            .modify(|_, w| w.rtc_cali_start().clear_bit());
        timg0.rtccalicfg.modify(|_, w| w.rtc_cali_start().set_bit());

        // Wait for calibration to finish up to another us_time_estimate
        unsafe {
            ets_delay_us(us_time_estimate);
        }

        let cal_val = loop {
            if timg0.rtccalicfg.read().rtc_cali_rdy().bit_is_set() {
                let minor: u8 = Efuse::read_field_le(WAFER_VERSION_MINOR);
                let major: u8 = Efuse::read_field_le(WAFER_VERSION_MAJOR);

                // The Fosc CLK of calibration circuit is divided by 32 for ECO1.
                // So we need to multiply the frequency of the Fosc for ECO1 and above chips by
                // 32 times. And ensure that this modification will not affect
                // ECO0. PS: For ESP32C6 ECO0 chip version is v0.0 only, which
                // means that both MAJOR and MINOR are 0. The chip version is
                // calculated using the following formula: MAJOR * 100 + MINOR. (if the result
                // is 1, then version is v0.1) https://github.com/espressif/esp-idf/commit/e3148369f32fdc6de62c35a67f7adb6f4faef4e3
                if (major * 100 + minor) > 0 {
                    if cal_clk == RtcCalSel::RtcCalRcFast {
                        break timg0.rtccalicfg1.read().rtc_cali_value().bits() >> 5;
                    }
                    break timg0.rtccalicfg1.read().rtc_cali_value().bits();
                } else {
                    break timg0.rtccalicfg1.read().rtc_cali_value().bits();
                }
            }

            if timg0.rtccalicfg2.read().rtc_cali_timeout().bit_is_set() {
                // Timed out waiting for calibration
                break 0;
            }
        };

        timg0
            .rtccalicfg
            .modify(|_, w| w.rtc_cali_start().clear_bit());

        if cal_clk == RtcCalSel::RtcCal32kXtal && !dig_32k_xtal_enabled {
            lp_clkrst
                .clk_to_hp
                .modify(|_, w| w.icg_hp_xtal32k().clear_bit());
        }

        if cal_clk == RtcCalSel::RtcCalRcFast {
            if rc_fast_enabled {
                pmu.hp_sleep_lp_ck_power
                    .modify(|_, w| w.hp_sleep_xpd_fosc_clk().set_bit());
                unsafe {
                    ets_delay_us(50);
                }
            }

            if dig_rc_fast_enabled {
                lp_clkrst.clk_to_hp.modify(|_, w| w.icg_hp_fosc().set_bit());
                unsafe {
                    ets_delay_us(5);
                }
            }
        }

        if cal_clk == RtcCalSel::RtcCal32kRc {
            if rc32k_enabled {
                pmu.hp_sleep_lp_ck_power
                    .modify(|_, w| w.hp_sleep_xpd_rc32k().set_bit());
                unsafe {
                    ets_delay_us(300);
                }
            }
            if dig_rc32k_enabled {
                lp_clkrst
                    .clk_to_hp
                    .modify(|_, w| w.icg_hp_osc32k().set_bit());
            }
        }

        cal_val
    }

    /// Measure RTC slow clock's period, based on main XTAL frequency
    ///
    /// This function will time out and return 0 if the time for the given
    /// number of cycles to be counted exceeds the expected time twice. This
    /// may happen if 32k XTAL is being calibrated, but the oscillator has
    /// not started up (due to incorrect loading capacitance, board design
    /// issue, or lack of 32 XTAL on board).
    fn calibrate(cal_clk: RtcCalSel, slowclk_cycles: u32) -> u32 {
        let xtal_freq = RtcClock::get_xtal_freq();

        let minor: u8 = Efuse::read_field_le(WAFER_VERSION_MINOR);
        let major: u8 = Efuse::read_field_le(WAFER_VERSION_MAJOR);

        let mut slowclk_cycles = slowclk_cycles;

        // The Fosc CLK of calibration circuit is divided by 32 for ECO1.
        // So we need to divide the calibrate cycles of the FOSC for ECO1 and above
        // chips by 32 to avoid excessive calibration time.*/
        // PS: For ESP32C6 ECO0 chip version is v0.0 only, which means that both MAJOR
        // and MINOR are 0. The chip version is calculated using the following
        // formula: MAJOR * 100 + MINOR. (if the result is 1, then version is v0.1)
        if (major * 100 + minor) > 0 {
            if cal_clk == RtcCalSel::RtcCalRcFast {
                slowclk_cycles >>= 5;
            }
        }

        let xtal_cycles = RtcClock::calibrate_internal(cal_clk, slowclk_cycles) as u64;
        let divider = xtal_freq.mhz() as u64 * slowclk_cycles as u64;
        let period_64 = ((xtal_cycles << RtcClock::CAL_FRACT) + divider / 2u64 - 1u64) / divider;

        (period_64 & u32::MAX as u64) as u32
    }

    /// Calculate the necessary RTC_SLOW_CLK cycles to complete 1 millisecond.
    pub(crate) fn cycles_to_1ms() -> u16 {
        let period_13q19 = RtcClock::calibrate(
            match RtcClock::get_slow_freq() {
                RtcSlowClock::RtcSlowClockRcSlow => RtcCalSel::RtcCalRtcMux,
                RtcSlowClock::RtcSlowClock32kXtal => RtcCalSel::RtcCal32kXtal,
                RtcSlowClock::RtcSlowClock32kRc => RtcCalSel::RtcCal32kRc,
                RtcSlowClock::RtcSlowOscSlow => RtcCalSel::RtcCal32kOscSlow,
                // RtcSlowClock::RtcCalRcFast => RtcCalSel::RtcCalRcFast,
            },
            1024,
        );

        // 100_000_000 is used to get rid of `float` calculations
        let period = (100_000_000 * period_13q19 as u64) / (1 << RtcClock::CAL_FRACT);

        (100_000_000 * 1000 / period) as u16
    }
}
