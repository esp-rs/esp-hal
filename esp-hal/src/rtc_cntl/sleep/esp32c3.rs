use super::{TimerWakeupSource, WakeSource, WakeTriggers, WakeupLevel};
use crate::{
    gpio::{RtcFunction, RtcPinWithResistors},
    regi2c_write_mask,
    rtc_cntl::{sleep::RtcioWakeupSource, Clock, Rtc, RtcClock},
};

const I2C_DIG_REG: u32 = 0x6D;
const I2C_DIG_REG_HOSTID: u32 = 0;

const I2C_DIG_REG_EXT_RTC_DREG: u32 = 4;
const I2C_DIG_REG_EXT_RTC_DREG_MSB: u32 = 4;
const I2C_DIG_REG_EXT_RTC_DREG_LSB: u32 = 0;

const I2C_DIG_REG_EXT_RTC_DREG_SLEEP: u32 = 5;
const I2C_DIG_REG_EXT_RTC_DREG_SLEEP_MSB: u32 = 4;
const I2C_DIG_REG_EXT_RTC_DREG_SLEEP_LSB: u32 = 0;

const I2C_DIG_REG_EXT_DIG_DREG: u32 = 6;
const I2C_DIG_REG_EXT_DIG_DREG_MSB: u32 = 4;
const I2C_DIG_REG_EXT_DIG_DREG_LSB: u32 = 0;

const I2C_DIG_REG_EXT_DIG_DREG_SLEEP: u32 = 7;
const I2C_DIG_REG_EXT_DIG_DREG_SLEEP_MSB: u32 = 4;
const I2C_DIG_REG_EXT_DIG_DREG_SLEEP_LSB: u32 = 0;

const I2C_DIG_REG_XPD_RTC_REG: u32 = 13;
const I2C_DIG_REG_XPD_RTC_REG_MSB: u32 = 2;
const I2C_DIG_REG_XPD_RTC_REG_LSB: u32 = 2;

const I2C_DIG_REG_XPD_DIG_REG: u32 = 13;
const I2C_DIG_REG_XPD_DIG_REG_MSB: u32 = 3;
const I2C_DIG_REG_XPD_DIG_REG_LSB: u32 = 3;

const I2C_ULP_IR_FORCE_XPD_CK: u8 = 0;
const I2C_ULP_IR_FORCE_XPD_CK_MSB: u8 = 2;
const I2C_ULP_IR_FORCE_XPD_CK_LSB: u8 = 2;

const I2C_ULP: u8 = 0x61;
const I2C_ULP_HOSTID: u8 = 0;

// Approximate mapping of voltages to RTC_CNTL_DBIAS_WAK, RTC_CNTL_DBIAS_SLP,
// RTC_CNTL_DIG_DBIAS_WAK, RTC_CNTL_DIG_DBIAS_SLP values.
// Valid if RTC_CNTL_DBG_ATTEN is 0.
pub const RTC_CNTL_DBIAS_0V90: u32 = 13;
pub const RTC_CNTL_DBIAS_0V95: u32 = 16;
pub const RTC_CNTL_DBIAS_1V00: u32 = 18;
pub const RTC_CNTL_DBIAS_1V05: u32 = 20;
pub const RTC_CNTL_DBIAS_1V10: u32 = 23;
pub const RTC_CNTL_DBIAS_1V15: u32 = 25;
pub const RTC_CNTL_DBIAS_1V20: u32 = 28;
pub const RTC_CNTL_DBIAS_1V25: u32 = 30;
pub const RTC_CNTL_DBIAS_1V30: u32 = 31; //< voltage is about 1.34v in fact

pub const RTC_CNTL_DBG_ATTEN_LIGHTSLEEP_DEFAULT: u8 = 5;
pub const RTC_CNTL_DBG_ATTEN_LIGHTSLEEP_NODROP: u8 = 0;
pub const RTC_CNTL_DBG_ATTEN_DEEPSLEEP_DEFAULT: u8 = 15;
pub const RTC_CNTL_DBG_ATTEN_DEEPSLEEP_NODROP: u8 = 0;
pub const RTC_CNTL_BIASSLP_SLEEP_DEFAULT: u8 = 1;
pub const RTC_CNTL_BIASSLP_SLEEP_ON: u8 = 0;
pub const RTC_CNTL_PD_CUR_SLEEP_DEFAULT: u8 = 1;
pub const RTC_CNTL_PD_CUR_SLEEP_ON: u8 = 0;
pub const RTC_CNTL_DG_VDD_DRV_B_SLP_DEFAULT: u8 = 254;

pub const RTC_CNTL_DBG_ATTEN_MONITOR_DEFAULT: u8 = 0;
pub const RTC_CNTL_BIASSLP_MONITOR_DEFAULT: bool = false;
pub const RTC_CNTL_PD_CUR_MONITOR_DEFAULT: bool = false;

pub const RTC_CNTL_PLL_BUF_WAIT_DEFAULT: u8 = 20;
pub const RTC_CNTL_XTL_BUF_WAIT_DEFAULT: u8 = 100;
pub const RTC_CNTL_CK8M_WAIT_DEFAULT: u8 = 20;
pub const RTC_CK8M_ENABLE_WAIT_DEFAULT: u8 = 5;

pub const RTC_CNTL_MIN_SLP_VAL_MIN: u8 = 2;

pub const OTHER_BLOCKS_POWERUP: u8 = 1;
pub const OTHER_BLOCKS_WAIT: u16 = 1;

pub const GPIO_INTR_DISABLE: u8 = 0;
pub const GPIO_INTR_LOW_LEVEL: u8 = 4;
pub const GPIO_INTR_HIGH_LEVEL: u8 = 5;

pub const PIN_FUNC_GPIO: u8 = 1;
pub const SIG_GPIO_OUT_IDX: u32 = 128;
pub const GPIO_NUM_MAX: usize = 22;

impl WakeSource for TimerWakeupSource {
    fn apply(
        &self,
        rtc: &Rtc<'_>,
        triggers: &mut WakeTriggers,
        _sleep_config: &mut RtcSleepConfig,
    ) {
        triggers.set_timer(true);
        let rtc_cntl = unsafe { &*esp32c3::RTC_CNTL::ptr() };
        let clock_freq = RtcClock::get_slow_freq();
        // TODO: maybe add sleep time adjustlemnt like idf
        // TODO: maybe add check to prevent overflow?
        let clock_hz = clock_freq.frequency().to_Hz() as u64;
        let ticks = self.duration.as_micros() as u64 * clock_hz / 1_000_000u64;
        // "alarm" time in slow rtc ticks
        let now = rtc.get_time_raw();
        let time_in_ticks = now + ticks;
        unsafe {
            rtc_cntl
                .slp_timer0()
                .write(|w| w.slp_val_lo().bits((time_in_ticks & 0xffffffff) as u32));

            rtc_cntl
                .int_clr()
                .write(|w| w.main_timer().clear_bit_by_one());

            rtc_cntl.slp_timer1().write(|w| {
                w.slp_val_hi()
                    .bits(((time_in_ticks >> 32) & 0xffff) as u16)
                    .main_timer_alarm_en()
                    .set_bit()
            });
        }
    }
}

impl<'a, 'b> RtcioWakeupSource<'a, 'b> {
    fn apply_pin(&self, pin: &mut dyn RtcPinWithResistors, level: WakeupLevel) {
        // The pullup/pulldown part is like in gpio_deep_sleep_wakeup_prepare
        let level = match level {
            WakeupLevel::High => {
                pin.rtcio_pullup(false);
                pin.rtcio_pulldown(true);
                GPIO_INTR_HIGH_LEVEL
            }
            WakeupLevel::Low => {
                pin.rtcio_pullup(true);
                pin.rtcio_pulldown(false);
                GPIO_INTR_LOW_LEVEL
            }
        };
        pin.rtcio_pad_hold(true);

        // apply_wakeup does the same as idf's esp_deep_sleep_enable_gpio_wakeup
        unsafe {
            pin.apply_wakeup(true, level);
        }
    }
}

fn isolate_digital_gpio() {
    // like esp_sleep_isolate_digital_gpio
    let rtc_cntl = unsafe { &*crate::peripherals::RTC_CNTL::ptr() };
    let io_mux = unsafe { &*crate::peripherals::IO_MUX::ptr() };
    let gpio = unsafe { &*crate::peripherals::GPIO::ptr() };

    let dig_iso = &rtc_cntl.dig_iso().read();
    let deep_sleep_hold_is_en =
        !dig_iso.dg_pad_force_unhold().bit() && dig_iso.dg_pad_autohold_en().bit();
    if !deep_sleep_hold_is_en {
        return;
    }

    // TODO: assert that the task stack is not in external ram

    for pin_num in 0..GPIO_NUM_MAX {
        let pin_hold = rtc_cntl.dig_pad_hold().read().bits() & (1 << pin_num) != 0;
        if !pin_hold {
            // input disable, like gpio_ll_input_disable
            io_mux.gpio(pin_num).modify(|_, w| w.fun_ie().clear_bit());
            // output disable, like gpio_ll_output_disable
            unsafe {
                gpio.func_out_sel_cfg(pin_num)
                    .modify(|_, w| w.bits(SIG_GPIO_OUT_IDX));
            }

            // disable pull-up and pull-down
            io_mux.gpio(pin_num).modify(|_, w| w.fun_wpu().clear_bit());
            io_mux.gpio(pin_num).modify(|_, w| w.fun_wpd().clear_bit());

            // make pad work as gpio (otherwise, deep_sleep bottom current will rise)
            io_mux
                .gpio(pin_num)
                .modify(|_, w| unsafe { w.mcu_sel().bits(RtcFunction::Digital as u8) });
        }
    }
}

impl WakeSource for RtcioWakeupSource<'_, '_> {
    fn apply(
        &self,
        _rtc: &Rtc<'_>,
        triggers: &mut WakeTriggers,
        sleep_config: &mut RtcSleepConfig,
    ) {
        let mut pins = self.pins.borrow_mut();

        if pins.is_empty() {
            return;
        }

        triggers.set_gpio(true);

        // If deep sleep is enabled, esp_start_sleep calls
        // gpio_deep_sleep_wakeup_prepare which sets these pullup and
        // pulldown values. But later in esp_start_sleep it calls
        // esp_sleep_isolate_digital_gpio, which disables the pullup and pulldown (but
        // only if it isn't held).
        // But it looks like gpio_deep_sleep_wakeup_prepare enables hold for all pins
        // in the wakeup mask.
        //
        // So: all pins in the wake mask should get this treatment here, and all pins
        // not in the wake mask should get
        // - pullup and pulldowns disabled
        // - input and output disabled, and
        // - their func should get set to GPIO.
        // But this last block of things gets skipped if hold is disabled globally (see
        // gpio_ll_deep_sleep_hold_is_en)

        let rtc_cntl = unsafe { &*crate::peripherals::RTC_CNTL::PTR };

        rtc_cntl
            .gpio_wakeup()
            .modify(|_, w| w.gpio_pin_clk_gate().set_bit());

        rtc_cntl
            .ext_wakeup_conf()
            .modify(|_, w| w.gpio_wakeup_filter().set_bit());

        if sleep_config.deep_slp() {
            for (pin, level) in pins.iter_mut() {
                self.apply_pin(*pin, *level);
            }

            isolate_digital_gpio();
        }

        // like rtc_cntl_ll_gpio_clear_wakeup_status, as called from
        // gpio_deep_sleep_wakeup_prepare
        rtc_cntl
            .gpio_wakeup()
            .modify(|_, w| w.gpio_wakeup_status_clr().set_bit());
        rtc_cntl
            .gpio_wakeup()
            .modify(|_, w| w.gpio_wakeup_status_clr().clear_bit());
    }
}

// impl Drop for RtcioWakeupSource<'_, '_> {
// fn drop(&mut self) {
// should we have saved the pin configuration first?
// set pin back to IO_MUX (input_enable and func have no effect when pin is sent
// to IO_MUX)
// let mut pins = self.pins.borrow_mut();
// for (pin, _level) in pins.iter_mut() {
// pin.rtc_set_config(true, false, RtcFunction::Rtc);
// }
// }
// }

bitfield::bitfield! {
    #[derive(Clone, Copy)]
    pub struct RtcConfig(u32);
    impl Debug;
    /// Number of rtc_fast_clk cycles to wait for 8M clock to be ready
    pub u8, ck8m_wait, set_ck8m_wait: 7, 0;
    /// Number of rtc_fast_clk cycles to wait for XTAL clock to be ready
    pub u8, xtal_wait, set_xtal_wait: 15, 8;
    /// Number of rtc_fast_clk cycles to wait for PLL clock to be ready
    pub u8, pll_wait, set_pll_wait: 23, 16;
    // Perform clock control related initialization.
    pub clkctl_init, set_clkctl_init: 24;
    // Perform power control related initialization.
    pub pwrctl_init, set_pwrctl_init: 25;
    // Force power down RTC_DBOOST
    pub rtc_dboost_fpd, set_rtc_dboost_fpd: 26;
    pub xtal_fpu, set_xtal_fpu: 27;
    pub bbpll_fpu, set_bbpll_fpu: 28;
    pub cpu_waiti_clk_gate, set_cpu_waiti_clk_gate: 29;
    // Calibrate Ocode to make bandgap voltage more precise.
    pub cali_ocode, set_cali_ocode: 30;
}

impl Default for RtcConfig {
    fn default() -> Self {
        let mut cfg = Self(Default::default());
        cfg.set_ck8m_wait(RTC_CNTL_CK8M_WAIT_DEFAULT);
        cfg.set_xtal_wait(RTC_CNTL_XTL_BUF_WAIT_DEFAULT);
        cfg.set_pll_wait(RTC_CNTL_PLL_BUF_WAIT_DEFAULT);
        cfg.set_clkctl_init(true);
        cfg.set_pwrctl_init(true);
        cfg.set_rtc_dboost_fpd(true);
        cfg.set_cpu_waiti_clk_gate(true);
        cfg
    }
}

bitfield::bitfield! {
    #[derive(Clone, Copy)]
    pub struct RtcInitConfig(u128);
    impl Debug;
    pub u8, wifi_powerup_cycles, set_wifi_powerup_cycles: 6, 0;
    pub u16, wifi_wait_cycles, set_wifi_wait_cycles: 15, 7;
    pub u8, bt_powerup_cycles, set_bt_powerup_cycles: 22, 16;
    pub u16, bt_wait_cycles, set_bt_wait_cycles: 31, 23;
    pub u8, cpu_top_powerup_cycles, set_cpu_top_powerup_cycles: 38, 32;
    pub u16, cpu_top_wait_cycles, set_cpu_top_wait_cycles: 47, 39;
    pub u8, dg_wrap_powerup_cycles, set_dg_wrap_powerup_cycles: 54, 48;
    pub u16, dg_wrap_wait_cycles, set_dg_wrap_wait_cycles: 63, 55;
    pub u8, dg_peri_powerup_cycles, set_dg_peri_powerup_cycles: 70, 64;
    pub u16, dg_peri_wait_cycles, set_dg_peri_wait_cycles: 79, 71;
}

impl Default for RtcInitConfig {
    fn default() -> Self {
        let mut cfg = Self(Default::default());
        cfg.set_wifi_powerup_cycles(OTHER_BLOCKS_POWERUP);
        cfg.set_bt_powerup_cycles(OTHER_BLOCKS_POWERUP);
        cfg.set_cpu_top_powerup_cycles(OTHER_BLOCKS_POWERUP);
        cfg.set_dg_wrap_powerup_cycles(OTHER_BLOCKS_POWERUP);
        cfg.set_dg_peri_powerup_cycles(OTHER_BLOCKS_POWERUP);
        cfg.set_wifi_wait_cycles(OTHER_BLOCKS_WAIT);
        cfg.set_bt_wait_cycles(OTHER_BLOCKS_WAIT);
        cfg.set_cpu_top_wait_cycles(OTHER_BLOCKS_WAIT);
        cfg.set_dg_wrap_wait_cycles(OTHER_BLOCKS_WAIT);
        cfg.set_dg_peri_wait_cycles(OTHER_BLOCKS_WAIT);
        cfg
    }
}

bitfield::bitfield! {
    #[derive(Clone, Copy)]
    pub struct RtcSleepConfig(u64);
    impl Debug;
    /// force normal voltage in sleep mode (digital domain memory)
    pub lslp_mem_inf_fpu, set_lslp_mem_inf_fpu: 0;
    /// keep low voltage in sleep mode (even if ULP/touch is used)
    pub rtc_mem_inf_follow_cpu, set_rtc_mem_inf_follow_cpu: 1;
    /// power down RTC fast memory
    pub rtc_fastmem_pd_en, set_rtc_fastmem_pd_en: 2;
    /// power down RTC slow memory
    pub rtc_slowmem_pd_en, set_rtc_slowmem_pd_en: 3;
    /// power down RTC peripherals
    pub rtc_peri_pd_en, set_rtc_peri_pd_en: 4;
    /// power down WiFi
    pub wifi_pd_en, set_wifi_pd_en: 5;
    /// power down BT
    pub bt_pd_en, set_bt_pd_en: 6;
    /// power down CPU, but not restart when lightsleep.
    pub cpu_pd_en, set_cpu_pd_en: 7;
    /// Power down Internal 8M oscillator
    pub int_8m_pd_en, set_int_8m_pd_en: 8;
    /// power down digital peripherals
    pub dig_peri_pd_en, set_dig_peri_pd_en: 9;
    /// power down digital domain
    pub deep_slp, set_deep_slp: 10;
    /// enable WDT flashboot mode
    pub wdt_flashboot_mod_en, set_wdt_flashboot_mod_en: 11;
    /// set bias for digital domain, in sleep mode
    pub u32, dig_dbias_slp, set_dig_dbias_slp: 16, 12;
    /// set bias for RTC domain, in sleep mode
    pub u32, rtc_dbias_slp, set_rtc_dbias_slp: 21, 17;
    /// voltage parameter, in sleep mode
    pub u8, dbg_atten_slp, set_dbg_atten_slp: 25, 22;
    /// circuit control parameter, in monitor mode
    pub bias_sleep_monitor, set_bias_sleep_monitor: 26;
    /// circuit control parameter, in sleep mode
    pub bias_sleep_slp, set_bias_sleep_slp: 27;
    /// circuit control parameter, in monitor mode
    pub pd_cur_slp, set_pd_cur_slp: 28;
    /// power down VDDSDIO regulator
    pub vddsdio_pd_en, set_vddsdio_pd_en: 29;
    /// keep main XTAL powered up in sleep
    pub xtal_fpu, set_xtal_fpu: 30;
    /// keep rtc regulator powered up in sleep
    pub rtc_regulator_fpu, set_rtc_regulator_fpu: 31;
    /// enable deep sleep reject
    pub deep_slp_reject, set_deep_slp_reject: 32;
    /// enable light sleep reject
    pub light_slp_reject, set_light_slp_reject: 33;
}

impl Default for RtcSleepConfig {
    fn default() -> Self {
        let mut cfg = Self(Default::default());
        cfg.set_deep_slp_reject(true);
        cfg.set_light_slp_reject(true);
        cfg.set_rtc_dbias_slp(RTC_CNTL_DBIAS_1V10);
        cfg.set_dig_dbias_slp(RTC_CNTL_DBIAS_1V10);
        cfg
    }
}

const SYSCON_SRAM_POWER_UP: u8 = 0x0000000F;
const SYSCON_ROM_POWER_UP: u8 = 0x00000003;

fn rtc_sleep_pu(val: bool) {
    let rtc_cntl = unsafe { &*esp32c3::RTC_CNTL::ptr() };
    let syscon = unsafe { &*esp32c3::APB_CTRL::ptr() };
    let bb = unsafe { &*esp32c3::BB::ptr() };
    let nrx = unsafe { &*esp32c3::NRX::ptr() };
    let fe = unsafe { &*esp32c3::FE::ptr() };
    let fe2 = unsafe { &*esp32c3::FE2::ptr() };

    rtc_cntl
        .dig_pwc()
        .modify(|_, w| w.lslp_mem_force_pu().bit(val).fastmem_force_lpu().bit(val));

    syscon.front_end_mem_pd().modify(|_r, w| {
        w.dc_mem_force_pu()
            .bit(val)
            .pbus_mem_force_pu()
            .bit(val)
            .agc_mem_force_pu()
            .bit(val)
    });

    bb.bbpd_ctrl()
        .modify(|_r, w| w.fft_force_pu().bit(val).dc_est_force_pu().bit(val));

    if val {
        nrx.nrxpd_ctrl().modify(|_, w| {
            w.rx_rot_force_pu()
                .set_bit()
                .vit_force_pu()
                .set_bit()
                .demap_force_pu()
                .set_bit()
        });
    } else {
        nrx.nrxpd_ctrl().modify(|_, w| {
            w.rx_rot_force_pu()
                .clear_bit()
                .vit_force_pu()
                .clear_bit()
                .demap_force_pu()
                .clear_bit()
        });
    }

    if val {
        fe.gen_ctrl().modify(|_, w| w.iq_est_force_pu().set_bit());
    } else {
        fe.gen_ctrl().modify(|_, w| w.iq_est_force_pu().clear_bit());
    }

    if val {
        fe2.tx_interp_ctrl()
            .modify(|_, w| w.tx_inf_force_pu().set_bit());
    } else {
        fe2.tx_interp_ctrl()
            .modify(|_, w| w.tx_inf_force_pu().clear_bit());
    }

    syscon.mem_power_up().modify(|_r, w| unsafe {
        w.sram_power_up()
            .bits(if val { SYSCON_SRAM_POWER_UP } else { 0 })
            .rom_power_up()
            .bits(if val { SYSCON_ROM_POWER_UP } else { 0 })
    });
}

impl RtcSleepConfig {
    pub fn deep() -> Self {
        // Set up for ultra-low power sleep. Wakeup sources may modify these settings.
        let mut cfg = Self::default();

        cfg.set_lslp_mem_inf_fpu(false);
        cfg.set_rtc_mem_inf_follow_cpu(true); // ?
        cfg.set_rtc_fastmem_pd_en(true);
        cfg.set_rtc_slowmem_pd_en(true);
        cfg.set_rtc_peri_pd_en(true);
        cfg.set_wifi_pd_en(true);
        cfg.set_bt_pd_en(true);
        cfg.set_cpu_pd_en(true);
        cfg.set_int_8m_pd_en(true);

        cfg.set_dig_peri_pd_en(true);
        cfg.set_dig_dbias_slp(0); // because of dig_peri_pd_en

        cfg.set_deep_slp(true);
        cfg.set_wdt_flashboot_mod_en(false);
        cfg.set_vddsdio_pd_en(true);
        cfg.set_xtal_fpu(false);
        cfg.set_deep_slp_reject(true);
        cfg.set_light_slp_reject(true);
        cfg.set_rtc_dbias_slp(RTC_CNTL_DBIAS_1V10);

        // because of dig_peri_pd_en
        cfg.set_rtc_regulator_fpu(false);
        cfg.set_dbg_atten_slp(RTC_CNTL_DBG_ATTEN_DEEPSLEEP_DEFAULT);

        // because of xtal_fpu
        cfg.set_bias_sleep_monitor(true);
        cfg.set_bias_sleep_slp(true);
        cfg.set_pd_cur_slp(true);

        cfg
    }

    pub(crate) fn base_settings(_rtc: &Rtc<'_>) {
        let cfg = RtcConfig::default();

        // settings derived from esp_clk_init -> rtc_init
        unsafe {
            let rtc_cntl = &*esp32c3::RTC_CNTL::ptr();
            let extmem = &*esp32c3::EXTMEM::ptr();
            let system = &*esp32c3::SYSTEM::ptr();

            rtc_cntl
                .dig_pwc()
                .modify(|_, w| w.wifi_force_pd().clear_bit());

            regi2c_write_mask!(I2C_DIG_REG, I2C_DIG_REG_XPD_RTC_REG, 0);

            regi2c_write_mask!(I2C_DIG_REG, I2C_DIG_REG_XPD_DIG_REG, 0);

            rtc_cntl.ana_conf().modify(|_, w| w.pvtmon_pu().clear_bit());

            rtc_cntl.timer1().modify(|_, w| {
                w.pll_buf_wait()
                    .bits(cfg.pll_wait())
                    .ck8m_wait()
                    .bits(cfg.ck8m_wait())
            });

            // Moved from rtc sleep to rtc init to save sleep function running time
            // set shortest possible sleep time limit

            rtc_cntl
                .timer5()
                .modify(|_, w| w.min_slp_val().bits(RTC_CNTL_MIN_SLP_VAL_MIN));

            let init_cfg = RtcInitConfig::default();

            rtc_cntl.timer3().modify(|_, w| {
                w
                    // set wifi timer
                    .wifi_powerup_timer()
                    .bits(init_cfg.wifi_powerup_cycles())
                    .wifi_wait_timer()
                    .bits(init_cfg.wifi_wait_cycles())
                    // set bt timer
                    .bt_powerup_timer()
                    .bits(init_cfg.bt_powerup_cycles())
                    .bt_wait_timer()
                    .bits(init_cfg.bt_wait_cycles())
            });

            rtc_cntl.timer4().modify(|_, w| {
                w.cpu_top_powerup_timer()
                    .bits(init_cfg.cpu_top_powerup_cycles())
                    .cpu_top_wait_timer()
                    .bits(init_cfg.cpu_top_wait_cycles())
                    // set digital wrap timer
                    .dg_wrap_powerup_timer()
                    .bits(init_cfg.dg_wrap_powerup_cycles())
                    .dg_wrap_wait_timer()
                    .bits(init_cfg.dg_wrap_wait_cycles())
            });

            rtc_cntl.timer6().modify(|_, w| {
                w.dg_peri_powerup_timer()
                    .bits(init_cfg.dg_peri_powerup_cycles())
                    .dg_peri_wait_timer()
                    .bits(init_cfg.dg_peri_wait_cycles())
            });

            // TODO: something about cali_ocode

            // Reset RTC bias to default value (needed if waking up from deep sleep)
            regi2c_write_mask!(
                I2C_DIG_REG,
                I2C_DIG_REG_EXT_RTC_DREG_SLEEP,
                RTC_CNTL_DBIAS_1V10
            );

            // LDO dbias initialization
            // TODO: this modifies g_rtc_dbias_pvt_non_240m and g_dig_dbias_pvt_non_240m.
            //       We're using a high enough default but we should read from the efuse.
            // set_rtc_dig_dbias();

            regi2c_write_mask!(I2C_DIG_REG, I2C_DIG_REG_EXT_RTC_DREG, RTC_CNTL_DBIAS_1V25);
            regi2c_write_mask!(I2C_DIG_REG, I2C_DIG_REG_EXT_DIG_DREG, RTC_CNTL_DBIAS_1V25);

            if cfg.clkctl_init() {
                // clear CMMU clock force on

                extmem
                    .cache_mmu_power_ctrl()
                    .modify(|_, w| w.cache_mmu_mem_force_on().clear_bit());
                // clear tag clock force on

                extmem
                    .icache_tag_power_ctrl()
                    .modify(|_, w| w.icache_tag_mem_force_on().clear_bit());
                // clear register clock force on
                // clear register clock force on
                (*esp32c3::SPI0::ptr())
                    .clock_gate()
                    .modify(|_, w| w.clk_en().clear_bit());
                (*esp32c3::SPI1::ptr())
                    .clock_gate()
                    .modify(|_, w| w.clk_en().clear_bit());
            }

            if cfg.pwrctl_init() {
                rtc_cntl
                    .clk_conf()
                    .modify(|_, w| w.ck8m_force_pu().clear_bit());

                rtc_cntl
                    .options0()
                    .modify(|_, w| w.xtl_force_pu().bit(cfg.xtal_fpu() || cfg.bbpll_fpu()));

                // force pd APLL

                rtc_cntl
                    .ana_conf()
                    .modify(|_, w| w.plla_force_pu().clear_bit().plla_force_pd().set_bit());

                rtc_cntl.ana_conf().modify(|_, w| {
                    w
                        // open sar_i2c protect function to avoid sar_i2c reset when rtc_ldo is low.
                        .reset_por_force_pd()
                        .clear_bit()
                });

                // cancel bbpll force pu if setting no force power up

                rtc_cntl.options0().modify(|_, w| {
                    w.bbpll_force_pu()
                        .bit(cfg.bbpll_fpu())
                        .bbpll_i2c_force_pu()
                        .bit(cfg.bbpll_fpu())
                        .bb_i2c_force_pu()
                        .bit(cfg.bbpll_fpu())
                });

                rtc_cntl.rtc_cntl().modify(|_, w| {
                    w.regulator_force_pu()
                        .clear_bit()
                        .dboost_force_pu()
                        .clear_bit()
                        .dboost_force_pd()
                        .bit(cfg.rtc_dboost_fpd())
                });

                // If this mask is enabled, all soc memories cannot enter power down mode
                // We should control soc memory power down mode from RTC, so we will not touch
                // this register any more

                system
                    .mem_pd_mask()
                    .modify(|_, w| w.lslp_mem_pd_mask().clear_bit());

                // If this pd_cfg is set to 1, all memory won't enter low power mode during
                // light sleep If this pd_cfg is set to 0, all memory will enter low
                // power mode during light sleep
                rtc_sleep_pu(false);

                rtc_cntl.dig_pwc().modify(|_, w| {
                    w.dg_wrap_force_pu()
                        .clear_bit()
                        .wifi_force_pu()
                        .clear_bit()
                        .bt_force_pu()
                        .clear_bit()
                        .cpu_top_force_pu()
                        .clear_bit()
                        .dg_peri_force_pu()
                        .clear_bit()
                });

                rtc_cntl.dig_iso().modify(|_, w| {
                    w.dg_wrap_force_noiso()
                        .clear_bit()
                        .wifi_force_noiso()
                        .clear_bit()
                        .bt_force_noiso()
                        .clear_bit()
                        .cpu_top_force_noiso()
                        .clear_bit()
                        .dg_peri_force_noiso()
                        .clear_bit()
                });

                // if SYSTEM_CPU_WAIT_MODE_FORCE_ON == 0 , the cpu clk will be closed when cpu
                // enter WAITI mode

                system
                    .cpu_per_conf()
                    .modify(|_, w| w.cpu_wait_mode_force_on().bit(!cfg.cpu_waiti_clk_gate()));

                // cancel digital PADS force no iso

                rtc_cntl.dig_iso().modify(|_, w| {
                    w.dg_pad_force_unhold()
                        .clear_bit()
                        .dg_pad_force_noiso()
                        .clear_bit()
                });
            }

            // force power down modem(wifi and ble) power domain

            rtc_cntl
                .dig_iso()
                .modify(|_, w| w.wifi_force_iso().set_bit().bt_force_iso().set_bit());

            rtc_cntl
                .dig_pwc()
                .modify(|_, w| w.wifi_force_pd().set_bit().bt_force_pd().set_bit());

            rtc_cntl.int_ena().write(|w| w.bits(0));
            rtc_cntl.int_clr().write(|w| w.bits(u32::MAX));

            regi2c_write_mask!(I2C_ULP, I2C_ULP_IR_FORCE_XPD_CK, 1);
        }
    }

    pub(crate) fn apply(&self) {
        // like esp-idf rtc_sleep_init()
        let rtc_cntl = unsafe { &*esp32c3::RTC_CNTL::ptr() };

        if self.lslp_mem_inf_fpu() {
            rtc_sleep_pu(true);
        }

        if self.wifi_pd_en() {
            rtc_cntl.dig_iso().modify(|_, w| {
                w.wifi_force_noiso()
                    .clear_bit()
                    .wifi_force_iso()
                    .clear_bit()
            });

            rtc_cntl
                .dig_pwc()
                .modify(|_, w| w.wifi_force_pu().clear_bit().wifi_pd_en().set_bit());
        } else {
            rtc_cntl.dig_pwc().modify(|_, w| w.wifi_pd_en().clear_bit());
        }
        if self.bt_pd_en() {
            rtc_cntl
                .dig_iso()
                .modify(|_, w| w.bt_force_noiso().clear_bit().bt_force_iso().clear_bit());

            rtc_cntl
                .dig_pwc()
                .modify(|_, w| w.bt_force_pu().clear_bit().bt_pd_en().set_bit());
        } else {
            rtc_cntl.dig_pwc().modify(|_, w| w.bt_pd_en().clear_bit());
        }

        rtc_cntl
            .dig_pwc()
            .modify(|_, w| w.cpu_top_pd_en().bit(self.cpu_pd_en()));

        rtc_cntl
            .dig_pwc()
            .modify(|_, w| w.dg_peri_pd_en().bit(self.dig_peri_pd_en()));

        unsafe {
            rtc_cntl.bias_conf().modify(|_, w| {
                w.dbg_atten_monitor()
                    .bits(RTC_CNTL_DBG_ATTEN_MONITOR_DEFAULT)
                    // We have config values for this in self, so I don't know why we're setting
                    // hardcoded defaults for these next two. It's what IDF does...
                    .bias_sleep_monitor()
                    .bit(RTC_CNTL_BIASSLP_MONITOR_DEFAULT)
                    .pd_cur_monitor()
                    .bit(RTC_CNTL_PD_CUR_MONITOR_DEFAULT)
            });

            assert!(!self.pd_cur_slp() || self.bias_sleep_slp());

            regi2c_write_mask!(
                I2C_DIG_REG,
                I2C_DIG_REG_EXT_RTC_DREG_SLEEP,
                self.rtc_dbias_slp()
            );

            regi2c_write_mask!(
                I2C_DIG_REG,
                I2C_DIG_REG_EXT_DIG_DREG_SLEEP,
                self.dig_dbias_slp()
            );

            rtc_cntl.bias_conf().modify(|_, w| {
                w.dbg_atten_deep_slp()
                    .bits(self.dbg_atten_slp())
                    .bias_sleep_deep_slp()
                    .bit(self.bias_sleep_slp())
                    .pd_cur_deep_slp()
                    .bit(self.pd_cur_slp())
            });

            if self.deep_slp() {
                regi2c_write_mask!(I2C_ULP, I2C_ULP_IR_FORCE_XPD_CK, 0);

                rtc_cntl
                    .dig_pwc()
                    .modify(|_, w| w.dg_wrap_pd_en().set_bit());

                rtc_cntl.ana_conf().modify(|_, w| {
                    w.ckgen_i2c_pu()
                        .clear_bit()
                        .pll_i2c_pu()
                        .clear_bit()
                        .rfrx_pbus_pu()
                        .clear_bit()
                        .txrf_i2c_pu()
                        .clear_bit()
                });

                rtc_cntl
                    .options0()
                    .modify(|_, w| w.bb_i2c_force_pu().clear_bit());
            } else {
                rtc_cntl.bias_conf().modify(|_, w| {
                    w.dg_vdd_drv_b_slp_en()
                        .set_bit()
                        .dg_vdd_drv_b_slp()
                        .bits(RTC_CNTL_DG_VDD_DRV_B_SLP_DEFAULT)
                });

                rtc_cntl
                    .dig_pwc()
                    .modify(|_, w| w.dg_wrap_pd_en().clear_bit());
            }

            // mem force pu

            rtc_cntl
                .dig_pwc()
                .modify(|_, w| w.lslp_mem_force_pu().set_bit());

            rtc_cntl
                .rtc_cntl()
                .modify(|_, w| w.regulator_force_pu().bit(self.rtc_regulator_fpu()));

            rtc_cntl.clk_conf().modify(|_, w| {
                w.ck8m_force_pu()
                    .bit(!self.int_8m_pd_en())
                    .ck8m_force_nogating()
                    .bit(!self.int_8m_pd_en())
            });

            // enable VDDSDIO control by state machine

            rtc_cntl.sdio_conf().modify(|_, w| {
                w.sdio_force()
                    .clear_bit()
                    .sdio_reg_pd_en()
                    .bit(self.vddsdio_pd_en())
            });

            rtc_cntl.slp_reject_conf().modify(|_, w| {
                w.deep_slp_reject_en()
                    .bit(self.deep_slp_reject())
                    .light_slp_reject_en()
                    .bit(self.light_slp_reject())
            });

            rtc_cntl
                .options0()
                .modify(|_, w| w.xtl_force_pu().bit(self.xtal_fpu()));

            rtc_cntl
                .clk_conf()
                .modify(|_, w| w.xtal_global_force_nogating().bit(self.xtal_fpu()));
        }
    }

    pub(crate) fn start_sleep(&self, wakeup_triggers: WakeTriggers) {
        unsafe {
            let rtc_cntl = &*esp32c3::RTC_CNTL::ptr();

            // set bits for what can wake us up
            rtc_cntl
                .wakeup_state()
                .modify(|_, w| w.wakeup_ena().bits(wakeup_triggers.0.into()));

            rtc_cntl
                .state0()
                .write(|w| w.sleep_en().set_bit().slp_wakeup().set_bit());
        }
    }

    pub(crate) fn finish_sleep(&self) {
        // In deep sleep mode, we never get here
        unsafe {
            let rtc_cntl = &*esp32c3::RTC_CNTL::ptr();

            rtc_cntl.int_clr().write(|w| {
                w.slp_reject()
                    .clear_bit_by_one()
                    .slp_wakeup()
                    .clear_bit_by_one()
            });

            // restore config if it is a light sleep
            if self.lslp_mem_inf_fpu() {
                rtc_sleep_pu(true);
            }
        }
    }
}
