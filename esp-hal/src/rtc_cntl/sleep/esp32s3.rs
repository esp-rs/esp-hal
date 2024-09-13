use super::{
    Ext0WakeupSource,
    Ext1WakeupSource,
    TimerWakeupSource,
    WakeSource,
    WakeTriggers,
    WakeupLevel,
};
use crate::{
    gpio::{RtcFunction, RtcPin},
    regi2c_write_mask,
    rtc_cntl::{sleep::RtcioWakeupSource, Clock, Rtc, RtcClock},
};

const I2C_DIG_REG: u32 = 0x6d;
const I2C_DIG_REG_HOSTID: u32 = 1;

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

// Approximate mapping of voltages to RTC_CNTL_DBIAS_WAK, RTC_CNTL_DBIAS_SLP,
// RTC_CNTL_DIG_DBIAS_WAK, RTC_CNTL_DIG_DBIAS_SLP values.
// Valid if RTC_CNTL_DBG_ATTEN is 0.
/// Digital bias setting for 0.90V.
pub const RTC_CNTL_DBIAS_0V90: u32 = 13;
/// Digital bias setting for 0.95V.
pub const RTC_CNTL_DBIAS_0V95: u32 = 16;
/// Digital bias setting for 1.00V.
pub const RTC_CNTL_DBIAS_1V00: u32 = 18;
/// Digital bias setting for 1.05V.
pub const RTC_CNTL_DBIAS_1V05: u32 = 20;
/// Digital bias setting for 1.10V.
pub const RTC_CNTL_DBIAS_1V10: u32 = 23;
/// Digital bias setting for 1.15V.
pub const RTC_CNTL_DBIAS_1V15: u32 = 25;
/// Digital bias setting for 1.20V.
pub const RTC_CNTL_DBIAS_1V20: u32 = 28;
/// Digital bias setting for 1.25V.
pub const RTC_CNTL_DBIAS_1V25: u32 = 30;
/// Digital bias setting for 1.30V. Voltage is approximately 1.34V in practice.
pub const RTC_CNTL_DBIAS_1V30: u32 = 31;
/// Default monitor debug attenuation value.
pub const RTC_CNTL_DBG_ATTEN_MONITOR_DEFAULT: u8 = 0;
/// ULP co-processor touch start wait time during sleep, set to maximum.
pub const RTC_CNTL_ULPCP_TOUCH_START_WAIT_IN_SLEEP: u16 = 0xFF;
/// ULP co-processor touch start wait time default value.
pub const RTC_CNTL_ULPCP_TOUCH_START_WAIT_DEFAULT: u16 = 0x10;
/// Default wait time for PLL buffer during startup.
pub const RTC_CNTL_PLL_BUF_WAIT_DEFAULT: u8 = 20;
/// Default wait time for CK8M during startup.
pub const RTC_CNTL_CK8M_WAIT_DEFAULT: u8 = 20;
/// Minimum sleep value.
pub const RTC_CNTL_MIN_SLP_VAL_MIN: u8 = 2;
/// Deep sleep debug attenuation setting for ultra-low power mode.
pub const RTC_CNTL_DBG_ATTEN_DEEPSLEEP_ULTRA_LOW: u8 = 15;
/// Power-up setting for other blocks.
pub const OTHER_BLOCKS_POWERUP: u8 = 1;
/// Wait cycles for other blocks.
pub const OTHER_BLOCKS_WAIT: u16 = 1;
/// WiFi power-up cycles.
pub const WIFI_POWERUP_CYCLES: u8 = OTHER_BLOCKS_POWERUP;
/// WiFi wait cycles.
pub const WIFI_WAIT_CYCLES: u16 = OTHER_BLOCKS_WAIT;
/// Bluetooth power-up cycles.
pub const BT_POWERUP_CYCLES: u8 = OTHER_BLOCKS_POWERUP;
/// Bluetooth wait cycles.
pub const BT_WAIT_CYCLES: u16 = OTHER_BLOCKS_WAIT;
/// RTC power-up cycles.
pub const RTC_POWERUP_CYCLES: u8 = OTHER_BLOCKS_POWERUP;
/// RTC wait cycles.
pub const RTC_WAIT_CYCLES: u16 = OTHER_BLOCKS_WAIT;
/// CPU top power-up cycles.
pub const CPU_TOP_POWERUP_CYCLES: u8 = OTHER_BLOCKS_POWERUP;
/// CPU top wait cycles.
pub const CPU_TOP_WAIT_CYCLES: u16 = OTHER_BLOCKS_WAIT;
/// DG wrap power-up cycles.
pub const DG_WRAP_POWERUP_CYCLES: u8 = OTHER_BLOCKS_POWERUP;
/// DG wrap wait cycles.
pub const DG_WRAP_WAIT_CYCLES: u16 = OTHER_BLOCKS_WAIT;
/// DG peripheral power-up cycles.
pub const DG_PERI_POWERUP_CYCLES: u8 = OTHER_BLOCKS_POWERUP;
/// DG peripheral wait cycles.
pub const DG_PERI_WAIT_CYCLES: u16 = OTHER_BLOCKS_WAIT;
/// RTC memory power-up cycles.
pub const RTC_MEM_POWERUP_CYCLES: u8 = OTHER_BLOCKS_POWERUP;
/// RTC memory wait cycles.
pub const RTC_MEM_WAIT_CYCLES: u16 = OTHER_BLOCKS_WAIT;

impl WakeSource for TimerWakeupSource {
    fn apply(
        &self,
        rtc: &Rtc<'_>,
        triggers: &mut WakeTriggers,
        _sleep_config: &mut RtcSleepConfig,
    ) {
        triggers.set_timer(true);
        let rtc_cntl = unsafe { &*esp32s3::RTC_CNTL::ptr() };
        let clock_freq = RtcClock::get_slow_freq();
        // TODO: maybe add sleep time adjustlemnt like idf
        // TODO: maybe add check to prevent overflow?
        let clock_hz = clock_freq.frequency().to_Hz() as u64;
        let ticks = self.duration.as_micros() as u64 * clock_hz / 1_000_000u64;
        // "alarm" time in slow rtc ticks
        let now = rtc.time_since_boot_raw();
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

impl<P: RtcPin> WakeSource for Ext0WakeupSource<'_, P> {
    fn apply(
        &self,
        _rtc: &Rtc<'_>,
        triggers: &mut WakeTriggers,
        sleep_config: &mut RtcSleepConfig,
    ) {
        // don't power down RTC peripherals
        sleep_config.set_rtc_peri_pd_en(false);
        triggers.set_ext0(true);

        // set pin to RTC function
        self.pin
            .borrow_mut()
            .rtc_set_config(true, true, RtcFunction::Rtc);

        unsafe {
            let rtc_io = &*esp32s3::RTC_IO::ptr();
            // set pin register field
            rtc_io
                .ext_wakeup0()
                .modify(|_, w| w.sel().bits(self.pin.borrow().rtc_number()));
            // set level register field
            let rtc_cntl = &*esp32s3::RTC_CNTL::ptr();
            rtc_cntl
                .ext_wakeup_conf()
                .modify(|_r, w| w.ext_wakeup0_lv().bit(self.level == WakeupLevel::High));
        }
    }
}

impl<P: RtcPin> Drop for Ext0WakeupSource<'_, P> {
    fn drop(&mut self) {
        // should we have saved the pin configuration first?
        // set pin back to IO_MUX (input_enable and func have no effect when pin is sent
        // to IO_MUX)
        self.pin
            .borrow_mut()
            .rtc_set_config(true, false, RtcFunction::Rtc);
    }
}

impl WakeSource for Ext1WakeupSource<'_, '_> {
    fn apply(
        &self,
        _rtc: &Rtc<'_>,
        triggers: &mut WakeTriggers,
        sleep_config: &mut RtcSleepConfig,
    ) {
        // don't power down RTC peripherals
        sleep_config.set_rtc_peri_pd_en(false);
        triggers.set_ext1(true);

        // set pins to RTC function
        let mut pins = self.pins.borrow_mut();
        let mut bits = 0u32;
        for pin in pins.iter_mut() {
            pin.rtc_set_config(true, true, RtcFunction::Rtc);
            bits |= 1 << pin.rtc_number();
        }

        unsafe {
            let rtc_cntl = &*esp32s3::RTC_CNTL::ptr();
            // clear previous wakeup status
            rtc_cntl
                .ext_wakeup1()
                .modify(|_, w| w.ext_wakeup1_status_clr().set_bit());
            // set pin register field
            rtc_cntl
                .ext_wakeup1()
                .modify(|_, w| w.ext_wakeup1_sel().bits(bits));
            // set level register field
            rtc_cntl
                .ext_wakeup_conf()
                .modify(|_r, w| w.ext_wakeup1_lv().bit(self.level == WakeupLevel::High));
        }
    }
}

impl Drop for Ext1WakeupSource<'_, '_> {
    fn drop(&mut self) {
        // should we have saved the pin configuration first?
        // set pin back to IO_MUX (input_enable and func have no effect when pin is sent
        // to IO_MUX)
        let mut pins = self.pins.borrow_mut();
        for pin in pins.iter_mut() {
            pin.rtc_set_config(true, false, RtcFunction::Rtc);
        }
    }
}

impl<'a, 'b> RtcioWakeupSource<'a, 'b> {
    fn apply_pin(&self, pin: &mut dyn RtcPin, level: WakeupLevel) {
        let rtcio = unsafe { &*crate::peripherals::RTC_IO::PTR };

        pin.rtc_set_config(true, true, RtcFunction::Rtc);

        rtcio.pin(pin.number() as usize).modify(|_, w| unsafe {
            w.wakeup_enable().set_bit().int_type().bits(match level {
                WakeupLevel::Low => 4,
                WakeupLevel::High => 5,
            })
        });
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

        // don't power down RTC peripherals
        sleep_config.set_rtc_peri_pd_en(false);
        triggers.set_gpio(true);

        // Since we only use RTCIO pins, we can keep deep sleep enabled.
        let sens = unsafe { &*crate::peripherals::SENS::PTR };

        // TODO: disable clock when not in use
        sens.sar_peri_clk_gate_conf()
            .modify(|_, w| w.iomux_clk_en().set_bit());

        for (pin, level) in pins.iter_mut() {
            self.apply_pin(*pin, *level);
        }
    }
}

impl Drop for RtcioWakeupSource<'_, '_> {
    fn drop(&mut self) {
        // should we have saved the pin configuration first?
        // set pin back to IO_MUX (input_enable and func have no effect when pin is sent
        // to IO_MUX)
        let mut pins = self.pins.borrow_mut();
        for (pin, _level) in pins.iter_mut() {
            pin.rtc_set_config(true, false, RtcFunction::Rtc);
        }
    }
}

bitfield::bitfield! {
    /// Configuration for the RTC sleep behavior.
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
    /// power down Modem(wifi and ble)
    pub modem_pd_en, set_modem_pd_en: 5;
    /// power down CPU, but not restart when lightsleep.
    pub cpu_pd_en, set_cpu_pd_en: 6;
    /// Power down Internal 8M oscillator
    pub int_8m_pd_en, set_int_8m_pd_en: 7;
    /// power down digital peripherals
    pub dig_peri_pd_en, set_dig_peri_pd_en: 8;
    /// power down digital domain
    pub deep_slp, set_deep_slp: 9;
    /// enable WDT flashboot mode
    pub wdt_flashboot_mod_en, set_wdt_flashboot_mod_en: 10;
    /// set bias for digital domain, in sleep mode
    pub u32, dig_dbias_slp, set_dig_dbias_slp: 15, 11;
    /// set bias for RTC domain, in sleep mode
    pub u32, rtc_dbias_slp, set_rtc_dbias_slp: 20, 16;
    /// circuit control parameter, in monitor mode
    pub bias_sleep_monitor, set_bias_sleep_monitor: 21;
    /// voltage parameter, in sleep mode
    pub u8, dbg_atten_slp, set_dbg_atten_slp: 25, 22;
    /// circuit control parameter, in sleep mode
    pub bias_sleep_slp, set_bias_sleep_slp: 26;
    /// circuit control parameter, in monitor mode
    pub pd_cur_monitor, set_pd_cur_monitor: 27;
    /// circuit control parameter, in sleep mode
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

const SYSCON_SRAM_POWER_UP: u16 = 0x7FF;
const SYSCON_ROM_POWER_UP: u8 = 0x7;

fn rtc_sleep_pu(val: bool) {
    let rtc_cntl = unsafe { &*esp32s3::RTC_CNTL::ptr() };
    let syscon = unsafe { &*esp32s3::APB_CTRL::ptr() };
    let bb = unsafe { &*esp32s3::BB::ptr() };
    let nrx = unsafe { &*esp32s3::NRX::ptr() };
    let fe = unsafe { &*esp32s3::FE::ptr() };
    let fe2 = unsafe { &*esp32s3::FE2::ptr() };

    rtc_cntl
        .dig_pwc()
        .modify(|_, w| w.lslp_mem_force_pu().bit(val));

    rtc_cntl
        .pwc()
        .modify(|_, w| w.slowmem_force_lpu().bit(val).fastmem_force_lpu().bit(val));

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

    nrx.nrxpd_ctrl().modify(|_, w| {
        w.rx_rot_force_pu()
            .bit(val)
            .vit_force_pu()
            .bit(val)
            .demap_force_pu()
            .bit(val)
    });

    fe.gen_ctrl().modify(|_, w| w.iq_est_force_pu().bit(val));

    fe2.tx_interp_ctrl()
        .modify(|_, w| w.tx_inf_force_pu().bit(val));

    syscon.mem_power_up().modify(|_r, w| unsafe {
        w.sram_power_up()
            .bits(if val { SYSCON_SRAM_POWER_UP } else { 0 })
            .rom_power_up()
            .bits(if val { SYSCON_ROM_POWER_UP } else { 0 })
    });
}

impl RtcSleepConfig {
    /// Configures the RTC for deep sleep mode.
    pub fn deep() -> Self {
        // Set up for ultra-low power sleep. Wakeup sources may modify these settings.
        let mut cfg = Self::default();

        cfg.set_lslp_mem_inf_fpu(false);
        cfg.set_rtc_mem_inf_follow_cpu(true); // ?
        cfg.set_rtc_fastmem_pd_en(true);
        cfg.set_rtc_slowmem_pd_en(true);
        cfg.set_rtc_peri_pd_en(true);
        cfg.set_modem_pd_en(true);
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
        cfg.set_dbg_atten_slp(RTC_CNTL_DBG_ATTEN_DEEPSLEEP_ULTRA_LOW);

        // because of xtal_fpu
        cfg.set_bias_sleep_monitor(true);
        cfg.set_pd_cur_monitor(true);
        cfg.set_bias_sleep_slp(true);
        cfg.set_pd_cur_slp(true);

        cfg
    }

    pub(crate) fn base_settings(_rtc: &Rtc<'_>) {
        // settings derived from esp_clk_init -> rtc_init
        unsafe {
            let rtc_cntl = &*esp32s3::RTC_CNTL::ptr();
            let syscon = &*esp32s3::APB_CTRL::ptr();
            let extmem = &*esp32s3::EXTMEM::ptr();
            let system = &*esp32s3::SYSTEM::ptr();

            rtc_cntl
                .dig_pwc()
                .modify(|_, w| w.wifi_force_pd().clear_bit());

            regi2c_write_mask!(I2C_DIG_REG, I2C_DIG_REG_XPD_RTC_REG, 0);
            regi2c_write_mask!(I2C_DIG_REG, I2C_DIG_REG_XPD_DIG_REG, 0);

            rtc_cntl.ana_conf().modify(|_, w| w.pvtmon_pu().clear_bit());

            rtc_cntl.timer1().modify(|_, w| {
                w.pll_buf_wait()
                    .bits(RTC_CNTL_PLL_BUF_WAIT_DEFAULT)
                    .ck8m_wait()
                    .bits(RTC_CNTL_CK8M_WAIT_DEFAULT)
            });

            // Moved from rtc sleep to rtc init to save sleep function running time
            // set shortest possible sleep time limit

            rtc_cntl
                .timer5()
                .modify(|_, w| w.min_slp_val().bits(RTC_CNTL_MIN_SLP_VAL_MIN));

            rtc_cntl.timer3().modify(|_, w| {
                w
                    // set wifi timer
                    .wifi_powerup_timer()
                    .bits(WIFI_POWERUP_CYCLES)
                    .wifi_wait_timer()
                    .bits(WIFI_WAIT_CYCLES)
                    // set bt timer
                    .bt_powerup_timer()
                    .bits(BT_POWERUP_CYCLES)
                    .bt_wait_timer()
                    .bits(BT_WAIT_CYCLES)
            });

            rtc_cntl.timer6().modify(|_, w| {
                w.cpu_top_powerup_timer()
                    .bits(CPU_TOP_POWERUP_CYCLES)
                    .cpu_top_wait_timer()
                    .bits(CPU_TOP_WAIT_CYCLES)
            });

            rtc_cntl.timer4().modify(|_, w| {
                w
                    // set rtc peri timer
                    .powerup_timer()
                    .bits(RTC_POWERUP_CYCLES)
                    .wait_timer()
                    .bits(RTC_WAIT_CYCLES)
                    // set digital wrap timer
                    .dg_wrap_powerup_timer()
                    .bits(DG_WRAP_POWERUP_CYCLES)
                    .dg_wrap_wait_timer()
                    .bits(DG_WRAP_WAIT_CYCLES)
            });

            rtc_cntl.timer6().modify(|_, w| {
                w.dg_peri_powerup_timer()
                    .bits(DG_PERI_POWERUP_CYCLES)
                    .dg_peri_wait_timer()
                    .bits(DG_PERI_WAIT_CYCLES)
            });

            // Reset RTC bias to default value (needed if waking up from deep sleep)
            regi2c_write_mask!(
                I2C_DIG_REG,
                I2C_DIG_REG_EXT_RTC_DREG_SLEEP,
                RTC_CNTL_DBIAS_1V10
            );
            regi2c_write_mask!(I2C_DIG_REG, I2C_DIG_REG_EXT_RTC_DREG, RTC_CNTL_DBIAS_1V10);

            // Set the wait time to the default value.

            rtc_cntl.timer2().modify(|_, w| {
                w.ulpcp_touch_start_wait()
                    .bits(RTC_CNTL_ULPCP_TOUCH_START_WAIT_DEFAULT)
            });

            // LDO dbias initialization
            // TODO: this modifies g_rtc_dbias_pvt_non_240m and g_dig_dbias_pvt_non_240m.
            //       We're using a high enough default but we should read from the efuse.
            // rtc_set_stored_dbias();

            regi2c_write_mask!(I2C_DIG_REG, I2C_DIG_REG_EXT_RTC_DREG, RTC_CNTL_DBIAS_1V25);
            regi2c_write_mask!(I2C_DIG_REG, I2C_DIG_REG_EXT_DIG_DREG, RTC_CNTL_DBIAS_1V25);

            // clear CMMU clock force on

            extmem
                .cache_mmu_power_ctrl()
                .modify(|_, w| w.cache_mmu_mem_force_on().clear_bit());

            // clear clkgate force on
            syscon.clkgate_force_on().write(|w| w.bits(0));

            // clear tag clock force on

            extmem
                .dcache_tag_power_ctrl()
                .modify(|_, w| w.dcache_tag_mem_force_on().clear_bit());

            extmem
                .icache_tag_power_ctrl()
                .modify(|_, w| w.icache_tag_mem_force_on().clear_bit());

            // clear register clock force on
            (*esp32s3::SPI0::ptr())
                .clock_gate()
                .modify(|_, w| w.clk_en().clear_bit());
            (*esp32s3::SPI1::ptr())
                .clock_gate()
                .modify(|_, w| w.clk_en().clear_bit());

            rtc_cntl
                .clk_conf()
                .modify(|_, w| w.ck8m_force_pu().clear_bit());

            rtc_cntl
                .options0()
                .modify(|_, w| w.xtl_force_pu().clear_bit());

            rtc_cntl.ana_conf().modify(|_, w| {
                w
                    // open sar_i2c protect function to avoid sar_i2c reset when rtc_ldo is low.
                    // clear i2c_reset_protect pd force, need tested in low temperature.
                    // NOTE: this bit is written again in esp-idf, but it's not clear why.
                    .i2c_reset_por_force_pd()
                    .clear_bit()
            });

            // cancel bbpll force pu if setting no force power up

            rtc_cntl.options0().modify(|_, w| {
                w.bbpll_force_pu()
                    .clear_bit()
                    .bbpll_i2c_force_pu()
                    .clear_bit()
                    .bb_i2c_force_pu()
                    .clear_bit()
            });

            // cancel RTC REG force PU

            rtc_cntl.pwc().modify(|_, w| w.force_pu().clear_bit());

            rtc_cntl.rtc().modify(|_, w| {
                w.regulator_force_pu()
                    .clear_bit()
                    .dboost_force_pu()
                    .clear_bit()
            });

            rtc_cntl.pwc().modify(|_, w| {
                w.slowmem_force_noiso()
                    .clear_bit()
                    .fastmem_force_noiso()
                    .clear_bit()
            });

            rtc_cntl.rtc().modify(|_, w| w.dboost_force_pd().set_bit());

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

            rtc_cntl
                .dig_pwc()
                .modify(|_, w| w.dg_wrap_force_pu().clear_bit());

            rtc_cntl.dig_iso().modify(|_, w| {
                w.dg_wrap_force_noiso()
                    .clear_bit()
                    .dg_wrap_force_iso()
                    .clear_bit()
            });

            rtc_cntl.dig_iso().modify(|_, w| {
                w.wifi_force_noiso()
                    .clear_bit()
                    .wifi_force_iso()
                    .clear_bit()
            });

            rtc_cntl
                .dig_pwc()
                .modify(|_, w| w.wifi_force_pu().clear_bit());

            rtc_cntl
                .dig_iso()
                .modify(|_, w| w.bt_force_noiso().clear_bit().bt_force_iso().clear_bit());

            rtc_cntl
                .dig_pwc()
                .modify(|_, w| w.bt_force_pu().clear_bit());

            rtc_cntl.dig_iso().modify(|_, w| {
                w.cpu_top_force_noiso()
                    .clear_bit()
                    .cpu_top_force_iso()
                    .clear_bit()
            });

            rtc_cntl
                .dig_pwc()
                .modify(|_, w| w.cpu_top_force_pu().clear_bit());

            rtc_cntl.dig_iso().modify(|_, w| {
                w.dg_peri_force_noiso()
                    .clear_bit()
                    .dg_peri_force_iso()
                    .clear_bit()
            });

            rtc_cntl
                .dig_pwc()
                .modify(|_, w| w.dg_peri_force_pu().clear_bit());

            rtc_cntl.pwc().modify(|_, w| {
                w.force_noiso()
                    .clear_bit()
                    .force_iso()
                    .clear_bit()
                    .force_pu()
                    .clear_bit()
            });

            // if SYSTEM_CPU_WAIT_MODE_FORCE_ON == 0,
            // the cpu clk will be closed when cpu enter WAITI mode

            system
                .cpu_per_conf()
                .modify(|_, w| w.cpu_wait_mode_force_on().clear_bit());

            // cancel digital PADS force no iso

            rtc_cntl.dig_iso().modify(|_, w| {
                w.dg_pad_force_unhold()
                    .clear_bit()
                    .dg_pad_force_noiso()
                    .clear_bit()
            });

            // force power down modem(wifi and ble) power domain

            rtc_cntl
                .dig_iso()
                .modify(|_, w| w.wifi_force_iso().set_bit());

            rtc_cntl
                .dig_pwc()
                .modify(|_, w| w.wifi_force_pd().set_bit());

            rtc_cntl.int_ena().write(|w| w.bits(0));
            rtc_cntl.int_clr().write(|w| w.bits(u32::MAX));
        }
    }

    pub(crate) fn apply(&self) {
        // like esp-idf rtc_sleep_init()
        let rtc_cntl = unsafe { &*esp32s3::RTC_CNTL::ptr() };

        if self.lslp_mem_inf_fpu() {
            rtc_sleep_pu(true);
        }

        if self.modem_pd_en() {
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

        if self.cpu_pd_en() {
            rtc_cntl.dig_iso().modify(|_, w| {
                w.cpu_top_force_noiso()
                    .clear_bit()
                    .cpu_top_force_iso()
                    .clear_bit()
            });

            rtc_cntl
                .dig_pwc()
                .modify(|_, w| w.cpu_top_force_pu().clear_bit().cpu_top_pd_en().set_bit());
        } else {
            rtc_cntl
                .dig_pwc()
                .modify(|_, w| w.cpu_top_pd_en().clear_bit());
        }

        if self.dig_peri_pd_en() {
            rtc_cntl.dig_iso().modify(|_, w| {
                w.dg_peri_force_noiso()
                    .clear_bit()
                    .dg_peri_force_iso()
                    .clear_bit()
            });

            rtc_cntl
                .dig_pwc()
                .modify(|_, w| w.dg_peri_force_pu().clear_bit().dg_peri_pd_en().set_bit());
        } else {
            rtc_cntl
                .dig_pwc()
                .modify(|_, w| w.dg_peri_pd_en().clear_bit());
        }

        if self.rtc_peri_pd_en() {
            rtc_cntl.pwc().modify(|_, w| {
                w.force_noiso()
                    .clear_bit()
                    .force_iso()
                    .clear_bit()
                    .force_pu()
                    .clear_bit()
                    .pd_en()
                    .set_bit()
            });
        } else {
            rtc_cntl.pwc().modify(|_, w| w.pd_en().clear_bit());
        }

        unsafe {
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
                    .dbg_atten_monitor()
                    .bits(RTC_CNTL_DBG_ATTEN_MONITOR_DEFAULT)
                    .bias_sleep_monitor()
                    .bit(self.bias_sleep_monitor())
                    .pd_cur_monitor()
                    .bit(self.pd_cur_monitor())
            });

            if self.deep_slp() {
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
                rtc_cntl
                    .regulator_drv_ctrl()
                    .modify(|_, w| w.dg_vdd_drv_b_slp().bits(0xF));

                rtc_cntl
                    .dig_pwc()
                    .modify(|_, w| w.dg_wrap_pd_en().clear_bit());
            }

            // mem force pu

            rtc_cntl
                .dig_pwc()
                .modify(|_, w| w.lslp_mem_force_pu().set_bit());

            rtc_cntl
                .rtc()
                .modify(|_, w| w.regulator_force_pu().bit(self.rtc_regulator_fpu()));

            rtc_cntl
                .clk_conf()
                .modify(|_, w| w.ck8m_force_pu().bit(!self.int_8m_pd_en()));

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

            // Set wait cycle for touch or COCPU after deep sleep and light
            // sleep.

            rtc_cntl.timer2().modify(|_, w| {
                w.ulpcp_touch_start_wait()
                    .bits(RTC_CNTL_ULPCP_TOUCH_START_WAIT_IN_SLEEP)
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
            let rtc_cntl = &*esp32s3::RTC_CNTL::ptr();

            rtc_cntl
                .reset_state()
                .modify(|_, w| w.procpu_stat_vector_sel().set_bit());

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
            let rtc_cntl = &*esp32s3::RTC_CNTL::ptr();

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

            // Recover default wait cycle for touch or COCPU after wakeup.

            rtc_cntl.timer2().modify(|_, w| {
                w.ulpcp_touch_start_wait()
                    .bits(RTC_CNTL_ULPCP_TOUCH_START_WAIT_DEFAULT)
            });
        }
    }
}
