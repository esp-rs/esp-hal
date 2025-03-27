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
    peripherals::{EXTMEM, LPWR, RTC_IO, SENS, SPI0, SPI1, SYSTEM},
    rom::regi2c_write_mask,
    rtc_cntl::{sleep::RtcioWakeupSource, Clock, Rtc, RtcClock},
};

// Approximate mapping of voltages to RTC_CNTL_DBIAS_WAK, RTC_CNTL_DBIAS_SLP,
// RTC_CNTL_DIG_DBIAS_WAK, RTC_CNTL_DIG_DBIAS_SLP values.
// Valid if RTC_CNTL_DBG_ATTEN is 0.
/// Digital bias setting for 0.90V.
pub const RTC_CNTL_DBIAS_0V90: u8 = 0;
/// Digital bias setting for 0.95V.
pub const RTC_CNTL_DBIAS_0V95: u8 = 1;
/// Digital bias setting for 1.00V.
pub const RTC_CNTL_DBIAS_1V00: u8 = 2;
/// Digital bias setting for 1.05V.
pub const RTC_CNTL_DBIAS_1V05: u8 = 3;
/// Digital bias setting for 1.10V.
pub const RTC_CNTL_DBIAS_1V10: u8 = 4;
/// Digital bias setting for 1.15V.
pub const RTC_CNTL_DBIAS_1V15: u8 = 5;
/// Digital bias setting for 1.20V.
pub const RTC_CNTL_DBIAS_1V20: u8 = 6;
/// Digital bias setting for 1.25V.
pub const RTC_CNTL_DBIAS_1V25: u8 = 7;
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
/// Default wait time for XTL buffer during startup.
pub const RTC_CNTL_XTL_BUF_WAIT_DEFAULT: u8 = 100;
/// Minimum sleep value.
pub const RTC_CNTL_MIN_SLP_VAL_MIN: u8 = 2;
/// Deep sleep debug attenuation setting for ultra-low power mode.
pub const RTC_CNTL_DBG_ATTEN_DEEPSLEEP_DEFAULT: u8 = 15;
/// Power-up setting for other blocks.
pub const OTHER_BLOCKS_POWERUP: u8 = 1;
/// Wait cycles for other blocks.
pub const OTHER_BLOCKS_WAIT: u16 = 1;
/// WiFi power-up cycles.
pub const WIFI_POWERUP_CYCLES: u8 = OTHER_BLOCKS_POWERUP;
/// WiFi wait cycles.
pub const WIFI_WAIT_CYCLES: u16 = OTHER_BLOCKS_WAIT;
/// RTC power-up cycles.
pub const RTC_POWERUP_CYCLES: u8 = OTHER_BLOCKS_POWERUP;
/// RTC wait cycles.
pub const RTC_WAIT_CYCLES: u16 = OTHER_BLOCKS_WAIT;
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

const I2C_BOD_REG: u32 = 0x61;
const I2C_BOD_REG_HOSTID: u32 = 1;

const I2C_BOD_REG_THRESHOLD: u32 = 0x5;
const I2C_BOD_REG_THRESHOLD_MSB: u32 = 2;
const I2C_BOD_REG_THRESHOLD_LSB: u32 = 0;

impl WakeSource for TimerWakeupSource {
    fn apply(
        &self,
        rtc: &Rtc<'_>,
        triggers: &mut WakeTriggers,
        _sleep_config: &mut RtcSleepConfig,
    ) {
        triggers.set_timer(true);
        let rtc_cntl = LPWR::regs();
        let clock_freq = RtcClock::slow_freq();
        // TODO: maybe add sleep time adjustlemnt like idf
        // TODO: maybe add check to prevent overflow?
        let clock_hz = clock_freq.frequency().as_hz() as u64;
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
                w.slp_val_hi().bits(((time_in_ticks >> 32) & 0xffff) as u16);
                w.main_timer_alarm_en().set_bit()
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
        // Checked TODO: remove comment
        // don't power down RTC peripherals
        sleep_config.set_rtc_peri_pd_en(false);
        triggers.set_ext0(true);

        // TODO: disable clock when not in use
        SENS::regs()
            .sar_io_mux_conf()
            .modify(|_, w| w.iomux_clk_gate_en().set_bit());

        // set pin to RTC function
        self.pin
            .borrow_mut()
            .rtc_set_config(true, true, RtcFunction::Rtc);

        // rtcio_hal_ext0_set_wakeup_pin
        unsafe {
            let rtc_io = RTC_IO::regs();
            // set pin register field
            rtc_io
                .ext_wakeup0()
                .modify(|_, w| w.sel().bits(self.pin.borrow().rtc_number()));
            // set level register field
            let rtc_cntl = LPWR::regs();
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
        // Checked TODO: Remove comment
        sleep_config.set_rtc_peri_pd_en(false);
        triggers.set_ext1(true);

        // TODO: disable clock when not in use
        SENS::regs()
            .sar_io_mux_conf()
            .modify(|_, w| w.iomux_clk_gate_en().set_bit());

        // set pins to RTC function
        let mut pins = self.pins.borrow_mut();
        let mut bits = 0u32;
        for pin in pins.iter_mut() {
            pin.rtc_set_config(true, true, RtcFunction::Rtc);
            bits |= 1 << pin.rtc_number();
        }

        unsafe {
            let rtc_cntl = LPWR::regs();
            // clear previous wakeup status
            rtc_cntl
                .ext_wakeup1()
                .modify(|_, w| w.status_clr().set_bit());
            // set pin register field
            rtc_cntl.ext_wakeup1().modify(|_, w| w.sel().bits(bits));
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

impl RtcioWakeupSource<'_, '_> {
    fn apply_pin(&self, pin: &mut dyn RtcPin, level: WakeupLevel) {
        let rtcio = RTC_IO::regs();

        pin.rtc_set_config(true, true, RtcFunction::Rtc);

        rtcio.pin(pin.number() as usize).modify(|_, w| unsafe {
            w.gpio_pin_wakeup_enable().set_bit();
            w.gpio_pin_int_type().bits(match level {
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
        let sens = crate::peripherals::SENS::regs();

        // TODO: disable clock when not in use
        sens.sar_io_mux_conf()
            .modify(|_, w| w.iomux_clk_gate_en().set_bit());

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
    /// power down Wifi
    pub wifi_pd_en, set_wifi_pd_en: 5;
    /// Power down Internal 8M oscillator
    pub int_8m_pd_en, set_int_8m_pd_en: 6;
    /// power down digital domain
    pub deep_slp, set_deep_slp: 8;
    /// enable WDT flashboot mode
    pub wdt_flashboot_mod_en, set_wdt_flashboot_mod_en: 9;
    /// set bias for digital domain, in sleep mode
    pub u8, dig_dbias_slp, set_dig_dbias_slp: 12, 10;
    /// set bias for RTC domain, in sleep mode
    pub u8, rtc_dbias_slp, set_rtc_dbias_slp: 16, 13;
    /// circuit control parameter, in monitor mode
    pub bias_sleep_monitor, set_bias_sleep_monitor: 17;
    /// voltage parameter, in sleep mode
    pub u8, dbg_atten_slp, set_dbg_atten_slp: 22, 18;
    /// circuit control parameter, in sleep mode
    pub bias_sleep_slp, set_bias_sleep_slp: 23;
    /// circuit control parameter, in monitor mode
    pub pd_cur_monitor, set_pd_cur_monitor: 24;
    /// circuit control parameter, in sleep mode
    pub pd_cur_slp, set_pd_cur_slp: 25;
    /// power down VDDSDIO regulator
    pub vddsdio_pd_en, set_vddsdio_pd_en: 26;
    /// keep main XTAL powered up in sleep
    pub xtal_fpu, set_xtal_fpu: 27;
    /// keep rtc regulator powered up in sleep
    pub rtc_regulator_fpu, set_rtc_regulator_fpu: 28;
    /// enable deep sleep reject
    pub deep_slp_reject, set_deep_slp_reject: 29;
    /// enable light sleep reject
    pub light_slp_reject, set_light_slp_reject: 30;
}

impl Default for RtcSleepConfig {
    fn default() -> Self {
        let mut cfg = Self(Default::default());
        cfg.set_deep_slp_reject(true);
        cfg.set_light_slp_reject(true);
        cfg.set_rtc_dbias_slp(RTC_CNTL_DBIAS_1V10);
        cfg.set_dig_dbias_slp(RTC_CNTL_DBIAS_1V10);
        cfg.set_rtc_slowmem_pd_en(true);
        cfg.set_rtc_fastmem_pd_en(true);
        cfg
    }
}

fn rtc_sleep_pu(val: bool) {
    // Called rtc_sleep_pd in idf, but makes more sense like this with the single
    // boolean argument Checked: OK TODO: Remove comment
    let rtc_cntl = LPWR::regs();
    let syscon = unsafe { &*esp32s2::SYSCON::ptr() };
    let bb = unsafe { &*esp32s2::BB::ptr() };
    let i2s = unsafe { &*esp32s2::I2S0::ptr() };
    let nrx = unsafe { &*esp32s2::NRX::ptr() };
    let fe = unsafe { &*esp32s2::FE::ptr() };
    let fe2 = unsafe { &*esp32s2::FE2::ptr() };

    rtc_cntl
        .dig_pwc()
        .modify(|_, w| w.lslp_mem_force_pu().bit(val));

    rtc_cntl
        .pwc()
        .modify(|_, w| w.slowmem_force_lpu().bit(val).fastmem_force_lpu().bit(val));

    i2s.pd_conf()
        .write(|w| w.plc_mem_force_pu().bit(val).fifo_force_pu().bit(val));

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
        cfg.set_wifi_pd_en(true);
        cfg.set_int_8m_pd_en(true);

        // Because of force_flags
        cfg.set_vddsdio_pd_en(true);

        // because of dig_peri_pd_en
        cfg.set_dig_dbias_slp(0);

        cfg.set_deep_slp(true);
        cfg.set_wdt_flashboot_mod_en(false);
        cfg.set_vddsdio_pd_en(true);
        cfg.set_xtal_fpu(false);
        cfg.set_deep_slp_reject(true);
        cfg.set_light_slp_reject(true);

        // because of RTC_SLEEP_PD_DIG
        // NOTE: Might be the a different case for RTC_SLEEP_PD_DIG in
        // rtc_sleep_get_default_config
        cfg.set_rtc_regulator_fpu(false);
        cfg.set_dbg_atten_slp(RTC_CNTL_DBG_ATTEN_DEEPSLEEP_DEFAULT);
        cfg.set_rtc_dbias_slp(0);

        // because of xtal_fpu
        cfg.set_xtal_fpu(false);
        cfg.set_bias_sleep_monitor(true);
        cfg.set_pd_cur_monitor(true);
        cfg.set_bias_sleep_slp(true);
        cfg.set_pd_cur_slp(true);

        cfg
    }

    pub(crate) fn base_settings(_rtc: &Rtc<'_>) {
        // Checked TODO: Remove comment
        // settings derived from esp_clk_init -> rtc_init
        unsafe {
            let rtc_cntl = LPWR::regs();
            let extmem = EXTMEM::regs();
            let system = SYSTEM::regs();

            rtc_cntl
                .dig_pwc()
                .modify(|_, w| w.wifi_force_pd().clear_bit());
            rtc_cntl
                .dig_iso()
                .modify(|_, w| w.wifi_force_iso().clear_bit());

            rtc_cntl.ana_conf().modify(|_, w| w.pvtmon_pu().clear_bit());

            rtc_cntl.timer1().modify(|_, w| {
                w.pll_buf_wait().bits(RTC_CNTL_PLL_BUF_WAIT_DEFAULT);
                w.ck8m_wait().bits(RTC_CNTL_CK8M_WAIT_DEFAULT)
            });

            // idf: "Moved from rtc sleep to rtc init to save sleep function running time
            // set shortest possible sleep time limit"

            rtc_cntl
                .timer5()
                .modify(|_, w| w.min_slp_val().bits(RTC_CNTL_MIN_SLP_VAL_MIN));

            rtc_cntl.timer3().modify(|_, w| {
                // set wifi timer
                w.wifi_powerup_timer().bits(WIFI_POWERUP_CYCLES);
                w.wifi_wait_timer().bits(WIFI_WAIT_CYCLES)
            });

            rtc_cntl.timer4().modify(|_, w| {
                // set rtc peri timer
                w.powerup_timer().bits(RTC_POWERUP_CYCLES);
                w.wait_timer().bits(RTC_WAIT_CYCLES);
                // set digital wrap timer
                w.dg_wrap_powerup_timer().bits(DG_WRAP_POWERUP_CYCLES);
                w.dg_wrap_wait_timer().bits(DG_WRAP_WAIT_CYCLES)
            });

            rtc_cntl.timer5().modify(|_, w| {
                w.rtcmem_powerup_timer().bits(RTC_MEM_POWERUP_CYCLES);
                w.rtcmem_wait_timer().bits(RTC_MEM_WAIT_CYCLES)
            });

            rtc_cntl.bias_conf().modify(|_, w| {
                w.dec_heartbeat_width().set_bit();
                w.inc_heartbeat_period().set_bit()
            });

            // Reset RTC bias to default value (needed if waking up from deep sleep)
            rtc_cntl.reg().modify(|_, w| {
                w.dbias_wak().bits(RTC_CNTL_DBIAS_1V10);
                w.dbias_slp().bits(RTC_CNTL_DBIAS_1V10)
            });

            // Set the wait time to the default value.
            rtc_cntl.timer2().modify(|_, w| {
                w.ulpcp_touch_start_wait()
                    .bits(RTC_CNTL_ULPCP_TOUCH_START_WAIT_DEFAULT)
            });

            // TODO: Check all the if statements in idf here
            //
            //
            //
            // clkctl_init
            {
                // clear CMMU clock force on
                extmem
                    .pro_cache_mmu_power_ctrl()
                    .modify(|_, w| w.pro_cache_mmu_mem_force_on().clear_bit());

                // clear tag clock force on
                extmem
                    .pro_dcache_tag_power_ctrl()
                    .modify(|_, w| w.pro_dcache_tag_mem_force_on().clear_bit());

                extmem
                    .pro_icache_tag_power_ctrl()
                    .modify(|_, w| w.pro_icache_tag_mem_force_on().clear_bit());

                system.rom_ctrl_0().modify(|_, w| w.rom_fo().bits(0));
                system.sram_ctrl_0().modify(|_, w| w.sram_fo().bits(0));

                // clear register clock force on
                SPI0::regs()
                    .clock_gate()
                    .modify(|_, w| w.clk_en().clear_bit());
                SPI1::regs()
                    .clock_gate()
                    .modify(|_, w| w.clk_en().clear_bit());
            }

            // pwrctl_init
            {
                rtc_cntl
                    .clk_conf()
                    .modify(|_, w| w.ck8m_force_pu().clear_bit());

                rtc_cntl
                    .options0()
                    .modify(|_, w| w.xtl_force_pu().clear_bit());

                // CLEAR APLL close
                rtc_cntl.ana_conf().modify(|_, w| {
                    w.plla_force_pu().clear_bit();
                    w.plla_force_pd().set_bit()
                });

                // cancel bbpll force pu if setting no force power up
                rtc_cntl.options0().modify(|_, w| {
                    w.bbpll_force_pu().clear_bit();
                    w.bbpll_i2c_force_pu().clear_bit();
                    w.bb_i2c_force_pu().clear_bit()
                });

                // cancel RTC REG force PU

                rtc_cntl.pwc().modify(|_, w| w.force_pu().clear_bit());
                rtc_cntl.reg().modify(|_, w| {
                    w.regulator_force_pu().clear_bit();
                    w.dboost_force_pu().clear_bit()
                });

                rtc_cntl.pwc().modify(|_, w| {
                    w.slowmem_force_pu().clear_bit();
                    w.fastmem_force_pu().clear_bit();
                    w.slowmem_force_noiso().clear_bit();
                    w.fastmem_force_noiso().clear_bit()
                });

                rtc_cntl.reg().modify(|_, w| w.dboost_force_pd().set_bit());

                // cancel sar i2c pd force
                rtc_cntl
                    .ana_conf()
                    .modify(|_, w| w.sar_i2c_force_pd().clear_bit());
                // cancel digital pu force
                // NOTE: duplicate from idf
                rtc_cntl.pwc().modify(|_, w| {
                    w.slowmem_force_pu().clear_bit();
                    w.fastmem_force_pu().clear_bit()
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
                    w.dg_wrap_force_pu().clear_bit();
                    w.wifi_force_pu().clear_bit()
                });

                rtc_cntl.dig_iso().modify(|_, w| {
                    w.dg_wrap_force_noiso().clear_bit();
                    // NOTE: not present in idf.
                    w.dg_wrap_force_iso().clear_bit()
                });

                rtc_cntl.dig_iso().modify(|_, w| {
                    w.wifi_force_noiso().clear_bit();
                    // NOTE: not present in idf.
                    w.wifi_force_iso().clear_bit()
                });

                rtc_cntl.pwc().modify(|_, w| w.force_noiso().clear_bit());

                // cancel digital PADS force no iso
                system
                    .cpu_per_conf()
                    .modify(|_, w| w.cpu_wait_mode_force_on().clear_bit());

                // if DPORT_CPU_WAIT_MODE_FORCE_ON == 0,
                // the cpu clk will be closed when cpu enter WAITI mode
                rtc_cntl.dig_iso().modify(|_, w| {
                    w.dg_pad_force_unhold().clear_bit();
                    w.dg_pad_force_noiso().clear_bit()
                });
            }

            // force power down wifi and bt power domain
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
        // like esp-idf rtc_sleep_init() and deep_sleep_start()
        let rtc_cntl = LPWR::regs();

        if self.deep_slp() {
            // "Due to hardware limitations, on S2 the brownout detector
            // sometimes trigger during deep sleep to circumvent
            // this we disable the brownout detector before sleeping' - from
            // idf's deep_sleep_start()
            unsafe {
                // brownout_hal_config(brownlout_hal_config_t{0})
                rtc_cntl.brown_out().modify(|_, w| {
                    w.int_wait().bits(2);
                    w.close_flash_ena().clear_bit();
                    w.pd_rf_ena().clear_bit();
                    w.cnt_clr().set_bit()
                });
                rtc_cntl.brown_out().modify(|_, w| {
                    // Set followed by clear in idf
                    w.cnt_clr().clear_bit();
                    w.rst_wait().bits(0x3fff);
                    w.rst_ena().clear_bit();
                    w.brown_out2_ena().set_bit();
                    w.rst_sel().set_bit()
                });
                regi2c_write_mask!(I2C_BOD_REG, I2C_BOD_REG_THRESHOLD, 0);
                rtc_cntl.brown_out().modify(|_, w| w.ena().clear_bit());
                rtc_cntl.int_ena().modify(|_, w| w.brown_out().clear_bit());
                // NOTE: rtc_isr_deregister?
            }
        }

        if self.lslp_mem_inf_fpu() {
            rtc_sleep_pu(true);
        }

        let mem_folw_cpu = self.rtc_mem_inf_follow_cpu();
        rtc_cntl.pwc().modify(|_, w| {
            w.slowmem_folw_cpu().bit(mem_folw_cpu);
            w.fastmem_folw_cpu().bit(mem_folw_cpu)
        });

        let rtc_fastmem_pd_en = self.rtc_fastmem_pd_en();
        rtc_cntl.pwc().modify(|_, w| {
            w.fastmem_pd_en().bit(rtc_fastmem_pd_en);
            w.fastmem_force_pu().bit(!rtc_fastmem_pd_en);
            w.fastmem_force_noiso().bit(!rtc_fastmem_pd_en)
        });

        let rtc_slowmem_pd_en = self.rtc_slowmem_pd_en();
        rtc_cntl.pwc().modify(|_, w| {
            w.slowmem_pd_en().bit(rtc_slowmem_pd_en);
            w.slowmem_force_pu().bit(!rtc_slowmem_pd_en);
            w.slowmem_force_noiso().bit(!rtc_slowmem_pd_en)
        });

        let rtc_peri_pd_en = self.rtc_peri_pd_en();
        rtc_cntl.pwc().modify(|_, w| w.pd_en().bit(rtc_peri_pd_en));

        if self.wifi_pd_en() {
            rtc_cntl
                .dig_iso()
                .modify(|_, w| w.wifi_force_noiso().clear_bit());

            rtc_cntl.dig_pwc().modify(|_, w| {
                w.wifi_force_pu().clear_bit();
                w.wifi_pd_en().set_bit()
            });
        } else {
            rtc_cntl.dig_pwc().modify(|_, w| w.wifi_pd_en().clear_bit());
        }

        unsafe {
            rtc_cntl.reg().modify(|_, w| {
                w.dbias_slp().bits(self.rtc_dbias_slp());
                w.dig_reg_dbias_slp().bits(self.dig_dbias_slp())
            });

            rtc_cntl.bias_conf().modify(|_, w| {
                w.dbg_atten_monitor()
                    .bits(RTC_CNTL_DBG_ATTEN_MONITOR_DEFAULT);
                w.bias_sleep_monitor().bit(self.bias_sleep_monitor());
                w.bias_sleep_deep_slp().bit(self.bias_sleep_slp());
                w.pd_cur_monitor().bit(self.pd_cur_monitor());
                w.pd_cur_deep_slp().bit(self.pd_cur_slp());
                w.dbg_atten_deep_slp().bits(self.dbg_atten_slp())
            });

            if self.deep_slp() {
                rtc_cntl
                    .dig_pwc()
                    .modify(|_, w| w.dg_wrap_pd_en().set_bit());

                rtc_cntl.ana_conf().modify(|_, w| {
                    w.ckgen_i2c_pu().clear_bit();
                    w.pll_i2c_pu().clear_bit();
                    w.rfrx_pbus_pu().clear_bit();
                    w.txrf_i2c_pu().clear_bit()
                });

                rtc_cntl
                    .options0()
                    .modify(|_, w| w.bb_i2c_force_pu().clear_bit());
            } else {
                rtc_cntl
                    .dig_pwc()
                    .modify(|_, w| w.dg_wrap_pd_en().clear_bit());
            }

            let rtc_regulator_fpu = self.rtc_regulator_fpu();
            rtc_cntl
                .reg()
                .modify(|_, w| w.regulator_force_pu().bit(rtc_regulator_fpu));

            let int_8m_pd_en = self.int_8m_pd_en();
            rtc_cntl
                .clk_conf()
                .modify(|_, w| w.ck8m_force_pu().bit(!int_8m_pd_en));

            // enable VDDSDIO control by state machine
            rtc_cntl.sdio_conf().modify(|_, w| {
                w.sdio_force().clear_bit();
                w.sdio_reg_pd_en().bit(self.vddsdio_pd_en())
            });

            rtc_cntl.slp_reject_conf().modify(|_, w| {
                w.deep_slp_reject_en().bit(self.deep_slp_reject());
                w.light_slp_reject_en().bit(self.light_slp_reject())
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
        }
    }

    pub(crate) fn start_sleep(&self, wakeup_triggers: WakeTriggers) {
        // NOTE: esp32s2 deep sleep uses asm. Can this work without?
        // Note: OK. TODO: Remove comment
        // TODO: Add reject triggers
        unsafe {
            LPWR::regs()
                .reset_state()
                .modify(|_, w| w.procpu_stat_vector_sel().set_bit());

            // set bits for what can wake us up
            LPWR::regs()
                .wakeup_state()
                .modify(|_, w| w.wakeup_ena().bits(wakeup_triggers.0.into()));

            // WARN: slp_wakeup is not set in esp-idf
            LPWR::regs().state0().write(|w| {
                w.sleep_en().set_bit();
                w.slp_wakeup().set_bit()
            });
        }
    }

    pub(crate) fn finish_sleep(&self) {
        // OK Checked TODO: Remove comment
        // In deep sleep mode, we never get here
        unsafe {
            LPWR::regs().int_clr().write(|w| {
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

            LPWR::regs().timer2().modify(|_, w| {
                w.ulpcp_touch_start_wait()
                    .bits(RTC_CNTL_ULPCP_TOUCH_START_WAIT_DEFAULT)
            });
        }
    }
}
