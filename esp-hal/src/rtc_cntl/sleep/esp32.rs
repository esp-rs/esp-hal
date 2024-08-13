use super::{Ext0WakeupSource, Ext1WakeupSource, TimerWakeupSource, WakeSource, WakeTriggers};
use crate::{
    gpio::{RtcFunction, RtcPin},
    rtc_cntl::{sleep::WakeupLevel, Clock, Rtc, RtcClock},
};

// Approximate mapping of voltages to RTC_CNTL_DBIAS_WAK, RTC_CNTL_DBIAS_SLP,
// RTC_CNTL_DIG_DBIAS_WAK, RTC_CNTL_DIG_DBIAS_SLP values.
// Valid if RTC_CNTL_DBG_ATTEN is 0.
pub const RTC_CNTL_DBIAS_0V90: u8 = 0;
pub const RTC_CNTL_DBIAS_0V95: u8 = 1;
pub const RTC_CNTL_DBIAS_1V00: u8 = 2;
pub const RTC_CNTL_DBIAS_1V05: u8 = 3;
pub const RTC_CNTL_DBIAS_1V10: u8 = 4;
pub const RTC_CNTL_DBIAS_1V15: u8 = 5;
pub const RTC_CNTL_DBIAS_1V20: u8 = 6;
pub const RTC_CNTL_DBIAS_1V25: u8 = 7;
// Various delays to be programmed into power control state machines
pub const RTC_CNTL_XTL_BUF_WAIT_SLP_US: u32 = 1000;
pub const RTC_CNTL_PLL_BUF_WAIT_SLP_CYCLES: u8 = 1;
pub const RTC_CNTL_CK8M_WAIT_SLP_CYCLES: u8 = 4;
pub const RTC_CNTL_WAKEUP_DELAY_CYCLES: u8 = 7;
pub const RTC_CNTL_OTHER_BLOCKS_POWERUP_CYCLES: u8 = 1;
pub const RTC_CNTL_OTHER_BLOCKS_WAIT_CYCLES: u16 = 1;
pub const RTC_CNTL_MIN_SLP_VAL_MIN: u8 = 128;
pub const RTC_CNTL_DBG_ATTEN_DEFAULT: u8 = 3;

pub const RTC_MEM_POWERUP_CYCLES: u8 = RTC_CNTL_OTHER_BLOCKS_POWERUP_CYCLES;
pub const RTC_MEM_WAIT_CYCLES: u16 = RTC_CNTL_OTHER_BLOCKS_WAIT_CYCLES;
pub const ROM_RAM_POWERUP_CYCLES: u8 = RTC_CNTL_OTHER_BLOCKS_POWERUP_CYCLES;
pub const ROM_RAM_WAIT_CYCLES: u16 = RTC_CNTL_OTHER_BLOCKS_WAIT_CYCLES;
pub const WIFI_POWERUP_CYCLES: u8 = RTC_CNTL_OTHER_BLOCKS_POWERUP_CYCLES;
pub const WIFI_WAIT_CYCLES: u16 = RTC_CNTL_OTHER_BLOCKS_WAIT_CYCLES;
pub const RTC_POWERUP_CYCLES: u8 = RTC_CNTL_OTHER_BLOCKS_POWERUP_CYCLES;
pub const RTC_WAIT_CYCLES: u16 = RTC_CNTL_OTHER_BLOCKS_WAIT_CYCLES;
pub const DG_WRAP_POWERUP_CYCLES: u8 = RTC_CNTL_OTHER_BLOCKS_POWERUP_CYCLES;
pub const DG_WRAP_WAIT_CYCLES: u16 = RTC_CNTL_OTHER_BLOCKS_WAIT_CYCLES;

pub const RTC_CNTL_CK8M_WAIT_DEFAULT: u8 = 20;
pub const RTC_CK8M_ENABLE_WAIT_DEFAULT: u8 = 5;

impl WakeSource for TimerWakeupSource {
    fn apply(
        &self,
        rtc: &Rtc<'_>,
        triggers: &mut WakeTriggers,
        _sleep_config: &mut RtcSleepConfig,
    ) {
        triggers.set_timer(true);
        let rtc_cntl = unsafe { &*esp32::RTC_CNTL::ptr() };
        let clock_freq = RtcClock::get_slow_freq();
        // TODO: maybe add sleep time adjustlemnt like idf
        // TODO: maybe add check to prevent overflow?
        let clock_hz = clock_freq.frequency().to_Hz() as u64;
        let ticks = self.duration.as_micros() as u64 * clock_hz / 1_000_000u64;
        // "alarm" time in slow rtc ticks
        let now = rtc.get_rtc_time_raw();
        let time_in_ticks = now + ticks;
        unsafe {
            rtc_cntl
                .slp_timer0()
                .write(|w| w.slp_val_lo().bits((time_in_ticks & 0xffffffff) as u32));

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
            let rtc_io = &*esp32::RTC_IO::ptr();
            // set pin register field
            rtc_io
                .ext_wakeup0()
                .modify(|_, w| w.sel().bits(self.pin.borrow().rtc_number()));
            // set level register field
            let rtc_cntl = &*esp32::RTC_CNTL::ptr();
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
            let rtc_cntl = &*esp32::RTC_CNTL::ptr();
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

bitfield::bitfield! {
    #[derive(Clone, Copy)]
    pub struct RtcSleepConfig(u32);
    impl Debug;
    /// force normal voltage in sleep mode (digital domain memory)
    pub lslp_mem_inf_fpu, set_lslp_mem_inf_fpu: 0;
    /// force normal voltage in sleep mode (RTC memory)
    pub rtc_mem_inf_fpu, set_rtc_mem_inf_fpu: 1;
    /// keep low voltage in sleep mode (even if ULP/touch is used)
    pub rtc_mem_inf_follow_cpu, set_rtc_mem_inf_follow_cpu: 2;
    /// power down RTC fast memory
    pub rtc_fastmem_pd_en, set_rtc_fastmem_pd_en: 3;
    /// power down RTC slow memory
    pub rtc_slowmem_pd_en, set_rtc_slowmem_pd_en: 4;
    /// power down RTC peripherals
    pub rtc_peri_pd_en, set_rtc_peri_pd_en: 5;
    /// power down WiFi
    pub wifi_pd_en, set_wifi_pd_en: 6;
    /// Power down Internal 8M oscillator
    pub int_8m_pd_en, set_int_8m_pd_en: 7;
    /// power down main RAM and ROM
    pub rom_mem_pd_en, set_rom_mem_pd_en: 8;
    /// power down digital domain
    pub deep_slp, set_deep_slp: 9;
    /// enable WDT flashboot mode
    pub wdt_flashboot_mod_en, set_wdt_flashboot_mod_en: 10;
    /// bias for digital domain, in active mode
    pub u8, dig_dbias_wak, set_dig_dbias_wak: 13, 11;
    /// bias for digital domain, in sleep mode
    pub u8, dig_dbias_slp, set_dig_dbias_slp: 16, 14;
    /// bias for RTC domain, in active mode
    pub u8, rtc_dbias_wak, set_rtc_dbias_wak: 19, 17;
    /// bias for RTC domain, in sleep mode
    pub u8, rtc_dbias_slp, set_rtc_dbias_slp: 22, 20;
    /// remove all peripheral force power up flags
    pub lslp_meminf_pd, set_lslp_meminf_pd: 23;
    /// power down VDDSDIO regulator
    pub vddsdio_pd_en, set_vddsdio_pd_en: 24;
    /// keep main XTAL powered up in sleep
    pub xtal_fpu, set_xtal_fpu: 25;
    /// enable deep sleep reject
    pub deep_slp_reject, set_deep_slp_reject: 26;
    /// enable light sleep reject
    pub light_slp_reject, set_light_slp_reject: 27;
}

impl Default for RtcSleepConfig {
    fn default() -> Self {
        let mut cfg = Self(Default::default());
        cfg.set_lslp_meminf_pd(true);
        cfg.set_deep_slp_reject(true);
        cfg.set_light_slp_reject(true);
        cfg.set_dig_dbias_wak(RTC_CNTL_DBIAS_1V10);
        cfg.set_dig_dbias_slp(RTC_CNTL_DBIAS_1V10);
        cfg.set_rtc_dbias_wak(RTC_CNTL_DBIAS_1V10);
        cfg.set_rtc_dbias_slp(RTC_CNTL_DBIAS_1V10);
        cfg
    }
}

impl RtcSleepConfig {
    pub fn deep() -> Self {
        let mut cfg = Self::default();
        cfg.set_deep_slp(true);
        cfg.set_dig_dbias_slp(RTC_CNTL_DBIAS_0V90);
        // cfg.set_rtc_dbias_slp(RTC_CNTL_DBIAS_0V90);
        cfg.set_vddsdio_pd_en(true);
        cfg.set_int_8m_pd_en(true);
        cfg.set_xtal_fpu(false);
        cfg.set_wifi_pd_en(true);
        cfg.set_rom_mem_pd_en(true);
        cfg.set_rtc_peri_pd_en(true);
        cfg.set_rtc_fastmem_pd_en(true);
        cfg.set_rtc_slowmem_pd_en(true);
        cfg
    }

    pub(crate) fn base_settings(_rtc: &Rtc<'_>) {
        // settings derived from esp-idf after basic boot
        unsafe {
            let rtc_cntl = &*esp32::RTC_CNTL::ptr();

            rtc_cntl.options0().modify(|_, w| {
                w.bias_core_force_pu()
                    .clear_bit()
                    .bias_core_folw_8m()
                    .set_bit()
                    .bias_i2c_force_pu()
                    .clear_bit()
                    .bias_i2c_folw_8m()
                    .set_bit()
                    .bias_force_nosleep()
                    .clear_bit()
                    .bias_sleep_folw_8m()
                    .set_bit()
                    .xtl_force_pu()
                    .clear_bit()
            });

            rtc_cntl.reg().modify(|_, w| {
                w.force_pu()
                    .clear_bit()
                    .dboost_force_pu()
                    .clear_bit()
                    .dboost_force_pd()
                    .set_bit()
            });

            rtc_cntl.pwc().modify(|_, w| {
                w.slowmem_force_pu()
                    .clear_bit()
                    .fastmem_force_pu()
                    .clear_bit()
                    .force_noiso()
                    .clear_bit()
                    .slowmem_force_noiso()
                    .clear_bit()
                    .fastmem_force_noiso()
                    .clear_bit()
            });

            rtc_cntl.dig_pwc().modify(|_, w| {
                w.dg_wrap_force_pu()
                    .clear_bit()
                    .wifi_force_pu()
                    .clear_bit()
                    .wifi_force_pd()
                    .set_bit()
                    .inter_ram4_force_pu()
                    .clear_bit()
                    .inter_ram3_force_pu()
                    .clear_bit()
                    .inter_ram2_force_pu()
                    .clear_bit()
                    .inter_ram1_force_pu()
                    .clear_bit()
                    .inter_ram0_force_pu()
                    .clear_bit()
                    .rom0_force_pu()
                    .clear_bit()
                    .lslp_mem_force_pu()
                    .clear_bit()
            });

            rtc_cntl.dig_iso().modify(|_, w| {
                w.dg_wrap_force_noiso()
                    .clear_bit()
                    .wifi_force_noiso()
                    .clear_bit()
                    .wifi_force_iso()
                    .set_bit()
                    .inter_ram4_force_noiso()
                    .clear_bit()
                    .inter_ram3_force_noiso()
                    .clear_bit()
                    .inter_ram2_force_noiso()
                    .clear_bit()
                    .inter_ram1_force_noiso()
                    .clear_bit()
                    .inter_ram0_force_noiso()
                    .clear_bit()
                    .rom0_force_noiso()
                    .clear_bit()
                    .dg_pad_force_unhold()
                    .clear_bit()
                    .dg_pad_force_noiso()
                    .clear_bit()
            });

            rtc_cntl.int_ena().modify(|_, w| w.brown_out().set_bit());
        }
    }

    pub(crate) fn apply(&self) {
        // like esp-idf rtc_sleep_init()
        unsafe {
            let rtc_cntl = &*esp32::RTC_CNTL::ptr();

            rtc_cntl.timer5().modify(|_, w| {
                w.min_slp_val()
                    .bits(RTC_CNTL_MIN_SLP_VAL_MIN)
                    // set rtc memory timer
                    .rtcmem_powerup_timer()
                    .bits(RTC_MEM_POWERUP_CYCLES)
                    .rtcmem_wait_timer()
                    .bits(RTC_MEM_WAIT_CYCLES)
            });

            rtc_cntl.timer3().modify(|_, w| {
                w
                    // set rom&ram timer
                    .rom_ram_powerup_timer()
                    .bits(ROM_RAM_POWERUP_CYCLES)
                    .rom_ram_wait_timer()
                    .bits(ROM_RAM_WAIT_CYCLES)
                    // set wifi timer
                    .wifi_powerup_timer()
                    .bits(WIFI_POWERUP_CYCLES)
                    .wifi_wait_timer()
                    .bits(WIFI_WAIT_CYCLES)
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

            rtc_cntl
                .dig_pwc()
                .modify(|_, w| w.lslp_mem_force_pu().bit(self.lslp_mem_inf_fpu()));

            // remove all peripheral force power up flags
            if self.lslp_meminf_pd() {
                rtc_cntl
                    .dig_pwc()
                    .modify(|_, w| w.lslp_mem_force_pu().clear_bit());

                rtc_cntl.pwc().modify(|_, w| {
                    w.slowmem_force_pu()
                        .clear_bit()
                        .fastmem_force_pu()
                        .clear_bit()
                });

                // esp-idf also clears these:

                (*esp32::DPORT::ptr())
                    .mem_pd_mask()
                    .modify(|_, w| w.lslp_mem_pd_mask().clear_bit());

                (*esp32::I2S0::ptr())
                    .pd_conf()
                    .modify(|_, w| w.plc_mem_force_pu().clear_bit().fifo_force_pu().clear_bit());

                (*esp32::BB::ptr())
                    .bbpd_ctrl()
                    .modify(|_, w| w.fft_force_pu().clear_bit().dc_est_force_pu().clear_bit());

                (*esp32::NRX::ptr()).nrxpd_ctrl().modify(|_, w| {
                    w.rx_rot_force_pu()
                        .clear_bit()
                        .vit_force_pu()
                        .clear_bit()
                        .demap_force_pu()
                        .clear_bit()
                });
                // (&*esp32::FE::ptr()).gen_ctrl.modify(|_, w| w
                //     .iq_est_force_pu().clear_bit()
                // );
                //
                // (&*esp32::FE2::ptr()).tx_interp_ctrl.modify(|_, w| w
                //     .inf_force_pu().clear_bit()
                // );
            }

            rtc_cntl.pwc().modify(|_, w| {
                w.slowmem_folw_cpu()
                    .bit(self.rtc_mem_inf_follow_cpu())
                    .fastmem_folw_cpu()
                    .bit(self.rtc_mem_inf_follow_cpu())
                    // TODO: does this need to be optional based on if there is something stored in
                    // fastmem?
                    //.fastmem_pd_en().bit(self.rtc_fastmem_pd_en())
                    .fastmem_force_pu()
                    .bit(!self.rtc_fastmem_pd_en())
                    .fastmem_force_lpu()
                    .bit(!self.rtc_fastmem_pd_en())
                    .fastmem_force_noiso()
                    .bit(!self.rtc_fastmem_pd_en())
                    .slowmem_pd_en()
                    .bit(self.rtc_slowmem_pd_en())
                    .slowmem_force_pu()
                    .bit(!self.rtc_slowmem_pd_en())
                    .slowmem_force_noiso()
                    .bit(!self.rtc_slowmem_pd_en())
                    .slowmem_force_lpu()
                    .bit(!self.rtc_slowmem_pd_en())
                    .pd_en()
                    .bit(self.rtc_peri_pd_en())
            });

            // rtc_cntl.dig_pwc.modify(|_, w| w
            //     .wifi_pd_en().bit(self.wifi_pd_en())
            //     .rom0_pd_en().bit(self.rom_mem_pd_en())
            // );

            if self.deep_slp() {
                rtc_cntl.dig_iso().modify(|_, w| {
                    w.dg_wrap_force_noiso()
                        .clear_bit()
                        .wifi_force_noiso()
                        .clear_bit()
                        .dg_pad_force_iso()
                        .clear_bit()
                        .dg_pad_force_noiso()
                        .clear_bit()
                });

                rtc_cntl.dig_pwc().modify(|_, w| {
                    w.dg_wrap_pd_en()
                        .set_bit()
                        .dg_wrap_force_pu()
                        .clear_bit()
                        .dg_wrap_force_pd()
                        .clear_bit()
                });

                rtc_cntl.options0().modify(|_, w| {
                    w.bias_force_nosleep()
                        .clear_bit()
                        .bb_i2c_force_pu()
                        .clear_bit()
                });

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
            } else {
                rtc_cntl
                    .dig_pwc()
                    .modify(|_, w| w.dg_wrap_pd_en().clear_bit());

                rtc_cntl.bias_conf().modify(|_, w| w.dbg_atten().bits(0));
            }

            rtc_cntl
                .options0()
                .modify(|_, w| w.xtl_force_pu().bit(self.xtal_fpu()));

            rtc_cntl
                .clk_conf()
                .modify(|_, w| w.ck8m_force_pu().bit(!self.int_8m_pd_en()));

            // enable VDDSDIO control by state machine

            rtc_cntl.sdio_conf().modify(|_, w| {
                w.sdio_force()
                    .clear_bit()
                    .sdio_pd_en()
                    .bit(self.vddsdio_pd_en())
            });

            rtc_cntl.reg().modify(|_, w| {
                w.dbias_slp()
                    .bits(self.rtc_dbias_slp())
                    .dbias_wak()
                    .bits(self.rtc_dbias_wak())
                    .dig_dbias_slp()
                    .bits(self.dig_dbias_slp())
                    .dig_dbias_wak()
                    .bits(self.dig_dbias_wak())
            });

            rtc_cntl.slp_reject_conf().modify(|_, w| {
                w.deep_slp_reject_en()
                    .bit(self.deep_slp_reject())
                    .light_slp_reject_en()
                    .bit(self.light_slp_reject())
            });
        }
    }

    pub(crate) fn start_sleep(&self, wakeup_triggers: WakeTriggers) {
        unsafe {
            let rtc_cntl = &*esp32::RTC_CNTL::ptr();

            rtc_cntl
                .reset_state()
                .modify(|_, w| w.procpu_stat_vector_sel().set_bit());

            // set bits for what can wake us up
            rtc_cntl
                .wakeup_state()
                .modify(|_, w| w.wakeup_ena().bits(wakeup_triggers.0));

            rtc_cntl
                .state0()
                .write(|w| w.sleep_en().set_bit().slp_wakeup().set_bit());
        }
    }

    pub(crate) fn finish_sleep(&self) {
        // In deep sleep mode, we never get here
        unsafe {
            let rtc_cntl = &*esp32::RTC_CNTL::ptr();

            rtc_cntl.int_clr().write(|w| {
                w.slp_reject()
                    .clear_bit_by_one()
                    .slp_wakeup()
                    .clear_bit_by_one()
            });

            // restore DBG_ATTEN to the default value

            rtc_cntl
                .bias_conf()
                .modify(|_, w| w.dbg_atten().bits(RTC_CNTL_DBG_ATTEN_DEFAULT));
        }
    }
}
