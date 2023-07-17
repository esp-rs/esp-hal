use crate::Rtc;

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
    pub u8, dig_dbias_slp, set_dig_dbias_slp: 15, 11;
    /// set bias for RTC domain, in sleep mode
    pub u8, rtc_dbias_slp, set_rtc_dbias_slp: 20, 16;
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

impl RtcSleepConfig {
    pub fn deep() -> Self {
        let mut cfg = Self::default();
        cfg.set_deep_slp(true);
        cfg.set_dig_dbias_slp(RTC_CNTL_DBIAS_0V90);
        // cfg.set_rtc_dbias_slp(RTC_CNTL_DBIAS_0V90);
        cfg.set_vddsdio_pd_en(true);
        cfg.set_int_8m_pd_en(true);
        cfg.set_xtal_fpu(false);
        cfg.set_modem_pd_en(true);
        cfg.set_rtc_peri_pd_en(true);
        cfg.set_rtc_fastmem_pd_en(true);
        cfg.set_rtc_slowmem_pd_en(true);
        cfg
    }

    fn base_settings(&self, _rtc: &Rtc) {
        // settings derived from esp-idf after basic boot
        unsafe {
            let rtc_cntl = &*esp32s3::RTC_CNTL::ptr();
            #[rustfmt::skip]
            rtc_cntl.options0.modify(|_, w| w
                .bias_core_force_pu().clear_bit()
                .bias_core_folw_8m().set_bit()
                .bias_i2c_force_pu().clear_bit()
                .bias_i2c_folw_8m().set_bit()
                .bias_force_nosleep().clear_bit()
                .bias_sleep_folw_8m().set_bit()
                .xtl_force_pu().clear_bit()
            );
            #[rustfmt::skip]
            rtc_cntl.reg.modify(|_, w| w
                .force_pu().clear_bit()
                .dboost_force_pu().clear_bit()
                .dboost_force_pd().set_bit()
            );
            #[rustfmt::skip]
            rtc_cntl.pwc.modify(|_, w| w
                .slowmem_force_pu().clear_bit()
                .fastmem_force_pu().clear_bit()
                .force_noiso().clear_bit()
                .slowmem_force_noiso().clear_bit()
                .fastmem_force_noiso().clear_bit()
            );
            #[rustfmt::skip]
            rtc_cntl.dig_pwc.modify(|_, w| w
                .dg_wrap_force_pu().clear_bit()
                .wifi_force_pu().clear_bit()
                .wifi_force_pd().set_bit()
                .inter_ram4_force_pu().clear_bit()
                .inter_ram3_force_pu().clear_bit()
                .inter_ram2_force_pu().clear_bit()
                .inter_ram1_force_pu().clear_bit()
                .inter_ram0_force_pu().clear_bit()
                .rom0_force_pu().clear_bit()
                .lslp_mem_force_pu().clear_bit()
            );
            #[rustfmt::skip]
            rtc_cntl.dig_iso.modify(|_, w| w
                .dg_wrap_force_noiso().clear_bit()
                .wifi_force_noiso().clear_bit()
                .wifi_force_iso().set_bit()
                .inter_ram4_force_noiso().clear_bit()
                .inter_ram3_force_noiso().clear_bit()
                .inter_ram2_force_noiso().clear_bit()
                .inter_ram1_force_noiso().clear_bit()
                .inter_ram0_force_noiso().clear_bit()
                .rom0_force_noiso().clear_bit()
                .dg_pad_force_unhold().clear_bit()
                .dg_pad_force_noiso().clear_bit()
            );
            #[rustfmt::skip]
            rtc_cntl.int_ena.modify(|_, w| w
                .brown_out_int_ena().set_bit()
            );
        }
    }

    pub(crate) fn apply(&self, rtc: &Rtc) {
        self.base_settings(rtc);
        // like esp-idf rtc_sleep_init()
        unsafe {
            let rtc_cntl = &*esp32s3::RTC_CNTL::ptr();

            #[rustfmt::skip]
            rtc_cntl.timer5.modify(|_, w| w
                .min_slp_val().bits(RTC_CNTL_MIN_SLP_VAL_MIN)
                // set rtc memory timer
                .rtcmem_powerup_timer().bits(RTC_MEM_POWERUP_CYCLES)
                .rtcmem_wait_timer().bits(RTC_MEM_WAIT_CYCLES)
            );

            #[rustfmt::skip]
            rtc_cntl.timer3.modify(|_, w| w
                // set rom&ram timer
                .rom_ram_powerup_timer().bits(ROM_RAM_POWERUP_CYCLES)
                .rom_ram_wait_timer().bits(ROM_RAM_WAIT_CYCLES)
                // set wifi timer
                .wifi_powerup_timer().bits(WIFI_POWERUP_CYCLES)
                .wifi_wait_timer().bits(WIFI_WAIT_CYCLES)
            );

            #[rustfmt::skip]
            rtc_cntl.timer4.modify(|_, w| w
                // set rtc peri timer
                .powerup_timer().bits(RTC_POWERUP_CYCLES)
                .wait_timer().bits(RTC_WAIT_CYCLES)
                // set digital wrap timer
                .dg_wrap_powerup_timer().bits(DG_WRAP_POWERUP_CYCLES)
                .dg_wrap_wait_timer().bits(DG_WRAP_WAIT_CYCLES)
            );

            #[rustfmt::skip]
            rtc_cntl.dig_pwc.modify(|_, w| w
                .lslp_mem_force_pu().bit(self.lslp_mem_inf_fpu())
            );

            // remove all peripheral force power up flags
            if self.lslp_meminf_pd() {
                #[rustfmt::skip]
                rtc_cntl.dig_pwc.modify(|_, w| w
                    .lslp_mem_force_pu().clear_bit()
                );

                #[rustfmt::skip]
                rtc_cntl.pwc.modify(|_, w| w
                    .slowmem_force_pu().clear_bit()
                    .fastmem_force_pu().clear_bit()
                );

                // esp-idf also clears these:
                #[rustfmt::skip]
                (&*esp32::DPORT::ptr()).mem_pd_mask.modify(|_, w| w
                    .lslp_mem_pd_mask().clear_bit()
                );
                #[rustfmt::skip]
                (&*esp32::I2S0::ptr()).pd_conf.modify(|_, w| w
                    .plc_mem_force_pu().clear_bit()
                    .fifo_force_pu().clear_bit()
                );
                #[rustfmt::skip]
                (&*esp32::BB::ptr()).bbpd_ctrl.modify(|_, w| w
                    .fft_force_pu().clear_bit()
                    .dc_est_force_pu().clear_bit()
                );
                #[rustfmt::skip]
                (&*esp32::NRX::ptr()).nrxpd_ctrl.modify(|_, w| w
                    .rx_rot_force_pu().clear_bit()
                    .vit_force_pu().clear_bit()
                    .demap_force_pu().clear_bit()
                );
                // #[rustfmt::skip]
                // (&*esp32::FE::ptr()).gen_ctrl.modify(|_, w| w
                //     .iq_est_force_pu().clear_bit()
                // );
                // #[rustfmt::skip]
                // (&*esp32::FE2::ptr()).tx_interp_ctrl.modify(|_, w| w
                //     .inf_force_pu().clear_bit()
                // );
            }

            #[rustfmt::skip]
            rtc_cntl.pwc.modify(|_, w| w
                .slowmem_folw_cpu().bit(self.rtc_mem_inf_follow_cpu())
                .fastmem_folw_cpu().bit(self.rtc_mem_inf_follow_cpu())
                // TODO: does this need to be optional based on if there is something stored in fastmem?
                //.fastmem_pd_en().bit(self.rtc_fastmem_pd_en())
                .fastmem_force_pu().bit(!self.rtc_fastmem_pd_en())
                .fastmem_force_lpu().bit(!self.rtc_fastmem_pd_en())
                .fastmem_force_noiso().bit(!self.rtc_fastmem_pd_en())
                .slowmem_pd_en().bit(self.rtc_slowmem_pd_en())
                .slowmem_force_pu().bit(!self.rtc_slowmem_pd_en())
                .slowmem_force_noiso().bit(!self.rtc_slowmem_pd_en())
                .slowmem_force_lpu().bit(!self.rtc_slowmem_pd_en())
                .pd_en().bit(self.rtc_peri_pd_en())
            );

            // #[rustfmt::skip]
            // rtc_cntl.dig_pwc.modify(|_, w| w
            //     .wifi_pd_en().bit(self.wifi_pd_en())
            //     .rom0_pd_en().bit(self.rom_mem_pd_en())
            // );

            if self.deep_slp() {
                #[rustfmt::skip]
                rtc_cntl.dig_iso.modify(|_, w| w
                    .dg_wrap_force_noiso().clear_bit()
                    .wifi_force_noiso().clear_bit()
                    .dg_pad_force_iso().clear_bit()
                    .dg_pad_force_noiso().clear_bit()
                );
                #[rustfmt::skip]
                rtc_cntl.dig_pwc.modify(|_, w| w
                    .dg_wrap_pd_en().set_bit()
                    .dg_wrap_force_pu().clear_bit()
                    .dg_wrap_force_pd().clear_bit()
                );
                #[rustfmt::skip]
                rtc_cntl.options0.modify(|_, w| w
                    .bias_force_nosleep().clear_bit()
                    .bb_i2c_force_pu().clear_bit()
                );
                #[rustfmt::skip]
                rtc_cntl.ana_conf.modify(|_, w| w
                    .ckgen_i2c_pu().clear_bit()
                    .pll_i2c_pu().clear_bit()
                    .rfrx_pbus_pu().clear_bit()
                    .txrf_i2c_pu().clear_bit()
                );
            } else {
                #[rustfmt::skip]
                rtc_cntl.dig_pwc.modify(|_, w| w
                    .dg_wrap_pd_en().clear_bit()
                );
                #[rustfmt::skip]
                rtc_cntl.bias_conf.modify(|_, w| w
                    .dbg_atten().bits(0)
                );
            }

            #[rustfmt::skip]
            rtc_cntl.options0.modify(|_, w| w
                .xtl_force_pu().bit(self.xtal_fpu())
            );

            #[rustfmt::skip]
            rtc_cntl.clk_conf.modify(|_, w| w
                .ck8m_force_pu().bit(!self.int_8m_pd_en())
            );

            // enable VDDSDIO control by state machine
            #[rustfmt::skip]
            rtc_cntl.sdio_conf.modify(|_, w| w
                .sdio_force().clear_bit()
                .sdio_pd_en().bit(self.vddsdio_pd_en())
            );

            #[rustfmt::skip]
            rtc_cntl.reg.modify(|_, w| w
                .dbias_slp().bits(self.rtc_dbias_slp())
                .dbias_wak().bits(self.rtc_dbias_wak())
                .dig_dbias_slp().bits(self.dig_dbias_slp())
                .dig_dbias_wak().bits(self.dig_dbias_wak())
            );

            #[rustfmt::skip]
            rtc_cntl.slp_reject_conf.modify(|_, w| w
                .deep_slp_reject_en().bit(self.deep_slp_reject())
                .light_slp_reject_en().bit(self.light_slp_reject())
            );
        }
    }
}
