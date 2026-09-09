use super::SleepKind;
use crate::{
    peripherals::{APB_CTRL, EXTMEM, LPWR, SPI0, SPI1, SYSTEM},
    rtc_cntl::{Rtc, cpu_retention},
    soc::regi2c,
};

// Approximate mapping of voltages to RTC_CNTL_DBIAS_WAK, RTC_CNTL_DBIAS_SLP,
// RTC_CNTL_DIG_DBIAS_WAK, RTC_CNTL_DIG_DBIAS_SLP values.
// Valid if RTC_CNTL_DBG_ATTEN is 0.
/// Digital bias setting for 0.90V.
pub const RTC_CNTL_DBIAS_0V90: u8 = 13;
/// Digital bias setting for 0.95V.
pub const RTC_CNTL_DBIAS_0V95: u8 = 16;
/// Digital bias setting for 1.00V.
pub const RTC_CNTL_DBIAS_1V00: u8 = 18;
/// Digital bias setting for 1.05V.
pub const RTC_CNTL_DBIAS_1V05: u8 = 20;
/// Digital bias setting for 1.10V.
pub const RTC_CNTL_DBIAS_1V10: u8 = 23;
/// Digital bias setting for 1.15V.
pub const RTC_CNTL_DBIAS_1V15: u8 = 25;
/// Digital bias setting for 1.20V.
pub const RTC_CNTL_DBIAS_1V20: u8 = 28;
/// Digital bias setting for 1.25V.
pub const RTC_CNTL_DBIAS_1V25: u8 = 30;
/// Digital bias setting for 1.30V. Voltage is approximately 1.34V in practice.
pub const RTC_CNTL_DBIAS_1V30: u8 = 31;
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
/// Waits cycles for other blocks.
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
    /// Powers down Internal 8M oscillator.
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

        // This is the light-sleep config. The main XTAL is powered down in sleep
        // (`xtal_fpu` stays false), so the analog regulator/bias must use the
        // same XTAL-down settings that `deep()` applies "because of xtal_fpu".
        cfg.set_rtc_regulator_fpu(true);
        cfg.set_bias_sleep_monitor(true);
        cfg.set_pd_cur_monitor(true);
        cfg.set_bias_sleep_slp(true);
        cfg.set_pd_cur_slp(true);
        cfg
    }
}

const SYSCON_SRAM_POWER_UP: u16 = 0x7FF;
const SYSCON_ROM_POWER_UP: u8 = 0x7;

fn rtc_sleep_pu(val: bool) {
    let rtc_cntl = LPWR::regs();
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
        w.dc_mem_force_pu().bit(val);
        w.pbus_mem_force_pu().bit(val);
        w.agc_mem_force_pu().bit(val)
    });

    bb.bbpd_ctrl()
        .modify(|_r, w| w.fft_force_pu().bit(val).dc_est_force_pu().bit(val));

    nrx.nrxpd_ctrl().modify(|_, w| {
        w.rx_rot_force_pu().bit(val);
        w.vit_force_pu().bit(val);
        w.demap_force_pu().bit(val)
    });

    fe.gen_ctrl().modify(|_, w| w.iq_est_force_pu().bit(val));

    fe2.tx_interp_ctrl()
        .modify(|_, w| w.tx_inf_force_pu().bit(val));

    syscon.mem_power_up().modify(|_r, w| unsafe {
        w.sram_power_up()
            .bits(if val { SYSCON_SRAM_POWER_UP } else { 0 });
        w.rom_power_up()
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

    pub(crate) fn is_deep_sleep(&self) -> bool {
        self.deep_slp()
    }

    pub(crate) fn set_sleep_kind(&mut self, kind: SleepKind) {
        self.set_deep_slp(kind == SleepKind::Deep);
    }

    pub(crate) fn base_settings(_rtc: &Rtc<'_>) {
        // settings derived from esp_clk_init -> rtc_init
        unsafe {
            let rtc_cntl = LPWR::regs();
            let syscon = APB_CTRL::regs();
            let extmem = EXTMEM::regs();
            let system = SYSTEM::regs();

            rtc_cntl
                .dig_pwc()
                .modify(|_, w| w.wifi_force_pd().clear_bit());

            regi2c::I2C_DIG_REG_XPD_RTC_REG.write_field(0);
            regi2c::I2C_DIG_REG_XPD_DIG_REG.write_field(0);

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
                // set wifi timer
                w.wifi_powerup_timer().bits(WIFI_POWERUP_CYCLES);
                // set bt timer
                w.wifi_wait_timer().bits(WIFI_WAIT_CYCLES);
                w.bt_powerup_timer().bits(BT_POWERUP_CYCLES);
                w.bt_wait_timer().bits(BT_WAIT_CYCLES)
            });

            rtc_cntl.timer6().modify(|_, w| {
                w.cpu_top_powerup_timer().bits(CPU_TOP_POWERUP_CYCLES);
                w.cpu_top_wait_timer().bits(CPU_TOP_WAIT_CYCLES)
            });

            rtc_cntl.timer4().modify(|_, w| {
                // set rtc peri timer
                w.powerup_timer().bits(RTC_POWERUP_CYCLES);
                // set digital wrap timer
                w.wait_timer().bits(RTC_WAIT_CYCLES);
                w.dg_wrap_powerup_timer().bits(DG_WRAP_POWERUP_CYCLES);
                w.dg_wrap_wait_timer().bits(DG_WRAP_WAIT_CYCLES)
            });

            rtc_cntl.timer6().modify(|_, w| {
                w.dg_peri_powerup_timer().bits(DG_PERI_POWERUP_CYCLES);
                w.dg_peri_wait_timer().bits(DG_PERI_WAIT_CYCLES)
            });

            // Reset RTC bias to default value (needed if waking up from deep sleep)
            regi2c::I2C_DIG_REG_EXT_RTC_DREG_SLEEP.write_field(RTC_CNTL_DBIAS_1V10);
            regi2c::I2C_DIG_REG_EXT_RTC_DREG.write_field(RTC_CNTL_DBIAS_1V10);

            // Set the wait time to the default value.

            rtc_cntl.timer2().modify(|_, w| {
                w.ulpcp_touch_start_wait()
                    .bits(RTC_CNTL_ULPCP_TOUCH_START_WAIT_DEFAULT)
            });

            // LDO dbias initialization
            // TODO: this modifies g_rtc_dbias_pvt_non_240m and g_dig_dbias_pvt_non_240m.
            //       We're using a high enough default but we should read from the efuse.
            // rtc_set_stored_dbias();

            regi2c::I2C_DIG_REG_EXT_RTC_DREG.write_field(RTC_CNTL_DBIAS_1V25);
            regi2c::I2C_DIG_REG_EXT_DIG_DREG.write_field(RTC_CNTL_DBIAS_1V25);

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
            SPI0::regs()
                .clock_gate()
                .modify(|_, w| w.clk_en().clear_bit());
            SPI1::regs()
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
                w.bbpll_force_pu().clear_bit();
                w.bbpll_i2c_force_pu().clear_bit();
                w.bb_i2c_force_pu().clear_bit()
            });

            // cancel RTC REG force PU

            rtc_cntl.pwc().modify(|_, w| w.force_pu().clear_bit());

            rtc_cntl.rtc().modify(|_, w| {
                w.regulator_force_pu().clear_bit();
                w.dboost_force_pu().clear_bit()
            });

            rtc_cntl.pwc().modify(|_, w| {
                w.slowmem_force_noiso().clear_bit();
                w.fastmem_force_noiso().clear_bit()
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
                w.dg_wrap_force_noiso().clear_bit();
                w.dg_wrap_force_iso().clear_bit()
            });

            rtc_cntl.dig_iso().modify(|_, w| {
                w.wifi_force_noiso().clear_bit();
                w.wifi_force_iso().clear_bit()
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
                w.cpu_top_force_noiso().clear_bit();
                w.cpu_top_force_iso().clear_bit()
            });

            rtc_cntl
                .dig_pwc()
                .modify(|_, w| w.cpu_top_force_pu().clear_bit());

            rtc_cntl.dig_iso().modify(|_, w| {
                w.dg_peri_force_noiso().clear_bit();
                w.dg_peri_force_iso().clear_bit()
            });

            rtc_cntl
                .dig_pwc()
                .modify(|_, w| w.dg_peri_force_pu().clear_bit());

            rtc_cntl.pwc().modify(|_, w| {
                w.force_noiso().clear_bit();
                w.force_iso().clear_bit();
                w.force_pu().clear_bit()
            });

            // if SYSTEM_CPU_WAIT_MODE_FORCE_ON == 0,
            // the cpu clk will be closed when cpu enter WAITI mode

            system
                .cpu_per_conf()
                .modify(|_, w| w.cpu_wait_mode_force_on().clear_bit());

            // cancel digital PADS force no iso

            rtc_cntl.dig_iso().modify(|_, w| {
                w.dg_pad_force_unhold().clear_bit();
                w.dg_pad_force_noiso().clear_bit()
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
        let rtc_cntl = LPWR::regs();

        if self.lslp_mem_inf_fpu() {
            rtc_sleep_pu(true);
        }

        if self.modem_pd_en() {
            rtc_cntl.dig_iso().modify(|_, w| {
                w.wifi_force_noiso().clear_bit();
                w.wifi_force_iso().clear_bit()
            });

            rtc_cntl
                .dig_pwc()
                .modify(|_, w| w.wifi_force_pu().clear_bit().wifi_pd_en().set_bit());
        } else {
            rtc_cntl.options0().modify(|_, w| {
                w.bbpll_force_pu().set_bit();
                w.bbpll_i2c_force_pu().set_bit();
                w.bb_i2c_force_pu().set_bit()
            });

            rtc_cntl
                .dig_pwc()
                .modify(|_, w| w.wifi_force_pu().set_bit().wifi_pd_en().clear_bit());
        }

        if self.cpu_pd_en() {
            rtc_cntl.dig_iso().modify(|_, w| {
                w.cpu_top_force_noiso().clear_bit();
                w.cpu_top_force_iso().clear_bit()
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
                w.dg_peri_force_noiso().clear_bit();
                w.dg_peri_force_iso().clear_bit()
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
                w.force_noiso().clear_bit();
                w.force_iso().clear_bit();
                w.force_pu().clear_bit();
                w.pd_en().set_bit()
            });
        } else {
            rtc_cntl.pwc().modify(|_, w| w.pd_en().clear_bit());
        }

        unsafe {
            regi2c::I2C_DIG_REG_EXT_RTC_DREG_SLEEP.write_field(self.rtc_dbias_slp());
            regi2c::I2C_DIG_REG_EXT_DIG_DREG_SLEEP.write_field(self.dig_dbias_slp());

            rtc_cntl.bias_conf().modify(|_, w| {
                w.dbg_atten_deep_slp().bits(self.dbg_atten_slp());
                w.bias_sleep_deep_slp().bit(self.bias_sleep_slp());
                w.pd_cur_deep_slp().bit(self.pd_cur_slp());
                w.dbg_atten_monitor()
                    .bits(RTC_CNTL_DBG_ATTEN_MONITOR_DEFAULT);
                w.bias_sleep_monitor().bit(self.bias_sleep_monitor());
                w.pd_cur_monitor().bit(self.pd_cur_monitor())
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

            rtc_cntl
                .clk_conf()
                .modify(|_, w| w.xtal_global_force_nogating().bit(self.xtal_fpu()));
        }
    }

    /// Configures the wakeup and reject sources of the sleep.
    ///
    /// [`Self::enter_sleep`] requests the sleep after this call.
    pub(crate) fn start_sleep(&self, wakeup_mask: u32, reject_mask: u32) {
        unsafe {
            LPWR::regs()
                .reset_state()
                .modify(|_, w| w.procpu_stat_vector_sel().set_bit());

            // set bits for what can wake us up
            LPWR::regs()
                .wakeup_state()
                .modify(|_, w| w.wakeup_ena().bits(wakeup_mask));

            // Set the bits of the sources that reject the sleep. The reject enables that `apply`
            // wrote arm those sources.
            LPWR::regs()
                .slp_reject_conf()
                .modify(|_, w| w.sleep_reject_ena().bits(reject_mask));
        }
    }

    /// Requests the sleep.
    ///
    /// The caller waits for the result of the request.
    pub(crate) fn enter_sleep(&self) {
        LPWR::regs().state0().modify(|_, w| w.sleep_en().set_bit());
    }

    pub(crate) fn finish_sleep(&self) {
        // In deep sleep mode, we never get here
        unsafe {
            LPWR::regs().int_clr().write(|w| {
                w.slp_reject().clear_bit_by_one();
                w.slp_wakeup().clear_bit_by_one()
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

// The cache tag memory powers down with the CPU, so a retained sleep must write the data cache back
// first and invalidate both caches after. Tag memory retention, which lets a program skip this
// work, comes later; until then the answer is always yes.

// `SOC_RTC_CNTL_CPU_PD_REG_FILE_NUM` (549) times `SOC_RTC_CNTL_CPU_PD_DMA_BLOCK_SIZE` (16).
const _: () = assert!(cpu_retention::payload_size() == 549 * 16);

const RETENTION_CONFIG_WORD3: u32 = 0xfffe_0000;
const RETENTION_WAIT_CYCLES: u8 = 0x7f;
const RETENTION_CLKOFF_WAIT_CYCLES: u8 = 0x0f;
const RETENTION_DONE_WAIT_CYCLES: u8 = 0x07;
// `RETENTION_TARGET` is a bit per target: the CPU is 1 and the cache tag memory is 2.
const RETENTION_TARGET_CPU: u8 = 1;
const RETENTION_TARGET_TAGMEM: u8 = 2;

/// Whether the tag memory survives the sleep, which is what lets the cache work be skipped.
///
/// Both caches are retained together, so one answer serves all three questions below. esp-idf
/// gates them separately because it retains only the tags that cover its mapped segments.
fn tag_memory_retained() -> bool {
    crate::rtc_cntl::tagmem::installed_buffer_ptr().is_some()
}

/// Whether a retained sleep must write the data cache back before the CPU domain powers down.
fn dcache_writeback_needed() -> bool {
    !tag_memory_retained()
}

/// Whether a retained sleep must invalidate the instruction cache after wake.
fn icache_invalidate_needed() -> bool {
    !tag_memory_retained()
}

/// Whether a retained sleep must invalidate the data cache after wake.
fn dcache_invalidate_needed() -> bool {
    !tag_memory_retained()
}

/// Sets CPU power-down in the sleep configuration when a retention buffer is installed.
pub(crate) fn configure_cpu_retention(config: &mut RtcSleepConfig, buffer: Option<*mut u8>) {
    if buffer.is_some() {
        config.set_cpu_pd_en(true);
    }
}

/// Prepares CPU retention for the upcoming sleep.
pub(crate) fn prepare_cpu_retention(buffer: Option<*mut u8>) {
    let Some(buffer) = buffer else {
        return;
    };

    unsafe {
        cpu_retention::init_cpu_dma_link(buffer, RETENTION_CONFIG_WORD3);
        enable_cpu_retention(buffer as usize);

        // The tags are lost only when the CPU domain powers down, so this is armed with the CPU
        // and never on its own.
        if let Some(tag_memory) = crate::rtc_cntl::tagmem::installed_buffer_ptr() {
            init_tag_memory_dma_link(tag_memory);
            enable_tag_memory_retention(tag_memory as usize);
        }

        // Only PSRAM data in the d-cache is at risk, and nothing writes to it between here and the
        // sleep request, so a writeback now is enough. Follows
        // `rtc_cntl_hal_enable_cpu_retention`.
        if dcache_writeback_needed() {
            crate::soc::cache_writeback_all();
        }
    }
}

/// Finishes CPU retention after the sleep request returns.
pub(crate) fn finish_cpu_retention(buffer: Option<*mut u8>, rejected: bool) {
    if buffer.is_none() {
        return;
    }

    // Disarm on every exit, including a rejected request that never slept. A stale descriptor
    // would otherwise affect the next unretained sleep.
    disable_cpu_retention();

    if tag_memory_retained() {
        disable_tag_memory_retention();
    }

    // A rejected request never slept, so it lost no cache contents.
    if rejected {
        return;
    }

    // The tag memory powered down with the CPU, so every line is stale. Follows
    // `rtc_cntl_hal_disable_cpu_retention`.
    unsafe {
        if icache_invalidate_needed() {
            crate::soc::cache_invalidate_icache_all();
        }
        if dcache_invalidate_needed() {
            crate::soc::cache_invalidate_dcache_all();
        }
    }
}

fn enable_cpu_retention(link_addr: usize) {
    // The field is 27 bits and the address needs 30, so the write drops the top three. The DMA
    // reaches internal SRAM only, so the hardware supplies those bits; esp-idf truncates the same
    // way through `REG_SET_FIELD`. `modify` keeps `nobypass_cpu_iso_rst`, which shares the
    // register.
    APB_CTRL::regs()
        .retention_ctrl()
        .modify(|_, w| unsafe { w.retention_cpu_link_addr().bits(link_addr as u32) });

    LPWR::regs().retention_ctrl().modify(|_, w| unsafe {
        w.retention_wait().bits(RETENTION_WAIT_CYCLES);
        w.retention_clkoff_wait().bits(RETENTION_CLKOFF_WAIT_CYCLES);
        w.retention_done_wait().bits(RETENTION_DONE_WAIT_CYCLES)
    });

    LPWR::regs()
        .clk_conf()
        .modify(|_, w| w.dig_clk8m_en().set_bit());

    LPWR::regs().retention_ctrl().modify(|r, w| unsafe {
        w.retention_target()
            .bits(r.retention_target().bits() | RETENTION_TARGET_CPU);
        w.retention_en().set_bit()
    });
}

fn disable_cpu_retention() {
    LPWR::regs()
        .retention_ctrl()
        .modify(|_, w| w.retention_en().clear_bit());
}

/// The tag memory needs no configuration header, unlike the CPU frames.
unsafe fn init_tag_memory_dma_link(buffer: *mut u8) {
    unsafe {
        cpu_retention::init_link(buffer, crate::rtc_cntl::tagmem::payload_size());
    }
}

fn enable_tag_memory_retention(link_addr: usize) {
    let (icache_size, dcache_size) = crate::rtc_cntl::tagmem::size_fields();

    APB_CTRL::regs()
        .retention_ctrl1()
        .modify(|_, w| unsafe { w.retention_tag_link_addr().bits(link_addr as u32) });

    // Every set is retained, so the valid size is the whole size and the starting row does not
    // matter. esp-idf narrows both to the rows its mapped code and data segments occupy, which
    // saves DMA time but needs those segment addresses; retaining all of it needs nothing and
    // cannot under-cover. The size field wraps, so the largest cache writes zero.
    APB_CTRL::regs().retention_ctrl2().modify(|_, w| unsafe {
        w.ret_icache_start_point().bits(0);
        w.ret_icache_vld_size().bits(icache_size);
        w.ret_icache_size().bits(icache_size);
        w.ret_icache_enable().set_bit()
    });

    APB_CTRL::regs().retention_ctrl3().modify(|_, w| unsafe {
        w.ret_dcache_start_point().bits(0);
        w.ret_dcache_vld_size().bits(dcache_size);
        w.ret_dcache_size().bits(dcache_size);
        w.ret_dcache_enable().set_bit()
    });

    // The CPU shares this field, and it is armed first, so the bit is added rather than written.
    LPWR::regs().retention_ctrl().modify(|r, w| unsafe {
        w.retention_target()
            .bits(r.retention_target().bits() | RETENTION_TARGET_TAGMEM)
    });
}

fn disable_tag_memory_retention() {
    LPWR::regs().retention_ctrl().modify(|r, w| unsafe {
        w.retention_target()
            .bits(r.retention_target().bits() & !RETENTION_TARGET_TAGMEM)
    });

    APB_CTRL::regs()
        .retention_ctrl2()
        .modify(|_, w| w.ret_icache_enable().clear_bit());

    APB_CTRL::regs()
        .retention_ctrl3()
        .modify(|_, w| w.ret_dcache_enable().clear_bit());
}
