use crate::{regi2c_write_mask, rom::rom_i2c_writeReg_Mask, Rtc};

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
pub const RTC_CNTL_DBIAS_0V90: u32 = 13;
pub const RTC_CNTL_DBIAS_0V95: u32 = 16;
pub const RTC_CNTL_DBIAS_1V00: u32 = 18;
pub const RTC_CNTL_DBIAS_1V05: u32 = 20;
pub const RTC_CNTL_DBIAS_1V10: u32 = 23;
pub const RTC_CNTL_DBIAS_1V15: u32 = 25;
pub const RTC_CNTL_DBIAS_1V20: u32 = 28;
pub const RTC_CNTL_DBIAS_1V25: u32 = 30;
pub const RTC_CNTL_DBIAS_1V30: u32 = 31; //< voltage is about 1.34v in fact

pub const RTC_CNTL_DBG_ATTEN_MONITOR_DEFAULT: u8 = 0;
pub const RTC_CNTL_ULPCP_TOUCH_START_WAIT_IN_SLEEP: u16 = 0xFF;
pub const RTC_CNTL_ULPCP_TOUCH_START_WAIT_DEFAULT: u16 = 0x10;

pub const RTC_CNTL_PLL_BUF_WAIT_DEFAULT: u8 = 20;
pub const RTC_CNTL_CK8M_WAIT_DEFAULT: u8 = 20;
pub const RTC_CNTL_MIN_SLP_VAL_MIN: u8 = 2;

pub const OTHER_BLOCKS_POWERUP: u8 = 1;
pub const OTHER_BLOCKS_WAIT: u16 = 1;

pub const WIFI_POWERUP_CYCLES: u8 = OTHER_BLOCKS_POWERUP;
pub const WIFI_WAIT_CYCLES: u16 = OTHER_BLOCKS_WAIT;
pub const BT_POWERUP_CYCLES: u8 = OTHER_BLOCKS_POWERUP;
pub const BT_WAIT_CYCLES: u16 = OTHER_BLOCKS_WAIT;
pub const RTC_POWERUP_CYCLES: u8 = OTHER_BLOCKS_POWERUP;
pub const RTC_WAIT_CYCLES: u16 = OTHER_BLOCKS_WAIT;
pub const CPU_TOP_POWERUP_CYCLES: u8 = OTHER_BLOCKS_POWERUP;
pub const CPU_TOP_WAIT_CYCLES: u16 = OTHER_BLOCKS_WAIT;
pub const DG_WRAP_POWERUP_CYCLES: u8 = OTHER_BLOCKS_POWERUP;
pub const DG_WRAP_WAIT_CYCLES: u16 = OTHER_BLOCKS_WAIT;
pub const DG_PERI_POWERUP_CYCLES: u8 = OTHER_BLOCKS_POWERUP;
pub const DG_PERI_WAIT_CYCLES: u16 = OTHER_BLOCKS_WAIT;
pub const RTC_MEM_POWERUP_CYCLES: u8 = OTHER_BLOCKS_POWERUP;
pub const RTC_MEM_WAIT_CYCLES: u16 = OTHER_BLOCKS_WAIT;

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

const DR_REG_SYSCON_BASE: u32 = 0x60026000;
const DR_REG_BB_BASE: u32 = 0x6001D000;
const DR_REG_NRX_BASE: u32 = 0x6001CC00;
const DR_REG_FE_BASE: u32 = 0x60006000;
const DR_REG_FE2_BASE: u32 = 0x60005000;
const DR_REG_SPI1_BASE: u32 = 0x60002000;
const DR_REG_SPI0_BASE: u32 = 0x60003000;

const SPI_MEM_CLOCK_GATE_REG0: u32 = DR_REG_SPI0_BASE + 0xE8;
const SPI_MEM_CLOCK_GATE_REG1: u32 = DR_REG_SPI1_BASE + 0xE8;

const SYSCON_FRONT_END_MEM_PD_REG: u32 = DR_REG_SYSCON_BASE + 0x9C;
const SYSCON_CLKGATE_FORCE_ON_REG: u32 = DR_REG_SYSCON_BASE + 0xA8;
const SYSCON_MEM_POWER_UP_REG: u32 = DR_REG_SYSCON_BASE + 0xB0;

const BBPD_CTRL: u32 = DR_REG_BB_BASE + 0x0054;
const NRXPD_CTRL: u32 = DR_REG_NRX_BASE + 0x00d4;
const FE_GEN_CTRL: u32 = DR_REG_FE_BASE + 0x0090;
const FE2_TX_INTERP_CTRL: u32 = DR_REG_FE2_BASE + 0x00f0;

const SYSCON_DC_MEM_FORCE_PU: u32 = 1 << 4;
const SYSCON_PBUS_MEM_FORCE_PU: u32 = 1 << 2;
const SYSCON_AGC_MEM_FORCE_PU: u32 = 1 << 0;
const SYSCON_SRAM_POWER_UP: u32 = 0x7FF << 3;
const SYSCON_ROM_POWER_UP: u32 = 0x7 << 0;

const BB_FFT_FORCE_PU: u32 = 1 << 3;
const BB_DC_EST_FORCE_PU: u32 = 1 << 1;

const NRX_RX_ROT_FORCE_PU: u32 = 1 << 5;
const NRX_VIT_FORCE_PU: u32 = 1 << 3;
const NRX_DEMAP_FORCE_PU: u32 = 1 << 1;

const FE_IQ_EST_FORCE_PU: u32 = 1 << 5;
const FE2_TX_INF_FORCE_PU: u32 = 1 << 10;

const SPI_MEM_CLK_EN: u32 = 1 << 0;

fn write_register(reg: u32, value: u32) {
    let reg = reg as *mut u32;

    unsafe { reg.write_volatile(value) };
}

fn modify_register(reg: u32, mask: u32, value: u32) {
    let reg = reg as *mut u32;

    unsafe { reg.write_volatile((reg.read_volatile() & !mask) | value) };
}

fn register_modify_bits(reg: u32, bits: u32, set: bool) {
    if set {
        modify_register(reg, bits, bits);
    } else {
        modify_register(reg, bits, 0);
    }
}

fn rtc_sleep_pu(val: bool) {
    let rtc_cntl = unsafe { &*esp32s3::RTC_CNTL::ptr() };

    #[rustfmt::skip]
    rtc_cntl
        .dig_pwc
        .modify(|_, w| w
            .lslp_mem_force_pu().bit(val)
        );

    #[rustfmt::skip]
    rtc_cntl.pwc.modify(|_, w| w
        .slowmem_force_lpu().bit(val)
        .fastmem_force_lpu().bit(val)
    );

    #[rustfmt::skip]
    register_modify_bits(
        SYSCON_FRONT_END_MEM_PD_REG,
        SYSCON_DC_MEM_FORCE_PU | SYSCON_PBUS_MEM_FORCE_PU | SYSCON_AGC_MEM_FORCE_PU,
        val,
    );

    #[rustfmt::skip]
    register_modify_bits(
        BBPD_CTRL,
        BB_FFT_FORCE_PU | BB_DC_EST_FORCE_PU,
        val,
    );

    #[rustfmt::skip]
    register_modify_bits(
        NRXPD_CTRL,
        NRX_RX_ROT_FORCE_PU | NRX_VIT_FORCE_PU | NRX_DEMAP_FORCE_PU,
        val,
    );

    #[rustfmt::skip]
    register_modify_bits(
        FE_GEN_CTRL,
        FE_IQ_EST_FORCE_PU,
        val,
    );

    #[rustfmt::skip]
    register_modify_bits(
        FE2_TX_INTERP_CTRL,
        FE2_TX_INF_FORCE_PU,
        val,
    );

    #[rustfmt::skip]
    register_modify_bits(
        SYSCON_MEM_POWER_UP_REG,
        SYSCON_SRAM_POWER_UP | SYSCON_ROM_POWER_UP,
        val,
    );
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
        // TODO: some of this needs to run at startup, possibly during clock setup
        // settings derived from esp_clk_init -> rtc_init

        unsafe {
            let rtc_cntl = &*esp32s3::RTC_CNTL::ptr();

            #[rustfmt::skip]
            rtc_cntl.dig_pwc.modify(|_, w| w
                .wifi_force_pd().clear_bit()
            );

            regi2c_write_mask!(I2C_DIG_REG, I2C_DIG_REG_XPD_RTC_REG, 0);

            regi2c_write_mask!(I2C_DIG_REG, I2C_DIG_REG_XPD_DIG_REG, 0);

            #[rustfmt::skip]
            rtc_cntl.ana_conf.modify(|_, w| w
                .pvtmon_pu().clear_bit()
            );

            #[rustfmt::skip]
            rtc_cntl.timer1.modify(|_, w| w
                .pll_buf_wait().bits(RTC_CNTL_PLL_BUF_WAIT_DEFAULT)
                .ck8m_wait().bits(RTC_CNTL_CK8M_WAIT_DEFAULT)
            );

            // Moved from rtc sleep to rtc init to save sleep function running time
            // set shortest possible sleep time limit
            #[rustfmt::skip]
            rtc_cntl.timer5.modify(|_, w| w
                .min_slp_val().bits(RTC_CNTL_MIN_SLP_VAL_MIN)
            );

            #[rustfmt::skip]
            rtc_cntl.timer3.modify(|_, w| w
                // set wifi timer
                .wifi_powerup_timer().bits(WIFI_POWERUP_CYCLES)
                .wifi_wait_timer().bits(WIFI_WAIT_CYCLES)
                // set bt timer
                .bt_powerup_timer().bits(BT_POWERUP_CYCLES)
                .bt_wait_timer().bits(BT_WAIT_CYCLES)
            );

            #[rustfmt::skip]
            rtc_cntl.timer6.modify(|_, w| w
                .cpu_top_powerup_timer().bits(CPU_TOP_POWERUP_CYCLES)
                .cpu_top_wait_timer().bits(CPU_TOP_WAIT_CYCLES)
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
            rtc_cntl.timer6.modify(|_, w| w
                .dg_peri_powerup_timer().bits(DG_PERI_POWERUP_CYCLES)
                .dg_peri_wait_timer().bits(DG_PERI_WAIT_CYCLES)
            );

            // Reset RTC bias to default value (needed if waking up from deep sleep)
            regi2c_write_mask!(
                I2C_DIG_REG,
                I2C_DIG_REG_EXT_RTC_DREG_SLEEP,
                RTC_CNTL_DBIAS_1V10
            );
            regi2c_write_mask!(I2C_DIG_REG, I2C_DIG_REG_EXT_RTC_DREG, RTC_CNTL_DBIAS_1V10);

            // Set the wait time to the default value.
            #[rustfmt::skip]
            rtc_cntl.timer2.modify(|_, w| w
                .ulpcp_touch_start_wait().bits(RTC_CNTL_ULPCP_TOUCH_START_WAIT_DEFAULT)
            );

            // LDO dbias initialization
            // TODO: this modifies g_rtc_dbias_pvt_non_240m and g_dig_dbias_pvt_non_240m.
            //       We're using a high enough default but we should read from the efuse.
            // rtc_set_stored_dbias();

            regi2c_write_mask!(I2C_DIG_REG, I2C_DIG_REG_EXT_RTC_DREG, RTC_CNTL_DBIAS_1V25);
            regi2c_write_mask!(I2C_DIG_REG, I2C_DIG_REG_EXT_DIG_DREG, RTC_CNTL_DBIAS_1V25);

            // clear CMMU clock force on
            #[rustfmt::skip]
            (&*esp32s3::EXTMEM::ptr()).cache_mmu_power_ctrl.modify(|_,w| w
                .cache_mmu_mem_force_on().clear_bit()
            );

            // clear clkgate force on
            write_register(SYSCON_CLKGATE_FORCE_ON_REG, 0);

            // clear tag clock force on
            #[rustfmt::skip]
            (&*esp32s3::EXTMEM::ptr()).dcache_tag_power_ctrl.modify(|_,w| w
                .dcache_tag_mem_force_on().clear_bit()
            );
            #[rustfmt::skip]
            (&*esp32s3::EXTMEM::ptr()).icache_tag_power_ctrl.modify(|_,w| w
                .icache_tag_mem_force_on().clear_bit()
            );

            // clear register clock force on
            register_modify_bits(SPI_MEM_CLOCK_GATE_REG0, SPI_MEM_CLK_EN, false);
            register_modify_bits(SPI_MEM_CLOCK_GATE_REG1, SPI_MEM_CLK_EN, false);

            #[rustfmt::skip]
            rtc_cntl.clk_conf.modify(|_, w| w
                .ck8m_force_pu().clear_bit()
            );
            #[rustfmt::skip]
            rtc_cntl.options0.modify(|_, w| w
                .xtl_force_pu().clear_bit()
            );

            #[rustfmt::skip]
            rtc_cntl.ana_conf.modify(|_, w| w
                // open sar_i2c protect function to avoid sar_i2c reset when rtc_ldo is low.
                // clear i2c_reset_protect pd force, need tested in low temperature.
                // NOTE: this bit is written again in esp-idf, but it's not clear why.
                .i2c_reset_por_force_pd().clear_bit()
            );

            // cancel bbpll force pu if setting no force power up
            #[rustfmt::skip]
            rtc_cntl.options0.modify(|_, w| w
                .bbpll_force_pu().clear_bit()
                .bbpll_i2c_force_pu().clear_bit()
                .bb_i2c_force_pu().clear_bit()
            );

            // cancel RTC REG force PU
            #[rustfmt::skip]
            rtc_cntl.pwc.modify(|_, w| w
                .force_pu().clear_bit()
            );
            #[rustfmt::skip]
            rtc_cntl.rtc.modify(|_, w| w
                .regulator_force_pu().clear_bit()
                .dboost_force_pu().clear_bit()
            );

            #[rustfmt::skip]
            rtc_cntl.pwc.modify(|_, w| w
                .slowmem_force_noiso().clear_bit()
                .fastmem_force_noiso().clear_bit()
            );

            #[rustfmt::skip]
            rtc_cntl.rtc.modify(|_, w| w
                .dboost_force_pd().set_bit()
            );

            // If this mask is enabled, all soc memories cannot enter power down mode
            // We should control soc memory power down mode from RTC, so we will not touch
            // this register any more
            #[rustfmt::skip]
            (&*esp32s3::SYSTEM::ptr()).mem_pd_mask.modify(|_,w| w
                .lslp_mem_pd_mask().clear_bit()
            );

            // If this pd_cfg is set to 1, all memory won't enter low power mode during
            // light sleep If this pd_cfg is set to 0, all memory will enter low
            // power mode during light sleep
            rtc_sleep_pu(false);

            #[rustfmt::skip]
            rtc_cntl.dig_pwc.modify(|_, w| w
                .dg_wrap_force_pu().clear_bit()
            );
            #[rustfmt::skip]
            rtc_cntl.dig_iso.modify(|_, w| w
                .dg_wrap_force_noiso().clear_bit()
                .dg_wrap_force_iso().clear_bit()
            );

            #[rustfmt::skip]
            rtc_cntl.dig_iso.modify(|_, w| w
                .wifi_force_noiso().clear_bit()
                .wifi_force_iso().clear_bit()
            );
            #[rustfmt::skip]
            rtc_cntl.dig_pwc.modify(|_, w| w
                .wifi_force_pu().clear_bit()
            );

            #[rustfmt::skip]
            rtc_cntl.dig_iso.modify(|_, w| w
                .bt_force_noiso().clear_bit()
                .bt_force_iso().clear_bit()
            );
            #[rustfmt::skip]
            rtc_cntl.dig_pwc.modify(|_, w| w
                .bt_force_pu().clear_bit()
            );

            #[rustfmt::skip]
            rtc_cntl.dig_iso.modify(|_, w| w
                .cpu_top_force_noiso().clear_bit()
                .cpu_top_force_iso().clear_bit()
            );
            #[rustfmt::skip]
            rtc_cntl.dig_pwc.modify(|_, w| w
                .cpu_top_force_pu().clear_bit()
            );

            #[rustfmt::skip]
            rtc_cntl.dig_iso.modify(|_, w| w
                .dg_peri_force_noiso().clear_bit()
                .dg_peri_force_iso().clear_bit()
            );
            #[rustfmt::skip]
            rtc_cntl.dig_pwc.modify(|_, w| w
                .dg_peri_force_pu().clear_bit()
            );

            #[rustfmt::skip]
            rtc_cntl.pwc.modify(|_, w| w
                .force_noiso().clear_bit()
                .force_iso().clear_bit()
                .force_pu().clear_bit()
            );

            // if SYSTEM_CPU_WAIT_MODE_FORCE_ON == 0,
            // the cpu clk will be closed when cpu enter WAITI mode
            #[rustfmt::skip]
            (&*esp32s3::SYSTEM::ptr()).cpu_per_conf.modify(|_,w| w
                .cpu_wait_mode_force_on().clear_bit()
            );

            // cancel digital PADS force no iso
            #[rustfmt::skip]
            rtc_cntl.dig_iso.modify(|_, w| w
                .dg_pad_force_unhold().clear_bit()
                .dg_pad_force_noiso().clear_bit()
            );

            // force power down modem(wifi and ble) power domain
            #[rustfmt::skip]
            rtc_cntl.dig_iso.modify(|_, w| w
                .wifi_force_iso().set_bit()
            );
            #[rustfmt::skip]
            rtc_cntl.dig_pwc.modify(|_, w| w
                .wifi_force_pd().set_bit()
            );

            rtc_cntl.int_ena_rtc.write(|w| w.bits(0));
            rtc_cntl.int_clr_rtc.write(|w| w.bits(u32::MAX));
        }
    }

    pub(crate) fn apply(&self, rtc: &Rtc) {
        self.base_settings(rtc);

        // like esp-idf rtc_sleep_init()
        let rtc_cntl = unsafe { &*esp32s3::RTC_CNTL::ptr() };

        if self.lslp_mem_inf_fpu() {
            rtc_sleep_pu(true);
        }

        if self.modem_pd_en() {
            #[rustfmt::skip]
            rtc_cntl.dig_iso.modify(|_, w| w
                .wifi_force_noiso().clear_bit()
                .wifi_force_iso().clear_bit()
            );

            #[rustfmt::skip]
            rtc_cntl.dig_pwc.modify(|_, w| w
                .wifi_force_pu().clear_bit()
                .wifi_pd_en().set_bit()
            );
        } else {
            #[rustfmt::skip]
            rtc_cntl.dig_pwc.modify(|_, w| w
                .wifi_pd_en().clear_bit()
            );
        }

        if self.cpu_pd_en() {
            #[rustfmt::skip]
            rtc_cntl.dig_iso.modify(|_, w| w
                .cpu_top_force_noiso().clear_bit()
                .cpu_top_force_iso().clear_bit()
            );

            #[rustfmt::skip]
            rtc_cntl.dig_pwc.modify(|_, w| w
                .cpu_top_force_pu().clear_bit()
                .cpu_top_pd_en().set_bit()
            );
        } else {
            #[rustfmt::skip]
            rtc_cntl.dig_pwc.modify(|_, w| w
                .cpu_top_pd_en().clear_bit()
            );
        }

        if self.dig_peri_pd_en() {
            #[rustfmt::skip]
            rtc_cntl.dig_iso.modify(|_, w| w
                .dg_peri_force_noiso().clear_bit()
                .dg_peri_force_iso().clear_bit()
            );

            #[rustfmt::skip]
            rtc_cntl.dig_pwc.modify(|_, w| w
                .dg_peri_force_pu().clear_bit()
                .dg_peri_pd_en().set_bit()
            );
        } else {
            #[rustfmt::skip]
            rtc_cntl.dig_pwc.modify(|_, w| w
                .dg_peri_pd_en().clear_bit()
            );
        }

        if self.rtc_peri_pd_en() {
            #[rustfmt::skip]
            rtc_cntl.pwc.modify(|_, w| w
                .force_noiso().clear_bit()
                .force_iso().clear_bit()
                .force_pu().clear_bit()
                .pd_en().set_bit()
            );
        } else {
            #[rustfmt::skip]
            rtc_cntl.pwc.modify(|_, w| w
                .pd_en().clear_bit()
            );
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

            #[rustfmt::skip]
            rtc_cntl.bias_conf.modify(|_, w| w
                .dbg_atten_deep_slp().bits(self.dbg_atten_slp())
                .bias_sleep_deep_slp().bit(self.bias_sleep_slp())
                .pd_cur_deep_slp().bit(self.pd_cur_slp())
                .dbg_atten_monitor().bits(RTC_CNTL_DBG_ATTEN_MONITOR_DEFAULT)
                .bias_sleep_monitor().bit(self.bias_sleep_monitor())
                .pd_cur_monitor().bit(self.pd_cur_monitor())
            );

            if self.deep_slp() {
                #[rustfmt::skip]
                rtc_cntl.dig_pwc.modify(|_, w| w
                    .dg_wrap_pd_en().set_bit()
                );

                #[rustfmt::skip]
                rtc_cntl.ana_conf.modify(|_, w| w
                    .ckgen_i2c_pu().clear_bit()
                    .pll_i2c_pu().clear_bit()
                    .rfrx_pbus_pu().clear_bit()
                    .txrf_i2c_pu().clear_bit()
                );

                #[rustfmt::skip]
                rtc_cntl.options0.modify(|_, w| w
                    .bb_i2c_force_pu().clear_bit()
                );
            } else {
                #[rustfmt::skip]
                rtc_cntl.regulator_drv_ctrl.modify(|_, w| w
                    .dg_vdd_drv_b_slp().bits(0xF)
                );

                #[rustfmt::skip]
                rtc_cntl.dig_pwc.modify(|_, w| w
                    .dg_wrap_pd_en().clear_bit()
                );
            }

            // mem force pu
            #[rustfmt::skip]
            rtc_cntl.dig_pwc.modify(|_, w| w
                .lslp_mem_force_pu().set_bit()
            );

            #[rustfmt::skip]
            rtc_cntl.rtc.modify(|_, w| w
                .regulator_force_pu().bit(self.rtc_regulator_fpu())
            );

            #[rustfmt::skip]
            rtc_cntl.clk_conf.modify(|_, w| w
                .ck8m_force_pu().bit(self.int_8m_pd_en())
            );

            // enable VDDSDIO control by state machine
            #[rustfmt::skip]
            rtc_cntl.sdio_conf.modify(|_, w| w
                .sdio_force().clear_bit()
                .sdio_reg_pd_en().bit(self.vddsdio_pd_en())
            );

            #[rustfmt::skip]
            rtc_cntl.slp_reject_conf.modify(|_, w| w
                .deep_slp_reject_en().bit(self.deep_slp_reject())
                .light_slp_reject_en().bit(self.light_slp_reject())
            );

            // Set wait cycle for touch or COCPU after deep sleep and light
            // sleep.
            #[rustfmt::skip]
            rtc_cntl.timer2.modify(|_, w| w
                .ulpcp_touch_start_wait().bits(RTC_CNTL_ULPCP_TOUCH_START_WAIT_IN_SLEEP)
            );

            #[rustfmt::skip]
            rtc_cntl.options0.modify(|_, w| w
                .xtl_force_pu().bit(self.xtal_fpu())
            );

            #[rustfmt::skip]
            rtc_cntl.clk_conf.modify(|_, w| w
                .xtal_global_force_nogating().bit(self.xtal_fpu())
            );
        }
    }
}
