use paste::paste;

use crate::{
    clock::XtalClock, pac::APB_CTRL, pac::EXTMEM, pac::RTC_CNTL, pac::SPI0, pac::SPI1, pac::SYSTEM,
};

use crate::rtc_cntl::{RtcCalSel, RtcClock, RtcFastClock, RtcSlowClock};

use crate::regi2c_write_mask;
use crate::rom::regi2c_ctrl_write_reg_mask;

const I2C_DIG_REG: u32 = 0x6d;
const I2C_DIG_REG_HOSTID: u32 = 0;

const I2C_ULP: u32 = 0x61;
const I2C_ULP_HOSTID: u32 = 0;

const I2C_DIG_REG_XPD_RTC_REG: u32 = 13;
const I2C_DIG_REG_XPD_RTC_REG_MSB: u32 = 2;
const I2C_DIG_REG_XPD_RTC_REG_LSB: u32 = 2;

const I2C_DIG_REG_XPD_DIG_REG: u32 = 13;
const I2C_DIG_REG_XPD_DIG_REG_MSB: u32 = 3;
const I2C_DIG_REG_XPD_DIG_REG_LSB: u32 = 3;

const I2C_ULP_IR_FORCE_XPD_CK: u32 = 0;
const I2C_ULP_IR_FORCE_XPD_CK_MSB: u32 = 2;
const I2C_ULP_IR_FORCE_XPD_CK_LSB: u32 = 2;

pub(crate) fn init() {
    let rtc_cntl = unsafe { &*RTC_CNTL::ptr() };

    unsafe {
        regi2c_write_mask!(I2C_DIG_REG, I2C_DIG_REG_XPD_DIG_REG, 0);

        regi2c_write_mask!(I2C_DIG_REG, I2C_DIG_REG_XPD_RTC_REG, 0);
    }

    rtc_cntl.ana_conf.modify(|_, w| w.pvtmon_pu().clear_bit());

    unsafe {
        rtc_cntl
            .timer1
            .modify(|_, w| w.pll_buf_wait().bits(20u8).ck8m_wait().bits(20u8));
        rtc_cntl.timer5.modify(|_, w| w.min_slp_val().bits(2u8));

        // Set default powerup & wait time
        rtc_cntl.timer3.modify(|_, w| {
            w.wifi_powerup_timer()
                .bits(1u8)
                .wifi_wait_timer()
                .bits(1u16)
                .bt_powerup_timer()
                .bits(1u8)
                .bt_wait_timer()
                .bits(1u16)
        });
        rtc_cntl.timer4.modify(|_, w| {
            w.cpu_top_powerup_timer()
                .bits(1u8)
                .cpu_top_wait_timer()
                .bits(1u16)
                .dg_wrap_powerup_timer()
                .bits(1u8)
                .dg_wrap_wait_timer()
                .bits(1u16)
        });
        rtc_cntl.timer6.modify(|_, w| {
            w.dg_peri_powerup_timer()
                .bits(1u8)
                .dg_peri_wait_timer()
                .bits(1u16)
        });
    }

    calibrate_ocode();

    set_rtc_dig_dbias();

    clock_control_init();

    power_control_init();

    unsafe {
        rtc_cntl.int_ena_rtc.write(|w| w.bits(0));
        rtc_cntl.int_clr_rtc.write(|w| w.bits(u32::MAX));

        regi2c_write_mask!(I2C_ULP, I2C_ULP_IR_FORCE_XPD_CK, 0);
    }
}

pub(crate) fn configure_clock() {
    assert!(matches!(
        RtcClock::get_xtal_freq(),
        XtalClock::RtcXtalFreq40M
    ));

    RtcClock::set_fast_freq(RtcFastClock::RtcFastClock8m);

    let cal_val = loop {
        RtcClock::set_slow_freq(RtcSlowClock::RtcSlowClockRtc);

        let res = RtcClock::calibrate(RtcCalSel::RtcCalRtcMux, 1024);
        if res != 0 {
            break res;
        }
    };

    unsafe {
        let rtc_cntl = &*RTC_CNTL::ptr();
        rtc_cntl.store1.write(|w| w.bits(cal_val));
    }
}

fn calibrate_ocode() {}

fn set_rtc_dig_dbias() {}

/// Perform clock control related initialization
fn clock_control_init() {
    let extmem = unsafe { &*EXTMEM::ptr() };
    let spi_mem_0 = unsafe { &*SPI0::ptr() };
    let spi_mem_1 = unsafe { &*SPI1::ptr() };

    // Clear CMMU clock force on
    extmem
        .cache_mmu_power_ctrl
        .modify(|_, w| w.cache_mmu_mem_force_on().clear_bit());

    // Clear tag clock force on
    extmem
        .icache_tag_power_ctrl
        .modify(|_, w| w.icache_tag_mem_force_on().clear_bit());

    // Clear register clock force on
    spi_mem_0.clock_gate.modify(|_, w| w.clk_en().clear_bit());
    spi_mem_1.clock_gate.modify(|_, w| w.clk_en().clear_bit());
}

/// Perform power control related initialization
fn power_control_init() {
    let rtc_cntl = unsafe { &*RTC_CNTL::ptr() };
    let system = unsafe { &*SYSTEM::ptr() };
    rtc_cntl
        .clk_conf
        .modify(|_, w| w.ck8m_force_pu().clear_bit());

    // Cancel XTAL force PU if no need to force power up
    // Cannot cancel XTAL force PU if PLL is force power on
    rtc_cntl
        .options0
        .modify(|_, w| w.xtl_force_pu().clear_bit());

    // Force PD APLL
    rtc_cntl.ana_conf.modify(|_, w| {
        w.plla_force_pu()
            .clear_bit()
            .plla_force_pd()
            .set_bit()
            // Open SAR_I2C protect function to avoid SAR_I2C
            // Reset when rtc_ldo is low.
            .reset_por_force_pd()
            .clear_bit()
    });

    // Cancel BBPLL force PU if setting no force power up
    rtc_cntl.options0.modify(|_, w| {
        w.bbpll_force_pu()
            .clear_bit()
            .bbpll_i2c_force_pu()
            .clear_bit()
            .bb_i2c_force_pu()
            .clear_bit()
    });
    rtc_cntl.rtc_cntl.modify(|_, w| {
        w.regulator_force_pu()
            .clear_bit()
            .dboost_force_pu()
            .clear_bit()
            .dboost_force_pd()
            .set_bit()
    });

    // If this mask is enabled, all soc memories cannot enter power down mode.
    // We should control soc memory power down mode from RTC,
    // so we will not touch this register any more.
    system
        .mem_pd_mask
        .modify(|_, w| w.lslp_mem_pd_mask().clear_bit());

    rtc_sleep_pu();

    rtc_cntl.dig_pwc.modify(|_, w| {
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
    rtc_cntl.dig_iso.modify(|_, w| {
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

    // Cancel digital PADS force no iso
    system
        .cpu_per_conf
        .modify(|_, w| w.cpu_wait_mode_force_on().clear_bit());

    // If SYSTEM_CPU_WAIT_MODE_FORCE_ON == 0,
    // the CPU clock will be closed when CPU enter WAITI mode.
    rtc_cntl.dig_iso.modify(|_, w| {
        w.dg_pad_force_unhold()
            .clear_bit()
            .dg_pad_force_noiso()
            .clear_bit()
    });
}

/// Configure whether certain peripherals are powered down in deep sleep
fn rtc_sleep_pu() {
    let rtc_cntl = unsafe { &*RTC_CNTL::ptr() };
    let apb_ctrl = unsafe { &*APB_CTRL::ptr() };

    rtc_cntl.dig_pwc.modify(|_, w| {
        w.lslp_mem_force_pu()
            .clear_bit()
            .rtc_fastmem_force_lpu()
            .clear_bit()
    });

    apb_ctrl.front_end_mem_pd.modify(|_, w| {
        w.dc_mem_force_pu()
            .clear_bit()
            .pbus_mem_force_pu()
            .clear_bit()
            .agc_mem_force_pu()
            .clear_bit()
    });
    apb_ctrl
        .mem_power_up
        .modify(|_, w| unsafe { w.sram_power_up().bits(0u8).rom_power_up().bits(0u8) });
}
