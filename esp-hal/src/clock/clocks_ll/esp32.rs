use crate::{
    clock::{Clock, PllClock, XtalClock},
    efuse::{Efuse, VOL_LEVEL_HP_INV},
    peripherals::{APB_CTRL, DPORT, LPWR},
    soc::regi2c,
};

const REF_CLK_FREQ: u32 = 1000000;

const MHZ: u32 = 1000000;
const UINT16_MAX: u32 = 0xffff;

const RTC_CNTL_DBIAS_1V10: u8 = 4;
const RTC_CNTL_DBIAS_1V25: u8 = 7;

const DIG_DBIAS_80M_160M: u8 = RTC_CNTL_DBIAS_1V10;
const DIG_DBIAS_XTAL: u8 = RTC_CNTL_DBIAS_1V10;

const BBPLL_IR_CAL_DELAY_VAL: u8 = 0x18;
const BBPLL_IR_CAL_EXT_CAP_VAL: u8 = 0x20;
const BBPLL_OC_ENB_FCAL_VAL: u8 = 0x9a;
const BBPLL_OC_ENB_VCON_VAL: u8 = 0x00;
const BBPLL_BBADC_CAL_7_0_VAL: u8 = 0x00;

const BBPLL_ENDIV5_VAL_320M: u8 = 0x43;
const BBPLL_BBADC_DSMP_VAL_320M: u8 = 0x84;
const BBPLL_ENDIV5_VAL_480M: u8 = 0xc3;
const BBPLL_BBADC_DSMP_VAL_480M: u8 = 0x74;

pub(crate) fn esp32_rtc_bbpll_configure(xtal_freq: XtalClock, pll_freq: PllClock) {
    let rtc_cntl_dbias_hp_volt = RTC_CNTL_DBIAS_1V25 - Efuse::read_field_le::<u8>(VOL_LEVEL_HP_INV);
    let dig_dbias_240_m = rtc_cntl_dbias_hp_volt;

    let div_ref: u8;
    let div7_0: u8;
    let div10_8: u8;
    let lref: u8;
    let dcur: u8;
    let bw: u8;

    if matches!(pll_freq, PllClock::Pll320MHz) {
        // Raise the voltage, if needed
        LPWR::regs()
            .reg()
            .modify(|_, w| unsafe { w.dig_dbias_wak().bits(DIG_DBIAS_80M_160M) });

        // Configure 320M PLL
        match xtal_freq {
            XtalClock::_40M => {
                div_ref = 0;
                div7_0 = 32;
                div10_8 = 0;
                lref = 0;
                dcur = 6;
                bw = 3;
            }

            XtalClock::_26M => {
                div_ref = 12;
                div7_0 = 224;
                div10_8 = 4;
                lref = 1;
                dcur = 0;
                bw = 1;
            }

            XtalClock::Other(_) => {
                div_ref = 12;
                div7_0 = 224;
                div10_8 = 4;
                lref = 0;
                dcur = 0;
                bw = 0;
            }
        }

        regi2c::I2C_BBPLL_ENDIV5.write_reg(BBPLL_ENDIV5_VAL_320M);
        regi2c::I2C_BBPLL_BBADC_DSMP.write_reg(BBPLL_BBADC_DSMP_VAL_320M);
    } else {
        // Raise the voltage
        LPWR::regs()
            .reg()
            .modify(|_, w| unsafe { w.dig_dbias_wak().bits(dig_dbias_240_m) });

        // Configure 480M PLL
        match xtal_freq {
            XtalClock::_40M => {
                div_ref = 0;
                div7_0 = 28;
                div10_8 = 0;
                lref = 0;
                dcur = 6;
                bw = 3;
            }

            XtalClock::_26M => {
                div_ref = 12;
                div7_0 = 144;
                div10_8 = 4;
                lref = 1;
                dcur = 0;
                bw = 1;
            }

            XtalClock::Other(_) => {
                div_ref = 12;
                div7_0 = 224;
                div10_8 = 4;
                lref = 0;
                dcur = 0;
                bw = 0;
            }
        }

        regi2c::I2C_BBPLL_ENDIV5.write_reg(BBPLL_ENDIV5_VAL_480M);
        regi2c::I2C_BBPLL_BBADC_DSMP.write_reg(BBPLL_BBADC_DSMP_VAL_480M);
    }

    let i2c_bbpll_lref = (lref << 7) | (div10_8 << 4) | (div_ref);
    let i2c_bbpll_dcur = (bw << 6) | dcur;

    regi2c::I2C_BBPLL_OC_LREF.write_reg(i2c_bbpll_lref);
    regi2c::I2C_BBPLL_OC_DIV_REG.write_reg(div7_0);
    regi2c::I2C_BBPLL_OC_DCUR.write_reg(i2c_bbpll_dcur);
}

pub(crate) fn esp32_rtc_bbpll_enable() {
    LPWR::regs().options0().modify(|_, w| {
        w.bias_i2c_force_pd().clear_bit();
        w.bb_i2c_force_pd().clear_bit();
        w.bbpll_force_pd().clear_bit();
        w.bbpll_i2c_force_pd().clear_bit()
    });

    // reset BBPLL configuration
    regi2c::I2C_BBPLL_IR_CAL_DELAY.write_reg(BBPLL_IR_CAL_DELAY_VAL);
    regi2c::I2C_BBPLL_IR_CAL_EXT_CAP.write_reg(BBPLL_IR_CAL_EXT_CAP_VAL);
    regi2c::I2C_BBPLL_OC_ENB_FCAL.write_reg(BBPLL_OC_ENB_FCAL_VAL);
    regi2c::I2C_BBPLL_OC_ENB_VCON.write_reg(BBPLL_OC_ENB_VCON_VAL);
    regi2c::I2C_BBPLL_BBADC_CAL_REG.write_reg(BBPLL_BBADC_CAL_7_0_VAL);
}

pub(crate) fn esp32_rtc_update_to_xtal(freq: XtalClock, _div: u32) {
    let value = ((freq.hz() >> 12) & UINT16_MAX) | (((freq.hz() >> 12) & UINT16_MAX) << 16);
    esp32_update_cpu_freq(freq.hz());

    // set divider from XTAL to APB clock
    APB_CTRL::regs().sysclk_conf().modify(|_, w| unsafe {
        w.pre_div_cnt()
            .bits(((freq.hz()) / REF_CLK_FREQ - 1) as u16)
    });

    // adjust ref_tick
    APB_CTRL::regs().xtal_tick_conf().modify(|_, w| unsafe {
        w.xtal_tick_num()
            .bits(((freq.hz()) / REF_CLK_FREQ - 1) as u8)
    });

    // switch clock source
    LPWR::regs()
        .clk_conf()
        .modify(|_, w| w.soc_clk_sel().xtal());
    LPWR::regs()
        .store5()
        .modify(|_, w| unsafe { w.scratch5().bits(value) });

    // lower the voltage
    LPWR::regs()
        .reg()
        .modify(|_, w| unsafe { w.dig_dbias_wak().bits(DIG_DBIAS_XTAL) });
}

pub(crate) fn set_cpu_freq(cpu_freq_mhz: crate::clock::CpuClock) {
    let rtc_cntl_dbias_hp_volt = RTC_CNTL_DBIAS_1V25 - Efuse::read_field_le::<u8>(VOL_LEVEL_HP_INV);

    let dig_dbias_240_m = rtc_cntl_dbias_hp_volt;

    const CPU_80M: u8 = 0;
    const CPU_160M: u8 = 1;
    const CPU_240M: u8 = 2;

    let mut dbias = DIG_DBIAS_80M_160M;
    let per_conf = match cpu_freq_mhz {
        crate::clock::CpuClock::_160MHz => CPU_160M,
        crate::clock::CpuClock::_240MHz => {
            dbias = dig_dbias_240_m;
            CPU_240M
        }
        crate::clock::CpuClock::_80MHz => CPU_80M,
    };

    let value = (((80 * MHZ) >> 12) & UINT16_MAX) | ((((80 * MHZ) >> 12) & UINT16_MAX) << 16);
    DPORT::regs()
        .cpu_per_conf()
        .write(|w| unsafe { w.cpuperiod_sel().bits(per_conf) });
    LPWR::regs()
        .reg()
        .modify(|_, w| unsafe { w.dig_dbias_wak().bits(dbias) });
    LPWR::regs().clk_conf().modify(|_, w| w.soc_clk_sel().pll());
    LPWR::regs()
        .store5()
        .modify(|_, w| unsafe { w.scratch5().bits(value) });

    esp32_update_cpu_freq(cpu_freq_mhz.mhz());
}

/// Pass the CPU clock in MHz so that ets_delay_us
/// will be accurate. Call this function when CPU frequency is changed.
fn esp32_update_cpu_freq(mhz: u32) {
    const G_TICKS_PER_US_PRO: u32 = 0x3ffe01e0;
    unsafe {
        // Update scale factors used by esp_rom_delay_us
        (G_TICKS_PER_US_PRO as *mut u32).write_volatile(mhz);
    }
}

const DPORT_WIFI_CLK_WIFI_BT_COMMON_M: u32 = 0x000003c9;
const DPORT_WIFI_CLK_WIFI_EN_M: u32 = 0x00000406;
const DPORT_WIFI_CLK_BT_EN_M: u32 = 0x00030800;

pub(super) fn enable_phy(enable: bool) {
    // `periph_ll_wifi_bt_module_enable_clk_clear_rst`
    // `periph_ll_wifi_bt_module_disable_clk_set_rst`
    DPORT::regs().wifi_clk_en().modify(|r, w| unsafe {
        if enable {
            w.bits(r.bits() | DPORT_WIFI_CLK_WIFI_BT_COMMON_M)
        } else {
            w.bits(r.bits() & !DPORT_WIFI_CLK_WIFI_BT_COMMON_M)
        }
    });
}

pub(super) fn enable_bt(enable: bool) {
    DPORT::regs().wifi_clk_en().modify(|r, w| unsafe {
        if enable {
            w.bits(r.bits() | DPORT_WIFI_CLK_BT_EN_M)
        } else {
            w.bits(r.bits() & !DPORT_WIFI_CLK_BT_EN_M)
        }
    });
}

pub(super) fn enable_wifi(enable: bool) {
    // `periph_ll_wifi_module_enable_clk_clear_rst`
    // `periph_ll_wifi_module_disable_clk_set_rst`
    DPORT::regs().wifi_clk_en().modify(|r, w| unsafe {
        if enable {
            w.bits(r.bits() | DPORT_WIFI_CLK_WIFI_EN_M)
        } else {
            w.bits(r.bits() & !DPORT_WIFI_CLK_WIFI_EN_M)
        }
    });
}

pub(super) fn reset_mac() {
    DPORT::regs()
        .wifi_rst_en()
        .modify(|_, w| w.mac_rst().set_bit());
    DPORT::regs()
        .wifi_rst_en()
        .modify(|_, w| w.mac_rst().clear_bit());
}

pub(super) fn init_clocks() {
    // esp-idf assumes all clocks are enabled by default, and disables the following
    // bits:
    //
    // ```
    // const DPORT_WIFI_CLK_SDIOSLAVE_EN: u32 = 1 << 4;
    // const DPORT_WIFI_CLK_UNUSED_BIT5: u32 = 1 << 5;
    // const DPORT_WIFI_CLK_UNUSED_BIT12: u32 = 1 << 12;
    // const DPORT_WIFI_CLK_SDIO_HOST_EN: u32 = 1 << 13;
    // const DPORT_WIFI_CLK_EMAC_EN: u32 = 1 << 14;
    //
    // const WIFI_BT_SDIO_CLK: u32 = DPORT_WIFI_CLK_WIFI_EN_M
    //     | DPORT_WIFI_CLK_BT_EN_M
    //     | DPORT_WIFI_CLK_UNUSED_BIT5
    //     | DPORT_WIFI_CLK_UNUSED_BIT12
    //     | DPORT_WIFI_CLK_SDIOSLAVE_EN
    //     | DPORT_WIFI_CLK_SDIO_HOST_EN
    //     | DPORT_WIFI_CLK_EMAC_EN;
    // ```
    //
    // However, we can't do this because somehow our initialization process is
    // different, and disabling some bits, or not enabling them makes the BT
    // stack crash.

    DPORT::regs()
        .wifi_clk_en()
        .write(|w| unsafe { w.bits(u32::MAX) });
}

pub(super) fn ble_rtc_clk_init() {
    // nothing for this target
}

pub(super) fn reset_rpa() {
    // nothing for this target
}
