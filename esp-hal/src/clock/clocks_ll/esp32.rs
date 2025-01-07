use crate::{
    clock::{Clock, PllClock, XtalClock},
    regi2c_write,
};

const REF_CLK_FREQ: u32 = 1000000;

const MHZ: u32 = 1000000;
const UINT16_MAX: u32 = 0xffff;

const RTC_CNTL_DBIAS_1V10: u32 = 4;
const RTC_CNTL_DBIAS_1V25: u32 = 7;

const DIG_DBIAS_80M_160M: u32 = RTC_CNTL_DBIAS_1V10;
const DIG_DBIAS_XTAL: u32 = RTC_CNTL_DBIAS_1V10;

const I2C_BBPLL: u32 = 0x66;
const I2C_BBPLL_HOSTID: u32 = 4;

const I2C_BBPLL_IR_CAL_DELAY: u32 = 0;
const I2C_BBPLL_IR_CAL_EXT_CAP: u32 = 1;
const I2C_BBPLL_OC_ENB_FCAL: u32 = 4;
const I2C_BBPLL_OC_ENB_VCON: u32 = 10;
const I2C_BBPLL_BBADC_CAL_7_0: u32 = 12;

const BBPLL_IR_CAL_DELAY_VAL: u32 = 0x18;
const BBPLL_IR_CAL_EXT_CAP_VAL: u32 = 0x20;
const BBPLL_OC_ENB_FCAL_VAL: u32 = 0x9a;
const BBPLL_OC_ENB_VCON_VAL: u32 = 0x00;
const BBPLL_BBADC_CAL_7_0_VAL: u32 = 0x00;

const I2C_BBPLL_ENDIV5: u32 = 11;

const BBPLL_ENDIV5_VAL_320M: u32 = 0x43;
const BBPLL_BBADC_DSMP_VAL_320M: u32 = 0x84;
const BBPLL_ENDIV5_VAL_480M: u32 = 0xc3;
const BBPLL_BBADC_DSMP_VAL_480M: u32 = 0x74;

const I2C_BBPLL_BBADC_DSMP: u32 = 9;
const I2C_BBPLL_OC_LREF: u32 = 2;
const I2C_BBPLL_OC_DIV_7_0: u32 = 3;
const I2C_BBPLL_OC_DCUR: u32 = 5;

pub(crate) fn esp32_rtc_bbpll_configure(xtal_freq: XtalClock, pll_freq: PllClock) {
    let efuse = unsafe { &*crate::peripherals::EFUSE::ptr() };
    let rtc_cntl = unsafe { &*crate::peripherals::RTC_CNTL::ptr() };

    let rtc_cntl_dbias_hp_volt: u32 =
        RTC_CNTL_DBIAS_1V25 - efuse.blk0_rdata5().read().rd_vol_level_hp_inv().bits() as u32;
    let dig_dbias_240_m: u32 = rtc_cntl_dbias_hp_volt;

    let div_ref: u32;
    let div7_0: u32;
    let div10_8: u32;
    let lref: u32;
    let dcur: u32;
    let bw: u32;

    if matches!(pll_freq, PllClock::Pll320MHz) {
        // Raise the voltage, if needed
        rtc_cntl
            .reg()
            .modify(|_, w| unsafe { w.dig_dbias_wak().bits(DIG_DBIAS_80M_160M as u8) });

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

        regi2c_write!(I2C_BBPLL, I2C_BBPLL_ENDIV5, BBPLL_ENDIV5_VAL_320M);
        regi2c_write!(I2C_BBPLL, I2C_BBPLL_BBADC_DSMP, BBPLL_BBADC_DSMP_VAL_320M);
    } else {
        // Raise the voltage
        rtc_cntl
            .reg()
            .modify(|_, w| unsafe { w.dig_dbias_wak().bits(dig_dbias_240_m as u8) });

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

        regi2c_write!(I2C_BBPLL, I2C_BBPLL_ENDIV5, BBPLL_ENDIV5_VAL_480M);
        regi2c_write!(I2C_BBPLL, I2C_BBPLL_BBADC_DSMP, BBPLL_BBADC_DSMP_VAL_480M);
    }

    let i2c_bbpll_lref = (lref << 7) | (div10_8 << 4) | (div_ref);
    let i2c_bbpll_div_7_0 = div7_0;
    let i2c_bbpll_dcur = (bw << 6) | dcur;

    regi2c_write!(I2C_BBPLL, I2C_BBPLL_OC_LREF, i2c_bbpll_lref);
    regi2c_write!(I2C_BBPLL, I2C_BBPLL_OC_DIV_7_0, i2c_bbpll_div_7_0);
    regi2c_write!(I2C_BBPLL, I2C_BBPLL_OC_DCUR, i2c_bbpll_dcur);
}

pub(crate) fn esp32_rtc_bbpll_enable() {
    unsafe { &*crate::peripherals::RTC_CNTL::ptr() }
        .options0()
        .modify(|_, w| {
            w.bias_i2c_force_pd()
                .clear_bit()
                .bb_i2c_force_pd()
                .clear_bit()
                .bbpll_force_pd()
                .clear_bit()
                .bbpll_i2c_force_pd()
                .clear_bit()
        });

    // reset BBPLL configuration
    regi2c_write!(I2C_BBPLL, I2C_BBPLL_IR_CAL_DELAY, BBPLL_IR_CAL_DELAY_VAL);
    regi2c_write!(
        I2C_BBPLL,
        I2C_BBPLL_IR_CAL_EXT_CAP,
        BBPLL_IR_CAL_EXT_CAP_VAL
    );
    regi2c_write!(I2C_BBPLL, I2C_BBPLL_OC_ENB_FCAL, BBPLL_OC_ENB_FCAL_VAL);
    regi2c_write!(I2C_BBPLL, I2C_BBPLL_OC_ENB_VCON, BBPLL_OC_ENB_VCON_VAL);
    regi2c_write!(I2C_BBPLL, I2C_BBPLL_BBADC_CAL_7_0, BBPLL_BBADC_CAL_7_0_VAL);
}

pub(crate) fn esp32_rtc_update_to_xtal(freq: XtalClock, _div: u32) {
    let apb_cntl = unsafe { &*crate::peripherals::APB_CTRL::ptr() };
    let rtc_cntl = unsafe { &*crate::peripherals::RTC_CNTL::ptr() };

    unsafe {
        let value = (((freq.hz()) >> 12) & UINT16_MAX) | ((((freq.hz()) >> 12) & UINT16_MAX) << 16);
        esp32_update_cpu_freq(freq.hz());

        // set divider from XTAL to APB clock
        apb_cntl.sysclk_conf().modify(|_, w| {
            w.pre_div_cnt()
                .bits(((freq.hz()) / REF_CLK_FREQ - 1) as u16)
        });

        // adjust ref_tick
        apb_cntl.xtal_tick_conf().modify(|_, w| {
            w.xtal_tick_num()
                .bits(((freq.hz()) / REF_CLK_FREQ - 1) as u8)
        });

        // switch clock source
        rtc_cntl.clk_conf().modify(|_, w| w.soc_clk_sel().xtal());
        rtc_cntl.store5().modify(|_, w| w.scratch5().bits(value));

        // lower the voltage
        rtc_cntl
            .reg()
            .modify(|_, w| w.dig_dbias_wak().bits(DIG_DBIAS_XTAL as u8));
    }
}

pub(crate) fn set_cpu_freq(cpu_freq_mhz: crate::clock::CpuClock) {
    let efuse = unsafe { &*crate::peripherals::EFUSE::ptr() };
    let dport = unsafe { &*crate::peripherals::DPORT::ptr() };
    let rtc_cntl = unsafe { &*crate::peripherals::RTC_CNTL::ptr() };

    unsafe {
        const RTC_CNTL_DBIAS_1V25: u32 = 7;

        let rtc_cntl_dbias_hp_volt: u32 =
            RTC_CNTL_DBIAS_1V25 - efuse.blk0_rdata5().read().rd_vol_level_hp_inv().bits() as u32;
        let dig_dbias_240_m: u32 = rtc_cntl_dbias_hp_volt;

        const CPU_80M: u32 = 0;
        const CPU_160M: u32 = 1;
        const CPU_240M: u32 = 2;

        let mut dbias = DIG_DBIAS_80M_160M;
        let per_conf;

        match cpu_freq_mhz {
            crate::clock::CpuClock::_160MHz => {
                per_conf = CPU_160M;
            }
            crate::clock::CpuClock::_240MHz => {
                dbias = dig_dbias_240_m;
                per_conf = CPU_240M;
            }
            crate::clock::CpuClock::_80MHz => {
                per_conf = CPU_80M;
            }
        }

        let value = (((80 * MHZ) >> 12) & UINT16_MAX) | ((((80 * MHZ) >> 12) & UINT16_MAX) << 16);
        dport
            .cpu_per_conf()
            .write(|w| w.cpuperiod_sel().bits(per_conf as u8));
        rtc_cntl
            .reg()
            .modify(|_, w| w.dig_dbias_wak().bits(dbias as u8));
        rtc_cntl.clk_conf().modify(|_, w| w.soc_clk_sel().pll());
        rtc_cntl.store5().modify(|_, w| w.scratch5().bits(value));

        esp32_update_cpu_freq(cpu_freq_mhz.mhz());
    }
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
