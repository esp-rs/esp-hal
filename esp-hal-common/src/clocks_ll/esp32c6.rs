use paste::paste;

use crate::{
    clock::{ApbClock, Clock, CpuClock, PllClock, XtalClock},
    regi2c_write,
    regi2c_write_mask,
    rom::{ets_update_cpu_frequency, regi2c_ctrl_write_reg, regi2c_ctrl_write_reg_mask},
};

const I2C_BBPLL: u32 = 0x66;
const I2C_BBPLL_HOSTID: u32 = 0;

const I2C_BBPLL_IR_CAL_DELAY: u32 = 0;
const I2C_BBPLL_IR_CAL_DELAY_MSB: u32 = 3;
const I2C_BBPLL_IR_CAL_DELAY_LSB: u32 = 0;

const I2C_BBPLL_IR_CAL_CK_DIV: u32 = 0;
const I2C_BBPLL_IR_CAL_CK_DIV_MSB: u32 = 7;
const I2C_BBPLL_IR_CAL_CK_DIV_LSB: u32 = 4;

const I2C_BBPLL_IR_CAL_EXT_CAP: u32 = 1;
const I2C_BBPLL_IR_CAL_EXT_CAP_MSB: u32 = 3;
const I2C_BBPLL_IR_CAL_EXT_CAP_LSB: u32 = 0;

const I2C_BBPLL_IR_CAL_ENX_CAP: u32 = 1;
const I2C_BBPLL_IR_CAL_ENX_CAP_MSB: u32 = 4;
const I2C_BBPLL_IR_CAL_ENX_CAP_LSB: u32 = 4;

const I2C_BBPLL_IR_CAL_RSTB: u32 = 1;
const I2C_BBPLL_IR_CAL_RSTB_MSB: u32 = 5;
const I2C_BBPLL_IR_CAL_RSTB_LSB: u32 = 5;

const I2C_BBPLL_IR_CAL_START: u32 = 1;
const I2C_BBPLL_IR_CAL_START_MSB: u32 = 6;
const I2C_BBPLL_IR_CAL_START_LSB: u32 = 6;

const I2C_BBPLL_IR_CAL_UNSTOP: u32 = 1;
const I2C_BBPLL_IR_CAL_UNSTOP_MSB: u32 = 7;
const I2C_BBPLL_IR_CAL_UNSTOP_LSB: u32 = 7;

const I2C_BBPLL_OC_REF_DIV: u32 = 2;
const I2C_BBPLL_OC_REF_DIV_MSB: u32 = 3;
const I2C_BBPLL_OC_REF_DIV_LSB: u32 = 0;

const I2C_BBPLL_OC_DCHGP: u32 = 2;
const I2C_BBPLL_OC_DCHGP_MSB: u32 = 6;
const I2C_BBPLL_OC_DCHGP_LSB: u32 = 4;

const I2C_BBPLL_OC_ENB_FCAL: u32 = 2;
const I2C_BBPLL_OC_ENB_FCAL_MSB: u32 = 7;
const I2C_BBPLL_OC_ENB_FCAL_LSB: u32 = 7;

const I2C_BBPLL_OC_DIV_7_0: u32 = 3;
const I2C_BBPLL_OC_DIV_7_0_MSB: u32 = 7;
const I2C_BBPLL_OC_DIV_7_0_LSB: u32 = 0;

const I2C_BBPLL_RSTB_DIV_ADC: u32 = 4;
const I2C_BBPLL_RSTB_DIV_ADC_MSB: u32 = 0;
const I2C_BBPLL_RSTB_DIV_ADC_LSB: u32 = 0;

const I2C_BBPLL_MODE_HF: u32 = 4;
const I2C_BBPLL_MODE_HF_MSB: u32 = 1;
const I2C_BBPLL_MODE_HF_LSB: u32 = 1;

const I2C_BBPLL_DIV_ADC: u32 = 4;
const I2C_BBPLL_DIV_ADC_MSB: u32 = 3;
const I2C_BBPLL_DIV_ADC_LSB: u32 = 2;

const I2C_BBPLL_DIV_DAC: u32 = 4;
const I2C_BBPLL_DIV_DAC_MSB: u32 = 4;
const I2C_BBPLL_DIV_DAC_LSB: u32 = 4;

const I2C_BBPLL_DIV_CPU: u32 = 4;
const I2C_BBPLL_DIV_CPU_MSB: u32 = 5;
const I2C_BBPLL_DIV_CPU_LSB: u32 = 5;

const I2C_BBPLL_OC_ENB_VCON: u32 = 4;
const I2C_BBPLL_OC_ENB_VCON_MSB: u32 = 6;
const I2C_BBPLL_OC_ENB_VCON_LSB: u32 = 6;

const I2C_BBPLL_OC_TSCHGP: u32 = 4;
const I2C_BBPLL_OC_TSCHGP_MSB: u32 = 7;
const I2C_BBPLL_OC_TSCHGP_LSB: u32 = 7;

const I2C_BBPLL_OC_DR1: u32 = 5;
const I2C_BBPLL_OC_DR1_MSB: u32 = 2;
const I2C_BBPLL_OC_DR1_LSB: u32 = 0;

const I2C_BBPLL_OC_DR3: u32 = 5;
const I2C_BBPLL_OC_DR3_MSB: u32 = 6;
const I2C_BBPLL_OC_DR3_LSB: u32 = 4;

const I2C_BBPLL_EN_USB: u32 = 5;
const I2C_BBPLL_EN_USB_MSB: u32 = 7;
const I2C_BBPLL_EN_USB_LSB: u32 = 7;

const I2C_BBPLL_OC_DCUR: u32 = 6;
const I2C_BBPLL_OC_DCUR_MSB: u32 = 2;
const I2C_BBPLL_OC_DCUR_LSB: u32 = 0;

const I2C_BBPLL_INC_CUR: u32 = 6;
const I2C_BBPLL_INC_CUR_MSB: u32 = 3;
const I2C_BBPLL_INC_CUR_LSB: u32 = 3;

const I2C_BBPLL_OC_DHREF_SEL: u32 = 6;
const I2C_BBPLL_OC_DHREF_SEL_MSB: u32 = 5;
const I2C_BBPLL_OC_DHREF_SEL_LSB: u32 = 4;

const I2C_BBPLL_OC_DLREF_SEL: u32 = 6;
const I2C_BBPLL_OC_DLREF_SEL_MSB: u32 = 7;
const I2C_BBPLL_OC_DLREF_SEL_LSB: u32 = 6;

const I2C_BBPLL_OR_CAL_CAP: u32 = 8;
const I2C_BBPLL_OR_CAL_CAP_MSB: u32 = 3;
const I2C_BBPLL_OR_CAL_CAP_LSB: u32 = 0;

const I2C_BBPLL_OR_CAL_UDF: u32 = 8;
const I2C_BBPLL_OR_CAL_UDF_MSB: u32 = 4;
const I2C_BBPLL_OR_CAL_UDF_LSB: u32 = 4;

const I2C_BBPLL_OR_CAL_OVF: u32 = 8;
const I2C_BBPLL_OR_CAL_OVF_MSB: u32 = 5;
const I2C_BBPLL_OR_CAL_OVF_LSB: u32 = 5;

const I2C_BBPLL_OR_CAL_END: u32 = 8;
const I2C_BBPLL_OR_CAL_END_MSB: u32 = 6;
const I2C_BBPLL_OR_CAL_END_LSB: u32 = 6;

const I2C_BBPLL_OR_LOCK: u32 = 8;
const I2C_BBPLL_OR_LOCK_MSB: u32 = 7;
const I2C_BBPLL_OR_LOCK_LSB: u32 = 7;

const I2C_BBPLL_OC_VCO_DBIAS: u32 = 9;
const I2C_BBPLL_OC_VCO_DBIAS_MSB: u32 = 1;
const I2C_BBPLL_OC_VCO_DBIAS_LSB: u32 = 0;

const I2C_BBPLL_BBADC_DELAY2: u32 = 9;
const I2C_BBPLL_BBADC_DELAY2_MSB: u32 = 3;
const I2C_BBPLL_BBADC_DELAY2_LSB: u32 = 2;

const I2C_BBPLL_BBADC_DVDD: u32 = 9;
const I2C_BBPLL_BBADC_DVDD_MSB: u32 = 5;
const I2C_BBPLL_BBADC_DVDD_LSB: u32 = 4;

const I2C_BBPLL_BBADC_DREF: u32 = 9;
const I2C_BBPLL_BBADC_DREF_MSB: u32 = 7;
const I2C_BBPLL_BBADC_DREF_LSB: u32 = 6;

const I2C_BBPLL_BBADC_DCUR: u32 = 10;
const I2C_BBPLL_BBADC_DCUR_MSB: u32 = 1;
const I2C_BBPLL_BBADC_DCUR_LSB: u32 = 0;

const I2C_BBPLL_BBADC_INPUT_SHORT: u32 = 10;
const I2C_BBPLL_BBADC_INPUT_SHORT_MSB: u32 = 2;
const I2C_BBPLL_BBADC_INPUT_SHORT_LSB: u32 = 2;

const I2C_BBPLL_ENT_PLL: u32 = 10;
const I2C_BBPLL_ENT_PLL_MSB: u32 = 3;
const I2C_BBPLL_ENT_PLL_LSB: u32 = 3;

const I2C_BBPLL_DTEST: u32 = 10;
const I2C_BBPLL_DTEST_MSB: u32 = 5;
const I2C_BBPLL_DTEST_LSB: u32 = 4;

const I2C_BBPLL_ENT_ADC: u32 = 10;
const I2C_BBPLL_ENT_ADC_MSB: u32 = 7;
const I2C_BBPLL_ENT_ADC_LSB: u32 = 6;

pub(crate) fn esp32c6_rtc_bbpll_configure(xtal_freq: XtalClock, pll_freq: PllClock) {
    unsafe {
        let div_ref = 0u32;
        let div7_0 = 8u32;
        let dr1 = 0u32;
        let dr3 = 0u32;
        let dchgp = 5u32;
        let dcur = 3u32;
        let dbias = 2u32;

        let i2c_bbpll_lref = (dchgp << I2C_BBPLL_OC_DCHGP_LSB) | div_ref;
        let i2c_bbpll_div_7_0 = div7_0;
        let i2c_bbpll_dcur =
            (1 << I2C_BBPLL_OC_DLREF_SEL_LSB) | (3 << I2C_BBPLL_OC_DHREF_SEL_LSB) | dcur;

        regi2c_write!(I2C_BBPLL, I2C_BBPLL_OC_REF_DIV, i2c_bbpll_lref);
        regi2c_write!(I2C_BBPLL, I2C_BBPLL_OC_DIV_7_0, i2c_bbpll_div_7_0);
        regi2c_write_mask!(I2C_BBPLL, I2C_BBPLL_OC_DR1, dr1);
        regi2c_write_mask!(I2C_BBPLL, I2C_BBPLL_OC_DR3, dr3);
        regi2c_write!(I2C_BBPLL, I2C_BBPLL_OC_DCUR, i2c_bbpll_dcur);
        regi2c_write_mask!(I2C_BBPLL, I2C_BBPLL_OC_VCO_DBIAS, dbias);
    }
}

pub(crate) fn esp32c6_rtc_bbpll_enable() {
    let pmu = unsafe { &*crate::peripherals::PMU::PTR };

    pmu.imm_hp_ck_power.modify(|_, w| {
        w.tie_high_xpd_bb_i2c()
            .set_bit()
            .tie_high_xpd_bbpll()
            .set_bit()
            .tie_high_xpd_bbpll_i2c()
            .set_bit()
            .tie_high_global_bbpll_icg()
            .set_bit()
    });
}

pub(crate) fn esp32c6_rtc_update_to_xtal(freq: XtalClock, _div: u8) {
    // TODO
    let pcr =  unsafe { &*crate::peripherals::PCR::PTR };
    unsafe {
        ets_update_cpu_frequency(freq.mhz());
        // Set divider from XTAL to APB clock. Need to set divider to 1 (reg. value 0)
        // first.
        pcr.apb_freq_conf.modify(|_, w| {
            w.apb_div_num()
                .bits(0)
                .apb_div_num()
                .bits(_div - 1)
        });

        // Switch clock source
        pcr
            .sysclk_conf
            .modify(|_, w| w.soc_clk_sel().bits(0));
    }
}

pub(crate) fn esp32c6_rtc_apb_freq_update(apb_freq: ApbClock) {
    // TODO
    let lp_aon = unsafe { &*crate::peripherals::LP_AON::ptr() };
    let value = ((apb_freq.hz() >> 12) & u16::MAX as u32)
        | (((apb_freq.hz() >> 12) & u16::MAX as u32) << 16);

    lp_aon
        .store5
        .modify(|_, w| unsafe { w.lp_aon_store5().bits(value) });
}
