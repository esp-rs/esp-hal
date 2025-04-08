use crate::{
    peripherals::{I2C_ANA_MST, SYSCON},
    ram,
    rom::regi2c::{define_regi2c, RawRegI2cField, RegI2cMaster, RegI2cRegister},
};

define_regi2c! {
    master: REGI2C_SAR(0x69, 1) {
        reg: I2C_SAR_REG0(0) {
            field: ADC_SAR1_INITIAL_CODE_LOW(7..0)
        }
        reg: I2C_SAR_REG1(1) {
            field: ADC_SAR1_INITIAL_CODE_HIGH(3..0)
        }
        reg: I2C_SAR_REG2(2) {
            field: ADC_SAR1_DREF(6..4),
            field: ADC_SAR1_SAMPLE_CYCLE(2..0)
        }
        reg: I2C_SAR_REG3(3) {
            field: ADC_SAR2_INITIAL_CODE_LOW(7..0)
        }
        reg: I2C_SAR_REG4(4) {
            field: ADC_SAR2_INITIAL_CODE_HIGH(3..0)
        }
        reg: I2C_SAR_REG5(5) {
            field: ADC_SAR2_DREF(6..4)
        }
        reg: I2C_SAR_REG6(6) {
            field: I2C_SARADC_TSENS_DAC(3..0)
        }
        reg: I2C_SAR_REG7(7) {
            field: ADC_SAR2_ENCAL_GND(7..7),
            field: ADC_SAR2_ENCAL_REF(6..6), // inferred, esp-idf doesn't have this
            field: ADC_SAR1_ENCAL_GND(5..5),
            field: ADC_SAR1_ENCAL_REF(4..4),
            field: ADC_SAR_ENT_RTC(3..3),
            field: ADC_SAR_ENT_TSENS(2..2),
            field: ADC_SAR_DTEST_RTC(1..0)
        }
    }
    master: REGI2C_BOD(0x61, 1) {
        reg: I2C_BOD_REG5(5) {
            field: I2C_BOD_REG_THRESHOLD(2..0)
        }
    }
    master: REGI2C_BBPLL(0x66, 1) {
        reg: I2C_BBPLL_REG0(0) {
            field: I2C_BBPLL_IR_CAL_DELAY(3..0),
            field: I2C_BBPLL_IR_CAL_CK_DIV(7..4)
        }
        reg: I2C_BBPLL_REG1(1) {
            field: I2C_BBPLL_IR_CAL_EXT_CAP(3..0),
            field: I2C_BBPLL_IR_CAL_ENX_CAP(4..4),
            field: I2C_BBPLL_IR_CAL_RSTB(5..5),
            field: I2C_BBPLL_IR_CAL_START(6..6),
            field: I2C_BBPLL_IR_CAL_UNSTOP(7..7)
        }
        reg: I2C_BBPLL_REG2(2) {
            field: I2C_BBPLL_OC_REF_DIV(3..0),
            field: I2C_BBPLL_OC_DCHGP(6..4),
            field: I2C_BBPLL_OC_ENB_FCAL(7..7)
        }
        reg: I2C_BBPLL_REG3(3) {
            field: I2C_BBPLL_OC_DIV_7_0(7..0)
        }
        reg: I2C_BBPLL_REG4(4) {
            field: I2C_BBPLL_RSTB_DIV_ADC(0..0),
            field: I2C_BBPLL_MODE_HF(1..1),
            field: I2C_BBPLL_DIV_ADC(3..2),
            field: I2C_BBPLL_DIV_DAC(4..4),
            field: I2C_BBPLL_DIV_CPU(5..5),
            field: I2C_BBPLL_OC_ENB_VCON(6..6),
            field: I2C_BBPLL_OC_TSCHGP(7..7)
        }
        reg: I2C_BBPLL_REG5(5) {
            field: I2C_BBPLL_OC_DR1(2..0),
            field: I2C_BBPLL_OC_DR3(6..4),
            field: I2C_BBPLL_EN_USB(7..7)
        }
        reg: I2C_BBPLL_REG6(6) {
            field: I2C_BBPLL_OC_DCUR(2..0),
            field: I2C_BBPLL_INC_CUR(3..3),
            field: I2C_BBPLL_OC_DHREF_SEL(5..4),
            field: I2C_BBPLL_OC_DLREF_SEL(7..6)
        }
        reg: I2C_BBPLL_REG8(8) {
            field: I2C_BBPLL_OR_CAL_CAP(3..0),
            field: I2C_BBPLL_OR_CAL_UDF(4..4),
            field: I2C_BBPLL_OR_CAL_OVF(5..5),
            field: I2C_BBPLL_OR_CAL_END(6..6),
            field: I2C_BBPLL_OR_LOCK(7..7)
        }
        reg: I2C_BBPLL_REG9(9) {
            field: I2C_BBPLL_BBADC_DELAY2(3..2),
            field: I2C_BBPLL_BBADC_DVDD(5..4),
            field: I2C_BBPLL_BBADC_DREF(7..6)
        }
        reg: I2C_BBPLL_REG10(10) {
            field: I2C_BBPLL_BBADC_DCUR(1..0),
            field: I2C_BBPLL_BBADC_INPUT_SHORT(2..2),
            field: I2C_BBPLL_ENT_PLL(3..3),
            field: I2C_BBPLL_DTEST(5..4),
            field: I2C_BBPLL_ENT_ADC(7..6)
        }
    }
    master: REGI2C_APLL(0x6D, 1) {
        reg: I2C_APLL_REG0(0) {
            field: I2C_APLL_IR_CAL_DELAY(3..0),
            field: I2C_APLL_IR_CAL_RSTB(4..4),
            field: I2C_APLL_IR_CAL_START(5..5),
            field: I2C_APLL_IR_CAL_UNSTOP(6..6),
            field: I2C_APLL_OC_ENB_FCAL(7..7)
        }
        reg: I2C_APLL_REG1(1) {
            field: I2C_APLL_IR_CAL_EXT_CAP(4..0),
            field: I2C_APLL_IR_CAL_ENX_CAP(5..5),
            field: I2C_APLL_OC_LBW(6..6)
        }
        reg: I2C_APLL_REG2(2) {
            field: I2C_APLL_IR_CAL_CK_DIV(3..0),
            field: I2C_APLL_OC_DCHGP(6..4),
            field: I2C_APLL_OC_ENB_VCON(7..7)
        }
        reg: I2C_APLL_REG3(3) {
            field: I2C_APLL_OR_CAL_CAP(4..0),
            field: I2C_APLL_OR_CAL_UDF(5..5),
            field: I2C_APLL_OR_CAL_OVF(6..6),
            field: I2C_APLL_OR_CAL_END(7..7)
        }
        reg: I2C_APLL_REG4(4) {
            field: I2C_APLL_OR_OUTPUT_DIV(4..0),
            field: I2C_APLL_OC_TSCHGP(6..6),
            field: I2C_APLL_EN_FAST_CAL(7..7)
        }
        reg: I2C_APLL_REG5(5) {
            field: I2C_APLL_OC_DHREF_SEL(1..0),
            field: I2C_APLL_OC_DLREF_SEL(3..2),
            field: I2C_APLL_SDM_DITHER(4..4),
            field: I2C_APLL_SDM_STOP(5..5),
            field: I2C_APLL_SDM_RSTB(6..6)
        }
        reg: I2C_APLL_REG6(6) {
            field: I2C_APLL_OC_DVDD(4..0)
        }
        reg: I2C_APLL_REG7(7) {
            field: I2C_APLL_DSDM2(5..0)
        }
        reg: I2C_APLL_REG8(8) {
            field: I2C_APLL_DSDM1(7..0)
        }
        reg: I2C_APLL_REG9(9) {
            field: I2C_APLL_DSDM0(7..0)
        }
    }
}
#[ram]
pub unsafe fn i2c_rtc_enable_block(block: u8) {
    I2C_ANA_MST::regs().config0().modify(|_, w| unsafe {
        const MAGIC_DEFAULT: u16 = 0x1c40;
        w.magic_ctrl().bits(MAGIC_DEFAULT)
    }); // 0x1c40 = MAGIC_DEFAULT
    I2C_ANA_MST::regs().config1().modify(|_, w| unsafe {
        const ALL_MASK_V: u16 = 0x7FFF;
        w.all_mask().bits(ALL_MASK_V)
    });

    SYSCON::regs()
        .wifi_clk_en()
        .modify(|_, w| w.mac_clk_en().set_bit());

    match block {
        v if v == REGI2C_APLL.master => I2C_ANA_MST::regs()
            .config1()
            .modify(|_, w| w.apll().clear_bit()),
        v if v == REGI2C_BBPLL.master => I2C_ANA_MST::regs()
            .config1()
            .modify(|_, w| w.bbpll().clear_bit()),
        v if v == REGI2C_SAR.master => I2C_ANA_MST::regs()
            .config1()
            .modify(|_, w| w.sar().clear_bit()),
        v if v == REGI2C_BOD.master => I2C_ANA_MST::regs()
            .config1()
            .modify(|_, w| w.bod().clear_bit()),
        _ => unreachable!(),
    };
}

#[ram]
pub(crate) fn regi2c_read(block: u8, _host_id: u8, reg_add: u8) -> u8 {
    unsafe { i2c_rtc_enable_block(block) };

    I2C_ANA_MST::regs()
        .config2()
        .modify(|_, w| unsafe { w.slave_id().bits(block).addr().bits(reg_add) });

    while I2C_ANA_MST::regs().config2().read().busy().bit_is_set() {}

    I2C_ANA_MST::regs().config2().read().data().bits()
}

#[ram]
pub(crate) fn regi2c_write(block: u8, _host_id: u8, reg_add: u8, data: u8) {
    unsafe { i2c_rtc_enable_block(block) };

    I2C_ANA_MST::regs().config2().modify(|_, w| unsafe {
        w.slave_id()
            .bits(block)
            .addr()
            .bits(reg_add)
            .wr_cntl()
            .bit(true)
            .data()
            .bits(data)
    });

    while I2C_ANA_MST::regs().config2().read().busy().bit_is_set() {}
}
