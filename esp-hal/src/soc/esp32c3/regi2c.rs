use crate::rom::regi2c::{define_regi2c, RawRegI2cField, RegI2cMaster, RegI2cRegister};

define_regi2c! {
    master: REGI2C_BBPLL(0x66, 0) {
        reg: I2C_BBPLL_IR_CAL(0) {
            field: I2C_BBPLL_IR_CAL_CK_DIV(7..4),
            field: I2C_BBPLL_IR_CAL_DELAY(3..0)
        }
        reg: I2C_BBPLL_IR_CAL_EXT_REG(1) {
            field: I2C_BBPLL_IR_CAL_UNSTOP(7..7),
            field: I2C_BBPLL_IR_CAL_START(6..6),
            field: I2C_BBPLL_IR_CAL_RSTB(5..5),
            field: I2C_BBPLL_IR_CAL_ENX_CAP(4..4),
            field: I2C_BBPLL_IR_CAL_EXT_CAP(3..0)
        }
        reg: I2C_BBPLL_OC_REF(2) {
            field: I2C_BBPLL_OC_ENB_FCAL(7..7),
            field: I2C_BBPLL_OC_DCHGP(6..4),
            field: I2C_BBPLL_OC_REF_DIV(3..0)
        }
        reg: I2C_BBPLL_OC_DIV_REG(3) {
            field: I2C_BBPLL_OC_DIV(7..0)
        }
        reg: I2C_BBPLL_REG4(4) {
            field: I2C_BBPLL_OC_TSCHGP(7..7),
            field: I2C_BBPLL_OC_ENB_VCON(6..6),
            field: I2C_BBPLL_DIV_CPU(5..5),
            field: I2C_BBPLL_DIV_DAC(4..4),
            field: I2C_BBPLL_DIV_ADC(3..2),
            field: I2C_BBPLL_MODE_HF(1..1),
            field: I2C_BBPLL_RSTB_DIV_ADC(0..0)
        }
        reg: I2C_BBPLL_OC_DR(5) {
            field: I2C_BBPLL_EN_USB(7..7),
            field: I2C_BBPLL_OC_DR3(6..4),
            field: I2C_BBPLL_OC_DR1(2..0)
        }
        reg: I2C_BBPLL_REG6(6) {
            field: I2C_BBPLL_OC_DLREF_SEL(7..6),
            field: I2C_BBPLL_OC_DHREF_SEL(5..4),
            field: I2C_BBPLL_INC_CUR(3..3),
            field: I2C_BBPLL_OC_DCUR(2..0)
        }
        reg: I2C_BBPLL_REG8(8) {
            field: I2C_BBPLL_OR_LOCK(7..7),
            field: I2C_BBPLL_OR_CAL_END(6..6),
            field: I2C_BBPLL_OR_CAL_OVF(5..5),
            field: I2C_BBPLL_OR_CAL_UDF(4..4),
            field: I2C_BBPLL_OR_CAL_CAP(3..0)
        }
        reg: I2C_BBPLL_REG9(9) {
            field: I2C_BBPLL_BBADC_DREF(7..6),
            field: I2C_BBPLL_BBADC_DVDD(5..4),
            field: I2C_BBPLL_BBADC_DELAY2(3..2),
            field: I2C_BBPLL_OC_VCO_DBIAS(1..0)
        }
        reg: I2C_BBPLL_REG10(10) {
            field: I2C_BBPLL_ENT_ADC(7..6),
            field: I2C_BBPLL_DTEST(5..4),
            field: I2C_BBPLL_ENT_PLL_MSB(3..3),
            field: I2C_BBPLL_BBADC_INPUT_SHORT(2..2),
            field: I2C_BBPLL_BBADC_DCUR(1..0)
        }
    }
    master: REGI2C_BIAS(0x6a, 0) {
        reg: I2C_BIAS_DREG(1) {
            field: I2C_BIAS_DREG_1P1_PVT(3..0)
        }
    }
    master: REGI2C_DIG_REG(0x6d, 0) {
        reg: I2C_DIG_REG4(4) {
            field: I2C_DIG_REG_ENX_RTC_DREG(7..7),
            field: I2C_DIG_REG_EXT_RTC_DREG(4..0)
        }
        reg: I2C_DIG_REG5(5) {
            field: I2C_DIG_REG_ENIF_RTC_DREG(7..7),
            field: I2C_DIG_REG_EXT_RTC_DREG_SLEEP(4..0)
        }
        reg: I2C_DIG_REG6(6) {
            field: I2C_DIG_REG_ENX_DIG_DREG(7..7),
            field: I2C_DIG_REG_EXT_DIG_DREG(4..0)
        }
        reg: I2C_DIG_REG_ENIF_DIG(7) {
            field: I2C_DIG_REG_ENIF_DIG_DREG(7..7),
            field: I2C_DIG_REG_EXT_DIG_DREG_SLEEP(4..0)
        }
        reg: I2C_DIG_REG9(9) {
            field: I2C_DIG_REG_OR_EN_CONT_CAL(7..7)
        }
        reg: I2C_DIG_REG_XPD(13) {
            field: I2C_DIG_REG_XPD_DIG_REG(3..3),
            field: I2C_DIG_REG_XPD_RTC_REG(2..2)
        }
        reg: I2C_DIG_REG_SCK_DCAP(14) {}
    }
    master: REGI2C_ULP_CAL(0x61, 0) {
        reg: I2C_ULP_CAL_IR(0) {
            field: I2C_ULP_IR_DISABLE_WATCHDOG_CK(6..6),
            field: I2C_ULP_IR_FORCE_XPD_IPH(4..4),
            field: I2C_ULP_IR_FORCE_XPD_CK(2..2),
            field: I2C_ULP_IR_RESETB(0..0)
        }
        reg: I2C_ULP_CAL_O(3) {
            field: I2C_ULP_BG_O_DONE_FLAG(3..3),
            field: I2C_ULP_O_DONE_FLAG(0..0)
        }
        reg: I2C_ULP_CAL_OCODE(4) {}
        reg: I2C_ULP_IR_FORCE(5) {
            field: I2C_ULP_IR_FORCE_CODE(6..6),
            field: I2C_BOD_THRESHOLD(2..0)
        }
        reg: I2C_ULP_EXT_CODE(6) {}
    }
    master: REGI2C_SAR_I2C(0x69, 0) {
        reg: I2C_SAR_REG0(0) {
            field: ADC_SAR1_INITIAL_CODE_LOW(7..0)
        }
        reg: I2C_SAR_REG1(1) {
            field: ADC_SAR1_INITIAL_CODE_HIGH(3..0)
        }
        reg: I2C_SAR_REG2(2) {
            field: ADC_SAR1_DREF(6..4)
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
        reg: I2C_SAR_REG6(6) {}
        reg: I2C_SAR_REG7(7) {
            field: ADC_SAR2_ENCAL_GND(7..7),
            field: ADC_SAR2_ENCAL_REF(6..6),
            field: ADC_SAR1_ENCAL_GND(5..5),
            field: ADC_SAR1_ENCAL_REF(4..4),
            field: ADC_SAR_ENT_RTC(3..3),
            field: ADC_SAR_ENT_TSENS(2..2),
            field: ADC_SAR_DTEST_RTC(1..0)
        }
    }
}

pub(crate) fn regi2c_read(block: u8, host_id: u8, reg_add: u8) -> u8 {
    unsafe extern "C" {
        pub(crate) fn esp_rom_regi2c_read(block: u8, block_hostid: u8, reg_add: u8) -> u8;
    }
    unsafe { esp_rom_regi2c_read(block, host_id, reg_add) }
}

pub(crate) fn regi2c_write(block: u8, host_id: u8, reg_add: u8, data: u8) {
    unsafe extern "C" {
        pub(crate) fn rom_i2c_writeReg(block: u8, block_hostid: u8, reg_add: u8, indata: u8);
    }
    unsafe { rom_i2c_writeReg(block, host_id, reg_add, data) };
}
