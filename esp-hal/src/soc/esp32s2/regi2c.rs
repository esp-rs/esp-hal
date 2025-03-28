use crate::rom::regi2c::{define_regi2c, RawRegI2cField, RegI2cMaster, RegI2cRegister};

define_regi2c! {
    master: REGI2C_SAR_I2C(0x69, 1) {
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
}

pub(crate) fn regi2c_read(block: u8, host_id: u8, reg_add: u8) -> u8 {
    extern "C" {
        pub(crate) fn esp_rom_regi2c_read(block: u8, block_hostid: u8, reg_add: u8) -> u8;
    }
    unsafe { esp_rom_regi2c_read(block, host_id, reg_add) }
}

pub(crate) fn regi2c_write(block: u8, host_id: u8, reg_add: u8, data: u8) {
    extern "C" {
        pub(crate) fn rom_i2c_writeReg(block: u8, block_hostid: u8, reg_add: u8, indata: u8);
    }
    unsafe { rom_i2c_writeReg(block, host_id, reg_add, data) };
}
