use crate::rom::regi2c::{define_regi2c, RegI2cMaster, RegI2cRegister};

define_regi2c! {
    master: REGI2C_BBPLL(0x66, 4) {
        reg: I2C_BBPLL_IR_CAL_DELAY(0) {}
        reg: I2C_BBPLL_IR_CAL_EXT_CAP(1) {}
        reg: I2C_BBPLL_OC_LREF(2) {}
        reg: I2C_BBPLL_OC_DIV_REG(3) {}
        reg: I2C_BBPLL_OC_ENB_FCAL(4) {}
        reg: I2C_BBPLL_OC_DCUR(5) {}
        reg: I2C_BBPLL_BBADC_DSMP(9) {}
        reg: I2C_BBPLL_OC_ENB_VCON(10) {}
        reg: I2C_BBPLL_ENDIV5(11) {}
        reg: I2C_BBPLL_BBADC_CAL_REG(12) {}
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
