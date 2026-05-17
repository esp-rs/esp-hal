use crate::rom::regi2c::{RawRegI2cField, RegI2cMaster, RegI2cRegister, define_regi2c};

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
    // Audio PLL (APLL). Block 0x6D, host ID 3.
    // Field bit ranges are inclusive [msb..lsb] per `RawRegI2cField::new`.
    // Source of truth: components/soc/esp32/include/soc/regi2c_apll.h
    master: REGI2C_APLL(0x6D, 3) {
        // Register 0 — IR calibration control / status
        reg: I2C_APLL_IR_CAL(0) {
            field: I2C_APLL_OC_ENB_FCAL(7..7),
            field: I2C_APLL_IR_CAL_UNSTOP(6..6),
            field: I2C_APLL_IR_CAL_START(5..5),
            field: I2C_APLL_IR_CAL_RSTB(4..4),
            field: I2C_APLL_IR_CAL_DELAY(3..0)
        }
        // Register 1 — IR calibration external capacitor
        reg: I2C_APLL_EXT_CAP(1) {
            field: I2C_APLL_OC_LBW(6..6),
            field: I2C_APLL_IR_CAL_ENX_CAP(5..5),
            field: I2C_APLL_IR_CAL_EXT_CAP(4..0)
        }
        // Register 2 — OC control / IR cal clock divider
        reg: I2C_APLL_OC_REG(2) {
            field: I2C_APLL_OC_ENB_VCON(7..7),
            field: I2C_APLL_OC_DCHGP(6..4),
            field: I2C_APLL_IR_CAL_CK_DIV(3..0)
        }
        // Register 3 — IR calibration output / read-back
        reg: I2C_APLL_OR_CAL(3) {
            field: I2C_APLL_OR_CAL_END(7..7),
            field: I2C_APLL_OR_CAL_OVF(6..6),
            field: I2C_APLL_OR_CAL_UDF(5..5),
            field: I2C_APLL_OR_CAL_CAP(4..0)
        }
        // Register 4 — output divider + fast-cal enable
        // Named I2C_APLL_DIV_REG to avoid clashing with the I2C_APLL_OR_OUTPUT_DIV
        // field constant that lives inside it.
        reg: I2C_APLL_DIV_REG(4) {
            field: I2C_APLL_EN_FAST_CAL(7..7),
            field: I2C_APLL_OC_TSCHGP(6..6),
            field: I2C_APLL_OR_OUTPUT_DIV(4..0)
        }
        // Register 5 — SDM control (reset, stop, dither) + reference selects
        reg: I2C_APLL_SDM_CTRL(5) {
            field: I2C_APLL_SDM_RSTB(6..6),
            field: I2C_APLL_SDM_STOP(5..5),
            field: I2C_APLL_SDM_DITHER(4..4),
            field: I2C_APLL_OC_DLREF_SEL(3..2),
            field: I2C_APLL_OC_DHREF_SEL(1..0)
        }
        // Register 6 — OC VDD
        reg: I2C_APLL_OC_DVDD_REG(6) {
            field: I2C_APLL_OC_DVDD(4..0)
        }
        // Registers 7–9 — SDM Σ-Δ modulator coefficients.
        // Named *_REG to avoid clashing with the field constants of the same
        // ESP-IDF name that live inside each register.
        reg: I2C_APLL_DSDM2_REG(7) {
            field: I2C_APLL_DSDM2(5..0)
        }
        reg: I2C_APLL_DSDM1_REG(8) {
            field: I2C_APLL_DSDM1(7..0)
        }
        reg: I2C_APLL_DSDM0_REG(9) {
            field: I2C_APLL_DSDM0(7..0)
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
