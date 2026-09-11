//! Analog register access over the internal I2C master.
//!
//! Each analog slave is reachable through one of the two I2C masters. `ANA_CONF2` says which one,
//! `ANA_CONF1` selects the slave itself.

use crate::{
    peripherals::{I2C_ANA_MST, MODEM_LPCON, MODEM_SYSCON},
    rom::regi2c::{RawRegI2cField, RegI2cMaster, RegI2cRegister, define_regi2c},
};

define_regi2c! {
    master: REGI2C_ULP(0x61, 0) {
        reg: I2C_ULP_IR(0) {
            field: I2C_ULP_IR_FORCE_XPD_IPH(4..4),
            field: I2C_ULP_IR_FORCE_XPD_CK(2..2),
            field: I2C_ULP_IR_RESETB(0..0)
        }
        reg: I2C_ULP_O(3) {
            field: I2C_ULP_BG_O_DONE_FLAG(3..3),
            field: I2C_ULP_O_DONE_FLAG(0..0)
        }
        reg: I2C_ULP_OCODE_REG(4) {
            field: I2C_ULP_OCODE(7..0)
        }
        reg: I2C_ULP_IR_FORCE(5) {
            field: I2C_ULP_IR_FORCE_CODE(3..3)
        }
        reg: I2C_ULP_EXT_CODE_REG(6) {
            field: I2C_ULP_EXT_CODE(7..0)
        }
        reg: I2C_ULP_CPREG(8) {
            field: I2C_ULP_CPREG_DREG1P1(4..3),
            field: I2C_ULP_CPREG_DREG(2..0)
        }
    }
    master: REGI2C_PLL(0x62, 0) {}
    master: REGI2C_SDM(0x63, 0) {}
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
            field: I2C_BBPLL_OC_REF_DIV(3..0)
        }
        reg: I2C_BBPLL_OC_DIV_REG(3) {
            field: I2C_BBPLL_OC_DIV(5..0)
        }
        reg: I2C_BBPLL_REG4(4) {
            field: I2C_BBPLL_OC_TSCHGP(6..6)
        }
        reg: I2C_BBPLL_REG5(5) {
            field: I2C_BBPLL_OC_DLREF_SEL(7..6),
            field: I2C_BBPLL_OC_DHREF_SEL(5..4)
        }
        reg: I2C_BBPLL_REG8(8) {
            field: I2C_BBPLL_OR_LOCK(7..7),
            field: I2C_BBPLL_OR_CAL_END(6..6),
            field: I2C_BBPLL_OR_CAL_OVF(5..5),
            field: I2C_BBPLL_OR_CAL_UDF(4..4),
            field: I2C_BBPLL_OR_CAL_CAP(3..0)
        }
        reg: I2C_BBPLL_REG10(10) {
            field: I2C_BBPLL_ENT_PLL(2..2),
            field: I2C_BBPLL_DTEST(1..0)
        }
    }
    master: REGI2C_BBTOP(0x67, 0) {}
    master: REGI2C_PERIF(0x69, 0) {}
    master: REGI2C_DCDC(0x6d, 0) {
        reg: I2C_DCDC_REG1(1) {
            field: I2C_DCDC_XPD_TRX(7..7)
        }
        reg: I2C_DCDC_CCM(7) {
            field: I2C_DCDC_CCM_PCUR_LIMIT0(7..5),
            field: I2C_DCDC_CCM_DREG0(4..0)
        }
        reg: I2C_DCDC_VCM(10) {
            field: I2C_DCDC_VCM_PCUR_LIMIT0(7..5),
            field: I2C_DCDC_VCM_DREG0(4..0)
        }
        reg: I2C_BOD(13) {
            field: I2C_BOD_THRESHOLD(6..3)
        }
    }
}

/// The `ANA_CONF1` bit that selects a slave, see the `REGI2C_CONF1_*_SEL` bit list in esp-idf's
/// `i2c_ana_mst_reg.h`. The matching `ANA_CONF2` bits sit two positions higher.
fn slave_select_bit(block: u8) -> u32 {
    match block {
        v if v == REGI2C_BBTOP.master => 2,
        v if v == REGI2C_SDM.master => 4,
        v if v == REGI2C_PLL.master => 5,
        v if v == REGI2C_BBPLL.master => 7,
        v if v == REGI2C_ULP.master => 8,
        v if v == REGI2C_PERIF.master => 9,
        v if v == REGI2C_DCDC.master => 10,
        _ => unreachable!(),
    }
}

fn regi2c_enable_block(block: u8) -> usize {
    // esp-idf only asserts that the master clock is running (the modem clock driver turns it on),
    // so enable it here. The clock is also lost across sleep with the top domain powered down,
    // which needs the ICG bitmaps that the sleep code will have to set up.
    MODEM_SYSCON::regs()
        .clk_conf()
        .modify(|_, w| w.clk_i2c_mst_sel_160m().set_bit());
    MODEM_LPCON::regs()
        .clk_conf()
        .modify(|_, w| w.clk_i2c_mst_en().set_bit());

    let select_bit = slave_select_bit(block);

    // A set bit means the slave is wired to I2C0, a clear bit means I2C1.
    const CONF2_SELECT_SHIFT: u32 = 2;
    let uses_i2c0 = I2C_ANA_MST::regs().ana_conf2().read().ana_conf2().bits()
        & (1 << (select_bit + CONF2_SELECT_SHIFT))
        != 0;

    // Enabling a slave means clearing its bit while leaving every other one set.
    const ANA_CONF1_MASK: u32 = 0x00FF_FFFF;
    I2C_ANA_MST::regs()
        .ana_conf1()
        .write(|w| unsafe { w.ana_conf1().bits(!(1 << select_bit) & ANA_CONF1_MASK) });

    if uses_i2c0 { 0 } else { 1 }
}

pub(crate) fn regi2c_read(block: u8, _host_id: u8, reg_add: u8) -> u8 {
    let master = regi2c_enable_block(block);

    while I2C_ANA_MST::regs().i2c_ctrl(master).read().busy().bit() {}

    I2C_ANA_MST::regs().i2c_ctrl(master).write(|w| unsafe {
        w.slave_addr().bits(block);
        w.slave_reg_addr().bits(reg_add)
    });

    while I2C_ANA_MST::regs().i2c_ctrl(master).read().busy().bit() {}

    I2C_ANA_MST::regs().i2c_ctrl(master).read().data().bits()
}

pub(crate) fn regi2c_write(block: u8, _host_id: u8, reg_add: u8, data: u8) {
    let master = regi2c_enable_block(block);

    while I2C_ANA_MST::regs().i2c_ctrl(master).read().busy().bit() {}

    I2C_ANA_MST::regs().i2c_ctrl(master).write(|w| unsafe {
        w.slave_addr().bits(block);
        w.slave_reg_addr().bits(reg_add);
        w.read_write().set_bit();
        w.data().bits(data)
    });

    while I2C_ANA_MST::regs().i2c_ctrl(master).read().busy().bit() {}
}
