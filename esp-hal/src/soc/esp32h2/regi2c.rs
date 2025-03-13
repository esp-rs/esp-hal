use crate::{
    peripherals::{I2C_ANA_MST, MODEM_LPCON},
    rom::regi2c::{define_regi2c, RawRegI2cField, RegI2cMaster, RegI2cRegister},
};

define_regi2c! {
    master: REGI2C_BBPLL(0x66, 0) {
        reg: I2C_BBPLL_OC_REF(2) {
            field: I2C_BBPLL_OC_REF_DIV(3..0)
        }
        reg: I2C_BBPLL_OC_DIV_REG(3) {
            field: I2C_BBPLL_OC_DIV(5..0)
        }
        reg: I2C_BBPLL_OC_DR(5) {
            field: I2C_BBPLL_OC_DLREF_SEL(7..6),
            field: I2C_BBPLL_OC_DHREF_SEL(5..4)
        }
    }
    master: REGI2C_BIAS(0x6a, 0) {
        reg: I2C_BIAS_DREG0(0) {
            field: I2C_BIAS_DREG_0P8(7..4)
        }
        reg: I2C_BIAS_DREG1(1) {
            field: I2C_BIAS_DREG_1P1_PVT(3..0)
        }
    }
    master: REGI2C_PMU_REG(0x6d, 0) {
        reg: I2C_PMU_REG8(8) {
            field: I2C_PMU_EN_I2C_DIG_DREG_SLP(3..3),
            field: I2C_PMU_EN_I2C_RTC_DREG_SLP(2..2),
            field: I2C_PMU_EN_I2C_DIG_DREG(1..1),
            field: I2C_PMU_EN_I2C_RTC_DREG(0..0)
        }
        reg: I2C_PMU_XPD(9) {
            field: I2C_PMU_OR_XPD_DIG_REG(6..6),
            field: I2C_PMU_OR_XPD_RTC_REG(4..4)
        }
        reg: I2C_PMU_OC_SCK_DCAP(14) {}
        reg: I2C_PMU_REG15(15) {
            field: I2C_PMU_OR_XPD_TRX(2..2)
        }
        reg: I2C_PMU_REG21(21) {
            field: I2C_PMU_SEL_PLL8M_REF(6..6)
        }
    }
    master: REGI2C_ULP_CAL(0x61, 0) {
        reg: I2C_ULP_CAL_IR(0) {
            field: I2C_ULP_IR_RESETB(0..0)
        }
    }
    master: REGI2C_SAR_I2C(0x69, 0) {
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
            field: ADC_SARADC_TSENS_DAC(3..0)
        }
        reg: I2C_SAR_REG7(7) {
            field: ADC_SARADC_EN_TOUT_SAR1_BUS(5..5),
            field: ADC_SARADC_ENT_SAR(3..1),
            field: ADC_SARADC_DTEST(1..0)
        }
        reg: I2C_SAR_REG8(8) {
            field: ADC_SAR1_ENCAL_GND(1..1)
        }
    }
}

fn regi2c_enable_block(block: u8) -> usize {
    MODEM_LPCON::regs()
        .clk_conf()
        .modify(|_, w| w.clk_i2c_mst_en().set_bit());

    // Before config I2C register, enable corresponding slave.
    let i2c_sel_bits = I2C_ANA_MST::regs().ana_conf2().read();
    let i2c_sel = match block {
        v if v == REGI2C_BBPLL.master => i2c_sel_bits.bbpll_mst_sel().bit_is_set(),
        v if v == REGI2C_BIAS.master => i2c_sel_bits.bias_mst_sel().bit_is_set(),
        v if v == REGI2C_PMU_REG.master => i2c_sel_bits.dig_reg_mst_sel().bit_is_set(),
        v if v == REGI2C_ULP_CAL.master => i2c_sel_bits.ulp_cal_mst_sel().bit_is_set(),
        v if v == REGI2C_SAR_I2C.master => i2c_sel_bits.sar_i2c_mst_sel().bit_is_set(),
        _ => unreachable!(),
    };
    I2C_ANA_MST::regs().ana_conf1().write(|w| unsafe {
        const I2C_MST_ANA_CONF1_M: u32 = 0x00FFFFFF;
        w.bits(I2C_MST_ANA_CONF1_M);
        match block {
            v if v == REGI2C_BBPLL.master => w.bbpll_rd().clear_bit(),
            v if v == REGI2C_BIAS.master => w.bias_rd().clear_bit(),
            v if v == REGI2C_PMU_REG.master => w.dig_reg_rd().clear_bit(),
            v if v == REGI2C_ULP_CAL.master => w.ulp_cal_rd().clear_bit(),
            v if v == REGI2C_SAR_I2C.master => w.sar_i2c_rd().clear_bit(),
            _ => unreachable!(),
        }
    });

    if i2c_sel {
        0
    } else {
        1
    }
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

    I2C_ANA_MST::regs().i2c_ctrl(master).write(|w| unsafe {
        w.slave_addr().bits(block);
        w.slave_reg_addr().bits(reg_add);
        w.read_write().set_bit();
        w.data().bits(data)
    });

    while I2C_ANA_MST::regs().i2c_ctrl(master).read().busy().bit() {}
}
