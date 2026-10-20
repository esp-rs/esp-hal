use crate::{
    peripherals::{MODEM_LPCON, MODEM_SYSCON},
    rom::regi2c::{RawRegI2cField, RegI2cMaster, RegI2cRegister, define_regi2c},
};

define_regi2c! {
    master: REGI2C_BB(0x67, 0) {}
    master: REGI2C_TXRF(0x6B, 0) {}
    master: REGI2C_SDM(0x63, 0) {}
    master: REGI2C_RFPLL(0x62, 0) {}
    master: REGI2C_BIAS(0x6A, 0) {
        reg: I2C_BIAS_REG0(0) {
            field: I2C_BIAS_DREG_1P6(3..0),
            field: I2C_BIAS_DREG_1P1(7..4)
        }
        reg: I2C_BIAS_DREG(1) {
            field: I2C_BIAS_DREG_1P1_PVT(3..0),
            field: I2C_BIAS_DREG_2P2_PVT(7..4)
        }
    }
    master: REGI2C_BBPLL(0x66, 0) {
        reg: I2C_BBPLL_IR_CAL(0) {
            field: I2C_BBPLL_IR_CAL_DELAY(3..0),
            field: I2C_BBPLL_IR_CAL_CK_DIV(7..4)
        }
        reg: I2C_BBPLL_IR_CAL_EXT_REG(1) {
            field: I2C_BBPLL_IR_CAL_EXT_CAP(3..0),
            field: I2C_BBPLL_IR_CAL_ENX_CAP(4..4),
            field: I2C_BBPLL_IR_CAL_RSTB(5..5),
            field: I2C_BBPLL_IR_CAL_START(6..6),
            field: I2C_BBPLL_IR_CAL_UNSTOP(7..7)
        }
        reg: I2C_BBPLL_OC_REF(2) {
            field: I2C_BBPLL_OC_REF_DIV(3..0),
            field: I2C_BBPLL_OC_DCHGP(6..4)
        }
        reg: I2C_BBPLL_OC_DIV_REG(3) {
            field: I2C_BBPLL_OC_DIV_7_0(7..0)
        }
        reg: I2C_BBPLL_REG4(4) {
            field: I2C_BBPLL_RSTB_DIV_ADC(0..0),
            field: I2C_BBPLL_MODE_HF(1..1),
            field: I2C_BBPLL_DIV_DAC(4..4),
            field: I2C_BBPLL_DIV_CPU(5..5)
        }
        reg: I2C_BBPLL_OC_DR(5) {
            field: I2C_BBPLL_OC_DR1(2..0),
            field: I2C_BBPLL_OC_DR3(6..4),
            field: I2C_BBPLL_EN_USB(7..7)
        }
        reg: I2C_BBPLL_REG6(6) {
            field: I2C_BBPLL_OC_DHREF_SEL(5..4),
            field: I2C_BBPLL_OC_DLREF_SEL(7..6)
        }
        reg: I2C_BBPLL_REG8(8) {
            field: I2C_BBPLL_OR_CAL_UDF(4..4),
            field: I2C_BBPLL_OR_CAL_OVF(5..5),
            field: I2C_BBPLL_OR_CAL_END(6..6),
            field: I2C_BBPLL_OR_LOCK(7..7)
        }
        reg: I2C_BBPLL_REG9(9) {
            field: I2C_BBPLL_OC_VCO_DBIAS(1..0)
        }
        reg: I2C_BBPLL_REG10(10) {
            field: I2C_BBPLL_ENT_PLL(3..3),
            field: I2C_BBPLL_DTEST(5..4)
        }
    }
    master: REGI2C_ULP(0x61, 0) {
        reg: I2C_ULP_REG5(5) {
            field: I2C_BOD_THRESHOLD(2..0)
        }
    }
    master: REGI2C_SAR_MASTER(0x10, 0) {}
    master: REGI2C_SAR_SLAVE(0x11, 0) {}
    master: REGI2C_PERIF(0x69, 0) {
        reg: I2C_SAR_REG0(0) {
            field: ADC_SAR1_INIT_CODE_LSB(7..0)
        }
        reg: I2C_SAR_REG1(1) {
            field: ADC_SAR1_INIT_CODE_MSB(3..0)
        }
        reg: I2C_SAR_REG2(2) {
            field: ADC_SAR1_SAMPLE_CYCLE(2..0),
            field: ADC_SAR1_DREF(6..4)
        }
        reg: I2C_SAR_REG3(3) {
            field: ADC_SAR2_INITIAL_CODE_LOW(7..0)
        }
        reg: I2C_SAR_REG4(4) {
            field: ADC_SAR2_INITIAL_CODE_HIGH(3..0)
        }
        reg: I2C_SAR_REG5(5) {
            field: ADC_SAR2_SAMPLE_CYCLE(2..0),
            field: ADC_SAR2_DREF(6..4)
        }
        reg: I2C_SAR_REG6(6) {
            field: I2C_SARADC_TSENS_DAC(3..0)
        }
        reg: I2C_SAR_REG7(7) {
            field: ADC_SARADC_DTEST_RTC(1..0),
            field: ADC_SARADC_ENT_TSENS(2..2)
        }
    }
    master: REGI2C_APLL(0x0C, 0) {
        reg: I2C_APLL_IR_CAL(0) {
            field: I2C_APLL_IR_CAL_DELAY(3..0),
            field: I2C_APLL_IR_CAL_RSTB(4..4),
            field: I2C_APLL_IR_CAL_START(5..5),
            field: I2C_APLL_IR_CAL_UNSTOP(6..6),
            field: I2C_APLL_OC_ENB_FCAL(7..7)
        }
        reg: I2C_APLL_IR_CAL_EXT_REG(1) {
            field: I2C_APLL_IR_CAL_EXT_CAP(4..0),
            field: I2C_APLL_IR_CAL_ENX_CAP(5..5),
            field: I2C_APLL_OC_LBW(6..6)
        }
        reg: I2C_APLL_OC_REF(2) {
            field: I2C_APLL_IR_CAL_CK_DIV(3..0),
            field: I2C_APLL_OC_DCHGP(6..4),
            field: I2C_APLL_OC_ENB_VCON(7..7)
        }
        reg: I2C_APLL_OC_CAL(3) {
            field: I2C_APLL_OR_CAL_CAP(4..0),
            field: I2C_APLL_OR_CAL_UDF(5..5),
            field: I2C_APLL_OR_CAL_OVF(6..6),
            field: I2C_APLL_OR_CAL_END(7..7)
        }
        reg: I2C_APLL_OC_TSCHGP_REG(4) {
            field: I2C_APLL_OR_OUTPUT_DIV(4..0),
            field: I2C_APLL_OC_TSCHGP(6..6),
            field: I2C_APLL_EN_FAST_CAL(7..7)
        }
        reg: I2C_APLL_SDM(5) {
            field: I2C_APLL_OC_DHREF_SEL(1..0),
            field: I2C_APLL_OC_DLREF_SEL(3..2),
            field: I2C_APLL_SDM_DITHER(4..4),
            field: I2C_APLL_SDM_STOP(5..5),
            field: I2C_APLL_SDM_RSTB(6..6)
        }
        reg: I2C_APLL_OC_DVDD_REG(6) {
            field: I2C_APLL_OC_DVDD(4..0),
            field: I2C_APLL_OC_REF_DIV(7..5)
        }
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
    master: REGI2C_CPLL(0x0A, 0) {}
    master: REGI2C_MPLL(0x0B, 0) {}
    master: REGI2C_DIG_REG(0x6D, 0) {
        reg: I2C_DIG_REG4(4) {
            field: I2C_DIG_REG_EXT_RTC_DREG(4..0),
            field: I2C_DIG_REG_ENX_RTC_DREG(7..7)
        }
        reg: I2C_DIG_REG5(5) {
            field: I2C_DIG_REG_EXT_RTC_DREG_SLEEP(4..0),
            field: I2C_DIG_REG_ENIF_RTC_DREG(7..7)
        }
        reg: I2C_DIG_REG6(6) {
            field: I2C_DIG_REG_EXT_DIG_DREG(4..0),
            field: I2C_DIG_REG_ENX_DIG_DREG(7..7)
        }
        reg: I2C_DIG_REG_ENIF_DIG(7) {
            field: I2C_DIG_REG_EXT_DIG_DREG_SLEEP(4..0),
            field: I2C_DIG_REG_ENIF_DIG_DREG(7..7)
        }
        reg: I2C_DIG_REG9(9) {
            field: I2C_DIG_REG_OR_EN_CONT_CAL(7..7)
        }
        reg: I2C_DIG_REG_XPD(13) {
            field: I2C_DIG_REG_XPD_RTC_REG(2..2),
            field: I2C_DIG_REG_XPD_DIG_REG(3..3)
        }
        reg: I2C_DIG_REG_SCK_DCAP_REG(14) {
            field: I2C_DIG_REG_SCK_DCAP(7..0)
        }
    }
}

const REGI2C_BB_MASK: u32 = 1 << 0;
const REGI2C_TXRF_MASK: u32 = 1 << 1;
const REGI2C_SDM_MASK: u32 = 1 << 2;
const REGI2C_RFPLL_MASK: u32 = 1 << 3;
const REGI2C_BIAS_MASK: u32 = 1 << 4;
const REGI2C_BBPLL_MASK: u32 = 1 << 5;
const REGI2C_ULP_MASK: u32 = 1 << 6;
const REGI2C_SAR_MASTER_MASK: u32 = 1 << 7;
const REGI2C_SAR_SLAVE_MASK: u32 = 1 << 8;
const REGI2C_PERIF_MASK: u32 = 1 << 9;
const REGI2C_APLL_MASK: u32 = 1 << 10;
const REGI2C_CPLL_MASK: u32 = 1 << 11;
const REGI2C_MPLL_MASK: u32 = 1 << 12;
const REGI2C_DIG_REG_MASK: u32 = 1 << 13;

const CONF1_SLAVE_SEL_SHIFT: u32 = 2;
const CONF2_SLAVE_SEL_SHIFT: u32 = 4;

/// 24-bit write mask for I2C_ANA_MST_ANA_CONF1.
const ANA_CONF1_M: u32 = 0x00FF_FFFF;

/// I2C ANA MST register block: MODEM_PWR_BASE (0x2010_D000) + 0x2800.
const I2C_ANA_MST_BASE: u32 = 0x2010_F800;
const I2C_ANA_MST_I2C0_CTRL_REG: u32 = I2C_ANA_MST_BASE;
const I2C_ANA_MST_I2C1_CTRL_REG: u32 = I2C_ANA_MST_BASE + 0x04;

/// I2C control register bit fields (same layout as C5/C61).
const REGI2C_RTC_BUSY_BIT: u32 = 1 << 25;
const REGI2C_RTC_WR_CNTL_BIT: u32 = 1 << 24;
const REGI2C_RTC_DATA_SHIFT: u32 = 16;
const REGI2C_RTC_DATA_MASK: u32 = 0xFF;
const REGI2C_RTC_ADDR_SHIFT: u32 = 8;
const REGI2C_RTC_ADDR_MASK: u32 = 0xFF;
const REGI2C_RTC_SLAVE_ID_MASK: u32 = 0xFF;

fn regi2c_enable_block(block: u8) -> usize {
    // Match the S31 bootloader setup explicitly instead of relying on a
    // previous-stage bootloader to leave the analog-I2C source and force bit
    // configured for us.
    MODEM_SYSCON::regs()
        .clk_conf()
        .modify(|_, w| w.clk_i2c_mst_sel_160m().set_bit());
    MODEM_LPCON::regs()
        .clk_conf_force_on()
        .modify(|_, w| w.clk_i2c_mst_fo().set_bit());
    MODEM_LPCON::regs()
        .clk_conf()
        .modify(|_, w| w.clk_i2c_mst_en().set_bit());

    let bit_mask: u32 = match block {
        v if v == REGI2C_BB.master => REGI2C_BB_MASK,
        v if v == REGI2C_TXRF.master => REGI2C_TXRF_MASK,
        v if v == REGI2C_SDM.master => REGI2C_SDM_MASK,
        v if v == REGI2C_RFPLL.master => REGI2C_RFPLL_MASK,
        v if v == REGI2C_BIAS.master => REGI2C_BIAS_MASK,
        v if v == REGI2C_BBPLL.master => REGI2C_BBPLL_MASK,
        v if v == REGI2C_ULP.master => REGI2C_ULP_MASK,
        v if v == REGI2C_SAR_MASTER.master => REGI2C_SAR_MASTER_MASK,
        v if v == REGI2C_SAR_SLAVE.master => REGI2C_SAR_SLAVE_MASK,
        v if v == REGI2C_PERIF.master => REGI2C_PERIF_MASK,
        v if v == REGI2C_APLL.master => REGI2C_APLL_MASK,
        v if v == REGI2C_CPLL.master => REGI2C_CPLL_MASK,
        v if v == REGI2C_MPLL.master => REGI2C_MPLL_MASK,
        v if v == REGI2C_DIG_REG.master => REGI2C_DIG_REG_MASK,
        _ => return 0,
    };

    let i2c_ana_mst = unsafe { &*crate::pac::I2C_ANA_MST::PTR };
    let conf2 = i2c_ana_mst.ana_conf2().read().ana_conf2().bits();
    let i2c_sel_set = conf2 & (bit_mask << CONF2_SLAVE_SEL_SHIFT) != 0;

    i2c_ana_mst.ana_conf1().write(|w| unsafe {
        w.ana_conf1()
            .bits(!(bit_mask << CONF1_SLAVE_SEL_SHIFT) & ANA_CONF1_M)
    });

    // i2c_sel_set == true → use master 0; false → use master 1
    if i2c_sel_set { 0 } else { 1 }
}

#[inline]
fn i2c_ctrl_reg(master: usize) -> u32 {
    if master == 0 {
        I2C_ANA_MST_I2C0_CTRL_REG
    } else {
        I2C_ANA_MST_I2C1_CTRL_REG
    }
}

#[inline]
fn wait_i2c_idle(ctrl: u32) {
    unsafe {
        while (ctrl as *const u32).read_volatile() & REGI2C_RTC_BUSY_BIT != 0 {
            core::hint::spin_loop();
        }
    }
}

pub(crate) fn regi2c_read(block: u8, _host_id: u8, reg_add: u8) -> u8 {
    let master = regi2c_enable_block(block);
    let ctrl = i2c_ctrl_reg(master);

    wait_i2c_idle(ctrl);

    let cmd = (
        (block as u32 & REGI2C_RTC_SLAVE_ID_MASK)
        // << 0
    ) | ((reg_add as u32 & REGI2C_RTC_ADDR_MASK) << REGI2C_RTC_ADDR_SHIFT);

    unsafe { (ctrl as *mut u32).write_volatile(cmd) };

    wait_i2c_idle(ctrl);

    let val = unsafe { (ctrl as *const u32).read_volatile() };
    ((val >> REGI2C_RTC_DATA_SHIFT) & REGI2C_RTC_DATA_MASK) as u8
}

pub(crate) fn regi2c_write(block: u8, _host_id: u8, reg_add: u8, data: u8) {
    let master = regi2c_enable_block(block);
    let ctrl = i2c_ctrl_reg(master);

    wait_i2c_idle(ctrl);

    let cmd = (
        (block as u32 & REGI2C_RTC_SLAVE_ID_MASK)
        // << 0
    ) | ((reg_add as u32 & REGI2C_RTC_ADDR_MASK) << REGI2C_RTC_ADDR_SHIFT)
        | REGI2C_RTC_WR_CNTL_BIT
        | ((data as u32 & REGI2C_RTC_DATA_MASK) << REGI2C_RTC_DATA_SHIFT);

    unsafe { (ctrl as *mut u32).write_volatile(cmd) };

    wait_i2c_idle(ctrl);
}
