use crate::{
    peripherals::{LP_I2C_ANA_MST, LP_PERI, PMU},
    rom::{
        ets_delay_us,
        regi2c::{RawRegI2cField, RegI2cMaster, RegI2cRegister, define_regi2c},
    },
};

define_regi2c! {
    master: REGI2C_SDIO_PLL(0x62, 0) {}
    master: REGI2C_MSPI(0x63, 0) {
        reg: I2C_MPLL_IR_CAL_RSTB(1) {}
        reg: I2C_MPLL_DIV_REG_ADDR(2) {}
        reg: I2C_MPLL_DHREF(3) {
            field: I2C_MPLL_DHREF_DHREF(5..4)
        }
    }
    master: REGI2C_SYS_PLL(0x66, 0) {
        reg: I2C_SPLL_OC_REF_DIV(2) {}
        reg: I2C_SPLL_OC_DIV_7_0(3) {}
        reg: I2C_SPLL_OC_DCUR(6) {}
    }
    master: REGI2C_CPU_PLL(0x67, 0) {
        reg: I2C_CPLL_OC_REF_DIV(2) {}
        reg: I2C_CPLL_OC_DIV_7_0(3) {}
        reg: I2C_CPLL_OC_DCUR(6) {}
    }
    master: REGI2C_SAR_I2C(0x69, 0) {}
    master: REGI2C_BIAS(0x6a, 0) {}
    master: REGI2C_DIG_REG(0x6d, 0) {
        reg: I2C_DIG_REG9(9) {
            field: I2C_DIG_REG_OR_EN_CONT_CAL(7..7)
        }
        reg: I2C_DIG_REG10(10) {
            field: I2C_DIG_REG_FORCE_RTC_DREG(0..0),
            field: I2C_DIG_REG_FORCE_DIG_DREG(1..1)
        }
        reg: I2C_DIG_REG_XPD(13) {
            field: I2C_DIG_REG_XPD_RTC_REG(2..2),
            field: I2C_DIG_REG_XPD_DIG_REG(3..3)
        }
        reg: I2C_DIG_REG_SCK_DCAP(14) {}
    }
    master: REGI2C_PLLA(0x6f, 0) {}
}

// Master select bits in ANA_CONF2 register
const REGI2C_DIG_REG_MST_SEL: u16 = 1 << 10;
const REGI2C_PLL_CPU_MST_SEL: u16 = 1 << 11;
const REGI2C_PLL_SDIO_MST_SEL: u16 = 1 << 6;
const REGI2C_BIAS_MST_SEL: u16 = 1 << 12;
const REGI2C_MSPI_XTAL_MST_SEL: u16 = 1 << 9;
const REGI2C_PLL_SYS_MST_SEL: u16 = 1 << 5;
const REGI2C_PLLA_MST_SEL: u16 = 1 << 8;
const REGI2C_SAR_I2C_MST_SEL: u16 = 1 << 7;

/// I2C control register bit fields
const REGI2C_RTC_WR_CNTL_BIT: u32 = 1 << 24;
const REGI2C_RTC_DATA_SHIFT: u32 = 16;
const REGI2C_RTC_DATA_MASK: u32 = 0xFF;
const REGI2C_RTC_ADDR_SHIFT: u32 = 8;
const REGI2C_RTC_ADDR_MASK: u32 = 0xFF;
const REGI2C_RTC_SLAVE_ID_SHIFT: u32 = 0;
const REGI2C_RTC_SLAVE_ID_MASK: u32 = 0xFF;

/// Select the I2C master for the given analog block.
fn regi2c_enable_block(block: u8) {
    // Enable I2C master clock
    LP_PERI::regs()
        .clk_en()
        .modify(|_, w| w.ck_en_lp_i2cmst().set_bit());

    // The master has no clock source after reset, so it never completes a
    // transaction until the 160MHz source is selected.
    // Ref: IDF `regi2c_ctrl_ll_master_configure_clock` (esp32p4).
    LP_I2C_ANA_MST::regs()
        .clk160m()
        .modify(|_, w| w.clk_i2c_mst_sel_160m().set_bit());

    // Clear both conf registers first
    LP_I2C_ANA_MST::regs()
        .ana_conf2()
        .modify(|_, w| unsafe { w.ana_conf2().bits(0) });
    LP_I2C_ANA_MST::regs()
        .ana_conf1()
        .modify(|_, w| unsafe { w.ana_conf1().bits(0) });

    // Set the master select bit for this block
    let sel_bit: u32 = match block {
        v if v == REGI2C_DIG_REG.master => REGI2C_DIG_REG_MST_SEL as u32,
        v if v == REGI2C_CPU_PLL.master => REGI2C_PLL_CPU_MST_SEL as u32,
        v if v == REGI2C_SDIO_PLL.master => REGI2C_PLL_SDIO_MST_SEL as u32,
        v if v == REGI2C_BIAS.master => REGI2C_BIAS_MST_SEL as u32,
        v if v == REGI2C_MSPI.master => REGI2C_MSPI_XTAL_MST_SEL as u32,
        v if v == REGI2C_SYS_PLL.master => REGI2C_PLL_SYS_MST_SEL as u32,
        v if v == REGI2C_PLLA.master => REGI2C_PLLA_MST_SEL as u32,
        v if v == REGI2C_SAR_I2C.master => REGI2C_SAR_I2C_MST_SEL as u32,
        _ => return,
    };

    LP_I2C_ANA_MST::regs()
        .ana_conf2()
        .modify(|r, w| unsafe { w.ana_conf2().bits(r.ana_conf2().bits() | sel_bit) });
}

/// Wait for I2C bus to become idle.
#[inline]
fn wait_i2c_idle() {
    while LP_I2C_ANA_MST::regs()
        .i2c0_ctrl()
        .read()
        .i2c0_busy()
        .bit_is_set()
    {
        core::hint::spin_loop();
    }
}

/// Read an analog I2C register.
pub(crate) fn regi2c_read(block: u8, _host_id: u8, reg_add: u8) -> u8 {
    regi2c_enable_block(block);
    wait_i2c_idle();

    // Build read command: slave_id[7:0] | addr[15:8]
    let cmd = ((block as u32 & REGI2C_RTC_SLAVE_ID_MASK) << REGI2C_RTC_SLAVE_ID_SHIFT)
        | ((reg_add as u32 & REGI2C_RTC_ADDR_MASK) << REGI2C_RTC_ADDR_SHIFT);

    LP_I2C_ANA_MST::regs()
        .i2c0_ctrl()
        .write(|w| unsafe { w.i2c0_ctrl().bits(cmd) });
    wait_i2c_idle();

    // Read data from bits [23:16]
    let val = LP_I2C_ANA_MST::regs().i2c0_ctrl().read().i2c0_ctrl().bits();
    ((val >> REGI2C_RTC_DATA_SHIFT) & REGI2C_RTC_DATA_MASK) as u8
}

/// Write an analog I2C register.
pub(crate) fn regi2c_write(block: u8, _host_id: u8, reg_add: u8, data: u8) {
    regi2c_enable_block(block);
    wait_i2c_idle();

    // Build write command: slave_id[7:0] | addr[15:8] | data[23:16] | wr_cntl[24]
    let cmd = ((block as u32 & REGI2C_RTC_SLAVE_ID_MASK) << REGI2C_RTC_SLAVE_ID_SHIFT)
        | ((reg_add as u32 & REGI2C_RTC_ADDR_MASK) << REGI2C_RTC_ADDR_SHIFT)
        | REGI2C_RTC_WR_CNTL_BIT
        | ((data as u32 & REGI2C_RTC_DATA_MASK) << REGI2C_RTC_DATA_SHIFT);

    LP_I2C_ANA_MST::regs()
        .i2c0_ctrl()
        .write(|w| unsafe { w.i2c0_ctrl().bits(cmd) });
    wait_i2c_idle();
}
