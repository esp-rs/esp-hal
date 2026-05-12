use crate::rom::regi2c::{RawRegI2cField, RegI2cMaster, RegI2cRegister, define_regi2c};

define_regi2c! {
    master: REGI2C_SDIO_PLL(0x62, 0) {}
    master: REGI2C_MSPI(0x63, 0) {
        reg: I2C_MPLL_DIV_REG_ADDR(2) {}
        reg: I2C_MPLL_DHREF(3) {}
        reg: I2C_MPLL_IR_CAL_RSTB(5) {}
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
    master: REGI2C_BIAS(0x6a, 0) {
        reg: I2C_BIAS_REG_4(4) {
            field: I2C_BIAS_OR_FORCE_XPD_CK          (0..0),
            field: I2C_BIAS_OR_FORCE_XPD_REF_OUT_BUF (1..1),
            field: I2C_BIAS_OR_FORCE_XPD_IPH         (2..2),
            field: I2C_BIAS_OR_FORCE_XPD_VGATE_BUF   (3..3)
        }
    }
    master: REGI2C_DIG_REG(0x6d, 0) {
        reg: I2C_DIG_REG_REG_10(10) {
            field: I2C_DIG_REG_FORCE_RTC_DREG (0..0),
            field: I2C_DIG_REG_FORCE_DIG_DREG (1..1)
        }
        reg: I2C_DIG_REG_REG_13(13) {
            field: I2C_DIG_REG_XPD_RTC_REG (2..2),
            field: I2C_DIG_REG_XPD_DIG_REG (3..3)
        }
        reg: I2C_DIG_REG_REG_14(14) {
            field: I2C_DIG_REG_SCK_DCAP (7..0)
        }
    }
    master: REGI2C_PLLA(0x6f, 0) {}
}

const REGI2C_DIG_REG_MST_SEL: u16 = 1 << 10;
const REGI2C_PLL_CPU_MST_SEL: u16 = 1 << 11;
const REGI2C_PLL_SDIO_MST_SEL: u16 = 1 << 6;
const REGI2C_BIAS_MST_SEL: u16 = 1 << 12;
const REGI2C_MSPI_XTAL_MST_SEL: u16 = 1 << 9;
const REGI2C_PLL_SYS_MST_SEL: u16 = 1 << 5;
const REGI2C_PLLA_MST_SEL: u16 = 1 << 8;
const REGI2C_SAR_I2C_MST_SEL: u16 = 1 << 7;

const REGI2C_RTC_BUSY_BIT: u32 = 1 << 25;
const REGI2C_RTC_WR_CNTL_BIT: u32 = 1 << 24;
const REGI2C_RTC_DATA_SHIFT: u32 = 16;
const REGI2C_RTC_DATA_MASK: u32 = 0xFF;
const REGI2C_RTC_ADDR_SHIFT: u32 = 8;
const REGI2C_RTC_ADDR_MASK: u32 = 0xFF;
const REGI2C_RTC_SLAVE_ID_SHIFT: u32 = 0;
const REGI2C_RTC_SLAVE_ID_MASK: u32 = 0xFF;

// LP_I2C_ANA_MST base + register offsets.
const LP_I2C_ANA_MST_BASE: u32 = 0x5012_4000;
const LP_I2C_ANA_MST_I2C0_CTRL_REG: u32 = LP_I2C_ANA_MST_BASE + 0x14;
const LP_I2C_ANA_MST_ANA_CONF1_REG: u32 = LP_I2C_ANA_MST_BASE + 0x04;
const LP_I2C_ANA_MST_ANA_CONF2_REG: u32 = LP_I2C_ANA_MST_BASE + 0x08;

fn regi2c_enable_block(block: u8) {
    unsafe {
        (LP_I2C_ANA_MST_ANA_CONF2_REG as *mut u32).write_volatile(0);
        (LP_I2C_ANA_MST_ANA_CONF1_REG as *mut u32).write_volatile(0);
    }

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

    unsafe {
        let reg = LP_I2C_ANA_MST_ANA_CONF2_REG as *mut u32;
        reg.write_volatile(reg.read_volatile() | sel_bit);
    }
}

#[inline]
fn wait_i2c_idle() {
    unsafe {
        let reg = LP_I2C_ANA_MST_I2C0_CTRL_REG as *const u32;
        while reg.read_volatile() & REGI2C_RTC_BUSY_BIT != 0 {
            core::hint::spin_loop();
        }
    }
}

pub(crate) fn regi2c_read(block: u8, _host_id: u8, reg_add: u8) -> u8 {
    regi2c_enable_block(block);
    wait_i2c_idle();

    let cmd = ((block as u32 & REGI2C_RTC_SLAVE_ID_MASK) << REGI2C_RTC_SLAVE_ID_SHIFT)
        | ((reg_add as u32 & REGI2C_RTC_ADDR_MASK) << REGI2C_RTC_ADDR_SHIFT);

    unsafe {
        (LP_I2C_ANA_MST_I2C0_CTRL_REG as *mut u32).write_volatile(cmd);
    }
    wait_i2c_idle();

    let val = unsafe { (LP_I2C_ANA_MST_I2C0_CTRL_REG as *const u32).read_volatile() };
    ((val >> REGI2C_RTC_DATA_SHIFT) & REGI2C_RTC_DATA_MASK) as u8
}

pub(crate) fn regi2c_write(block: u8, _host_id: u8, reg_add: u8, data: u8) {
    regi2c_enable_block(block);
    wait_i2c_idle();

    let cmd = ((block as u32 & REGI2C_RTC_SLAVE_ID_MASK) << REGI2C_RTC_SLAVE_ID_SHIFT)
        | ((reg_add as u32 & REGI2C_RTC_ADDR_MASK) << REGI2C_RTC_ADDR_SHIFT)
        | REGI2C_RTC_WR_CNTL_BIT
        | ((data as u32 & REGI2C_RTC_DATA_MASK) << REGI2C_RTC_DATA_SHIFT);

    unsafe {
        (LP_I2C_ANA_MST_I2C0_CTRL_REG as *mut u32).write_volatile(cmd);
    }
    wait_i2c_idle();
}
