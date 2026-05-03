//! I2C analog register access for ESP32-P4.
//!
//! Used for PLL configuration (CPLL, SPLL, MPLL) and other analog peripherals.
//! Accesses the LP_I2C_ANA_MST peripheral which bridges to internal analog I2C bus.
//!
//! Ref: TRM v0.5 Ch 49 (Analog I2C Controller)

// I2C slave addresses for analog blocks
#[allow(dead_code)]
pub(crate) const REGI2C_DIG_REG: u8 = 0x6d;
#[allow(dead_code)]
pub(crate) const REGI2C_CPU_PLL: u8 = 0x67;
#[allow(dead_code)]
pub(crate) const REGI2C_SDIO_PLL: u8 = 0x62;
#[allow(dead_code)]
pub(crate) const REGI2C_BIAS: u8 = 0x6a;
#[allow(dead_code)]
pub(crate) const REGI2C_MSPI: u8 = 0x63;
#[allow(dead_code)]
pub(crate) const REGI2C_SYS_PLL: u8 = 0x66;
#[allow(dead_code)]
pub(crate) const REGI2C_PLLA: u8 = 0x6f;
#[allow(dead_code)]
pub(crate) const REGI2C_SAR_I2C: u8 = 0x69;

// Master select bits in ANA_CONF2 register
const REGI2C_DIG_REG_MST_SEL: u16 = 1 << 10;
const REGI2C_PLL_CPU_MST_SEL: u16 = 1 << 11;
#[allow(dead_code)]
const REGI2C_PLL_SDIO_MST_SEL: u16 = 1 << 6;
#[allow(dead_code)]
const REGI2C_BIAS_MST_SEL: u16 = 1 << 12;
#[allow(dead_code)]
const REGI2C_MSPI_XTAL_MST_SEL: u16 = 1 << 9;
#[allow(dead_code)]
const REGI2C_PLL_SYS_MST_SEL: u16 = 1 << 5;
#[allow(dead_code)]
const REGI2C_PLLA_MST_SEL: u16 = 1 << 8;
#[allow(dead_code)]
const REGI2C_SAR_I2C_MST_SEL: u16 = 1 << 7;

/// I2C control register bit fields
const REGI2C_RTC_BUSY_BIT: u32 = 1 << 25;
const REGI2C_RTC_WR_CNTL_BIT: u32 = 1 << 24;
const REGI2C_RTC_DATA_SHIFT: u32 = 16;
const REGI2C_RTC_DATA_MASK: u32 = 0xFF;
const REGI2C_RTC_ADDR_SHIFT: u32 = 8;
const REGI2C_RTC_ADDR_MASK: u32 = 0xFF;
const REGI2C_RTC_SLAVE_ID_SHIFT: u32 = 0;
const REGI2C_RTC_SLAVE_ID_MASK: u32 = 0xFF;

/// LP_I2C_ANA_MST base address (from PAC: 0x5012_4000)
/// I2C0_CTRL_REG is at offset 0x14
const LP_I2C_ANA_MST_BASE: u32 = 0x5012_4000;
const LP_I2C_ANA_MST_I2C0_CTRL_REG: u32 = LP_I2C_ANA_MST_BASE + 0x14;
const LP_I2C_ANA_MST_ANA_CONF1_REG: u32 = LP_I2C_ANA_MST_BASE + 0x04;
const LP_I2C_ANA_MST_ANA_CONF2_REG: u32 = LP_I2C_ANA_MST_BASE + 0x08;

/// Select the I2C master for the given analog block.
fn regi2c_enable_block(block: u8) {
    // Clear both conf registers first
    unsafe {
        (LP_I2C_ANA_MST_ANA_CONF2_REG as *mut u32).write_volatile(0);
        (LP_I2C_ANA_MST_ANA_CONF1_REG as *mut u32).write_volatile(0);
    }

    // Set the master select bit for this block
    let sel_bit: u32 = match block {
        REGI2C_DIG_REG => REGI2C_DIG_REG_MST_SEL as u32,
        REGI2C_CPU_PLL => REGI2C_PLL_CPU_MST_SEL as u32,
        REGI2C_SDIO_PLL => REGI2C_PLL_SDIO_MST_SEL as u32,
        REGI2C_BIAS => REGI2C_BIAS_MST_SEL as u32,
        REGI2C_MSPI => REGI2C_MSPI_XTAL_MST_SEL as u32,
        REGI2C_SYS_PLL => REGI2C_PLL_SYS_MST_SEL as u32,
        REGI2C_PLLA => REGI2C_PLLA_MST_SEL as u32,
        REGI2C_SAR_I2C => REGI2C_SAR_I2C_MST_SEL as u32,
        _ => return,
    };

    unsafe {
        let reg = LP_I2C_ANA_MST_ANA_CONF2_REG as *mut u32;
        reg.write_volatile(reg.read_volatile() | sel_bit);
    }
}

/// Wait for I2C bus to become idle.
#[inline]
fn wait_i2c_idle() {
    unsafe {
        let reg = LP_I2C_ANA_MST_I2C0_CTRL_REG as *const u32;
        while reg.read_volatile() & REGI2C_RTC_BUSY_BIT != 0 {
            core::hint::spin_loop();
        }
    }
}

/// Read an analog I2C register.
pub(crate) fn regi2c_read(block: u8, _host_id: u8, reg_add: u8) -> u8 {
    regi2c_enable_block(block);
    wait_i2c_idle();

    // Build read command: slave_id[7:0] | addr[15:8]
    let cmd = ((block as u32 & REGI2C_RTC_SLAVE_ID_MASK) << REGI2C_RTC_SLAVE_ID_SHIFT)
        | ((reg_add as u32 & REGI2C_RTC_ADDR_MASK) << REGI2C_RTC_ADDR_SHIFT);

    unsafe {
        (LP_I2C_ANA_MST_I2C0_CTRL_REG as *mut u32).write_volatile(cmd);
    }
    wait_i2c_idle();

    // Read data from bits [23:16]
    let val = unsafe { (LP_I2C_ANA_MST_I2C0_CTRL_REG as *const u32).read_volatile() };
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

    unsafe {
        (LP_I2C_ANA_MST_I2C0_CTRL_REG as *mut u32).write_volatile(cmd);
    }
    wait_i2c_idle();
}
