use crate::peripherals::{APB_SARADC, LPWR, SENS, SYSTEM};

const ANA_CONFIG_REG: u32 = 0x6000E044;
const ANA_CONFIG2_REG: u32 = 0x6000E048;
const I2C_SAR_ADC: u8 = 0x69;
const I2C_SAR_ADC_HOSTID: u8 = 1;
const ADC_SAR1_DREF_ADDR: u8 = 0x2;
const ADC_SAR1_DREF_ADDR_MSB: u8 = 0x6;
const ADC_SAR1_DREF_ADDR_LSB: u8 = 0x4;
const ADC_SAR2_DREF_ADDR: u8 = 0x5;
const ADC_SAR2_DREF_ADDR_MSB: u8 = 0x6;
const ADC_SAR2_DREF_ADDR_LSB: u8 = 0x4;
const ADC_SARADC_ENCAL_REF_ADDR: u8 = 0x7;
const ADC_SARADC_ENCAL_REF_ADDR_MSB: u8 = 4;
const ADC_SARADC_ENCAL_REF_ADDR_LSB: u8 = 4;
const ADC_SARADC_ENT_TSENS_ADDR: u8 = 0x7;
const ADC_SARADC_ENT_TSENS_ADDR_MSB: u8 = 2;
const ADC_SARADC_ENT_TSENS_ADDR_LSB: u8 = 2;
const ADC_SARADC_ENT_RTC_ADDR: u8 = 0x7;
const ADC_SARADC_ENT_RTC_ADDR_MSB: u8 = 3;
const ADC_SARADC_ENT_RTC_ADDR_LSB: u8 = 3;

const I2C_RTC_SLAVE_ID_V: u8 = 0xFF;
const I2C_RTC_SLAVE_ID_S: u8 = 0;
const I2C_RTC_ADDR_V: u8 = 0xFF;
const I2C_RTC_ADDR_S: u8 = 0x8;
const I2C_RTC_CONFIG2: u32 = 0x6000e000;
const I2C_RTC_BUSY: u32 = 1 << 25;
const I2C_RTC_DATA_V: u32 = 0xFF;
const I2C_RTC_DATA_S: u32 = 16;
const I2C_RTC_WR_CNTL_V: u8 = 0x1;
const I2C_RTC_WR_CNTL_S: u8 = 24;
const I2C_RTC_CONFIG0: u32 = 0x6000e048;
const I2C_RTC_CONFIG1: u32 = 0x6000e044;
const I2C_RTC_MAGIC_CTRL_V: u32 = 0x1FFF;
const I2C_RTC_MAGIC_CTRL_S: u32 = 4;
const I2C_RTC_MAGIC_DEFAULT: u32 = 0x1c40;
const I2C_RTC_ALL_MASK_V: u32 = 0x7FFF;
const I2C_RTC_ALL_MASK_S: u32 = 8;
const I2C_RTC_WIFI_CLK_EN: u32 = 0x3f426000 + 0x090;
const I2C_RTC_CLK_GATE_EN: u32 = 1 << 18;
const I2C_BOD: u8 = 0x61;
const I2C_BBPLL: u8 = 0x66;
const I2C_APLL: u8 = 0x6D;
const I2C_RTC_APLL_MASK: u32 = 1 << 14;
const I2C_RTC_BBPLL_MASK: u32 = 1 << 17;
const I2C_RTC_SAR_MASK: u32 = 1 << 18;
const I2C_RTC_BOD_MASK: u32 = 1 << 22;

/// Enable true randomness by enabling the entropy source.
/// Blocks `ADC` usage.
pub(crate) fn ensure_randomness() {
    let rtc_cntl = LPWR::regs();
    let dport = SYSTEM::regs();
    let apb_saradc = APB_SARADC::regs();
    let sens = SENS::regs();

    unsafe {
        // Enable 8M clock source for RNG (this is actually enough to produce strong
        // random results, but enabling the SAR ADC as well adds some insurance.)
        rtc_cntl
            .clk_conf()
            .modify(|_, w| w.dig_clk8m_en().set_bit());

        // Enable SAR ADC to read a disconnected input for additional entropy
        dport
            .perip_clk_en0()
            .modify(|_, w| w.apb_saradc_clk_en().set_bit());

        apb_saradc.clkm_conf().modify(|_, w| w.clk_sel().bits(2));

        rtc_cntl
            .ana_conf()
            .modify(|_, w| w.sar_i2c_force_pd().clear_bit());

        rtc_cntl
            .ana_conf()
            .modify(|_, w| w.sar_i2c_force_pu().set_bit());

        // Temporarily not in PACs
        // esp-idf/components/soc/esp32s2/include/soc/regi2c_defs.h
        clear_peri_reg_mask(ANA_CONFIG_REG, 1 << 18);
        set_peri_reg_mask(ANA_CONFIG2_REG, 1 << 16);

        regi2c_write_mask(
            I2C_SAR_ADC,
            I2C_SAR_ADC_HOSTID,
            ADC_SAR1_DREF_ADDR,
            ADC_SAR1_DREF_ADDR_MSB,
            ADC_SAR1_DREF_ADDR_LSB,
            0x4,
        );

        regi2c_write_mask(
            I2C_SAR_ADC,
            I2C_SAR_ADC_HOSTID,
            ADC_SAR2_DREF_ADDR,
            ADC_SAR2_DREF_ADDR_MSB,
            ADC_SAR2_DREF_ADDR_LSB,
            0x4,
        );

        regi2c_write_mask(
            I2C_SAR_ADC,
            I2C_SAR_ADC_HOSTID,
            ADC_SARADC_ENCAL_REF_ADDR,
            ADC_SARADC_ENCAL_REF_ADDR_MSB,
            ADC_SARADC_ENCAL_REF_ADDR_LSB,
            1,
        );

        regi2c_write_mask(
            I2C_SAR_ADC,
            I2C_SAR_ADC_HOSTID,
            ADC_SARADC_ENT_TSENS_ADDR,
            ADC_SARADC_ENT_TSENS_ADDR_MSB,
            ADC_SARADC_ENT_TSENS_ADDR_LSB,
            1,
        );

        regi2c_write_mask(
            I2C_SAR_ADC,
            I2C_SAR_ADC_HOSTID,
            ADC_SARADC_ENT_RTC_ADDR,
            ADC_SARADC_ENT_RTC_ADDR_MSB,
            ADC_SARADC_ENT_RTC_ADDR_LSB,
            0,
        );

        apb_saradc.ctrl().modify(|_, w| w.sar1_patt_len().bits(0));

        apb_saradc
            .sar1_patt_tab1()
            .modify(|_, w| w.bits(0xafffffff));

        apb_saradc.ctrl().modify(|_, w| w.sar2_patt_len().bits(0));

        apb_saradc
            .sar2_patt_tab1()
            .modify(|_, w| w.bits(0xafffffff));

        sens.sar_meas1_mux()
            .modify(|_, w| w.sar1_dig_force().set_bit());

        apb_saradc.ctrl().modify(|_, w| w.work_mode().bits(1));

        apb_saradc
            .ctrl2()
            .modify(|_, w| w.meas_num_limit().clear_bit());

        sens.sar_power_xpd_sar()
            .modify(|_, w| w.force_xpd_sar().bits(3));

        apb_saradc.ctrl2().modify(|_, w| w.timer_sel().set_bit());

        apb_saradc.ctrl2().modify(|_, w| w.timer_target().bits(100));

        apb_saradc.ctrl().modify(|_, w| w.start_force().clear_bit());

        apb_saradc.ctrl2().modify(|_, w| w.timer_en().set_bit());
    }
}

/// Disable true randomness. Unlocks `ADC` peripheral.
pub(crate) fn revert_trng() {
    unsafe {
        // Restore internal I2C bus state
        regi2c_write_mask(
            I2C_SAR_ADC,
            I2C_SAR_ADC_HOSTID,
            ADC_SAR1_DREF_ADDR,
            ADC_SAR1_DREF_ADDR_MSB,
            ADC_SAR1_DREF_ADDR_LSB,
            0x1,
        );

        regi2c_write_mask(
            I2C_SAR_ADC,
            I2C_SAR_ADC_HOSTID,
            ADC_SAR2_DREF_ADDR,
            ADC_SAR2_DREF_ADDR_MSB,
            ADC_SAR2_DREF_ADDR_LSB,
            0x1,
        );

        regi2c_write_mask(
            I2C_SAR_ADC,
            I2C_SAR_ADC_HOSTID,
            ADC_SARADC_ENCAL_REF_ADDR,
            ADC_SARADC_ENCAL_REF_ADDR_MSB,
            ADC_SARADC_ENCAL_REF_ADDR_LSB,
            0,
        );

        regi2c_write_mask(
            I2C_SAR_ADC,
            I2C_SAR_ADC_HOSTID,
            ADC_SARADC_ENT_TSENS_ADDR,
            ADC_SARADC_ENT_TSENS_ADDR_MSB,
            ADC_SARADC_ENT_TSENS_ADDR_LSB,
            0,
        );

        regi2c_write_mask(
            I2C_SAR_ADC,
            I2C_SAR_ADC_HOSTID,
            ADC_SARADC_ENT_RTC_ADDR,
            ADC_SARADC_ENT_RTC_ADDR_MSB,
            ADC_SARADC_ENT_RTC_ADDR_LSB,
            0,
        );

        // Restore SARADC to default mode
        SENS::regs()
            .sar_meas1_mux()
            .modify(|_, w| w.sar1_dig_force().clear_bit());

        SYSTEM::regs()
            .perip_clk_en0()
            .modify(|_, w| w.apb_saradc_clk_en().set_bit());

        SENS::regs()
            .sar_power_xpd_sar()
            .modify(|_, w| w.force_xpd_sar().bits(0));

        APB_SARADC::regs()
            .ctrl2()
            .modify(|_, w| w.timer_en().clear_bit());
    }
}
// Temporarily not in PACs
fn regi2c_enable_block(block: u8) {
    reg_set_field(
        I2C_RTC_CONFIG0,
        I2C_RTC_MAGIC_CTRL_V,
        I2C_RTC_MAGIC_CTRL_S,
        I2C_RTC_MAGIC_DEFAULT,
    );
    reg_set_field(
        I2C_RTC_CONFIG1,
        I2C_RTC_ALL_MASK_V,
        I2C_RTC_ALL_MASK_S,
        I2C_RTC_ALL_MASK_V,
    );

    reg_set_bit(I2C_RTC_WIFI_CLK_EN, I2C_RTC_CLK_GATE_EN);

    // Before config I2C register, enable corresponding slave.
    match block {
        v if v == I2C_APLL => {
            reg_set_bit(I2C_RTC_CONFIG1, I2C_RTC_APLL_MASK);
        }
        v if v == I2C_BBPLL => {
            reg_set_bit(I2C_RTC_CONFIG1, I2C_RTC_BBPLL_MASK);
        }
        v if v == I2C_SAR_ADC => {
            reg_set_bit(I2C_RTC_CONFIG1, I2C_RTC_SAR_MASK);
        }
        v if v == I2C_BOD => {
            reg_set_bit(I2C_RTC_CONFIG1, I2C_RTC_BOD_MASK);
        }
        _ => (),
    }
}

fn regi2c_disable_block(block: u8) {
    match block {
        v if v == I2C_APLL => {
            reg_clr_bit(I2C_RTC_CONFIG1, I2C_RTC_APLL_MASK);
        }
        v if v == I2C_BBPLL => {
            reg_clr_bit(I2C_RTC_CONFIG1, I2C_RTC_BBPLL_MASK);
        }
        v if v == I2C_SAR_ADC => {
            reg_clr_bit(I2C_RTC_CONFIG1, I2C_RTC_SAR_MASK);
        }
        v if v == I2C_BOD => {
            reg_clr_bit(I2C_RTC_CONFIG1, I2C_RTC_BOD_MASK);
        }
        _ => (),
    }
}

pub(crate) fn regi2c_write_mask(block: u8, _host_id: u8, reg_add: u8, msb: u8, lsb: u8, data: u8) {
    assert!(msb - lsb < 8);
    regi2c_enable_block(block);

    // Read the i2c bus register
    let mut temp: u32 = ((block as u32 & I2C_RTC_SLAVE_ID_V as u32) << I2C_RTC_SLAVE_ID_S as u32)
        | (reg_add as u32 & I2C_RTC_ADDR_V as u32) << I2C_RTC_ADDR_S as u32;
    reg_write(I2C_RTC_CONFIG2, temp);
    while reg_get_bit(I2C_RTC_CONFIG2, I2C_RTC_BUSY) != 0 {}
    temp = reg_get_field(I2C_RTC_CONFIG2, I2C_RTC_DATA_S, I2C_RTC_DATA_V);
    // Write the i2c bus register
    temp &= (!(0xFFFFFFFF << lsb)) | (0xFFFFFFFF << (msb + 1));
    temp |= (data as u32 & (!(0xFFFFFFFF << (msb as u32 - lsb as u32 + 1)))) << (lsb as u32);
    temp = ((block as u32 & I2C_RTC_SLAVE_ID_V as u32) << I2C_RTC_SLAVE_ID_S as u32)
        | ((reg_add as u32 & I2C_RTC_ADDR_V as u32) << I2C_RTC_ADDR_S as u32)
        | ((0x1 & I2C_RTC_WR_CNTL_V as u32) << I2C_RTC_WR_CNTL_S as u32)
        | ((temp & I2C_RTC_DATA_V) << I2C_RTC_DATA_S);
    reg_write(I2C_RTC_CONFIG2, temp);
    while reg_get_bit(I2C_RTC_CONFIG2, I2C_RTC_BUSY) != 0 {}

    regi2c_disable_block(block);
}

fn reg_write(reg: u32, v: u32) {
    unsafe {
        (reg as *mut u32).write_volatile(v);
    }
}

fn reg_get_bit(reg: u32, b: u32) -> u32 {
    unsafe { (reg as *mut u32).read_volatile() & b }
}

fn reg_get_field(reg: u32, s: u32, v: u32) -> u32 {
    unsafe { ((reg as *mut u32).read_volatile() >> s) & v }
}

fn reg_clr_bit(reg: u32, bit: u32) {
    unsafe {
        (reg as *mut u32).write_volatile((reg as *mut u32).read_volatile() & !bit);
    }
}

fn reg_set_field(reg: u32, field_v: u32, field_s: u32, value: u32) {
    unsafe {
        (reg as *mut u32).write_volatile(
            ((reg as *mut u32).read_volatile() & !(field_v << field_s))
                | ((value & field_v) << field_s),
        )
    }
}

fn reg_set_bit(reg: u32, bit: u32) {
    unsafe {
        (reg as *mut u32).write_volatile((reg as *mut u32).read_volatile() | bit);
    }
}

fn set_peri_reg_mask(reg: u32, mask: u32) {
    unsafe {
        (reg as *mut u32).write_volatile((reg as *mut u32).read_volatile() | mask);
    }
}

fn clear_peri_reg_mask(reg: u32, mask: u32) {
    unsafe {
        (reg as *mut u32).write_volatile((reg as *mut u32).read_volatile() & !mask);
    }
}
