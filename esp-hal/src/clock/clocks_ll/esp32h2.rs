use crate::clock::{ApbClock, Clock, CpuClock, PllClock, XtalClock};

const I2C_BBPLL: u8 = 0x66;
const I2C_BBPLL_HOSTID: u8 = 0;
const I2C_BBPLL_OC_REF_DIV: u8 = 2;
const I2C_BBPLL_OC_REF_DIV_MSB: u8 = 3;
const I2C_BBPLL_OC_REF_DIV_LSB: u8 = 0;

const I2C_BBPLL_OC_DIV: u8 = 3;
const I2C_BBPLL_OC_DIV_MSB: u8 = 5;
const I2C_BBPLL_OC_DIV_LSB: u8 = 0;

const I2C_BBPLL_OC_DHREF_SEL: u8 = 5;
const I2C_BBPLL_OC_DHREF_SEL_MSB: u8 = 5;
const I2C_BBPLL_OC_DHREF_SEL_LSB: u8 = 4;

const I2C_BBPLL_OC_DLREF_SEL: u8 = 5;
const I2C_BBPLL_OC_DLREF_SEL_MSB: u8 = 7;
const I2C_BBPLL_OC_DLREF_SEL_LSB: u8 = 6;

const I2C_MST_ANA_CONF0_REG: u32 = 0x600AD800 + 0x18;
const I2C_MST_BBPLL_STOP_FORCE_HIGH: u32 = 1 << 2;
const I2C_MST_BBPLL_STOP_FORCE_LOW: u32 = 1 << 3;
const I2C_MST_BBPLL_CAL_DONE: u32 = 1 << 24;

const MODEM_LPCON_CLK_CONF_FORCE_ON_REG: u32 = DR_REG_MODEM_LPCON_BASE + 0xc;
const MODEM_LPCON_CLK_I2C_MST_FO: u32 = 1 << 2;

// May be needed for enabling I2C clock
const MODEM_LPCON_I2C_CLK_CONF_REG: u32 = DR_REG_MODEM_LPCON_BASE + 0x8;
const MODEM_LPCON_CLK_I2C_SEL_96M: u32 = 1 << 0;

const DR_REG_MODEM_LPCON_BASE: u32 = 0x600AD000;
const MODEM_LPCON_CLK_CONF_REG: u32 = DR_REG_MODEM_LPCON_BASE + 0x8;
const MODEM_LPCON_CLK_I2C_MST_EN: u32 = 1 << 2;

const DR_REG_I2C_ANA_MST_BASE: u32 = 0x600AD800;
const I2C_MST_DATE_REG: u32 = DR_REG_I2C_ANA_MST_BASE + 0x34;
const I2C_MST_ANA_CONF2_REG: u32 = DR_REG_I2C_ANA_MST_BASE + 0x20;
const I2C_MST_ANA_CONF2: u32 = 0x00FFFFFF;
const I2C_MST_CLK_EN: u32 = 1 << 28;

const REGI2C_BBPLL: u8 = 0x66;
const REGI2C_BIAS: u8 = 0x6a;
const REGI2C_PMU_REG: u8 = 0x6d;
const REGI2C_ULP_CAL: u8 = 0x61;
const REGI2C_SAR_I2C: u8 = 0x69;

const REGI2C_BBPLL_DEVICE_EN: u32 = 1 << 9; // (1 << 5) << 4;
const REGI2C_BIAS_DEVICE_EN: u32 = 1 << 8; // (1 << 4) << 4;
const REGI2C_PMU_DEVICE_EN: u32 = 1 << 12; // (1 << 8) << 4;
const REGI2C_ULP_CAL_DEVICE_EN: u32 = 1 << 10; // (1 << 6) << 4;
const REGI2C_SAR_I2C_DEVICE_EN: u32 = 1 << 11; // (1 << 7) << 4;

const REGI2C_RTC_SLAVE_ID_V: u8 = 0xFF;
const REGI2C_RTC_SLAVE_ID_S: u8 = 0;
const REGI2C_RTC_ADDR_V: u8 = 0xFF;
const REGI2C_RTC_ADDR_S: u8 = 8;
const REGI2C_RTC_WR_CNTL_V: u8 = 0x1;
const REGI2C_RTC_WR_CNTL_S: u8 = 24;
const REGI2C_RTC_DATA_V: u8 = 0xFF;
const REGI2C_RTC_DATA_S: u8 = 16;

const I2C_MST_I2C0_CTRL_REG: u32 = DR_REG_I2C_ANA_MST_BASE;
const REGI2C_RTC_BUSY: u32 = 1 << 25;

pub(crate) fn esp32h2_rtc_bbpll_configure(_xtal_freq: XtalClock, _pll_freq: PllClock) {
    unsafe {
        // Enable I2C master clock
        (MODEM_LPCON_CLK_CONF_FORCE_ON_REG as *mut u32).write_volatile(
            (MODEM_LPCON_CLK_CONF_FORCE_ON_REG as *mut u32).read_volatile()
                | MODEM_LPCON_CLK_I2C_MST_FO,
        );

        // Set I2C clock to 96MHz
        (MODEM_LPCON_I2C_CLK_CONF_REG as *mut u32).write_volatile(
            (MODEM_LPCON_I2C_CLK_CONF_REG as *mut u32).read_volatile()
                | MODEM_LPCON_CLK_I2C_SEL_96M,
        );

        let i2c_mst_ana_conf0_reg_ptr = I2C_MST_ANA_CONF0_REG as *mut u32;

        // BPPLL calibration start
        i2c_mst_ana_conf0_reg_ptr.write_volatile(
            i2c_mst_ana_conf0_reg_ptr.read_volatile() & !I2C_MST_BBPLL_STOP_FORCE_HIGH,
        );
        i2c_mst_ana_conf0_reg_ptr.write_volatile(
            i2c_mst_ana_conf0_reg_ptr.read_volatile() | I2C_MST_BBPLL_STOP_FORCE_LOW,
        );

        let oc_ref_div = 0u32;
        let oc_div = 1u32;
        let oc_dhref_sel = 3u32;
        let oc_dlref_sel = 1u32;

        regi2c_write_mask(
            I2C_BBPLL,
            I2C_BBPLL_HOSTID,
            I2C_BBPLL_OC_REF_DIV,
            I2C_BBPLL_OC_REF_DIV_MSB,
            I2C_BBPLL_OC_REF_DIV_LSB,
            oc_ref_div as u8,
        );

        regi2c_write_mask(
            I2C_BBPLL,
            I2C_BBPLL_HOSTID,
            I2C_BBPLL_OC_DIV,
            I2C_BBPLL_OC_DIV_MSB,
            I2C_BBPLL_OC_DIV_LSB,
            oc_div as u8,
        );

        regi2c_write_mask(
            I2C_BBPLL,
            I2C_BBPLL_HOSTID,
            I2C_BBPLL_OC_DHREF_SEL,
            I2C_BBPLL_OC_DHREF_SEL_MSB,
            I2C_BBPLL_OC_DHREF_SEL_LSB,
            oc_dhref_sel as u8,
        );

        regi2c_write_mask(
            I2C_BBPLL,
            I2C_BBPLL_HOSTID,
            I2C_BBPLL_OC_DLREF_SEL,
            I2C_BBPLL_OC_DLREF_SEL_MSB,
            I2C_BBPLL_OC_DLREF_SEL_LSB,
            oc_dlref_sel as u8,
        );

        // WAIT CALIBRATION DONE
        while (i2c_mst_ana_conf0_reg_ptr.read_volatile() & I2C_MST_BBPLL_CAL_DONE) == 0 {}

        // BBPLL CALIBRATION STOP
        i2c_mst_ana_conf0_reg_ptr.write_volatile(
            i2c_mst_ana_conf0_reg_ptr.read_volatile() | I2C_MST_BBPLL_STOP_FORCE_HIGH,
        );
        i2c_mst_ana_conf0_reg_ptr.write_volatile(
            i2c_mst_ana_conf0_reg_ptr.read_volatile() & !I2C_MST_BBPLL_STOP_FORCE_LOW,
        );
    }
}

pub(crate) fn esp32h2_rtc_bbpll_enable() {
    let pmu = unsafe { &*crate::peripherals::PMU::PTR };

    pmu.imm_hp_ck_power().modify(|_, w| {
        w.tie_high_xpd_bb_i2c()
            .set_bit()
            .tie_high_xpd_bbpll()
            .set_bit()
            .tie_high_xpd_bbpll_i2c()
            .set_bit()
    });

    pmu.imm_hp_ck_power()
        .modify(|_, w| w.tie_high_global_bbpll_icg().set_bit());
}

pub(crate) fn esp32h2_rtc_update_to_xtal(freq: XtalClock, _div: u8) {
    unsafe {
        let pcr = &*crate::peripherals::PCR::PTR;
        crate::rom::ets_update_cpu_frequency_rom(freq.mhz());
        // Set divider from XTAL to APB clock. Need to set divider to 1 (reg. value 0)
        // first.
        clk_ll_ahb_set_divider(_div as u32);

        pcr.cpu_freq_conf()
            .modify(|_, w| w.cpu_div_num().bits(_div - 1));
        // Switch clock source
        pcr.sysclk_conf().modify(|_, w| w.soc_clk_sel().bits(0));

        clk_ll_bus_update();
    }
}

pub(crate) fn esp32h2_rtc_freq_to_pll_mhz(cpu_clock_speed: CpuClock) {
    let cpu_divider = 96 / cpu_clock_speed.mhz();
    clk_ll_cpu_set_divider(cpu_divider);
    let ahb_divider = match cpu_divider {
        1 => 3,
        2 => 4,
        _ => cpu_divider,
    };
    clk_ll_ahb_set_divider(ahb_divider);

    unsafe {
        (*crate::peripherals::PCR::PTR)
            .sysclk_conf()
            .modify(|_, w| w.soc_clk_sel().bits(1));

        clk_ll_bus_update();
    }

    crate::rom::ets_update_cpu_frequency_rom(cpu_clock_speed.mhz());
}

pub(crate) fn esp32h2_rtc_apb_freq_update(apb_freq: ApbClock) {
    let lp_aon = unsafe { &*crate::peripherals::LP_AON::ptr() };
    let value = ((apb_freq.hz() >> 12) & u16::MAX as u32)
        | (((apb_freq.hz() >> 12) & u16::MAX as u32) << 16);

    lp_aon
        .store5()
        .modify(|_, w| unsafe { w.lp_aon_store5().bits(value) });
}

fn clk_ll_cpu_set_divider(divider: u32) {
    assert!(divider >= 1);

    unsafe {
        let pcr = &*crate::peripherals::PCR::PTR;
        pcr.cpu_freq_conf()
            .modify(|_, w| w.cpu_div_num().bits((divider - 1) as u8));
    }
}

fn clk_ll_ahb_set_divider(divider: u32) {
    assert!(divider >= 1);

    unsafe {
        let pcr = &*crate::peripherals::PCR::PTR;
        pcr.ahb_freq_conf()
            .modify(|_, w| w.ahb_div_num().bits((divider - 1) as u8));
    }
}

fn clk_ll_bus_update() {
    unsafe {
        let pcr = &*crate::peripherals::PCR::PTR;

        pcr.bus_clk_update()
            .modify(|_, w| w.bus_clock_update().bit(true));

        // reg_get_bit
        while pcr.bus_clk_update().read().bus_clock_update().bit_is_set() {}
    }
}

fn regi2c_enable_block(block: u8) {
    reg_set_bit(MODEM_LPCON_CLK_CONF_REG, MODEM_LPCON_CLK_I2C_MST_EN);
    reg_set_bit(I2C_MST_DATE_REG, I2C_MST_CLK_EN);

    // Make I2C_MST_ANA_CONF2 in I2C_MST_ANA_CONF2_REG be 0
    unsafe {
        (I2C_MST_ANA_CONF2_REG as *mut u32).write_volatile(
            // (1 << 18)
            (I2C_MST_ANA_CONF2_REG as *mut u32).read_volatile() & !I2C_MST_ANA_CONF2,
        );
    }

    // Before config I2C register, enable corresponding slave.
    match block {
        REGI2C_BBPLL => {
            reg_set_bit(I2C_MST_ANA_CONF2_REG, REGI2C_BBPLL_DEVICE_EN);
        }
        REGI2C_BIAS => {
            reg_set_bit(I2C_MST_ANA_CONF2_REG, REGI2C_BIAS_DEVICE_EN);
        }
        REGI2C_PMU_REG => {
            reg_set_bit(I2C_MST_ANA_CONF2_REG, REGI2C_PMU_DEVICE_EN);
        }
        REGI2C_ULP_CAL => {
            reg_set_bit(I2C_MST_ANA_CONF2_REG, REGI2C_ULP_CAL_DEVICE_EN);
        }
        REGI2C_SAR_I2C => {
            reg_set_bit(I2C_MST_ANA_CONF2_REG, REGI2C_SAR_I2C_DEVICE_EN);
        }
        _ => (),
    }
}

fn regi2c_disable_block(block: u8) {
    match block {
        REGI2C_BBPLL => {
            reg_clr_bit(I2C_MST_ANA_CONF2_REG, REGI2C_BBPLL_DEVICE_EN);
        }
        REGI2C_BIAS => {
            reg_clr_bit(I2C_MST_ANA_CONF2_REG, REGI2C_BIAS_DEVICE_EN);
        }
        REGI2C_PMU_REG => {
            reg_clr_bit(I2C_MST_ANA_CONF2_REG, REGI2C_PMU_DEVICE_EN);
        }
        REGI2C_ULP_CAL => {
            reg_clr_bit(I2C_MST_ANA_CONF2_REG, REGI2C_ULP_CAL_DEVICE_EN);
        }
        REGI2C_SAR_I2C => {
            reg_clr_bit(I2C_MST_ANA_CONF2_REG, REGI2C_SAR_I2C_DEVICE_EN);
        }
        _ => (),
    }
}

fn reg_set_bit(reg: u32, bit: u32) {
    unsafe {
        (reg as *mut u32).write_volatile((reg as *mut u32).read_volatile() | bit);
    }
}

fn reg_clr_bit(reg: u32, bit: u32) {
    unsafe {
        (reg as *mut u32).write_volatile((reg as *mut u32).read_volatile() & !bit);
    }
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

pub(crate) fn regi2c_write_mask(block: u8, _host_id: u8, reg_add: u8, msb: u8, lsb: u8, data: u8) {
    assert!(msb - lsb < 8);
    regi2c_enable_block(block);

    // Read the i2c bus register
    let mut temp: u32 = ((block as u32 & REGI2C_RTC_SLAVE_ID_V as u32)
        << REGI2C_RTC_SLAVE_ID_S as u32)
        | (reg_add as u32 & REGI2C_RTC_ADDR_V as u32) << REGI2C_RTC_ADDR_S as u32;
    reg_write(I2C_MST_I2C0_CTRL_REG, temp);
    while reg_get_bit(I2C_MST_I2C0_CTRL_REG, REGI2C_RTC_BUSY) != 0 {}
    temp = reg_get_field(
        I2C_MST_I2C0_CTRL_REG,
        REGI2C_RTC_DATA_S as u32,
        REGI2C_RTC_DATA_V as u32,
    );
    // Write the i2c bus register
    temp &= (!(0xFFFFFFFF << lsb)) | (0xFFFFFFFF << (msb + 1));
    temp |= (data as u32 & (!(0xFFFFFFFF << (msb as u32 - lsb as u32 + 1)))) << (lsb as u32);
    temp = ((block as u32 & REGI2C_RTC_SLAVE_ID_V as u32) << REGI2C_RTC_SLAVE_ID_S as u32)
        | ((reg_add as u32 & REGI2C_RTC_ADDR_V as u32) << REGI2C_RTC_ADDR_S as u32)
        | ((0x1 & REGI2C_RTC_WR_CNTL_V as u32) << REGI2C_RTC_WR_CNTL_S as u32)
        | ((temp & REGI2C_RTC_DATA_V as u32) << REGI2C_RTC_DATA_S as u32);
    reg_write(I2C_MST_I2C0_CTRL_REG, temp);
    while reg_get_bit(I2C_MST_I2C0_CTRL_REG, REGI2C_RTC_BUSY) != 0 {}

    regi2c_disable_block(block);
}
