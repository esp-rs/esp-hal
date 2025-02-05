use crate::{
    clock::{ApbClock, Clock, CpuClock, PllClock, XtalClock},
    peripherals::{LP_AON, MODEM_LPCON, MODEM_SYSCON, PCR, PMU},
    system::{RadioClockController, RadioPeripherals},
};

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

const REGI2C_BBPLL: u8 = 0x66;
const REGI2C_BIAS: u8 = 0x6a;
const REGI2C_PMU: u8 = 0x6d;
const REGI2C_ULP_CAL: u8 = 0x61;
const REGI2C_SAR_I2C: u8 = 0x69;

const I2C_MST_ANA_CONF1_M: u32 = 0x00FFFFFF;
const I2C_MST_ANA_CONF1_REG: u32 = I2C_MST_I2C0_CTRL_REG + 0x1c;

const REGI2C_BBPLL_RD_MASK: u32 = !(1 << 7) & I2C_MST_ANA_CONF1_M;
const REGI2C_BIAS_RD_MASK: u32 = !(1 << 6) & I2C_MST_ANA_CONF1_M;
const REGI2C_DIG_REG_RD_MASK: u32 = !(1 << 10) & I2C_MST_ANA_CONF1_M;
const REGI2C_ULP_CAL_RD_MASK: u32 = !(1 << 8) & I2C_MST_ANA_CONF1_M;
const REGI2C_SAR_I2C_RD_MASK: u32 = !(1 << 9) & I2C_MST_ANA_CONF1_M;

const REGI2C_RTC_SLAVE_ID_V: u8 = 0xFF;
const REGI2C_RTC_SLAVE_ID_S: u8 = 0;
const REGI2C_RTC_ADDR_V: u8 = 0xFF;
const REGI2C_RTC_ADDR_S: u8 = 8;
const REGI2C_RTC_WR_CNTL_S: u8 = 24;
const REGI2C_RTC_DATA_V: u8 = 0xFF;
const REGI2C_RTC_DATA_S: u8 = 16;

const I2C_MST_I2C0_CTRL_REG: u32 = 0x600AD800;
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
    PMU::regs().imm_hp_ck_power().modify(|_, w| {
        w.tie_high_xpd_bb_i2c().set_bit();
        w.tie_high_xpd_bbpll().set_bit();
        w.tie_high_xpd_bbpll_i2c().set_bit()
    });

    PMU::regs()
        .imm_hp_ck_power()
        .modify(|_, w| w.tie_high_global_bbpll_icg().set_bit());
}

pub(crate) fn esp32h2_rtc_update_to_xtal(freq: XtalClock, _div: u8) {
    crate::rom::ets_update_cpu_frequency_rom(freq.mhz());
    // Set divider from XTAL to APB clock. Need to set divider to 1 (reg. value 0)
    // first.
    clk_ll_ahb_set_divider(_div as u32);

    unsafe {
        PCR::regs()
            .cpu_freq_conf()
            .modify(|_, w| w.cpu_div_num().bits(_div - 1));
        // Switch clock source
        PCR::regs()
            .sysclk_conf()
            .modify(|_, w| w.soc_clk_sel().bits(0));
    }

    clk_ll_bus_update();
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
        PCR::regs()
            .sysclk_conf()
            .modify(|_, w| w.soc_clk_sel().bits(1));

        clk_ll_bus_update();
    }

    crate::rom::ets_update_cpu_frequency_rom(cpu_clock_speed.mhz());
}

pub(crate) fn esp32h2_rtc_apb_freq_update(apb_freq: ApbClock) {
    let value = ((apb_freq.hz() >> 12) & u16::MAX as u32)
        | (((apb_freq.hz() >> 12) & u16::MAX as u32) << 16);

    LP_AON::regs()
        .store5()
        .modify(|_, w| unsafe { w.lp_aon_store5().bits(value) });
}

fn clk_ll_cpu_set_divider(divider: u32) {
    assert!(divider >= 1);

    unsafe {
        PCR::regs()
            .cpu_freq_conf()
            .modify(|_, w| w.cpu_div_num().bits((divider - 1) as u8));
    }
}

fn clk_ll_ahb_set_divider(divider: u32) {
    assert!(divider >= 1);

    unsafe {
        PCR::regs()
            .ahb_freq_conf()
            .modify(|_, w| w.ahb_div_num().bits((divider - 1) as u8));
    }
}

fn clk_ll_bus_update() {
    PCR::regs()
        .bus_clk_update()
        .modify(|_, w| w.bus_clock_update().bit(true));

    // reg_get_bit
    while PCR::regs()
        .bus_clk_update()
        .read()
        .bus_clock_update()
        .bit_is_set()
    {}
}

fn regi2c_enable_block(block: u8) {
    crate::peripherals::MODEM_LPCON::regs()
        .clk_conf()
        .modify(|_, w| w.clk_i2c_mst_en().set_bit());

    // Before config I2C register, enable corresponding slave.

    let en_mask = match block {
        v if v == REGI2C_BBPLL => REGI2C_BBPLL_RD_MASK,
        v if v == REGI2C_BIAS => REGI2C_BIAS_RD_MASK,
        v if v == REGI2C_PMU => REGI2C_DIG_REG_RD_MASK,
        v if v == REGI2C_ULP_CAL => REGI2C_ULP_CAL_RD_MASK,
        v if v == REGI2C_SAR_I2C => REGI2C_SAR_I2C_RD_MASK,
        _ => return,
    };
    reg_set_bit(I2C_MST_ANA_CONF1_REG, en_mask)
}

fn regi2c_disable_block(block: u8) {
    let en_mask = match block {
        v if v == REGI2C_BBPLL => REGI2C_BBPLL_RD_MASK,
        v if v == REGI2C_BIAS => REGI2C_BIAS_RD_MASK,
        v if v == REGI2C_PMU => REGI2C_DIG_REG_RD_MASK,
        v if v == REGI2C_ULP_CAL => REGI2C_ULP_CAL_RD_MASK,
        v if v == REGI2C_SAR_I2C => REGI2C_SAR_I2C_RD_MASK,
        _ => return,
    };
    reg_clr_bit(I2C_MST_ANA_CONF1_REG, en_mask)
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
    assert!(msb < 8 + lsb);
    regi2c_enable_block(block);

    let block_shifted = (block as u32 & REGI2C_RTC_SLAVE_ID_V as u32) << REGI2C_RTC_SLAVE_ID_S;
    let reg_add_shifted = (reg_add as u32 & REGI2C_RTC_ADDR_V as u32) << REGI2C_RTC_ADDR_S;
    let write_bit = 1u32 << REGI2C_RTC_WR_CNTL_S;

    // Read the i2c bus register
    while reg_get_bit(I2C_MST_I2C0_CTRL_REG, REGI2C_RTC_BUSY) != 0 {}

    reg_write(I2C_MST_I2C0_CTRL_REG, block_shifted | reg_add_shifted);
    while reg_get_bit(I2C_MST_I2C0_CTRL_REG, REGI2C_RTC_BUSY) != 0 {}
    let mut temp = reg_get_field(
        I2C_MST_I2C0_CTRL_REG,
        REGI2C_RTC_DATA_S as u32,
        REGI2C_RTC_DATA_V as u32,
    );

    // Mask the value field
    temp &= (!(0xFFFFFFFF << lsb)) | (0xFFFFFFFF << (msb + 1));

    // Write the value into the temporary
    temp |= (data as u32 & (!(0xFFFFFFFF << (msb - lsb + 1)))) << lsb;

    let new_value = (temp & REGI2C_RTC_DATA_V as u32) << REGI2C_RTC_DATA_S;
    reg_write(
        I2C_MST_I2C0_CTRL_REG,
        block_shifted | reg_add_shifted | write_bit | new_value,
    );
    while reg_get_bit(I2C_MST_I2C0_CTRL_REG, REGI2C_RTC_BUSY) != 0 {}

    regi2c_disable_block(block);
}

impl RadioClockController for crate::peripherals::RADIO_CLK {
    fn enable(&mut self, peripheral: RadioPeripherals) {
        match peripheral {
            RadioPeripherals::Phy => enable_phy(true),
            RadioPeripherals::Bt | RadioPeripherals::Ieee802154 => {
                ble_ieee802154_clock_enable(true)
            }
        }
    }

    fn disable(&mut self, peripheral: RadioPeripherals) {
        match peripheral {
            RadioPeripherals::Phy => enable_phy(false),
            RadioPeripherals::Bt | RadioPeripherals::Ieee802154 => {
                ble_ieee802154_clock_enable(false)
            }
        }
    }

    fn reset_mac(&mut self) {
        reset_mac();
    }

    fn init_clocks(&mut self) {
        init_clocks();
    }

    fn ble_rtc_clk_init(&mut self) {
        // nothing for this target (yet)
    }

    fn reset_rpa(&mut self) {
        // nothing for this target (yet)
    }
}

fn enable_phy(en: bool) {
    MODEM_LPCON::regs()
        .clk_conf()
        .modify(|_, w| w.clk_i2c_mst_en().bit(en));
}

fn ble_ieee802154_clock_enable(en: bool) {
    MODEM_SYSCON::regs().clk_conf().modify(|_, w| {
        w.clk_zb_apb_en().bit(en);
        w.clk_zb_mac_en().bit(en)
    });

    MODEM_SYSCON::regs().clk_conf1().modify(|_, w| {
        w.clk_bt_apb_en().bit(en);
        w.clk_bt_en().bit(en);
        w.clk_fe_16m_en().bit(en);
        w.clk_fe_32m_en().bit(en);
        w.clk_fe_adc_en().bit(en);
        w.clk_fe_apb_en().bit(en);
        w.clk_fe_sdm_en().bit(en)
    });

    MODEM_LPCON::regs()
        .clk_conf()
        .modify(|_, w| w.clk_coex_en().bit(en));
}

fn reset_mac() {
    // empty
}

fn init_clocks() {
    unsafe {
        let pmu = PMU::regs();

        pmu.hp_sleep_icg_modem()
            .modify(|_, w| w.hp_sleep_dig_icg_modem_code().bits(0));
        pmu.hp_modem_icg_modem()
            .modify(|_, w| w.hp_modem_dig_icg_modem_code().bits(1));
        pmu.hp_active_icg_modem()
            .modify(|_, w| w.hp_active_dig_icg_modem_code().bits(2));
        pmu.imm_modem_icg()
            .write(|w| w.update_dig_icg_modem_en().set_bit());
        pmu.imm_sleep_sysclk()
            .write(|w| w.update_dig_icg_switch().set_bit());

        MODEM_LPCON::regs().clk_conf().modify(|_, w| {
            w.clk_i2c_mst_en()
                .set_bit()
                .clk_coex_en()
                .set_bit()
                .clk_fe_mem_en()
                .set_bit()
        });
    }
}
