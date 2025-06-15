use crate::{
    clock::{ApbClock, Clock, CpuClock, PllClock, XtalClock},
    peripherals::{I2C_ANA_MST, LP_AON, MODEM_LPCON, MODEM_SYSCON, PCR, PMU},
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

// May be needed for enabling I2C clock
const MODEM_LPCON_CLK_I2C_SEL_96M: u32 = 1 << 0;

const REGI2C_BBPLL: u8 = 0x66;
const REGI2C_BIAS: u8 = 0x6a;
const REGI2C_PMU: u8 = 0x6d;
const REGI2C_ULP_CAL: u8 = 0x61;
const REGI2C_SAR_I2C: u8 = 0x69;

const I2C_MST_ANA_CONF1_M: u32 = 0x00FFFFFF;

pub(crate) fn esp32h2_rtc_bbpll_configure(_xtal_freq: XtalClock, _pll_freq: PllClock) {
    // Enable I2C master clock
    MODEM_LPCON::regs()
        .clk_conf_force_on()
        .modify(|_, w| w.clk_i2c_mst_fo().set_bit());

    // Set I2C clock to 96MHz
    MODEM_LPCON::regs()
        .clk_conf()
        .modify(|r, w| unsafe { w.bits(r.bits() | MODEM_LPCON_CLK_I2C_SEL_96M) });

    // BPPLL calibration start
    I2C_ANA_MST::regs().ana_conf0().modify(|_, w| {
        w.bbpll_stop_force_high().clear_bit();
        w.bbpll_stop_force_low().set_bit()
    });

    let oc_ref_div = 0;
    let oc_div = 1;
    let oc_dhref_sel = 3;
    let oc_dlref_sel = 1;

    regi2c_write_mask(
        I2C_BBPLL,
        I2C_BBPLL_HOSTID,
        I2C_BBPLL_OC_REF_DIV,
        I2C_BBPLL_OC_REF_DIV_MSB,
        I2C_BBPLL_OC_REF_DIV_LSB,
        oc_ref_div,
    );

    regi2c_write_mask(
        I2C_BBPLL,
        I2C_BBPLL_HOSTID,
        I2C_BBPLL_OC_DIV,
        I2C_BBPLL_OC_DIV_MSB,
        I2C_BBPLL_OC_DIV_LSB,
        oc_div,
    );

    regi2c_write_mask(
        I2C_BBPLL,
        I2C_BBPLL_HOSTID,
        I2C_BBPLL_OC_DHREF_SEL,
        I2C_BBPLL_OC_DHREF_SEL_MSB,
        I2C_BBPLL_OC_DHREF_SEL_LSB,
        oc_dhref_sel,
    );

    regi2c_write_mask(
        I2C_BBPLL,
        I2C_BBPLL_HOSTID,
        I2C_BBPLL_OC_DLREF_SEL,
        I2C_BBPLL_OC_DLREF_SEL_MSB,
        I2C_BBPLL_OC_DLREF_SEL_LSB,
        oc_dlref_sel,
    );

    // WAIT CALIBRATION DONE
    while I2C_ANA_MST::regs()
        .ana_conf0()
        .read()
        .cal_done()
        .bit_is_clear()
    {}

    // workaround bbpll calibration might stop early
    crate::rom::ets_delay_us(10);

    // BBPLL CALIBRATION STOP
    I2C_ANA_MST::regs().ana_conf0().modify(|_, w| {
        w.bbpll_stop_force_high().set_bit();
        w.bbpll_stop_force_low().clear_bit()
    });
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

pub(crate) fn esp32h2_rtc_update_to_xtal(freq: XtalClock, div: u8) {
    crate::rom::ets_update_cpu_frequency_rom(freq.mhz());
    // Set divider from XTAL to APB clock. Need to set divider to 1 (reg. value 0)
    // first.
    clk_ll_ahb_set_divider(div as u32);

    PCR::regs()
        .cpu_freq_conf()
        .modify(|_, w| unsafe { w.cpu_div_num().bits(div - 1) });
    // Switch clock source
    PCR::regs()
        .sysclk_conf()
        .modify(|_, w| unsafe { w.soc_clk_sel().bits(0) });

    clk_ll_bus_update();
}

pub(crate) fn esp32h2_rtc_freq_to_pll_mhz(cpu_clock_speed: CpuClock) {
    let cpu_divider = 96 / cpu_clock_speed.mhz();
    clk_ll_cpu_set_divider(cpu_divider);
    let ahb_divider = match cpu_divider {
        1 | 2 => cpu_divider + 2,
        _ => cpu_divider,
    };
    clk_ll_ahb_set_divider(ahb_divider);

    PCR::regs()
        .sysclk_conf()
        .modify(|_, w| unsafe { w.soc_clk_sel().bits(1) });

    clk_ll_bus_update();

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

    PCR::regs()
        .cpu_freq_conf()
        .modify(|_, w| unsafe { w.cpu_div_num().bits((divider - 1) as u8) });
}

fn clk_ll_ahb_set_divider(divider: u32) {
    assert!(divider >= 1);

    PCR::regs()
        .ahb_freq_conf()
        .modify(|_, w| unsafe { w.ahb_div_num().bits((divider - 1) as u8) });
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

fn regi2c_enable_block(block: u8) -> usize {
    MODEM_LPCON::regs()
        .clk_conf()
        .modify(|_, w| w.clk_i2c_mst_en().set_bit());

    // Before config I2C register, enable corresponding slave.
    let i2c_sel_bits = I2C_ANA_MST::regs().ana_conf2().read();
    let i2c_sel = match block {
        v if v == REGI2C_BBPLL => i2c_sel_bits.bbpll_mst_sel().bit_is_set(),
        v if v == REGI2C_BIAS => i2c_sel_bits.bias_mst_sel().bit_is_set(),
        v if v == REGI2C_PMU => i2c_sel_bits.dig_reg_mst_sel().bit_is_set(),
        v if v == REGI2C_ULP_CAL => i2c_sel_bits.ulp_cal_mst_sel().bit_is_set(),
        v if v == REGI2C_SAR_I2C => i2c_sel_bits.sar_i2c_mst_sel().bit_is_set(),
        _ => unreachable!(),
    };
    I2C_ANA_MST::regs().ana_conf1().write(|w| unsafe {
        w.bits(I2C_MST_ANA_CONF1_M);
        match block {
            v if v == REGI2C_BBPLL => w.bbpll_rd().clear_bit(),
            v if v == REGI2C_BIAS => w.bias_rd().clear_bit(),
            v if v == REGI2C_PMU => w.dig_reg_rd().clear_bit(),
            v if v == REGI2C_ULP_CAL => w.ulp_cal_rd().clear_bit(),
            v if v == REGI2C_SAR_I2C => w.sar_i2c_rd().clear_bit(),
            _ => unreachable!(),
        }
    });

    if i2c_sel { 0 } else { 1 }
}

pub(crate) fn regi2c_write_mask(block: u8, _host_id: u8, reg_add: u8, msb: u8, lsb: u8, data: u8) {
    assert!(msb < 8 + lsb);
    let master = regi2c_enable_block(block);

    // Read the i2c bus register
    I2C_ANA_MST::regs().i2c_ctrl(master).write(|w| unsafe {
        w.slave_addr().bits(block);
        w.slave_reg_addr().bits(reg_add)
    });

    while I2C_ANA_MST::regs().i2c_ctrl(master).read().busy().bit() {}

    // Example: LSB=2, MSB = 5
    // unwritten_bits = 1100 0011
    // data_mask      = 0000 1111
    // data_bits      = 00xx xx00
    let unwritten_bits = (!(u32::MAX << lsb) | (u32::MAX << (msb + 1))) as u8;
    let data_mask = !(u32::MAX << (msb - lsb + 1)) as u8;
    let data_bits = (data & data_mask) << lsb;

    I2C_ANA_MST::regs().i2c_ctrl(master).modify(|r, w| unsafe {
        w.slave_addr().bits(block);
        w.slave_reg_addr().bits(reg_add);
        w.read_write().set_bit();
        w.data()
            .bits((r.data().bits() & unwritten_bits) | data_bits)
    });

    while I2C_ANA_MST::regs().i2c_ctrl(master).read().busy().bit() {}
}

pub(super) fn enable_phy(en: bool) {
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

pub(super) fn enable_bt(en: bool) {
    ble_ieee802154_clock_enable(en);
}

pub(super) fn enable_ieee802154(en: bool) {
    ble_ieee802154_clock_enable(en);
}

pub(super) fn reset_mac() {
    // empty
}

pub(super) fn init_clocks() {
    PMU::regs()
        .hp_sleep_icg_modem()
        .modify(|_, w| unsafe { w.hp_sleep_dig_icg_modem_code().bits(0) });
    PMU::regs()
        .hp_modem_icg_modem()
        .modify(|_, w| unsafe { w.hp_modem_dig_icg_modem_code().bits(1) });
    PMU::regs()
        .hp_active_icg_modem()
        .modify(|_, w| unsafe { w.hp_active_dig_icg_modem_code().bits(2) });
    PMU::regs()
        .imm_modem_icg()
        .write(|w| w.update_dig_icg_modem_en().set_bit());
    PMU::regs()
        .imm_sleep_sysclk()
        .write(|w| w.update_dig_icg_switch().set_bit());

    MODEM_LPCON::regs().clk_conf().modify(|_, w| {
        w.clk_i2c_mst_en().set_bit();
        w.clk_coex_en().set_bit();
        w.clk_fe_mem_en().set_bit()
    });
}

pub(super) fn ble_rtc_clk_init() {
    // nothing for this target (yet)
}

pub(super) fn reset_rpa() {
    // nothing for this target (yet)
}
