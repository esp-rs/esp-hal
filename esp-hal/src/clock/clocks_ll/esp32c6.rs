use crate::{
    clock::{ApbClock, Clock, CpuClock, PllClock, XtalClock},
    rtc_cntl::rtc::CpuClockSource,
};

const I2C_BBPLL: u8 = 0x66;
const I2C_BBPLL_HOSTID: u8 = 0;

const I2C_BBPLL_OC_REF_DIV: u8 = 2;
const I2C_BBPLL_OC_DCHGP_LSB: u32 = 4;

const I2C_BBPLL_OC_DIV_7_0: u8 = 3;

const I2C_BBPLL_OC_DR1: u8 = 5;
const I2C_BBPLL_OC_DR1_MSB: u8 = 2;
const I2C_BBPLL_OC_DR1_LSB: u8 = 0;

const I2C_BBPLL_OC_DR3: u8 = 5;
const I2C_BBPLL_OC_DR3_MSB: u8 = 6;
const I2C_BBPLL_OC_DR3_LSB: u8 = 4;

const I2C_BBPLL_OC_DCUR: u8 = 6;

const I2C_BBPLL_OC_DHREF_SEL_LSB: u32 = 4;

const I2C_BBPLL_OC_DLREF_SEL_LSB: u32 = 6;

const I2C_BBPLL_OC_VCO_DBIAS: u8 = 9;
const I2C_BBPLL_OC_VCO_DBIAS_MSB: u8 = 1;
const I2C_BBPLL_OC_VCO_DBIAS_LSB: u8 = 0;

// Analog function control register
const I2C_MST_ANA_CONF0_REG: u32 = 0x600AF818;
const I2C_MST_BBPLL_STOP_FORCE_HIGH: u32 = 1 << 2;
const I2C_MST_BBPLL_STOP_FORCE_LOW: u32 = 1 << 3;
const I2C_MST_BBPLL_CAL_DONE: u32 = 1 << 24;

unsafe fn modem_lpcon<'a>() -> &'a esp32c6::modem_lpcon::RegisterBlock {
    &*esp32c6::MODEM_LPCON::ptr()
}

unsafe fn pcr<'a>() -> &'a esp32c6::pcr::RegisterBlock {
    &*esp32c6::PCR::ptr()
}

// rtc_clk_bbpll_configure
pub(crate) fn esp32c6_rtc_bbpll_configure(xtal_freq: XtalClock, pll_freq: PllClock) {
    esp32c6_rtc_bbpll_configure_raw(xtal_freq.mhz(), pll_freq.mhz())
}

pub(crate) fn esp32c6_rtc_bbpll_configure_raw(_xtal_freq: u32, pll_freq: u32) {
    // clk_ll_bbpll_set_freq_mhz
    // The target SPLL is fixed to 480MHz
    // Do nothing
    debug_assert!(pll_freq == 480);

    critical_section::with(|_| unsafe {
        // enable i2c mst clk by force on (temporarily)
        let was_i2c_mst_en = modem_lpcon().clk_conf().read().clk_i2c_mst_en().bit();
        modem_lpcon()
            .clk_conf()
            .modify(|_, w| w.clk_i2c_mst_en().set_bit());

        modem_lpcon()
            .i2c_mst_clk_conf()
            .modify(|_, w| w.clk_i2c_mst_sel_160m().set_bit());

        let i2c_mst_ana_conf0_reg_ptr = I2C_MST_ANA_CONF0_REG as *mut u32;
        // BBPLL CALIBRATION START
        i2c_mst_ana_conf0_reg_ptr.write_volatile(
            i2c_mst_ana_conf0_reg_ptr.read_volatile() & !I2C_MST_BBPLL_STOP_FORCE_HIGH,
        );
        i2c_mst_ana_conf0_reg_ptr.write_volatile(
            i2c_mst_ana_conf0_reg_ptr.read_volatile() | I2C_MST_BBPLL_STOP_FORCE_LOW,
        );

        let div_ref = 0u32;
        let div7_0 = 8u32;
        let dr1 = 0u32;
        let dr3 = 0u32;
        let dchgp = 5u32;
        let dcur = 3u32;
        let dbias = 2u32;

        let i2c_bbpll_lref = (dchgp << I2C_BBPLL_OC_DCHGP_LSB) | div_ref;
        let i2c_bbpll_div_7_0 = div7_0;
        let i2c_bbpll_dcur =
            (1 << I2C_BBPLL_OC_DLREF_SEL_LSB) | (3 << I2C_BBPLL_OC_DHREF_SEL_LSB) | dcur;

        regi2c_write(
            I2C_BBPLL,
            I2C_BBPLL_HOSTID,
            I2C_BBPLL_OC_REF_DIV,
            i2c_bbpll_lref as u8,
        );
        regi2c_write(
            I2C_BBPLL,
            I2C_BBPLL_HOSTID,
            I2C_BBPLL_OC_DIV_7_0,
            i2c_bbpll_div_7_0 as u8,
        );
        regi2c_write_mask(
            I2C_BBPLL,
            I2C_BBPLL_HOSTID,
            I2C_BBPLL_OC_DR1,
            I2C_BBPLL_OC_DR1_MSB,
            I2C_BBPLL_OC_DR1_LSB,
            dr1 as u8,
        );
        regi2c_write_mask(
            I2C_BBPLL,
            I2C_BBPLL_HOSTID,
            I2C_BBPLL_OC_DR3,
            I2C_BBPLL_OC_DR3_MSB,
            I2C_BBPLL_OC_DR3_LSB,
            dr3 as u8,
        );
        regi2c_write(
            I2C_BBPLL,
            I2C_BBPLL_HOSTID,
            I2C_BBPLL_OC_DCUR,
            i2c_bbpll_dcur as u8,
        );
        regi2c_write_mask(
            I2C_BBPLL,
            I2C_BBPLL_HOSTID,
            I2C_BBPLL_OC_VCO_DBIAS,
            I2C_BBPLL_OC_VCO_DBIAS_MSB,
            I2C_BBPLL_OC_VCO_DBIAS_LSB,
            dbias as u8,
        );

        // WAIT CALIBRATION DONE
        while (i2c_mst_ana_conf0_reg_ptr.read_volatile() & I2C_MST_BBPLL_CAL_DONE) == 0 {}

        // workaround bbpll calibration might stop early
        crate::rom::ets_delay_us(10);

        // BBPLL CALIBRATION STOP
        i2c_mst_ana_conf0_reg_ptr.write_volatile(
            i2c_mst_ana_conf0_reg_ptr.read_volatile() & !I2C_MST_BBPLL_STOP_FORCE_LOW,
        );
        i2c_mst_ana_conf0_reg_ptr.write_volatile(
            i2c_mst_ana_conf0_reg_ptr.read_volatile() | I2C_MST_BBPLL_STOP_FORCE_HIGH,
        );

        modem_lpcon()
            .clk_conf()
            .modify(|_, w| w.clk_i2c_mst_en().bit(was_i2c_mst_en));
    });
}

pub(crate) fn esp32c6_rtc_bbpll_enable() {
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

pub(crate) fn esp32c6_rtc_update_to_xtal(freq: XtalClock, div: u8) {
    esp32c6_rtc_update_to_xtal_raw(freq.mhz(), div)
}

pub(crate) fn esp32c6_rtc_update_to_xtal_raw(freq_mhz: u32, div: u8) {
    esp32c6_ahb_set_ls_divider(div);
    esp32c6_cpu_set_ls_divider(div);

    CpuClockSource::Xtal.select();

    crate::rom::ets_update_cpu_frequency_rom(freq_mhz);
}

pub(crate) fn esp32c6_rtc_update_to_8m() {
    esp32c6_ahb_set_ls_divider(1);
    esp32c6_cpu_set_ls_divider(1);

    CpuClockSource::RcFast.select();

    crate::rom::ets_update_cpu_frequency_rom(20);
}

pub(crate) fn esp32c6_rtc_freq_to_pll_mhz(cpu_clock_speed: CpuClock) {
    esp32c6_rtc_freq_to_pll_mhz_raw(cpu_clock_speed.mhz());
}

pub(crate) fn esp32c6_rtc_freq_to_pll_mhz_raw(cpu_clock_speed_mhz: u32) {
    // On ESP32C6, MSPI source clock's default HS divider leads to 120MHz, which is
    // unusable before calibration Therefore, before switching SOC_ROOT_CLK to
    // HS, we need to set MSPI source clock HS divider to make it run at
    // 80MHz after the switch. PLL = 480MHz, so divider is 6.
    clk_ll_mspi_fast_set_hs_divider(6);

    let pcr = unsafe { &*crate::peripherals::PCR::PTR };
    unsafe {
        pcr.cpu_freq_conf().modify(|_, w| {
            w.cpu_hs_div_num()
                .bits(((480 / cpu_clock_speed_mhz / 3) - 1) as u8)
                .cpu_hs_120m_force()
                .clear_bit()
        });

        pcr.cpu_freq_conf()
            .modify(|_, w| w.cpu_hs_120m_force().clear_bit());

        CpuClockSource::Pll.select();
    }

    crate::rom::ets_update_cpu_frequency_rom(cpu_clock_speed_mhz);
}

pub(crate) fn esp32c6_rtc_apb_freq_update(apb_freq: ApbClock) {
    let lp_aon = unsafe { &*crate::peripherals::LP_AON::ptr() };
    let value = ((apb_freq.hz() >> 12) & u16::MAX as u32)
        | (((apb_freq.hz() >> 12) & u16::MAX as u32) << 16);

    lp_aon
        .store5()
        .modify(|_, w| unsafe { w.lp_aon_store5().bits(value) });
}

fn clk_ll_mspi_fast_set_hs_divider(divider: u32) {
    // SOC_ROOT_CLK ------> MSPI_FAST_CLK
    // HS divider option: 4, 5, 6 (PCR_MSPI_FAST_HS_DIV_NUM=3, 4, 5)
    let pcr = unsafe { &*crate::peripherals::PCR::PTR };

    unsafe {
        match divider {
            4 => pcr
                .mspi_clk_conf()
                .modify(|_, w| w.mspi_fast_hs_div_num().bits(3)),
            5 => pcr
                .mspi_clk_conf()
                .modify(|_, w| w.mspi_fast_hs_div_num().bits(4)),
            6 => pcr
                .mspi_clk_conf()
                .modify(|_, w| w.mspi_fast_hs_div_num().bits(5)),
            _ => panic!("Unsupported HS MSPI_FAST divider"),
        }
    }
}

const REGI2C_BIAS: u8 = 0x6a;
const REGI2C_DIG_REG: u8 = 0x6d;
const REGI2C_ULP_CAL: u8 = 0x61;
const REGI2C_SAR_I2C: u8 = 0x69;

const REGI2C_RTC_SLAVE_ID_V: u8 = 0xFF;
const REGI2C_RTC_SLAVE_ID_S: u8 = 0;
const REGI2C_RTC_ADDR_V: u8 = 0xFF;
const REGI2C_RTC_ADDR_S: u8 = 8;
const REGI2C_RTC_WR_CNTL_V: u8 = 0x1;
const REGI2C_RTC_WR_CNTL_S: u8 = 24;
const REGI2C_RTC_DATA_V: u8 = 0xFF;
const REGI2C_RTC_DATA_S: u8 = 16;

const REGI2C_BBPLL: u8 = 0x66;

const DR_REG_LP_I2C_ANA_MST_BASE: u32 = 0x600B2400;
const LP_I2C_ANA_MST_DEVICE_EN_REG: u32 = DR_REG_LP_I2C_ANA_MST_BASE + 0x14;
const REGI2C_BBPLL_DEVICE_EN: u32 = 1 << 5;
const REGI2C_BIAS_DEVICE_EN: u32 = 1 << 4;
const REGI2C_DIG_REG_DEVICE_EN: u32 = 1 << 8;
const REGI2C_ULP_CAL_DEVICE_EN: u32 = 1 << 6;
const REGI2C_SAR_I2C_DEVICE_EN: u32 = 1 << 7;

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

fn regi2c_enable_block(block: u8) {
    let modem_lpcon = unsafe { &*crate::peripherals::MODEM_LPCON::ptr() };
    let lp_i2c_ana = unsafe { &*crate::peripherals::LP_I2C_ANA_MST::ptr() };

    modem_lpcon
        .clk_conf()
        .modify(|_, w| w.clk_i2c_mst_en().set_bit());

    modem_lpcon
        .i2c_mst_clk_conf()
        .modify(|_, w| w.clk_i2c_mst_sel_160m().set_bit());

    lp_i2c_ana
        .date()
        .modify(|_, w| w.lp_i2c_ana_mast_i2c_mat_clk_en().set_bit());

    // Before config I2C register, enable corresponding slave.
    match block {
        v if v == REGI2C_BBPLL => {
            reg_set_bit(LP_I2C_ANA_MST_DEVICE_EN_REG, REGI2C_BBPLL_DEVICE_EN);
        }
        v if v == REGI2C_BIAS => {
            reg_set_bit(LP_I2C_ANA_MST_DEVICE_EN_REG, REGI2C_BIAS_DEVICE_EN);
        }
        v if v == REGI2C_DIG_REG => {
            reg_set_bit(LP_I2C_ANA_MST_DEVICE_EN_REG, REGI2C_DIG_REG_DEVICE_EN);
        }
        v if v == REGI2C_ULP_CAL => {
            reg_set_bit(LP_I2C_ANA_MST_DEVICE_EN_REG, REGI2C_ULP_CAL_DEVICE_EN);
        }
        v if v == REGI2C_SAR_I2C => {
            reg_set_bit(LP_I2C_ANA_MST_DEVICE_EN_REG, REGI2C_SAR_I2C_DEVICE_EN);
        }
        _ => (),
    }
}

fn regi2c_disable_block(block: u8) {
    match block {
        v if v == REGI2C_BBPLL => {
            reg_clr_bit(LP_I2C_ANA_MST_DEVICE_EN_REG, REGI2C_BBPLL_DEVICE_EN);
        }
        v if v == REGI2C_BIAS => {
            reg_clr_bit(LP_I2C_ANA_MST_DEVICE_EN_REG, REGI2C_BIAS_DEVICE_EN);
        }
        v if v == REGI2C_DIG_REG => {
            reg_clr_bit(LP_I2C_ANA_MST_DEVICE_EN_REG, REGI2C_DIG_REG_DEVICE_EN);
        }
        v if v == REGI2C_ULP_CAL => {
            reg_clr_bit(LP_I2C_ANA_MST_DEVICE_EN_REG, REGI2C_ULP_CAL_DEVICE_EN);
        }
        v if v == REGI2C_SAR_I2C => {
            reg_clr_bit(LP_I2C_ANA_MST_DEVICE_EN_REG, REGI2C_SAR_I2C_DEVICE_EN);
        }
        _ => (),
    }
}

pub(crate) fn regi2c_write(block: u8, _host_id: u8, reg_add: u8, data: u8) {
    regi2c_enable_block(block);
    let lp_i2c_ana = unsafe { &*crate::peripherals::LP_I2C_ANA_MST::ptr() };

    let temp: u32 = ((block as u32 & REGI2C_RTC_SLAVE_ID_V as u32) << REGI2C_RTC_SLAVE_ID_S as u32)
                    | ((reg_add as u32 & REGI2C_RTC_ADDR_V as u32) << REGI2C_RTC_ADDR_S as u32)
                    | ((0x1 & REGI2C_RTC_WR_CNTL_V as u32) << REGI2C_RTC_WR_CNTL_S as u32) // 0: READ I2C register; 1: Write I2C register;
                    | (((data as u32) & REGI2C_RTC_DATA_V as u32) << REGI2C_RTC_DATA_S as u32);

    unsafe {
        lp_i2c_ana.i2c0_ctrl().modify(|_, w| w.bits(temp));
    }

    while lp_i2c_ana
        .i2c0_ctrl()
        .read()
        .lp_i2c_ana_mast_i2c0_busy()
        .bit()
    {}

    regi2c_disable_block(block);
}

pub(crate) fn regi2c_write_mask(block: u8, _host_id: u8, reg_add: u8, msb: u8, lsb: u8, data: u8) {
    assert!(msb - lsb < 8);
    let lp_i2c_ana = unsafe { &*crate::peripherals::LP_I2C_ANA_MST::ptr() };

    unsafe {
        regi2c_enable_block(block);

        // Read the i2c bus register
        let mut temp: u32 = ((block as u32 & REGI2C_RTC_SLAVE_ID_V as u32)
            << REGI2C_RTC_SLAVE_ID_S as u32)
            | (reg_add as u32 & REGI2C_RTC_ADDR_V as u32) << REGI2C_RTC_ADDR_S as u32;

        lp_i2c_ana
            .i2c0_ctrl()
            .modify(|_, w| w.lp_i2c_ana_mast_i2c0_ctrl().bits(temp));

        while lp_i2c_ana
            .i2c0_ctrl()
            .read()
            .lp_i2c_ana_mast_i2c0_busy()
            .bit()
        {}

        temp = lp_i2c_ana
            .i2c0_data()
            .read()
            .lp_i2c_ana_mast_i2c0_rdata()
            .bits() as u32;

        // Write the i2c bus register
        temp &= (!(0xFFFFFFFF << lsb)) | (0xFFFFFFFF << (msb + 1));
        temp |= (data as u32 & (!(0xFFFFFFFF << (msb as u32 - lsb as u32 + 1)))) << (lsb as u32);
        temp = ((block as u32 & REGI2C_RTC_SLAVE_ID_V as u32) << REGI2C_RTC_SLAVE_ID_S as u32)
            | ((reg_add as u32 & REGI2C_RTC_ADDR_V as u32) << REGI2C_RTC_ADDR_S as u32)
            | ((0x1 & REGI2C_RTC_WR_CNTL_V as u32) << REGI2C_RTC_WR_CNTL_S as u32)
            | ((temp & REGI2C_RTC_DATA_V as u32) << REGI2C_RTC_DATA_S as u32);

        lp_i2c_ana
            .i2c0_ctrl()
            .modify(|_, w| w.lp_i2c_ana_mast_i2c0_ctrl().bits(temp));

        while lp_i2c_ana
            .i2c0_ctrl()
            .read()
            .lp_i2c_ana_mast_i2c0_busy()
            .bit()
        {}

        regi2c_disable_block(block);
    }
}

// clk_ll_ahb_set_ls_divider
fn esp32c6_ahb_set_ls_divider(div: u8) {
    unsafe {
        pcr()
            .ahb_freq_conf()
            .modify(|_, w| w.ahb_ls_div_num().bits(div - 1));
    }
}

// clk_ll_cpu_set_ls_divider
fn esp32c6_cpu_set_ls_divider(div: u8) {
    unsafe {
        pcr()
            .cpu_freq_conf()
            .modify(|_, w| w.cpu_ls_div_num().bits(div - 1));
    }
}

// clk_ll_cpu_get_ls_divider
pub(crate) fn esp32c6_cpu_get_ls_divider() -> u8 {
    unsafe {
        let cpu_ls_div = pcr().cpu_freq_conf().read().cpu_ls_div_num().bits();
        let hp_root_ls_div = pcr().sysclk_conf().read().ls_div_num().bits();
        (hp_root_ls_div + 1) * (cpu_ls_div + 1)
    }
}

// clk_ll_cpu_get_hs_divider
pub(crate) fn esp32c6_cpu_get_hs_divider() -> u8 {
    unsafe {
        let force_120m = pcr().cpu_freq_conf().read().cpu_hs_120m_force().bit();
        let cpu_hs_div = pcr().cpu_freq_conf().read().cpu_hs_div_num().bits();
        if cpu_hs_div == 0 && force_120m {
            return 4;
        }
        let hp_root_hs_div = pcr().sysclk_conf().read().hs_div_num().bits();
        (hp_root_hs_div + 1) * (cpu_hs_div + 1)
    }
}

// clk_ll_bbpll_get_freq_mhz
pub(crate) fn esp32c6_bbpll_get_freq_mhz() -> u32 {
    // The target has a fixed 480MHz SPLL
    const CLK_LL_PLL_480M_FREQ_MHZ: u32 = 480;

    CLK_LL_PLL_480M_FREQ_MHZ
}
