use crate::{
    clock::{ApbClock, Clock, CpuClock, PllClock, XtalClock},
    peripherals::{APB_CTRL, I2C_ANA_MST, LPWR, MODEM_CLKRST, SYSTEM},
    rom::{regi2c_write, regi2c_write_mask},
};

const I2C_BBPLL: u32 = 0x66;
const I2C_BBPLL_HOSTID: u32 = 0;

const I2C_BBPLL_MODE_HF: u32 = 4;

const I2C_BBPLL_OC_REF_DIV: u32 = 2;
const I2C_BBPLL_OC_DCHGP_LSB: u32 = 4;
const I2C_BBPLL_OC_DIV_7_0: u32 = 3;

const I2C_BBPLL_OC_DR1: u32 = 5;
const I2C_BBPLL_OC_DR1_MSB: u32 = 2;
const I2C_BBPLL_OC_DR1_LSB: u32 = 0;

const I2C_BBPLL_OC_DR3: u32 = 5;
const I2C_BBPLL_OC_DR3_MSB: u32 = 6;
const I2C_BBPLL_OC_DR3_LSB: u32 = 4;

const I2C_BBPLL_OC_DCUR: u32 = 6;

const I2C_BBPLL_OC_VCO_DBIAS: u32 = 9;
const I2C_BBPLL_OC_VCO_DBIAS_MSB: u32 = 1;
const I2C_BBPLL_OC_VCO_DBIAS_LSB: u32 = 0;

const I2C_BBPLL_OC_DHREF_SEL_LSB: u32 = 4;

const I2C_BBPLL_OC_DLREF_SEL_LSB: u32 = 6;

pub(crate) fn esp32c2_rtc_bbpll_configure(xtal_freq: XtalClock, _pll_freq: PllClock) {
    let div_ref: u32;
    let div7_0: u32;
    let dr1: u32;
    let dr3: u32;
    let dchgp: u32;
    let dcur: u32;
    let dbias: u32;

    // Set this register to let the digital part know 480M PLL is used
    SYSTEM::regs()
        .cpu_per_conf()
        .modify(|_, w| w.pll_freq_sel().set_bit());

    I2C_ANA_MST::regs().ana_conf0().modify(|_, w| {
        w.bbpll_stop_force_high().clear_bit();
        w.bbpll_stop_force_low().set_bit()
    });

    // Configure 480M PLL
    match xtal_freq {
        XtalClock::_26M => {
            div_ref = 12;
            div7_0 = 236;
            dr1 = 4;
            dr3 = 4;
            dchgp = 0;
            dcur = 0;
            dbias = 2;
        }
        XtalClock::_40M | XtalClock::Other(_) => {
            div_ref = 0;
            div7_0 = 8;
            dr1 = 0;
            dr3 = 0;
            dchgp = 5;
            dcur = 3;
            dbias = 2;
        }
    }

    regi2c_write!(I2C_BBPLL, I2C_BBPLL_MODE_HF, 0x6b);

    let i2c_bbpll_lref = (dchgp << I2C_BBPLL_OC_DCHGP_LSB) | div_ref;
    let i2c_bbpll_div_7_0 = div7_0;
    let i2c_bbpll_dcur =
        (1 << I2C_BBPLL_OC_DLREF_SEL_LSB) | (3 << I2C_BBPLL_OC_DHREF_SEL_LSB) | dcur;

    regi2c_write!(I2C_BBPLL, I2C_BBPLL_OC_REF_DIV, i2c_bbpll_lref);

    regi2c_write!(I2C_BBPLL, I2C_BBPLL_OC_DIV_7_0, i2c_bbpll_div_7_0);

    regi2c_write_mask!(I2C_BBPLL, I2C_BBPLL_OC_DR1, dr1);

    regi2c_write_mask!(I2C_BBPLL, I2C_BBPLL_OC_DR3, dr3);

    regi2c_write!(I2C_BBPLL, I2C_BBPLL_OC_DCUR, i2c_bbpll_dcur);

    regi2c_write_mask!(I2C_BBPLL, I2C_BBPLL_OC_VCO_DBIAS, dbias);

    // WAIT CALIBRATION DONE
    while I2C_ANA_MST::regs()
        .ana_conf0()
        .read()
        .bbpll_cal_done()
        .bit_is_clear()
    {}

    // workaround bbpll calibration might stop early
    crate::rom::ets_delay_us(10);

    // Stop BBPLL self-calibration
    I2C_ANA_MST::regs().ana_conf0().modify(|_, w| {
        w.bbpll_stop_force_high().set_bit();
        w.bbpll_stop_force_low().clear_bit()
    });
}

pub(crate) fn esp32c2_rtc_bbpll_enable() {
    LPWR::regs().options0().modify(|_, w| {
        w.bb_i2c_force_pd().clear_bit();
        w.bbpll_force_pd().clear_bit();
        w.bbpll_i2c_force_pd().clear_bit()
    });
}

pub(crate) fn esp32c2_rtc_update_to_xtal(freq: XtalClock, div: u32) {
    crate::rom::ets_update_cpu_frequency_rom(freq.mhz());

    // Set divider from XTAL to APB clock. Need to set divider to 1 (reg. value 0)
    // first.
    SYSTEM::regs().sysclk_conf().modify(|_, w| unsafe {
        w.pre_div_cnt().bits(0);
        w.pre_div_cnt().bits((div - 1) as u16)
    });

    // No need to adjust the REF_TICK

    // Switch clock source
    SYSTEM::regs()
        .sysclk_conf()
        .modify(|_, w| unsafe { w.soc_clk_sel().bits(0) });
}

pub(crate) fn esp32c2_rtc_freq_to_pll_mhz(cpu_clock_speed: CpuClock) {
    SYSTEM::regs().sysclk_conf().modify(|_, w| unsafe {
        w.pre_div_cnt().bits(0);
        w.soc_clk_sel().bits(1)
    });
    SYSTEM::regs().cpu_per_conf().modify(|_, w| unsafe {
        w.cpuperiod_sel().bits(match cpu_clock_speed {
            CpuClock::_80MHz => 0,
            CpuClock::_120MHz => 1,
        })
    });

    crate::rom::ets_update_cpu_frequency_rom(cpu_clock_speed.mhz());
}

pub(crate) fn esp32c2_rtc_apb_freq_update(apb_freq: ApbClock) {
    let value = ((apb_freq.hz() >> 12) & u16::MAX as u32)
        | (((apb_freq.hz() >> 12) & u16::MAX as u32) << 16);

    LPWR::regs()
        .store5()
        .modify(|_, w| unsafe { w.scratch5().bits(value) });
}

// Mask for clock bits used by both WIFI and Bluetooth, 0, 1, 2, 3, 7, 8, 9, 10,
// 19, 20, 21, 22, 23
const SYSTEM_WIFI_CLK_WIFI_BT_COMMON_M: u32 = 0x78078F;
// SYSTEM_WIFI_CLK_EN : R/W ;bitpos:[31:0] ;default: 32'hfffce030
const SYSTEM_WIFI_CLK_EN: u32 = 0x00FB9FCF;

pub(super) fn enable_phy(enable: bool) {
    // `periph_ll_wifi_bt_module_enable_clk_clear_rst`
    // `periph_ll_wifi_bt_module_disable_clk_set_rst`
    APB_CTRL::regs().wifi_clk_en().modify(|r, w| unsafe {
        if enable {
            w.bits(r.bits() | SYSTEM_WIFI_CLK_WIFI_BT_COMMON_M)
        } else {
            w.bits(r.bits() & !SYSTEM_WIFI_CLK_WIFI_BT_COMMON_M)
        }
    });
}

pub(super) fn enable_bt(_: bool) {
    // `periph_ll_wifi_module_enable_clk_clear_rst`, no-op
    // `periph_ll_wifi_module_disable_clk_clear_rst`, no-op
}

pub(super) fn enable_wifi(_: bool) {
    // `periph_ll_wifi_module_enable_clk_clear_rst`, no-op
    // `periph_ll_wifi_module_disable_clk_clear_rst`, no-op
}

pub(super) fn reset_mac() {
    APB_CTRL::regs()
        .wifi_rst_en()
        .modify(|_, w| w.mac_rst().set_bit());
    APB_CTRL::regs()
        .wifi_rst_en()
        .modify(|_, w| w.mac_rst().clear_bit());
}

pub(super) fn init_clocks() {
    // from `esp_perip_clk_init`
    const SYSTEM_WIFI_CLK_UNUSED_BIT5: u32 = 1 << 5;
    const SYSTEM_WIFI_CLK_UNUSED_BIT12: u32 = 1 << 12;
    const WIFI_BT_SDIO_CLK: u32 = SYSTEM_WIFI_CLK_UNUSED_BIT5 | SYSTEM_WIFI_CLK_UNUSED_BIT12;

    APB_CTRL::regs()
        .wifi_clk_en()
        .modify(|r, w| unsafe { w.bits(r.bits() & !WIFI_BT_SDIO_CLK | SYSTEM_WIFI_CLK_EN) });
}

pub(super) fn ble_rtc_clk_init() {
    MODEM_CLKRST::regs().modem_lp_timer_conf().modify(|_, w| {
        w.lp_timer_sel_xtal32k().clear_bit();
        w.lp_timer_sel_xtal().set_bit();
        w.lp_timer_sel_8m().clear_bit();
        w.lp_timer_sel_rtc_slow().clear_bit()
    });

    // assume 40MHz xtal
    MODEM_CLKRST::regs()
        .modem_lp_timer_conf()
        .modify(|_, w| unsafe { w.lp_timer_clk_div_num().bits(249) });

    MODEM_CLKRST::regs().etm_clk_conf().modify(|_, w| {
        w.etm_clk_active().set_bit();
        w.etm_clk_sel().clear_bit()
    });
}

pub(super) fn reset_rpa() {
    APB_CTRL::regs()
        .wifi_rst_en()
        .modify(|_, w| w.ble_rpa_rst().set_bit());
    APB_CTRL::regs()
        .wifi_rst_en()
        .modify(|_, w| w.ble_rpa_rst().clear_bit());
}
