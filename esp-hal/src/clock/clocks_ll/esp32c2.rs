use crate::{
    clock::{ApbClock, Clock, CpuClock, PllClock, XtalClock},
    peripherals::{APB_CTRL, MODEM_CLKRST},
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

const I2C_MST_ANA_CONF0_REG: u32 = 0x6004_E840;
const I2C_MST_BBPLL_STOP_FORCE_HIGH: u32 = 1 << 2;
const I2C_MST_BBPLL_STOP_FORCE_LOW: u32 = 1 << 3;

pub(crate) fn esp32c2_rtc_bbpll_configure(xtal_freq: XtalClock, _pll_freq: PllClock) {
    let system = crate::peripherals::SYSTEM::regs();

    let div_ref: u32;
    let div7_0: u32;
    let dr1: u32;
    let dr3: u32;
    let dchgp: u32;
    let dcur: u32;
    let dbias: u32;

    unsafe {
        let clear_reg_mask = |reg, mask: u32| {
            (reg as *mut u32).write_volatile((reg as *mut u32).read_volatile() & !mask)
        };
        let set_reg_mask = |reg, mask: u32| {
            (reg as *mut u32).write_volatile((reg as *mut u32).read_volatile() | mask)
        };

        clear_reg_mask(I2C_MST_ANA_CONF0_REG, I2C_MST_BBPLL_STOP_FORCE_HIGH);
        set_reg_mask(I2C_MST_ANA_CONF0_REG, I2C_MST_BBPLL_STOP_FORCE_LOW);
    }

    // Set this register to let the digital part know 480M PLL is used
    system
        .cpu_per_conf()
        .modify(|_, w| w.pll_freq_sel().set_bit());

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
}

pub(crate) fn esp32c2_rtc_bbpll_enable() {
    let rtc_cntl = crate::peripherals::LPWR::regs();

    rtc_cntl.options0().modify(|_, w| {
        w.bb_i2c_force_pd()
            .clear_bit()
            .bbpll_force_pd()
            .clear_bit()
            .bbpll_i2c_force_pd()
            .clear_bit()
    });
}

pub(crate) fn esp32c2_rtc_update_to_xtal(freq: XtalClock, _div: u32) {
    crate::rom::ets_update_cpu_frequency_rom(freq.mhz());

    let system_control = crate::peripherals::SYSTEM::regs();
    unsafe {
        // Set divider from XTAL to APB clock. Need to set divider to 1 (reg. value 0)
        // first.
        system_control.sysclk_conf().modify(|_, w| {
            w.pre_div_cnt()
                .bits(0)
                .pre_div_cnt()
                .bits((_div - 1) as u16)
        });

        // No need to adjust the REF_TICK

        // Switch clock source
        system_control
            .sysclk_conf()
            .modify(|_, w| w.soc_clk_sel().bits(0));
    }
}

pub(crate) fn esp32c2_rtc_freq_to_pll_mhz(cpu_clock_speed: CpuClock) {
    let system_control = crate::peripherals::SYSTEM::regs();

    unsafe {
        system_control
            .sysclk_conf()
            .modify(|_, w| w.pre_div_cnt().bits(0).soc_clk_sel().bits(1));
        system_control.cpu_per_conf().modify(|_, w| {
            w.cpuperiod_sel().bits(match cpu_clock_speed {
                CpuClock::_80MHz => 0,
                CpuClock::_120MHz => 1,
            })
        });
    }

    crate::rom::ets_update_cpu_frequency_rom(cpu_clock_speed.mhz());
}

pub(crate) fn esp32c2_rtc_apb_freq_update(apb_freq: ApbClock) {
    let rtc_cntl = crate::peripherals::LPWR::regs();

    let value = ((apb_freq.hz() >> 12) & u16::MAX as u32)
        | (((apb_freq.hz() >> 12) & u16::MAX as u32) << 16);

    rtc_cntl
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
    const SYSTEM_MAC_RST: u32 = 1 << 2;
    APB_CTRL::regs()
        .wifi_rst_en()
        .modify(|r, w| unsafe { w.wifi_rst().bits(r.wifi_rst().bits() | SYSTEM_MAC_RST) });
    APB_CTRL::regs()
        .wifi_rst_en()
        .modify(|r, w| unsafe { w.wifi_rst().bits(r.wifi_rst().bits() & !SYSTEM_MAC_RST) });
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
    let modem_clkrst = MODEM_CLKRST::regs();
    modem_clkrst
        .modem_lp_timer_conf()
        .modify(|_, w| w.lp_timer_sel_xtal32k().clear_bit());
    modem_clkrst
        .modem_lp_timer_conf()
        .modify(|_, w| w.lp_timer_sel_xtal().set_bit());
    modem_clkrst
        .modem_lp_timer_conf()
        .modify(|_, w| w.lp_timer_sel_8m().clear_bit());
    modem_clkrst
        .modem_lp_timer_conf()
        .modify(|_, w| w.lp_timer_sel_rtc_slow().clear_bit());

    // assume 40MHz xtal
    modem_clkrst
        .modem_lp_timer_conf()
        .modify(|_, w| unsafe { w.lp_timer_clk_div_num().bits(249) });

    modem_clkrst
        .etm_clk_conf()
        .modify(|_, w| w.etm_clk_active().set_bit());
    modem_clkrst
        .etm_clk_conf()
        .modify(|_, w| w.etm_clk_sel().clear_bit());
}

pub(super) fn reset_rpa() {
    const BLE_RPA_REST_BIT: u32 = 1 << 27;
    APB_CTRL::regs()
        .wifi_rst_en()
        .modify(|r, w| unsafe { w.bits(r.bits() | BLE_RPA_REST_BIT) });
    APB_CTRL::regs()
        .wifi_rst_en()
        .modify(|r, w| unsafe { w.bits(r.bits() & !BLE_RPA_REST_BIT) });
}
