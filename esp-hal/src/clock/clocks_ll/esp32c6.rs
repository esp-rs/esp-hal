use crate::{
    clock::{ApbClock, Clock, CpuClock, PllClock, XtalClock},
    peripherals::{I2C_ANA_MST, LP_AON, MODEM_LPCON, MODEM_SYSCON, PCR, PMU},
    rtc_cntl::rtc::CpuClockSource,
    soc::regi2c,
};

// rtc_clk_bbpll_configure
pub(crate) fn esp32c6_rtc_bbpll_configure(xtal_freq: XtalClock, pll_freq: PllClock) {
    esp32c6_rtc_bbpll_configure_raw(xtal_freq.mhz(), pll_freq.mhz())
}

pub(crate) fn esp32c6_rtc_bbpll_configure_raw(_xtal_freq: u32, pll_freq: u32) {
    // clk_ll_bbpll_set_freq_mhz
    // The target SPLL is fixed to 480MHz
    // Do nothing
    debug_assert!(pll_freq == 480);

    critical_section::with(|_| {
        // enable i2c mst clk by force on (temporarily)
        let was_i2c_mst_en = MODEM_LPCON::regs().clk_conf().read().clk_i2c_mst_en().bit();
        MODEM_LPCON::regs()
            .clk_conf()
            .modify(|_, w| w.clk_i2c_mst_en().set_bit());

        MODEM_LPCON::regs()
            .i2c_mst_clk_conf()
            .modify(|_, w| w.clk_i2c_mst_sel_160m().set_bit());

        // BBPLL CALIBRATION START
        I2C_ANA_MST::regs().ana_conf0().modify(|_, w| {
            w.bbpll_stop_force_high().clear_bit();
            w.bbpll_stop_force_low().set_bit()
        });

        const DIV_REF: u8 = 0;
        const DCHGP: u8 = 5;
        const DCUR: u8 = 3;

        const I2C_BBPLL_OC_DCHGP_LSB: u32 = 4;
        const I2C_BBPLL_OC_DHREF_SEL_LSB: u32 = 4;
        const I2C_BBPLL_OC_DLREF_SEL_LSB: u32 = 6;

        const I2C_BBPLL_LREF: u8 = (DCHGP << I2C_BBPLL_OC_DCHGP_LSB) | DIV_REF;
        const I2C_BBPLL_DCUR: u8 =
            (1 << I2C_BBPLL_OC_DLREF_SEL_LSB) | (3 << I2C_BBPLL_OC_DHREF_SEL_LSB) | DCUR;

        regi2c::I2C_BBPLL_OC_REF.write_reg(I2C_BBPLL_LREF);
        regi2c::I2C_BBPLL_OC_DIV_REG.write_reg(8);
        regi2c::I2C_BBPLL_OC_DR1.write_field(0);
        regi2c::I2C_BBPLL_OC_DR3.write_field(0);
        regi2c::I2C_BBPLL_REG6.write_reg(I2C_BBPLL_DCUR);
        regi2c::I2C_BBPLL_OC_VCO_DBIAS.write_field(2);

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

        MODEM_LPCON::regs()
            .clk_conf()
            .modify(|_, w| w.clk_i2c_mst_en().bit(was_i2c_mst_en));
    });
}

pub(crate) fn esp32c6_rtc_bbpll_enable() {
    PMU::regs().imm_hp_ck_power().modify(|_, w| {
        w.tie_high_xpd_bb_i2c().set_bit();
        w.tie_high_xpd_bbpll().set_bit();
        w.tie_high_xpd_bbpll_i2c().set_bit()
    });

    PMU::regs()
        .imm_hp_ck_power()
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

    PCR::regs().cpu_freq_conf().modify(|_, w| unsafe {
        w.cpu_hs_div_num()
            .bits(((480 / cpu_clock_speed_mhz / 3) - 1) as u8);
        w.cpu_hs_120m_force().clear_bit()
    });

    PCR::regs()
        .cpu_freq_conf()
        .modify(|_, w| w.cpu_hs_120m_force().clear_bit());

    CpuClockSource::Pll.select();

    crate::rom::ets_update_cpu_frequency_rom(cpu_clock_speed_mhz);
}

pub(crate) fn esp32c6_rtc_apb_freq_update(apb_freq: ApbClock) {
    let value = ((apb_freq.hz() >> 12) & u16::MAX as u32)
        | (((apb_freq.hz() >> 12) & u16::MAX as u32) << 16);

    LP_AON::regs()
        .store5()
        .modify(|_, w| unsafe { w.lp_aon_store5().bits(value) });
}

fn clk_ll_mspi_fast_set_hs_divider(divider: u32) {
    // SOC_ROOT_CLK ------> MSPI_FAST_CLK
    // HS divider option: 4, 5, 6 (PCR_MSPI_FAST_HS_DIV_NUM=3, 4, 5)

    let div_num = match divider {
        4..=6 => divider as u8 - 1,
        _ => panic!("Unsupported HS MSPI_FAST divider"),
    };

    PCR::regs()
        .mspi_clk_conf()
        .modify(|_, w| unsafe { w.mspi_fast_hs_div_num().bits(div_num) });
}

// clk_ll_ahb_set_ls_divider
fn esp32c6_ahb_set_ls_divider(div: u8) {
    PCR::regs()
        .ahb_freq_conf()
        .modify(|_, w| unsafe { w.ahb_ls_div_num().bits(div - 1) });
}

// clk_ll_cpu_set_ls_divider
fn esp32c6_cpu_set_ls_divider(div: u8) {
    PCR::regs()
        .cpu_freq_conf()
        .modify(|_, w| unsafe { w.cpu_ls_div_num().bits(div - 1) });
}

// clk_ll_cpu_get_ls_divider
pub(crate) fn esp32c6_cpu_get_ls_divider() -> u8 {
    let cpu_ls_div = PCR::regs().cpu_freq_conf().read().cpu_ls_div_num().bits();
    let hp_root_ls_div = PCR::regs().sysclk_conf().read().ls_div_num().bits();
    (hp_root_ls_div + 1) * (cpu_ls_div + 1)
}

// clk_ll_cpu_get_hs_divider
pub(crate) fn esp32c6_cpu_get_hs_divider() -> u8 {
    let force_120m = PCR::regs().cpu_freq_conf().read().cpu_hs_120m_force().bit();
    let cpu_hs_div = PCR::regs().cpu_freq_conf().read().cpu_hs_div_num().bits();
    if cpu_hs_div == 0 && force_120m {
        return 4;
    }
    let hp_root_hs_div = PCR::regs().sysclk_conf().read().hs_div_num().bits();
    (hp_root_hs_div + 1) * (cpu_hs_div + 1)
}

// clk_ll_bbpll_get_freq_mhz
pub(crate) fn esp32c6_bbpll_get_freq_mhz() -> u32 {
    // The target has a fixed 480MHz SPLL
    const CLK_LL_PLL_480M_FREQ_MHZ: u32 = 480;

    CLK_LL_PLL_480M_FREQ_MHZ
}

pub(super) fn enable_phy(en: bool) {
    MODEM_LPCON::regs()
        .clk_conf()
        .modify(|_, w| w.clk_i2c_mst_en().bit(en));
    MODEM_LPCON::regs()
        .i2c_mst_clk_conf()
        .modify(|_, w| w.clk_i2c_mst_sel_160m().bit(en));
}

pub(super) fn enable_wifi(en: bool) {
    MODEM_SYSCON::regs().clk_conf1().modify(|_, w| {
        w.clk_wifi_apb_en().bit(en);
        w.clk_wifimac_en().bit(en);
        w.clk_fe_apb_en().bit(en);
        w.clk_fe_cal_160m_en().bit(en);
        w.clk_fe_160m_en().bit(en);
        w.clk_fe_80m_en().bit(en);
        w.clk_wifibb_160x1_en().bit(en);
        w.clk_wifibb_80x1_en().bit(en);
        w.clk_wifibb_40x1_en().bit(en);
        w.clk_wifibb_80x_en().bit(en);
        w.clk_wifibb_40x_en().bit(en);
        w.clk_wifibb_80m_en().bit(en);
        w.clk_wifibb_44m_en().bit(en);
        w.clk_wifibb_40m_en().bit(en);
        w.clk_wifibb_22m_en().bit(en)
    });

    MODEM_LPCON::regs().clk_conf().modify(|_, w| {
        w.clk_wifipwr_en().bit(en);
        w.clk_coex_en().bit(en)
    });
}

pub(super) fn enable_ieee802154(en: bool) {
    MODEM_SYSCON::regs().clk_conf().modify(|_, w| {
        w.clk_zb_apb_en().bit(en);
        w.clk_zb_mac_en().bit(en)
    });

    MODEM_SYSCON::regs().clk_conf1().modify(|_, w| {
        w.clk_fe_apb_en().bit(en);
        w.clk_fe_cal_160m_en().bit(en);
        w.clk_fe_160m_en().bit(en);
        w.clk_fe_80m_en().bit(en);
        w.clk_bt_apb_en().bit(en);
        w.clk_bt_en().bit(en);
        w.clk_wifibb_160x1_en().bit(en);
        w.clk_wifibb_80x1_en().bit(en);
        w.clk_wifibb_40x1_en().bit(en);
        w.clk_wifibb_80x_en().bit(en);
        w.clk_wifibb_40x_en().bit(en);
        w.clk_wifibb_80m_en().bit(en);
        w.clk_wifibb_44m_en().bit(en);
        w.clk_wifibb_40m_en().bit(en);
        w.clk_wifibb_22m_en().bit(en)
    });

    MODEM_LPCON::regs()
        .clk_conf()
        .modify(|_, w| w.clk_coex_en().set_bit());
}

pub(super) fn enable_bt(en: bool) {
    MODEM_SYSCON::regs().clk_conf().modify(|_, w| {
        w.clk_etm_en().bit(en);
        w.clk_modem_sec_en().bit(en);
        w.clk_modem_sec_ecb_en().bit(en);
        w.clk_modem_sec_ccm_en().bit(en);
        w.clk_modem_sec_bah_en().bit(en);
        w.clk_modem_sec_apb_en().bit(en);
        w.clk_ble_timer_en().bit(en)
    });

    MODEM_SYSCON::regs().clk_conf1().modify(|_, w| {
        w.clk_fe_apb_en().bit(en);
        w.clk_fe_cal_160m_en().bit(en);
        w.clk_fe_160m_en().bit(en);
        w.clk_fe_80m_en().bit(en);
        w.clk_bt_apb_en().bit(en);
        w.clk_bt_en().bit(en)
    });

    MODEM_LPCON::regs()
        .clk_conf()
        .modify(|_, w| w.clk_coex_en().bit(en));
}

pub(super) fn reset_mac() {
    // empty
}

pub(super) fn init_clocks() {
    unsafe {
        PMU::regs()
            .hp_sleep_icg_modem()
            .modify(|_, w| w.hp_sleep_dig_icg_modem_code().bits(0));
        PMU::regs()
            .hp_modem_icg_modem()
            .modify(|_, w| w.hp_modem_dig_icg_modem_code().bits(1));
        PMU::regs()
            .hp_active_icg_modem()
            .modify(|_, w| w.hp_active_dig_icg_modem_code().bits(2));
        PMU::regs()
            .imm_modem_icg()
            .write(|w| w.update_dig_icg_modem_en().set_bit());
        PMU::regs()
            .imm_sleep_sysclk()
            .write(|w| w.update_dig_icg_switch().set_bit());

        MODEM_SYSCON::regs().clk_conf_power_st().modify(|_, w| {
            w.clk_modem_apb_st_map().bits(6);
            w.clk_modem_peri_st_map().bits(4);
            w.clk_wifi_st_map().bits(6);
            w.clk_bt_st_map().bits(6);
            w.clk_fe_st_map().bits(6);
            w.clk_zb_st_map().bits(6)
        });

        MODEM_LPCON::regs().clk_conf_power_st().modify(|_, w| {
            w.clk_lp_apb_st_map().bits(6);
            w.clk_i2c_mst_st_map().bits(6);
            w.clk_coex_st_map().bits(6);
            w.clk_wifipwr_st_map().bits(6)
        });

        MODEM_LPCON::regs().wifi_lp_clk_conf().modify(|_, w| {
            w.clk_wifipwr_lp_sel_osc_slow().set_bit();
            w.clk_wifipwr_lp_sel_osc_fast().set_bit();
            w.clk_wifipwr_lp_sel_xtal32k().set_bit();
            w.clk_wifipwr_lp_sel_xtal().set_bit()
        });

        MODEM_LPCON::regs()
            .wifi_lp_clk_conf()
            .modify(|_, w| w.clk_wifipwr_lp_div_num().bits(0));

        MODEM_LPCON::regs()
            .clk_conf()
            .modify(|_, w| w.clk_wifipwr_en().set_bit());
    }
}

pub(super) fn ble_rtc_clk_init() {
    // nothing for this target (yet)
}

pub(super) fn reset_rpa() {
    // nothing for this target (yet)
}
