use crate::peripherals::{MODEM_LPCON, MODEM_SYSCON, PMU, PCR};

// done
pub(super) fn enable_phy(en: bool) {
    PCR::regs()
        .reset_event_bypass()
        .modify(|_, w| w.reset_event_bypass(en));
    MODEM_LPCON::regs()
        .clk_conf()
        .modify(|_, w| w.clk_i2c_mst_en().bit(en));
    MODEM_LPCON::regs()
        .i2c_mst_clk_conf()
        .modify(|_, w| w.clk_i2c_mst_sel_160m().bit(en));
}

#[cfg_attr(not(feature = "unstable"), expect(unused))]
pub(super) fn enable_wifi(en: bool) {
    MODEM_SYSCON::regs().clk_conf1().modify(|_, w| {
        // modem_clock_modem_adc_common_fe_configure
        w.clk_fe_apb_en().bit(en);
        w.clk_fe_80m_en().bit(en);
        // modem_clock_modem_private_fe_configure
        w.clk_fe_160m_en().bit(en);
        w.clk_fe_dac_en().bit(en);
        w.clk_fe_pwdet_adc_en().bit(en);
        w.clk_fe_adc_en().bit(en);
        // modem_clock_coex_configure
        w.clk_coex_en().bit(en);

        // SOC_PHY_CALIBRATION_CLOCK_IS_INDEPENDENT = 1
        // modem_syscon_ll_enable_wifi_mac_clock
        w.clk_wifimac_en().bit(en);

        // modem_syscon_ll_clk_wifibb_configure -> modem_syscon_ll_clk_conf1_configure
        // clk_conf1 | 0x1fb
        w.clk_wifibb_22m_en().bit(en);
        w.clk_wifibb_40m_en().bit(en);
        w.clk_wifibb_80m_en().bit(en);
        w.clk_wifibb_40x_en().bit(en);
        w.clk_wifibb_80x_en().bit(en);
        w.clk_wifibb_40x1_en().bit(en);
        w.clk_wifibb_80x1_en().bit(en);
        w.clk_wifibb_160x1_en().bit(en);
    });

    // modem_lpcon_ll_enable_wifipwr_clock
    MODEM_LPCON::regs().clk_conf().modify(|_, w| {
        w.clk_wifipwr_en().bit(en);
        w.clk_coex_en().bit(en)
    });
}

#[cfg_attr(not(feature = "unstable"), expect(unused))]
pub(super) fn enable_ieee802154(en: bool) {
    todo!()
    
}

#[cfg_attr(not(feature = "unstable"), expect(unused))]
pub(super) fn enable_bt(en: bool) {
    todo!()
}

pub(super) fn reset_wifi_mac() {
    // empty
}

pub(super) fn init_clocks() {
    todo!()
}

pub(super) fn ble_rtc_clk_init() {
    // nothing for this target (yet)
}

pub(super) fn reset_rpa() {
    // nothing for this target (yet)
}
