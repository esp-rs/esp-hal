use crate::peripherals::{MODEM_LPCON, MODEM_SYSCON};

// done
#[allow(unused)]
pub(super) fn enable_phy(_en: bool) {
    MODEM_SYSCON::regs()
        .clk_conf_power_st()
        .modify(|_, w| unsafe {
            w.clk_modem_apb_st_map().bits(6);
            w.clk_modem_peri_st_map().bits(4);
            w.clk_wifi_st_map().bits(6);
            w.clk_bt_st_map().bits(4);
            w.clk_fe_st_map().bits(6);
            w.clk_zb_st_map().bits(4)
        });

    MODEM_LPCON::regs()
        .clk_conf_power_st()
        .modify(|_, w| unsafe {
            w.clk_lp_apb_st_map().bits(6);
            w.clk_i2c_mst_st_map().bits(6);
            w.clk_coex_st_map().bits(6);
            w.clk_wifipwr_st_map().bits(6)
        });

    MODEM_LPCON::regs()
        .clk_conf()
        .modify(|_, w| w.clk_coex_en().set_bit());

    MODEM_SYSCON::regs().clk_conf1().modify(|_, w| {
        w.clk_wifi_apb_en().set_bit();
        w.clk_wifibb_22m_en().set_bit();
        w.clk_fe_40m_en().set_bit();
        w.clk_fe_80m_en().set_bit();
        w.clk_wifibb_44m_en().set_bit();
        w.clk_wifimac_en().set_bit()
    });

    MODEM_SYSCON::regs()
        .clk_conf1()
        .modify(|r, w| unsafe { w.bits(r.bits() | 0x1fb) });

    MODEM_SYSCON::regs().clk_conf1().modify(|_, w| {
        w.clk_fe_apb_en().set_bit();
        w.clk_fe_80m_en().set_bit();
        w.clk_fe_160m_en().set_bit();
        w.clk_fe_dac_en().set_bit();
        w.clk_fe_pwdet_adc_en().set_bit();
        w.clk_fe_adc_en().set_bit()
    });
}

#[allow(unused)]
pub(super) fn enable_wifi(en: bool) {
    MODEM_SYSCON::regs().clk_conf1().modify(|_, w| {
        w.clk_wifi_apb_en().set_bit();
        w.clk_wifibb_44m_en().set_bit();
        w.clk_wifimac_en().set_bit();
        w.clk_fe_apb_en().set_bit();
        w.clk_fe_80m_en().set_bit();
        w.clk_fe_160m_en().set_bit()
    });
}

#[allow(unused)]
pub(super) fn enable_ieee802154(en: bool) {
    todo!()
}

#[allow(unused)]
pub(super) fn enable_bt(en: bool) {
    todo!()
}

#[allow(unused)]
pub(super) fn reset_wifi_mac() {
    // empty
}

// #[allow(unused)]
pub(super) fn init_clocks() {
    MODEM_LPCON::regs()
        .clk_conf()
        .modify(|_, w| w.clk_i2c_mst_en().set_bit().clk_wifipwr_en().set_bit());
}

#[allow(unused)]
pub(super) fn ble_rtc_clk_init() {
    // nothing for this target (yet)
}

#[allow(unused)]
pub(super) fn reset_rpa() {
    // nothing for this target (yet)
}
