fn ble_ieee802154_clock_enable(en: bool) {
    regs!(MODEM_SYSCON).clk_conf().modify(|_, w| {
        w.clk_zb_apb_en().bit(en);
        w.clk_zb_mac_en().bit(en)
    });

    regs!(MODEM_SYSCON).clk_conf1().modify(|_, w| {
        w.clk_bt_apb_en().bit(en);
        w.clk_bt_en().bit(en);
        w.clk_fe_16m_en().bit(en);
        w.clk_fe_32m_en().bit(en);
        w.clk_fe_adc_en().bit(en);
        w.clk_fe_apb_en().bit(en);
        w.clk_fe_sdm_en().bit(en)
    });

    regs!(MODEM_LPCON)
        .clk_conf()
        .modify(|_, w| w.clk_coex_en().bit(en));
}

pub(crate) fn enable_bt(en: bool) {
    ble_ieee802154_clock_enable(en);
}

pub(crate) fn enable_ieee802154(en: bool) {
    ble_ieee802154_clock_enable(en);
}

pub(crate) fn init_clocks() {
    regs!(PMU)
        .hp_sleep_icg_modem()
        .modify(|_, w| unsafe { w.hp_sleep_dig_icg_modem_code().bits(0) });
    regs!(PMU)
        .hp_modem_icg_modem()
        .modify(|_, w| unsafe { w.hp_modem_dig_icg_modem_code().bits(1) });
    regs!(PMU)
        .hp_active_icg_modem()
        .modify(|_, w| unsafe { w.hp_active_dig_icg_modem_code().bits(2) });
    regs!(PMU)
        .imm_modem_icg()
        .write(|w| w.update_dig_icg_modem_en().set_bit());
    regs!(PMU)
        .imm_sleep_sysclk()
        .write(|w| w.update_dig_icg_switch().set_bit());

    regs!(MODEM_LPCON).clk_conf().modify(|_, w| {
        w.clk_i2c_mst_en().set_bit();
        w.clk_coex_en().set_bit();
        w.clk_fe_mem_en().set_bit()
    });
}

pub(crate) fn ble_rtc_clk_init() {
    // nothing for this target (yet)
}

pub(crate) fn reset_rpa() {
    // nothing for this target (yet)
}
