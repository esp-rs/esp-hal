// Clock dependencies follow ESP-IDF's `esp_hw_support/modem/port/esp32h4/modem_clock_impl.c`.

pub(crate) fn enable_ieee802154(en: bool) {
    enable_bt_ieee802154_common(en);
    regs!(MODEM_SYSCON).clk_conf().modify(|_, w| {
        w.clk_zb_apb_en().bit(en);
        w.clk_zbmac_en().bit(en)
    });
}

pub(crate) fn enable_bt(en: bool) {
    enable_bt_ieee802154_common(en);
    regs!(MODEM_SYSCON).clk_conf().modify(|_, w| {
        w.clk_modem_sec_en().bit(en);
        w.clk_modem_sec_ecb_en().bit(en);
        w.clk_modem_sec_ccm_en().bit(en);
        w.clk_modem_sec_bah_en().bit(en);
        w.clk_ble_timer_en().bit(en)
    });
    regs!(MODEM_SYSCON)
        .clk_conf1()
        .modify(|_, w| w.clk_btmac_en().bit(en));
}

/// The clocks that BLE and IEEE 802.15.4 share, so they stay on for as long as either of them
/// is up.
fn enable_bt_ieee802154_common(en: bool) {
    fn enable_common(en: bool) {
        regs!(MODEM_SYSCON).clk_conf().modify(|_, w| {
            w.clk_etm_en().bit(en);
            w.clk_modem_sec_apb_en().bit(en)
        });
        regs!(MODEM_SYSCON).clk_conf1().modify(|_, w| {
            w.clk_bt_apb_en().bit(en);
            w.clk_btbb_en().bit(en)
        });
        regs!(MODEM_LPCON)
            .clk_conf()
            .modify(|_, w| w.clk_coex_en().bit(en));
    }

    use crate::radio_clocks::Refcount;

    static REFCOUNT: Refcount = Refcount::new();
    REFCOUNT.update(en, enable_common);
}

pub(crate) fn init_clocks() {
    const ICG_NOGATING_MODEM: u8 = 1 << 1;
    const ICG_NOGATING_ACTIVE: u8 = 1 << 2;
    const ICG_NOGATING_ACTIVE_MODEM: u8 = ICG_NOGATING_ACTIVE | ICG_NOGATING_MODEM;

    // IDF `modem_clock_domain_icg_config`. The maps must be in place before the PMU switches to
    // the codes below, or the switch gates the modem clocks.
    regs!(MODEM_SYSCON)
        .clk_conf_power_st()
        .modify(|r, w| unsafe {
            w.clk_modem_apb_st_map()
                .bits(r.clk_modem_apb_st_map().bits() | ICG_NOGATING_ACTIVE_MODEM);
            w.clk_modem_peri_st_map()
                .bits(r.clk_modem_peri_st_map().bits() | ICG_NOGATING_ACTIVE);
            w.clk_bt_st_map()
                .bits(r.clk_bt_st_map().bits() | ICG_NOGATING_ACTIVE);
            w.clk_fe_st_map()
                .bits(r.clk_fe_st_map().bits() | ICG_NOGATING_ACTIVE_MODEM);
            w.clk_zb_st_map()
                .bits(r.clk_zb_st_map().bits() | ICG_NOGATING_ACTIVE)
        });
    regs!(MODEM_LPCON)
        .clk_conf_power_st()
        .modify(|r, w| unsafe {
            w.clk_lp_apb_st_map()
                .bits(r.clk_lp_apb_st_map().bits() | ICG_NOGATING_ACTIVE_MODEM);
            w.clk_i2c_mst_st_map()
                .bits(r.clk_i2c_mst_st_map().bits() | ICG_NOGATING_ACTIVE_MODEM);
            w.clk_coex_st_map()
                .bits(r.clk_coex_st_map().bits() | ICG_NOGATING_ACTIVE_MODEM)
        });

    // IDF `pmu_init` defaults (`PMU_HP_*_CLOCK_CONFIG_DEFAULT` in `pmu_param.c`).
    let pmu = regs!(PMU);
    pmu.hp_sleep_icg_modem()
        .modify(|_, w| unsafe { w.hp_sleep_dig_icg_modem_code().bits(0) });
    pmu.hp_modem_icg_modem()
        .modify(|_, w| unsafe { w.hp_modem_dig_icg_modem_code().bits(1) });
    pmu.hp_active_icg_modem()
        .modify(|_, w| unsafe { w.hp_active_dig_icg_modem_code().bits(2) });
    pmu.imm_modem_icg()
        .write(|w| w.update_dig_icg_modem_en().set_bit());
    pmu.imm_sleep_sysclk()
        .write(|w| w.update_dig_icg_switch().set_bit());
}

pub(crate) fn deinit_clocks() {
    // IDF sets the ICG configuration once and keeps it.
}

pub(crate) fn ble_rtc_clk_init() {
    // `BLE_LP_CLK` is programmed by the clock tree.
}

pub(crate) fn reset_rpa() {
    // nothing for this target (yet)
}
