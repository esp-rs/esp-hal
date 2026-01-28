// done
#[allow(unused)]
pub(super) fn enable_phy(_en: bool) {
    todo!() // update PACs
    // PCR::regs()
    //     .reset_event_bypass()
    //     .modify(|_, w| w.reset_event_bypass(en));
    // MODEM_LPCON::regs()
    //     .clk_conf()
    //     .modify(|_, w| w.clk_i2c_mst_en().bit(en));
    // MODEM_LPCON::regs()
    //     .i2c_mst_clk_conf()
    //     .modify(|_, w| w.clk_i2c_mst_sel_160m().bit(en));
}

#[allow(unused)]
pub(super) fn enable_wifi(en: bool) {
    todo!()
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
    todo!()
}

#[allow(unused)]
pub(super) fn ble_rtc_clk_init() {
    // nothing for this target (yet)
}

#[allow(unused)]
pub(super) fn reset_rpa() {
    // nothing for this target (yet)
}
