use crate::peripherals::PMU;

pub(super) fn enable_phy(en: bool) {
    // empty
}

#[cfg_attr(not(feature = "unstable"), expect(unused))]
pub(super) fn enable_wifi(en: bool) {
    // empty
}

#[cfg_attr(not(feature = "unstable"), expect(unused))]
pub(super) fn enable_ieee802154(en: bool) {
    // empty
}

#[cfg_attr(not(feature = "unstable"), expect(unused))]
pub(super) fn enable_bt(en: bool) {
    // empty
}

pub(super) fn reset_wifi_mac() {
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

    }
}

pub(super) fn ble_rtc_clk_init() {
    // nothing for this target (yet)
}

pub(super) fn reset_rpa() {
    // nothing for this target (yet)
}
