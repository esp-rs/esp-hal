use crate::peripherals::{MODEM_LPCON, MODEM_SYSCON, PMU};

pub(super) fn enable_phy(en: bool) {
    MODEM_LPCON::regs()
        .clk_conf()
        .modify(|_, w| w.clk_i2c_mst_en().bit(en));
}

fn ble_ieee802154_clock_enable(en: bool) {
    MODEM_SYSCON::regs().clk_conf().modify(|_, w| {
        w.clk_zb_apb_en().bit(en);
        w.clk_zb_mac_en().bit(en)
    });

    MODEM_SYSCON::regs().clk_conf1().modify(|_, w| {
        w.clk_bt_apb_en().bit(en);
        w.clk_bt_en().bit(en);
        w.clk_fe_16m_en().bit(en);
        w.clk_fe_32m_en().bit(en);
        w.clk_fe_adc_en().bit(en);
        w.clk_fe_apb_en().bit(en);
        w.clk_fe_sdm_en().bit(en)
    });

    MODEM_LPCON::regs()
        .clk_conf()
        .modify(|_, w| w.clk_coex_en().bit(en));
}

#[cfg_attr(not(feature = "unstable"), expect(unused))]
pub(super) fn enable_bt(en: bool) {
    ble_ieee802154_clock_enable(en);
}

#[cfg_attr(not(feature = "unstable"), expect(unused))]
pub(super) fn enable_ieee802154(en: bool) {
    ble_ieee802154_clock_enable(en);
}

pub(super) fn init_clocks() {
    PMU::regs()
        .hp_sleep_icg_modem()
        .modify(|_, w| unsafe { w.hp_sleep_dig_icg_modem_code().bits(0) });
    PMU::regs()
        .hp_modem_icg_modem()
        .modify(|_, w| unsafe { w.hp_modem_dig_icg_modem_code().bits(1) });
    PMU::regs()
        .hp_active_icg_modem()
        .modify(|_, w| unsafe { w.hp_active_dig_icg_modem_code().bits(2) });
    PMU::regs()
        .imm_modem_icg()
        .write(|w| w.update_dig_icg_modem_en().set_bit());
    PMU::regs()
        .imm_sleep_sysclk()
        .write(|w| w.update_dig_icg_switch().set_bit());

    MODEM_LPCON::regs().clk_conf().modify(|_, w| {
        w.clk_i2c_mst_en().set_bit();
        w.clk_coex_en().set_bit();
        w.clk_fe_mem_en().set_bit()
    });
}

pub(super) fn ble_rtc_clk_init() {
    // nothing for this target (yet)
}

pub(super) fn reset_rpa() {
    // nothing for this target (yet)
}
