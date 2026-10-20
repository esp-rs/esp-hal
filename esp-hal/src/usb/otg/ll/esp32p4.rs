use crate::{
    RegisterToggle,
    peripherals::{HP_SYS, LP_AON_CLKRST, LP_SYS},
};

pub fn fs_common_init() {
    LP_AON_CLKRST::regs()
        .lp_aonclkrst_hp_usb_clkrst_ctrl0()
        .modify(|_, w| w.lp_aonclkrst_usb_otg11_48m_clk_en().set_bit());

    LP_AON_CLKRST::regs()
        .lp_aonclkrst_hp_usb_clkrst_ctrl1()
        .toggle(|w, en| w.lp_aonclkrst_rst_en_usb_otg11().bit(en));
}

pub fn hs_enable_device_mode() {
    hs_init();
    connect_hs_pulldowns(false);
}

pub fn hs_enable_host_mode() {
    hs_init();
    connect_hs_pulldowns(true);
}

fn hs_init() {
    LP_AON_CLKRST::regs()
        .lp_aonclkrst_hp_usb_clkrst_ctrl1()
        .modify(|_, w| w.lp_aonclkrst_usb_otg20_phyref_clk_en().set_bit());

    LP_AON_CLKRST::regs()
        .lp_aonclkrst_hp_usb_clkrst_ctrl1()
        .toggle(|w, en| {
            w.lp_aonclkrst_rst_en_usb_otg20().bit(en);
            w.lp_aonclkrst_rst_en_usb_otg20_phy().bit(en);
            w
        });

    HP_SYS::regs()
        .usbotg20_ctrl()
        .modify(|_, w| w.otg_suspendm().set_bit());

    const USB_UTMI: usize = 0x5009C000;
    let fc_06 = (USB_UTMI as *mut u32).wrapping_add(6);
    let bits = unsafe { fc_06.read_volatile() };
    unsafe { fc_06.write_volatile(bits | (1 << 3) | (1 << 0)) };
}

fn connect_hs_pulldowns(connect: bool) {
    LP_SYS::regs().hp_usb_otghs_phy_ctrl().modify(|_, w| {
        w.hp_utmiotg_dppulldown().bit(connect);
        w.hp_utmiotg_dmpulldown().bit(connect);
        w
    });
}
