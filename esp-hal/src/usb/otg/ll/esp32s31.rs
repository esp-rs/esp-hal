use crate::peripherals::{CNNT_SYS, HP_ALIVE_SYS, HP_SYS_CLKRST};

pub fn hs_enable_device_mode() {
    hs_init();
    connect_hs_pulldowns(false);
}

pub fn hs_enable_host_mode() {
    hs_init();
    connect_hs_pulldowns(true);
}

fn hs_init() {
    // Mirrors ESP-IDF's S31 UTMI initialization:
    // https://github.com/espressif/esp-idf/blob/055ba9d3f9c6fd9a0efacd4993a2a942972dd65d/components/esp_hal_usb/usb_utmi_hal.c#L10-L23
    // https://github.com/espressif/esp-idf/blob/055ba9d3f9c6fd9a0efacd4993a2a942972dd65d/components/esp_hal_usb/esp32s31/include/hal/usb_utmi_ll.h#L52-L137
    HP_SYS_CLKRST::regs().usb_otghs_ctrl0().modify(|_, w| {
        w.usb_otghs_apb_clk_en().set_bit();
        w.usb_otghs_sys_clk_en().set_bit()
    });

    CNNT_SYS::regs().sys_usb_otg20_ctrl().modify(|_, w| {
        w.sys_usb_otg20_utmifs_clk_en().set_bit();
        w.sys_usb_otg20_phyref_clk_en().set_bit()
    });

    HP_ALIVE_SYS::regs().usb_otghs_ctrl().modify(|_, w| {
        w.reg_usb_otghs_phy_suspendm_force_en().clear_bit();
        w.reg_usb_otghs_phy_pll_force_en().clear_bit()
    });

    CNNT_SYS::regs().sys_usb_otg20_ctrl().modify(|_, w| {
        w.sys_usb_otg20_ahb_rst_en().set_bit();
        w.sys_usb_otg20_apb_rst_en().set_bit();
        w.sys_usb_otg20_phy_rst_en().set_bit()
    });
    CNNT_SYS::regs()
        .sys_usb_otg20_ctrl()
        .modify(|_, w| w.sys_usb_otg20_phy_rst_en().clear_bit());
    CNNT_SYS::regs().sys_usb_otg20_ctrl().modify(|_, w| {
        w.sys_usb_otg20_ahb_rst_en().clear_bit();
        w.sys_usb_otg20_apb_rst_en().clear_bit()
    });

    HP_ALIVE_SYS::regs()
        .usb_otghs_ctrl()
        .modify(|_, w| w.reg_usb_otghs_phy_otg_suspendm().set_bit());

    const USB_UTMI: usize = 0x2038_0000;
    let fc_06 = (USB_UTMI as *mut u32).wrapping_add(6);
    // SAFETY: USB_HS ownership provides exclusive access to the UTMI PHY.
    unsafe { fc_06.write_volatile(fc_06.read_volatile() | (1 << 3) | (1 << 0)) };
}

fn connect_hs_pulldowns(connect: bool) {
    HP_ALIVE_SYS::regs().usb_ctrl().modify(|_, w| {
        w.usb_otghs_phy_dppulldown().bit(connect);
        w.usb_otghs_phy_dmpulldown().bit(connect)
    });
}
