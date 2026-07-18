pub fn fs_common_init() {
    crate::peripherals::LPWR::regs().usb_conf().modify(|_, w| {
        w.sw_hw_usb_phy_sel().set_bit();
        w.sw_usb_phy_sel().set_bit()
    });
}
