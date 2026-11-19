#[cfg(any(esp32s3, esp32h4, esp32p4))]
use crate::efuse;
use crate::peripherals::{self, USB_DEVICE};

/// A pad group of an internal USB PHY.
#[derive(Clone, Copy, PartialEq, Eq)]
#[doc(hidden)]
pub enum PhyPads {
    Usj,
    UsbFs,
}

/// Returns whether software or an eFuse moved the USB Serial/JTAG controller away from its default
/// PHY.
fn phy_swapped() -> bool {
    #[cfg(esp32s3)]
    if USB_DEVICE::regs().conf0().read().phy_sel().bit_is_set() {
        return true;
    }

    cfg_select! {
        any(esp32s3, esp32h4, esp32p4, esp32s31) => {
            let phy_sel = cfg_select! {
                esp32s3 => peripherals::LPWR::regs().usb_conf().read(),
                esp32h4 => peripherals::LP_AON::regs().usb_phy_sel().read(),
                _ => peripherals::LP_SYS::regs().usb_ctrl().read(),
            };
            if phy_sel.sw_hw_usb_phy_sel().bit_is_set() {
                phy_sel.sw_usb_phy_sel().bit_is_set()
            } else {
                cfg_select! {
                    // The ESP32-S31 has no `USB_PHY_SEL` eFuse.
                    esp32s31 => false,
                    _ => efuse::read_bit(efuse::USB_PHY_SEL),
                }
            }
        }
        _ => false,
    }
}

/// Returns the pads that the USB Serial/JTAG controller drives, or `None` if it uses a PHY with
/// unknown pads, for example an external PHY.
pub(crate) fn usj_pads() -> Option<PhyPads> {
    let default = if cfg!(usb_serial_jtag_phy_pads_usj) {
        PhyPads::Usj
    } else {
        PhyPads::UsbFs
    };
    if !phy_swapped() {
        return Some(default);
    }
    // The other PHY has known pads only if the metadata lists two pad groups.
    if cfg!(all(
        usb_serial_jtag_phy_pads_usj,
        usb_serial_jtag_phy_pads_usb_fs
    )) {
        Some(PhyPads::UsbFs)
    } else {
        None
    }
}
