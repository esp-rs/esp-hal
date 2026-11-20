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

impl PhyPads {
    /// Returns the GPIOs of the D- and D+ pads.
    pub(crate) fn gpios(self) -> [u8; 2] {
        let mut gpios = [0; 2];
        for_each_analog_function! {
            (USJ_DM, $gpio:ident) => {
                if self == Self::Usj {
                    gpios[0] = peripherals::$gpio::NUMBER;
                }
            };
            (USJ_DP, $gpio:ident) => {
                if self == Self::Usj {
                    gpios[1] = peripherals::$gpio::NUMBER;
                }
            };
            (USB_FS_DM, $gpio:ident) => {
                if self == Self::UsbFs {
                    gpios[0] = peripherals::$gpio::NUMBER;
                }
            };
            (USB_FS_DP, $gpio:ident) => {
                if self == Self::UsbFs {
                    gpios[1] = peripherals::$gpio::NUMBER;
                }
            };
        }
        gpios
    }
}

/// Returns the pad group of `gpio`, if the pin is a D+ or D- pin of an internal USB PHY.
fn pads_of(gpio: u8) -> Option<PhyPads> {
    for_each_analog_function! {
        (USJ_DP, $gpio:ident) => {
            if gpio == peripherals::$gpio::NUMBER {
                return Some(PhyPads::Usj);
            }
        };
        (USJ_DM, $gpio:ident) => {
            if gpio == peripherals::$gpio::NUMBER {
                return Some(PhyPads::Usj);
            }
        };
        (USB_FS_DP, $gpio:ident) => {
            if gpio == peripherals::$gpio::NUMBER {
                return Some(PhyPads::UsbFs);
            }
        };
        (USB_FS_DM, $gpio:ident) => {
            if gpio == peripherals::$gpio::NUMBER {
                return Some(PhyPads::UsbFs);
            }
        };
    }
    None
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

/// Disconnects the USB controller that drives the pad of `gpio`, so the pad works as a GPIO.
///
/// The pad and pull bits of each controller follow the controller to the PHY that it is mapped
/// to, so the PHY selection decides which register to write.
pub(crate) fn release_pad(gpio: u8) {
    let Some(pads) = pads_of(gpio) else {
        return;
    };
    if usj_pads() == Some(pads) {
        // Workaround to make D+ and D- work when the pin is assigned to
        // the `USB_SERIAL_JTAG` peripheral by default.
        USB_DEVICE::regs().conf0().modify(|_, w| {
            w.usb_pad_enable().clear_bit();
            w.dm_pullup().clear_bit();
            w.dm_pulldown().clear_bit();
            w.dp_pullup().clear_bit();
            w.dp_pulldown().clear_bit()
        });
    } else {
        #[cfg(any(esp32h4, esp32p4))]
        peripherals::USB_WRAP::regs().otg_conf().modify(|_, w| {
            w.pad_pull_override().set_bit();
            w.dm_pullup().clear_bit();
            w.dm_pulldown().clear_bit();
            w.dp_pullup().clear_bit();
            w.dp_pulldown().clear_bit()
        });
    }
}
