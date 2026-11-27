#[cfg(any(esp32s3, esp32h4, esp32p4))]
use crate::efuse;
use crate::{
    peripherals::{self, USB_DEVICE},
    system::{Peripheral, PeripheralClockControl},
};

/// A pad group of an internal USB PHY.
#[derive(Clone, Copy, PartialEq, Eq)]
#[doc(hidden)]
pub enum PhyPads {
    Usj,
    UsbFs,
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

/// Disconnects the USB Serial/JTAG controller from its pads and removes the pulls, so the host
/// sees the device detach.
pub(crate) fn disconnect_usj() {
    USB_DEVICE::regs().conf0().modify(|_, w| {
        w.usb_pad_enable().clear_bit();
        w.dm_pullup().clear_bit();
        w.dm_pulldown().clear_bit();
        w.dp_pullup().clear_bit();
        w.dp_pulldown().clear_bit()
    });
}

/// Gives the USB pads back to the USB Serial/JTAG controller, in the state that the ROM download
/// mode expects.
///
/// On the ESP32-S3, if the USB OTG controller drives the pads, it first detaches from the host.
/// This blocks for 300 ms.
pub(crate) fn prepare_usj_for_rom_download() {
    cfg_select! {
        esp32s3 => {
            let usb_conf = peripherals::LPWR::regs().usb_conf();
            let phy_sel = usb_conf.read();
            if phy_sel.sw_hw_usb_phy_sel().bit_is_set() && phy_sel.sw_usb_phy_sel().bit_is_set() {
                // `USB_WRAP` registers are on the bus clock of the USB OTG peripheral.
                PeripheralClockControl::enable(Peripheral::UsbFs);
                peripherals::USB_WRAP::regs().otg_conf().modify(|_, w| {
                    w.pad_pull_override().set_bit();
                    w.dp_pullup().clear_bit();
                    w.dm_pullup().clear_bit()
                });
                // The host must see the detach before USB Serial/JTAG takes the same pads, or it
                // keeps talking to the OTG device and does not enumerate the ROM loader.
                crate::rom::ets_delay_us(300_000);
                peripherals::USB_WRAP::regs()
                    .otg_conf()
                    .modify(|_, w| w.usb_pad_enable().clear_bit());
            }
            // The PHY selection is in the RTC domain and keeps its value through a software reset.
            // See <https://github.com/espressif/arduino-esp32/blob/c8f45c85dcda0e837d9ffdbf0ed3c16a4620c7a4/cores/esp32/esp32-hal-tinyusb.c#L632-L636>.
            usb_conf.modify(|_, w| {
                w.sw_hw_usb_phy_sel().clear_bit();
                w.sw_usb_phy_sel().clear_bit()
            });
        }
        _ => {}
    }

    PeripheralClockControl::enable(Peripheral::UsbDevice);
    USB_DEVICE::regs().conf0().reset();
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
        disconnect_usj();
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
