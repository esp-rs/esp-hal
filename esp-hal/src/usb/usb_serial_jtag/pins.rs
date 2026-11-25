use super::ConfigError;
use crate::{
    peripherals::USB_DEVICE,
    soc::usb_phy::{self, PhyPads},
};

/// A D+ or D- pin of a PHY that the USB Serial/JTAG controller can use.
#[instability::unstable]
pub trait UsbSerialJtagPin: crate::private::Sealed {
    #[doc(hidden)]
    fn phy_pads(&self) -> PhyPads;
}

macro_rules! impl_pin {
    ($gpio:ident, $pads:ident, $cfg:ident) => {
        #[cfg($cfg)]
        impl UsbSerialJtagPin for crate::peripherals::$gpio<'_> {
            fn phy_pads(&self) -> PhyPads {
                PhyPads::$pads
            }
        }
    };
}

for_each_analog_function! {
    (USJ_DP, $gpio:ident) => { impl_pin!($gpio, Usj, usb_serial_jtag_phy_pads_usj); };
    (USJ_DM, $gpio:ident) => { impl_pin!($gpio, Usj, usb_serial_jtag_phy_pads_usj); };
    (USB_FS_DP, $gpio:ident) => { impl_pin!($gpio, UsbFs, usb_serial_jtag_phy_pads_usb_fs); };
    (USB_FS_DM, $gpio:ident) => { impl_pin!($gpio, UsbFs, usb_serial_jtag_phy_pads_usb_fs); };
}

pub(super) fn validate_pins(
    dp: &impl UsbSerialJtagPin,
    dm: &impl UsbSerialJtagPin,
) -> Result<(), ConfigError> {
    let active = usb_phy::usj_pads().ok_or(ConfigError::UnsupportedPhy)?;
    if dp.phy_pads() != active || dm.phy_pads() != active {
        return Err(ConfigError::WrongPins);
    }
    Ok(())
}

/// Restores the pad settings that the GPIO driver clears when a program uses the pins as GPIOs.
///
/// The pad and pull bits of `CONF0` belong to the controller, and the PHY selection routes them to
/// the selected PHY. Do not write `USB_WRAP` here: its bits belong to the OTG controller.
pub(super) fn enable_pads() {
    USB_DEVICE::regs().conf0().modify(|_, w| {
        w.usb_pad_enable().set_bit();
        w.dm_pullup().clear_bit();
        w.dm_pulldown().clear_bit();
        w.dp_pullup().set_bit();
        w.dp_pulldown().clear_bit()
    });
}
