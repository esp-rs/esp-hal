use super::ConfigError;

#[derive(Clone, Copy, PartialEq, Eq)]
#[doc(hidden)]
pub enum PhyPads {
    Usj,
    UsbFs,
}

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

/// Returns whether software or an eFuse moved the controller away from its default PHY.
fn phy_swapped() -> bool {
    #[cfg(esp32s3)]
    if crate::peripherals::USB_DEVICE::regs()
        .conf0()
        .read()
        .phy_sel()
        .bit_is_set()
    {
        return true;
    }

    cfg_select! {
        any(esp32s3, esp32h4, esp32p4, esp32s31) => {
            let phy_sel = cfg_select! {
                esp32s3 => crate::peripherals::LPWR::regs().usb_conf().read(),
                esp32h4 => crate::peripherals::LP_AON::regs().usb_phy_sel().read(),
                _ => crate::peripherals::LP_SYS::regs().usb_ctrl().read(),
            };
            if phy_sel.sw_hw_usb_phy_sel().bit_is_set() {
                phy_sel.sw_usb_phy_sel().bit_is_set()
            } else {
                cfg_select! {
                    // The ESP32-S31 has no `USB_PHY_SEL` eFuse.
                    esp32s31 => false,
                    _ => crate::efuse::read_bit(crate::efuse::USB_PHY_SEL),
                }
            }
        }
        _ => false,
    }
}

/// Returns the pads of the PHY that the controller uses.
fn active_phy_pads() -> Result<PhyPads, ConfigError> {
    let default = if cfg!(usb_serial_jtag_phy_pads_usj) {
        PhyPads::Usj
    } else {
        PhyPads::UsbFs
    };
    if !phy_swapped() {
        return Ok(default);
    }
    // The other PHY has known pads only if the metadata lists two pad groups.
    if cfg!(all(
        usb_serial_jtag_phy_pads_usj,
        usb_serial_jtag_phy_pads_usb_fs
    )) {
        Ok(PhyPads::UsbFs)
    } else {
        Err(ConfigError::UnsupportedPhy)
    }
}

pub(super) fn validate_pins(
    dp: &impl UsbSerialJtagPin,
    dm: &impl UsbSerialJtagPin,
) -> Result<PhyPads, ConfigError> {
    let active = active_phy_pads()?;
    if dp.phy_pads() != active || dm.phy_pads() != active {
        return Err(ConfigError::WrongPins);
    }
    Ok(active)
}

/// Restores the pad settings that the GPIO driver clears when a program uses the pins as GPIOs.
pub(super) fn enable_pads(pads: PhyPads) {
    crate::peripherals::USB_DEVICE::regs()
        .conf0()
        .modify(|_, w| {
            w.usb_pad_enable().set_bit();
            w.dm_pullup().clear_bit();
            w.dm_pulldown().clear_bit();
            w.dp_pullup().set_bit();
            w.dp_pulldown().clear_bit()
        });

    #[cfg(any(esp32h4, esp32p4))]
    if pads == PhyPads::UsbFs {
        crate::peripherals::USB_WRAP::regs()
            .otg_conf()
            .modify(|_, w| {
                w.pad_pull_override().clear_bit();
                w.dm_pullup().clear_bit();
                w.dm_pulldown().clear_bit();
                w.dp_pullup().clear_bit();
                w.dp_pulldown().clear_bit()
            });
    }
    #[cfg(not(any(esp32h4, esp32p4)))]
    let _ = pads;
}
