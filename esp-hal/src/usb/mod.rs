//! USB support

#[cfg(usb_serial_jtag_driver_supported)]
pub mod usb_serial_jtag;

#[cfg(usb_otg_driver_supported)]
pub mod otg_fs;

#[cfg(usb_otg_hs_driver_supported)]
pub mod otg_hs;
