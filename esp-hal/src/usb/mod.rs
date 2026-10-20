//! USB support

#[cfg(usb_serial_jtag_driver_supported)]
pub mod usb_serial_jtag;

#[cfg(any(usb_otg_driver_supported, usb_otg_hs_driver_supported))]
pub mod otg;
