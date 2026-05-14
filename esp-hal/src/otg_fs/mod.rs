//! USB peripheral driver.
//!
//! This module provides support for `embassy-usb` (both host and device).
//!
//! Start by creating a [`Usb`] instance from the `USB0` peripheral and the
//! D+ / D- pins, then pass it to the driver of your choice:
//!
//! ```rust, ignore
//! let usb = Usb::new(peripherals.USB0, peripherals.GPIO20, peripherals.GPIO19);
//! ```

use crate::{
    gpio::InputSignal,
    peripherals::{self, USB0},
    system::{GenericPeripheralGuard, Peripheral as PeripheralEnable},
};

pub mod embassy_usb_device;
pub mod embassy_usb_host;

#[doc(hidden)]
/// Trait representing the USB D+ (data plus) pin.
pub trait UsbDp: crate::private::Sealed {}

#[doc(hidden)]
/// Trait representing the USB D- (data minus) pin.
pub trait UsbDm: crate::private::Sealed {}

for_each_analog_function! {
    (USB_DM, $gpio:ident) => {
        impl UsbDm for crate::peripherals::$gpio<'_> {}
    };
    (USB_DP, $gpio:ident) => {
        impl UsbDp for crate::peripherals::$gpio<'_> {}
    };
}

/// USB peripheral.
pub struct Usb<'d> {
    _usb0: peripherals::USB0<'d>,
    _guard: GenericPeripheralGuard<{ PeripheralEnable::Usb as u8 }>,
}

impl<'d> Usb<'d> {
    const REGISTERS: *const () = USB0::PTR.cast();
    const FIFO_DEPTH_WORDS: usize = 256;

    // S2/S3 are identical: Six additional endpoints (endpoint numbers 1 to 6), configurable as IN
    // or OUT
    const MAX_EP_COUNT: usize = 7;

    // S2/S3 are identical: Eight channels (pipes)
    const MAX_HOST_CH_COUNT: usize = 8;

    /// Creates a new `Usb` instance.
    pub fn new(
        usb0: peripherals::USB0<'d>,
        _usb_dp: impl UsbDp + 'd,
        _usb_dm: impl UsbDm + 'd,
    ) -> Self {
        let guard = GenericPeripheralGuard::new();

        Self {
            _usb0: usb0,
            _guard: guard,
        }
    }

    fn _enable() {
        peripherals::USB_WRAP::regs().otg_conf().modify(|_, w| {
            w.usb_pad_enable().set_bit();
            w.phy_sel().clear_bit();
            w.clk_en().set_bit();
            w.ahb_clk_force_on().set_bit();
            w.phy_clk_force_on().set_bit()
        });

        #[cfg(esp32s3)]
        peripherals::LPWR::regs().usb_conf().modify(|_, w| {
            w.sw_hw_usb_phy_sel().set_bit();
            w.sw_usb_phy_sel().set_bit()
        });

        use crate::gpio::Level;

        InputSignal::USB_OTG_IDDIG.connect_to(&Level::High); // connected connector is mini-B side
        InputSignal::USB_SRP_BVALID.connect_to(&Level::High); // HIGH to force USB device mode
        InputSignal::USB_OTG_VBUSVALID.connect_to(&Level::High); // receiving a valid Vbus from host
        InputSignal::USB_OTG_AVALID.connect_to(&Level::Low);
    }

    fn _enable_host() {
        peripherals::USB_WRAP::regs().otg_conf().modify(|_, w| {
            w.usb_pad_enable().set_bit();
            w.phy_sel().clear_bit();
            w.clk_en().set_bit();
            w.ahb_clk_force_on().set_bit();
            w.phy_clk_force_on().set_bit();
            w.pad_pull_override().set_bit();
            w.dp_pulldown().set_bit();
            w.dm_pulldown().set_bit();
            w.dp_pullup().clear_bit();
            w.dm_pullup().clear_bit()
        });

        #[cfg(esp32s3)]
        peripherals::LPWR::regs().usb_conf().modify(|_, w| {
            w.sw_hw_usb_phy_sel().set_bit();
            w.sw_usb_phy_sel().set_bit()
        });

        use crate::gpio::Level;
        InputSignal::USB_SRP_BVALID.connect_to(&Level::Low);
        InputSignal::USB_OTG_IDDIG.connect_to(&Level::Low); // Indicate A-Device
        InputSignal::USB_OTG_VBUSVALID.connect_to(&Level::High);
        InputSignal::USB_OTG_AVALID.connect_to(&Level::High); // Assume valid A device
    }

    fn _disable() {
        // TODO
    }
}
