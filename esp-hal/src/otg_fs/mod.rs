//! # USB On-The-Go (USB OTG)
//!
//! ## Overview
//! The USB OTG Full-speed (FS) peripheral allows communication
//! with USB devices using either blocking (usb-device) or asynchronous
//! (embassy-usb) APIs.
//!
//! It can operate as either a USB Host or Device, and supports full-speed (FS)
//! and low-speed (LS) data rates of the USB 2.0 specification.
//!
//! The blocking driver uses the `esp_synopsys_usb_otg` crate, which provides
//! the `USB bus` implementation and `USB peripheral traits`.
//!
//! The asynchronous driver uses the `embassy_usb_synopsys_otg` crate, which
//! provides the `USB bus` and `USB device` implementations.
//!
//! The module also relies on other peripheral modules, such as `GPIO`,
//! `system`, and `clock control`, to configure and enable the `USB` peripheral.
//!
//! ## Configuration
//! To use the USB OTG Full-speed peripheral driver, you need to initialize the
//! peripheral and configure its settings. The [`Usb`] struct represents the USB
//! peripheral and requires the GPIO pins that implement [`UsbDp`], and
//! [`UsbDm`], which define the specific types used for USB pin selection.
//!
//! The returned `Usb` instance can be used with the `usb-device` crate, or it
//! can be further configured with [`asynch::Driver`] to be used with the
//! `embassy-usb` crate.
//!
//! ## Examples
//! Visit the [USB Serial] example for an example of using the USB OTG
//! peripheral.
//!
//! [USB Serial]: https://github.com/esp-rs/esp-hal/blob/main/examples/peripheral/usb_serial/src/main.rs
//!
//! ## Implementation State
//! - Low-speed (LS) is not supported.

pub use esp_synopsys_usb_otg::UsbBus;

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
    const HIGH_SPEED: bool = false;
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
        InputSignal::USB_OTG_VBUSVALID.connect_to(&Level::High); // receiving a valid Vbus from device
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
        InputSignal::USB_SRP_BVALID.connect_to(&Level::Low); // HIGH to force USB device mode
        InputSignal::USB_OTG_IDDIG.connect_to(&Level::Low); // Indicate A-Device by floating the ID pin
        InputSignal::USB_OTG_VBUSVALID.connect_to(&Level::High);
        InputSignal::USB_OTG_AVALID.connect_to(&Level::High); // Assume valid A device
    }

    fn _disable() {
        // TODO
    }
}

// unsafe impl Sync for Usb<'_> {}

unsafe impl esp_synopsys_usb_otg::UsbPeripheral for Usb<'_> {
    const REGISTERS: *const () = Self::REGISTERS;
    const HIGH_SPEED: bool = Self::HIGH_SPEED;
    const FIFO_DEPTH_WORDS: usize = Self::FIFO_DEPTH_WORDS;
    const ENDPOINT_COUNT: usize = Self::MAX_EP_COUNT;

    fn enable() {
        Self::_enable();
    }

    fn ahb_frequency_hz(&self) -> u32 {
        // unused
        80_000_000
    }
}
