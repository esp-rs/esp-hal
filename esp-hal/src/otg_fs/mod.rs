#![cfg_attr(docsrs, procmacros::doc_replace(
    "dm" => {
        cfg(any(esp32s2, esp32s3)) => "GPIO19",
        cfg(esp32p4) => "GPIO26",
    },
    "dp" => {
        cfg(any(esp32s2, esp32s3)) => "GPIO20",
        cfg(esp32p4) => "GPIO27",
    },
))]
//! USB peripheral driver.
//!
//! This module provides support for `embassy-usb` (both host and device).
//!
//! Start by creating a [`Usb`] instance from the `USB_FS` peripheral and the
//! D+ / D- pins, then pass it to the driver of your choice:
//!
//! ```rust, ignore
//! let usb = Usb::new(peripherals.USB_FS, peripherals.__dp__, peripherals.__dm__);
//! ```

use crate::{
    gpio::InputSignal,
    peripherals::{self, USB_FS},
    system::{GenericPeripheralGuard, Peripheral as PeripheralEnable},
};

pub mod embassy_usb_device;
pub mod embassy_usb_host;

/// USB D+ (data plus) pin.
pub trait UsbFsDp: crate::private::Sealed {}

/// USB D- (data minus) pin.
pub trait UsbFsDm: crate::private::Sealed {}

for_each_analog_function! {
    (USB_FS_DM, $gpio:ident) => {
        impl UsbFsDm for crate::peripherals::$gpio<'_> {}
    };
    (USB_FS_DP, $gpio:ident) => {
        impl UsbFsDp for crate::peripherals::$gpio<'_> {}
    };
}

/// USB peripheral.
pub struct Usb<'d> {
    _usb: peripherals::USB_FS<'d>,
    _guard: GenericPeripheralGuard<{ PeripheralEnable::UsbFs as u8 }>,
}

impl<'d> Usb<'d> {
    const REGISTERS: *const () = USB_FS::PTR.cast();
    const FIFO_DEPTH_WORDS: usize = 256;

    // S2/S3 are identical: Six additional endpoints (endpoint numbers 1 to 6), configurable as IN
    // or OUT
    const MAX_EP_COUNT: usize = 7;

    // S2/S3 are identical: Eight channels (pipes)
    const MAX_HOST_CH_COUNT: usize = 8;

    /// Creates a new `Usb` instance.
    pub fn new(
        usb: peripherals::USB_FS<'d>,
        _usb_dp: impl UsbFsDp + 'd,
        _usb_dm: impl UsbFsDm + 'd,
    ) -> Self {
        let guard = GenericPeripheralGuard::new();

        Self {
            _usb: usb,
            _guard: guard,
        }
    }

    fn _enable() {
        #[cfg(esp32p4)]
        Self::_p4_init();

        peripherals::USB_WRAP::regs().otg_conf().modify(|_, w| {
            w.usb_pad_enable().set_bit();
            w.phy_sel().clear_bit();
            w.clk_en().set_bit();
            w.ahb_clk_force_on().set_bit();
            w.phy_clk_force_on().set_bit();
            w.pad_pull_override().clear_bit();

            w
        });

        #[cfg(esp32s3)]
        peripherals::LPWR::regs().usb_conf().modify(|_, w| {
            w.sw_hw_usb_phy_sel().set_bit();
            w.sw_usb_phy_sel().set_bit()
        });

        use crate::gpio::Level;

        InputSignal::USB_FS_IDDIG.connect_to(&Level::High); // connected connector is mini-B side
        InputSignal::USB_FS_SRP_BVALID.connect_to(&Level::High); // HIGH to force USB device mode
        InputSignal::USB_FS_VBUSVALID.connect_to(&Level::High); // receiving a valid Vbus from host
        InputSignal::USB_FS_AVALID.connect_to(&Level::Low);
    }

    fn _enable_host() {
        #[cfg(esp32p4)]
        Self::_p4_init();

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
        InputSignal::USB_FS_SRP_BVALID.connect_to(&Level::Low);
        InputSignal::USB_FS_IDDIG.connect_to(&Level::Low); // Indicate A-Device
        InputSignal::USB_FS_VBUSVALID.connect_to(&Level::High);
        InputSignal::USB_FS_AVALID.connect_to(&Level::High); // Assume valid A device
    }

    #[cfg(esp32p4)]
    fn _p4_init() {
        // Enable the 48MHz FSLS PHY clock.

        use crate::RegisterToggle;
        peripherals::LP_AON_CLKRST::regs()
            .lp_aonclkrst_hp_usb_clkrst_ctrl0()
            .modify(|_, w| w.lp_aonclkrst_usb_otg11_48m_clk_en().set_bit());

        // Assert then deassert reset for USB_DWC_FS and USB_WRAP.
        peripherals::LP_AON_CLKRST::regs()
            .lp_aonclkrst_hp_usb_clkrst_ctrl1()
            .toggle(|w, en| w.lp_aonclkrst_rst_en_usb_otg11().bit(en));

        // GPIO26 = DM, GPIO27 = DP: set to maximum drive strength (40mA) for
        // USB FSLS signal integrity. The pads are shared with the GPIO peripheral.
        peripherals::IO_MUX::regs()
            .gpio(26)
            .modify(|_, w| unsafe { w.fun_drv().bits(3) });
        peripherals::IO_MUX::regs()
            .gpio(27)
            .modify(|_, w| unsafe { w.fun_drv().bits(3) });
    }

    fn _disable() {
        // TODO
    }
}
