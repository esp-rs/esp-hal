//! USB On-The-Go High-Speed (OTG HS) peripheral driver.
//!
//! Supports two protocol stacks:
//!
//! - **[`embassy-usb`]** device mode — via [`embassy_usb_device`].
//! - **[`embassy-usb-host`]** host mode — via [`embassy_usb_host`].
//!
//! Start by creating a [`Usb`] instance from the `USB_HS` peripheral,
//! then pass it to the driver of your choice:
//!
//! ```rust, no_run
//! # {before_snippet}
//! use esp_hal::usb::otg_hs::Usb;
//!
//! let usb = Usb::new(peripherals.USB_HS);
//! # {after_snippet}
//! ```
//!
//! [`embassy-usb`]: https://crates.io/crates/embassy-usb
//! [`embassy-usb-host`]: https://crates.io/crates/embassy-usb-host

use crate::{
    peripherals::{self, USB_HS},
    system::{GenericPeripheralGuard, Peripheral as PeripheralEnable},
};

pub mod embassy_usb_device;
pub mod embassy_usb_host;

/// USB peripheral.
pub struct Usb<'d> {
    _usb: peripherals::USB_HS<'d>,
    _guard: GenericPeripheralGuard<{ PeripheralEnable::UsbHs as u8 }>,
}

impl<'d> Usb<'d> {
    const REGISTERS: *const () = USB_HS::PTR.cast();
    const FIFO_DEPTH_WORDS: usize = property!("usb_otg_hs.fifo_depth_words");

    const MAX_EP_COUNT: usize = 16;
    const MAX_HOST_CH_COUNT: usize = 16;

    /// Creates a new `Usb` instance.
    pub fn new(usb: peripherals::USB_HS<'d>) -> Self {
        let guard = GenericPeripheralGuard::new();

        Self {
            _usb: usb,
            _guard: guard,
        }
    }

    fn _enable() {
        #[cfg(esp32p4)]
        {
            Self::_p4_init();
            Self::connect_pulldowns(false);
        }
    }

    fn _enable_host() {
        #[cfg(esp32p4)]
        {
            Self::_p4_init();

            Self::connect_pulldowns(true);
        }
    }

    // On ESP32-P4 v3+, pulldowns are no longer controlled by USB-OTG peripheral
    // and must be controlled by software via LP_SYS registers.
    // On earlier revisions, pulldowns are controlled by USB-OTG hardware.
    #[cfg(esp32p4)]
    fn connect_pulldowns(connect: bool) {
        // Enable/disable 15k pulldown resistors on D+/D- lines
        //
        // In USB Host mode, 15k pulldown resistors must be connected on both D+ and D-.
        // In USB Device mode, pulldown resistors must be disconnected.
        //
        peripherals::LP_SYS::regs()
            .hp_usb_otghs_phy_ctrl()
            .modify(|_, w| {
                w.hp_utmiotg_dppulldown().bit(connect);
                w.hp_utmiotg_dmpulldown().bit(connect);
                w
            });
    }

    #[cfg(esp32p4)]
    fn _p4_init() {
        use crate::RegisterToggle;

        // Enable PHY ref clock (48MHz) for USB UTMI PHY
        peripherals::LP_AON_CLKRST::regs()
            .lp_aonclkrst_hp_usb_clkrst_ctrl1()
            .modify(|_, w| w.lp_aonclkrst_usb_otg20_phyref_clk_en().set_bit());

        // Assert then deassert reset for USB_DWC_HS and USB_UTMI.
        peripherals::LP_AON_CLKRST::regs()
            .lp_aonclkrst_hp_usb_clkrst_ctrl1()
            .toggle(|w, en| {
                w.lp_aonclkrst_rst_en_usb_otg20().bit(en);
                w.lp_aonclkrst_rst_en_usb_otg20_phy().bit(en);
                w
            });

        // Additional setting to solve missing DCONN event on ESP32P4 (IDF-9953).
        //
        // Note: On ESP32P4, the HP_SYSTEM_OTG_SUSPENDM is not connected to 1 by hardware.
        // For correct detection of the device detaching, internal signal should be set to 1 by the
        // software.
        peripherals::HP_SYS::regs()
            .usbotg20_ctrl()
            .modify(|_, w| w.otg_suspendm().set_bit());

        // Parallel LS mode - TODO add to PAC
        const USB_UTMI: usize = 0x5009C000;

        let fc_06 = (USB_UTMI as *mut u32).wrapping_add(6);
        let bits = unsafe { fc_06.read_volatile() };
        unsafe { fc_06.write_volatile(bits | (1 << 3) | (1 << 0)) };
    }

    fn _disable() {
        // TODO
    }
}
