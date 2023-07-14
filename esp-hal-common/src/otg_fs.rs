//! USB OTG full-speed peripheral (TBF examples)
//! 
//! ## Overview
//! The USB OTG Full-speed peripheral driver provides support for the USB On-The-Go (OTG) full-speed
//! functionality on ESP chips, allows communication with USB devices.
//! 
//! The driver uses the `esp_synopsys_usb_otg` crate, which provides the `USB bus` implementation
//! and `USB peripheral traits`. It also relies on other peripheral modules,
//! such as `GPIO`, `system`, and `clock control`, to configure and enable the `USB` peripheral.
//!
//! To use the USB OTG Full-speed peripheral driver, you need to initialize the peripheral and configure
//! its settings. The USB struct represents the USB peripheral and requires the implementation
//! of the `UsbSel`, `UsbDp`, and `UsbDm` traits, which define the specific types
//! used for USB pin selection and data pins.
//!
//! The USB struct provides a `new` function for initialization.
//! Inside the `new` function, the `into_ref!` macro is used to convert the peripheral
//! references into `PeripheralRef` instances.
//!
//! The `USB` struct implements the `UsbPeripheral` trait from the `esp_synopsys_usb_otg` crate, which
//! defines the required constants and functions for `USB peripheral` operation.
//! The trait implementation includes enabling the `USB peripheral`, configuring the `USB` settings
//! and connecting the appropriate `GPIO` pins to the `USB peripheral`.

pub use esp_synopsys_usb_otg::UsbBus;
use esp_synopsys_usb_otg::UsbPeripheral;

use crate::{
    gpio::InputSignal,
    peripheral::{Peripheral, PeripheralRef},
    peripherals,
    system::{Peripheral as PeripheralEnable, PeripheralClockControl},
};

#[doc(hidden)]
pub trait UsbSel {}

#[doc(hidden)]
pub trait UsbDp {}

#[doc(hidden)]
pub trait UsbDm {}

pub struct USB<'d, S, P, M>
where
    S: UsbSel + Send + Sync,
    P: UsbDp + Send + Sync,
    M: UsbDm + Send + Sync,
{
    _usb0: PeripheralRef<'d, peripherals::USB0>,
    _usb_sel: PeripheralRef<'d, S>,
    _usb_dp: PeripheralRef<'d, P>,
    _usb_dm: PeripheralRef<'d, M>,
}

impl<'d, S, P, M> USB<'d, S, P, M>
where
    S: UsbSel + Send + Sync,
    P: UsbDp + Send + Sync,
    M: UsbDm + Send + Sync,
{
    pub fn new(
        usb0: impl Peripheral<P = peripherals::USB0> + 'd,
        usb_sel: impl Peripheral<P = S> + 'd,
        usb_dp: impl Peripheral<P = P> + 'd,
        usb_dm: impl Peripheral<P = M> + 'd,
        peripheral_clock_control: &mut PeripheralClockControl,
    ) -> Self {
        crate::into_ref!(usb_sel, usb_dp, usb_dm);
        peripheral_clock_control.enable(PeripheralEnable::Usb);
        Self {
            _usb0: usb0.into_ref(),
            _usb_sel: usb_sel,
            _usb_dp: usb_dp,
            _usb_dm: usb_dm,
        }
    }
}

unsafe impl<'d, S, P, M> Sync for USB<'d, S, P, M>
where
    S: UsbSel + Send + Sync,
    P: UsbDp + Send + Sync,
    M: UsbDm + Send + Sync,
{
}

unsafe impl<'d, S, P, M> UsbPeripheral for USB<'d, S, P, M>
where
    S: UsbSel + Send + Sync,
    P: UsbDp + Send + Sync,
    M: UsbDm + Send + Sync,
{
    const REGISTERS: *const () = peripherals::USB0::ptr() as *const ();

    const HIGH_SPEED: bool = false;
    const FIFO_DEPTH_WORDS: usize = 256;
    const ENDPOINT_COUNT: usize = 5;

    fn enable() {
        unsafe {
            let usb_wrap = &*peripherals::USB_WRAP::PTR;
            usb_wrap.otg_conf.modify(|_, w| {
                w.usb_pad_enable()
                    .set_bit()
                    .phy_sel()
                    .clear_bit()
                    .clk_en()
                    .set_bit()
                    .ahb_clk_force_on()
                    .set_bit()
                    .phy_clk_force_on()
                    .set_bit()
            });

            #[cfg(esp32s3)]
            {
                let rtc = &*peripherals::RTC_CNTL::PTR;
                rtc.usb_conf
                    .modify(|_, w| w.sw_hw_usb_phy_sel().set_bit().sw_usb_phy_sel().set_bit());
            }

            crate::gpio::connect_high_to_peripheral(InputSignal::USB_OTG_IDDIG); // connected connector is mini-B side
            crate::gpio::connect_high_to_peripheral(InputSignal::USB_SRP_BVALID); // HIGH to force USB device mode
            crate::gpio::connect_high_to_peripheral(InputSignal::USB_OTG_VBUSVALID); // receiving a valid Vbus from device
            crate::gpio::connect_low_to_peripheral(InputSignal::USB_OTG_AVALID);

            usb_wrap.otg_conf.modify(|_, w| {
                w.pad_pull_override()
                    .set_bit()
                    .dp_pullup()
                    .set_bit()
                    .dp_pulldown()
                    .clear_bit()
                    .dm_pullup()
                    .clear_bit()
                    .dm_pulldown()
                    .clear_bit()
            });
        }
    }

    fn ahb_frequency_hz(&self) -> u32 {
        // unused
        80_000_000
    }
}
