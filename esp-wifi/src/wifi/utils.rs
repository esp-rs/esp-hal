//! Convenience utilities for non-async code

use smoltcp::{
    iface::{Config, Interface},
    wire::{EthernetAddress, HardwareAddress},
};

use super::{WifiApDevice, WifiController, WifiDevice, WifiDeviceMode, WifiError, WifiStaDevice};
use crate::EspWifiController;

// [esp_hal::time::now()] as a smoltcp [`Instant]`
#[cfg(feature = "smoltcp")]
fn timestamp() -> smoltcp::time::Instant {
    smoltcp::time::Instant::from_micros(
        esp_hal::time::now().duration_since_epoch().to_micros() as i64
    )
}

fn setup_iface<Dm: WifiDeviceMode>(device: &mut WifiDevice<'_, Dm>, mode: Dm) -> Interface {
    let mac = mode.mac_address();
    let hw_address = HardwareAddress::Ethernet(EthernetAddress::from_bytes(&mac));

    let config = Config::new(hw_address);
    let iface = Interface::new(config, device, timestamp());
    iface
}

/// Convenient way to create an `smoltcp` ethernet interface
pub fn create_network_interface<'d, Dm: WifiDeviceMode>(
    inited: &'d EspWifiController<'d>,
    device: impl crate::hal::peripheral::Peripheral<P = crate::hal::peripherals::WIFI> + 'd,
    mode: Dm,
) -> Result<(Interface, WifiDevice<'d, Dm>, WifiController<'d>), WifiError> {
    let (mut device, controller) = crate::wifi::new_with_mode(inited, device, mode)?;

    let iface = setup_iface(&mut device, mode);

    Ok((iface, device, controller))
}

pub struct ApStaInterface<'d> {
    pub ap_interface: Interface,
    pub sta_interface: Interface,
    pub ap_device: WifiDevice<'d, WifiApDevice>,
    pub sta_device: WifiDevice<'d, WifiStaDevice>,
    pub controller: WifiController<'d>,
}

pub fn create_ap_sta_network_interface<'d>(
    inited: &'d EspWifiController<'d>,
    device: impl crate::hal::peripheral::Peripheral<P = crate::hal::peripherals::WIFI> + 'd,
) -> Result<ApStaInterface<'d>, WifiError> {
    let (mut ap_device, mut sta_device, controller) = crate::wifi::new_ap_sta(inited, device)?;

    let ap_interface = setup_iface(&mut ap_device, WifiApDevice);
    let sta_interface = setup_iface(&mut sta_device, WifiStaDevice);

    Ok(ApStaInterface {
        ap_interface,
        sta_interface,
        ap_device,
        sta_device,
        controller,
    })
}
