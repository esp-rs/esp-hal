//! Convenience utilities for non-async code

#[cfg(feature = "dhcpv4")]
use smoltcp::socket::dhcpv4::Socket as Dhcpv4Socket;
use smoltcp::{
    iface::{Config, Interface, SocketSet, SocketStorage},
    wire::{EthernetAddress, HardwareAddress},
};

use super::{WifiApDevice, WifiController, WifiDevice, WifiDeviceMode, WifiError, WifiStaDevice};
use crate::{timestamp, EspWifiInitialization};

fn setup_iface<'a, MODE: WifiDeviceMode>(
    device: &mut WifiDevice<'_, MODE>,
    mode: MODE,
    storage: &'a mut [SocketStorage<'a>],
) -> (Interface, SocketSet<'a>) {
    let mac = mode.mac_address();
    let hw_address = HardwareAddress::Ethernet(EthernetAddress::from_bytes(&mac));

    let config = Config::new(hw_address);
    let iface = Interface::new(config, device, timestamp());

    #[allow(unused_mut)]
    let mut socket_set = SocketSet::new(storage);

    #[cfg(feature = "dhcpv4")]
    if mode.mode().is_sta() {
        // only add DHCP client in STA mode
        let dhcp_socket = Dhcpv4Socket::new();
        socket_set.add(dhcp_socket);
    }

    (iface, socket_set)
}

/// Convenient way to create an `smoltcp` ethernet interface
/// You can use the provided macros to create and pass a suitable backing
/// storage.
pub fn create_network_interface<'a, 'd, MODE: WifiDeviceMode>(
    inited: &EspWifiInitialization,
    device: impl crate::hal::peripheral::Peripheral<P = crate::hal::peripherals::WIFI> + 'd,
    mode: MODE,
    storage: &'a mut [SocketStorage<'a>],
) -> Result<
    (
        Interface,
        WifiDevice<'d, MODE>,
        WifiController<'d>,
        SocketSet<'a>,
    ),
    WifiError,
> {
    let (mut device, controller) = crate::wifi::new_with_mode(inited, device, mode)?;

    let (iface, socket_set) = setup_iface(&mut device, mode, storage);

    Ok((iface, device, controller, socket_set))
}

pub struct ApStaInterface<'a, 'd> {
    pub ap_interface: Interface,
    pub sta_interface: Interface,
    pub ap_device: WifiDevice<'d, WifiApDevice>,
    pub sta_device: WifiDevice<'d, WifiStaDevice>,
    pub controller: WifiController<'d>,
    pub ap_socket_set: SocketSet<'a>,
    pub sta_socket_set: SocketSet<'a>,
}

pub fn create_ap_sta_network_interface<'a, 'd>(
    inited: &EspWifiInitialization,
    device: impl crate::hal::peripheral::Peripheral<P = crate::hal::peripherals::WIFI> + 'd,
    ap_storage: &'a mut [SocketStorage<'a>],
    sta_storage: &'a mut [SocketStorage<'a>],
) -> Result<ApStaInterface<'a, 'd>, WifiError> {
    let (mut ap_device, mut sta_device, controller) = crate::wifi::new_ap_sta(inited, device)?;

    let (ap_interface, ap_socket_set) = setup_iface(&mut ap_device, WifiApDevice, ap_storage);
    let (sta_interface, sta_socket_set) = setup_iface(&mut sta_device, WifiStaDevice, sta_storage);

    Ok(ApStaInterface {
        ap_interface,
        sta_interface,
        ap_device,
        sta_device,
        controller,
        ap_socket_set,
        sta_socket_set,
    })
}
