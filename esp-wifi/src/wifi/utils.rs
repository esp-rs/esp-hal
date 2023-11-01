use smoltcp::{
    iface::{Config, Interface, SocketSet, SocketStorage},
    socket::dhcpv4::Socket as Dhcpv4Socket,
    time::Instant,
    wire::{EthernetAddress, HardwareAddress},
};

use crate::{current_millis, wifi::get_sta_mac};
use crate::{wifi::get_ap_mac, EspWifiInitialization};

use super::{WifiController, WifiDevice, WifiError, WifiMode};

/// Convenient way to create an `smoltcp` ethernet interface
/// You can use the provided macros to create and pass a suitable backing storage.
pub fn create_network_interface<'a, 'd>(
    inited: &EspWifiInitialization,
    device: impl crate::hal::peripheral::Peripheral<P = crate::hal::peripherals::WIFI> + 'd,
    mode: WifiMode,
    storage: &'a mut [SocketStorage<'a>],
) -> Result<(Interface, WifiDevice<'d>, WifiController<'d>, SocketSet<'a>), WifiError> {
    let socket_set_entries = storage;

    let mut mac = [0u8; 6];
    match mode.is_ap() {
        true => get_ap_mac(&mut mac),
        false => get_sta_mac(&mut mac),
    }
    let hw_address = HardwareAddress::Ethernet(EthernetAddress::from_bytes(&mac));

    let (mut device, controller) = crate::wifi::new_with_mode(inited, device, mode)?;

    let config = Config::new(hw_address);
    let iface = Interface::new(
        config,
        &mut device,
        Instant::from_millis(current_millis() as i64),
    );

    let mut socket_set = SocketSet::new(socket_set_entries);

    if !mode.is_ap() {
        // only add DHCP client in STA mode
        let dhcp_socket = Dhcpv4Socket::new();
        socket_set.add(dhcp_socket);
    }

    Ok((iface, device, controller, socket_set))
}
