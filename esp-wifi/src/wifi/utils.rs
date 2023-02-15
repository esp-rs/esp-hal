use smoltcp::{
    iface::{Config, Interface, SocketSet, SocketStorage},
    phy::{Device, Medium},
    socket::dhcpv4::Socket as Dhcpv4Socket,
    wire::EthernetAddress,
};

use crate::wifi::get_sta_mac;

use super::{WifiController, WifiDevice};

/// Convenient way to create an `smoltcp` ethernet interface
/// You can use the provided macros to create and pass a suitable backing storage.
pub fn create_network_interface<'a>(
    storage: &'a mut [SocketStorage<'a>],
) -> (Interface, WifiDevice, WifiController, SocketSet<'a>) {
    let socket_set_entries = storage;

    let mut mac = [0u8; 6];
    get_sta_mac(&mut mac);
    let hw_address = EthernetAddress::from_bytes(&mac);

    let (mut device, controller) = crate::wifi::new();

    let mut config = Config::new();

    if device.capabilities().medium == Medium::Ethernet {
        config.hardware_addr = Some(hw_address.into());
    }

    let iface = Interface::new(config, &mut device);

    let mut socket_set = SocketSet::new(socket_set_entries);

    let dhcp_socket = Dhcpv4Socket::new();
    socket_set.add(dhcp_socket);

    (iface, device, controller, socket_set)
}
