use smoltcp::{
    iface::{Interface, InterfaceBuilder, Neighbor, NeighborCache, Route, Routes, SocketStorage},
    socket::Dhcpv4Socket,
    wire::{EthernetAddress, IpAddress, IpCidr, Ipv4Address},
};

use crate::wifi::get_sta_mac;

use super::WifiDevice;

#[macro_export]
macro_rules! create_network_stack_storage {
    ($socket_count:literal , $cache_count:literal , $routes_count:literal, $multicast_storage_size:literal) => {{
        use smoltcp::iface::{Neighbor, NeighborCache, Route, SocketStorage};
        use smoltcp::wire::{IpAddress, IpCidr, Ipv4Address};

        let mut socket_set_entries: [SocketStorage; $socket_count] = Default::default();
        let mut neighbor_cache_storage: [Option<(IpAddress, Neighbor)>; $cache_count] =
            Default::default();
        let mut routes_storage: [Option<(IpCidr, Route)>; $routes_count] = Default::default();
        let ip_addr = IpCidr::new(Ipv4Address::UNSPECIFIED.into(), 0);
        let mut ip_addrs = [ip_addr];
        let mut ipv4_multicast_storage = [None; $multicast_storage_size];

        (
            socket_set_entries,
            neighbor_cache_storage,
            routes_storage,
            ip_addrs,
            ipv4_multicast_storage,
        )
    }};
}

#[macro_export]
macro_rules! network_stack_storage {
    ($param:ident) => {{
        (
            &mut $param.0,
            &mut $param.1,
            &mut $param.2,
            &mut $param.3,
            &mut $param.4,
        )
    }};
}

/// Convenient way to create an `smoltcp` ethernet interface
/// You can use the provided macros to create and pass a suitable backing storage.
pub fn create_network_interface<'a>(
    storage: (
        &'a mut [SocketStorage<'a>],
        &'a mut [Option<(IpAddress, Neighbor)>],
        &'a mut [Option<(IpCidr, Route)>],
        &'a mut [IpCidr; 1],
        &'a mut [Option<(Ipv4Address, ())>],
    ),
) -> Interface<WifiDevice> {
    let socket_set_entries = storage.0;
    let neighbor_cache_storage = storage.1;
    let routes_storage = storage.2;
    let ip_addrs = storage.3;
    let ipv4_multicast_groups = storage.4;

    let mut mac = [0u8; 6];
    get_sta_mac(&mut mac);
    let hw_address = EthernetAddress::from_bytes(&mac);

    let device = WifiDevice::new();

    let neighbor_cache = NeighborCache::new(&mut neighbor_cache_storage[..]);
    let routes = Routes::new(&mut routes_storage[..]);

    let mut ethernet = InterfaceBuilder::new(device, socket_set_entries)
        .hardware_addr(smoltcp::wire::HardwareAddress::Ethernet(hw_address))
        .neighbor_cache(neighbor_cache)
        .ip_addrs(&mut ip_addrs[..])
        .ipv4_multicast_groups(ipv4_multicast_groups)
        .routes(routes)
        .finalize();

    let dhcp_socket = Dhcpv4Socket::new();
    ethernet.add_socket(dhcp_socket);

    ethernet
}
