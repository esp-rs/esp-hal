use core::cell::RefCell;
use core::fmt::Display;
use embedded_io::blocking::{Read, Write};
use embedded_io::Io;

use embedded_svc::ipv4;
use smoltcp::iface::{Interface, SocketHandle, SocketSet};
use smoltcp::socket::{dhcpv4::Socket as Dhcpv4Socket, tcp::Socket as TcpSocket};
use smoltcp::time::Instant;
use smoltcp::wire::{IpAddress, IpCidr, IpEndpoint, Ipv4Address, DnsQueryType};

use crate::current_millis;
use crate::wifi::{get_ap_mac, get_sta_mac, WifiDevice, WifiMode};

use core::borrow::BorrowMut;

pub struct WifiStack<'a> {
    device: RefCell<WifiDevice<'static>>, // TODO allow non static lifetime
    network_interface: RefCell<Interface>,
    sockets: RefCell<SocketSet<'a>>,
    current_millis_fn: fn() -> u64,
    local_port: RefCell<u16>,
    pub(crate) network_config: RefCell<ipv4::Configuration>,
    pub(crate) ip_info: RefCell<Option<ipv4::IpInfo>>,
    pub(crate) dhcp_socket_handle: RefCell<Option<SocketHandle>>,
    dns_socket_handle: RefCell<Option<SocketHandle>>,
}

impl<'a> WifiStack<'a> {
    pub fn new(
        network_interface: Interface,
        device: WifiDevice<'static>, // TODO relax this lifetime requirement
        mut sockets: SocketSet<'a>,
        current_millis_fn: fn() -> u64,
    ) -> WifiStack<'a> {
        let mut dhcp_socket_handle: Option<SocketHandle> = None;
        let mut dns_socket_handle: Option<SocketHandle> = None;

        for (handle, socket) in sockets.iter_mut() {
            match socket {
                smoltcp::socket::Socket::Dhcpv4(_) => dhcp_socket_handle = Some(handle),
                smoltcp::socket::Socket::Dns(_) => dns_socket_handle = Some(handle),
                _ => {}
            }
        }

        let this = Self {
            device: RefCell::new(device),
            network_interface: RefCell::new(network_interface),
            network_config: RefCell::new(ipv4::Configuration::Client(
                ipv4::ClientConfiguration::DHCP(ipv4::DHCPClientSettings {
                    //FIXME: smoltcp currently doesn't have a way of giving a hostname through DHCP
                    hostname: Some("Espressif".into()),
                }),
            )),
            ip_info: RefCell::new(None),
            dhcp_socket_handle: RefCell::new(dhcp_socket_handle),
            sockets: RefCell::new(sockets),
            current_millis_fn,
            local_port: RefCell::new(41000),
            dns_socket_handle: RefCell::new(dns_socket_handle),
        };

        this.reset();

        this
    }

    pub fn update_iface_configuration(
        &self,
        conf: &ipv4::Configuration,
    ) -> Result<(), WifiStackError> {
        let mut mac = [0u8; 6];
        match self.device.borrow().get_wifi_mode() {
            Ok(WifiMode::Sta) => get_sta_mac(&mut mac),
            Ok(WifiMode::Ap) => get_ap_mac(&mut mac),
            _ => (),
        }
        let hw_address = smoltcp::wire::HardwareAddress::Ethernet(
            smoltcp::wire::EthernetAddress::from_bytes(&mac),
        );
        self.network_interface
            .borrow_mut()
            .set_hardware_addr(hw_address);
        log::info!("Set hardware address: {:02x?}", hw_address);

        self.reset(); // reset IP address

        let mut dhcp_socket_handle_ref = self.dhcp_socket_handle.borrow_mut();
        let mut sockets_ref = self.sockets.borrow_mut();

        if let Some(dhcp_handle) = *dhcp_socket_handle_ref {
            // remove the DHCP client if we use a static IP
            if matches!(
                conf,
                ipv4::Configuration::Client(ipv4::ClientConfiguration::Fixed(_))
            ) {
                sockets_ref.remove(dhcp_handle);
                *dhcp_socket_handle_ref = None;
            }
        }

        // re-add the DHCP client if we use DHCP and it has been removed before
        if matches!(
            conf,
            ipv4::Configuration::Client(ipv4::ClientConfiguration::DHCP(_))
        ) && dhcp_socket_handle_ref.is_none()
        {
            let dhcp_socket = Dhcpv4Socket::new();
            let dhcp_socket_handle = sockets_ref.add(dhcp_socket);
            *dhcp_socket_handle_ref = Some(dhcp_socket_handle);
        }

        if let Some(dhcp_handle) = *dhcp_socket_handle_ref {
            let dhcp_socket = sockets_ref.get_mut::<Dhcpv4Socket>(dhcp_handle);
            log::info!("Reset DHCP client");
            dhcp_socket.reset();
        }

        *self.network_config.borrow_mut() = conf.clone();
        Ok(())
    }

    pub fn reset(&self) {
        log::debug!("Reset TCP stack");

        let dhcp_socket_handle_ref = self.dhcp_socket_handle.borrow_mut();
        if let Some(dhcp_handle) = *dhcp_socket_handle_ref {
            self.with_mut(|_, _, sockets| {
                let dhcp_socket = sockets.get_mut::<Dhcpv4Socket>(dhcp_handle);
                log::debug!("Reset DHCP client");
                dhcp_socket.reset();
            });
        }

        self.with_mut(|interface, _, _| {
            interface.routes_mut().remove_default_ipv4_route();
            interface.update_ip_addrs(|addrs| {
                addrs.clear();
            });

            #[cfg(feature = "ipv6")]
            {
                interface
                    .routes_mut()
                    .add_default_ipv6_route(smoltcp::wire::Ipv6Address::new(
                        0xfe80, 0, 0, 0, 0, 0, 0, 0,
                    ))
                    .unwrap();

                let mut mac = [0u8; 6];
                match interface.hardware_addr() {
                    smoltcp::wire::HardwareAddress::Ethernet(hw_address) => {
                        mac.copy_from_slice(hw_address.as_bytes());
                    }
                }

                let a4 = ((mac[0] ^ 2) as u16) << 8 | mac[1] as u16;
                let a5 = (mac[2] as u16) << 8 | 0xff;
                let a6 = 0xfe << 8 | mac[3] as u16;
                let a7 = (mac[4] as u16) << 8 | mac[5] as u16;

                log::info!(
                    "IPv6 link-local address fe80::{:x}:{:x}:{:x}:{:x}",
                    a4,
                    a5,
                    a6,
                    a7
                );

                interface.update_ip_addrs(|addrs| {
                    addrs
                        .push(IpCidr::new(
                            smoltcp::wire::IpAddress::v6(0xfe80, 0, 0, 0, a4, a5, a6, a7),
                            64,
                        ))
                        .unwrap();
                });
            }
        });
    }

    /// Retrieve all current IP addresses
    pub fn get_ip_addresses(&self, f: impl FnOnce(&[smoltcp::wire::IpCidr])) {
        self.with_mut(|interface, _, _| f(interface.ip_addrs()))
    }

    /// Convenience function to poll the DHCP socket.
    pub fn poll_dhcp(
        &self,
        interface: &mut Interface,
        sockets: &mut SocketSet<'a>,
    ) -> Result<(), WifiStackError> {
        let dhcp_socket_handle_ref = self.dhcp_socket_handle.borrow_mut();
        if let Some(dhcp_handle) = *dhcp_socket_handle_ref {
            let dhcp_socket = sockets.get_mut::<Dhcpv4Socket>(dhcp_handle);
            let event = dhcp_socket.poll();
            if let Some(event) = event {
                match event {
                    smoltcp::socket::dhcpv4::Event::Deconfigured => {
                        *self.ip_info.borrow_mut() = None;
                        interface.routes_mut().remove_default_ipv4_route();
                    }
                    smoltcp::socket::dhcpv4::Event::Configured(config) => {
                        let dns = config.dns_servers.get(0);
                        *self.ip_info.borrow_mut() = Some(ipv4::IpInfo {
                            ip: config.address.address().0.into(),
                            subnet: ipv4::Subnet {
                                gateway: config.router.unwrap().0.into(),
                                mask: ipv4::Mask(config.address.prefix_len()),
                            },
                            dns: dns.map(|x| x.0.into()),
                            secondary_dns: config.dns_servers.get(1).map(|x| x.0.into()),
                        });

                        let address = config.address;
                        interface.borrow_mut().update_ip_addrs(|addrs| {
                            addrs.push(smoltcp::wire::IpCidr::Ipv4(address)).unwrap();
                        });
                        if let Some(route) = config.router {
                            interface
                                .routes_mut()
                                .add_default_ipv4_route(route)
                                .unwrap();
                        }

                        if let (Some(&dns), Some(dns_handle)) =
                            (dns, *self.dns_socket_handle.borrow()) {

                            sockets.get_mut::<smoltcp::socket::dns::Socket>(dns_handle)
                                .update_servers(&[dns.into()]);
                        }
                    }
                }
            }
        }

        Ok(())
    }

    pub fn get_socket<'s>(
        &'s self,
        rx_buffer: &'a mut [u8],
        tx_buffer: &'a mut [u8],
    ) -> Socket<'s, 'a>
    where
        'a: 's,
    {
        let socket = TcpSocket::new(
            smoltcp::socket::tcp::SocketBuffer::new(rx_buffer),
            smoltcp::socket::tcp::SocketBuffer::new(tx_buffer),
        );

        let socket_handle =
            self.with_mut(|_interface, _device, sockets| sockets.borrow_mut().add(socket));

        Socket {
            socket_handle,
            network: self,
        }
    }

    pub fn get_udp_socket<'s>(
        &'s self,
        rx_meta: &'a mut [smoltcp::socket::udp::PacketMetadata],
        rx_buffer: &'a mut [u8],
        tx_meta: &'a mut [smoltcp::socket::udp::PacketMetadata],
        tx_buffer: &'a mut [u8],
    ) -> UdpSocket<'s, 'a>
    where
        'a: 's,
    {
        let socket = smoltcp::socket::udp::Socket::new(
            smoltcp::socket::udp::PacketBuffer::new(rx_meta, rx_buffer),
            smoltcp::socket::udp::PacketBuffer::new(tx_meta, tx_buffer),
        );

        let socket_handle =
            self.with_mut(|_interface, _device, sockets| sockets.borrow_mut().add(socket));

        UdpSocket {
            socket_handle,
            network: self,
        }
    }

    pub fn is_dns_configured(&self) -> bool {
        self.dns_socket_handle.borrow().is_some()
    }

    pub fn configure_dns(
        &'a self,
        servers: &[IpAddress],
        query_storage: &'a mut [Option<smoltcp::socket::dns::DnsQuery>]
    ) {
        if let Some(old_handle) = self.dns_socket_handle.take() {
            self.with_mut(|_interface, _device, sockets| sockets.remove(old_handle));
            // the returned socket get dropped and frees a slot for the new one
        }

        let dns = smoltcp::socket::dns::Socket::new(servers, query_storage);
        let handle = self.with_mut(|_interface, _device, sockets| sockets.add(dns));
        self.dns_socket_handle.replace(Some(handle));
    }

    pub fn update_dns_servers(&self, servers: &[IpAddress]) {
        if let Some(dns_handle) = *self.dns_socket_handle.borrow_mut() {
            self.with_mut(|_interface, _device, sockets| {
                sockets.get_mut::<smoltcp::socket::dns::Socket>(dns_handle)
                    .update_servers(servers);
            });
        }
    }

    pub fn dns_query(
        &self,
        name: &str,
        query_type: DnsQueryType,
    ) -> Result<heapless::Vec<IpAddress, 1>, WifiStackError> {
        use smoltcp::socket::dns;

        match query_type { // check if name is already an IP
            DnsQueryType::A => {
                if let Ok(ip) = name.parse::<Ipv4Address>() {
                    return Ok([ip.into()].into_iter().collect());
                }
            }
            #[cfg(feature = "ipv6")]
            DnsQueryType::Aaaa => {
                if let Ok(ip) = name.parse::<smoltcp::wire::Ipv6Address>() {
                    return Ok([ip.into()].into_iter().collect());
                }
            }
            _ => {}
        }

        let Some(dns_handle) = *self.dns_socket_handle.borrow() else {
            return Err(WifiStackError::DnsNotConfigured);
        };

        let query = self.with_mut(|interface, _device, sockets| {
            sockets.get_mut::<dns::Socket>(dns_handle)
                .start_query(interface.context(), name, query_type)
                .map_err(|e| WifiStackError::DnsQueryError(e))
        })?;

        loop {
            self.work();

            let result = self.with_mut(|_interface, _device, sockets| {
                sockets.get_mut::<dns::Socket>(dns_handle).get_query_result(query)
            });

            match result {
                Ok(addrs) => return Ok(addrs), // query finished
                Err(dns::GetQueryResultError::Pending) => {}, // query not finished
                Err(_) => return Err(WifiStackError::DnsQueryFailed)
            }
        }
    }

    pub fn work(&self) {
        loop {
            if let false = self.with_mut(|interface, device, sockets| {
                let network_config = self.network_config.borrow().clone();
                if let ipv4::Configuration::Client(ipv4::ClientConfiguration::DHCP(_)) =
                    network_config
                {
                    self.poll_dhcp(interface, sockets).ok();
                } else if let ipv4::Configuration::Client(ipv4::ClientConfiguration::Fixed(
                    settings,
                )) = network_config
                {
                    let addr = Ipv4Address::from_bytes(&settings.ip.octets());
                    if !interface.has_ip_addr(addr) {
                        let gateway = Ipv4Address::from_bytes(&settings.subnet.gateway.octets());
                        interface.routes_mut().add_default_ipv4_route(gateway).ok();
                        interface.update_ip_addrs(|addrs| {
                            addrs
                                .push(IpCidr::new(addr.into(), settings.subnet.mask.0))
                                .unwrap();
                        });
                    }
                }
                interface.poll(
                    Instant::from_millis((self.current_millis_fn)() as i64),
                    device,
                    sockets,
                )
            }) {
                break;
            }
        }
    }

    fn next_local_port(&self) -> u16 {
        let mut local_port = self.local_port.borrow_mut();
        *local_port += 1;
        if *local_port == 65535 {
            *local_port = 41000;
        }
        *local_port
    }

    #[allow(unused)]
    fn with<R>(&self, f: impl FnOnce(&Interface, &WifiDevice, &SocketSet<'a>) -> R) -> R {
        f(
            &self.network_interface.borrow(),
            &self.device.borrow(),
            &self.sockets.borrow(),
        )
    }

    fn with_mut<R>(
        &self,
        f: impl FnOnce(&mut Interface, &mut WifiDevice, &mut SocketSet<'a>) -> R,
    ) -> R {
        f(
            &mut self.network_interface.borrow_mut(),
            &mut self.device.borrow_mut(),
            &mut self.sockets.borrow_mut(),
        )
    }
}

#[derive(Debug, Copy, Clone)]
pub enum WifiStackError {
    Unknown(i32),
    InitializationError(crate::InitializationError),
    DeviceError(crate::wifi::WifiError),
    MissingIp,
    DnsNotConfigured,
    DnsQueryError(smoltcp::socket::dns::StartQueryError),
    DnsQueryFailed
}

impl Display for WifiStackError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{:?}", self)
    }
}

pub fn timestamp() -> Instant {
    Instant::from_millis(current_millis() as i64)
}

impl<'a> ipv4::Interface for WifiStack<'a> {
    type Error = WifiStackError;

    fn get_iface_configuration(&self) -> Result<ipv4::Configuration, Self::Error> {
        Ok(self.network_config.borrow().clone())
    }

    fn set_iface_configuration(&mut self, conf: &ipv4::Configuration) -> Result<(), Self::Error> {
        self.update_iface_configuration(conf)
    }

    fn is_iface_up(&self) -> bool {
        self.ip_info.borrow().is_some()
    }

    fn get_ip_info(&self) -> Result<ipv4::IpInfo, Self::Error> {
        self.ip_info.borrow().ok_or(WifiStackError::MissingIp)
    }
}

pub struct Socket<'s, 'n: 's> {
    socket_handle: SocketHandle,
    network: &'s WifiStack<'n>,
}

impl<'s, 'n: 's> Socket<'s, 'n> {
    pub fn open<'i>(&'i mut self, addr: IpAddress, port: u16) -> Result<(), IoError>
    where
        's: 'i,
    {
        {
            let res = self.network.with_mut(|interface, _device, sockets| {
                let sock = sockets.get_mut::<TcpSocket>(self.socket_handle);
                let cx = interface.context();
                let remote_endpoint = (addr, port);
                sock.set_ack_delay(Some(smoltcp::time::Duration::from_millis(100)));
                sock.connect(cx, remote_endpoint, self.network.next_local_port())
            });

            res.map_err(|e| IoError::ConnectError(e))?;
        }

        loop {
            let can_send = self.network.with_mut(|_interface, _device, sockets| {
                let sock = sockets.get_mut::<TcpSocket>(self.socket_handle);
                if sock.can_send() {
                    true
                } else {
                    false
                }
            });

            if can_send {
                break;
            }

            self.work();
        }

        Ok(())
    }

    pub fn listen<'i>(&'i mut self, port: u16) -> Result<(), IoError>
    where
        's: 'i,
    {
        {
            let res = self.network.with_mut(|_interface, _device, sockets| {
                let sock = sockets.get_mut::<TcpSocket>(self.socket_handle);
                sock.listen(port)
            });

            res.map_err(|e| IoError::ListenError(e))?;
        }

        loop {
            let can_send = self.network.with_mut(|_interface, _device, sockets| {
                let sock = sockets.get_mut::<TcpSocket>(self.socket_handle);
                if sock.can_send() {
                    true
                } else {
                    false
                }
            });

            if can_send {
                break;
            }

            self.work();
        }

        Ok(())
    }

    pub fn listen_unblocking<'i>(&'i mut self, port: u16) -> Result<(), IoError>
    where
        's: 'i,
    {
        {
            let res = self.network.with_mut(|_interface, _device, sockets| {
                let sock = sockets.get_mut::<TcpSocket>(self.socket_handle);
                sock.listen(port)
            });

            res.map_err(|e| IoError::ListenError(e))?;
        }

        self.work();
        Ok(())
    }

    pub fn close(&mut self) {
        self.network.with_mut(|_interface, _device, sockets| {
            sockets.get_mut::<TcpSocket>(self.socket_handle).close();
        });

        self.work();
    }

    pub fn disconnect(&mut self) {
        self.network.with_mut(|_interface, _device, sockets| {
            sockets.get_mut::<TcpSocket>(self.socket_handle).abort();
        });

        self.work();
    }

    pub fn is_open(&mut self) -> bool {
        self.network.with_mut(|_interface, _device, sockets| {
            sockets.get_mut::<TcpSocket>(self.socket_handle).is_open()
        })
    }

    pub fn is_connected(&mut self) -> bool {
        self.network.with_mut(|_interface, _device, sockets| {
            let socket = sockets.get_mut::<TcpSocket>(self.socket_handle);

            socket.may_recv() && socket.may_send()
        })
    }

    pub fn work(&mut self) {
        self.network.work()
    }
}

impl<'s, 'n: 's> Drop for Socket<'s, 'n> {
    fn drop(&mut self) {
        self.network
            .with_mut(|_interface, _device, sockets| sockets.remove(self.socket_handle));
    }
}

#[derive(Debug)]
pub enum IoError {
    SocketClosed,
    MultiCastError(smoltcp::iface::MulticastError),
    TcpRecvError(smoltcp::socket::tcp::RecvError),
    UdpRecvError(smoltcp::socket::udp::RecvError),
    TcpSendError(smoltcp::socket::tcp::SendError),
    UdpSendError(smoltcp::socket::udp::SendError),
    ConnectError(smoltcp::socket::tcp::ConnectError),
    BindError(smoltcp::socket::udp::BindError),
    ListenError(smoltcp::socket::tcp::ListenError),
}

impl embedded_io::Error for IoError {
    fn kind(&self) -> embedded_io::ErrorKind {
        embedded_io::ErrorKind::Other
    }
}

impl<'s, 'n: 's> Io for Socket<'s, 'n> {
    type Error = IoError;
}

impl<'s, 'n: 's> Read for Socket<'s, 'n> {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        loop {
            self.network.with_mut(|interface, device, sockets| {
                interface.poll(
                    Instant::from_millis((self.network.current_millis_fn)() as i64),
                    device,
                    sockets,
                )
            });

            let (may_recv, is_open, can_recv) =
                self.network.with_mut(|_interface, _device, sockets| {
                    let socket = sockets.get_mut::<TcpSocket>(self.socket_handle);

                    (socket.may_recv(), socket.is_open(), socket.can_recv())
                });
            if may_recv {
                break;
            }

            if !is_open {
                return Err(IoError::SocketClosed);
            }

            if !can_recv {
                return Err(IoError::SocketClosed);
            }
        }

        loop {
            let res = self.network.with_mut(|interface, device, sockets| {
                interface.poll(
                    Instant::from_millis((self.network.current_millis_fn)() as i64),
                    device,
                    sockets,
                )
            });

            if let false = res {
                break;
            }
        }

        self.network.with_mut(|_interface, _device, sockets| {
            let socket = sockets.get_mut::<TcpSocket>(self.socket_handle);

            socket.recv_slice(buf).map_err(|e| IoError::TcpRecvError(e))
        })
    }
}

impl<'s, 'n: 's> Write for Socket<'s, 'n> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        loop {
            let (may_send, is_open, can_send) =
                self.network.with_mut(|interface, device, sockets| {
                    interface.poll(
                        Instant::from_millis((self.network.current_millis_fn)() as i64),
                        device,
                        sockets,
                    );

                    let socket = sockets.get_mut::<TcpSocket>(self.socket_handle);

                    (socket.may_send(), socket.is_open(), socket.can_send())
                });

            if may_send {
                break;
            }

            if !is_open || !can_send {
                return Err(IoError::SocketClosed);
            }
        }

        let mut written = 0;
        loop {
            self.flush()?;

            self.network.with_mut(|_interface, _device, sockets| {
                sockets
                    .get_mut::<TcpSocket>(self.socket_handle)
                    .send_slice(&buf[written..])
                    .map(|len| written += len)
                    .map_err(IoError::TcpSendError)
            })?;

            if written >= buf.len() {
                break;
            }
        }

        Ok(written)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        loop {
            let res = self.network.with_mut(|interface, device, sockets| {
                interface.poll(
                    Instant::from_millis((self.network.current_millis_fn)() as i64),
                    device,
                    sockets,
                )
            });

            if let false = res {
                break;
            }
        }

        Ok(())
    }
}

pub struct UdpSocket<'s, 'n: 's> {
    socket_handle: SocketHandle,
    network: &'s WifiStack<'n>,
}

impl<'s, 'n: 's> UdpSocket<'s, 'n> {
    pub fn bind<'i>(&'i mut self, port: u16) -> Result<(), IoError>
    where
        's: 'i,
    {
        self.work();

        {
            let res = self.network.with_mut(|_interface, _device, sockets| {
                let sock = sockets.get_mut::<smoltcp::socket::udp::Socket>(self.socket_handle);
                sock.bind(port)
            });

            if let Err(err) = res {
                return Err(IoError::BindError(err));
            }
        }

        loop {
            let can_send = self.network.with_mut(|_interface, _device, sockets| {
                let sock = sockets.get_mut::<smoltcp::socket::udp::Socket>(self.socket_handle);
                if sock.can_send() {
                    true
                } else {
                    false
                }
            });

            if can_send {
                break;
            }

            self.work();
        }

        Ok(())
    }

    pub fn close(&mut self) {
        self.network.with_mut(|_interface, _device, sockets| {
            sockets
                .get_mut::<smoltcp::socket::udp::Socket>(self.socket_handle)
                .close();
        });

        self.work();
    }

    pub fn send(&mut self, addr: IpAddress, port: u16, data: &[u8]) -> Result<(), IoError> {
        loop {
            self.work();

            let (can_send, packet_capacity, payload_capacity) =
                self.network.with_mut(|_interface, _device, sockets| {
                    let sock = sockets.get_mut::<smoltcp::socket::udp::Socket>(self.socket_handle);
                    (
                        sock.can_send(),
                        sock.packet_send_capacity(),
                        sock.payload_send_capacity(),
                    )
                });

            if can_send && packet_capacity > 0 && payload_capacity > data.len() {
                break;
            }
        }

        self.network
            .with_mut(|_interface, _device, sockets| {
                let endpoint = (addr, port);
                let endpoint: IpEndpoint = endpoint.into();

                sockets
                    .get_mut::<smoltcp::socket::udp::Socket>(self.socket_handle)
                    .send_slice(data, endpoint)
            })
            .map_err(|e| IoError::UdpSendError(e))?;

        self.work();

        Ok(())
    }

    pub fn receive(&mut self, data: &mut [u8]) -> Result<(usize, IpAddress, u16), IoError> {
        self.work();

        let res = self.network.with_mut(|_interface, _device, sockets| {
            sockets
                .get_mut::<smoltcp::socket::udp::Socket>(self.socket_handle)
                .recv_slice(data)
        });

        match res {
            Ok((len, endpoint)) => {
                let addr = endpoint.endpoint.addr;
                Ok((len, addr, endpoint.endpoint.port))
            }
            Err(e) => Err(IoError::UdpRecvError(e)),
        }
    }

    pub fn join_multicast_group(&mut self, addr: IpAddress) -> Result<bool, IoError> {
        self.work();

        let res = self.network.with_mut(|interface, device, _| {
            interface.join_multicast_group(
                device,
                addr,
                Instant::from_millis((self.network.current_millis_fn)() as i64),
            )
        });

        self.work();

        res.map_err(|e| IoError::MultiCastError(e))
    }

    pub fn work(&mut self) {
        self.network.work()
    }
}

impl<'s, 'n: 's> Drop for UdpSocket<'s, 'n> {
    fn drop(&mut self) {
        self.network
            .with_mut(|_, _, sockets| sockets.borrow_mut().remove(self.socket_handle));
    }
}
