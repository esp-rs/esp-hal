use core::cell::RefCell;
use core::fmt::Display;
use embedded_io::blocking::{Read, Write};
use embedded_io::Io;

use embedded_svc::ipv4;
use embedded_svc::wifi::AccessPointInfo;
use enumset::EnumSet;
use smoltcp::iface::{Interface, SocketHandle};
use smoltcp::socket::{Dhcpv4Socket, TcpSocket};
use smoltcp::storage::PacketMetadata;
use smoltcp::time::Instant;
use smoltcp::wire::{IpAddress, IpCidr, IpEndpoint, Ipv4Address};

use crate::current_millis;
use crate::wifi::WifiDevice;

/// An implementation of `embedded-svc`'s wifi trait.
pub struct Wifi<'a> {
    network_interface: Interface<'a, WifiDevice>,
    pub(crate) network_config: ipv4::Configuration,
    pub(crate) ip_info: Option<ipv4::IpInfo>,
    dhcp_socket_handle: Option<SocketHandle>,
}

impl<'a> Wifi<'a> {
    /// Create a new instance from a `NetworkStack`
    pub fn new(mut network_interface: Interface<'a, WifiDevice>) -> Wifi<'a> {
        let mut dhcp_socket_handle: Option<SocketHandle> = None;

        for (handle, socket) in network_interface.sockets_mut() {
            match socket {
                smoltcp::socket::Socket::Dhcpv4(_) => dhcp_socket_handle = Some(handle),
                _ => {}
            }
        }

        Wifi {
            network_interface,
            network_config: ipv4::Configuration::Client(ipv4::ClientConfiguration::DHCP(
                ipv4::DHCPClientSettings {
                    //FIXME: smoltcp currently doesn't have a way of giving a hostname through DHCP
                    hostname: Some("Espressif".into()),
                },
            )),
            ip_info: None,
            dhcp_socket_handle,
        }
    }

    /// Get a mutable reference to the `NetworkStack`
    pub fn network_interface(&mut self) -> &mut Interface<'a, WifiDevice> {
        &mut self.network_interface
    }

    /// Convenience function to poll the DHCP socket.
    pub fn poll_dhcp(&mut self) -> Result<(), WifiStackError> {
        if let Some(dhcp_handle) = self.dhcp_socket_handle {
            let dhcp_socket = self
                .network_interface
                .get_socket::<Dhcpv4Socket>(dhcp_handle);
            let event = dhcp_socket.poll();
            if let Some(event) = event {
                match event {
                    smoltcp::socket::Dhcpv4Event::Deconfigured => {
                        self.ip_info = None;
                        self.network_interface
                            .routes_mut()
                            .remove_default_ipv4_route();
                    }
                    smoltcp::socket::Dhcpv4Event::Configured(config) => {
                        self.ip_info = Some(ipv4::IpInfo {
                            ip: config.address.address().0.into(),
                            subnet: ipv4::Subnet {
                                gateway: config.router.unwrap().0.into(),
                                mask: ipv4::Mask(config.address.prefix_len()),
                            },
                            dns: config
                                .dns_servers
                                .get(0)
                                .map(|x| x.as_ref())
                                .flatten()
                                .map(|x| x.0.into()),
                            secondary_dns: config
                                .dns_servers
                                .get(1)
                                .map(|x| x.as_ref())
                                .flatten()
                                .map(|x| x.0.into()),
                        });

                        let address = config.address;
                        self.network_interface.update_ip_addrs(|addrs| {
                            let addr = addrs
                                .iter_mut()
                                .filter(|cidr| match cidr.address() {
                                    IpAddress::Ipv4(_) => true,
                                    _ => false,
                                })
                                .next()
                                .expect("No address");

                            *addr = IpCidr::Ipv4(address);
                        });
                        if let Some(route) = config.router {
                            self.network_interface
                                .routes_mut()
                                .add_default_ipv4_route(route)?;
                        }
                    }
                }
            }
        }

        Ok(())
    }
}

#[derive(Debug, Copy, Clone)]
pub enum WifiStackError {
    Unknown(i32),
    SmolTcpError(smoltcp::Error),
    InitializationError(crate::InitializationError),
    DeviceError(crate::wifi::WifiError),
    MissingIp,
}

impl From<smoltcp::Error> for WifiStackError {
    fn from(error: smoltcp::Error) -> Self {
        WifiStackError::SmolTcpError(error)
    }
}

impl Display for WifiStackError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{:?}", self)
    }
}

impl embedded_svc::wifi::Wifi for Wifi<'_> {
    type Error = crate::wifi::WifiError;

    fn get_capabilities(&self) -> Result<EnumSet<embedded_svc::wifi::Capability>, Self::Error> {
        self.network_interface.device().get_capabilities()
    }

    fn get_configuration(&self) -> Result<embedded_svc::wifi::Configuration, Self::Error> {
        self.network_interface.device().get_configuration()
    }

    fn set_configuration(&mut self, conf: &embedded_svc::wifi::Configuration) -> Result<(), Self::Error> {
        self.network_interface.device_mut().set_configuration(conf)
    }

    fn start(&mut self) -> Result<(), Self::Error> {
        self.network_interface.device_mut().start()
    }

    fn stop(&mut self) -> Result<(), Self::Error> {
        self.network_interface.device_mut().stop()
    }

    fn connect(&mut self) -> Result<(), Self::Error> {
        self.network_interface.device_mut().connect()
    }

    fn disconnect(&mut self) -> Result<(), Self::Error> {
        self.network_interface.device_mut().disconnect()
    }

    fn is_started(&self) -> Result<bool, Self::Error> {
        self.network_interface.device().is_started()
    }

    fn is_connected(&self) -> Result<bool, Self::Error> {
        self.network_interface.device().is_connected()
    }

    fn scan_n<const N: usize>(
        &mut self,
    ) -> Result<(heapless::Vec<AccessPointInfo, N>, usize), Self::Error> {
        self.network_interface.device_mut().scan_n::<N>()
    }
}

pub fn timestamp() -> Instant {
    Instant::from_millis(current_millis() as i64)
}

// Following code is not well tested, yet.
// It's currently more or less just here for the DHCP example.
// Might get replaced or improved in future.

pub struct Network<'a> {
    interface: RefCell<crate::wifi_interface::Wifi<'a>>,
    current_millis_fn: fn() -> u64,
    local_port: RefCell<u16>,
}

impl<'a> Network<'a> {
    pub fn new(
        interface: crate::wifi_interface::Wifi<'a>,
        current_millis_fn: fn() -> u64,
    ) -> Network {
        Self {
            interface: RefCell::new(interface),
            current_millis_fn,
            local_port: RefCell::new(41000),
        }
    }

    fn with_interface<F, R>(&self, f: F) -> R
    where
        F: FnOnce(&mut crate::wifi_interface::Wifi<'a>) -> R,
    {
        let mut interface = self.interface.borrow_mut();
        f(&mut interface)
    }

    pub fn poll_dhcp(&self) -> Result<(), WifiStackError> {
        self.with_interface(|i| i.poll_dhcp())
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
            smoltcp::socket::TcpSocketBuffer::new(rx_buffer),
            smoltcp::socket::TcpSocketBuffer::new(tx_buffer),
        );

        let socket_handle =
            self.with_interface(|interface| interface.network_interface().add_socket(socket));

        Socket {
            socket_handle,
            network: self,
        }
    }

    pub fn get_udp_socket<'s>(
        &'s self,
        rx_meta: &'a mut [PacketMetadata<IpEndpoint>],
        rx_buffer: &'a mut [u8],
        tx_meta: &'a mut [PacketMetadata<IpEndpoint>],
        tx_buffer: &'a mut [u8],
    ) -> UdpSocket<'s, 'a>
    where
        'a: 's,
    {
        let socket = smoltcp::socket::UdpSocket::new(
            smoltcp::socket::UdpSocketBuffer::new(rx_meta, rx_buffer),
            smoltcp::socket::UdpSocketBuffer::new(tx_meta, tx_buffer),
        );

        let socket_handle =
            self.with_interface(|interface| interface.network_interface().add_socket(socket));

        UdpSocket {
            socket_handle,
            network: self,
        }
    }

    pub fn work(&self) {
        loop {
            self.with_interface(|interface| interface.poll_dhcp().ok());
            if let Ok(false) = self.with_interface(|interface| {
                interface
                    .network_interface()
                    .poll(Instant::from_millis((self.current_millis_fn)() as i64))
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
}

impl<'a> ipv4::Interface for Network<'a> {
    type Error = WifiStackError;

    fn get_iface_configuration(&self) -> Result<ipv4::Configuration, Self::Error> {
        Ok(self.interface.borrow().network_config.clone())
    }

    fn set_iface_configuration(&mut self, conf: &ipv4::Configuration) -> Result<(), Self::Error> {
        self.with_interface(|x| x.network_config = conf.clone());
        Ok(())
    }

    fn is_iface_up(&self) -> bool {
        self.interface.borrow().ip_info.is_some()
    }

    fn get_ip_info(&self) -> Result<ipv4::IpInfo, Self::Error> {
        self.interface.borrow().ip_info.ok_or(WifiStackError::MissingIp)
    }
}

pub struct Socket<'s, 'n: 's> {
    socket_handle: SocketHandle,
    network: &'s Network<'n>,
}

impl<'s, 'n: 's> Socket<'s, 'n> {
    pub fn open<'i>(&'i mut self, addr: Ipv4Address, port: u16) -> Result<(), IoError>
    where
        's: 'i,
    {
        {
            let res = self.network.with_interface(|interface| {
                let (sock, cx) = interface
                    .network_interface()
                    .get_socket_and_context::<TcpSocket>(self.socket_handle);
                let remote_endpoint = (addr, port);
                sock.set_ack_delay(Some(smoltcp::time::Duration::from_millis(100)));
                sock.connect(cx, remote_endpoint, self.network.next_local_port())
            });

            if let Err(err) = res {
                return Err(err.into());
            }
        }

        loop {
            let can_send = self.network.with_interface(|interface| {
                let sock = interface
                    .network_interface()
                    .get_socket::<TcpSocket>(self.socket_handle);
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
            let res = self.network.with_interface(|interface| {
                let (sock, _cx) = interface
                    .network_interface()
                    .get_socket_and_context::<TcpSocket>(self.socket_handle);
                sock.listen(port)
            });

            if let Err(err) = res {
                return Err(err.into());
            }
        }

        loop {
            let can_send = self.network.with_interface(|interface| {
                let sock = interface
                    .network_interface()
                    .get_socket::<TcpSocket>(self.socket_handle);
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
        self.network.with_interface(|interface| {
            interface
                .network_interface()
                .get_socket::<TcpSocket>(self.socket_handle)
                .close();
        });

        self.work();
    }

    pub fn disconnect(&mut self) {
        self.network.with_interface(|interface| {
            interface
                .network_interface()
                .get_socket::<TcpSocket>(self.socket_handle)
                .abort();
        });

        self.work();
    }

    pub fn is_open(&mut self) -> bool {
        self.network.with_interface(|interface| {
            interface
                .network_interface()
                .get_socket::<TcpSocket>(self.socket_handle)
                .is_open()
        })
    }

    pub fn is_connected(&mut self) -> bool {
        self.network.with_interface(|interface| {
            let socket = interface
                .network_interface()
                .get_socket::<TcpSocket>(self.socket_handle);

            socket.may_recv() && socket.may_send()
        })
    }

    pub fn work(&mut self) {
        loop {
            self.network.with_interface(|interface| {
                if let ipv4::Configuration::Client(ipv4::ClientConfiguration::DHCP(_)) =
                    interface.network_config
                {
                    interface.poll_dhcp().ok();
                } else if let ipv4::Configuration::Client(ipv4::ClientConfiguration::Fixed(
                    settings,
                )) = interface.network_config
                {
                    let addr = Ipv4Address::from_bytes(&settings.ip.octets());
                    if !interface.network_interface().has_ip_addr(addr) {
                        let gateway = Ipv4Address::from_bytes(&settings.subnet.gateway.octets());
                        interface
                            .network_interface()
                            .routes_mut()
                            .add_default_ipv4_route(gateway)
                            .ok();
                        interface.network_interface().update_ip_addrs(|addrs| {
                            addrs[0] = IpCidr::new(addr.into(), settings.subnet.mask.0);
                        });
                    }
                }
            });

            if let Ok(false) = self.network.with_interface(|interface| {
                interface
                    .network_interface()
                    .poll(Instant::from_millis(
                        (self.network.current_millis_fn)() as i64
                    ))
            }) {
                break;
            }
        }
    }
}

impl<'s, 'n: 's> Drop for Socket<'s, 'n> {
    fn drop(&mut self) {
        self.network.with_interface(|interface| {
            interface
                .network_interface
                .remove_socket(self.socket_handle)
        });
    }
}

#[derive(Debug)]
pub enum IoError {
    Other(smoltcp::Error),
    SocketClosed,
}

impl embedded_io::Error for IoError {
    fn kind(&self) -> embedded_io::ErrorKind {
        embedded_io::ErrorKind::Other
    }
}

impl From<smoltcp::Error> for IoError {
    fn from(e: smoltcp::Error) -> Self {
        IoError::Other(e)
    }
}

impl<'s, 'n: 's> Io for Socket<'s, 'n> {
    type Error = IoError;
}

impl<'s, 'n: 's> Read for Socket<'s, 'n> {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        loop {
            let res = self.network.with_interface(|interface| {
                interface
                    .network_interface()
                    .poll(Instant::from_millis(
                        (self.network.current_millis_fn)() as i64
                    ))
            });

            if let Err(err) = res {
                return Err(err.into());
            }

            let (may_recv, is_open, can_recv) = self.network.with_interface(|interface| {
                let socket = interface
                    .network_interface()
                    .get_socket::<TcpSocket>(self.socket_handle);

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
            let res = self.network.with_interface(|interface| {
                interface
                    .network_interface()
                    .poll(Instant::from_millis(
                        (self.network.current_millis_fn)() as i64
                    ))
            });

            if let Ok(false) = res {
                break;
            }
        }

        self.network.with_interface(|interface| {
            let socket = interface
                .network_interface()
                .get_socket::<TcpSocket>(self.socket_handle);

            socket.recv_slice(buf).map_err(|e| IoError::Other(e))
        })
    }
}

impl<'s, 'n: 's> Write for Socket<'s, 'n> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        loop {
            let res = self.network.with_interface(|interface| {
                interface
                    .network_interface()
                    .poll(Instant::from_millis(
                        (self.network.current_millis_fn)() as i64
                    ))
            });

            if let Err(err) = res {
                return Err(err.into());
            }

            let (may_send, is_open, can_send) = self.network.with_interface(|interface| {
                let socket = interface
                    .network_interface()
                    .get_socket::<TcpSocket>(self.socket_handle);

                (socket.may_send(), socket.is_open(), socket.can_send())
            });

            if may_send {
                break;
            }

            if !is_open {
                return Err(IoError::SocketClosed);
            }

            if !can_send {
                return Err(IoError::SocketClosed);
            }
        }

        loop {
            let res = self.network.with_interface(|interface| {
                interface
                    .network_interface()
                    .poll(Instant::from_millis(
                        (self.network.current_millis_fn)() as i64
                    ))
            });

            if let Ok(false) = res {
                break;
            }
        }

        let res = self.network.with_interface(|interface| {
            let socket = interface
                .network_interface()
                .get_socket::<TcpSocket>(self.socket_handle);

            let mut written = 0;
            loop {
                match socket.send_slice(&buf[written..]) {
                    Ok(len) => {
                        written += len;

                        if written >= buf.len() {
                            break Ok(written);
                        }

                        log::info!("not fully written: {}", len);
                    }
                    Err(err) => break Err(IoError::Other(err)),
                }
            }
        });

        res
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        loop {
            let res = self.network.with_interface(|interface| {
                interface
                    .network_interface()
                    .poll(Instant::from_millis(
                        (self.network.current_millis_fn)() as i64
                    ))
            });

            if let Ok(false) = res {
                break;
            }
        }

        Ok(())
    }
}

pub struct UdpSocket<'s, 'n: 's> {
    socket_handle: SocketHandle,
    network: &'s Network<'n>,
}

impl<'s, 'n: 's> UdpSocket<'s, 'n> {
    pub fn bind<'i>(&'i mut self, port: u16) -> Result<(), IoError>
    where
        's: 'i,
    {
        self.work();

        {
            let res = self.network.with_interface(|interface| {
                let (sock, _cx) = interface
                    .network_interface()
                    .get_socket_and_context::<smoltcp::socket::UdpSocket>(self.socket_handle);
                sock.bind(port)
            });

            if let Err(err) = res {
                return Err(err.into());
            }
        }

        loop {
            let can_send = self.network.with_interface(|interface| {
                let sock = interface
                    .network_interface()
                    .get_socket::<smoltcp::socket::UdpSocket>(self.socket_handle);
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
        self.network.with_interface(|interface| {
            interface
                .network_interface()
                .get_socket::<smoltcp::socket::UdpSocket>(self.socket_handle)
                .close();
        });

        self.work();
    }

    pub fn send(&mut self, addr: Ipv4Address, port: u16, data: &[u8]) -> Result<(), IoError> {
        loop {
            self.work();

            let (can_send, packet_capacity, payload_capacity) =
                self.network.with_interface(|interface| {
                    let sock = interface
                        .network_interface()
                        .get_socket::<smoltcp::socket::UdpSocket>(self.socket_handle);
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

        let res = self.network.with_interface(|interface| {
            let endpoint = (addr, port);

            interface
                .network_interface()
                .get_socket::<smoltcp::socket::UdpSocket>(self.socket_handle)
                .send_slice(data, endpoint.into())
        });

        self.work();

        match res {
            Ok(res) => Ok(res),
            Err(e) => Err(e.into()),
        }
    }

    pub fn receive(&mut self, data: &mut [u8]) -> Result<(usize, [u8; 4], u16), IoError> {
        self.work();

        let res = self.network.with_interface(|interface| {
            interface
                .network_interface()
                .get_socket::<smoltcp::socket::UdpSocket>(self.socket_handle)
                .recv_slice(data)
        });

        match res {
            Ok((len, endpoint)) => {
                let addr = match endpoint.addr {
                    IpAddress::Unspecified => todo!(),
                    IpAddress::Ipv4(ipv4) => ipv4,
                    _ => todo!(),
                };
                Ok((len, addr.0, endpoint.port))
            }
            Err(e) => Err(e.into()),
        }
    }

    pub fn join_multicast_group(&mut self, addr: Ipv4Address) -> Result<bool, IoError> {
        self.work();

        let res = self.network.with_interface(|interface| {
            interface.network_interface().join_multicast_group(
                addr,
                Instant::from_millis((self.network.current_millis_fn)() as i64),
            )
        });

        self.work();

        match res {
            Ok(res) => Ok(res),
            Err(e) => Err(e.into()),
        }
    }

    pub fn work(&mut self) {
        loop {
            self.network
                .with_interface(|interface| interface.poll_dhcp().ok());
            if let Ok(false) = self.network.with_interface(|interface| {
                interface
                    .network_interface()
                    .poll(Instant::from_millis(
                        (self.network.current_millis_fn)() as i64
                    ))
            }) {
                break;
            }
        }
    }
}

impl<'s, 'n: 's> Drop for UdpSocket<'s, 'n> {
    fn drop(&mut self) {
        self.network.with_interface(|interface| {
            interface
                .network_interface
                .remove_socket(self.socket_handle)
        });
    }
}
