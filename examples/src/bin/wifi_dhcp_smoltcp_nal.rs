//! DHCP Example using [smoltcp-nal](https://crates.io/crates/smoltcp-nal)
//!
//!
//! Set SSID and PASSWORD env variable before running this example.
//!
//! This gets an ip address via DHCP then performs an HTTP get request to some "random" server
//! When using USB-SERIAL-JTAG you may have to activate the feature `phy-enable-usb` in the esp-wifi crate.

//% FEATURES: esp-wifi esp-wifi/wifi-default esp-wifi/wifi esp-wifi/utils
//% CHIPS: esp32 esp32s2 esp32s3 esp32c2 esp32c3 esp32c6

#![no_std]
#![no_main]

extern crate alloc;

use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{
    prelude::*,
    rng::Rng,
    time::{self, Duration},
    timer::timg::TimerGroup,
};
use esp_println::{print, println};
use esp_wifi::{
    init,
    wifi::{
        utils::create_network_interface,
        AccessPointInfo,
        ClientConfiguration,
        Configuration,
        WifiError,
        WifiStaDevice,
    },
};
use smoltcp::iface::SocketStorage;
use smoltcp_nal::{
    embedded_nal::{Ipv4Addr, SocketAddr, SocketAddrV4, TcpClientStack},
    NetworkStack,
};

const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");

#[entry]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init({
        let mut config = esp_hal::Config::default();
        config.cpu_clock = CpuClock::max();
        config
    });

    esp_alloc::heap_allocator!(72 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);

    let init = init(
        timg0.timer0,
        Rng::new(peripherals.RNG),
        peripherals.RADIO_CLK,
    )
    .unwrap();

    let mut wifi = peripherals.WIFI;
    let mut socket_set_entries: [SocketStorage; 3] = Default::default();
    let (iface, device, mut controller, mut sockets) =
        create_network_interface(&init, &mut wifi, WifiStaDevice, &mut socket_set_entries).unwrap();

    let mut rx_buffer = [0u8; 128];
    let mut tx_buffer = [0u8; 128];
    sockets.add(smoltcp::socket::tcp::Socket::new(
        smoltcp::socket::tcp::SocketBuffer::new(&mut rx_buffer[..]),
        smoltcp::socket::tcp::SocketBuffer::new(&mut tx_buffer[..]),
    ));
    let mut network_stack = NetworkStack::new(iface, device, sockets, StackClock);

    let client_config = Configuration::Client(ClientConfiguration {
        ssid: SSID.try_into().unwrap(),
        password: PASSWORD.try_into().unwrap(),
        ..Default::default()
    });
    let res = controller.set_configuration(&client_config);
    println!("wifi_set_configuration returned {:?}", res);

    controller.start().unwrap();
    println!("is wifi started: {:?}", controller.is_started());

    println!("Start Wifi Scan");
    let res: Result<(heapless::Vec<AccessPointInfo, 10>, usize), WifiError> = controller.scan_n();
    if let Ok((res, _count)) = res {
        for ap in res {
            println!("{:?}", ap);
        }
    }

    println!("{:?}", controller.get_capabilities());
    println!("wifi_connect {:?}", controller.connect());

    // wait to get connected
    println!("Wait to get connected");
    loop {
        match controller.is_connected() {
            Ok(true) => break,
            Ok(false) => {}
            Err(err) => {
                println!("{:?}", err);
                loop {}
            }
        }
    }
    println!("{:?}", controller.is_connected());

    // wait for getting an ip address
    println!("Wait to get an ip address");
    loop {
        network_stack.poll().unwrap();

        if let Some(ip) = network_stack.interface().ipv4_addr() {
            if !ip.is_unspecified() {
                println!("got ip {:?}", ip);
                break;
            }
        }
    }

    println!("Start busy loop on main");

    let mut socket = network_stack.socket().unwrap();

    loop {
        println!("Making HTTP request");
        while network_stack.poll().unwrap() {}

        with_network_stack(&mut network_stack, |network_stack| {
            network_stack.connect(
                &mut socket,
                SocketAddr::V4(SocketAddrV4::new(Ipv4Addr::new(142, 250, 185, 115), 80)),
            )
        })
        .unwrap();

        println!("connected");

        with_network_stack(&mut network_stack, |network_stack| {
            network_stack.send(
                &mut socket,
                b"GET / HTTP/1.0\r\nHost: www.mobile-j.de\r\n\r\n",
            )
        })
        .unwrap();

        let deadline = time::now() + Duration::secs(20);
        let mut buffer = [0u8; 512];
        while let Ok(len) = with_network_stack(&mut network_stack, |network_stack| {
            network_stack.receive(&mut socket, &mut buffer)
        }) {
            let to_print = unsafe { core::str::from_utf8_unchecked(&buffer[..len]) };
            print!("{}", to_print);

            if time::now() > deadline {
                println!("Timeout");
                break;
            }

            network_stack.poll().unwrap();
        }
        println!();

        network_stack.close_sockets();

        let deadline = time::now() + Duration::secs(5);
        while time::now() < deadline {
            network_stack.poll().unwrap();
        }
    }
}

fn with_network_stack<'a, D: smoltcp::phy::Device, C: embedded_time::Clock<T = u32>, R>(
    network_stack: &mut smoltcp_nal::NetworkStack<'a, D, C>,
    mut f: impl FnMut(
        &mut smoltcp_nal::NetworkStack<'a, D, C>,
    ) -> smoltcp_nal::embedded_nal::nb::Result<R, smoltcp_nal::NetworkError>,
) -> smoltcp_nal::embedded_nal::nb::Result<R, smoltcp_nal::NetworkError> {
    let res = loop {
        let res = f(network_stack);
        if let nb::Result::Err(nb::Error::WouldBlock) = res {
            network_stack.poll().unwrap();
            continue;
        }

        break res;
    };

    res
}

struct StackClock;

impl embedded_time::Clock for StackClock {
    type T = u32;

    const SCALING_FACTOR: embedded_time::rate::Fraction =
        embedded_time::rate::Fraction::new(1, 1_000_000);

    fn try_now(&self) -> Result<embedded_time::Instant<Self>, embedded_time::clock::Error> {
        Ok(embedded_time::Instant::new(
            esp_hal::time::now().ticks() as u32
        ))
    }
}
