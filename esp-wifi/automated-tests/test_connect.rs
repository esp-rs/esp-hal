#![no_std]
#![no_main]

#[path = "../../examples-util/util.rs"]
mod examples_util;
use examples_util::hal;

use embedded_io::*;
use esp_wifi::wifi::{AccessPointInfo, AuthMethod, ClientConfiguration, Configuration};

use esp_backtrace as _;
use esp_println::println;
use esp_wifi::initialize;
use esp_wifi::wifi::WifiStaDevice;
use esp_wifi::wifi::{utils::create_network_interface, WifiError};
use esp_wifi::wifi_interface::WifiStack;
use esp_wifi::{current_millis, EspWifiInitFor};
use hal::clock::ClockControl;
use hal::Rng;
use hal::{peripherals::Peripherals, prelude::*};

use smoltcp::iface::SocketStorage;
use smoltcp::wire::{IpAddress, Ipv4Address};

const SSID: &str = "esp-wifi";
const STATIC_IP: &str = "192.168.2.2";
const GATEWAY_IP: &str = "192.168.2.1";

#[entry]
fn main() -> ! {
    #[cfg(feature = "log")]
    esp_println::logger::init_logger(log::LevelFilter::Info);

    println!("Running test");

    #[cfg(not(feature = "esp32"))]
    println!("[RUN esp32 open_access_point]");

    #[cfg(feature = "esp32")]
    println!("[RUN esp32c3 open_access_point]");

    let peripherals = Peripherals::take();

    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::max(system.clock_control).freeze();

    #[cfg(target_arch = "xtensa")]
    let timer = hal::timer::TimerGroup::new(peripherals.TIMG1, &clocks).timer0;
    #[cfg(target_arch = "riscv32")]
    let timer = hal::systimer::SystemTimer::new(peripherals.SYSTIMER).alarm0;
    let init = initialize(
        EspWifiInitFor::Wifi,
        timer,
        Rng::new(peripherals.RNG),
        system.radio_clock_control,
        &clocks,
    )
    .unwrap();

    let wifi = peripherals.WIFI;
    let mut socket_set_entries: [SocketStorage; 3] = Default::default();
    let (iface, device, mut controller, sockets) =
        create_network_interface(&init, wifi, WifiStaDevice, &mut socket_set_entries).unwrap();
    let mut wifi_stack = WifiStack::new(iface, device, sockets, current_millis);

    let client_config = Configuration::Client(ClientConfiguration {
        ssid: SSID.try_into().unwrap(),
        auth_method: AuthMethod::None,
        ..Default::default()
    });
    let res = controller.set_configuration(&client_config);
    println!("wifi_set_configuration returned {:?}", res);

    controller.start().unwrap();
    println!("is wifi started: {:?}", controller.is_started());

    println!("{:?}", controller.get_capabilities());

    let mut tries = 15;

    'outer: loop {
        println!("Start Wifi Scan");
        let res: Result<(heapless::Vec<AccessPointInfo, 10>, usize), WifiError> =
            controller.scan_n();
        if let Ok((res, _count)) = res {
            for ap in res {
                println!("{:?}", ap);
                if ap.ssid == SSID {
                    break 'outer;
                }
            }
        }
        tries -= 1;
        if tries == 0 {
            break 'outer;
        }

        let wait_end = current_millis() + 1 * 1000;
        while current_millis() < wait_end {
            // wait
        }
    }

    println!("wifi_connect {:?}", controller.connect());

    // wait to get connected
    println!("Wait to get connected");
    loop {
        let res = controller.is_connected();
        match res {
            Ok(connected) => {
                if connected {
                    break;
                }
            }
            Err(err) => {
                println!("{:?}", err);
                break;
            }
        }
    }
    println!("{:?}", controller.is_connected());

    if let Ok(c) = controller.is_connected() {
        if !c {
            println!("[FAILED]");
            loop {}
        }
    }
    if let Err(WifiError::Disconnected) = controller.is_connected() {
        println!("[FAILED]");
        loop {}
    }

    println!("Setting static IP {}", STATIC_IP);

    wifi_stack
        .set_iface_configuration(&esp_wifi::wifi::ipv4::Configuration::Client(
            esp_wifi::wifi::ipv4::ClientConfiguration::Fixed(
                esp_wifi::wifi::ipv4::ClientSettings {
                    ip: esp_wifi::wifi::ipv4::Ipv4Addr::from(parse_ip(STATIC_IP)),
                    subnet: esp_wifi::wifi::ipv4::Subnet {
                        gateway: esp_wifi::wifi::ipv4::Ipv4Addr::from(parse_ip(GATEWAY_IP)),
                        mask: esp_wifi::wifi::ipv4::Mask(24),
                    },
                    dns: None,
                    secondary_dns: None,
                },
            ),
        ))
        .unwrap();

    let mut rx_buffer = [0u8; 1536];
    let mut tx_buffer = [0u8; 1536];
    let mut socket = wifi_stack.get_socket(&mut rx_buffer, &mut tx_buffer);

    'outer: loop {
        socket.work();

        socket
            .open(IpAddress::Ipv4(Ipv4Address::new(192, 168, 2, 1)), 8080)
            .unwrap();

        loop {
            let mut buffer = [0u8; 512];
            if let Ok(len) = socket.read(&mut buffer) {
                let to_print = unsafe { core::str::from_utf8_unchecked(&buffer[..len]) };
                println!("{}", to_print);
                if to_print.contains("DATA") {
                    println!("[PASSED]");
                    break 'outer;
                }
            } else {
                break;
            }
        }
        println!();

        socket.disconnect();
    }

    loop {}
}

fn parse_ip(ip: &str) -> [u8; 4] {
    let mut result = [0u8; 4];
    for (idx, octet) in ip.split(".").into_iter().enumerate() {
        result[idx] = u8::from_str_radix(octet, 10).unwrap();
    }
    result
}
