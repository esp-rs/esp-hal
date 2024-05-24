#![no_std]
#![no_main]

#[path = "../../examples-util/util.rs"]
mod examples_util;
use examples_util::hal;

use embedded_io::*;
use esp_wifi::wifi::{AccessPointInfo, ClientConfiguration, Configuration};

use esp_backtrace as _;
use esp_println::{print, println};
use esp_wifi::wifi::utils::create_network_interface;
use esp_wifi::wifi::{WifiError, WifiStaDevice};
use esp_wifi::wifi_interface::WifiStack;
use esp_wifi::{current_millis, initialize, EspWifiInitFor};
use hal::clock::ClockControl;
use hal::rng::Rng;
use hal::{peripherals::Peripherals, prelude::*};
use smoltcp::iface::SocketStorage;
use smoltcp::wire::IpAddress;
use smoltcp::wire::Ipv4Address;

const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");

#[entry]
fn main() -> ! {
    #[cfg(feature = "log")]
    esp_println::logger::init_logger(log::LevelFilter::Info);

    let peripherals = Peripherals::take();

    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::max(system.clock_control).freeze();

    #[cfg(target_arch = "xtensa")]
    let timer = hal::timer::TimerGroup::new(peripherals.TIMG1, &clocks, None).timer0;
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
    let wifi_stack = WifiStack::new(iface, device, sockets, current_millis);

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
        let res = controller.is_connected();
        match res {
            Ok(connected) => {
                if connected {
                    break;
                }
            }
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
        wifi_stack.work();

        if wifi_stack.is_iface_up() {
            println!("got ip {:?}", wifi_stack.get_ip_info());
            break;
        }
    }

    println!("Start busy loop on main");

    let mut rx_buffer = [0u8; 1536];
    let mut tx_buffer = [0u8; 1536];
    let mut socket = wifi_stack.get_socket(&mut rx_buffer, &mut tx_buffer);

    loop {
        println!("Making HTTP request");
        socket.work();

        socket
            .open(IpAddress::Ipv4(Ipv4Address::new(142, 250, 185, 115)), 80)
            .unwrap();

        socket
            .write(b"GET / HTTP/1.0\r\nHost: www.mobile-j.de\r\n\r\n")
            .unwrap();
        socket.flush().unwrap();

        let wait_end = current_millis() + 20 * 1000;
        loop {
            let mut buffer = [0u8; 512];
            if let Ok(len) = socket.read(&mut buffer) {
                let to_print = unsafe { core::str::from_utf8_unchecked(&buffer[..len]) };
                print!("{}", to_print);
            } else {
                break;
            }

            if current_millis() > wait_end {
                println!("Timeout");
                break;
            }
        }
        println!();

        socket.disconnect();

        let wait_end = current_millis() + 5 * 1000;
        while current_millis() < wait_end {
            socket.work();
        }
    }
}
