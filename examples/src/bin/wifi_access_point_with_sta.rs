//! Access point with station
//!
//! Set SSID and PASSWORD env variable before running this example.
//!
//! Gets an ip address via DHCP, creates an open access-point with SSID `esp-wifi`
//! You can connect to it using a static IP in range 192.168.2.2 .. 192.168.2.255, gateway 192.168.2.1
//! Open http://192.168.2.1:8080/ in your browser - the example will perform an HTTP get request to some "random" server
//!
//! On Android you might need to choose _Keep Accesspoint_ when it tells you the WiFi has no internet connection, Chrome might not want to load the URL - you can use a shell and try `curl` and `ping`
//!

//% FEATURES: esp-wifi esp-wifi/wifi esp-wifi/utils
//% CHIPS: esp32 esp32s2 esp32s3 esp32c2 esp32c3 esp32c6

#![no_std]
#![no_main]

use blocking_network_stack::Stack;
use embedded_io::*;
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
        utils::{create_ap_sta_network_interface, ApStaInterface},
        AccessPointConfiguration,
        ClientConfiguration,
        Configuration,
    },
};
use smoltcp::{
    iface::{SocketSet, SocketStorage},
    wire::{IpAddress, Ipv4Address},
};

const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");

#[entry]
fn main() -> ! {
    esp_println::logger::init_logger(log::LevelFilter::Info);
    let peripherals = esp_hal::init({
        let mut config = esp_hal::Config::default();
        config.cpu_clock = CpuClock::max();
        config
    });

    esp_alloc::heap_allocator!(72 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);

    let mut rng = Rng::new(peripherals.RNG);

    let init = init(timg0.timer0, rng.clone(), peripherals.RADIO_CLK).unwrap();

    let wifi = peripherals.WIFI;

    let ApStaInterface {
        ap_interface,
        sta_interface,
        ap_device,
        sta_device,
        mut controller,
    } = create_ap_sta_network_interface(&init, wifi).unwrap();

    let now = || time::now().duration_since_epoch().to_millis();
    let mut ap_socket_set_entries: [SocketStorage; 3] = Default::default();
    let ap_socket_set = SocketSet::new(&mut ap_socket_set_entries[..]);
    let mut ap_stack = Stack::new(ap_interface, ap_device, ap_socket_set, now, rng.random());

    let mut sta_socket_set_entries: [SocketStorage; 3] = Default::default();
    let mut sta_socket_set = SocketSet::new(&mut sta_socket_set_entries[..]);
    sta_socket_set.add(smoltcp::socket::dhcpv4::Socket::new());
    let sta_stack = Stack::new(sta_interface, sta_device, sta_socket_set, now, rng.random());

    let client_config = Configuration::Mixed(
        ClientConfiguration {
            ssid: SSID.try_into().unwrap(),
            password: PASSWORD.try_into().unwrap(),
            ..Default::default()
        },
        AccessPointConfiguration {
            ssid: "esp-wifi".try_into().unwrap(),
            ..Default::default()
        },
    );
    let res = controller.set_configuration(&client_config);
    println!("wifi_set_configuration returned {:?}", res);

    controller.start().unwrap();
    println!("is wifi started: {:?}", controller.is_started());

    println!("{:?}", controller.capabilities());

    ap_stack
        .set_iface_configuration(&blocking_network_stack::ipv4::Configuration::Client(
            blocking_network_stack::ipv4::ClientConfiguration::Fixed(
                blocking_network_stack::ipv4::ClientSettings {
                    ip: blocking_network_stack::ipv4::Ipv4Addr::from(parse_ip("192.168.2.1")),
                    subnet: blocking_network_stack::ipv4::Subnet {
                        gateway: blocking_network_stack::ipv4::Ipv4Addr::from(parse_ip(
                            "192.168.2.1",
                        )),
                        mask: blocking_network_stack::ipv4::Mask(24),
                    },
                    dns: None,
                    secondary_dns: None,
                },
            ),
        ))
        .unwrap();

    println!("wifi_connect {:?}", controller.connect());

    // wait for STA getting an ip address
    println!("Wait to get an ip address");
    loop {
        sta_stack.work();

        if sta_stack.is_iface_up() {
            println!("got ip {:?}", sta_stack.get_ip_info());
            break;
        }
    }

    println!("Start busy loop on main. Connect to the AP `esp-wifi` and point your browser to http://192.168.2.1:8080/");
    println!("Use a static IP in the range 192.168.2.2 .. 192.168.2.255, use gateway 192.168.2.1");

    let mut rx_buffer = [0u8; 1536];
    let mut tx_buffer = [0u8; 1536];
    let mut ap_socket = ap_stack.get_socket(&mut rx_buffer, &mut tx_buffer);

    let mut sta_rx_buffer = [0u8; 1536];
    let mut sta_tx_buffer = [0u8; 1536];
    let mut sta_socket = sta_stack.get_socket(&mut sta_rx_buffer, &mut sta_tx_buffer);

    ap_socket.listen(8080).unwrap();

    loop {
        ap_socket.work();

        if !ap_socket.is_open() {
            ap_socket.listen(8080).unwrap();
        }

        if ap_socket.is_connected() {
            println!("Connected");

            let mut time_out = false;
            let deadline = time::now() + Duration::secs(20);
            let mut buffer = [0u8; 1024];
            let mut pos = 0;
            loop {
                if let Ok(len) = ap_socket.read(&mut buffer[pos..]) {
                    let to_print =
                        unsafe { core::str::from_utf8_unchecked(&buffer[..(pos + len)]) };

                    if to_print.contains("\r\n\r\n") {
                        print!("{}", to_print);
                        println!();
                        break;
                    }

                    pos += len;
                } else {
                    break;
                }

                if time::now() > deadline {
                    println!("Timeout");
                    time_out = true;
                    break;
                }
            }

            if !time_out {
                println!("Making HTTP request");
                sta_socket.work();

                sta_socket
                    .open(IpAddress::Ipv4(Ipv4Address::new(142, 250, 185, 115)), 80)
                    .unwrap();

                sta_socket
                    .write(b"GET / HTTP/1.0\r\nHost: www.mobile-j.de\r\n\r\n")
                    .unwrap();
                sta_socket.flush().unwrap();

                let deadline = time::now() + Duration::secs(20);
                loop {
                    let mut buffer = [0u8; 512];
                    if let Ok(len) = sta_socket.read(&mut buffer) {
                        ap_socket.write_all(&buffer[..len]).unwrap();
                        ap_socket.flush().unwrap();
                    } else {
                        break;
                    }

                    if time::now() > deadline {
                        println!("Timeout");
                        break;
                    }
                }
                println!();

                sta_socket.disconnect();
            }

            ap_socket.close();

            println!("Done\n");
            println!();
        }

        let deadline = time::now() + Duration::secs(5);
        while time::now() < deadline {
            ap_socket.work();
        }
    }
}

fn parse_ip(ip: &str) -> [u8; 4] {
    let mut result = [0u8; 4];
    for (idx, octet) in ip.split(".").into_iter().enumerate() {
        result[idx] = u8::from_str_radix(octet, 10).unwrap();
    }
    result
}
