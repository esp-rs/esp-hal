//! Access point
//!
//! Creates an open access-point with SSID `esp-wifi`.
//! You can connect to it using a static IP in range 192.168.2.2 .. 192.168.2.255, gateway 192.168.2.1
//!
//! Open http://192.168.2.1:8080/ in your browser
//!
//! On Android you might need to choose _Keep Accesspoint_ when it tells you the WiFi has no internet connection, Chrome might not want to load the URL - you can use a shell and try `curl` and `ping`

//% FEATURES: esp-wifi esp-wifi/wifi-default esp-wifi/wifi esp-wifi/utils
//% CHIPS: esp32 esp32s2 esp32s3 esp32c2 esp32c3 esp32c6

#![no_std]
#![no_main]

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
        utils::create_network_interface,
        AccessPointConfiguration,
        Configuration,
        WifiApDevice,
    },
    wifi_interface::WifiStack,
    EspWifiInitFor,
};
use smoltcp::iface::SocketStorage;

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
        EspWifiInitFor::Wifi,
        timg0.timer0,
        Rng::new(peripherals.RNG),
        peripherals.RADIO_CLK,
    )
    .unwrap();

    let mut wifi = peripherals.WIFI;
    let mut socket_set_entries: [SocketStorage; 3] = Default::default();
    let (iface, device, mut controller, sockets) =
        create_network_interface(&init, &mut wifi, WifiApDevice, &mut socket_set_entries).unwrap();
    let now = || time::now().duration_since_epoch().to_millis();
    let mut wifi_stack = WifiStack::new(iface, device, sockets, now);

    let client_config = Configuration::AccessPoint(AccessPointConfiguration {
        ssid: "esp-wifi".try_into().unwrap(),
        ..Default::default()
    });
    let res = controller.set_configuration(&client_config);
    println!("wifi_set_configuration returned {:?}", res);

    controller.start().unwrap();
    println!("is wifi started: {:?}", controller.is_started());

    println!("{:?}", controller.get_capabilities());

    wifi_stack
        .set_iface_configuration(&esp_wifi::wifi::ipv4::Configuration::Client(
            esp_wifi::wifi::ipv4::ClientConfiguration::Fixed(
                esp_wifi::wifi::ipv4::ClientSettings {
                    ip: esp_wifi::wifi::ipv4::Ipv4Addr::from(parse_ip("192.168.2.1")),
                    subnet: esp_wifi::wifi::ipv4::Subnet {
                        gateway: esp_wifi::wifi::ipv4::Ipv4Addr::from(parse_ip("192.168.2.1")),
                        mask: esp_wifi::wifi::ipv4::Mask(24),
                    },
                    dns: None,
                    secondary_dns: None,
                },
            ),
        ))
        .unwrap();

    println!("Start busy loop on main. Connect to the AP `esp-wifi` and point your browser to http://192.168.2.1:8080/");
    println!("Use a static IP in the range 192.168.2.2 .. 192.168.2.255, use gateway 192.168.2.1");

    let mut rx_buffer = [0u8; 1536];
    let mut tx_buffer = [0u8; 1536];
    let mut socket = wifi_stack.get_socket(&mut rx_buffer, &mut tx_buffer);

    socket.listen(8080).unwrap();

    loop {
        socket.work();

        if !socket.is_open() {
            socket.listen(8080).unwrap();
        }

        if socket.is_connected() {
            println!("Connected");

            let mut time_out = false;
            let deadline = time::now() + Duration::secs(20);
            let mut buffer = [0u8; 1024];
            let mut pos = 0;
            while let Ok(len) = socket.read(&mut buffer[pos..]) {
                let to_print = unsafe { core::str::from_utf8_unchecked(&buffer[..(pos + len)]) };

                if to_print.contains("\r\n\r\n") {
                    print!("{}", to_print);
                    println!();
                    break;
                }

                pos += len;

                if time::now() > deadline {
                    println!("Timeout");
                    time_out = true;
                    break;
                }
            }

            if !time_out {
                socket
                    .write_all(
                        b"HTTP/1.0 200 OK\r\n\r\n\
                    <html>\
                        <body>\
                            <h1>Hello Rust! Hello esp-wifi!</h1>\
                        </body>\
                    </html>\r\n\
                    ",
                    )
                    .unwrap();

                socket.flush().unwrap();
            }

            socket.close();

            println!("Done\n");
            println!();
        }

        let deadline = time::now() + Duration::secs(5);
        while time::now() < deadline {
            socket.work();
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
