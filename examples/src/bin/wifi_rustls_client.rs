//! Example using Rustls
//!
//! In general, if you can get away with < https://crates.io/crates/embedded-tls > you should prefer that.
//! While Rustls can do more it's more of a heavy-weight dependency.
//!
//!
//! Set SSID and PASSWORD env variable before running this example.
//!
//! This gets an ip address via DHCP then performs an HTTPS request for "google.com"

//% FEATURES: esp-wifi esp-wifi/wifi-default esp-wifi/wifi esp-wifi/utils esp-rustls-provider ntp-nostd
//% CHIPS: esp32 esp32s3 esp32c6

#![no_std]
#![no_main]

extern crate alloc;

use embedded_io::*;
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    peripherals::Peripherals,
    prelude::*,
    rng::Rng,
    system::SystemControl,
    timer::{timg::TimerGroup, ErasedTimer, PeriodicTimer},
};
use esp_println::{print, println};
use esp_rustls_provider::{
    adapter::client::ClientConnection,
    rustls,
    webpki_roots,
    EspTimeProvider,
};
use esp_wifi::{
    current_millis,
    initialize,
    wifi::{
        utils::create_network_interface,
        AccessPointInfo,
        ClientConfiguration,
        Configuration,
        WifiError,
        WifiStaDevice,
    },
    wifi_interface::WifiStack,
    EspWifiInitFor,
};
use smoltcp::{
    iface::SocketStorage,
    wire::{IpAddress, Ipv4Address},
};

const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");

const KB: usize = 1024;
const INCOMING_TLS_BUFSIZE: usize = 16 * KB;
const OUTGOING_TLS_INITIAL_BUFSIZE: usize = KB;

#[entry]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    // Rustls is no-std but alloc - so we need a heap
    esp_alloc::heap_allocator!(48 * 1024);

    let peripherals = Peripherals::take();

    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::max(system.clock_control).freeze();

    let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let timer0: ErasedTimer = timg0.timer0.into();
    let timer = PeriodicTimer::new(timer0);

    let init = initialize(
        EspWifiInitFor::Wifi,
        timer,
        Rng::new(peripherals.RNG),
        peripherals.RADIO_CLK,
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

    let mut rx_meta1 = [smoltcp::socket::udp::PacketMetadata::EMPTY; 10];
    let mut rx_buffer1 = [0u8; 1536];
    let mut tx_meta1 = [smoltcp::socket::udp::PacketMetadata::EMPTY; 10];
    let mut tx_buffer1 = [0u8; 1536];
    let mut udp_socket = wifi_stack.get_udp_socket(
        &mut rx_meta1,
        &mut rx_buffer1,
        &mut tx_meta1,
        &mut tx_buffer1,
    );
    udp_socket.bind(50123).unwrap();

    let mut rx_buffer = [0u8; 1536];
    let mut tx_buffer = [0u8; 1536];
    let mut socket = wifi_stack.get_socket(&mut rx_buffer, &mut tx_buffer);

    let mut incoming_tls = [0; INCOMING_TLS_BUFSIZE];
    let mut outgoing_tls = [0; OUTGOING_TLS_INITIAL_BUFSIZE];
    let mut plaintext_in = [0u8; INCOMING_TLS_BUFSIZE];
    let mut plaintext_out = [0u8; KB];

    let unix_ts_now = get_current_unix_ts(&mut udp_socket);
    let time_provider = EspTimeProvider::new(unix_ts_now);
    println!("unix_ts_now = {}", unix_ts_now);

    let root_store =
        rustls::RootCertStore::from_iter(webpki_roots::TLS_SERVER_ROOTS.iter().cloned());

    let mut config = rustls::ClientConfig::builder_with_details(
        esp_rustls_provider::provider().into(),
        alloc::sync::Arc::new(time_provider),
    )
    .with_safe_default_protocol_versions()
    .unwrap()
    .with_root_certificates(root_store)
    .with_no_client_auth();
    config.enable_early_data = true;

    let config = alloc::sync::Arc::new(config);

    println!("Start busy loop on main");

    loop {
        println!("Making HTTP request");
        socket.work();

        socket
            .open(IpAddress::Ipv4(Ipv4Address::new(142, 251, 36, 164)), 443)
            .unwrap();

        let server_name = "google.com".try_into().unwrap();
        let mut tls = ClientConnection::new(
            config.clone(),
            server_name,
            socket,
            &mut incoming_tls,
            &mut outgoing_tls,
            &mut plaintext_in,
            &mut plaintext_out,
        )
        .unwrap();

        tls.write(b"GET / HTTP/1.0\r\nHost: google.com\r\n\r\n")
            .unwrap();
        tls.flush().unwrap();

        let wait_end = current_millis() + 20 * 1000;
        loop {
            let mut buffer = [0u8; 512];

            let res = tls.read(&mut buffer);
            if let Ok(len) = res {
                let to_print = unsafe { core::str::from_utf8_unchecked(&buffer[..len]) };
                print!("{}", to_print);
            } else {
                println!("{:?}", res);
                break;
            }

            if current_millis() > wait_end {
                println!("Timeout");
                break;
            }
        }
        println!();

        socket = tls.free();
        socket.disconnect();

        let wait_end = current_millis() + 5 * 1000;
        while current_millis() < wait_end {
            socket.work();
        }

        println!();
    }
}

fn get_current_unix_ts<'s, 'n>(
    udp_socket: &mut esp_wifi::wifi_interface::UdpSocket<'s, 'n, esp_wifi::wifi::WifiStaDevice>,
) -> u32 {
    let req_data = ntp_nostd::get_client_request();
    let mut rcvd_data = [0_u8; 1536];

    udp_socket
        // using ip from https://tf.nist.gov/tf-cgi/servers.cgi (time-b-wwv.nist.gov)
        .send(Ipv4Address::new(132, 163, 97, 2).into(), 123, &req_data)
        .unwrap();
    let mut count = 0;

    loop {
        count += 1;
        let rcvd = udp_socket.receive(&mut rcvd_data);
        if rcvd.is_ok() {
            break;
        }

        let wait_end = current_millis() + 500;
        while current_millis() < wait_end {}

        if count > 10 {
            udp_socket
                // retry with another server
                // using ip from https://tf.nist.gov/tf-cgi/servers.cgi (time-b-g.nist.gov)
                .send(Ipv4Address::new(129, 6, 15, 29).into(), 123, &req_data)
                .unwrap();
            println!("Trying another NTP server...");
            count = 0;
        }
    }
    let response = ntp_nostd::NtpServerResponse::from(rcvd_data.as_ref());
    if response.headers.tx_time_seconds == 0 {
        panic!("No timestamp received");
    }

    response.headers.get_unix_timestamp()
}
