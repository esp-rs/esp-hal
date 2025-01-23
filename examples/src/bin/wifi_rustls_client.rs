//! Example using Rustls
//!
//! In general, if you can get away with < https://crates.io/crates/embedded-tls > you should prefer that.
//! While Rustls can do more it's more of a heavy-weight dependency.
//!
//!
//! Set SSID and PASSWORD env variable before running this example.
//!
//! This gets an ip address via DHCP then performs an HTTPS request for "google.com"

//% FEATURES: esp-wifi esp-wifi/wifi esp-wifi/utils esp-hal/unstable esp-rustls-provider ntp-nostd
//% CHIPS: esp32 esp32s3 esp32c6

#![no_std]
#![no_main]

extern crate alloc;

use blocking_network_stack::Stack;
use embedded_io::*;
use esp_backtrace as _;
use esp_hal::{clock::CpuClock, main, rng::Rng, time::Duration, timer::timg::TimerGroup};
use esp_println::{print, println};
use esp_rustls_provider::{
    adapter::client::ClientConnection,
    rustls,
    webpki_roots,
    EspTimeProvider,
};
use esp_wifi::{
    config::PowerSaveMode,
    wifi::{
        utils::create_network_interface,
        AccessPointInfo,
        ClientConfiguration,
        Configuration,
        WifiDevice,
        WifiError,
        WifiStaDevice,
    },
};
use smoltcp::{
    iface::{SocketSet, SocketStorage},
    wire::{DhcpOption, IpAddress, Ipv4Address},
};

const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");

const KB: usize = 1024;
const INCOMING_TLS_BUFSIZE: usize = 16 * KB;
const OUTGOING_TLS_INITIAL_BUFSIZE: usize = KB;

#[main]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    static mut HEAP: core::mem::MaybeUninit<[u8; 72 * 1024]> = core::mem::MaybeUninit::uninit();

    #[link_section = ".dram2_uninit"]
    static mut HEAP2: core::mem::MaybeUninit<[u8; 64 * 1024]> = core::mem::MaybeUninit::uninit();

    #[allow(static_mut_refs)]
    unsafe {
        esp_alloc::HEAP.add_region(esp_alloc::HeapRegion::new(
            HEAP.as_mut_ptr() as *mut u8,
            core::mem::size_of_val(&*core::ptr::addr_of!(HEAP)),
            esp_alloc::MemoryCapability::Internal.into(),
        ));

        // COEX needs more RAM - add some more
        esp_alloc::HEAP.add_region(esp_alloc::HeapRegion::new(
            HEAP2.as_mut_ptr() as *mut u8,
            core::mem::size_of_val(&*core::ptr::addr_of!(HEAP2)),
            esp_alloc::MemoryCapability::Internal.into(),
        ));
    }

    let timg0 = TimerGroup::new(peripherals.TIMG0);

    let mut rng = Rng::new(peripherals.RNG);

    let init = esp_wifi::init(timg0.timer0, rng.clone(), peripherals.RADIO_CLK).unwrap();

    let mut wifi = peripherals.WIFI;
    let (iface, device, mut controller) =
        create_network_interface(&init, &mut wifi, WifiStaDevice).unwrap();
    controller.set_power_saving(PowerSaveMode::None).unwrap();

    let mut socket_set_entries: [SocketStorage; 3] = Default::default();
    let mut socket_set = SocketSet::new(&mut socket_set_entries[..]);
    let mut dhcp_socket = smoltcp::socket::dhcpv4::Socket::new();
    // we can set a hostname here (or add other DHCP options)
    dhcp_socket.set_outgoing_options(&[DhcpOption {
        kind: 12,
        data: b"esp-wifi",
    }]);
    socket_set.add(dhcp_socket);

    let now = || esp_hal::time::now().duration_since_epoch().to_millis();
    let stack = Stack::new(iface, device, socket_set, now, rng.random());

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

    println!("{:?}", controller.capabilities());
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
        stack.work();

        if stack.is_iface_up() {
            println!("got ip {:?}", stack.get_ip_info());
            break;
        }
    }

    let mut rx_meta1 = [smoltcp::socket::udp::PacketMetadata::EMPTY; 10];
    let mut rx_buffer1 = [0u8; 1536];
    let mut tx_meta1 = [smoltcp::socket::udp::PacketMetadata::EMPTY; 10];
    let mut tx_buffer1 = [0u8; 1536];
    let mut udp_socket = stack.get_udp_socket(
        &mut rx_meta1,
        &mut rx_buffer1,
        &mut tx_meta1,
        &mut tx_buffer1,
    );
    udp_socket.bind(50123).unwrap();

    let mut rx_buffer = [0u8; 1536];
    let mut tx_buffer = [0u8; 1536];
    let mut socket = stack.get_socket(&mut rx_buffer, &mut tx_buffer);

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
        let tls = ClientConnection::new(
            config.clone(),
            server_name,
            socket,
            &mut incoming_tls,
            &mut outgoing_tls,
            &mut plaintext_in,
            &mut plaintext_out,
        );

        match tls {
            Ok(mut tls) => {
                tls.write(b"GET / HTTP/1.0\r\nHost: google.com\r\n\r\n")
                    .unwrap();
                tls.flush().unwrap();

                let deadline = esp_hal::time::now() + Duration::secs(20);
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

                    if esp_hal::time::now() > deadline {
                        println!("Timeout");
                        break;
                    }
                }
                println!();

                socket = tls.free();
            }
            Err((s, _)) => {
                socket = s;
            }
        }

        socket.disconnect();

        let deadline = esp_hal::time::now() + Duration::secs(5);
        while esp_hal::time::now() < deadline {
            socket.work();
        }

        println!();
    }
}

fn get_current_unix_ts<'s, 'n, 'd>(
    udp_socket: &mut blocking_network_stack::UdpSocket<'s, 'n, WifiDevice<'d, WifiStaDevice>>,
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

        let deadline = esp_hal::time::now() + Duration::secs(1);
        while esp_hal::time::now() < deadline {}

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
