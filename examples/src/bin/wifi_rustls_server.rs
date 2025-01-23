//! Example using Rustls as a server
//!
//! Set SSID and PASSWORD env variable before running this example.
//!
//! This gets an ip address via DHCP then runs an HTTPS server on port 4443

//% FEATURES: esp-wifi esp-wifi/wifi esp-wifi/utils esp-hal/unstable esp-rustls-provider ntp-nostd
//% CHIPS: esp32 esp32s3 esp32c6

#![no_std]
#![no_main]

extern crate alloc;

use alloc::sync::Arc;

use blocking_network_stack::Stack;
use embedded_io::*;
use esp_backtrace as _;
use esp_hal::{clock::CpuClock, main, rng::Rng, time::Duration, timer::timg::TimerGroup};
use esp_println::{print, println};
use esp_rustls_provider::{adapter::server::ServerConnection, rustls, EspTimeProvider};
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
    wire::{DhcpOption, Ipv4Address},
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

    let pki = TestPki::new();

    let server_config = rustls::ServerConfig::builder_with_details(
        esp_rustls_provider::provider().into(),
        Arc::new(time_provider),
    )
    .with_safe_default_protocol_versions()
    .unwrap()
    .with_no_client_auth()
    .with_single_cert(alloc::vec![pki.server_cert_der], pki.server_key_der)
    .unwrap();

    let server_config = alloc::sync::Arc::new(server_config);

    println!("Open https://{}:4443/", stack.get_ip_info().unwrap().ip);

    socket.listen(4443).unwrap();

    loop {
        socket.work();

        if !socket.is_open() {
            socket.listen(4443).unwrap();
        }

        if socket.is_connected() {
            println!("Connected");

            let tls = ServerConnection::new(
                server_config.clone(),
                socket,
                &mut incoming_tls,
                &mut outgoing_tls,
                &mut plaintext_in,
                &mut plaintext_out,
            );

            match tls {
                Ok(mut tls) => {
                    let mut time_out = false;
                    let mut err = false;
                    let mut got_data = false;
                    let deadline = esp_hal::time::now() + Duration::secs(4);
                    let mut buffer = [0u8; 1024];
                    let mut pos = 0;
                    loop {
                        let r = tls.read(&mut buffer[pos..]);
                        if let Ok(len) = r {
                            if len > 0 {
                                got_data = true;
                            }

                            let to_print =
                                unsafe { core::str::from_utf8_unchecked(&buffer[..(pos + len)]) };

                            if to_print.contains("\r\n\r\n") {
                                print!("{}", to_print);
                                println!();
                                break;
                            }

                            pos += len;

                            if len == 0 && got_data {
                                break;
                            }
                        } else {
                            println!("{:?}", r);
                            err = true;
                            break;
                        }

                        if esp_hal::time::now() > deadline {
                            println!("Timeout");
                            time_out = true;
                            break;
                        }
                    }

                    if !time_out && !err {
                        tls.write_all(
                            b"HTTP/1.0 200 OK\r\n\r\n\
                            <html>\
                                <body>\
                                    <h1>Hello Rust! Hello Rustls!</h1>\
                                    <img src=\"https://rustacean.net/more-crabby-things/dancing-ferris.gif\"/>
                                </body>\
                            </html>\r\n\
                            "
                        ).ok();

                        tls.flush().ok();
                    }

                    socket = tls.free();
                }
                Err((s, _)) => {
                    socket = s;
                }
            }

            let deadline = esp_hal::time::now() + Duration::secs(1);
            while esp_hal::time::now() < deadline {
                socket.work();
            }

            socket.disconnect();
            socket.close();

            println!("Done\n");
            println!();
        }
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

struct TestPki {
    server_cert_der: rustls::pki_types::CertificateDer<'static>,
    server_key_der: rustls::pki_types::PrivateKeyDer<'static>,
}

impl TestPki {
    fn new() -> Self {
        static CERT: &[u8] = &[
            48u8, 130, 1, 138, 48, 130, 1, 47, 160, 3, 2, 1, 2, 2, 20, 80, 89, 193, 6, 153, 91, 16,
            212, 128, 110, 89, 53, 108, 183, 139, 150, 172, 160, 238, 160, 48, 10, 6, 8, 42, 134,
            72, 206, 61, 4, 3, 2, 48, 55, 49, 19, 48, 17, 6, 3, 85, 4, 3, 12, 10, 69, 120, 97, 109,
            112, 108, 101, 32, 67, 65, 49, 32, 48, 30, 6, 3, 85, 4, 10, 12, 23, 80, 114, 111, 118,
            105, 100, 101, 114, 32, 83, 101, 114, 118, 101, 114, 32, 69, 120, 97, 109, 112, 108,
            101, 48, 32, 23, 13, 55, 53, 48, 49, 48, 49, 48, 48, 48, 48, 48, 48, 90, 24, 15, 52,
            48, 57, 54, 48, 49, 48, 49, 48, 48, 48, 48, 48, 48, 90, 48, 33, 49, 31, 48, 29, 6, 3,
            85, 4, 3, 12, 22, 114, 99, 103, 101, 110, 32, 115, 101, 108, 102, 32, 115, 105, 103,
            110, 101, 100, 32, 99, 101, 114, 116, 48, 89, 48, 19, 6, 7, 42, 134, 72, 206, 61, 2, 1,
            6, 8, 42, 134, 72, 206, 61, 3, 1, 7, 3, 66, 0, 4, 242, 150, 10, 198, 8, 151, 136, 40,
            123, 104, 14, 246, 178, 151, 176, 193, 229, 222, 187, 216, 154, 223, 176, 221, 103, 87,
            75, 171, 64, 29, 30, 70, 47, 34, 93, 94, 18, 94, 19, 252, 33, 12, 249, 145, 104, 6, 75,
            46, 111, 147, 253, 99, 206, 64, 83, 126, 244, 34, 54, 112, 83, 87, 5, 18, 163, 45, 48,
            43, 48, 20, 6, 3, 85, 29, 17, 4, 13, 48, 11, 130, 9, 108, 111, 99, 97, 108, 104, 111,
            115, 116, 48, 19, 6, 3, 85, 29, 37, 4, 12, 48, 10, 6, 8, 43, 6, 1, 5, 5, 7, 3, 1, 48,
            10, 6, 8, 42, 134, 72, 206, 61, 4, 3, 2, 3, 73, 0, 48, 70, 2, 33, 0, 216, 8, 139, 33,
            131, 48, 10, 188, 169, 173, 34, 67, 18, 208, 37, 151, 185, 90, 170, 248, 53, 82, 136,
            220, 192, 89, 23, 88, 152, 107, 64, 171, 2, 33, 0, 142, 29, 9, 154, 225, 201, 34, 174,
            248, 51, 216, 50, 205, 46, 103, 243, 155, 190, 125, 115, 95, 8, 45, 40, 72, 146, 113,
            74, 226, 15, 56, 215,
        ];

        static KEY: &[u8] = &[
            48u8, 129, 135, 2, 1, 0, 48, 19, 6, 7, 42, 134, 72, 206, 61, 2, 1, 6, 8, 42, 134, 72,
            206, 61, 3, 1, 7, 4, 109, 48, 107, 2, 1, 1, 4, 32, 238, 131, 128, 95, 255, 63, 243, 16,
            105, 169, 88, 28, 10, 118, 255, 188, 38, 9, 49, 102, 178, 232, 146, 94, 165, 117, 24,
            179, 118, 197, 4, 77, 161, 68, 3, 66, 0, 4, 242, 150, 10, 198, 8, 151, 136, 40, 123,
            104, 14, 246, 178, 151, 176, 193, 229, 222, 187, 216, 154, 223, 176, 221, 103, 87, 75,
            171, 64, 29, 30, 70, 47, 34, 93, 94, 18, 94, 19, 252, 33, 12, 249, 145, 104, 6, 75, 46,
            111, 147, 253, 99, 206, 64, 83, 126, 244, 34, 54, 112, 83, 87, 5, 18,
        ];

        Self {
            server_cert_der: rustls::pki_types::CertificateDer::from(CERT),
            server_key_der: rustls::pki_types::PrivateKeyDer::Pkcs8(
                rustls::pki_types::PrivatePkcs8KeyDer::from(KEY),
            ),
        }
    }
}
