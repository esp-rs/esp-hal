//! Embassy access point with station
//!
//! Set SSID and PASSWORD env variable before running the example.
//!
//! - gets an ip address via DHCP
//! - creates an open access-point with SSID `esp-radio`
//! - if you either:
//!   - connect to it using a static IP in range 192.168.2.2 .. 192.168.2.255, gateway 192.168.2.1
//!   - open http://192.168.2.1:8080/ in your browser
//! - or:
//!   - connect to the network referenced by the SSID env variable and open the IP address printed
//!     by the example
//! - the example will perform an HTTP get request to some "random" server and return the response
//!
//! On Android you might need to choose _Keep Accesspoint_ when it tells you the
//! WiFi has no internet connection, Chrome might not want to load the URL - you
//! can use a shell and try `curl` and `ping`

#![no_std]
#![no_main]

use core::net::Ipv4Addr;

use embassy_executor::Spawner;
use embassy_futures::select::Either;
use embassy_net::{
    IpListenEndpoint,
    Ipv4Cidr,
    Runner,
    StackResources,
    StaticConfigV4,
    dns::DnsSocket,
    tcp::{
        TcpSocket,
        client::{TcpClient, TcpClientState},
    },
};
use embassy_time::{Duration, Timer};
use embedded_io_async::Read;
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    interrupt::software::SoftwareInterruptControl,
    ram,
    rng::Rng,
    timer::timg::TimerGroup,
};
use esp_println::{print, println};
use esp_radio::wifi::{
    Config,
    ControllerConfig,
    Interface,
    WifiController,
    ap::AccessPointConfig,
    sta::StationConfig,
};
use reqwless::{
    client::HttpClient,
    request::{Method, RequestBuilder},
};
esp_bootloader_esp_idf::esp_app_desc!();

const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");

// When you are okay with using a nightly compiler it's better to use https://docs.rs/static_cell/2.1.0/static_cell/macro.make_static.html
macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    esp_println::logger::init_logger_from_env();
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(#[ram(reclaimed)] size: 64 * 1024);
    esp_alloc::heap_allocator!(size: 36 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    let access_point_station_config = Config::AccessPointStation(
        StationConfig::default()
            .with_ssid(SSID)
            .with_password(PASSWORD.into()),
        AccessPointConfig::default().with_ssid("esp-radio-apsta"),
    );

    println!("Starting wifi");
    let (controller, interfaces) = esp_radio::wifi::new(
        peripherals.WIFI,
        ControllerConfig::default().with_initial_config(access_point_station_config),
    )
    .unwrap();
    println!("Wifi started!");

    let wifi_ap_device = interfaces.access_point;
    let wifi_sta_device = interfaces.station;

    let ap_config = embassy_net::Config::ipv4_static(StaticConfigV4 {
        address: Ipv4Cidr::new(Ipv4Addr::new(192, 168, 2, 1), 24),
        gateway: Some(Ipv4Addr::new(192, 168, 2, 1)),
        dns_servers: Default::default(),
    });
    let sta_config = embassy_net::Config::dhcpv4(Default::default());

    let rng = Rng::new();
    let seed = (rng.random() as u64) << 32 | rng.random() as u64;

    // Init network stacks
    let (ap_stack, ap_runner) = embassy_net::new(
        wifi_ap_device,
        ap_config,
        mk_static!(StackResources<3>, StackResources::<3>::new()),
        seed,
    );
    let (sta_stack, sta_runner) = embassy_net::new(
        wifi_sta_device,
        sta_config,
        mk_static!(StackResources<4>, StackResources::<4>::new()),
        seed,
    );

    spawner.spawn(connection(controller)).ok();
    spawner.spawn(net_task(ap_runner)).ok();
    spawner.spawn(net_task(sta_runner)).ok();

    let sta_address = loop {
        if let Some(config) = sta_stack.config_v4() {
            let address = config.address.address();
            println!("Got IP: {}", address);
            break address;
        }
        println!("Waiting for IP...");
        Timer::after(Duration::from_millis(500)).await;
    };
    ap_stack.wait_config_up().await;

    println!(
        "Connect to the AP `esp-radio-apsta` and point your browser to http://192.168.2.1:8080/"
    );
    println!("Use a static IP in the range 192.168.2.2 .. 192.168.2.255, use gateway 192.168.2.1");
    println!("Or connect to the ap `{SSID}` and point your browser to http://{sta_address}:8080/");

    // Init HTTP client
    let tcp_client = TcpClient::new(
        sta_stack,
        mk_static!(
            TcpClientState<1, 1500, 1500>,
            TcpClientState::<1, 1500, 1500>::new()
        ),
    );
    let dns_client = DnsSocket::new(sta_stack);

    let mut ap_server_rx_buffer = [0; 1536];
    let mut ap_server_tx_buffer = [0; 1536];
    let mut sta_server_rx_buffer = [0; 1536];
    let mut sta_server_tx_buffer = [0; 1536];

    let mut ap_server_socket =
        TcpSocket::new(ap_stack, &mut ap_server_rx_buffer, &mut ap_server_tx_buffer);
    ap_server_socket.set_timeout(Some(embassy_time::Duration::from_secs(10)));

    let mut sta_server_socket = TcpSocket::new(
        sta_stack,
        &mut sta_server_rx_buffer,
        &mut sta_server_tx_buffer,
    );
    sta_server_socket.set_timeout(Some(embassy_time::Duration::from_secs(10)));

    loop {
        println!("Wait for connection...");
        // FIXME: If connections are attempted on both sockets at the same time, we
        // might end up dropping one of them. Might be better to spawn both
        // accept() calls, or use fused futures? Note that we only attempt to
        // serve one connection at a time, so we don't run out of ram.
        let either_socket = embassy_futures::select::select(
            ap_server_socket.accept(IpListenEndpoint {
                addr: None,
                port: 8080,
            }),
            sta_server_socket.accept(IpListenEndpoint {
                addr: None,
                port: 8080,
            }),
        )
        .await;
        let (r, server_socket) = match either_socket {
            Either::First(r) => (r, &mut ap_server_socket),
            Either::Second(r) => (r, &mut sta_server_socket),
        };
        println!("Connected...");

        if let Err(e) = r {
            println!("connect error: {:?}", e);
            continue;
        }

        use embedded_io_async::Write;

        let mut buffer = [0u8; 1024];
        let mut pos = 0;
        loop {
            match server_socket.read(&mut buffer).await {
                Ok(0) => {
                    println!("AP read EOF");
                    break;
                }
                Ok(len) => {
                    let to_print =
                        unsafe { core::str::from_utf8_unchecked(&buffer[..(pos + len)]) };

                    if to_print.contains("\r\n\r\n") {
                        print!("{}", to_print);
                        println!();
                        break;
                    }

                    pos += len;
                }
                Err(e) => {
                    println!("AP read error: {:?}", e);
                    break;
                }
            };
        }
        if sta_stack.is_link_up() {
            println!("connecting via HttpClient...");
            let mut client = HttpClient::new(&tcp_client, &dns_client);
            let mut rx_buf = [0u8; 4096];

            let builder_result = client
                .request(Method::GET, "http://httpbin.org/get?hello=Hello+esp-hal")
                .await;

            match builder_result {
                Ok(req_builder) => {
                    let headers = [("Host", "httpbin.org"), ("Connection", "close")];
                    let mut req_builder = req_builder.headers(&headers);
                    let response_result = req_builder.send(&mut rx_buf).await;

                    match response_result {
                        Ok(response) => {
                            println!("HTTP request successful, streaming body...");

                            let _ = server_socket.write_all(b"HTTP/1.0 200 OK\r\n").await;
                            let _ = server_socket
                                .write_all(b"Content-Type: application/json\r\n")
                                .await;
                            let _ = server_socket.write_all(b"Connection: close\r\n\r\n").await;

                            let mut body_reader = response.body().reader();
                            let mut chunk_buf = [0u8; 1024];

                            loop {
                                match body_reader.read(&mut chunk_buf).await {
                                    Ok(0) => break,
                                    Ok(n) => {
                                        if let Err(e) =
                                            server_socket.write_all(&chunk_buf[..n]).await
                                        {
                                            println!("AP write error: {:?}", e);
                                            break;
                                        }
                                    }
                                    Err(e) => {
                                        println!("Body read error: {:?}", e);
                                        break;
                                    }
                                }
                            }
                        }
                        Err(e) => {
                            println!("Station request error: {:?}", e);
                            let _ = server_socket
                                .write_all(
                                    b"HTTP/1.0 500 Internal Server Error\r\n\r\nRequest failed",
                                )
                                .await;
                        }
                    }
                }
                Err(e) => {
                    println!("DNS/Connect error: {:?}", e);
                    let _ = server_socket
                        .write_all(b"HTTP/1.0 500 Internal Server Error\r\n\r\nDNS Error")
                        .await;
                }
            }
        } else {
            let r = server_socket
                .write_all(
                    b"HTTP/1.0 200 OK\r\n\r\n\
                    <html>\
                        <body>\
                            <h1>Hello Rust! Hello esp-radio! Station is not connected.</h1>\
                        </body>\
                    </html>\r\n\
                    ",
                )
                .await;
            if let Err(e) = r {
                println!("AP write error: {:?}", e);
            }
        }
        let r = server_socket.flush().await;
        if let Err(e) = r {
            println!("AP flush error: {:?}", e);
        }
        Timer::after(Duration::from_millis(1000)).await;
        server_socket.close();
        Timer::after(Duration::from_millis(1000)).await;
        server_socket.abort();
    }
}

#[embassy_executor::task]
async fn connection(mut controller: WifiController<'static>) {
    println!("start connection task");

    loop {
        match controller.connect_async().await {
            Ok(_) => {
                // wait until we're no longer connected
                loop {
                    let info = embassy_futures::select::select(
                        controller.wait_for_disconnect_async(),
                        controller.wait_for_access_point_connected_event_async(),
                    )
                    .await;

                    match info {
                        Either::First(station_disconnected) => {
                            if let Ok(station_disconnected) = station_disconnected {
                                println!("Station disconnected: {:?}", station_disconnected);
                                break;
                            }
                        }
                        Either::Second(event) => {
                            if let Ok(event) = event {
                                match event {
                                    esp_radio::wifi::AccessPointStationEventInfo::Connected(
                                        access_point_station_connected_info,
                                    ) => {
                                        println!(
                                            "Station connected: {:?}",
                                            access_point_station_connected_info
                                        );
                                    }
                                    esp_radio::wifi::AccessPointStationEventInfo::Disconnected(
                                        access_point_station_disconnected_info,
                                    ) => {
                                        println!(
                                            "Station disconnected: {:?}",
                                            access_point_station_disconnected_info
                                        );
                                    }
                                }
                            }
                        }
                    }
                }
            }
            Err(e) => {
                println!("Failed to connect to wifi: {e:?}");
                Timer::after(Duration::from_millis(5000)).await
            }
        }
    }
}

#[embassy_executor::task(pool_size = 2)]
async fn net_task(mut runner: Runner<'static, Interface<'static>>) {
    runner.run().await
}
