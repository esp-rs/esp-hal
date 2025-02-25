//! Embassy access point with station
//!
//! Set SSID and PASSWORD env variable before running the example.
//!
//! - gets an ip address via DHCP
//! - creates an open access-point with SSID `esp-wifi`
//! - if you either:
//!   - connect to it using a static IP in range 192.168.2.2 .. 192.168.2.255, gateway 192.168.2.1
//!   - open http://192.168.2.1:8080/ in your browser
//! - or:
//!   - connect to the network referenced by the SSID env variable and open the IP address printed by the example
//! - the example will perform an HTTP get request to some "random" server and return the response
//!
//! On Android you might need to choose _Keep Accesspoint_ when it tells you the WiFi has no internet connection, Chrome might not want to load the URL - you can use a shell and try `curl` and `ping`
//!

//% FEATURES: embassy esp-wifi esp-wifi/wifi esp-hal/unstable
//% CHIPS: esp32 esp32s2 esp32s3 esp32c2 esp32c3 esp32c6

#![no_std]
#![no_main]

use core::net::Ipv4Addr;

use embassy_executor::Spawner;
use embassy_futures::select::Either;
use embassy_net::{
    tcp::TcpSocket,
    IpListenEndpoint,
    Ipv4Cidr,
    Runner,
    StackResources,
    StaticConfigV4,
};
use embassy_time::{Duration, Timer};
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{clock::CpuClock, rng::Rng, timer::timg::TimerGroup};
use esp_println::{print, println};
use esp_wifi::{
    init,
    wifi::{
        AccessPointConfiguration,
        ClientConfiguration,
        Configuration,
        WifiController,
        WifiDevice,
        WifiEvent,
        WifiState,
    },
    EspWifiController,
};

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

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) -> ! {
    esp_println::logger::init_logger_from_env();
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(size: 72 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let mut rng = Rng::new(peripherals.RNG);

    let esp_wifi_ctrl = &*mk_static!(
        EspWifiController<'static>,
        init(timg0.timer0, rng.clone(), peripherals.RADIO_CLK).unwrap()
    );

    let (mut controller, interfaces) =
        esp_wifi::wifi::new(&esp_wifi_ctrl, peripherals.WIFI).unwrap();

    let wifi_ap_device = interfaces.ap;
    let wifi_sta_device = interfaces.sta;

    cfg_if::cfg_if! {
        if #[cfg(feature = "esp32")] {
            let timg1 = TimerGroup::new(peripherals.TIMG1);
            esp_hal_embassy::init(timg1.timer0);
        } else {
            use esp_hal::timer::systimer::SystemTimer;
            let systimer = SystemTimer::new(peripherals.SYSTIMER);
            esp_hal_embassy::init(systimer.alarm0);
        }
    }

    let ap_config = embassy_net::Config::ipv4_static(StaticConfigV4 {
        address: Ipv4Cidr::new(Ipv4Addr::new(192, 168, 2, 1), 24),
        gateway: Some(Ipv4Addr::new(192, 168, 2, 1)),
        dns_servers: Default::default(),
    });
    let sta_config = embassy_net::Config::dhcpv4(Default::default());

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
    controller.set_configuration(&client_config).unwrap();

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
    loop {
        if ap_stack.is_link_up() {
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }
    println!("Connect to the AP `esp-wifi` and point your browser to http://192.168.2.1:8080/");
    println!("Use a static IP in the range 192.168.2.2 .. 192.168.2.255, use gateway 192.168.2.1");
    println!("Or connect to the ap `{SSID}` and point your browser to http://{sta_address}:8080/");

    let mut ap_server_rx_buffer = [0; 1536];
    let mut ap_server_tx_buffer = [0; 1536];
    let mut sta_server_rx_buffer = [0; 1536];
    let mut sta_server_tx_buffer = [0; 1536];
    let mut sta_client_rx_buffer = [0; 1536];
    let mut sta_client_tx_buffer = [0; 1536];

    let mut ap_server_socket =
        TcpSocket::new(ap_stack, &mut ap_server_rx_buffer, &mut ap_server_tx_buffer);
    ap_server_socket.set_timeout(Some(embassy_time::Duration::from_secs(10)));

    let mut sta_server_socket = TcpSocket::new(
        sta_stack,
        &mut sta_server_rx_buffer,
        &mut sta_server_tx_buffer,
    );
    sta_server_socket.set_timeout(Some(embassy_time::Duration::from_secs(10)));

    let mut sta_client_socket = TcpSocket::new(
        sta_stack,
        &mut sta_client_rx_buffer,
        &mut sta_client_tx_buffer,
    );
    sta_client_socket.set_timeout(Some(embassy_time::Duration::from_secs(10)));

    loop {
        println!("Wait for connection...");
        // FIXME: If connections are attempted on both sockets at the same time, we might end up
        // dropping one of them. Might be better to spawn both accept() calls, or use fused futures?
        // Note that we only attempt to serve one connection at a time, so we don't run out of ram.
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
            let remote_endpoint = (Ipv4Addr::new(142, 250, 185, 115), 80);
            println!("connecting...");
            let r = sta_client_socket.connect(remote_endpoint).await;
            if let Err(e) = r {
                println!("STA connect error: {:?}", e);
                continue;
            }

            use embedded_io_async::Write;
            let r = sta_client_socket
                .write_all(b"GET / HTTP/1.0\r\nHost: www.mobile-j.de\r\n\r\n")
                .await;

            if let Err(e) = r {
                println!("STA write error: {:?}", e);

                let r = server_socket
                    .write_all(
                        b"HTTP/1.0 500 Internal Server Error\r\n\r\n\
                        <html>\
                            <body>\
                                <h1>Hello Rust! Hello esp-wifi! STA failed to send request.</h1>\
                            </body>\
                        </html>\r\n\
                        ",
                    )
                    .await;
                if let Err(e) = r {
                    println!("AP write error: {:?}", e);
                }
            } else {
                let r = sta_client_socket.flush().await;
                if let Err(e) = r {
                    println!("STA flush error: {:?}", e);
                } else {
                    println!("connected!");
                    let mut buf = [0; 1024];
                    loop {
                        match sta_client_socket.read(&mut buf).await {
                            Ok(0) => {
                                println!("STA read EOF");
                                break;
                            }
                            Ok(n) => {
                                let r = server_socket.write_all(&buf[..n]).await;
                                if let Err(e) = r {
                                    println!("AP write error: {:?}", e);
                                    break;
                                }
                            }
                            Err(e) => {
                                println!("STA read error: {:?}", e);
                                break;
                            }
                        }
                    }
                }
            }

            sta_client_socket.close();
        } else {
            let r = server_socket
                .write_all(
                    b"HTTP/1.0 200 OK\r\n\r\n\
                    <html>\
                        <body>\
                            <h1>Hello Rust! Hello esp-wifi! STA is not connected.</h1>\
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
    println!("Device capabilities: {:?}", controller.capabilities());

    println!("Starting wifi");
    controller.start_async().await.unwrap();
    println!("Wifi started!");

    loop {
        match esp_wifi::wifi::ap_state() {
            WifiState::ApStarted => {
                println!("About to connect...");

                match controller.connect_async().await {
                    Ok(_) => {
                        // wait until we're no longer connected
                        controller.wait_for_event(WifiEvent::StaDisconnected).await;
                        println!("STA disconnected");
                    }
                    Err(e) => {
                        println!("Failed to connect to wifi: {e:?}");
                        Timer::after(Duration::from_millis(5000)).await
                    }
                }
            }
            _ => return,
        }
    }
}

#[embassy_executor::task(pool_size = 2)]
async fn net_task(mut runner: Runner<'static, WifiDevice<'static>>) {
    runner.run().await
}
