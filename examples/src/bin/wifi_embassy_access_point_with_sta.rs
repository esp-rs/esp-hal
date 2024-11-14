//! Embassy access point with station
//!
//! Set SSID and PASSWORD env variable before running the example.
//!
//! - gets an ip address via DHCP
//! - creates an open access-point with SSID `esp-wifi`
//! - you can connect to it using a static IP in range 192.168.2.2 .. 192.168.2.255, gateway 192.168.2.1
//! - open http://192.168.2.1:8080/ in your browser - the example will perform an HTTP get request to some "random" server
//!
//! On Android you might need to choose _Keep Accesspoint_ when it tells you the WiFi has no internet connection, Chrome might not want to load the URL - you can use a shell and try `curl` and `ping`
//!
//! Because of the huge task-arena size configured this won't work on ESP32-S2
//!

//% FEATURES: embassy embassy-generic-timers esp-wifi esp-wifi/wifi esp-wifi/utils
//% CHIPS: esp32 esp32s2 esp32s3 esp32c2 esp32c3 esp32c6

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_net::{
    tcp::TcpSocket,
    IpListenEndpoint,
    Ipv4Address,
    Ipv4Cidr,
    Stack,
    StackResources,
    StaticConfigV4,
};
use embassy_time::{Duration, Timer};
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{prelude::*, rng::Rng, timer::timg::TimerGroup};
use esp_println::{print, println};
use esp_wifi::{
    init,
    wifi::{
        AccessPointConfiguration,
        ClientConfiguration,
        Configuration,
        WifiApDevice,
        WifiController,
        WifiDevice,
        WifiEvent,
        WifiStaDevice,
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
    let peripherals = esp_hal::init({
        let mut config = esp_hal::Config::default();
        config.cpu_clock = CpuClock::max();
        config
    });

    esp_alloc::heap_allocator!(72 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);

    let init = &*mk_static!(
        EspWifiController<'static>,
        init(
            timg0.timer0,
            Rng::new(peripherals.RNG),
            peripherals.RADIO_CLK,
        )
        .unwrap()
    );

    let wifi = peripherals.WIFI;
    let (wifi_ap_interface, wifi_sta_interface, mut controller) =
        esp_wifi::wifi::new_ap_sta(&init, wifi).unwrap();

    cfg_if::cfg_if! {
        if #[cfg(feature = "esp32")] {
            let timg1 = TimerGroup::new(peripherals.TIMG1);
            esp_hal_embassy::init(timg1.timer0);
        } else {
            use esp_hal::timer::systimer::SystemTimer;
            let systimer = SystemTimer::new(peripherals.SYSTIMER).split();
            esp_hal_embassy::init(systimer.alarm0);
        }
    }

    let ap_config = embassy_net::Config::ipv4_static(StaticConfigV4 {
        address: Ipv4Cidr::new(Ipv4Address::new(192, 168, 2, 1), 24),
        gateway: Some(Ipv4Address::from_bytes(&[192, 168, 2, 1])),
        dns_servers: Default::default(),
    });
    let sta_config = embassy_net::Config::dhcpv4(Default::default());

    let seed = 1234; // very random, very secure seed

    // Init network stacks
    let ap_stack = &*mk_static!(
        Stack<WifiDevice<'_, WifiApDevice>>,
        Stack::new(
            wifi_ap_interface,
            ap_config,
            mk_static!(StackResources<3>, StackResources::<3>::new()),
            seed
        )
    );
    let sta_stack = &*mk_static!(
        Stack<WifiDevice<'_, WifiStaDevice>>,
        Stack::new(
            wifi_sta_interface,
            sta_config,
            mk_static!(StackResources<3>, StackResources::<3>::new()),
            seed
        )
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
    spawner.spawn(ap_task(&ap_stack)).ok();
    spawner.spawn(sta_task(&sta_stack)).ok();

    loop {
        if sta_stack.is_link_up() {
            break;
        }
        println!("Waiting for IP...");
        Timer::after(Duration::from_millis(500)).await;
    }
    loop {
        if ap_stack.is_link_up() {
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }
    println!("Connect to the AP `esp-wifi` and point your browser to http://192.168.2.1:8080/");
    println!("Use a static IP in the range 192.168.2.2 .. 192.168.2.255, use gateway 192.168.2.1");

    let mut ap_rx_buffer = [0; 1536];
    let mut ap_tx_buffer = [0; 1536];

    let mut ap_socket = TcpSocket::new(&ap_stack, &mut ap_rx_buffer, &mut ap_tx_buffer);
    ap_socket.set_timeout(Some(embassy_time::Duration::from_secs(10)));

    let mut sta_rx_buffer = [0; 1536];
    let mut sta_tx_buffer = [0; 1536];

    let mut sta_socket = TcpSocket::new(&sta_stack, &mut sta_rx_buffer, &mut sta_tx_buffer);
    sta_socket.set_timeout(Some(embassy_time::Duration::from_secs(10)));

    loop {
        println!("Wait for connection...");
        let r = ap_socket
            .accept(IpListenEndpoint {
                addr: None,
                port: 8080,
            })
            .await;
        println!("Connected...");

        if let Err(e) = r {
            println!("connect error: {:?}", e);
            continue;
        }

        use embedded_io_async::Write;

        let mut buffer = [0u8; 1024];
        let mut pos = 0;
        loop {
            match ap_socket.read(&mut buffer).await {
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
            let remote_endpoint = (Ipv4Address::new(142, 250, 185, 115), 80);
            println!("connecting...");
            let r = sta_socket.connect(remote_endpoint).await;
            if let Err(e) = r {
                println!("STA connect error: {:?}", e);
                continue;
            }

            use embedded_io_async::Write;
            let r = sta_socket
                .write_all(b"GET / HTTP/1.0\r\nHost: www.mobile-j.de\r\n\r\n")
                .await;

            if let Err(e) = r {
                println!("STA write error: {:?}", e);

                let r = ap_socket
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
                let r = sta_socket.flush().await;
                if let Err(e) = r {
                    println!("STA flush error: {:?}", e);
                } else {
                    println!("connected!");
                    let mut buf = [0; 1024];
                    loop {
                        match sta_socket.read(&mut buf).await {
                            Ok(0) => {
                                println!("STA read EOF");
                                break;
                            }
                            Ok(n) => {
                                let r = ap_socket.write_all(&buf[..n]).await;
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

            sta_socket.close();
        } else {
            let r = ap_socket
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

        let r = ap_socket.flush().await;
        if let Err(e) = r {
            println!("AP flush error: {:?}", e);
        }
        Timer::after(Duration::from_millis(1000)).await;

        ap_socket.close();
        Timer::after(Duration::from_millis(1000)).await;

        ap_socket.abort();
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

#[embassy_executor::task]
async fn ap_task(stack: &'static Stack<WifiDevice<'static, WifiApDevice>>) {
    stack.run().await
}

#[embassy_executor::task]
async fn sta_task(stack: &'static Stack<WifiDevice<'static, WifiStaDevice>>) {
    stack.run().await
}
