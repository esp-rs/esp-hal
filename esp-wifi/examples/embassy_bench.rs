#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_net::tcp::TcpSocket;
use embassy_net::{Config, Ipv4Address, Stack, StackResources};
#[path = "../../examples-util/util.rs"]
mod examples_util;
use examples_util::hal;

use embassy_time::{with_timeout, Duration, Timer};
use embedded_svc::wifi::{ClientConfiguration, Configuration, Wifi};
use esp_backtrace as _;
use esp_println::println;
use esp_wifi::wifi::{WifiApDevice, WifiController, WifiDevice, WifiEvent, WifiState};
use esp_wifi::{initialize, EspWifiInitFor};
use hal::clock::ClockControl;
use hal::Rng;
use hal::{embassy, peripherals::Peripherals, prelude::*, timer::TimerGroup};
use static_cell::make_static;

const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");
const HOST_IP: &str = env!("HOST_IP");

const TEST_DURATION: usize = 15;
const RX_BUFFER_SIZE: usize = 16384;
const TX_BUFFER_SIZE: usize = 16384;
const IO_BUFFER_SIZE: usize = 1024;
const DOWNLOAD_PORT: u16 = 4321;
const UPLOAD_PORT: u16 = 4322;
const UPLOAD_DOWNLOAD_PORT: u16 = 4323;

#[main]
async fn main(spawner: Spawner) -> ! {
    #[cfg(feature = "log")]
    esp_println::logger::init_logger(log::LevelFilter::Info);

    let peripherals = Peripherals::take();

    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::max(system.clock_control).freeze();

    let server_address: Ipv4Address = HOST_IP.parse().expect("Invalid HOST_IP address");

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
    let (wifi_interface, controller) =
        esp_wifi::wifi::new_with_mode(&init, wifi, WifiApDevice).unwrap();

    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    embassy::init(&clocks, timer_group0);

    let config = Config::dhcpv4(Default::default());

    let seed = 1234; // very random, very secure seed

    // Init network stack
    let stack = &*make_static!(Stack::new(
        wifi_interface,
        config,
        make_static!(StackResources::<3>::new()),
        seed
    ));

    spawner.spawn(connection(controller)).ok();
    spawner.spawn(net_task(&stack)).ok();

    loop {
        if stack.is_link_up() {
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }

    println!("Waiting to get IP address...");
    loop {
        if let Some(config) = stack.config_v4() {
            println!("Got IP: {}", config.address);
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }

    let mut rx_buffer = [0; RX_BUFFER_SIZE];
    let mut tx_buffer = [0; TX_BUFFER_SIZE];
    let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);

    loop {
        let _down = test_download(server_address, &mut socket).await;
        let _up = test_upload(server_address, &mut socket).await;
        let _updown = test_upload_download(server_address, &mut socket).await;

        Timer::after(Duration::from_millis(10000)).await;
    }
}

#[embassy_executor::task]
async fn connection(mut controller: WifiController<'static>) {
    println!("start connection task");
    println!("Device capabilities: {:?}", controller.get_capabilities());
    loop {
        match esp_wifi::wifi::get_wifi_state() {
            WifiState::StaConnected => {
                // wait until we're no longer connected
                controller.wait_for_event(WifiEvent::StaDisconnected).await;
                Timer::after(Duration::from_millis(5000)).await
            }
            _ => {}
        }
        if !matches!(controller.is_started(), Ok(true)) {
            let client_config = Configuration::Client(ClientConfiguration {
                ssid: SSID.try_into().unwrap(),
                password: PASSWORD.try_into().unwrap(),
                ..Default::default()
            });
            controller.set_configuration(&client_config).unwrap();
            println!("Starting wifi");
            controller.start().await.unwrap();
            println!("Wifi started!");
        }
        println!("About to connect...");

        match controller.connect().await {
            Ok(_) => println!("Wifi connected!"),
            Err(e) => {
                println!("Failed to connect to wifi: {e:?}");
                Timer::after(Duration::from_millis(5000)).await
            }
        }
    }
}

#[embassy_executor::task]
async fn net_task(stack: &'static Stack<WifiDevice<'static, WifiApDevice>>) {
    stack.run().await
}

async fn test_download(server_address: Ipv4Address, socket: &mut TcpSocket<'_>) -> usize {
    println!("Testing download...");

    socket.abort();
    socket.set_timeout(Some(Duration::from_secs(10)));

    println!("connecting to {:?}:{}...", server_address, DOWNLOAD_PORT);
    if let Err(e) = socket.connect((server_address, DOWNLOAD_PORT)).await {
        println!("connect error: {:?}", e);
        return 0;
    }
    println!("connected, testing...");

    let mut buf = [0; IO_BUFFER_SIZE];
    let mut total: usize = 0;
    with_timeout(Duration::from_secs(TEST_DURATION as _), async {
        loop {
            match socket.read(&mut buf).await {
                Ok(0) => {
                    println!("read EOF");
                    return 0;
                }
                Ok(n) => total += n,
                Err(e) => {
                    println!("read error: {:?}", e);
                    return 0;
                }
            }
        }
    })
    .await
    .ok();

    let kbps = (total + 512) / 1024 / TEST_DURATION;
    println!("download: {} kB/s", kbps);
    kbps
}

async fn test_upload(server_address: Ipv4Address, socket: &mut TcpSocket<'_>) -> usize {
    println!("Testing upload...");
    socket.abort();
    socket.set_timeout(Some(Duration::from_secs(10)));

    println!("connecting to {:?}:{}...", server_address, UPLOAD_PORT);
    if let Err(e) = socket.connect((server_address, UPLOAD_PORT)).await {
        println!("connect error: {:?}", e);
        return 0;
    }
    println!("connected, testing...");

    let buf = [0; IO_BUFFER_SIZE];
    let mut total: usize = 0;
    with_timeout(Duration::from_secs(TEST_DURATION as _), async {
        loop {
            match socket.write(&buf).await {
                Ok(0) => {
                    println!("write zero?!??!?!");
                    return 0;
                }
                Ok(n) => total += n,
                Err(e) => {
                    println!("write error: {:?}", e);
                    return 0;
                }
            }
        }
    })
    .await
    .ok();

    let kbps = (total + 512) / 1024 / TEST_DURATION;
    println!("upload: {} kB/s", kbps);
    kbps
}

async fn test_upload_download(server_address: Ipv4Address, socket: &mut TcpSocket<'_>) -> usize {
    println!("Testing upload+download...");

    socket.abort();
    socket.set_timeout(Some(Duration::from_secs(10)));

    println!(
        "connecting to {:?}:{}...",
        server_address, UPLOAD_DOWNLOAD_PORT
    );
    if let Err(e) = socket.connect((server_address, UPLOAD_DOWNLOAD_PORT)).await {
        println!("connect error: {:?}", e);
        return 0;
    }
    println!("connected, testing...");

    let (mut reader, mut writer) = socket.split();

    let tx_buf = [0; IO_BUFFER_SIZE];
    let mut rx_buf = [0; IO_BUFFER_SIZE];
    let mut total: usize = 0;
    let tx_fut = async {
        loop {
            match writer.write(&tx_buf).await {
                Ok(0) => {
                    println!("write zero?!??!?!");
                    return 0;
                }
                Ok(_) => {}
                Err(e) => {
                    println!("write error: {:?}", e);
                    return 0;
                }
            }
        }
    };

    let rx_fut = async {
        loop {
            match reader.read(&mut rx_buf).await {
                Ok(0) => {
                    println!("read EOF");
                    return 0;
                }
                Ok(n) => total += n,
                Err(e) => {
                    println!("read error: {:?}", e);
                    return 0;
                }
            }
        }
    };

    with_timeout(
        Duration::from_secs(TEST_DURATION as _),
        join(tx_fut, rx_fut),
    )
    .await
    .ok();

    let kbps = (total + 512) / 1024 / TEST_DURATION;
    println!("upload+download: {} kB/s", kbps);
    kbps
}
