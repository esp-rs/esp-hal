//! Run a test of download, upload and download+upload in async fashion.
//!
//! A prerequisite to running the benchmark examples is to run the benchmark server on your local machine. Simply run the following commands to do so.
//! ```
//! cd extras/bench-server
//! cargo run --release
//! ```
//! Ensure you have set the IP of your local machine in the `HOST_IP` env variable. E.g `HOST_IP="192.168.0.24"` and also set SSID and PASSWORD env variable before running this example.
//!
//! Because of the huge task-arena size configured this won't work on ESP32-S2 and ESP32-C2
//!

//% FEATURES: embassy embassy-generic-timers esp-wifi esp-wifi/async esp-wifi/embassy-net esp-wifi/wifi-default esp-wifi/wifi esp-wifi/utils
//% CHIPS: esp32 esp32s2 esp32s3 esp32c3 esp32c6

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_net::{tcp::TcpSocket, Ipv4Address, Stack, StackResources};
use embassy_time::{with_timeout, Duration, Timer};
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{prelude::*, rng::Rng, timer::timg::TimerGroup};
use esp_println::println;
use esp_wifi::{
    initialize,
    wifi::{
        ClientConfiguration,
        Configuration,
        WifiController,
        WifiDevice,
        WifiEvent,
        WifiStaDevice,
        WifiState,
    },
    EspWifiInitFor,
};

// When you are okay with using a nightly compiler it's better to use https://docs.rs/static_cell/2.1.0/static_cell/macro.make_static.html
macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

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

// static buffers to not need a huge task-arena
static mut RX_BUFFER: [u8; RX_BUFFER_SIZE] = [0; RX_BUFFER_SIZE];
static mut TX_BUFFER: [u8; TX_BUFFER_SIZE] = [0; TX_BUFFER_SIZE];

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) -> ! {
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init({
        let mut config = esp_hal::Config::default();
        config.cpu_clock = CpuClock::max();
        config
    });

    static mut HEAP: core::mem::MaybeUninit<[u8; 32 * 1024]> = core::mem::MaybeUninit::uninit();

    #[link_section = ".dram2_uninit"]
    static mut HEAP2: core::mem::MaybeUninit<[u8; 64 * 1024]> = core::mem::MaybeUninit::uninit();

    unsafe {
        esp_alloc::HEAP.add_region(esp_alloc::HeapRegion::new(
            HEAP.as_mut_ptr() as *mut u8,
            core::mem::size_of_val(&*core::ptr::addr_of!(HEAP)),
            esp_alloc::MemoryCapability::Internal.into(),
        ));

        // add some more RAM
        esp_alloc::HEAP.add_region(esp_alloc::HeapRegion::new(
            HEAP2.as_mut_ptr() as *mut u8,
            core::mem::size_of_val(&*core::ptr::addr_of!(HEAP2)),
            esp_alloc::MemoryCapability::Internal.into(),
        ));
    }

    let server_address: Ipv4Address = HOST_IP.parse().expect("Invalid HOST_IP address");

    let timg0 = TimerGroup::new(peripherals.TIMG0);

    let init = init(
        EspWifiInitFor::Wifi,
        timg0.timer0,
        Rng::new(peripherals.RNG),
        peripherals.RADIO_CLK,
    )
    .unwrap();

    let wifi = peripherals.WIFI;
    let (wifi_interface, controller) =
        esp_wifi::wifi::new_with_mode(&init, wifi, WifiStaDevice).unwrap();

    cfg_if::cfg_if! {
        if #[cfg(feature = "esp32")] {
            let timg1 = TimerGroup::new(peripherals.TIMG1);
            esp_hal_embassy::init(timg1.timer0);
        } else {
            use esp_hal::timer::systimer::{SystemTimer, Target};
            let systimer = SystemTimer::new(peripherals.SYSTIMER).split::<Target>();
            esp_hal_embassy::init(systimer.alarm0);
        }
    }

    let config = embassy_net::Config::dhcpv4(Default::default());

    let seed = 1234; // very random, very secure seed

    // Init network stack
    let stack = &*mk_static!(
        Stack<WifiDevice<'_, WifiStaDevice>>,
        Stack::new(
            wifi_interface,
            config,
            mk_static!(StackResources<3>, StackResources::<3>::new()),
            seed
        )
    );

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

    let mut socket = TcpSocket::new(
        stack,
        unsafe { &mut *core::ptr::addr_of_mut!(RX_BUFFER) },
        unsafe { &mut *core::ptr::addr_of_mut!(TX_BUFFER) },
    );

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
async fn net_task(stack: &'static Stack<WifiDevice<'static, WifiStaDevice>>) {
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
