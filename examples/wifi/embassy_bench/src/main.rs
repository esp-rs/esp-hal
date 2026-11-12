//! Wi-Fi TCP throughput benchmark: download, upload, and both at the same time.
//!
//! Run the benchmark server on a computer in the same network:
//! ```
//! cd extras/bench-server
//! cargo run --release
//! ```
//! Set `HOST_IP` to the address of that computer, for example `HOST_IP="192.168.0.24"`, and set
//! `SSID` and `PASSWORD`.
//!
//! On chips with a small instruction cache, the example places code that runs for each Wi-Fi frame
//! in RAM. `ESP_HAL_CONFIG_USE_RWTEXT_LD_HOOK` makes esp-hal's linker script include
//! `rwtext_hook.x`. `build.rs` makes this file for the selected chip from `rwtext_heap.x` and
//! `rwtext_os.x`.
//!
//! On the ESP32-S2, set `ESP_HAL_CONFIG_INSTRUCTION_CACHE_SIZE=16KB` and
//! `ESP_HAL_CONFIG_INSTRUCTION_CACHE_LINE_SIZE=16B`. With the default 8 KB instruction cache, the
//! throughput is 25 % to 55 % lower. `.cargo/config.toml` cannot set these values, because
//! they also apply to the ESP32-S3.

//% CHIP_FILTER: wifi_driver_supported

#![allow(static_mut_refs)]
#![no_std]
#![no_main]

use core::net::Ipv4Addr;

use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_net::{Runner, StackResources, tcp::TcpSocket};
use embassy_time::{Duration, Timer, with_timeout};
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{clock::CpuClock, ram, rng::Rng, timer::timg::TimerGroup};
use esp_println::println;
use esp_radio::wifi::{
    AuthenticationMethodConfig,
    Config,
    ControllerConfig,
    Interface,
    WifiController,
    sta::StationConfig,
};
esp_bootloader_esp_idf::esp_app_desc!();

// Full `.dram2_uninit` region for this chip.
const DRAM2_UNINIT_SIZE: usize = {
    let range: core::ops::Range<usize> = esp_metadata_generated::memory_range!("DRAM2_UNINIT");
    range.end - range.start
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

static mut RX_BUFFER: [u8; RX_BUFFER_SIZE] = [0; RX_BUFFER_SIZE];
static mut TX_BUFFER: [u8; TX_BUFFER_SIZE] = [0; TX_BUFFER_SIZE];

struct WifiBuffers {
    static_rx: u8,
    dynamic_rx: u16,
    dynamic_tx: u16,
    rx_ba_win: u8,
}

// Values from `sdkconfig.defaults.<chip>` of ESP-IDF's `examples/wifi/iperf`, so the results are
// comparable with an ESP-IDF build of the same benchmark.
const WIFI_BUFFERS: WifiBuffers = cfg_select! {
    feature = "esp32" => WifiBuffers {
        static_rx: 16,
        dynamic_rx: 64,
        dynamic_tx: 64,
        rx_ba_win: 32,
    },
    feature = "esp32c2" => WifiBuffers {
        static_rx: 14,
        dynamic_rx: 14,
        dynamic_tx: 18,
        rx_ba_win: 14,
    },
    feature = "esp32c3" => WifiBuffers {
        static_rx: 20,
        dynamic_rx: 40,
        dynamic_tx: 40,
        rx_ba_win: 32,
    },
    feature = "esp32c5" => WifiBuffers {
        static_rx: 20,
        dynamic_rx: 48,
        dynamic_tx: 48,
        rx_ba_win: 22,
    },
    feature = "esp32c6" => WifiBuffers {
        static_rx: 40,
        dynamic_rx: 60,
        dynamic_tx: 40,
        rx_ba_win: 32,
    },
    feature = "esp32c61" => WifiBuffers {
        static_rx: 14,
        dynamic_rx: 28,
        dynamic_tx: 30,
        rx_ba_win: 20,
    },
    feature = "esp32s2" => WifiBuffers {
        static_rx: 8,
        dynamic_rx: 24,
        dynamic_tx: 24,
        rx_ba_win: 16,
    },
    feature = "esp32s3" => WifiBuffers {
        static_rx: 16,
        dynamic_rx: 64,
        dynamic_tx: 64,
        rx_ba_win: 32,
    },
    feature = "esp32s31" => WifiBuffers {
        static_rx: 48,
        dynamic_rx: 72,
        dynamic_tx: 64,
        rx_ba_win: 64,
    },
    _ => compile_error!("no Wi-Fi buffer settings for this chip"),
};

// Static RX buffers are allocated at init and take about 1.6 KB each. The rest covers the driver
// and the dynamic buffers in flight. The reclaimed DRAM2 region covers part of the total.
const HEAP_SIZE: usize = {
    let total = 128 * 1024 + WIFI_BUFFERS.static_rx as usize * 1600;
    let main = total.saturating_sub(DRAM2_UNINIT_SIZE);
    let main = if main > 32 * 1024 { main } else { 32 * 1024 };
    main - CODE_IN_RAM_RESERVE
};

// On these chips, `build.rs` places code in RAM (see `rwtext_heap.x` and `rwtext_os.x`). The main
// stack needs at least 8 KB, so the main heap is smaller. On the ESP32-S2, the value also leaves
// room for the 16 KB instruction cache.
const CODE_IN_RAM_RESERVE: usize = cfg_select! {
    feature = "esp32c2" => 8 * 1024,
    feature = "esp32s2" => 24 * 1024,
    _ => 0,
};

#[esp_hal::main]
async fn main(spawner: Spawner) -> ! {
    esp_println::logger::init_logger_from_env();
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(size: HEAP_SIZE);
    // add some more RAM
    esp_alloc::heap_allocator!(#[ram(reclaimed)] size: DRAM2_UNINIT_SIZE);

    let server_address: Ipv4Addr = HOST_IP.parse().expect("Invalid HOST_IP address");

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    let station_config = Config::Station(
        StationConfig::default()
            .with_ssid(SSID.try_into().unwrap())
            .with_authentication(AuthenticationMethodConfig::Wpa2Personal(
                PASSWORD.try_into().unwrap(),
            )),
    );

    // Each queued frame holds a driver buffer, so a longer queue gains nothing.
    let controller_config = ControllerConfig::default()
        .with_initial_config(station_config)
        .with_static_rx_buf_num(WIFI_BUFFERS.static_rx)
        .with_dynamic_rx_buf_num(WIFI_BUFFERS.dynamic_rx)
        .with_dynamic_tx_buf_num(WIFI_BUFFERS.dynamic_tx)
        .with_rx_ba_win(WIFI_BUFFERS.rx_ba_win)
        .with_rx_queue_size(usize::from(WIFI_BUFFERS.dynamic_rx).min(32))
        .with_tx_queue_size(usize::from(WIFI_BUFFERS.dynamic_tx).min(32));

    println!("Starting wifi");
    let wifi_interface = esp_radio::wifi::Interface::station();
    let mut controller =
        esp_radio::wifi::WifiController::new(peripherals.WIFI, controller_config).unwrap();
    println!("Wifi configured and started!");

    controller
        .set_power_saving(esp_radio::wifi::PowerSaveMode::None)
        .unwrap();

    let config = embassy_net::Config::dhcpv4(Default::default());

    let rng = Rng::new();
    let seed = (rng.random() as u64) << 32 | rng.random() as u64;

    // Init network stack
    let (stack, runner) = embassy_net::new(
        wifi_interface,
        config,
        mk_static!(StackResources<3>, StackResources::<3>::new()),
        seed,
    );

    spawner.spawn(connection(controller).unwrap());
    spawner.spawn(net_task(runner).unwrap());

    stack.wait_config_up().await;

    if let Some(config) = stack.config_v4() {
        println!("Got IP: {}", config.address);
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

    loop {
        println!("About to connect...");

        match controller.connect_async().await {
            Ok(info) => {
                println!("Wifi connected to {:?}", info);

                // wait until we're no longer connected
                let info = controller.wait_for_disconnect_async().await.ok();
                println!("Disconnected: {:?}", info);
            }
            Err(e) => {
                println!("Failed to connect to wifi: {e:?}");
            }
        }

        Timer::after(Duration::from_millis(5000)).await
    }
}

#[embassy_executor::task]
async fn net_task(mut runner: Runner<'static, Interface>) {
    runner.run().await
}

async fn test_download(server_address: Ipv4Addr, socket: &mut TcpSocket<'_>) -> usize {
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

async fn test_upload(server_address: Ipv4Addr, socket: &mut TcpSocket<'_>) -> usize {
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

async fn test_upload_download(server_address: Ipv4Addr, socket: &mut TcpSocket<'_>) -> usize {
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
