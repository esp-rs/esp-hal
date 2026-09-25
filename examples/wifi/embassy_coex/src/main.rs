//! Embassy COEX Example
//!
//! - set SSID and PASSWORD env variable
//! - gets an ip address via DHCP
//! - performs an HTTP get request to some "random" server
//! - does BLE advertising and allows to connect
//!
//! The example also shows how to save power. Change these constants to compare:
//!
//! - `POWER_SAVE` selects the Wi-Fi modem power save mode. With power save, the radio is off
//!   between beacons.
//! - `MODEM_SLEEP` lets the BLE controller turn the radio off between its events.
//! - `LIGHT_SLEEP` lets the chip enter automatic light sleep when all tasks are idle. The chip
//!   sleeps only while both radios allow it. On the ESP32, Wi-Fi keeps the chip awake.
//! - `CPU_POWERDOWN` lets light sleep power the CPU down, and retains its state in RAM. This saves
//!   more current, but it makes the sleep and the wake slower. Only the ESP32-C3, -C5, -C6, -C61,
//!   -S3 and -S31 support this. On other chips the constant has no effect.
//!
//! The USB Serial/JTAG console stops while the chip is in light sleep. Use the UART port to see
//! the output.

//% CHIP_FILTER: wifi_driver_supported && bt_driver_supported

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_futures::{join::join, select::select};
use embassy_net::{
    Runner,
    StackResources,
    dns::DnsSocket,
    tcp::client::{TcpClient, TcpClientState},
};
use embassy_time::{Duration, Timer};
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{
    clock::{ClockConfig, CpuClock},
    ram,
    rng::Rng,
    timer::timg::TimerGroup,
};
use esp_println::println;
use esp_radio::{
    ble::controller::BleConnector,
    wifi::{
        AuthenticationMethodConfig,
        Config,
        ControllerConfig,
        Interface,
        PowerSaveMode,
        WifiController,
        scan::ScanConfig,
        sta::StationConfig,
    },
};
use reqwless::{
    client::HttpClient,
    request::{Method, RequestBuilder},
};
use trouble_host::prelude::*;
esp_bootloader_esp_idf::esp_app_desc!();

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

/// The Wi-Fi modem power save mode. [`PowerSaveMode::None`] keeps the radio on and prevents
/// light-sleep.
const POWER_SAVE: PowerSaveMode = PowerSaveMode::Minimum;
/// Whether the BLE controller turns the radio off between its events.
const MODEM_SLEEP: bool = true;
/// Whether the chip enters automatic light sleep when all tasks are idle. Requires [`POWER_SAVE`]
/// and [`MODEM_SLEEP`] to be enabled.
const LIGHT_SLEEP: bool = true;
/// Whether light sleep powers the CPU down. Requires [`LIGHT_SLEEP`] to be enabled.
const CPU_POWERDOWN: bool = true;

// An example reads no chip capability, so this condition lists the chips that support CPU
// power-down.
cfg_select! {
    any(
        feature = "esp32c3",
        feature = "esp32c5",
        feature = "esp32c6",
        feature = "esp32c61",
        feature = "esp32s3",
        feature = "esp32s31",
    ) => {
        use esp_hal::rtc_cntl::CpuRetentionStorage;

        // The memory that the bootloader used is otherwise unused after boot.
        #[ram(reclaimed, unstable(zeroed))]
        static CPU_RETENTION_MEMORY: CpuRetentionStorage = CpuRetentionStorage::new();

        // Keeping the cache tags makes the wake faster, because the cache stays warm.
        #[cfg(feature = "esp32s3")]
        #[ram(reclaimed, unstable(zeroed))]
        static CACHE_TAGMEM: esp_hal::rtc_cntl::CacheTagRetentionStorage =
            esp_hal::rtc_cntl::CacheTagRetentionStorage::new();

        /// The reclaimed RAM that CPU power-down takes from the heap. The 16 bytes per buffer
        /// cover its alignment.
        const CPU_POWERDOWN_RAM: usize = size_of::<CpuRetentionStorage>()
            + 16
            + cfg_select! {
                feature = "esp32s3" => size_of::<esp_hal::rtc_cntl::CacheTagRetentionStorage>() + 16,
                _ => 0,
            };

        fn enable_cpu_powerdown(sleep: &mut esp_rtos::sleep::Sleep) {
            sleep
                .enable_cpu_powerdown(CPU_RETENTION_MEMORY.take())
                .unwrap();

            #[cfg(feature = "esp32s3")]
            sleep.keep_cache_tags(CACHE_TAGMEM.take()).unwrap();
        }
    }
    _ => {
        const CPU_POWERDOWN_RAM: usize = 0;

        fn enable_cpu_powerdown(_sleep: &mut esp_rtos::sleep::Sleep) {}
    }
}

/// Max number of connections
const CONNECTIONS_MAX: usize = 1;
/// Max number of L2CAP channels.
const L2CAP_CHANNELS_MAX: usize = 2; // Signal + att

// GATT Server definition
#[gatt_server]
struct Server {
    battery_service: BatteryService,
}

/// Battery service
#[gatt_service(uuid = service::BATTERY)]
struct BatteryService {
    /// Battery Level
    #[descriptor(uuid = descriptors::VALID_RANGE, read, value = [0, 100])]
    #[descriptor(uuid = descriptors::MEASUREMENT_DESCRIPTION, name = "hello", read, value = "Battery Level", type = &'static str)]
    #[characteristic(uuid = characteristic::BATTERY_LEVEL, read, notify, value = 10)]
    level: u8,
    #[characteristic(uuid = "408813df-5dd4-1f87-ec11-cdb001100000", write, read, notify)]
    status: bool,
}

#[esp_hal::main]
async fn main(spawner: Spawner) -> ! {
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock({
        #[cfg_attr(feature = "esp32c2", allow(unused_mut))]
        let mut config = ClockConfig::from(CpuClock::max());

        #[cfg(not(feature = "esp32c2"))]
        {
            use esp_hal::clock::ll::BleLpClkConfig;

            // For now, only Xtal can be selected if modem-sleep is enabled.
            // This is our default anyway, a safe choice even in light sleep,
            // although it can raise the sleep current a bit.
            config.ble_lp_clk = Some(BleLpClkConfig::Xtal);
        }

        config
    }));

    // COEX needs more RAM - add some more
    #[cfg(feature = "esp32")]
    {
        esp_alloc::heap_allocator!(#[ram(reclaimed)] size: 96 * 1024);
        esp_alloc::heap_allocator!(size: 24 * 1024);
    }
    #[cfg(not(feature = "esp32"))]
    {
        esp_alloc::heap_allocator!(#[ram(reclaimed)] size: 64 * 1024 - CPU_POWERDOWN_RAM);
        esp_alloc::heap_allocator!(size: 64 * 1024 + CPU_POWERDOWN_RAM);
    }

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    if LIGHT_SLEEP {
        let mut sleep = esp_rtos::sleep::configure(peripherals.LPWR);
        if CPU_POWERDOWN {
            enable_cpu_powerdown(&mut sleep);
        }
        esp_rtos::start_with_idle_hook(timg0.timer0, sleep.light_sleep_hook);
    } else {
        esp_rtos::start(timg0.timer0);
    }

    let connector = BleConnector::new(
        peripherals.BT,
        esp_radio::ble::Config::default().with_modem_sleep(MODEM_SLEEP),
    )
    .unwrap();
    let ble_controller: ExternalController<_, 1> = ExternalController::new(connector);

    let station_config = Config::Station(
        StationConfig::default()
            .with_ssid(SSID.try_into().unwrap())
            .with_authentication(AuthenticationMethodConfig::Wpa2Personal(
                PASSWORD.try_into().unwrap(),
            )),
    );

    println!("Starting wifi");
    let wifi_interface = esp_radio::wifi::Interface::station();
    let mut controller = esp_radio::wifi::WifiController::new(
        peripherals.WIFI,
        ControllerConfig::default().with_initial_config(station_config),
    )
    .unwrap();
    controller.set_power_saving(POWER_SAVE).unwrap();
    println!("Wifi started!");

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

    println!("Scan");
    let scan_config = ScanConfig::default().with_max(10);
    let result = controller.scan_async(&scan_config).await.unwrap();
    for ap in result {
        println!("{:?}", ap);
    }

    spawner.spawn(ble_task(ble_controller).unwrap());
    spawner.spawn(connection(controller).unwrap());
    spawner.spawn(net_task(runner).unwrap());

    stack.wait_config_up().await;
    if let Some(config) = stack.config_v4() {
        println!("Got IP: {}", config.address);
    }

    // Init HTTP client
    let tcp_client = TcpClient::new(
        stack,
        mk_static!(
            TcpClientState<1, 1500, 1500>,
            TcpClientState::<1, 1500, 1500>::new()
        ),
    );
    let dns_client = DnsSocket::new(stack);

    loop {
        Timer::after(Duration::from_millis(1000)).await;

        let mut client = HttpClient::new(&tcp_client, &dns_client);
        let mut rx_buf = [0u8; 4096];

        let builder = client
            .request(Method::GET, "http://httpbin.org/get?hello=Hello+esp-hal")
            .await
            .unwrap();

        let mut builder = builder.headers(&[("Host", "httpbin.org"), ("Connection", "close")]);

        let response = builder.send(&mut rx_buf).await.unwrap();

        match response.body().read_to_end().await {
            Ok(data) => {
                if let Ok(st) = core::str::from_utf8(data) {
                    println!("Body: {}", st);
                }
            }
            Err(e) => println!("Body error: {:?}", e),
        }
        Timer::after(Duration::from_millis(3000)).await;
    }
}

#[embassy_executor::task]
pub async fn ble_task(controller: ExternalController<BleConnector<'static>, 1>) {
    let address = Address::random([0xff, 0xe4, 0x05, 0x1a, 0x8f, 0xff]);

    println!("Our address = {:?}", address);

    let mut resources: HostResources<_, DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX> =
        HostResources::new();
    let stack = trouble_host::new(controller, &mut resources)
        .set_random_address(address)
        .build();
    let mut peripheral = stack.peripheral();
    let mut runner = stack.runner();

    let mut adv_data = [0; 31];
    let adv_len = AdStructure::encode_slice(
        &[
            AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
            AdStructure::CompleteLocalName(esp_hal::chip!().as_bytes()),
        ],
        &mut adv_data[..],
    )
    .unwrap();

    println!("Starting advertising and GATT service");
    let server = Server::new_with_config(GapConfig::Peripheral(PeripheralConfig {
        name: "TrouBLE",
        appearance: &appearance::power_device::GENERIC_POWER_DEVICE,
    }))
    .unwrap();

    let _ = join(runner.run(), async {
        let mut params = AdvertisementParameters::default();
        params.interval_min = Duration::from_millis(100);
        params.interval_max = Duration::from_millis(100);

        loop {
            match peripheral
                .advertise(
                    &params,
                    Advertisement::ConnectableScannableUndirected {
                        adv_data: &adv_data[..adv_len],
                        scan_data: &[],
                    },
                )
                .await
            {
                Ok(adv) => {
                    match adv.accept().await.unwrap().with_attribute_server(&server) {
                        Ok(conn) => {
                            println!("got connection");
                            let a = gatt_events_task(&server, &conn);
                            let b = custom_task(&server, &conn, &stack);
                            // run until any task ends (usually because the connection has been
                            // closed), then return to advertising
                            // state.
                            select(a, b).await;
                        }
                        Err(err) => println!("Error occurred: {:?}", err),
                    }
                }
                Err(e) => {
                    panic!("[adv] error: {:?}", e);
                }
            }
        }
    })
    .await;
}

/// Stream Events until the connection closes.
///
/// This function will handle the GATT events and process them.
/// This is how we interact with read and write requests.
async fn gatt_events_task<P: PacketPool>(
    server: &Server<'_>,
    conn: &GattConnection<'_, '_, P>,
) -> Result<(), Error> {
    let level = server.battery_service.level;
    let reason = loop {
        match conn.next().await {
            GattConnectionEvent::Disconnected { reason } => break reason,
            GattConnectionEvent::Gatt { event } => {
                match &event {
                    GattEvent::Read(event) => {
                        if event.handle() == level.handle {
                            let value = server.get(&level);
                            println!("[gatt] Read Event to Level Characteristic: {:?}", value);
                        }
                    }
                    GattEvent::Write(event) => {
                        if event.handle() == level.handle {
                            event.with_data(|offset, data| {
                                println!(
                                    "[gatt] Write Event to Level Characteristic at {}: {:?}",
                                    offset, data
                                )
                            });
                        }
                    }
                    _ => {}
                };
                // This step is also performed at drop(), but writing it explicitly is necessary
                // in order to ensure reply is sent.
                match event.accept() {
                    Ok(reply) => reply.send().await,
                    Err(e) => println!("[gatt] error sending response: {:?}", e),
                };
            }
            _ => {} // ignore other Gatt Connection Events
        }
    };
    println!("[gatt] disconnected: {:?}", reason);
    Ok(())
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

/// Example task to use the BLE notifier interface.
/// This task will notify the connected central of a counter value every 2 seconds.
/// It will also read the RSSI value every 2 seconds.
/// and will stop when the connection is closed by the central or an error occurs.
async fn custom_task<C: Controller, P: PacketPool>(
    server: &Server<'_>,
    conn: &GattConnection<'_, '_, P>,
    stack: &Stack<'_, C, P>,
) {
    let mut tick: u8 = 0;
    let level = server.battery_service.level;
    loop {
        tick = tick.wrapping_add(1);
        println!("[custom_task] notifying connection of tick {}", tick);
        if level.notify(conn, &tick, true).await.is_err() {
            println!("[custom_task] error notifying connection");
            break;
        };
        // read RSSI (Received Signal Strength Indicator) of the connection.
        if let Ok(rssi) = conn.raw().rssi(stack).await {
            println!("[custom_task] RSSI: {:?}", rssi);
        } else {
            println!("[custom_task] error getting RSSI");
            break;
        };
        Timer::after_secs(2).await;
    }
}
