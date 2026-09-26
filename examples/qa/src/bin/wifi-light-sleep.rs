//! Wi-Fi station in power save with automatic light sleep.
//!
//! Set SSID and PASSWORD env variable before running this example.
//!
//! The station connects with `PowerSaveMode::Minimum`, and the chip light-sleeps between beacons.
//! Every 30 seconds, the example opens a TCP connection to port 80 of the gateway, which needs both
//! receive and transmit to work. It counts beacon timeouts and disconnects. A disconnect with
//! reason 200 is a beacon timeout.
//!
//! Use the UART port. USB Serial/JTAG stops when the chip sleeps.

//% FEATURES: esp-radio esp-radio/wifi esp-radio/unstable esp-hal/unstable
//% CHIP_FILTER: wifi_driver_supported && sleep_light_sleep

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_net::{IpEndpoint, Runner, StackResources, tcp::TcpSocket};
use embassy_time::{Duration, Instant, Timer, with_timeout};
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{ram, rng::Rng, timer::timg::TimerGroup};
use esp_println::println;
use esp_radio::wifi::{
    AuthenticationMethodConfig,
    Config,
    ControllerConfig,
    Interface,
    PowerSaveMode,
    WifiController,
    event::{EventInfo, WifiEvent, enable_wifi_events},
    sta::StationConfig,
};

esp_bootloader_esp_idf::esp_app_desc!();

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

const PROBE_INTERVAL: Duration = Duration::from_secs(30);

#[esp_hal::main]
async fn main(spawner: Spawner) -> ! {
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init(esp_hal::Config::default());

    esp_alloc::heap_allocator!(#[ram(reclaimed)] size: 64 * 1024);
    esp_alloc::heap_allocator!(size: 36 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sleep = esp_rtos::sleep::configure(peripherals.LPWR);
    esp_rtos::start_with_idle_hook(timg0.timer0, sleep.light_sleep_hook);

    let station_config = Config::Station(
        StationConfig::default()
            .with_ssid(SSID.try_into().unwrap())
            .with_authentication(AuthenticationMethodConfig::Wpa2Personal(
                PASSWORD.try_into().unwrap(),
            )),
    );

    let wifi_interface = Interface::station();
    let mut controller = WifiController::new(
        peripherals.WIFI,
        ControllerConfig::default().with_initial_config(station_config),
    )
    .unwrap();
    controller.set_power_saving(PowerSaveMode::Minimum).unwrap();
    enable_wifi_events(WifiEvent::StationBeaconTimeout.into());

    let rng = Rng::new();
    let seed = (rng.random() as u64) << 32 | rng.random() as u64;

    let (stack, runner) = embassy_net::new(
        wifi_interface,
        embassy_net::Config::dhcpv4(Default::default()),
        mk_static!(StackResources<3>, StackResources::<3>::new()),
        seed,
    );

    spawner.spawn(connection(controller).unwrap());
    spawner.spawn(net_task(runner).unwrap());
    stack.wait_config_up().await;
    let config = stack.config_v4().unwrap();
    let gateway = config.gateway.unwrap();
    println!("Got IP {}, gateway {}", config.address, gateway);

    let mut rx_buffer = [0u8; 1536];
    let mut tx_buffer = [0u8; 1536];
    let mut probes = 0u32;
    let mut failures = 0u32;

    loop {
        Timer::after(PROBE_INTERVAL).await;

        let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
        let started = Instant::now();
        let result = with_timeout(
            Duration::from_secs(5),
            socket.connect(IpEndpoint::new(gateway.into(), 80)),
        )
        .await;
        let elapsed = started.elapsed().as_millis();
        socket.abort();
        let _ = socket.flush().await;

        probes += 1;
        let ok = matches!(result, Ok(Ok(())));
        if !ok {
            failures += 1;
        }
        println!(
            "[{} s] probe {}: {} in {} ms, {} failed so far",
            Instant::now().as_secs(),
            probes,
            if ok { "connected" } else { "failed" },
            elapsed,
            failures,
        );
    }
}

#[embassy_executor::task]
async fn connection(mut controller: WifiController<'static>) {
    let mut beacon_timeouts = 0u32;
    let mut disconnects = 0u32;

    loop {
        match controller.connect_async().await {
            Ok(info) => {
                println!("Connected to {:?}", info);

                // Subscribe only while connected. A subscriber that nobody reads fills the event
                // channel, and `connect_async` then misses its events.
                let mut events = controller.subscribe().unwrap();
                loop {
                    match events.next_event_pure().await {
                        EventInfo::StationBeaconTimeout => {
                            beacon_timeouts += 1;
                            println!("Beacon timeout, {beacon_timeouts} in total");
                        }
                        EventInfo::StationDisconnected { reason, .. } => {
                            disconnects += 1;
                            println!("Disconnected, reason {reason}, {disconnects} in total");
                            break;
                        }
                        _ => {}
                    }
                }
            }
            Err(e) => println!("Failed to connect: {e:?}"),
        }

        Timer::after(Duration::from_secs(5)).await;
    }
}

#[embassy_executor::task]
async fn net_task(mut runner: Runner<'static, Interface>) {
    runner.run().await
}
