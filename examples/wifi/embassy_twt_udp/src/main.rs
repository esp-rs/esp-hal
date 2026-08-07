//% CHIP_FILTER: wifi_has_wifi6
//! Embassy iTWT + UDP Voice-over-WiFi Example
//!
//! Set SSID, PASSWORD, and TARGET_IP env variables before running this
//! example (defaults are in `.cargo/config.toml`).
//!
//! Simulates a voice-over-WiFi scenario: connects to a Wi-Fi 6 AP,
//! negotiates a 20ms iTWT wake interval, and sends a 160-byte UDP
//! packet (simulating a G.711 voice frame) on every TWT wakeup.
//!
//! The AP must support 802.11ax (Wi-Fi 6) for TWT to work.
//!
//! Listen for packets with: `socat UDP-LISTEN:4444,fork,reuseaddr -`

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_net::{Runner, StackResources, udp::UdpSocket};
use embassy_time::Timer;
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    interrupt::software::SoftwareInterruptControl,
    ram,
    rng::Rng,
    time::Duration,
    timer::timg::TimerGroup,
};
use esp_println::println;
use esp_radio::wifi::{
    Config,
    ControllerConfig,
    Interface,
    PowerSaveMode,
    Protocol,
    Protocols,
    sta::StationConfig,
    twt::{ITwtSetupConfig, TwtConfig},
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
const TARGET_IP: &str = env!("TARGET_IP");
const UDP_PORT: u16 = 4444;

#[esp_hal::main]
async fn main(spawner: Spawner) -> ! {
    esp_println::logger::init_logger_from_env();
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(#[ram(reclaimed)] size: 64 * 1024);
    esp_alloc::heap_allocator!(size: 36 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    let station_config = Config::Station(
        StationConfig::default()
            .with_ssid(SSID)
            .with_password(PASSWORD.into())
            .with_protocols(Protocols::default().with_2_4(Protocol::AX.into())),
    );

    println!("Starting wifi");
    let wifi_interface = esp_radio::wifi::Interface::station();
    let mut controller = esp_radio::wifi::WifiController::new(
        peripherals.WIFI,
        ControllerConfig::default().with_initial_config(station_config),
    )
    .unwrap();

    let rng = Rng::new();
    let seed = (rng.random() as u64) << 32 | rng.random() as u64;

    let (stack, runner) = embassy_net::new(
        wifi_interface,
        embassy_net::Config::dhcpv4(Default::default()),
        mk_static!(StackResources<3>, StackResources::<3>::new()),
        seed,
    );

    spawner.spawn(net_task(runner).unwrap());

    loop {
        println!("Connecting...");
        match controller.connect_async().await {
            Ok(info) => {
                println!("Wifi connected: {:?}", info);
                break;
            }
            Err(e) => {
                println!("Failed to connect: {e:?}, retrying in 5s");
                Timer::after(embassy_time::Duration::from_secs(5)).await;
            }
        }
    }

    stack.wait_config_up().await;
    if let Some(config) = stack.config_v4() {
        println!("Got IP: {}", config.address);
    }

    controller
        .twt_config(
            &TwtConfig::default()
                .with_post_wakeup_event(true)
                .with_enable_keep_alive(true),
        )
        .unwrap();
    controller.set_power_saving(PowerSaveMode::Minimum).unwrap();

    let setup_config = ITwtSetupConfig::default()
        .with_wake_interval(Duration::from_micros(20_000))
        .with_min_wake_duration(Duration::from_micros(2048));

    println!("Requesting iTWT setup...");
    let flow_id = match controller.itwt_setup(setup_config).await {
        Ok(info) => {
            println!("iTWT setup OK: {:?}", info.config);
            info.config.flow_id
        }
        Err(e) => {
            println!("iTWT setup FAILED: {e:?}");
            loop {
                Timer::after(embassy_time::Duration::from_secs(60)).await;
            }
        }
    };

    let target: embassy_net::IpAddress = TARGET_IP.parse().unwrap();
    let target_endpoint = embassy_net::IpEndpoint::new(target, UDP_PORT);

    let mut rx_meta = [embassy_net::udp::PacketMetadata::EMPTY; 1];
    let mut rx_buf = [0u8; 1];
    let mut tx_meta = [embassy_net::udp::PacketMetadata::EMPTY; 2];
    let mut tx_buf = [0u8; 512];

    let mut socket = UdpSocket::new(stack, &mut rx_meta, &mut rx_buf, &mut tx_meta, &mut tx_buf);
    socket.bind(UDP_PORT).unwrap();

    let mut voice_frame = [0u8; 160];
    let mut wakeup_count: u32 = 0;

    loop {
        let wakeup = controller.wait_for_next_twt_wakeup(flow_id).await.unwrap();
        wakeup_count += 1;

        voice_frame[..4].copy_from_slice(&wakeup_count.to_le_bytes());

        match socket.send_to(&voice_frame, target_endpoint).await {
            Ok(()) => {
                if wakeup_count % 50 == 1 {
                    println!(
                        "#{wakeup_count}: sent 160B voice frame (type={:?}, flow={:?})",
                        wakeup.twt_type, wakeup.flow_id
                    );
                }
            }
            Err(e) => {
                println!("#{wakeup_count}: UDP send failed: {e:?}");
            }
        }
    }
}

#[embassy_executor::task]
async fn net_task(mut runner: Runner<'static, Interface>) {
    runner.run().await
}
