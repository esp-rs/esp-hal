//% CHIP_FILTER: wifi_has_wifi6
//! Embassy iTWT + UDP Voice-over-WiFi Example
//!
//! Set SSID, PASSWORD, and TARGET_IP env variables before running this
//! example (defaults are in `.cargo/config.toml`).
//!
//! Simulates a voice-over-WiFi scenario: connects to an AP, negotiates a 20ms
//! iTWT wake interval, and sends a 160-byte UDP packet (simulating a G.711
//! voice frame) on every TWT wakeup.
//!
//! The AP must support 802.11ax (Wi-Fi 6) for TWT to work. If TWT can not be
//! set up (unsupported or rejected by the AP), the example falls back to plain
//! modem power save and sends the frames on a periodic timer instead. If the
//! AP answers with different parameters, the setup is retried once with them.
//! After a disconnect, the example reconnects and sets up TWT again, because
//! TWT agreements do not survive a reconnect.
//!
//! The power save constants below work as in the `embassy_dhcp` example.
//!
//! Set the `BAND` env variable to `auto`, `2.4` or `5` to select the Wi-Fi
//! band on dual-band chips (ESP32-C5). Other chips ignore it.
//!
//! Listen for packets with: `socat UDP-LISTEN:4444,fork,reuseaddr -`

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_net::{
    IpEndpoint,
    Runner,
    Stack,
    StackResources,
    udp::{PacketMetadata, UdpSocket},
};
use embassy_time::{Duration, Ticker, Timer};
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{clock::CpuClock, ram, rng::Rng, timer::timg::TimerGroup};
use esp_println::println;
use esp_radio::wifi::{
    AuthenticationMethodConfig,
    Config,
    ControllerConfig,
    Interface,
    PowerSaveMode,
    Protocol,
    Protocols,
    WifiController,
    WifiError,
    sta::StationConfig,
    twt::{FlowId, ITwtSetupConfig, ITwtSetupError, TwtConfig, TwtSetupCommand},
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
/// One of `auto`, `2.4` or `5`. Only used on chips that support 5 GHz.
#[cfg(feature = "esp32c5")]
const BAND: &str = env!("BAND");

/// The interval between voice frames, and the requested TWT wake interval.
const FRAME_INTERVAL_MS: u64 = 20;

/// The Wi-Fi modem power save mode. [`PowerSaveMode::None`] keeps the radio on and prevents
/// light-sleep.
const POWER_SAVE: PowerSaveMode = PowerSaveMode::Minimum;
/// Whether the chip enters automatic light sleep when all tasks are idle. Requires [`POWER_SAVE`]
/// to be enabled. The chip sleeps only while the station is in power save. On the ESP32, Wi-Fi
/// keeps the chip awake.
const LIGHT_SLEEP: bool = true;
/// Whether light sleep powers the CPU down. Requires [`LIGHT_SLEEP`] to be enabled.
const CPU_POWERDOWN: bool = true;

// An example reads no chip capability, so this condition lists the chips that support CPU
// power-down.
cfg_select! {
    any(
        feature = "esp32c5",
        feature = "esp32c6",
        feature = "esp32c61",
        feature = "esp32s31",
    ) => {
        use esp_hal::rtc_cntl::CpuRetentionStorage;

        // The memory that the bootloader used is otherwise unused after boot.
        #[ram(reclaimed, unstable(zeroed))]
        static CPU_RETENTION_MEMORY: CpuRetentionStorage = CpuRetentionStorage::new();

        /// The reclaimed RAM that CPU power-down takes from the heap. The 16 bytes per buffer
        /// cover its alignment.
        const CPU_POWERDOWN_RAM: usize = size_of::<CpuRetentionStorage>() + 16;

        fn enable_cpu_powerdown(sleep: &mut esp_rtos::sleep::Sleep) {
            sleep
                .enable_cpu_powerdown(CPU_RETENTION_MEMORY.take())
                .unwrap();
        }
    }
    _ => {
        const CPU_POWERDOWN_RAM: usize = 0;

        fn enable_cpu_powerdown(_sleep: &mut esp_rtos::sleep::Sleep) {}
    }
}

/// The result of trying to set up an iTWT agreement.
enum TwtSetup {
    /// The agreement is active on this flow.
    Active(FlowId),
    /// TWT is not available with this AP; continue with plain power save.
    Unavailable,
    /// The station disconnected.
    Disconnected,
}

/// Requests an iTWT agreement. If the AP answers with its own parameters, retries once with them.
async fn setup_twt(controller: &WifiController<'_>) -> TwtSetup {
    let request = ITwtSetupConfig::default()
        .with_setup_cmd(TwtSetupCommand::Suggest)
        .with_wake_interval(esp_hal::time::Duration::from_millis(FRAME_INTERVAL_MS))
        .with_min_wake_duration(esp_hal::time::Duration::from_micros(2048));

    println!("Requesting iTWT setup: {:?}", request);
    let error = match controller.itwt_setup(request).await {
        Ok(info) => {
            println!("iTWT setup OK: {:?}", info.config);
            return TwtSetup::Active(info.config.flow_id);
        }
        Err(e) => e,
    };

    let response = match error {
        ITwtSetupError::Wifi(WifiError::NotConnected) => return TwtSetup::Disconnected,
        ITwtSetupError::Rejected(failed) => failed.config,
        e => {
            println!("iTWT setup failed: {e:?}");
            return TwtSetup::Unavailable;
        }
    };

    // The AP may counter-propose. Accept its parameters once.
    let setup_cmd = match response.setup_cmd {
        TwtSetupCommand::Alternate => TwtSetupCommand::Suggest,
        TwtSetupCommand::Dictate => TwtSetupCommand::Demand,
        cmd => {
            println!("iTWT setup rejected by the AP ({cmd:?}): {:?}", response);
            return TwtSetup::Unavailable;
        }
    };
    let retry = response.with_setup_cmd(setup_cmd);
    println!(
        "AP answered {:?}, retrying with its parameters: {:?} (was {:?})",
        response.setup_cmd, retry, request
    );

    match controller.itwt_setup(retry).await {
        Ok(info) => {
            println!("iTWT setup OK: {:?}", info.config);
            TwtSetup::Active(info.config.flow_id)
        }
        Err(ITwtSetupError::Wifi(WifiError::NotConnected)) => TwtSetup::Disconnected,
        Err(e) => {
            println!("iTWT setup retry failed: {e:?}");
            TwtSetup::Unavailable
        }
    }
}

/// Sends one voice frame. The first four bytes carry the frame counter.
async fn send_frame(socket: &UdpSocket<'_>, target: IpEndpoint, count: &mut u32) {
    let mut frame = [0u8; 160];
    *count += 1;
    frame[..4].copy_from_slice(&count.to_le_bytes());

    match socket.send_to(&frame, target).await {
        Ok(()) if *count % 50 == 1 => println!("#{count}: sent 160B voice frame"),
        Ok(()) => {}
        Err(e) => println!("#{count}: UDP send failed: {e:?}"),
    }
}

#[esp_hal::main]
async fn main(spawner: Spawner) -> ! {
    esp_println::logger::init_logger_from_env();
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(#[ram(reclaimed)] size: 64 * 1024 - CPU_POWERDOWN_RAM);
    esp_alloc::heap_allocator!(size: 36 * 1024 + CPU_POWERDOWN_RAM);

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

    let station_config = Config::Station(
        StationConfig::default()
            .with_ssid(SSID.try_into().unwrap())
            .with_authentication(AuthenticationMethodConfig::Wpa2Personal(
                PASSWORD.try_into().unwrap(),
            ))
            // The default 2.4 GHz protocols exclude 802.11ax, and without it the station
            // associates without HE and iTWT setup fails with `Unsupported`.
            .with_protocols(
                Protocols::default()
                    .with_2_4(Protocol::B | Protocol::G | Protocol::N | Protocol::AX),
            ),
    );

    println!("Starting wifi");
    let wifi_interface = esp_radio::wifi::Interface::station();
    let mut controller = esp_radio::wifi::WifiController::new(
        peripherals.WIFI,
        ControllerConfig::default().with_initial_config(station_config),
    )
    .unwrap();
    controller.set_power_saving(POWER_SAVE).unwrap();

    #[cfg(feature = "esp32c5")]
    {
        use esp_radio::wifi::BandMode;

        let band = match BAND {
            "2.4" => BandMode::_2_4G,
            "5" => BandMode::_5G,
            "auto" => BandMode::Auto,
            other => {
                println!("Unknown BAND {other:?}, using auto");
                BandMode::Auto
            }
        };
        println!("Band mode: {:?}", band);
        if let Err(e) = controller.set_band_mode(band) {
            println!("Failed to set band mode: {e:?}");
        }
    }

    let rng = Rng::new();
    let seed = (rng.random() as u64) << 32 | rng.random() as u64;

    let (stack, runner) = embassy_net::new(
        wifi_interface,
        embassy_net::Config::dhcpv4(Default::default()),
        mk_static!(StackResources<3>, StackResources::<3>::new()),
        seed,
    );
    spawner.spawn(net_task(runner).unwrap());

    let target: embassy_net::IpAddress = TARGET_IP.parse().unwrap();
    let target_endpoint = IpEndpoint::new(target, UDP_PORT);

    let mut rx_meta = [PacketMetadata::EMPTY; 1];
    let mut rx_buf = [0u8; 1];
    let mut tx_meta = [PacketMetadata::EMPTY; 2];
    let mut tx_buf = [0u8; 512];
    let mut socket = UdpSocket::new(stack, &mut rx_meta, &mut rx_buf, &mut tx_meta, &mut tx_buf);
    socket.bind(UDP_PORT).unwrap();

    let mut count: u32 = 0;

    // Connect -> DHCP -> TWT (or plain power save) -> send. Any disconnect starts over.
    loop {
        // After a torn down TWT flow the station is still connected.
        if !controller.is_connected() {
            println!("Connecting...");
            match controller.connect_async().await {
                Ok(info) => println!("Wifi connected: {:?}", info),
                Err(e) => {
                    println!("Failed to connect: {e:?}, retrying in 5s");
                    Timer::after(Duration::from_secs(5)).await;
                    continue;
                }
            }
        }

        if wait_for_ip(&controller, stack).await.is_err() {
            println!("Disconnected while waiting for an IP");
            continue;
        }

        if let Err(e) = controller.twt_config(
            &TwtConfig::default()
                .with_post_wakeup_event(true)
                .with_enable_keep_alive(true),
        ) {
            println!("TWT config failed: {e:?}");
        }

        match setup_twt(&controller).await {
            TwtSetup::Disconnected => println!("Disconnected during TWT setup"),
            TwtSetup::Active(flow_id) => loop {
                match controller.wait_for_next_twt_wakeup(flow_id).await {
                    Ok(_) => send_frame(&socket, target_endpoint, &mut count).await,
                    Err(e) => {
                        println!("TWT wakeup wait ended: {e:?}");
                        break;
                    }
                }
            },
            TwtSetup::Unavailable => {
                println!("No TWT, sending every {FRAME_INTERVAL_MS}ms on a timer");
                let mut ticker = Ticker::every(Duration::from_millis(FRAME_INTERVAL_MS));
                loop {
                    let tick = ticker.next();
                    let disconnect = controller.wait_for_disconnect_async();
                    match select(tick, disconnect).await {
                        Either::First(()) => send_frame(&socket, target_endpoint, &mut count).await,
                        Either::Second(info) => {
                            println!("Disconnected: {:?}", info.ok());
                            break;
                        }
                    }
                }
            }
        }
    }
}

/// Waits for DHCP. Fails if the station disconnects first.
async fn wait_for_ip(controller: &WifiController<'_>, stack: Stack<'_>) -> Result<(), ()> {
    let up = stack.wait_config_up();
    match select(up, controller.wait_for_disconnect_async()).await {
        Either::First(()) => {
            if let Some(config) = stack.config_v4() {
                println!("Got IP: {}", config.address);
            }
            Ok(())
        }
        Either::Second(_) => Err(()),
    }
}

#[embassy_executor::task]
async fn net_task(mut runner: Runner<'static, Interface>) {
    runner.run().await
}
