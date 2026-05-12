//% CHIP_FILTER: wifi_has_wifi6
//! Embassy iTWT (Individual Target Wake Time) Example
//!
//! Set SSID and PASSWORD env variable before running this example.
//!
//! Demonstrates multiple concurrent TWT flows: sets up three flows with
//! different intervals, tears one down, suspends another, then settles
//! into a single-flow loop with periodic TSF probes.
//!
//! The AP must support 802.11ax (Wi-Fi 6) for TWT to work.

#![no_std]
#![no_main]

use embassy_time::Timer;
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    interrupt::software::SoftwareInterruptControl,
    ram,
    time::{Duration, Instant},
    timer::timg::TimerGroup,
};

macro_rules! tprintln {
    ($($arg:tt)*) => {{
        let t = Instant::now().duration_since_epoch();
        let secs = t.as_millis() / 1000;
        let ms = t.as_millis() % 1000;
        esp_println::println!("[{:>5}.{:03}] {}", secs, ms, format_args!($($arg)*));
    }};
}
use enumset::EnumSet;
use esp_radio::wifi::{
    Config,
    ControllerConfig,
    PowerSaveMode,
    Protocol,
    Protocols,
    sta::StationConfig,
    twt::{FlowId, ITwtSetupConfig, TwtConfig},
};

esp_bootloader_esp_idf::esp_app_desc!();

const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");

fn print_flow_status(controller: &esp_radio::wifi::WifiController) {
    match controller.itwt_flow_id_status() {
        Ok(active) => tprintln!("Active flows: {:?}", active),
        Err(e) => tprintln!("Failed to get flow status: {e:?}"),
    }
}

/// Wait for a few wakeup events across all active flows.
async fn collect_wakeups(controller: &esp_radio::wifi::WifiController<'_>, count: u32) {
    for i in 0..count {
        let wakeup = controller
            .wait_for_next_twt_wakeup(EnumSet::<FlowId>::all())
            .await
            .unwrap();
        tprintln!(
            "  wakeup {}/{}: type={:?}, flow={:?}",
            i + 1,
            count,
            wakeup.twt_type,
            wakeup.flow_id
        );
    }
}

#[esp_hal::main]
async fn main(_spawner: embassy_executor::Spawner) -> ! {
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

    tprintln!("Starting wifi");
    let mut controller = esp_radio::wifi::WifiController::new(
        peripherals.WIFI,
        ControllerConfig::default().with_initial_config(station_config),
    )
    .unwrap();

    loop {
        tprintln!("Connecting...");
        match controller.connect_async().await {
            Ok(info) => {
                tprintln!("Wifi connected: {:?}", info);
                break;
            }
            Err(e) => {
                tprintln!("Failed to connect: {e:?}, retrying in 5s");
                Timer::after(embassy_time::Duration::from_secs(5)).await;
            }
        }
    }

    controller
        .twt_config(&TwtConfig::default().with_post_wakeup_event(true))
        .unwrap();
    controller.set_power_saving(PowerSaveMode::Minimum).unwrap();

    // --- Set up three flows with different intervals ---

    tprintln!("\n=== Setting up flow A (500ms interval, 65ms wake) ===");
    let flow_a = controller
        .itwt_setup(
            ITwtSetupConfig::default()
                .with_trigger(false)
                .with_wake_interval(Duration::from_millis(500))
                .with_min_wake_duration(Duration::from_millis(65)),
        )
        .await;
    match &flow_a {
        Ok(info) => tprintln!("Flow A: {:?}", info.config),
        Err(e) => tprintln!("Flow A FAILED: {e:?}"),
    }
    let flow_a_id = flow_a.as_ref().map(|i| i.config.flow_id).ok();
    print_flow_status(&controller);

    Timer::after(embassy_time::Duration::from_millis(200)).await;
    tprintln!("\n=== Setting up flow B (1s interval, 10ms wake) ===");
    let flow_b = controller
        .itwt_setup(
            ITwtSetupConfig::default()
                .with_wake_interval(Duration::from_secs(1))
                .with_min_wake_duration(Duration::from_millis(10)),
        )
        .await;
    match &flow_b {
        Ok(info) => tprintln!("Flow B: {:?}", info.config),
        Err(e) => tprintln!("Flow B FAILED: {e:?}"),
    }
    let flow_b_id = flow_b.as_ref().map(|i| i.config.flow_id).ok();
    print_flow_status(&controller);

    Timer::after(embassy_time::Duration::from_millis(200)).await;
    tprintln!("\n=== Setting up flow C (2s interval, 20ms wake) ===");
    let flow_c = controller
        .itwt_setup(
            ITwtSetupConfig::default()
                .with_wake_interval(Duration::from_secs(2))
                .with_min_wake_duration(Duration::from_millis(20)),
        )
        .await;
    match &flow_c {
        Ok(info) => tprintln!("Flow C: {:?}", info.config),
        Err(e) => tprintln!("Flow C FAILED: {e:?}"),
    }
    let flow_c_id = flow_c.as_ref().map(|i| i.config.flow_id).ok();
    print_flow_status(&controller);

    // --- Observe wakeups from all three flows ---

    tprintln!("\n=== Collecting wakeups from all three flows ===");
    collect_wakeups(&controller, 15).await;

    // --- Tear down flow B ---

    if let Some(id) = flow_b_id {
        tprintln!("\n=== Tearing down flow B ({id:?}) ===");
        match controller.itwt_teardown(id).await {
            Ok(status) => tprintln!("Teardown B: {status:?}"),
            Err(e) => tprintln!("Teardown B failed: {e:?}"),
        }
        print_flow_status(&controller);

        tprintln!("\n=== Collecting wakeups (A + C only) ===");
        collect_wakeups(&controller, 10).await;
    }

    // --- Suspend flow A for 10 seconds ---

    if let Some(id) = flow_a_id {
        tprintln!("\n=== Suspending flow A ({id:?}) for 10s ===");
        match controller.itwt_suspend(id, Duration::from_secs(10)).await {
            Ok(()) => tprintln!("Suspend A: OK"),
            Err(e) => tprintln!("Suspend A failed: {e:?}"),
        }
        print_flow_status(&controller);

        tprintln!("\n=== Collecting wakeups during suspension (C only expected) ===");
        collect_wakeups(&controller, 8).await;

        tprintln!("\n=== Flow A should have resumed by now ===");
        print_flow_status(&controller);

        tprintln!("\n=== Collecting wakeups (A + C again) ===");
        collect_wakeups(&controller, 10).await;
    }

    // --- Tear down flow C, leaving only flow A ---

    if let Some(id) = flow_c_id {
        tprintln!("\n=== Tearing down flow C ({id:?}) ===");
        match controller.itwt_teardown(id).await {
            Ok(status) => tprintln!("Teardown C: {status:?}"),
            Err(e) => tprintln!("Teardown C failed: {e:?}"),
        }
        print_flow_status(&controller);
    }

    // --- Single-flow loop with periodic probes ---

    tprintln!("\n=== Entering single-flow loop with periodic probes ===");
    print_flow_status(&controller);

    let mut wakeup_count: u32 = 0;
    loop {
        let wakeup = controller
            .wait_for_next_twt_wakeup(EnumSet::<FlowId>::all())
            .await
            .unwrap();
        wakeup_count += 1;
        tprintln!(
            "TWT wakeup #{}: type={:?}, flow={:?}",
            wakeup_count,
            wakeup.twt_type,
            wakeup.flow_id
        );

        if wakeup_count % 20 == 0 {
            match controller.itwt_send_probe(Duration::from_millis(50)).await {
                Ok(status) => tprintln!("TSF probe: {status:?}"),
                Err(e) => tprintln!("TSF probe failed: {e:?}"),
            }
        }
    }
}
