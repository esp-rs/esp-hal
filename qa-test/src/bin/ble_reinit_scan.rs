//! Repeatedly initializes and deinitializes the BLE controller and scans for
//! advertisements. Every iteration must receive advertisements — the
//! iterations after the first exercise re-initialization after the modem
//! clocks were torn down.
//!
//! You need at least one BLE advertiser in range (e.g. a smartphone or a
//! second board running a BLE advertising example).
//!
//! The test is also suited to verify the power savings of the teardown with
//! a (slow) power meter:
//! - after boot there is a startup delay with the radio never initialized (baseline consumption),
//! - each iteration then has a 10s active phase (radio initialized, scanning) followed by a 10s
//!   inactive phase (radio deinitialized). The inactive-phase consumption should match the
//!   baseline.
//!
//! Success: every iteration receives at least one advertisement.
//! Failure: an iteration receives no advertisements or fails to start the
//! scan.

//% FEATURES: esp-radio esp-radio/ble esp-radio/unstable esp-hal/unstable
//% CHIP_FILTER: bt_driver_supported

#![no_std]
#![no_main]

use core::sync::atomic::{AtomicU32, Ordering};

use embassy_executor::Spawner;
use embassy_futures::select::select;
use embassy_time::{Duration, Timer};
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{clock::CpuClock, timer::timg::TimerGroup};
use esp_println::println;
use esp_radio::ble::controller::BleConnector;
use trouble_host::prelude::*;

esp_bootloader_esp_idf::esp_app_desc!();

/// Max number of connections
const CONNECTIONS_MAX: usize = 1;
const L2CAP_CHANNELS_MAX: usize = 1;

static ADV_COUNT: AtomicU32 = AtomicU32::new(0);

/// Settle time after boot, with the radio never initialized (baseline power).
const STARTUP_DELAY: Duration = Duration::from_secs(5);
/// How long the radio stays initialized per iteration.
const ACTIVE_PHASE: Duration = Duration::from_secs(10);
/// How long the radio stays deinitialized per iteration.
const INACTIVE_PHASE: Duration = Duration::from_secs(10);

#[esp_hal::main]
async fn main(_spawner: Spawner) {
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));

    esp_alloc::heap_allocator!(size: 72 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    println!(
        "Waiting {}s (baseline power, radio never initialized)",
        STARTUP_DELAY.as_secs()
    );
    Timer::after(STARTUP_DELAY).await;

    let mut bt = peripherals.BT;
    let mut iteration = 0u32;
    loop {
        iteration += 1;
        ADV_COUNT.store(0, Ordering::Relaxed);

        {
            let connector = BleConnector::new(bt.reborrow(), esp_radio::ble::Config::default())
                .expect("failed to create BLE connector");
            let controller: ExternalController<_, 1> = ExternalController::new(connector);

            // Using a fixed "random" address can be useful for testing. In real scenarios, one
            // would use e.g. the MAC 6 byte array as the address (how to get that varies by the
            // platform).
            let address: Address = Address::random([0xff, 0x8f, 0x1b, 0x05, 0xe4, 0xff]);

            let mut resources: HostResources<
                _,
                DefaultPacketPool,
                CONNECTIONS_MAX,
                L2CAP_CHANNELS_MAX,
            > = HostResources::new();
            let stack = trouble_host::new(controller, &mut resources)
                .set_random_address(address)
                .build();
            let central = stack.central();
            let mut runner = stack.runner();

            let printer = Printer;
            let mut scanner = Scanner::new(central);
            println!(
                "Iteration {iteration}: active for {}s",
                ACTIVE_PHASE.as_secs()
            );
            select(runner.run_with_handler(&printer), async {
                let config = ScanConfig {
                    active: false,
                    phys: PhySet::M1,
                    interval: Duration::from_millis(500),
                    window: Duration::from_millis(100),
                    timeout: Duration::from_secs(0),
                    ..Default::default()
                };
                match scanner.scan(&config).await {
                    Ok(_session) => Timer::after(ACTIVE_PHASE).await,
                    Err(e) => println!("Iteration {iteration}: failed to start scan: {e:?}"),
                }
            })
            .await;
        }

        println!(
            "Iteration {iteration}: received {} advertisements, inactive for {}s (measure power now)",
            ADV_COUNT.load(Ordering::Relaxed),
            INACTIVE_PHASE.as_secs()
        );

        Timer::after(INACTIVE_PHASE).await;
    }
}

struct Printer;

impl EventHandler for Printer {
    fn on_adv_reports(&self, mut reports: LeAdvReportsIter<'_>) {
        while let Some(Ok(report)) = reports.next() {
            println!("  adv {:?} len {}", report.addr, report.data.len());
            ADV_COUNT.store(ADV_COUNT.load(Ordering::Relaxed) + 1, Ordering::Relaxed);
        }
    }
}
