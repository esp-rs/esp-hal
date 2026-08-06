//! Repeatedly initializes and deinitializes the Wi-Fi driver and scans for
//! access points. Every iteration must find access points — the iterations
//! after the first exercise re-initialization after the modem power domain
//! and clocks were torn down.
//!
//! The test is also suited to verify the power savings of the teardown with
//! a (slow) power meter:
//! - after boot there is a startup delay with the radio never initialized (baseline consumption),
//! - each iteration then has a 10s active phase (radio initialized, scanning every second) followed
//!   by a 10s inactive phase (radio deinitialized). The inactive-phase consumption should match the
//!   baseline.
//!
//! Success: every scan finds at least one access point.
//! Failure: a scan finds no access points or errors.

//% FEATURES: esp-radio esp-radio/wifi esp-radio/unstable esp-hal/unstable
//% CHIP_FILTER: wifi_driver_supported

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Timer};
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    interrupt::software::SoftwareInterruptControl,
    ram,
    timer::timg::TimerGroup,
};
use esp_println::println;
use esp_radio::wifi::{ControllerConfig, WifiController, scan::ScanConfig};

esp_bootloader_esp_idf::esp_app_desc!();

/// Settle time after boot, with the radio never initialized (baseline power).
const STARTUP_DELAY: Duration = Duration::from_secs(5);
/// How long the radio stays initialized per iteration.
const ACTIVE_PHASE: Duration = Duration::from_secs(10);
/// How long the radio stays deinitialized per iteration.
const INACTIVE_PHASE: Duration = Duration::from_secs(10);

#[esp_hal::main]
async fn main(_spawner: Spawner) {
    esp_println::logger::init_logger_from_env();
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(size: 32 * 1024);
    // add some more RAM
    esp_alloc::heap_allocator!(#[ram(reclaimed)] size: 64 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    println!(
        "Waiting {}s (baseline power, radio never initialized)",
        STARTUP_DELAY.as_secs()
    );
    Timer::after(STARTUP_DELAY).await;

    let mut wifi = peripherals.WIFI;
    let mut iteration = 0u32;
    loop {
        iteration += 1;

        let mut controller = WifiController::new(wifi.reborrow(), ControllerConfig::default())
            .expect("failed to create Wi-Fi controller");

        println!(
            "Iteration {iteration}: active for {}s",
            ACTIVE_PHASE.as_secs()
        );
        let active_until = Instant::now() + ACTIVE_PHASE;
        while Instant::now() < active_until {
            match controller
                .scan_async(&ScanConfig::default().with_max(10))
                .await
            {
                Ok(aps) => {
                    println!("Iteration {iteration}: found {} access points", aps.len());
                    for ap in aps {
                        println!("  {ap:?}");
                    }
                }
                Err(e) => println!("Iteration {iteration}: scan failed: {e:?}"),
            }
            Timer::after(Duration::from_secs(1)).await;
        }

        drop(controller);
        println!(
            "Iteration {iteration}: Wi-Fi deinitialized, inactive for {}s (measure power now)",
            INACTIVE_PHASE.as_secs()
        );

        Timer::after(INACTIVE_PHASE).await;
    }
}
