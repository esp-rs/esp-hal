//! A check that extended-advertisement scanning works.
//!
//! To really test this you need a way to have large advertisements received.
//!
//! e.g. on Windows use `Bluetooth LE Explorer` - use `Advertisement Beacon`,
//! make sure to enable "Use Extended Format" and some payload.
//! Enable advertising.
//!
//! Not supported on ESP32 since this feature needs BLE 5.0+ (and ESP32 is 4.2)

//% FEATURES: esp-radio esp-radio/ble esp-hal/unstable
//% CHIPS: esp32s3 esp32h2 esp32c2 esp32c3 esp32c5 esp32c6 esp32c61

#![no_std]
#![no_main]
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_time::{Duration, Timer};
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    interrupt::software::SoftwareInterruptControl,
    timer::timg::TimerGroup,
};
use esp_println::println;
use esp_radio::ble::controller::BleConnector;
use trouble_host::prelude::*;

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_hal::main]
async fn main(_s: Spawner) {
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));

    esp_alloc::heap_allocator!(size: 72 * 1024);

    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    let bluetooth = peripherals.BT;
    let connector = BleConnector::new(bluetooth, esp_radio::ble::Config::default()).unwrap();
    let controller: ExternalController<_, 1> = ExternalController::new(connector);

    // Using a fixed "random" address can be useful for testing. In real scenarios, one would
    // use e.g. the MAC 6 byte array as the address (how to get that varies by the platform).
    let address: Address = Address::random([0xff, 0x8f, 0x1b, 0x05, 0xe4, 0xff]);

    println!("Our address = {:?}", address);

    let mut resources: HostResources<DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX> =
        HostResources::new();
    let stack = trouble_host::new(controller, &mut resources).set_random_address(address);

    let Host {
        central,
        mut runner,
        ..
    } = stack.build();

    let printer = Printer {};
    let mut scanner = Scanner::new(central);
    let _ = join(runner.run_with_handler(&printer), async {
        let config = ScanConfig {
            active: false,
            phys: PhySet::M1,
            interval: Duration::from_millis(500),
            window: Duration::from_millis(100),
            timeout: Duration::from_secs(0),
            ..Default::default()
        };
        let mut _session = scanner.scan_ext(&config).await.unwrap();
        // Scan forever
        loop {
            Timer::after(Duration::from_secs(1)).await;
        }
    })
    .await;
}

/// Max number of connections
const CONNECTIONS_MAX: usize = 1;
const L2CAP_CHANNELS_MAX: usize = 1;

struct Printer {}

impl EventHandler for Printer {
    fn on_ext_adv_reports(&self, reports: LeExtAdvReportsIter<'_>) {
        for report in reports {
            let report = report.unwrap();
            println!("ext adv {:?} {}", report.addr, report.data.len());
        }
    }
}
