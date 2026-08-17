// ESP32 has some serious (light-)sleep issues - so don't include it here until that is fixed.

//% FEATURES: esp-radio esp-radio/wifi esp-hal/unstable
//% CHIP_FILTER: wifi_driver_supported && sleep_driver_supported && !esp32

#![no_std]
#![no_main]

extern crate alloc;

use embassy_executor::Spawner;
use esp_alloc;
use esp_backtrace as _;
use esp_hal::{
    rtc_cntl::sleep::{LowPower, RtcSleepConfig},
    time::{Duration, Instant},
    timer::timg::TimerGroup,
};
use esp_println::println;

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_hal::main]
async fn main(_spawner: Spawner) {
    // Enable logging from the ESP_LOG environment variable (set at build time)
    // Example: ESP_LOG=warn,esp_rtos=trace,esp_radio=info
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init(esp_hal::Config::default());

    // Provide a heap for components that allocate (esp-rtos/esp-radio, etc.)
    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 64 * 1024);

    // Preempt scheduler (WiFi)
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0, peripherals.FROM_CPU_INTR0);

    let delay = esp_hal::delay::Delay::new();
    delay.delay_millis(100);

    // Sleep for one second
    let mut lpwr = LowPower::new(peripherals.LPWR);
    lpwr.set_wakeup_deadline(Instant::now() + Duration::from_secs(1));

    esp_println::println!("Start sleep");
    delay.delay_millis(100);

    lpwr.sleep_light(RtcSleepConfig::default());
    delay.delay_millis(100);

    esp_println::println!("Done sleeping");

    // WiFi
    let mut controller =
        esp_radio::wifi::WifiController::new(peripherals.WIFI, Default::default()).unwrap();

    let res = controller
        .scan_async(&esp_radio::wifi::scan::ScanConfig::default())
        .await
        .unwrap();

    println!("Found {:?}", res);

    if res.len() == 0 {
        println!("No networks found - might indicate there is a problem");
    }

    println!("Test done");
}
