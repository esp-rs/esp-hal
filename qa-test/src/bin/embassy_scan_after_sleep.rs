// ESP32 has some serious (light-)sleep issues - so don't include it here until that is fixed.

//% FEATURES: esp-radio esp-radio/wifi esp-hal/unstable
//% CHIPS: esp32s2 esp32s3 esp32c2 esp32c3 esp32c6

#![no_std]
#![no_main]

extern crate alloc;

use embassy_executor::Spawner;
use esp_alloc;
use esp_backtrace as _;
use esp_hal::{interrupt::software::SoftwareInterruptControl, timer::timg::TimerGroup};
use esp_println::println;

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(_spawner: Spawner) {
    // Enable logging from the ESP_LOG environment variable (set at build time)
    // Example: ESP_LOG=warn,esp_rtos=trace,esp_radio=info
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init(esp_hal::Config::default());

    // Provide a heap for components that allocate (esp-rtos/esp-radio, etc.)
    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 64 * 1024);

    // Preempt scheduler (WiFi)
    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    // Sleep for one second
    let sleep_config = esp_hal::rtc_cntl::sleep::RtcSleepConfig::default();

    let delay = esp_hal::delay::Delay::new();
    delay.delay_millis(100);

    let timer =
        esp_hal::rtc_cntl::sleep::TimerWakeupSource::new(core::time::Duration::from_secs(1));

    let mut rtc = esp_hal::rtc_cntl::Rtc::new(peripherals.LPWR);
    esp_println::println!("Start sleep");
    delay.delay_millis(100);

    rtc.sleep(&sleep_config, &[&timer]);
    delay.delay_millis(100);

    esp_println::println!("Done sleeping");

    // WiFi
    let (mut controller, _interfaces) =
        esp_radio::wifi::new(peripherals.WIFI, Default::default()).unwrap();

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
