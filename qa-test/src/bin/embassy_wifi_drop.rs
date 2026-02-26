//! Check that trying to send after drop doesn't crash

//% FEATURES: esp-radio esp-radio/wifi esp-radio/unstable esp-hal/unstable
//% CHIPS: esp32 esp32c2 esp32c3 esp32c5 esp32c6 esp32s2 esp32s3

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    interrupt::software::SoftwareInterruptControl,
    ram,
    timer::timg::TimerGroup,
};
use esp_println::println;
use esp_radio::wifi::{Config, ControllerConfig, sta::StationConfig};

esp_bootloader_esp_idf::esp_app_desc!();

const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");

#[esp_rtos::main]
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

    let mut wifi = peripherals.WIFI;
    {
        let station_config = Config::Station(
            StationConfig::default()
                .with_ssid(SSID)
                .with_password(PASSWORD.into()),
        );

        let (mut controller, interfaces) = esp_radio::wifi::new(
            wifi.reborrow(),
            ControllerConfig::default().with_initial_config(station_config),
        )
        .unwrap();

        let mut wifi_interface = interfaces.station;

        let token = wifi_interface.transmit();
        assert!(matches!(token, None));

        controller.connect_async().await.unwrap();

        let token = wifi_interface.transmit();

        println!("got token {}", token.is_some());

        core::mem::drop(controller);

        if let Some(token) = token {
            token.consume_token(10, |tx| tx.fill(0));
            println!("survived consumer_token");
        } else {
            println!("no token");
        }

        let token = wifi_interface.transmit();
        assert!(matches!(token, None));
    }

    {
        let station_config = Config::Station(
            StationConfig::default()
                .with_ssid(SSID)
                .with_password(PASSWORD.into()),
        );

        let (mut controller, interfaces) = esp_radio::wifi::new(
            wifi.reborrow(),
            ControllerConfig::default().with_initial_config(station_config),
        )
        .unwrap();
        controller.connect_async().await.unwrap();

        let mut wifi_interface = interfaces.station;

        let token = wifi_interface.transmit();

        println!("got token {}", token.is_some());

        if let Some(token) = token {
            token.consume_token(10, |tx| tx.fill(0));
            println!("tx done");
        } else {
            println!("no token");
        }
    }

    println!("Done");
}
