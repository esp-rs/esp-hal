//! This shows how to asynchronously read ADC data

//% CHIPS: esp32c3 esp32c6 esp32h2

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use esp_backtrace as _;
use esp_hal::{
    analog::adc::{Adc, AdcConfig, Attenuation},
    delay::Delay,
    timer::timg::TimerGroup,
};
use esp_println::println;

#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) {
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    let mut adc1_config = AdcConfig::new();
    let analog_pin1 = peripherals.GPIO4;
    let mut pin1 = adc1_config.enable_pin(analog_pin1, Attenuation::_11dB);
    let mut adc1 = Adc::new(peripherals.ADC1, adc1_config).into_async();

    cfg_if::cfg_if! {
        if #[cfg(feature = "esp32c3")] {
            let mut adc2_config = AdcConfig::new();
            let analog_pin2 = peripherals.GPIO5;
            let mut pin2 = adc2_config.enable_pin(analog_pin2, Attenuation::_11dB);
            let mut adc2 = Adc::new(peripherals.ADC2, adc2_config).into_async();
        }
    }

    let delay = Delay::new();

    loop {
        let adc1_value: u16 = adc1.read_oneshot(&mut pin1).await;
        println!("ADC1 value: {}", adc1_value);
        cfg_if::cfg_if! {
            if #[cfg(feature = "esp32c3")] {
                let adc2_value: u16 = adc2.read_oneshot(&mut pin2).await;
                println!("ADC2 value: {}", adc2_value);
            }
        }
        delay.delay_millis(1000);
    }
}
