//! This shows how to asynchronously read ADC data
//!
//! PINS
//! GPIO4 for ADC1, GPIO43 on the ESP32-S31, GPIO20 on the ESP32-P4
//! ONLY ESP32-C3: GPIO5 for ADC2
//!
//! Note that the S31 ADC is differential with a maximum raw value of 4393,
//! so measuring GND will return about 2198.

//% CHIP_FILTER: adc_driver_supported && !esp32

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use esp_backtrace as _;
#[cfg(not(feature = "esp32s31"))]
use esp_hal::analog::adc::AdcCalLine;
use esp_hal::{
    analog::adc::{Adc, AdcConfig, Attenuation},
    delay::Delay,
    timer::timg::TimerGroup,
};
use esp_println::println;

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_hal::main]
async fn main(_spawner: Spawner) {
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    let mut adc1_config = AdcConfig::new();

    let adc1_pin = cfg_select! {
        feature = "esp32s31" => peripherals.GPIO43,
        feature = "esp32p4" => peripherals.GPIO20,
        _ => peripherals.GPIO4,
    };

    // The ESP32-S31 has no calibration scheme implemented.
    let mut pin1 = cfg_select! {
        feature = "esp32s31" => adc1_config.enable_pin(adc1_pin, Attenuation::_11dB),
        _ => adc1_config
            .enable_pin_with_cal::<_, AdcCalLine<esp_hal::peripherals::ADC1<'static>>>(
                adc1_pin,
                Attenuation::_11dB,
            ),
    };

    let mut adc1 = Adc::new(peripherals.ADC1, adc1_config).into_async();

    cfg_select! {
        feature = "esp32c3" => {
            let mut adc2_config = AdcConfig::new();
            let analog_pin2 = peripherals.GPIO5;
            let mut pin2 = adc2_config.enable_pin(analog_pin2, Attenuation::_11dB);
            let mut adc2 = Adc::new(peripherals.ADC2, adc2_config).into_async();
        }
        _ => {}
    }

    let delay = Delay::new();

    loop {
        let adc1_value: u16 = adc1.read_oneshot(&mut pin1).await;
        println!("ADC1 value: {}", adc1_value);
        cfg_select! {
            feature = "esp32c3" => {
                let adc2_value: u16 = adc2.read_oneshot(&mut pin2).await;
                println!("ADC2 value: {}", adc2_value);
            }
            _ => {}
        }
        delay.delay_millis(1000);
    }
}
