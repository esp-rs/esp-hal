//! This shows how to asynchronously read ADC data
//!
//! PINS
//! GPIO4 for ADC1
//! ONLY ESP32-C3: GPIO5 for ADC2

//% CHIPS: esp32c2 esp32c3 esp32c6 esp32h2

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use esp_backtrace as _;
#[cfg(target_arch = "riscv32")]
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::{
    analog::adc::{Adc, AdcConfig, Attenuation},
    delay::Delay,
    timer::timg::TimerGroup,
};
use esp_println::println;

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(_spawner: Spawner) {
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init(esp_hal::Config::default());
    #[cfg(target_arch = "riscv32")]
    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(
        timg0.timer0,
        #[cfg(target_arch = "riscv32")]
        sw_int.software_interrupt0,
    );

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
