//! This example tests all calibration types with all Attenuations
//! and continuosly read and print ADC read pin values.
//!
//! Following pins are used:
//! - ADC read pin => GPIO3

//% CHIPS: esp32c2 esp32c3 esp32c6 esp32s3

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::Timer;
use esp_backtrace as _;
use esp_hal::analog::adc::*;
use esp_hal::delay::Delay;
use esp_hal::gpio::GpioPin;
use esp_hal::main;
use esp_hal::peripherals::ADC1;
use esp_println::*;

// Samples per second
const SAMPLES: usize = 100;

#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) -> ! {
    esp_println::logger::init_logger_from_env();

    println!("\nStarting adc overflow check");

    let ph = esp_hal::init(esp_hal::Config::default());

    let atenuattion: &[Attenuation] = &[
        Attenuation::_0dB,
        #[cfg(not(feature = "esp32c2"))]
        Attenuation::_2p5dB,
        #[cfg(not(feature = "esp32c2"))]
        Attenuation::_6dB,
        Attenuation::_11dB,
    ];

    //check for overflow
    for atn in atenuattion {
        println!();
        println!("Starting Test with {:?} Attenuation:", atn);
        println!();

        let basic = AdcCalBasic::<ADC1>::new_cal(*atn);

        //only resolution its 13b
        for value in 0..4096 {
            basic.adc_val(value);
        }

        println!("Passed basic calibration.");

        let line = AdcCalLine::<ADC1>::new_cal(*atn);

        //only resolution its 13b
        for value in 0..4096 {
            line.adc_val(value);
        }

        println!("Passed line calibration.");

        cfg_if::cfg_if! {
            if #[cfg(not(feature = "esp32c2"))]
            {
                let curve = AdcCalCurve::<ADC1>::new_cal(*atn);

                //only resolution its 13b
                for value in 0..4096 {
                    curve.adc_val(value);
                }

                println!(
                    "Passed curve calibration."
                );
            }
        }
    }

    let mut adc1_config = AdcConfig::new();

    // this need to be called to calibrate the esp
    AdcConfig::<ADC1>::adc_calibrate(Attenuation::_11dB, AdcCalSource::Ref);

    // Instead of create a new calibration and convert outside, attach the pin with a calibration scheme
    let mut pin = adc1_config
        .enable_pin_with_cal::<GpioPin<3>, AdcCalCurve<ADC1>>(ph.GPIO3, Attenuation::_11dB);

    let mut adc = Adc::new(ph.ADC1, adc1_config);

    let delay = Delay::new();

    loop {
        // Taking the arithmetic average over 1 Second
        let average = {
            let mut val = 0.0;
            for _ in 1..SAMPLES {
                val += (adc.read_blocking(&mut pin) as f32) / 1000.0;

                delay.delay_millis(1000 / SAMPLES as u32);
            }
            val
        } / SAMPLES as f32;

        println!("Value: {:?}", average);
    }
}
