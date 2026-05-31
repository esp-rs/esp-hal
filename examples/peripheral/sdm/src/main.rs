//! This example shows how to generate a sigma-delta output signal.
//!
//! The following wiring is assumed:
//! - SDM output => GPIO2

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{delay::Delay, main, sdm::Sdm, time::Rate};
use esp_println::print;

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let sdm = Sdm::new(peripherals.GPIO_SD);
    let config = sdm
        .channel_config()
        .with_frequency(Rate::from_khz(500))
        .unwrap()
        .with_pulse_density(0);
    let mut channel = sdm.channel0.connect(peripherals.GPIO2, config).unwrap();

    let delay = Delay::new();
    let mut duty = 0;

    loop {
        channel.set_duty(duty);
        print!("SDM duty: {duty:3}\r");

        duty = duty.wrapping_add(1);
        delay.delay_millis(100);
    }
}
