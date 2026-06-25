//! Demonstrates light sleep with two wakeup sources (timer + GPIO) and prints
//! the wakeup cause after each wakeup so they can be compared.
//!
//! Unlike deep sleep, light sleep does not reset the chip, so execution simply
//! resumes after `sleep_light` returns. This example loops forever:
//!
//! - If left alone, the timer fires after 5s    => wake reason: `Timer`
//! - If the BOOT button is pressed (pin pulled   => wake reason: `Gpio` low), the GPIO wakeup fires
//!   first

//% CHIP_FILTER: sleep_light_sleep

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    gpio::{Input, InputConfig, Pull, WakeEvent},
    main,
    rtc_cntl::{
        Rtc,
        reset_reason,
        sleep::{GpioWakeupSource, TimerWakeupSource},
        wakeup_cause,
    },
    system::Cpu,
    time::Duration,
};
use esp_println::println;

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let delay = Delay::new();
    let mut rtc = Rtc::new(peripherals.LPWR);

    println!("up and running!");
    println!("reset reason: {:?}", reset_reason(Cpu::ProCpu));
    // Before any sleep there is no wakeup, so this prints `Undefined`.
    println!("wake reason (before sleep): {:?}", wakeup_cause());

    cfg_select! {
        any(feature = "esp32", feature = "esp32s2", feature = "esp32s3") => {
            let button = peripherals.GPIO0;
        }
        _ => {
            let button = peripherals.GPIO9;
        }
    }

    let mut button = Input::new(button, InputConfig::default().with_pull(Pull::Up));
    button
        .wakeup_enable(true, WakeEvent::LowLevel)
        .expect("failed to configure GPIO wakeup");

    loop {
        let timer = TimerWakeupSource::new(Duration::from_secs(5));
        let gpio = GpioWakeupSource::new();

        println!("entering light sleep (press BOOT to wake early)...");
        delay.delay_millis(100);

        rtc.sleep_light(&[&timer, &gpio]);

        println!("woke up! wake reason: {:?}", wakeup_cause());

        delay.delay_millis(500);
    }
}
