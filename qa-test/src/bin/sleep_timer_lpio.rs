//! Demonstrates deep sleep with timer, using low and high level pins as wakeup
//! sources.
//!
//! Wiring
//!
//! | Function           | ESP32-C5 | ESP32-C6 | ESP32-C61 | ESP32-H2  | ESP32-P4 |
//! | ------------------ | -------- | -------- | --------- | --------- | -------- |
//! | Wake on low level  | GPIO2    | GPIO2    | GPIO2     | GPIO9     | GPIO2    |
//! | Wake on high level | GPIO3    | GPIO3    | GPIO3     | GPIO10    | GPIO3    |

//% CHIP_FILTER: esp32c5 || esp32c6 || esp32c61 || esp32h2 || esp32p4

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    gpio::RtcPinWithResistors,
    main,
    rtc_cntl::{
        SocResetReason,
        reset_reason,
        sleep::{Ext1WakeupSource, LowPower, TimerWakeupSource, WakeupLevel},
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

    let mut lpwr = LowPower::new(peripherals.LPWR);

    cfg_select! {
        any(feature = "esp32c5", feature = "esp32c6", feature = "esp32c61", feature = "esp32p4") => {
            let mut pin_low = peripherals.GPIO2;
            let mut pin_high = peripherals.GPIO3;
        }
        feature = "esp32h2" => {
            let mut pin_low = peripherals.GPIO9; // typically a boot mode button, low when pressed
            let mut pin_high = peripherals.GPIO10;
        }
        _ => {}
    }

    println!("up and running!");
    let reason = reset_reason(Cpu::ProCpu).unwrap_or(SocResetReason::ChipPowerOn);
    println!("reset reason: {:?}", reason);
    let wake_reason = wakeup_cause();
    println!("wake reason: {:?}", wake_reason);

    let delay = Delay::new();
    let timer = TimerWakeupSource::new(Duration::from_secs(10));

    let wakeup_pins: &mut [(&mut dyn RtcPinWithResistors, WakeupLevel)] = &mut [
        (&mut pin_low, WakeupLevel::Low),
        (&mut pin_high, WakeupLevel::High),
    ];

    let rtcio = Ext1WakeupSource::new(wakeup_pins);
    println!("sleeping!");
    delay.delay_millis(100);
    lpwr.sleep_deep(&[&timer, &rtcio]);
}
