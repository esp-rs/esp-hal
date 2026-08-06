//! Demonstrates deep sleep with a timer and two pins as wakeup sources.
//!
//! Sleep entry picks the hardware path for each pin - `ext0`, `ext1`, or the per-pin low-power
//! path - so the program only says which pins may wake the chip, and on what.
//!
//! Wiring
//!
//! | Function           | ESP32 | ESP32-S2/S3 | ESP32-C2/C3 | ESP32-C5/C6/C61/P4 | ESP32-H2 |
//! | ------------------ | ----- | ----------- | ----------- | ------------------ | -------- |
//! | Wake on low level  | GPIO2 | GPIO17      | GPIO2       | GPIO2              | GPIO9    |
//! | Wake on high level | GPIO4 | GPIO18      | GPIO3       | GPIO3              | GPIO10   |
//!
//! Each pin idles at the level that does not wake the chip. Pull the low-level pin to ground, or
//! the high-level pin to 3V3, to end the sleep.

//% CHIP_FILTER: sleep_driver_supported

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    gpio::{Event, Input, InputConfig, Pull, WakeupConfig},
    main,
    rtc_cntl::{
        SocResetReason,
        reset_reason,
        sleep::{LowPower, RtcSleepConfig},
        wakeup_cause,
    },
    system::Cpu,
    time::{Duration, Instant},
};
use esp_println::println;

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let delay = Delay::new();
    let mut lpwr = LowPower::new(peripherals.LPWR);

    cfg_select! {
        feature = "esp32" => {
            let (pin_low, pin_high) = (peripherals.GPIO2, peripherals.GPIO4);
        }
        any(feature = "esp32s2", feature = "esp32s3") => {
            let (pin_low, pin_high) = (peripherals.GPIO17, peripherals.GPIO18);
        }
        feature = "esp32h2" => {
            // GPIO9 is typically a boot mode button, low when pressed.
            let (pin_low, pin_high) = (peripherals.GPIO9, peripherals.GPIO10);
        }
        _ => {
            let (pin_low, pin_high) = (peripherals.GPIO2, peripherals.GPIO3);
        }
    }

    println!("up and running!");
    let reason = reset_reason(Cpu::ProCpu).unwrap_or(SocResetReason::ChipPowerOn);
    println!("reset reason: {:?}", reason);
    let wake_reason = wakeup_cause();
    println!("wake reason: {:?}", wake_reason);

    // The pull holds the pin at the level that does not wake the chip, so that the sleep is not
    // rejected before it starts.
    let mut pin_low = Input::new(pin_low, InputConfig::default().with_pull(Pull::Up));
    let mut pin_high = Input::new(pin_high, InputConfig::default().with_pull(Pull::Down));

    let config = WakeupConfig::default().with_low_power_path(true);
    pin_low.apply_wakeup_config(&config).unwrap();
    pin_high.apply_wakeup_config(&config).unwrap();

    // The interrupt trigger is the wake condition.
    pin_low.listen(Event::LowLevel);
    pin_high.listen(Event::HighLevel);

    // The deadline is absolute, so the delay below does not shorten the sleep.
    lpwr.set_wakeup_deadline(Instant::now() + Duration::from_secs(30));

    println!("sleeping!");
    delay.delay_millis(100);
    lpwr.sleep_deep(RtcSleepConfig::deep());
}
