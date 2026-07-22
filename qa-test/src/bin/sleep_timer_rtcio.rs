//! Demonstrates deep sleep with timer, using RTC pins as
//! wakeup.
//!
//! The following wiring is assumed for ESP32C3:
//! - RTC wakeup pin => GPIO2 (low level)
//! - RTC wakeup pin => GPIO3 (high level)
//! The following wiring is assumed for ESP32S2/ESP32S3:
//! - RTC wakeup pin => GPIO17 (low level)
//! - RTC wakeup pin => GPIO18 (high level)

//% CHIP_FILTER: esp32c3 || esp32s2 || esp32s3 || esp32c2

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    gpio,
    gpio::{Input, InputConfig, Level, Pull},
    main,
    rtc_cntl::{
        SocResetReason,
        reset_reason,
        sleep::{LowPower, RtcioWakeupSource, TimerWakeupSource},
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

    println!("up and running!");
    let reason = reset_reason(Cpu::ProCpu).unwrap_or(SocResetReason::ChipPowerOn);
    println!("reset reason: {:?}", reason);
    let wake_reason = wakeup_cause();
    println!("wake reason: {:?}", wake_reason);

    let delay = Delay::new();
    let timer = TimerWakeupSource::new(Duration::from_secs(10));

    let config = InputConfig::default().with_pull(Pull::None);
    cfg_select! {
        any(feature = "esp32c3", feature = "esp32c2") => {
            let mut pin2 = peripherals.GPIO2;
            let mut pin3 = peripherals.GPIO3;
            let _pin2_input = Input::new(pin2.reborrow(), config);

            let wakeup_pins: &mut [(&mut dyn gpio::RtcPinWithResistors, Level)] = &mut [
                (&mut pin2, Level::Low),
                (&mut pin3, Level::High),
            ];
        }
        any(feature = "esp32s2", feature = "esp32s3") => {
            let mut pin17 = peripherals.GPIO17;
            let mut pin18 = peripherals.GPIO18;
            let _pin17_input = Input::new(pin17.reborrow(), config);

            let wakeup_pins: &mut [(&mut dyn gpio::RtcPin, Level)] = &mut [
                (&mut pin17, Level::Low),
                (&mut pin18, Level::High),
            ];
        }
        _ => {}
    }

    let rtcio = RtcioWakeupSource::new(wakeup_pins);
    println!("sleeping!");
    delay.delay_millis(100);
    lpwr.sleep_deep(&[&timer, &rtcio]);
}
