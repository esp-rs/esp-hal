//! Demonstrates deep sleep with timer, using RTC pins as
//! wakeup.
//!
//! The following wiring is assumed for ESP32C3:
//! - RTC wakeup pin => GPIO2 (low level)
//! - RTC wakeup pin => GPIO3 (high level)
//! The following wiring is assumed for ESP32S3:
//! - RTC wakeup pin => GPIO18 (low level)

//% CHIPS: esp32c3 esp32s3 esp32c2

#![no_std]
#![no_main]

use core::time::Duration;

use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    gpio,
    gpio::{Input, InputConfig, Pull},
    main,
    rtc_cntl::{
        reset_reason,
        sleep::{RtcioWakeupSource, TimerWakeupSource, WakeupLevel},
        wakeup_cause,
        Rtc,
        SocResetReason,
    },
    Cpu,
};
use esp_println::println;

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let mut rtc = Rtc::new(peripherals.LPWR);

    println!("up and runnning!");
    let reason = reset_reason(Cpu::ProCpu).unwrap_or(SocResetReason::ChipPowerOn);
    println!("reset reason: {:?}", reason);
    let wake_reason = wakeup_cause();
    println!("wake reason: {:?}", wake_reason);

    let delay = Delay::new();
    let timer = TimerWakeupSource::new(Duration::from_secs(10));

    let config = InputConfig::default().with_pull(Pull::None);
    cfg_if::cfg_if! {
        if #[cfg(any(feature = "esp32c3", feature = "esp32c2"))] {
            let mut pin2 = peripherals.GPIO2;
            let mut pin3 = peripherals.GPIO3;
            let _pin2_input = Input::new(&mut pin2, config);

            let wakeup_pins: &mut [(&mut dyn gpio::RtcPinWithResistors, WakeupLevel)] = &mut [
                (&mut pin2, WakeupLevel::Low),
                (&mut pin3, WakeupLevel::High),
            ];
        } else if #[cfg(feature = "esp32s3")] {
            let mut pin17 = peripherals.GPIO17;
            let mut pin18 = peripherals.GPIO18;
            let _pin17_input = Input::new(&mut pin17, config);

            let wakeup_pins: &mut [(&mut dyn gpio::RtcPin, WakeupLevel)] = &mut [
                (&mut pin17, WakeupLevel::Low),
                (&mut pin18, WakeupLevel::High),
            ];
        }
    }

    let rtcio = RtcioWakeupSource::new(wakeup_pins);
    println!("sleeping!");
    delay.delay_millis(100);
    rtc.sleep_deep(&[&timer, &rtcio]);
}
