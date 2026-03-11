//! Demonstrates deep sleep with timer, using low and high level pins as wakeup
//! sources.
//!
//! The following wiring is assumed for ESP32-C6:
//! - ext1 wakeup pin => GPIO2 (low level) / GPIO3 (high level)
//! The following wiring is assumed for ESP32-H2:
//! - ext1 wakeup pin => GPIO9 (low level) / GPIO10 (high level)

//% CHIPS: esp32c6 esp32h2

#![no_std]
#![no_main]

use core::time::Duration;

use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    gpio::RtcPinWithResistors,
    main,
    rtc_cntl::{
        Rtc,
        SocResetReason,
        reset_reason,
        sleep::{Ext1WakeupSource, TimerWakeupSource, WakeupLevel},
        wakeup_cause,
    },
    system::Cpu,
};
use esp_println::println;

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let mut rtc = Rtc::new(peripherals.LPWR);

    cfg_if::cfg_if! {
        if #[cfg(feature = "esp32c6")] {
            use esp_hal::gpio::{Input, InputConfig, Pull};

            let mut pin_low = peripherals.GPIO2;
            let mut pin_high = peripherals.GPIO3;
            let input = Input::new(
                pin_low.reborrow(),
                InputConfig::default().with_pull(Pull::None),
            );
            core::mem::drop(input);
        } else if #[cfg(feature = "esp32h2")] {
            let mut pin_low = peripherals.GPIO9; // typically a boot mode button, low when pressed
            let mut pin_high = peripherals.GPIO10;
        }
    }

    println!("up and runnning!");
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
    rtc.sleep_deep(&[&timer, &rtcio]);
}
