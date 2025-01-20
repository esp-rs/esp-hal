//! Demonstrates deep sleep with timer, using low and high level pins as wakeup
//! sources.
//!
//! The following wiring is assumed:
//! - ext1 wakeup pin => GPIO2 (low level) / GPIO3 (high level)

//% CHIPS: esp32c6

#![no_std]
#![no_main]

use core::time::Duration;

use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    gpio::{Input, InputConfig, Pull, RtcPinWithResistors},
    main,
    rtc_cntl::{
        reset_reason,
        sleep::{Ext1WakeupSource, TimerWakeupSource, WakeupLevel},
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

    let mut pin2 = peripherals.GPIO2;
    let input = Input::new(
        &mut pin2,
        InputConfig::default().with_pull(Pull::None),
    )
    .unwrap();

    println!("up and runnning!");
    let reason = reset_reason(Cpu::ProCpu).unwrap_or(SocResetReason::ChipPowerOn);
    println!("reset reason: {:?}", reason);
    let wake_reason = wakeup_cause();
    println!("wake reason: {:?}", wake_reason);

    let delay = Delay::new();
    let timer = TimerWakeupSource::new(Duration::from_secs(10));

    core::mem::drop(input);

    let wakeup_pins: &mut [(&mut dyn RtcPinWithResistors, WakeupLevel)] = &mut [
        (&mut pin2, WakeupLevel::Low),
        (&mut pin3, WakeupLevel::High),
    ];

    let rtcio = Ext1WakeupSource::new(wakeup_pins);
    println!("sleeping!");
    delay.delay_millis(100);
    rtc.sleep_deep(&[&timer, &rtcio]);
}
