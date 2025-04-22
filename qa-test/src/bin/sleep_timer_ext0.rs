//! Demonstrates deep sleep with timer and ext0 wakeup
//!
//! The following wiring is assumed:
//! - ext0 wakeup pin => GPIO4

//% CHIPS: esp32 esp32s2 esp32s3

#![no_std]
#![no_main]

use core::time::Duration;

use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    gpio::{Input, InputConfig, Pull},
    main,
    rtc_cntl::{
        Rtc,
        SocResetReason,
        reset_reason,
        sleep::{Ext0WakeupSource, TimerWakeupSource, WakeupLevel},
        wakeup_cause,
    },
    system::Cpu,
};
use esp_println::println;

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let mut rtc = Rtc::new(peripherals.LPWR);

    let mut pin4 = peripherals.GPIO4;
    let ext0_pin = Input::new(
        pin4.reborrow(),
        InputConfig::default().with_pull(Pull::None),
    );

    println!("up and runnning!");
    let reason = reset_reason(Cpu::ProCpu).unwrap_or(SocResetReason::ChipPowerOn);
    println!("reset reason: {:?}", reason);
    let wake_reason = wakeup_cause();
    println!("wake reason: {:?}", wake_reason);

    let delay = Delay::new();

    core::mem::drop(ext0_pin);

    let timer = TimerWakeupSource::new(Duration::from_secs(30));
    let ext0 = Ext0WakeupSource::new(pin4, WakeupLevel::High);
    println!("sleeping!");
    delay.delay_millis(100);
    rtc.sleep_deep(&[&timer, &ext0]);
}
