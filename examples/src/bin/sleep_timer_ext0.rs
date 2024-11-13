//! Demonstrates deep sleep with timer and ext0 wakeup
//!
//! The following wiring is assumed:
//! - ext0 wakeup pin => GPIO4

//% CHIPS: esp32 esp32s3

#![no_std]
#![no_main]

use core::time::Duration;

use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    entry,
    gpio::{Input, Pull},
    rtc_cntl::{
        reset_reason,
        sleep::{Ext0WakeupSource, TimerWakeupSource, WakeupLevel},
        wakeup_cause,
        Rtc,
        SocResetReason,
    },
    Cpu,
};
use esp_println::println;

#[entry]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let mut rtc = Rtc::new(peripherals.LPWR);

    let ext0_pin = Input::new(peripherals.GPIO4, Pull::None);

    println!("up and runnning!");
    let reason = reset_reason(Cpu::ProCpu).unwrap_or(SocResetReason::ChipPowerOn);
    println!("reset reason: {:?}", reason);
    let wake_reason = wakeup_cause();
    println!("wake reason: {:?}", wake_reason);

    let delay = Delay::new();

    let timer = TimerWakeupSource::new(Duration::from_secs(30));
    let ext0 = Ext0WakeupSource::new(ext0_pin, WakeupLevel::High);
    println!("sleeping!");
    delay.delay_millis(100);
    rtc.sleep_deep(&[&timer, &ext0]);
}
