//! Demonstrates deep sleep with timer and ext1 wakeup
//!
//! The following wiring is assumed:
//! - ext1 wakeup pins => GPIO2, GPIO4

//% CHIPS: esp32 esp32s3

#![no_std]
#![no_main]

use core::time::Duration;

use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    entry,
    gpio::{Input, Pull, RtcPin},
    peripheral::Peripheral,
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

#[entry]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let mut rtc = Rtc::new(peripherals.LPWR);

    let pin_0 = Input::new(peripherals.GPIO4, Pull::None);
    let mut pin_2 = peripherals.GPIO2;

    println!("up and runnning!");
    let reason = reset_reason(Cpu::ProCpu).unwrap_or(SocResetReason::ChipPowerOn);
    println!("reset reason: {:?}", reason);
    let wake_reason = wakeup_cause();
    println!("wake reason: {:?}", wake_reason);

    let delay = Delay::new();

    let timer = TimerWakeupSource::new(Duration::from_secs(30));
    let mut wakeup_pins: [&mut dyn RtcPin; 2] = [&mut *pin_0.into_ref(), &mut pin_2];
    let ext1 = Ext1WakeupSource::new(&mut wakeup_pins, WakeupLevel::High);
    println!("sleeping!");
    delay.delay_millis(100);
    rtc.sleep_deep(&[&timer, &ext1]);
}
