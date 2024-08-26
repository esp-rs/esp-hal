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
    gpio::Io,
    rtc_cntl::{
        get_reset_reason,
        get_wakeup_cause,
        sleep::{Ext0WakeupSource, TimerWakeupSource, WakeupLevel},
        Rtc,
        SocResetReason,
    },
    Cpu,
};
use esp_println::println;

#[entry]
fn main() -> ! {
    let (peripherals, clocks) = esp_hal::init(CpuClock::boot_default());

    let mut rtc = Rtc::new(peripherals.LPWR);

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    let mut ext0_pin = io.pins.gpio4;

    println!("up and runnning!");
    let reason = get_reset_reason(Cpu::ProCpu).unwrap_or(SocResetReason::ChipPowerOn);
    println!("reset reason: {:?}", reason);
    let wake_reason = get_wakeup_cause();
    println!("wake reason: {:?}", wake_reason);

    let delay = Delay::new(&clocks);

    let timer = TimerWakeupSource::new(Duration::from_secs(30));
    let ext0 = Ext0WakeupSource::new(&mut ext0_pin, WakeupLevel::High);
    println!("sleeping!");
    delay.delay_millis(100);
    rtc.sleep_deep(&[&timer, &ext0]);
}
