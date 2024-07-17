//! Demonstrates deep sleep with timer wakeup

//% CHIPS: esp32 esp32c3 esp32c6 esp32s3

#![no_std]
#![no_main]

use core::time::Duration;

use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    delay::Delay,
    entry,
    peripherals::Peripherals,
    rtc_cntl::{get_reset_reason, get_wakeup_cause, sleep::TimerWakeupSource, Rtc, SocResetReason},
    system::SystemControl,
    Cpu,
};
use esp_println::println;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let delay = Delay::new(&clocks);
    let mut rtc = Rtc::new(peripherals.LPWR);

    println!("up and runnning!");
    let reason = get_reset_reason(Cpu::ProCpu).unwrap_or(SocResetReason::ChipPowerOn);
    println!("reset reason: {:?}", reason);
    let wake_reason = get_wakeup_cause();
    println!("wake reason: {:?}", wake_reason);

    let timer = TimerWakeupSource::new(Duration::from_secs(5));
    println!("sleeping!");
    delay.delay_millis(100);
    rtc.sleep_deep(&[&timer]);
}
