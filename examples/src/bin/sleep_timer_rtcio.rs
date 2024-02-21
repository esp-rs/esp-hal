//! Demonstrates deep sleep with timer and rtcio wakeup

//% CHIPS: esp32c3 esp32s3

#![no_std]
#![no_main]

use core::time::Duration;

use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    entry,
    gpio,
    peripherals::Peripherals,
    prelude::*,
    rtc_cntl::{
        get_reset_reason,
        get_wakeup_cause,
        sleep::{RtcioWakeupSource, TimerWakeupSource, WakeupLevel},
        SocResetReason,
    },
    Cpu,
    Delay,
    Rtc,
    IO,
};
use esp_println::println;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let mut io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let mut rtc = Rtc::new(peripherals.LPWR);

    println!("up and runnning!");
    let reason = get_reset_reason(Cpu::ProCpu).unwrap_or(SocResetReason::ChipPowerOn);
    println!("reset reason: {:?}", reason);
    let wake_reason = get_wakeup_cause();
    println!("wake reason: {:?}", wake_reason);

    let mut delay = Delay::new(&clocks);
    let timer = TimerWakeupSource::new(Duration::from_secs(10));

    #[cfg(feature = "esp32c3")]
    let wakeup_pins: &mut [(&mut dyn gpio::RTCPinWithResistors, WakeupLevel)] = &mut [
        (&mut io.pins.gpio2, WakeupLevel::Low),
        (&mut io.pins.gpio3, WakeupLevel::High),
    ];

    #[cfg(feature = "esp32s3")]
    let mut wakeup_pins: &mut [(&mut dyn gpio::RTCPin, WakeupLevel)] =
        &mut [(&mut io.pins.gpio18, WakeupLevel::Low)];

    let rtcio = RtcioWakeupSource::new(wakeup_pins);
    println!("sleeping!");
    delay.delay_ms(100u32);
    rtc.sleep_deep(&[&timer, &rtcio], &mut delay);
}
