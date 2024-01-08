//! Demonstrates deep sleep with gpio2 (low) and gpio3 (high) as wakeup sources.

#![no_std]
#![no_main]

use esp32c6_hal::{
    clock::ClockControl,
    entry,
    gpio::RTCPinWithResistors,
    peripherals::Peripherals,
    prelude::*,
    rtc_cntl::{
        get_reset_reason,
        get_wakeup_cause,
        sleep::{Ext1WakeupSource, WakeupLevel},
        SocResetReason,
    },
    Cpu,
    Delay,
    Rtc,
    IO,
};
use esp_backtrace as _;
use esp_println::println;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let mut rtc = Rtc::new(peripherals.LPWR);

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let mut pin2 = io.pins.gpio2;
    let mut pin3 = io.pins.gpio3;

    println!("up and runnning!");
    let reason = get_reset_reason(Cpu::ProCpu).unwrap_or(SocResetReason::ChipPowerOn);
    println!("reset reason: {:?}", reason);
    let wake_reason = get_wakeup_cause();
    println!("wake reason: {:?}", wake_reason);

    let mut delay = Delay::new(&clocks);
    let wakeup_pins: &mut [(&mut dyn RTCPinWithResistors, WakeupLevel)] = &mut [
        (&mut pin2, WakeupLevel::Low),
        (&mut pin3, WakeupLevel::High),
    ];

    let rtcio = Ext1WakeupSource::new(wakeup_pins);
    println!("sleeping!");
    delay.delay_ms(100u32);
    rtc.sleep_deep(&[&rtcio], &mut delay);
}
