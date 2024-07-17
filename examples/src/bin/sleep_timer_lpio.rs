//! Demonstrates deep sleep with timer, using low and high level pins as wakeup sources.
//!
//! The following wiring is assumed:
//! - ext1 wakeup pin => GPIO2 (low level)
//!                   => GPIO3 (high level)

//% CHIPS: esp32c6

#![no_std]
#![no_main]

use core::time::Duration;

use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    delay::Delay,
    entry,
    gpio::{Io, RtcPinWithResistors},
    peripherals::Peripherals,
    rtc_cntl::{
        get_reset_reason,
        get_wakeup_cause,
        sleep::{Ext1WakeupSource, TimerWakeupSource, WakeupLevel},
        Rtc,
        SocResetReason,
    },
    system::SystemControl,
    Cpu,
};
use esp_println::println;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let mut rtc = Rtc::new(peripherals.LPWR);

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    let mut pin2 = io.pins.gpio2;
    let mut pin3 = io.pins.gpio3;

    println!("up and runnning!");
    let reason = get_reset_reason(Cpu::ProCpu).unwrap_or(SocResetReason::ChipPowerOn);
    println!("reset reason: {:?}", reason);
    let wake_reason = get_wakeup_cause();
    println!("wake reason: {:?}", wake_reason);

    let delay = Delay::new(&clocks);
    let timer = TimerWakeupSource::new(Duration::from_secs(10));

    let wakeup_pins: &mut [(&mut dyn RtcPinWithResistors, WakeupLevel)] = &mut [
        (&mut pin2, WakeupLevel::Low),
        (&mut pin3, WakeupLevel::High),
    ];

    let rtcio = Ext1WakeupSource::new(wakeup_pins);
    println!("sleeping!");
    delay.delay_millis(100);
    rtc.sleep_deep(&[&timer, &rtcio]);
}
