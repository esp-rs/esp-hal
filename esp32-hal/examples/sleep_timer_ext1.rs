//! Demonstrates deep sleep with timer and ext1 (using gpio27 & gpio23) wakeup

#![no_std]
#![no_main]

use core::time::Duration;

use esp32_hal as hal;
use esp_backtrace as _;
use esp_println::println;
use hal::{
    clock::ClockControl,
    entry,
    peripherals::Peripherals,
    prelude::*,
    rtc_cntl::{
        get_reset_reason,
        get_wakeup_cause,
        sleep::{Ext1WakeupSource, TimerWakeupSource, WakeupLevel},
        SocResetReason,
    },
    Delay,
    Rtc,
    IO,
};

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.DPORT.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let mut rtc = Rtc::new(peripherals.RTC_CNTL);

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let mut pin27 = io.pins.gpio27;
    let mut pin32 = io.pins.gpio32;

    println!("up and runnning!");
    let reason = get_reset_reason(hal::Cpu::ProCpu).unwrap_or(SocResetReason::ChipPowerOn);
    println!("reset reason: {:?}", reason);
    let wake_reason = get_wakeup_cause();
    println!("wake reason: {:?}", wake_reason);

    let mut delay = Delay::new(&clocks);

    let timer = TimerWakeupSource::new(Duration::from_secs(30));
    let mut wakeup_pins: [&mut dyn hal::gpio::RTCPin; 2] = [&mut pin27, &mut pin32];
    let ext1 = Ext1WakeupSource::new(&mut wakeup_pins, WakeupLevel::High);
    println!("sleeping!");
    delay.delay_ms(100u32);
    rtc.sleep_deep(&[&timer, &ext1], &mut delay);
}
