//! Demonstrates deep sleep with timer and ext0 (using gpio18) wakeup

#![no_std]
#![no_main]

use core::time::Duration;

use esp32s3_hal as hal;
use esp_backtrace as _;
use esp_hal_common::{
    gpio::{RTCPin, RTCPinWithResistors},
    rtc_cntl::sleep::RtcioWakeupSource,
};
use esp_println::println;
use hal::{
    clock::ClockControl,
    entry,
    peripherals::Peripherals,
    prelude::*,
    rtc_cntl::{
        get_reset_reason,
        get_wakeup_cause,
        sleep::{TimerWakeupSource, WakeupLevel},
        SocResetReason,
    },
    timer::TimerGroup,
    Delay,
    Rtc,
    IO,
};

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let mut system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    // Disable the RTC and TIMG watchdog timers
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    let timer_group0 = TimerGroup::new(
        peripherals.TIMG0,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(
        peripherals.TIMG1,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut wdt1 = timer_group1.wdt;

    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let mut rtcio_pin18 = io.pins.gpio18;

    rtcio_pin18.rtcio_pad_hold(true);
    rtcio_pin18.rtcio_pullup(true);

    println!("up and runnning!");
    let reason = get_reset_reason(hal::Cpu::ProCpu).unwrap_or(SocResetReason::ChipPowerOn);
    println!("reset reason: {:?}", reason);
    let wake_reason = get_wakeup_cause();
    println!("wake reason: {:?}", wake_reason);

    let mut delay = Delay::new(&clocks);

    let timer = TimerWakeupSource::new(Duration::from_secs(30));

    let mut wakeup_pins: [(&mut dyn RTCPin, WakeupLevel); 1] =
        [(&mut rtcio_pin18, WakeupLevel::Low)];
    let rtcio = RtcioWakeupSource::new(&mut wakeup_pins);
    println!("sleeping!");
    delay.delay_ms(100u32);
    rtc.sleep_deep(&[&timer, &rtcio], &mut delay);
}
