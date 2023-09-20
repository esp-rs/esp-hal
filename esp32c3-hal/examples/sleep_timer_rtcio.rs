//! Demonstrates deep sleep with timer and ext0 (using gpio18) wakeup

#![no_std]
#![no_main]

use core::time::Duration;

use esp32c3_hal as hal;
use esp_backtrace as _;
use esp_hal_common::{gpio::RTCPin, rtc_cntl::sleep::RtcioWakeupSource};
use esp_hal_smartled::{smartLedAdapter, SmartLedsAdapter};
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
    Delay,
    Rmt,
    Rtc,
    IO,
};
use smart_leds::{SmartLedsWrite, RGB8};

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let mut system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let mut rtc = Rtc::new(peripherals.RTC_CNTL);

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let mut pin2 = io.pins.gpio2;

    println!("up and runnning!");
    let reason = get_reset_reason(hal::Cpu::ProCpu).unwrap_or(SocResetReason::ChipPowerOn);
    println!("reset reason: {:?}", reason);
    let wake_reason = get_wakeup_cause();
    println!("wake reason: {:?}", wake_reason);

    // Configure RMT peripheral globally
    let rmt = Rmt::new(
        peripherals.RMT,
        80u32.MHz(),
        &mut system.peripheral_clock_control,
        &clocks,
    )
    .unwrap();

    let mut delay = Delay::new(&clocks);

    // We use one of the RMT channels to instantiate a `SmartLedsAdapter` which can
    // be used directly with all `smart_led` implementations
    let mut led = <smartLedAdapter!(0, 1)>::new(rmt.channel0, io.pins.gpio7);
    led.write(core::iter::once(RGB8::new(5, 0, 0))).unwrap();
    delay.delay_ms(1000u32);
    led.write(core::iter::once(RGB8::new(0, 0, 0))).unwrap();

    let timer = TimerWakeupSource::new(Duration::from_secs(10));

    let mut wakeup_pins: [(&mut dyn RTCPin, WakeupLevel); 1] = [(&mut pin2, WakeupLevel::Low)];
    let rtcio = RtcioWakeupSource::new(&mut wakeup_pins);
    println!("sleeping!");
    delay.delay_ms(100u32);
    rtc.sleep_deep(&[&timer, &rtcio], &mut delay);
}
