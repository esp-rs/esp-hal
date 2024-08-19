//! Demonstrates deep sleep with timer, using RTC pins as
//! wakeup.
//!
//! The following wiring is assumed for ESP32C3:
//! - RTC wakeup pin => GPIO2 (low level)
//! - RTC wakeup pin => GPIO3 (high level)
//! The following wiring is assumed for ESP32S3:
//! - RTC wakeup pin => GPIO18 (low level)

//% CHIPS: esp32c3 esp32s3

#![no_std]
#![no_main]

use core::time::Duration;

use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    entry,
    gpio,
    gpio::Io,
    rtc_cntl::{
        get_reset_reason,
        get_wakeup_cause,
        sleep::{RtcioWakeupSource, TimerWakeupSource, WakeupLevel},
        Rtc,
        SocResetReason,
    },
    Cpu,
};
use esp_println::println;

#[entry]
fn main() -> ! {
    let System {
        peripherals,
        clocks,
        ..
    } = esp_hal::init(CpuClock::boot_default());

    let mut io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    let mut rtc = Rtc::new(peripherals.LPWR);

    println!("up and runnning!");
    let reason = get_reset_reason(Cpu::ProCpu).unwrap_or(SocResetReason::ChipPowerOn);
    println!("reset reason: {:?}", reason);
    let wake_reason = get_wakeup_cause();
    println!("wake reason: {:?}", wake_reason);

    let delay = Delay::new(&clocks);
    let timer = TimerWakeupSource::new(Duration::from_secs(10));

    cfg_if::cfg_if! {
        if #[cfg(feature = "esp32c3")] {
            let wakeup_pins: &mut [(&mut dyn gpio::RtcPinWithResistors, WakeupLevel)] = &mut [
                (&mut io.pins.gpio2, WakeupLevel::Low),
                (&mut io.pins.gpio3, WakeupLevel::High),
            ];
        } else if #[cfg(feature = "esp32s3")] {
            let wakeup_pins: &mut [(&mut dyn gpio::RtcPin, WakeupLevel)] = &mut [
                (&mut io.pins.gpio18, WakeupLevel::Low)
            ];
        }
    }

    let rtcio = RtcioWakeupSource::new(wakeup_pins);
    println!("sleeping!");
    delay.delay_millis(100);
    rtc.sleep_deep(&[&timer, &rtcio]);
}
