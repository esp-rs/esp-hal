//! RMT / PulseControl
//!
//! Folowing pins are used:
//! GPIO4 (RMT channel 0)
//!
//! Depending on your target and the board you are using you have to change the
//! pins.
//!
//! This is an example of generating pulses. Attach a Logic Analyzer to the RTM
//! channel 0 pin to see the gerated signal.

#![no_std]
#![no_main]

use esp32s3_hal::{
    clock::ClockControl,
    gpio::IO,
    peripherals::Peripherals,
    prelude::*,
    pulse_control::{ClockSource, ConfiguredChannel, OutputChannel, PulseCode, RepeatMode},
    timer::TimerGroup,
    PulseControl,
    Rtc,
};
use esp_backtrace as _;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let mut system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let timer_group0 = TimerGroup::new(
        peripherals.TIMG0,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut wdt = timer_group0.wdt;
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);

    // Disable MWDT and RWDT (Watchdog) flash boot protection
    wdt.disable();
    rtc.rwdt.disable();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    // Configure RMT peripheral globally
    let pulse = PulseControl::new(
        peripherals.RMT,
        &mut system.peripheral_clock_control,
        ClockSource::APB,
        0,
        0,
        0,
    )
    .unwrap();

    let mut rmt_channel0 = pulse.channel0;

    // Set up channel
    rmt_channel0
        .set_idle_output_level(false)
        .set_carrier_modulation(false)
        .set_channel_divider(1)
        .set_idle_output(true);

    // Assign GPIO pin where pulses should be sent to
    let mut rmt_channel0 = rmt_channel0.assign_pin(io.pins.gpio4);

    // Create pulse sequence
    let mut seq = [PulseCode {
        level1: true,
        length1: 0u32.nanos(),
        level2: false,
        length2: 0u32.nanos(),
    }; 128];

    // -1 to make sure that the last element is a transmission end marker (i.e.
    // lenght 0)
    for i in 0..(seq.len() - 1) {
        seq[i] = PulseCode {
            level1: true,
            length1: (10u32 * (i as u32 + 1u32)).nanos(),
            level2: false,
            length2: 60u32.nanos(),
        };
    }

    loop {
        // Send sequence
        rmt_channel0
            .send_pulse_sequence(RepeatMode::SingleShot, &seq)
            .unwrap();
    }
}
