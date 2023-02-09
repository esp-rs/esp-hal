//! This demos basic usage of RMT / PulseControl
//! Use a logic analyzer to see the generated pulses.
//! The correct output is only achieved when running in release mode.

#![no_std]
#![no_main]

use esp32_hal::{
    clock::ClockControl,
    gpio::IO,
    peripherals::Peripherals,
    prelude::*,
    pulse_control::{ConfiguredChannel, OutputChannel, PulseCode, RepeatMode},
    timer::TimerGroup,
    PulseControl,
    Rtc,
};
use esp_backtrace as _;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let mut system = peripherals.DPORT.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut wdt = timer_group0.wdt;
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);

    // Disable MWDT and RWDT (Watchdog) flash boot protection
    wdt.disable();
    rtc.rwdt.disable();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    // Configure RMT peripheral globally
    let pulse = PulseControl::new(peripherals.RMT, &mut system.peripheral_clock_control).unwrap();

    let mut rmt_channel0 = pulse.channel0;

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
