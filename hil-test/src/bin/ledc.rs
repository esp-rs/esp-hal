//! LEDC output-frequency HIL test.
//!
//! Verifies that a LowSpeed LEDC timer produces PWM at the frequency it was
//! configured for, by measuring the real signal period on hardware.
//!
//! This guards the timer-divisor computation, which derives the divisor from the
//! *source clock* the LEDC slow-clock mux actually selects. If the driver models
//! the wrong source frequency, the divisor is miscalculated and the output comes
//! out at a fixed ratio of the requested rate - a mistake that a purely
//! register-level or `frequency()`-getter check cannot catch, because the getter
//! returns the *requested* value, not the achieved one. Concretely, on the
//! ESP32-H2 `set_global_slow_clock(APBClk)` selects the 32 MHz XTAL while the
//! divisor was briefly computed from the 96 MHz "APB" figure, so every timer ran
//! at 1/3 of its configured frequency (see PR #5941).
//!
//! No external wiring: one `common_test_pins!` pin is split into an input and an
//! output driver, LEDC drives the output half and the input half samples it.

//% CHIP_FILTER: ledc_driver_supported
//% FEATURES: unstable

#![no_std]
#![no_main]

use esp_hal::{
    delay::Delay,
    gpio::{AnyPin, DriveMode, Flex, Input, Pin},
    ledc::{
        LSGlobalClkSource,
        Ledc,
        LowSpeed,
        channel::{self, ChannelIFace},
        timer::{self, TimerIFace},
    },
    peripherals::LEDC,
    time::{Instant, Rate},
};
#[allow(unused_imports)]
use hil_test::{assert, assert_eq};

/// Rising edges to average the period over. More edges tighten the estimate; 64
/// edges is ~32 ms at 2 kHz, comfortably inside the default test timeout.
const PERIOD_SAMPLE_EDGES: usize = 64;

/// Allowed deviation of the measured period from the configured one. GPIO-polled
/// edge timing carries some jitter, so the bound is generous; the bug this test
/// exists for shifts the frequency by ~3x (a ~200% error), an order of magnitude
/// past this tolerance.
const PERIOD_TOLERANCE_PERCENT: u32 = 20;

/// Measure the average PWM period (in microseconds) over `edges` rising edges of
/// `input`, using the hardware timer for timestamps (not a loop counter, so it is
/// insensitive to polling-loop speed).
///
/// A stuck (never-toggling) output never reaches `edges` and surfaces as a test
/// timeout rather than a wrong value - still a failure, which is the point.
fn average_period_us(input: &Input<'_>, edges: usize) -> u32 {
    assert!(edges > 1);

    let mut prev = input.is_high();

    // Align to the first rising edge so the first measured interval is a full period.
    loop {
        let now = input.is_high();
        if !prev && now {
            break;
        }
        prev = now;
        core::hint::spin_loop();
    }

    let first_edge = Instant::now();
    let mut last_edge = first_edge;
    let mut seen_edges = 1usize;
    prev = true;

    while seen_edges < edges {
        let now = input.is_high();
        if !prev && now {
            seen_edges += 1;
            last_edge = Instant::now();
        }
        prev = now;
        core::hint::spin_loop();
    }

    let total_us = (last_edge - first_edge).as_micros() as u32;
    total_us / (edges as u32 - 1)
}

/// Configure a LowSpeed timer+channel at `frequency` (50% duty) on `test_pin` and
/// assert the measured output period matches, within `PERIOD_TOLERANCE_PERCENT`.
fn assert_output_frequency(
    ledc: LEDC<'static>,
    test_pin: AnyPin<'static>,
    delay: Delay,
    frequency: Rate,
) {
    let pin = Flex::new(test_pin);
    // SAFETY: the output half is driven only by LEDC and the input half is only sampled.
    let (input, output) = unsafe { pin.split_into_drivers() };

    let mut ledc = Ledc::new(ledc);
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);

    let mut timer0 = ledc.timer::<LowSpeed>(timer::Number::Timer0);
    timer0
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty8Bit,
            clock_source: timer::LSClockSource::APBClk,
            frequency,
        })
        .unwrap();

    let mut channel0 = ledc.channel(channel::Number::Channel0, output);
    channel0
        .configure(channel::config::Config {
            timer: &timer0,
            duty_pct: 50,
            drive_mode: DriveMode::PushPull,
        })
        .unwrap();

    // Let the output settle before sampling.
    delay.delay_millis(4);

    let measured_us = average_period_us(&input, PERIOD_SAMPLE_EDGES);

    let expected_us = 1_000_000u32 / frequency.as_hz();
    let min_us = expected_us * (100 - PERIOD_TOLERANCE_PERCENT) / 100;
    let max_us = expected_us * (100 + PERIOD_TOLERANCE_PERCENT) / 100;

    assert!(
        measured_us >= min_us && measured_us <= max_us,
        "measured period {} us at {} Hz, expected {} us (+/- {}%)",
        measured_us,
        frequency.as_hz(),
        expected_us,
        PERIOD_TOLERANCE_PERCENT
    );
}

struct Context {
    ledc: LEDC<'static>,
    test_pin: AnyPin<'static>,
    delay: Delay,
}

#[embedded_test::tests(default_timeout = 3, executor = hil_test::Executor::new())]
mod tests {
    use super::*;

    #[init]
    fn init() -> Context {
        let peripherals = esp_hal::init(esp_hal::Config::default());
        let (_, test_pin) = hil_test::common_test_pins!(peripherals);

        Context {
            ledc: peripherals.LEDC,
            test_pin: test_pin.degrade(),
            delay: Delay::new(),
        }
    }

    /// The core regression guard: a mid-range frequency must appear at the output.
    /// Off-by-source-clock divisor bugs make this a fixed ratio too high or low.
    #[test]
    fn output_frequency_matches_configuration_2khz(ctx: Context) {
        assert_output_frequency(ctx.ledc, ctx.test_pin, ctx.delay, Rate::from_khz(2));
    }

    /// A second, 4x-lower frequency: since a wrong source clock scales *every*
    /// frequency by the same factor, this both re-confirms the fix and catches a
    /// regression where the output ignores the configured rate altogether.
    #[test]
    fn output_frequency_matches_configuration_500hz(ctx: Context) {
        assert_output_frequency(ctx.ledc, ctx.test_pin, ctx.delay, Rate::from_hz(500));
    }
}
