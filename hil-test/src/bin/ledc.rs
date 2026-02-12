//! LEDC HIL tests.
//!
//! This test uses one `common_test_pins!` pin split into input/output drivers. LEDC drives the
//! output half and the input half samples the resulting signal.

//% CHIPS: esp32 esp32c2 esp32c3 esp32c5 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: unstable

#![no_std]
#![no_main]

use esp_hal::{
    delay::Delay,
    gpio::{AnyPin, Flex, Input, Pin},
    ledc::{
        LSGlobalClkSource, Ledc, LowSpeed,
        channel::{self, ChannelIFace},
        timer::{self, TimerIFace},
    },
    peripherals::LEDC,
    time::{Instant, Rate},
};

const PWM_FREQUENCY: Rate = Rate::from_khz(2);
const SAMPLE_COUNT: usize = 1_000_000;
const PERIOD_SAMPLE_EDGES: usize = 32;
const PERIOD_TOLERANCE_PERCENT: u32 = 20;

fn sample_high_count(input: &Input<'_>) -> usize {
    let mut high_count = 0;

    for _ in 0..SAMPLE_COUNT {
        if input.is_high() {
            high_count += 1;
        }
        core::hint::spin_loop();
    }

    high_count
}

fn average_period_us(input: &Input<'_>, edges: usize) -> u32 {
    assert!(edges > 1);

    let mut prev = input.is_high();

    // Wait for the first rising edge.
    loop {
        let now = input.is_high();
        if !prev && now {
            prev = now;
            break;
        }
        prev = now;
        core::hint::spin_loop();
    }

    let first_edge = Instant::now();
    let mut last_edge = first_edge;
    let mut seen_edges = 1usize;

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

    #[test]
    fn duty_cycle_update_changes_high_ratio(ctx: Context) {
        let Context {
            ledc,
            test_pin,
            delay,
        } = ctx;

        let pin = Flex::new(test_pin);
        // SAFETY: we only configure the pin output through LEDC and only sample the input.
        let (input, output) = unsafe { pin.split_into_drivers() };

        let mut ledc = Ledc::new(ledc);
        ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);

        let mut timer0 = ledc.timer::<LowSpeed>(timer::Number::Timer0);
        timer0
            .configure(timer::config::Config {
                duty: timer::config::Duty::Duty8Bit,
                clock_source: timer::LSClockSource::APBClk,
                frequency: PWM_FREQUENCY,
            })
            .unwrap();

        let mut channel0 = ledc.channel(channel::Number::Channel0, output);
        channel0
            .configure(channel::config::Config {
                timer: &timer0,
                duty_pct: 20,
                drive_mode: esp_hal::gpio::DriveMode::PushPull,
            })
            .unwrap();

        delay.delay_millis(4);
        let low_duty_high_count = sample_high_count(&input);

        channel0.set_duty(80).unwrap();

        delay.delay_millis(4);
        let high_duty_high_count = sample_high_count(&input);

        assert!(low_duty_high_count < SAMPLE_COUNT * 2 / 5);
        assert!(high_duty_high_count > SAMPLE_COUNT * 3 / 5);
    }

    #[test]
    fn zero_percent_duty_is_constant_low(ctx: Context) {
        let Context {
            ledc,
            test_pin,
            delay,
        } = ctx;

        let pin = Flex::new(test_pin);
        // SAFETY: we only configure the pin output through LEDC and only sample the input.
        let (input, output) = unsafe { pin.split_into_drivers() };

        let mut ledc = Ledc::new(ledc);
        ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);

        let mut timer0 = ledc.timer::<LowSpeed>(timer::Number::Timer0);
        timer0
            .configure(timer::config::Config {
                duty: timer::config::Duty::Duty8Bit,
                clock_source: timer::LSClockSource::APBClk,
                frequency: PWM_FREQUENCY,
            })
            .unwrap();

        let mut channel0 = ledc.channel(channel::Number::Channel0, output);
        channel0
            .configure(channel::config::Config {
                timer: &timer0,
                duty_pct: 0,
                drive_mode: esp_hal::gpio::DriveMode::PushPull,
            })
            .unwrap();

        delay.delay_millis(4);
        let zero_duty_high_count = sample_high_count(&input);
        assert_eq!(zero_duty_high_count, 0);

        channel0.set_duty(50).unwrap();
        delay.delay_millis(4);

        let mid_duty_high_count = sample_high_count(&input);

        assert!(mid_duty_high_count > SAMPLE_COUNT / 5);
        assert!(mid_duty_high_count < SAMPLE_COUNT * 4 / 5);
    }

    #[test]
    fn configured_frequency_matches_output_period(ctx: Context) {
        let Context {
            ledc,
            test_pin,
            delay,
        } = ctx;

        let pin = Flex::new(test_pin);
        // SAFETY: we only configure the pin output through LEDC and only sample the input.
        let (input, output) = unsafe { pin.split_into_drivers() };

        let mut ledc = Ledc::new(ledc);
        ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);

        let mut timer0 = ledc.timer::<LowSpeed>(timer::Number::Timer0);
        timer0
            .configure(timer::config::Config {
                duty: timer::config::Duty::Duty8Bit,
                clock_source: timer::LSClockSource::APBClk,
                frequency: PWM_FREQUENCY,
            })
            .unwrap();

        let mut channel0 = ledc.channel(channel::Number::Channel0, output);
        channel0
            .configure(channel::config::Config {
                timer: &timer0,
                duty_pct: 50,
                drive_mode: esp_hal::gpio::DriveMode::PushPull,
            })
            .unwrap();

        delay.delay_millis(4);

        let measured_period_us = average_period_us(&input, PERIOD_SAMPLE_EDGES);

        let expected_period_us = 1_000_000u32 / PWM_FREQUENCY.as_hz();
        let min_period_us = expected_period_us * (100 - PERIOD_TOLERANCE_PERCENT) / 100;
        let max_period_us = expected_period_us * (100 + PERIOD_TOLERANCE_PERCENT) / 100;

        assert!(
            measured_period_us >= min_period_us && measured_period_us <= max_period_us,
            "measured period {} us, expected {} us (+/- {}%)",
            measured_period_us,
            expected_period_us,
            PERIOD_TOLERANCE_PERCENT
        );
    }

    #[cfg(all(esp32c5, ledc_has_gamma_fade))]
    #[test]
    fn duty_fade_changes_output_and_completes(ctx: Context) {
        let Context {
            ledc,
            test_pin,
            delay,
        } = ctx;

        let pin = Flex::new(test_pin);
        // SAFETY: we only configure the pin output through LEDC and only sample the input.
        let (input, output) = unsafe { pin.split_into_drivers() };

        let mut ledc = Ledc::new(ledc);
        ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);

        let mut timer0 = ledc.timer::<LowSpeed>(timer::Number::Timer0);
        timer0
            .configure(timer::config::Config {
                duty: timer::config::Duty::Duty8Bit,
                clock_source: timer::LSClockSource::APBClk,
                frequency: PWM_FREQUENCY,
            })
            .unwrap();

        let mut channel0 = ledc.channel(channel::Number::Channel0, output);
        channel0
            .configure(channel::config::Config {
                timer: &timer0,
                duty_pct: 10,
                drive_mode: esp_hal::gpio::DriveMode::PushPull,
            })
            .unwrap();

        delay.delay_millis(6);
        let initial_high_count = sample_high_count(&input);
        assert!(initial_high_count < SAMPLE_COUNT / 3);

        channel0.start_duty_fade(10, 90, 200).unwrap();

        while !channel0.is_duty_fade_running() {
            core::hint::spin_loop();
        }

        delay.delay_millis(260);
        assert!(!channel0.is_duty_fade_running());

        let final_high_count = sample_high_count(&input);
        assert!(final_high_count > SAMPLE_COUNT * 2 / 3);
    }
}
