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
    time::Rate,
};

const PWM_FREQUENCY: Rate = Rate::from_khz(2);
const SAMPLE_COUNT: usize = 400;
const SAMPLE_INTERVAL_US: u32 = 37;

fn sample_high_count(input: &Input<'_>, delay: &Delay) -> usize {
    let mut high_count = 0;

    for _ in 0..SAMPLE_COUNT {
        if input.is_high() {
            high_count += 1;
        }

        delay.delay_micros(SAMPLE_INTERVAL_US);
    }

    high_count
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
        let low_duty_high_count = sample_high_count(&input, &delay);

        channel0.set_duty(80).unwrap();

        delay.delay_millis(4);
        let high_duty_high_count = sample_high_count(&input, &delay);

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
        let zero_duty_high_count = sample_high_count(&input, &delay);
        assert_eq!(zero_duty_high_count, 0);

        channel0.set_duty(50).unwrap();
        delay.delay_millis(4);

        let mid_duty_high_count = sample_high_count(&input, &delay);

        assert!(mid_duty_high_count > SAMPLE_COUNT / 5);
        assert!(mid_duty_high_count < SAMPLE_COUNT * 4 / 5);
    }
}
