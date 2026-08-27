//! LEDC Test
//% CHIP_FILTER: pcnt_driver_supported
//% FEATURES: unstable

#![no_std]
#![no_main]

#[embedded_test::tests(default_timeout = 3)]
mod tests {
    use esp_hal::{
        Blocking,
        delay::Delay,
        gpio::{AnyPin, Flex, Pin},
        ledc::{
            Config as LedcConfig,
            Ledc,
            channel::Config as ChannelConfig,
            timer::{Config as TimerConfig, Duty},
        },
        pcnt::{Pcnt, channel::EdgeMode},
        time::{Instant, Rate},
    };

    struct Context {
        ledc: Ledc<'static, Blocking>,
        pcnt: Pcnt<'static>,
        pin1: AnyPin<'static>,
        pin2: AnyPin<'static>,
    }

    #[init]
    fn init() -> Context {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let (pin1, pin2) = hil_test::common_test_pins!(peripherals);

        let pin1 = pin1.degrade();
        let pin2 = pin2.degrade();

        Context {
            ledc: Ledc::new(peripherals.LEDC, LedcConfig::default()).unwrap(),
            pcnt: Pcnt::new(peripherals.PCNT),
            pin1,
            pin2,
        }
    }

    #[test]
    fn test_ledc_pwm(ctx: Context) {
        let ledc = ctx.ledc;
        let pcnt = ctx.pcnt;

        let timer0_config = TimerConfig::default()
            .with_duty(Duty::Bit10)
            .with_frequency(Rate::from_khz(1));

        let timer0 = ledc.timer0.configure(timer0_config).unwrap();
        let mut channel0 = ledc
            .channel0
            .configure(&timer0, ChannelConfig::default())
            .unwrap()
            .with_pin(ctx.pin1);

        let unit = pcnt.unit0;

        let mut pin2_flex = Flex::new(ctx.pin2);
        pin2_flex.set_input_enable(true);
        let pin2_in = pin2_flex.peripheral_input();

        unit.channel0.set_edge_signal(pin2_in);
        unit.channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        let delay = Delay::new();
        unit.clear();
        unit.resume();

        let max_duty = channel0.max_duty_cycle();

        // 50% Duty
        channel0.set_duty_cycle(max_duty / 2);
        delay.delay_millis(100);

        let count = unit.value();
        assert!(count >= 90 && count <= 110, "{count}");

        // Fading to 0 duty
        let start_time = Instant::now();
        let fade_time_ms = 250;
        channel0
            .start_duty_fade(max_duty / 2, 0, fade_time_ms)
            .unwrap();

        while channel0.is_duty_fade_running() {
            delay.delay_millis(1);
        }

        let elapsed = start_time.elapsed().as_millis();

        assert!(elapsed >= 240 && elapsed <= 260, "{elapsed}");

        // 0% Duty
        channel0.set_duty_cycle(0);
        delay.delay_millis(10);
        unit.clear();
        delay.delay_millis(100);

        let value = unit.value();
        assert_eq!(value, 0, "{value}");

        // 100% Duty
        channel0.set_duty_cycle(max_duty);
        delay.delay_millis(10);
        unit.clear();
        delay.delay_millis(100);

        let value = unit.value();
        assert_eq!(value, 0, "{value}");

        // Updating timer to 2kHz
        let timer1_config = TimerConfig::default()
            .with_duty(Duty::Bit10)
            .with_frequency(Rate::from_khz(2));
        let timer1 = ledc.timer1.configure(timer1_config).unwrap();

        let mut channel0 = channel0.with_timer(&timer1);

        channel0.set_duty_cycle(max_duty / 2);
        delay.delay_millis(10);
        unit.clear();
        delay.delay_millis(100);

        let count = unit.value();
        assert!(count >= 190 && count <= 210, "{count}");
    }
}
