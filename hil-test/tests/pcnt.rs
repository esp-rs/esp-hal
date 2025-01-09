//! PCNT tests

//% CHIPS: esp32 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: unstable

#![no_std]
#![no_main]

use esp_hal::{
    delay::Delay,
    gpio::{AnyPin, Input, Level, Output, Pin, Pull},
    pcnt::{channel::EdgeMode, Pcnt},
};
use hil_test as _;

struct Context<'d> {
    pcnt: Pcnt<'d>,
    input: AnyPin,
    output: AnyPin,
    delay: Delay,
}

#[cfg(test)]
#[embedded_test::tests(default_timeout = 3)]
mod tests {
    use super::*;

    #[init]
    fn init() -> Context<'static> {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let (din, dout) = hil_test::common_test_pins!(peripherals);

        let din = din.degrade();
        let dout = dout.degrade();

        Context {
            pcnt: Pcnt::new(peripherals.PCNT),
            input: din,
            output: dout,
            delay: Delay::new(),
        }
    }

    #[test]
    fn test_increment_on_pos_edge(ctx: Context<'static>) {
        let unit = ctx.pcnt.unit0;

        // Setup channel 0 to increment the count when gpio2 does LOW -> HIGH
        unit.channel0
            .set_edge_signal(Input::new(ctx.input, Pull::Down));
        unit.channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        let mut output = Output::new(ctx.output, Level::Low);

        unit.resume();

        assert_eq!(0, unit.value());

        output.set_high();
        ctx.delay.delay_micros(1);

        assert_eq!(1, unit.value());

        output.set_low();
        ctx.delay.delay_micros(1);

        assert_eq!(1, unit.value());

        output.set_high();
        ctx.delay.delay_micros(1);

        assert_eq!(2, unit.value());

        output.set_low();
        ctx.delay.delay_micros(1);

        assert_eq!(2, unit.value());
    }

    #[test]
    fn test_increment_on_neg_edge(ctx: Context<'static>) {
        let unit = ctx.pcnt.unit1;

        // Setup channel 0 to increment the count when gpio2 does LOW -> HIGH
        unit.channel0
            .set_edge_signal(Input::new(ctx.input, Pull::Up));
        unit.channel0
            .set_input_mode(EdgeMode::Increment, EdgeMode::Hold);

        let mut output = Output::new(ctx.output, Level::High);

        unit.resume();

        assert_eq!(0, unit.value());

        output.set_low();
        ctx.delay.delay_micros(1);

        assert_eq!(1, unit.value());

        output.set_high();
        ctx.delay.delay_micros(1);

        assert_eq!(1, unit.value());

        output.set_low();
        ctx.delay.delay_micros(1);

        assert_eq!(2, unit.value());

        output.set_high();
        ctx.delay.delay_micros(1);

        assert_eq!(2, unit.value());
    }

    #[test]
    fn test_increment_past_high_limit(ctx: Context<'static>) {
        let unit = ctx.pcnt.unit3;

        unit.set_high_limit(Some(3)).unwrap();

        // Setup channel 0 to increment the count when gpio2 does LOW -> HIGH
        unit.channel0
            .set_edge_signal(Input::new(ctx.input, Pull::Up));
        unit.channel0
            .set_input_mode(EdgeMode::Increment, EdgeMode::Hold);

        let mut output = Output::new(ctx.output, Level::High);

        unit.resume();

        assert_eq!(0, unit.value());

        output.set_low();
        ctx.delay.delay_micros(1);
        output.set_high();
        ctx.delay.delay_micros(1);

        assert_eq!(1, unit.value());

        output.set_low();
        ctx.delay.delay_micros(1);
        output.set_high();
        ctx.delay.delay_micros(1);

        assert_eq!(2, unit.value());

        output.set_low();
        ctx.delay.delay_micros(1);
        output.set_high();
        ctx.delay.delay_micros(1);

        assert_eq!(0, unit.value());
        assert!(unit.events().high_limit);
        assert!(unit.interrupt_is_set());

        output.set_low();
        ctx.delay.delay_micros(1);
        output.set_high();
        ctx.delay.delay_micros(1);

        assert_eq!(1, unit.value());

        // high limit event remains after next increment.
        assert!(unit.events().high_limit);

        unit.reset_interrupt();
        assert!(!unit.interrupt_is_set());

        // high limit event remains after interrupt is cleared.
        assert!(unit.events().high_limit);
    }

    #[test]
    fn test_increment_past_thresholds(ctx: Context<'static>) {
        let unit = ctx.pcnt.unit0;

        unit.set_threshold0(Some(2));
        unit.set_threshold1(Some(4));
        // For some reason this is needed for the above thresholds to apply.
        unit.clear();

        // Setup channel 0 to increment the count when gpio2 does LOW -> HIGH
        unit.channel0
            .set_edge_signal(Input::new(ctx.input, Pull::Up));
        unit.channel0
            .set_input_mode(EdgeMode::Increment, EdgeMode::Hold);

        let mut output = Output::new(ctx.output, Level::High);

        unit.resume();

        assert_eq!(0, unit.value());

        output.set_low();
        ctx.delay.delay_micros(1);
        output.set_high();
        ctx.delay.delay_micros(1);

        assert_eq!(1, unit.value());
        assert!(!unit.interrupt_is_set());
        assert!(!unit.events().threshold0);
        assert!(!unit.events().threshold1);

        output.set_low();
        ctx.delay.delay_micros(1);
        output.set_high();
        ctx.delay.delay_micros(1);

        assert_eq!(2, unit.value());
        assert!(unit.interrupt_is_set());
        assert!(unit.events().threshold0);
        assert!(!unit.events().threshold1);

        unit.reset_interrupt();

        output.set_low();
        ctx.delay.delay_micros(1);
        output.set_high();
        ctx.delay.delay_micros(1);

        assert_eq!(3, unit.value());
        assert!(!unit.interrupt_is_set());
        // threshold event remains after next increment and after interrupt is cleared.
        assert!(unit.events().threshold0);
        assert!(!unit.events().threshold1);

        output.set_low();
        ctx.delay.delay_micros(1);
        output.set_high();
        ctx.delay.delay_micros(1);

        assert_eq!(4, unit.value());
        assert!(unit.interrupt_is_set());
        // threshold event cleared after new event occurs.
        assert!(!unit.events().threshold0);
        assert!(unit.events().threshold1);
    }

    #[test]
    fn test_decrement_past_low_limit(ctx: Context<'static>) {
        let unit = ctx.pcnt.unit0;

        unit.set_low_limit(Some(-3)).unwrap();
        // For some reason this is needed for the above limit to apply.
        unit.clear();

        // Setup channel 0 to decrement the count when gpio2 does LOW -> HIGH
        unit.channel0
            .set_edge_signal(Input::new(ctx.input, Pull::Up));
        unit.channel0
            .set_input_mode(EdgeMode::Decrement, EdgeMode::Hold);

        let mut output = Output::new(ctx.output, Level::High);

        unit.resume();

        assert_eq!(0, unit.value());

        output.set_low();
        ctx.delay.delay_micros(1);
        output.set_high();
        ctx.delay.delay_micros(1);

        assert_eq!(-1, unit.value());

        output.set_low();
        ctx.delay.delay_micros(1);
        output.set_high();
        ctx.delay.delay_micros(1);

        assert_eq!(-2, unit.value());

        output.set_low();
        ctx.delay.delay_micros(1);
        output.set_high();
        ctx.delay.delay_micros(1);

        assert_eq!(0, unit.value());
        assert!(unit.events().low_limit);
        assert!(unit.interrupt_is_set());

        output.set_low();
        ctx.delay.delay_micros(1);
        output.set_high();
        ctx.delay.delay_micros(1);

        assert_eq!(-1, unit.value());

        // low limit event remains after next increment.
        assert!(unit.events().low_limit);

        unit.reset_interrupt();
        assert!(!unit.interrupt_is_set());

        // low limit event remains after interrupt is cleared.
        assert!(unit.events().low_limit);
    }

    #[test]
    fn test_unit_count_range(ctx: Context<'static>) {
        let unit = ctx.pcnt.unit2;

        // Setup channel 1 to increment the count when gpio2 does LOW -> HIGH
        unit.channel1
            .set_edge_signal(Input::new(ctx.input, Pull::Up));
        unit.channel1
            .set_input_mode(EdgeMode::Increment, EdgeMode::Hold);

        let mut output = Output::new(ctx.output, Level::High);

        unit.resume();

        assert_eq!(0, unit.value());

        for i in (0..=i16::MAX).chain(i16::MIN..0) {
            assert_eq!(i, unit.value());

            output.set_low();
            ctx.delay.delay_micros(1);
            output.set_high();
            ctx.delay.delay_micros(1);
        }

        assert_eq!(0, unit.value());

        // Channel 1 should now decrement.
        unit.channel1
            .set_input_mode(EdgeMode::Decrement, EdgeMode::Hold);

        for i in (0..=i16::MAX).chain(i16::MIN..=0).rev() {
            assert_eq!(i, unit.value());

            output.set_low();
            ctx.delay.delay_micros(1);
            output.set_high();
            ctx.delay.delay_micros(1);
        }
    }
}
