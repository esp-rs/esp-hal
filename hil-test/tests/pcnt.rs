//! PCNT tests
//!
//! It's assumed GPIO2 is connected to GPIO3

//% CHIPS: esp32 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use defmt_rtt as _;
use esp_backtrace as _;
use esp_hal::{delay::Delay, gpio::GpioPin, pcnt::Pcnt};

struct Context<'d> {
    pcnt: Pcnt<'d>,
    gpio2: GpioPin<2>,
    gpio3: GpioPin<3>,
    delay: Delay,
}

#[cfg(test)]
#[embedded_test::tests]
mod tests {
    use esp_hal::{
        clock::ClockControl,
        delay::Delay,
        gpio::{Io, Level, Output, Pull},
        pcnt::{
            channel,
            channel::{CtrlMode, EdgeMode, PcntInputConfig, PcntSource},
            unit,
        },
        peripherals::Peripherals,
        system::SystemControl,
    };

    use super::*;

    #[init]
    fn init() -> Context<'static> {
        let peripherals = Peripherals::take();
        let system = SystemControl::new(peripherals.SYSTEM);
        let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

        let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

        Context {
            pcnt: Pcnt::new(peripherals.PCNT, None),
            gpio2: io.pins.gpio2,
            gpio3: io.pins.gpio3,
            delay: Delay::new(&clocks),
        }
    }

    #[test]
    fn test_increment_on_pos_edge(ctx: Context<'static>) {
        let mut unit = ctx.pcnt.get_unit(unit::Number::Unit0);
        unit.configure(unit::Config {
            low_limit: -100,
            high_limit: 100,
            ..Default::default()
        })
        .unwrap();

        // Setup channel 0 to increment the count when gpio2 does LOW -> HIGH
        unit.get_channel(channel::Number::Channel0).configure(
            PcntSource::always_high(),
            PcntSource::from_pin(ctx.gpio2, PcntInputConfig { pull: Pull::Down }),
            channel::Config {
                lctrl_mode: CtrlMode::Keep,
                hctrl_mode: CtrlMode::Keep,
                pos_edge: EdgeMode::Increment,
                neg_edge: EdgeMode::Hold,
                invert_ctrl: false,
                invert_sig: false,
            },
        );

        // Disable channel 1.
        unit.get_channel(channel::Number::Channel1).configure(
            PcntSource::always_high(),
            PcntSource::always_high(),
            channel::Config {
                lctrl_mode: CtrlMode::Keep,
                hctrl_mode: CtrlMode::Keep,
                pos_edge: EdgeMode::Hold,
                neg_edge: EdgeMode::Hold,
                invert_ctrl: false,
                invert_sig: false,
            },
        );

        let mut output = Output::new(ctx.gpio3, Level::Low);

        unit.resume();

        assert_eq!(0, unit.get_value());

        output.set_high();
        ctx.delay.delay_micros(1);

        assert_eq!(1, unit.get_value());

        output.set_low();
        ctx.delay.delay_micros(1);

        assert_eq!(1, unit.get_value());

        output.set_high();
        ctx.delay.delay_micros(1);

        assert_eq!(2, unit.get_value());

        output.set_low();
        ctx.delay.delay_micros(1);

        assert_eq!(2, unit.get_value());
    }
}
