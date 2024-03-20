//! GPIO Test
//!
//! Folowing pins are used:
//! GPIO2
//! GPIO4

#![no_std]
#![no_main]

#[cfg(feature = "defmt")]
use defmt_rtt as _;
use embedded_hal_1::digital::{InputPin as _, OutputPin as _, StatefulOutputPin as _};
use esp_hal::{
    gpio::{GpioPin, Input, Output, PullDown, PushPull, IO},
    peripherals::Peripherals,
};

struct Context {
    io2: GpioPin<Input<PullDown>, 2>,
    io4: GpioPin<Output<PushPull>, 4>,
}

impl Context {
    pub fn init() -> Self {
        let peripherals = Peripherals::take();
        let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

        Context {
            io2: io.pins.gpio2.into_pull_down_input(),
            io4: io.pins.gpio4.into_push_pull_output(),
        }
    }
}

#[cfg(test)]
#[embedded_test::tests]
mod tests {
    #[cfg(feature = "defmt")]
    use defmt::assert_eq;

    use super::*;

    #[init]
    fn init() -> Context {
        Context::init()
    }

    #[test]
    fn test_gpio_input(mut ctx: Context) {
        // `InputPin`:
        assert_eq!(ctx.io2.is_low(), Ok(true));
        assert_eq!(ctx.io2.is_high(), Ok(false));
    }

    #[test]
    fn test_gpio_output(mut ctx: Context) {
        // `StatefulOutputPin`:
        assert_eq!(ctx.io4.is_set_low(), Ok(true));
        assert_eq!(ctx.io4.is_set_high(), Ok(false));
        assert!(ctx.io4.set_high().is_ok());
        assert_eq!(ctx.io4.is_set_low(), Ok(false));
        assert_eq!(ctx.io4.is_set_high(), Ok(true));

        // `ToggleableOutputPin`:
        assert!(ctx.io4.toggle().is_ok());
        assert_eq!(ctx.io4.is_set_low(), Ok(true));
        assert_eq!(ctx.io4.is_set_high(), Ok(false));
        assert!(ctx.io4.toggle().is_ok());
        assert_eq!(ctx.io4.is_set_low(), Ok(false));
        assert_eq!(ctx.io4.is_set_high(), Ok(true));
        // Leave in initial state for next test
        assert!(ctx.io4.toggle().is_ok());
    }
}
