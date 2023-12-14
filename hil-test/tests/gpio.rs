#![no_std]
#![no_main]

use embedded_hal_1::digital::{
    InputPin as _,
    OutputPin as _,
    StatefulOutputPin as _,
    ToggleableOutputPin as _,
};
use hil_test::esp_hal::{
    gpio::{GpioPin, Input, Output, PullDown, PushPull, IO},
    peripherals::Peripherals,
};

struct Context {
    io0: GpioPin<Input<PullDown>, 0>,
    io1: GpioPin<Output<PushPull>, 1>,
}

impl Context {
    pub fn init() -> Self {
        let peripherals = Peripherals::take();
        let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

        Context {
            io0: io.pins.gpio0.into_pull_down_input(),
            io1: io.pins.gpio1.into_push_pull_output(),
        }
    }
}

#[embedded_test::tests]
mod tests {
    use defmt::{assert_eq, unwrap};

    use super::*;

    #[init]
    fn init() -> Context {
        Context::init()
    }

    #[test]
    fn test_gpio_input(ctx: Context) {
        // `InputPin`:
        assert_eq!(ctx.io0.is_low(), Ok(true));
        assert_eq!(ctx.io0.is_high(), Ok(false));
    }

    #[test]
    fn test_gpio_output(mut ctx: Context) {
        // `StatefulOutputPin`:
        assert_eq!(ctx.io1.is_set_low(), Ok(true));
        assert_eq!(ctx.io1.is_set_high(), Ok(false));
        unwrap!(ctx.io1.set_high());
        assert_eq!(ctx.io1.is_set_low(), Ok(false));
        assert_eq!(ctx.io1.is_set_high(), Ok(true));

        // `ToggleableOutputPin`:
        unwrap!(ctx.io1.toggle());
        assert_eq!(ctx.io1.is_set_low(), Ok(true));
        assert_eq!(ctx.io1.is_set_high(), Ok(false));
        unwrap!(ctx.io1.toggle());
        assert_eq!(ctx.io1.is_set_low(), Ok(false));
        assert_eq!(ctx.io1.is_set_high(), Ok(true));
    }
}
