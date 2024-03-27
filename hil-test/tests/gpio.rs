//! GPIO Test
//!
//! Folowing pins are used:
//! GPIO2
//! GPIO4

#![no_std]
#![no_main]

use core::cell::RefCell;

use critical_section::Mutex;
use defmt_rtt as _;
use embedded_hal::digital::{InputPin as _, OutputPin as _, StatefulOutputPin as _};
use esp_hal::{
    clock::ClockControl,
    delay::Delay,
    gpio::{GpioPin, Input, Output, OutputPin, PullDown, PushPull, IO},
    macros::handler,
    peripherals::Peripherals,
    system::SystemExt,
};

static COUNTER: Mutex<RefCell<u32>> = Mutex::new(RefCell::new(0));
static INPUT_PIN: Mutex<RefCell<Option<esp_hal::gpio::Gpio2<Input<PullDown>>>>> =
    Mutex::new(RefCell::new(None));

struct Context {
    io2: GpioPin<Input<PullDown>, 2>,
    io4: GpioPin<Output<PushPull>, 4>,
    delay: Delay,
}

impl Context {
    pub fn init() -> Self {
        let peripherals = Peripherals::take();
        let system = peripherals.SYSTEM.split();
        let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

        let mut io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
        io.set_interrupt_handler(interrupt_handler);

        let delay = Delay::new(&clocks);

        Context {
            io2: io.pins.gpio2.into_pull_down_input(),
            io4: io.pins.gpio4.into_push_pull_output(),
            delay,
        }
    }
}

#[handler]
pub fn interrupt_handler() {
    critical_section::with(|cs| {
        use esp_hal::gpio::Pin;

        *COUNTER.borrow_ref_mut(cs) += 1;
        INPUT_PIN
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .clear_interrupt();
    });
}

#[cfg(test)]
#[embedded_test::tests]
mod tests {
    use defmt::assert_eq;
    use esp_hal::gpio::{Event, Pin};

    use super::*;

    #[init]
    fn init() -> Context {
        let mut ctx = Context::init();
        // make sure tests don't interfere with each other
        ctx.io4.set_low().ok();
        ctx
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

    #[test]
    fn test_gpio_interrupt(mut ctx: Context) {
        assert!(ctx.io4.set_high().is_ok());
        critical_section::with(|cs| {
            *COUNTER.borrow_ref_mut(cs) = 0;
            ctx.io2.listen(Event::AnyEdge);
            INPUT_PIN.borrow_ref_mut(cs).replace(ctx.io2);
        });
        assert!(ctx.io4.set_low().is_ok());
        assert!(ctx.io4.set_high().is_ok());
        assert!(ctx.io4.set_low().is_ok());

        // make sure the interrupt is serviced before reading the counter
        ctx.delay.delay_millis(5);

        let count = critical_section::with(|cs| *COUNTER.borrow_ref(cs));
        assert_eq!(count, 3);

        ctx.io2 = critical_section::with(|cs| INPUT_PIN.borrow_ref_mut(cs).take().unwrap());
        ctx.io2.unlisten();
    }

    #[test]
    fn test_gpio_od(ctx: Context) {
        let mut io2 = ctx.io2.into_open_drain_output();
        io2.internal_pull_up(true);
        let mut io4 = ctx.io4.into_open_drain_output();
        io4.internal_pull_up(true);

        assert!(io2.set_high().is_ok());
        assert!(io4.set_high().is_ok());
        ctx.delay.delay_millis(1);

        assert_eq!(io2.is_high(), Ok(true));
        assert_eq!(io4.is_high(), Ok(true));

        assert!(io2.set_low().is_ok());
        assert!(io4.set_high().is_ok());
        ctx.delay.delay_millis(1);

        assert_eq!(io2.is_low(), Ok(true));
        assert_eq!(io4.is_low(), Ok(true));

        assert!(io2.set_high().is_ok());
        assert!(io4.set_high().is_ok());
        ctx.delay.delay_millis(1);

        assert_eq!(io2.is_high(), Ok(true));
        assert_eq!(io4.is_high(), Ok(true));

        assert!(io2.set_high().is_ok());
        assert!(io4.set_low().is_ok());
        ctx.delay.delay_millis(1);

        assert_eq!(io2.is_low(), Ok(true));
        assert_eq!(io4.is_low(), Ok(true));
    }
}
