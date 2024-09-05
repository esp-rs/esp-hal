//! GPIO Test
//!
//! Folowing pins are used:
//! GPIO2 / GPIO9 (esp32s2 and esp32s3)
//! GPIO3 / GPIO10 (esp32s2 and esp32s3)

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: generic-queue

#![no_std]
#![no_main]

use core::cell::RefCell;

use critical_section::Mutex;
use esp_hal::{
    delay::Delay,
    gpio::{AnyPin, GpioPin, Input, Io, Level, Output, Pull},
    macros::handler,
    timer::timg::TimerGroup,
    InterruptConfigurable,
};
use hil_test as _;

static COUNTER: Mutex<RefCell<u32>> = Mutex::new(RefCell::new(0));
static INPUT_PIN: Mutex<RefCell<Option<Input<'static, TestGpio1>>>> =
    Mutex::new(RefCell::new(None));

cfg_if::cfg_if! {
    if #[cfg(not(any(esp32s2, esp32s3)))] {
        pub type TestGpio1 = GpioPin<2>;
        pub type TestGpio2 = GpioPin<3>;
    } else if #[cfg(any(esp32s2, esp32s3))] {
        pub type TestGpio1 = GpioPin<9>;
        pub type TestGpio2 = GpioPin<10>;
    }
}

struct Context<'d> {
    test_gpio1: Input<'d, TestGpio1>,
    test_gpio2: Output<'d, TestGpio2>,
    delay: Delay,
}

#[handler]
pub fn interrupt_handler() {
    critical_section::with(|cs| {
        *COUNTER.borrow_ref_mut(cs) += 1;
        INPUT_PIN
            .borrow_ref_mut(cs)
            .as_mut() // we can't unwrap as the handler may get called for async operations
            .map(|pin| pin.clear_interrupt());
    });
}

#[cfg(test)]
#[embedded_test::tests(executor = esp_hal_embassy::Executor::new())]
mod tests {
    use defmt::assert_eq;
    use embassy_time::{Duration, Timer};
    use esp_hal::gpio::{Event, Flex, OutputOpenDrain};
    use portable_atomic::{AtomicUsize, Ordering};

    use super::*;

    #[init]
    fn init() -> Context<'static> {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let mut io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
        io.set_interrupt_handler(interrupt_handler);

        let delay = Delay::new();

        let (gpio1, gpio2) = hil_test::common_test_pins!(io);

        let timg0 = TimerGroup::new(peripherals.TIMG0);
        esp_hal_embassy::init(timg0.timer0);

        Context {
            test_gpio1: Input::new(gpio1, Pull::Down),
            test_gpio2: Output::new(gpio2, Level::Low),
            delay,
        }
    }

    #[test]
    async fn test_async_edge(ctx: Context<'static>) {
        let counter = AtomicUsize::new(0);
        let Context {
            mut test_gpio1,
            mut test_gpio2,
            ..
        } = ctx;
        embassy_futures::select::select(
            async {
                loop {
                    test_gpio1.wait_for_rising_edge().await;
                    counter.fetch_add(1, Ordering::SeqCst);
                }
            },
            async {
                for _ in 0..5 {
                    test_gpio2.set_high();
                    Timer::after(Duration::from_millis(25)).await;
                    test_gpio2.set_low();
                    Timer::after(Duration::from_millis(25)).await;
                }
            },
        )
        .await;
        assert_eq!(counter.load(Ordering::SeqCst), 5);
    }

    #[test]
    async fn test_a_pin_can_wait(_ctx: Context<'static>) {
        let mut first = Input::new(unsafe { GpioPin::<0>::steal() }, Pull::Down);

        embassy_futures::select::select(
            first.wait_for_rising_edge(),
            // Other futures won't return, this one will, make sure its last so all other futures
            // are polled first
            embassy_futures::yield_now(),
        )
        .await;
    }

    #[test]
    fn test_gpio_input(ctx: Context<'static>) {
        // `InputPin`:
        assert_eq!(ctx.test_gpio1.is_low(), true);
        assert_eq!(ctx.test_gpio1.is_high(), false);
    }

    #[test]
    fn test_gpio_output(mut ctx: Context<'static>) {
        // `StatefulOutputPin`:
        assert_eq!(ctx.test_gpio2.is_set_low(), true);
        assert_eq!(ctx.test_gpio2.is_set_high(), false);
        ctx.test_gpio2.set_high();
        assert_eq!(ctx.test_gpio2.is_set_low(), false);
        assert_eq!(ctx.test_gpio2.is_set_high(), true);

        // `ToggleableOutputPin`:
        ctx.test_gpio2.toggle();
        assert_eq!(ctx.test_gpio2.is_set_low(), true);
        assert_eq!(ctx.test_gpio2.is_set_high(), false);
        ctx.test_gpio2.toggle();
        assert_eq!(ctx.test_gpio2.is_set_low(), false);
        assert_eq!(ctx.test_gpio2.is_set_high(), true);
    }

    #[test]
    fn test_gpio_interrupt(mut ctx: Context<'static>) {
        critical_section::with(|cs| {
            *COUNTER.borrow_ref_mut(cs) = 0;
            ctx.test_gpio1.listen(Event::AnyEdge);
            INPUT_PIN.borrow_ref_mut(cs).replace(ctx.test_gpio1);
        });
        ctx.test_gpio2.set_high();
        ctx.delay.delay_millis(1);
        ctx.test_gpio2.set_low();
        ctx.delay.delay_millis(1);
        ctx.test_gpio2.set_high();
        ctx.delay.delay_millis(1);
        ctx.test_gpio2.set_low();
        ctx.delay.delay_millis(1);
        ctx.test_gpio2.set_high();
        ctx.delay.delay_millis(1);
        ctx.test_gpio2.set_low();
        ctx.delay.delay_millis(1);
        ctx.test_gpio2.set_high();
        ctx.delay.delay_millis(1);
        ctx.test_gpio2.set_low();
        ctx.delay.delay_millis(1);
        ctx.test_gpio2.set_high();
        ctx.delay.delay_millis(1);

        let count = critical_section::with(|cs| *COUNTER.borrow_ref(cs));
        assert_eq!(count, 9);

        ctx.test_gpio1 = critical_section::with(|cs| INPUT_PIN.borrow_ref_mut(cs).take().unwrap());
        ctx.test_gpio1.unlisten();
    }

    #[test]
    fn test_gpio_od(ctx: Context<'static>) {
        let mut test_gpio1 =
            OutputOpenDrain::new(unsafe { TestGpio1::steal() }, Level::High, Pull::Up);
        let mut test_gpio2 =
            OutputOpenDrain::new(unsafe { TestGpio2::steal() }, Level::High, Pull::Up);

        ctx.delay.delay_millis(1);

        assert_eq!(test_gpio1.is_high(), true);
        assert_eq!(test_gpio2.is_high(), true);

        test_gpio1.set_low();
        test_gpio2.set_high();
        ctx.delay.delay_millis(1);

        assert_eq!(test_gpio1.is_low(), true);
        assert_eq!(test_gpio2.is_low(), true);

        test_gpio1.set_high();
        test_gpio2.set_high();
        ctx.delay.delay_millis(1);

        assert_eq!(test_gpio1.is_high(), true);
        assert_eq!(test_gpio2.is_high(), true);

        test_gpio1.set_high();
        test_gpio2.set_low();
        ctx.delay.delay_millis(1);

        assert_eq!(test_gpio1.is_low(), true);
        assert_eq!(test_gpio2.is_low(), true);

        test_gpio1.set_high();
        test_gpio2.set_high();
        ctx.delay.delay_millis(1);

        assert_eq!(test_gpio1.is_high(), true);
        assert_eq!(test_gpio2.is_high(), true);

        test_gpio1.set_low();
        test_gpio2.set_low();
        ctx.delay.delay_millis(1);

        assert_eq!(test_gpio1.is_low(), true);
        assert_eq!(test_gpio2.is_low(), true);
    }

    #[test]
    fn test_gpio_flex(ctx: Context<'static>) {
        let mut test_gpio1 = Flex::new(unsafe { TestGpio1::steal() });
        let mut test_gpio2 = Flex::new(unsafe { TestGpio2::steal() });

        test_gpio1.set_high();
        test_gpio1.set_as_output();
        test_gpio2.set_as_input(Pull::None);

        ctx.delay.delay_millis(1);

        assert_eq!(test_gpio1.is_set_high(), true);
        assert_eq!(test_gpio2.is_high(), true);

        test_gpio1.set_low();
        ctx.delay.delay_millis(1);

        assert_eq!(test_gpio1.is_set_high(), false);
        assert_eq!(test_gpio2.is_high(), false);

        test_gpio1.set_as_input(Pull::None);
        test_gpio2.set_as_output();
        ctx.delay.delay_millis(1);

        assert_eq!(test_gpio1.is_high(), false);
        assert_eq!(test_gpio2.is_set_high(), false);

        test_gpio2.set_high();
        ctx.delay.delay_millis(1);

        assert_eq!(test_gpio1.is_high(), true);
        assert_eq!(test_gpio2.is_set_high(), true);

        test_gpio2.set_low();
        ctx.delay.delay_millis(1);

        assert_eq!(test_gpio1.is_low(), true);
        assert_eq!(test_gpio2.is_set_low(), true);
    }

    // Tests touch pin (GPIO2) as AnyPin and Output
    // https://github.com/esp-rs/esp-hal/issues/1943
    #[test]
    fn test_gpio_touch_anypin_output() {
        let any_pin2 = AnyPin::new(unsafe { TestGpio1::steal() });
        let any_pin3 = AnyPin::new(unsafe { TestGpio2::steal() });

        let out_pin = Output::new(any_pin2, Level::High);
        let in_pin = Input::new(any_pin3, Pull::Down);

        assert_eq!(out_pin.is_set_high(), true);
        assert_eq!(in_pin.is_high(), true);
    }

    // Tests touch pin (GPIO2) as AnyPin and Input
    // https://github.com/esp-rs/esp-hal/issues/1943
    #[test]
    fn test_gpio_touch_anypin_input() {
        let any_pin2 = AnyPin::new(unsafe { TestGpio1::steal() });
        let any_pin3 = AnyPin::new(unsafe { TestGpio2::steal() });

        let out_pin = Output::new(any_pin3, Level::Low);
        let in_pin = Input::new(any_pin2, Pull::Down);

        assert_eq!(out_pin.is_set_high(), false);
        assert_eq!(in_pin.is_high(), false);
    }
}
