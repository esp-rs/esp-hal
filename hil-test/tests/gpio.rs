//! GPIO Test

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: generic-queue

#![no_std]
#![no_main]

use core::cell::RefCell;

use critical_section::Mutex;
use esp_hal::{
    delay::Delay,
    gpio::{AnyPin, Input, Io, Level, Output, Pin, Pull},
    macros::handler,
    timer::timg::TimerGroup,
    InterruptConfigurable,
};
use hil_test as _;

static COUNTER: Mutex<RefCell<u32>> = Mutex::new(RefCell::new(0));
static INPUT_PIN: Mutex<RefCell<Option<Input>>> = Mutex::new(RefCell::new(None));

struct Context {
    test_gpio1: AnyPin,
    test_gpio2: AnyPin,
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
    use embassy_time::{Duration, Timer};
    use esp_hal::gpio::{Event, Flex, OutputOpenDrain};
    use portable_atomic::{AtomicUsize, Ordering};

    use super::*;

    #[init]
    fn init() -> Context {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let mut io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
        io.set_interrupt_handler(interrupt_handler);

        let delay = Delay::new();

        let (gpio1, gpio2) = hil_test::common_test_pins!(io);

        let timg0 = TimerGroup::new(peripherals.TIMG0);
        esp_hal_embassy::init(timg0.timer0);

        Context {
            test_gpio1: gpio1.degrade(),
            test_gpio2: gpio2.degrade(),
            delay,
        }
    }

    #[test]
    async fn async_edge(ctx: Context) {
        let counter = AtomicUsize::new(0);
        let Context {
            test_gpio1,
            test_gpio2,
            ..
        } = ctx;
        let mut test_gpio1 = Input::new(test_gpio1, Pull::Down);
        let mut test_gpio2 = Output::new(test_gpio2, Level::Low);
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
    async fn a_pin_can_wait(ctx: Context) {
        let mut first = Input::new(ctx.test_gpio1, Pull::Down);

        embassy_futures::select::select(
            first.wait_for_rising_edge(),
            // Other futures won't return, this one will, make sure its last so all other futures
            // are polled first
            embassy_futures::yield_now(),
        )
        .await;
    }

    #[test]
    fn gpio_input(ctx: Context) {
        let test_gpio1 = Input::new(ctx.test_gpio1, Pull::Down);
        // `InputPin`:
        assert_eq!(test_gpio1.is_low(), true);
        assert_eq!(test_gpio1.is_high(), false);
    }

    #[test]
    fn gpio_output(ctx: Context) {
        let mut test_gpio2 = Output::new(ctx.test_gpio2, Level::Low);

        // `StatefulOutputPin`:
        assert_eq!(test_gpio2.is_set_low(), true);
        assert_eq!(test_gpio2.is_set_high(), false);
        test_gpio2.set_high();
        assert_eq!(test_gpio2.is_set_low(), false);
        assert_eq!(test_gpio2.is_set_high(), true);

        // `ToggleableOutputPin`:
        test_gpio2.toggle();
        assert_eq!(test_gpio2.is_set_low(), true);
        assert_eq!(test_gpio2.is_set_high(), false);
        test_gpio2.toggle();
        assert_eq!(test_gpio2.is_set_low(), false);
        assert_eq!(test_gpio2.is_set_high(), true);
    }

    #[test]
    fn gpio_output_embedded_hal_0_2(ctx: Context) {
        let test_gpio1 = Input::new(ctx.test_gpio1, Pull::Down);
        let mut test_gpio2 = Output::new(ctx.test_gpio2, Level::Low);

        fn set<T>(pin: &mut T, state: bool)
        where
            T: embedded_hal_02::digital::v2::OutputPin,
        {
            if state {
                pin.set_high().ok();
            } else {
                pin.set_low().ok();
            }
        }

        fn toggle<T>(pin: &mut T)
        where
            T: embedded_hal_02::digital::v2::ToggleableOutputPin,
        {
            pin.toggle().ok();
        }

        // `StatefulOutputPin`:
        assert_eq!(test_gpio2.is_set_low(), true);
        assert_eq!(test_gpio2.is_set_high(), false);
        assert_eq!(test_gpio1.is_low(), true);
        assert_eq!(test_gpio1.is_high(), false);
        set(&mut test_gpio2, true);
        assert_eq!(test_gpio2.is_set_low(), false);
        assert_eq!(test_gpio2.is_set_high(), true);
        assert_eq!(test_gpio1.is_low(), false);
        assert_eq!(test_gpio1.is_high(), true);

        // `ToggleableOutputPin`:
        toggle(&mut test_gpio2);
        assert_eq!(test_gpio2.is_set_low(), true);
        assert_eq!(test_gpio2.is_set_high(), false);
        assert_eq!(test_gpio1.is_low(), true);
        assert_eq!(test_gpio1.is_high(), false);
        toggle(&mut test_gpio2);
        assert_eq!(test_gpio2.is_set_low(), false);
        assert_eq!(test_gpio2.is_set_high(), true);
        assert_eq!(test_gpio1.is_low(), false);
        assert_eq!(test_gpio1.is_high(), true);
    }

    #[test]
    fn gpio_output_embedded_hal_1_0(ctx: Context) {
        let test_gpio1 = Input::new(ctx.test_gpio1, Pull::Down);
        let mut test_gpio2 = Output::new(ctx.test_gpio2, Level::Low);

        fn set<T>(pin: &mut T, state: bool)
        where
            T: embedded_hal::digital::OutputPin,
        {
            if state {
                pin.set_high().ok();
            } else {
                pin.set_low().ok();
            }
        }

        fn toggle<T>(pin: &mut T)
        where
            T: embedded_hal::digital::StatefulOutputPin,
        {
            pin.toggle().ok();
        }

        // `StatefulOutputPin`:
        assert_eq!(test_gpio2.is_set_low(), true);
        assert_eq!(test_gpio2.is_set_high(), false);
        assert_eq!(test_gpio1.is_low(), true);
        assert_eq!(test_gpio1.is_high(), false);
        set(&mut test_gpio2, true);
        assert_eq!(test_gpio2.is_set_low(), false);
        assert_eq!(test_gpio2.is_set_high(), true);
        assert_eq!(test_gpio1.is_low(), false);
        assert_eq!(test_gpio1.is_high(), true);

        // `ToggleableOutputPin`:
        toggle(&mut test_gpio2);
        assert_eq!(test_gpio2.is_set_low(), true);
        assert_eq!(test_gpio2.is_set_high(), false);
        assert_eq!(test_gpio1.is_low(), true);
        assert_eq!(test_gpio1.is_high(), false);
        toggle(&mut test_gpio2);
        assert_eq!(test_gpio2.is_set_low(), false);
        assert_eq!(test_gpio2.is_set_high(), true);
        assert_eq!(test_gpio1.is_low(), false);
        assert_eq!(test_gpio1.is_high(), true);
    }

    #[test]
    fn gpio_interrupt(ctx: Context) {
        let mut test_gpio1 = Input::new(ctx.test_gpio1, Pull::Down);
        let mut test_gpio2 = Output::new(ctx.test_gpio2, Level::Low);

        critical_section::with(|cs| {
            *COUNTER.borrow_ref_mut(cs) = 0;
            test_gpio1.listen(Event::AnyEdge);
            INPUT_PIN.borrow_ref_mut(cs).replace(test_gpio1);
        });
        test_gpio2.set_high();
        ctx.delay.delay_millis(1);
        test_gpio2.set_low();
        ctx.delay.delay_millis(1);
        test_gpio2.set_high();
        ctx.delay.delay_millis(1);
        test_gpio2.set_low();
        ctx.delay.delay_millis(1);
        test_gpio2.set_high();
        ctx.delay.delay_millis(1);
        test_gpio2.set_low();
        ctx.delay.delay_millis(1);
        test_gpio2.set_high();
        ctx.delay.delay_millis(1);
        test_gpio2.set_low();
        ctx.delay.delay_millis(1);
        test_gpio2.set_high();
        ctx.delay.delay_millis(1);

        let count = critical_section::with(|cs| *COUNTER.borrow_ref(cs));
        assert_eq!(count, 9);

        let mut test_gpio1 =
            critical_section::with(|cs| INPUT_PIN.borrow_ref_mut(cs).take().unwrap());
        test_gpio1.unlisten();
    }

    #[test]
    fn gpio_od(ctx: Context) {
        let mut test_gpio1 = OutputOpenDrain::new(ctx.test_gpio1, Level::High, Pull::Up);
        let mut test_gpio2 = OutputOpenDrain::new(ctx.test_gpio2, Level::High, Pull::Up);

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
    fn gpio_flex(ctx: Context) {
        let mut test_gpio1 = Flex::new(ctx.test_gpio1);
        let mut test_gpio2 = Flex::new(ctx.test_gpio2);

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
    fn gpio_touch_anypin_output(ctx: Context) {
        let any_pin2 = ctx.test_gpio1;
        let any_pin3 = ctx.test_gpio2;

        let out_pin = Output::new(any_pin2, Level::High);
        let in_pin = Input::new(any_pin3, Pull::Down);

        assert_eq!(out_pin.is_set_high(), true);
        assert_eq!(in_pin.is_high(), true);
    }

    // Tests touch pin (GPIO2) as AnyPin and Input
    // https://github.com/esp-rs/esp-hal/issues/1943
    #[test]
    fn gpio_touch_anypin_input(ctx: Context) {
        let any_pin2 = ctx.test_gpio1;
        let any_pin3 = ctx.test_gpio2;

        let out_pin = Output::new(any_pin3, Level::Low);
        let in_pin = Input::new(any_pin2, Pull::Down);

        assert_eq!(out_pin.is_set_high(), false);
        assert_eq!(in_pin.is_high(), false);
    }

    #[cfg(esp32)]
    #[test]
    fn can_configure_rtcio_pins_as_input() {
        let pins = unsafe { esp_hal::gpio::Pins::steal() };

        _ = Input::new(pins.gpio37, Pull::Down);
    }
}
