//! GPIO Test
//!
//! Folowing pins are used:
//! GPIO2
//! GPIO4

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use core::cell::RefCell;

use critical_section::Mutex;
use defmt_rtt as _;
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    delay::Delay,
    gpio::{Gpio2, Gpio4, GpioPin, Input, Io, Level, Output, Pull},
    macros::handler,
    peripherals::Peripherals,
    system::SystemControl,
    timer::timg::TimerGroup,
};

static COUNTER: Mutex<RefCell<u32>> = Mutex::new(RefCell::new(0));
static INPUT_PIN: Mutex<RefCell<Option<Input<'static, Gpio2>>>> = Mutex::new(RefCell::new(None));

struct Context<'d> {
    io2: Input<'d, Gpio2>,
    io4: Output<'d, Gpio4>,
    delay: Delay,
}

impl<'d> Context<'d> {
    pub fn init() -> Self {
        let peripherals = Peripherals::take();
        let system = SystemControl::new(peripherals.SYSTEM);
        let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

        let mut io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
        io.set_interrupt_handler(interrupt_handler);

        let delay = Delay::new(&clocks);

        let timg0 = TimerGroup::new_async(peripherals.TIMG0, &clocks);
        esp_hal_embassy::init(&clocks, timg0);

        Context {
            io2: Input::new(io.pins.gpio2, Pull::Down),
            io4: Output::new(io.pins.gpio4, Level::Low),
            delay,
        }
    }
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
#[embedded_test::tests(executor = esp_hal_embassy::executor::Executor::new())]
mod tests {
    use defmt::assert_eq;
    use embassy_time::{Duration, Timer};
    use esp_hal::gpio::{Event, OutputOpenDrain};
    use portable_atomic::{AtomicUsize, Ordering};

    use super::*;

    #[init]
    fn init() -> Context<'static> {
        let mut ctx = Context::init();
        // make sure tests don't interfere with each other
        ctx.io4.set_low();
        ctx
    }

    #[test]
    async fn test_async_edge(ctx: Context<'static>) {
        let counter = AtomicUsize::new(0);
        let Context {
            mut io2, mut io4, ..
        } = ctx;
        embassy_futures::select::select(
            async {
                loop {
                    io2.wait_for_rising_edge().await;
                    counter.fetch_add(1, Ordering::SeqCst);
                }
            },
            async {
                for _ in 0..5 {
                    io4.set_high();
                    Timer::after(Duration::from_millis(25)).await;
                    io4.set_low();
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
        assert_eq!(ctx.io2.is_low(), true);
        assert_eq!(ctx.io2.is_high(), false);
    }

    #[test]
    fn test_gpio_output(mut ctx: Context<'static>) {
        // `StatefulOutputPin`:
        assert_eq!(ctx.io4.is_set_low(), true);
        assert_eq!(ctx.io4.is_set_high(), false);
        ctx.io4.set_high();
        assert_eq!(ctx.io4.is_set_low(), false);
        assert_eq!(ctx.io4.is_set_high(), true);

        // `ToggleableOutputPin`:
        ctx.io4.toggle();
        assert_eq!(ctx.io4.is_set_low(), true);
        assert_eq!(ctx.io4.is_set_high(), false);
        ctx.io4.toggle();
        assert_eq!(ctx.io4.is_set_low(), false);
        assert_eq!(ctx.io4.is_set_high(), true);
    }

    #[test]
    fn test_gpio_interrupt(mut ctx: Context<'static>) {
        critical_section::with(|cs| {
            *COUNTER.borrow_ref_mut(cs) = 0;
            ctx.io2.listen(Event::AnyEdge);
            INPUT_PIN.borrow_ref_mut(cs).replace(ctx.io2);
        });
        ctx.io4.set_high();
        ctx.delay.delay_millis(1);
        ctx.io4.set_low();
        ctx.delay.delay_millis(1);
        ctx.io4.set_high();
        ctx.delay.delay_millis(1);
        ctx.io4.set_low();
        ctx.delay.delay_millis(1);
        ctx.io4.set_high();
        ctx.delay.delay_millis(1);
        ctx.io4.set_low();
        ctx.delay.delay_millis(1);
        ctx.io4.set_high();
        ctx.delay.delay_millis(1);
        ctx.io4.set_low();
        ctx.delay.delay_millis(1);
        ctx.io4.set_high();
        ctx.delay.delay_millis(1);

        let count = critical_section::with(|cs| *COUNTER.borrow_ref(cs));
        assert_eq!(count, 9);

        ctx.io2 = critical_section::with(|cs| INPUT_PIN.borrow_ref_mut(cs).take().unwrap());
        ctx.io2.unlisten();
    }

    #[test]
    fn test_gpio_od(ctx: Context<'static>) {
        let mut io2 = OutputOpenDrain::new(unsafe { GpioPin::<2>::steal() }, Level::High, Pull::Up);
        let mut io4 = OutputOpenDrain::new(unsafe { GpioPin::<4>::steal() }, Level::High, Pull::Up);

        ctx.delay.delay_millis(1);

        assert_eq!(io2.is_high(), true);
        assert_eq!(io4.is_high(), true);

        io2.set_low();
        io4.set_high();
        ctx.delay.delay_millis(1);

        assert_eq!(io2.is_low(), true);
        assert_eq!(io4.is_low(), true);

        io2.set_high();
        io4.set_high();
        ctx.delay.delay_millis(1);

        assert_eq!(io2.is_high(), true);
        assert_eq!(io4.is_high(), true);

        io2.set_high();
        io4.set_low();
        ctx.delay.delay_millis(1);

        assert_eq!(io2.is_low(), true);
        assert_eq!(io4.is_low(), true);

        io2.set_high();
        io4.set_high();
        ctx.delay.delay_millis(1);

        assert_eq!(io2.is_high(), true);
        assert_eq!(io4.is_high(), true);

        io2.set_low();
        io4.set_low();
        ctx.delay.delay_millis(1);

        assert_eq!(io2.is_low(), true);
        assert_eq!(io4.is_low(), true);
    }
}
