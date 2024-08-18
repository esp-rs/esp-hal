//! GPIO Test
//!
//! Folowing pins are used:
//! GPIO2
//! GPIO3

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
    gpio::{Gpio2, Gpio3, GpioPin, Input, Io, Level, Output, Pull},
    macros::handler,
    peripherals::Peripherals,
    system::SystemControl,
    timer::{timg::TimerGroup, ErasedTimer, OneShotTimer},
    InterruptConfigurable,
};

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

static COUNTER: Mutex<RefCell<u32>> = Mutex::new(RefCell::new(0));
static INPUT_PIN: Mutex<RefCell<Option<Input<'static, Gpio2>>>> = Mutex::new(RefCell::new(None));

struct Context<'d> {
    io2: Input<'d, Gpio2>,
    io3: Output<'d, Gpio3>,
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

        let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks);
        esp_hal_embassy::init(&clocks, timg0.timer0);

        Context {
            io2: Input::new(io.pins.gpio2, Pull::Down),
            io3: Output::new(io.pins.gpio3, Level::Low),
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
#[embedded_test::tests(executor = esp_hal_embassy::Executor::new())]
mod tests {
    use defmt::assert_eq;
    use embassy_time::{Duration, Timer};
    use esp_hal::gpio::{Event, Flex, OutputOpenDrain};
    use portable_atomic::{AtomicUsize, Ordering};

    use super::*;

    #[init]
    fn init() -> Context<'static> {
        let mut ctx = Context::init();
        // make sure tests don't interfere with each other
        ctx.io3.set_low();
        ctx
    }

    #[test]
    async fn test_async_edge(ctx: Context<'static>) {
        let counter = AtomicUsize::new(0);
        let Context {
            mut io2, mut io3, ..
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
                    io3.set_high();
                    Timer::after(Duration::from_millis(25)).await;
                    io3.set_low();
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
        assert_eq!(ctx.io3.is_set_low(), true);
        assert_eq!(ctx.io3.is_set_high(), false);
        ctx.io3.set_high();
        assert_eq!(ctx.io3.is_set_low(), false);
        assert_eq!(ctx.io3.is_set_high(), true);

        // `ToggleableOutputPin`:
        ctx.io3.toggle();
        assert_eq!(ctx.io3.is_set_low(), true);
        assert_eq!(ctx.io3.is_set_high(), false);
        ctx.io3.toggle();
        assert_eq!(ctx.io3.is_set_low(), false);
        assert_eq!(ctx.io3.is_set_high(), true);
    }

    #[test]
    fn test_gpio_interrupt(mut ctx: Context<'static>) {
        critical_section::with(|cs| {
            *COUNTER.borrow_ref_mut(cs) = 0;
            ctx.io2.listen(Event::AnyEdge);
            INPUT_PIN.borrow_ref_mut(cs).replace(ctx.io2);
        });
        ctx.io3.set_high();
        ctx.delay.delay_millis(1);
        ctx.io3.set_low();
        ctx.delay.delay_millis(1);
        ctx.io3.set_high();
        ctx.delay.delay_millis(1);
        ctx.io3.set_low();
        ctx.delay.delay_millis(1);
        ctx.io3.set_high();
        ctx.delay.delay_millis(1);
        ctx.io3.set_low();
        ctx.delay.delay_millis(1);
        ctx.io3.set_high();
        ctx.delay.delay_millis(1);
        ctx.io3.set_low();
        ctx.delay.delay_millis(1);
        ctx.io3.set_high();
        ctx.delay.delay_millis(1);

        let count = critical_section::with(|cs| *COUNTER.borrow_ref(cs));
        assert_eq!(count, 9);

        ctx.io2 = critical_section::with(|cs| INPUT_PIN.borrow_ref_mut(cs).take().unwrap());
        ctx.io2.unlisten();
    }

    #[test]
    fn test_gpio_od(ctx: Context<'static>) {
        let mut io2 = OutputOpenDrain::new(unsafe { GpioPin::<2>::steal() }, Level::High, Pull::Up);
        let mut io3 = OutputOpenDrain::new(unsafe { GpioPin::<3>::steal() }, Level::High, Pull::Up);

        ctx.delay.delay_millis(1);

        assert_eq!(io2.is_high(), true);
        assert_eq!(io3.is_high(), true);

        io2.set_low();
        io3.set_high();
        ctx.delay.delay_millis(1);

        assert_eq!(io2.is_low(), true);
        assert_eq!(io3.is_low(), true);

        io2.set_high();
        io3.set_high();
        ctx.delay.delay_millis(1);

        assert_eq!(io2.is_high(), true);
        assert_eq!(io3.is_high(), true);

        io2.set_high();
        io3.set_low();
        ctx.delay.delay_millis(1);

        assert_eq!(io2.is_low(), true);
        assert_eq!(io3.is_low(), true);

        io2.set_high();
        io3.set_high();
        ctx.delay.delay_millis(1);

        assert_eq!(io2.is_high(), true);
        assert_eq!(io3.is_high(), true);

        io2.set_low();
        io3.set_low();
        ctx.delay.delay_millis(1);

        assert_eq!(io2.is_low(), true);
        assert_eq!(io3.is_low(), true);
    }

    #[test]
    fn test_gpio_flex(ctx: Context<'static>) {
        let mut io2 = Flex::new(unsafe { GpioPin::<2>::steal() });
        let mut io3 = Flex::new(unsafe { GpioPin::<3>::steal() });

        io2.set_high();
        io2.set_as_output();
        io3.set_as_input(Pull::None);

        ctx.delay.delay_millis(1);

        assert_eq!(io2.is_set_high(), true);
        assert_eq!(io3.is_high(), true);

        io2.set_low();
        ctx.delay.delay_millis(1);

        assert_eq!(io2.is_set_high(), false);
        assert_eq!(io3.is_high(), false);

        io2.set_as_input(Pull::None);
        io3.set_as_output();
        ctx.delay.delay_millis(1);

        assert_eq!(io2.is_high(), false);
        assert_eq!(io3.is_set_high(), false);

        io3.set_high();
        ctx.delay.delay_millis(1);

        assert_eq!(io2.is_high(), true);
        assert_eq!(io3.is_set_high(), true);

        io3.set_low();
        ctx.delay.delay_millis(1);

        assert_eq!(io2.is_low(), true);
        assert_eq!(io3.is_set_low(), true);
    }
}
