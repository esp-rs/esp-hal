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
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    delay::Delay,
    embassy,
    gpio::{GpioPin, Input, Output, OutputPin, PullDown, PushPull, Unknown, IO, NUM_PINS},
    macros::handler,
    peripherals::Peripherals,
    system::SystemExt,
    timer::TimerGroup,
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

        let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks);
        embassy::init(&clocks, timg0);

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
            .as_mut() // we can't unwrap as the handler may get called for async operations
            .map(|pin| pin.clear_interrupt());
    });
}

#[cfg(test)]
#[embedded_test::tests]
mod tests {
    use defmt::assert_eq;
    use embassy_time::{Duration, Timer};
    use embedded_hal_async::digital::Wait;
    use esp_hal::gpio::{Event, Pin};
    use portable_atomic::{AtomicUsize, Ordering};

    use super::*;

    #[init]
    fn init() -> Context {
        let mut ctx = Context::init();
        // make sure tests don't interfere with each other
        ctx.io4.set_low().ok();
        ctx
    }

    #[test]
    async fn test_async_edge(ctx: Context) {
        let counter = AtomicUsize::new(0);
        let Context {
            mut io2, mut io4, ..
        } = ctx;
        embassy_futures::select::select(
            async {
                loop {
                    io2.wait_for_rising_edge().await.unwrap();
                    counter.fetch_add(1, Ordering::SeqCst);
                }
            },
            async {
                for _ in 0..5 {
                    io4.set_high().unwrap();
                    Timer::after(Duration::from_millis(25)).await;
                    io4.set_low().unwrap();
                    Timer::after(Duration::from_millis(25)).await;
                }
            },
        )
        .await;
        assert_eq!(counter.load(Ordering::SeqCst), 5);
    }

    #[test]
    async fn test_pins_can_wait(_ctx: Context) {
        let mut first = unsafe { GpioPin::<Unknown, 0>::steal() }.into_pull_down_input();
        // convenient way to get the last pin without cfgs for all chips
        let mut last =
            unsafe { GpioPin::<Unknown, { (NUM_PINS - 1) as u8 }>::steal() }.into_pull_down_input();

        embassy_futures::select::select3(
            first.wait_for_rising_edge(),
            last.wait_for_rising_edge(),
            // Other futures won't return, this one will, make sure its last so all other futures
            // are polled first
            embassy_futures::yield_now(),
        )
        .await;
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
        critical_section::with(|cs| {
            *COUNTER.borrow_ref_mut(cs) = 0;
            ctx.io2.listen(Event::AnyEdge);
            INPUT_PIN.borrow_ref_mut(cs).replace(ctx.io2);
        });
        assert!(ctx.io4.set_high().is_ok());
        ctx.delay.delay_millis(1);
        assert!(ctx.io4.set_low().is_ok());
        ctx.delay.delay_millis(1);
        assert!(ctx.io4.set_high().is_ok());
        ctx.delay.delay_millis(1);
        assert!(ctx.io4.set_low().is_ok());
        ctx.delay.delay_millis(1);
        assert!(ctx.io4.set_high().is_ok());
        ctx.delay.delay_millis(1);
        assert!(ctx.io4.set_low().is_ok());
        ctx.delay.delay_millis(1);
        assert!(ctx.io4.set_high().is_ok());
        ctx.delay.delay_millis(1);
        assert!(ctx.io4.set_low().is_ok());
        ctx.delay.delay_millis(1);
        assert!(ctx.io4.set_high().is_ok());
        ctx.delay.delay_millis(1);

        let count = critical_section::with(|cs| *COUNTER.borrow_ref(cs));
        assert_eq!(count, 9);

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
