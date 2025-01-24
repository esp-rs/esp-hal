//! GPIO Test

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES(unstable): unstable embassy
//% FEATURES(stable):

#![no_std]
#![no_main]

#[cfg(feature = "unstable")] // unused in stable build
use core::cell::RefCell;

#[cfg(feature = "unstable")] // unused in stable build
use critical_section::Mutex;
#[cfg(feature = "unstable")]
use embassy_time::{Duration, Timer};
use esp_hal::gpio::{AnyPin, Input, InputConfig, Level, Output, OutputConfig, Pin, Pull};
#[cfg(feature = "unstable")]
use esp_hal::{
    // OutputOpenDrain is here because will be unused otherwise
    delay::Delay,
    gpio::{DriveMode, Event, Flex, Io},
    handler,
    timer::timg::TimerGroup,
};
use hil_test as _;
#[cfg(feature = "unstable")]
use portable_atomic::{AtomicUsize, Ordering};

#[cfg(feature = "unstable")] // unused in stable build
static COUNTER: Mutex<RefCell<u32>> = Mutex::new(RefCell::new(0));
#[cfg(feature = "unstable")] // unused in stable build
static INPUT_PIN: Mutex<RefCell<Option<Input>>> = Mutex::new(RefCell::new(None));

struct Context {
    test_gpio1: AnyPin,
    test_gpio2: AnyPin,
    #[cfg(feature = "unstable")]
    delay: Delay,
}

#[cfg_attr(feature = "unstable", handler)]
#[cfg(feature = "unstable")]
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
#[embedded_test::tests(default_timeout = 3, executor = hil_test::Executor::new())]
mod tests {
    use super::*;

    #[init]
    fn init() -> Context {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        #[cfg(feature = "unstable")]
        let delay = Delay::new();

        let (gpio1, gpio2) = hil_test::common_test_pins!(peripherals);

        #[cfg(feature = "unstable")]
        {
            // Interrupts are unstable
            let mut io = Io::new(peripherals.IO_MUX);
            io.set_interrupt_handler(interrupt_handler);

            // Timers are unstable
            let timg0 = TimerGroup::new(peripherals.TIMG0);
            esp_hal_embassy::init(timg0.timer0);
        }

        Context {
            test_gpio1: gpio1.degrade(),
            test_gpio2: gpio2.degrade(),
            #[cfg(feature = "unstable")]
            delay,
        }
    }

    #[test]
    #[cfg(feature = "unstable")] // Timers are unstable
    async fn async_edge(ctx: Context) {
        let counter = AtomicUsize::new(0);
        let Context {
            test_gpio1,
            test_gpio2,
            ..
        } = ctx;
        let mut test_gpio1 = Input::new(test_gpio1, InputConfig::default().with_pull(Pull::Down));
        let mut test_gpio2 = Output::new(test_gpio2, Level::Low, OutputConfig::default());
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
        let mut first = Input::new(ctx.test_gpio1, InputConfig::default().with_pull(Pull::Down));

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
        let test_gpio1 = Input::new(ctx.test_gpio1, InputConfig::default().with_pull(Pull::Down));
        // `InputPin`:
        assert_eq!(test_gpio1.is_low(), true);
        assert_eq!(test_gpio1.is_high(), false);
    }

    #[test]
    async fn waiting_for_level_does_not_hang(ctx: Context) {
        let mut test_gpio1 =
            Input::new(ctx.test_gpio1, InputConfig::default().with_pull(Pull::Down));
        let _test_gpio2 = Output::new(ctx.test_gpio2, Level::High, OutputConfig::default());

        test_gpio1.wait_for_high().await;
    }

    #[test]
    fn gpio_output(ctx: Context) {
        let mut test_gpio2 = Output::new(ctx.test_gpio2, Level::Low, OutputConfig::default());

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
    fn gpio_output_embedded_hal_1_0(ctx: Context) {
        let test_gpio1 = Input::new(ctx.test_gpio1, InputConfig::default().with_pull(Pull::Down));
        let mut test_gpio2 = Output::new(ctx.test_gpio2, Level::Low, OutputConfig::default());

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
    #[cfg(feature = "unstable")] // Interrupts are unstable
    fn gpio_interrupt(ctx: Context) {
        let mut test_gpio1 =
            Input::new(ctx.test_gpio1, InputConfig::default().with_pull(Pull::Down));
        let mut test_gpio2 = Output::new(ctx.test_gpio2, Level::Low, OutputConfig::default());

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
    #[cfg(feature = "unstable")] // delay is unstable
    fn gpio_od(ctx: Context) {
        let config = OutputConfig::default()
            .with_drive_mode(DriveMode::OpenDrain)
            .with_input_enabled(true)
            .with_pull(Pull::Up);
        let mut test_gpio1 = Output::new(ctx.test_gpio1, Level::High, config);
        let mut test_gpio2 = Output::new(ctx.test_gpio2, Level::High, config);

        ctx.delay.delay_millis(1);

        assert_eq!(test_gpio1.level(), Level::High);
        assert_eq!(test_gpio2.level(), Level::High);

        test_gpio1.set_low();
        test_gpio2.set_high();
        ctx.delay.delay_millis(1);

        assert_eq!(test_gpio1.level(), Level::Low);
        assert_eq!(test_gpio2.level(), Level::Low);

        test_gpio1.set_high();
        test_gpio2.set_high();
        ctx.delay.delay_millis(1);

        assert_eq!(test_gpio1.level(), Level::High);
        assert_eq!(test_gpio2.level(), Level::High);

        test_gpio1.set_high();
        test_gpio2.set_low();
        ctx.delay.delay_millis(1);

        assert_eq!(test_gpio1.level(), Level::Low);
        assert_eq!(test_gpio2.level(), Level::Low);

        test_gpio1.set_high();
        test_gpio2.set_high();
        ctx.delay.delay_millis(1);

        assert_eq!(test_gpio1.level(), Level::High);
        assert_eq!(test_gpio2.level(), Level::High);

        test_gpio1.set_low();
        test_gpio2.set_low();
        ctx.delay.delay_millis(1);

        assert_eq!(test_gpio1.level(), Level::Low);
        assert_eq!(test_gpio2.level(), Level::Low);
    }

    #[test]
    #[cfg(feature = "unstable")]
    fn gpio_flex(ctx: Context) {
        let mut test_gpio1 = Flex::new(ctx.test_gpio1);
        let mut test_gpio2 = Flex::new(ctx.test_gpio2);

        test_gpio1.set_high();
        test_gpio1.set_as_output();
        test_gpio2.enable_input(true);

        ctx.delay.delay_millis(1);

        assert_eq!(test_gpio1.is_set_high(), true);
        assert_eq!(test_gpio2.is_high(), true);

        test_gpio1.set_low();
        ctx.delay.delay_millis(1);

        assert_eq!(test_gpio1.is_set_high(), false);
        assert_eq!(test_gpio2.is_high(), false);

        test_gpio1.enable_input(true);
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

        let out_pin = Output::new(any_pin2, Level::High, OutputConfig::default());
        let in_pin = Input::new(any_pin3, InputConfig::default().with_pull(Pull::Down));

        assert_eq!(out_pin.is_set_high(), true);
        assert_eq!(in_pin.is_high(), true);
    }

    // Tests touch pin (GPIO2) as AnyPin and Input
    // https://github.com/esp-rs/esp-hal/issues/1943
    #[test]
    fn gpio_touch_anypin_input(ctx: Context) {
        let any_pin2 = ctx.test_gpio1;
        let any_pin3 = ctx.test_gpio2;

        let out_pin = Output::new(any_pin3, Level::Low, OutputConfig::default());
        let in_pin = Input::new(any_pin2, InputConfig::default().with_pull(Pull::Down));

        assert_eq!(out_pin.is_set_high(), false);
        assert_eq!(in_pin.is_high(), false);
    }

    #[cfg(esp32)]
    #[test]
    fn can_configure_rtcio_pins_as_input() {
        let pin = unsafe { esp_hal::gpio::GpioPin::<37>::steal() };

        _ = Input::new(pin, InputConfig::default().with_pull(Pull::Down));
    }

    #[test]
    #[cfg(feature = "unstable")]
    fn interrupt_executor_is_not_frozen(ctx: Context) {
        use esp_hal::interrupt::{software::SoftwareInterrupt, Priority};
        use esp_hal_embassy::InterruptExecutor;
        use static_cell::StaticCell;

        static INTERRUPT_EXECUTOR: StaticCell<InterruptExecutor<1>> = StaticCell::new();
        let interrupt_executor = INTERRUPT_EXECUTOR.init(InterruptExecutor::new(unsafe {
            SoftwareInterrupt::<1>::steal()
        }));

        let spawner = interrupt_executor.start(Priority::max());

        spawner.must_spawn(test_task(ctx.test_gpio1.degrade()));

        #[embassy_executor::task]
        async fn test_task(pin: AnyPin) {
            let mut pin = Input::new(pin, InputConfig::default().with_pull(Pull::Down));

            // This line must return, even if the executor
            // is running at a higher priority than the GPIO handler.
            pin.wait_for_low().await;

            embedded_test::export::check_outcome(());
        }

        loop {}
    }
}
