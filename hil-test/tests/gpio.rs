//! GPIO Test

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES(unstable): unstable embassy
//% FEATURES(stable):

#![no_std]
#![no_main]

use esp_hal::gpio::{AnyPin, Input, InputConfig, Level, Output, OutputConfig, Pin, Pull};
use hil_test as _;

cfg_if::cfg_if! {
    if #[cfg(feature = "unstable")] {
        use core::cell::RefCell;
        use critical_section::Mutex;
        use embassy_time::{Duration, Timer};
        use esp_hal::{
            // OutputOpenDrain is here because will be unused otherwise
            delay::Delay,
            gpio::{DriveMode, Event, Flex, Io},
            handler,
            timer::timg::TimerGroup,
        };
        use portable_atomic::{AtomicUsize, Ordering};

        static COUNTER: Mutex<RefCell<u32>> = Mutex::new(RefCell::new(0));
        static INPUT_PIN: Mutex<RefCell<Option<Input>>> = Mutex::new(RefCell::new(None));
    }
}

cfg_if::cfg_if! {
    if #[cfg(all(multi_core, feature = "unstable"))] {
        use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
    }
}

struct Context {
    test_gpio1: AnyPin<'static>,
    test_gpio2: AnyPin<'static>,
    #[cfg(feature = "unstable")]
    delay: Delay,
    #[cfg(feature = "unstable")]
    io: Io<'static>,
}

#[cfg_attr(feature = "unstable", handler)]
#[cfg(feature = "unstable")]
pub fn interrupt_handler() {
    critical_section::with(|cs| {
        *COUNTER.borrow_ref_mut(cs) += 1;
        INPUT_PIN
            .borrow_ref_mut(cs)
            .as_mut()
            .map(|pin| pin.clear_interrupt());
    });
}

// Compile-time test to check that GPIOs can be passed by reference.
fn _gpios_can_be_reused() {
    let p = esp_hal::init(esp_hal::Config::default());

    let mut gpio1 = p.GPIO1;

    {
        let _driver = Input::new(
            gpio1.reborrow(),
            InputConfig::default().with_pull(Pull::Down),
        );
    }

    {
        let _driver = esp_hal::spi::master::Spi::new(p.SPI2, Default::default())
            .unwrap()
            .with_mosi(gpio1.reborrow());
    }

    {
        let _driver = Input::new(
            gpio1.reborrow(),
            InputConfig::default().with_pull(Pull::Down),
        );
    }
}

#[cfg(all(multi_core, feature = "unstable"))]
#[embassy_executor::task]
async fn edge_counter_task(
    mut in_pin: Input<'static>,
    signal: &'static Signal<CriticalSectionRawMutex, u32>,
) {
    let mut edge_count = 0;
    loop {
        // This join will:
        // - first set up the pin to listen
        // - then poll the pin future once (which will return Pending)
        // - then signal that the pin is listening, which enables the other core to
        //   toggle the matching OutputPin
        // - then will wait for the pin future to resolve.
        embassy_futures::join::join(in_pin.wait_for_any_edge(), async {
            signal.signal(edge_count);
        })
        .await;

        edge_count += 1;
    }
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

        // Interrupts are unstable
        #[cfg(feature = "unstable")]
        let io = Io::new(peripherals.IO_MUX);

        #[cfg(feature = "unstable")]
        {
            // Timers are unstable
            let timg0 = TimerGroup::new(peripherals.TIMG0);
            esp_hal_embassy::init(timg0.timer0);
        }

        Context {
            test_gpio1: gpio1.degrade(),
            test_gpio2: gpio2.degrade(),
            #[cfg(feature = "unstable")]
            delay,
            #[cfg(feature = "unstable")]
            io,
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
    #[cfg(feature = "unstable")] // Interrupts are unstable
    async fn a_pin_can_wait_with_custom_handler(mut ctx: Context) {
        ctx.io.set_interrupt_handler(interrupt_handler);

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
    fn gpio_interrupt(mut ctx: Context) {
        ctx.io.set_interrupt_handler(interrupt_handler);

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
        let input_pull_up = InputConfig::default().with_pull(Pull::Up);
        let input_pull_down = InputConfig::default().with_pull(Pull::Down);
        let input_no_pull = InputConfig::default().with_pull(Pull::None);

        let mut output = Output::new(
            ctx.test_gpio1,
            Level::High,
            OutputConfig::default()
                .with_drive_mode(DriveMode::OpenDrain)
                .with_pull(Pull::None),
        );
        let mut input = Input::new(ctx.test_gpio2, input_pull_up);

        ctx.delay.delay_millis(1);

        // With pull up resistor

        assert_eq!(input.level(), Level::High);
        output.set_low();
        ctx.delay.delay_millis(1);
        assert_eq!(input.level(), Level::Low);
        output.set_high();
        ctx.delay.delay_millis(1);
        assert_eq!(input.level(), Level::High);

        // With pull down resistor
        input.apply_config(&input_pull_down);

        output.set_high();
        ctx.delay.delay_millis(1);
        assert_eq!(input.level(), Level::Low);
        output.set_low();
        ctx.delay.delay_millis(1);
        assert_eq!(input.level(), Level::Low);

        // With pull up on output
        input.apply_config(&input_no_pull);
        output.apply_config(
            &OutputConfig::default()
                .with_drive_mode(DriveMode::OpenDrain)
                .with_pull(Pull::Up),
        );

        ctx.delay.delay_millis(1);
        assert_eq!(input.level(), Level::Low);
        output.set_high();
        ctx.delay.delay_millis(1);
        assert_eq!(input.level(), Level::High);
    }

    #[test]
    #[cfg(feature = "unstable")]
    fn gpio_flex(ctx: Context) {
        let mut test_gpio1 = Flex::new(ctx.test_gpio1);
        let mut test_gpio2 = Flex::new(ctx.test_gpio2);

        test_gpio1.set_high();
        test_gpio1.set_output_enable(true);
        test_gpio2.set_input_enable(true);

        ctx.delay.delay_millis(1);

        assert_eq!(test_gpio1.is_set_high(), true);
        assert_eq!(test_gpio2.is_high(), true);

        test_gpio1.set_low();
        ctx.delay.delay_millis(1);

        assert_eq!(test_gpio1.is_set_high(), false);
        assert_eq!(test_gpio2.is_high(), false);

        test_gpio1.set_input_enable(true);
        test_gpio2.set_output_enable(true);
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
        let pin = unsafe { esp_hal::peripherals::GPIO37::steal() };

        _ = Input::new(pin, InputConfig::default().with_pull(Pull::Down));
    }

    #[test]
    #[cfg(feature = "unstable")]
    fn interrupt_executor_is_not_frozen(ctx: Context) {
        use esp_hal::interrupt::{Priority, software::SoftwareInterrupt};
        use esp_hal_embassy::InterruptExecutor;
        use static_cell::StaticCell;

        static INTERRUPT_EXECUTOR: StaticCell<InterruptExecutor<1>> = StaticCell::new();
        let interrupt_executor = INTERRUPT_EXECUTOR.init(InterruptExecutor::new(unsafe {
            SoftwareInterrupt::<1>::steal()
        }));

        let spawner = interrupt_executor.start(Priority::max());

        spawner.must_spawn(test_task(ctx.test_gpio1.degrade()));

        #[embassy_executor::task]
        async fn test_task(pin: AnyPin<'static>) {
            let mut pin = Input::new(pin, InputConfig::default().with_pull(Pull::Down));

            // This line must return, even if the executor
            // is running at a higher priority than the GPIO handler.
            pin.wait_for_low().await;

            embedded_test::export::check_outcome(());
        }

        loop {}
    }

    #[test]
    #[cfg(feature = "unstable")]
    async fn pending_interrupt_does_not_cause_future_to_resolve_immediately(ctx: Context) {
        use embassy_futures::{
            select::{Either, select},
            yield_now,
        };

        let mut out_pin = Output::new(ctx.test_gpio2, Level::Low, OutputConfig::default());
        let mut in_pin = Input::new(ctx.test_gpio1, InputConfig::default().with_pull(Pull::Down));

        in_pin.listen(Event::RisingEdge);

        out_pin.set_high();

        assert!(in_pin.is_interrupt_set());

        let should_timeout = select(in_pin.wait_for_falling_edge(), async {
            // Give the future a bit of time, don't rely on the first poll resolving
            for _ in 0..5 {
                yield_now().await;
            }
        })
        .await;

        assert!(matches!(should_timeout, Either::Second(_)));
    }

    #[test]
    #[cfg(all(multi_core, feature = "unstable"))]
    async fn pin_waits_on_core_different_from_interrupt_handler(ctx: Context) {
        // This test exercises cross-core pin events. Core 1 will wait for edge events
        // that Core 0 generates. Interrupts are handled on Core 0. A signal is used to
        // throttle the toggling, so that Core 1 will be expected to count the
        // exact number of edge transitions.

        use esp_hal::{
            peripherals::CPU_CTRL,
            system::{CpuControl, Stack},
        };
        use esp_hal_embassy::Executor;
        use hil_test::mk_static;

        let mut out_pin = Output::new(ctx.test_gpio2, Level::Low, OutputConfig::default());
        let in_pin = Input::new(ctx.test_gpio1, InputConfig::default().with_pull(Pull::Down));

        // `edge_counter_task` also returns the edge count as part of this signal
        let input_pin_listening = &*mk_static!(Signal<CriticalSectionRawMutex, u32>, Signal::new());

        // No need to thread this through `Context` for one test case
        let cpu_ctrl = unsafe { CPU_CTRL::steal() };
        const CORE1_STACK_SIZE: usize = 8192;
        let app_core_stack = mk_static!(Stack<CORE1_STACK_SIZE>, Stack::new());
        let _second_core = CpuControl::new(cpu_ctrl)
            .start_app_core(app_core_stack, {
                move || {
                    let executor = mk_static!(Executor, Executor::new());
                    executor.run(|spawner| {
                        spawner.must_spawn(edge_counter_task(in_pin, input_pin_listening));
                    });
                }
            })
            .unwrap();

        // Now drive the OutputPin and assert that the other core saw exactly as many
        // edges as we generated here.
        const EDGE_COUNT: u32 = 10_000;
        for _ in 0..EDGE_COUNT {
            input_pin_listening.wait().await;
            out_pin.toggle();
        }
        // wait for signal
        let edge_counter = input_pin_listening.wait().await;

        assert_eq!(edge_counter, EDGE_COUNT);
    }
}
