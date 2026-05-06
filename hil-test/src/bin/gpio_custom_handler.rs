//! GPIO interrupt handler tests
//!
//! This test checks that during HAL initialization we do not overwrite custom
//! GPIO interrupt handlers. We also check that binding a custom interrupt
//! handler explicitly overwrites the handler set by the user, as well as the
//! async API works for user handlers automatically.
//!
//! It also covers the `handle_gpio_interrupt`/`wake_pin` escape hatches that
//! let user-defined raw `GPIO()` handlers re-enable the async API
//! (regression coverage for esp-rs/esp-hal#2659).

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: unstable embassy

#![no_std]
#![no_main]

use embassy_executor::task;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Duration, Timer};
use esp_hal::{
    gpio::{
        AnyPin,
        Flex,
        Input,
        InputConfig,
        InputPin,
        Io,
        Level,
        Output,
        OutputConfig,
        OutputPin,
        Pin,
        Pull,
    },
    handler,
    interrupt::{InterruptHandler, Priority, bind_handler, software::SoftwareInterruptControl},
    peripherals::Interrupt,
    timer::timg::TimerGroup,
};
use esp_rtos::embassy::InterruptExecutor;
use hil_test::mk_static;
use portable_atomic::{AtomicU8, AtomicUsize, Ordering};

// Selects what the user-defined `GPIO()` handler does. Set by each test
// before triggering any pin events, read inside the ISR.
#[repr(u8)]
#[derive(Clone, Copy)]
enum HandlerMode {
    // Original behavior: clear the pin's listen state by hand. Used by the
    // negative test that documents the pre-fix bug.
    UnlistenOnly    = 0,
    // Call `handle_gpio_interrupt()` to drive the whole async pass.
    HandleAll       = 1,
    // Call `wake_pin(SENSE_PIN_NR)` for the pin recorded by the test.
    WakeSpecificPin = 2,
}

static HANDLER_MODE: AtomicU8 = AtomicU8::new(HandlerMode::UnlistenOnly as u8);
static SENSE_PIN_NR: AtomicU8 = AtomicU8::new(u8::MAX);
static HANDLER_INVOCATIONS: AtomicUsize = AtomicUsize::new(0);

extern "C" fn gpio_isr() {
    HANDLER_INVOCATIONS.fetch_add(1, Ordering::Relaxed);

    match HANDLER_MODE.load(Ordering::Relaxed) {
        x if x == HandlerMode::HandleAll as u8 => {
            unsafe { esp_hal::gpio::handle_gpio_interrupt() };
        }
        x if x == HandlerMode::WakeSpecificPin as u8 => {
            let pin = SENSE_PIN_NR.load(Ordering::Relaxed);
            if pin != u8::MAX {
                unsafe { esp_hal::gpio::wake_pin(pin) };
            }
        }
        _ => {
            // UnlistenOnly: prevents binding the default handler, but we need
            // to clear the GPIO interrupts by hand and we deliberately do
            // *not* wake any async future.
            let peripherals = unsafe { esp_hal::peripherals::Peripherals::steal() };
            let (gpio1, _) = hil_test::common_test_pins!(peripherals);
            let mut gpio1 = Flex::new(gpio1);
            gpio1.unlisten();
        }
    }
}

#[unsafe(no_mangle)]
unsafe extern "C" fn GPIO() {
    gpio_isr();
}

#[handler]
pub fn interrupt_handler() {
    // Do nothing
}

async fn drive_pins(gpio1: impl InputPin, gpio2: impl OutputPin) -> usize {
    let counter = AtomicUsize::new(0);
    let mut test_gpio1 = Input::new(gpio1, InputConfig::default().with_pull(Pull::Down));
    let mut test_gpio2 = Output::new(gpio2, Level::Low, OutputConfig::default());
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

    counter.load(Ordering::SeqCst)
}

#[task]
async fn drive_pin(gpio: AnyPin<'static>) {
    let mut test_gpio = Output::new(gpio, Level::Low, OutputConfig::default());
    for _ in 0..5 {
        test_gpio.set_high();
        Timer::after(Duration::from_millis(25)).await;
        test_gpio.set_low();
        Timer::after(Duration::from_millis(25)).await;
    }
}

#[task]
async fn sense_pin(gpio: AnyPin<'static>, done: &'static Signal<CriticalSectionRawMutex, ()>) {
    let mut test_gpio = Input::new(gpio, InputConfig::default().with_pull(Pull::Down));
    test_gpio.wait_for_rising_edge().await;
    test_gpio.wait_for_rising_edge().await;
    test_gpio.wait_for_rising_edge().await;
    test_gpio.wait_for_rising_edge().await;
    test_gpio.wait_for_rising_edge().await;
    done.signal(());
}

#[embedded_test::tests(executor = hil_test::Executor::new(), default_timeout = 3)]
mod tests {
    use super::*;

    // Restore the GPIO vector entry to point at the user-defined handler.
    fn rebind_user_gpio_handler() {
        bind_handler(
            Interrupt::GPIO,
            InterruptHandler::new(gpio_isr, Priority::Priority1),
        );
    }

    #[test]
    async fn default_handler_does_not_run_because_gpio_is_defined() {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let (gpio1, gpio2) = hil_test::common_test_pins!(peripherals);

        let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
        let timg0 = TimerGroup::new(peripherals.TIMG0);
        esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

        let counter = drive_pins(gpio1, gpio2).await;

        // GPIO is bound to something else, so we don't expect the async API to work.
        assert_eq!(counter, 0);
    }

    #[test]
    async fn default_handler_runs_because_handler_is_set() {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let mut io = Io::new(peripherals.IO_MUX);
        io.set_interrupt_handler(interrupt_handler);

        let (gpio1, gpio2) = hil_test::common_test_pins!(peripherals);
        let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
        let timg0 = TimerGroup::new(peripherals.TIMG0);
        esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

        let counter = drive_pins(gpio1, gpio2).await;

        // We expect the async API to keep working even if a user handler is set.
        assert_eq!(counter, 5);
    }

    #[test]
    async fn task_that_runs_at_handlers_priority_is_not_locked_up() {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        // Register an interrupt handler. Since we are not dealing with raw interrupts
        // here, it's okay to do nothing. Handling async GPIO events will
        // disable the corresponding interrupts.
        let mut io = Io::new(peripherals.IO_MUX);
        io.set_interrupt_handler(interrupt_handler);

        let (gpio1, gpio2) = hil_test::common_test_pins!(peripherals);

        let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
        let timg0 = TimerGroup::new(peripherals.TIMG0);
        esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

        let interrupt_executor = mk_static!(
            InterruptExecutor<1>,
            InterruptExecutor::new(sw_int.software_interrupt1)
        );
        // Run the executor at interrupt priority 1, which is the same as the default
        // interrupt priority of the GPIO interrupt handler.
        let interrupt_spawner = interrupt_executor.start(Priority::Priority1);

        let done = mk_static!(Signal<CriticalSectionRawMutex, ()>, Signal::new());

        interrupt_spawner.spawn(sense_pin(gpio1.degrade(), done).unwrap());
        interrupt_spawner.spawn(drive_pin(gpio2.degrade()).unwrap());

        done.wait().await;
    }

    // Regression test for esp-rs/esp-hal#2659: a user-defined raw `GPIO()`
    // handler that calls `handle_gpio_interrupt()` keeps the async API
    // fully functional, even though esp-hal's own dispatch shim never gets
    // to bind itself.
    #[test]
    async fn user_gpio_handler_can_drive_async_api_via_handle_gpio_interrupt() {
        rebind_user_gpio_handler();
        HANDLER_MODE.store(HandlerMode::HandleAll as u8, Ordering::Relaxed);

        let peripherals = esp_hal::init(esp_hal::Config::default());

        let (gpio1, gpio2) = hil_test::common_test_pins!(peripherals);

        let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
        let timg0 = TimerGroup::new(peripherals.TIMG0);
        esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

        let counter = drive_pins(gpio1, gpio2).await;

        assert!(
            HANDLER_INVOCATIONS.load(Ordering::Relaxed) > 0,
            "user-defined GPIO() handler was never invoked"
        );
        assert_eq!(counter, 5);
    }

    // Companion test for the per-pin escape hatch: the user handler calls
    // `wake_pin(n)` for exactly the pin being awaited.
    #[test]
    async fn user_gpio_handler_can_drive_async_api_via_wake_pin() {
        rebind_user_gpio_handler();

        let peripherals = esp_hal::init(esp_hal::Config::default());

        let (gpio1, gpio2) = hil_test::common_test_pins!(peripherals);

        // Record the awaited pin's number for the ISR before any event can fire.
        SENSE_PIN_NR.store(gpio1.number(), Ordering::Relaxed);
        HANDLER_MODE.store(HandlerMode::WakeSpecificPin as u8, Ordering::Relaxed);

        let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
        let timg0 = TimerGroup::new(peripherals.TIMG0);
        esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

        let counter = drive_pins(gpio1, gpio2).await;

        assert!(
            HANDLER_INVOCATIONS.load(Ordering::Relaxed) > 0,
            "user-defined GPIO() handler was never invoked"
        );
        assert_eq!(counter, 5);
    }
}
