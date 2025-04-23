//! GPIO interrupt handler tests
//!
//! This test checks that during HAL initialization we do not overwrite custom
//! GPIO interrupt handlers. We also check that binding a custom interrupt
//! handler explicitly overwrites the handler set by the user, as well as the
//! async API works for user handlers automatically.

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
    interrupt::{Priority, software::SoftwareInterruptControl},
    timer::timg::TimerGroup,
};
use esp_hal_embassy::InterruptExecutor;
use hil_test as _;
use portable_atomic::{AtomicUsize, Ordering};

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

#[unsafe(no_mangle)]
unsafe extern "C" fn GPIO() {
    // Prevents binding the default handler, but we need to clear the GPIO
    // interrupts by hand.
    let peripherals = unsafe { esp_hal::peripherals::Peripherals::steal() };

    let (gpio1, _) = hil_test::common_test_pins!(peripherals);

    // Using flex will reinitialize the pin, but it's okay here since we access an
    // Input.
    let mut gpio1 = Flex::new(gpio1);

    gpio1.unlisten();
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

#[cfg(test)]
#[embedded_test::tests(executor = hil_test::Executor::new(), default_timeout = 3)]
mod tests {
    use super::*;

    #[test]
    async fn default_handler_does_not_run_because_gpio_is_defined() {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let (gpio1, gpio2) = hil_test::common_test_pins!(peripherals);

        let timg0 = TimerGroup::new(peripherals.TIMG0);
        esp_hal_embassy::init(timg0.timer0);

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

        let timg0 = TimerGroup::new(peripherals.TIMG0);
        esp_hal_embassy::init(timg0.timer0);

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

        let timg0 = TimerGroup::new(peripherals.TIMG0);
        esp_hal_embassy::init(timg0.timer0);

        let sw_ints = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
        let interrupt_executor = mk_static!(
            InterruptExecutor<1>,
            InterruptExecutor::new(sw_ints.software_interrupt1)
        );
        // Run the executor at interrupt priority 1, which is the same as the default
        // interrupt priority of the GPIO interrupt handler.
        let interrupt_spwaner = interrupt_executor.start(Priority::Priority1);

        let done = mk_static!(Signal<CriticalSectionRawMutex, ()>, Signal::new());

        interrupt_spwaner.must_spawn(sense_pin(gpio1.degrade(), done));
        interrupt_spwaner.must_spawn(drive_pin(gpio2.degrade()));

        done.wait().await;
    }
}
