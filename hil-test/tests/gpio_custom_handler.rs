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

use embassy_time::{Duration, Timer};
use esp_hal::{
    gpio::{AnyPin, Flex, Input, InputConfig, Io, Level, Output, OutputConfig, Pull},
    handler,
    timer::timg::TimerGroup,
};
use hil_test as _;
use portable_atomic::{AtomicUsize, Ordering};

#[no_mangle]
unsafe extern "C" fn GPIO() {
    // Prevents binding the default handler, but we need to clear the GPIO
    // interrupts by hand.
    let peripherals = esp_hal::peripherals::Peripherals::steal();

    let (gpio1, _) = hil_test::common_test_pins!(peripherals);

    // Using flex will not mutate the pin.
    let mut gpio1 = Flex::new(gpio1);

    gpio1.clear_interrupt();
}

#[handler]
pub fn interrupt_handler() {
    // Do nothing
}

async fn drive_pins(gpio1: impl Into<AnyPin>, gpio2: impl Into<AnyPin>) -> usize {
    let counter = AtomicUsize::new(0);
    let mut test_gpio1 = Input::new(gpio1.into(), InputConfig::default().with_pull(Pull::Down));
    let mut test_gpio2 = Output::new(gpio2.into(), Level::Low, OutputConfig::default());
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

#[cfg(test)]
#[embedded_test::tests(executor = hil_test::Executor::new())]
mod tests {

    use super::*;

    #[test]
    async fn default_handler_does_not_run_because_gpio_is_defined() {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let (gpio1, gpio2) = hil_test::common_test_pins!(peripherals);

        let timg0 = TimerGroup::new(peripherals.TIMG0);
        esp_hal_embassy::init(timg0.timer0);

        // We need to enable the GPIO interrupt, otherwise the async Future's
        // setup or Drop implementation hangs.
        esp_hal::interrupt::enable(
            esp_hal::peripherals::Interrupt::GPIO,
            esp_hal::interrupt::Priority::Priority1,
        )
        .unwrap();

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
}
