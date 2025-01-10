//! System Timer Test

// esp32 disabled as it does not have a systimer
//% CHIPS: esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: unstable

#![no_std]
#![no_main]

use core::cell::RefCell;

use critical_section::Mutex;
use embedded_hal::delay::DelayNs;
use esp_hal::{
    delay::Delay,
    handler,
    time::ExtU64,
    timer::{
        systimer::{Alarm, SystemTimer},
        OneShotTimer,
        PeriodicTimer,
    },
    Blocking,
};
use hil_test as _;
use portable_atomic::{AtomicUsize, Ordering};

static ALARM_TARGET: Mutex<RefCell<Option<OneShotTimer<'static, Blocking>>>> =
    Mutex::new(RefCell::new(None));
static ALARM_PERIODIC: Mutex<RefCell<Option<PeriodicTimer<'static, Blocking>>>> =
    Mutex::new(RefCell::new(None));

struct Context {
    alarm0: Alarm,
    alarm1: Alarm,
}

#[handler(priority = esp_hal::interrupt::Priority::min())]
fn pass_test_if_called() {
    critical_section::with(|cs| {
        ALARM_TARGET
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .clear_interrupt()
    });
    embedded_test::export::check_outcome(());
}

#[handler(priority = esp_hal::interrupt::Priority::min())]
fn handle_periodic_interrupt() {
    critical_section::with(|cs| {
        ALARM_PERIODIC
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .clear_interrupt()
    });
}

static COUNTER: AtomicUsize = AtomicUsize::new(0);

#[handler(priority = esp_hal::interrupt::Priority::min())]
fn pass_test_if_called_twice() {
    critical_section::with(|cs| {
        ALARM_PERIODIC
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .clear_interrupt()
    });
    COUNTER.fetch_add(1, Ordering::Relaxed);
    if COUNTER.load(Ordering::Relaxed) == 2 {
        embedded_test::export::check_outcome(());
    }
}

#[handler(priority = esp_hal::interrupt::Priority::min())]
fn target_fail_test_if_called_twice() {
    critical_section::with(|cs| {
        ALARM_TARGET
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .clear_interrupt()
    });
    COUNTER.fetch_add(1, Ordering::Relaxed);
    assert!(COUNTER.load(Ordering::Relaxed) != 2);
}

#[cfg(test)]
#[embedded_test::tests(default_timeout = 3)]
mod tests {
    use super::*;

    #[init]
    fn init() -> Context {
        let peripherals = esp_hal::init(esp_hal::Config::default());
        let systimer = SystemTimer::new(peripherals.SYSTIMER);

        Context {
            alarm0: systimer.alarm0,
            alarm1: systimer.alarm1,
        }
    }

    #[test]
    fn target_interrupt_is_handled(ctx: Context) {
        let mut alarm0 = OneShotTimer::new(ctx.alarm0);

        critical_section::with(|cs| {
            alarm0.set_interrupt_handler(pass_test_if_called);
            alarm0.enable_interrupt(true);
            alarm0.schedule(10_u64.millis()).unwrap();

            ALARM_TARGET.borrow_ref_mut(cs).replace(alarm0);
        });

        // We'll end the test in the interrupt handler.
        loop {}
    }

    #[test]
    fn target_interrupt_is_handled_once(ctx: Context) {
        let mut alarm0 = OneShotTimer::new(ctx.alarm0);
        let mut alarm1 = PeriodicTimer::new(ctx.alarm1);

        COUNTER.store(0, Ordering::Relaxed);

        critical_section::with(|cs| {
            alarm0.set_interrupt_handler(target_fail_test_if_called_twice);
            alarm0.enable_interrupt(true);
            alarm0.schedule(10_u64.millis()).unwrap();

            alarm1.set_interrupt_handler(handle_periodic_interrupt);
            alarm1.enable_interrupt(true);
            alarm1.start(100u64.millis()).unwrap();

            ALARM_TARGET.borrow_ref_mut(cs).replace(alarm0);
            ALARM_PERIODIC.borrow_ref_mut(cs).replace(alarm1);
        });

        let mut delay = Delay::new();
        delay.delay_ms(300);
    }

    #[test]
    fn periodic_interrupt_is_handled(ctx: Context) {
        let mut alarm1 = PeriodicTimer::new(ctx.alarm1);

        COUNTER.store(0, Ordering::Relaxed);

        critical_section::with(|cs| {
            alarm1.set_interrupt_handler(pass_test_if_called_twice);
            alarm1.enable_interrupt(true);
            alarm1.start(100u64.millis()).unwrap();

            ALARM_PERIODIC.borrow_ref_mut(cs).replace(alarm1);
        });

        // We'll end the test in the interrupt handler.
        loop {}
    }
}
