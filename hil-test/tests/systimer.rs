//! System Timer Test

// esp32 disabled as it does not have a systimer
//% CHIPS: esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use core::cell::RefCell;

use critical_section::Mutex;
use embedded_hal::delay::DelayNs;
use esp_hal::{
    clock::{ClockControl, Clocks},
    delay::Delay,
    peripherals::Peripherals,
    prelude::*,
    system::SystemControl,
    timer::systimer::{
        Alarm,
        FrozenUnit,
        Periodic,
        SpecificComparator,
        SpecificUnit,
        SystemTimer,
        Target,
    },
    Blocking,
};
use fugit::ExtU32;
use hil_test as _;
use portable_atomic::{AtomicUsize, Ordering};
use static_cell::StaticCell;

type TestAlarm<M, const C: u8> =
    Alarm<'static, M, Blocking, SpecificComparator<'static, C>, SpecificUnit<'static, 0>>;

static ALARM_TARGET: Mutex<RefCell<Option<TestAlarm<Target, 0>>>> = Mutex::new(RefCell::new(None));
static ALARM_PERIODIC: Mutex<RefCell<Option<TestAlarm<Periodic, 1>>>> =
    Mutex::new(RefCell::new(None));

struct Context {
    unit: FrozenUnit<'static, SpecificUnit<'static, 0>>,
    comparator0: SpecificComparator<'static, 0>,
    comparator1: SpecificComparator<'static, 1>,
    clocks: Clocks<'static>,
}

impl Context {
    pub fn init() -> Self {
        let peripherals = Peripherals::take();
        let system = SystemControl::new(peripherals.SYSTEM);
        let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

        let systimer = SystemTimer::new(peripherals.SYSTIMER);
        static UNIT0: StaticCell<SpecificUnit<'static, 0>> = StaticCell::new();

        let unit0 = UNIT0.init(systimer.unit0);
        let frozen_unit = FrozenUnit::new(unit0);

        Context {
            clocks,
            unit: frozen_unit,
            comparator0: systimer.comparator0,
            comparator1: systimer.comparator1,
        }
    }
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
#[embedded_test::tests]
mod tests {
    use super::*;

    #[init]
    fn init() -> Context {
        Context::init()
    }

    #[test]
    #[timeout(3)]
    fn target_interrupt_is_handled(ctx: Context) {
        let alarm0 = Alarm::new(ctx.comparator0, &ctx.unit);

        critical_section::with(|cs| {
            alarm0.set_interrupt_handler(pass_test_if_called);
            alarm0.set_target(SystemTimer::now() + SystemTimer::ticks_per_second() / 10);
            alarm0.enable_interrupt(true);

            ALARM_TARGET.borrow_ref_mut(cs).replace(alarm0);
        });

        // We'll end the test in the interrupt handler.
        loop {}
    }

    #[test]
    #[timeout(3)]
    fn target_interrupt_is_handled_once(ctx: Context) {
        let alarm0 = Alarm::new(ctx.comparator0, &ctx.unit);
        let alarm1 = Alarm::new(ctx.comparator1, &ctx.unit);

        COUNTER.store(0, Ordering::Relaxed);

        critical_section::with(|cs| {
            alarm0.set_interrupt_handler(target_fail_test_if_called_twice);
            alarm0.set_target(SystemTimer::now() + SystemTimer::ticks_per_second() / 10);
            alarm0.enable_interrupt(true);

            let alarm1 = alarm1.into_periodic();
            alarm1.set_interrupt_handler(handle_periodic_interrupt);
            alarm1.set_period(100u32.millis());
            alarm1.enable_interrupt(true);

            ALARM_TARGET.borrow_ref_mut(cs).replace(alarm0);
            ALARM_PERIODIC.borrow_ref_mut(cs).replace(alarm1);
        });

        let mut delay = Delay::new(&ctx.clocks);
        delay.delay_ms(300);
    }

    #[test]
    #[timeout(3)]
    fn periodic_interrupt_is_handled(ctx: Context) {
        let alarm1 = Alarm::new(ctx.comparator1, &ctx.unit);

        COUNTER.store(0, Ordering::Relaxed);

        critical_section::with(|cs| {
            let alarm1 = alarm1.into_periodic();
            alarm1.set_interrupt_handler(pass_test_if_called_twice);
            alarm1.set_period(100u32.millis());
            alarm1.enable_interrupt(true);

            ALARM_PERIODIC.borrow_ref_mut(cs).replace(alarm1);
        });

        // We'll end the test in the interrupt handler.
        loop {}
    }
}
