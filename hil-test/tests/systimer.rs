//! System Timer Test

// esp32 disabled as it does not have a systimer
//% CHIPS: esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use core::cell::RefCell;

use critical_section::Mutex;
use esp_hal::{
    clock::ClockControl,
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

type TestAlarm<M> =
    Alarm<'static, M, Blocking, SpecificComparator<'static, 0>, SpecificUnit<'static, 0>>;

static ALARM_PERIODIC: Mutex<RefCell<Option<TestAlarm<Periodic>>>> = Mutex::new(RefCell::new(None));
static ALARM_TARGET: Mutex<RefCell<Option<TestAlarm<Target>>>> = Mutex::new(RefCell::new(None));

struct Context {
    unit: FrozenUnit<'static, SpecificUnit<'static, 0>>,
    comparator: SpecificComparator<'static, 0>,
}

impl Context {
    pub fn init() -> Self {
        let peripherals = Peripherals::take();
        let system = SystemControl::new(peripherals.SYSTEM);
        let _clocks = ClockControl::boot_defaults(system.clock_control).freeze();

        let systimer = SystemTimer::new(peripherals.SYSTIMER);
        static UNIT0: StaticCell<SpecificUnit<'static, 0>> = StaticCell::new();

        let unit0 = UNIT0.init(systimer.unit0);
        let frozen_unit = FrozenUnit::new(unit0);

        Context {
            unit: frozen_unit,
            comparator: systimer.comparator0,
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
        let alarm0 = Alarm::new(ctx.comparator, &ctx.unit);

        critical_section::with(|cs| {
            alarm0.set_interrupt_handler(pass_test_if_called);
            alarm0.set_target(SystemTimer::now() + SystemTimer::TICKS_PER_SECOND / 10);
            alarm0.enable_interrupt(true);

            ALARM_TARGET.borrow_ref_mut(cs).replace(alarm0);
        });

        // We'll end the test in the interrupt handler.
        loop {}
    }

    #[test]
    #[timeout(3)]
    fn periodic_interrupt_is_handled(ctx: Context) {
        let alarm0 = Alarm::new(ctx.comparator, &ctx.unit);

        COUNTER.store(0, Ordering::Relaxed);

        critical_section::with(|cs| {
            let alarm0 = alarm0.into_periodic();
            alarm0.set_interrupt_handler(pass_test_if_called_twice);
            alarm0.set_period(100u32.millis());
            alarm0.enable_interrupt(true);

            ALARM_PERIODIC.borrow_ref_mut(cs).replace(alarm0);
        });

        // We'll end the test in the interrupt handler.
        loop {}
    }
}
