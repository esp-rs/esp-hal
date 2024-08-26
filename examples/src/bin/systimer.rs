//! This shows how to use the SYSTIMER peripheral including interrupts.
//!
//! It's an additional timer besides the TIMG peripherals.

//% CHIPS: esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use core::cell::RefCell;

use critical_section::Mutex;
use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    prelude::*,
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
use esp_println::println;
use fugit::ExtU32;
use static_cell::StaticCell;

static ALARM0: Mutex<
    RefCell<
        Option<Alarm<Periodic, Blocking, SpecificComparator<'static, 0>, SpecificUnit<'static, 0>>>,
    >,
> = Mutex::new(RefCell::new(None));
static ALARM1: Mutex<
    RefCell<
        Option<Alarm<Target, Blocking, SpecificComparator<'static, 1>, SpecificUnit<'static, 0>>>,
    >,
> = Mutex::new(RefCell::new(None));
static ALARM2: Mutex<
    RefCell<
        Option<Alarm<Target, Blocking, SpecificComparator<'static, 2>, SpecificUnit<'static, 0>>>,
    >,
> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let (peripherals, clocks) = esp_hal::init(CpuClock::boot_default());

    let systimer = SystemTimer::new(peripherals.SYSTIMER);
    println!("SYSTIMER Current value = {}", SystemTimer::now());

    static UNIT0: StaticCell<SpecificUnit<'static, 0>> = StaticCell::new();

    let unit0 = UNIT0.init(systimer.unit0);

    let frozen_unit = FrozenUnit::new(unit0);

    let alarm0 = Alarm::new(systimer.comparator0, &frozen_unit);
    let alarm1 = Alarm::new(systimer.comparator1, &frozen_unit);
    let alarm2 = Alarm::new(systimer.comparator2, &frozen_unit);

    critical_section::with(|cs| {
        let alarm0 = alarm0.into_periodic();
        alarm0.set_interrupt_handler(systimer_target0);
        alarm0.set_period(1u32.secs());
        alarm0.enable_interrupt(true);

        alarm1.set_interrupt_handler(systimer_target1);
        alarm1.set_target(SystemTimer::now() + (SystemTimer::ticks_per_second() * 2));
        alarm1.enable_interrupt(true);

        alarm2.set_interrupt_handler(systimer_target2);
        alarm2.set_target(SystemTimer::now() + (SystemTimer::ticks_per_second() * 3));
        alarm2.enable_interrupt(true);

        ALARM0.borrow_ref_mut(cs).replace(alarm0);
        ALARM1.borrow_ref_mut(cs).replace(alarm1);
        ALARM2.borrow_ref_mut(cs).replace(alarm2);
    });

    let delay = Delay::new(&clocks);

    loop {
        delay.delay_millis(500);
    }
}

#[handler(priority = esp_hal::interrupt::Priority::min())]
fn systimer_target0() {
    println!("Interrupt lvl1 (alarm0)");
    critical_section::with(|cs| {
        ALARM0
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .clear_interrupt()
    });
}

#[handler(priority = esp_hal::interrupt::Priority::Priority1)]
fn systimer_target1() {
    println!("Interrupt lvl2 (alarm1)");
    critical_section::with(|cs| {
        ALARM1
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .clear_interrupt()
    });
}

#[handler(priority = esp_hal::interrupt::Priority::max())]
fn systimer_target2() {
    println!("Interrupt lvl2 (alarm2)");
    critical_section::with(|cs| {
        ALARM2
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .clear_interrupt()
    });
}
