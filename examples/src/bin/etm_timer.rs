//! This shows how to use the general purpose timers ETM tasks and events
//! Notice you need to import the traits esp_hal::timer::etm::{TimerEtmEvents, TimerEtmTasks}

//% CHIPS: esp32c6 esp32h2

#![no_std]
#![no_main]

use core::cell::RefCell;

use critical_section::Mutex;
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    delay::Delay,
    etm::Etm,
    interrupt::{self, Priority},
    peripherals::{Interrupt, Peripherals, TIMG0},
    prelude::*,
    timer::{
        etm::{TimerEtmEvents, TimerEtmTasks},
        Timer,
        Timer0,
        TimerGroup,
    },
};

static TIMER0: Mutex<RefCell<Option<Timer<Timer0<TIMG0>>>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let mut delay = Delay::new(&clocks);

    let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut timer0 = timg0.timer0;

    // Configure ETM to stop timer0 when alarm is triggered
    let event = timer0.on_alarm();
    let task = timer0.cnt_stop();

    let etm = Etm::new(peripherals.SOC_ETM);

    let channel0 = etm.channel0;

    let _configured_channel = channel0.setup(&event, &task);

    // Setup alarm at 100ms
    // 80 / 2 (default divider) timer clock cycles == 1 us
    timer0.load_alarm_value(100 * 1_000 * 40);
    timer0.set_alarm_active(true);

    // Enable interrupt that will be triggered by the alarm
    interrupt::enable(Interrupt::TG0_T0_LEVEL, Priority::Priority1).unwrap();

    timer0.listen();
    timer0.set_counter_active(true);

    critical_section::with(|cs| {
        TIMER0.borrow_ref_mut(cs).replace(timer0);
    });

    loop {
        delay.delay_ms(500u32);

        critical_section::with(|cs| {
            let mut timer0 = TIMER0.borrow_ref_mut(cs);
            let timer0 = timer0.as_mut().unwrap();
            // Counter value should be the same than in interrupt because we configured ETM to stop the timer on alarm
            esp_println::println!("counter in main: {}", timer0.now());
        });
    }
}

#[interrupt]
fn TG0_T0_LEVEL() {
    critical_section::with(|cs| {
        let mut timer0 = TIMER0.borrow_ref_mut(cs);
        let timer0 = timer0.as_mut().unwrap();

        timer0.clear_interrupt();

        // Counter value should be a very small number
        esp_println::println!("counter in interrupt: {}", timer0.now());
    });
}
