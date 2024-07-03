use core::cell::RefCell;

use critical_section::Mutex;
use embassy_time_driver::AlarmHandle;
use esp_hal::{
    clock::Clocks,
    interrupt::{InterruptHandler, Priority},
    prelude::*,
    time::current_time,
    timer::{ErasedTimer, OneShotTimer},
};

use crate::AlarmState;

pub const MAX_SUPPORTED_ALARM_COUNT: usize = 7;

pub type TimerType = OneShotTimer<ErasedTimer>;

static TIMERS: Mutex<RefCell<Option<&'static mut [TimerType]>>> = Mutex::new(RefCell::new(None));

pub struct EmbassyTimer {
    pub(crate) alarms: Mutex<[AlarmState; MAX_SUPPORTED_ALARM_COUNT]>,
}

#[allow(clippy::declare_interior_mutable_const)]
const ALARM_STATE_NONE: AlarmState = AlarmState::new();

embassy_time_driver::time_driver_impl!(static DRIVER: EmbassyTimer = EmbassyTimer {
    alarms: Mutex::new([ALARM_STATE_NONE; MAX_SUPPORTED_ALARM_COUNT]),
});

impl EmbassyTimer {
    pub(crate) fn now() -> u64 {
        current_time().ticks()
    }

    pub(crate) fn on_alarm_allocated(&self, _n: usize) {
        info!("on_alarm_allocated {}", _n);
    }

    fn on_interrupt(&self, id: usize) {
        let cb = critical_section::with(|cs| {
            let mut timers = TIMERS.borrow_ref_mut(cs);
            let timers = timers.as_mut().unwrap();
            let timer = &mut timers[id];

            timer.clear_interrupt();

            let alarm = &self.alarms.borrow(cs)[id];

            if let Some((f, ctx)) = alarm.callback.get() {
                Some((f, ctx))
            } else {
                None
            }
        });

        if let Some((f, ctx)) = cb {
            f(ctx);
        }
    }

    pub fn init(_clocks: &Clocks, timers: &'static mut [TimerType]) {
        static HANDLERS: [InterruptHandler; MAX_SUPPORTED_ALARM_COUNT] = [
            handler0, handler1, handler2, handler3, handler4, handler5, handler6,
        ];

        timers
            .iter_mut()
            .enumerate()
            .for_each(|(n, timer)| timer.set_interrupt_handler(HANDLERS[n]));

        critical_section::with(|cs| {
            TIMERS.replace(cs, Some(timers));
        });

        #[handler(priority = Priority::max())]
        fn handler0() {
            DRIVER.on_interrupt(0);
        }
        #[handler(priority = Priority::max())]
        fn handler1() {
            DRIVER.on_interrupt(1);
        }
        #[handler(priority = Priority::max())]
        fn handler2() {
            DRIVER.on_interrupt(2);
        }
        #[handler(priority = Priority::max())]
        fn handler3() {
            DRIVER.on_interrupt(3);
        }
        #[handler(priority = Priority::max())]
        fn handler4() {
            DRIVER.on_interrupt(4);
        }
        #[handler(priority = Priority::max())]
        fn handler5() {
            DRIVER.on_interrupt(5);
        }
        #[handler(priority = Priority::max())]
        fn handler6() {
            DRIVER.on_interrupt(6);
        }
    }

    pub(crate) fn set_alarm(&self, alarm: AlarmHandle, timestamp: u64) -> bool {
        // The hardware fires the alarm even if timestamp is lower than the current
        // time. In this case the interrupt handler will pend a wakeup when we exit the
        // critical section.
        //
        // This is correct behavior. See https://docs.rs/embassy-time-driver/0.1.0/embassy_time_driver/trait.Driver.html#tymethod.set_alarm
        // (... the driver should return true and arrange to call the alarm callback as
        // soon as possible, but not synchronously.)
        critical_section::with(|cs| {
            let mut timers = TIMERS.borrow_ref_mut(cs);
            let timers = timers.as_mut().unwrap();
            let timer = &mut timers[alarm.id() as usize];

            Self::arm(timer, timestamp);
        });

        true
    }

    fn arm(timer: &mut TimerType, timestamp: u64) {
        let now = current_time().duration_since_epoch();
        let ts = timestamp.micros();
        let timeout = if ts > now { ts - now } else { now };
        timer.schedule(timeout).unwrap();
        timer.enable_interrupt(true);
    }
}
