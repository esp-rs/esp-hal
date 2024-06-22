use core::cell::OnceCell;

use critical_section::{CriticalSection, Mutex};
use embassy_time_driver::AlarmHandle;
use esp_hal::{
    clock::Clocks,
    interrupt,
    peripherals::Interrupt,
    prelude::*,
    timer::systimer::{Alarm, SystemTimer, Target},
    Async,
};

use crate::AlarmState;

pub const ALARM_COUNT: usize = 3;

pub type TimerType = SystemTimer<'static>;

pub struct EmbassyTimer {
    pub(crate) alarms: Mutex<[AlarmState; ALARM_COUNT]>,
    pub(crate) alarm0: Mutex<OnceCell<Alarm<'static, Target, Async, 0, 0>>>,
    pub(crate) alarm1: Mutex<OnceCell<Alarm<'static, Target, Async, 1, 0>>>,
    pub(crate) alarm2: Mutex<OnceCell<Alarm<'static, Target, Async, 2, 0>>>,
}

#[allow(clippy::declare_interior_mutable_const)]
const ALARM_STATE_NONE: AlarmState = AlarmState::new();

embassy_time_driver::time_driver_impl!(static DRIVER: EmbassyTimer = EmbassyTimer {
    alarms: Mutex::new([ALARM_STATE_NONE; ALARM_COUNT]),
    alarm0: Mutex::new(OnceCell::new()),
    alarm1: Mutex::new(OnceCell::new()),
    alarm2: Mutex::new(OnceCell::new()),
});

impl EmbassyTimer {
    pub(crate) fn now() -> u64 {
        SystemTimer::now()
    }

    fn trigger_alarm(&self, n: usize, cs: CriticalSection) {
        let alarm = &self.alarms.borrow(cs)[n];

        if let Some((f, ctx)) = alarm.callback.get() {
            f(ctx);
        }
    }

    pub(crate) fn on_alarm_allocated(&self, n: usize) {
        critical_section::with(|cs| match n {
            0 => self.alarm0.borrow(cs).get().unwrap().enable_interrupt(true),
            1 => self.alarm1.borrow(cs).get().unwrap().enable_interrupt(true),
            2 => self.alarm2.borrow(cs).get().unwrap().enable_interrupt(true),
            _ => {}
        })
    }

    fn on_interrupt(&self, id: usize) {
        critical_section::with(|cs| {
            self.clear_interrupt(id);
            self.trigger_alarm(id, cs);
        })
    }

    pub fn init(_clocks: &Clocks, systimer: TimerType) {
        let alarms = systimer.split_async();

        critical_section::with(|cs| {
            DRIVER.alarm0.borrow(cs).set(alarms.alarm0).unwrap();
            DRIVER.alarm1.borrow(cs).set(alarms.alarm1).unwrap();
            DRIVER.alarm2.borrow(cs).set(alarms.alarm2).unwrap();
        });

        unsafe {
            interrupt::bind_interrupt(Interrupt::SYSTIMER_TARGET0, target0_handler.handler());
            unwrap!(interrupt::enable(
                Interrupt::SYSTIMER_TARGET0,
                target0_handler.priority()
            ));

            interrupt::bind_interrupt(Interrupt::SYSTIMER_TARGET1, target1_handler.handler());
            unwrap!(interrupt::enable(
                Interrupt::SYSTIMER_TARGET1,
                target1_handler.priority()
            ));

            interrupt::bind_interrupt(Interrupt::SYSTIMER_TARGET2, target2_handler.handler());
            unwrap!(interrupt::enable(
                Interrupt::SYSTIMER_TARGET2,
                target2_handler.priority()
            ));
        }

        #[handler]
        fn target0_handler() {
            DRIVER.on_interrupt(0);
        }

        #[handler]
        fn target1_handler() {
            DRIVER.on_interrupt(1);
        }

        #[handler]
        fn target2_handler() {
            DRIVER.on_interrupt(2);
        }
    }

    pub(crate) fn set_alarm(&self, alarm: AlarmHandle, timestamp: u64) -> bool {
        critical_section::with(|_cs| {
            let n = alarm.id() as usize;

            // The hardware fires the alarm even if timestamp is lower than the current
            // time. In this case the interrupt handler will pend a wakeup when we exit the
            // critical section.
            self.arm(n, timestamp);
        });

        // In theory, the above comment is true. However, in practice, we seem to be
        // missing interrupt for very short timeouts, so let's make sure and catch
        // timestamps that already passed. Returning `false` means embassy will
        // run one more poll loop.
        Self::now() < timestamp
    }

    fn clear_interrupt(&self, id: usize) {
        critical_section::with(|cs| match id {
            0 => self.alarm0.borrow(cs).get().unwrap().clear_interrupt(),
            1 => self.alarm1.borrow(cs).get().unwrap().clear_interrupt(),
            2 => self.alarm2.borrow(cs).get().unwrap().clear_interrupt(),
            _ => {}
        })
    }

    fn arm(&self, id: usize, timestamp: u64) {
        critical_section::with(|cs| match id {
            0 => self.alarm0.borrow(cs).get().unwrap().set_target(timestamp),
            1 => self.alarm1.borrow(cs).get().unwrap().set_target(timestamp),
            2 => self.alarm2.borrow(cs).get().unwrap().set_target(timestamp),
            _ => {}
        })
    }
}
