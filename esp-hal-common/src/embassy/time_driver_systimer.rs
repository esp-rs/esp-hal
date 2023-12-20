use critical_section::{CriticalSection, Mutex};

use super::AlarmState;
use crate::{
    clock::Clocks,
    peripherals,
    systimer::{Alarm, SystemTimer, Target},
};

pub const ALARM_COUNT: usize = 3;

pub type TimerType = SystemTimer<'static>;

pub struct EmbassyTimer {
    pub(crate) alarms: Mutex<[AlarmState; ALARM_COUNT]>,
    pub(crate) alarm0: Alarm<Target, 0>,
    pub(crate) alarm1: Alarm<Target, 1>,
    pub(crate) alarm2: Alarm<Target, 2>,
}

const ALARM_STATE_NONE: AlarmState = AlarmState::new();

embassy_time::time_driver_impl!(static DRIVER: EmbassyTimer = EmbassyTimer {
    alarms: Mutex::new([ALARM_STATE_NONE; ALARM_COUNT]),
    alarm0: unsafe { Alarm::<_, 0>::conjure() },
    alarm1: unsafe { Alarm::<_, 1>::conjure() },
    alarm2: unsafe { Alarm::<_, 2>::conjure() },
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

    pub(super) fn on_alarm_allocated(&self, n: usize) {
        match n {
            0 => self.alarm0.enable_interrupt(true),
            1 => self.alarm1.enable_interrupt(true),
            2 => self.alarm2.enable_interrupt(true),
            _ => {}
        }
    }

    fn on_interrupt(&self, id: usize) {
        critical_section::with(|cs| {
            self.clear_interrupt(id);
            self.trigger_alarm(id, cs);
        })
    }

    pub fn init(_clocks: &Clocks, _systimer: TimerType) {
        use crate::{interrupt, interrupt::Priority, macros::interrupt_internal};

        unwrap!(interrupt::enable(
            peripherals::Interrupt::SYSTIMER_TARGET0,
            Priority::max()
        ));
        unwrap!(interrupt::enable(
            peripherals::Interrupt::SYSTIMER_TARGET1,
            Priority::max()
        ));
        unwrap!(interrupt::enable(
            peripherals::Interrupt::SYSTIMER_TARGET2,
            Priority::max()
        ));

        #[interrupt_internal]
        fn SYSTIMER_TARGET0() {
            DRIVER.on_interrupt(0);
        }
        #[interrupt_internal]
        fn SYSTIMER_TARGET1() {
            DRIVER.on_interrupt(1);
        }
        #[interrupt_internal]
        fn SYSTIMER_TARGET2() {
            DRIVER.on_interrupt(2);
        }
    }

    pub(crate) fn set_alarm(
        &self,
        alarm: embassy_time::driver::AlarmHandle,
        timestamp: u64,
    ) -> bool {
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
        match id {
            0 => self.alarm0.clear_interrupt(),
            1 => self.alarm1.clear_interrupt(),
            2 => self.alarm2.clear_interrupt(),
            _ => {}
        }
    }

    fn arm(&self, id: usize, timestamp: u64) {
        match id {
            0 => self.alarm0.set_target(timestamp),
            1 => self.alarm1.set_target(timestamp),
            2 => self.alarm2.set_target(timestamp),
            _ => {}
        }
    }
}
