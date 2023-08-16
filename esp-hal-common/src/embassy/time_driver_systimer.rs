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

    pub(crate) fn trigger_alarm(&self, n: usize, cs: CriticalSection) {
        let alarm = &self.alarms.borrow(cs)[n];
        alarm.timestamp.set(u64::MAX);

        if let Some((f, ctx)) = alarm.callback.get() {
            f(ctx);
        }
    }

    fn on_interrupt(&self, id: u8) {
        match id {
            0 => self.alarm0.clear_interrupt(),
            1 => self.alarm1.clear_interrupt(),
            2 => self.alarm2.clear_interrupt(),
            _ => unreachable!(),
        };
        critical_section::with(|cs| {
            self.trigger_alarm(id as usize, cs);
        })
    }

    pub fn init(_clocks: &Clocks, _systimer: TimerType) {
        use crate::{interrupt, interrupt::Priority, macros::interrupt};

        interrupt::enable(peripherals::Interrupt::SYSTIMER_TARGET0, Priority::max()).unwrap();
        interrupt::enable(peripherals::Interrupt::SYSTIMER_TARGET1, Priority::max()).unwrap();
        interrupt::enable(peripherals::Interrupt::SYSTIMER_TARGET2, Priority::max()).unwrap();

        #[interrupt]
        fn SYSTIMER_TARGET0() {
            DRIVER.on_interrupt(0);
        }
        #[interrupt]
        fn SYSTIMER_TARGET1() {
            DRIVER.on_interrupt(1);
        }
        #[interrupt]
        fn SYSTIMER_TARGET2() {
            DRIVER.on_interrupt(2);
        }
    }

    pub(crate) fn set_alarm(
        &self,
        alarm: embassy_time::driver::AlarmHandle,
        timestamp: u64,
    ) -> bool {
        critical_section::with(|cs| {
            let now = Self::now();
            let alarm_state = unsafe { self.alarms.borrow(cs).get_unchecked(alarm.id() as usize) };
            if timestamp < now {
                // If alarm timestamp has passed the alarm will not fire.
                // Disarm the alarm and return `false` to indicate that.
                self.disable_interrupt(alarm.id());
                alarm_state.timestamp.set(u64::MAX);
                return false;
            }
            alarm_state.timestamp.set(timestamp);
            match alarm.id() {
                0 => {
                    self.alarm0.set_target(timestamp);
                    self.alarm0.interrupt_enable(true);
                }
                1 => {
                    self.alarm1.set_target(timestamp);
                    self.alarm1.interrupt_enable(true);
                }
                2 => {
                    self.alarm2.set_target(timestamp);
                    self.alarm2.interrupt_enable(true);
                }
                _ => panic!(),
            }

            true
        })
    }

    fn disable_interrupt(&self, id: u8) {
        match id {
            0 => self.alarm0.interrupt_enable(false),
            1 => self.alarm1.interrupt_enable(false),
            2 => self.alarm2.interrupt_enable(false),
            _ => unreachable!(),
        };
    }
}
