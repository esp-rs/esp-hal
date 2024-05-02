use critical_section::{CriticalSection, Mutex};
use peripherals::TIMG0;

use super::AlarmState;
#[cfg(any(esp32, esp32s2, esp32s3))]
use crate::timer::timg::Timer1;
use crate::{
    clock::Clocks,
    peripherals,
    prelude::*,
    timer::timg::{Instance, Timer0, TimerGroup},
};

#[cfg(not(any(esp32, esp32s2, esp32s3)))]
pub const ALARM_COUNT: usize = 1;
#[cfg(any(esp32, esp32s2, esp32s3))]
pub const ALARM_COUNT: usize = 2;

pub type TimerType = TimerGroup<'static, TIMG0, crate::Async>;

pub struct EmbassyTimer {
    pub(crate) alarms: Mutex<[AlarmState; ALARM_COUNT]>,
}

const ALARM_STATE_NONE: AlarmState = AlarmState::new();

embassy_time_driver::time_driver_impl!(static DRIVER: EmbassyTimer = EmbassyTimer {
    alarms: Mutex::new([ALARM_STATE_NONE; ALARM_COUNT]),
});

impl EmbassyTimer {
    pub(crate) fn now() -> u64 {
        unsafe { Timer0::<TIMG0>::steal() }.now()
    }

    fn trigger_alarm(&self, n: usize, cs: CriticalSection) {
        let alarm = &self.alarms.borrow(cs)[n];

        if let Some((f, ctx)) = alarm.callback.get() {
            f(ctx);
        }
    }

    pub(super) fn on_alarm_allocated(&self, _n: usize) {}

    fn on_interrupt<Timer: Instance>(&self, id: u8, mut timer: Timer) {
        critical_section::with(|cs| {
            timer.clear_interrupt();
            self.trigger_alarm(id as usize, cs);
        });
    }

    pub fn init(clocks: &Clocks, mut timer: TimerType) {
        // set divider to get a 1mhz clock. APB (80mhz) / 80 = 1mhz...
        timer.timer0.set_divider(clocks.apb_clock.to_MHz() as u16);
        timer.timer0.set_counter_active(true);

        #[cfg(any(esp32, esp32s2, esp32s3))]
        {
            timer.timer1.set_divider(clocks.apb_clock.to_MHz() as u16);
            timer.timer1.set_counter_active(true);
        }

        unsafe {
            crate::interrupt::bind_interrupt(
                crate::peripherals::Interrupt::TG0_T0_LEVEL,
                tg0_t0_level.handler(),
            );
            crate::interrupt::enable(
                crate::peripherals::Interrupt::TG0_T0_LEVEL,
                tg0_t0_level.priority(),
            )
            .unwrap();
        }
        #[cfg(any(esp32, esp32s2, esp32s3))]
        unsafe {
            crate::interrupt::bind_interrupt(
                crate::peripherals::Interrupt::TG0_T1_LEVEL,
                tg0_t1_level.handler(),
            );
            crate::interrupt::enable(
                crate::peripherals::Interrupt::TG0_T1_LEVEL,
                tg0_t1_level.priority(),
            )
            .unwrap();
        }

        #[handler(priority = crate::interrupt::Priority::max())]
        fn tg0_t0_level() {
            let timer = unsafe { Timer0::<TIMG0>::steal() };
            DRIVER.on_interrupt(0, timer);
        }

        #[cfg(any(esp32, esp32s2, esp32s3))]
        #[handler(priority = crate::interrupt::Priority::max())]
        fn tg0_t1_level() {
            let timer = unsafe { Timer1::<TIMG0>::steal() };
            DRIVER.on_interrupt(1, timer);
        }
    }

    pub(crate) fn set_alarm(
        &self,
        _alarm: embassy_time_driver::AlarmHandle,
        timestamp: u64,
    ) -> bool {
        critical_section::with(|_cs| {
            // The hardware fires the alarm even if timestamp is lower than the current
            // time. In this case the interrupt handler will pend a wakeup when we exit the
            // critical section.
            #[cfg(any(esp32, esp32s2, esp32s3))]
            if _alarm.id() == 1 {
                let mut tg = unsafe { Timer1::<TIMG0>::steal() };
                Self::arm(&mut tg, timestamp);
                return;
            }

            let mut tg = unsafe { Timer0::<TIMG0>::steal() };
            Self::arm(&mut tg, timestamp);
        });

        true
    }

    fn arm<Timer: Instance>(tg: &mut Timer, timestamp: u64) {
        tg.load_alarm_value(timestamp);
        tg.listen();
        tg.set_counter_decrementing(false);
        tg.set_auto_reload(false);
        tg.set_alarm_active(true);
    }
}
