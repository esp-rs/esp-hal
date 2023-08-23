use critical_section::{CriticalSection, Mutex};
use peripherals::TIMG0;

use super::AlarmState;
#[cfg(any(esp32, esp32s2, esp32s3))]
use crate::timer::Timer1;
use crate::{
    clock::Clocks,
    peripherals,
    prelude::*,
    timer::{Instance, Timer, Timer0},
};

#[cfg(not(any(esp32, esp32s2, esp32s3)))]
pub const ALARM_COUNT: usize = 1;
#[cfg(any(esp32, esp32s2, esp32s3))]
pub const ALARM_COUNT: usize = 2;

pub type TimerType = Timer<Timer0<TIMG0>>;

pub struct EmbassyTimer {
    pub(crate) alarms: Mutex<[AlarmState; ALARM_COUNT]>,
}

const ALARM_STATE_NONE: AlarmState = AlarmState::new();

embassy_time::time_driver_impl!(static DRIVER: EmbassyTimer = EmbassyTimer {
    alarms: Mutex::new([ALARM_STATE_NONE; ALARM_COUNT]),
});

impl EmbassyTimer {
    pub(crate) fn now() -> u64 {
        unsafe { Timer0::<TIMG0>::steal() }.now()
    }

    pub(crate) fn trigger_alarm(&self, n: usize, cs: CriticalSection) {
        let alarm = &self.alarms.borrow(cs)[n];
        alarm.timestamp.set(u64::MAX);

        if let Some((f, ctx)) = alarm.callback.get() {
            f(ctx);
        }
    }

    fn on_interrupt<Timer: Instance>(&self, id: u8, mut timer: Timer) {
        critical_section::with(|cs| {
            timer.clear_interrupt();
            self.trigger_alarm(id as usize, cs);
        });
    }

    pub fn init(clocks: &Clocks, mut timer: TimerType) {
        use crate::{interrupt, interrupt::Priority};

        // set divider to get a 1mhz clock. APB (80mhz) / 80 = 1mhz...
        // TODO: assert APB clock is the source and its at the correct speed for the
        // divider
        timer.set_divider(clocks.apb_clock.to_MHz() as u16);

        timer.set_counter_active(true);

        interrupt::enable(peripherals::Interrupt::TG0_T0_LEVEL, Priority::max()).unwrap();
        #[cfg(any(esp32, esp32s2, esp32s3))]
        interrupt::enable(peripherals::Interrupt::TG0_T1_LEVEL, Priority::max()).unwrap();

        #[interrupt]
        fn TG0_T0_LEVEL() {
            let timer = unsafe { Timer0::<TIMG0>::steal() };
            DRIVER.on_interrupt(0, timer);
        }

        #[cfg(any(esp32, esp32s2, esp32s3))]
        #[interrupt]
        fn TG0_T1_LEVEL() {
            let timer = unsafe { Timer1::<TIMG0>::steal() };
            DRIVER.on_interrupt(1, timer);
        }
    }

    pub(crate) fn set_alarm(
        &self,
        alarm: embassy_time::driver::AlarmHandle,
        timestamp: u64,
    ) -> bool {
        critical_section::with(|cs| {
            let n = alarm.id() as usize;
            let alarm_state = &self.alarms.borrow(cs)[n];

            #[cfg(any(esp32, esp32s2, esp32s3))]
            if n == 1 {
                let tg = unsafe { Timer1::<TIMG0>::steal() };
                return Self::set_alarm_impl(tg, alarm_state, timestamp);
            }

            let tg = unsafe { Timer0::<TIMG0>::steal() };
            Self::set_alarm_impl(tg, alarm_state, timestamp)
        })
    }

    fn set_alarm_impl<Timer: Instance>(
        mut tg: Timer,
        alarm_state: &AlarmState,
        timestamp: u64,
    ) -> bool {
        // The hardware fires the alarm even if timestamp is lower than the current
        // time.
        alarm_state.timestamp.set(timestamp);
        Self::arm(&mut tg, timestamp);

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
