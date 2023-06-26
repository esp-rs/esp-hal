use critical_section::{CriticalSection, Mutex};
use peripherals::TIMG0;

use super::AlarmState;
use crate::{
    clock::Clocks,
    peripherals,
    prelude::*,
    timer::{Timer, Timer0},
};

#[cfg(not(any(esp32, esp32s2, esp32s3)))]
pub const ALARM_COUNT: usize = 1;
#[cfg(any(esp32, esp32s2, esp32s3))]
pub const ALARM_COUNT: usize = 2;

pub type TimerInner = Timer0<TIMG0>;
pub type TimerType = Timer<TimerInner>;

pub struct EmbassyTimer {
    pub(crate) alarms: Mutex<[AlarmState; ALARM_COUNT]>,
}

const ALARM_STATE_NONE: AlarmState = AlarmState::new();

embassy_time::time_driver_impl!(static DRIVER: EmbassyTimer = EmbassyTimer {
    alarms: Mutex::new([ALARM_STATE_NONE; ALARM_COUNT]),
});

impl EmbassyTimer {
    pub(crate) fn now() -> u64 {
        unsafe { TimerInner::steal() }.now()
    }

    pub(crate) fn trigger_alarm(&self, n: usize, cs: CriticalSection) {
        let alarm = &self.alarms.borrow(cs)[n];
        // safety:
        // - we can ignore the possiblity of `f` being unset (null) because of the
        //   safety contract of `allocate_alarm`.
        // - other than that we only store valid function pointers into alarm.callback
        let f: fn(*mut ()) = unsafe { core::mem::transmute(alarm.callback.get()) };
        f(alarm.ctx.get());
    }

    fn on_interrupt(&self, id: u8) {
        critical_section::with(|cs| unsafe {
            TimerInner::steal().clear_interrupt();
            self.trigger_alarm(id as usize, cs);
        });
    }

    pub fn init(clocks: &Clocks, mut timer: TimerType) {
        use crate::{interrupt, interrupt::Priority};

        // set divider to get a 1mhz clock. APB (80mhz) / 80 = 1mhz...
        // TODO: assert APB clock is the source and its at the correct speed for the
        // divider
        timer.set_divider(clocks.apb_clock.to_MHz() as u16);

        interrupt::enable(peripherals::Interrupt::TG0_T0_LEVEL, Priority::max()).unwrap();
        #[cfg(any(esp32, esp32s2, esp32s3))]
        interrupt::enable(peripherals::Interrupt::TG0_T1_LEVEL, Priority::max()).unwrap();

        #[interrupt]
        fn TG0_T0_LEVEL() {
            DRIVER.on_interrupt(0);
        }

        #[cfg(any(esp32, esp32s2, esp32s3))]
        #[interrupt]
        fn TG0_T1_LEVEL() {
            DRIVER.on_interrupt(1);
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
            let mut tg = unsafe { TimerInner::steal() };
            if timestamp < now {
                tg.unlisten();
                alarm_state.timestamp.set(u64::MAX);
                return false;
            }
            alarm_state.timestamp.set(timestamp);

            tg.load_alarm_value(timestamp);
            tg.listen();
            tg.set_counter_decrementing(false);
            tg.set_auto_reload(false);
            tg.set_counter_active(true);
            tg.set_alarm_active(true);

            true
        })
    }
}
