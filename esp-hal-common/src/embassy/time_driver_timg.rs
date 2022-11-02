use core::cell::RefCell;

use critical_section::{CriticalSection, Mutex};
use pac::TIMG0;

use super::AlarmState;
use crate::{
    clock::Clocks,
    pac,
    prelude::*,
    timer::{Instance, TimerGroup},
};

pub const ALARM_COUNT: usize = 2;

pub struct EmbassyTimer {
    pub alarms: Mutex<[AlarmState; ALARM_COUNT]>,
}

const ALARM_STATE_NONE: AlarmState = AlarmState::new();

static TG: Mutex<RefCell<Option<TimerGroup<TIMG0>>>> = Mutex::new(RefCell::new(None));

embassy_time::time_driver_impl!(static DRIVER: EmbassyTimer = EmbassyTimer {
    alarms: Mutex::new([ALARM_STATE_NONE; ALARM_COUNT]),
});

impl EmbassyTimer {
    pub(crate) fn now() -> u64 {
        critical_section::with(|cs| TG.borrow_ref(cs).as_ref().unwrap().timer0.now())
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
        critical_section::with(|cs| {
            let mut tg = TG.borrow_ref_mut(cs);
            let tg = tg.as_mut().unwrap();
            match id {
                0 => tg.timer0.clear_interrupt(),
                1 => tg.timer1.clear_interrupt(),
                _ => unreachable!(),
            };
            self.trigger_alarm(id as usize, cs);
        });
    }

    pub(crate) fn init(clocks: &Clocks) {
        use crate::{interrupt, interrupt::Priority};

        // TODO can we avoid this steal in the future...
        let mut tg = TimerGroup::new(unsafe { pac::Peripherals::steal().TIMG0 }, clocks);
        // set divider to get a 1mhz clock. abp (80mhz) / 80 = 1mhz... // TODO assert
        // abp clock is the source and its at the correct speed for the divider
        tg.timer0.set_divider(clocks.apb_clock.to_MHz() as u16);
        tg.timer1.set_divider(clocks.apb_clock.to_MHz() as u16);

        critical_section::with(|cs| TG.borrow_ref_mut(cs).replace(tg));

        // TODO these priorities should probably be higher than 1...
        interrupt::enable(pac::Interrupt::TG0_T0_LEVEL, Priority::Priority1).unwrap();
        interrupt::enable(pac::Interrupt::TG0_T1_LEVEL, Priority::Priority1).unwrap();

        #[interrupt]
        fn TG0_T0_LEVEL() {
            DRIVER.on_interrupt(0);
        }

        #[interrupt]
        fn TG0_T1_LEVEL() {
            DRIVER.on_interrupt(1);
        }
    }

    pub(crate) fn set_alarm(&self, alarm: embassy_time::driver::AlarmHandle, timestamp: u64) -> bool {
        critical_section::with(|cs| {
            let now = Self::now();
            let alarm_state = unsafe { self.alarms.borrow(cs).get_unchecked(alarm.id() as usize) };
            let mut tg = TG.borrow_ref_mut(cs);
            let tg = tg.as_mut().unwrap();
            if timestamp < now {
                match alarm.id() {
                    0 => tg.timer0.unlisten(),
                    1 => tg.timer1.unlisten(),
                    _ => unreachable!()
                }
                alarm_state.timestamp.set(u64::MAX);
                return false;
            }
            alarm_state.timestamp.set(timestamp);

            match alarm.id() {
                0 => {
                    tg.timer0.load_alarm_value(timestamp);
                    tg.timer0.listen();
                    tg.timer0.set_counter_decrementing(false);
                    tg.timer0.set_auto_reload(false);
                    tg.timer0.set_counter_active(true);
                    tg.timer0.set_alarm_active(true);
                }
                1 => {
                    tg.timer1.load_alarm_value(timestamp);
                    tg.timer1.listen();
                    tg.timer1.set_counter_decrementing(false);
                    tg.timer1.set_auto_reload(false);
                    tg.timer1.set_counter_active(true);
                    tg.timer1.set_alarm_active(true);
                }
                _ => unreachable!(),
            }

            true
        })
    }
}
