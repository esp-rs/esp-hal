use core::cell::{Cell, RefCell};

use critical_section::Mutex;
use embassy_time_driver::{AlarmHandle, Driver};
use esp_hal::{
    interrupt::{InterruptHandler, Priority},
    prelude::*,
    time::now,
    timer::{ErasedTimer, OneShotTimer},
};

pub const MAX_SUPPORTED_ALARM_COUNT: usize = 7;

pub type Timer = OneShotTimer<'static, ErasedTimer>;

static TIMERS: Mutex<RefCell<Option<&'static mut [Timer]>>> = Mutex::new(RefCell::new(None));

#[allow(clippy::type_complexity)]
struct AlarmState {
    pub callback: Cell<Option<(fn(*mut ()), *mut ())>>,
    pub allocated: Cell<bool>,
}

unsafe impl Send for AlarmState {}

impl AlarmState {
    pub const fn new() -> Self {
        Self {
            callback: Cell::new(None),
            allocated: Cell::new(false),
        }
    }
}

pub(super) struct EmbassyTimer {
    alarms: Mutex<[AlarmState; MAX_SUPPORTED_ALARM_COUNT]>,
}

#[allow(clippy::declare_interior_mutable_const)]
const ALARM_STATE_NONE: AlarmState = AlarmState::new();

embassy_time_driver::time_driver_impl!(static DRIVER: EmbassyTimer = EmbassyTimer {
    alarms: Mutex::new([ALARM_STATE_NONE; MAX_SUPPORTED_ALARM_COUNT]),
});

impl EmbassyTimer {
    pub(super) fn init(timers: &'static mut [Timer]) {
        if timers.len() > MAX_SUPPORTED_ALARM_COUNT {
            panic!(
                "Maximum of {} timers can be used.",
                MAX_SUPPORTED_ALARM_COUNT
            );
        }

        static HANDLERS: [InterruptHandler; MAX_SUPPORTED_ALARM_COUNT] = [
            handler0, handler1, handler2, handler3, handler4, handler5, handler6,
        ];

        critical_section::with(|cs| {
            timers.iter_mut().enumerate().for_each(|(n, timer)| {
                timer.enable_interrupt(false);
                timer.stop();
                timer.set_interrupt_handler(HANDLERS[n]);
            });

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

    fn on_interrupt(&self, id: usize) {
        let cb = critical_section::with(|cs| {
            let mut timers = TIMERS.borrow_ref_mut(cs);
            let timers = timers.as_mut().expect("Time driver not initialized");
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

    fn arm(timer: &mut Timer, timestamp: u64) {
        let now = now().duration_since_epoch();
        let ts = timestamp.micros();
        // if the TS is already in the past make the timer fire immediately
        let timeout = if ts > now { ts - now } else { 0.micros() };
        timer.schedule(timeout).unwrap();
        timer.enable_interrupt(true);
    }
}

impl Driver for EmbassyTimer {
    fn now(&self) -> u64 {
        now().ticks()
    }

    unsafe fn allocate_alarm(&self) -> Option<AlarmHandle> {
        critical_section::with(|cs| {
            for (i, alarm) in self.alarms.borrow(cs).iter().enumerate() {
                if !alarm.allocated.get() {
                    // set alarm so it is not overwritten
                    alarm.allocated.set(true);
                    return Some(AlarmHandle::new(i as u8));
                }
            }
            None
        })
    }

    fn set_alarm_callback(&self, alarm: AlarmHandle, callback: fn(*mut ()), ctx: *mut ()) {
        let n = alarm.id() as usize;
        critical_section::with(|cs| {
            let alarm = &self.alarms.borrow(cs)[n];
            alarm.callback.set(Some((callback, ctx)));
        })
    }

    fn set_alarm(&self, alarm: AlarmHandle, timestamp: u64) -> bool {
        // If `embassy-executor/integrated-timers` is enabled and there are no pending
        // timers, embassy still calls `set_alarm` with `u64::MAX`. By returning
        // `true` we signal that no re-polling is necessary.
        if timestamp == u64::MAX {
            return true;
        }

        // The hardware fires the alarm even if timestamp is lower than the current
        // time. In this case the interrupt handler will pend a wake-up when we exit the
        // critical section.
        //
        // This is correct behavior. See https://docs.rs/embassy-time-driver/0.1.0/embassy_time_driver/trait.Driver.html#tymethod.set_alarm
        // (... the driver should return true and arrange to call the alarm callback as
        // soon as possible, but not synchronously.)
        critical_section::with(|cs| {
            let mut timers = TIMERS.borrow_ref_mut(cs);
            let timers = timers.as_mut().expect("Time driver not initialized");
            let timer = &mut timers[alarm.id() as usize];

            Self::arm(timer, timestamp);
        });

        true
    }
}
