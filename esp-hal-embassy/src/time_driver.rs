use core::cell::Cell;

use embassy_time_driver::{AlarmHandle, Driver};
use esp_hal::{
    interrupt::{InterruptHandler, Priority},
    prelude::*,
    sync::Locked,
    time::now,
    timer::{AnyTimer, OneShotTimer},
};

pub type Timer = OneShotTimer<'static, AnyTimer>;

enum AlarmState {
    Created(extern "C" fn()),
    Allocated(extern "C" fn()),
    Initialized(&'static mut Timer),
}

struct Alarm {
    pub callback: Cell<(*const (), *mut ())>,
    pub state: AlarmState,
}

unsafe impl Send for Alarm {}

impl Alarm {
    pub const fn new(handler: extern "C" fn()) -> Self {
        Self {
            callback: Cell::new((core::ptr::null(), core::ptr::null_mut())),
            state: AlarmState::Created(handler),
        }
    }
}

pub(super) struct EmbassyTimer {
    alarms: Locked<[Alarm; MAX_SUPPORTED_ALARM_COUNT]>,
    available_timers: Locked<Option<&'static mut [Timer]>>,
}

/// Repeats the `Alarm::new` constructor for each alarm, creating an interrupt
/// handler for each of them.
macro_rules! alarms {
    ($($idx:literal),*) => {
        [$(
            Alarm::new({
                // Not #[handler] so we don't have to store the priority - which is constant.
                extern "C" fn handler() {
                    DRIVER.on_interrupt($idx);
                }
                handler
            }),
        )*]
    };
}

const MAX_SUPPORTED_ALARM_COUNT: usize = 7;
embassy_time_driver::time_driver_impl!(static DRIVER: EmbassyTimer = EmbassyTimer {
    alarms: Locked::new(alarms!(0, 1, 2, 3, 4, 5, 6)),
    available_timers: Locked::new(None),
});

impl EmbassyTimer {
    pub(super) fn init(mut timers: &'static mut [Timer]) {
        assert!(
            timers.len() <= MAX_SUPPORTED_ALARM_COUNT,
            "Maximum {} timers can be used.",
            MAX_SUPPORTED_ALARM_COUNT
        );

        // Reset timers
        timers.iter_mut().for_each(|timer| {
            timer.enable_interrupt(false);
            timer.stop();
        });

        // Initialize already allocated timers
        DRIVER.alarms.with(move |alarms| {
            for allocated_alarm in alarms.iter_mut() {
                if let AlarmState::Allocated(interrupt_handler) = allocated_alarm.state {
                    // Pluck off a timer

                    let Some((timer, rest)) = timers.split_first_mut() else {
                        not_enough_timers();
                    };
                    timers = rest;

                    // FIXME: we should track which core allocated an alarm and bind the
                    // interrupt to that core.
                    timer.set_interrupt_handler(InterruptHandler::new(
                        interrupt_handler,
                        Priority::max(),
                    ));
                    allocated_alarm.state = AlarmState::Initialized(timer);
                }
            }

            // Store the available timers
            DRIVER
                .available_timers
                .with(|available_timers| *available_timers = Some(timers));
        });
    }

    fn on_interrupt(&self, id: usize) {
        let (cb, ctx) = self.alarms.with(|alarms| {
            let alarm = &mut alarms[id];
            if let AlarmState::Initialized(timer) = &mut alarm.state {
                timer.clear_interrupt();
                alarm.callback.get()
            } else {
                unsafe {
                    // SAFETY: `on_interrupt` is registered right when the alarm is initialized.
                    core::hint::unreachable_unchecked()
                }
            }
        });

        let cb: fn(*mut ()) = unsafe {
            // Safety:
            // - we can ignore the possiblity of `f` being unset (null) because of the
            //   safety contract of `allocate_alarm`.
            // - other than that we only store valid function pointers into alarm.callback
            core::mem::transmute(cb)
        };

        cb(ctx);
    }

    fn arm(timer: &mut Timer, timestamp: u64) {
        let now = now().duration_since_epoch();
        let ts = timestamp.micros();
        // if the TS is already in the past make the timer fire immediately
        let timeout = if ts > now { ts - now } else { 0.micros() };
        unwrap!(timer.schedule(timeout));
        timer.enable_interrupt(true);
    }
}

impl Driver for EmbassyTimer {
    fn now(&self) -> u64 {
        now().ticks()
    }

    unsafe fn allocate_alarm(&self) -> Option<AlarmHandle> {
        let timer = self.available_timers.with(|available_timers| {
            // `allocate_alarm` may be called before `esp_hal_embassy::init()`. If
            // `timers` is `None`, we return `None` to signal that the alarm cannot be
            // initialized yet. These alarms will be initialized when `init` is called.
            if let Some(timers) = available_timers.take() {
                // If the driver is initialized, we can allocate a timer.
                // If this fails, we can't do anything about it.
                let Some((timer, rest)) = timers.split_first_mut() else {
                    not_enough_timers();
                };
                *available_timers = Some(rest);
                Some(timer)
            } else {
                None
            }
        });

        self.alarms.with(|alarms| {
            for (i, alarm) in alarms.iter_mut().enumerate() {
                if let AlarmState::Created(interrupt_handler) = alarm.state {
                    alarm.state = match timer {
                        Some(timer) => {
                            // If the driver is initialized, bind the interrupt handler to the
                            // timer. This ensures that alarms allocated after init are correctly
                            // bound to the core that created the executor.
                            timer.set_interrupt_handler(InterruptHandler::new(
                                interrupt_handler,
                                Priority::max(),
                            ));
                            AlarmState::Initialized(timer)
                        }

                        None => {
                            // No timers are available yet, mark the alarm as allocated.
                            AlarmState::Allocated(interrupt_handler)
                        }
                    };

                    return Some(AlarmHandle::new(i as u8));
                }
            }
            None
        })
    }

    fn set_alarm_callback(&self, alarm: AlarmHandle, callback: fn(*mut ()), ctx: *mut ()) {
        let n = alarm.id() as usize;

        self.alarms.with(|alarms| {
            alarms[n].callback.set((callback as *const (), ctx));
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

        self.alarms.with(|alarms| {
            let alarm = &mut alarms[alarm.id() as usize];
            if let AlarmState::Initialized(timer) = &mut alarm.state {
                Self::arm(timer, timestamp);
            } else {
                panic!("set_alarm called before esp_hal_embassy::init()")
            }
        });

        true
    }
}

#[cold]
#[track_caller]
fn not_enough_timers() -> ! {
    // This is wrapped in a separate function because rustfmt does not like
    // extremely long strings. Also, if log is used, this avoids storing the string
    // twice.
    panic!("There are not enough timers to allocate a new alarm. Call `esp_hal_embassy::init()` with the correct number of timers.");
}
