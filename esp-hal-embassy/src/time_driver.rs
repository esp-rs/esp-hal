use core::cell::Cell;

use embassy_time_driver::{AlarmHandle, Driver};
use esp_hal::{
    interrupt::{InterruptHandler, Priority},
    prelude::*,
    sync::Locked,
    time::now,
    timer::{AnyTimer, OneShotTimer},
    Blocking,
};

pub type Timer = OneShotTimer<'static, AnyTimer>;

enum AlarmState {
    Created(extern "C" fn()),
    Allocated(extern "C" fn()),
    Initialized(&'static mut Timer),
}
impl AlarmState {
    fn initialize(
        timer: &'static mut OneShotTimer<'_, AnyTimer>,
        interrupt_handler: extern "C" fn(),
    ) -> AlarmState {
        // If the driver is initialized, bind the interrupt handler to the
        // timer. This ensures that alarms allocated after init are correctly
        // bound to the core that created the executor.
        timer.set_interrupt_handler(InterruptHandler::new(interrupt_handler, Priority::max()));
        timer.enable_interrupt(true);
        AlarmState::Initialized(timer)
    }
}

struct AlarmInner {
    pub callback: Cell<(*const (), *mut ())>,
    pub state: AlarmState,
}

struct Alarm {
    pub inner: Locked<AlarmInner>,
}

unsafe impl Send for Alarm {}

impl Alarm {
    pub const fn new(handler: extern "C" fn()) -> Self {
        Self {
            inner: Locked::new(AlarmInner {
                callback: Cell::new((core::ptr::null(), core::ptr::null_mut())),
                state: AlarmState::Created(handler),
            }),
        }
    }
}

pub(super) struct EmbassyTimer {
    alarms: [Alarm; MAX_SUPPORTED_ALARM_COUNT],
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
            })
        ),*]
    };
}

const MAX_SUPPORTED_ALARM_COUNT: usize = 7;
embassy_time_driver::time_driver_impl!(static DRIVER: EmbassyTimer = EmbassyTimer {
    alarms: alarms!(0, 1, 2, 3, 4, 5, 6),
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
        for alarm in DRIVER.alarms.iter() {
            timers = alarm.inner.with(move |alarm| {
                if let AlarmState::Allocated(interrupt_handler) = alarm.state {
                    // Pluck off a timer

                    let Some((timer, remaining_timers)) = timers.split_first_mut() else {
                        not_enough_timers();
                    };

                    alarm.state = AlarmState::initialize(timer, interrupt_handler);

                    remaining_timers
                } else {
                    timers
                }
            });
        }

        // Store the available timers
        DRIVER
            .available_timers
            .with(|available_timers| *available_timers = Some(timers));
    }

    fn on_interrupt(&self, id: usize) {
        let (cb, ctx) = self.alarms[id].inner.with(|alarm| {
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
            // - we can ignore the possibility of `f` being unset (null) because of the
            //   safety contract of `allocate_alarm`.
            // - other than that we only store valid function pointers into alarm.callback
            core::mem::transmute(cb)
        };

        cb(ctx);
    }

    /// Returns `true` if the timer was armed, `false` if the timestamp is in
    /// the past.
    fn arm(timer: &mut Timer, timestamp: u64) -> bool {
        let now = now().duration_since_epoch();
        let ts = timestamp.micros();

        if ts > now {
            let timeout = ts - now;
            unwrap!(timer.schedule(timeout));
            true
        } else {
            // If the timestamp is past, we return `false` to ask embassy to poll again
            // immediately.
            timer.stop();
            false
        }
    }
}

impl Driver for EmbassyTimer {
    fn now(&self) -> u64 {
        now().ticks()
    }

    unsafe fn allocate_alarm(&self) -> Option<AlarmHandle> {
        for (i, alarm) in self.alarms.iter().enumerate() {
            let handle = alarm.inner.with(|alarm| {
                let AlarmState::Created(interrupt_handler) = alarm.state else {
                    return None;
                };

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

                alarm.state = match timer {
                    Some(timer) => AlarmState::initialize(timer, interrupt_handler),

                    None => {
                        // No timers are available yet, mark the alarm as allocated.
                        AlarmState::Allocated(interrupt_handler)
                    }
                };

                Some(AlarmHandle::new(i as u8))
            });

            if handle.is_some() {
                return handle;
            }
        }

        None
    }

    fn set_alarm_callback(&self, alarm: AlarmHandle, callback: fn(*mut ()), ctx: *mut ()) {
        let n = alarm.id() as usize;

        self.alarms[n].inner.with(|alarm| {
            alarm.callback.set((callback as *const (), ctx));
        })
    }

    fn set_alarm(&self, alarm: AlarmHandle, timestamp: u64) -> bool {
        let alarm = &self.alarms[alarm.id() as usize];

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

        alarm.inner.with(|alarm| {
            if let AlarmState::Initialized(timer) = &mut alarm.state {
                Self::arm(timer, timestamp)
            } else {
                panic!("set_alarm called before esp_hal_embassy::init()")
            }
        })
    }
}

#[cold]
#[track_caller]
fn not_enough_timers() -> ! {
    // This is wrapped in a separate function because rustfmt does not like
    // extremely long strings. Also, if log is used, this avoids storing the string
    // twice.
    panic!("There are not enough timers to allocate a new alarm. Call esp_hal_embassy::init() with the correct number of timers, or consider using one of the embassy-timer/generic-queue-X features.");
}
