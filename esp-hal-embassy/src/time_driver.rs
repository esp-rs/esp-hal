use core::cell::Cell;

use embassy_time_driver::Driver;
use esp_hal::{
    interrupt::{InterruptHandler, Priority},
    sync::Locked,
    time::{now, ExtU64},
    timer::OneShotTimer,
    Blocking,
};

pub type Timer = OneShotTimer<'static, Blocking>;

/// Alarm handle, assigned by the driver.
#[derive(Clone, Copy)]
pub(crate) struct AlarmHandle {
    id: usize,
}

impl AlarmHandle {
    /// Create an AlarmHandle
    ///
    /// Safety: May only be called by the current global Driver impl.
    /// The impl is allowed to rely on the fact that all `AlarmHandle` instances
    /// are created by itself in unsafe code (e.g. indexing operations)
    pub unsafe fn new(id: usize) -> Self {
        Self { id }
    }

    pub fn update(&self, expiration: u64) -> bool {
        if expiration == u64::MAX {
            true
        } else {
            DRIVER.set_alarm(*self, expiration)
        }
    }
}

enum AlarmState {
    Created(extern "C" fn()),
    Initialized(&'static mut Timer),
}
impl AlarmState {
    fn initialize(timer: &'static mut Timer, interrupt_handler: InterruptHandler) -> AlarmState {
        // If the driver is initialized, bind the interrupt handler to the
        // timer. This ensures that alarms allocated after init are correctly
        // bound to the core that created the executor.
        timer.set_interrupt_handler(interrupt_handler);
        timer.enable_interrupt(true);
        AlarmState::Initialized(timer)
    }
}

struct AlarmInner {
    pub callback: Cell<*const ()>,

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
                callback: Cell::new(core::ptr::null_mut()),
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
    pub(super) fn init(timers: &'static mut [Timer]) {
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

        // Store the available timers
        DRIVER
            .available_timers
            .with(|available_timers| *available_timers = Some(timers));
    }

    #[cfg(not(feature = "single-queue"))]
    pub(crate) fn set_callback_ctx(&self, alarm: AlarmHandle, ctx: *const ()) {
        self.alarms[alarm.id].inner.with(|alarm| {
            alarm.callback.set(ctx.cast_mut());
        })
    }

    fn on_interrupt(&self, id: usize) {
        let ctx = self.alarms[id].inner.with(|alarm| {
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

        TIMER_QUEUE_DRIVER.handle_alarm(ctx);
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

    pub(crate) unsafe fn allocate_alarm(&self, priority: Priority) -> Option<AlarmHandle> {
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
                    Some(timer) => AlarmState::initialize(
                        timer,
                        InterruptHandler::new(interrupt_handler, priority),
                    ),

                    None => panic!(),
                };

                Some(AlarmHandle::new(i))
            });

            if handle.is_some() {
                return handle;
            }
        }

        None
    }

    pub(crate) fn set_alarm(&self, alarm: AlarmHandle, timestamp: u64) -> bool {
        let alarm = &self.alarms[alarm.id];

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

impl Driver for EmbassyTimer {
    fn now(&self) -> u64 {
        now().ticks()
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

pub(crate) struct TimerQueueDriver {
    #[cfg(any(feature = "single-queue", not(feature = "integrated-timers")))]
    pub(crate) inner: crate::timer_queue::TimerQueue,
}

impl TimerQueueDriver {
    const fn new() -> Self {
        Self {
            #[cfg(any(feature = "single-queue", not(feature = "integrated-timers")))]
            inner: crate::timer_queue::TimerQueue::new(Priority::max()),
        }
    }

    fn handle_alarm(&self, _ctx: *const ()) {
        #[cfg(not(feature = "single-queue"))]
        {
            let executor = unsafe { &*_ctx.cast::<crate::executor::InnerExecutor>() };
            executor.timer_queue.dispatch();
        }

        #[cfg(feature = "single-queue")]
        self.inner.dispatch();
    }
}

pub(crate) fn set_up_alarm(priority: Priority, _ctx: *mut ()) -> AlarmHandle {
    let alarm = unsafe {
        DRIVER
            .allocate_alarm(priority)
            .unwrap_or_else(|| not_enough_timers())
    };
    #[cfg(not(feature = "single-queue"))]
    DRIVER.set_callback_ctx(alarm, _ctx);
    alarm
}

embassy_time_queue_driver::timer_queue_impl!(static TIMER_QUEUE_DRIVER: TimerQueueDriver = TimerQueueDriver::new());
