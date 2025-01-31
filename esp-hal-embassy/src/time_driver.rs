//! Embassy time driver implementation
//!
//! The time driver is responsible for keeping time, as well as to manage the
//! wait queue for embassy-time.

#[cfg(not(single_queue))]
use core::cell::Cell;

use embassy_time_driver::Driver;
use esp_hal::{
    interrupt::{InterruptHandler, Priority},
    sync::Locked,
    time::{now, Duration},
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
    /// If multiple queues are used, we store the appropriate timer queue here.
    // FIXME: we currently store the executor, but we could probably avoid an addition by actually
    // storing a reference to the timer queue.
    #[cfg(not(single_queue))]
    pub context: Cell<*const ()>,

    pub state: AlarmState,
}

struct Alarm {
    // FIXME: we should be able to use priority-limited locks here, but we can initialize alarms
    // while running at an arbitrary priority level. We need to rework alarm allocation to only use
    // a critical section to allocate an alarm, but not when using it.
    pub inner: Locked<AlarmInner>,
}

unsafe impl Send for Alarm {}

impl Alarm {
    pub const fn new(handler: extern "C" fn()) -> Self {
        Self {
            inner: Locked::new(AlarmInner {
                #[cfg(not(single_queue))]
                context: Cell::new(core::ptr::null_mut()),
                state: AlarmState::Created(handler),
            }),
        }
    }
}

/// embassy requires us to implement the [embassy_time_driver::Driver] trait,
/// which we do here. This trait needs us to be able to tell the current time,
/// as well as to schedule a wake-up at a certain time.
///
/// We are free to choose how we implement these features, and we provide three
/// options:
///
/// - If the `generic` feature is enabled, we implement a single timer queue,
///   using the implementation provided by embassy-time-queue-driver.
/// - If the `single-integrated` feature is enabled, we implement a single timer
///   queue, using our own integrated timer implementation. Our implementation
///   is a copy of the embassy integrated timer queue, with the addition of
///   clearing the "owner" information upon dequeueing.
/// - If the `multiple-integrated` feature is enabled, we provide a separate
///   timer queue for each executor. We store a separate timer queue for each
///   executor, and we use the scheduled task's owner to determine which queue
///   to use. This mode allows us to use less disruptive locks around the timer
///   queue, but requires more timers - one per timer queue.
pub(super) struct EmbassyTimer {
    /// The timer queue, if we use a single one (single-integrated, or generic).
    #[cfg(single_queue)]
    pub(crate) inner: crate::timer_queue::TimerQueue,

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

// TODO: we can reduce this to 1 for single_queue, but that would break current
// tests. Resolve when tests can use separate configuration sets, or update
// tests to always pass a single timer.
const MAX_SUPPORTED_ALARM_COUNT: usize = 7;

embassy_time_driver::time_driver_impl!(static DRIVER: EmbassyTimer = EmbassyTimer {
    // Single queue, needs maximum priority.
    #[cfg(single_queue)]
    inner: crate::timer_queue::TimerQueue::new(Priority::max()),
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

    #[cfg(not(single_queue))]
    pub(crate) fn set_callback_ctx(&self, alarm: AlarmHandle, ctx: *const ()) {
        self.alarms[alarm.id].inner.with(|alarm| {
            alarm.context.set(ctx.cast_mut());
        })
    }

    fn on_interrupt(&self, id: usize) {
        // On interrupt, we clear the alarm that was triggered...
        #[cfg_attr(single_queue, allow(clippy::let_unit_value))]
        let _ctx = self.alarms[id].inner.with(|alarm| {
            if let AlarmState::Initialized(timer) = &mut alarm.state {
                timer.clear_interrupt();
                #[cfg(not(single_queue))]
                alarm.context.get()
            } else {
                unsafe {
                    // SAFETY: `on_interrupt` is registered right when the alarm is initialized.
                    core::hint::unreachable_unchecked()
                }
            }
        });

        // ... and process the timer queue if we have one. For multiple queues, the
        // timer queue is stored in the alarm's context.
        #[cfg(all(integrated_timers, not(single_queue)))]
        {
            let executor = unsafe { &*_ctx.cast::<crate::executor::InnerExecutor>() };
            executor.timer_queue.dispatch();
        }

        // If we have a single queue, it lives in this struct.
        #[cfg(single_queue)]
        self.inner.dispatch();
    }

    /// Returns `true` if the timer was armed, `false` if the timestamp is in
    /// the past.
    fn arm(timer: &mut Timer, timestamp: u64) -> bool {
        let now = now().duration_since_epoch().as_micros();

        if timestamp > now {
            let timeout = Duration::from_micros(timestamp - now);
            unwrap!(timer.schedule(timeout));
            true
        } else {
            // If the timestamp is past, we return `false` to ask embassy to poll again
            // immediately.
            timer.stop();
            false
        }
    }

    /// Allocate an alarm, if possible.
    ///
    /// Returns `None` if there are no available alarms.
    ///
    /// When using multiple timer queues, the `priority` parameter indicates the
    /// priority of the interrupt handler. It is 1 for thread-mode
    /// executors, or equals to the priority of an interrupt executor.
    ///
    /// When using a single timer queue, the `priority` parameter is always the
    /// highest value possible.
    pub(crate) unsafe fn allocate_alarm(&self, priority: Priority) -> Option<AlarmHandle> {
        for (i, alarm) in self.alarms.iter().enumerate() {
            let handle = alarm.inner.with(|alarm| {
                let AlarmState::Created(interrupt_handler) = alarm.state else {
                    return None;
                };

                let timer = self.available_timers.with(|available_timers| {
                    if let Some(timers) = available_timers.take() {
                        // If the driver is initialized, we can allocate a timer.
                        // If this fails, we can't do anything about it.
                        let Some((timer, rest)) = timers.split_first_mut() else {
                            not_enough_timers();
                        };
                        *available_timers = Some(rest);
                        timer
                    } else {
                        panic!("schedule_wake called before esp_hal_embassy::init()")
                    }
                });

                alarm.state = AlarmState::initialize(
                    timer,
                    InterruptHandler::new(interrupt_handler, priority),
                );

                Some(AlarmHandle::new(i))
            });

            if handle.is_some() {
                return handle;
            }
        }

        None
    }

    /// Set an alarm to fire at a certain timestamp.
    ///
    /// Returns `false` if the timestamp is in the past.
    fn set_alarm(&self, alarm: AlarmHandle, timestamp: u64) -> bool {
        let alarm = &self.alarms[alarm.id];

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
                unsafe {
                    // SAFETY: We only create `AlarmHandle` instances after the alarm is
                    // initialized.
                    core::hint::unreachable_unchecked()
                }
            }
        })
    }
}

impl Driver for EmbassyTimer {
    fn now(&self) -> u64 {
        now().duration_since_epoch().as_micros()
    }

    fn schedule_wake(&self, at: u64, waker: &core::task::Waker) {
        #[cfg(not(single_queue))]
        unsafe {
            // If we have multiple queues, we have integrated timers and our own timer queue
            // implementation.
            use embassy_executor::raw::Executor as RawExecutor;
            use portable_atomic::{AtomicPtr, Ordering};

            let task = embassy_executor::raw::task_from_waker(waker);

            // SAFETY: it is impossible to schedule a task that has not yet been spawned,
            // so the executor is guaranteed to be set to a non-null value.
            let mut executor = task.executor().unwrap_unchecked() as *const RawExecutor;

            let owner = task
                .timer_queue_item()
                .payload
                .as_ref::<AtomicPtr<RawExecutor>>();

            // Try to take ownership over the timer item.
            let owner = owner.compare_exchange(
                core::ptr::null_mut(),
                executor.cast_mut(),
                Ordering::AcqRel,
                Ordering::Acquire,
            );

            // We can't take ownership, but we may still be able to enqueue the task. Point
            // at the current owner.
            if let Err(owner) = owner {
                executor = owner;
            };

            // It is possible that the task's owner changes in the mean time. It doesn't
            // matter, at this point the only interesting question is: can we enqueue in the
            // currently loaded owner's timer queue?

            // Try to enqueue in the current owner's timer queue. This will fail if the
            // owner has a lower priority ceiling than the current context.

            // SAFETY: we've exposed provenance in `InnerExecutor::init`, which is called
            // when the executor is started. Because the executor wasn't running
            // before init, it is impossible to get a pointer here that has no
            // provenance exposed.
            // The cast is then safe, because the RawExecutor is the first field of the
            // InnerExecutor, and repr(C) guarantees that the fields are laid out in the
            // order they are defined, and the first field has 0 offset.
            let executor_addr = executor as usize;
            let executor = core::ptr::with_exposed_provenance_mut::<crate::executor::InnerExecutor>(
                executor_addr,
            );
            (*executor).timer_queue.schedule_wake(at, waker);
        }

        #[cfg(single_queue)]
        self.inner.schedule_wake(at, waker);
    }
}

#[cold]
#[track_caller]
fn not_enough_timers() -> ! {
    // This is wrapped in a separate function because rustfmt does not like
    // extremely long strings. Also, if log is used, this avoids storing the string
    // twice.
    panic!("There are not enough timers to allocate a new alarm. Call esp_hal_embassy::init() with the correct number of timers, or consider either using the `single-integrated` or the `generic` timer queue flavors.");
}

pub(crate) fn set_up_alarm(priority: Priority, _ctx: *mut ()) -> AlarmHandle {
    let alarm = unsafe {
        DRIVER
            .allocate_alarm(priority)
            .unwrap_or_else(|| not_enough_timers())
    };
    #[cfg(not(single_queue))]
    DRIVER.set_callback_ctx(alarm, _ctx);
    alarm
}
