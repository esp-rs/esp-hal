//! # Embassy
//!
//! The [embassy](https://github.com/embassy-rs/embassy) project is a toolkit to leverage async Rust
//! in embedded applications. This module adds the required
//! support to use embassy on Espressif chips.
//!
//! ## Initialization
//!
//! Embassy **must** be initialized by calling [`init`], before beginning any
//! async operations.
//!
//! [`init`] installs a [global time driver](https://github.com/embassy-rs/embassy/tree/main/embassy-time#global-time-driver)
//! allowing users to use [embassy-time](https://docs.rs/embassy-time/latest/embassy_time/) APIs in any async context
//! within their application. A time driver must be chosen by enabling the
//! correct feature on esp-hal, see the crate level documentation for more
//! details.
//!
//! ## Executors
//!
//! We offer two executor types, a thread mode [`Executor`](executor::Executor)
//! and [`InterruptExecutor`](executor::InterruptExecutor).
//! An [`InterruptExecutor`](executor::InterruptExecutor) can be used to achieve
//! preemptive multitasking in async applications, which is usually something reserved for more traditional RTOS systems, read more about it in [the embassy documentation](https://embassy.dev/book/dev/runtime.html).

pub mod executor;

use core::cell::Cell;

use embassy_time_driver::{AlarmHandle, Driver};

#[cfg_attr(
    all(
        systimer,
        any(
            feature = "embassy-time-systick-16mhz",
            feature = "embassy-time-systick-80mhz"
        )
    ),
    path = "time_driver_systimer.rs"
)]
#[cfg_attr(
    all(timg0, feature = "embassy-time-timg0"),
    path = "time_driver_timg.rs"
)]
mod time_driver;

use time_driver::EmbassyTimer;

use crate::clock::Clocks;

/// Initialize embassy
pub fn init(clocks: &Clocks, time_driver: time_driver::TimerType) {
    EmbassyTimer::init(clocks, time_driver)
}

pub(crate) struct AlarmState {
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

impl Driver for EmbassyTimer {
    fn now(&self) -> u64 {
        EmbassyTimer::now()
    }

    unsafe fn allocate_alarm(&self) -> Option<AlarmHandle> {
        critical_section::with(|cs| {
            for (i, alarm) in self.alarms.borrow(cs).iter().enumerate() {
                if !alarm.allocated.get() {
                    // set alarm so it is not overwritten
                    alarm.allocated.set(true);
                    self.on_alarm_allocated(i);
                    return Some(AlarmHandle::new(i as u8));
                }
            }
            None
        })
    }

    fn set_alarm_callback(
        &self,
        alarm: embassy_time_driver::AlarmHandle,
        callback: fn(*mut ()),
        ctx: *mut (),
    ) {
        let n = alarm.id() as usize;
        critical_section::with(|cs| {
            let alarm = &self.alarms.borrow(cs)[n];
            alarm.callback.set(Some((callback, ctx)));
        })
    }

    fn set_alarm(&self, alarm: embassy_time_driver::AlarmHandle, timestamp: u64) -> bool {
        self.set_alarm(alarm, timestamp)
    }
}
