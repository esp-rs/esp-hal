//! # Embassy driver
//!
//! ## Overview
//! The `embassy` driver for ESP chips is an essential part of the Embassy
//! embedded async/await runtime and is used by applications to perform
//! time-based operations and schedule asynchronous tasks. It provides a
//! high-level API for handling timers and alarms, abstracting the underlying
//! hardware details, and allowing users to focus on application logic rather
//! than low-level timer management.
//!
//! Here are important details about the module:
//!   * `time_driver` module (`time_driver_systimer` or `time_driver_timg`,
//!     depends on enabled feature)
//!     - This module contains the implementations of the timer drivers for
//!       different ESP chips.<br> It includes the `EmbassyTimer` struct, which
//!       is responsible for handling alarms and timer events.
//!     - `EmbassyTimer` struct represents timer driver for ESP chips. It
//!       contains `alarms` - an array of `AlarmState` structs, which describe
//!       the state of alarms associated with the timer driver.
//!   * `AlarmState` struct
//!     - This struct represents the state of an alarm. It contains information
//!       about the alarm's timestamp, a callback function to be executed when
//!       the alarm triggers, and a context pointer for passing user-defined
//!       data to the callback.
//!
//! ## Example
//! The following example demonstrates how to use the `embassy` driver to
//! schedule asynchronous tasks.<br> In this example, we use the `embassy`
//! driver to wait for a GPIO 9 pin state to change. ```no_run
//! #[cfg(feature = "embassy-time-systick")]
//! embassy::init(
//!     &clocks,
//!     esp32c6_hal::systimer::SystemTimer::new(peripherals.SYSTIMER),
//! );
//!
//! #[cfg(feature = "embassy-time-timg0")]
//! embassy::init(&clocks, timer_group0.timer0);
//!
//! let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
//! // GPIO 9 as input
//! let input = io.pins.gpio9.into_pull_down_input();
//!
//! // Async requires the GPIO interrupt to wake futures
//! esp32c6_hal::interrupt::enable(
//!     esp32c6_hal::peripherals::Interrupt::GPIO,
//!     esp32c6_hal::interrupt::Priority::Priority1,
//! )
//! .unwrap();
//!
//! let executor = EXECUTOR.init(Executor::new());
//! executor.run(|spawner| {
//!     spawner.spawn(ping(input)).ok();
//! });
//! ```
//! 
//! Where `ping` defined as:
//! ```no_run
//! async fn ping(mut pin: Gpio9<Input<PullDown>>) {
//!     loop {
//!         esp_println::println!("Waiting...");
//!         pin.wait_for_rising_edge().await.unwrap();
//!         esp_println::println!("Ping!");
//!         Timer::after(Duration::from_millis(100)).await;
//!     }
//! }
//! ```
//! For more embassy-related examples check out the [examples repo](https://github.com/esp-rs/esp-hal/tree/main/esp32-hal/examples)
//! for a corresponding board.

use core::cell::Cell;

use embassy_time::driver::{AlarmHandle, Driver};

#[cfg_attr(
    all(systimer, feature = "embassy-time-systick",),
    path = "time_driver_systimer.rs"
)]
#[cfg_attr(
    all(timg0, feature = "embassy-time-timg0"),
    path = "time_driver_timg.rs"
)]
mod time_driver;

use time_driver::EmbassyTimer;

use crate::clock::Clocks;

pub fn init(clocks: &Clocks, td: time_driver::TimerType) {
    EmbassyTimer::init(clocks, td)
}

pub struct AlarmState {
    pub timestamp: Cell<u64>,
    pub callback: Cell<Option<(fn(*mut ()), *mut ())>>,
    pub allocated: Cell<bool>,
}

unsafe impl Send for AlarmState {}

impl AlarmState {
    pub const fn new() -> Self {
        Self {
            timestamp: Cell::new(0),
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
        return critical_section::with(|cs| {
            let alarms = self.alarms.borrow(cs);
            for i in 0..time_driver::ALARM_COUNT {
                let c = alarms.get_unchecked(i);
                if !c.allocated.get() {
                    // set alarm so it is not overwritten
                    c.allocated.set(true);
                    return Option::Some(AlarmHandle::new(i as u8));
                }
            }
            return Option::None;
        });
    }

    fn set_alarm_callback(
        &self,
        alarm: embassy_time::driver::AlarmHandle,
        callback: fn(*mut ()),
        ctx: *mut (),
    ) {
        let n = alarm.id() as usize;
        critical_section::with(|cs| {
            let alarm = &self.alarms.borrow(cs)[n];
            alarm.callback.set(Some((callback, ctx)));
        })
    }

    fn set_alarm(&self, alarm: embassy_time::driver::AlarmHandle, timestamp: u64) -> bool {
        self.set_alarm(alarm, timestamp)
    }
}
