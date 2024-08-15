//! Embassy support for [esp-hal].
//!
//! [Embassy] is a modern asynchronous framework intended for use with embedded
//! systems. This package provides support for building applications using
//! Embassy with [esp-hal].
//!
//! [esp-hal]: https://github.com/esp-rs/esp-hal
//! [embassy]: https://github.com/embassy-rs/embassy
//!
//! ## Executors
//!
//! Two types of executors are provided:
//!
//! - [Executor]: A thread-mode executor
//! - [InterruptExecutor]: An interrupt-mode executor
//!
//! [InterruptExecutor] can be used to achieve preemptive multitasking in
//! asynchronous applications, which is typically something reserved for more
//! traditional RTOS. More information can be found in the [Embassy
//! documentation].
//!
//! [embassy documentation]: https://embassy.dev/book/
//!
//! ## Initialization
//!
//! Embassy **must** be initialized by calling the [init] function. This
//! initialization must be performed *prior* to spawning any tasks.
//!
//! ## Feature Flags
#![doc = document_features::document_features!()]
#![doc(html_logo_url = "https://avatars.githubusercontent.com/u/46717278")]
#![deny(missing_docs)]
#![cfg_attr(xtensa, feature(asm_experimental_arch))]
#![no_std]

// MUST be the first module
mod fmt;

#[cfg(not(feature = "esp32"))]
use esp_hal::timer::systimer::Alarm;
#[cfg(timg_timer1)]
use esp_hal::timer::timg::Timer1;
use esp_hal::{
    clock::Clocks,
    timer::{
        timg::{Timer as Timg, Timer0, TimerGroupInstance},
        ErasedTimer,
    },
};
pub use macros::main;

#[cfg(feature = "executors")]
pub use self::executor::{Executor, InterruptExecutor};
use self::time_driver::{EmbassyTimer, Timer};

#[cfg(feature = "executors")]
mod executor;
mod time_driver;

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

/// A trait to allow better UX for initializing the timers.
pub trait TimerCollection {
    /// Returns the timers as a slice.
    fn timers(self) -> &'static mut [Timer];
}

impl TimerCollection for Timer {
    fn timers(self) -> &'static mut [Timer] {
        let timers = mk_static!([Timer; 1], [self]);
        timers.timers()
    }
}

impl<T, DM> TimerCollection for Timg<Timer0<T>, DM>
where
    DM: esp_hal::Mode,
    T: TimerGroupInstance,
    ErasedTimer: From<Timg<Timer0<T>, DM>>,
{
    fn timers(self) -> &'static mut [Timer] {
        Timer::new(ErasedTimer::from(self)).timers()
    }
}

#[cfg(timg_timer1)]
impl<T, DM> TimerCollection for Timg<Timer1<T>, DM>
where
    DM: esp_hal::Mode,
    T: TimerGroupInstance,
    ErasedTimer: From<Timg<Timer1<T>, DM>>,
{
    fn timers(self) -> &'static mut [Timer] {
        Timer::new(ErasedTimer::from(self)).timers()
    }
}

#[cfg(not(feature = "esp32"))]
impl<T, DM, const N: u8> TimerCollection for Alarm<T, DM, N>
where
    DM: esp_hal::Mode,
    ErasedTimer: From<Alarm<T, DM, N>>,
{
    fn timers(self) -> &'static mut [Timer] {
        Timer::new(ErasedTimer::from(self)).timers()
    }
}

impl TimerCollection for &'static mut [Timer] {
    fn timers(self) -> &'static mut [Timer] {
        self
    }
}

impl<const N: usize> TimerCollection for &'static mut [Timer; N] {
    fn timers(self) -> &'static mut [Timer] {
        self.as_mut()
    }
}

/// Initialize embassy.
///
/// Call this as soon as possible, before the first timer-related operation.
///
/// The time driver can be one of a number of different options:
///
/// - A single timer unit, such as a TIMG timer or a SYSTIMER alarm.
/// - A slice (or array reference) of multiple `[OneShotTimer<ErasedTimer>]`
///   objects.
pub fn init(clocks: &Clocks, time_driver: impl TimerCollection) {
    EmbassyTimer::init(clocks, time_driver.timers())
}
