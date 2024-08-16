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
use esp_hal::{
    clock::Clocks,
    timer::{timg::Timer as TimgTimer, ErasedTimer},
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
///
/// This trait is meant to be used only for the `init` function.
/// Calling `timers()` multiple times may panic.
pub trait TimerCollection {
    /// Returns the timers as a slice.
    fn timers(self) -> &'static mut [Timer];
}

/// Helper trait to reduce boilerplate.
///
/// We can't blanket-implement for `Into<ErasedTimer>` because of possible
/// conflicting implementations.
trait IntoErasedTimer: Into<ErasedTimer> {}

impl IntoErasedTimer for ErasedTimer {}

impl<T, DM> IntoErasedTimer for TimgTimer<T, DM>
where
    DM: esp_hal::Mode,
    Self: Into<ErasedTimer>,
{
}

#[cfg(not(feature = "esp32"))]
impl<T, DM, const N: u8> IntoErasedTimer for Alarm<T, DM, N>
where
    DM: esp_hal::Mode,
    Self: Into<ErasedTimer>,
{
}

impl<T> TimerCollection for T
where
    T: IntoErasedTimer,
{
    fn timers(self) -> &'static mut [Timer] {
        Timer::new(self.into()).timers()
    }
}

impl TimerCollection for Timer {
    fn timers(self) -> &'static mut [Timer] {
        let timers = mk_static!([Timer; 1], [self]);
        timers.timers()
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
/// - A timg `Timer` instance
/// - A systimer `Alarm` instance
/// - An `ErasedTimer` instance
/// - A `OneShotTimer` instance
/// - A mutable static slice of `OneShotTimer` instances
/// - A mutable static array of `OneShotTimer` instances
///
/// # Examples
///
/// ```rust, no_run
#[doc = esp_hal::before_snippet!()]
/// use esp_hal::timg::TimerGroup;
///
/// let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks);
/// esp_hal_embassy::init(&clocks, timg0.timer0);
///
/// // ... now you can spawn embassy tasks or use `Timer::after` etc.
/// # }
/// ```
pub fn init(clocks: &Clocks, time_driver: impl TimerCollection) {
    EmbassyTimer::init(clocks, time_driver.timers())
}
