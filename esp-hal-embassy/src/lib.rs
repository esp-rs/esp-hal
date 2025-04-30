//! Embassy support for [esp-hal].
//!
//! [Embassy] is a modern asynchronous framework intended for use with embedded
//! systems. This package provides support for building applications using
//! Embassy with [esp-hal].
//!
//! Note that this crate currently requires you to enable the `unstable` feature
//! on `esp-hal`.
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
//! Initialization requires a number of timers to be passed in. The number of
//! timers required depends on the timer queue flavour used, as well as the
//! number of executors started. If you use the `multiple-integrated` timer
//! queue flavour, then you need to pass as many timers as you start executors.
//! In other cases, you can pass a single timer.
//!
//! ## Configuration
//!
//! You can configure the behaviour of the embassy runtime by using the
//! following environment variables:
#![doc = ""]
#![doc = include_str!(concat!(env!("OUT_DIR"), "/esp_hal_embassy_config_table.md"))]
#![doc = ""]
//! ## Feature Flags
#![doc = document_features::document_features!()]
#![doc(html_logo_url = "https://avatars.githubusercontent.com/u/46717278")]
#![deny(missing_docs, rust_2018_idioms, rustdoc::all)]
#![cfg_attr(xtensa, feature(asm_experimental_arch))]
#![no_std]

// MUST be the first module
mod fmt;

use esp_hal::timer::{AnyTimer, timg::Timer as TimgTimer};
pub use macros::embassy_main as main;

#[cfg(feature = "executors")]
pub use self::executor::{Executor, InterruptExecutor};
use self::time_driver::{EmbassyTimer, Timer};

#[cfg(feature = "executors")]
pub(crate) mod executor;
mod time_driver;
mod timer_queue;

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

mod private {
    pub trait Sealed {}

    #[derive(Clone, Copy)]
    pub struct Internal;
}

/// A collection of timers that can be passed to [`init`].
pub trait TimeBaseCollection: private::Sealed {
    #[doc(hidden)]
    fn timers(self, _: private::Internal) -> &'static mut [Timer];
}

/// A timer that can be passed to [`init`].
pub trait TimeBase: private::Sealed {
    #[doc(hidden)]
    fn into_timer(self, _: private::Internal) -> Timer;
}

// We can't blanket-implement TimeBase for `Into<AnyTimer>` because of possible
// conflicting implementations.

impl private::Sealed for Timer {}
impl private::Sealed for &'static mut [Timer] {}
impl<const N: usize> private::Sealed for &'static mut [Timer; N] {}

impl private::Sealed for AnyTimer<'static> {}
impl private::Sealed for TimgTimer<'static> {}
#[cfg(systimer)]
impl private::Sealed for esp_hal::timer::systimer::Alarm<'static> {}

impl TimeBase for Timer {
    fn into_timer(self, _: private::Internal) -> Timer {
        self
    }
}
impl TimeBase for AnyTimer<'static> {
    fn into_timer(self, _: private::Internal) -> Timer {
        Timer::new(self)
    }
}
impl TimeBase for TimgTimer<'static> {
    fn into_timer(self, _: private::Internal) -> Timer {
        Timer::new(self)
    }
}
#[cfg(systimer)]
impl TimeBase for esp_hal::timer::systimer::Alarm<'static> {
    fn into_timer(self, _: private::Internal) -> Timer {
        Timer::new(self)
    }
}

// Single timers

impl<T> TimeBaseCollection for T
where
    T: TimeBase,
{
    fn timers(self, private: private::Internal) -> &'static mut [Timer] {
        let timers = mk_static!([Timer; 1], [self.into_timer(private)]);
        timers.timers(private)
    }
}

impl TimeBaseCollection for &'static mut [Timer] {
    fn timers(self, _: private::Internal) -> &'static mut [Timer] {
        self
    }
}

impl<const N: usize> TimeBaseCollection for &'static mut [Timer; N] {
    fn timers(self, _: private::Internal) -> &'static mut [Timer] {
        self.as_mut()
    }
}

macro_rules! impl_array {
    ($n:literal) => {
        impl<T> private::Sealed for [T; $n] where T: private::Sealed {}

        impl<T> TimeBaseCollection for [T; $n]
        where
            T: TimeBase,
        {
            fn timers(self, private: private::Internal) -> &'static mut [Timer] {
                mk_static!([Timer; $n], self.map(|t| t.into_timer(private)))
            }
        }
    };
}

impl_array!(1);
impl_array!(2);
impl_array!(3);
impl_array!(4);

/// Initialize embassy.
///
/// Call this as soon as possible, before the first timer-related operation.
///
/// The time driver can be one of a number of different options:
///
/// - A single object of, or a 1-4 element array of the following:
///   - `esp_hal::timer::timg::Timer`
#[cfg_attr(systimer, doc = "  - `esp_hal::timer::systimer::Alarm`")]
///   - `esp_hal::timer::AnyTimer`
///   - `esp_hal::timer::OneShotTimer`
/// - A mutable static slice of `OneShotTimer` instances
/// - A mutable static array of `OneShotTimer` instances
///
/// Note that if you use the `multiple-integrated` timer-queue flavour, then
/// you need to pass as many timers as you start executors. In other cases,
/// you can pass a single timer.
///
/// # Examples
///
/// ```rust, no_run
#[doc = esp_hal::before_snippet!()]
/// use esp_hal::timer::timg::TimerGroup;
///
/// let timg0 = TimerGroup::new(peripherals.TIMG0);
/// esp_hal_embassy::init(timg0.timer0);
///
/// // ... now you can spawn embassy tasks or use `Timer::after` etc.
/// # }
/// ```
pub fn init(time_driver: impl TimeBaseCollection) {
    #[cfg(all(feature = "executors", multi_core, low_power_wait))]
    unsafe {
        use esp_hal::interrupt::software::SoftwareInterrupt;

        #[esp_hal::ram]
        extern "C" fn software3_interrupt() {
            // This interrupt is fired when the thread-mode executor's core needs to be
            // woken. It doesn't matter which core handles this interrupt first, the
            // point is just to wake up the core that is currently executing
            // `waiti`.
            unsafe { SoftwareInterrupt::<3>::steal().reset() };
        }

        esp_hal::interrupt::bind_interrupt(
            esp_hal::peripherals::Interrupt::FROM_CPU_INTR3,
            software3_interrupt,
        );
    }

    EmbassyTimer::init(time_driver.timers(private::Internal))
}
