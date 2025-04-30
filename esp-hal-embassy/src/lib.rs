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

/// A timer or collection on timers that can be passed to [`init`].
pub trait TimeBase: private::Sealed {
    #[doc(hidden)]
    fn timers(self, _: private::Internal) -> &'static mut [Timer];
}

macro_rules! impl_timebase {
    ($timebase:path) => {
        impl private::Sealed for $timebase {}

        impl TimeBase for $timebase {
            fn timers(self, _: private::Internal) -> &'static mut [Timer] {
                mk_static!([Timer; 1], [Timer::new(self)])
            }
        }
    };
}

macro_rules! impl_timebase_array {
    ($timebase:path, $n:literal) => {
        impl private::Sealed for [$timebase; $n] {}

        impl TimeBase for [$timebase; $n] {
            fn timers(self, _: private::Internal) -> &'static mut [Timer] {
                mk_static!([Timer; $n], self.map(|t| Timer::new(t)))
            }
        }
    };
}

macro_rules! impl_array {
    ($n:literal) => {
        impl private::Sealed for [Timer; $n] {}

        impl TimeBase for [Timer; $n] {
            fn timers(self, _: private::Internal) -> &'static mut [Timer] {
                mk_static!([Timer; $n], self)
            }
        }

        impl_timebase_array!(AnyTimer<'static>, $n);
        impl_timebase_array!(TimgTimer<'static>, $n);
        #[cfg(systimer)]
        impl_timebase_array!(esp_hal::timer::systimer::Alarm<'static>, $n);
    };
}

impl private::Sealed for Timer {}
impl TimeBase for Timer {
    fn timers(self, _: private::Internal) -> &'static mut [Timer] {
        mk_static!([Timer; 1], [self])
    }
}

impl_timebase!(AnyTimer<'static>);
impl_timebase!(TimgTimer<'static>);
#[cfg(systimer)]
impl_timebase!(esp_hal::timer::systimer::Alarm<'static>);

impl private::Sealed for &'static mut [Timer] {}
impl TimeBase for &'static mut [Timer] {
    fn timers(self, _: private::Internal) -> &'static mut [Timer] {
        self
    }
}

impl<const N: usize> private::Sealed for &'static mut [Timer; N] {}
impl<const N: usize> TimeBase for &'static mut [Timer; N] {
    fn timers(self, _: private::Internal) -> &'static mut [Timer] {
        self.as_mut()
    }
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
pub fn init(time_driver: impl TimeBase) {
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
