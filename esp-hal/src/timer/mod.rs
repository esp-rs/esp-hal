//! # General-purpose Timers
//!
//! ## Overview
//! The [OneShotTimer] and [PeriodicTimer] types can be backed by any hardware
//! peripheral which implements the [Timer] trait. This means that the same API
//! can be used to interact with different hardware timers, like the `TIMG` and
//! SYSTIMER.
#![cfg_attr(
    not(feature = "esp32"),
    doc = "See the [timg] and [systimer] modules for more information."
)]
#![cfg_attr(feature = "esp32", doc = "See the [timg] module for more information.")]
//! ## Examples
//!
//! ### One-shot Timer
//!
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! # use esp_hal::timer::{OneShotTimer, PeriodicTimer, timg::TimerGroup};
//! #
//! let timg0 = TimerGroup::new(peripherals.TIMG0);
//! let one_shot = OneShotTimer::new(timg0.timer0);
//!
//! one_shot.delay_millis(500);
//! # }
//! ```
//! 
//! ### Periodic Timer
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! # use esp_hal::timer::{PeriodicTimer, timg::TimerGroup};
//! #
//! let timg0 = TimerGroup::new(peripherals.TIMG0);
//! let mut periodic = PeriodicTimer::new(timg0.timer0);
//!
//! periodic.start(1.secs());
//! loop {
//!     nb::block!(periodic.wait());
//! }
//! # }
//! ```

use fugit::{ExtU64, Instant, MicrosDurationU64};

use crate::{
    interrupt::{InterruptConfigurable, InterruptHandler},
    peripheral::{Peripheral, PeripheralRef},
};

#[cfg(systimer)]
pub mod systimer;
#[cfg(any(timg0, timg1))]
pub mod timg;

/// Timer errors.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// The timer is already active.
    TimerActive,
    /// The timer is not currently active.
    TimerInactive,
    /// The alarm is not currently active.
    AlarmInactive,
    /// The provided timeout is too large.
    InvalidTimeout,
}

/// Functionality provided by any timer peripheral.
pub trait Timer: crate::private::Sealed {
    /// Start the timer.
    fn start(&self);

    /// Stop the timer.
    fn stop(&self);

    /// Reset the timer value to 0.
    fn reset(&self);

    /// Is the timer running?
    fn is_running(&self) -> bool;

    /// The current timer value.
    fn now(&self) -> Instant<u64, 1, 1_000_000>;

    /// Load a target value into the timer.
    fn load_value(&self, value: MicrosDurationU64) -> Result<(), Error>;

    /// Enable auto reload of the loaded value.
    fn enable_auto_reload(&self, auto_reload: bool);

    /// Enable or disable the timer's interrupt.
    fn enable_interrupt(&self, state: bool);

    /// Clear the timer's interrupt.
    fn clear_interrupt(&self);

    /// Set the interrupt handler
    ///
    /// Note that this will replace any previously set interrupt handler
    fn set_interrupt_handler(&self, handler: InterruptHandler);

    /// Has the timer triggered?
    fn is_interrupt_set(&self) -> bool;

    // NOTE: This is an unfortunate implementation detail of `TIMGx`
    #[doc(hidden)]
    fn set_alarm_active(&self, state: bool);
}

/// A one-shot timer.
pub struct OneShotTimer<'d, T> {
    inner: PeripheralRef<'d, T>,
}

impl<'d, T> OneShotTimer<'d, T>
where
    T: Timer,
{
    /// Construct a new instance of [`OneShotTimer`].
    pub fn new(inner: impl Peripheral<P = T> + 'd) -> Self {
        crate::into_ref!(inner);

        Self { inner }
    }

    /// Pauses execution for *at least* `ms` milliseconds.
    pub fn delay_millis(&self, ms: u32) {
        self.delay((ms as u64).millis());
    }

    /// Pauses execution for *at least* `us` microseconds.
    pub fn delay_micros(&self, us: u32) {
        self.delay((us as u64).micros());
    }

    /// Pauses execution for *at least* `ns` nanoseconds.
    pub fn delay_nanos(&self, ns: u32) {
        self.delay((ns.div_ceil(1000) as u64).micros())
    }

    fn delay(&self, us: MicrosDurationU64) {
        if self.inner.is_running() {
            self.inner.stop();
        }

        self.inner.clear_interrupt();
        self.inner.reset();

        self.inner.enable_auto_reload(false);
        self.inner.load_value(us).unwrap();
        self.inner.start();

        while !self.inner.is_interrupt_set() {
            // Wait
        }

        self.inner.stop();
        self.inner.clear_interrupt();
    }

    /// Start counting until the given timeout and raise an interrupt
    pub fn schedule(&mut self, timeout: MicrosDurationU64) -> Result<(), Error> {
        if self.inner.is_running() {
            self.inner.stop();
        }

        self.inner.clear_interrupt();
        self.inner.reset();

        self.inner.enable_auto_reload(false);
        self.inner.load_value(timeout)?;
        self.inner.start();

        Ok(())
    }

    /// Stop the timer
    pub fn stop(&mut self) {
        self.inner.stop();
    }

    /// Set the interrupt handler
    ///
    /// Note that this will replace any previously set interrupt handler
    pub fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        self.inner.set_interrupt_handler(handler);
    }

    /// Enable listening for interrupts
    pub fn enable_interrupt(&mut self, enable: bool) {
        self.inner.enable_interrupt(enable);
    }

    /// Clear the interrupt flag
    pub fn clear_interrupt(&mut self) {
        self.inner.clear_interrupt();
        self.inner.set_alarm_active(false);
    }
}

impl<T> crate::private::Sealed for OneShotTimer<'_, T> where T: Timer {}

impl<T> InterruptConfigurable for OneShotTimer<'_, T>
where
    T: Timer,
{
    fn set_interrupt_handler(&mut self, handler: crate::interrupt::InterruptHandler) {
        OneShotTimer::set_interrupt_handler(self, handler);
    }
}

impl<T> embedded_hal::delay::DelayNs for OneShotTimer<'_, T>
where
    T: Timer,
{
    fn delay_ns(&mut self, ns: u32) {
        self.delay_nanos(ns);
    }
}

/// A periodic timer.
pub struct PeriodicTimer<'d, T> {
    inner: PeripheralRef<'d, T>,
}

impl<'d, T> PeriodicTimer<'d, T>
where
    T: Timer,
{
    /// Construct a new instance of [`PeriodicTimer`].
    pub fn new(inner: impl Peripheral<P = T> + 'd) -> Self {
        crate::into_ref!(inner);

        Self { inner }
    }

    /// Start a new count down.
    pub fn start(&mut self, timeout: MicrosDurationU64) -> Result<(), Error> {
        if self.inner.is_running() {
            self.inner.stop();
        }

        self.inner.clear_interrupt();
        self.inner.reset();

        self.inner.enable_auto_reload(true);
        self.inner.load_value(timeout)?;
        self.inner.start();

        Ok(())
    }

    /// "Wait" until the count down finishes without blocking.
    pub fn wait(&mut self) -> nb::Result<(), void::Void> {
        if self.inner.is_interrupt_set() {
            self.inner.clear_interrupt();
            self.inner.set_alarm_active(true);

            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    /// Tries to cancel the active count down.
    pub fn cancel(&mut self) -> Result<(), Error> {
        if !self.inner.is_running() {
            return Err(Error::TimerInactive);
        }

        self.inner.stop();

        Ok(())
    }

    /// Set the interrupt handler
    ///
    /// Note that this will replace any previously set interrupt handler
    pub fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        self.inner.set_interrupt_handler(handler);
    }

    /// Enable/disable listening for interrupts
    pub fn enable_interrupt(&mut self, enable: bool) {
        self.inner.enable_interrupt(enable);
    }

    /// Clear the interrupt flag
    pub fn clear_interrupt(&mut self) {
        self.inner.clear_interrupt();
        self.inner.set_alarm_active(true);
    }
}

impl<T> crate::private::Sealed for PeriodicTimer<'_, T> where T: Timer {}

impl<T> InterruptConfigurable for PeriodicTimer<'_, T>
where
    T: Timer,
{
    fn set_interrupt_handler(&mut self, handler: crate::interrupt::InterruptHandler) {
        PeriodicTimer::set_interrupt_handler(self, handler);
    }
}

/// An enum of all timer types
enum AnyTimerInner {
    /// Timer 0 of the TIMG0 peripheral in blocking mode.
    TimgTimer(timg::Timer),
    /// Systimer Alarm
    #[cfg(systimer)]
    SystimerAlarm(systimer::Alarm),
}

/// A type-erased timer
///
/// You can create an instance of this by just calling `.into()` on a timer.
pub struct AnyTimer(AnyTimerInner);

impl crate::private::Sealed for AnyTimer {}

impl From<timg::Timer> for AnyTimer {
    fn from(value: timg::Timer) -> Self {
        Self(AnyTimerInner::TimgTimer(value))
    }
}

#[cfg(systimer)]
impl From<systimer::Alarm> for AnyTimer {
    fn from(value: systimer::Alarm) -> Self {
        Self(AnyTimerInner::SystimerAlarm(value))
    }
}

impl Timer for AnyTimer {
    delegate::delegate! {
        to match &self.0 {
            AnyTimerInner::TimgTimer(inner) => inner,
            #[cfg(systimer)]
            AnyTimerInner::SystimerAlarm(inner) => inner,
        } {
            fn start(&self);
            fn stop(&self);
            fn reset(&self);
            fn is_running(&self) -> bool;
            fn now(&self) -> Instant<u64, 1, 1_000_000>;
            fn load_value(&self, value: MicrosDurationU64) -> Result<(), Error>;
            fn enable_auto_reload(&self, auto_reload: bool);
            fn enable_interrupt(&self, state: bool);
            fn clear_interrupt(&self);
            fn set_interrupt_handler(&self, handler: InterruptHandler);
            fn is_interrupt_set(&self) -> bool;
            fn set_alarm_active(&self, state: bool);
        }
    }
}

impl Peripheral for AnyTimer {
    type P = Self;

    #[inline]
    unsafe fn clone_unchecked(&self) -> Self::P {
        core::ptr::read(self as *const _)
    }
}
