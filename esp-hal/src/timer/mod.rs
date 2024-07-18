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
//! ### One-shot Timer
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! # use esp_hal::timer::{OneShotTimer, PeriodicTimer, timg::TimerGroup};
//! # use esp_hal::prelude::*;
//! let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks);
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
//! # use esp_hal::prelude::*;
//! let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks);
//! let mut periodic = PeriodicTimer::new(timg0.timer0);
//!
//! periodic.start(1.secs());
//! loop {
//!     nb::block!(periodic.wait());
//! }
//! # }
//! ```

#![deny(missing_docs)]

use fugit::{ExtU64, Instant, MicrosDurationU64};

use crate::{interrupt::InterruptHandler, private, InterruptConfigurable};

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
pub struct OneShotTimer<T> {
    inner: T,
}

impl<T> OneShotTimer<T>
where
    T: Timer,
{
    /// Construct a new instance of [`OneShotTimer`].
    pub fn new(inner: T) -> Self {
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
        self.delay((ns as u64 / 1000).micros())
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

impl<T> crate::private::Sealed for OneShotTimer<T> where T: Timer {}

impl<T> InterruptConfigurable for OneShotTimer<T>
where
    T: Timer,
{
    fn set_interrupt_handler(&mut self, handler: crate::interrupt::InterruptHandler) {
        OneShotTimer::set_interrupt_handler(self, handler);
    }
}

#[cfg(feature = "embedded-hal-02")]
impl<T, UXX> embedded_hal_02::blocking::delay::DelayMs<UXX> for OneShotTimer<T>
where
    T: Timer,
    UXX: Into<u32>,
{
    fn delay_ms(&mut self, ms: UXX) {
        self.delay_millis(ms.into());
    }
}

#[cfg(feature = "embedded-hal-02")]
impl<T, UXX> embedded_hal_02::blocking::delay::DelayUs<UXX> for OneShotTimer<T>
where
    T: Timer,
    UXX: Into<u32>,
{
    fn delay_us(&mut self, us: UXX) {
        self.delay_micros(us.into());
    }
}

#[cfg(feature = "embedded-hal")]
impl<T> embedded_hal::delay::DelayNs for OneShotTimer<T>
where
    T: Timer,
{
    fn delay_ns(&mut self, ns: u32) {
        self.delay_nanos(ns);
    }
}

/// A periodic timer.
pub struct PeriodicTimer<T> {
    inner: T,
}

impl<T> PeriodicTimer<T>
where
    T: Timer,
{
    /// Construct a new instance of [`PeriodicTimer`].
    pub fn new(inner: T) -> Self {
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

impl<T> crate::private::Sealed for PeriodicTimer<T> where T: Timer {}

impl<T> InterruptConfigurable for PeriodicTimer<T>
where
    T: Timer,
{
    fn set_interrupt_handler(&mut self, handler: crate::interrupt::InterruptHandler) {
        PeriodicTimer::set_interrupt_handler(self, handler);
    }
}

/// A type-erased timer
///
/// You can create an instance of this by just calling `.into()` on a timer.
#[allow(missing_docs)]
pub enum ErasedTimer {
    Timg0Timer0(timg::Timer<timg::Timer0<crate::peripherals::TIMG0>, crate::Blocking>),
    #[cfg(timg_timer1)]
    Timg0Timer1(timg::Timer<timg::Timer1<crate::peripherals::TIMG0>, crate::Blocking>),
    #[cfg(timg1)]
    Timg1Timer0(timg::Timer<timg::Timer0<crate::peripherals::TIMG1>, crate::Blocking>),
    #[cfg(all(timg1, timg_timer1))]
    Timg1Timer1(timg::Timer<timg::Timer1<crate::peripherals::TIMG1>, crate::Blocking>),
    #[cfg(systimer)]
    SystimerAlarm0Periodic(systimer::Alarm<systimer::Periodic, crate::Blocking, 0>),
    #[cfg(systimer)]
    SystimerAlarm1Periodic(systimer::Alarm<systimer::Periodic, crate::Blocking, 1>),
    #[cfg(systimer)]
    SystimerAlarm2Periodic(systimer::Alarm<systimer::Periodic, crate::Blocking, 2>),
    #[cfg(systimer)]
    SystimerAlarm0Target(systimer::Alarm<systimer::Target, crate::Blocking, 0>),
    #[cfg(systimer)]
    SystimerAlarm1Target(systimer::Alarm<systimer::Target, crate::Blocking, 1>),
    #[cfg(systimer)]
    SystimerAlarm2Target(systimer::Alarm<systimer::Target, crate::Blocking, 2>),
}

impl private::Sealed for ErasedTimer {}

impl From<timg::Timer<timg::Timer0<crate::peripherals::TIMG0>, crate::Blocking>> for ErasedTimer {
    fn from(value: timg::Timer<timg::Timer0<crate::peripherals::TIMG0>, crate::Blocking>) -> Self {
        Self::Timg0Timer0(value)
    }
}

#[cfg(timg_timer1)]
impl From<timg::Timer<timg::Timer1<crate::peripherals::TIMG0>, crate::Blocking>> for ErasedTimer {
    fn from(value: timg::Timer<timg::Timer1<crate::peripherals::TIMG0>, crate::Blocking>) -> Self {
        Self::Timg0Timer1(value)
    }
}

#[cfg(timg1)]
impl From<timg::Timer<timg::Timer0<crate::peripherals::TIMG1>, crate::Blocking>> for ErasedTimer {
    fn from(value: timg::Timer<timg::Timer0<crate::peripherals::TIMG1>, crate::Blocking>) -> Self {
        Self::Timg1Timer0(value)
    }
}

#[cfg(all(timg1, timg_timer1))]
impl From<timg::Timer<timg::Timer1<crate::peripherals::TIMG1>, crate::Blocking>> for ErasedTimer {
    fn from(value: timg::Timer<timg::Timer1<crate::peripherals::TIMG1>, crate::Blocking>) -> Self {
        Self::Timg1Timer1(value)
    }
}

#[cfg(systimer)]
impl From<systimer::Alarm<systimer::Periodic, crate::Blocking, 0>> for ErasedTimer {
    fn from(value: systimer::Alarm<systimer::Periodic, crate::Blocking, 0>) -> Self {
        Self::SystimerAlarm0Periodic(value)
    }
}

#[cfg(systimer)]
impl From<systimer::Alarm<systimer::Periodic, crate::Blocking, 1>> for ErasedTimer {
    fn from(value: systimer::Alarm<systimer::Periodic, crate::Blocking, 1>) -> Self {
        Self::SystimerAlarm1Periodic(value)
    }
}

#[cfg(systimer)]
impl From<systimer::Alarm<systimer::Periodic, crate::Blocking, 2>> for ErasedTimer {
    fn from(value: systimer::Alarm<systimer::Periodic, crate::Blocking, 2>) -> Self {
        Self::SystimerAlarm2Periodic(value)
    }
}

#[cfg(systimer)]
impl From<systimer::Alarm<systimer::Target, crate::Blocking, 0>> for ErasedTimer {
    fn from(value: systimer::Alarm<systimer::Target, crate::Blocking, 0>) -> Self {
        Self::SystimerAlarm0Target(value)
    }
}

#[cfg(systimer)]
impl From<systimer::Alarm<systimer::Target, crate::Blocking, 1>> for ErasedTimer {
    fn from(value: systimer::Alarm<systimer::Target, crate::Blocking, 1>) -> Self {
        Self::SystimerAlarm1Target(value)
    }
}

#[cfg(systimer)]
impl From<systimer::Alarm<systimer::Target, crate::Blocking, 2>> for ErasedTimer {
    fn from(value: systimer::Alarm<systimer::Target, crate::Blocking, 2>) -> Self {
        Self::SystimerAlarm2Target(value)
    }
}

impl Timer for ErasedTimer {
    delegate::delegate! {
        to match self {
            ErasedTimer::Timg0Timer0(inner) => inner,
            #[cfg(timg_timer1)]
            ErasedTimer::Timg0Timer1(inner) => inner,
            #[cfg(timg1)]
            ErasedTimer::Timg1Timer0(inner) => inner,
            #[cfg(all(timg1,timg_timer1))]
            ErasedTimer::Timg1Timer1(inner) => inner,
            #[cfg(systimer)]
            ErasedTimer::SystimerAlarm0Periodic(inner) => inner,
            #[cfg(systimer)]
            ErasedTimer::SystimerAlarm1Periodic(inner) => inner,
            #[cfg(systimer)]
            ErasedTimer::SystimerAlarm2Periodic(inner) => inner,
            #[cfg(systimer)]
            ErasedTimer::SystimerAlarm0Target(inner) => inner,
            #[cfg(systimer)]
            ErasedTimer::SystimerAlarm1Target(inner) => inner,
            #[cfg(systimer)]
            ErasedTimer::SystimerAlarm2Target(inner) => inner,
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

#[cfg(feature = "embedded-hal-02")]
impl<T> embedded_hal_02::timer::CountDown for PeriodicTimer<T>
where
    T: Timer,
{
    type Time = MicrosDurationU64;

    fn start<Time>(&mut self, timeout: Time)
    where
        Time: Into<Self::Time>,
    {
        self.start(timeout.into()).unwrap();
    }

    fn wait(&mut self) -> nb::Result<(), void::Void> {
        self.wait()
    }
}

#[cfg(feature = "embedded-hal-02")]
impl<T> embedded_hal_02::timer::Cancel for PeriodicTimer<T>
where
    T: Timer,
{
    type Error = Error;

    fn cancel(&mut self) -> Result<(), Self::Error> {
        self.cancel()
    }
}

#[cfg(feature = "embedded-hal-02")]
impl<T> embedded_hal_02::timer::Periodic for PeriodicTimer<T> where T: Timer {}
