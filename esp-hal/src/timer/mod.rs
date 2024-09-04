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
    interrupt::InterruptHandler,
    peripheral::{Peripheral, PeripheralRef},
    Blocking,
    InterruptConfigurable,
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

impl<'d, T> crate::private::Sealed for OneShotTimer<'d, T> where T: Timer {}

impl<'d, T> InterruptConfigurable for OneShotTimer<'d, T>
where
    T: Timer,
{
    fn set_interrupt_handler(&mut self, handler: crate::interrupt::InterruptHandler) {
        OneShotTimer::set_interrupt_handler(self, handler);
    }
}

impl<'d, T, UXX> embedded_hal_02::blocking::delay::DelayMs<UXX> for OneShotTimer<'d, T>
where
    T: Timer,
    UXX: Into<u32>,
{
    fn delay_ms(&mut self, ms: UXX) {
        self.delay_millis(ms.into());
    }
}

impl<'d, T, UXX> embedded_hal_02::blocking::delay::DelayUs<UXX> for OneShotTimer<'d, T>
where
    T: Timer,
    UXX: Into<u32>,
{
    fn delay_us(&mut self, us: UXX) {
        self.delay_micros(us.into());
    }
}

impl<'d, T> embedded_hal::delay::DelayNs for OneShotTimer<'d, T>
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

impl<'d, T> crate::private::Sealed for PeriodicTimer<'d, T> where T: Timer {}

impl<'d, T> InterruptConfigurable for PeriodicTimer<'d, T>
where
    T: Timer,
{
    fn set_interrupt_handler(&mut self, handler: crate::interrupt::InterruptHandler) {
        PeriodicTimer::set_interrupt_handler(self, handler);
    }
}

impl<'d, T> embedded_hal_02::timer::CountDown for PeriodicTimer<'d, T>
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

impl<'d, T> embedded_hal_02::timer::Cancel for PeriodicTimer<'d, T>
where
    T: Timer,
{
    type Error = Error;

    fn cancel(&mut self) -> Result<(), Self::Error> {
        self.cancel()
    }
}

impl<'d, T> embedded_hal_02::timer::Periodic for PeriodicTimer<'d, T> where T: Timer {}

/// A type-erased timer
///
/// You can create an instance of this by just calling `.into()` on a timer.
pub enum ErasedTimer {
    /// Timer 0 of the TIMG0 peripheral in blocking mode.
    Timg0Timer0(timg::Timer<timg::Timer0<crate::peripherals::TIMG0>, Blocking>),
    /// Timer 1 of the TIMG0 peripheral in blocking mode.
    #[cfg(timg_timer1)]
    Timg0Timer1(timg::Timer<timg::Timer1<crate::peripherals::TIMG0>, Blocking>),
    /// Timer 0 of the TIMG1 peripheral in blocking mode.
    #[cfg(timg1)]
    Timg1Timer0(timg::Timer<timg::Timer0<crate::peripherals::TIMG1>, Blocking>),
    /// Timer 1 of the TIMG1 peripheral in blocking mode.
    #[cfg(all(timg1, timg_timer1))]
    Timg1Timer1(timg::Timer<timg::Timer1<crate::peripherals::TIMG1>, Blocking>),
    /// Systimer Alarm in periodic mode with blocking behavior.
    #[cfg(systimer)]
    SystimerAlarmPeriodic(systimer::Alarm<'static, systimer::Periodic, Blocking>),
    /// Systimer Target in periodic mode with blocking behavior.
    #[cfg(systimer)]
    SystimerAlarmTarget(systimer::Alarm<'static, systimer::Target, Blocking>),
}

impl crate::private::Sealed for ErasedTimer {}

impl From<timg::Timer<timg::Timer0<crate::peripherals::TIMG0>, Blocking>> for ErasedTimer {
    fn from(value: timg::Timer<timg::Timer0<crate::peripherals::TIMG0>, Blocking>) -> Self {
        Self::Timg0Timer0(value)
    }
}

#[cfg(timg_timer1)]
impl From<timg::Timer<timg::Timer1<crate::peripherals::TIMG0>, Blocking>> for ErasedTimer {
    fn from(value: timg::Timer<timg::Timer1<crate::peripherals::TIMG0>, Blocking>) -> Self {
        Self::Timg0Timer1(value)
    }
}

#[cfg(timg1)]
impl From<timg::Timer<timg::Timer0<crate::peripherals::TIMG1>, Blocking>> for ErasedTimer {
    fn from(value: timg::Timer<timg::Timer0<crate::peripherals::TIMG1>, Blocking>) -> Self {
        Self::Timg1Timer0(value)
    }
}

#[cfg(all(timg1, timg_timer1))]
impl From<timg::Timer<timg::Timer1<crate::peripherals::TIMG1>, Blocking>> for ErasedTimer {
    fn from(value: timg::Timer<timg::Timer1<crate::peripherals::TIMG1>, Blocking>) -> Self {
        Self::Timg1Timer1(value)
    }
}

#[cfg(systimer)]
impl From<systimer::Alarm<'static, systimer::Periodic, Blocking>> for ErasedTimer {
    fn from(value: systimer::Alarm<'static, systimer::Periodic, Blocking>) -> Self {
        Self::SystimerAlarmPeriodic(value)
    }
}

#[cfg(systimer)]
impl From<systimer::Alarm<'static, systimer::Target, Blocking>> for ErasedTimer {
    fn from(value: systimer::Alarm<'static, systimer::Target, Blocking>) -> Self {
        Self::SystimerAlarmTarget(value)
    }
}

impl Timer for ErasedTimer {
    // Rather than manually implementing `Timer` for each variant of `ErasedTimer`,
    // we use `delegate::delegate!{}` to do this for us. Otherwise, each function
    // implementation would require its own `match` block for each enum variant,
    // which would very quickly result in a large amount of duplicated code.
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
            ErasedTimer::SystimerAlarmPeriodic(inner) => inner,
            #[cfg(systimer)]
            ErasedTimer::SystimerAlarmTarget(inner) => inner,
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

impl Peripheral for ErasedTimer {
    type P = Self;

    #[inline]
    unsafe fn clone_unchecked(&mut self) -> Self::P {
        core::ptr::read(self as *const _)
    }
}
