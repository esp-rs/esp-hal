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
//! let mut one_shot = OneShotTimer::new(timg0.timer0);
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

use core::marker::PhantomData;

use fugit::{ExtU64, Instant, MicrosDurationU64};

use crate::{
    interrupt::{InterruptConfigurable, InterruptHandler},
    peripheral::{Peripheral, PeripheralRef},
    peripherals::Interrupt,
    Async,
    Blocking,
    Cpu,
    Mode,
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
pub trait Timer: Into<AnyTimer> + InterruptConfigurable + 'static + crate::private::Sealed {
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

    /// Has the timer triggered?
    fn is_interrupt_set(&self) -> bool;

    /// Asynchronously wait for the timer interrupt to fire.
    ///
    /// Requires the correct `InterruptHandler` to be installed to function
    /// correctly.
    async fn wait(&self);

    /// Returns the HAL provided async interrupt handler
    fn async_interrupt_handler(&self) -> InterruptHandler;

    /// Returns the interrupt source for the underlying timer
    fn peripheral_interrupt(&self) -> Interrupt;
}

/// A one-shot timer.
pub struct OneShotTimer<'d, M, T = AnyTimer> {
    inner: PeripheralRef<'d, T>,
    _ph: PhantomData<M>,
}

impl<'d> OneShotTimer<'d, Blocking> {
    /// Construct a new instance of [`OneShotTimer`].
    pub fn new(inner: impl Peripheral<P = impl Timer> + 'd) -> OneShotTimer<'d, Blocking> {
        Self::new_typed(inner.map_into())
    }
}

impl<'d, T> OneShotTimer<'d, Blocking, T>
where
    T: Timer,
{
    /// Construct a typed instance of [`OneShotTimer`].
    pub fn new_typed(inner: impl Peripheral<P = T> + 'd) -> Self {
        crate::into_ref!(inner);
        Self {
            inner,
            _ph: PhantomData,
        }
    }

    /// Converts the driver to [`Async`] mode.
    pub fn into_async(mut self) -> OneShotTimer<'d, Async, T> {
        let handler = self.inner.async_interrupt_handler();
        self.inner.set_interrupt_handler(handler);
        OneShotTimer {
            inner: self.inner,
            _ph: PhantomData,
        }
    }
}

impl<T> OneShotTimer<'_, Async, T>
where
    T: Timer,
{
    /// Converts the driver to [`Blocking`] mode.
    pub fn into_blocking(self) -> Self {
        crate::interrupt::disable(Cpu::current(), self.inner.peripheral_interrupt());
        Self {
            inner: self.inner,
            _ph: PhantomData,
        }
    }
}

impl<T> OneShotTimer<'_, Async, T>
where
    T: Timer,
{
    /// Delay for *at least* `ns` nanoseconds.
    pub async fn delay_nanos_async(&mut self, ns: u32) {
        self.delay_async(MicrosDurationU64::from_ticks(ns.div_ceil(1000) as u64))
            .await
    }

    /// Delay for *at least* `ms` milliseconds.
    pub async fn delay_millis_async(&mut self, ms: u32) {
        self.delay_async((ms as u64).millis()).await;
    }

    /// Delay for *at least* `us` microseconds.
    pub async fn delay_micros_async(&mut self, us: u32) {
        self.delay_async((us as u64).micros()).await;
    }

    async fn delay_async(&mut self, us: MicrosDurationU64) {
        unwrap!(self.schedule(us));
        self.inner.wait().await;
        self.stop();
        self.clear_interrupt();
    }
}

impl<M, T> OneShotTimer<'_, M, T>
where
    M: Mode,
    T: Timer,
{
    /// Delay for *at least* `ms` milliseconds.
    pub fn delay_millis(&mut self, ms: u32) {
        self.delay((ms as u64).millis());
    }

    /// Delay for *at least* `us` microseconds.
    pub fn delay_micros(&mut self, us: u32) {
        self.delay((us as u64).micros());
    }

    /// Delay for *at least* `ns` nanoseconds.
    pub fn delay_nanos(&mut self, ns: u32) {
        self.delay((ns.div_ceil(1000) as u64).micros())
    }

    fn delay(&mut self, us: MicrosDurationU64) {
        self.schedule(us).unwrap();

        while !self.inner.is_interrupt_set() {
            // Wait
        }

        self.stop();
        self.clear_interrupt();
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
    }
}

impl<M, T> crate::private::Sealed for OneShotTimer<'_, M, T>
where
    T: Timer,
    M: Mode,
{
}

impl<M, T> InterruptConfigurable for OneShotTimer<'_, M, T>
where
    M: Mode,
    T: Timer,
{
    fn set_interrupt_handler(&mut self, handler: crate::interrupt::InterruptHandler) {
        OneShotTimer::set_interrupt_handler(self, handler);
    }
}

impl<T> embedded_hal::delay::DelayNs for OneShotTimer<'_, Blocking, T>
where
    T: Timer,
{
    fn delay_ns(&mut self, ns: u32) {
        self.delay_nanos(ns);
    }
}

impl<T> embedded_hal_async::delay::DelayNs for OneShotTimer<'_, Async, T>
where
    T: Timer,
{
    async fn delay_ns(&mut self, ns: u32) {
        self.delay_nanos_async(ns).await
    }
}

/// A periodic timer.
pub struct PeriodicTimer<'d, M, T = AnyTimer> {
    inner: PeripheralRef<'d, T>,
    _ph: PhantomData<M>,
}

impl<'d> PeriodicTimer<'d, Blocking> {
    /// Construct a new instance of [`PeriodicTimer`].
    pub fn new(inner: impl Peripheral<P = impl Timer> + 'd) -> PeriodicTimer<'d, Blocking> {
        Self::new_typed(inner.map_into())
    }
}

impl<'d, T> PeriodicTimer<'d, Blocking, T>
where
    T: Timer,
{
    /// Construct a typed instance of [`PeriodicTimer`].
    pub fn new_typed(inner: impl Peripheral<P = T> + 'd) -> Self {
        crate::into_ref!(inner);
        Self {
            inner,
            _ph: PhantomData,
        }
    }
}

impl<M, T> PeriodicTimer<'_, M, T>
where
    M: Mode,
    T: Timer,
{
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
    }
}

impl<M, T> crate::private::Sealed for PeriodicTimer<'_, M, T> where T: Timer {}

impl<M, T> InterruptConfigurable for PeriodicTimer<'_, M, T>
where
    M: Mode,
    T: Timer,
{
    fn set_interrupt_handler(&mut self, handler: crate::interrupt::InterruptHandler) {
        PeriodicTimer::set_interrupt_handler(self, handler);
    }
}

crate::any_peripheral! {
    /// Any Timer peripheral.
    pub peripheral AnyTimer {
        TimgTimer(timg::Timer),
        #[cfg(systimer)]
        SystimerAlarm(systimer::Alarm),
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
            fn is_interrupt_set(&self) -> bool;
            async fn wait(&self);
            fn async_interrupt_handler(&self) -> InterruptHandler;
            fn peripheral_interrupt(&self) -> Interrupt;
        }
    }
}

impl InterruptConfigurable for AnyTimer {
    delegate::delegate! {
        to match &mut self.0 {
            AnyTimerInner::TimgTimer(inner) => inner,
            #[cfg(systimer)]
            AnyTimerInner::SystimerAlarm(inner) => inner,
        } {
            fn set_interrupt_handler(&mut self, handler: InterruptHandler);
        }
    }
}
