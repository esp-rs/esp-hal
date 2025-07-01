//! # General-purpose Timers
//!
//! ## Overview
//! The [OneShotTimer] and [PeriodicTimer] types can be backed by any hardware
//! peripheral which implements the [Timer] trait. This means that the same API
//! can be used to interact with different hardware timers, like the `TIMG` and
//! SYSTIMER.
#![cfg_attr(
    systimer,
    doc = "See the [timg] and [systimer] modules for more information."
)]
#![cfg_attr(not(systimer), doc = "See the [timg] module for more information.")]
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
//! # Ok(())
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
//! periodic.start(Duration::from_secs(1));
//! loop {
//!    periodic.wait();
//! }
//! # }
//! ```

use core::{
    marker::PhantomData,
    pin::Pin,
    task::{Context, Poll},
};

use crate::{
    Async,
    Blocking,
    DriverMode,
    asynch::AtomicWaker,
    interrupt::{InterruptConfigurable, InterruptHandler},
    peripherals::Interrupt,
    system::Cpu,
    time::{Duration, Instant},
};

#[cfg(systimer)]
pub mod systimer;
#[cfg(timergroup)]
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
    #[doc(hidden)]
    fn start(&self);

    /// Stop the timer.
    #[doc(hidden)]
    fn stop(&self);

    /// Reset the timer value to 0.
    #[doc(hidden)]
    fn reset(&self);

    /// Is the timer running?
    #[doc(hidden)]
    fn is_running(&self) -> bool;

    /// The current timer value.
    #[doc(hidden)]
    fn now(&self) -> Instant;

    /// Load a target value into the timer.
    #[doc(hidden)]
    fn load_value(&self, value: Duration) -> Result<(), Error>;

    /// Enable auto reload of the loaded value.
    #[doc(hidden)]
    fn enable_auto_reload(&self, auto_reload: bool);

    /// Enable or disable the timer's interrupt.
    #[doc(hidden)]
    fn enable_interrupt(&self, state: bool);

    /// Clear the timer's interrupt.
    fn clear_interrupt(&self);

    /// Has the timer triggered?
    fn is_interrupt_set(&self) -> bool;

    /// Returns the HAL provided async interrupt handler
    #[doc(hidden)]
    fn async_interrupt_handler(&self) -> InterruptHandler;

    /// Returns the interrupt source for the underlying timer
    fn peripheral_interrupt(&self) -> Interrupt;

    /// Configures the interrupt handler.
    #[doc(hidden)]
    fn set_interrupt_handler(&self, handler: InterruptHandler);

    #[doc(hidden)]
    fn waker(&self) -> &AtomicWaker;
}

/// A one-shot timer.
pub struct OneShotTimer<'d, Dm: DriverMode> {
    inner: AnyTimer<'d>,
    _ph: PhantomData<Dm>,
}

impl<'d> OneShotTimer<'d, Blocking> {
    /// Construct a new instance of [`OneShotTimer`].
    pub fn new(inner: impl Timer + Into<AnyTimer<'d>>) -> OneShotTimer<'d, Blocking> {
        Self {
            inner: inner.into(),
            _ph: PhantomData,
        }
    }
}

impl<'d> OneShotTimer<'d, Blocking> {
    /// Converts the driver to [`Async`] mode.
    pub fn into_async(self) -> OneShotTimer<'d, Async> {
        let handler = self.inner.async_interrupt_handler();
        self.inner.set_interrupt_handler(handler);
        OneShotTimer {
            inner: self.inner,
            _ph: PhantomData,
        }
    }
}

impl OneShotTimer<'_, Async> {
    /// Converts the driver to [`Blocking`] mode.
    pub fn into_blocking(self) -> Self {
        crate::interrupt::disable(Cpu::current(), self.inner.peripheral_interrupt());
        Self {
            inner: self.inner,
            _ph: PhantomData,
        }
    }

    /// Delay for *at least* `ns` nanoseconds.
    pub async fn delay_nanos_async(&mut self, ns: u32) {
        self.delay_async(Duration::from_micros(ns.div_ceil(1000) as u64))
            .await
    }

    /// Delay for *at least* `ms` milliseconds.
    pub async fn delay_millis_async(&mut self, ms: u32) {
        self.delay_async(Duration::from_millis(ms as u64)).await;
    }

    /// Delay for *at least* `us` microseconds.
    pub async fn delay_micros_async(&mut self, us: u32) {
        self.delay_async(Duration::from_micros(us as u64)).await;
    }

    async fn delay_async(&mut self, us: Duration) {
        unwrap!(self.schedule(us));

        WaitFuture::new(self.inner.reborrow()).await;

        self.stop();
        self.clear_interrupt();
    }
}

#[must_use = "futures do nothing unless you `.await` or poll them"]
struct WaitFuture<'d> {
    timer: AnyTimer<'d>,
}

impl<'d> WaitFuture<'d> {
    fn new(timer: AnyTimer<'d>) -> Self {
        // For some reason, on the S2 we need to enable the interrupt before we
        // read its status. Doing so in the other order causes the interrupt
        // request to never be fired.
        timer.enable_interrupt(true);
        Self { timer }
    }

    fn is_done(&self) -> bool {
        self.timer.is_interrupt_set()
    }
}

impl core::future::Future for WaitFuture<'_> {
    type Output = ();

    fn poll(self: Pin<&mut Self>, ctx: &mut Context<'_>) -> Poll<Self::Output> {
        // Interrupts are enabled, so we need to register the waker before we check for
        // done. Otherwise we might miss the interrupt that would wake us.
        self.timer.waker().register(ctx.waker());

        if self.is_done() {
            Poll::Ready(())
        } else {
            Poll::Pending
        }
    }
}

impl Drop for WaitFuture<'_> {
    fn drop(&mut self) {
        self.timer.enable_interrupt(false);
    }
}

impl<Dm> OneShotTimer<'_, Dm>
where
    Dm: DriverMode,
{
    /// Delay for *at least* `ms` milliseconds.
    pub fn delay_millis(&mut self, ms: u32) {
        self.delay(Duration::from_millis(ms as u64));
    }

    /// Delay for *at least* `us` microseconds.
    pub fn delay_micros(&mut self, us: u32) {
        self.delay(Duration::from_micros(us as u64));
    }

    /// Delay for *at least* `ns` nanoseconds.
    pub fn delay_nanos(&mut self, ns: u32) {
        self.delay(Duration::from_micros(ns.div_ceil(1000) as u64))
    }

    fn delay(&mut self, us: Duration) {
        self.schedule(us).unwrap();

        while !self.inner.is_interrupt_set() {
            // Wait
        }

        self.stop();
        self.clear_interrupt();
    }

    /// Start counting until the given timeout and raise an interrupt
    pub fn schedule(&mut self, timeout: Duration) -> Result<(), Error> {
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
    #[instability::unstable]
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

impl<Dm> crate::private::Sealed for OneShotTimer<'_, Dm> where Dm: DriverMode {}

impl<Dm> InterruptConfigurable for OneShotTimer<'_, Dm>
where
    Dm: DriverMode,
{
    fn set_interrupt_handler(&mut self, handler: crate::interrupt::InterruptHandler) {
        OneShotTimer::set_interrupt_handler(self, handler);
    }
}

impl embedded_hal::delay::DelayNs for OneShotTimer<'_, Blocking> {
    fn delay_ns(&mut self, ns: u32) {
        self.delay_nanos(ns);
    }
}

impl embedded_hal_async::delay::DelayNs for OneShotTimer<'_, Async> {
    async fn delay_ns(&mut self, ns: u32) {
        self.delay_nanos_async(ns).await
    }
}

/// A periodic timer.
pub struct PeriodicTimer<'d, Dm: DriverMode> {
    inner: AnyTimer<'d>,
    _ph: PhantomData<Dm>,
}

impl<'d> PeriodicTimer<'d, Blocking> {
    /// Construct a new instance of [`PeriodicTimer`].
    pub fn new(inner: impl Timer + Into<AnyTimer<'d>>) -> PeriodicTimer<'d, Blocking> {
        Self {
            inner: inner.into(),
            _ph: PhantomData,
        }
    }
}

impl<Dm> PeriodicTimer<'_, Dm>
where
    Dm: DriverMode,
{
    /// Start a new count down.
    pub fn start(&mut self, period: Duration) -> Result<(), Error> {
        if self.inner.is_running() {
            self.inner.stop();
        }

        self.inner.clear_interrupt();
        self.inner.reset();

        self.inner.enable_auto_reload(true);
        self.inner.load_value(period)?;
        self.inner.start();

        Ok(())
    }

    /// "Wait", by blocking, until the count down finishes.
    pub fn wait(&mut self) {
        while !self.inner.is_interrupt_set() {}
        self.inner.clear_interrupt();
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
    #[instability::unstable]
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

impl<Dm> crate::private::Sealed for PeriodicTimer<'_, Dm> where Dm: DriverMode {}

impl<Dm> InterruptConfigurable for PeriodicTimer<'_, Dm>
where
    Dm: DriverMode,
{
    fn set_interrupt_handler(&mut self, handler: crate::interrupt::InterruptHandler) {
        PeriodicTimer::set_interrupt_handler(self, handler);
    }
}

crate::any_peripheral! {
    /// Any Timer peripheral.
    pub peripheral AnyTimer<'d> {
        #[cfg(timergroup)]
        TimgTimer(timg::Timer<'d>),
        #[cfg(systimer)]
        SystimerAlarm(systimer::Alarm<'d>),
    }
}

impl Timer for AnyTimer<'_> {
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
            fn now(&self) -> Instant;
            fn load_value(&self, value: Duration) -> Result<(), Error>;
            fn enable_auto_reload(&self, auto_reload: bool);
            fn enable_interrupt(&self, state: bool);
            fn clear_interrupt(&self);
            fn is_interrupt_set(&self) -> bool;
            fn async_interrupt_handler(&self) -> InterruptHandler;
            fn peripheral_interrupt(&self) -> Interrupt;
            fn set_interrupt_handler(&self, handler: InterruptHandler);
            fn waker(&self) -> &AtomicWaker;
        }
    }
}
