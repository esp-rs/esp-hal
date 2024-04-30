//! General-purpose timers.

use fugit::MicrosDurationU64;

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
    fn now(&self) -> u64;

    /// Load a target value into the timer.
    fn load_value(&self, value: MicrosDurationU64);

    /// Enable auto reload of the loaded value.
    fn enable_auto_reload(&self, auto_reload: bool);

    /// Enable or disable the timer's interrupt.
    fn enable_interrupt(&self, state: bool);

    /// Clear the timer's interrupt.
    fn clear_interrupt(&self);

    /// Has the timer triggered?
    fn is_interrupt_set(&self) -> bool;

    /// FIXME: This is (hopefully?) temporary...
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
    pub fn start(&mut self, timeout: MicrosDurationU64) {
        if self.inner.is_running() {
            self.inner.stop();
        }

        self.inner.clear_interrupt();
        self.inner.reset();

        self.inner.load_value(timeout);
        self.inner.enable_auto_reload(true);
        self.inner.enable_interrupt(true);
        self.inner.start();
    }

    /// "Wait" until the count down finishes without blocking.
    pub fn wait(&mut self) -> nb::Result<(), void::Void> {
        if self.inner.is_interrupt_set() {
            self.inner.clear_interrupt();
            self.inner.set_alarm_active(true); // FIXME: Remove if/when able

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
        self.start(timeout.into());
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
