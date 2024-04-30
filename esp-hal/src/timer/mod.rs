//! General-purpose timers.

use fugit::MicrosDurationU64;

#[cfg(systimer)]
pub mod systimer;
#[cfg(any(timg0, timg1))]
pub mod timg;

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
}
