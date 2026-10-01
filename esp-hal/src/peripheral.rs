//! Peripheral ownership model traits.

/// A trait implemented by all peripheral singletons.
pub trait Peripheral: Sized {
    /// The peripheral type itself.
    type P;

    /// Consume the peripheral and return a generic/unbounded one if needed, 
    /// or convert it.
    unsafe fn clone_unchecked(&self) -> Self::P;

    /// Converts the peripheral into its underlying type or reborrows it.
    #[inline]
    fn into_peripheral(self) -> Self::P {
        unsafe { self.clone_unchecked() }
    }
}