use core::mem::ManuallyDrop;

pub trait Sealed {}

#[non_exhaustive]
#[doc(hidden)]
/// Magical incantation to gain access to internal APIs.
pub struct Internal;

impl Internal {
    /// Obtain magical powers to access internal APIs.
    ///
    /// # Safety
    ///
    /// By calling this function, you accept that you are using an internal
    /// API that is not guaranteed to be documented, stable, working
    /// and may change at any time.
    ///
    /// You declare that you have tried to look for other solutions, that
    /// you have opened a feature request or an issue to discuss the
    /// need for this function.
    pub unsafe fn conjure() -> Self {
        Self
    }
}

pub(crate) struct OnDrop<F: FnOnce()>(ManuallyDrop<F>);
impl<F: FnOnce()> OnDrop<F> {
    pub fn new(cb: F) -> Self {
        Self(ManuallyDrop::new(cb))
    }

    pub fn defuse(self) {
        core::mem::forget(self);
    }
}

impl<F: FnOnce()> Drop for OnDrop<F> {
    fn drop(&mut self) {
        unsafe { (ManuallyDrop::take(&mut self.0))() }
    }
}
