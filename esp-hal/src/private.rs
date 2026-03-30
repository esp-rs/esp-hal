use core::{
    mem::ManuallyDrop,
    ops::{Deref, DerefMut},
};

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

pub(crate) struct DropGuard<I, F: FnOnce(I)> {
    inner: ManuallyDrop<I>,
    on_drop: ManuallyDrop<F>,
}

impl<I, F: FnOnce(I)> DropGuard<I, F> {
    pub(crate) fn new(inner: I, on_drop: F) -> Self {
        Self {
            inner: ManuallyDrop::new(inner),
            on_drop: ManuallyDrop::new(on_drop),
        }
    }

    pub(crate) fn defuse(self) {
        core::mem::forget(self);
    }
}

impl<I, F: FnOnce(I)> Drop for DropGuard<I, F> {
    fn drop(&mut self) {
        let inner = unsafe { ManuallyDrop::take(&mut self.inner) };
        let on_drop = unsafe { ManuallyDrop::take(&mut self.on_drop) };
        (on_drop)(inner)
    }
}

impl<I, F: FnOnce(I)> Deref for DropGuard<I, F> {
    type Target = I;

    fn deref(&self) -> &I {
        &self.inner
    }
}

impl<I, F: FnOnce(I)> DerefMut for DropGuard<I, F> {
    fn deref_mut(&mut self) -> &mut I {
        &mut self.inner
    }
}
