use core::{
    mem::ManuallyDrop,
    ops::{Deref, DerefMut},
};

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

impl<I: core::fmt::Debug, F: FnOnce(I)> core::fmt::Debug for DropGuard<I, F> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("DropGuard")
            .field("inner", &*self.inner)
            .finish_non_exhaustive()
    }
}

#[cfg(feature = "defmt")]
impl<I: defmt::Format, F: FnOnce(I)> defmt::Format for DropGuard<I, F> {
    fn format(&self, f: defmt::Formatter<'_>) {
        defmt::write!(f, "DropGuard {{ inner: {}, .. }}", &*self.inner)
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
