#![allow(dead_code)]

use portable_atomic::{AtomicPtr, Ordering};

/// An optional `extern "C" fn()` stored as an atomic pointer.
pub(crate) struct CFnPtr(AtomicPtr<()>);

impl CFnPtr {
    pub const fn new() -> Self {
        Self(AtomicPtr::new(core::ptr::null_mut()))
    }

    pub fn store(&self, f: extern "C" fn()) {
        self.0.store(f as *mut (), Ordering::Relaxed);
    }

    pub fn clear(&self) {
        self.0.store(core::ptr::null_mut(), Ordering::Relaxed);
    }

    pub fn call(&self) -> bool {
        let ptr = self.0.load(Ordering::Relaxed);
        if !ptr.is_null() {
            unsafe { (core::mem::transmute::<*mut (), extern "C" fn()>(ptr))() };
            true
        } else {
            false
        }
    }
}

pub trait Sealed {}

#[non_exhaustive]
#[doc(hidden)]
/// Magical incantation to gain access to internal APIs.
pub struct Internal;

impl Internal {
    /// Obtains magical powers to access internal APIs.
    ///
    /// # Safety
    ///
    /// Calling this method accepts that this is an internal
    /// API that is not guaranteed to be documented, stable, or working,
    /// and may change at any time.
    ///
    /// Calling this method also declares that other solutions have been tried, and that
    /// a feature request or an issue has been opened to discuss the
    /// need for this function.
    pub unsafe fn conjure() -> Self {
        Self
    }
}
