#![deny(missing_docs)]

//! Support for statics that persist across resets

use core::{cell::UnsafeCell, mem::MaybeUninit};

use crate::{reset::get_reset_reason, rtc_cntl::SocResetReason};

/// A wrapper type required for `#[ram(persistent)]` statics that handles
/// initialization after power on and unexpected resets.
///
/// # Example
///
/// ```rust
/// #use crate::prelude::*;
/// #use crate::persistent::Persistent;
///
/// #[derive(Debug)]
/// enum BootMode {
///     Normal,
///     Alternate,
///     // ...
/// }
///
/// let boot_mode = {
///     #[ram(rtc_fast, persistent)]
///     static BOOT_MODE: Persistent<BootMode> = Persistent::new();
///     unsafe { BOOT_MODE.get(BootMode::Normal) }
/// };
///
/// println!("Booted in {boot_mode:?} mode");
/// ```
///
/// Also check the [ram example](https://github.com/esp-rs/esp-hal/blob/main/examples/src/bin/ram.rs).
#[derive(Debug)]
pub struct Persistent<T> {
    inner: UnsafeCell<MaybeUninit<T>>,
}

// SAFETY: `&Persistent<T>` can be safely sent to another thread since the only
// way to get access to the internals is through the unsafe get method, which
// requires the caller to manage synchronization.
unsafe impl<T: Sync> Sync for Persistent<T> {}

impl<T> Persistent<T> {
    /// Creates a new, uninitialized instance
    pub const fn new() -> Self {
        Self {
            inner: UnsafeCell::new(MaybeUninit::uninit()),
        }
    }

    /// Gets an exclusive, mutable reference to the persistent data,
    /// initializing it to `init` if necessary.
    ///
    /// # Safety
    ///
    /// This method must be called *exactly once* before anything that could
    /// call [`software_reset()`][crate::reset::software_reset]. Multiple calls
    /// will result in multiple `&mut` references to the same memory. Resetting
    /// before this is called may result in future calls returning a reference
    /// to uninitialized memory, which is unsound, even for types that lack
    /// invalid bit patterns. See the [Initialization Invariant section of the
    /// docs for `MaybeUninit`][init-invariant].
    ///
    /// Ideally, put the `static` and the `.get()` call alone in their own block
    /// near the beginning of the entry/main function, as is done in the `ram`
    /// example.
    ///
    /// [init-invariant]: https://doc.rust-lang.org/std/mem/union.MaybeUninit.html#initialization-invariant
    pub unsafe fn get(&self, init: T) -> &'static mut T {
        let inner = unsafe { &mut *self.inner.get() };

        if !matches!(get_reset_reason(), Some(SocResetReason::CoreSw)) {
            inner.write(init);
        }

        unsafe { inner.assume_init_mut() }
    }
}

impl<T> Default for Persistent<T> {
    fn default() -> Self {
        Self::new()
    }
}
