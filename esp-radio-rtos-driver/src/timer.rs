//! Timers (callbacks scheduled to run in the future)
//!
//! ## Implementation
//!
//! Implement the `TimerImplementation` trait for an object, and use the
//! `register_timer_implementation` to register that implementation for esp-radio.
//!
//! See the [`TimerImplementation`] documentation for more information.
//!
//! ## Usage
//!
//! Users should use [`TimerHandle`] to interact with timers created by the driver implementation.
//!
//! > Note that the only expected user of this crate is esp-radio.

use core::{ffi::c_void, ptr::NonNull};

/// Pointer to an opaque timer created by the driver implementation.
pub type TimerPtr = NonNull<()>;

unsafe extern "Rust" {
    fn esp_rtos_timer_create(
        function: unsafe extern "C" fn(*mut c_void),
        data: *mut c_void,
    ) -> TimerPtr;
    fn esp_rtos_timer_delete(timer: TimerPtr);

    fn esp_rtos_timer_arm(timer: TimerPtr, timeout: u64, periodic: bool);
    fn esp_rtos_timer_is_active(timer: TimerPtr) -> bool;
    fn esp_rtos_timer_disarm(timer: TimerPtr);
}

/// A timer implementation.
///
/// The following snippet demonstrates the boilerplate necessary to implement a timer using the
/// `TimerImplementation` trait:
///
/// ```rust,no_run
/// use esp_radio_rtos_driver::{
///     register_timer_implementation,
///     timer::{TimerImplementation, TimerPtr},
/// };
///
/// struct MyTimer {
///     // Timer implementation details
/// }
///
/// impl TimerImplementation for MyTimer {
///     fn create(function: unsafe extern "C" fn(*mut c_void), data: *mut c_void) -> TimerPtr {
///         unimplemented!()
///     }
///
///     unsafe fn delete(mutex: MutexPtr) {
///         unimplemented!()
///     }
///
///     unsafe fn arm(timer: TimerPtr, timeout: u64, periodic: bool) {
///         unimplemented!()
///     }
///
///     unsafe fn is_active(timer: TimerPtr) -> bool {
///         unimplemented!()
///     }
///
///     unsafe fn disarm(timer: TimerPtr) -> bool {
///         unimplemented!()
///     }
/// }
///
/// register_timer_implementation!(MyTimer);
/// ```
pub trait TimerImplementation {
    /// Creates a new timer instance from the given callback.
    fn create(function: unsafe extern "C" fn(*mut c_void), data: *mut c_void) -> TimerPtr;

    /// Deletes a timer instance.
    ///
    /// # Safety
    ///
    /// `timer` must be a pointer returned from [`Self::create`].
    unsafe fn delete(timer: TimerPtr);

    /// Configures the timer to be triggered after the given timeout.
    ///
    /// The timeout is specified in microsecond. If the timer is set to be periodic,
    /// the timer will be triggered with a constant frequency.
    ///
    /// # Safety
    ///
    /// `timer` must be a pointer returned from [`Self::create`].
    unsafe fn arm(timer: TimerPtr, timeout: u64, periodic: bool);

    /// Checks if the timer is currently active.
    ///
    /// # Safety
    ///
    /// `timer` must be a pointer returned from [`Self::create`].
    unsafe fn is_active(timer: TimerPtr) -> bool;

    /// Stops the timer.
    ///
    /// # Safety
    ///
    /// `timer` must be a pointer returned from [`Self::create`].
    unsafe fn disarm(timer: TimerPtr);
}

#[macro_export]
macro_rules! register_timer_implementation {
    ($t: ty) => {
        #[unsafe(no_mangle)]
        #[inline]
        fn esp_rtos_timer_create(
            function: unsafe extern "C" fn(*mut ::core::ffi::c_void),
            data: *mut ::core::ffi::c_void,
        ) -> $crate::timer::TimerPtr {
            <$t as $crate::timer::TimerImplementation>::create(function, data)
        }

        #[unsafe(no_mangle)]
        #[inline]
        fn esp_rtos_timer_delete(timer: $crate::timer::TimerPtr) {
            unsafe { <$t as $crate::timer::TimerImplementation>::delete(timer) }
        }

        #[unsafe(no_mangle)]
        #[inline]
        fn esp_rtos_timer_arm(timer: $crate::timer::TimerPtr, timeout: u64, periodic: bool) {
            unsafe { <$t as $crate::timer::TimerImplementation>::arm(timer, timeout, periodic) }
        }

        #[unsafe(no_mangle)]
        #[inline]
        fn esp_rtos_timer_is_active(timer: $crate::timer::TimerPtr) -> bool {
            unsafe { <$t as $crate::timer::TimerImplementation>::is_active(timer) }
        }

        #[unsafe(no_mangle)]
        #[inline]
        fn esp_rtos_timer_disarm(timer: $crate::timer::TimerPtr) {
            unsafe { <$t as $crate::timer::TimerImplementation>::disarm(timer) }
        }
    };
}

/// A timer handle.
///
/// This handle is used to interact with timers created by the driver implementation.
#[repr(transparent)]
pub struct TimerHandle(TimerPtr);
impl TimerHandle {
    /// Creates a new timer instance from the given callback.
    ///
    /// # Safety
    ///
    /// - The callback and its data must be valid for the lifetime of the timer.
    /// - The callback and its data need to be able to be sent across threads.
    #[inline]
    pub unsafe fn new(function: unsafe extern "C" fn(*mut c_void), data: *mut c_void) -> Self {
        Self(unsafe { esp_rtos_timer_create(function, data) })
    }

    /// Converts this object into a pointer without dropping it.
    #[inline]
    pub fn leak(self) -> TimerPtr {
        let ptr = self.0;
        core::mem::forget(self);
        ptr
    }

    /// Recovers the object from a leaked pointer.
    ///
    /// # Safety
    ///
    /// - The caller must only use pointers created using [`Self::leak`].
    /// - The caller must ensure the pointer is not shared.
    #[inline]
    pub unsafe fn from_ptr(ptr: TimerPtr) -> Self {
        Self(ptr)
    }

    /// Creates a reference to this object from a leaked pointer.
    ///
    /// This function is used in the esp-radio code to interact with the timer.
    ///
    /// # Safety
    ///
    /// - The caller must only use pointers created using [`Self::leak`].
    #[inline]
    pub unsafe fn ref_from_ptr(ptr: &TimerPtr) -> &Self {
        unsafe { core::mem::transmute(ptr) }
    }

    /// Configures the timer to be triggered after the given timeout.
    ///
    /// The timeout is specified in microsecond. If the timer is set to be periodic,
    /// the timer will be triggered with a constant frequency.
    #[inline]
    pub fn arm(&self, timeout: u64, periodic: bool) {
        unsafe { esp_rtos_timer_arm(self.0, timeout, periodic) }
    }

    /// Checks if the timer is currently active.
    #[inline]
    pub fn is_active(&self) -> bool {
        unsafe { esp_rtos_timer_is_active(self.0) }
    }

    /// Stops the timer.
    #[inline]
    pub fn disarm(&self) {
        unsafe { esp_rtos_timer_disarm(self.0) }
    }
}

impl Drop for TimerHandle {
    #[inline]
    fn drop(&mut self) {
        unsafe { esp_rtos_timer_delete(self.0) };
    }
}
