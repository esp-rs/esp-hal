//! Semaphores
//!
//! Semaphores are synchronization primitives that allow threads to coordinate their execution.
//! They are used to control access to a shared resource by limiting the number of threads that can
//! access it simultaneously.
//!
//! esp-radio sometimes mixes up semaphores and mutexes (FreeRTOS allows this), so this crate
//! exposes a single interface to work with both.
//!
//! ## Implementation
//!
//! Implement the `SemaphoreImplementation` trait for an object, and use the
//! `register_semaphore_implementation` to register that implementation for esp-radio.
//!
//! See the [`SemaphoreImplementation`] documentation for more information.
//!
//! ## Usage
//!
//! Users should use [`SemaphoreHandle`] to interact with semaphores created by the driver
//! implementation. Use [`SemaphoreKind`] to specify the type of semaphore or mutex to create.
//!
//! > Note that the only expected user of this crate is esp-radio.

use core::ptr::NonNull;

/// Pointer to an opaque semaphore created by the driver implementation.
pub type SemaphorePtr = NonNull<()>;

/// The type of semaphore or mutex to create.
pub enum SemaphoreKind {
    /// Counting semaphore.
    Counting { max: u32, initial: u32 },

    /// Non-recursive mutex.
    Mutex,

    /// Recursive mutex.
    RecursiveMutex,
}

unsafe extern "Rust" {
    fn esp_rtos_semaphore_create(kind: SemaphoreKind) -> SemaphorePtr;
    fn esp_rtos_semaphore_delete(semaphore: SemaphorePtr);

    fn esp_rtos_semaphore_take(semaphore: SemaphorePtr, timeout_us: Option<u32>) -> bool;
    fn esp_rtos_semaphore_give(semaphore: SemaphorePtr) -> bool;
    fn esp_rtos_semaphore_try_give_from_isr(
        semaphore: SemaphorePtr,
        higher_prio_task_waken: Option<&mut bool>,
    ) -> bool;
    fn esp_rtos_semaphore_current_count(semaphore: SemaphorePtr) -> u32;

    fn esp_rtos_semaphore_try_take(semaphore: SemaphorePtr) -> bool;
    fn esp_rtos_semaphore_try_take_from_isr(
        semaphore: SemaphorePtr,
        higher_prio_task_waken: Option<&mut bool>,
    ) -> bool;
}

/// A semaphore primitive.
///
/// The following snippet demonstrates the boilerplate necessary to implement a semaphore using the
/// `SemaphoreImplementation` trait:
///
/// ```rust,no_run
/// use esp_radio_rtos_driver::{
///     register_semaphore_implementation,
///     semaphore::{SemaphoreImplementation, SemaphoreKind, SemaphorePtr},
/// };
///
/// struct MySemaphore {
///     // Semaphore implementation details
/// }
///
/// impl SemaphoreImplementation for MySemaphore {
///     fn create(kind: SemaphoreKind) -> SemaphorePtr {
///         unimplemented!()
///     }
///
///     unsafe fn delete(semaphore: SemaphorePtr) {
///         unimplemented!()
///     }
///
///     unsafe fn take(semaphore: SemaphorePtr, timeout_us: Option<u32>) -> bool {
///         unimplemented!()
///     }
///
///     unsafe fn give(semaphore: SemaphorePtr) -> bool {
///         unimplemented!()
///     }
///
///     unsafe fn try_give_from_isr(
///         semaphore: SemaphorePtr,
///         higher_prio_task_waken: Option<&mut bool>,
///     ) -> bool {
///         unimplemented!()
///     }
///
///     unsafe fn current_count(semaphore: SemaphorePtr) -> u32 {
///         unimplemented!()
///     }
///
///     unsafe fn try_take(semaphore: SemaphorePtr) -> bool {
///         unimplemented!()
///     }
///
///     unsafe fn try_take_from_isr(
///         semaphore: SemaphorePtr,
///         higher_prio_task_waken: Option<&mut bool>,
///     ) -> bool {
///         unimplemented!()
///     }
/// }
///
/// register_semaphore_implementation!(MySemaphore);
/// ```
pub trait SemaphoreImplementation {
    /// Creates a new semaphore instance.
    ///
    /// `kind` specifies the type of semaphore to create.
    ///
    /// - `SemaphoreKind::Counting` should create counting, non-recursive semaphores/mutexes.
    /// - `SemaphoreKind::RecursiveMutex` should create recursive mutexes.
    fn create(kind: SemaphoreKind) -> SemaphorePtr;

    /// Deletes a semaphore instance.
    ///
    /// # Safety
    ///
    /// `semaphore` must be a pointer returned from [`Self::create`].
    unsafe fn delete(semaphore: SemaphorePtr);

    /// Increments the semaphore's counter.
    ///
    /// If a timeout is given, this function should block until either a semaphore could be taken,
    /// or the timeout has been reached. If no timeout is specified, the function should block
    /// indefinitely.
    ///
    /// Recursive mutexes can be repeatedly taken by the same task.
    ///
    /// The timeout is specified in microseconds.
    ///
    /// This function returns `true` if the semaphore was taken, `false` if the timeout was reached.
    ///
    /// # Safety
    ///
    /// `semaphore` must be a pointer returned from [`Self::create`].
    unsafe fn take(semaphore: SemaphorePtr, timeout_us: Option<u32>) -> bool;

    /// Increments the semaphore's counter.
    ///
    /// This function returns `true` if the semaphore was given, `false` if the counter is at
    /// its maximum.
    ///
    /// Recursive mutexes can not be given by a task other than the one that first locked it.
    ///
    /// # Safety
    ///
    /// `semaphore` must be a pointer returned from [`Self::create`].
    unsafe fn give(semaphore: SemaphorePtr) -> bool;

    /// Attempts to increment the semaphore's counter from an ISR.
    ///
    /// This function returns `true` if the semaphore was given, `false` if the counter is at
    /// its maximum.
    ///
    /// The `higher_prio_task_waken` parameter is an optional mutable reference to a boolean flag.
    /// If the flag is `Some`, the implementation may set it to `true` to request a context switch.
    ///
    /// # Safety
    ///
    /// `semaphore` must be a pointer returned from [`Self::create`].
    unsafe fn try_give_from_isr(
        semaphore: SemaphorePtr,
        higher_prio_task_waken: Option<&mut bool>,
    ) -> bool;

    /// Returns the semaphore's current counter value.
    ///
    /// # Safety
    ///
    /// `semaphore` must be a pointer returned from [`Self::create`].
    unsafe fn current_count(semaphore: SemaphorePtr) -> u32;

    /// Attempts to decrement the semaphore's counter.
    ///
    /// If the counter is zero, this function must immediately return `false`.
    ///
    /// # Safety
    ///
    /// `semaphore` must be a pointer returned from [`Self::create`].
    unsafe fn try_take(semaphore: SemaphorePtr) -> bool;

    /// Attempts to decrement the semaphore's counter from an ISR.
    ///
    /// If the counter is zero, this function must immediately return `false`.
    ///
    /// The `higher_prio_task_waken` parameter is an optional mutable reference to a boolean flag.
    /// If the flag is `Some`, the implementation may set it to `true` to request a context switch.
    ///
    /// # Safety
    ///
    /// `semaphore` must be a pointer returned from [`Self::create`].
    unsafe fn try_take_from_isr(
        semaphore: SemaphorePtr,
        higher_prio_task_waken: Option<&mut bool>,
    ) -> bool;
}

#[macro_export]
macro_rules! register_semaphore_implementation {
    ($t: ty) => {
        #[unsafe(no_mangle)]
        #[inline]
        fn esp_rtos_semaphore_create(
            kind: $crate::semaphore::SemaphoreKind,
        ) -> $crate::semaphore::SemaphorePtr {
            <$t as $crate::semaphore::SemaphoreImplementation>::create(kind)
        }

        #[unsafe(no_mangle)]
        #[inline]
        fn esp_rtos_semaphore_delete(semaphore: $crate::semaphore::SemaphorePtr) {
            unsafe { <$t as $crate::semaphore::SemaphoreImplementation>::delete(semaphore) }
        }

        #[unsafe(no_mangle)]
        #[inline]
        fn esp_rtos_semaphore_take(
            semaphore: $crate::semaphore::SemaphorePtr,
            timeout_us: Option<u32>,
        ) -> bool {
            unsafe {
                <$t as $crate::semaphore::SemaphoreImplementation>::take(semaphore, timeout_us)
            }
        }

        #[unsafe(no_mangle)]
        #[inline]
        fn esp_rtos_semaphore_give(semaphore: $crate::semaphore::SemaphorePtr) -> bool {
            unsafe { <$t as $crate::semaphore::SemaphoreImplementation>::give(semaphore) }
        }

        #[unsafe(no_mangle)]
        #[inline]
        fn esp_rtos_semaphore_try_give_from_isr(
            semaphore: $crate::semaphore::SemaphorePtr,
            higher_prio_task_waken: Option<&mut bool>,
        ) -> bool {
            unsafe {
                <$t as $crate::semaphore::SemaphoreImplementation>::try_give_from_isr(
                    semaphore,
                    higher_prio_task_waken,
                )
            }
        }

        #[unsafe(no_mangle)]
        #[inline]
        fn esp_rtos_semaphore_current_count(semaphore: $crate::semaphore::SemaphorePtr) -> u32 {
            unsafe { <$t as $crate::semaphore::SemaphoreImplementation>::current_count(semaphore) }
        }

        #[unsafe(no_mangle)]
        #[inline]
        fn esp_rtos_semaphore_try_take(semaphore: $crate::semaphore::SemaphorePtr) -> bool {
            unsafe { <$t as $crate::semaphore::SemaphoreImplementation>::try_take(semaphore) }
        }

        #[unsafe(no_mangle)]
        #[inline]
        fn esp_rtos_semaphore_try_take_from_isr(
            semaphore: $crate::semaphore::SemaphorePtr,
            higher_prio_task_waken: Option<&mut bool>,
        ) -> bool {
            unsafe {
                <$t as $crate::semaphore::SemaphoreImplementation>::try_take_from_isr(
                    semaphore,
                    higher_prio_task_waken,
                )
            }
        }
    };
}

/// Semaphore handle.
///
/// This handle is used to interact with semaphores created by the driver implementation.
#[repr(transparent)]
pub struct SemaphoreHandle(SemaphorePtr);
impl SemaphoreHandle {
    /// Creates a new semaphore instance.
    ///
    /// `kind` specifies the type of semaphore to create.
    ///
    /// - Use `SemaphoreKind::Counting` to create counting semaphores and non-recursive mutexes.
    /// - Use `SemaphoreKind::RecursiveMutex` to create recursive mutexes.
    #[inline]
    pub fn new(kind: SemaphoreKind) -> Self {
        let ptr = unsafe { esp_rtos_semaphore_create(kind) };
        Self(ptr)
    }

    /// Converts this object into a pointer without dropping it.
    #[inline]
    pub fn leak(self) -> SemaphorePtr {
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
    pub unsafe fn from_ptr(ptr: SemaphorePtr) -> Self {
        Self(ptr)
    }

    /// Creates a reference to this object from a leaked pointer.
    ///
    /// This function is used in the esp-radio code to interact with the semaphore.
    ///
    /// # Safety
    ///
    /// - The caller must only use pointers created using [`Self::leak`].
    #[inline]
    pub unsafe fn ref_from_ptr(ptr: &SemaphorePtr) -> &Self {
        unsafe { core::mem::transmute(ptr) }
    }

    /// Increments the semaphore's counter.
    ///
    /// If a timeout is given, this function blocks until either a semaphore could be taken, or the
    /// timeout has been reached. If no timeout is given, this function blocks until the operation
    /// succeeds.
    ///
    /// This function returns `true` if the semaphore was taken, `false` if the timeout was reached.
    #[inline]
    pub fn take(&self, timeout_us: Option<u32>) -> bool {
        unsafe { esp_rtos_semaphore_take(self.0, timeout_us) }
    }

    /// Increments the semaphore's counter.
    ///
    /// This function returns `true` if the semaphore was given, `false` if the counter is at its
    /// maximum.
    #[inline]
    pub fn give(&self) -> bool {
        unsafe { esp_rtos_semaphore_give(self.0) }
    }

    /// Attempts to increment the semaphore's counter from an ISR.
    ///
    /// If the counter is at its maximum, this function returns `false`.
    ///
    /// If the flag is `Some`, the implementation may set it to `true` to request a context switch.
    #[inline]
    pub fn try_give_from_isr(&self, higher_prio_task_waken: Option<&mut bool>) -> bool {
        unsafe { esp_rtos_semaphore_try_give_from_isr(self.0, higher_prio_task_waken) }
    }

    /// Returns the current counter value.
    #[inline]
    pub fn current_count(&self) -> u32 {
        unsafe { esp_rtos_semaphore_current_count(self.0) }
    }

    /// Attempts to decrement the semaphore's counter.
    ///
    /// If the counter is zero, this function returns `false`.
    #[inline]
    pub fn try_take(&self) -> bool {
        unsafe { esp_rtos_semaphore_try_take(self.0) }
    }

    /// Attempts to decrement the semaphore's counter from an ISR.
    ///
    /// If the counter is zero, this function returns `false`.
    ///
    /// If a higher priority task is woken up by this operation, the `higher_prio_task_waken` flag
    /// is set to `true`.
    #[inline]
    pub fn try_take_from_isr(&self, higher_prio_task_waken: Option<&mut bool>) -> bool {
        unsafe { esp_rtos_semaphore_try_take_from_isr(self.0, higher_prio_task_waken) }
    }
}

impl Drop for SemaphoreHandle {
    #[inline]
    fn drop(&mut self) {
        unsafe { esp_rtos_semaphore_delete(self.0) };
    }
}
