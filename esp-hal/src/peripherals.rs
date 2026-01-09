//! # Peripheral Instances
//!
//! This module creates singleton instances for each of the various peripherals,
//! and re-exports them to allow users to access and use them in their
//! applications.
//!
//! Should be noted that that the module also re-exports the [Interrupt] enum
//! from the PAC, allowing users to handle interrupts associated with these
//! peripherals.

// We need to export this for users to use
#[doc(hidden)]
pub use pac::Interrupt;

pub(crate) use crate::soc::pac;

/// Macro to create a peripheral structure.
macro_rules! create_peripheral {
    ($(#[$attr:meta])? $name:ident <= virtual ($($interrupt:ident: { $bind:ident, $enable:ident, $disable:ident }),*)) => {
        $(#[$attr])?
        #[derive(Debug)]
        #[cfg_attr(feature = "defmt", derive(defmt::Format))]
        #[non_exhaustive]
        #[allow(non_camel_case_types, clippy::upper_case_acronyms)]
        #[doc = concat!(stringify!($name), " peripheral singleton")]
        pub struct $name<'a> {
            _marker: core::marker::PhantomData<&'a mut ()>,
        }

        impl $name<'_> {
            /// Unsafely create an instance of this peripheral out of thin air.
            ///
            /// # Safety
            ///
            /// You must ensure that you're only using one instance of this type at a time.
            #[inline]
            pub unsafe fn steal() -> Self {
                Self {
                    _marker: core::marker::PhantomData,
                }
            }

            /// Unsafely clone this peripheral reference.
            ///
            /// # Safety
            ///
            /// You must ensure that you're only using one instance of this type at a time.
            #[inline]
            #[allow(dead_code)]
            pub unsafe fn clone_unchecked(&self) -> Self {
                unsafe { Self::steal() }
            }

            /// Creates a new peripheral reference with a shorter lifetime.
            ///
            /// Use this method if you would like to keep working with the peripheral after
            /// you dropped the driver that consumes this.
            #[inline]
            #[allow(dead_code)]
            pub fn reborrow(&mut self) -> $name<'_> {
                unsafe { self.clone_unchecked() }
            }

            $(
                /// Binds an interrupt handler to the corresponding interrupt for this peripheral.
                #[instability::unstable]
                pub fn $bind(&self, handler: $crate::interrupt::IsrCallback) {
                    unsafe { $crate::interrupt::bind_interrupt($crate::peripherals::Interrupt::$interrupt, handler); }
                }

                /// Disables the interrupt handler
                #[instability::unstable]
                pub fn $disable(&self) {
                    for core in $crate::system::Cpu::other() {
                        $crate::interrupt::disable(core, $crate::peripherals::Interrupt::$interrupt);
                    }
                }

                /// Enables the interrupt handler on the given core
                #[instability::unstable]
                pub fn $enable(&self, priority: $crate::interrupt::Priority) {
                    unwrap!($crate::interrupt::enable($crate::peripherals::Interrupt::$interrupt, priority));
                }
            )*
        }

        impl $crate::private::Sealed for $name<'_> {}
    };

    ($(#[$attr:meta])? $name:ident <= $base:ident $interrupts:tt) => {
        create_peripheral!($(#[$attr])? $name <= virtual $interrupts);

        impl $name<'_> {
            #[doc = r"Pointer to the register block"]
            #[instability::unstable]
            pub const PTR: *const <pac::$base as core::ops::Deref>::Target = pac::$base::PTR;

            #[doc = r"Return the pointer to the register block"]
            #[inline(always)]
            #[instability::unstable]
            pub const fn ptr() -> *const <pac::$base as core::ops::Deref>::Target {
                pac::$base::PTR
            }

            #[doc = r"Return a reference to the register block"]
            #[inline(always)]
            #[instability::unstable]
            pub const fn regs<'a>() -> &'a <pac::$base as core::ops::Deref>::Target {
                unsafe { &*Self::PTR }
            }

            #[doc = r"Return a reference to the register block"]
            #[inline(always)]
            #[instability::unstable]
            pub fn register_block(&self) -> &<pac::$base as core::ops::Deref>::Target {
                unsafe { &*Self::PTR }
            }
        }
    };
}

for_each_peripheral! {
    // Define stable peripheral singletons
    (@peri_type $name:ident <= $from_pac:tt $interrupts:tt) => {
        create_peripheral!($name <= $from_pac $interrupts);
    };

    // Define unstable peripheral singletons
    (@peri_type $name:ident <= $from_pac:tt $interrupts:tt (unstable)) => {
        create_peripheral!(#[instability::unstable] $name <= $from_pac $interrupts);
    };

    // Define the Peripherals struct
    (singletons $( ($name:ident $(($unstable:ident))?) ),*) => {
        // We need a way to ignore the "unstable" marker, but macros can't generate attributes or struct fields.
        // The solution is printing an empty doc comment.
        macro_rules! ignore { ($any:tt) => {""} }

        /// The `Peripherals` struct provides access to all of the hardware peripherals on the chip.
        #[allow(non_snake_case)]
        #[non_exhaustive]
        pub struct Peripherals {
            $(
                // This is a bit hairy, but non-macro attributes are not allowed on struct fields. We work
                // around this by excluding code with the `$()?` optional macro syntax and an "unstable" marker
                // in the source data. The marker itself is passed through the `ignore` macro so that it doesn't
                // appear in the generated code.
                //
                // The code can end up looking two ways:
                //
                // - Without `unstable` we just generate the field:
                // ```
                // #[attributes]
                // pub PERI: PERI<'static>,
                // ```
                //
                // - With `unstable` we're basically emulating what `instability::unstable` would do:
                // ```
                // #[attributes]
                // #[cfg(feature = "unstable")]
                // pub PERI: PERI<'static>,
                //
                // #[attributes]
                // #[cfg(not(feature = "unstable"))]
                // pub(crate) PERI: PERI<'static>,
                // ```
                #[doc = concat!("The ", stringify!($name), " peripheral.")]
                $(
                    #[doc = "**This API is marked as unstable** and is only available when the `unstable`
                            crate feature is enabled. This comes with no stability guarantees, and could be changed
                            or removed at any time."]
                    #[doc = ignore!($unstable)]
                    #[cfg(feature = "unstable")]
                    #[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
                )?
                pub $name: $name<'static>,

                $(
                    #[doc = concat!("The ", stringify!($unstable_name), " peripheral.")]
                    #[doc = "**This API is marked as unstable** and is only available when the `unstable`
                            crate feature is enabled. This comes with no stability guarantees, and could be changed
                            or removed at any time."]
                    #[doc = ignore!($unstable)]
                    #[cfg(not(feature = "unstable"))]
                    #[allow(unused)]
                    pub(crate) $name: $name<'static>,
                )?
            )*
        }

        impl Peripherals {
            /// Returns all the peripherals *once*.
            #[inline]
            #[cfg(feature = "rt")]
            pub(crate) fn take() -> Self {
                #[unsafe(no_mangle)]
                static mut _ESP_HAL_DEVICE_PERIPHERALS: bool = false;

                crate::ESP_HAL_LOCK.lock(|| unsafe {
                    if _ESP_HAL_DEVICE_PERIPHERALS {
                        panic!("init called more than once!")
                    }
                    _ESP_HAL_DEVICE_PERIPHERALS = true;
                    Self::steal()
                })
            }

            /// Unsafely create an instance of this peripheral out of thin air.
            ///
            /// # Safety
            ///
            /// You must ensure that you're only using one instance of this type at a time.
            #[inline]
            pub unsafe fn steal() -> Self {
                unsafe {
                    Self {
                        $(
                            $name: $name::steal(),
                        )*
                    }
                }
            }
        }
    };
}
