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

/// Creates a new `Peripherals` struct and its associated methods.
///
/// The macro has a few fields doing different things, in the form of
/// `second <= third (fourth)`.
/// - The second field is the name of the peripheral, as it appears in the `Peripherals` struct.
/// - The third field is the name of the peripheral as it appears in the PAC. This may be `virtual`
///   if the peripheral is not present in the PAC.
/// - The fourth field is an optional list of interrupts that can be bound to the peripheral.
macro_rules! peripherals {
    (
        peripherals: [
            $(
                $name:ident <= $from_pac:tt $interrupts:tt,
            )*
        ],
        unstable_peripherals: [
            $(
                $unstable_name:ident <= $unstable_from_pac:tt $unstable_interrupts:tt,
            )*
        ],
    ) => {
        /// The `Peripherals` struct provides access to all of the hardware peripherals on the chip.
        #[allow(non_snake_case)]
        pub struct Peripherals {
            $(
                #[doc = concat!("The ", stringify!($name), " peripheral.")]
                pub $name: $name<'static>,
            )*
            $(
                #[doc = concat!("The ", stringify!($unstable_name), " peripheral.")]
                #[doc = "**This API is marked as unstable** and is only available when the `unstable`
                        crate feature is enabled. This comes with no stability guarantees, and could be changed
                        or removed at any time."]
                #[cfg(feature = "unstable")]
                #[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
                pub $unstable_name: $unstable_name<'static>,

                #[doc = concat!("The ", stringify!($unstable_name), " peripheral.")]
                #[doc = "**This API is marked as unstable** and is only available when the `unstable`
                        crate feature is enabled. This comes with no stability guarantees, and could be changed
                        or removed at any time."]
                #[cfg(not(feature = "unstable"))]
                #[allow(unused)]
                pub(crate) $unstable_name: $unstable_name<'static>,
            )*
        }

        impl Peripherals {
            /// Returns all the peripherals *once*
            #[inline]
            pub(crate) fn take() -> Self {
                #[unsafe(no_mangle)]
                static mut _ESP_HAL_DEVICE_PERIPHERALS: bool = false;

                critical_section::with(|_| unsafe {
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
                        $(
                            $unstable_name: $unstable_name::steal(),
                        )*
                    }
                }
            }
        }
    };
}

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
                pub fn $bind(&self, handler: unsafe extern "C" fn() -> ()) {
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

include!(concat!(env!("OUT_DIR"), "/_generated_peris.rs"));
include!(concat!(env!("OUT_DIR"), "/_generated_gpio.rs"));

define_peripherals!();

for_each_peripheral! {
    (stable $name:ident <= $from_pac:tt $interrupts:tt) => {
        create_peripheral!($name <= $from_pac $interrupts);
    };

    (unstable $name:ident <= $from_pac:tt $interrupts:tt) => {
        create_peripheral!(#[instability::unstable] $name <= $from_pac $interrupts);
    };
}
