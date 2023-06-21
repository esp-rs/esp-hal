//! Exclusive peripheral access

use core::{
    marker::PhantomData,
    ops::{Deref, DerefMut},
};

/// An exclusive reference to a peripheral.
///
/// This is functionally the same as a `&'a mut T`. The reason for having a
/// dedicated struct is memory efficiency:
///
/// Peripheral singletons are typically either zero-sized (for concrete
/// peripehrals like `PA9` or `Spi4`) or very small (for example `AnyPin` which
/// is 1 byte). However `&mut T` is always 4 bytes for 32-bit targets, even if T
/// is zero-sized. PeripheralRef stores a copy of `T` instead, so it's the same
/// size.
///
/// but it is the size of `T` not the size
/// of a pointer. This is useful if T is a zero sized type.
pub struct PeripheralRef<'a, T> {
    inner: T,
    _lifetime: PhantomData<&'a mut T>,
}

impl<'a, T> PeripheralRef<'a, T> {
    #[inline]
    pub fn new(inner: T) -> Self {
        Self {
            inner,
            _lifetime: PhantomData,
        }
    }

    /// Unsafely clone (duplicate) a peripheral singleton.
    ///
    /// # Safety
    ///
    /// This returns an owned clone of the peripheral. You must manually ensure
    /// only one copy of the peripheral is in use at a time. For example, don't
    /// create two SPI drivers on `SPI1`, because they will "fight" each other.
    ///
    /// You should strongly prefer using `reborrow()` instead. It returns a
    /// `PeripheralRef` that borrows `self`, which allows the borrow checker
    /// to enforce this at compile time.
    pub unsafe fn clone_unchecked(&mut self) -> PeripheralRef<'a, T>
    where
        T: Peripheral<P = T>,
    {
        PeripheralRef::new(self.inner.clone_unchecked())
    }

    /// Reborrow into a "child" PeripheralRef.
    ///
    /// `self` will stay borrowed until the child PeripheralRef is dropped.
    pub fn reborrow(&mut self) -> PeripheralRef<'_, T>
    where
        T: Peripheral<P = T>,
    {
        // safety: we're returning the clone inside a new PeripheralRef that borrows
        // self, so user code can't use both at the same time.
        PeripheralRef::new(unsafe { self.inner.clone_unchecked() })
    }

    /// Map the inner peripheral using `Into`.
    ///
    /// This converts from `PeripheralRef<'a, T>` to `PeripheralRef<'a, U>`,
    /// using an `Into` impl to convert from `T` to `U`.
    ///
    /// For example, this can be useful to degrade GPIO pins: converting from
    /// PeripheralRef<'a, PB11>` to `PeripheralRef<'a, AnyPin>`.
    #[inline]
    pub fn map_into<U>(self) -> PeripheralRef<'a, U>
    where
        T: Into<U>,
    {
        PeripheralRef {
            inner: self.inner.into(),
            _lifetime: PhantomData,
        }
    }
}

impl<'a, T> Deref for PeripheralRef<'a, T> {
    type Target = T;

    #[inline]
    fn deref(&self) -> &Self::Target {
        &self.inner
    }
}

impl<'a, T> DerefMut for PeripheralRef<'a, T> {
    #[inline]
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.inner
    }
}

/// Trait for any type that can be used as a peripheral of type `P`.
///
/// This is used in driver constructors, to allow passing either owned
/// peripherals (e.g. `TWISPI0`), or borrowed peripherals (e.g. `&mut TWISPI0`).
///
/// For example, if you have a driver with a constructor like this:
///
/// ```ignore
/// impl<'d, T: Instance> Twim<'d, T> {
///     pub fn new(
///         twim: impl Peripheral<P = T> + 'd,
///         irq: impl Peripheral<P = T::Interrupt> + 'd,
///         sda: impl Peripheral<P = impl GpioPin> + 'd,
///         scl: impl Peripheral<P = impl GpioPin> + 'd,
///         config: Config,
///     ) -> Self { .. }
/// }
/// ```
///
/// You may call it with owned peripherals, which yields an instance that can
/// live forever (`'static`):
///
/// ```ignore
/// let mut twi: Twim<'static, ...> = Twim::new(p.TWISPI0, irq, p.P0_03, p.P0_04, config);
/// ```
///
/// Or you may call it with borrowed peripherals, which yields an instance that
/// can only live for as long as the borrows last:
///
/// ```ignore
/// let mut twi: Twim<'_, ...> = Twim::new(&mut p.TWISPI0, &mut irq, &mut p.P0_03, &mut p.P0_04, config);
/// ```
///
/// # Implementation details, for HAL authors
///
/// When writing a HAL, the intended way to use this trait is to take `impl
/// Peripheral<P = ..>` in the HAL's public API (such as driver constructors),
/// calling `.into_ref()` to obtain a `PeripheralRef`, and storing that in the
/// driver struct.
///
/// `.into_ref()` on an owned `T` yields a `PeripheralRef<'static, T>`.
/// `.into_ref()` on an `&'a mut T` yields a `PeripheralRef<'a, T>`.
pub trait Peripheral: Sized + sealed::Sealed {
    /// Peripheral singleton type
    type P;

    /// Unsafely clone (duplicate) a peripheral singleton.
    ///
    /// # Safety
    ///
    /// This returns an owned clone of the peripheral. You must manually ensure
    /// only one copy of the peripheral is in use at a time. For example, don't
    /// create two SPI drivers on `SPI1`, because they will "fight" each other.
    ///
    /// You should strongly prefer using `into_ref()` instead. It returns a
    /// `PeripheralRef`, which allows the borrow checker to enforce this at
    /// compile time.
    unsafe fn clone_unchecked(&mut self) -> Self::P;

    /// Convert a value into a `PeripheralRef`.
    ///
    /// When called on an owned `T`, yields a `PeripheralRef<'static, T>`.
    /// When called on an `&'a mut T`, yields a `PeripheralRef<'a, T>`.
    #[inline]
    fn into_ref<'a>(mut self) -> PeripheralRef<'a, Self::P>
    where
        Self: 'a,
    {
        PeripheralRef::new(unsafe { self.clone_unchecked() })
    }
}

impl<T> Peripheral for &mut T
where
    T: Peripheral<P = T>,
{
    type P = T;

    unsafe fn clone_unchecked(&mut self) -> Self::P {
        T::clone_unchecked(self)
    }
}

impl<T> sealed::Sealed for &mut T where T: sealed::Sealed {}

pub(crate) mod sealed {
    pub trait Sealed {}
}

mod peripheral_macros {
    #[doc(hidden)]
    #[macro_export]
    macro_rules! peripherals {
        ($($(#[$cfg:meta])? $name:ident => $from_pac:tt),*$(,)?) => {

            /// Contains the generated peripherals which implement [`Peripheral`]
            mod peripherals {
                pub use super::pac::*;
                $(
                    crate::create_peripheral!($(#[$cfg])? $name => $from_pac);
                )*
            }

            #[allow(non_snake_case)]
            pub struct Peripherals {
                $(
                    $(#[$cfg])?
                    pub $name: peripherals::$name,
                )*
            }

            impl Peripherals {
                /// Returns all the peripherals *once*
                #[inline]
                pub fn take() -> Self {

                    #[no_mangle]
                    static mut _ESP_HAL_DEVICE_PERIPHERALS: bool = false;

                    critical_section::with(|_| unsafe {
                        if _ESP_HAL_DEVICE_PERIPHERALS {
                            panic!("init called more than once!")
                        }
                        _ESP_HAL_DEVICE_PERIPHERALS = true;
                        Self::steal()
                    })
                }
            }

            impl Peripherals {
                /// Unsafely create an instance of this peripheral out of thin air.
                ///
                /// # Safety
                ///
                /// You must ensure that you're only using one instance of this type at a time.
                #[inline]
                pub unsafe fn steal() -> Self {
                    Self {
                        $(
                            $(#[$cfg])?
                            $name: peripherals::$name::steal(),
                        )*
                    }
                }
            }

            // expose the new structs
            $(
                pub use peripherals::$name;
            )*
        }
    }

    #[doc(hidden)]
    #[macro_export]
    macro_rules! into_ref {
        ($($name:ident),*) => {
            $(
                #[allow(unused_mut)]
                let mut $name = $name.into_ref();
            )*
        }
    }

    #[doc(hidden)]
    #[macro_export]
    macro_rules! create_peripheral {
        ($(#[$cfg:meta])? $name:ident => true) => {
            $(#[$cfg])?
            #[derive(Debug)]
            #[allow(non_camel_case_types)]
            pub struct $name { _inner: () }

            $(#[$cfg])?
            impl $name {
                /// Unsafely create an instance of this peripheral out of thin air.
                ///
                /// # Safety
                ///
                /// You must ensure that you're only using one instance of this type at a time.
                #[inline]
                pub unsafe fn steal() -> Self {
                    Self { _inner: () }
                }

                #[doc = r"Pointer to the register block"]
                pub const PTR: *const <super::pac::$name as core::ops::Deref>::Target = super::pac::$name::PTR;

                #[doc = r"Return the pointer to the register block"]
                #[inline(always)]
                pub const fn ptr() -> *const <super::pac::$name as core::ops::Deref>::Target {
                    super::pac::$name::PTR
                }
            }

            impl core::ops::Deref for $name {
                type Target = <super::pac::$name as core::ops::Deref>::Target;

                fn deref(&self) -> &Self::Target {
                    unsafe { &*Self::PTR }
                }
            }

            impl core::ops::DerefMut for $name {

                fn deref_mut(&mut self) -> &mut Self::Target {
                    unsafe { &mut *(Self::PTR as *mut _)  }
                }
            }

            impl crate::peripheral::Peripheral for $name {
                type P = $name;

                #[inline]
                unsafe fn clone_unchecked(&mut self) -> Self::P {
                    Self::steal()
                }
            }

            impl crate::peripheral::sealed::Sealed for $name {}
        };
        ($(#[$cfg:meta])? $name:ident => false) => {
            $(#[$cfg])?
            #[derive(Debug)]
            #[allow(non_camel_case_types)]
            pub struct $name { _inner: () }

            $(#[$cfg])?
            impl $name {
                /// Unsafely create an instance of this peripheral out of thin air.
                ///
                /// # Safety
                ///
                /// You must ensure that you're only using one instance of this type at a time.
                #[inline]
                pub unsafe fn steal() -> Self {
                    Self { _inner: () }
                }
            }

            impl crate::peripheral::Peripheral for $name {
                type P = $name;

                #[inline]
                unsafe fn clone_unchecked(&mut self) -> Self::P {
                    Self::steal()
                }
            }

            impl crate::peripheral::sealed::Sealed for $name {}
        }
    }
}
