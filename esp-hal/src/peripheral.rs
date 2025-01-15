//! # Exclusive peripheral access

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
/// peripherals like `SPI2` or `UART0`) or very small (for example `AnyPin`
/// which is 1 byte). However `&mut T` is always 4 bytes for 32-bit targets,
/// even if T is zero-sized. PeripheralRef stores a copy of `T` instead, so it's
/// the same size.
///
/// but it is the size of `T` not the size
/// of a pointer. This is useful if T is a zero sized type.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PeripheralRef<'a, T> {
    inner: T,
    _lifetime: PhantomData<&'a mut T>,
}

impl<'a, T> PeripheralRef<'a, T> {
    /// Create a new exclusive reference to a peripheral
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
    pub unsafe fn clone_unchecked(&self) -> PeripheralRef<'a, T>
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

    /// Transform the inner peripheral.
    ///
    /// This converts from `PeripheralRef<'a, T>` to `PeripheralRef<'a, U>`,
    /// using a user-provided impl to convert from `T` to `U`.
    #[inline]
    pub fn map<U>(self, transform: impl FnOnce(T) -> U) -> PeripheralRef<'a, U> {
        PeripheralRef {
            inner: transform(self.inner),
            _lifetime: PhantomData,
        }
    }

    /// Map the inner peripheral using `Into`.
    ///
    /// This converts from `PeripheralRef<'a, T>` to `PeripheralRef<'a, U>`,
    /// using an `Into` impl to convert from `T` to `U`.
    ///
    /// For example, this can be useful to degrade GPIO pins: converting from
    /// `PeripheralRef<'a, GpioPin<11>>` to `PeripheralRef<'a, AnyPin>`.
    #[inline]
    pub fn map_into<U>(self) -> PeripheralRef<'a, U>
    where
        T: Into<U>,
    {
        self.map(Into::into)
    }
}

impl<T> Deref for PeripheralRef<'_, T> {
    type Target = T;

    #[inline]
    fn deref(&self) -> &Self::Target {
        &self.inner
    }
}

impl<T> DerefMut for PeripheralRef<'_, T> {
    #[inline]
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.inner
    }
}

/// Trait for any type that can be used as a peripheral of type `P`.
///
/// This is used in driver constructors, to allow passing either owned
/// peripherals (e.g. `UART0`), or borrowed peripherals (e.g. `&mut UART0`).
///
/// For example, if you have a driver with a constructor like this:
///
/// ```rust, ignore
/// impl<'d, T> Uart<'d, T, Blocking> {
///     pub fn new<TX: PeripheralOutput, RX: PeripheralInput>(
///         uart: impl Peripheral<P = T> + 'd,
///         rx: impl Peripheral<P = RX> + 'd,
///         tx: impl Peripheral<P = TX> + 'd,
///     ) -> Result<Self, Error> {
///         Ok(Self { .. })
///     }
/// }
/// ```
///
/// You may call it with owned peripherals, which yields an instance that can
/// live forever (`'static`):
///
/// ```rust, ignore
/// let mut uart: Uart<'static, ...> = Uart::new(p.UART0, p.GPIO0, p.GPIO1);
/// ```
///
/// Or you may call it with borrowed peripherals, which yields an instance that
/// can only live for as long as the borrows last:
///
/// ```rust, ignore
/// let mut uart: Uart<'_, ...> = Uart::new(&mut p.UART0, &mut p.GPIO0, &mut p.GPIO1);
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
pub trait Peripheral: Sized + crate::private::Sealed {
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
    unsafe fn clone_unchecked(&self) -> Self::P;

    /// Convert a value into a `PeripheralRef`.
    ///
    /// When called on an owned `T`, yields a `PeripheralRef<'static, T>`.
    /// When called on an `&'a mut T`, yields a `PeripheralRef<'a, T>`.
    #[inline]
    fn into_ref<'a>(self) -> PeripheralRef<'a, Self::P>
    where
        Self: 'a,
    {
        PeripheralRef::new(unsafe { self.clone_unchecked() })
    }

    /// Map the peripheral using `Into`.
    ///
    /// This converts from `Peripheral<P = T>` to `Peripheral<P = U>`,
    /// using an `Into` impl to convert from `T` to `U`.
    #[inline]
    fn map_into<U>(self) -> U
    where
        Self::P: Into<U>,
        U: Peripheral<P = U>,
    {
        self.map(Into::into)
    }

    /// Map the peripheral using `Into`.
    ///
    /// This converts from `Peripheral<P = T>` to `Peripheral<P = U>`,
    /// using an `Into` impl to convert from `T` to `U`.
    #[inline]
    fn map<U>(self, transform: impl FnOnce(Self::P) -> U) -> U
    where
        U: Peripheral<P = U>,
    {
        transform(unsafe { self.clone_unchecked() })
    }
}

impl<T, P> Peripheral for &mut T
where
    T: Peripheral<P = P>,
{
    type P = P;

    unsafe fn clone_unchecked(&self) -> Self::P {
        T::clone_unchecked(self)
    }
}

impl<T> crate::private::Sealed for &mut T where T: crate::private::Sealed {}
impl<T> crate::private::Sealed for PeripheralRef<'_, T> where T: crate::private::Sealed {}

impl<T: Peripheral> Peripheral for PeripheralRef<'_, T> {
    type P = T::P;

    #[inline]
    unsafe fn clone_unchecked(&self) -> Self::P {
        T::clone_unchecked(self)
    }
}

mod peripheral_macros {
    /// Creates a new `Peripherals` struct and its associated methods.
    ///
    /// The macro has a few fields doing different things, in the form of
    /// `second <= third (fourth)`.
    /// - The second field is the name of the peripheral, as it appears in the
    ///   `Peripherals` struct.
    /// - The third field is the name of the peripheral as it appears in the
    ///   PAC. This may be `virtual` if the peripheral is not present in the
    ///   PAC.
    /// - The fourth field is an optional list of interrupts that can be bound
    ///   to the peripheral.
    #[doc(hidden)]
    #[macro_export]
    macro_rules! peripherals {
        (
            peripherals: [
                $(
                    $name:ident <= $from_pac:tt $(($($interrupt:ident),*))?
                ),* $(,)?
            ],
            unstable_peripherals: [
                $(
                    $unstable_name:ident <= $unstable_from_pac:tt $(($($unstable_interrupt:ident),*))?
                ),* $(,)?
            ],
            pins: [
                $( ( $pin:literal, $($pin_tokens:tt)* ) )*
            ],
            dma_channels: [
                $(
                    $channel_name:ident : $channel_ty:path
                ),* $(,)?
            ]
        ) => {


            /// Contains the generated peripherals which implement [`Peripheral`]
            mod peripherals {
                pub use super::pac::*;
                $(
                    $crate::create_peripheral!($name <= $from_pac);
                )*
                $(
                    $crate::create_peripheral!($unstable_name <= $unstable_from_pac);
                )*
            }

            pub(crate) mod gpio {
                $crate::gpio! {
                    $( ($pin, $($pin_tokens)* ) )*
                }
            }

            paste::paste! {
                /// The `Peripherals` struct provides access to all of the hardware peripherals on the chip.
                #[allow(non_snake_case)]
                pub struct Peripherals {
                    $(
                        #[doc = concat!("The ", stringify!($name), " peripheral.")]
                        pub $name: $name,
                    )*
                    $(
                        #[doc = concat!("The ", stringify!($unstable_name), " peripheral.")]
                        #[doc = "**This API is marked as unstable** and is only available when the `unstable`
                                crate feature is enabled. This comes with no stability guarantees, and could be changed
                                or removed at any time."]
                        #[cfg(any(doc, feature = "unstable"))]
                        #[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
                        pub $unstable_name: $unstable_name,

                        #[doc = concat!("The ", stringify!($unstable_name), " peripheral.")]
                        #[doc = "**This API is marked as unstable** and is only available when the `unstable`
                                crate feature is enabled. This comes with no stability guarantees, and could be changed
                                or removed at any time."]
                        #[cfg(not(any(doc, feature = "unstable")))]
                        #[allow(unused)]
                        pub(crate) $unstable_name: $unstable_name,
                    )*

                    $(
                        #[doc = concat!("GPIO", stringify!($pin))]
                        pub [<GPIO $pin>]: $crate::gpio::GpioPin<$pin>,
                    )*

                    $(
                        #[doc = concat!(stringify!($channel_name), " DMA channel.")]
                        pub $channel_name: $crate::dma::$channel_ty,
                    )*
                }

                impl Peripherals {
                    /// Returns all the peripherals *once*
                    #[inline]
                    pub(crate) fn take() -> Self {
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

                    /// Unsafely create an instance of this peripheral out of thin air.
                    ///
                    /// # Safety
                    ///
                    /// You must ensure that you're only using one instance of this type at a time.
                    #[inline]
                    pub unsafe fn steal() -> Self {
                        Self {
                            $(
                                $name: $name::steal(),
                            )*
                            $(
                                $unstable_name: $unstable_name::steal(),
                            )*

                            $(
                                [<GPIO $pin>]: $crate::gpio::GpioPin::<$pin>::steal(),
                            )*

                            $(
                                $channel_name: $crate::dma::$channel_ty::steal(),
                            )*
                        }
                    }
                }
            }

            // expose the new structs, implement interrupt binder

            $(
                pub use peripherals::$name;
                $(
                    impl peripherals::$name {
                        $(
                            paste::paste!{
                                /// Binds an interrupt handler to the corresponding interrupt for this peripheral.
                                #[instability::unstable]
                                pub fn [<bind_ $interrupt:lower _interrupt >](&mut self, handler: unsafe extern "C" fn() -> ()) {
                                    unsafe { $crate::interrupt::bind_interrupt($crate::peripherals::Interrupt::$interrupt, handler); }
                                }
                            }
                        )*
                    }
                )*
            )*

            $(
                pub use peripherals::$unstable_name;
                $(
                    impl peripherals::$unstable_name {
                        $(
                            paste::paste!{
                                /// Binds an interrupt handler to the corresponding interrupt for this peripheral.
                                #[instability::unstable]
                                pub fn [<bind_ $unstable_interrupt:lower _interrupt >](&mut self, handler: unsafe extern "C" fn() -> ()) {
                                    unsafe { $crate::interrupt::bind_interrupt($crate::peripherals::Interrupt::$unstable_interrupt, handler); }
                                }
                            }
                        )*
                    }
                )*
            )*
        };
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
    macro_rules! into_mapped_ref {
        ($($name:ident),*) => {
            $(
                #[allow(unused_mut)]
                let mut $name = $name.map_into().into_ref();
            )*
        }
    }

    #[doc(hidden)]
    #[macro_export]
    /// Macro to create a peripheral structure.
    macro_rules! create_peripheral {
        ($name:ident <= virtual) => {
            #[derive(Debug)]
            #[cfg_attr(feature = "defmt", derive(defmt::Format))]
            #[non_exhaustive]
            #[allow(non_camel_case_types, clippy::upper_case_acronyms)]
            #[doc = concat!(stringify!($name), " peripheral singleton")]
            pub struct $name;

            impl $name {
                /// Unsafely create an instance of this peripheral out of thin air.
                ///
                /// # Safety
                ///
                /// You must ensure that you're only using one instance of this type at a time.
                #[inline]
                pub unsafe fn steal() -> Self {
                    Self
                }
            }

            impl $crate::peripheral::Peripheral for $name {
                type P = $name;

                #[inline]
                unsafe fn clone_unchecked(&self) -> Self::P {
                    Self::steal()
                }
            }

            impl $crate::private::Sealed for $name {}
        };

        ($name:ident <= $base:ident) => {
            $crate::create_peripheral!($name <= virtual);

            impl $name {
                #[doc = r"Pointer to the register block"]
                #[instability::unstable]
                pub const PTR: *const <super::pac::$base as core::ops::Deref>::Target = super::pac::$base::PTR;

                #[doc = r"Return the pointer to the register block"]
                #[inline(always)]
                #[instability::unstable]
                pub const fn ptr() -> *const <super::pac::$base as core::ops::Deref>::Target {
                    super::pac::$base::PTR
                }
            }

            #[doc(hidden)]
            impl core::ops::Deref for $name {
                type Target = <super::pac::$base as core::ops::Deref>::Target;

                fn deref(&self) -> &Self::Target {
                    unsafe { &*Self::PTR }
                }
            }
        };
    }
}
