//! # Exclusive peripheral access
//!
//! ## Overview
//!
//! The Peripheral module provides an exclusive access mechanism to peripherals
//! on ESP chips. It includes the `PeripheralRef` struct, which represents an
//! exclusive reference to a peripheral. It offers memory efficiency benefits
//! for zero-sized types.
//!
//! ## Configuration
//!
//! The `PeripheralRef` struct is used to access and interact with peripherals.
//! It implements the `Deref` and `DerefMut` traits, allowing you to dereference
//! it to access the underlying peripheral. It also provides methods for cloning
//! and re-borrowing the peripheral.
//!
//! The module also defines the `Peripheral` trait, which is implemented by
//! types that can be used as peripherals. The trait allows conversion between
//! owned and borrowed peripherals and provides an unsafe method for cloning the
//! peripheral. By implementing this trait, a type can be used with the
//! `PeripheralRef` struct.
//!
//! The module also includes a `peripheral_macros` module, which contains macros
//! for generating peripheral structs and associated traits based on
//! configuration options.

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
/// peripherals like `PA9` or `Spi4`) or very small (for example `AnyPin` which
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
/// ```rust, ignore
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
/// ```rust, ignore
/// let mut twi: Twim<'static, ...> = Twim::new(p.TWISPI0, irq, p.P0_03, p.P0_04, config);
/// ```
///
/// Or you may call it with borrowed peripherals, which yields an instance that
/// can only live for as long as the borrows last:
///
/// ```rust, ignore
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

impl<T, P> Peripheral for &mut T
where
    T: Peripheral<P = P>,
{
    type P = P;

    unsafe fn clone_unchecked(&mut self) -> Self::P {
        T::clone_unchecked(self)
    }
}

impl<T> crate::private::Sealed for &mut T where T: crate::private::Sealed {}

mod peripheral_macros {
    #[doc(hidden)]
    #[macro_export]
    macro_rules! impl_dma_eligible {
        ($name:ident,$dma:ident) => {
            impl $crate::dma::DmaEligible for $name {
                const DMA_PERIPHERAL: $crate::dma::DmaPeripheral = $crate::dma::DmaPeripheral::$dma;
            }
        };
    }

    #[doc(hidden)]
    #[macro_export]
    macro_rules! peripherals {
        ($($(#[$cfg:meta])? $name:ident <= $from_pac:tt $(($($interrupt:ident),*))? ),*$(,)?) => {

            /// Contains the generated peripherals which implement [`Peripheral`]
            mod peripherals {
                pub use super::pac::*;
                $(
                    $crate::create_peripheral!($(#[$cfg])? $name <= $from_pac);
                )*


                $crate::impl_dma_eligible!(SPI2,Spi2);
                #[cfg(any(pdma, esp32s3))]
                $crate::impl_dma_eligible!(SPI3,Spi3);
                #[cfg(any(esp32c6, esp32h2))]
                $crate::impl_dma_eligible!(MEM2MEM1,Mem2Mem1);
                #[cfg(any(esp32c3, esp32c6, esp32h2, esp32s3))]
                $crate::impl_dma_eligible!(UHCI0,Uhci0);
                #[cfg(any(esp32, esp32s2, esp32c3, esp32c6, esp32h2, esp32s3))]
                $crate::impl_dma_eligible!(I2S0,I2s0);
                #[cfg(any(esp32, esp32s3))]
                $crate::impl_dma_eligible!(I2S1,I2s1);
                #[cfg(any(esp32c6, esp32h2))]
                $crate::impl_dma_eligible!(MEM2MEM4,Mem2Mem4);
                #[cfg(esp32s3)]
                $crate::impl_dma_eligible!(LCD_CAM,LcdCam);
                #[cfg(any(esp32c6, esp32h2))]
                $crate::impl_dma_eligible!(MEM2MEM5,Mem2Mem5);
                #[cfg(not(esp32c2))]
                $crate::impl_dma_eligible!(AES,Aes);
                #[cfg(gdma)]
                $crate::impl_dma_eligible!(SHA,Sha);
                #[cfg(any(esp32c3, esp32c6, esp32h2, esp32s3))]
                $crate::impl_dma_eligible!(ADC1,Adc);
                #[cfg(any(esp32c3, esp32s3))]
                $crate::impl_dma_eligible!(ADC2,Adc);
                #[cfg(esp32s3)]
                $crate::impl_dma_eligible!(RMT,Rmt);
                #[cfg(parl_io)]
                $crate::impl_dma_eligible!(PARL_IO,ParlIo);
                #[cfg(any(esp32c6, esp32h2))]
                $crate::impl_dma_eligible!(MEM2MEM10,Mem2Mem10);
                #[cfg(any(esp32c6, esp32h2))]
                $crate::impl_dma_eligible!(MEM2MEM11,Mem2Mem11);
                #[cfg(any(esp32c6, esp32h2))]
                $crate::impl_dma_eligible!(MEM2MEM12,Mem2Mem12);
                #[cfg(any(esp32c6, esp32h2))]
                $crate::impl_dma_eligible!(MEM2MEM13,Mem2Mem13);
                #[cfg(any(esp32c6, esp32h2))]
                $crate::impl_dma_eligible!(MEM2MEM14,Mem2Mem14);
                #[cfg(any(esp32c6, esp32h2))]
                $crate::impl_dma_eligible!(MEM2MEM15,Mem2Mem15);
            }

            /// The `Peripherals` struct provides access to all of the hardware peripherals on the chip.
            #[allow(non_snake_case)]
            pub struct Peripherals {
                $(
                    $(#[$cfg])?
                    /// Each field represents a hardware peripheral.
                    pub $name: peripherals::$name,
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

            $(
                $(
                    impl peripherals::$name {
                        $(
                            paste::paste!{
                                /// Binds an interrupt handler to the corresponding interrupt for this peripheral.
                                pub fn [<bind_ $interrupt:lower _interrupt >](&mut self, handler: unsafe extern "C" fn() -> ()) {
                                    unsafe { $crate::interrupt::bind_interrupt($crate::peripherals::Interrupt::$interrupt, handler); }
                                }
                            }
                        )*
                    }
                )*
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
    /// Macro to create a peripheral structure.
    macro_rules! create_peripheral {
        ($(#[$cfg:meta])? $name:ident <= virtual) => {
            $(#[$cfg])?
            #[derive(Debug)]
            #[allow(non_camel_case_types)]
            /// Represents a virtual peripheral with no associated hardware.
            ///
            /// This struct is generated by the `create_peripheral!` macro when the peripheral
            /// is defined as virtual.
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

            impl $crate::peripheral::Peripheral for $name {
                type P = $name;

                #[inline]
                unsafe fn clone_unchecked(&mut self) -> Self::P {
                    Self::steal()
                }
            }

            impl $crate::private::Sealed for $name {}
        };
        ($(#[$cfg:meta])? $name:ident <= $base:ident) => {
            $(#[$cfg])?
            #[derive(Debug)]
            #[allow(non_camel_case_types)]
            /// Represents a concrete hardware peripheral.
            ///
            /// This struct is generated by the `create_peripheral!` macro when the peripheral
            /// is tied to an actual hardware device.
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
                pub const PTR: *const <super::pac::$base as core::ops::Deref>::Target = super::pac::$base::PTR;

                #[doc = r"Return the pointer to the register block"]
                #[inline(always)]
                pub const fn ptr() -> *const <super::pac::$base as core::ops::Deref>::Target {
                    super::pac::$base::PTR
                }
            }

            impl core::ops::Deref for $name {
                type Target = <super::pac::$base as core::ops::Deref>::Target;

                fn deref(&self) -> &Self::Target {
                    unsafe { &*Self::PTR }
                }
            }

            impl core::ops::DerefMut for $name {

                fn deref_mut(&mut self) -> &mut Self::Target {
                    unsafe { &mut *(Self::PTR as *mut _)  }
                }
            }

            impl $crate::peripheral::Peripheral for $name {
                type P = $name;

                #[inline]
                unsafe fn clone_unchecked(&mut self) -> Self::P {
                    Self::steal()
                }
            }

            impl $crate::private::Sealed for $name {}
        };
    }
}
