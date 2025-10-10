//! Macros used by the HAL.
//!
//! Most of the macros in this module are hidden and intended for internal use
//! only. For the list of public macros, see the [procmacros](https://docs.rs/esp-hal-procmacros/latest/esp_hal_procmacros/)
//! documentation.

#[doc(hidden)]
/// Helper macro for checking doctest code snippets
#[macro_export]
macro_rules! before_snippet {
    () => {
        r#"
# #![no_std]
# use procmacros::handler;
# use esp_hal::{interrupt::{self, InterruptConfigurable}, time::{Duration, Instant, Rate}};
# macro_rules! println {
#     ($($tt:tt)*) => { };
# }
# macro_rules! print {
#     ($($tt:tt)*) => { };
# }
# #[panic_handler]
# fn panic(_ : &core::panic::PanicInfo) -> ! {
#     loop {}
# }
# fn main() {
#   let _ = example();
# }
# struct ExampleError {}
# impl <T> From<T> for ExampleError where T: core::fmt::Debug {
#   fn from(_value: T) -> Self {
#       Self{}
#   }
# }
# async fn example() -> Result<(), ExampleError> {
#   let mut peripherals = esp_hal::init(esp_hal::Config::default());
"#
    };
}

#[doc(hidden)]
#[macro_export]
macro_rules! after_snippet {
    () => {
        r#"
# Ok(())
# }
"#
    };
}

#[doc(hidden)]
#[macro_export]
macro_rules! trm_markdown_link {
    () => {
        concat!("[Technical Reference Manual](", property!("trm"), ")")
    };
    ($anchor:literal) => {
        concat!(
            "[Technical Reference Manual](",
            property!("trm"),
            "#",
            $anchor,
            ")"
        )
    };
}

#[doc(hidden)]
/// Shorthand to define AnyPeripheral instances.
///
/// This macro generates the following:
///
/// - An `AnyPeripheral` struct, name provided by the macro call.
/// - An `any::Degrade` trait which is supposed to be used as a supertrait of a relevant Instance.
/// - An `any::Inner` enum, with the same variants as the original peripheral.
/// - A `From` implementation for each peripheral variant.
/// - A `degrade` method for each peripheral variant using the `any::Degrade` trait.
#[macro_export]
macro_rules! any_peripheral {
    ($(#[$meta:meta])* $vis:vis peripheral $name:ident<'d> {
        $(
            $(#[cfg($variant_meta:meta)])*
            $variant:ident($inner:ty)
        ),* $(,)?
    }) => {
        #[doc = concat!("Utilities related to [`", stringify!($name), "`]")]
        #[doc(hidden)]
        #[instability::unstable]
        pub mod any {
            #[allow(unused_imports)]
            use super::*;

            macro_rules! delegate {
                ($any:ident, $inner_ident:ident => $code:tt) => {
                    match &$any.0 {
                        $(
                            $(#[cfg($variant_meta)])*
                            any::Inner::$variant($inner_ident) => $code,
                        )*
                    }
                }
            }

            pub(crate) use delegate;

            $(#[$meta])*
            #[derive(Debug)]
            pub(crate) enum Inner<'d> {
                $(
                    $(#[cfg($variant_meta)])*
                    $variant($inner),
                )*
            }

            #[cfg(feature = "defmt")]
            impl defmt::Format for Inner<'_> {
                fn format(&self, fmt: defmt::Formatter<'_>) {
                    match self {
                        $(
                            $(#[cfg($variant_meta)])*
                            Self::$variant(inner) => inner.format(fmt),
                        )*
                    }
                }
            }

            // Trick to make peripherals implement something Into-like, without
            // requiring Instance traits to have lifetimes. Rustdoc will list
            // this trait as a supertrait, but will not give its definition.
            // Users are encouraged to use From to convert a singleton into its
            // relevant AnyPeripheral counterpart.
            #[allow(unused)]
            pub trait Degrade: Sized + $crate::private::Sealed {
                fn degrade<'a>(self) -> super::$name<'a>
                where
                    Self: 'a;
            }
        }

        $(#[$meta])*
        ///
        /// This struct is a type-erased version of a peripheral singleton. It is useful
        /// for creating arrays of peripherals, or avoiding generics. Peripheral singletons
        /// can be type erased by using their `From` implementation.
        ///
        /// ```rust,ignore
        #[doc = concat!("let any_peripheral = ", stringify!($name), "::from(peripheral);")]
        /// ```
        #[derive(Debug)]
        #[cfg_attr(feature = "defmt", derive(defmt::Format))]
        $vis struct $name<'d>(any::Inner<'d>);

        impl $name<'_> {
            /// Unsafely clone this peripheral reference.
            ///
            /// # Safety
            ///
            /// You must ensure that you're only using one instance of this type at a time.
            #[inline]
            pub unsafe fn clone_unchecked(&self) -> Self { unsafe {
                any::delegate!(self, inner => { Self::from(inner.clone_unchecked()) })
            }}

            /// Creates a new peripheral reference with a shorter lifetime.
            ///
            /// Use this method if you would like to keep working with the peripheral after
            /// you dropped the driver that consumes this.
            ///
            /// See [Peripheral singleton] section for more information.
            ///
            /// [Peripheral singleton]: crate#peripheral-singletons
            #[inline]
            pub fn reborrow(&mut self) -> $name<'_> {
                unsafe { self.clone_unchecked() }
            }

            #[procmacros::doc_replace]
            /// Attempts to downcast the pin into the underlying peripheral instance.
            ///
            /// ## Example
            ///
            /// ```rust,no_run
            /// # {before_snippet}
            /// #
            /// # use esp_hal::{
            /// #     uart::AnyUart as AnyPeripheral,
            /// #     peripherals::{UART0 as PERI0, UART1 as PERI1},
            /// # };
            /// #
            /// # let peri0 = peripherals.UART0;
            /// # let peri1 = peripherals.UART1;
            /// // let peri0 = peripherals.PERI0;
            /// // let peri1 = peripherals.PERI1;
            /// let any_peri0 = AnyPeripheral::from(peri0);
            /// let any_peri1 = AnyPeripheral::from(peri1);
            ///
            /// let uart0 = any_peri0
            ///     .downcast::<PERI0>()
            ///     .expect("This downcast succeeds because AnyPeripheral was created from Peri0");
            /// let uart0 = any_peri1
            ///     .downcast::<PERI0>()
            ///     .expect_err("This AnyPeripheral was created from Peri1, it cannot be downcast to Peri0");
            /// #
            /// # {after_snippet}
            /// ```
            #[inline]
            pub fn downcast<P>(self) -> Result<P, Self>
            where
                Self: TryInto<P, Error = Self>
            {
                self.try_into()
            }
        }

        impl $crate::private::Sealed for $name<'_> {}

        // AnyPeripheral converts into itself
        impl<'d> any::Degrade for $name<'d> {
            #[inline]
            fn degrade<'a>(self) -> $name<'a>
            where
                Self: 'a,
            {
                self
            }
        }

        $(
            // Variants convert into AnyPeripheral
            $(#[cfg($variant_meta)])*
            impl<'d> any::Degrade for $inner {
                #[inline]
                fn degrade<'a>(self) -> $name<'a>
                where
                    Self: 'a,
                {
                    $name::from(self)
                }
            }

            $(#[cfg($variant_meta)])*
            impl<'d> From<$inner> for $name<'d> {
                #[inline]
                fn from(inner: $inner) -> Self {
                    Self(any::Inner::$variant(inner))
                }
            }

            $(#[cfg($variant_meta)])*
            impl<'d> TryFrom<$name<'d>> for $inner {
                type Error = $name<'d>;

                #[inline]
                fn try_from(any: $name<'d>) -> Result<Self, $name<'d>> {
                    #[allow(irrefutable_let_patterns)]
                    if let $name(any::Inner::$variant(inner)) = any {
                        Ok(inner)
                    } else {
                        Err(any)
                    }
                }
            }
        )*
    };
}

/// Macro to choose between two expressions. Useful for implementing "else" for
/// `$()?` macro syntax.
#[macro_export]
#[doc(hidden)]
macro_rules! if_set {
    (, $not_set:expr) => {
        $not_set
    };
    ($set:expr, $not_set:expr) => {
        $set
    };
}

/// Macro to ignore tokens.
///
/// This is useful when we need existence of a metavariable (to expand a
/// repetition), but we don't need to use it.
#[macro_export]
#[doc(hidden)]
macro_rules! ignore {
    ($($item:tt)*) => {};
}

/// Define a piece of (Espressif-specific) metadata that external tools may
/// parse.
///
/// The symbol name be formatted as `_ESP_METADATA_<category>_<name>`.
///
/// This metadata is zero cost, i.e. the value will not be flashed to the
/// device.
#[macro_export]
#[doc(hidden)]
macro_rules! metadata {
    ($category:literal, $key:ident, $value:expr) => {
        #[cfg(feature = "rt")]
        #[unsafe(link_section = concat!(".espressif.metadata"))]
        #[used]
        #[unsafe(export_name = concat!($category, ".", stringify!($key)))]
        static $key: [u8; $value.len()] = const {
            let val_bytes = $value.as_bytes();
            let mut val_bytes_array = [0; $value.len()];
            let mut i = 0;
            while i < val_bytes.len() {
                val_bytes_array[i] = val_bytes[i];
                i += 1;
            }
            val_bytes_array
        };
    };
}

#[procmacros::doc_replace]
/// Extract fields from [`Peripherals`][crate::peripherals::Peripherals] into named groups.
///
/// ## Example
///
/// ```rust,no_run
/// # {before_snippet}
/// #
/// use esp_hal::assign_resources;
///
/// assign_resources! {
///     Resources<'d> {
///         display: DisplayResources<'d> {
///             spi:  SPI2,
///             sda:  GPIO5,
///             sclk: GPIO4,
///             cs:   GPIO3,
///             dc:   GPIO2,
///         },
///         axl: AccelerometerResources<'d> {
///             i2c: I2C0,
///             sda: GPIO0,
///             scl: GPIO1,
///         },
///     }
/// }
///
/// # struct Display<'d>(core::marker::PhantomData<&'d ()>);
/// fn init_display<'d>(r: DisplayResources<'d>) -> Display<'d> {
///     // use `r.spi`, `r.sda`, `r.sclk`, `r.cs`, `r.dc`
///     todo!()
/// }
///
/// # struct Accelerometer<'d>(core::marker::PhantomData<&'d ()>);
/// fn init_accelerometer<'d>(r: AccelerometerResources<'d>) -> Accelerometer<'d> {
///     // use `r.i2c`, `r.sda`, `r.scl`
///     todo!()
/// }
///
/// // let peripherals = esp_hal::init(...);
/// let resources = split_resources!(peripherals);
///
/// let display = init_display(resources.display);
/// let axl = init_accelerometer(resources.axl);
///
/// // Other fields (`peripherals.UART0`, ...) of the `peripherals` struct can still be accessed.
/// # {after_snippet}
/// ```
// Based on https://crates.io/crates/assign-resources
#[macro_export]
#[cfg(feature = "unstable")]
#[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
macro_rules! assign_resources {
    {
        $(#[$struct_meta:meta])*
        $vis:vis $struct_name:ident<$struct_lt:lifetime> {
            $(
                $(#[$group_meta:meta])*
                $group_name:ident : $group_struct:ident<$group_lt:lifetime> {
                    $(
                        $(#[$resource_meta:meta])*
                        $resource_name:ident : $resource_field:ident
                    ),*
                    $(,)?
                }
            ),+
            $(,)?
        }
    } => {
        // Group structs
        $(
            $(#[$group_meta])*
            #[allow(missing_docs)]
            $vis struct $group_struct<$group_lt> {
                $(
                    $(#[$resource_meta])*
                    pub $resource_name: $crate::peripherals::$resource_field<$group_lt>,
                )+
            }

            impl<$group_lt> $group_struct<$group_lt> {
                /// Unsafely create an instance of the assigned peripherals out of thin air.
                ///
                /// # Safety
                ///
                /// You must ensure that you're only using one instance of the contained peripherals at a time.
                pub unsafe fn steal() -> Self {
                    unsafe {
                        Self {
                            $($resource_name: $crate::peripherals::$resource_field::steal()),*
                        }
                    }
                }

                /// Creates a new reference to the peripheral group with a shorter lifetime.
                ///
                /// Use this method if you would like to keep working with the peripherals after
                /// you dropped the drivers that consume this.
                pub fn reborrow(&mut self) -> $group_struct<'_> {
                    $group_struct {
                        $($resource_name: self.$resource_name.reborrow()),*
                    }
                }
            }
        )+

        // Outer struct
        $(#[$struct_meta])*
        /// Assigned resources.
        $vis struct $struct_name<$struct_lt> {
            $( pub $group_name: $group_struct<$struct_lt>, )+
        }

        impl<$struct_lt> $struct_name<$struct_lt> {
            /// Unsafely create an instance of the assigned peripherals out of thin air.
            ///
            /// # Safety
            ///
            /// You must ensure that you're only using one instance of the contained peripherals at a time.
            pub unsafe fn steal() -> Self {
                unsafe {
                    Self {
                        $($group_name: $group_struct::steal()),*
                    }
                }
            }

            /// Creates a new reference to the assigned peripherals with a shorter lifetime.
            ///
            /// Use this method if you would like to keep working with the peripherals after
            /// you dropped the drivers that consume this.
            pub fn reborrow(&mut self) -> $struct_name<'_> {
                $struct_name {
                    $($group_name: self.$group_name.reborrow()),*
                }
            }
        }

        /// Extracts resources from the `Peripherals` struct.
        #[macro_export]
        macro_rules! split_resources {
            ($peris:ident) => {
                $struct_name {
                    $($group_name: $group_struct {
                        $($resource_name: $peris.$resource_field),*
                    }),*
                }
            }
        }
    };
}
