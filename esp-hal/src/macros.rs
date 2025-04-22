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
# fn example() -> Result<(), ExampleError> {
#   let mut peripherals = esp_hal::init(esp_hal::Config::default());
"#
    };
}

#[doc(hidden)]
#[macro_export]
macro_rules! trm_markdown_link {
    () => {
        concat!("[Technical Reference Manual](", $crate::trm_link!(), ")")
    };
    ($anchor:literal) => {
        concat!(
            "[Technical Reference Manual](",
            $crate::trm_link!(),
            "#",
            $anchor,
            ")"
        )
    };
}

#[doc(hidden)]
/// Shorthand to define enums with From implementations.
#[macro_export]
macro_rules! any_enum {
    ($(#[$meta:meta])* $vis:vis enum $name:ident {
        $(
            $(#[$variant_meta:meta])*
            $variant:ident($inner:ty)
        ),* $(,)?
    }) => {
        $(#[$meta])*
        $vis enum $name {
            $(
                $(#[$variant_meta])*
                $variant($inner),
            )*
        }

        $(
            $(#[$variant_meta])*
            impl From<$inner> for $name {
                fn from(inner: $inner) -> Self {
                    $name::$variant(inner)
                }
            }
        )*
    };
}

#[doc(hidden)]
/// Shorthand to define AnyPeripheral instances.
///
/// This macro generates the following:
///
/// - An `AnyPeripheral` struct, name provided by the macro call.
/// - An `AnyPeripheralInner` enum, with the same variants as the original
///   peripheral.
/// - A `From` implementation for each peripheral variant.
/// - A `degrade` method for each peripheral variant using the
///   `IntoAnyPeripheral` trait.
#[macro_export]
macro_rules! any_peripheral {
    ($(#[$meta:meta])* $vis:vis peripheral $name:ident<'d> {
        $(
            $(#[cfg($variant_meta:meta)])*
            $variant:ident($inner:ty)
        ),* $(,)?
    }) => {
        paste::paste! {
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
            $vis struct $name<'d>([< $name Inner >]<'d>);

            impl $name<'_> {
                /// Unsafely clone this peripheral reference.
                ///
                /// # Safety
                ///
                /// You must ensure that you're only using one instance of this type at a time.
                #[inline]
                pub unsafe fn clone_unchecked(&self) -> Self { unsafe {
                    match &self.0 {
                        $(
                            $(#[cfg($variant_meta)])*
                            [< $name Inner >]::$variant(inner) => $name([<$name Inner>]::$variant(inner.clone_unchecked())),
                        )*
                    }
                }}

                /// Creates a new peripheral reference with a shorter lifetime.
                ///
                /// Use this method if you would like to keep working with the peripheral after
                /// you dropped the driver that consumes this.
                #[inline]
                pub fn reborrow(&mut self) -> $name<'_> {
                    unsafe { self.clone_unchecked() }
                }
            }

            impl $crate::private::Sealed for $name<'_> {}

            $(#[$meta])*
            #[derive(Debug)]
            enum [< $name Inner >]<'d> {
                $(
                    $(#[cfg($variant_meta)])*
                    $variant($inner),
                )*
            }

            #[cfg(feature = "defmt")]
            impl defmt::Format for [< $name Inner >]<'_> {
                fn format(&self, fmt: defmt::Formatter<'_>) {
                    match self {
                        $(
                            $(#[cfg($variant_meta)])*
                            [< $name Inner >]::$variant(inner) => inner.format(fmt),
                        )*
                    }
                }
            }

            // Trick to make peripherals implement Into, without
            // requiring Instance traits to have lifetimes.
            #[doc(hidden)]
            pub trait [<Into $name>]: Sized + $crate::private::Sealed {
                fn degrade<'a>(self) -> $name<'a>
                where
                    Self: 'a;
            }

            // AnyPeripheral converts into itself
            impl<'d> [<Into $name>] for $name<'d> {
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
                impl<'d> [<Into $name>] for $inner {
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
                        Self([< $name Inner >]::$variant(inner))
                    }
                }
            )*
        }
    };
}

/// Macro to choose between two expressions. Useful for implementing "else" for
/// `$()?` macro syntax.
#[macro_export]
#[doc(hidden)]
macro_rules! if_set {
    (, $not_set:expr_2021) => {
        $not_set
    };
    ($set:expr_2021, $not_set:expr_2021) => {
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
    ($category:literal, $key:ident, $value:expr_2021) => {
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
