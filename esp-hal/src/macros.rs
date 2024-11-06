//! Macros used by the HAL.
//!
//! Most of the macros in this module are hidden and intended for internal use
//! only. For the list of public macros, see the [procmacros](https://docs.rs/esp-hal-procmacros/latest/esp_hal_procmacros/)
//! documentation.

pub use procmacros::*;

#[doc(hidden)]
/// Helper macro for checking doctest code snippets
#[macro_export]
macro_rules! before_snippet {
    () => {
        r#"
# #![no_std]
# use esp_hal::prelude::*;
# use procmacros::handler;
# use esp_hal::interrupt;
# #[panic_handler]
# fn panic(_ : &core::panic::PanicInfo) -> ! {
#     loop {}
# }
# fn main() {
#     let mut peripherals = esp_hal::init(esp_hal::Config::default());
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
#[macro_export]
macro_rules! any_peripheral {
    ($(#[$meta:meta])* $vis:vis peripheral $name:ident {
        $(
            $(#[cfg($variant_meta:meta)])*
            $variant:ident($inner:ty)
        ),* $(,)?
    }) => {
        paste::paste! {
            $(#[$meta])*
            $vis struct $name([< $name Inner >]);
            impl $crate::private::Sealed for $name {}

            impl $crate::dma::PeripheralMarker for $name {
                #[inline(always)]
                fn peripheral(&self) -> $crate::system::Peripheral {
                    match &self.0 {
                        $(
                            $(#[cfg($variant_meta)])*
                            [<$name Inner>]::$variant(inner) => inner.peripheral(),
                        )*
                    }
                }
            }

            impl $crate::peripheral::Peripheral for $name {
                type P = $name;

                unsafe fn clone_unchecked(&self) -> Self::P {
                    match &self.0 {
                        $(
                            $(#[cfg($variant_meta)])*
                            [<$name Inner>]::$variant(inner) => $name::from(inner.clone_unchecked()),
                        )*
                    }
                }
            }

            enum [< $name Inner >] {
                $(
                    $(#[cfg($variant_meta)])*
                    $variant($inner),
                )*
            }

            $(
                $(#[cfg($variant_meta)])*
                impl From<$inner> for $name {
                    fn from(inner: $inner) -> Self {
                        Self([< $name Inner >]::$variant(inner))
                    }
                }
            )*
        }
    };
}
