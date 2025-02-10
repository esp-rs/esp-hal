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
            #[derive(Debug)]
            #[cfg_attr(feature = "defmt", derive(defmt::Format))]
            $vis struct $name([< $name Inner >]);
            impl $crate::private::Sealed for $name {}

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

            $(#[$meta])*
            #[derive(Debug)]
            enum [< $name Inner >] {
                $(
                    $(#[cfg($variant_meta)])*
                    $variant($inner),
                )*
            }

            #[cfg(feature = "defmt")]
            impl defmt::Format for [< $name Inner >] {
                fn format(&self, fmt: defmt::Formatter<'_>) {
                    match self {
                        $(
                            $(#[cfg($variant_meta)])*
                            [< $name Inner >]::$variant(inner) => inner.format(fmt),
                        )*
                    }
                }
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

/// Macro to define the application descriptor.
#[macro_export]
macro_rules! esp_app_desc {
    () => {
        #[repr(C)]
        pub struct EspAppDesc {
            pub magic_word: u32,     // Magic word ESP_APP_DESC_MAGIC_WORD
            pub secure_version: u32, // Secure version
            pub reserv1: [u32; 2],   // reserv1
            pub version: [core::ffi::c_char; 32], // Application version
            pub project_name: [core::ffi::c_char; 32], // Project name
            pub time: [core::ffi::c_char; 16], // Compile time
            pub date: [core::ffi::c_char; 16], // Compile date
            pub idf_ver: [core::ffi::c_char; 32], // Version IDF
            pub app_elf_sha256: [u8; 32], // sha256 of elf file
            pub min_efuse_blk_rev_full: u16, // Minimal eFuse block revision supported by image, in format: major * 100 + minor
            pub max_efuse_blk_rev_full: u16, // Maximal eFuse block revision supported by image, in format: major * 100 + minor
            pub mmu_page_size: u8,  // MMU page size in log base 2 format
            pub reserv3: [u8; 3],   // reserv3
            pub reserv2: [u32; 18], // reserv2
        }

        #[no_mangle]
        #[used]
        #[link_section = ".rodata_desc"]
        #[allow(non_upper_case_globals)]
        pub static esp_app_desc: EspAppDesc = {
            const ESP_APP_DESC_MAGIC_WORD: u32 = 0xABCD5432;

            const fn str_to_cstr_array<const C: usize>(s: &str) -> [::core::ffi::c_char; C] {
                let bytes = s.as_bytes();
                if bytes.len() >= C {
                    assert!(true, "String is too long for the C-string field");
                }

                let mut ret: [::core::ffi::c_char; C] = [0; C];
                let mut i = 0;
                loop {
                    ret[i] = bytes[i] as _;
                    i += 1;
                    if i >= bytes.len() {
                        break;
                    }
                }
                ret
            }

            EspAppDesc {
                magic_word: ESP_APP_DESC_MAGIC_WORD,
                secure_version: 0,
                reserv1: [0; 2],
                version: str_to_cstr_array(env!("CARGO_PKG_VERSION")),
                project_name: str_to_cstr_array(env!("CARGO_PKG_NAME")),
                time: str_to_cstr_array($crate::BUILD_TIME),
                date: str_to_cstr_array($crate::BUILD_DATE),
                // just pretending some esp-idf version here
                idf_ver: str_to_cstr_array("5.3.1"),
                app_elf_sha256: [0; 32],
                min_efuse_blk_rev_full: 0,
                max_efuse_blk_rev_full: u16::MAX,
                mmu_page_size: 0,
                reserv3: [0; 3],
                reserv2: [0; 18],
            }
        };
    };
}
