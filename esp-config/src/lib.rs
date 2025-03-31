#![doc = include_str!("../README.md")]
//! ## Feature Flags
#![doc = document_features::document_features!(feature_label = r#"<span class="stab portability"><code>{feature}</code></span>"#)]
#![doc(html_logo_url = "https://avatars.githubusercontent.com/u/46717278")]
#![cfg_attr(not(feature = "build"), no_std)]
#![deny(missing_docs, rust_2018_idioms, rustdoc::all)]

#[cfg(feature = "build")]
mod generate;
#[cfg(feature = "build")]
pub use generate::{generate_config, Error, Validator, Value};

/// Parse the value of an environment variable as a [bool] at compile time.
#[macro_export]
macro_rules! esp_config_bool {
    ( $var:expr ) => {
        match env!($var) {
            b"true" => true,
            b"false" => false,
            _ => ::core::panic!("boolean value must be either 'true' or 'false'"),
        }
    };
}

/// Parse the value of an environment variable as an integer at compile time.
#[macro_export]
macro_rules! esp_config_int {
    ( $ty:ty, $var:expr ) => {
        const { $crate::esp_config_int_parse!($ty, env!($var)) }
    };
}

/// Get the string value of an environment variable at compile time.
#[macro_export]
macro_rules! esp_config_str {
    ( $var:expr ) => {
        env!($var)
    };
}

/// Parse a string like "777" into an integer, which _can_ be used in a `const`
/// context
///
/// Not inlined into `esp_config_int` to make this easy to test.
#[doc(hidden)] // To avoid confusion with `esp_config_int`, hide this in the docs
#[macro_export]
macro_rules! esp_config_int_parse {
    ( $ty:ty, $s:expr ) => {{
        let val: $ty = match <$ty>::from_str_radix($s, 10) {
            Ok(val) => val as $ty,
            Err(_) => {
                core::assert!(false, concat!("Unable to parse '", $s, "' as a number."));
                0
            }
        };
        val
    }};
}

#[cfg(test)]
mod test {
    // We can only test success in the const context
    const _: () = {
        core::assert!(esp_config_int_parse!(i64, "-77777") == -77777);
        core::assert!(esp_config_int_parse!(isize, "-7777") == -7777);
        core::assert!(esp_config_int_parse!(i32, "-999") == -999);
        core::assert!(esp_config_int_parse!(i16, "-99") == -99);
        core::assert!(esp_config_int_parse!(i8, "-9") == -9);

        core::assert!(esp_config_int_parse!(u64, "77777") == 77777);
        core::assert!(esp_config_int_parse!(usize, "7777") == 7777);
        core::assert!(esp_config_int_parse!(u32, "999") == 999);
        core::assert!(esp_config_int_parse!(u16, "99") == 99);
        core::assert!(esp_config_int_parse!(u8, "9") == 9);
    };

    #[test]
    #[should_panic]
    fn test_expect_positive() {
        esp_config_int_parse!(u8, "-5");
    }

    #[test]
    #[should_panic]
    fn test_invalid_digit() {
        esp_config_int_parse!(u32, "a");
    }
}
